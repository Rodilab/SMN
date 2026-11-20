#include "smn_controls.h"
#include "lfo_wavetable_bank.h"

#include <math.h>

#ifndef TEST
#include "stm32f7xx.h"
#endif

typedef struct {
    float phase[3]; // Combined encoder + fixed voice dispersion offset.
    float cv[3];
    float dispersion_encoder[3];
    float dispersion_cv[3];
} SmnControlInputs;

typedef struct {
    float current;
    float target;
    float dispersion_target;
    // Compute phase as target + remaining * phase_step to avoid accumulated
    // rounding error and reach the target exactly. Set the step on retarget.
    float phase_step;
    // Dispersion uses the phase ramp clock to track the headroom left for live CV.
    float dispersion_step;
    uint32_t remaining;
} SmnControlRamp;

static volatile SmnControlInputs inputs[SMN_SYNTH_MAX_VOICES];
static volatile uint8_t initialized[SMN_SYNTH_MAX_VOICES];
// Writers flag new inputs; audio consumes them under the snapshot lock.
// Internal modulation changes do not invalidate the manual/CV inputs.
static volatile uint8_t inputs_pending[SMN_SYNTH_MAX_VOICES];
static struct {
    SmnControlInputs inputs;
    float phase[3]; // Reduced manual phase, before live CV/internal modulation is added.
    float dispersion_cv[3];
    uint8_t finite_phase;
    uint8_t settled; // Manual ramps only; dose ramps still advance every block.
} preparation[SMN_SYNTH_MAX_VOICES]; // Prepared by audio; a lock change invalidates the affected dimension.
// Audio advances these states. A targeted lock event may rebase one dimension
// under lock_controls(), together with its published inputs.
static SmnControlRamp ramps[SMN_SYNTH_MAX_VOICES][3];
static uint8_t ramp_initialized[SMN_SYNTH_MAX_VOICES];
// Audio publishes; the lower-priority LED reader briefly locks interrupts.
static volatile SmnTimbreControls current_controls[SMN_SYNTH_MAX_VOICES];
// Writers increment this under the publication lock. Audio reuses H/T/M
// results until the settings or modulation amounts change.
static volatile uint32_t control_revision[SMN_SYNTH_MAX_VOICES];
static struct {
    uint32_t revision;
    float lfo;
    uint8_t settled;
} control_cache[SMN_SYNTH_MAX_VOICES]; // Used only by audio; not saved.

typedef struct {
    float current, target, step;
    uint32_t remaining;
} SmnModulationRamp;

// UI prepares these ramps under lock_controls(). Audio only advances them;
// it does not need a lock, target comparison or division.
static SmnModulationRamp modulation[SMN_SYNTH_MAX_VOICES][SMN_MOD_DESTINATIONS];
static uint16_t modulation_steps[SMN_SYNTH_MAX_VOICES][SMN_MOD_DESTINATIONS];
static uint8_t modulation_active[SMN_SYNTH_MAX_VOICES];

_Static_assert(SMN_CONTROL_RAMP_BLOCKS > 0, "Ramp needs at least one block");
_Static_assert(SMN_MOD_FULL_STEPS <= UINT16_MAX, "Breathe target must fit");
_Static_assert(SMN_MOD_DESTINATIONS <= 8, "Active mask must fit a byte");

// A-F x H/T/M. Fixed irregular sets, each with zero mean and no opposite
// equal-magnitude pairs, to reduce coincidences when folding at an endpoint.
// Full-intensity spans stay below 3 raw units (half of the 6-unit cycle).
// Scaled once here, not recalculated in the audio IRQ.
static const float dispersion_offset[SMN_SYNTH_MAX_VOICES][3] = {
    {-0.54f,  0.99f, -1.15f},
    { 1.38f, -0.57f,  0.60f},
    {-0.89f,  1.47f,  0.15f},
    { 0.28f, -1.28f,  1.48f},
    {-1.26f,  0.23f, -0.34f},
    { 1.03f, -0.84f, -0.74f}
};

static float clamp_intensity(float value)
{
    if (!isfinite(value) || value < 0.0f) return 0.0f;
    return value > 1.0f ? 1.0f : value;
}

// Publish the whole encoder/CV group together so audio cannot read a partial
// update. This also serializes main-loop and interrupt writers. Host tests
// run on one thread and do not access the interrupt controller.
static uint32_t lock_controls(void)
{
#ifndef TEST
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return previous;
#else
    return 0;
#endif
}

static void unlock_controls(uint32_t previous)
{
#ifndef TEST
    __DMB();
    __set_PRIMASK(previous);
#else
    (void)previous;
#endif
}

// Precomputed glide durations: linear from 0 to 10 ms, then rounded geometric
// steps. Every fifth entry matches a panel landmark.
const uint32_t smn_glide_times_ms[SMN_GLIDE_TIME_COUNT] = {
    0, 2, 4, 6, 8, 10, 11, 13, 15, 17,
    20, 22, 24, 26, 28, 30, 33, 37, 41, 45,
    50, 54, 59, 64, 69, 75, 79, 84, 89, 94,
    100, 108, 118, 128, 138, 150, 166, 184, 204, 226,
    250, 275, 302, 331, 364, 400, 434, 470, 510, 553,
    600, 665, 736, 815, 903, 1000, 1084, 1176, 1275, 1383,
    1500, 1661, 1840, 2038, 2257, 2500, 2746, 3017, 3314, 3641,
    4000, 4474, 5004, 5596, 6259, 7000, 7518, 8073, 8670, 9311,
    10000, 10845, 11761, 12754, 13832, 15000, 17230, 19793, 22736, 26117,
    30000
};

uint8_t smn_glide_time_index(uint32_t milliseconds)
{
    if (!milliseconds) return 0;
    if (milliseconds >= SMN_GLIDE_MAX_MS) return SMN_GLIDE_TIME_COUNT - 1;
    uint8_t low = 0, high = SMN_GLIDE_TIME_COUNT - 1;
    while (low < high) {
        const uint8_t mid = (low + high) / 2;
        if (smn_glide_times_ms[mid] < milliseconds) low = mid + 1;
        else high = mid;
    }
    // Out-of-table valid legacy values use the nearest duration for editing.
    if (low && milliseconds - smn_glide_times_ms[low - 1] <
        smn_glide_times_ms[low] - milliseconds) --low;
    return low;
}

typedef struct {
    float destination, ratio, relative_step, error;
    uint32_t remaining, revision;
} SmnGlide;
static SmnGlide glides[SMN_SYNTH_MAX_VOICES];

void smn_glide_reset(uint8_t voice)
{
    if (voice >= SMN_SYNTH_MAX_VOICES) return;
    const uint32_t previous = lock_controls();
    SmnGlide *g = &glides[voice];
    const uint32_t revision = g->revision + 1;
    *g = (SmnGlide){.ratio = 1.0f, .revision = revision};
    unlock_controls(previous);
}

void smn_glide_update(uint8_t voice, float destination_hz, uint32_t milliseconds,
    uint8_t flags, float tuned_pitch_hz, float *published_pitch)
{
    if (voice >= SMN_SYNTH_MAX_VOICES) return;
    SmnGlide *g = &glides[voice];
    // Stable note, including duration edits that apply to the next glide.
    // Only the parameter writer changes destination; audio changes ratio.
    if (g->destination == destination_hz && !(flags & SMN_GLIDE_ATTACK) &&
        ((flags & SMN_GLIDE_ENABLED) || !g->remaining)) {
        *published_pitch = tuned_pitch_hz;
        return;
    }
    if (!(flags & SMN_GLIDE_ENABLED) || !(flags & SMN_GLIDE_LEGATO) ||
        !milliseconds || milliseconds > SMN_GLIDE_MAX_MS ||
        !(destination_hz > 0.0f) || !isfinite(destination_hz)) {
        if (!g->remaining) {
            // With glide off/zero the audio reader uses only published_pitch;
            // no lock is needed for these aligned stores, even under live CV.
            g->destination = destination_hz;
            *published_pitch = tuned_pitch_hz;
            return;
        }
        const uint32_t previous = lock_controls();
        g->destination = destination_hz;
        g->ratio = 1.0f;
        g->remaining = 0;
        ++g->revision;
        *published_pitch = tuned_pitch_hz;
        unlock_controls(previous);
        return;
    }
    const uint32_t blocks = (milliseconds * SMN_SYNTH_SAMPLE_RATE_HZ +
        500u * SMN_SYNTH_BLOCK_SIZE) / (1000u * SMN_SYNTH_BLOCK_SIZE);
    for (;;) {
        uint32_t previous = lock_controls();
        const uint32_t revision = g->revision;
        const float current = g->destination * g->ratio;
        unlock_controls(previous);
        const float ratio = current > 0.0f && isfinite(current)
            ? current / destination_hz : 1.0f;
        // expm1 retains tiny increments: exp(x)-1 would round small 30 s
        // semitone/quarter-tone steps severely. Never calculated in audio.
        const float step = ratio != 1.0f ? expm1f(-logf(ratio) / blocks) : 0.0f;
        previous = lock_controls();
        if (g->revision != revision) {
            // Audio ran during preparation. Read the pitch it reached and retry;
            // the audio interrupt must not wait for this writer.
            unlock_controls(previous);
            continue;
        }
        g->destination = destination_hz;
        g->ratio = ratio;
        g->relative_step = step;
        g->error = 0.0f;
        g->remaining = step ? blocks : 0;
        ++g->revision;
        *published_pitch = tuned_pitch_hz;
        unlock_controls(previous);
        return;
    }
}

uint8_t smn_glide_process_pitch(uint8_t voice, float *frequency_hz)
{
    if (voice >= SMN_SYNTH_MAX_VOICES) return 0;
    SmnGlide *g = &glides[voice];
    if (!g->remaining) return 0;
    ++g->revision;
    if (!--g->remaining) {
        g->ratio = 1.0f;
        return 0; // Exact destination through the normal tuned-pitch path.
    }
    // Geometric in Hz = linear in notes. Compensated addition keeps long,
    // small-interval glides accurate without double precision or block logs.
    const float increment = g->ratio * g->relative_step + g->error;
    const float reached = g->ratio + increment;
    g->error = increment - (reached - g->ratio);
    g->ratio = reached;
    *frequency_hz = g->destination * reached;
    return 1;
}

static void adjust_modulation(uint8_t voice, uint8_t destination, int32_t events)
{
    if (voice >= SMN_SYNTH_MAX_VOICES || destination >= SMN_MOD_DESTINATIONS || !events)
        return;
    const uint32_t previous = lock_controls();
    int32_t steps = (int32_t)modulation_steps[voice][destination] + events;
    if (steps < 0) steps = 0;
    if (steps > SMN_MOD_FULL_STEPS) steps = SMN_MOD_FULL_STEPS;
    if (steps != modulation_steps[voice][destination]) {
        SmnModulationRamp *ramp = &modulation[voice][destination];
        modulation_steps[voice][destination] = steps;
        // Destination scaling is done on a Browse event, outside audio.
        const float scale = destination == SMN_MOD_GAIN
            ? 4095.0f / SMN_MOD_FULL_STEPS : destination == SMN_MOD_PITCH
            ? SMN_MOD_PITCH_MAX_SEMITONES / SMN_MOD_FULL_STEPS
            : destination == SMN_MOD_PAN ? 1.0f / SMN_MOD_FULL_STEPS
            : 3.0f / SMN_MOD_FULL_STEPS;
        ramp->target = (float)steps * scale;
        ramp->step = (ramp->current - ramp->target) * (1.0f / SMN_CONTROL_RAMP_BLOCKS);
        ramp->remaining = SMN_CONTROL_RAMP_BLOCKS;
        modulation_active[voice] |= 1u << destination;
        if (destination >= SMN_MOD_HARMONICS && destination <= SMN_MOD_MORPH)
            ++control_revision[voice];
    }
    unlock_controls(previous);
}

void smn_modulation_adjust(uint8_t voice, uint8_t destination, int16_t events, uint8_t fine)
{
    adjust_modulation(voice, destination, (int32_t)events *
        (int32_t)(fine ? 1 : SMN_MOD_FINE_DIVISOR));
}

float smn_modulation_target(uint8_t voice, uint8_t destination)
{
    if (voice >= SMN_SYNTH_MAX_VOICES || destination >= SMN_MOD_DESTINATIONS)
        return 0.0f;
    const uint32_t previous = lock_controls();
    const uint16_t steps = modulation_steps[voice][destination];
    unlock_controls(previous);
    return (float)steps * (1.0f / SMN_MOD_FULL_STEPS);
}

void smn_modulation_clear_gain(uint8_t voice)
{
    smn_modulation_clear(voice, SMN_MOD_GAIN);
}

void smn_modulation_clear(uint8_t voice, uint8_t destination)
{
    if (voice >= SMN_SYNTH_MAX_VOICES || destination >= SMN_MOD_DESTINATIONS) return;
    const uint32_t previous = lock_controls();
    modulation[voice][destination] = (SmnModulationRamp){0};
    modulation_steps[voice][destination] = 0;
    modulation_active[voice] &= ~(1u << destination);
    if (destination >= SMN_MOD_HARMONICS && destination <= SMN_MOD_MORPH) ++control_revision[voice];
    unlock_controls(previous);
}

void smn_modulation_reset(void)
{
    // Update one voice at a time to keep interrupt masking brief.
    for (unsigned v = 0; v < SMN_SYNTH_MAX_VOICES; ++v) {
        const uint32_t previous = lock_controls();
        for (unsigned d = 0; d < SMN_MOD_DESTINATIONS; ++d) {
            modulation[v][d] = (SmnModulationRamp){0};
            modulation_steps[v][d] = 0;
        }
        modulation_active[v] = 0;
        ++control_revision[v];
        unlock_controls(previous);
    }
}

void smn_modulation_peek_steps(uint8_t voice, uint16_t steps[SMN_MOD_DESTINATIONS])
{
    if (voice >= SMN_SYNTH_MAX_VOICES) return;
    // Aligned uint16 loads are indivisible on this Cortex-M7. Volatile prevents
    // the compiler reusing values across the caller's two independent captures.
    const volatile uint16_t *source = modulation_steps[voice];
    for (unsigned d = 0; d < SMN_MOD_DESTINATIONS; ++d) steps[d] = source[d];
}

void smn_modulation_capture_steps(uint8_t voice, uint16_t steps[SMN_MOD_DESTINATIONS])
{
    if (voice >= SMN_SYNTH_MAX_VOICES) return;
    const uint32_t previous = lock_controls();
    for (unsigned d = 0; d < SMN_MOD_DESTINATIONS; ++d)
        steps[d] = modulation_steps[voice][d];
    unlock_controls(previous);
}

void smn_modulation_restore_steps(uint8_t voice, const uint16_t steps[SMN_MOD_DESTINATIONS])
{
    if (voice >= SMN_SYNTH_MAX_VOICES) return;
    SmnModulationRamp restored[SMN_MOD_DESTINATIONS];
    uint16_t bounded[SMN_MOD_DESTINATIONS];
    uint8_t active = 0;
    for (unsigned d = 0; d < SMN_MOD_DESTINATIONS; ++d) {
        bounded[d] = steps[d] <= SMN_MOD_FULL_STEPS ? steps[d] : 0;
        const float scale = d == SMN_MOD_GAIN ? 4095.0f / SMN_MOD_FULL_STEPS
            : d == SMN_MOD_PITCH ? SMN_MOD_PITCH_MAX_SEMITONES / SMN_MOD_FULL_STEPS
            : d == SMN_MOD_PAN ? 1.0f / SMN_MOD_FULL_STEPS
            : 3.0f / SMN_MOD_FULL_STEPS;
        const float target = (float)bounded[d] * scale;
        restored[d] = (SmnModulationRamp){target, target, 0.0f, 0};
        if (bounded[d]) active |= 1u << d;
    }
    const uint32_t previous = lock_controls();
    for (unsigned d = 0; d < SMN_MOD_DESTINATIONS; ++d) {
        modulation_steps[voice][d] = bounded[d];
        modulation[voice][d] = restored[d];
    }
    modulation_active[voice] = active;
    ++control_revision[voice];
    unlock_controls(previous);
}

static inline float advance_modulation(uint8_t voice, unsigned destination)
{
    SmnModulationRamp *ramp = &modulation[voice][destination];
    if (ramp->remaining) {
        --ramp->remaining;
        ramp->current = ramp->target + (float)ramp->remaining * ramp->step;
        if (!ramp->remaining && ramp->target == 0.0f)
            modulation_active[voice] &= ~(1u << destination);
    }
    return ramp->current;
}

float smn_modulation_process_gain(uint8_t voice, float lfo, uint8_t allowed)
{
    if (voice >= SMN_SYNTH_MAX_VOICES || !(modulation_active[voice] & (1u << SMN_MOD_GAIN)))
        return 0.0f;
    if (!allowed) {
        smn_modulation_clear_gain(voice);
        return 0.0f;
    }
    return advance_modulation(voice, SMN_MOD_GAIN) * lfo;
}

float smn_modulation_process_pan(uint8_t voice, float pan, float lfo)
{
    // Firmware calls only with a valid channel and the existing 0..1 LFO.
    // Idle: one mask test; no ramp/source/clamp calculation.
    if (!(modulation_active[voice] & (1u << SMN_MOD_PAN))) return pan;
    const float depth = advance_modulation(voice, SMN_MOD_PAN);
    const float position = pan + (2.0f * lfo - 1.0f) * depth;
    return position < 0.0f ? 0.0f : position > 1.0f ? 1.0f : position;
}

float smn_modulation_process_pitch(uint8_t voice, float frequency_hz, float lfo,
                                   uint8_t envelope_mode, uint16_t shape)
{
    if (voice >= SMN_SYNTH_MAX_VOICES ||
        !(modulation_active[voice] & (1u << SMN_MOD_PITCH)))
        return frequency_hz;

    const float depth = advance_modulation(voice, SMN_MOD_PITCH);
    float source = clamp_intensity(lfo);
    // Two ramp banks remain unipolar; all other banks are bipolar in LFO mode.
    if (!envelope_mode && shape >= 2u * LFO_BANK_SIZE)
        source = 2.0f * source - 1.0f;
    const float semitones = source * depth;
    // exp(s * ln(2)/12), degree 4 over [-2,+2] semitones. Four FMAs on
    // Cortex-M7, no table, division, powf/expf or Hz -> note round trip.
    // Coefficients are (ln(2)/12)^n / n!, maximum error < 0.001 cent.
    const float ratio = 1.0f + semitones * (0.05776226505f + semitones *
        (0.001668239631f + semitones * (0.00003211940757f +
        semitones * 0.0000004638194565f)));
    // Preserve the adapter's rejection of corrupt/invalid base frequencies.
    if (!isfinite(frequency_hz) || frequency_hz < SMN_SYNTH_MIN_FREQUENCY_HZ)
        return frequency_hz;
    const float modulated = frequency_hz * ratio;
    if (modulated < SMN_SYNTH_MIN_FREQUENCY_HZ) return SMN_SYNTH_MIN_FREQUENCY_HZ;
    if (modulated > SMN_SYNTH_MAX_FREQUENCY_HZ) return SMN_SYNTH_MAX_FREQUENCY_HZ;
    return modulated;
}

// Exact signed remainder over a six-unit cycle for ordinary control values.
// The integer product is exact below 2^23. Reciprocal rounding can select the
// adjacent quotient, corrected by at most one cycle; there is no wrap loop.
// Huge finite preset values retain the library's robust reduction.
static float cycle_remainder(float value)
{
    const float period = (float)SMN_CONTROL_CYCLE_UNITS;
    const float magnitude = fabsf(value);
    if (magnitude < period)
        return value;
    if (magnitude < 0x1p23f) {
        const int32_t turns = (int32_t)(magnitude * (1.0f / period));
        float remainder = magnitude - (float)turns * period;
        if (remainder < 0.0f)
            remainder += period;
        else if (remainder >= period)
            remainder -= period;
        return copysignf(remainder, value);
    }
    return fmodf(value, period);
}

_Static_assert(SMN_CONTROL_CYCLE_UNITS == 6u,
               "Fast cycle reduction assumes the six-unit control cycle");

static float fold_remainders(float encoder_remainder, float cv_remainder)
{
    const float position = encoder_remainder + cv_remainder;
    float phase = position * (2.0f / SMN_CONTROL_CYCLE_UNITS) + 0.5f;
    phase -= (float)(int32_t)(phase * 0.5f) * 2.0f;
    if (phase < 0.0f)
        phase += 2.0f;
    return 1.0f - fabsf(phase - 1.0f);
}

float smn_controls_fold(float encoder_position, float cv_position)
{
    if (!isfinite(encoder_position) || !isfinite(cv_position))
        return 0.5f;

    // Reduce before adding to preserve small CV on a large encoder position.
    // Each remainder is in (-6,6), hence phase is bounded to (-3.5,4.5).
    return fold_remainders(cycle_remainder(encoder_position),
                           cycle_remainder(cv_position));
}

void smn_controls_update(uint8_t voice, const float encoder_positions[3],
                         const float cv_positions[3], uint8_t cv_locked)
{
    smn_controls_update_with_dispersion(voice, encoder_positions, cv_positions,
                                        0, 0, cv_locked);
}

void smn_controls_update_with_dispersion(uint8_t voice,
    const float encoder_positions[3], const float cv_positions[3],
    const float dispersion_encoder[3], const float dispersion_cv[3],
    uint8_t cv_locked)
{
    if (voice >= SMN_SYNTH_MAX_VOICES || !encoder_positions || !cv_positions)
        return;

    SmnControlInputs next;
    for (unsigned dim = 0; dim < 3; ++dim) {
        next.cv[dim] = cv_locked ? 0.0f : cv_positions[dim];
        next.dispersion_encoder[dim] = dispersion_encoder ?
            clamp_intensity(dispersion_encoder[dim]) : 0.0f;
        next.dispersion_cv[dim] = !cv_locked && dispersion_cv ?
            clamp_intensity(dispersion_cv[dim]) : 0.0f;
        // Combine manual sources before folding, outside the audio interrupt.
        next.phase[dim] = encoder_positions[dim]
            + dispersion_offset[voice][dim] * next.dispersion_encoder[dim];
    }
    const uint32_t previous = lock_controls();
    inputs[voice] = next;
    initialized[voice] = 1;
    inputs_pending[voice] = 1;
    ++control_revision[voice];
    unlock_controls(previous);
}

// Both callers validate voice. Write into their snapshot directly to avoid
// an extra 48-byte structure copy after the protected volatile read.
static void read_inputs(uint8_t voice, SmnControlInputs *result, uint8_t consume)
{
    const uint32_t previous = lock_controls();
    const uint8_t available = initialized[voice];
    if (available)
        *result = inputs[voice];
    if (consume)
        inputs_pending[voice] = 0;
    unlock_controls(previous);
    // Avoid clearing a whole snapshot before overwriting it every audio block.
    if (!available)
        *result = (SmnControlInputs){0};
}

SmnTimbreControls smn_controls_get(uint8_t voice)
{
    SmnTimbreControls result = {0.5f, 0.5f, 0.5f};
    if (voice >= SMN_SYNTH_MAX_VOICES)
        return result;
    SmnControlInputs snapshot;
    read_inputs(voice, &snapshot, 0);
    float values[3];
    for (unsigned dim = 0; dim < 3; ++dim) {
        const float extra = fminf(snapshot.dispersion_cv[dim],
                                  1.0f - snapshot.dispersion_encoder[dim]);
        values[dim] = smn_controls_fold(snapshot.phase[dim], snapshot.cv[dim]
                                    + dispersion_offset[voice][dim] * extra);
    }
    result.harmonics = values[0];
    result.timbre = values[1];
    result.morph = values[2];
    return result;
}

SmnTimbreControls smn_controls_get_current(uint8_t voice)
{
    SmnTimbreControls result = {0.5f, 0.5f, 0.5f};
    if (voice >= SMN_SYNTH_MAX_VOICES)
        return result;

    const uint32_t previous = lock_controls();
    if (ramp_initialized[voice])
        result = current_controls[voice];
    unlock_controls(previous);
    return result;
}

void smn_controls_init(void)
{
    smn_modulation_reset();
    for (unsigned voice = 0; voice < SMN_SYNTH_MAX_VOICES; ++voice) {
        smn_glide_reset(voice);
        ramp_initialized[voice] = 0;
    }
}

static float current_dispersion(const SmnControlRamp *ramp)
{
    // Derive only the headroom needed by dispersion CV from the shared clock.
    // Clamp small rounding errors so encoder + CV never exceeds full intensity.
    return clamp_intensity(ramp->dispersion_target
                           - (float)ramp->remaining * ramp->dispersion_step);
}

void smn_controls_lock_dispersion(uint8_t voice, uint8_t dim,
    float *position, float direct_cv, float dispersion_encoder,
    float dispersion_cv, uint8_t locking)
{
    if (voice >= SMN_SYNTH_MAX_VOICES || dim >= 3 || !position) return;
    const float dose = clamp_intensity(dispersion_encoder);
    const float cv = fminf(clamp_intensity(dispersion_cv), 1.0f - dose);
    const uint32_t previous = lock_controls();
    SmnControlRamp *r = &ramps[voice][dim];
    float base;
    if (locking) {
        // Freeze the reached combined phase, including dispersion CV but
        // excluding direct CV and internal modulation. Cancel only this manual ramp.
        const float reached = ramp_initialized[voice] ? r->current :
            *position + dispersion_offset[voice][dim] * dose;
        const float reached_dose = ramp_initialized[voice] ? current_dispersion(r) : dose;
        base = reached + dispersion_offset[voice][dim] *
            fminf(clamp_intensity(dispersion_cv), 1.0f - reached_dose);
    } else {
        base = (ramp_initialized[voice] ? r->current : *position)
            - dispersion_offset[voice][dim] * (dose + cv);
    }
    *position = base;
    const float phase = base + (locking ? 0.0f : dispersion_offset[voice][dim] * dose);
    *r = (SmnControlRamp){.current = phase, .target = phase,
                         .dispersion_target = locking ? 0.0f : dose};
    inputs[voice].phase[dim] = phase;
    inputs[voice].cv[dim] = locking ? 0.0f : direct_cv;
    inputs[voice].dispersion_encoder[dim] = locking ? 0.0f : dose;
    inputs[voice].dispersion_cv[dim] = locking ? 0.0f : clamp_intensity(dispersion_cv);
    initialized[voice] = inputs_pending[voice] = 1;
    preparation[voice].finite_phase &= ~(1u << dim);
    preparation[voice].settled = 0;
    ++control_revision[voice];
    unlock_controls(previous);
}

// Return whether current was recalculated, so an unchanged manual phase keeps
// its exact reduction even when a publication updates only live CV.
static uint8_t advance_control(SmnControlRamp *ramp, float target,
                             float dispersion, uint8_t first)
{
    if (first || !isfinite(target)) {
        ramp->current = ramp->target = isfinite(target) ? target : 0.0f;
        ramp->dispersion_target = dispersion;
        ramp->phase_step = ramp->dispersion_step = 0.0f;
        ramp->remaining = 0;
    } else if (target != ramp->target ||
               dispersion != ramp->dispersion_target) {
        const float dose = current_dispersion(ramp);
        // Start each new manual transition from the phase already reached.
        const float error = ramp->current - target;
        ramp->target = target;
        ramp->dispersion_target = dispersion;
        if (isfinite(error)) {
            ramp->phase_step = error * (1.0f / SMN_CONTROL_RAMP_BLOCKS);
            ramp->dispersion_step = (dispersion - dose)
                                  * (1.0f / SMN_CONTROL_RAMP_BLOCKS);
            ramp->remaining = SMN_CONTROL_RAMP_BLOCKS;
        } else {
            // Reject extreme preset values that would produce an infinite ramp.
            ramp->phase_step = ramp->dispersion_step = 0.0f;
            ramp->remaining = 0;
            ramp->current = target;
        }
    } else if (!ramp->remaining) {
        // The ramp has finished and its inputs have not changed.
        return 0;
    }

    if (ramp->remaining)
        --ramp->remaining;
    ramp->current = ramp->target + (float)ramp->remaining * ramp->phase_step;
    return 1;
}

SmnTimbreControls smn_controls_process_lfo(uint8_t voice, float lfo)
{
    SmnTimbreControls result = {0.5f, 0.5f, 0.5f};
    if (voice >= SMN_SYNTH_MAX_VOICES)
        return result;

    const uint8_t first = !ramp_initialized[voice];
    const uint32_t revision = control_revision[voice];
    // Check the live source first: a moving LFO avoids loading/comparing the
    // rest of the cache metadata when H/T/M are actively modulated.
    if ((!(modulation_active[voice] & ((1u << SMN_MOD_HARMONICS) |
            (1u << SMN_MOD_TIMBRE) | (1u << SMN_MOD_MORPH))) ||
         lfo == control_cache[voice].lfo) &&
        !first && control_cache[voice].settled &&
        control_cache[voice].revision == revision)
        return current_controls[voice];

    const uint8_t changed = first || inputs_pending[voice];
    if (changed)
        read_inputs(voice, &preparation[voice].inputs, 1);
    const SmnControlInputs *snapshot = &preparation[voice].inputs;
    if (changed || !preparation[voice].settled) {
        uint32_t manual_remaining = 0;
        uint8_t finite_phase = preparation[voice].finite_phase;
        for (unsigned dim = 0; dim < 3; ++dim) {
            SmnControlRamp *ramp = &ramps[voice][dim];
            if (!changed && !ramp->remaining)
                continue;
            const uint8_t advanced = advance_control(ramp, snapshot->phase[dim],
                snapshot->dispersion_encoder[dim], first);
            manual_remaining |= ramp->remaining;
            if (advanced || !(finite_phase & (1u << dim))) {
                finite_phase &= ~(1u << dim);
                if (isfinite(snapshot->phase[dim]) && isfinite(ramp->current)) {
                    finite_phase |= 1u << dim;
                    preparation[voice].phase[dim] = cycle_remainder(ramp->current);
                }
            }
            float live_cv = 0.0f;
            if (snapshot->dispersion_cv[dim] > 0.0f) {
                const float extra = fminf(snapshot->dispersion_cv[dim],
                                          1.0f - current_dispersion(ramp));
                live_cv = dispersion_offset[voice][dim] * extra;
            }
            preparation[voice].dispersion_cv[dim] = live_cv;
        }
        preparation[voice].finite_phase = finite_phase;
        preparation[voice].settled = !manual_remaining;
    }
    uint32_t remaining = !preparation[voice].settled;
    float values[3];
    for (unsigned dim = 0; dim < 3; ++dim) {
        float live_cv = preparation[voice].dispersion_cv[dim];
        if (modulation_active[voice] & (1u << (dim + SMN_MOD_HARMONICS))) {
            live_cv += advance_modulation(voice, dim + SMN_MOD_HARMONICS) * lfo;
            remaining |= modulation[voice][dim + SMN_MOD_HARMONICS].remaining;
        }
        const float cv = snapshot->cv[dim] + live_cv;
        values[dim] = (preparation[voice].finite_phase & (1u << dim)) && isfinite(cv)
            ? fold_remainders(preparation[voice].phase[dim], cycle_remainder(cv))
            : 0.5f;
    }
    result.harmonics = values[0];
    result.timbre = values[1];
    result.morph = values[2];
    current_controls[voice] = result;
    ramp_initialized[voice] = 1;
    control_cache[voice].revision = revision;
    control_cache[voice].lfo = lfo;
    control_cache[voice].settled = !remaining;
    return result;
}
