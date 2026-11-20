// SMN controls and smoothing. Presets save targets, not running ramps.
#pragma once

#include <stdint.h>
#include "smn_synth.h"

#define SMN_CONTROL_CYCLE_UNITS 6u
#define SMN_CONTROL_RAMP_MS 300u
// Round to the nearest audio block: 827 blocks = 300.045 ms at 44.1 kHz.
#define SMN_CONTROL_RAMP_BLOCKS \
    ((SMN_SYNTH_SAMPLE_RATE_HZ * SMN_CONTROL_RAMP_MS + \
      500u * SMN_SYNTH_BLOCK_SIZE) / (1000u * SMN_SYNTH_BLOCK_SIZE))

typedef struct {
    float harmonics;
    float timbre;
    float morph;
} SmnTimbreControls;

// 3 navigation units (50 normal steps) cover minimum -> maximum; 6 units
// cover the full return cycle (100 normal / 500 Fine steps).
// Position zero gives 0.5 rising; existing raw positions preserve direction.
// CV adds to the phase before folding; non-finite inputs give neutral 0.5.
float smn_controls_fold(float encoder_position, float cv_position);

// Called by the existing parameter calculation for each channel, after UI/CV
// updates and preset recalls. Base-only writer; dispersion defaults to zero.
// Publishes encoder targets and live CV separately. No ramp runs in this writer.
void smn_controls_update(uint8_t voice, const float encoder_positions[3],
                         const float cv_positions[3], uint8_t cv_locked);

// Full coherent publication. Dispersion targets/CV are normalized 0..1.
// Encoder position and dispersion share one raw-phase ramp per parameter.
// Direct CV bypasses it; dispersion CV uses the shared clock's dose headroom.
// Its encoder + CV intensity remains capped at one, including during ramps.
// CV lock suppresses both direct and dispersion CV, as in the SWN.
void smn_controls_update_with_dispersion(uint8_t voice,
    const float encoder_positions[3], const float cv_positions[3],
    const float dispersion_encoder[3], const float dispersion_cv[3],
    uint8_t cv_locked);

// On lock, include the reached manual/CV dispersion in the base position.
// On unlock, subtract the current global offset. Publish both changes together.
void smn_controls_lock_dispersion(uint8_t voice, uint8_t dimension,
    float *encoder_position, float direct_cv, float dispersion_encoder,
    float dispersion_cv, uint8_t locking);
void smn_modulation_clear(uint8_t voice, uint8_t destination);

// Coherent snapshot, safe before the first update (defaults to 0.5).
// Returns the immediate target; does not advance the audio ramps.
SmnTimbreControls smn_controls_get(uint8_t voice);

// Last values actually passed to the engine (ramp + CV + folding), for LEDs.
// Read-only coherent snapshot: never advances a ramp. Neutral before audio.
SmnTimbreControls smn_controls_get_current(uint8_t voice);

// Reset transient ramp state before audio starts; keeps published targets.
// Never call concurrently with process(). The first block adopts current
// positions directly, so a startup preset does not glide from arbitrary values.
void smn_controls_init(void);

// Audio calls this once per voice per 16-sample block. Moving an H/T/M or
// dispersion encoder restarts the shared 300 ms ramp from the reached phase.
// The step is precomputed and the ramp reaches its target exactly.
// CV is added after the ramp, before folding, and never restarts its clock.

// Internal modulation: live ramps stay private; presets store only integer targets.
enum SmnModulationDestination {
    SMN_MOD_GAIN, SMN_MOD_HARMONICS, SMN_MOD_TIMBRE, SMN_MOD_MORPH,
    SMN_MOD_PITCH, SMN_MOD_PAN,
    SMN_MOD_DESTINATIONS
};
// Browse uses HALFSTEP decoding: 24 electrical cycles -> 48 events per turn.
// Mechanical correspondence remains to be checked on the physical module.
#define SMN_MOD_BROWSE_STEPS_PER_TURN 48u
#define SMN_MOD_NORMAL_STEPS (2u * SMN_MOD_BROWSE_STEPS_PER_TURN)
#define SMN_MOD_FINE_DIVISOR 5u
#define SMN_MOD_FULL_STEPS (SMN_MOD_NORMAL_STEPS * SMN_MOD_FINE_DIVISOR)
#define SMN_MOD_PITCH_MAX_SEMITONES 2.0f

// Quantized glide: one duration per voice, in the existing persistent ms fields.
#define SMN_GLIDE_FINE_DIVISOR 5u
#define SMN_GLIDE_TIME_COUNT (18u * SMN_GLIDE_FINE_DIVISOR + 1u)
#define SMN_GLIDE_MAX_MS 30000u
extern const uint32_t smn_glide_times_ms[SMN_GLIDE_TIME_COUNT];
uint8_t smn_glide_time_index(uint32_t milliseconds);
enum SmnGlideFlags {
    SMN_GLIDE_ENABLED = 1u, // Scale active, normal play; bypass cancels a ramp.
    SMN_GLIDE_LEGATO = 2u, // Mute, automatic Note, or an envelope open before the event.
    SMN_GLIDE_ATTACK = 4u  // Retrigger, including an unchanged destination.
};
// Parameter writer. Prepares a transition only on destination/attack events.
// Publishes the corresponding tuned pitch under the same short lock as the
// glide state. Expensive preparation stays preemptible, outside the audio IRQ.
void smn_glide_update(uint8_t voice, float destination_hz, uint32_t milliseconds,
    uint8_t flags, float tuned_pitch_hz, float *published_pitch);
void smn_glide_reset(uint8_t voice);
// Audio calls this once per block. Returns 1 and writes the intermediate
// frequency in Hz to *frequency_hz, before Fine. The caller applies live
// tuning, bounds and pitch noise.
// Returns 0 without writing when idle/arrived: use the existing tuned pitch.
uint8_t smn_glide_process_pitch(uint8_t voice, float *frequency_hz);

// UI writers briefly mask interrupts; ramps advance only in the audio reader.
// Signed encoder events change a bounded 0..1 dose; identical/clipped targets
// never restart a ramp. Full scale: 4095 gain units / 3 navigation units /
// 2 semitones / 1 pan unit. Every destination uses five Fine ticks per normal event.
void smn_modulation_adjust(uint8_t voice, uint8_t destination, int16_t events, uint8_t fine);
float smn_modulation_target(uint8_t voice, uint8_t destination);
void smn_modulation_reset(void);
void smn_modulation_clear_gain(uint8_t voice);

// Preset/undo helpers, outside audio. One coherent group per voice. Restore
// adopts exact targets immediately, without saving/replaying transition ramps.
// Out-of-range stored counters are neutralized to zero.
void smn_modulation_capture_steps(uint8_t voice, uint16_t steps[SMN_MOD_DESTINATIONS]);
void smn_modulation_restore_steps(uint8_t voice, const uint16_t steps[SMN_MOD_DESTINATIONS]);

// Same controls with the existing unipolar LFO/envelope, before its CV gain.
// H/T/M modulation bypasses position ramps and CV locks; fold the sum once.
SmnTimbreControls smn_controls_process_lfo(uint8_t voice, float lfo);
// Zero-source compatibility for offline callers; no extra firmware wrapper.
static inline SmnTimbreControls smn_controls_process(uint8_t voice)
{
    return smn_controls_process_lfo(voice, 0.0f);
}
// Called once per audio block/voice, including muted voices. A forbidden mode
// clears both the target and reached gain dose immediately; H/T/M are intact.
float smn_modulation_process_gain(uint8_t voice, float lfo, uint8_t allowed);

// Audio calls this once per block. Returns Hz after quantization/tuning,
// without changing the base pitch. The first two LFO banks and all envelope
// modes modulate upwards; other LFO banks modulate around the base note.
// Caller bypasses this reader during calibration (dose preserved, ramp paused).
float smn_modulation_process_pitch(uint8_t voice, float frequency_hz, float lfo,
                                   uint8_t envelope_mode, uint16_t shape);

// Once per block, bipolar for every shape and play mode. Full dose offsets
// the manual position by +/-1, clamped to 0..1. Never writes the manual pan.
float smn_modulation_process_pan(uint8_t voice, float pan, float lfo);

// Unmasked atomic counter loads; caller checks two snapshots for consistency.
void smn_modulation_peek_steps(uint8_t voice, uint16_t steps[SMN_MOD_DESTINATIONS]);
