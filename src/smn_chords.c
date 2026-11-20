#include "smn_chords.h"
#include "smn_model_selection.h"
#include "smn_patch.h"
#include "params_update.h"
#include "analog_conditioning.h"
#include "smn_resume.h"
#include "timekeeper.h"
#include "ui_modes.h"
#include "quantz_scales.h"
#include "led_cont.h"
#include <math.h>

extern o_params params;
extern o_calc_params calc_params;
extern o_analog analog[NUM_ANALOG_ELEMENTS];
extern enum UI_Modes ui_mode;
extern o_led_cont led_cont;
static volatile int8_t cv_feedback_memory = -1;
static uint32_t cv_feedback_start;

int8_t smn_chord_cv_feedback(uint32_t ticks)
{
    if (ui_mode != PLAY || led_cont.ongoing_display == ONGOING_DISPLAY_PRESET ||
        (uint32_t)(ticks - cv_feedback_start) >= 300 * TICKS_PER_MS) return -1;
    return cv_feedback_memory;
}

void smn_chords_reset(void)
{
    // Only main accesses the bank; no audio/LFO pause or pitch recall needed.
    smn_patch_chords_init(&params.smn_extra);
    smn_resume_request();
}

void smn_chord_store(uint8_t memory)
{
    if (memory >= SMN_CHORD_MEMORIES) return;
    float frequency[NUM_CHANNELS];
    int8_t octave[NUM_CHANNELS];
    // A coherent snapshot, without pausing audio, LFOs or the ADC. Do not
    // leave the parameter callback paused during the logarithms below.
    pause_timer_IRQ(OSC_TIM_number);
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v) {
        frequency[v] = calc_params.qtz_freq[v];
        const uint8_t held = (params.key_sw[v] == ksw_NOTE || params.key_sw[v] == ksw_KEYS) &&
            params.note_on[v];
        octave[v] = held ? calc_params.pitch_octave[v] :
            params.oct[v] < MIN_OCT ? MIN_OCT : params.oct[v] > MAX_OCT ? MAX_OCT : params.oct[v];
    }
    resume_timer_IRQ(OSC_TIM_number);
    params.smn_extra.chord_scale[memory] = params.indiv_scale[0];
    params.smn_extra.chord_root[memory] = scale_root(&params);
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v) {
        // Rounding is for the memory alone, including bypass/quarter tones.
        // qtz_freq already includes the effective octave and CV, but not Fine.
        const float note = frequency[v] > 0 && isfinite(frequency[v]) ?
            roundf(12.0f * log2f(frequency[v] / F_BASE_FREQ)) : NAN;
        const float pitch = note - 12 * octave[v];
        if (!isfinite(pitch) || pitch < -512 || pitch > 512) {
            params.smn_extra.chord_pitch[memory][v] = 0;
            params.smn_extra.chord_octave[memory][v] = INIT_OCT;
        } else {
            params.smn_extra.chord_pitch[memory][v] = (int32_t)pitch;
            params.smn_extra.chord_octave[memory][v] = octave[v];
        }
    }
    smn_resume_request();
}

static void recall(uint8_t memory, uint8_t from_cv)
{
    if (memory >= SMN_CHORD_MEMORIES) return;
    // Scale remains global. Only an enabled -> enabled recall replaces it;
    // legacy banks default to NONE and therefore preserve the current scale.
    const uint8_t saved_scale = params.smn_extra.chord_scale[memory];
    const uint8_t saved_root = params.smn_extra.chord_root[memory];
    // Publish Scale and the six targets together. Only parameter TIM6 is
    // paused; the bounded table build uses no logarithms and audio/LFOs run.
    pause_timer_IRQ(OSC_TIM_number);
    if (scale_quantized() && saved_scale > sclm_NONE &&
        saved_scale < NUM_QTZ_SCALES && saved_root < 12) {
        scale_set(&params, saved_scale, saved_root);
        scale_restore(&params);
    }
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v) {
        if (params.osc_param_lock[v]) continue;
        if (smn_parameter_locked(&params, v, SMN_LOCK_PITCH)) {
            // A targeted pitch lock protects the note, not its articulation.
            // Note always uses its normal quantized-change detection, even locked.
            if (params.key_sw[v] != ksw_MUTE && params.key_sw[v] != ksw_NOTE)
                request_chord_envelope(v);
            continue;
        }
        int32_t pitch = params.smn_extra.chord_pitch[memory][v];
        int8_t octave = params.smn_extra.chord_octave[memory][v];
        if (pitch < -512 || pitch > 512 || octave < MIN_OCT || octave > MAX_OCT) {
            pitch = 0;
            octave = INIT_OCT;
        }
        params.transpose_enc[v] = pitch;
        params.oct[v] = octave;
        params.spread_enc[v] = 0;
        if (params.key_sw[v] != ksw_MUTE && params.key_sw[v] != ksw_NOTE && !from_cv)
            request_chord_envelope(v);
    }
    resume_timer_IRQ(OSC_TIM_number);
    smn_resume_request();
}

void smn_chord_recall(uint8_t memory) { recall(memory, 0); }
void smn_chord_recall_cv(uint8_t memory)
{
    if (memory >= SMN_CHORD_MEMORIES) return;
    recall(memory, 1);
    cv_feedback_start = HAL_GetTick();
    __asm__ volatile("" ::: "memory");
    cv_feedback_memory = memory;
}

uint8_t spread_cv_is_global_gate(void)
{
    if (!analog_jack_plugged(TRANSPOSE_CV)) return 0;
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v)
        if (params.key_sw[v] == ksw_KEYS_EXT_TRIG ||
            params.key_sw[v] == ksw_KEYS_EXT_TRIG_SUSTAIN) return 1;
    return 0;
}

void smn_chords_poll_cv(uint32_t ticks)
{
    static uint32_t last_poll;
    static uint8_t initialized, previous_inhibited, segment;
    if (initialized && (uint32_t)(ticks - last_poll) < 10 * TICKS_PER_MS) return;
    last_poll = ticks;
    const uint8_t gate = spread_cv_is_global_gate();
    const uint8_t enabled = ui_mode == PLAY && !gate;
    const float raw = analog[CHORD_CV].lpf_val;
    // ADC full scale is the original Spread input's 0..5 V range. Hysteresis
    // is 16 counts (~20 mV) on either side of each of the five boundaries.
    const int32_t adc = !isfinite(raw) || raw < 0 ? 0 : raw > 4095 ? 4095 : (int32_t)raw;
    const int32_t scaled = adc * 6;
    const uint8_t current = scaled >= 4095 * 5 ? 5 : scaled / 4095;
    if (!initialized || previous_inhibited || !enabled) {
        segment = current;
        initialized = 1;
    } else if ((scaled > (segment + 1) * 4095 + 16 * 6 && segment < 5) ||
               (scaled < segment * 4095 - 16 * 6 && segment > 0)) {
        segment = current;
        smn_chord_recall_cv(segment);
    }
    // Also rebase after calibration/other UI modes, without an immediate recall.
    previous_inhibited = !enabled;
}
