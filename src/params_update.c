/*
 * params_update.c - Parameters
 *
 * Authors: Dan Green (danngreen1@gmail.com), Hugo Paris (hugoplho@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * See http://creativecommons.org/licenses/MIT/ for more information.
 *
 * -----------------------------------------------------------------------------
 */

#include "params_update.h"
#include "params_changes.h"
#include "led_cont.h"
#include "gpio_pins.h"
#include "exp_1voct_10_41V.h"
#include "flash_params.h"
#include "math.h"
#include "lfo_wavetable_bank.h"
#include "quantz_scales.h"
#include "led_cont.h"
#include "adc_interface.h"
#include "analog_conditioning.h"
#include "UI_conditioning.h"
#include "timekeeper.h"
#include "system_settings.h"
#include "math_util.h"
#include "ui_modes.h"
#include "key_combos.h"
#include "preset_manager.h"
#include "preset_manager_UI.h"
#include "preset_manager_undo.h"
#include "params_pitch.h"
#include "params_lfo_clk.h"
#include "params_lfo_period.h"
#include "params_lfo.h"
#include "oscillator.h"
#include "drivers/flashram_spidma.h"
#include "smn_chords.h"
#include "smn_controls.h"
#include "smn_models.h"
#include "smn_model_selection.h"
#include "smn_resume.h"

_Static_assert(NUM_CHANNELS == SMN_SYNTH_MAX_VOICES, "SMN/SWN voice count");
_Static_assert(NUM_QTZ_SCALES == 25, "Update Polyrotate Scale ID bound in params_update.h");
_Static_assert(sizeof(((o_params *)0)->PADDING) > SMN_POLYROTATE_BYTE, "Polyrotate RAM metadata fits");

extern enum UI_Modes ui_mode;
extern o_lfos lfos;
extern SystemCalibrations *system_calibrations;

// Live CV, separate from the encoder targets saved in presets.
static float smn_dispersion_cv[3];
// Main publishes one-byte requests with TIM6 paused; TIM6 consumes them.
static volatile uint8_t chord_envelope_pending[NUM_CHANNELS];
// TIM6 only: 0 none, 1 rest, 2 overlap, 3 automatic Note (glide already prepared).
static uint8_t glide_attack[NUM_CHANNELS];
// Nine TIM6 pitch updates at ~1.8 kHz: ~5 ms after a fresh CV/Gate attack.
// Apply late pitch changes directly so they do not delay the envelope.
#define CVGATE_GLIDE_GUARD_UPDATES 9
static uint8_t cvgate_glide_guard[NUM_CHANNELS];
// TIM6 only: a Browse edit consumes the LFO buttons until full release.
static uint8_t breathe_lfo_consumed;

extern	o_systemSettings	system_settings;
extern	o_analog	analog[NUM_ANALOG_ELEMENTS];
extern	o_macro_states macro_states;
extern	o_rotary	rotary[NUM_ROTARIES];
extern	o_button	button[NUM_BUTTONS];
extern	o_switch	hwSwitch[NUM_SWITCHES];
extern	o_monoLed 	monoLed[NUM_MONO_LED];
extern	o_led_cont	led_cont;


extern const uint8_t ALL_CHANNEL_MASK;

// TIM6-owned, transient candidates for automatic Note attacks only.
static uint8_t note_candidate[NUM_CHANNELS];
static int8_t note_candidate_oct[NUM_CHANNELS];
static uint8_t note_candidate_reads[NUM_CHANNELS];

const int8_t		CHORD_LIST[NUM_CHORDS][NUM_CHANNELS] =
{
	// DEFAULT
	{ 0 	, 0 	, 0 	, 0 	, 0 	, 0		}  , 	// NONE

	// FIFTH
	{ 0 	, 0 	, 7 	, 7 	, 0 	, 0		}  , 	// FIFTH
	{ 0 	, 0 	, 7 	, 7 	, -5 	, -5	}  , 	// FIFTH w/ inversion
	{ 0 	, 12	, 7	, 19	, -12	, -5	}  , 	// FIFTH w/ oct

	// MAJOR
	{ 0 	, 0 	, 4 	, 4 	, 7 	, 7		}  , 	// M3rd
	{ -8	, -5	, 0 	, 4 	, 7 	, 7	}  ,	// M3rd w inv
	{ -12	, -5	, 0 	, 4 	, 7 	, 12	}  ,	// M3rd w/ oct

	// MINOR
	{ 0 	, 0 	, 3 	, 3 	, 7 	, 7		}  , 	// m3rd
	{ -9	, -5	, 0 	, 3 	, 7 	, 7	}  ,	// m3rd w inv
	{ -12	, -5	, 0 	, 3 	, 7 	, 12	}  ,	// m3rd w/ oct

	// MAJOR EXT chords (6th, 7th 9th, 11th)
	{ 0 	, 0 	, 4 	, 4 	, 7 	, 9		}  , 	// M6th
	{ -12 	, 0 	, 4 	, 7 	, 9 	, 9		}  , 	// M6th w/oct
	{ -12 	, -3	, 0 	, 4 	, 4 	, 9		}  , 	// M6th w/ inv
	{ 0 	, 0 	, 4 	, 4 	, 7 	, 11	}  , 	// M7th
	{ -12 	, 0 	, 4 	, 7 	, 11 	, 11	}  , 	// M7th w/oct
	{ -12 	, -1	, 0 	, 4 	, 7 	, 11	}  , 	// M7th w/ inv
	{ 0 	, 0		, 4 	, 7 	, 11	, 14	}  , 	// M9th
	{ 0 	, 4		, 7 	, 11 	, 14	, 18	}  , 	// M11th

	// MINOR EXT chords (6th, 7th 9th, 11th)
	{ 0 	, 0 	, 3 	, 3 	, 7 	, 9		}  , 	// m6th
	{ -12 	, 0 	, 3 	, 7 	, 9 	, 9		}  , 	// m6th w/oct
	{ -12 	, -3	, 0 	, 3 	, 3 	, 9		}  , 	// m6th w/ inv
	{ 0 	, 0 	, 3 	, 3 	, 7 	, 10	}  , 	// m7th
	{ -12 	, 0 	, 3 	, 7 	, 10 	, 10	}  , 	// m7th w/oct
	{ -12 	, -2	, 0 	, 3 	, 7 	, 10	}  , 	// m7th w/ inv
	{ 0 	, 0		, 3 	, 7 	, 10	, 14	}  , 	// m9th
	{ 0 	, 3		, 7 	, 10 	, 14	, 17	}   	// m11th
};

extern const float exp_1voct_10_41V[4096];

o_params			params;
o_calc_params		calc_params;
// Audio-range mode cache must not survive a preset recall or global reset.
static uint8_t lfo_mode_cached[NUM_CHANNELS];

//Todo: replace with init_param_object(&params), plus a few other differences
void init_params(void){

	uint8_t  i;
	uint16_t j;

	// global & display
	smn_parameter_locks_init(&params);
	smn_dispersion_init(&params);
	smn_breathe_init(&params);
	for (uint8_t dim = 0; dim < 3; ++dim) smn_dispersion_cv[dim] = 0.0f;
	params.disppatt_enc = 0;
	params.noise_on = 0;
	params.spread_cv = 0;

	for (i=0; i<NUM_CHANNELS; i++){
		params.key_sw[i] = ksw_MUTE;

		params.note_on[i] = 1;
		calc_params.level[i] = 4093;
		calc_params.adjusting_pan_state[i] = pan_INACTIVE;
		calc_params.cached_level[i] = 0.f;

		// individual locks
		params.osc_param_lock[i] = 0;
		params.wtsel_lock[i] = 0;
		params.wt_pos_lock[i] = 0;

		params.wtsel_spread_enc[i] = 0;
		calc_params.wtsel[i] = 1;
		params.wtsel_enc[i] = 0;
		params.wt_bank[i] = 0;

		params.wt_browse_step_pos_enc[i] = 0;
		params.wt_nav_enc[0][i] = 0;
		params.wt_nav_enc[1][i] = 0;
		params.wt_nav_enc[2][i] = 0;

		params.spread_enc[i] = 0;
		calc_params.transpose[i] = 0;
		params.transpose_enc[i] = 0;

		params.pan[i] = default_pan(i);

		calc_params.gate_in_is_sustaining[i]	= 0;

		calc_wt_pos(i);

		// flags
		calc_params.already_handled_button[i] = 0;

		for (j=0; j<NUM_ARM_FLAGS; j++)
			calc_params.armed[j][i] = 0;

		params.random[i] = 1.0;
	}

	calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 0;
	calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 0;
	for (i=0; i<(SMN_LEGACY_SPHERE_MASK_BYTES); i++)
		params.enabled_spheres[i]=0xFF;
	smn_model_selection_init(&params);
	update_wtsel(); // Correct initial model even after the catalog is reordered.

}

void init_pitch_params(void)
{
	uint8_t chan;
	for (chan = 0; chan<NUM_CHANNELS; chan++)
	{
		calc_params.tuning[chan]				= 1;
		calc_params.transposition[chan]		= 1;

		params.finetune[chan]					= 0;
		params.transpose_enc[chan]				= 0;

		params.oct[chan]						= INIT_OCT;
		params.indiv_scale[chan]				= 0;
		params.indiv_scale_buf[chan]			= params.indiv_scale[chan];

		params.qtz_note_changed[chan]			= 0;
	}
	scale_set(&params, sclm_NONE, 0);
	init_quantz_scales();
}

void init_param_object(o_params *t_params){
	uint8_t chan;
    smn_patch_extra_init(&t_params->smn_extra);
    t_params->PADDING[SMN_POLYROTATE_BYTE] = 0;

	// global & display
	t_params->dispersion_enc			= 0;
	t_params->disppatt_enc				= 0;
	t_params->noise_on					= 0;
	t_params->spread_cv				= 0;

	// individual params
	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		t_params->key_sw[chan]					= ksw_MUTE;
		t_params->note_on[chan]				= 1;

		// individual locks
		t_params->osc_param_lock[chan]			= 0;
		t_params->wtsel_lock[chan]				= 0;

		t_params->wtsel_enc[chan]				= 0;
		t_params->wtsel_spread_enc[chan]		= 0;
		t_params->wt_bank[chan]					= 0;

		t_params->wt_browse_step_pos_enc[chan]	= 0.0;

		t_params->wt_pos_lock[chan]				= 0;
		t_params->wt_nav_enc[0][chan]			= 0;
		t_params->wt_nav_enc[1][chan]			= 0;
		t_params->wt_nav_enc[2][chan]			= 0;

		t_params->spread_enc[chan]				= 0;
		t_params->transpose_enc[chan]			= 0;

		t_params->random[chan]					= 1.0;

		t_params->finetune[chan]				= 0;

		t_params->oct[chan]					= INIT_OCT;
		t_params->indiv_scale[chan]			= 0;
		t_params->indiv_scale_buf[chan]		= 0;

		t_params->pan[chan]						= default_pan(chan);
		t_params->qtz_note_changed[chan]		= 0;

	}

	for (uint8_t i=0; i<(SMN_LEGACY_SPHERE_MASK_BYTES); i++)
		t_params->enabled_spheres[i]=0xFF;
	smn_model_selection_init(t_params);
	smn_parameter_locks_init(t_params);
	smn_dispersion_init(t_params);
	smn_breathe_init(t_params);
	scale_set(t_params, sclm_NONE, 0);

}

void init_calc_params(void)
{
    smn_polyrotate_reset();
	uint8_t chan, j;
	for (chan=0; chan<NUM_CHANNELS; chan++)
	{

		chord_envelope_pending[chan] = 0;
        note_candidate_reads[chan] = 0;
        glide_attack[chan] = 0;
        cvgate_glide_guard[chan] = 0;
        smn_glide_reset(chan);
        calc_params.pitch_octave[chan] = INIT_OCT;
        // Preset/undo may restore a live envelope: refresh its pitch once,
        // without restarting the saved envelope cycle.
        if (params.key_sw[chan] == ksw_NOTE || params.key_sw[chan] == ksw_KEYS)
            params.new_key[chan] = 1;
		calc_params.prev_qtz_note[chan] = 0xFF;
		calc_params.prev_qtz_oct[chan] = 0xFF;
		calc_params.gate_in_is_sustaining[chan] = 0;
		lfo_mode_cached[chan] = 0;

		for (j = 0; j < NUM_ARM_FLAGS; j++)
			calc_params.armed[j][chan] = 0;

		calc_params.adjusting_pan_state[chan] = pan_INACTIVE;
		calc_params.cached_level[chan] = 0.f;
		calc_params.already_handled_button[chan] = 0;

	}
	calc_params.keymode_pressed = 0;
	calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 0;
	calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 0;
	calc_params.button_safe_release[0] = 0;
	calc_params.button_safe_release[1] = 0;
}

void check_reset_navigation(void)
{
	// These shortcuts only affect manual targets; live CV and audio ramps stay active.
	if (!rotary_pressed(rotm_PRESET))
		return;

	for (uint8_t dim = 0; dim < 3; ++dim)
	{
		if (key_combo_reset_navigation(dim))
		{
            cancel_chord_gesture();
			exit_preset_manager();
			stop_all_displays();
			smn_dispersion_set(&params, dim, 0.0f);

			for (uint8_t i = 0; i < NUM_CHANNELS; ++i)
				if (!params.wt_pos_lock[i] && !smn_parameter_locked(&params, i, dim))
					params.wt_nav_enc[dim][i] = 0.0f;
		}
	}

	if (key_combo_reset_model_sel())
	{
        cancel_chord_gesture();
		exit_preset_manager();
		stop_all_displays();

		// Reset every manual selection to the first catalog entry, independent of ID.
		for (uint8_t i = 0; i < NUM_CHANNELS; ++i)
		{
			params.wtsel_spread_enc[i] = 0;
			params.wtsel_enc[i] = 0;
		}
		update_wtsel();
	}
}

static volatile uint8_t polyrotate_calibrating;

void cache_uncache_pitch_params(enum CacheUncache cache_uncache)
{
    static uint8_t polyrotate_cached;
    if (cache_uncache == CACHE) {
        polyrotate_calibrating = 1;
        polyrotate_cached = smn_polyrotate_enabled(&params);
    }
	static int16_t		finetune				[NUM_CHANNELS];
	static float		tuning					[NUM_CHANNELS];
	static int32_t		transpose				[NUM_CHANNELS];
	static float		transposition			[NUM_CHANNELS];
	static int8_t		oct					[NUM_CHANNELS];
	static uint8_t		indiv_scale			[NUM_CHANNELS];
	static uint8_t		indiv_scale_buf			[NUM_CHANNELS];

	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++){
		switch (cache_uncache)
		{
			case CACHE:
				finetune[chan]					= params.finetune[chan];
				tuning[chan]					= calc_params.tuning[chan];
				transpose[chan]				= params.transpose_enc[chan];
				transposition[chan]			= calc_params.transposition[chan];
				oct[chan]						= params.oct[chan];
				indiv_scale[chan]				= params.indiv_scale[chan];
				indiv_scale_buf[chan]			= params.indiv_scale_buf[chan];
				break;

			case UNCACHE:
				params.finetune[chan]			= finetune[chan];
				calc_params.tuning[chan]		= tuning[chan];
				params.transpose_enc[chan]		= transpose[chan];
				calc_params.transposition[chan] = transposition[chan];
				params.oct[chan]				= oct[chan];
				params.indiv_scale[chan]		= indiv_scale[chan];
				params.indiv_scale_buf[chan]  	= indiv_scale_buf[chan];
				break;
		}
	}
	if (cache_uncache == UNCACHE) {
        scale_restore(&params);
        params.PADDING[SMN_POLYROTATE_BYTE] = polyrotate_cached &&
            scale_quantized() && analog_jack_plugged(TRANSPOSE_CV);
        polyrotate_calibrating = 0;
    }
}

static uint8_t new_key_armed[NUM_CHANNELS] = {0};
// Lives outside params/calc_params so a global reset cannot free held A–F.
// Main sets this byte; TIM6 clears it after all A–F buttons are released.
// A single byte avoids a shared bitmask read/modify/write race.
static volatile uint8_t preset_af_consumed;
// Selection gestures remain consumed until Scale AND all A-F are released,
// including when Scale is released first or a global reset happens mid-gesture.
static volatile uint8_t glide_af_consumed;
static volatile uint8_t parameter_lock_af_consumed;

static uint8_t parameter_lock_context(void)
{
    return ui_mode == PLAY && !rotary_pressed(rotm_PRESET) &&
        !rotary_pressed(rotm_OCT) && !rotary_pressed(rotm_WAVETABLE) &&
        !rotary_pressed(rotm_LFOSPEED) && !rotary_pressed(rotm_LFOSHAPE) &&
        !button_pressed(butm_LFOVCA_BUTTON) && !button_pressed(butm_LFOMODE_BUTTON);
}

static uint8_t parameter_lock_gesture_held(void)
{
    return !macro_states.all_af_buttons_released && switch_pressed(FINE_BUTTON) &&
        parameter_lock_context() &&
        (rotary_pressed(rotm_DEPTH) || rotary_pressed(rotm_LATITUDE) ||
         rotary_pressed(rotm_LONGITUDE) || rotary_pressed(rotm_TRANSPOSE));
}

void read_parameter_locks(void)
{
    // Main owns this state. The consumption byte alone is shared with TIM6.
    // Keep it outside params/calc_params so reset/undo cannot free held keys.
    static uint8_t active, cancelled, pending[NUM_CHANNELS];
    static uint32_t motion_start;
    const uint8_t fine = switch_pressed(FINE_BUTTON);
    const uint8_t af = !macro_states.all_af_buttons_released;
    if (!active) {
        if (!fine || !af) return;
        active = 1;
        cancelled = !parameter_lock_context() || encoder_motion_pending();
        motion_start = encoder_fine_af_motion_start();
    }
    if (!parameter_lock_context() || encoder_motion_revision() != motion_start)
        cancelled = 1;
    if (!cancelled && encoder_motion_pending()) cancelled = 1;

    const uint8_t encoders[4] = {rotm_DEPTH, rotm_LATITUDE, rotm_LONGITUDE, rotm_TRANSPOSE};
    uint8_t dimensions = 0;
    for (unsigned d = 0; d < 4; ++d)
        if (rotary_pressed(encoders[d])) dimensions |= 1u << d;
    if (fine && af && dimensions && parameter_lock_context()) {
        parameter_lock_af_consumed = 1;
        for (unsigned v = 0; v < NUM_CHANNELS; ++v)
            if (button_pressed(v)) pending[v] |= dimensions;
    }
    if (parameter_lock_af_consumed || cancelled) {
        for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
            calc_params.already_handled_button[v] = 1;
            calc_params.armed[armf_NOTE_ON][v] = 0;
            calc_params.lock_change_staged[v] = 0;
        }
    }
    // Without a targeted candidate, a released A-F completes the ordinary
    // channel gesture even if Fine remains down (Preset/undo rules included).
    if (!parameter_lock_af_consumed && !af) {
        active = cancelled = 0;
        return;
    }
    // Wait for all gesture keys, in any release order. Rotations continue to
    // use their normal readers; even a turn at a bound cancels both locks.
    if (fine || af || dimensions) return;
    if (!cancelled) for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
        if (!pending[v]) continue;
        // Main owns pitch encoders and these toggles. Pause only TIM6 while
        // publishing the baked base and its mask; audio/LFO continue.
        pause_timer_IRQ(OSC_TIM_number);
        uint8_t mask = params.PADDING[92 + v];
        for (unsigned d = 0; d < 4; ++d) {
            if (!(pending[v] & (1u << d))) continue;
            const uint8_t locking = !(mask & (1u << d));
            if (d < 3) {
                smn_controls_lock_dispersion(v, d, &params.wt_nav_enc[d][v],
                    params.wt_pos_lock[v] ? 0.0f : params.wt_nav_cv[d],
                    smn_dispersion_get(&params, d),
                    params.wt_pos_lock[v] ? 0.0f : smn_dispersion_cv[d], locking);
            } else {
                const uint8_t chord = _WRAP_I8(params.spread_enc[v], 0, NUM_CHORDS);
                params.transpose_enc[v] += (locking ? 1 : -1) * CHORD_LIST[chord][v];
                start_ongoing_display_transpose();
            }
            mask ^= 1u << d;
        }
        smn_parameter_lock_set(&params, v, mask);
        resume_timer_IRQ(OSC_TIM_number);
    }
    for (unsigned v = 0; v < NUM_CHANNELS; ++v) pending[v] = 0;
    active = cancelled = parameter_lock_af_consumed = 0;
}

static uint8_t glide_selection_held(void)
{
    return ui_mode == PLAY && rotary_pressed(rotm_OCT) &&
        !rotary_pressed(rotm_PRESET) && !rotary_pressed(rotm_WAVETABLE);
}

static uint8_t channel_buttons_consumed(void)
{
    return preset_af_consumed || glide_af_consumed || glide_selection_held() ||
        parameter_lock_af_consumed || parameter_lock_gesture_held();
}

static void capture_glide_attack(uint8_t chan)
{
    // First attack wins if two sources coincide before update_pitch().
    if (!glide_attack[chan])
        glide_attack[chan] = params.note_on[chan] && lfos.out_lpf[chan] > 0.0f ? 2 : 1;
}

void read_ext_trigs(void)
{
	uint8_t chan;
	uint8_t chans_in_cvgate_mode=0;

	static uint8_t last_trig_level[NUM_CHANNELS+1]={0};
	// CV/Gate edges must remain visible while its channel button is held.
	static uint8_t last_cvgate_level[NUM_CHANNELS]={0};
	uint8_t trig_level[NUM_CHANNELS+1]={0};
	uint8_t sustain_level[NUM_CHANNELS];

	trig_level[0] = analog[WTSEL_CV].raw_val > 2048;
	trig_level[1] = analog[DISP_CV].raw_val > 2048;
	trig_level[2] = analog[DEPTH_CV].raw_val > 2048;
	trig_level[3] = analog[DISPPAT_CV].raw_val > 2048;
	trig_level[4] = analog[LATITUDE_CV].raw_val > 2048;
	trig_level[5] = analog[WTSEL_SPREAD_CV].raw_val > 2048;
	trig_level[6] = analog[CHORD_CV].raw_val > 2048;

	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		uint8_t is_sustaining = 0;
		const uint8_t key = button_pressed(chan) && !channel_buttons_consumed();
		if (params.key_sw[chan]==ksw_KEYS_EXT_TRIG || params.key_sw[chan]==ksw_KEYS_EXT_TRIG_SUSTAIN)
		{
			chans_in_cvgate_mode++;

			if (analog_jack_plugged(A_VOCT+chan) || key) {
				const uint8_t gate_attack = params.key_sw[chan]==ksw_KEYS_EXT_TRIG
                    ? trig_level[chan] && !last_cvgate_level[chan]
                    : (trig_level[chan] || key) && !last_trig_level[chan];
				// In CV/Gate, read_noteon() handles the button independently.
				if (gate_attack)
				{
                    capture_glide_attack(chan);
					lfos.cycle_pos[chan] = 0.f;
					params.note_on[chan] = 1;
					params.new_key[chan] = 1;
					if (params.key_sw[chan]!=ksw_KEYS_EXT_TRIG)
                        new_key_armed[chan] = 0;
				}
				if (params.key_sw[chan]==ksw_KEYS_EXT_TRIG_SUSTAIN && (trig_level[chan] || key))
					is_sustaining = 1;
			}
		}
		last_trig_level[chan] = (trig_level[chan] || key);
		last_cvgate_level[chan] = trig_level[chan];
		sustain_level[chan] = is_sustaining;
	}

	//Transpose CV jack must be patched for gate to work on Chord CV
	const uint8_t global_gate = chans_in_cvgate_mode &&
        analog_jack_plugged(TRANSPOSE_CV) && trig_level[6];
	const uint8_t global_attack = global_gate && !last_trig_level[6];
	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		// TIM9 may preempt TIM6 here. Publish only the combined level so a
		// held global gate never exposes a false release to the envelope.
		calc_params.gate_in_is_sustaining[chan] = sustain_level[chan] ||
            (global_gate && params.key_sw[chan]==ksw_KEYS_EXT_TRIG_SUSTAIN);
		if (global_attack)
		{
            capture_glide_attack(chan);
			lfos.cycle_pos[chan] = 0.f;
			params.note_on[chan] = 1;
			params.new_key[chan] = 1;
			if (params.key_sw[chan]!=ksw_KEYS_EXT_TRIG)
                new_key_armed[chan] = 0;
		}
	}
	last_trig_level[6] = trig_level[6];
}
void request_chord_envelope(uint8_t chan)
{
    if (chan < NUM_CHANNELS) chord_envelope_pending[chan] = 1;
}

static void start_key_envelope(uint8_t chan)
{
    // A single attack/retrigger, never restarted by a held button.
    capture_glide_attack(chan);
    lfos.cycle_pos[chan] = 0;
    params.new_key[chan] = 1;
    params.note_on[chan] = 1;
    params.qtz_note_changed[chan] = 0;
}

void read_noteon(uint8_t i)
{
    if (chord_envelope_pending[i]) {
        chord_envelope_pending[i] = 0;
        if (ui_mode == PLAY && !params.osc_param_lock[i] && params.key_sw[i] != ksw_MUTE)
            start_key_envelope(i);
    }
	if (ui_mode == PLAY)
	{
		// Button mode: Mute
		if (params.key_sw[i] == ksw_MUTE)
		{
			if (!calc_params.already_handled_button[i] && button_pressed(i))
			{
				calc_params.armed[armf_NOTE_ON][i] = 1;
			}

			else if (button_released(i))
			{
				if (calc_params.armed[armf_NOTE_ON][i])
				{
					calc_params.armed[armf_NOTE_ON][i] = 0;
					if (!calc_params.already_handled_button[i])
					{
						if (calc_params.lock_change_staged[i]==1) {
							toggle_lock(i);
							calc_params.lock_change_staged[i] = 2;
						}
						else
							params.note_on[i] = 1 - params.note_on[i];
					}
					else
						calc_params.lock_change_staged[i] = 0;
				}
				calc_params.already_handled_button[i] = 0;
			}
		}

		// Button Mode: Note/Keyboard/CVGate/CVGateSus
		// ... and auto-notes at qtz crossings
		else
		{
			const uint8_t key_down = button_pressed(i) != RELEASED;
			if (key_down && !channel_buttons_consumed())
			{
				if (!new_key_armed[i]) {
                    new_key_armed[i] = 1;
                    start_key_envelope(i);
                }
			}
			else // Released or consumed by a selection gesture.
			{
				if (!key_down)
				{
					new_key_armed[i] = 0;
					// Keyboard envelopes finish naturally in update_lfo_wt_pos().

					if (!calc_params.already_handled_button[i])
					{
						if (calc_params.lock_change_staged[i]==1) {
							toggle_lock(i);
							calc_params.lock_change_staged[i] = 2;
						}
					}
					else
						calc_params.lock_change_staged[i] = 0;

					calc_params.already_handled_button[i] = 0;
				}
			}
            // Automatic Note is independent of a held channel button.
            if (params.key_sw[i] == ksw_NOTE && scale_quantized() &&
                !params.note_on[i] && params.qtz_note_changed[i] == 1) {
                start_key_envelope(i);
                // update_pitch accepted this stable destination on the previous tick.
                // Keep its glide running instead of treating this as a fresh attack.
                glide_attack[i] = 3;
            }
		}
	}

	// other UI modes
	else{
		params.note_on[i] = 1;
	}
}

float default_pan(uint8_t chan)
{
	// Hardware outputs: pan 0 = left, pan 1 = right (see audio diagnostic).
	return (chan&1) ? 1.f : 0.f;
}

void read_level_and_pan(uint8_t chan)
{
	static float last_slider_val[NUM_CHANNELS] = {0};
	float slider_val = analog[A_SLIDER + chan].lpf_val;
	float slider_motion = fabs(analog[A_SLIDER + chan].lpf_val - last_slider_val[chan]);
	float level = 0.f;

	if (slider_motion > 20)
	{
		last_slider_val[chan] = analog[A_SLIDER + chan].lpf_val;
		if (button_pressed(chan))
		{
			calc_params.already_handled_button[chan] = 1;
			calc_params.adjusting_pan_state[chan] = pan_PANNING;
			params.pan[chan] = slider_val / 4095.f;
		}
	}

	switch (calc_params.adjusting_pan_state[chan]) {
		case (pan_PANNING):
			level = calc_params.cached_level[chan];
			if (!button_pressed(chan)) {
				calc_params.adjusting_pan_state[chan] = pan_CACHED_LEVEL;
			}
			break;

		case (pan_CACHED_LEVEL):
			level = calc_params.cached_level[chan];
			if (fabs(slider_val - calc_params.cached_level[chan]) < 10) {
				calc_params.adjusting_pan_state[chan] = pan_INACTIVE;
			}
			break;

		case (pan_INACTIVE):
			level = slider_val - 20.f;
			calc_params.cached_level[chan] = level;
			break;
	}

	// Advance even while muted, and clear the dose in the excluded play modes.
	const float internal_gain = smn_modulation_process_gain(chan, lfos.out_lpf[chan],
		params.key_sw[chan] == ksw_MUTE && !lfos.to_vca[chan]);
	//Adjust level by CV and Mute button
	if (!params.note_on[chan])
		calc_params.level[chan] = 0.f;

	else {
		if (lfos.to_vca[chan])	level *= lfos.out_lpf[chan];
		// Add to the fader, cap it BEFORE external VCA CV. Zero external CV
		// and Mute always close the voice, including with a raised internal dose.
		else if (internal_gain > 0.0f)
			level = _CLAMP_F(_CLAMP_F(level, 0.f, 4095.f) + internal_gain, 0.f, 4095.f);

		level *= read_vca_cv(chan);

		calc_params.level[chan] = _CLAMP_F(level, 0.f, 4095.f);
	}

}

float read_vca_cv(uint8_t chan)
{
	//No VCA CV if switch is set to V/oct, or if Key Mode is Key or Note
	if ((params.voct_switch_state[chan] == SW_VOCT) || (params.key_sw[chan] != ksw_MUTE))
		return 1.0;
	else
	if (analog[A_VOCT + chan].plug_sense_switch.pressed == RELEASED)
		return 1.0;
	else
	{
		//Handle Bipolar setting by treating negative voltage as 0
		if (analog[A_VOCT + chan].polarity == AP_UNIPOLAR)
			return ((analog[A_VOCT + chan].lpf_val / 2047.5) * F_SCALING_MAX_VCACV_GAIN);
		else {
			return (_CLAMP_F(analog[A_VOCT + chan].lpf_val - 2048.0, 0.0, 2048.0) / 2048.0 ) * F_SCALING_MAX_VCACV_GAIN;
		}
	}
}

// Per-voice LFO modes and routing.

void read_lfoto_vca_vco(uint8_t i){

	static uint8_t any_button_pressed;

	if (button_pressed(butm_LFOVCA_BUTTON)
			&& !calc_params.keymode_pressed
			&& (params.key_sw[i] == ksw_MUTE)
			&& !calc_params.already_handled_button[butm_LFOVCA_BUTTON]) {

		start_ongoing_display_lfo_tovca();

		if (!macro_states.all_af_buttons_released){
			any_button_pressed = 1;
			calc_params.armed[armf_LFOTOVCA][i] = 0;

			if (button_pressed(i))
			{
				calc_params.armed[armf_LFOTOVCA][i] = 1;
				calc_params.already_handled_button[i] = 1;
				calc_params.button_safe_release[0] = 1;
			}
		}

		if (!button_pressed(i) && any_button_pressed && calc_params.armed[armf_LFOTOVCA][i]){
				lfos.to_vca[i] = 1 - lfos.to_vca[i];
			calc_params.armed[armf_LFOTOVCA][i] = 0;
		}

		else if(!any_button_pressed){
			if  (button_pressed(butm_LFOVCA_BUTTON) < MED_PRESSED) {
				calc_params.button_safe_release[0] = 0;
				if (!lfos.locked[i])
					calc_params.armed[armf_LFOTOVCA][i] = 1;
			} else {
				calc_params.button_safe_release[0] = 1;
				calc_params.armed[armf_LFOTOVCA][i] = 0;
			}
		}
	}

	else if (!button_pressed(butm_LFOVCA_BUTTON))
	{
		if (calc_params.armed[armf_LFOTOVCA][i]){
				lfos.to_vca[i] = 1 - lfos.to_vca[i];
			calc_params.armed[armf_LFOTOVCA][i] = 0;
		}
		if (calc_params.button_safe_release[0]) {
			calc_params.button_safe_release[0] = 0;
			stop_all_displays();
		}
		any_button_pressed = 0;
		calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 0;
	}
}

// Per-voice LFO modes and routing.

void read_lfomode(uint8_t i)
{
	static uint8_t any_button_pressed;

	if (!lfos.audio_mode[i])
	{
		if (lfo_mode_cached[i]){ // uncache lfo mode when exiting audio range
			cache_uncache_lfomode(i, UNCACHE);
			lfo_mode_cached[i] = 0;
		}

		else if (button_pressed(butm_LFOMODE_BUTTON)
				&& !calc_params.keymode_pressed
				&& (params.key_sw[i] == ksw_MUTE)
				&& !calc_params.already_handled_button[butm_LFOMODE_BUTTON]) {

			start_ongoing_display_lfo_mode();

			if (!macro_states.all_af_buttons_released){
				any_button_pressed = 1;
				calc_params.armed[armf_LFOMODE][i] = 0;

				if(button_pressed(i)){
						calc_params.armed[armf_LFOMODE][i] = 1;
					calc_params.already_handled_button[i] = 1;
					calc_params.button_safe_release[1] = 1;
				}
			}

			if (!button_pressed(i) && any_button_pressed && calc_params.armed[armf_LFOMODE][i]){
					lfos.mode[i] ++;
					lfos.mode[i] %= NUM_LFO_MODES;
				calc_params.armed[armf_LFOMODE][i]   = 0;
			}

			else if(!any_button_pressed){
				if  (button_pressed(butm_LFOMODE_BUTTON) < MED_PRESSED) {
					calc_params.button_safe_release[1] = 0;
					if (!lfos.locked[i]) calc_params.armed[armf_LFOMODE][i] = 1;
				} else {
					calc_params.button_safe_release[1] = 1;
					calc_params.armed[armf_LFOMODE][i] = 0;
				}
			}
		}

		else if (!button_pressed(butm_LFOMODE_BUTTON)){
			if(calc_params.armed[armf_LFOMODE][i]){
				if (!lfos.locked[i]) {
					lfos.mode[i] ++;
					lfos.mode[i] %= NUM_LFO_MODES;
				}
				calc_params.armed[armf_LFOMODE][i] = 0;
			}
			if (calc_params.button_safe_release[1]) {
				calc_params.button_safe_release[1] = 0;
				stop_all_displays();
			}
			calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 0;
			any_button_pressed = 0;
		}
	}

	else if (!lfo_mode_cached[i]){ // cache lfo mode and set to shape when entering audio range
		cache_uncache_lfomode(i, CACHE);
		lfos.mode[i] = lfot_SHAPE;
		lfo_mode_cached[i] = 1;
	}
}

/*** Move to params_keymode.c ***/

void read_all_keymodes(void){

	uint8_t				i;
	static uint8_t			any_button_pressed=0;
	static uint8_t preset_buttons_consumed = 0;
	uint8_t				change_keymode[NUM_CHANNELS] = {0};
	enum MuteNoteKeyStates	new_keymode;
	const uint8_t preset_down = rotary_pressed(rotm_PRESET) != RELEASED;
	// This runs before trigger/note readers in TIM6, so a newly debounced
	// Preset+A–F cannot trigger an envelope before main() sees the gesture.
	if (preset_down) preset_af_consumed = 1;
	if (glide_selection_held()) glide_af_consumed = 1;
	if (glide_af_consumed && !rotary_pressed(rotm_OCT) && macro_states.all_af_buttons_released)
        glide_af_consumed = 0;
	if (glide_af_consumed || parameter_lock_af_consumed || parameter_lock_gesture_held()) {
        for (i = 0; i < NUM_CHANNELS; ++i) {
            calc_params.already_handled_button[i] = 1;
            calc_params.armed[armf_NOTE_ON][i] = 0;
            calc_params.lock_change_staged[i] = 0;
        }
    }
	if (preset_af_consumed) {
		for (i = 0; i < NUM_CHANNELS; ++i) {
			calc_params.already_handled_button[i] = 1;
			calc_params.armed[armf_NOTE_ON][i] = 0;
			calc_params.lock_change_staged[i] = 0;
		}
		if (!preset_down && macro_states.all_af_buttons_released) preset_af_consumed = 0;
	}

	// Consume Preset or VCA/Browse gestures until BOTH LFO buttons are
	// released. Clear pending normal actions before release.
	if (preset_down &&
	    (button_pressed(butm_LFOVCA_BUTTON) || button_pressed(butm_LFOMODE_BUTTON)))
		preset_buttons_consumed = 1;
	if (preset_buttons_consumed || breathe_lfo_consumed) {
		for (i = 0; i < NUM_CHANNELS; ++i) {
			calc_params.armed[armf_KEYMODE][i] = 0;
			calc_params.armed[armf_LFOTOVCA][i] = 0;
			calc_params.armed[armf_LFOMODE][i] = 0;
		}
		calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 1;
		calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 1;
		calc_params.keymode_pressed = 0;
		any_button_pressed = 0;
		if (!button_pressed(butm_LFOVCA_BUTTON) && !button_pressed(butm_LFOMODE_BUTTON)) {
			preset_buttons_consumed = 0;
			breathe_lfo_consumed = 0;
		}
		return;
	}

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		if (key_combo_keymode_pressed())
		{
			stop_all_displays();
			calc_params.keymode_pressed = 1;

			calc_params.armed[armf_LFOMODE][i]  = 0;
			calc_params.armed[armf_LFOTOVCA][i] = 0;

			if (!macro_states.all_af_buttons_released){
				any_button_pressed = 1;
				calc_params.armed[armf_KEYMODE][i] = 0;

				if(/*!lfos.locked[i] && */button_pressed(i)){
					calc_params.armed[armf_KEYMODE][i]= 1;
					calc_params.already_handled_button[i] = 1;
					calc_params.button_safe_release[0] = 1;
					calc_params.button_safe_release[1] = 1;
				}
			}

			if (/*!lfos.locked[i] && */ !button_pressed(i) && any_button_pressed && calc_params.armed[armf_KEYMODE][i]){
				change_keymode[i] = 1;
				calc_params.armed[armf_KEYMODE][i] = 0;
			}

			else if(!any_button_pressed){
				if (button_pressed(butm_LFOVCA_BUTTON) < MED_PRESSED) {
					calc_params.button_safe_release[0] = 0;
					calc_params.button_safe_release[1] = 0;
					if (!lfos.locked[i]) calc_params.armed[armf_KEYMODE][i]= 1;
				} else {
					calc_params.button_safe_release[0] = 1;
					calc_params.button_safe_release[1] = 1;
					if (!lfos.locked[i]) calc_params.armed[armf_KEYMODE][i]= 0;
				}
			}
		}

		else if (key_combo_keymode_released()){
			if (calc_params.armed[armf_KEYMODE][i]){
					change_keymode[i] = 1;
				calc_params.armed[armf_KEYMODE][i] = 0;
			}
			if (calc_params.button_safe_release[0] && calc_params.button_safe_release[1]){
				calc_params.button_safe_release[0] = 0;
				calc_params.button_safe_release[1] = 0;
				stop_all_displays();
			}
			any_button_pressed = 0;
			calc_params.keymode_pressed = 0;
		}
	}

	for (i=0; i<NUM_CHANNELS; i++){
		if (change_keymode[i]){
			new_keymode = (params.key_sw[i]+1) % NUM_MUTE_NOTE_KEY_STATES;
			apply_keymode(i, new_keymode);
		}
	}
}

//Flip Mute to Note: cache LFO params, set envelope speed, remove phase
//Flip Note to Keys: do nothing
//Flip Keys to Mute: uncache LFO params
//Keep the current bank/shape through every play-mode change.
void apply_keymode(uint8_t chan, enum MuteNoteKeyStates new_keymode)
{
	if (params.key_sw[chan] != new_keymode)
	{
        cvgate_glide_guard[chan] = 0;
		//Switched from MUTE to NOTE
		if (params.key_sw[chan] == ksw_MUTE)
		{
			lfos.muted[chan] = 1; //mute to prevent race condition with update_lfo_wt_pos()

			cache_uncache_keys_params_and_lfos(chan, CACHE);
			params.note_on[chan] = 0;
			lfos.to_vca[chan] = 1;
			lfos.mode[chan] = 0;

			lfos.cycle_pos[chan] = 1.0;
			lfos.divmult_id[chan] = LFO_UNITY_DIVMULT_ID+2;
			flag_lfo_recalc(chan);

			lfos.phase_id[chan] = 0;
			lfos.phase[chan] = 0;
		}

		//Switched to MUTE
		else if (new_keymode == ksw_MUTE)
		{
			calc_params.gate_in_is_sustaining[chan] = 0;
			lfos.muted[chan] = 1; //mute to prevent race condition with update_lfo_wt_pos()
			cache_uncache_keys_params_and_lfos(chan, UNCACHE);
		}

		else if (new_keymode == ksw_KEYS_EXT_TRIG_SUSTAIN)
		{
			calc_params.gate_in_is_sustaining[chan] = 0;

		}

		params.key_sw[chan] = new_keymode;
	}

	lfos.muted[chan] = 0;
}

void cache_uncache_keys_params_and_lfos(uint8_t chan, enum CacheUncache cache_uncache)
{
	if (cache_uncache==CACHE)
	{
		lfos.divmult_id_buf[chan]		= lfos.divmult_id[chan];
		lfos.shape_buf[chan]			= lfos.shape[chan];
		lfos.gain_buf[chan]			= lfos.gain[chan];
		lfos.phase_id_buf[chan]		= lfos.phase_id[chan];
		lfos.mode_buf[chan]				= lfos.mode[chan];
		lfos.to_vca_buf[chan]			= lfos.to_vca[chan];

		params.note_on_buf[chan] 		= params.note_on[chan];
	}
	else
	{
		lfos.divmult_id[chan]			= lfos.divmult_id_buf[chan];
		flag_lfo_recalc(chan);

		lfos.phase_id[chan]			= lfos.phase_id_buf[chan];
		lfos.phase[chan]				= calc_lfo_phase(lfos.phase_id[chan]);

		lfos.gain[chan]				= lfos.gain_buf[chan];
		lfos.mode[chan]					= lfos.mode_buf[chan];
		lfos.to_vca[chan]				= lfos.to_vca_buf[chan];

		params.note_on[chan] 			= params.note_on_buf[chan];

		stage_resync(chan);
	}
}

// Pitch controls.

SmnPolyrotateFeedback smn_polyrotate_feedback;
static float polyrotate_cv[NUM_CHANNELS];
static uint32_t polyrotate_generation;
static volatile uint8_t polyrotate_ready; // Main requests a reset; TIM6 initializes live slots.
static uint8_t polyrotate_next, polyrotate_reads;
static uint8_t polyrotate_note, polyrotate_previous_note;
static int8_t polyrotate_octave, polyrotate_previous_octave;
static uint8_t polyrotate_reference;

// Main publishes a single-byte reset request, even if TIM6 is running.
// ADC sensing runs >800 ms before automatic resume is restored at boot.
void smn_polyrotate_reset(void)
{
    if (!analog_jack_plugged(TRANSPOSE_CV)) params.PADDING[SMN_POLYROTATE_BYTE] = 0;
    polyrotate_ready = 0;
    smn_polyrotate_feedback.animation = smn_polyrotate_feedback.flash_mask = 0;
}

static void polyrotate_animation(uint8_t enabled, uint32_t now)
{
    smn_polyrotate_feedback.animation_start = now;
    smn_polyrotate_feedback.flash_mask = 0;
    smn_polyrotate_feedback.animation = enabled ? 1 : 2;
}

// Main only. A clean Transpose hold must precede the complete insertion.
void smn_polyrotate_poll(void)
{
    static uint8_t initialized, raw_previous, stable, eligible;
    static uint8_t was_enabled;
    static uint32_t changed;
    const uint32_t now = HAL_GetTick();
    const uint8_t raw = analog_jack_plugged(TRANSPOSE_CV) == PRESSED;
    if (smn_polyrotate_feedback.suppress_transpose && !rotary_pressed(rotm_TRANSPOSE))
        smn_polyrotate_feedback.suppress_transpose = 0;
    if (!initialized) {
        initialized = 1;
        raw_previous = stable = raw;
        changed = now;
    }
    // No button scan in the common idle path: inspect gestures only while
    // confirming a new insertion, never for ordinary CV changes.
    uint8_t clean = 0;
    if (raw && !stable) {
        clean = ui_mode == PLAY && led_cont.ongoing_display != ONGOING_DISPLAY_PRESET &&
            rotary_pressed(rotm_TRANSPOSE) &&
            !switch_pressed(FINE_BUTTON) && macro_states.all_af_buttons_released &&
            !button_pressed(butm_LFOVCA_BUTTON) && !button_pressed(butm_LFOMODE_BUTTON);
        if (clean) for (uint8_t r = 0; r < NUM_ROTARIES; ++r)
            if (r != rotm_TRANSPOSE && rotary_pressed(r)) clean = 0;
    }
    if (raw != raw_previous) {
        raw_previous = raw;
        changed = now;
        eligible = clean;
    }
    if (!clean) eligible = 0;
    if (raw != stable && (uint32_t)(now - changed) >= 20 * TICKS_PER_MS) {
        stable = raw;
        if (stable && eligible) {
            pause_timer_IRQ(OSC_TIM_number);
            if (!scale_quantized()) {
                scale_set(&params, sclm_SEMITONES, scale_root(&params));
                scale_restore(&params);
            }
            // Freeze the six current offsets at activation; do not assign A yet.
            polyrotate_next = polyrotate_reads = polyrotate_reference = 0;
            polyrotate_generation = smn_scale_generation;
            params.PADDING[SMN_POLYROTATE_BYTE] = 1;
            smn_polyrotate_feedback.suppress_transpose = 1;
            resume_timer_IRQ(OSC_TIM_number);
            stop_all_displays();
        } else if (!stable) params.PADDING[SMN_POLYROTATE_BYTE] = 0;
    }
    if (!smn_polyrotate_enabled(&params)) params.PADDING[SMN_POLYROTATE_BYTE] = 0;
    const uint8_t enabled = smn_polyrotate_enabled(&params);
    if (was_enabled != enabled) polyrotate_animation(enabled, now);
    was_enabled = enabled;
}

void update_transpose_cv(void)
{
	// Calculate pitch multiplier from transpose jack CV
	params.transpose_cv = calc_expo_pitch(TRANSPOSE_CV, analog[TRANSPOSE_CV].lpf_val);
    if (polyrotate_calibrating || ui_mode == VOCT_CALIBRATE) return; // Also guard entry/exit.
    if (!smn_polyrotate_enabled(&params) || !polyrotate_ready) {
        for (uint8_t v = 0; v < NUM_CHANNELS; ++v) polyrotate_cv[v] = params.transpose_cv;
        polyrotate_ready = 1;
        polyrotate_reads = polyrotate_reference = polyrotate_next = 0;
        polyrotate_generation = smn_scale_generation;
        if (!smn_polyrotate_enabled(&params)) return;
    }
    if (ui_mode != PLAY || !analog_jack_plugged(TRANSPOSE_CV)) {
        polyrotate_reference = polyrotate_reads = 0;
        return;
    }
    if (polyrotate_generation != smn_scale_generation) {
        polyrotate_generation = smn_scale_generation;
        polyrotate_reference = polyrotate_reads = 0;
    }
    uint8_t note;
    int8_t octave;
    // Exact C reference from the quantizer; zero volts remains a unit ratio
    // in chromatic C, without the historical F_BASE_FREQ rounding offset.
    const float c0 = 16.351601562500f;
    const float input = calc_expo_pitch(TRANSPOSE_CV, analog[TRANSPOSE_CV].bracketed_val);
    const float ratio = quantize_to_scale(c0 * input, &note, &octave) / c0;
    if (note == UINT8_MAX) { polyrotate_reads = 0; return; }
    if (!polyrotate_reads || note != polyrotate_note || octave != polyrotate_octave) {
        polyrotate_note = note;
        polyrotate_octave = octave;
        polyrotate_reads = 1;
    } else if (polyrotate_reads < NOTE_PITCH_STABLE_READS) ++polyrotate_reads;
    if (polyrotate_reads < NOTE_PITCH_STABLE_READS) return;
    if (polyrotate_reference &&
        (note != polyrotate_previous_note || octave != polyrotate_previous_octave)) {
        const uint8_t v = polyrotate_next;
        polyrotate_cv[v] = ratio;
        smn_polyrotate_feedback.flash_start[v] = HAL_GetTick();
        smn_polyrotate_feedback.flash_mask |= 1u << v;
        polyrotate_next = (v + 1) % NUM_CHANNELS;
    }
    polyrotate_previous_note = note;
    polyrotate_previous_octave = octave;
    polyrotate_reference = 1;
}

void update_pitch(uint8_t chan)
{
	float ch_freq, ch_freq_adc, qtz_ch_freq;
	uint8_t note;
	int8_t oct;
	int16_t oct_clamped;
	const uint8_t quantized = scale_quantized();
    const uint8_t auto_note = ui_mode == PLAY && quantized &&
        params.key_sw[chan] == ksw_NOTE && !params.note_on[chan] &&
        !params.new_key[chan];
    if (!auto_note) note_candidate_reads[chan] = 0;
    const uint8_t attack = glide_attack[chan];
    glide_attack[chan] = 0;
    uint8_t glide_flags = quantized && (ui_mode == PLAY || ui_mode == SELECT_PARAMS ||
        ui_mode == RGB_COLOR_ADJUST) ? SMN_GLIDE_ENABLED : 0;
    if (attack && attack != 3 && params.key_sw[chan] != ksw_MUTE) glide_flags |= SMN_GLIDE_ATTACK;
    if (params.key_sw[chan] == ksw_MUTE || (attack ? (attack == 2 || attack == 3) :
        params.note_on[chan] && lfos.out_lpf[chan] > 0.0f))
        glide_flags |= SMN_GLIDE_LEGATO;
    if (!(glide_flags & SMN_GLIDE_ENABLED) ||
        (params.key_sw[chan] != ksw_KEYS_EXT_TRIG &&
         params.key_sw[chan] != ksw_KEYS_EXT_TRIG_SUSTAIN)) {
        cvgate_glide_guard[chan] = 0;
    } else {
        if (attack == 1) cvgate_glide_guard[chan] = CVGATE_GLIDE_GUARD_UPDATES;
        // An overlap retrigger keeps its normal glide, even inside
        // the previous fresh attack's window. Coincident sources use the
        // first pre-attack observation captured by capture_glide_attack().
        else if (attack == 2) cvgate_glide_guard[chan] = 0;
        if (cvgate_glide_guard[chan]) {
            --cvgate_glide_guard[chan];
            glide_flags &= ~SMN_GLIDE_LEGATO;
        }
    }
    // The existing qtz_freq is the held note. Encoder/CV/chord targets keep
    // changing in params, and are sampled on the next attack or while idle.
    if (ui_mode == PLAY &&
        (params.key_sw[chan] == ksw_NOTE || params.key_sw[chan] == ksw_KEYS) &&
        params.note_on[chan] && !params.new_key[chan]) goto fine_tuning;

	if ( params.key_sw[chan]==ksw_MUTE || params.key_sw[chan]==ksw_KEYS_EXT_TRIG_SUSTAIN || params.key_sw[chan]==ksw_KEYS_EXT_TRIG || params.new_key[chan] || ((params.key_sw[chan] == ksw_NOTE || params.key_sw[chan] == ksw_KEYS) && !params.note_on[chan]) )
	{
		// Calculate pitch multiplier from individual jack 1V/oct CV
		if ((params.voct_switch_state[chan] == SW_VOCT) || (params.key_sw[chan] != ksw_MUTE))
		{
			if (quantized)
				ch_freq_adc = analog[A_VOCT + chan].bracketed_val;
			else
				ch_freq_adc = analog[A_VOCT + chan].lpf_val;

			calc_params.voct[chan] = calc_expo_pitch(A_VOCT+chan, ch_freq_adc);
		} else
			calc_params.voct[chan] = 1.0;

		if (!params.osc_param_lock[chan] && !smn_parameter_locked(&params, chan, SMN_LOCK_PITCH))
			calc_params.voct[chan] *= smn_polyrotate_enabled(&params) ?
                polyrotate_cv[chan] : params.transpose_cv;

        if (params.new_key[chan]) {
            // Includes real external gates and restored preset playback.
            params.qtz_note_changed[chan] = 0;
            params.new_key[chan] = 0;
        }
	}

	ch_freq = F_BASE_FREQ  * calc_params.transposition[chan] * calc_params.voct[chan];
	oct_clamped = _CLAMP_I16(params.oct[chan], MIN_OCT , MAX_OCT);
	if (oct_clamped < 0)
		ch_freq /= (float)(1 << (-oct_clamped));
	else
		ch_freq *= (1 << oct_clamped);

	if (!quantized)
	{
		calc_params.qtz_freq[chan] = ch_freq;
        calc_params.pitch_octave[chan] = oct_clamped;
        params.qtz_note_changed[chan] = 0;
	}
	else
	{
		qtz_ch_freq = quantize_to_scale(ch_freq, &note, &oct);
        if (note == UINT8_MAX) glide_flags &= ~SMN_GLIDE_ENABLED;

        // Reject short intermediate pitches before publishing a destination or
        // requesting an attack. Keep the cheap held-envelope early exit above;
        // manual/chord attacks and restored playback still sample immediately.
        if (auto_note) {
            if (note == UINT8_MAX) note_candidate_reads[chan] = 0;
            else {
                if (!note_candidate_reads[chan] || note_candidate[chan] != note ||
                    note_candidate_oct[chan] != oct) {
                    note_candidate[chan] = note;
                    note_candidate_oct[chan] = oct;
                    note_candidate_reads[chan] = 1;
                } else if (note_candidate_reads[chan] < NOTE_PITCH_STABLE_READS) {
                    ++note_candidate_reads[chan];
                }
                if (note_candidate_reads[chan] < NOTE_PITCH_STABLE_READS)
                    goto fine_tuning;
            }
        }

		if (note != UINT8_MAX && !params.qtz_note_changed[chan]) {
			calc_params.qtz_freq[chan] = qtz_ch_freq;
            calc_params.pitch_octave[chan] = oct_clamped;
        }

		if (note != UINT8_MAX && params.qtz_note_changed[chan]==0 && (calc_params.prev_qtz_note[chan]!=note || calc_params.prev_qtz_oct[chan]!=oct))
		{
			// Automatic Note starts gliding as soon as the stable note is accepted.
            // The following envelope attack must preserve this same transition.
            if (auto_note) glide_flags |= SMN_GLIDE_LEGATO;
			calc_params.prev_qtz_note[chan] = note;
			calc_params.prev_qtz_oct[chan] = oct;
			calc_params.qtz_freq[chan] = qtz_ch_freq;
			if ((params.key_sw[chan]==ksw_KEYS || params.key_sw[chan]==ksw_NOTE) && !params.note_on[chan])
				params.qtz_note_changed[chan] = 1;
		}
		else {
			if (params.qtz_note_changed[chan]>0)
				params.qtz_note_changed[chan]++;
			if (params.qtz_note_changed[chan]>QTZ_CHANGE_LOCKOUT_PERIOD)
				params.qtz_note_changed[chan]=0;
		}
	}

fine_tuning:
	// Fine remains live even while the note is held.
    smn_glide_update(chan, calc_params.qtz_freq[chan], params.smn_extra.glide_ms[chan],
        glide_flags, _CLAMP_F(calc_params.qtz_freq[chan] * calc_params.tuning[chan],
        F_MIN_FREQ, F_MAX_FREQ), &calc_params.pitch[chan]);

}

// Pitch controls.

void update_noise(uint8_t chan)
{
	float random_cv;
	static uint32_t noise_poll_ctr=0;

	//If the channel is fine-tuned to 0, then don't apply noise
	params.noise_on = (params.finetune[chan]==0) ? 0 : 1;

	if (params.noise_on)
	{
		if ( noise_poll_ctr++ > RANDOM_UPDATE_TIME)
		{
			noise_poll_ctr	= 0;

			random_cv = _CLAMP_F(analog[RANDOM_CV].lpf_val, 0.0, 6.0);
			params.random[chan] = 1 + ((random_cv - 3.0) * F_SCALING_RANDOM);
		}
		calc_params.pitch[chan] *= params.random[chan];
	}
}

//##########################################################
// Pitch encoders and reset shortcuts.
//##########################################################

// Wait for BOTH encoders to be released: Fine may arrive after Preset+Octave.
// Once selected, the Scale reset supersedes Octave until this gesture ends.
static uint8_t read_octave_scale_reset(void)
{
    enum { RESET_NONE, RESET_OCTAVE, RESET_SCALE, RESET_CANCELLED };
    static uint8_t staged;
    const uint8_t held = rotary_pressed(rotm_PRESET) || rotary_pressed(rotm_OCT);
    if (ui_mode == PLAY) {
        if (key_combo_reset_scale() && staged != RESET_CANCELLED)
            staged = RESET_SCALE;
        else if (key_combo_reset_octaves() && !staged)
            staged = RESET_OCTAVE;
    } else if (staged) staged = RESET_CANCELLED;
    if (!staged) return 0;

    cancel_chord_gesture();
    if (!held) {
        const uint8_t action = staged;
        staged = RESET_NONE;
        if (action == RESET_SCALE) {
            scale_set(&params, sclm_NONE, 0);
            scale_restore(&params);
            start_ongoing_display_scale();
        } else if (action == RESET_OCTAVE) reset_octaves();
    }
    return 1;
}

void read_freq(void){

	int16_t tmp1, tmp2, tmp3, tmp4;
	static uint8_t reset_detuning_enabled = 1;
	static uint8_t reset_transpose_enabled = 1;

	tmp1 = pop_encoder_q (pec_OCT  );
	tmp2 = pop_encoder_q (sec_SCALE);
	tmp3 = pop_encoder_q (pec_TRANSPOSE );
	tmp4 = pop_encoder_q (sec_OSC_SPREAD);

    // Consume pitch motions during a reset, including its final release poll.
    if (read_octave_scale_reset()) return;

	// ---------------------
	//	OCT / SCALE
	// ---------------------

	if (tmp1) 		{update_oct 	(tmp1);}
	else if (tmp2)	{update_scale	(tmp2);}

	// ----------------------------------------------
	//		TRANSPOSE / TUNE / CHORD / DETUNE SPREAD
	// ----------------------------------------------

	else if(tmp3){
        if (rotary_pressed(rotm_OCT)) { update_scale_root(tmp3); }
		else if (switch_pressed(FINE_BUTTON)) { update_finetune (tmp3); }
		else							{ update_transpose(tmp3); }
	}

	else if (tmp4){
		if		(switch_pressed(FINE_BUTTON))	spread_finetune(tmp4);
		else if (!switch_pressed(FINE_BUTTON) && macro_states.all_af_buttons_released) update_spread(tmp4);
	}

	if (key_combo_reset_detuning())	{
		if (reset_detuning_enabled) {
            cancel_chord_gesture();
			retune_oscillators();
			reset_detuning_enabled = 0;
		}
	} else reset_detuning_enabled = 1;

	if (key_combo_reset_transpose())	{
		if (reset_transpose_enabled) {
            cancel_chord_gesture();
			reset_notes();
			reset_transpose_enabled = 0;
		}
	} else reset_transpose_enabled = 1;

}

void toggle_lock(uint8_t chan)
{
	uint8_t lock_status = params.osc_param_lock[chan] ? 0 : 1;
	params.osc_param_lock[chan] = lock_status;
	params.wt_pos_lock[chan] = lock_status;
	params.wtsel_lock[chan] = lock_status;
	lfos.locked[chan] = lock_status;
}

void update_osc_param_lock(void)
{
    read_parameter_locks();
	uint8_t chan;
	const uint8_t preset_down = rotary_pressed(rotm_PRESET) != RELEASED;
	if (preset_down) preset_af_consumed = 1;
    const uint8_t consumed = channel_buttons_consumed() || preset_down;

	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		if (consumed) {
			calc_params.lock_change_staged[chan] = 0;
			if (preset_down && button_pressed(chan)) {
				calc_params.already_handled_button[chan] = 1;
			}
		}
		else if (key_combo_lock_channel(chan))
		{
			if (calc_params.lock_change_staged[chan]==0)
				calc_params.lock_change_staged[chan] = 1;
		}
		else
			if (calc_params.lock_change_staged[chan]==2)
				calc_params.lock_change_staged[chan] = 0;
	}
}

void update_oct(int16_t tmp)
{
	uint8_t i;
	int32_t oct[NUM_CHANNELS];

	for (i = 0; i < NUM_CHANNELS; i++)
		oct[i] = params.oct[i];

	const uint8_t changed = change_param_i32_targeted(oct, tmp, SMN_LOCK_PITCH);
	if (changed)
	{
		start_ongoing_display_octave();

		//Clamp to active range, but preserve relative spacing
		trim_array(oct, NUM_CHANNELS, MIN_OCT, MAX_OCT);

		//don't allow any channel to be more than MAX_OCT away from active range
		for (i = 0; i < NUM_CHANNELS; i++)
			if (changed & (1u << i)) params.oct[i] = _CLAMP_I16(oct[i], -MAX_OCT, MAX_OCT*2);
	}
}

static uint8_t wrap_scale_step(int32_t value, uint8_t count)
{
    int32_t wrapped = value % count;
    return wrapped < 0 ? wrapped + count : wrapped;
}

void update_scale(int16_t tmp)
{
    const uint8_t position = wrap_scale_step((int32_t)scale_position(params.indiv_scale[0]) + tmp, NUM_QTZ_SCALES);
    scale_set(&params, scale_at_position(position), scale_root(&params));
    scale_restore(&params);
    start_ongoing_display_scale();
}

void update_scale_root(int16_t tmp)
{
    const uint8_t root = wrap_scale_step((int32_t)scale_root(&params) + tmp, 12);
    scale_set(&params, params.indiv_scale[0], root);
    scale_restore(&params);
    start_ongoing_display_scale_root();
}

void update_finetune(int16_t tmp)
{
	uint8_t i;
	int16_t finetune[NUM_CHANNELS];
	uint8_t channels_changed;

	for (i = 0; i < NUM_CHANNELS; i++)
		finetune[i] = params.finetune[i];

	channels_changed = change_param_i16_targeted(finetune, tmp, SMN_LOCK_PITCH);

	if (channels_changed)
	{
		for (i = 0; i < NUM_CHANNELS; i++)
		{
			if (channels_changed & (1<<i))
			{
				params.finetune[i] = finetune[i];

				compute_tuning(i);
				start_ongoing_display_finetune();
			}
		}
	}

}

void reset_octaves(void)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (!params.osc_param_lock[i] && !smn_parameter_locked(&params, i, SMN_LOCK_PITCH))
			params.oct[i] = INIT_OCT;
	}
}

void reset_notes(void)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (!params.osc_param_lock[i] && !smn_parameter_locked(&params, i, SMN_LOCK_PITCH)) {
			params.spread_enc[i] = 0;
			params.transpose_enc[i] = 0;
		}
	}
	combine_transpose_spread();
	// This UI path can be preempted by TIM6. Leave its private conversion cache
	// to compute_transpositions(); the rare reset uses the original calculation.
	for (i = 0; i < NUM_CHANNELS; i++)
		calc_params.transposition[i] = compute_transposition(calc_params.transpose[i]);
}

void retune_oscillators(void)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; i++){
		if (!params.osc_param_lock[i] && !smn_parameter_locked(&params, i, SMN_LOCK_PITCH)) {
			params.finetune[i] = 0;
			compute_tuning(i);

		}
	}

}

void spread_finetune(int16_t tmp)
{
	int8_t i;

	for (i = 0; i < NUM_CHANNELS; i++ )
	{
		if (button_pressed(i) || (macro_states.all_af_buttons_released &&
            !params.osc_param_lock[i] && !smn_parameter_locked(&params, i, SMN_LOCK_PITCH)))
        {
            if (button_pressed(i)) calc_params.already_handled_button[i] = 1;
			if (i<=2)
				params.finetune[i] += (i - 3) * tmp;
			else
				params.finetune[i] += (i - 2) * tmp;

			compute_tuning(i);
			start_ongoing_display_finetune();
		}
	}

}

void compute_tuning (uint8_t chan){

	int16_t i;
	float f_scaling_finetune;

	calc_params.tuning[chan] = 1;

	params.finetune[chan] = _CLAMP_I16(params.finetune[chan], MIN_FINETUNE_WRAP, MAX_FINETUNE_WRAP);
	f_scaling_finetune = F_SCALING_FINETUNE_WRAP;

	if(params.finetune[chan] > 0){
		for (i = 0; i < params.finetune[chan]; i++){
			calc_params.tuning[chan] *= f_scaling_finetune;
		}
	}

	else if (params.finetune[chan]<0) {
		for (i = 0; i > params.finetune[chan]; i--){
			calc_params.tuning[chan] /= f_scaling_finetune;
		}
	}
}

void update_transpose(int16_t tmp){
	uint8_t i;
	int32_t transpose_enc[NUM_CHANNELS];
	uint8_t channels_changed;
	int32_t min, max;

	for (i = 0; i < NUM_CHANNELS; i++)
		transpose_enc[i] = params.transpose_enc[i];

	channels_changed = change_param_i32_targeted(transpose_enc, tmp, SMN_LOCK_PITCH);
	if (channels_changed)
	{
		max = MAX_TRANSPOSE_WRAP;
		min = MIN_TRANSPOSE_WRAP;

		trim_array(transpose_enc, NUM_CHANNELS, min, max);

		start_ongoing_display_transpose();

		for (i = 0; i < NUM_CHANNELS; i++)
			if (channels_changed & (1u << i)) params.transpose_enc[i] = transpose_enc[i];
	}
}

void update_spread(int16_t tmp){

	uint8_t chan;

	for (chan=0; chan<NUM_CHANNELS; chan++){
		if (!params.osc_param_lock[chan])
			params.spread_enc[chan] += tmp;
	}
	start_ongoing_display_transpose();
}

void combine_transpose_spread(void)
{
    for (uint8_t chan = 0; chan < NUM_CHANNELS; ++chan) {
        const uint8_t chord_num = _WRAP_I8(params.spread_enc[chan], 0, NUM_CHORDS);
        calc_params.transpose[chan] = params.transpose_enc[chan] + (smn_parameter_locked(&params, chan, SMN_LOCK_PITCH) ? 0 : CHORD_LIST[chord_num][chan]);
    }
}

// trim_array() trims extra steps beyond MAX/MIN values, when all elements are beyond threshold of display_min or display_max
// It serves to clip values while still preserving the spacing between them
void trim_array(int32_t *a, uint32_t num_elements, int32_t display_min, int32_t display_max){

	uint8_t chan;
	int16_t min,max;

	uint8_t all_over_max, all_under_min;

	all_over_max = 1;
	for (chan=0; chan<num_elements; chan++)
	{
		if (a[chan] <= display_max) {
			all_over_max = 0;
			break;
		}
	}

	if (all_over_max)
	{
		min = INT16_MAX;
		for (chan = 0; chan < num_elements; chan++){
			if(a[chan] < min) min = a[chan];
		}

		for (chan = 0; chan < num_elements; chan++){
			a[chan] -= (min - display_max);
		}
		return;
	}

	all_under_min = 1;
	for (chan=0; chan<num_elements; chan++)
	{
		if (a[chan] >= display_min) {
			all_under_min = 0;
			break;
		}
	}
	if (all_under_min)
	{
		max = INT16_MIN;
		for (chan = 0; chan < num_elements; chan++){
			if(a[chan] > max) max = a[chan];
		}

		for (chan = 0; chan < num_elements; chan++){
			a[chan] -= (max - display_min);
		}
	}
}

void compute_transpositions(void)
{
	// TIM6 owns this cache. Zero semitones is exactly 1, including at startup.
	static struct { int32_t semitones; float ratio; } cache[NUM_CHANNELS] = {
		{0, 1.0f}, {0, 1.0f}, {0, 1.0f}, {0, 1.0f}, {0, 1.0f}, {0, 1.0f}
	};
	for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
		const int32_t semitones = calc_params.transpose[chan];
		if (semitones != cache[chan].semitones) {
			cache[chan].ratio = compute_transposition(semitones);
			cache[chan].semitones = semitones;
		}
		// Always republish: init/calibration can overwrite the destination, but
		// cannot invalidate this pure integer -> ratio mapping.
		calc_params.transposition[chan] = cache[chan].ratio;
	}
}

float compute_transposition(int32_t transpose)
{
	int16_t i;
	float transposition;

	transposition = 1.0;

	if(transpose > 0) {
		while(transpose>=12) {transposition *= 2.0; transpose-=12;}
		for (i = 0; i < transpose; i++){
			transposition *= F_SCALING_TRANSPOSE;
		}
	}

	else if (transpose<0) {
		while(transpose<=-12) {transposition /= 2.0; transpose+=12;}
		for (i = 0; i > transpose; i--){
			transposition /= F_SCALING_TRANSPOSE;
		}
	}

	return transposition;
}

// ########################################################################
//						SMN TIMBRE CONTROLS AND MODEL SELECTION
// ########################################################################

void update_wt(void)
{
	uint8_t		i;
	static uint8_t  poll_ctr =  0;

	switch(poll_ctr++){

		// ------------
		// WT SELECTION
		// -------------

		// Model choice
		case 2:
			// Former WT Spread input now belongs to Morph, not model selection.

			read_browse_encoder();	//calls read_wtsel() if encoder is pushed+turned and updates wtsel_enc[]
			read_wtsel_cv();		//updates wtsel_cv based on cv on the jack

			update_wtsel();		// Direct model choice plus its CV; no WT Spread.

		break;

		// ------------
		// WT POSITION
		// ------------

		case 4:
			update_wt_disp();
		break;

		// depth
		case 6:
			update_wt_nav_cv(0);
			read_nav_encoder(0);
		break;

		// latitude
		case 9:
			update_wt_nav_cv(1);
			read_nav_encoder(1);
		break;

		// longitude
		case 12:
			update_wt_nav_cv(2);
			read_nav_encoder(2);
		break;

		case 15:
			//Todo: make these functions into one looping function
			for (i = 0; i < NUM_CHANNELS; i++){
				calc_wt_pos				(i);
			}
			poll_ctr=0;
		break;
	}
}

void read_nav_encoder(uint8_t dim)
{
	const uint8_t encoders[3] = {pec_DEPTH, pec_LATITUDE, pec_LONGITUDE};
	const int16_t enc = pop_encoder_q(encoders[dim]);
	if (enc)
		update_wt_nav(dim, enc * (switch_pressed(FINE_BUTTON)
			? F_SCALING_FINE_NAVIGATE : F_SCALING_NAVIGATE));
}

static void update_glide_time(int16_t events)
{
    int32_t indices[NUM_CHANNELS];
    for (unsigned v = 0; v < NUM_CHANNELS; ++v)
        indices[v] = smn_glide_time_index(params.smn_extra.glide_ms[v]);
    const uint8_t changed = change_param_i32(indices, (int32_t)events *
        (switch_pressed(FINE_BUTTON) ? 1 : (int32_t)SMN_GLIDE_FINE_DIVISOR));
    for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
        if (!(changed & (1u << v))) continue;
        const int32_t index = indices[v] < 0 ? 0 :
            indices[v] >= SMN_GLIDE_TIME_COUNT ? SMN_GLIDE_TIME_COUNT - 1 : indices[v];
        params.smn_extra.glide_ms[v] = smn_glide_times_ms[index];
    }
    // The existing resume snapshot includes glide_ms: real changes alone
    // restart its one-second timer. Moving at a bound only refreshes the view.
    start_ongoing_display_glide();
}

void read_browse_encoder(void)
{
	static uint8_t set_new_global_brightness = 0;
	const int16_t enc = pop_encoder_q(pec_WBROWSE);
	const int16_t selection = pop_encoder_q(sec_WTSEL);

	// Existing gestures take priority over Internal Modulation.
	if (enc && rotary_pressed(rotm_PRESET)) {
        cancel_chord_gesture();
		exit_preset_manager();
		stop_all_displays();
		start_ongoing_display_globright();
		system_settings.global_brightness = _CLAMP_F(
			system_settings.global_brightness + enc * F_SCALING_NAVIGATE_GLOBAL_BRIGHTNESS,
			F_MIN_GLOBAL_BRIGHTNESS, 1.0);
		set_new_global_brightness = 1;
	}
	if (!rotary_pressed(rotm_PRESET) && set_new_global_brightness) {
		// Internal flash erase stalls audio. Use the existing external journal.
		smn_resume_request();
		set_new_global_brightness = 0;
	}
	if (selection && ui_mode == PLAY)
		read_wtsel(selection);
	if (enc && !selection && glide_selection_held()) {
        update_glide_time(enc);
        return; // Scale owns this gesture, including gain/H/T/M/Pitch internal modulation.
    }
	if (enc && !selection && ui_mode == PLAY && !rotary_pressed(rotm_PRESET)
		&& !rotary_pressed(rotm_WAVETABLE) && !key_combo_keymode_pressed()) {
		const uint8_t encoders[3] = {rotm_DEPTH, rotm_LATITUDE, rotm_LONGITUDE};
		uint8_t destinations = 0;
		for (unsigned dim = 0; dim < 3; ++dim)
			if (rotary_pressed(encoders[dim])) destinations |= 1u << (dim + SMN_MOD_HARMONICS);
		if (rotary_pressed(rotm_TRANSPOSE)) destinations |= 1u << SMN_MOD_PITCH;
		if (button_pressed(butm_LFOVCA_BUTTON)) {
			destinations |= 1u << SMN_MOD_GAIN;
			breathe_lfo_consumed = 1;
			calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 1;
			// Cancel all pending routes, including channels not edited, before
			// either the channel or LFO button can be released. Also at bounds.
			for (unsigned chan = 0; chan < NUM_CHANNELS; ++chan)
				calc_params.armed[armf_LFOTOVCA][chan] = 0;
		}
        if (button_pressed(butm_LFOMODE_BUTTON)) {
            destinations |= 1u << SMN_MOD_PAN;
            breathe_lfo_consumed = 1;
            calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 1;
            for (unsigned chan = 0; chan < NUM_CHANNELS; ++chan)
                calc_params.armed[armf_LFOMODE][chan] = 0;
        }
		const uint8_t fine = switch_pressed(FINE_BUTTON);
		for (unsigned chan = 0; chan < NUM_CHANNELS; ++chan) {
			if (!button_pressed(chan) && !(macro_states.all_af_buttons_released && !params.osc_param_lock[chan]))
				continue;
			// Consume the channel gesture even at a bound or in a forbidden
			// gain mode, so releasing it cannot accidentally toggle Mute.
			if (button_pressed(chan)) calc_params.already_handled_button[chan] = 1;
			// On an untargeted Browse event, read each voice's target counters
			// together. The audio ramps may still be moving toward them.
			uint16_t active_steps[SMN_MOD_DESTINATIONS];
			if (!destinations) smn_modulation_capture_steps(chan, active_steps);
			for (unsigned d = 0; d < SMN_MOD_DESTINATIONS; ++d) {
				if (destinations ? !(destinations & (1u << d)) : !active_steps[d]) continue;
                const uint8_t parameter = d == SMN_MOD_PITCH ? SMN_LOCK_PITCH :
                    d >= SMN_MOD_HARMONICS && d <= SMN_MOD_MORPH ? d - SMN_MOD_HARMONICS : SMN_LOCK_NONE;
                if (!button_pressed(chan) && smn_parameter_locked(&params, chan, parameter)) continue;
				if (d == SMN_MOD_GAIN && (params.key_sw[chan] != ksw_MUTE || lfos.to_vca[chan])) {
					smn_modulation_clear_gain(chan);
					continue;
				}
				smn_modulation_adjust(chan, d, enc, fine);
			}
		}
	}
}

// ####################################
//			WT SELECTION
// ####################################

// The catalog order is independent of the stable IDs stored in presets.
void read_wtsel(int16_t wtsel)
{
	uint8_t i;
	int16_t wtsel_enc[NUM_CHANNELS];

	if (wtsel)
	{
		for (i = 0; i < NUM_CHANNELS; i++)
			wtsel_enc[i] = params.wtsel_enc[i];

		if (change_param_i16(wtsel_enc, wtsel % smn_model_count()))
		{
			for (i = 0; i < NUM_CHANNELS; i++)
				params.wtsel_enc[i] = smn_model_wrap(wtsel_enc[i]);
		}

		start_ongoing_display_model_sel();
	}
}

void read_wtsel_cv(void){
	if ((params.key_sw[0]==ksw_KEYS_EXT_TRIG || params.key_sw[0]==ksw_KEYS_EXT_TRIG_SUSTAIN) && analog_jack_plugged(A_VOCT))
		params.wtsel_cv = 0;
	else
		// Full-scale CV selects the last model, never wraps back to the first.
		params.wtsel_cv = analog[WTSEL_CV].bracketed_val * smn_model_count() / 4096;
}

void update_wtsel(void)
{
	for (uint8_t chan = 0; chan < NUM_CHANNELS; ++chan)
	{
		const int32_t cv = params.wtsel_lock[chan] ? 0 : params.wtsel_cv;
		calc_params.wtsel[chan] = smn_model_wrap(params.wtsel_enc[chan] + cv);
	}
	smn_model_selection_capture(&params);
}

// ####################################
//			WT NAVIGATION
// ####################################

void update_wt_nav(uint8_t dim, float increment)
{
	if (increment)
		change_param_f_targeted(params.wt_nav_enc[dim], increment, dim);
}

// Continuous timbre CV uses the existing FIR average, retaining fractions.
// Keep bracketing for discrete choices (models/chords), not H/T/M modulation.
void update_wt_nav_cv(uint8_t wt_dim)
{
	switch (wt_dim) {
		case 0:
			if ((params.key_sw[2]==ksw_KEYS_EXT_TRIG || params.key_sw[2]==ksw_KEYS_EXT_TRIG_SUSTAIN) && analog_jack_plugged(C_VOCT))
				params.wt_nav_cv[0] = 0;
			else
				params.wt_nav_cv[0] = (float)(analog[DEPTH_CV].lpf_val) * (SMN_CONTROL_CYCLE_UNITS * 0.5f) / 4095.0; //0..3
			break;

		case 1:
			if ((params.key_sw[4]==ksw_KEYS_EXT_TRIG || params.key_sw[4]==ksw_KEYS_EXT_TRIG_SUSTAIN) && analog_jack_plugged(E_VOCT))
				params.wt_nav_cv[1] = 0;
			else
				params.wt_nav_cv[1] = (float)(analog[LATITUDE_CV].lpf_val) * (SMN_CONTROL_CYCLE_UNITS * 0.5f) / 4095.0; //0..3
			break;

		case 2:
			// Former WT Spread jack: direct Morph modulation, not dispersion.
			if ((params.key_sw[5]==ksw_KEYS_EXT_TRIG || params.key_sw[5]==ksw_KEYS_EXT_TRIG_SUSTAIN) && analog_jack_plugged(F_VOCT))
				params.wt_nav_cv[2] = 0;
			else
				params.wt_nav_cv[2] = (float)analog[WTSEL_SPREAD_CV].lpf_val * (SMN_CONTROL_CYCLE_UNITS * 0.5f) / 4095.0f;
			break;

		default:
			break;
	}

}

void update_wt_disp(void)
{
	// Audio owns the shared 300 ms ramps; publish manual targets and live CV.

	const uint8_t encoders[3] = {sec_DISPERSION, sec_DISPPATT, sec_WTSEL_SPREAD};
	const float step = switch_pressed(FINE_BUTTON) ? F_SCALING_FINE_DISPERSION : F_SCALING_DISPERSION;
	for (uint8_t dim = 0; dim < 3; ++dim) {
		const int16_t movement = pop_encoder_q(encoders[dim]);
		if (movement)
			smn_dispersion_set(&params, dim, smn_dispersion_get(&params, dim) + movement * step);
	}

	// Use the existing FIR averages here too; no extra filter or audio work.
	// Keep the shared-jack trigger priorities for B and D. F is checked in
	// update_wt_nav_cv(2), since that jack now modulates Morph directly.
	if ((params.key_sw[1]==ksw_KEYS_EXT_TRIG || params.key_sw[1]==ksw_KEYS_EXT_TRIG_SUSTAIN) && analog_jack_plugged(B_VOCT))
		smn_dispersion_cv[0] = 0;
	else
		smn_dispersion_cv[0] = (float)analog[DISP_CV].lpf_val / 4095.0f;
	if ((params.key_sw[3]==ksw_KEYS_EXT_TRIG || params.key_sw[3]==ksw_KEYS_EXT_TRIG_SUSTAIN) && analog_jack_plugged(D_VOCT))
		smn_dispersion_cv[1] = 0;
	else
		smn_dispersion_cv[1] = (float)analog[DISPPAT_CV].lpf_val / 4095.0f;
	smn_dispersion_cv[2] = (float)analog[WBROWSE_CV].lpf_val / 4095.0f;
	params.dispersion_cv = smn_dispersion_cv[0]; // Legacy field, not a target.
	params.disppatt_enc = params.disppatt_cv = 0;
}

float smn_dispersion_display_value(uint8_t dimension)
{
    if (dimension >= 3) return 0.0f;
    const float cv = smn_dispersion_cv[dimension];
    const float value = smn_dispersion_get(&params, dimension)
        + (cv > 0.0f ? cv : 0.0f);
    return value < 1.0f ? value : 1.0f;
}

// Publish manual phases and doses together. The audio reader uses one shared
// ramp per H/T/M, then adds direct/CV dispersion (capped) before folding once.
void calc_wt_pos(uint8_t chan)
{
	float nav_enc[3], dispersion_enc[3], nav_cv[3], dispersion_cv[3];
	for (uint8_t dim = 0; dim < 3; ++dim) {
		nav_enc[dim] = params.wt_nav_enc[dim][chan];
		const uint8_t locked = smn_parameter_locked(&params, chan, dim);
        dispersion_enc[dim] = locked ? 0.0f : smn_dispersion_get(&params, dim);
        nav_cv[dim] = locked ? 0.0f : params.wt_nav_cv[dim];
        dispersion_cv[dim] = locked ? 0.0f : smn_dispersion_cv[dim];
	}
	smn_controls_update_with_dispersion(chan, nav_enc, nav_cv,
		dispersion_enc, dispersion_cv, params.wt_pos_lock[chan]);
}

// Rebuild the six command snapshots after preset recall or undo.
void update_all_smn_controls(void)
{
	for (uint8_t i = 0; i < NUM_CHANNELS; ++i)
		calc_wt_pos(i);
}

// Stage like the LFO resets: the larger combination supersedes the smaller
// one, and the action runs once after both LFO buttons have been released.
static uint8_t read_preset_reset_buttons(void)
{
    enum { RESET_NONE, RESET_WAIT, RESET_LFO_VCA, RESET_LFO_MODE, RESET_ALL, RESET_CANCELLED };
    static uint8_t staged = RESET_NONE;
    const uint8_t held = button_pressed(butm_LFOVCA_BUTTON)
        || button_pressed(butm_LFOMODE_BUTTON);

    if (!staged && ui_mode == PLAY && rotary_pressed(rotm_PRESET) && held) {
        staged = RESET_WAIT;
        exit_preset_manager();
        stop_all_displays();
    }
    if (!staged) return 0;
    if (ui_mode != PLAY) staged = RESET_CANCELLED;
    if (staged != RESET_CANCELLED) {
        if (key_combo_reset_params_all()) staged = RESET_ALL;
        else if (staged != RESET_ALL) {
            if (key_combo_reset_lfo_vca()) staged = RESET_LFO_VCA;
            else if (key_combo_reset_lfo_modes()) staged = RESET_LFO_MODE;
        }
    }
    if (!held) {
        const uint8_t action = staged;
        staged = RESET_NONE;
        if (action == RESET_ALL) reset_active_params();
        else if (action == RESET_LFO_VCA || action == RESET_LFO_MODE) {
            // TIM6 owns button actions and the audio-range mode cache. Keep
            // its saved values consistent; audio and LFO interrupts continue.
            pause_timer_IRQ(OSC_TIM_number);
            for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
                if (action == RESET_LFO_VCA) {
                    lfos.to_vca[v] = lfos.to_vca_buf[v] = 0;
                } else {
                    lfos.mode[v] = lfos.mode_buf[v] = lfot_SHAPE;
                    if (lfo_mode_cached[v]) cache_uncache_lfomode(v, CACHE);
                }
            }
            resume_timer_IRQ(OSC_TIM_number);
        }
    }
    return 1;
}

// A press owns its context until release, independently of display timeouts.
enum PresetGestureOwner { PG_NONE, PG_CHORD, PG_MENU, PG_UNDO, PG_OTHER };
static volatile uint8_t preset_gesture_owner;
static volatile uint8_t chord_cancelled;
static volatile int8_t chord_feedback = -1;
static volatile uint32_t chord_feedback_start;

void cancel_chord_gesture(void)
{
    if (rotary_pressed(rotm_PRESET)) chord_cancelled = 1;
    chord_feedback = -1;
}

uint8_t chord_buttons_green(void)
{
    return ui_mode == PLAY && preset_gesture_owner == PG_CHORD &&
        !chord_cancelled && rotary_pressed(rotm_PRESET);
}

int8_t chord_saved_feedback(uint32_t ticks)
{
    if (ui_mode != PLAY || preset_gesture_owner == PG_MENU ||
        led_cont.ongoing_display == ONGOING_DISPLAY_PRESET)
        return -1;
    return (uint32_t)(ticks - chord_feedback_start) < 1000 * TICKS_PER_MS ? chord_feedback : -1;
}

static void capture_chord_with_feedback(uint8_t memory, uint32_t ticks)
{
    smn_chord_store(memory);
    chord_feedback_start = ticks;
    chord_feedback = memory;
}

void read_load_save_encoder(void)
{
    static uint8_t last_down, selected_memory = UINT8_MAX, wait_release, stored;
    static uint32_t combined_start;
    const int16_t enc = pop_encoder_q(pec_LOADPRESET);
    const int16_t enc2 = pop_encoder_q(sec_SAVEPRESET);
    const uint8_t down = rotary_pressed(rotm_PRESET) != RELEASED;
    const uint32_t now = HAL_GetTick();
    uint8_t af = 0;
    if (!macro_states.all_af_buttons_released)
        for (uint8_t v = 0; v < NUM_CHANNELS; ++v)
            if (button_pressed(v)) af |= 1u << v;
    const uint8_t reset = read_preset_reset_buttons();
    const uint8_t other = !key_combo_all_but_preset_released() || ui_mode != PLAY;

    if (down && !last_down) {
        selected_memory = UINT8_MAX;
        wait_release = stored = 0;
        if (led_cont.ongoing_display == ONGOING_DISPLAY_PRESET)
            preset_gesture_owner = PG_MENU;
        else {
            // This also forgets the prior press edge: no confirmation can leak
            // through an outside-menu gesture or its final release.
            cancel_preset_events();
            preset_gesture_owner = switch_pressed(FINE_BUTTON) && !af ? PG_UNDO : PG_CHORD;
            if (preset_gesture_owner == PG_UNDO) chord_feedback = -1;
        }
    }
    if (down && (other || reset)) cancel_chord_gesture();
    if (down && preset_gesture_owner == PG_CHORD && (enc || enc2)) cancel_chord_gesture();
    // Fine+Preset alone is undo. Adding A-F supersedes it with a one-shot
    // bank reset, independent of which of these three controls arrived first.
    if (down && af && switch_pressed(FINE_BUTTON) && !other && !reset &&
        !chord_cancelled && !enc && !enc2 &&
        (preset_gesture_owner == PG_CHORD || preset_gesture_owner == PG_UNDO)) {
        smn_chords_reset();
        cancel_chord_gesture();
    }
    if ((preset_gesture_owner == PG_MENU || preset_gesture_owner == PG_UNDO) && af)
        cancel_chord_gesture();
    if (chord_cancelled || reset || other) {
        if (preset_gesture_owner != PG_NONE) preset_gesture_owner = PG_OTHER;
        cancel_preset_events();
    }

    if (preset_gesture_owner == PG_CHORD && down) {
        if (!af) {
            if (selected_memory != UINT8_MAX && !stored && !wait_release) {
                // The release may be the first poll at the one-second edge.
                // It must never turn a completed long hold into a short recall.
                if ((uint32_t)(now - combined_start) >= 1000 * TICKS_PER_MS)
                    capture_chord_with_feedback(selected_memory, now);
                else smn_chord_recall(selected_memory);
            }
            selected_memory = UINT8_MAX;
            wait_release = stored = 0;
        } else if (!wait_release) {
            if ((af & (af - 1)) ||
                (selected_memory != UINT8_MAX && af != (1u << selected_memory))) {
                wait_release = 1;
                selected_memory = UINT8_MAX;
            } else if (selected_memory == UINT8_MAX) {
                selected_memory = 0;
                while (!(af & (1u << selected_memory))) ++selected_memory;
                combined_start = now;
            } else if (!stored && (uint32_t)(now - combined_start) >= 1000 * TICKS_PER_MS) {
                capture_chord_with_feedback(selected_memory, now);
                stored = 1;
            }
        }
    } else if (preset_gesture_owner == PG_MENU) {
        if (smn_resume_flash_available()) handle_preset_events(enc, enc2);
    } else if (preset_gesture_owner == PG_UNDO && !down) {
        // Preserve the existing Fine + Preset undo shortcut outside the menu.
        if (!af && smn_resume_flash_available()) undo_preset_action();
    } else if (preset_gesture_owner == PG_NONE && !down && !af && !other && !reset) {
        if (smn_resume_flash_available()) handle_preset_events(enc, 0);
    }
    if (!down) {
        // Releasing Preset first cancels: no recall is performed on this path.
        if (preset_gesture_owner == PG_CHORD || preset_gesture_owner == PG_OTHER)
            cancel_preset_events();
        preset_gesture_owner = PG_NONE;
        selected_memory = UINT8_MAX;
        if (!af && !reset) chord_cancelled = 0;
    }
    last_down = down;
}

void read_switches(void)
{
	uint8_t chan;

	if (switch_pressed(VOCTSW))
	{
		for (chan=0; chan<NUM_CHANNELS; chan++) {
			if (!params.osc_param_lock[chan])
				params.voct_switch_state[chan] = SW_VCA;
		}
	}
	else
	{
		for (chan=0; chan<NUM_CHANNELS; chan++) {
			if (!params.osc_param_lock[chan])
				params.voct_switch_state[chan] = SW_VOCT;
		}
	}
}
