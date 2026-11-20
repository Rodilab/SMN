/*
 * params_update.h
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

#pragma once

#include <stm32f7xx.h>
#include "globals.h"
#include "smn_patch_extra.h"
#include "hardware_controls.h"
#include "smn_controls.h"
#include "lfo_wavetable_bank.h"

// Keep the original 15-byte sphere mask in saved presets as reserved data.
#define SMN_LEGACY_SPHERE_MASK_BYTES 15

#define F_MIN_FREQ			(0.1)
#define F_BASE_FREQ			16.35
#define F_MAX_FREQ			(F_SAMPLERATE*3 - 36000.0) //96300.0
#define INIT_OCT			3	// C4 ~261Hz
#define MIN_OCT				(-3)
#define MAX_OCT				14
#define NUM_NOTES 			12
#define NUM_CHORDS			26
#define NUM_WTSEL_SPREADS	18
#define NUM_SCALES			2
#define NOTES_PER_SCALE		NUM_OUTRING_LEDS +1


// DISPLAY TIMERS: specified in # of tick
#define OSC_PARAM_LOCK_TIMER_LIMIT		700
#define WT_POS_LOCK_TIMER_LIMIT			2000
#define FINETUNE_TIMER_LIMIT			700
#define SCALE_TIMER_LIMIT				700
#define TRANSPOSE_TIMER_LIMIT			3000
#define OCTAVE_TIMER_LIMIT				700
#define PRESET_TIMER_LIMIT				2000
#define MODEL_SEL_TIMER_LIMIT			2000
#define GLOBRIGHT_TIMER_LIMIT			100

// SCALING
#define F_SCALING_FINETUNE_WRAP			1.00057779 // 100th root of 12th root of 2
#define F_SCALING_TRANSPOSE				1.05946309436

#define NUM_DIM_STEPS					50.0
#define NUM_DIM_FINE_STEPS				250.0
#define F_SCALING_NAVIGATE				((SMN_CONTROL_CYCLE_UNITS * 0.5f) / NUM_DIM_STEPS) // 0.06
#define F_SCALING_FINE_NAVIGATE			((SMN_CONTROL_CYCLE_UNITS * 0.5f) / NUM_DIM_FINE_STEPS) // 0.012

#define F_SCALING_DISPERSION			0.05
#define F_SCALING_FINE_DISPERSION		0.01

#define F_SCALING_RANDOM				0.0015
#define RANDOM_UPDATE_TIME				7

#define F_SCALING_MAX_VCACV_GAIN		1.0

#define MAX_FINETUNE_WRAP 				(2160)
#define MIN_FINETUNE_WRAP 				(-2160)

#define MAX_TRANSPOSE_WRAP				125
#define MIN_TRANSPOSE_WRAP				-126

#define QTZ_CHANGE_LOCKOUT_PERIOD		50
// TIM6 at 1.8 kHz: six equal observations span 2.78 ms, then attack next tick.
#define NOTE_PITCH_STABLE_READS 6u

#define F_SCALING_NAVIGATE_GLOBAL_BRIGHTNESS 	0.01
#define F_MIN_GLOBAL_BRIGHTNESS					0.01
#define F_DEFAULT_GLOBAL_BRIGHTNESS				0.8

enum armFlags
{
	armf_NOTE_ON,
	armf_LFOTOVCA,
	armf_LFOMODE,
	armf_KEYMODE,
	NUM_ARM_FLAGS
};

enum MuteNoteKeyStates {
	ksw_MUTE,
	ksw_NOTE,
	ksw_KEYS,
	ksw_KEYS_EXT_TRIG,
	ksw_KEYS_EXT_TRIG_SUSTAIN,

	NUM_MUTE_NOTE_KEY_STATES
};

#define FW_V1_PADDING (NUM_CHANNELS*32 - SMN_LEGACY_SPHERE_MASK_BYTES) //padding for future features in o_params
#define FW_V2_ADDED_PARAMS_SIZE (sizeof(float)*NUM_CHANNELS)

enum PanStates {
	pan_INACTIVE,
	pan_PANNING,
	pan_CACHED_LEVEL,
};

typedef struct o_calc_params{
	uint8_t		wtsel 					[NUM_CHANNELS];

	int32_t 	transpose				[NUM_CHANNELS];	//Combination of transpose encoder + chord
	float 		transposition 			[NUM_CHANNELS];	//Calculated pitch change based on transpose amount
	float 		tuning 					[NUM_CHANNELS];
	float 		pitch 					[NUM_CHANNELS];
	float		level  					[NUM_CHANNELS];
	float		cached_level			[NUM_CHANNELS];
	float 		voct 					[NUM_CHANNELS];
	float		qtz_freq				[NUM_CHANNELS];
    int8_t pitch_octave[NUM_CHANNELS]; // Effective octave of the sounding note; RAM only.

	uint8_t 	prev_qtz_note			[NUM_CHANNELS];
	int8_t 		prev_qtz_oct			[NUM_CHANNELS];

	// FLAGS
	uint8_t 	already_handled_button 	[NUM_CHANNELS + 2];
	uint8_t 	button_safe_release		[2];
	uint8_t		keymode_pressed;
	enum armFlags armed[NUM_ARM_FLAGS]	[NUM_CHANNELS];
	uint8_t		lock_change_staged		[NUM_CHANNELS];
	enum PanStates	adjusting_pan_state		[NUM_CHANNELS];

	uint8_t		gate_in_is_sustaining	[NUM_CHANNELS]		;
} o_calc_params;

typedef struct o_params{

	// MODEL SELECTION (legacy names keep the preset byte layout unchanged)
	int8_t		wtsel_cv;
	int8_t		wtsel_enc				[NUM_CHANNELS];
	uint8_t 	wtsel_spread_cv; //Legacy storage; WT Spread retired
	uint8_t 	wt_bank 				[NUM_CHANNELS];	//Legacy sphere bank; unused by SMN play

	// H/T/M PHASES AND DISPERSION (retain legacy fields for preset compatibility)
	float		wt_nav_enc				[3][NUM_CHANNELS];
	float		wt_nav_cv				[3];
	float 		dispersion_enc; //SMN Harmonics dispersion, bounded 0..1
	float 		dispersion_cv;
	int8_t		disppatt_enc; //Legacy selector, inactive
	int8_t		disppatt_cv;
	float		wt_browse_step_pos_enc	[NUM_CHANNELS];
	float		wt_browse_step_pos_cv;
	int8_t		wtsel_spread_enc		[NUM_CHANNELS]; //Legacy storage, inactive

	// OSCILLATORS
	int8_t		oct 					[NUM_CHANNELS];
	int16_t 	finetune 				[NUM_CHANNELS];
	int32_t 	transpose_enc			[NUM_CHANNELS];
	float		transpose_cv;
	int32_t 	spread_enc				[NUM_CHANNELS];
	uint8_t 	spread_cv; // Legacy byte layout only; CV now selects SMN memories.
	int8_t 		indiv_scale 			[NUM_CHANNELS];
	int8_t 		indiv_scale_buf 		[NUM_CHANNELS];		// SC v1 tag + global root (former per-voice cache)

	// KEYS
	uint8_t 	note_on 				[NUM_CHANNELS];		//Mute state
	uint8_t		note_on_buf				[NUM_CHANNELS];		//Stashed mute state for when in note/key mode
	uint8_t 	new_key 				[NUM_CHANNELS];
	enum MuteNoteKeyStates	key_sw		[NUM_CHANNELS];
	uint8_t		qtz_note_changed 		[NUM_CHANNELS];

	// NOISE LINE
	float		random 					[NUM_CHANNELS];
	uint8_t		noise_on;

	enum VoctVcaStates 	voct_switch_state[NUM_CHANNELS];
	uint8_t 	osc_param_lock 			[NUM_CHANNELS];
	uint8_t 	wt_pos_lock 			[NUM_CHANNELS];	//Updated with osc_param_lock by toggle_lock()
	uint8_t		wtsel_lock				[NUM_CHANNELS];	//Updated with osc_param_lock by toggle_lock()

	//v1.2:
	uint8_t		enabled_spheres			[SMN_LEGACY_SPHERE_MASK_BYTES]; //Legacy mask; models ignore it

	//v2.0
	float		pan						[NUM_CHANNELS];

	uint8_t		PADDING					[FW_V1_PADDING - FW_V2_ADDED_PARAMS_SIZE]; //SMN IDs, T/M dispersion and integer internal modulation targets
    // Appended after the unchanged 504-byte legacy prefix. Not a flash image.
    SmnPatchExtra smn_extra;
} o_params;

// Persistent flag uses one reserved RAM byte; live CV/animation state is not saved.
#define SMN_POLYROTATE_BYTE 98
static inline uint8_t smn_polyrotate_enabled(const o_params *p)
{
    return p->PADDING[SMN_POLYROTATE_BYTE] == 1 && p->indiv_scale[0] > 0 &&
        p->indiv_scale[0] < 25 /* current Scale IDs 0..24 */;
}
// Shared feedback is published by TIM6/main and read only at LED cadence.
typedef struct {
    uint32_t animation_start, flash_start[NUM_CHANNELS];
    uint8_t animation, flash_mask, suppress_transpose;
} SmnPolyrotateFeedback;
extern SmnPolyrotateFeedback smn_polyrotate_feedback;
void smn_polyrotate_poll(void);
void smn_polyrotate_reset(void);

void 		init_params(void);
void 		init_calc_params(void);
void 		init_pitch_params();
void 		init_param_object(o_params *t_params);

void 		check_reset_navigation(void);
void 		cache_uncache_pitch_params(enum CacheUncache cache_uncache);

void 		read_noteon(uint8_t i);
void request_chord_envelope(uint8_t chan);
void 		read_ext_trigs(void);
void 		read_level_and_pan(uint8_t chan);
float		default_pan(uint8_t chan);
float 		read_vca_cv(uint8_t chan);

void 		read_lfoto_vca_vco(uint8_t i);
void 		read_lfomode(uint8_t i);

void 		apply_keymode(uint8_t chan, enum MuteNoteKeyStates new_keymode);
void 		read_all_keymodes(void);

void 		cache_uncache_keys_params_and_lfos(uint8_t chan, enum CacheUncache cache_uncache);

void 		update_lfomode(uint8_t i);
void 		update_pitch(uint8_t chan);
void 		update_noise(uint8_t chan);

void 		read_freq(void);

// --------- LOCK ---------
void 		update_osc_param_lock(void);
void read_parameter_locks(void);
void 		toggle_lock(uint8_t chan);

// --------- OCTAVE ---------
void 		update_oct(int16_t tmp);
void		reset_octaves(void);

// --------- SCALE ---------
void 		update_scale(int16_t tmp);
void update_scale_root(int16_t tmp);

// --------- TUNE ---------
void 		update_finetune(int16_t tmp);
void 		retune_oscillators(void);
void 		spread_finetune(int16_t tmp);
void 		compute_tuning (uint8_t chan);

// --------- TRANSPOSE ---------
void 		update_transpose(int16_t tmp);
void 		update_transpose_cv(void);
void 		update_spread(int16_t tmp);
void 		combine_transpose_spread(void);
void 		trim_array(int32_t *a, uint32_t num_elements, int32_t min, int32_t max);
void 		compute_transpositions(void);
float 		compute_transposition(int32_t transpose);
void 		reset_notes(void);

void 		update_wt(void);

// ##### READ ENCODERS ######
void 		read_nav_encoder(uint8_t dim);
void		read_browse_encoder(void);
void 		read_load_save_encoder(void);

void 		update_oscillators(void);
void 		start_osc_updates(void);

// --------- MODEL SELECTION ---------
void 		read_wtsel(int16_t wtsel);
void 		read_wtsel_cv(void);
void 		update_wtsel(void);

// --------- H/T/M CONTROLS ---------

void 		update_wt_nav(uint8_t wt_dim, float wt_pos_increment);
void 		update_wt_nav_cv(uint8_t wt_dim);
void 		calc_wt_pos(uint8_t chan);

void 		update_all_smn_controls(void);

// -------- WT DISPERSION ----------
void 		update_wt_disp(void);
// Global dispersion setting for LEDs: manual + live CV, capped to 0..1.
// Does not inspect per-voice locks or advance the audio ramps.
float smn_dispersion_display_value(uint8_t dimension);

void		read_clear_settings(void);

void 		read_switches(void);

uint32_t 	abs(int32_t v);
