/*
 * params_lfo.c
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

#include <stm32f7xx.h>
#include "params_lfo.h"
#include "params_lfo_period.h"
#include "params_lfo_clk.h"
#include "params_update.h"
#include "params_changes.h"
#include "smn_chords.h"
#include "UI_conditioning.h"
#include "key_combos.h"
#include "preset_manager.h"
#include "math_util.h"
#include "led_cont.h"
#include "hardware_controls.h"
#include "system_settings.h"
#include "analog_conditioning.h"
#include "ui_modes.h"

extern o_params params;
extern o_calc_params calc_params;
extern o_led_cont led_cont;
extern o_macro_states 	macro_states;
extern o_systemSettings system_settings;
extern o_analog analog[NUM_ANALOG_ELEMENTS];
extern enum UI_Modes ui_mode;

o_lfos   lfos;
uint16_t divmult_cv;

void update_lfos(void)
{
	update_lfo_params();
	read_ext_clk();
	update_lfo_calcs();
	update_lfo_wt_pos();
	update_lfo_sample();
}

void clear_lfo_locks(void)
{
	uint8_t i;
	for (i = 0; i < NUM_CHANNELS; i++)
		lfos.locked[i] = 0;
}

void use_internal_lfo_base(void)
{
	lfos.use_ext_clock = 0;
}

void init_lfos(void)
{
	uint8_t i;

	// Initial phase offsets, in the existing 24-position cycle.
	if (!lfos.locked[0]) lfos.phase_id[0] =  0;
	if (!lfos.locked[1]) lfos.phase_id[1] =  20;
	if (!lfos.locked[2]) lfos.phase_id[2] =  16;
	if (!lfos.locked[3]) lfos.phase_id[3] =  12;
	if (!lfos.locked[4]) lfos.phase_id[4] =  8;
	if (!lfos.locked[5]) lfos.phase_id[5] =  4;

	lfos.phase_switch = 0;

	if (!lfos.use_ext_clock)
	{
		lfos.period[REF_CLK] 		= LFO_INIT_PERIOD;
		lfos.divmult_id[REF_CLK] 	= LFO_UNITY_DIVMULT_ID;
		lfos.divmult[REF_CLK] 		= 1.0;
		flag_all_lfos_recalc();
	}

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		if (!lfos.locked[i])
		{
			lfos.shape[i]			= LFO_INIT_SHAPE;
			lfos.gain[i]			= LFO_INIT_GAIN;
			lfos.cycle_pos[i]		= 0;
			lfos.div_cnt[i]			= 0;
			lfos.out_lpf[i] 		= 1;
			lfos.envout_pwm[i]		= 0;
			lfos.preload[i]			= 0;
			lfos.trigout[i]			= 0;
			lfos.audio_mode[i]		= 0;

			lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);

			lfos.trig_armed[i] = 0;
		}
	}

	init_lfo_speed();
}

void init_lfos_shape(void)
{
	uint8_t i;
	for (i = 0; i < NUM_CHANNELS; i++){
		if (!lfos.locked[i])
		{
			lfos.shape[i] = LFO_INIT_SHAPE;
			lfos.mode[i] = lfot_SHAPE;
		}
	}
}

void init_lfo_speed(void){

	uint8_t chan;

	lfos.divmult_id[GLO_CLK] = LFO_UNITY_DIVMULT_ID;

	for (chan = 0; chan < NUM_CHANNELS; chan++){
		if (!lfos.locked[chan]) {
			lfos.divmult_id[chan] = LFO_UNITY_DIVMULT_ID;
		}
	}

	flag_all_lfos_recalc();
	update_lfo_calcs();
}

void init_lfo_object(o_lfos *t_lfo){
	uint8_t i;

	// init phase (60deg)
	t_lfo->phase_id[0] =  0;
	t_lfo->phase_id[1] =  20;
	t_lfo->phase_id[2] =  16;
	t_lfo->phase_id[3] =  12;
	t_lfo->phase_id[4] =  8;
	t_lfo->phase_id[5] =  4;

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		t_lfo->phase[i] = calc_lfo_phase(lfos.phase_id[i]);

		t_lfo->divmult_id[i] 				= LFO_UNITY_DIVMULT_ID;
		t_lfo->divmult_id_global_locked[i] 	= LFO_UNITY_DIVMULT_ID;
		t_lfo->shape[i]						= LFO_INIT_SHAPE;
        t_lfo->shape_buf[i] = LFO_INIT_SHAPE;
		t_lfo->gain[i]						= LFO_INIT_GAIN;
		t_lfo->locked[i]					= 0;
		t_lfo->mode[i] 						= 0;
		t_lfo->to_vca[i] 					= 0;
		t_lfo->muted[i] 					= 0;

		t_lfo->out_lpf[i] 					= 1;
		t_lfo->envout_pwm[i]				= 0;
		t_lfo->preload[i]					= 0;

		t_lfo->cycle_pos[i]					= 0;
		t_lfo->div_cnt[i]					= 0;
		t_lfo->wt_pos[i]					= 0;

		t_lfo->trigout[i]					= 0;
		t_lfo->audio_mode[i]				= 0;

		// flags
		t_lfo->trig_armed[i] 				= 0;

		t_lfo->to_vca_buf[i] 				= t_lfo->to_vca[i];
	}

	t_lfo->phase_switch 		= 0;

	t_lfo->divmult_id[GLO_CLK] 	= LFO_UNITY_DIVMULT_ID;
	t_lfo->cycle_pos[GLO_CLK] 	= 0;

	t_lfo->use_ext_clock 		= 0;

	t_lfo->period[REF_CLK] 		= LFO_INIT_PERIOD;
	t_lfo->divmult_id[REF_CLK] 	= LFO_UNITY_DIVMULT_ID;
	t_lfo->divmult[REF_CLK] 	= 1.0;
	t_lfo->cycle_pos[REF_CLK] 	= 0;

}

void update_lfo_sample(void)
{
	float 			lfo_frac, pos_in_table;
	uint16_t		rh0, rh1;
	uint8_t			chan;

	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		if (!lfos.muted[chan])
		{
			pos_in_table		= lfos.wt_pos[chan] * F_LFO_TABLELEN;
			rh0 				= (uint16_t)pos_in_table;
			rh1 				= (rh0 + 1) & (LFO_TABLELEN-1); //if rh0==255, then rh1 should = 0. (255+1)&(255) == 0x100 & 0x0FF == 0
			lfo_frac 			= pos_in_table - (float)rh0;
			lfos.preload[chan] 	= (((lfo_wavetable[lfos.shape[chan]][rh0] * (1.0-lfo_frac) + lfo_wavetable[lfos.shape[chan]][rh1] * lfo_frac))) ;
		}
		else
			lfos.preload[chan] 	= 0;
	}
}

void update_lfo_params(void)
{
	apply_lfo_reset();
	read_LFO_phase();
	read_LFO_shape();
	read_LFO_speed_gain();
	read_lfo_cv();
}

void apply_lfo_reset(void){
	enum ResetTypes {
		NO_RESET_STAGED,
		RESET_SPEEDS,
		RESET_SHAPES,
		RESET_PHASES,
		RESET_ALL
	};
	static enum ResetTypes reset_staged = NO_RESET_STAGED;
	uint8_t i;

	if (key_combo_reset_lfos_all()){ //reset_all happens immediately and cancels any other staged reset action
		cancel_chord_gesture();
		exit_preset_manager();
		stop_all_displays();
		use_internal_lfo_base();
		init_lfos();

		reset_staged = RESET_ALL;
	}

	if (key_combo_reset_lfos_speeds() && !reset_staged){
		cancel_chord_gesture();
		exit_preset_manager();
		stop_all_displays();

		reset_staged = RESET_SPEEDS;
	}

	if (key_combo_reset_lfos_shapes() && !reset_staged){
		cancel_chord_gesture();
		exit_preset_manager();
		stop_all_displays();

		reset_staged = RESET_SHAPES;
	}

	if (key_combo_reset_lfos_phases() && !reset_staged){
		// exit_preset_manager();
		stop_all_displays();

		lfos.phase_switch = 1- lfos.phase_switch;

		reset_staged = RESET_PHASES;
	}

	if (reset_staged && key_combo_reset_lfos_released())
	{
		if (reset_staged == RESET_SPEEDS)
		{
			for ( i =0; i < NUM_CHANNELS; i++) {
				if (!lfos.locked[i])
					stage_resync(i);
			}
			init_lfo_speed();
		}
		else if (reset_staged == RESET_SHAPES) {
			init_lfos_shape();
		}
		else if (reset_staged == RESET_PHASES)
		{
			if (lfos.phase_switch)
			{
				// init phase (0deg)
				if (!lfos.locked[0]) lfos.phase_id[0] =  0;
				if (!lfos.locked[1]) lfos.phase_id[1] =  0;
				if (!lfos.locked[2]) lfos.phase_id[2] =  0;
				if (!lfos.locked[3]) lfos.phase_id[3] =  0;
				if (!lfos.locked[4]) lfos.phase_id[4] =  0;
				if (!lfos.locked[5]) lfos.phase_id[5] =  0;
			}
			else
			{
				// init phase (60deg)
				if (!lfos.locked[0]) lfos.phase_id[0] =  0;
				if (!lfos.locked[1]) lfos.phase_id[1] =  20;
				if (!lfos.locked[2]) lfos.phase_id[2] =  16;
				if (!lfos.locked[3]) lfos.phase_id[3] =  12;
				if (!lfos.locked[4]) lfos.phase_id[4] =  8;
				if (!lfos.locked[5]) lfos.phase_id[5] =  4;
			}

			for ( i =0; i < NUM_CHANNELS; i++) {
				if (!lfos.locked[i])
				{
					lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);
					lfos.cycle_pos[i] = 0;
				}
			}
		}

		else if (reset_staged == RESET_ALL){
			//here we can do anything upon release of buttons after doing a reset all
		}

		reset_staged = NO_RESET_STAGED;
	}
}

void read_LFO_speed_gain(void)
{
	int16_t	enc, enc2;

	enc  = pop_encoder_q(pec_LFOSPEED);
	enc2 = pop_encoder_q(sec_LFOGAIN);

	if(enc)
		read_lfo_speed(enc);
	else if (enc2)
		update_lfo_gain(enc2);

}

void update_lfo_gain(int16_t turn)
{
	uint8_t i;
	uint8_t channels_changed;
	float  	turn_amt;
	float 	gain[NUM_CHANNELS];

	turn_amt = turn * (switch_pressed(FINE_BUTTON) ? F_SCALING_FINE_LFO_GAIN : F_SCALING_LFO_GAIN);

	for (i=0; i<NUM_CHANNELS; i++)
		gain[i] = lfos.gain[i];

	channels_changed = change_param_f(gain, turn_amt);

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (channels_changed & (1<<i))
			lfos.gain[i] = _CLAMP_F(gain[i] + turn_amt, F_SCALING_LFO_GAIN, 1.0);
	}
}

void read_lfo_speed(int16_t turn)
{
	uint8_t i;
	uint8_t fine_pressed;
	float turn_amt, test_divmult_id;

	if (!turn) return;

	// FINE
	turn_amt = turn;
	fine_pressed = switch_pressed(FINE_BUTTON);

	if (fine_pressed)
		turn_amt *= F_SCALING_FINE_LFO_SPEED;

	// GLOBAL
	if (macro_states.all_af_buttons_released)
	{
		test_divmult_id = lfos.divmult_id[GLO_CLK] + turn_amt;
		if (!fine_pressed) test_divmult_id = (int8_t)(test_divmult_id);

		lfos.divmult_id[GLO_CLK] = _CLAMP_F(test_divmult_id, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);

		flag_all_lfos_recalc();

		if (lfos.use_ext_clock)
			stage_resync_lfos();

	}

	// INDIVIDUAL
	else{
		for (i = 0; i < NUM_CHANNELS; i++){
			if (button_pressed(i))
			{
				calc_params.already_handled_button[i] = 1;

				lfos.divmult_id[i] = _CLAMP_F(lfos.divmult_id[i] + turn_amt, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);

				flag_lfo_recalc(i);

				if (lfos.use_ext_clock) {
					stage_resync(i);
				}
			}
		}
	}
}

void sync_LFO_phase(void)
{
	uint8_t i;
	uint8_t channels_changed;
	int16_t	phase[NUM_CHANNELS];

	for (i=0; i<NUM_CHANNELS; i++)
		phase[i] = 0;

	channels_changed = change_param_i16(phase, 1);

	if (channels_changed==0b111111)
		stage_resync_lfos();
	else
	{
		for (i=0; i<NUM_CHANNELS; i++)
		{
			if (channels_changed & (1<<i))
				stage_resync(i);
		}
	}
}

void read_LFO_phase(void)
{
	uint8_t			i, fine, enc_pressed;
	int8_t			enc_turn;
	float			enc_amount;
	static uint8_t 	stage_phase_sync=0;
	static uint8_t 	disable_phase_sync=0;

	enc_turn  = pop_encoder_q(sec_LFOPHASE);
	enc_pressed = rotary_pressed(rotm_LFOSHAPE);
	fine = switch_pressed(FINE_BUTTON);

	if (enc_pressed)
	{
		if (!disable_phase_sync)
			stage_phase_sync=1;
	} else
	{
		if (!enc_turn && stage_phase_sync)
			sync_LFO_phase();
		stage_phase_sync = 0;
		disable_phase_sync = 0;
	}

	if (enc_turn)
	{
		disable_phase_sync = 1;
		stage_phase_sync = 0;
		if (fine)
			enc_amount = -enc_turn * F_SCALING_FINE_LFO_PHASE;
		else
			enc_amount = -enc_turn;

		// GLOBAL
		if (macro_states.all_af_buttons_released){
			for (i = 0; i < NUM_CHANNELS; i++){
				if (!lfos.locked[i])
				{
					if (!fine)
						lfos.phase_id[i] = _WRAP_I16(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);
					else
						lfos.phase_id[i] = _WRAP_F(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);

					lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);
				}
			}
		}

		// INDIVIDUAL
		else{
			for (i = 0; i < NUM_CHANNELS; i++){
				if(button_pressed(i))
				{
					if (!fine)
						lfos.phase_id[i] = _WRAP_I16(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);
					else
						lfos.phase_id[i] = _WRAP_F(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);

					calc_params.already_handled_button[i] = 1;
					lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);
				}
			}
		}
	}
}

// Only called on encoder movement; sample and sustain readers keep direct IDs.
static uint16_t change_lfo_shape(uint16_t shape, int8_t movement, uint8_t fine)
{
    if (shape >= NUM_LFO_SHAPES) shape = LFO_INIT_SHAPE;
    const int bank = shape / LFO_BANK_SIZE;
    int position = shape % LFO_BANK_SIZE;
    if (fine) {
        position += movement;
        if (position < 0) position = 0;
        if (position >= LFO_BANK_SIZE) position = LFO_BANK_SIZE - 1;
        return bank * LFO_BANK_SIZE + position;
    }
    const int next_bank = ((bank + movement) % LFO_NUM_BANKS + LFO_NUM_BANKS) % LFO_NUM_BANKS;
    return next_bank * LFO_BANK_SIZE + position;
}

void read_LFO_shape(void)
{
	uint8_t		i;
	int8_t		enc;

	enc  = pop_encoder_q(pec_LFOSHAPE);

	if(enc)
	{
        const uint8_t fine = switch_pressed(FINE_BUTTON);
		if (macro_states.all_af_buttons_released)
		{
			for (i = 0; i < NUM_CHANNELS; i++){
				if (!lfos.locked[i])
				{
					lfos.shape[i] = change_lfo_shape(lfos.shape[i], enc, fine);
					led_cont.lfoshape_started[i] = HAL_GetTick();
					led_cont.ongoing_lfoshape[i] = params.key_sw[i] != ksw_MUTE;
				}
			}
		}

		// INDIVIDUAL
		else{
			for (i=0; i < NUM_CHANNELS; i++){
				if(button_pressed(i))
				{
					lfos.shape[i] = change_lfo_shape(lfos.shape[i], enc, fine);
					calc_params.already_handled_button[i] = 1;

					led_cont.lfoshape_started[i] = HAL_GetTick();
					led_cont.ongoing_lfoshape[i] = params.key_sw[i] != ksw_MUTE;
				}
			}
		}
	}
}

float calc_lfo_phase(float phase_id)
{
	return phase_id/((float)LFO_PHASE_TABLELEN);

}

void read_lfo_cv(void)
{
	static uint16_t last_divmult_cv=0;

	if (system_settings.lfo_cv_mode == LFOCV_SPEED)
		divmult_cv = (analog[LFO_CV].bracketed_val * NUM_DIVMULTS)>>12;
	else
		divmult_cv = 0;

	if (divmult_cv != last_divmult_cv) {
		// flag_lfo_recalc(GLO_CLK);
		flag_all_lfos_recalc();
		last_divmult_cv = divmult_cv;
	}

	// else if (system_settings.lfo_cv_mode 	== LFOCV_SHAPE){new_cv  = (analog[LFO_CV].bracketed_val * NUM_LFO_SHAPES   / 4095);}
	// else if (system_settings.lfo_cv_mode 	== LFOCV_GROOVE){new_cv = (analog[LFO_CV].bracketed_val * 100   / 4095);}

}

void init_lfo_to_vc_mode(void)
{
	uint8_t i;

	for (i = 0; i < NUM_CHANNELS; i++){
		lfos.to_vca[i] 				= 0;
		lfos.to_vca_buf[i] 			= lfos.to_vca[i];
		lfos.mode[i] 				= 0;
	}
}

void cache_uncache_lfomode(uint8_t chan, enum CacheUncache cache_uncache)
{
	static uint8_t cached_mode[NUM_CHANNELS];

	switch (cache_uncache){

		case CACHE:
			cached_mode[chan] = lfos.mode[chan];
			break;

		case UNCACHE:
			lfos.mode[chan] = cached_mode[chan];
			break;
	}
}
