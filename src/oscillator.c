/*
 * oscillator.c
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

#include "oscillator.h"
#include "arm_math.h"
#include "globals.h"
#include "analog_conditioning.h"
#include "audio_util.h"
#include "flash_params.h"
#include "params_update.h"
#include "envout_pwm.h"
#include "adc_interface.h"
#include "timekeeper.h"
#include "compressor.h"
#include "system_settings.h"
#include "codec_sai.h"
#include "ui_modes.h"
#include "rotary_driver.h"
#include "math_util.h"
#include "gpio_pins.h"
#include "smn_synth.h"
#include "smn_controls.h"
#include "params_lfo.h"

// The selected variant keeps its Plaits gain, then uses the SWN level/pan/mix.
_Static_assert(MONO_BUFSZ == SMN_SYNTH_BLOCK_SIZE, "SMN block must match codec");
_Static_assert(SAMPLERATE == SMN_SYNTH_SAMPLE_RATE_HZ, "SMN ramps must match codec clock");

extern enum UI_Modes 	ui_mode;
extern o_rotary 		rotary[NUM_ROTARIES];
extern o_params 		params;
extern o_calc_params	calc_params;
extern o_systemSettings	system_settings;
extern o_led_cont 		led_cont;
extern o_lfos lfos;


void process_audio_block_codec(int32_t *src, int32_t *dst)
{
	int16_t 		i_sample;
	uint8_t 		chan;
	float 			smpl;
	int32_t			outL, outR;
	float			output_buffer_evens[MONO_BUFSZ] = {0.f};
	float			output_buffer_odds[MONO_BUFSZ] = {0.f};
	float			synth_output[SMN_SYNTH_BLOCK_SIZE];
	float audio_input[MONO_BUFSZ];
	static float 	prev_level[NUM_CHANNELS] = {0.f};
	float 			interpolated_level, level_inc;

	static float 	prev_pan[NUM_CHANNELS] = {0.f};
	float 			interpolated_pan, pan_inc;

	// Codec left input is Waveform In; right input is physically unconnected.
	// Convert once per block and share the immutable mono samples across voices.
	for (i_sample = 0; i_sample < MONO_BUFSZ; ++i_sample)
		audio_input[i_sample] = convert_s24_to_s32(src[2 * i_sample]) * (1.0f / 8388608.0f);
	// Calibration uses the exact existing pitch, even with retained internal modulation doses.
	const uint8_t modulate_pitch = ui_mode == PLAY || ui_mode == SELECT_PARAMS ||
		ui_mode == RGB_COLOR_ADJUST;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
	{
		const uint8_t model_id = smn_model_id_at(calc_params.wtsel[chan]);
		const float synth_scale = 32767.0f * smn_model_gain(model_id);
		// pitch[] is the existing aligned Hz value, read once for this block.
		const float lfo = lfos.out_lpf[chan];
        float pitch = calc_params.pitch[chan];
        if (modulate_pitch && smn_glide_process_pitch(chan, &pitch)) {
            // Glide is pre-Fine: clamp AFTER tuning, as in update_pitch().
            // Multiplying an already clipped destination by a glide ratio
            // would create wrong starts or sub-minimum frequencies at bounds.
            pitch *= calc_params.tuning[chan];
            if (pitch < F_MIN_FREQ) pitch = F_MIN_FREQ;
            else if (pitch > F_MAX_FREQ) pitch = F_MAX_FREQ;
            if (params.finetune[chan] != 0) pitch *= params.random[chan];
        }
		const SmnTimbreControls timbre = smn_controls_process_lfo(chan, lfo);
		const SmnSynthParameters synth_params = {
			.frequency_hz = modulate_pitch ? smn_modulation_process_pitch(chan,
				pitch, lfo, params.key_sw[chan] != ksw_MUTE,
				lfos.shape[chan]) : calc_params.pitch[chan],
			.harmonics = timbre.harmonics,
			.timbre = timbre.timbre, .morph = timbre.morph,
			.model_id = model_id
		};
		smn_synth_render_with_input(chan, &synth_params, audio_input, synth_output, MONO_BUFSZ);
		read_level_and_pan(chan);
		level_inc = (calc_params.level[chan] - prev_level[chan]) / MONO_BUFSZ;
		interpolated_level = prev_level[chan];
		prev_level[chan] = calc_params.level[chan];

		const float pan = smn_modulation_process_pan(chan, params.pan[chan], lfo);
		pan_inc = (pan - prev_pan[chan]) / MONO_BUFSZ;
		interpolated_pan = prev_pan[chan];
		prev_pan[chan] = pan;

		for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++)
		{
			smpl = synth_output[i_sample] * synth_scale * interpolated_level;
			interpolated_level += level_inc;

			output_buffer_evens[i_sample] += smpl * interpolated_pan;
			output_buffer_odds[i_sample] += smpl * (1.f - interpolated_pan);
			interpolated_pan += pan_inc;

			if (chan==5)
			{
				outL = (int32_t)(output_buffer_evens[i_sample] * system_settings.master_gain);
				outR = (int32_t)(output_buffer_odds[i_sample] * system_settings.master_gain);

				*dst++ = compress(outL);
				*dst++ = compress(outR);

			}
		}
	}

}

void update_oscillators(void){
	int8_t chan;

	check_reset_navigation();
	update_wt();
	read_all_keymodes();

	combine_transpose_spread();
	compute_transpositions();
	update_transpose_cv();

	read_ext_trigs();

	for (chan = 0; chan < NUM_CHANNELS; chan++){

		if ((ui_mode != SELECT_PARAMS) && (ui_mode != RGB_COLOR_ADJUST)) {

			read_noteon(chan);

			if(ui_mode == PLAY)
			{
				read_lfomode(chan);
				read_lfoto_vca_vco (chan);
			}
		}
		update_pitch (chan);

		if (ui_mode == PLAY)
			update_noise(chan);
	}

}

void start_osc_updates(void){
	start_timer_IRQ(OSC_TIM_number, &update_oscillators);
}
