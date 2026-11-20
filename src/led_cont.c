/*
 * led_cont.c - handles SWN LEDs
 *
 * Author: Dan Green (danngreen1@gmail.com), Hugo Paris (hugoplho@gmail.com)
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

#include "globals.h"
#include "led_cont.h"
#include "smn_models.h"
#include "smn_controls.h"
#include "smn_model_selection.h"
#include "smn_chords.h"
#include "led_colors.h"
#include "envout_pwm.h"
#include "params_update.h"
#include "params_lfo.h"
#include "params_lfo_period.h"
#include "gpio_pins.h"
#include "flash_params.h"
#include "analog_conditioning.h"
#include "UI_conditioning.h"
#include "preset_manager.h"
#include "preset_manager_UI.h"
#include "math_util.h"
#include "drivers/mono_led_driver.h"
#include "system_settings.h"
#include "ui_modes.h"
#include "key_combos.h"

#include "drivers/leds_pwm.h"
#include "quantz_scales.h"
#include "calibrate_voct.h"
#include "flash_params.h"
#include "timekeeper.h"
#include "ui_modes.h"

extern SystemCalibrations *system_calibrations;

// UI
extern enum UI_Modes 			ui_mode;

// LEDs
o_led_cont				 		led_cont;
extern const o_rgb_led			RGB_LED_OFF;

// Params
extern 		o_params 			params;
extern		o_calc_params		calc_params;
extern 		o_lfos				lfos;
extern		o_preset_manager	preset_mgr;
extern		o_systemSettings	system_settings;

// Hardware
extern 		o_monoLed   		monoLed[NUM_MONO_LED];
extern 		o_macro_states		macro_states;

// LED maps
extern const uint8_t	led_button_map[NUM_BUTTONS];
extern const uint8_t	led_rotary_map[NUM_LED_ROTARIES];
extern const uint8_t 	ledstring_map[NUM_CHANNELS+1];
extern const uint8_t 	led_outring_map[NUM_LED_OUTRING];
extern const uint8_t 	led_inring_map[NUM_LED_INRING];

const uint8_t OCT_OUTRING_MAP[NUM_LED_OUTRING] 		    	= { 9, 10, 11, 12, 13, 14, 15, 16, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8 };

// Color palettes
extern uint32_t colorPalette[NUM_LED_COLORS][NUM_PALETTE_COLORS];

// TABLES
extern const float 	exp_1voct_10_41V[4096];

const uint16_t CH_COLOR_MAP[6][3] = {
	{ 1		, 600	, 954	},
	{ 1		, 12	, 954	},
	{ 941  	, 366	, 954	},
	{ 941 	, 35 	, 947	},
	{ 954 	, 176	, 21 	},
	{ 800 	, 1	 	, 50	}
};

// HSV at V=1, S=0.5..1 by position; linear LED values 0..1023.
// H by bank (degrees): 210, 270, 60, 30, 120, 180, 300, 0. Generated offline.
const uint16_t LFO_BANK_COLOR[NUM_LFO_SHAPES][3] = {
    { 512, 767, 1023 },
    { 496, 759, 1023 },
    { 480, 751, 1023 },
    { 464, 743, 1023 },
    { 448, 735, 1023 },
    { 432, 727, 1023 },
    { 416, 719, 1023 },
    { 400, 711, 1023 },
    { 384, 703, 1023 },
    { 368, 695, 1023 },
    { 352, 687, 1023 },
    { 336, 679, 1023 },
    { 320, 671, 1023 },
    { 304, 663, 1023 },
    { 288, 655, 1023 },
    { 272, 647, 1023 },
    { 256, 639, 1023 },
    { 240, 631, 1023 },
    { 224, 623, 1023 },
    { 208, 615, 1023 },
    { 192, 607, 1023 },
    { 176, 599, 1023 },
    { 160, 591, 1023 },
    { 144, 583, 1023 },
    { 128, 575, 1023 },
    { 112, 567, 1023 },
    { 96, 559, 1023 },
    { 80, 551, 1023 },
    { 64, 543, 1023 },
    { 48, 535, 1023 },
    { 32, 527, 1023 },
    { 16, 519, 1023 },
    { 0, 512, 1023 },
    { 767, 512, 1023 },
    { 759, 496, 1023 },
    { 751, 480, 1023 },
    { 743, 464, 1023 },
    { 735, 448, 1023 },
    { 727, 432, 1023 },
    { 719, 416, 1023 },
    { 711, 400, 1023 },
    { 703, 384, 1023 },
    { 695, 368, 1023 },
    { 687, 352, 1023 },
    { 679, 336, 1023 },
    { 671, 320, 1023 },
    { 663, 304, 1023 },
    { 655, 288, 1023 },
    { 647, 272, 1023 },
    { 639, 256, 1023 },
    { 631, 240, 1023 },
    { 623, 224, 1023 },
    { 615, 208, 1023 },
    { 607, 192, 1023 },
    { 599, 176, 1023 },
    { 591, 160, 1023 },
    { 583, 144, 1023 },
    { 575, 128, 1023 },
    { 567, 112, 1023 },
    { 559, 96, 1023 },
    { 551, 80, 1023 },
    { 543, 64, 1023 },
    { 535, 48, 1023 },
    { 527, 32, 1023 },
    { 519, 16, 1023 },
    { 512, 0, 1023 },
    { 1023, 1023, 512 },
    { 1023, 1023, 496 },
    { 1023, 1023, 480 },
    { 1023, 1023, 464 },
    { 1023, 1023, 448 },
    { 1023, 1023, 432 },
    { 1023, 1023, 416 },
    { 1023, 1023, 400 },
    { 1023, 1023, 384 },
    { 1023, 1023, 368 },
    { 1023, 1023, 352 },
    { 1023, 1023, 336 },
    { 1023, 1023, 320 },
    { 1023, 1023, 304 },
    { 1023, 1023, 288 },
    { 1023, 1023, 272 },
    { 1023, 1023, 256 },
    { 1023, 1023, 240 },
    { 1023, 1023, 224 },
    { 1023, 1023, 208 },
    { 1023, 1023, 192 },
    { 1023, 1023, 176 },
    { 1023, 1023, 160 },
    { 1023, 1023, 144 },
    { 1023, 1023, 128 },
    { 1023, 1023, 112 },
    { 1023, 1023, 96 },
    { 1023, 1023, 80 },
    { 1023, 1023, 64 },
    { 1023, 1023, 48 },
    { 1023, 1023, 32 },
    { 1023, 1023, 16 },
    { 1023, 1023, 0 },
    { 1023, 767, 512 },
    { 1023, 759, 496 },
    { 1023, 751, 480 },
    { 1023, 743, 464 },
    { 1023, 735, 448 },
    { 1023, 727, 432 },
    { 1023, 719, 416 },
    { 1023, 711, 400 },
    { 1023, 703, 384 },
    { 1023, 695, 368 },
    { 1023, 687, 352 },
    { 1023, 679, 336 },
    { 1023, 671, 320 },
    { 1023, 663, 304 },
    { 1023, 655, 288 },
    { 1023, 647, 272 },
    { 1023, 639, 256 },
    { 1023, 631, 240 },
    { 1023, 623, 224 },
    { 1023, 615, 208 },
    { 1023, 607, 192 },
    { 1023, 599, 176 },
    { 1023, 591, 160 },
    { 1023, 583, 144 },
    { 1023, 575, 128 },
    { 1023, 567, 112 },
    { 1023, 559, 96 },
    { 1023, 551, 80 },
    { 1023, 543, 64 },
    { 1023, 535, 48 },
    { 1023, 527, 32 },
    { 1023, 519, 16 },
    { 1023, 512, 0 },
    { 512, 1023, 512 },
    { 496, 1023, 496 },
    { 480, 1023, 480 },
    { 464, 1023, 464 },
    { 448, 1023, 448 },
    { 432, 1023, 432 },
    { 416, 1023, 416 },
    { 400, 1023, 400 },
    { 384, 1023, 384 },
    { 368, 1023, 368 },
    { 352, 1023, 352 },
    { 336, 1023, 336 },
    { 320, 1023, 320 },
    { 304, 1023, 304 },
    { 288, 1023, 288 },
    { 272, 1023, 272 },
    { 256, 1023, 256 },
    { 240, 1023, 240 },
    { 224, 1023, 224 },
    { 208, 1023, 208 },
    { 192, 1023, 192 },
    { 176, 1023, 176 },
    { 160, 1023, 160 },
    { 144, 1023, 144 },
    { 128, 1023, 128 },
    { 112, 1023, 112 },
    { 96, 1023, 96 },
    { 80, 1023, 80 },
    { 64, 1023, 64 },
    { 48, 1023, 48 },
    { 32, 1023, 32 },
    { 16, 1023, 16 },
    { 0, 1023, 0 },
    { 512, 1023, 1023 },
    { 496, 1023, 1023 },
    { 480, 1023, 1023 },
    { 464, 1023, 1023 },
    { 448, 1023, 1023 },
    { 432, 1023, 1023 },
    { 416, 1023, 1023 },
    { 400, 1023, 1023 },
    { 384, 1023, 1023 },
    { 368, 1023, 1023 },
    { 352, 1023, 1023 },
    { 336, 1023, 1023 },
    { 320, 1023, 1023 },
    { 304, 1023, 1023 },
    { 288, 1023, 1023 },
    { 272, 1023, 1023 },
    { 256, 1023, 1023 },
    { 240, 1023, 1023 },
    { 224, 1023, 1023 },
    { 208, 1023, 1023 },
    { 192, 1023, 1023 },
    { 176, 1023, 1023 },
    { 160, 1023, 1023 },
    { 144, 1023, 1023 },
    { 128, 1023, 1023 },
    { 112, 1023, 1023 },
    { 96, 1023, 1023 },
    { 80, 1023, 1023 },
    { 64, 1023, 1023 },
    { 48, 1023, 1023 },
    { 32, 1023, 1023 },
    { 16, 1023, 1023 },
    { 0, 1023, 1023 },
    { 1023, 512, 1023 },
    { 1023, 496, 1023 },
    { 1023, 480, 1023 },
    { 1023, 464, 1023 },
    { 1023, 448, 1023 },
    { 1023, 432, 1023 },
    { 1023, 416, 1023 },
    { 1023, 400, 1023 },
    { 1023, 384, 1023 },
    { 1023, 368, 1023 },
    { 1023, 352, 1023 },
    { 1023, 336, 1023 },
    { 1023, 320, 1023 },
    { 1023, 304, 1023 },
    { 1023, 288, 1023 },
    { 1023, 272, 1023 },
    { 1023, 256, 1023 },
    { 1023, 240, 1023 },
    { 1023, 224, 1023 },
    { 1023, 208, 1023 },
    { 1023, 192, 1023 },
    { 1023, 176, 1023 },
    { 1023, 160, 1023 },
    { 1023, 144, 1023 },
    { 1023, 128, 1023 },
    { 1023, 112, 1023 },
    { 1023, 96, 1023 },
    { 1023, 80, 1023 },
    { 1023, 64, 1023 },
    { 1023, 48, 1023 },
    { 1023, 32, 1023 },
    { 1023, 16, 1023 },
    { 1023, 0, 1023 },
    { 1023, 512, 512 },
    { 1023, 496, 496 },
    { 1023, 480, 480 },
    { 1023, 464, 464 },
    { 1023, 448, 448 },
    { 1023, 432, 432 },
    { 1023, 416, 416 },
    { 1023, 400, 400 },
    { 1023, 384, 384 },
    { 1023, 368, 368 },
    { 1023, 352, 352 },
    { 1023, 336, 336 },
    { 1023, 320, 320 },
    { 1023, 304, 304 },
    { 1023, 288, 288 },
    { 1023, 272, 272 },
    { 1023, 256, 256 },
    { 1023, 240, 240 },
    { 1023, 224, 224 },
    { 1023, 208, 208 },
    { 1023, 192, 192 },
    { 1023, 176, 176 },
    { 1023, 160, 160 },
    { 1023, 144, 144 },
    { 1023, 128, 128 },
    { 1023, 112, 112 },
    { 1023, 96, 96 },
    { 1023, 80, 80 },
    { 1023, 64, 64 },
    { 1023, 48, 48 },
    { 1023, 32, 32 },
    { 1023, 16, 16 },
    { 1023, 0, 0 },
};

enum colorCodes key_sw_mode_colors[NUM_MUTE_NOTE_KEY_STATES];

void update_pwm_leds(void);

void update_pwm_leds(void)
{
	update_display_at_encoder_press();
	update_led_flash();
	update_button_leds();
	update_encoder_leds();
	update_array_leds();
	update_clockin_led();
	update_audioin_led();
	update_LED_rings();
}

void start_led_display(void)
{
	start_timer_IRQ(LED_UPDATE_TIM_number, &update_pwm_leds);
}

void init_led_cont_ongoing_display(void)
{
	led_cont.ongoing_display	= ONGOING_DISPLAY_NONE;
	led_cont.ongoing_timeout	= 0;
}

void init_led_cont(void)
{
	uint8_t i;

	// BUTTONS
	for (i=0; i<NUM_BUTTONS; i++)
	{
		if (i< NUM_CHANNELS)
			set_rgb_color_brightness(&led_cont.button[i], ledc_WHITE, 1.0/1.7);
		else
			set_rgb_color_brightness(&led_cont.button[i], ledc_PINK, 1.0/2.5);
	}

	// LFOs
	for (i=0; i<NUM_CHANNELS; i++)
		set_rgb_color_brightness(&led_cont.array[i], ledc_WHITE, 1.0/2.0);

	set_rgb_color_brightness(&led_cont.array[GLO_CLK], ledc_WHITE, 1.0/2.0);

	// Flags (LFO)
	for (i=0; i<NUM_CHANNELS; i++)
	{
		led_cont.ongoing_lfoshape[i] = 0;
		led_cont.lfoshape_started[i] = 0;
	}

	key_sw_mode_colors[ksw_MUTE] = ledc_GOLD;
	key_sw_mode_colors[ksw_NOTE] = ledc_PINK;
	key_sw_mode_colors[ksw_KEYS] = ledc_PURPLE;
	key_sw_mode_colors[ksw_KEYS_EXT_TRIG] = ledc_WHITE;
	key_sw_mode_colors[ksw_KEYS_EXT_TRIG_SUSTAIN] = ledc_BUTTERCUP;
}

static uint8_t octave_was_pressed;

// Same octave gradient as the outer ring, independent of its current view.
static void set_octave_color(o_rgb_led *led, uint8_t octave)
{
    led->c_red = 4032 - 4032 * octave / 18;
    led->c_green = 3800 * octave / 18;
    led->c_blue = 0;
}

static void display_octave_rings(uint8_t show_outer_ring);

void update_display_at_encoder_press(void)
{
    const uint8_t octave_pressed = rotary_pressed(rotm_OCT) != RELEASED;
    if (octave_pressed) {
        // Keep Scale visible throughout the press; preserve the root shortcut.
        if ((!octave_was_pressed ||
            led_cont.ongoing_display != ONGOING_DISPLAY_SCALE_ROOT) &&
            led_cont.ongoing_display != ONGOING_DISPLAY_GLIDE)
            start_ongoing_display_scale();
    } else {
        // Give a full normal timeout after release, even after a long hold.
        if (octave_was_pressed && led_cont.ongoing_display == ONGOING_DISPLAY_SCALE)
            start_ongoing_display_scale();
        if (rotary_pressed(rotm_TRANSPOSE) && !smn_polyrotate_feedback.suppress_transpose) {
            if (switch_pressed(FINE_BUTTON)) start_ongoing_display_finetune();
            else start_ongoing_display_transpose();
        }
    }
    octave_was_pressed = octave_pressed;

    if (rotary_pressed(rotm_WAVETABLE))
        start_ongoing_display_model_sel();
}

void update_led_flash(void)
{
	led_cont.flash_state = (HAL_GetTick()/TICKS_PER_MS) & 0x080; //128ms flash period
}

// Model colors are independent of the legacy sphere palette and selector order.
static void set_model_color(uint8_t model_id, o_rgb_led *rgb)
{
    // Slots 1..36: requested sRGB8 decoded to linear light, rounded to 0..1023.
    // x <= 0.04045: x/12.92; otherwise ((x+0.055)/1.055)^2.4, x=RGB8/255.
    // Precomputed colours; calibration and display brightness are applied later.
    // MODEL_RGB8 keeps the unassigned fourth page at its legacy intensities.
#define MODEL_RGB8(r, g, b) {((r) * 1023 + 127) / 255, \
                             ((g) * 1023 + 127) / 255, \
                             ((b) * 1023 + 127) / 255}
    // All 48 display slots, including unassigned (inactive) future models.
    static const uint16_t colors[SMN_MODEL_SLOT_COUNT][3] = {
        [0] = {809, 695, 203}, // Page 1, LED 2; sRGB8 230, 215, 123
        [1] = {917, 674, 76}, // Page 1, LED 3; sRGB8 243, 212, 77
        [2] = {934, 584, 15}, // Page 1, LED 5; sRGB8 245, 199, 32
        [3] = {908, 406, 13}, // Page 1, LED 6; sRGB8 242, 169, 30
        [4] = {850, 256, 12}, // Page 1, LED 8; sRGB8 235, 137, 28
        [5] = {866, 142, 14}, // Page 1, LED 9; sRGB8 237, 104, 31
        [6] = {802, 76, 20}, // Page 1, LED 11; sRGB8 229, 77, 38
        [7] = {710, 38, 29}, // Page 1, LED 12; sRGB8 217, 54, 47
        [8] = {558, 18, 48}, // Page 1, LED 14; sRGB8 195, 36, 61
        [9] = {411, 18, 57}, // Page 1, LED 15; sRGB8 170, 36, 67
        [10] = {248, 18, 54}, // Page 1, LED 17; sRGB8 135, 36, 65
        [11] = {128, 17, 46}, // Page 1, LED 18; sRGB8 99, 35, 60
        [12] = {17, 45, 277}, // Page 2, LED 2; sRGB8 35, 59, 142
        [13] = {18, 97, 565}, // Page 2, LED 3; sRGB8 36, 87, 196
        [14] = {22, 217, 770}, // Page 2, LED 5; sRGB8 40, 127, 225
        [15] = {70, 411, 891}, // Page 2, LED 6; sRGB8 74, 170, 240
        [16] = {68, 571, 794}, // Page 2, LED 8; sRGB8 73, 197, 228
        [17] = {8, 645, 645}, // Page 2, LED 9; sRGB8 22, 208, 208
        [18] = {7, 508, 401}, // Page 2, LED 11; sRGB8 20, 187, 168
        [19] = {33, 702, 331}, // Page 2, LED 12; sRGB8 50, 216, 154
        [20] = {105, 702, 189}, // Page 2, LED 14; sRGB8 90, 216, 119
        [21] = {22, 496, 120}, // Page 2, LED 15; sRGB8 40, 185, 96
        [22] = {8, 244, 80}, // Page 2, LED 17; sRGB8 22, 134, 79
        [23] = {8, 105, 51}, // Page 2, LED 18; sRGB8 22, 90, 63
        [24] = {866, 20, 213}, // Page 3, LED 2; sRGB8 237, 38, 126
        [25] = {917, 74, 340}, // Page 3, LED 3; sRGB8 243, 76, 156
        [26] = {866, 156, 484}, // Page 3, LED 5; sRGB8 237, 109, 183
        [27] = {688, 117, 546}, // Page 3, LED 6; sRGB8 214, 95, 193
        [28] = {552, 185, 702}, // Page 3, LED 8; sRGB8 194, 118, 216
        [29] = {427, 236, 809}, // Page 3, LED 9; sRGB8 173, 132, 230
        [30] = {317, 189, 817}, // Page 3, LED 11; sRGB8 151, 119, 231
        [31] = {240, 112, 717}, // Page 3, LED 12; sRGB8 133, 93, 218
        [32] = {182, 66, 591}, // Page 3, LED 14; sRGB8 117, 72, 200
        [33] = {133, 36, 444}, // Page 3, LED 15; sRGB8 101, 53, 176
        [34] = {86, 25, 290}, // Page 3, LED 17; sRGB8 82, 43, 145
        [35] = {40, 16, 139}, // Page 3, LED 18; sRGB8 56, 34, 103
        [36] = MODEL_RGB8(182, 213, 28), // Page 4, LED 2
        [37] = MODEL_RGB8(178, 204, 60), // Page 4, LED 3
        [38] = MODEL_RGB8(131, 197, 28), // Page 4, LED 5
        [39] = MODEL_RGB8(144, 200, 62), // Page 4, LED 6
        [40] = MODEL_RGB8(70, 209, 38), // Page 4, LED 8
        [41] = MODEL_RGB8(102, 207, 72), // Page 4, LED 9
        [42] = MODEL_RGB8(101, 169, 40), // Page 4, LED 11
        [43] = MODEL_RGB8(121, 179, 71), // Page 4, LED 12
        [44] = MODEL_RGB8(140, 142, 37), // Page 4, LED 14
        [45] = MODEL_RGB8(150, 151, 72), // Page 4, LED 15
        [46] = MODEL_RGB8(91, 113, 48), // Page 4, LED 17
        [47] = MODEL_RGB8(113, 130, 69), // Page 4, LED 18
    };
#undef MODEL_RGB8
    const uint16_t *color = colors[smn_model_color(model_id)];
    // Keep brightness owned by the current display (locks and flashing).
    rgb->c_red = color[0];
    rgb->c_green = color[1];
    rgb->c_blue = color[2];
}

void update_button_leds(void){

	uint8_t 					i;
	uint8_t 					color;
	enum VoctCalStates 			voct_state;

	int32_t 					tri_period, tri_phase;
	float 						tri_wave, brightness, lock_brightness;

	const uint32_t ticks = HAL_GetTick();
	uint32_t now = ticks/TICKS_PER_MS;
	static uint32_t				animation_phase = 0;
	static enum ongoingDisplays	display_cache = ONGOING_DISPLAY_NONE;
	int16_t oct;
    const uint8_t chords_green = chord_buttons_green();
    const int8_t saved_chord = chord_saved_feedback(ticks);
    const int8_t cv_chord = smn_chord_cv_feedback(ticks);
    // Only these pages own channel-button colors. Other pages must recompute
    // normal mode/lock/envelope colors, including after a temporary overlay.
    const uint8_t parameter_colors =
        (led_cont.ongoing_display >= ONGOING_DISPLAY_FINETUNE &&
         led_cont.ongoing_display <= ONGOING_DISPLAY_LFO_TOVCA &&
         led_cont.ongoing_display != ONGOING_DISPLAY_SCALE_ROOT) ||
        led_cont.ongoing_display == ONGOING_DISPLAY_MODEL_SEL ||
        led_cont.ongoing_display == ONGOING_DISPLAY_GLIDE;

	for (i = 0; i < NUM_BUTTONS; i++){

		if (ui_mode == PLAY){

			if (i<NUM_CHANNELS){

				if (i == saved_chord) {
                    set_rgb_color(&led_cont.button[i], ledc_FULLRED);
                } else if (chords_green || i == cv_chord) {
                    static const uint16_t green[3] = {0, 4095, 0};
                    set_rgb_color_by_array(&led_cont.button[i], green, F_MAX_BRIGHTNESS);
                } else if (led_cont.ongoing_display != ONGOING_DISPLAY_PRESET &&
                    smn_polyrotate_feedback.animation &&
                    (uint32_t)(ticks - smn_polyrotate_feedback.animation_start) < 500 * TICKS_PER_MS) {
                    const unsigned step = (uint32_t)(ticks - smn_polyrotate_feedback.animation_start) * 7 /
                        (500 * TICKS_PER_MS);
                    const uint8_t lit = smn_polyrotate_feedback.animation == 1 ?
                        step == (unsigned)i + 1 : (unsigned)i < 6 - (step > 6 ? 6 : step);
                    set_rgb_color(&led_cont.button[i], lit ? ledc_WHITE : ledc_OFF);
                } else if (led_cont.ongoing_display != ONGOING_DISPLAY_PRESET &&
                    smn_polyrotate_enabled(&params) &&
                    (smn_polyrotate_feedback.flash_mask & (1u << i)) &&
                    (uint32_t)(ticks - smn_polyrotate_feedback.flash_start[i]) < 150 * TICKS_PER_MS) {
                    set_rgb_color(&led_cont.button[i], ledc_WHITE);
                } else if (parameter_colors) {

					if (params.osc_param_lock[i] && lock_flash_state())
						lock_brightness = 0;
					else
						lock_brightness = F_MAX_BRIGHTNESS;

					if (params.key_sw[i]==ksw_MUTE && led_cont.ongoing_display != ONGOING_DISPLAY_GLIDE) {
						if (!params.note_on[i])
							lock_brightness = F_MAX_BRIGHTNESS - lock_brightness;
					}

					if (led_cont.ongoing_display == ONGOING_DISPLAY_TRANSPOSE ||
                        led_cont.ongoing_display == ONGOING_DISPLAY_GLIDE) {
						set_rgb_color_by_array(&led_cont.button[i], CH_COLOR_MAP[i], lock_brightness);
					}

					else if( (led_cont.ongoing_display == ONGOING_DISPLAY_MODEL_SEL) ){
						led_cont.button[i].brightness = lock_brightness*4.0;
						set_model_color(smn_model_id_at(calc_params.wtsel[i]), &led_cont.button[i]);
					}

					else if( (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE) ){
						set_rgb_color_by_array(&led_cont.button[i], CH_COLOR_MAP[i], lock_brightness);
					}

					else if (led_cont.ongoing_display == ONGOING_DISPLAY_OCTAVE ||
                             led_cont.ongoing_display == ONGOING_DISPLAY_SCALE){
						oct = _CLAMP_I16(params.oct[i], MIN_OCT , MAX_OCT) - MIN_OCT;
						oct = _CLAMP_I16(oct, 0, NUM_LED_OUTRING-1);
                        set_octave_color(&led_cont.button[i], oct);
						led_cont.button[i].brightness  	= F_MAX_BRIGHTNESS * lock_brightness;
					}

					else if ( led_cont.ongoing_display == ONGOING_DISPLAY_LFO_TOVCA){
						if (lfos.to_vca[i]) {
							if ( display_cache != ONGOING_DISPLAY_LFO_TOVCA ){
								animation_phase = 0xFFFFFFFF - now + 1; //starting phase: results in tri_phase starting at 0
								display_cache = ONGOING_DISPLAY_LFO_TOVCA;
							}
							tri_period = LFO_TOVCA_FLASH_PERIOD;
							tri_phase = (now + animation_phase) % (tri_period*2);
							tri_wave = _FOLD_F(tri_phase, tri_period);
							brightness = (float)(tri_wave/tri_period);
							set_rgb_color_brightness(&led_cont.button[i], ledc_LIGHT_GREEN, brightness);
						}
						else
							set_rgb_color(&led_cont.button[i], ledc_DIM_YELLOW);
					}

					else if ( led_cont.ongoing_display == ONGOING_DISPLAY_LFO_MODE){
						if (params.key_sw[i] == ksw_MUTE) {
							if (display_cache != ONGOING_DISPLAY_LFO_MODE){
								display_cache =  ONGOING_DISPLAY_LFO_MODE;
								animation_phase = 0xFFFFFFFF - now + 1; //starting phase: results in tri_phase starting at 0
							}

							tri_period = LFO_MODE_FLASH_PERIOD;
							tri_phase = (now + animation_phase + ((tri_period*2*(NUM_CHANNELS-i))/NUM_CHANNELS)) % (tri_period*2);

							if (lfos.mode[i]==lfot_TRIG){
								brightness = (tri_phase < (tri_period/NUM_CHANNELS)) ? 1.0 : 0.05;
								color = ledc_AQUA;
							}
							else if (lfos.mode[i]==lfot_GATE){
								brightness = (tri_phase > tri_period) ? 1.0 : 0.0;
								color = ledc_LIGHT_BLUE;
							}
							else{ //lfot_SHAPE
								tri_wave = _FOLD_F(tri_phase, tri_period);
								brightness = (float)(tri_wave/tri_period);
								color = ledc_BLUE;
							}
							if ((animation_phase + now) < 250) brightness = 0;

						} else {
							brightness = 0.2;
							color = ledc_BLUE;
						}
						set_rgb_color_brightness(&led_cont.button[i], color, brightness);
					}
				}

				else { //no ongoing_display

					if (params.key_sw[i] == ksw_MUTE)
					{
						if (params.osc_param_lock[i] && lock_flash_state())
							brightness = 0;
						else
							brightness = F_MAX_BRIGHTNESS;

						if (!params.note_on[i])
							brightness = F_MAX_BRIGHTNESS - brightness;
					}
					else{

						if (params.osc_param_lock[i] && lock_flash_state())
							brightness = 0;
						else
							brightness = 0.30 + lfos.out_lpf[i]/2.0;
					}

					set_rgb_color_brightness(&led_cont.button[i], key_sw_mode_colors[params.key_sw[i]], brightness);
				}
			}

			//LFO buttons:
			else
			{
					if (!calc_params.keymode_pressed)
				{
					//No channels are in mute mode ==> dim purple
					if ((params.key_sw[0] != ksw_MUTE) &&
						(params.key_sw[1] != ksw_MUTE) &&
						(params.key_sw[2] != ksw_MUTE) &&
						(params.key_sw[3] != ksw_MUTE) &&
						(params.key_sw[4] != ksw_MUTE) &&
						(params.key_sw[5] != ksw_MUTE) )					set_rgb_color_brightness(&led_cont.button[i], ledc_PURPLE, 0.1);
					else if (!button_pressed(i))							set_rgb_color(&led_cont.button[i], ledc_PINK);
					else if (calc_params.button_safe_release[i - NUM_CHANNELS])	set_rgb_color(&led_cont.button[i], ledc_CORAL);
					else													set_rgb_color(&led_cont.button[i], ledc_MED_BLUE);
				}
				else
				{
					if(calc_params.button_safe_release[i - NUM_CHANNELS])		set_rgb_color(&led_cont.button[i], ledc_CORAL);
					else													set_rgb_color(&led_cont.button[i], ledc_PURPLE);
				}
			}

		} //if (ui_mode==PLAY)

		else if (ui_mode==VOCT_CALIBRATE)
		{
			color = ledc_OFF;

			//Set the color of the channel buttons to the state of the Voct Calibration
			//(the Transpose calibration state is displayed on the led array)
			if (i<NUM_CHANNELS)
			{
				voct_state = get_voctcal_state(i);
				if (voct_state == VOCTCAL_READING_C1)		color = ledc_BLUE;
				if (voct_state == VOCTCAL_READING_C3)		color = ledc_RED;
				if (voct_state == VOCTCAL_CALIBRATED)		color = ledc_WHITE;
			}
			set_rgb_color(&led_cont.button[i], color);
		}
		set_pwm_led(led_button_map[i], &led_cont.button[i]);
	}
}

void update_encoder_leds(void){

	uint8_t i, color;

	if (led_cont.ongoing_display == ONGOING_DISPLAY_TRANSPOSE)
		color = ledc_FUSHIA;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE)
		color = ledc_MED_BLUE;

	else
		color = ledc_GOLD;

	for (i = ledrotm_DEPTH; i < NUM_LED_ROTARIES; i++)
	{
		if (color == ledc_GOLD) {
			// Interpolate the three colours at the LED refresh rate, outside audio.
			// Dim white by 25% because its three active channels look brighter.
			const float dose = smn_dispersion_display_value(i - ledrotm_DEPTH);
			const uint8_t lower = dose < 0.5f;
			const uint8_t from = lower ? ledc_WHITE : ledc_GOLD;
			const uint8_t to = lower ? ledc_GOLD : ledc_RED;
			const float mix = lower ? dose * 2.0f : dose * 2.0f - 1.0f;
			const float from_gain = lower ? 0.75f : 1.0f;
			uint16_t rgb[3];
			for (uint8_t c = 0; c < 3; ++c) {
				const float start = colorPalette[c][from] * from_gain;
				rgb[c] = (uint16_t)(start + mix * (colorPalette[c][to] - start));
			}
			set_rgb_color_by_array(&led_cont.encoder[i], rgb, 1.0f);
		}
		else
			set_rgb_color(&led_cont.encoder[i], color);

		set_pwm_led(led_rotary_map[i], &led_cont.encoder[i]);
	}
}

void update_mono_leds(void){
	uint8_t i;
	static uint32_t slider_pwm=0;
	float exp;

	// SLIDERS

	if (ui_mode==RGB_COLOR_ADJUST)
	{
		mono_led_off(mledm_SLIDER_A);
		mono_led_off(mledm_SLIDER_B);
		mono_led_off(mledm_SLIDER_C);
		mono_led_off(mledm_SLIDER_D);
		mono_led_off(mledm_SLIDER_E);
		mono_led_off(mledm_SLIDER_F);
	} else {

		if (slider_pwm-- == 0)
		{
			slider_pwm=32;		//32 steps of brightness

			for (i=0;i<NUM_CHANNELS;i++){
				mono_led_off(i);
			}
		}
		else{
			for (i=0;i<NUM_CHANNELS;i++) {
				uint32_t level = (uint32_t)calc_params.level[i];

				if (calc_params.adjusting_pan_state[i] == pan_CACHED_LEVEL && cached_param_flash_state())
					level = level < 3000 ? 4095 : 0;

				exp = exp_1voct_10_41V[level] * system_settings.global_brightness;
				if ((level > 50) && (exp > (slider_pwm*43))){
					mono_led_on(i);
				}
			}
		}
	}
}

void update_array_leds(void)
{
	uint8_t 	i;
	uint8_t		color;
	enum VoctCalStates voct_state;

	if (ui_mode != VOCT_CALIBRATE)
	{
		calculate_lfo_leds();
		calculate_lfocv_led();

	}
	else if (ui_mode == VOCT_CALIBRATE)
	{
		color = ledc_OFF;

		//Get voct calibration state of Transpose jack
		voct_state = get_voctcal_state(NUM_VOCT_CHANNELS-1);
		if (voct_state == VOCTCAL_READING_C1)		color = ledc_BLUE;
		if (voct_state == VOCTCAL_READING_C3)		color = ledc_RED;
		if (voct_state == VOCTCAL_CALIBRATED)		color = ledc_WHITE;

		//Set all array LEDs to the same color
		for (i = 0; i < NUM_CHANNELS+1; i++)
			set_rgb_color(&led_cont.array[i], color);
	}

	for (i=0; i<NUM_CHANNELS +1; i++)
	{
		set_pwm_led(ledstring_map[i], &led_cont.array[i]);
	}
}

void update_clockin_led(void)
{
	if (jack_plugged(CLK_SENSE)) {
		if (led_cont.waiting_for_clockin)
			set_single_pwm_led(singleledm_CLKIN, 0);
		if (lfos.use_ext_clock && lfos.cycle_pos[REF_CLK] < 0.5)
			set_single_pwm_led(singleledm_CLKIN, 1000);
		else
			set_single_pwm_led(singleledm_CLKIN, 0);
	}
	else if (system_settings.allow_bus_clock) {
		if (BUS_CLK())	set_single_pwm_led(singleledm_CLKIN, 100);
		else			set_single_pwm_led(singleledm_CLKIN, 0);
	}
	else
		set_single_pwm_led(singleledm_CLKIN, 0);
}

void update_audioin_led(void)
{
	// Recording/monitoring is retired; the input still serves trigger detection.
	o_rgb_led rgb;
	set_rgb_color(&rgb, ledc_OFF);
	set_pwm_led(ledm_AUDIOIN, &rgb);
}

void calculate_lfocv_led(void)
{
	if ( 	(params.key_sw[0] == ksw_MUTE) ||
			(params.key_sw[1] == ksw_MUTE) ||
			(params.key_sw[2] == ksw_MUTE) ||
			(params.key_sw[3] == ksw_MUTE) ||
			(params.key_sw[4] == ksw_MUTE) ||
			(params.key_sw[5] == ksw_MUTE)	){

		if (led_cont.waiting_for_clockin){
			led_cont.array[GLO_CLK].c_red 		= 1023 * (1.0 - led_cont.clockin_wait_progress);
			led_cont.array[GLO_CLK].c_green 	= 1023 * led_cont.clockin_wait_progress;
			led_cont.array[GLO_CLK].c_blue 		= 200;
		}
		else if (lfos.period[GLO_CLK] > F_LFO_AUDIO_RANGE_PERIOD_L){
			set_rgb_color(&led_cont.array[GLO_CLK], ledc_WHITE);
		}
		else{
			set_rgb_color(&led_cont.array[GLO_CLK], ledc_PURPLE);
		}
	}
	else{
		set_rgb_color(&led_cont.array[GLO_CLK], ledc_CORAL);
	}

	if (lfos.cycle_pos[GLO_CLK] < 0.5)
		led_cont.array[GLO_CLK].brightness = F_MAX_BRIGHTNESS;
	else
		led_cont.array[GLO_CLK].brightness = 0;
}

void calculate_lfo_leds(void)
{
	uint8_t chan=0;
	float brightness;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
	{
		// Mute always keeps its normal level display; discard any prior feedback.
		if (led_cont.ongoing_lfoshape[chan]) {
			// Read the event time before the clock: TIM9 can select a shape
			// during this lower-priority LED refresh.
			const uint32_t started = led_cont.lfoshape_started[chan];
			if (params.key_sw[chan] == ksw_MUTE ||
				(uint32_t)(HAL_GetTick() - started) >= LFOSHAPE_DISPLAY_MS * TICKS_PER_MS)
				led_cont.ongoing_lfoshape[chan] = 0;
		}

		if (led_cont.ongoing_lfoshape[chan])
			brightness = 1.0f; // Visible even when the envelope or gain is zero.
		else if (params.key_sw[chan] == ksw_MUTE && lfos.audio_mode[chan])
			brightness = lfos.gain[chan];
		else
			brightness = lfos.out_lpf[chan];

		set_rgb_color_by_array(&led_cont.array[chan], LFO_BANK_COLOR[lfos.shape[chan]], brightness);
	}
}

void update_LED_rings(void)
{
	uint8_t i;

	calculate_led_ring();

	for (i = 0; i < NUM_LED_OUTRING; i++){
		set_pwm_led(led_outring_map[i], &led_cont.outring[i]);
	}

	for (i = 0; i < NUM_LED_INRING; i++){
		set_pwm_led(led_inring_map[i], &led_cont.inring[i]);
	}
}

void calculate_led_ring(void){
	uint8_t i;

	update_ongoing_display_timers();

	if (ui_mode==VOCT_CALIBRATE) {
		turn_outring_off();

		for (i = 0; i < NUM_LED_INRING; i++){
			set_rgb_color(&led_cont.inring[i], ledc_OFF);
		}
	}
	else {
		switch (led_cont.ongoing_display)
		{
			case ONGOING_DISPLAY_TRANSPOSE:
				display_transpose();
				break;

            case ONGOING_DISPLAY_SCALE:
                display_scale();
                break;

            case ONGOING_DISPLAY_SCALE_ROOT:
                display_scale_root();
                break;

			case ONGOING_DISPLAY_FINETUNE:
				display_finetune();
				break;

			case ONGOING_DISPLAY_OCTAVE:
				display_octave();
				break;

			case ONGOING_DISPLAY_PRESET:
				display_preset();
				break;

			case ONGOING_DISPLAY_MODEL_SEL:
				display_model_sel();
				break;

            case ONGOING_DISPLAY_GLIDE:
                display_glide();
                break;

			default:
				display_wt_pos();
				flash_wt_lock();
				break;
		}
	}
}

void turn_outring_off(void)
{
	uint8_t i;

	for (i =0; i< NUM_LED_OUTRING; i++){
		led_cont.outring[i].brightness 	= 0;
	}
}

void flash_wt_lock(void)
{
	uint8_t chan, led;

	for ( chan = 0; chan < NUM_CHANNELS; chan++)
	{
		led = rotate_origin(chan, NUM_CHANNELS);
		if (params.osc_param_lock[chan] && lock_flash_state())
		{
			led_cont.outring[led * 3    ].brightness = 0;
			led_cont.outring[led * 3 + 1].brightness = 0;
			led_cont.outring[led * 3 + 2].brightness = 0;
			led_cont.inring[led].brightness = 0;
		}
	}
}

// 14 fractional bits: exact endpoints, at most one RGB unit of rounding.
// Pack the two nonnegative weights (0..16384) for the M7's dual multiply.
static inline uint32_t htm_color_weights(float value)
{
	const uint32_t level = (uint32_t)(value * 32768.0f);
#ifndef TEST
	const uint32_t white = __USAT((int32_t)level - 16384, 15);
#else
	const uint32_t white = level > 16384u ? level - 16384u : 0u;
#endif
	const uint32_t model = level - 2u * white;
	return model | (white << 16);
}

static inline uint32_t htm_color_component(uint32_t weights, uint32_t colors)
{
#ifndef TEST
	// Positive signed halfwords; sum <= 16384 * 1023, so no overflow.
	return __SMUAD(weights, colors) >> 14;
#else
	// Same arithmetic for the host, which has no ARM DSP instructions.
	return ((weights & 65535u) * (colors & 65535u)
		+ (weights >> 16) * (colors >> 16)) >> 14;
#endif
}

void display_wt_pos(void)
{
	for (uint8_t chan = 0; chan < NUM_CHANNELS; ++chan)
	{
		const SmnTimbreControls controls = smn_controls_get_current(chan);
		const float values[3] = {controls.harmonics, controls.timbre, controls.morph};
		o_rgb_led model_color = {0};
		set_model_color(smn_model_id_at(calc_params.wtsel[chan]), &model_color);

		// RGB components are 0..1023; pack model/white once per voice.
		const uint32_t red = model_color.c_red | (colorPalette[c_RED][ledc_WHITE] << 16);
		const uint32_t green = model_color.c_green | (colorPalette[c_GREEN][ledc_WHITE] << 16);
		const uint32_t blue = model_color.c_blue | (colorPalette[c_BLUE][ledc_WHITE] << 16);

		for (uint8_t dim = 0; dim < 3; ++dim)
		{
			// User LEDs 1..18: bottom-left, clockwise; A-H/T/M, B-H/T/M, ...
			const uint8_t led = rotate_origin(chan * 3 + dim, NUM_LED_OUTRING);
			const float value = values[dim]; // Already folded to 0..1, after ramp/CV.
			// 0 -> off, 0.5 -> model, 1 -> white; no branch or division on ARM.
			const uint32_t weights = htm_color_weights(value);
			led_cont.outring[led].c_red = htm_color_component(weights, red);
			led_cont.outring[led].c_green = htm_color_component(weights, green);
			led_cont.outring[led].c_blue = htm_color_component(weights, blue);
			led_cont.outring[led].brightness = smn_parameter_locked(&params, chan, dim) && lock_flash_state() ? 0.0f : F_MAX_BRIGHTNESS;
		}

		// Inner ring keeps its existing model color and brightness.
		const uint8_t inner = rotate_origin(chan, NUM_CHANNELS);
		led_cont.inring[inner].brightness = F_MAX_BRIGHTNESS;
		led_cont.inring[inner].c_red = model_color.c_red;
		led_cont.inring[inner].c_green = model_color.c_green;
		led_cont.inring[inner].c_blue = model_color.c_blue;
	}
}

void display_firmware_version(void)
{
	uint8_t i, j;

	set_rgb_color(&led_cont.encoder[ledrotm_DEPTH], ledc_OFF);
	set_rgb_color(&led_cont.encoder[ledrotm_LATITUDE], ledc_OFF);
	set_rgb_color(&led_cont.encoder[ledrotm_LONGITUDE], ledc_OFF);

	set_pwm_led(led_rotary_map[ledrotm_DEPTH], &led_cont.encoder[ledrotm_DEPTH]);
	set_pwm_led(led_rotary_map[ledrotm_LATITUDE], &led_cont.encoder[ledrotm_LATITUDE]);
	set_pwm_led(led_rotary_map[ledrotm_LONGITUDE], &led_cont.encoder[ledrotm_LONGITUDE]);

	for (i=0; i<NUM_BUTTONS; i++) {
		set_rgb_color(&led_cont.button[i], ledc_OFF);
		set_pwm_led(led_button_map[i], &led_cont.button[i]);
	}

	for (i=0; i<NUM_LED_ARRAY; i++) {
		set_rgb_color(&led_cont.array[i], ledc_OFF);
		set_pwm_led(ledstring_map[i], &led_cont.array[i]);
	}

	for (i =0; i< NUM_LED_OUTRING; i++)
		set_rgb_color(&led_cont.outring[i], ledc_OFF);

	for (i=0; i<NUM_CHANNELS; i++)
		set_rgb_color(&led_cont.inring[i], ledc_OFF);

	if (SMN_FW_MAJOR_VERSION > 0 && SMN_FW_MAJOR_VERSION <= NUM_CHANNELS) {
		j = SMN_FW_MAJOR_VERSION - 1;
		set_rgb_color(&led_cont.inring[j], ledc_RED);
	}

	j = SMN_FW_MINOR_VERSION;
	if (j < NUM_LED_OUTRING)
		set_rgb_color(&led_cont.outring[j], ledc_GOLD);

	for (i =0; i< NUM_LED_OUTRING; i++)
		set_pwm_led(led_outring_map[i], &led_cont.outring[i]);

	for (i=0; i<NUM_CHANNELS; i++)
		set_pwm_led(led_inring_map[i], &led_cont.inring[i]);

}

void display_transpose(void)
{
	uint8_t i, j, chan;
	int32_t t_transpose;
	int8_t transpose_pos[NUM_CHANNELS];
	uint32_t num_wraps;

	uint8_t overlap[NUM_LED_OUTRING][NUM_CHANNELS];
	uint8_t overlap_num[NUM_LED_OUTRING];
	static uint8_t overlap_ctr[NUM_LED_OUTRING]={0};

	uint8_t do_advance_overlap = 0;
	static uint32_t last_advance_overlap_tmr=0;
	uint32_t now = HAL_GetTick()/TICKS_PER_MS;

	if ((now - last_advance_overlap_tmr) > 300) {
		do_advance_overlap = 1;
		last_advance_overlap_tmr = now;
	}

	for (i = 0; i < NUM_LED_OUTRING; i++)
	{
		led_cont.outring[i].brightness = 0;
		overlap_num[i] = 0;
		for (chan=0; chan<NUM_CHANNELS; chan++)
			overlap[i][chan] = 99;
	}

	// Create overlap[led_position][channels_occupying_position] = channel#
	for (i = 0; i < NUM_CHANNELS; i++)
	{
		t_transpose = _CLAMP_I16(calc_params.transpose[i], MIN_TRANSPOSE_WRAP, MAX_TRANSPOSE_WRAP);
		transpose_pos[i] = (t_transpose - MIN_TRANSPOSE_WRAP)  % NUM_LED_OUTRING;

		j = rotate_origin(i, NUM_CHANNELS);

		if (params.osc_param_lock[i] && lock_flash_state() ){
			set_rgb_color(&led_cont.inring[j], ledc_OFF);
		} else {
			num_wraps = (uint32_t)((float)(t_transpose - transpose_pos[i] - MIN_TRANSPOSE_WRAP) / (float)(MAX_TRANSPOSE_WRAP - MIN_TRANSPOSE_WRAP) * 4096.0);
			set_rgb_color_by_array(&led_cont.inring[j], CH_COLOR_MAP[i], ((exp_1voct_10_41V[num_wraps] / 500.0) + 0.017) / F_MAX_BRIGHTNESS);
		}

		overlap[transpose_pos[i]][ overlap_num[transpose_pos[i]] ] = i;
		overlap_num[transpose_pos[i]]++;
	}

	for (i = 0; i<NUM_LED_OUTRING; i++)
	{
		if (do_advance_overlap) {
			overlap_ctr[i]++;
			if (overlap_ctr[i] >= overlap_num[i]) overlap_ctr[i]=0;
		}

		if (overlap_ctr[i] >= overlap_num[i]) overlap_ctr[i] = 0;
		chan = overlap[i][overlap_ctr[i]];
		if (chan < NUM_CHANNELS)
		{
			if (((overlap_num[i] == 1 && params.osc_param_lock[chan]) ||
                 smn_parameter_locked(&params, chan, SMN_LOCK_PITCH)) && lock_flash_state())
				set_rgb_color(&led_cont.outring[i], ledc_OFF);
			else
				set_rgb_color_by_array(&led_cont.outring[i], CH_COLOR_MAP[chan], 1.0);
		}

	}
}

void display_glide(void)
{
    // Integer fifths encode each voice's two neighboring LED contributions.
    uint8_t weights[NUM_LED_OUTRING][NUM_CHANNELS] = {{0}};
    static uint8_t overlap_ctr[NUM_LED_OUTRING];
    static uint32_t last_advance;
    const uint32_t now = HAL_GetTick() / TICKS_PER_MS;
    const uint8_t advance = (uint32_t)(now - last_advance) >= 300;
    if (advance) last_advance = now;
    turn_outring_off();
    for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
        const uint8_t index = smn_glide_time_index(params.smn_extra.glide_ms[v]);
        const uint8_t whole = index / SMN_GLIDE_FINE_DIVISOR;
        const uint8_t fine = index % SMN_GLIDE_FINE_DIVISOR;
        if (whole) weights[whole - 1][v] = SMN_GLIDE_FINE_DIVISOR - fine;
        if (fine) weights[whole][v] = fine;
        set_rgb_color_by_array(&led_cont.inring[rotate_origin(v, NUM_CHANNELS)],
            CH_COLOR_MAP[v], params.osc_param_lock[v] && lock_flash_state() ? 0.0f : 1.0f);
    }
    for (unsigned led = 0; led < NUM_LED_OUTRING; ++led) {
        uint8_t voices[NUM_CHANNELS], count = 0;
        for (unsigned v = 0; v < NUM_CHANNELS; ++v)
            if (weights[led][v]) voices[count++] = v;
        if (!count) { overlap_ctr[led] = 0; continue; }
        if (advance) ++overlap_ctr[led];
        if (overlap_ctr[led] >= count) overlap_ctr[led] = 0;
        const uint8_t voice = voices[overlap_ctr[led]];
        set_rgb_color_by_array(&led_cont.outring[rotate_origin(led, NUM_LED_OUTRING)],
            CH_COLOR_MAP[voice], weights[led][voice] * (1.0f / SMN_GLIDE_FINE_DIVISOR));
    }
}

void display_scale(void)
{
    static const uint8_t colors[NUM_LED_OUTRING] = {
        ledc_WHITE, ledc_WHITE, ledc_WHITE,
        ledc_GOLD, ledc_GOLD, ledc_GOLD,
        ledc_DEEP_BLUE, ledc_DEEP_BLUE, ledc_DEEP_BLUE,
        ledc_PINK, ledc_PINK, ledc_PINK,
        ledc_GREEN, ledc_GREEN, ledc_GREEN,
        ledc_PURPLE, ledc_PURPLE, ledc_PURPLE
    };
    const uint8_t position = scale_position(params.indiv_scale[0]);
    turn_outring_off();
    for (uint8_t i = 0; i < NUM_LED_INRING; ++i)
        set_rgb_color(&led_cont.inring[i], ledc_OFF);
    if (position && position <= NUM_LED_INRING)
        set_rgb_color(&led_cont.inring[rotate_origin(position - 1, NUM_LED_INRING)], ledc_WHITE);
    else if (position > NUM_LED_INRING) {
        const uint8_t outer = position - NUM_LED_INRING - 1;
        set_rgb_color(&led_cont.outring[rotate_origin(outer, NUM_LED_OUTRING)], colors[outer]);
    }
}

void display_scale_root(void)
{
    turn_outring_off();
    for (uint8_t i = 0; i < NUM_LED_INRING; ++i)
        set_rgb_color(&led_cont.inring[i], ledc_OFF);
    // Same semitone positions as Transpose at 0..11: C at transpose zero.
    const uint8_t root = scale_root(&params);
    const uint8_t position = (root - MIN_TRANSPOSE_WRAP) % NUM_LED_OUTRING;
    const uint8_t sharp = root == 1 || root == 3 || root == 6 || root == 8 || root == 10;
    set_rgb_color(&led_cont.outring[position], sharp ? ledc_GOLD : ledc_WHITE);
}

void display_finetune (void)
{
	uint8_t i, j;
	int16_t t_finetune;
	uint32_t triangle, saw;
	uint32_t tm;
	uint32_t period;
	uint8_t	detune_pos_i;
	float brightness;

	tm = HAL_GetTick()/TICKS_PER_MS;

	for (i = 0; i < NUM_LED_OUTRING; i++)
		set_rgb_color(&led_cont.outring[i], ledc_OFF);

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		//Inner ring: shows channel color (solid, full brightness = unlocked; flashing=locked)
		if (params.osc_param_lock[i] && lock_flash_state())
			brightness 	= 0.0;
		else
			brightness 	= F_MAX_BRIGHTNESS;

		j = rotate_origin(i, NUM_CHANNELS);
		set_rgb_color_by_array(&led_cont.inring[j], CH_COLOR_MAP[i], brightness);

		t_finetune = _CLAMP_I16(params.finetune[i], MIN_FINETUNE_WRAP, MAX_FINETUNE_WRAP);

		if (t_finetune==0) {
			detune_pos_i = j*3+1;
			set_rgb_color(&led_cont.outring[detune_pos_i], ledc_WHITE);
		}
		else {
			if (t_finetune<0) {
				period = _CLAMP_I16(1000+(t_finetune*8), 50, 1000);
				detune_pos_i = j*3;
				set_rgb_color(&led_cont.outring[detune_pos_i], ledc_BLUE);
			}
			else {
				period = _CLAMP_I16(1000-(t_finetune*8), 50, 1000);
				detune_pos_i = j*3+2;
				set_rgb_color(&led_cont.outring[detune_pos_i], ledc_RED);
			}

			//Calculate triangle wave to make LED fade faster as it gets more detuned
			saw = ((float)(tm % period)/(float)period) * 8191.0;
			triangle = (saw>4095) ? (8191-saw) : saw;

			led_cont.outring[detune_pos_i].brightness = (exp_1voct_10_41V[triangle] / 1367.0) + 0.03;
		}
	}
}

void display_preset(void)
{
	uint8_t slot_i, bank_i, led;

	uint16_t hover_bank, preset_i;
	uint8_t slot_color = ledc_OFF;

	hover_bank = preset_mgr.hover_num / NUM_LED_OUTRING;

	for (slot_i = 0; slot_i < NUM_LED_OUTRING; slot_i++)
	{
		led = rotate_origin(slot_i, NUM_LED_OUTRING);
		preset_i = slot_i + (hover_bank*NUM_LED_OUTRING);
		slot_color = animate_preset_ledring(slot_i, preset_i);

		set_rgb_color(&led_cont.outring[led], slot_color);
	}

	for ( bank_i = 0; bank_i < NUM_CHANNELS; bank_i++)
	{
		led = rotate_origin(bank_i, NUM_CHANNELS);
		if (bank_i==hover_bank)	slot_color = ledc_WHITE;
		else					slot_color = ledc_OFF;

		set_rgb_color(&led_cont.inring[led], slot_color);
	}
}

void display_model_sel(void)
{
	uint8_t i, led, chan;

	uint8_t overlap[NUM_LED_OUTRING][NUM_CHANNELS];
	uint8_t overlap_num[NUM_LED_OUTRING];
	static uint8_t overlap_ctr[NUM_LED_OUTRING]={0};
	uint8_t pos[NUM_CHANNELS];

	uint8_t do_advance_overlap = 0;
	static uint32_t last_advance_overlap_tmr=0;
	uint32_t now = HAL_GetTick()/TICKS_PER_MS;

	if ((now - last_advance_overlap_tmr) > 300)
	{
		do_advance_overlap = 1;
		last_advance_overlap_tmr = now;
	}

	for (i = 0; i < NUM_LED_OUTRING; i++){
		led_cont.outring[i].brightness = 0;
		overlap_num[i] = 0;
		for (chan=0; chan<NUM_CHANNELS; chan++)
			overlap[i][chan] = 99;
	}

	// Create overlap[led_position][channels_occupying_position] = channel#
	for (i = 0; i < NUM_CHANNELS; i++)
	{
		// Each page uses six groups: off, MAIN, AUX. Slots are independent of IDs.
		pos[i] = rotate_origin(smn_model_led(smn_model_id_at(calc_params.wtsel[i])), NUM_LED_OUTRING);
		led = rotate_origin(i, NUM_CHANNELS);

		set_model_color(smn_model_id_at(calc_params.wtsel[i]), &led_cont.inring[led]);

		if (params.osc_param_lock[i] && lock_flash_state())
			led_cont.inring[led].brightness = 0;
		else
			led_cont.inring[led].brightness = F_MAX_BRIGHTNESS;

		overlap[ pos[i] ][ overlap_num[pos[i]] ] = i;
		overlap_num[ pos[i] ]++;
	}

	for (i = 0; i<NUM_LED_OUTRING; i++)
	{
		if (do_advance_overlap) {
			overlap_ctr[i]++;
		}
		// A selection can reduce the occupants between animation ticks.
		if (overlap_ctr[i] >= overlap_num[i]) overlap_ctr[i] = 0;

		chan = overlap[i][overlap_ctr[i]];
		if (chan<NUM_CHANNELS)  {
			set_model_color(smn_model_id_at(calc_params.wtsel[chan]), &led_cont.outring[i]);

			if (params.osc_param_lock[chan] && lock_flash_state())
				led_cont.outring[i].brightness = 0;
			else
				led_cont.outring[i].brightness = F_MAX_BRIGHTNESS;
		}
	}
}

void display_octave(void)
{
    display_octave_rings(1);
}

static void display_octave_rings(uint8_t show_outer_ring)
{
	uint8_t i,j;
	int16_t oct;

    if (show_outer_ring) {
        for (i = 0; i < NUM_LED_OUTRING; i++) {
            led_cont.outring[i].brightness = 0;
            set_octave_color(&led_cont.outring[i], OCT_OUTRING_MAP[i]);
        }
    }

	// Light up LED ring positions corresponding to current indiv oct
	for (i=0; i<NUM_CHANNELS; i++){
		oct = _CLAMP_I16(params.oct[i], MIN_OCT , MAX_OCT) - MIN_OCT;
		oct = _CLAMP_I16(oct, 0, NUM_LED_OUTRING-1);

		j = rotate_origin(i, NUM_CHANNELS);
        set_octave_color(&led_cont.inring[j], oct);

		// flash locked channels
		if (led_cont.flash_state && params.osc_param_lock[i]){
			led_cont.inring[j].brightness = 0;
		}
		else
		{
			if(macro_states.all_af_buttons_released) {
				led_cont.inring[j].brightness = F_MAX_BRIGHTNESS;
			}
			else
			{
				if (button_pressed(i) || (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE)){ //Todo: What is FINETUNE doing here?
					led_cont.inring[j].brightness = F_MAX_BRIGHTNESS;
				}
				// idle channels are grey-ed
				else if (!button_pressed(i)){
					led_cont.inring[j].brightness = 0.1;
				}
			}
		}
        if (show_outer_ring)
            led_cont.outring[OCT_OUTRING_MAP[oct]].brightness = led_cont.inring[j].brightness;
	}
}

void update_ongoing_display_timers(void){

	static uint32_t 	last_systick = 0;
	uint32_t 			elapsed_ticks;
	uint8_t				tick_down=0;
	uint32_t 			now = HAL_GetTick()/TICKS_PER_MS;

	elapsed_ticks = now - last_systick;
	last_systick = now;

	if (led_cont.ongoing_display == ONGOING_DISPLAY_NONE)
		return;

	if ( led_cont.ongoing_display == ONGOING_DISPLAY_OSC_PARAM_LOCK)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_OCTAVE)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_SCALE && !rotary_pressed(rotm_OCT))
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_SCALE_ROOT)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_TRANSPOSE && macro_states.all_af_buttons_released && !rotary_pressed(rotm_TRANSPOSE) )
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE && macro_states.all_af_buttons_released && !rotary_pressed(rotm_TRANSPOSE) && !switch_pressed(FINE_BUTTON) )
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_LFO_TOVCA)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_LFO_MODE)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_MODEL_SEL)
		tick_down = 1;
	else if (led_cont.ongoing_display == ONGOING_DISPLAY_GLIDE)
		tick_down = 1;

	else if ((led_cont.ongoing_display == ONGOING_DISPLAY_PRESET) && rotary_released(rotm_PRESET))
		tick_down = 1;

	else if ((led_cont.ongoing_display == ONGOING_DISPLAY_GLOBRIGHT) && rotary_released(rotm_PRESET))
		tick_down = 1;


	if (!tick_down)
		return;

	if (led_cont.ongoing_timeout)
	{
		if (led_cont.ongoing_timeout < elapsed_ticks)
			led_cont.ongoing_timeout = 0;
		else
			led_cont.ongoing_timeout -= elapsed_ticks;
	}

	if (!led_cont.ongoing_timeout)
	{
		if (led_cont.ongoing_display == ONGOING_DISPLAY_GLIDE && rotary_pressed(rotm_OCT)) {
            start_ongoing_display_scale();
            return;
        }
		led_cont.ongoing_display 	= ONGOING_DISPLAY_NONE;
		led_cont.ongoing_timeout 	= 0;
	}
}

void start_ongoing_display_glide(void)
{
    led_cont.ongoing_display = ONGOING_DISPLAY_GLIDE;
    led_cont.ongoing_timeout = 1000;
}

void start_ongoing_display_finetune(void){
	led_cont.ongoing_display 	= ONGOING_DISPLAY_FINETUNE;
	led_cont.ongoing_timeout  	= FINETUNE_TIMER_LIMIT;
}

void start_ongoing_display_octave(void){
	led_cont.ongoing_display 	= ONGOING_DISPLAY_OCTAVE;
	led_cont.ongoing_timeout	= OCTAVE_TIMER_LIMIT;
}

void start_ongoing_display_scale(void){
    octave_was_pressed = rotary_pressed(rotm_OCT) != RELEASED;
	led_cont.ongoing_display 	= ONGOING_DISPLAY_SCALE;
	led_cont.ongoing_timeout 	= SCALE_TIMER_LIMIT;
}

void start_ongoing_display_scale_root(void) {
    octave_was_pressed = rotary_pressed(rotm_OCT) != RELEASED;
    led_cont.ongoing_display = ONGOING_DISPLAY_SCALE_ROOT;
    led_cont.ongoing_timeout = SCALE_TIMER_LIMIT;
}

void start_ongoing_display_transpose(void){
	led_cont.ongoing_display	= ONGOING_DISPLAY_TRANSPOSE;
	led_cont.ongoing_timeout  	= TRANSPOSE_TIMER_LIMIT;
}

void start_ongoing_display_lfo_tovca(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_LFO_TOVCA;
	led_cont.ongoing_timeout = LFO_TOVCA_TIMER_LIMIT;
}

void start_ongoing_display_lfo_mode(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_LFO_MODE;
	led_cont.ongoing_timeout = LFO_MODE_TIMER_LIMIT;
}

void start_ongoing_display_preset(void)
{
	if(led_cont.ongoing_display != ONGOING_DISPLAY_GLOBRIGHT){
		led_cont.ongoing_display 	= ONGOING_DISPLAY_PRESET;
		led_cont.ongoing_timeout 	= PRESET_TIMER_LIMIT;
	}
}

void start_ongoing_display_globright(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_GLOBRIGHT;
	led_cont.ongoing_timeout = GLOBRIGHT_TIMER_LIMIT;
}

void start_ongoing_display_model_sel(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_MODEL_SEL;
	led_cont.ongoing_timeout = MODEL_SEL_TIMER_LIMIT;
}

void stop_all_displays(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_NONE;
	led_cont.ongoing_timeout = 0;
}
