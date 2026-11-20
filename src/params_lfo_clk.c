/*
 * params_lfo_clk.c
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

#include <stm32f7xx.h>
#include <math.h>
#include "params_lfo.h"
#include "params_lfo_clk.h"
#include "params_lfo_period.h"
#include "gpio_pins.h"
#include "UI_conditioning.h"
#include "math_util.h"
#include "led_cont.h"
#include "params_update.h"
#include "system_settings.h"

extern o_led_cont 		led_cont;
extern o_lfos 			lfos;
extern o_systemSettings	system_settings;

// TIM5 is a private, free-running 1 MHz clock, with no interrupt or GPIO.
// Periods in o_lfos retain the historical 8000 ticks/second units.
#ifdef TEST
extern uint32_t smn_clock_test_us;
extern uint8_t smn_clock_test_source, smn_clock_test_level;
#define CLOCK_US() smn_clock_test_us
#else
#define CLOCK_US() (TIM5->CNT)
#endif
static uint32_t previous_update, previous_edge;
static uint8_t source_previous, clock_up, have_edge;
static float filtered_period, phase_rate = 1.0f;

void init_ext_clk(void)
{
#ifndef TEST
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0;
    TIM5->DIER = 0;
    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) timer_hz *= 2;
    TIM5->PSC = timer_hz / 1000000u - 1u;
    TIM5->ARR = UINT32_MAX;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0;
    TIM5->CNT = 0;
    TIM5->CR1 = TIM_CR1_CEN;
#endif
    previous_update = CLOCK_US();
    previous_edge = previous_update;
    source_previous = clock_up = have_edge = 0;
    filtered_period = 0.0f;
    phase_rate = lfo_clock_elapsed = lfo_clock_step = 1.0f;
}

void read_ext_clk(void)
{
    const uint32_t now = CLOCK_US();
    const uint32_t elapsed = now - previous_update; // unsigned wrap is intentional
    previous_update = now;
#ifdef TEST
    const uint8_t source = smn_clock_test_source;
    const uint8_t level = smn_clock_test_level;
#else
    const uint8_t source = jack_plugged(CLK_SENSE) ? 1 :
        (system_settings.allow_bus_clock == 1 ? 2 : 0);
    const uint8_t level = source == 1 ? CLK_IN() : (source == 2 ? BUS_CLK() : 0);
#endif
    if (source != source_previous) {
        source_previous = source;
        have_edge = clock_up = 0;
        filtered_period = 0.0f;
        phase_rate = 1.0f;
        lfos.use_ext_clock = 0;
        led_cont.waiting_for_clockin = 0;
        led_cont.clockin_wait_progress = 0;
    }

    if (!source) {
        lfo_clock_elapsed = lfo_clock_step = 1.0f;
        return;
    }
    if (level && !clock_up) {
        const uint32_t interval = now - previous_edge;
        previous_edge = now;
        clock_up = 1;
        led_cont.waiting_for_clockin = 0;
        led_cont.clockin_wait_progress = 0;
        if (have_edge && interval) {
            float period = interval * 0.008f;
            // Retain the original folding of fast incoming clocks below 50 ms.
            while (period < F_LFO_AUDIO_RANGE_PERIOD_L) period *= 2.0f;
            if (filtered_period == 0.0f ||
                fabsf(period - filtered_period) > filtered_period * 0.05f)
                filtered_period = period; // follow deliberate tempo changes promptly
            else
                filtered_period += (period - filtered_period) * 0.25f;
            const uint32_t ticks = (uint32_t)(filtered_period + 0.5f);
            if (ticks != lfos.period[REF_CLK]) {
                lfos.period[REF_CLK] = ticks;
                flag_all_lfos_recalc();
            }
            // Acquisition anchors only the reference. Never reset divided cycles.
            if (!lfos.use_ext_clock) lfos.cycle_pos[REF_CLK] = 0.0f;
            // Phase error is corrected by at most 2% speed, shared by unlocked
            // clocked cycles. Their existing phase retains the division count.
            float error = -lfos.cycle_pos[REF_CLK];
            if (error < -0.5f) error += 1.0f;
            phase_rate = interval >= 50000u ?
                1.0f + _CLAMP_F(error, -0.02f, 0.02f) : 1.0f;
            lfos.use_ext_clock = 1;
        }
        have_edge = 1;
    }
    if (!level) clock_up = 0;

    const float absent_ticks = (now - previous_edge) * 0.008f;
    if (have_edge && absent_ticks > 2.0f * lfos.period[REF_CLK]) {
        have_edge = 0;
        filtered_period = 0.0f;
        phase_rate = 1.0f;
        lfos.use_ext_clock = 0;
        led_cont.waiting_for_clockin = 0;
        led_cont.clockin_wait_progress = 0;
    } else if (lfos.use_ext_clock && absent_ticks > lfos.period[REF_CLK]) {
        led_cont.waiting_for_clockin = 1;
        led_cont.clockin_wait_progress = absent_ticks / (2.0f * lfos.period[REF_CLK]);
    }
    // Recover elapsed time even when TIM9 calls were delayed/coalesced by audio.
    // Note/Keyboard one-shot envelopes retain their existing advancement rules.
    lfo_clock_elapsed = lfos.use_ext_clock ? elapsed * 0.0072f : 1.0f;
    lfo_clock_step = lfos.use_ext_clock ? lfo_clock_elapsed * phase_rate : 1.0f;
}
