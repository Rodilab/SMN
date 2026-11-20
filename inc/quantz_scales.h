/*
 * quantz_scales.h
 *
 * Author: Dan Green (danngreen1@gmail.com)
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

#include <stdint.h>
#include "params_update.h"

#define MAX_NUM_QTZ_STEPS 24

// Stored IDs and selector order are independent.
enum scaleMap {
    sclm_NONE, sclm_MAJOR, sclm_MINOR, sclm_SEMITONES,
    sclm_QUARTERTONES, sclm_LYDIAN, sclm_MIXOLYDIAN,
    sclm_AEOLIAN, sclm_DORIAN, sclm_PHRYGIAN,
    sclm_MELODIC_MINOR, sclm_HARMONIC_MAJOR,
    sclm_MAJOR_PENTATONIC, sclm_MINOR_PENTATONIC, sclm_BLUES,
    sclm_WHOLE_TONE, sclm_DIM_HALF_WHOLE, sclm_DIM_WHOLE_HALF,
    sclm_OCTAVES, sclm_ROOT_FIFTH, sclm_ROOT_FOURTH,
    sclm_MAJOR_TRIAD, sclm_MINOR_TRIAD, sclm_SUS2_TRIAD,
    sclm_DOUBLE_HARMONIC_MAJOR,
    NUM_QTZ_SCALES
};

uint8_t scale_position(uint8_t scale);
uint8_t scale_at_position(uint8_t position);
uint8_t scale_root(const o_params *p);
uint8_t scale_config_valid(const int8_t scales[6], const int8_t config[6]);
extern volatile uint32_t smn_scale_generation;
void scale_set(o_params *p, uint8_t scale, uint8_t root);
void scale_restore(o_params *p);
void init_quantz_scales(void);
extern volatile uint8_t scale_quantization_enabled;
static inline uint8_t scale_quantized(void) { return scale_quantization_enabled; }
float quantize_to_scale(float frequency, uint8_t *note, int8_t *octave);
