/*
 * math_util.h
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

#include <stm32f7xx.h>

#ifndef INT16_MAX
	#define INT16_MAX	(32767)
#endif
#ifndef INT16_MIN
	#define INT16_MIN	(-32768)
#endif
#ifndef INT32_MAX
	#define INT32_MAX	(2147483647)
#endif
#ifndef INT32_MIN
	#define INT32_MIN	(-2147483648)
#endif

int16_t _CLAMP_I16(int16_t test, int16_t low, int16_t high);
uint32_t _CLAMP_U32(uint32_t test, uint32_t low, uint32_t high);
float _CLAMP_F(float test, float low, float high);


float _FOLD_F(float val, float fold_point);

int8_t	_WRAP_I8(int8_t val, int8_t min, int8_t max);
int16_t	_WRAP_I16(int16_t val, int16_t min, int16_t max);
float	_WRAP_F(float val, float min, float max);

float _AVERAGE_EXCL_MINMAX_F(float *lpf_values, uint32_t num_elements);

static inline float _CROSSFADE(float a, float b, float xfade)
{
	return (a*(1.0-xfade)) + (b*xfade);
}
