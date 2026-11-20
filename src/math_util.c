/*
 * math_util.c
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

#include "math_util.h"
#include "math.h"

int16_t _CLAMP_I16(int16_t test, int16_t low, int16_t high)
{
		if (test<low)	return low;
		if (test>high) 	return high;
		return test;
}

uint32_t _CLAMP_U32(uint32_t test, uint32_t low, uint32_t high)
{
		if (test<low)	return low;
		if (test>high) 	return high;
		return test;
}
float _CLAMP_F(float test, float low, float high)
{
		if (test<low)	return low;
		if (test>high) 	return high;
		return test;
}


//Fold turns a ramp from 0 to fold_point*2 into a triangle from 0 to fold_point to 0
//Max val of input must be fold_point * 2
//Min val of input must be 0
//Output will have min/max of 0 to fold_point
float _FOLD_F(float val, float fold_point)
{
	val = _CLAMP_F(val, 0, fold_point*2);
	if (val > fold_point) val = 2*fold_point - val;
	return val;
}

int8_t	_WRAP_I8(int8_t val, int8_t min, int8_t max)
{
	if (max<=min) return 0; //error: bad parameters
	while (val<min) val+=(max-min);
	while (val>=max) val-=(max-min);
	return val;
}
int16_t	_WRAP_I16(int16_t val, int16_t min, int16_t max)
{
	if (max<=min) return 0; //error: bad parameters
	while (val<min) val+=(max-min);
	while (val>=max) val-=(max-min);
	return val;
}

float _WRAP_F(float val, float min, float max)
{
	if (max<=min) return 0; //error: bad parameters
	while (val<min) val+=(max-min);
	while (val>=max) val-=(max-min);
	return val;
}

// Given an array of floats, tosses out the min and max values
// and returns the average of the remaining elements
//
float _AVERAGE_EXCL_MINMAX_F(float *lpf_values, uint32_t num_elements)
{
	uint32_t i;
	float max=-HUGE_VALF, min=HUGE_VALF;
	float sum=0;

	if (num_elements==0) return 0;
	if (num_elements==1) return lpf_values[0];
	if (num_elements==2) return (lpf_values[0] + lpf_values[1])/2;

	for (i=0;i<num_elements;i++)
	{
		if (lpf_values[i] > max) max = lpf_values[i];
		if (lpf_values[i] < min) min = lpf_values[i];
		sum+=lpf_values[i];
	}
	sum = sum - max - min;
	num_elements-=2;

	return ((float)sum)/((float)num_elements);
}
