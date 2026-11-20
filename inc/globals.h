/*
 * globals.h
 *
 * Author: Dan Green(danngreen1@gmail.com), Hugo Paris (hugoplho@gmail.com)
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

// Set the PCB version here:
#define PCB_VERSION 100			//version v1.0 and v1.1
// #define PCB_VERSION 25		//In-house Proto (white PCB)
// #define PCB_VERSION 24		//Beta version

#ifndef PCB_VERSION
	#define PCB_VERSION 100
#endif

// SMN release displayed at startup, independent of stored calibration versions.
// Major 0 denotes beta: inner ring off, outer ring shows the minor version.
#define SMN_FW_MAJOR_VERSION 0
#define SMN_FW_MINOR_VERSION 1

// Legacy system/calibration compatibility version; not the SMN release number.
#define FW_MAJOR_VERSION 	2
#define FW_MINOR_VERSION 	2

#define ALLOW_LOCKS

// Stored system versions above these limits are rejected.
#define MAX_VALID_MAJOR_FW_VERSION 15
#define MAX_VALID_MINOR_FW_VERSION 15

#define DEBUG_2_3_ENABLED

#define SAMPLERATE 			44100
#define F_SAMPLERATE 		44100.0

#define NUM_INRING_LEDS		6
#define NUM_OUTRING_LEDS	18
#define NUM_LEDLINE_LEDS	6
#define NUM_CHANNELS 		6

#define TICKS_PER_MS 	8

enum CacheUncache {
	CACHE,
	UNCACHE
};

// DMA buffers keep their default placement; SRAM1DATA opts into NOLOAD SRAM.

#define DMABUFFER
// #define DMABUFFER 	__attribute__ ((section (".dtcmdata")))

#define SRAM1DATA	__attribute__ ((section (".sram1data")))
// #define SRAM1DATA
