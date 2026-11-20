// Copyright 2016 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Extended Karplus-Strong, with all the niceties from Rings.

#include "plaits/dsp/physical_modelling/string_voice.h"

#include <algorithm>

#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/noise/dust.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void StringVoice::Init(BufferAllocator* allocator) {
  excitation_filter_.Init();
  string_.Init(allocator);
}

void StringVoice::Reset() {
  // Preserve the upstream excitation-filter history on a string reset.
  string_.Reset();
}

void StringVoice::RenderContinuous(float f0, float structure, float brightness,
    float damping, float* out, Random* random) {
  const float density = brightness * brightness;
  brightness += 0.25f * 0.8f * (1.0f - brightness);
  damping += 0.25f * 0.8f * (1.0f - damping);
  const float non_linearity = structure < 0.24f
      ? (structure - 0.24f) * 4.166f
      : (structure > 0.26f ? (structure - 0.26f) * 1.35135f : 0.0f);
  const float cutoff = min(
      (4.0f * f0) * SemitonesToRatio(
          (brightness * (2.0f - brightness) - 0.5f) * 72.0f), 0.499f);
  excitation_filter_.set_f_q<FREQUENCY_DIRTY>(cutoff, 1.0f);
  const float dust_f = min(
      (0.00005f + 0.99995f * density * density) * (48000.0f / kSampleRate), 1.0f);
  string_.ProcessContinuous(f0, non_linearity, brightness, damping, out,
                            &excitation_filter_, dust_f, random);
}

void StringVoice::RenderExternal(float f0, float structure, float damping,
    const float* input, float* out, Random* random) {
  // TIMBRE is input gain; keep the resonator brightness at the old midpoint.
  const float brightness = 0.4f; // (0.5^2) + 0.2 * (1 - 0.5^2).
  damping += 0.2f * (1.0f - damping);
  const float non_linearity = structure < 0.24f
      ? (structure - 0.24f) * 4.166f
      : (structure > 0.26f ? (structure - 0.26f) * 1.35135f : 0.0f);
  string_.ProcessExternal(f0, non_linearity, brightness, damping, input, out, random);
}

}  // namespace plaits
