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
// Feeds frames to the LPC10 speech synth.

#include "plaits/dsp/speech/lpc_speech_synth_controller.h"
#include <algorithm>
#include "stmlib/dsp/units.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "plaits/dsp/oscillator/oscillator.h"

namespace plaits {
using namespace std;
using namespace stmlib;

void LPCSpeechSynthController::Init() {
  clock_phase_ = 0.0f;
  fill_n(sample_, 2, 0.0f);
  fill_n(next_sample_, 2, 0.0f);
  gain_ = 0.0f;
  synth_.Init();
}

void LPCSpeechSynthController::Render(
    float frequency, float address, float formant_shift,
    float* excitation, float* output, size_t size, EngineOutput selected) {
  const float rate_ratio = SemitonesToRatio((formant_shift - 0.5f) * 36.0f);
  // The LPC excitation/filter runs at 8 kHz, even with a 44.1 kHz codec.
  const float rate = rate_ratio * (8000.0f / kSampleRate);
  const float pitch_shift = frequency /
      (rate_ratio * kLPCSpeechSynthDefaultF0 / kCorrectedSampleRate);
  synth_.PlayFrame(phonemes_, address * (kLPCSpeechSynthNumVowels - 1.0001f), true);

  ParameterInterpolator gain_modulation(&gain_, 1.0f, size);
  
  while (size--) {
    float this_sample[2];
    copy(&next_sample_[0], &next_sample_[2], &this_sample[0]);
    fill(&next_sample_[0], &next_sample_[2], 0.0f);
    
    clock_phase_ += rate;
    if (clock_phase_ >= 1.0f) {
      clock_phase_ -= 1.0f;
      float reset_time = clock_phase_ / rate;
      float new_sample[2];
      
      synth_.Render(
          pitch_shift,
          &new_sample[0],
          &new_sample[1], 1);
      
      float discontinuity[2] = {
        new_sample[0] - sample_[0],
        new_sample[1] - sample_[1]
      };
      this_sample[0] += discontinuity[0] * ThisBlepSample(reset_time);
      next_sample_[0] += discontinuity[0] * NextBlepSample(reset_time);
      this_sample[1] += discontinuity[1] * ThisBlepSample(reset_time);
      next_sample_[1] += discontinuity[1] * NextBlepSample(reset_time);
      copy(&new_sample[0], &new_sample[2], &sample_[0]);
    }
    next_sample_[0] += sample_[0];
    next_sample_[1] += sample_[1];
    const float gain = gain_modulation.Next();
    if (selected != ENGINE_OUTPUT_MAIN) *excitation++ = this_sample[0] * gain;
    if (selected != ENGINE_OUTPUT_AUX) *output++ = this_sample[1] * gain;
  }
}
}  // namespace plaits
