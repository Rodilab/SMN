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
// Harmonic oscillator based on Chebyshev polynomials.
// Works well for a small number of harmonics. For the higher order harmonics,
// we need to reinitialize the recurrence by computing two high harmonics.

#ifndef PLAITS_DSP_OSCILLATOR_HARMONIC_OSCILLATOR_H_
#define PLAITS_DSP_OSCILLATOR_HARMONIC_OSCILLATOR_H_

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"

#include "plaits/dsp/oscillator/sine_oscillator.h"

namespace plaits {

// Compile separate loops: no per-sample test of a scratch-buffer pointer.
enum class HarmonicFundamentalMode { LOCAL, STORE, READ };

template<int num_harmonics>
class HarmonicOscillator {
 public:
  // Temporary per-sample values, shared only by banks with identical clocks.
  struct Fundamental {
    float phase;
    float two_x;
  };

  HarmonicOscillator() { }
  ~HarmonicOscillator() { }

  void Init() {
    phase_ = 0.0f;
    frequency_ = 0.0f;
    for (int i = 0; i < num_harmonics; ++i) {
      amplitude_[i] = 0.0f;
    }
  }
  
  // harmonic_mask may exclude partials which are permanently zero (organ).
  // Such slots must start at zero and remain zero throughout this oscillator.
  // A silent bank may copy phase/frequency from an already rendered bank
  // with identical timing history and frequency input for this block.
  // STORE needs size temporary frames. READ consumes those same frames and
  // copies the completed clock from phase_source; both banks must be in sync.
  template<int first_harmonic_index, bool render_output = true,
           unsigned int harmonic_mask = 0xffffffffu,
           HarmonicFundamentalMode fundamental_mode = HarmonicFundamentalMode::LOCAL>
  void Render(
      float frequency,
      const float* amplitudes,
      float* out,
      size_t size,
      const HarmonicOscillator* phase_source = nullptr,
      Fundamental* fundamental = nullptr) {
    if (frequency >= 0.5f) {
      frequency = 0.5f;
    }
    
    if (!render_output) {
      // A silent bank needs only the final amplitude targets; skipping the
      // intermediate additions changes their rounding only. Copy phase and
      // frequency from the audible bank, or advance them normally if there
      // is no source bank.
      for (int i = 0; i < num_harmonics; ++i) {
        float f = frequency * static_cast<float>(first_harmonic_index + i);
        if (f >= 0.5f) f = 0.5f;
        amplitude_[i] = (harmonic_mask & (1u << i))
            ? amplitudes[i] * (1.0f - f * 2.0f) : 0.0f;
      }
      if (phase_source) {
        phase_ = phase_source->phase_;
        frequency_ = phase_source->frequency_;
      } else {
        stmlib::ParameterInterpolator fm(&frequency_, frequency, size);
        while (size--) {
          phase_ += fm.Next();
          if (phase_ >= 1.0f) phase_ -= 1.0f;
        }
      }
      return;
    }

    // A reader copies the completed clock below. Its unused interpolator must
    // not overwrite that copy when its destructor runs; it is optimized away.
    float unused_frequency = frequency_;
    stmlib::ParameterInterpolator fm(
        fundamental_mode == HarmonicFundamentalMode::READ
            ? &unused_frequency : &frequency_, frequency, size);
    stmlib::ParameterInterpolator am[num_harmonics];

    for (int i = 0; i < num_harmonics; ++i) {
      float f = frequency * static_cast<float>(first_harmonic_index + i);
      if (f >= 0.5f) {
        f = 0.5f;
      }
      const float target = (harmonic_mask & (1u << i))
          ? amplitudes[i] * (1.0f - f * 2.0f) : 0.0f;
      am[i].Init(&amplitude_[i], target, size);
    }

    while (size--) {
      float phase, two_x;
      if (fundamental_mode == HarmonicFundamentalMode::READ) {
        phase = fundamental->phase;
        two_x = fundamental->two_x;
        ++fundamental;
      } else {
        phase_ += fm.Next();
        if (phase_ >= 1.0f) {
          phase_ -= 1.0f;
        }
        phase = phase_;
        two_x = 2.0f * SineNoWrap(phase);
        if (fundamental_mode == HarmonicFundamentalMode::STORE) {
          fundamental->phase = phase;
          fundamental->two_x = two_x;
          ++fundamental;
        }
      }
      float previous, current;
      if (first_harmonic_index == 1) {
        previous = 1.0f;
        current = two_x * 0.5f;
      } else {
        const float k = first_harmonic_index;
        previous = SinePositiveWrap(phase * (k - 1.0f) + 0.25f);
        current = SinePositiveWrap(phase * k);
      }
      
      float sum = 0.0f;
      for (int i = 0; i < num_harmonics; ++i) {
        if (harmonic_mask & (1u << i)) sum += am[i].Next() * current;
        float temp = current;
        current = two_x * current - previous;
        previous = temp;
      }
      if (first_harmonic_index == 1) {
        *out++ = sum;
      } else {
        *out++ += sum;
      }
    }
    if (fundamental_mode == HarmonicFundamentalMode::READ) {
      phase_ = phase_source->phase_;
      frequency_ = phase_source->frequency_;
    }
  }

 private:
  // Oscillator state.
  float phase_;

  // For interpolation of parameters.
  float frequency_;
  float amplitude_[num_harmonics];
  
  DISALLOW_COPY_AND_ASSIGN(HarmonicOscillator);
};

}  // namespace plaits

#endif  // PLAITS_DSP_OSCILLATOR_HARMONIC_OSCILLATOR_H_
