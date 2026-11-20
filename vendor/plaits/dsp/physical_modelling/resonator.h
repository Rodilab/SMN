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
// Resonator, taken from Rings' code but with fixed position.

#ifndef PLAITS_DSP_PHYSICAL_MODELLING_RESONATOR_H_
#define PLAITS_DSP_PHYSICAL_MODELLING_RESONATOR_H_

#include "stmlib/dsp/filter.h"

namespace plaits {

// SMN: keep original modes 1-8, 10, 12, 14, 16, packed into three batches.
// Source ranks still determine frequencies, damping and excitation position.
const int kMaxNumModes = 12;
const int kModalSourceNumModes = 16;
constexpr unsigned kModalRetainedModes = 0xaaffu;
const int kModeBatchSize = 4;
static_assert(kMaxNumModes % kModeBatchSize == 0,
              "SMN must render complete batches of resonances");

// We render 4 modes simultaneously since there are enough registers to hold
// all state variables.
template<int batch_size>
class ResonatorSvf {
 public:
  ResonatorSvf() { }
  ~ResonatorSvf() { }
  
  void Init() {
    for (int i = 0; i < batch_size; ++i) {
      state_1_[i] = state_2_[i] = 0.0f;
    }
  }

  // Single excitation filter: consume generated samples directly, retaining
  // the original low-pass equations and its two private integrator states.
  template<typename Generator>
  void ProcessLowPassGenerated(float f, float q, Generator next,
                               float* out, size_t size) {
    static_assert(batch_size == 1, "Only the single excitation filter is fused");
    const float g = stmlib::OnePole::tan<stmlib::FREQUENCY_FAST>(f);
    const float r = 1.0f / q;
    const float h = 1.0f / (1.0f + r * g + g * g);
    const float r_plus_g = r + g;
    float state_1 = state_1_[0];
    float state_2 = state_2_[0];
    while (size--) {
      const float s_in = next();
      const float hp = (s_in - r_plus_g * state_1 - state_2) * h;
      const float bp = g * hp + state_1;
      state_1 = g * hp + bp;
      const float lp = g * bp + state_2;
      state_2 = g * bp + lp;
      *out++ = 0.0f + lp;
    }
    state_1_[0] = state_1;
    state_2_[0] = state_2;
  }
  
  template<stmlib::FilterMode mode, bool add, bool render = true>
  void Process(
      const float* f,
      const float* q,
      const float* gain,
      const float* in,
      float* out,
      size_t size) {
    float g[batch_size];
    float r_plus_g[batch_size];
    float h[batch_size];
    float g_h[batch_size];
    float two_g[batch_size];
    float state_1[batch_size];
    float state_2[batch_size];
    float gains[batch_size];
    for (int i = 0; i < batch_size; ++i) {
      g[i] = stmlib::OnePole::tan<stmlib::FREQUENCY_FAST>(f[i]);
      if constexpr (mode == stmlib::FILTER_MODE_BAND_PASS) {
        // Same denominator as 1 / (1 + g/Q + g*g), one division per mode.
        h[i] = q[i] / (q[i] * (1.0f + g[i] * g[i]) + g[i]);
        g_h[i] = g[i] * h[i];
        two_g[i] = g[i] + g[i];
      } else {
        // Keep the excitation low-pass arithmetic unchanged.
        const float r = 1.0f / q[i];
        h[i] = 1.0f / (1.0f + r * g[i] + g[i] * g[i]);
        r_plus_g[i] = r + g[i];
      }
      state_1[i] = state_1_[i];
      state_2[i] = state_2_[i];
      if constexpr (render) gains[i] = gain[i];
    }
    
    if constexpr (mode == stmlib::FILTER_MODE_BAND_PASS) {
      // Render four successive samples per loop; retain all selected modes and
      // every state update. Other block lengths keep their remainder samples.
      #pragma GCC unroll 4
      while (size--) {
        const float s_in = *in++;
        float s_out = 0.0f;
        for (int i = 0; i < batch_size; ++i) {
          // Factor the same SVF while retaining its two integrator states.
          const float bp = h[i] * state_1[i] + g_h[i] * (s_in - state_2[i]);
          state_1[i] = 2.0f * bp - state_1[i];
          state_2[i] += two_g[i] * bp;
          if constexpr (render) s_out += gains[i] * bp;
        }
        if constexpr (render) {
          if (add) {
            *out++ += s_out;
          } else {
            *out++ = s_out;
          }
        }
      }
    } else {
      // Keep the excitation low-pass loop, including its arithmetic, unchanged.
      while (size--) {
        const float s_in = *in++;
        float s_out = 0.0f;
        for (int i = 0; i < batch_size; ++i) {
          const float hp = (s_in - r_plus_g[i] * state_1[i] - state_2[i]) * h[i];
          const float bp = g[i] * hp + state_1[i];
          state_1[i] = g[i] * hp + bp;
          const float lp = g[i] * bp + state_2[i];
          state_2[i] = g[i] * bp + lp;
          if constexpr (render) s_out += gains[i] * lp;
        }
        if constexpr (render) {
          if (add) {
            *out++ += s_out;
          } else {
            *out++ = s_out;
          }
        }
      }
    }
    for (int i = 0; i < batch_size; ++i) {
      state_1_[i] = state_1[i];
      state_2_[i] = state_2[i];
    }
  }
  
 private:
  float state_1_[batch_size];
  float state_2_[batch_size];
  
  DISALLOW_COPY_AND_ASSIGN(ResonatorSvf);
};

class Resonator {
 public:
  Resonator() { }
  ~Resonator() { }
  
  void Init(float position, int resolution);
  // Clear vibrations only; keep the amplitudes/resolution prepared at Init.
  void Reset();
  void Process(
      float f0,
      float structure,
      float brightness,
      float damping,
      const float* in,
      float* out,
      size_t size);
  
 private:
  int resolution_;
  
  float mode_amplitude_[kMaxNumModes];
  ResonatorSvf<kModeBatchSize> mode_filters_[kMaxNumModes / kModeBatchSize];
  
  DISALLOW_COPY_AND_ASSIGN(Resonator);
};

}  // namespace plaits

#endif  // PLAITS_DSP_PHYSICAL_MODELLING_RESONATOR_H_
