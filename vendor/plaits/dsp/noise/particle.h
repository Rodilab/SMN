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
// Random impulse train processed by a resonant filter.

#ifndef PLAITS_DSP_NOISE_PARTICLE_H_
#define PLAITS_DSP_NOISE_PARTICLE_H_

#include <cstring>

#include "plaits/dsp/engine/engine.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/utils/random.h"

namespace plaits {

class Particle {
 public:
  Particle() { }
  ~Particle() { }

  inline void Init() {
    pre_gain_ = state_1_ = state_2_ = 0.0f;
    SetFrequency(0.01f, 0.01f);
  }

  // Largest uint32 whose correctly rounded float/2^32 is <= density.
  // Above 2^24, include the upper half-ULP; ties round to an even mantissa.
  // Densities passed by ParticleEngine are finite and in [0, 1].
  static uint32_t DensityThreshold(float density) {
    if (density >= 1.0f) return UINT32_MAX;
    const float scaled = density * 4294967296.0f;
    uint32_t bits;
    std::memcpy(&bits, &scaled, sizeof bits);
    const int exponent = static_cast<int>((bits >> 23) & 255) - 127;
    uint32_t threshold = static_cast<uint32_t>(scaled);
    if (exponent >= 24)
      threshold += (1u << (exponent - 24)) - (bits & 1u);
    return threshold;
  }

  // RNG state is borrowed once by the engine for all six particles. The final
  // unused draw and first-event pitch draw still advance exactly as before.
  template<EngineOutput output, bool add, size_t fixed_size = 0,
      bool continuous = false>
  inline void Render(bool sync, float density, uint32_t threshold, float gain,
      float frequency, float spread, float q, float q_inverse,
      float density_sqrt, float* out, float* aux, size_t size, uint32_t& rng) {
    auto next = [&rng]() {
      rng = rng * 1664525u + 1013904223u;
      return rng;
    };
    uint32_t u = next();
    bool first_event = true;
    float pre_gain = pre_gain_;
    float g = g_, r = r_, h = h_;
    float state_1 = state_1_, state_2 = state_2_;
    float g_h = g * h, two_g = g + g;
    const size_t count = fixed_size ? fixed_size : size;
    #pragma GCC unroll 2
    for (size_t i = 0; i < count; ++i) {
      const bool forced = !continuous && sync && i == 0;
      float s = 0.0f;
      if (forced || u <= threshold) {
        const float value = forced ? density
            : static_cast<float>(u) * (1.0f / 4294967296.0f);
        s = value * gain;
        if (first_event) {
          const uint32_t pitch_bits = next();
          if constexpr (output != ENGINE_OUTPUT_AUX) {
            const float pitch_random = 2.0f * (static_cast<float>(pitch_bits)
                * (1.0f / 4294967296.0f)) - 1.0f;
            const float f = std::min(
                stmlib::SemitonesToRatio(spread * pitch_random) * frequency, 0.25f);
            pre_gain = 0.5f / stmlib::Sqrt(q * f * density_sqrt);
            g = stmlib::OnePole::tan<stmlib::FREQUENCY_DIRTY>(f);
            r = q_inverse;
            h = 1.0f / (1.0f + r * g + g * g);
            g_h = g * h; two_g = g + g;
          }
          first_event = false;
        }
        if constexpr (output != ENGINE_OUTPUT_MAIN) {
          float* raw = output == ENGINE_OUTPUT_AUX ? out : aux;
          if constexpr (add) {
            // The old event branch rounded the product before the later sum.
            // Keep that boundary when moving the sum into the sparse branch;
            // otherwise fast FMA contraction changes rare overlapping impulses.
            const volatile float impulse = s;
            raw[i] += impulse;
          } else {
            raw[i] = s;
          }
        }

      }
      if constexpr (output != ENGINE_OUTPUT_AUX) {
        // Keep the established FMA grouping: splitting zero/nonzero input
        // changes long resonances for little or negative measured benefit.
        const float bp = h * state_1 + g_h * (pre_gain * s - state_2);
        state_1 = 2.0f * bp - state_1;
        state_2 += two_g * bp;
        if constexpr (add) out[i] += bp; else out[i] = bp;
      }
      u = next();
    }
    if constexpr (output != ENGINE_OUTPUT_AUX) {
      pre_gain_ = pre_gain;
      g_ = g; r_ = r; h_ = h;
      state_1_ = state_1; state_2_ = state_2;
    }
  }

 private:
  void SetFrequency(float f, float r) {
    g_ = stmlib::OnePole::tan<stmlib::FREQUENCY_DIRTY>(f);
    r_ = r;
    h_ = 1.0f / (1.0f + r * g_ + g_ * g_);
  }
  float pre_gain_, g_, r_, h_, state_1_, state_2_;

  DISALLOW_COPY_AND_ASSIGN(Particle);
};

}  // namespace plaits

#endif  // PLAITS_DSP_NOISE_PARTICLE_H_
