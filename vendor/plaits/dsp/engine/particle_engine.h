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
// Filtered random pulses.

#ifndef PLAITS_DSP_ENGINE_PARTICLE_ENGINE_H_
#define PLAITS_DSP_ENGINE_PARTICLE_ENGINE_H_

#include "plaits/dsp/engine/engine.h"
#include "plaits/dsp/fx/diffuser.h"
#include "plaits/dsp/noise/particle.h"
#include "stmlib/dsp/limiter.h"

namespace plaits {

const int kNumParticles = 6;
const int kNumContinuousMainParticles = 4;
const size_t kParticleBlockSize = 16;

// Particle's final low-pass uses the original SVF operation order. Copy it
// locally so writes to the output cannot alias its coefficients or state.
class ParticlePostFilter {
 public:
  void Init() {
    state_1_ = state_2_ = 0.0f;
    SetFrequency(0.01f, 100.0f);
  }
  void SetFrequency(float frequency, float q) {
    g_ = stmlib::OnePole::tan<stmlib::FREQUENCY_DIRTY>(frequency);
    r_ = 1.0f / q;
    h_ = 1.0f / (1.0f + r_ * g_ + g_ * g_);
  }
  float Process(float in) {
    const float hp = (in - r_ * state_1_ - g_ * state_1_ - state_2_) * h_;
    const float bp = g_ * hp + state_1_;
    state_1_ = g_ * hp + bp;
    const float lp = g_ * bp + state_2_;
    state_2_ = g_ * bp + lp;
    return lp;
  }

 private:
  float g_, r_, h_, state_1_, state_2_;
};

class ParticleEngine : public Engine {
 public:
  ParticleEngine() { }
  ~ParticleEngine() { }
  
  virtual void Init(stmlib::BufferAllocator* allocator);
  virtual void Reset();
  virtual void LoadUserData(const uint8_t* user_data) { }
  virtual void Render(const EngineParameters& parameters,
      float* out,
      float* aux,
      size_t size,
      bool* already_enveloped);

  // SMN: private RNG and direct normalized-frequency API.
  void RenderAtFrequency(const EngineParameters& parameters, float frequency,
      float* out, float* aux, size_t size,
      EngineOutput output = ENGINE_OUTPUT_BOTH);
  // SMN fixed-block route: four MAIN particles + half-rate wet FX, six AUX
  // particles. Limiter/clip included. The generic diagnostic route above
  // retains six particles/full-rate FX; use one route per initialized stream.
  void RenderContinuous(const EngineParameters& parameters, float frequency,
      float* out, EngineOutput output, stmlib::Limiter* limiter);
  void Seed(uint32_t seed) { random_.Seed(seed); }

 private:
  template<EngineOutput output, size_t fixed_size, bool continuous>
  void RenderBlock(const EngineParameters& parameters, float frequency,
      float* out, float* aux, size_t size, stmlib::Limiter* limiter);
  template<EngineOutput output, size_t fixed_size, bool continuous>
  void RenderParticles(bool sync, float density, float gain, float frequency,
      float spread, float q, float density_sqrt, float* out, float* aux, size_t size);
  stmlib::Random random_;
  float previous_timbre_, density_, gain_, density_sqrt_;
  float previous_frequency_;
  uint32_t density_threshold_;
  float previous_main_timbre_, main_density_, main_gain_;
  uint32_t main_density_threshold_;
  Particle particle_[kNumParticles];
  Diffuser diffuser_;
  ParticlePostFilter post_filter_;
  
  DISALLOW_COPY_AND_ASSIGN(ParticleEngine);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE_PARTICLE_ENGINE_H_
