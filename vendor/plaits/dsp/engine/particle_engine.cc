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
// Clocked noise processed by a filter.

#include "plaits/dsp/engine/particle_engine.h"

#include <algorithm>

namespace plaits {

using namespace std;
using namespace stmlib;

void ParticleEngine::Init(BufferAllocator* allocator) {
  for (int i = 0; i < kNumParticles; ++i) {
    particle_[i].Init();
  }
  diffuser_.Init(allocator->Allocate<uint16_t>(8192));
  post_filter_.Init();
  random_.Seed(0x50617274u);
  previous_timbre_ = previous_frequency_ = -1.0f;
  density_ = gain_ = density_sqrt_ = 0.0f;
  density_threshold_ = 0;
  previous_main_timbre_ = -1.0f;
  main_density_ = main_gain_ = 0.0f;
  main_density_threshold_ = 0;
}

void ParticleEngine::Reset() {
  diffuser_.Reset();
}

void ParticleEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note),
      out, aux, size, ENGINE_OUTPUT_BOTH);
}

template<EngineOutput output, size_t fixed_size, bool continuous>
void ParticleEngine::RenderParticles(bool sync, float density, float gain,
    float frequency, float spread, float q, float density_sqrt,
    float* out, float* aux, size_t size) {
  if constexpr (fixed_size) size = fixed_size;
  float q_inverse = 0.0f;
  if constexpr (output != ENGINE_OUTPUT_AUX) q_inverse = 1.0f / q;
  if constexpr (output != ENGINE_OUTPUT_MAIN)
    std::fill_n(output == ENGINE_OUTPUT_AUX ? out : aux, size, 0.0f);
  uint32_t rng = random_.state();
  constexpr bool reduced_main = continuous && output == ENGINE_OUTPUT_MAIN;
  constexpr int count = reduced_main ? kNumContinuousMainParticles : kNumParticles;
  const uint32_t threshold = reduced_main ? main_density_threshold_ : density_threshold_;
  particle_[0].Render<output, false, fixed_size, continuous>(sync, density, threshold, gain,
      frequency, spread, q, q_inverse, density_sqrt, out, aux, size, rng);
  for (int i = 1; i < count; ++i)
    particle_[i].Render<output, true, fixed_size, continuous>(sync, density, threshold, gain,
        frequency, spread, q, q_inverse, density_sqrt, out, aux, size, rng);
  random_.Seed(rng);
}

void ParticleEngine::RenderAtFrequency(const EngineParameters& parameters,
    float f0, float* out, float* aux, size_t size, EngineOutput output) {
  if (output == ENGINE_OUTPUT_MAIN)
    RenderBlock<ENGINE_OUTPUT_MAIN, 0, false>(parameters, f0, out, aux, size, nullptr);
  else if (output == ENGINE_OUTPUT_AUX)
    RenderBlock<ENGINE_OUTPUT_AUX, 0, false>(parameters, f0, out, aux, size, nullptr);
  else
    RenderBlock<ENGINE_OUTPUT_BOTH, 0, false>(parameters, f0, out, aux, size, nullptr);
}

void ParticleEngine::RenderContinuous(const EngineParameters& parameters,
    float f0, float* out, EngineOutput output, Limiter* limiter) {
  if (output == ENGINE_OUTPUT_AUX)
    RenderBlock<ENGINE_OUTPUT_AUX, kParticleBlockSize, true>(
        parameters, f0, out, nullptr, kParticleBlockSize, nullptr);
  else
    RenderBlock<ENGINE_OUTPUT_MAIN, kParticleBlockSize, true>(
        parameters, f0, out, nullptr, kParticleBlockSize, limiter);
}

template<EngineOutput output, size_t fixed_size, bool continuous>
void ParticleEngine::RenderBlock(const EngineParameters& parameters,
    float f0, float* out, float* aux, size_t size, Limiter* limiter) {
  if constexpr (fixed_size) size = fixed_size;
  if (parameters.timbre != previous_timbre_) {
    const float density_sqrt = NoteToFrequency(
        60.0f + parameters.timbre * parameters.timbre * 72.0f);
    // Preserve events per second, not the square of a sample-rate change.
    density_ = density_sqrt * density_sqrt * (1.0f / kNumParticles)
        * (kSampleRate / 48000.0f);
    density_threshold_ = Particle::DensityThreshold(density_);
    gain_ = 1.0f / density_;
    density_sqrt_ = Sqrt(density_);
    previous_timbre_ = parameters.timbre;
  }
  const bool sync = !continuous && (parameters.trigger & TRIGGER_RISING_EDGE);
  if constexpr (output == ENGINE_OUTPUT_AUX) {
    RenderParticles<ENGINE_OUTPUT_AUX, fixed_size, continuous>(sync, density_, gain_, f0,
        0.0f, 0.0f, 0.0f, out, aux, size);
    if constexpr (continuous)
      for (size_t i = 0; i < size; ++i) out[i] = std::clamp(out[i], -1.0f, 1.0f);
    return;
  }
  const float q_sqrt = SemitonesToRatio(parameters.morph >= 0.5f
      ? (parameters.morph - 0.5f) * 120.0f : 0.0f);
  const float q = 0.5f + q_sqrt * q_sqrt;
  const float spread = 48.0f * parameters.harmonics * parameters.harmonics;
  if constexpr (continuous && output == ENGINE_OUTPUT_MAIN) {
    if (previous_main_timbre_ != parameters.timbre) {
      // Four trains at 1.5 times the probability preserve the total event rate.
      // Keep the six-particle pre-gain normalization (density_sqrt_) so the
      // amplitude distribution of individual events is retained as well.
      main_density_ = density_ * (float(kNumParticles) / kNumContinuousMainParticles);
      main_gain_ = gain_ * (float(kNumContinuousMainParticles) / kNumParticles);
      main_density_threshold_ = Particle::DensityThreshold(main_density_);
      previous_main_timbre_ = parameters.timbre;
    }
    RenderParticles<output, fixed_size, continuous>(sync, main_density_, main_gain_, f0,
        spread, q, density_sqrt_, out, aux, size);
  } else {
    RenderParticles<output, fixed_size, continuous>(sync, density_, gain_, f0,
        spread, q, density_sqrt_, out, aux, size);
  }
  if (f0 != previous_frequency_) {
    post_filter_.SetFrequency(min(f0, 0.49f), 0.5f);
    previous_frequency_ = f0;
  }
  const float raw_diffusion_sqrt = 2.0f * fabsf(parameters.morph - 0.5f);
  const float diffusion = parameters.morph < 0.5f
      ? raw_diffusion_sqrt * raw_diffusion_sqrt : 0.0f;
  // Advance the delay and modulation even with no wet signal, so the tail
  // is ready when MORPH returns below 0.5. A local filter copy lets the
  // compiler keep coefficients in registers while writing the output.
  ParticlePostFilter post_filter = post_filter_;
  for (size_t i = 0; i < size; ++i) out[i] = post_filter.Process(out[i]);
  if constexpr (continuous && output == ENGINE_OUTPUT_MAIN)
    diffuser_.ProcessHalfRate(0.8f * diffusion * diffusion,
        0.5f * diffusion + 0.25f, out, size);
  else
    diffuser_.Process(0.8f * diffusion * diffusion,
        0.5f * diffusion + 0.25f, out, size);
  post_filter_ = post_filter;
  if constexpr (continuous && output == ENGINE_OUTPUT_MAIN) {
    limiter->Process(2.0f, out, size);
    for (size_t i = 0; i < size; ++i) out[i] = std::clamp(out[i], -1.0f, 1.0f);
  }
}

}  // namespace plaits
