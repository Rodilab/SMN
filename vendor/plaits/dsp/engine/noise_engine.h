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
// Clocked noise processed by a multimode filter.

#ifndef PLAITS_DSP_ENGINE_NOISE_ENGINE_H_
#define PLAITS_DSP_ENGINE_NOISE_ENGINE_H_

#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"

#include "plaits/dsp/engine/engine.h"
#include "plaits/dsp/noise/clocked_noise.h"

namespace plaits {

class NoiseEngine : public Engine {
 public:
  NoiseEngine() { }
  ~NoiseEngine() { }
  
  virtual void Init(stmlib::BufferAllocator* allocator);
  virtual void Reset();
  virtual void LoadUserData(const uint8_t* user_data) { }
  virtual void Render(const EngineParameters& parameters,
      float* out,
      float* aux,
      size_t size,
      bool* already_enveloped);
  
  // SMN supplies normalized frequency, with no note round-trip.
  void RenderAtFrequency(const EngineParameters& parameters, float frequency,
      float* out, float* aux, size_t size,
      EngineOutput output = ENGINE_OUTPUT_BOTH);
  void Seed(uint32_t seed) { random_.Seed(seed); }

 private:
  template<EngineOutput output>
  void FilterBlock(float* out, float* aux, size_t size,
      stmlib::ParameterInterpolator& f0_modulation,
      stmlib::ParameterInterpolator& f1_modulation,
      stmlib::ParameterInterpolator& q_modulation,
      stmlib::ParameterInterpolator& mode_modulation);

  void RenderWithFrequencies(const EngineParameters& parameters,
      float f0, float f1, float* out, float* aux, size_t size,
      EngineOutput output);

  ClockedNoise clocked_noise_[2];
  // The original LP/HP and first BP filters have identical inputs,
  // coefficients and histories. One filter retains that common state.
  stmlib::Svf primary_filter_;
  stmlib::Svf secondary_filter_;
  stmlib::Random random_;
  
  float previous_f0_;
  float previous_f1_;
  float previous_q_;
  float previous_mode_;
  
  float* temp_buffer_;
  
  DISALLOW_COPY_AND_ASSIGN(NoiseEngine);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE_NOISE_ENGINE_H_
