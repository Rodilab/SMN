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
// 8x8x3 wave terrain.

#ifndef PLAITS_DSP_ENGINE_WAVETABLE_ENGINE_H_
#define PLAITS_DSP_ENGINE_WAVETABLE_ENGINE_H_

#include "plaits/dsp/engine/engine.h"
#include "plaits/dsp/oscillator/wavetable_oscillator.h"

namespace plaits {

class WavetableEngine : public Engine {
 public:
  static constexpr size_t kNativeBlockSize = 16;
  WavetableEngine() { }
  ~WavetableEngine() { }
  
  virtual void Init(stmlib::BufferAllocator* allocator);
  virtual void Reset();
  // SMN uses the factory banks only, in read-only application flash.
  virtual void LoadUserData(const uint8_t* user_data) { }
  virtual void Render(const EngineParameters& parameters,
      float* out,
      float* aux,
      size_t size,
      bool* already_enveloped);

  // MAIN/AUX write the selected signal to out; BOTH retains the dual API.
  void RenderAtFrequency(const EngineParameters& parameters, float frequency,
      float* out, float* aux, size_t size,
      EngineOutput output = ENGINE_OUTPUT_BOTH);
  
 private:
  template<EngineOutput output>
  void RenderBlock(const EngineParameters& parameters, float frequency,
      float* out, float* aux, size_t size);
  float ReadWave(int x, int y, int z, int phase_i, float phase_f);
   
  float phase_;
  
  float x_pre_lp_;
  float y_pre_lp_;
  float z_pre_lp_;
  
  float x_lp_;
  float y_lp_;
  float z_lp_;

  float previous_x_;
  float previous_y_;
  float previous_z_;
  float previous_f0_;
  
  float xy_pre_coefficient_;
  float z_pre_coefficient_;
  float previous_lp_coefficient_48k_;
  float lp_coefficient_;
  
  Differentiator diff_out_;
  
  DISALLOW_COPY_AND_ASSIGN(WavetableEngine);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE_WAVETABLE_ENGINE_H_
