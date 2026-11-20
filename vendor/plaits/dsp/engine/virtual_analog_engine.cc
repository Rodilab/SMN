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
// 2 variable shape oscillators with sync, FM and crossfading.

#include "plaits/dsp/engine/virtual_analog_engine.h"

#include <algorithm>

#include "stmlib/dsp/parameter_interpolator.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void VirtualAnalogEngine::Init(BufferAllocator* allocator) {
  primary_.Init();
  auxiliary_.Init();
  auxiliary_.set_master_phase(0.25f);
  sync_.Init();
  variable_saw_.Init();
  
  auxiliary_amount_ = 0.0f;
  xmod_amount_ = 0.0f;
  
  temp_buffer_ = allocator->Allocate<float>(kMaxBlockSize);
}

void VirtualAnalogEngine::Reset() {
  
}

const float intervals[5] = {
  0.0f, 7.01f, 12.01f, 19.01f, 24.01f
};

inline float Squash(float x) {
  return x * x * (3.0f - 2.0f * x);
}

float VirtualAnalogEngine::ComputeDetuning(float detune) const {
  detune = 2.05f * detune - 1.025f;
  CONSTRAIN(detune, -1.0f, 1.0f);
  
  float sign = detune < 0.0f ? -1.0f : 1.0f;
  detune = detune * sign * 3.9999f;
  MAKE_INTEGRAL_FRACTIONAL(detune);
  
  float a = intervals[detune_integral];
  float b = intervals[detune_integral + 1];
  return (a + (b - a) * Squash(Squash(detune_fractional))) * sign;
}

void VirtualAnalogEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {

  RenderAtFrequency(parameters, NoteToFrequency(parameters.note),
                    out, aux, size);
}

void VirtualAnalogEngine::RenderAtFrequency(
    const EngineParameters& parameters, float frequency,
    float* out, float* aux, size_t size, EngineOutput output) {


  // 1 = variable square controlled by TIMBRE.
  // 2 = variable saw controlled by MORPH.
  // OUT = 1 + 2.
  // AUX = dual variable waveshape controlled by MORPH, self sync by TIMBRE.
  
  const float auxiliary_detune = ComputeDetuning(parameters.harmonics);
  const float primary_f = frequency;
  const float auxiliary_f = frequency * SemitonesToRatio(auxiliary_detune);

  // SMN: the unselected pair stays completely suspended, including its
  // interpolators and BLEP history. BOTH still renders the two pairs.
  if (output != ENGINE_OUTPUT_MAIN) {
    const float sync_amount = parameters.timbre * parameters.timbre;
    const float primary_sync_f = frequency * SemitonesToRatio(sync_amount * 48.0f);
    const float auxiliary_sync_f = frequency * SemitonesToRatio(auxiliary_detune + sync_amount * 48.0f);

    float shape = parameters.morph * 1.5f;
    CONSTRAIN(shape, 0.0f, 1.0f);

    float pw = 0.5f + (parameters.morph - 0.66f) * 1.46f;
    CONSTRAIN(pw, 0.5f, 0.995f);

    // Render monster sync to AUX.
    primary_.Render(primary_f, primary_sync_f, pw, shape, out, size);
    auxiliary_.Render(auxiliary_f, auxiliary_sync_f, pw, shape, aux, size);
    float* destination = output == ENGINE_OUTPUT_AUX ? out : aux;
    for (size_t i = 0; i < size; ++i) {
      destination[i] = (aux[i] - out[i]) * 0.5f;
    }
  }
  if (output == ENGINE_OUTPUT_AUX) {
    return;
  }
  
  // Render double varishape to OUT.
  float square_pw = 1.3f * parameters.timbre - 0.15f;
  CONSTRAIN(square_pw, 0.005f, 0.5f);
  
  const float square_sync_ratio = parameters.timbre < 0.5f
      ? 0.0f
      : (parameters.timbre - 0.5f) * (parameters.timbre - 0.5f) * 4.0f * 48.0f;
  
  const float square_gain = min(parameters.timbre * 8.0f, 1.0f);
  
  float saw_pw = parameters.morph < 0.5f
      ? parameters.morph + 0.5f
      : 1.0f - (parameters.morph - 0.5f) * 2.0f;
  saw_pw *= 1.1f;
  CONSTRAIN(saw_pw, 0.005f, 1.0f);
    
  float saw_shape = 10.0f - 21.0f * parameters.morph;
  CONSTRAIN(saw_shape, 0.0f, 1.0f);
  
  float saw_gain = 8.0f * (1.0f - parameters.morph);
  CONSTRAIN(saw_gain, 0.02f, 1.0f);
  
  const float square_sync_f = frequency * SemitonesToRatio(square_sync_ratio);
  
  sync_.RenderSyncSawSquare(
      primary_f, square_sync_f, square_pw, 1.0f, temp_buffer_, size);
  variable_saw_.Render(auxiliary_f, saw_pw, saw_shape, out, size);
  
  float norm = 1.0f / (std::max(square_gain, saw_gain));
  
  ParameterInterpolator square_gain_modulation(
      &auxiliary_amount_,
      square_gain * 0.3f * norm,
      size);

  ParameterInterpolator saw_gain_modulation(
      &xmod_amount_,
      saw_gain * 0.5f * norm,
      size);
  
  for (size_t i = 0; i < size; ++i) {
    out[i] = out[i] * saw_gain_modulation.Next() +
        square_gain_modulation.Next() * temp_buffer_[i];
  }

}

}  // namespace plaits
