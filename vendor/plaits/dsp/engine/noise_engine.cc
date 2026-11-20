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

#include "plaits/dsp/engine/noise_engine.h"

#include "stmlib/dsp/parameter_interpolator.h"

namespace plaits {

using namespace std;
using namespace stmlib;

template<EngineOutput output>
void NoiseEngine::FilterBlock(float* out, float* aux, size_t size,
    ParameterInterpolator& f0_modulation,
    ParameterInterpolator& f1_modulation,
    ParameterInterpolator& q_modulation,
    ParameterInterpolator& mode_modulation) {
  for (size_t i = 0; i < size; ++i) {
    const float f0 = f0_modulation.Next();
    const float f1 = f1_modulation.Next();
    const float q = q_modulation.Next();
    const float gain = 1.0f / Sqrt((0.5f + q) * 40.0f * f0);
    primary_filter_.set_f_q<FREQUENCY_ACCURATE>(f0, q);
    secondary_filter_.set_f_q<FREQUENCY_ACCURATE>(f1, q);
    const float input_1 = aux[i] * gain;
    const float input_2 = temp_buffer_[i] * gain;
    const float second_bp = secondary_filter_.Process<FILTER_MODE_BAND_PASS>(input_2);
    const float first_bp = primary_filter_.ProcessMultimodeLPtoHPWithBandPass<
        output != ENGINE_OUTPUT_AUX>(input_1, mode_modulation.Next(), &out[i]);
    if constexpr (output == ENGINE_OUTPUT_AUX)
      out[i] = first_bp + second_bp;
    else if constexpr (output == ENGINE_OUTPUT_BOTH)
      aux[i] = first_bp + second_bp;
  }
}

void NoiseEngine::Init(BufferAllocator* allocator) {
  clocked_noise_[0].Init();
  clocked_noise_[1].Init();
  primary_filter_.Init();
  secondary_filter_.Init();
  random_.Seed(0x21u);

  previous_f0_ = 0.0f;
  previous_f1_ = 0.0f;
  previous_q_ = 0.0f;
  previous_mode_ = 0.0f;

  temp_buffer_ = allocator->Allocate<float>(kMaxBlockSize);
}

void NoiseEngine::Reset() { }

void NoiseEngine::Render(
    const EngineParameters& parameters,
    float* out, float* aux, size_t size, bool* already_enveloped) {
  // Retain the original note API and its note-domain saturation.
  RenderWithFrequencies(parameters, NoteToFrequency(parameters.note),
      NoteToFrequency(parameters.note + parameters.harmonics * 48.0f - 24.0f),
      out, aux, size, ENGINE_OUTPUT_BOTH);
}

void NoiseEngine::RenderAtFrequency(
    const EngineParameters& parameters, float f0,
    float* out, float* aux, size_t size, EngineOutput output) {
  // Match NoteToFrequency's upper bound even when HARMONICS adds two octaves.
  // No logarithm is needed: SMN already knows the base frequency.
  const float f1 = std::min(
      f0 * SemitonesToRatio(parameters.harmonics * 48.0f - 24.0f),
      NoteToFrequency(136.0f));
  RenderWithFrequencies(parameters, f0, f1, out, aux, size, output);
}

void NoiseEngine::RenderWithFrequencies(
    const EngineParameters& parameters, float f0, float f1,
    float* out, float* aux, size_t size, EngineOutput output) {
  const float clock_lowest_note = parameters.trigger & TRIGGER_UNPATCHED
      ? 0.0f : -24.0f;
  const float clock_f = NoteToFrequency(
      parameters.timbre * (128.0f - clock_lowest_note) + clock_lowest_note);
  const float q = 0.5f * SemitonesToRatio(parameters.morph * 120.0f);
  const bool sync = parameters.trigger & TRIGGER_RISING_EDGE;
  clocked_noise_[0].Render(sync, clock_f, aux, size, &random_);
  clocked_noise_[1].Render(sync, clock_f * f1 / f0, temp_buffer_, size, &random_);

  ParameterInterpolator f0_modulation(&previous_f0_, f0, size);
  ParameterInterpolator f1_modulation(&previous_f1_, f1, size);
  ParameterInterpolator q_modulation(&previous_q_, q, size);
  ParameterInterpolator mode_modulation(&previous_mode_, parameters.harmonics, size);

  // Dispatch once per block. Both noise generators and both distinct filter
  // histories always advance, so switching outputs does not freeze their tails.
  if (output == ENGINE_OUTPUT_MAIN) {
    FilterBlock<ENGINE_OUTPUT_MAIN>(out, aux, size, f0_modulation,
        f1_modulation, q_modulation, mode_modulation);
  } else if (output == ENGINE_OUTPUT_AUX) {
    FilterBlock<ENGINE_OUTPUT_AUX>(out, aux, size, f0_modulation,
        f1_modulation, q_modulation, mode_modulation);
  } else {
    FilterBlock<ENGINE_OUTPUT_BOTH>(out, aux, size, f0_modulation,
        f1_modulation, q_modulation, mode_modulation);
  }
}

}  // namespace plaits
