// Copyright 2021 Emilie Gillet.
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
// Virtual analog with VCF.

#include "plaits/dsp/engine2/virtual_analog_vcf_engine.h"

#include <algorithm>

#include "stmlib/dsp/parameter_interpolator.h"

using namespace std;

namespace plaits {

using namespace std;
using namespace stmlib;

namespace {

// Specialize the final output loop; oscillator rendering and parameter
// preparation are shared. Both filters advance in every output mode.
template<EngineOutput output>
void FilterBlock(
    Svf* svf, float* out, float* aux, size_t size,
    ParameterInterpolator& sub_gain_modulation,
    ParameterInterpolator& cutoff_modulation,
    ParameterInterpolator& stage2_gain_modulation,
    ParameterInterpolator& q_modulation,
    ParameterInterpolator& gain_modulation) {
  // Local filter copies let the compiler keep state in registers while
  // writing audio. Save their full history at the end of the block.
  Svf local_svf[2];
  local_svf[0].CopyFrom(svf[0]);
  local_svf[1].CopyFrom(svf[1]);

  for (size_t i = 0; i < size; ++i) {
    const float cutoff = min(cutoff_modulation.Next(), 0.25f);
    const float q = q_modulation.Next();
    const float stage2_gain = stage2_gain_modulation.Next();

    local_svf[0].set_f_q<FREQUENCY_FAST>(cutoff, 0.5f + q);
    local_svf[1].set_f_q<FREQUENCY_FAST>(cutoff, 0.5f + 0.025f * q);

    const float gain = gain_modulation.Next();
    const float input = SoftClip(
        (out[i] + aux[i] * sub_gain_modulation.Next()) * gain);

    float lp, hp;
    local_svf[0].Process<FILTER_MODE_LOW_PASS, FILTER_MODE_HIGH_PASS>(input, &lp, &hp);
    lp = SoftClip(lp * gain);
    // Run the second filter in AUX too, so its history is ready on return
    // to MAIN. Its final clip and mix have no state and can be skipped.
    const float stage2 = local_svf[1].Process<FILTER_MODE_LOW_PASS>(lp);
    if constexpr (output != ENGINE_OUTPUT_AUX) {
      lp += stage2_gain * (SoftClip(stage2) - lp);
      out[i] = lp;
    }
    if constexpr (output != ENGINE_OUTPUT_MAIN) {
      const float value = SoftClip(hp * gain);
      if constexpr (output == ENGINE_OUTPUT_AUX) {
        out[i] = value;
      } else {
        aux[i] = value;
      }
    }
  }

  svf[0].CopyFrom(local_svf[0]);
  svf[1].CopyFrom(local_svf[1]);
}

}  // namespace

void VirtualAnalogVCFEngine::Init(BufferAllocator* allocator) {
  oscillator_.Init();
  sub_oscillator_.Init();
  
  svf_[0].Init();
  svf_[1].Init();
  
  previous_sub_gain_ = 0.0f;
  previous_cutoff_ = 0.0f;
  previous_stage2_gain_ = 0.0f;
  previous_q_ = 0.0f;
  previous_gain_ = 0.0f;
}

void VirtualAnalogVCFEngine::Reset() {
  
}

void VirtualAnalogVCFEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note), out, aux, size);
}

void VirtualAnalogVCFEngine::RenderAtFrequency(
    const EngineParameters& parameters,
    float f0,
    float* out,
    float* aux,
    size_t size) {
  RenderAtFrequency(parameters, f0, out, aux, size, ENGINE_OUTPUT_BOTH);
}

void VirtualAnalogVCFEngine::RenderAtFrequency(
    const EngineParameters& parameters,
    float f0,
    float* out,
    float* aux,
    size_t size,
    EngineOutput output) {
  // VA Oscillator (saw or PW square) + sub

  float shape = (parameters.morph - 0.25f) * 2.0f + 0.5f;
  CONSTRAIN(shape, 0.5f, 1.0f);

  float pw = (parameters.morph - 0.5f) * 2.0f + 0.5f;
  if (parameters.morph > 0.75f) {
    pw = 2.5f - parameters.morph * 2.0f;
  }
  CONSTRAIN(pw, 0.5f, 0.98f);
  
  float sub_gain = max(fabsf(parameters.morph - 0.5f) - 0.3f, 0.0f) * 5.0f;

  oscillator_.RenderVcf(f0, pw, shape, out, size);
  sub_oscillator_.RenderVcf<true>(f0 * 0.501f, 0.5f, 1.0f, aux, size);
  
  const float cutoff = f0 * SemitonesToRatio(
      (parameters.timbre - 0.2f) * 120.0f);

  float stage2_gain = 1.0f - (parameters.harmonics - 0.4f) * 4.0f;
  CONSTRAIN(stage2_gain, 0.0f, 1.0f);
  
  const float resonance = 2.667f * \
      max(fabsf(parameters.harmonics - 0.5f) - 0.125f, 0.0f);
  const float resonance_sqr = resonance * resonance;
  const float q = resonance_sqr * resonance_sqr * 48.0f;
  float gain = (parameters.harmonics - 0.7f) + 0.85f;
  CONSTRAIN(gain, 0.7f - resonance_sqr * 0.3f, 1.0f);

  ParameterInterpolator sub_gain_modulation(
      &previous_sub_gain_, sub_gain, size);
  ParameterInterpolator cutoff_modulation(
      &previous_cutoff_, cutoff, size);
  ParameterInterpolator stage2_gain_modulation(
      &previous_stage2_gain_, stage2_gain, size);
  ParameterInterpolator q_modulation(
      &previous_q_, q, size);
  ParameterInterpolator gain_modulation(&previous_gain_, gain, size);
  
  // One dispatch per block; the chosen loop has no output-selection branch.
  if (output == ENGINE_OUTPUT_MAIN) {
    FilterBlock<ENGINE_OUTPUT_MAIN>(svf_, out, aux, size, sub_gain_modulation,
        cutoff_modulation, stage2_gain_modulation, q_modulation, gain_modulation);
  } else if (output == ENGINE_OUTPUT_AUX) {
    FilterBlock<ENGINE_OUTPUT_AUX>(svf_, out, aux, size, sub_gain_modulation,
        cutoff_modulation, stage2_gain_modulation, q_modulation, gain_modulation);
  } else {
    FilterBlock<ENGINE_OUTPUT_BOTH>(svf_, out, aux, size, sub_gain_modulation,
        cutoff_modulation, stage2_gain_modulation, q_modulation, gain_modulation);
  }
}

}  // namespace plaits
