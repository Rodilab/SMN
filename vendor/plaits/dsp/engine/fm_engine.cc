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
// Classic 2-op FM found in Braids, Rings and Elements.

#include "plaits/dsp/engine/fm_engine.h"

#include <algorithm>
#include <cmath>
#include "stmlib/dsp/parameter_interpolator.h"
#include "plaits/dsp/oscillator/sine_oscillator.h"
#include "plaits/dsp/downsampler/4x_downsampler.h"

namespace plaits {
using namespace stmlib;

// Immutable coefficients shared by all voices, derived from lut_sine.
extern const float fm_sine_coefficients[512][2];
extern const float fm_sub_sine[8192];

namespace {
inline float SubSinePM(uint32_t phase, float pm) {
  phase += static_cast<uint32_t>((pm + 32.0f) * 67108864.0f) * 64u;
  // Round to the nearest of 8192 points. Unsigned overflow wraps the cycle.
  return fm_sub_sine[(phase + (1u << 18)) >> 19];
}

inline float SinePMWithCoefficients(uint32_t phase, float pm) {
  // Keep the original modulation quantization and unsigned phase wrapping.
  phase += static_cast<uint32_t>((pm + 32.0f) * 67108864.0f) * 64u;
  const float* pair = fm_sine_coefficients[phase >> (32 - kSineLUTBits)];
  const float fraction = static_cast<float>(phase << kSineLUTBits);
  // The table already contains (next - sample) * 2^-32.
  return pair[0] + pair[1] * fraction;
}
}  // namespace

void FMEngine::Init(BufferAllocator* allocator) {
  carrier_phase_ = modulator_phase_ = sub_phase_ = 0;
  previous_carrier_frequency_ = previous_modulator_frequency_ = a0;
  previous_amount_ = previous_feedback_ = previous_sample_ = 0.0f;
  // Plaits relies on zero-initialized storage. Also support explicit re-init.
  sub_fir_ = carrier_fir_ = 0.0f;
  cached_frequency_ = 0.0f; // Invalid for a positive SMN frequency.
  cached_note_ = 0.0f;
}

void FMEngine::Reset() { }

void FMEngine::Render(const EngineParameters& parameters,
    float* out, float* aux, size_t size, bool* already_enveloped) {
  // Retain the historical note API for comparisons and non-SMN callers.
  const float note = parameters.note - 24.0f;
  const float ratio = Interpolate(lut_fm_frequency_quantizer,
      parameters.harmonics, 128.0f);
  const float modulator_note = note + ratio;
  float hf_taming = std::clamp(1.0f - (modulator_note - 72.0f) * 0.025f,
      0.0f, 1.0f);
  RenderPrepared<ENGINE_OUTPUT_BOTH>(parameters, NoteToFrequency(note),
      std::clamp(NoteToFrequency(modulator_note), 0.0f, 0.5f),
      hf_taming * hf_taming, out, aux, size);
}

void FMEngine::RenderAtFrequency(const EngineParameters& parameters,
    float frequency, float* out, float* aux, size_t size, EngineOutput output) {
  const float ratio = Interpolate(lut_fm_frequency_quantizer,
      parameters.harmonics, 128.0f);
  const float carrier_frequency = 0.25f * frequency;
  const float modulator_frequency = std::min(0.5f,
      carrier_frequency * SemitonesToRatio(ratio));
  // Taming is defined in semitones in Plaits. Only this control law needs
  // a logarithm; neither oscillator converts the known frequency back to Hz.
  if (frequency != cached_frequency_) {
    cached_note_ = 45.0f +
        12.0f * log2f(frequency * (kSampleRate / 440.0f));
    cached_frequency_ = frequency;
  }
  const float modulator_note = cached_note_ + ratio;
  float hf_taming = std::clamp(1.0f - (modulator_note - 72.0f) * 0.025f,
      0.0f, 1.0f);
  hf_taming *= hf_taming;
  if (output == ENGINE_OUTPUT_MAIN) {
    RenderPrepared<ENGINE_OUTPUT_MAIN>(parameters, carrier_frequency,
        modulator_frequency, hf_taming, out, aux, size);
  } else if (output == ENGINE_OUTPUT_AUX) {
    RenderPrepared<ENGINE_OUTPUT_AUX>(parameters, carrier_frequency,
        modulator_frequency, hf_taming, out, aux, size);
  } else {
    RenderPrepared<ENGINE_OUTPUT_BOTH>(parameters, carrier_frequency,
        modulator_frequency, hf_taming, out, aux, size);
  }
}

template<EngineOutput output>
void FMEngine::RenderPrepared(const EngineParameters& parameters,
    float target_carrier_frequency, float target_modulator_frequency,
    float hf_taming, float* out, float* aux, size_t size) {
  if (!size) return;
  const float reciprocal = 1.0f / static_cast<float>(size);
  ParameterInterpolator carrier_frequency(&previous_carrier_frequency_,
      target_carrier_frequency, reciprocal);
  ParameterInterpolator modulator_frequency(&previous_modulator_frequency_,
      target_modulator_frequency, reciprocal);
  ParameterInterpolator amount_modulation(&previous_amount_,
      2.0f * parameters.timbre * parameters.timbre * hf_taming, reciprocal);
  ParameterInterpolator feedback_modulation(&previous_feedback_,
      2.0f * parameters.morph - 1.0f, reciprocal);
  Downsampler carrier_downsampler(&carrier_fir_);
  Downsampler sub_downsampler(&sub_fir_);
  uint32_t carrier_phase = carrier_phase_;
  uint32_t modulator_phase = modulator_phase_;
  uint32_t sub_phase = sub_phase_;
  float previous_sample = previous_sample_;
  while (size--) {
    const float max_uint32 = 4294967296.0f;
    const float amount = amount_modulation.Next();
    const float feedback = feedback_modulation.Next();
    const float phase_feedback = feedback < 0.0f
        ? 0.5f * feedback * feedback : 0.0f;
    // Constant across all four oversampled positions.
    const float modulator_fb = feedback > 0.0f
        ? 0.25f * feedback * feedback : 0.0f;
    const uint32_t carrier_increment = static_cast<uint32_t>(
        max_uint32 * carrier_frequency.Next());
    const float f = modulator_frequency.Next();
    // Only the final sample's tail carries into the next block. Reconstruct
    // it for the unselected output too, so MAIN/AUX switches retain both
    // oscillator phases and FIR histories.
    const bool render_sub = output != ENGINE_OUTPUT_MAIN || size == 0;
    const bool filter_carrier = output != ENGINE_OUTPUT_AUX || size == 0;
    for (size_t j = 0; j < kOversampling; ++j) {
      modulator_phase += static_cast<uint32_t>(max_uint32 * f *
          (1.0f + previous_sample * phase_feedback));
      carrier_phase += carrier_increment;
      sub_phase += carrier_increment >> 1;
      const float modulator = SinePMWithCoefficients(modulator_phase,
          modulator_fb * previous_sample);
      const float carrier = SinePMWithCoefficients(carrier_phase, amount * modulator);
      if (render_sub) {
        const float sub = SubSinePM(sub_phase, amount * carrier * 0.25f);
        sub_downsampler.Accumulate(j, sub);
      }
      ONE_POLE(previous_sample, carrier, 0.0542995754822f);
      if (filter_carrier) carrier_downsampler.Accumulate(j, carrier);
    }
    if (output == ENGINE_OUTPUT_BOTH) {
      *out++ = carrier_downsampler.Read();
      *aux++ = sub_downsampler.Read();
    } else if (output == ENGINE_OUTPUT_MAIN) {
      *out++ = carrier_downsampler.Read();
      if (render_sub) sub_downsampler.Read();
    } else {
      *out++ = sub_downsampler.Read();
      if (filter_carrier) carrier_downsampler.Read();
    }
  }
  carrier_phase_ = carrier_phase;
  modulator_phase_ = modulator_phase;
  sub_phase_ = sub_phase;
  previous_sample_ = previous_sample;
}
}  // namespace plaits
