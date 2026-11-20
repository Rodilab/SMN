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

#include "plaits/dsp/engine/wavetable_engine.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "plaits/resources.h"

namespace plaits {

using namespace std;
using namespace stmlib;

const int kNumBanks = 4;
const int kNumWavesPerBank = 64;
const int kNumWaves = 192;
const size_t kTableSize = 128;
const size_t kCoefficientsPerSample = 4;
// A known bound also makes pointer arithmetic a C++17 constant expression
// for Clang, so the map never needs a startup constructor.
extern const int16_t wav_hermite_coefficients[
    kNumWaves * kTableSize * kCoefficientsPerSample];
// The factory fourth bank is a deterministic shuffle of the same 192 waves.
// Build its 256 pointers at compile time, once in flash, never in voice RAM.
constexpr auto MakeWaveMap() {
  std::array<const int16_t*, kNumBanks * kNumWavesPerBank> map = {};
  for (int i = 0; i < kNumBanks * kNumWavesPerBank; ++i) {
    const int w = i < kNumWaves ? i : i * 101 % kNumWaves;
    map[i] = wav_hermite_coefficients +
        w * kTableSize * kCoefficientsPerSample;
  }
  return map;
}
constexpr auto wave_map = MakeWaveMap();

constexpr float kReferenceSampleRate = 48000.0f;

// Retain the elapsed-time response of the original 12-frame / 48 kHz block.
inline float PreCoefficient(float coefficient, size_t size) {
  return 1.0f - powf(1.0f - coefficient,
      float(size) * (kReferenceSampleRate / (12.0f * kSampleRate)));
}

const float kTableSizeF = float(kTableSize);

void WavetableEngine::Init(BufferAllocator* allocator) {
  phase_ = 0.0f;

  x_lp_ = 0.0f;
  y_lp_ = 0.0f;
  z_lp_ = 0.0f;
  
  x_pre_lp_ = 0.0f;
  y_pre_lp_ = 0.0f;
  z_pre_lp_ = 0.0f;

  previous_x_ = 0.0f;
  previous_y_ = 0.0f;
  previous_z_ = 0.0f;
  previous_f0_ = a0;

  diff_out_.Init();
  
  xy_pre_coefficient_ = PreCoefficient(0.2f, kNativeBlockSize);
  z_pre_coefficient_ = PreCoefficient(0.05f, kNativeBlockSize);
  previous_lp_coefficient_48k_ = -1.0f;
  lp_coefficient_ = 0.0f;
}

void WavetableEngine::Reset() {
}

inline float WavetableEngine::ReadWave(
    int x,
    int y,
    int z,
    int phase_integral,
    float phase_fractional) {
  // Half-integer coefficients are stored doubled, exactly, in application flash.
  const int16_t* table = wave_map[x + y * 8 + z * kNumWavesPerBank] +
      phase_integral * kCoefficientsPerSample;
  const float a = static_cast<float>(table[0]) * 0.5f;
  const float b = static_cast<float>(table[1]) * 0.5f;
  const float c = static_cast<float>(table[2]) * 0.5f;
  const float d = static_cast<float>(table[3]);
  const float f = phase_fractional;
  return (((a * f) - b) * f + c) * f + d;
}

void WavetableEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note),
      out, aux, size, ENGINE_OUTPUT_BOTH);
}

void WavetableEngine::RenderAtFrequency(
    const EngineParameters& parameters, float frequency,
    float* out, float* aux, size_t size, EngineOutput output) {
  if (!size) return;
  if (output == ENGINE_OUTPUT_MAIN) {
    RenderBlock<ENGINE_OUTPUT_MAIN>(parameters, frequency, out, aux, size);
  } else if (output == ENGINE_OUTPUT_AUX) {
    RenderBlock<ENGINE_OUTPUT_AUX>(parameters, frequency, out, aux, size);
  } else {
    RenderBlock<ENGINE_OUTPUT_BOTH>(parameters, frequency, out, aux, size);
  }
}

template<EngineOutput output>
void WavetableEngine::RenderBlock(
    const EngineParameters& parameters, float f0,
    float* out, float* aux, size_t size) {
  const float xy_pre_coefficient = size == kNativeBlockSize
      ? xy_pre_coefficient_ : PreCoefficient(0.2f, size);
  const float z_pre_coefficient = size == kNativeBlockSize
      ? z_pre_coefficient_ : PreCoefficient(0.05f, size);
  ONE_POLE(x_pre_lp_, parameters.timbre * 6.9999f, xy_pre_coefficient);
  ONE_POLE(y_pre_lp_, parameters.morph * 6.9999f, xy_pre_coefficient);
  // Expand original H 0..0.5 to SMN 0..1; remove quantization throughout.
  // z=3..3.49995 stays on the fourth bank, as in the original first half.
  ONE_POLE(z_pre_lp_, parameters.harmonics * 3.49995f, z_pre_coefficient);

  // The note-dependent filter is evaluated at the equivalent 48 kHz frequency
  // then its pole is mapped to 44.1 kHz. Reuse the exact result when unchanged.
  const float coefficient_48k = min(max(
      8.0f * f0 * (kSampleRate / kReferenceSampleRate), 0.01f), 0.1f);
  if (coefficient_48k != previous_lp_coefficient_48k_) {
    lp_coefficient_ = 1.0f - powf(1.0f - coefficient_48k,
        kReferenceSampleRate / kSampleRate);
    previous_lp_coefficient_48k_ = coefficient_48k;
  }
  const float lp_coefficient = lp_coefficient_;
  const float step = 1.0f / float(size);
  ParameterInterpolator x_modulation(&previous_x_, x_pre_lp_, step);
  ParameterInterpolator y_modulation(&previous_y_, y_pre_lp_, step);
  ParameterInterpolator z_modulation(&previous_z_, z_pre_lp_, step);
  ParameterInterpolator f0_modulation(&previous_f0_, f0, step);

  // Work on local copies during the block, then save the final state.
  // This keeps the same sample calculations and interpolation history.
  float phase = phase_;
  float x_lp = x_lp_, y_lp = y_lp_, z_lp = z_lp_;
  Differentiator diff_out;
  diff_out.CopyFrom(diff_out_);
  while (size--) {
    const float f0 = f0_modulation.Next();
    
    const float gain = (1.0f / (f0 * 131072.0f)) * (0.95f - f0);
    const float cutoff = min(kTableSizeF * f0, 1.0f);
    
    ONE_POLE(x_lp, x_modulation.Next(), lp_coefficient);
    ONE_POLE(y_lp, y_modulation.Next(), lp_coefficient);
    ONE_POLE(z_lp, z_modulation.Next(), lp_coefficient);
    
    const float x = x_lp;
    const float y = y_lp;
    const float z = z_lp;

    MAKE_INTEGRAL_FRACTIONAL(x);
    MAKE_INTEGRAL_FRACTIONAL(y);
    MAKE_INTEGRAL_FRACTIONAL(z);

    phase += f0;
    if (phase >= 1.0f) {
      phase -= 1.0f;
    }
    
    const float p = phase * kTableSizeF;
    MAKE_INTEGRAL_FRACTIONAL(p);
    
    {
      int x0 = x_integral;
      int x1 = x_integral + 1;
      int y0 = y_integral;
      int y1 = y_integral + 1;
      int z0 = z_integral;
      int z1 = z_integral + 1;
      
      if (z0 >= 4) {
        z0 = 7 - z0;
      }
      if (z1 >= 4) {
        z1 = 7 - z1;
      }
      
      float x0y0z0 = ReadWave(x0, y0, z0, p_integral, p_fractional);
      float x1y0z0 = ReadWave(x1, y0, z0, p_integral, p_fractional);
      float xy0z0 = x0y0z0 + (x1y0z0 - x0y0z0) * x_fractional;

      float x0y1z0 = ReadWave(x0, y1, z0, p_integral, p_fractional);
      float x1y1z0 = ReadWave(x1, y1, z0, p_integral, p_fractional);
      float xy1z0 = x0y1z0 + (x1y1z0 - x0y1z0) * x_fractional;

      float xyz0 = xy0z0 + (xy1z0 - xy0z0) * y_fractional;

      // At the last bank both sides point to the same four waveforms.
      // Reuse the identical interpolation, without reducing table resolution.
      float xyz1 = xyz0;
      if (z1 != z0) {
        float x0y0z1 = ReadWave(x0, y0, z1, p_integral, p_fractional);
        float x1y0z1 = ReadWave(x1, y0, z1, p_integral, p_fractional);
        float xy0z1 = x0y0z1 + (x1y0z1 - x0y0z1) * x_fractional;

        float x0y1z1 = ReadWave(x0, y1, z1, p_integral, p_fractional);
        float x1y1z1 = ReadWave(x1, y1, z1, p_integral, p_fractional);
        float xy1z1 = x0y1z1 + (x1y1z1 - x0y1z1) * x_fractional;
      
        xyz1 = xy0z1 + (xy1z1 - xy0z1) * y_fractional;
      }

      float mix = xyz0 + (xyz1 - xyz0) * z_fractional;
      mix = diff_out.Process(cutoff, mix) * gain;
      if constexpr (output != ENGINE_OUTPUT_AUX) {
        *out++ = mix;
      }
      if constexpr (output != ENGINE_OUTPUT_MAIN) {
        float* destination = output == ENGINE_OUTPUT_AUX ? out++ : aux++;
        *destination = static_cast<float>(static_cast<int>(mix * 32.0f)) / 32.0f;
      }
    }
  }
  phase_ = phase;
  x_lp_ = x_lp;
  y_lp_ = y_lp;
  z_lp_ = z_lp;
  diff_out_.CopyFrom(diff_out);
}

}  // namespace plaits
