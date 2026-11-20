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
// Additive synthesis with 32 partials.

#include "plaits/dsp/engine/additive_engine.h"

#include <algorithm>

#include "plaits/dsp/oscillator/sine_oscillator.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void AdditiveEngine::Init(BufferAllocator* allocator) {
  amplitudes_ = allocator->Allocate<float>(kNumHarmonics);
  for (int i = 0; i < kNumHarmonicOscillators; ++i) {
    harmonic_oscillator_[i].Init();
  }
}

void AdditiveEngine::Reset() {
  fill(
      &amplitudes_[0],
      &amplitudes_[kNumHarmonics],
      0.0f);
}

template<bool contiguous>
void AdditiveEngine::UpdateAmplitudes(
    float centroid,
    float slope,
    float bumps,
    float* amplitudes,
    const int* harmonic_indices,
    size_t num_harmonics) {
  const float n = (static_cast<float>(num_harmonics) - 1.0f);
  const float margin = (1.0f / slope - 1.0f) / (1.0f + bumps);
  const float center = centroid * (n + margin) - 0.5f * margin;

  // Positive gains form one contiguous window. Find its edges once, so
  // the sine loop does not need an extra branch for every harmonic.
  size_t first = 0;
  size_t last = num_harmonics;
  while (first < last &&
      fabsf(static_cast<float>(first) - center) * slope >= 1.0f) ++first;
  while (last > first &&
      fabsf(static_cast<float>(last - 1) - center) * slope >= 1.0f) --last;

  float sum = 0.001f;
  // Preserve the original accumulation order and the decay of old values.
  for (size_t i = 0; i < first; ++i) {
    const int j = contiguous ? static_cast<int>(i) : harmonic_indices[i];
    ONE_POLE(amplitudes[j], 0.0f, 0.001f);
    sum += amplitudes[j];
  }

  for (size_t i = first; i < last; ++i) {
    float order = fabsf(static_cast<float>(i) - center) * slope;
    float gain = 1.0f - order;
    gain += fabsf(gain);
    gain *= gain;

    float b = 0.25f + order * bumps;
    float bump_factor = 1.0f + SinePositiveWrap(b);

    gain *= bump_factor;
    gain *= gain;
    gain *= gain;
    
    const int j = contiguous ? static_cast<int>(i) : harmonic_indices[i];
    
    // Warning about the following line: this is not a proper LP filter because
    // of the normalization. But in spite of its strange working, this line
    // turns out to be absolutely essential.
    //
    // I have tried both normalizing the LP-ed spectrum, and LP-ing the
    // normalized spectrum, and both of them cause more annoyances than this
    // "incorrect" solution.
    
    ONE_POLE(amplitudes[j], gain, 0.001f);
    sum += amplitudes[j];
  }

  for (size_t i = last; i < num_harmonics; ++i) {
    const int j = contiguous ? static_cast<int>(i) : harmonic_indices[i];
    ONE_POLE(amplitudes[j], 0.0f, 0.001f);
    sum += amplitudes[j];
  }

  sum = 1.0f / sum;

  for (size_t i = 0; i < num_harmonics; ++i) {
    const int j = contiguous ? static_cast<int>(i) : harmonic_indices[i];
    amplitudes[j] *= sum;
  }
}

inline float Bump(float x, float centroid, float slope) {
  float d = fabsf(x - centroid);
  float bump = 1.0f - d * slope;
  return bump + fabsf(bump);
}

const int organ_harmonics[8] = {
  0, 1, 2, 3, 5, 7, 9, 11
};

void AdditiveEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note), out, aux, size);
}

void AdditiveEngine::RenderAtFrequency(
    const EngineParameters& parameters, float f0,
    float* out, float* aux, size_t size, EngineOutput output) {

  const float centroid = parameters.timbre;
  const float raw_bumps = parameters.harmonics;
  const float raw_slope = (1.0f - 0.6f * raw_bumps) * parameters.morph;
  const float slope = 0.01f + 1.99f * raw_slope * raw_slope * raw_slope;
  const float bumps = 16.0f * raw_bumps * raw_bumps;
  UpdateAmplitudes<true>(
      centroid,
      slope,
      bumps,
      &amplitudes_[0],
      nullptr,
      24);
  if (output != ENGINE_OUTPUT_AUX) {
    if (size <= kMaxBlockSize) {
      HarmonicOscillator<kHarmonicBatchSize>::Fundamental fundamental[kMaxBlockSize];
      harmonic_oscillator_[0].Render<
          1, true, 0xffffffffu, HarmonicFundamentalMode::STORE>(
          f0, &amplitudes_[0], out, size, nullptr, fundamental);
      harmonic_oscillator_[1].Render<13, true, 0xffffffffu, HarmonicFundamentalMode::READ>(
          f0, &amplitudes_[12], out, size, &harmonic_oscillator_[0], fundamental);
    } else {
      // Keep the historical API valid for callers with larger blocks.
      harmonic_oscillator_[0].Render<1>(f0, &amplitudes_[0], out, size);
      harmonic_oscillator_[1].Render<13>(f0, &amplitudes_[12], out, size);
    }
  }

  UpdateAmplitudes<false>(
      centroid,
      slope,
      bumps,
      &amplitudes_[24],
      organ_harmonics,
      8);

  // Eight nonzero organ slots: 0,1,2,3,5,7,9,11. Reset keeps the other
  // four slots exactly zero; skip their interpolation/multiply-add only.
  if (output == ENGINE_OUTPUT_BOTH) {
    harmonic_oscillator_[2].Render<1, true, 0xaaf>(f0, &amplitudes_[24], aux, size);
  } else if (output == ENGINE_OUTPUT_AUX) {
    harmonic_oscillator_[2].Render<1, true, 0xaaf>(f0, &amplitudes_[24], out, size);
    // All three banks start together and always receive the same f0/block.
    // Copy the completed audible clock, preserving each bank's amplitudes.
    harmonic_oscillator_[0].Render<1, false>(
        f0, &amplitudes_[0], nullptr, size, &harmonic_oscillator_[2]);
    harmonic_oscillator_[1].Render<13, false>(
        f0, &amplitudes_[12], nullptr, size, &harmonic_oscillator_[2]);
  } else {
    harmonic_oscillator_[2].Render<1, false, 0xaaf>(
        f0, &amplitudes_[24], nullptr, size, &harmonic_oscillator_[0]);
  }
}

}  // namespace plaits
