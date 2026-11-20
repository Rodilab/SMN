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
// Windowed sine segments.

#include <algorithm>

#include "plaits/dsp/engine/grain_engine.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void GrainEngine::Init(BufferAllocator* allocator) {
  grainlet_[0].Init();
  grainlet_[1].Init();
  z_oscillator_.Init();
  dc_blocker_[0].Init();
  dc_blocker_[1].Init();
}

void GrainEngine::Reset() {
  
}

void GrainEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderWithFrequencies(parameters, NoteToFrequency(parameters.note),
      NoteToFrequency(parameters.note + 96.0f * parameters.timbre),
      out, aux, size, ENGINE_OUTPUT_BOTH);
}

void GrainEngine::RenderAtFrequency(
    const EngineParameters& parameters, float frequency,
    float* out, float* aux, size_t size, EngineOutput output) {
  // AUX's relative cutoff retains the note converter's upper bound.
  const float cutoff = output == ENGINE_OUTPUT_MAIN ? 0.0f : min(
      frequency * SemitonesToRatio(96.0f * parameters.timbre),
      NoteToFrequency(136.0f));
  RenderWithFrequencies(parameters, frequency, cutoff, out, aux, size, output);
}

void GrainEngine::RenderWithFrequencies(
    const EngineParameters& parameters, float f0, float cutoff,
    float* out, float* aux, size_t size, EngineOutput output) {
  if (output != ENGINE_OUTPUT_AUX) {
    const float f1 = NoteToFrequency(24.0f + 84.0f * parameters.timbre);
    const float ratio = SemitonesToRatio(-24.0f + 48.0f * parameters.harmonics);
    const float carrier_bleed = parameters.harmonics < 0.5f
        ? 1.0f - 2.0f * parameters.harmonics
        : 0.0f;
    const float carrier_bleed_fixed = carrier_bleed * (2.0f - carrier_bleed);
    const float carrier_shape = 0.33f + (parameters.morph - 0.33f) *
        max(1.0f - f0 * 24.0f, 0.0f);

    // The second grainlet is part of MAIN, not the independently selected AUX.
    grainlet_[0].Render(f0, f1, carrier_shape, carrier_bleed_fixed, out, size);
    grainlet_[1].Render(f0, f1 * ratio, carrier_shape, carrier_bleed_fixed, aux, size);
    dc_blocker_[0].set_f<FREQUENCY_DIRTY>(0.3f * f0);
    for (size_t i = 0; i < size; ++i) {
      out[i] = dc_blocker_[0].Process<FILTER_MODE_HIGH_PASS>(out[i] + aux[i]);
    }
  }

  if (output != ENGINE_OUTPUT_MAIN) {
    float* destination = output == ENGINE_OUTPUT_AUX ? out : aux;
    z_oscillator_.Render(f0, cutoff, parameters.morph, parameters.harmonics,
        destination, size);
    dc_blocker_[1].set_f<FREQUENCY_DIRTY>(0.3f * f0);
    dc_blocker_[1].Process<FILTER_MODE_HIGH_PASS>(destination, size);
  }
}

}  // namespace plaits
