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
// One voice of modal synthesis.

#include "plaits/dsp/engine/modal_engine.h"

#include <algorithm>

namespace plaits {

using namespace std;
using namespace stmlib;

namespace {
// 1 - (1 - 0.01)^(frames * 48000 / (12 * 44100)).
// Preserve the original 12-frame/48 kHz HARMONICS smoothing duration.
const float kHarmonicsSmoothing[kMaxBlockSize + 1] = {
    0.0f, 0.0009111797122f, 0.001821529176f, 0.002731049148f, 0.003639740383f,
    0.004547603638f, 0.005454639666f, 0.006360849221f, 0.007266233056f, 0.008170791924f,
    0.009074526577f, 0.009977437764f, 0.01087952624f, 0.01178079275f, 0.01268123804f,
    0.01358086286f, 0.01447966797f, 0.0153776541f, 0.01627482201f, 0.01717117243f,
    0.01806670612f, 0.01896142382f, 0.01985532626f, 0.02074841421f, 0.02164068838f,
};
}  // namespace


void ModalEngine::Init(BufferAllocator* allocator) {
  temp_buffer_ = allocator->Allocate<float>(kMaxBlockSize);
  harmonics_lp_ = 0.0f;
  Reset();
}

void ModalEngine::Reset() {
  voice_.Init();
}

void ModalEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note),
      out, aux, size);
}

void ModalEngine::RenderAtFrequency(
    const EngineParameters& parameters, float frequency,
    float* out, float* aux, size_t size, EngineOutput output) {
  // Each selected path writes its first result directly; no zero-fill pass.
  
  ONE_POLE(harmonics_lp_, parameters.harmonics, kHarmonicsSmoothing[size]);
  
  voice_.Render(
      parameters.trigger & TRIGGER_UNPATCHED,
      parameters.trigger & TRIGGER_RISING_EDGE,
      parameters.accent,
      frequency,
      harmonics_lp_,
      parameters.timbre,
      parameters.morph,
      temp_buffer_,
      out,
      aux,
      size, output);
}

void ModalEngine::RenderExternal(float frequency, float harmonics, float morph,
    const float* input, float* out, size_t size) {
  ONE_POLE(harmonics_lp_, harmonics, kHarmonicsSmoothing[size]);
  voice_.RenderExternal(frequency, harmonics_lp_, morph, input, out, size);
}

}  // namespace plaits
