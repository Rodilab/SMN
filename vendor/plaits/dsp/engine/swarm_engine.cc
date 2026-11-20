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
// Swarm of sawtooths and sines.

#include "plaits/dsp/engine/swarm_engine.h"

#include <algorithm>

namespace plaits {

using namespace std;
using namespace stmlib;

float GrainEnvelope::transition_coefficients_[kTransitionTableSize];
bool GrainEnvelope::transition_table_ready_;

void GrainEnvelope::PrepareTransitionTable() {
  if (transition_table_ready_) return;
  float filter = 0.5f;
  for (size_t i = 0; i < kTransitionTableSize; ++i) {
    filter *= kFilterDecay[kTransitionBlockSize];
    const float retained = 0.5f + filter;
    transition_coefficients_[i] = retained == 0.5f
        ? kAmplitudeCoefficient[kTransitionBlockSize]
        : 1.0f - powf(retained, float(kTransitionBlockSize) *
            (48000.0f / (12.0f * kSampleRate)));
  }
  transition_table_ready_ = true;
}

void SwarmEngine::Init(BufferAllocator* allocator) {
  GrainEnvelope::PrepareTransitionTable();
  swarm_voice_ = allocator->Allocate<SwarmVoice>(kNumSwarmVoices);
  random_.Seed(0x53776172u);
  Reset();
}

void SwarmEngine::Reset() {
  const float n = (kNumSwarmVoices - 1) / 2;
  for (int i = 0; i < kNumSwarmVoices; ++i) {
    float rank = (static_cast<float>(i) - n) / n;
    swarm_voice_[i].Init(rank);
  }
}

void SwarmEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note),
      out, aux, size, ENGINE_OUTPUT_BOTH);
}

void SwarmEngine::RenderAtFrequency(const EngineParameters& parameters,
    float frequency, float* out, float* aux, size_t size, EngineOutput output) {
  if (output == ENGINE_OUTPUT_MAIN)
    RenderBlock<ENGINE_OUTPUT_MAIN>(parameters, frequency, out, aux, size);
  else if (output == ENGINE_OUTPUT_AUX)
    RenderBlock<ENGINE_OUTPUT_AUX>(parameters, frequency, out, aux, size);
  else
    RenderBlock<ENGINE_OUTPUT_BOTH>(parameters, frequency, out, aux, size);
}

template<EngineOutput output>
void SwarmEngine::RenderBlock(const EngineParameters& parameters, float f0,
    float* out, float* aux, size_t size) {
  const float control_rate = static_cast<float>(size);
  const float density = NoteToFrequency(parameters.timbre * 120.0f) * \
      0.025f * control_rate;
  const float spread = parameters.harmonics * parameters.harmonics * \
      parameters.harmonics;
  float size_ratio = 0.25f * SemitonesToRatio(
      (1.0f - parameters.morph) * 84.0f);
  
  const bool burst_mode = !(parameters.trigger & TRIGGER_UNPATCHED);
  const bool start_burst = parameters.trigger & TRIGGER_RISING_EDGE;

  fill(&out[0], &out[size], 0.0f);
  if constexpr (output == ENGINE_OUTPUT_BOTH) fill(&aux[0], &aux[size], 0.0f);

  const float inverse_size = 1.0f / static_cast<float>(size);
  for (int i = 0; i < kNumSwarmVoices; ++i) {
    swarm_voice_[i].Render<output>(
        f0,
        density,
        burst_mode,
        start_burst,
        spread,
        size_ratio,
        &random_,
        out,
        aux,
        size,
        inverse_size);
    size_ratio *= 0.97f;
  }
}

}  // namespace plaits
