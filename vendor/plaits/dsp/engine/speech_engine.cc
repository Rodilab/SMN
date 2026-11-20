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
// Various flavours of speech synthesis.

#include "plaits/dsp/engine/speech_engine.h"

namespace plaits {
using namespace stmlib;

void SpeechEngine::Init(BufferAllocator* allocator) {
  naive_speech_synth_.Init();
  sam_speech_synth_.Init();
  lpc_speech_synth_controller_.Init();
  temp_buffer_[0] = allocator->Allocate<float>(kMaxBlockSize);
  temp_buffer_[1] = allocator->Allocate<float>(kMaxBlockSize);
}

void SpeechEngine::Reset() { }

void SpeechEngine::Render(const EngineParameters& parameters,
    float* out, float* aux, size_t size, bool* already_enveloped) {
  *already_enveloped = false;
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note), out, aux, size);
}

void SpeechEngine::RenderAtFrequency(const EngineParameters& parameters,
    float frequency, float* out, float* aux, size_t size, EngineOutput selected) {
  // Original group = H * 6. Half range expanded over the whole SMN control.
  const float group = parameters.harmonics * 3.0f;
  float* excitation = selected == ENGINE_OUTPUT_AUX ? out :
      selected == ENGINE_OUTPUT_MAIN ? temp_buffer_[0] : aux;
  if (group > 2.0f) {
    lpc_speech_synth_controller_.Render(frequency, parameters.morph,
        parameters.timbre, excitation, out, size, selected);
    return;
  }

  float blend = group;
  if (group <= 1.0f) {
    // MAIN still needs the excitation, but AUX omits the five-formant sum.
    naive_speech_synth_.Render(frequency, parameters.morph,
        parameters.timbre, temp_buffer_[0], excitation, out, size, selected);
  } else {
    lpc_speech_synth_controller_.Render(frequency, parameters.morph,
        parameters.timbre, excitation, out, size, selected);
    blend = 2.0f - blend;
  }
  sam_speech_synth_.Render(frequency, parameters.morph,
      parameters.timbre, temp_buffer_[0], temp_buffer_[1], size, selected);
  blend *= blend * (3.0f - 2.0f * blend);
  blend *= blend * (3.0f - 2.0f * blend);
  for (size_t i = 0; i < size; ++i) {
    if (selected != ENGINE_OUTPUT_MAIN)
      excitation[i] += (temp_buffer_[0][i] - excitation[i]) * blend;
    if (selected != ENGINE_OUTPUT_AUX)
      out[i] += (temp_buffer_[1][i] - out[i]) * blend;
  }
}
}  // namespace plaits
