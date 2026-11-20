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
// Three voices of string synthesis.

#include "plaits/dsp/engine/string_engine.h"

#include <algorithm>

namespace plaits {

using namespace std;
using namespace stmlib;

void StringEngine::Init(BufferAllocator* allocator) {
  random_.Seed(0x21u);
  voice_.Init(allocator);
}

void StringEngine::Reset() { voice_.Reset(); }

void StringEngine::Render(const EngineParameters& parameters,
    float* out, float* aux, size_t size, bool* already_enveloped) {
  // Compatibility entry point; SMN calls the fixed continuous path directly.
  if (size != kStringBlockSize) { fill_n(out, size, 0.0f); return; }
  RenderContinuousMain(NoteToFrequency(parameters.note), parameters.harmonics,
                       parameters.timbre, parameters.morph, out);
}

void StringEngine::RenderContinuousMain(float frequency, float harmonics,
    float timbre, float morph, float* out) {
  // The old active string was index 2. Strings 0/1 were never excited and
  // had fixed f0=.01 (normal cadence): 16 dispersion draws each, before the
  // active excitation. Preserve their effect on the shared RNG exactly.
  if (harmonics > 0.26f) random_.Seed(AdvanceStringRandom<32>(random_.state()));
  voice_.RenderContinuous(frequency, harmonics, timbre * timbre, morph,
                          out, &random_);
}

}  // namespace plaits
