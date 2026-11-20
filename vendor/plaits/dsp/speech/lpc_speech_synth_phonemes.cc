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
// This is a table with vowels (a, e, i, o, u) and a random selection of
// consonnants for the LPC10 speech synth.

#include "plaits/dsp/speech/lpc_speech_synth_controller.h"

namespace plaits {

/* static */
const LPCSpeechSynth::Frame
  LPCSpeechSynthController::phonemes_[kLPCSpeechSynthNumVowels] = {
  {
    192,
    80,
    -18368, 11584, 52, 29, 23,
    14, -17, 79, 37, 4
  },
  {
    192,
    80,
    -14528, 1536, 38, 29, 11,
    14, -41, 79, 57, 4
  },
  {
    192,
    80,
    14528, 9216, 25, -54, -70,
    36, 19, 79, 57, 22
  },
  {
    192,
    80,
    -14528, -13440, 38, 57, 57,
    14, -53, 7, 37, 77
  },
  {
    192,
    80,
    -26368, 4160, 11, 15, -1,
    36, -41, 31, 77, 22
  }
};

}  // namespace plaits
