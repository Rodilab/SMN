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

#ifndef PLAITS_DSP_ENGINE_SWARM_ENGINE_H_
#define PLAITS_DSP_ENGINE_SWARM_ENGINE_H_

#include "stmlib/dsp/polyblep.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/engine/engine.h"
#include "plaits/dsp/oscillator/oscillator.h"
#include "plaits/dsp/oscillator/sine_oscillator.h"
#include "plaits/resources.h"

namespace plaits {

const int kNumSwarmVoices = 8;

class GrainEnvelope {
 public:
  static constexpr size_t kTransitionBlockSize = 16;
  static constexpr size_t kTransitionTableSize = 224;

  // Call before audio starts. The table is shared, then only read by voices.
  static void PrepareTransitionTable();

  static constexpr float kFilterDecay[kMaxBlockSize + 1] = {
    1.0f, 0.9953583529f, 0.9907382507f, 0.9861395933f,
    0.9815622813f, 0.9770062156f, 0.9724712975f, 0.9679574289f,
    0.9634645121f, 0.9589924499f, 0.9545411453f, 0.9501105022f,
    0.9457004245f, 0.9413108169f, 0.9369415842f, 0.932592632f,
    0.9282638661f, 0.9239551929f, 0.9196665189f, 0.9153977515f,
    0.9111487981f, 0.9069195669f, 0.9027099664f, 0.8985199053f,
    0.8943492929f
  };
  static constexpr float kAmplitudeCoefficient[kMaxBlockSize + 1] = {
    0.0f, 0.06093491828f, 0.1181567723f, 0.1718918173f,
    0.2223525217f, 0.2697384073f, 0.3142368378f, 0.35602376f,
    0.3952643996f, 0.432113914f, 0.4667180062f, 0.4992135009f,
    0.5297288853f, 0.5583848172f, 0.5852946023f, 0.6105646418f,
    0.6342948536f, 0.6565790668f, 0.6775053933f, 0.6971565758f,
    0.7156103151f, 0.7329395773f, 0.7492128823f, 0.7644945748f,
    0.7788450787f
  };

  GrainEnvelope() { }
  ~GrainEnvelope() { }
  
  void Init() {
    from_ = 0.0f;
    interval_ = 1.0f;
    phase_ = 1.0f;
    fm_ = 0.0f;
    amplitude_ = 0.5f;
    previous_size_ratio_ = 0.0f;
    filter_coefficient_ = 0.0f;
    transition_index_ = kTransitionTableSize - 1;
  }
  
  inline void Step(float rate, bool burst_mode, bool start_burst,
                   stmlib::Random* random) {
    bool randomize = false;
    if (start_burst) {
      phase_ = 0.5f;
      fm_ = 16.0f;
      randomize = true;
    } else {
      phase_ += rate * fm_;
      if (phase_ >= 1.0f) {
        phase_ -= static_cast<float>(static_cast<int>(phase_));
        randomize = true;
      }
    }
    
    if (randomize) {
      from_ += interval_;
      interval_ = random->GetFloat() - from_;
      // Randomize the duration of the grain.
      if (burst_mode) {
        fm_ *= 0.8f + 0.2f * random->GetFloat();
      } else {
        fm_ = 0.5f + 1.5f * random->GetFloat();
      }
    }
  }
  
  inline float frequency(float size_ratio) const {
    // We approximate two overlapping grains of frequencies f1 and f2
    // By a continuous tone ramping from f1 to f2. This allows a continuous
    // transition between the "grain cloud" and "swarm of glissandi" textures.
    if (size_ratio < 1.0f) {
      return 2.0f * (from_ + interval_ * phase_) - 1.0f;
    } else {
      return from_;
    }
  }
  
  inline float amplitude(float size_ratio, size_t size) {
    float target_amplitude = 1.0f;
    if (size_ratio >= 1.0f) {
      float phase = (phase_ - 0.5f) * size_ratio;
      CONSTRAIN(phase, -1.0f, 1.0f);
      // The existing clamp bounds this positive phase to [0.75, 1.75].
      float e = SinePositiveWrap(0.5f * phase + 1.25f);
      target_amplitude = 0.5f * (e + 1.0f);
    }
    
    if ((size_ratio >= 1.0f) ^ (previous_size_ratio_ >= 1.0f)) {
      filter_coefficient_ = 0.5f;
      transition_index_ = 0;
    }
    // Original envelope is updated every 12 samples at 48 kHz. Preserve
    // elapsed-time decay at SMN's rate/block size. Tables are prepared offline.
    filter_coefficient_ *= kFilterDecay[size];
    
    previous_size_ratio_ = size_ratio;
    float coefficient;
    if (size == kTransitionBlockSize &&
        transition_index_ < kTransitionTableSize) {
      coefficient = transition_coefficients_[transition_index_];
      if (transition_index_ < kTransitionTableSize - 1) ++transition_index_;
    } else {
      // Preserve the generic API, including changes of block size mid-tail.
      // A non-native block invalidates the cached timeline until the next
      // native regime change, or until this coefficient has settled.
      transition_index_ = kTransitionTableSize;
      const float retained = 0.5f + filter_coefficient_;
      coefficient = retained == 0.5f ? kAmplitudeCoefficient[size]
          : 1.0f - powf(retained,
              float(size) * (48000.0f / (12.0f * kSampleRate)));
      if (size == kTransitionBlockSize && retained == 0.5f)
        transition_index_ = kTransitionTableSize - 1;
    }
    ONE_POLE(amplitude_, target_amplitude, coefficient);
    return amplitude_;
  }
  
 private:
  static float transition_coefficients_[kTransitionTableSize];
  static bool transition_table_ready_;

  float from_;
  float interval_;
  float phase_;
  float fm_;
  float amplitude_;
  float previous_size_ratio_;
  float filter_coefficient_;
  uint16_t transition_index_;
  
  DISALLOW_COPY_AND_ASSIGN(GrainEnvelope);
};

class AdditiveSawOscillator {
 public:
  AdditiveSawOscillator() { }
  ~AdditiveSawOscillator() { }

  inline void Init() {
    phase_ = 0.0f;
    next_sample_ = 0.0f;
    frequency_ = 0.01f;
    gain_ = 0.0f;
  }

  template<bool audible = true>
  inline void Render(
      float frequency,
      float level,
      float* out,
      size_t size) {
    Render<audible>(frequency, level, out, size,
        1.0f / static_cast<float>(size));
  }

  template<bool audible = true>
  inline void Render(
      float frequency,
      float level,
      float* out,
      size_t size,
      float inverse_size) {
    if (frequency >= kMaxFrequency) {
      frequency = kMaxFrequency;
    }
    stmlib::ParameterInterpolator fm(&frequency_, frequency, inverse_size);
    stmlib::ParameterInterpolator gain(&gain_, level, inverse_size);

    float next_sample = next_sample_;
    float phase = phase_;

    while (size--) {
      float this_sample = next_sample;
      next_sample = 0.0f;

      const float frequency = fm.Next();

      phase += frequency;
  
      if (phase >= 1.0f) {
        phase -= 1.0f;
        if constexpr (audible) {
          const float t = phase / frequency;
          this_sample -= stmlib::ThisBlepSample(t);
          next_sample -= stmlib::NextBlepSample(t);
        } else if (size == 0) {
          // Only the final pending sample is observable on a MAIN return.
          next_sample -= stmlib::NextBlepSample(phase / frequency);
        }
      }

      next_sample += phase;
      const float level = gain.Next();
      if constexpr (audible) *out++ += (2.0f * this_sample - 1.0f) * level;
    }
    phase_ = phase;
    next_sample_ = next_sample;
  }

 private:
  // Oscillator state.
  float phase_;
  float next_sample_;

  // For interpolation of parameters.
  float frequency_;
  float gain_;

  DISALLOW_COPY_AND_ASSIGN(AdditiveSawOscillator);
};

class SwarmVoice {
 public:
  SwarmVoice() { }
  ~SwarmVoice() { }
  
  void Init(float rank) {
    rank_ = rank;
    linear_rank_ = rank * (rank + 0.01f);
    envelope_.Init();
    saw_.Init();
    sine_.Init();
  }
  
  template<EngineOutput output>
  void Render(
      float f0,
      float density,
      bool burst_mode,
      bool start_burst,
      float spread,
      float size_ratio,
      stmlib::Random* random,
      float* saw,
      float* sine,
      size_t size,
      float inverse_size) {
    envelope_.Step(density, burst_mode, start_burst, random);
    
    const float scale = 1.0f / kNumSwarmVoices;
    const float amplitude = envelope_.amplitude(size_ratio, size) * scale;

    const float expo_amount = envelope_.frequency(size_ratio);
    f0 *= stmlib::SemitonesToRatio(48.0f * expo_amount * spread * rank_);
    
    const float linear_amount = linear_rank_ * spread * 0.25f;
    f0 *= 1.0f + linear_amount;

    // Keep the inactive oscillator's phase/interpolation in RAM until selected
    // again. Shared grain envelope and RNG still advance in both variants.
    if constexpr (output != ENGINE_OUTPUT_AUX) {
      saw_.Render(f0, amplitude, saw, size, inverse_size);
    }
    if constexpr (output != ENGINE_OUTPUT_MAIN) {
      sine_.RenderAdditive<true>(f0, amplitude,
          output == ENGINE_OUTPUT_AUX ? saw : sine, size, inverse_size);
    }
  };
  
 private:
  float rank_;
  float linear_rank_;

  GrainEnvelope envelope_;
  AdditiveSawOscillator saw_;
  FastSineOscillator sine_;
};

class SwarmEngine : public Engine {
 public:
  SwarmEngine() { }
  ~SwarmEngine() { }
  
  virtual void Init(stmlib::BufferAllocator* allocator);
  virtual void Reset();
  virtual void LoadUserData(const uint8_t* user_data) { }
  virtual void Render(const EngineParameters& parameters,
      float* out,
      float* aux,
      size_t size,
      bool* already_enveloped);
  
  void RenderAtFrequency(const EngineParameters& parameters, float frequency,
      float* out, float* aux, size_t size,
      EngineOutput output = ENGINE_OUTPUT_BOTH);
  void Seed(uint32_t seed) { random_.Seed(seed); }

 private:
  template<EngineOutput output>
  void RenderBlock(const EngineParameters& parameters, float f0,
      float* out, float* aux, size_t size);
  stmlib::Random random_;
  SwarmVoice* swarm_voice_;
  
  DISALLOW_COPY_AND_ASSIGN(SwarmEngine);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE_SWARM_ENGINE_H_
