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
// Comb filter / KS string. "Lite" version of the implementation used in Rings.

#include "plaits/dsp/physical_modelling/string.h"

#include <cmath>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/dsp.h"
#include "plaits/resources.h"

namespace plaits {
  
using namespace std;
using namespace stmlib;

void String::Init(BufferAllocator* allocator) {
  string_.Init(allocator->Allocate<float>(kDelayLineSize));
  stretch_.Init(allocator->Allocate<float>(kDelayLineSize / 4));
  delay_ = 100.0f;
  Reset();
}

void String::Reset() {
  string_.Reset();
  stretch_.Reset();
  iir_damping_filter_.Init();
  dc_blocker_.Init(1.0f - 20.0f / kSampleRate);
  dispersion_noise_ = 0.0f;
  curved_bridge_ = 0.0f;
  out_sample_[0] = out_sample_[1] = 0.0f;
  src_phase_ = 0.0f;
}

void String::ProcessContinuous(float f0, float non_linearity_amount,
    float brightness, float damping, float* out, Svf* excitation_filter,
    float dust_f, Random* random) {
  if (non_linearity_amount <= 0.0f) {
    ProcessInternal<STRING_NON_LINEARITY_CURVED_BRIDGE>(f0, -non_linearity_amount,
        brightness, damping, out, excitation_filter, dust_f, random);
  } else {
    ProcessInternal<STRING_NON_LINEARITY_DISPERSION>(f0, non_linearity_amount,
        brightness, damping, out, excitation_filter, dust_f, random);
  }
}

void String::ProcessExternal(float f0, float non_linearity_amount,
    float brightness, float damping, const float* input, float* out, Random* random) {
  if (non_linearity_amount <= 0.0f) {
    ProcessInternal<STRING_NON_LINEARITY_CURVED_BRIDGE, true>(f0, -non_linearity_amount,
        brightness, damping, out, nullptr, 1.0f, random, input);
  } else {
    ProcessInternal<STRING_NON_LINEARITY_DISPERSION, true>(f0, non_linearity_amount,
        brightness, damping, out, nullptr, 1.0f, random, input);
  }
}

template<StringNonLinearity non_linearity, bool external>
void String::ProcessInternal(float f0, float non_linearity_amount,
    float brightness, float damping, float* out, Svf* excitation_filter,
    float dust_f, Random* random, const float* input) {
  float delay = 1.0f / f0;
  CONSTRAIN(delay, 4.0f, kDelayLineSize - 4.0f);
  
  // If there is not enough delay time in the delay line, we play at the
  // lowest possible note and we upsample on the fly with a linear
  // interpolator (below about 43.24 Hz at 44.1 kHz with 1020 samples).
  float src_ratio = delay * f0;
  if (src_ratio >= 0.9999f) {
    // At normal cadence, make sure that the linear interpolator
    // does not get in the way.
    src_phase_ = 1.0f;
    src_ratio = 1.0f;
  }

  float damping_cutoff = min(
      12.0f + damping * damping * 60.0f + brightness * 24.0f,
      84.0f);
  float damping_f = min(f0 * SemitonesToRatio(damping_cutoff), 0.499f);
  
  // Crossfade to infinite decay.
  if (damping >= 0.95f) {
    float to_infinite = 20.0f * (damping - 0.95f);
    brightness += to_infinite * (1.0f - brightness);
    damping_f += to_infinite * (0.4999f - damping_f);
    damping_cutoff += to_infinite * (128.0f - damping_cutoff);
  }
  
  iir_damping_filter_.set_f_q<FREQUENCY_FAST>(damping_f, 0.5f);
  
  float damping_compensation = Interpolate(lut_svf_shift, damping_cutoff, 1.0f);
  
  // Linearly interpolate delay time.
  ParameterInterpolator delay_modulation(
      &delay_, delay * damping_compensation, 1.0f / kStringBlockSize);
  
  float stretch_point = non_linearity_amount * (2.0f - non_linearity_amount) * 0.225f;
  float stretch_correction = (160.0f / kSampleRate) * delay;
  CONSTRAIN(stretch_correction, 1.0f, 2.1f);
  
  float noise_amount_sqrt = non_linearity_amount > 0.75f
      ? 4.0f * (non_linearity_amount - 0.75f)
      : 0.0f;
  float noise_amount = noise_amount_sqrt * noise_amount_sqrt * 0.1f;
  float noise_filter = 0.06f + 0.94f * brightness * brightness;
  
  float bridge_curving_sqrt = non_linearity_amount;
  float bridge_curving = bridge_curving_sqrt * bridge_curving_sqrt * 0.01f;
  
  float ap_gain = -0.618f * non_linearity_amount / (0.15f + fabsf(non_linearity_amount));
  
  Svf damping_filter;
  damping_filter.CopyFrom(iir_damping_filter_);
  DCBlocker dc_blocker = dc_blocker_;
  auto string = string_.BlockView();
  auto stretch = stretch_.BlockView();
  Svf excitation;
  if constexpr (!external) excitation.CopyFrom(*excitation_filter);
  const float inv_dust_f = 1.0f / dust_f;
  const float dust_gain = 8.0f - dust_f * 6.0f;
  Random excitation_random;
  excitation_random.Seed(random->state());
  Random local_random;
  // Plaits generates all 16 excitation draws before the dispersion draws.
  // Separate cursors let this loop combine the work without changing that order.
  local_random.Seed(external ? random->state()
      : AdvanceStringRandom<kStringBlockSize>(random->state()));
  size_t input_index = 0;
  auto excite = [&]() {
    if constexpr (external) return input[input_index++];
    const float u = excitation_random.GetFloat();
    const float noise = u < dust_f ? (u * inv_dust_f) * dust_gain * 0.8f : 0.0f;
    return excitation.Process<FILTER_MODE_LOW_PASS>(noise);
  };
  float dispersion_noise = dispersion_noise_;
  float curved_bridge = curved_bridge_;
  float src_phase = src_phase_;
  float previous_sample = out_sample_[1];
  float current_sample = out_sample_[0];
  // The common case needs neither a sample-rate-converter branch nor a
  // crossfade. Selecting the newest sample changes only output rounding;
  // the feedback loop retains exactly the same signal.
  const bool resample = src_ratio != 1.0f;
  auto tick = [&](float input) {
      float delay = delay_modulation.Next();
      float s = 0.0f;
      
      if (non_linearity == STRING_NON_LINEARITY_DISPERSION) {
        float noise = local_random.GetFloat() - 0.5f;
        ONE_POLE(dispersion_noise, noise, noise_filter)
        delay *= 1.0f + dispersion_noise * noise_amount;
      } else {
        delay *= 1.0f - curved_bridge * bridge_curving;
      }
      
      if (non_linearity == STRING_NON_LINEARITY_DISPERSION) {
        float ap_delay = delay * stretch_point;
        float main_delay = delay - ap_delay * (0.408f - stretch_point * 0.308f) * stretch_correction;
        if (ap_delay >= 4.0f && main_delay >= 4.0f) {
          s = string.Read(main_delay);
          s = stretch.Allpass(s, ap_delay, ap_gain);
        } else {
          s = string.ReadHermite(delay);
        }
      } else {
        s = string.ReadHermite(delay);
      }

      if (non_linearity == STRING_NON_LINEARITY_CURVED_BRIDGE) {
        float value = fabsf(s) - 0.025f;
        float sign = s > 0.0f ? 1.0f : -1.5f;
        curved_bridge = (fabsf(value) + value) * sign;
      }
    
      s += input;
      CONSTRAIN(s, -20.0f, +20.0f);
      dc_blocker.Process(&s, 1);
      s = damping_filter.Process<FILTER_MODE_LOW_PASS>(s);
      string.Write(s);

      previous_sample = current_sample;
      current_sample = s;
  };
  if (!resample) {
    for (size_t i = 0; i < kStringBlockSize; ++i) {
      tick(excite());
      out[i] = current_sample;
    }
  } else {
    for (size_t i = 0; i < kStringBlockSize; ++i) {
      // Excitation filter runs every sample, even when the very-low-pitch
      // converter does not tick the string. Match the old discarded inputs.
      const float input = excite();
      src_phase += src_ratio;
      if (src_phase > 1.0f) {
        src_phase -= 1.0f;
        tick(input);
      }
      out[i] = Crossfade(previous_sample, current_sample, src_phase);
    }
  }
  string_.CommitCursor(string);
  stretch_.CommitCursor(stretch);
  if constexpr (!external) excitation_filter->CopyFrom(excitation);
  iir_damping_filter_.CopyFrom(damping_filter);
  dc_blocker_ = dc_blocker;
  random->Seed(local_random.state());
  dispersion_noise_ = dispersion_noise;
  curved_bridge_ = curved_bridge;
  src_phase_ = src_phase;
  out_sample_[1] = previous_sample;
  out_sample_[0] = current_sample;
}

}  // namespace plaits
