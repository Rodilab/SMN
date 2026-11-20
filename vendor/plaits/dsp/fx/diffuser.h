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
// Granular diffuser.

#ifndef PLAITS_DSP_FX_DIFFUSER_H_
#define PLAITS_DSP_FX_DIFFUSER_H_

#include <cassert>

#include "stmlib/stmlib.h"

#include "plaits/dsp/fx/fx_engine.h"

namespace plaits {

class Diffuser {
 public:
  Diffuser() { }
  ~Diffuser() { }
  
  void Init(uint16_t* buffer) {
    engine_.Init(buffer);
    engine_.Clear();
    engine_.SetLFOFrequency(LFO_1, 0.3f / 44100.0f);
    lp_decay_ = 0.0f;
    half_rate_ = false;
    ClearConverter();
    PrepareReads<false>(engine_.lfo_value(), ap4_read_, del_read_);
  }
  
  void Reset() {
    engine_.Clear();
    ClearConverter();
    if (half_rate_) PrepareReads<true>(engine_.lfo_value(), ap4_read_, del_read_);
    else PrepareReads<false>(engine_.lfo_value(), ap4_read_, del_read_);
  }
  
  void Process(float amount, float rt, float* in_out, size_t size) {
    ProcessCore<false>(amount, rt, in_out, size);
  }

  // SMN uses this fixed 16-sample route exclusively; the legacy Process route
  // remains full-rate. Both share the delay allocation, not two FX histories.
  // Do not alternate these different-rate APIs within one audio stream.
  void ProcessHalfRate(float amount, float rt, float* in_out, size_t size) {
    assert(size == 16);
    float low[8], dry[16];
    // A whole 16-sample block wraps the input history exactly once; a whole
    // eight-sample wet block wraps its history once too. No moving indices.
    for (size_t i = 0; i < 8; ++i) {
      const unsigned n = 2 * i + 1;
      dry[2 * i] = down_history_[(n + 1) & 15];
      dry[2 * i + 1] = down_history_[(n + 2) & 15];
      down_history_[n - 1] = in_out[2 * i];
      down_history_[n] = in_out[2 * i + 1];
      low[i] = 0.5f * down_history_[(n - 7) & 15]
            + kC0 * (down_history_[n] + down_history_[(n - 14) & 15])
            + kC2 * (down_history_[(n - 2) & 15] + down_history_[(n - 12) & 15])
            + kC4 * (down_history_[(n - 4) & 15] + down_history_[(n - 10) & 15])
            + kC6 * (down_history_[(n - 6) & 15] + down_history_[(n - 8) & 15]);
    }
    // Only the wet signal traverses the rate converter; mixing is full-rate.
    ProcessCore<true>(1.0f, rt, low, 8);
    float pending_odd = pending_odd_;
    for (size_t i = 0; i < 8; ++i) {
      const unsigned n = i;
      up_history_[n] = low[i];
      const float even = 2.0f * (
          kC0 * (up_history_[n] + up_history_[(n - 7) & 7])
          + kC2 * (up_history_[(n - 1) & 7] + up_history_[(n - 6) & 7])
          + kC4 * (up_history_[(n - 2) & 7] + up_history_[(n - 5) & 7])
          + kC6 * (up_history_[(n - 3) & 7] + up_history_[(n - 4) & 7]));
      const float odd = up_history_[(n - 3) & 7];
      in_out[2 * i] = dry[2 * i] + amount * (pending_odd - dry[2 * i]);
      in_out[2 * i + 1] = dry[2 * i + 1] + amount * (even - dry[2 * i + 1]);
      pending_odd = odd;
    }
    pending_odd_ = pending_odd;
  }

 private:
  // 15-tap half-band FIR: Hamming-windowed sinc, centre exactly 1/2,
  // side coefficients normalized to a total of 1/2. DC gain is one in both
  // phases of the interpolation. Group delay: 7 + 7 full-rate samples.
  static constexpr float kC0 = -0.003665183180649189f;
  static constexpr float kC2 =  0.016240086159749482f;
  static constexpr float kC4 = -0.06866899988698508f;
  static constexpr float kC6 =  0.3060940969078848f;
  void ClearConverter() {
    std::fill_n(down_history_, 16, 0.0f);
    std::fill_n(up_history_, 8, 0.0f);
    pending_odd_ = 0.0f;
  }

  template<bool half_rate>
  void ProcessCore(float amount, float rt, float* in_out, size_t size) {
    if (half_rate_ != half_rate) {
      // No clearing or allocation in the audio callback.
      half_rate_ = half_rate;
      PrepareReads<half_rate>(engine_.lfo_value(), ap4_read_, del_read_);
    }
    typedef E::Reserve<half_rate ? 58 : 116,
      E::Reserve<half_rate ? 83 : 165,
      E::Reserve<half_rate ? 124 : 247,
      E::Reserve<half_rate ? 204 : 408,
      E::Reserve<half_rate ? 759 : 1519,
      E::Reserve<half_rate ? 923 : 1847,
      E::Reserve<half_rate ? 1567 : 3134> > > > > > > Memory;
    E::DelayLine<Memory, 0> ap1;
    E::DelayLine<Memory, 1> ap2;
    E::DelayLine<Memory, 2> ap3;
    E::DelayLine<Memory, 3> ap4;
    E::DelayLine<Memory, 4> dapa;
    E::DelayLine<Memory, 5> dapb;
    E::DelayLine<Memory, 6> del;
    E::Context c;
    const float kap = 0.625f;
    const float klp = 1.0f - powf(0.25f, 48000.0f / (half_rate ? 22050.0f : 44100.0f));
    float lp = lp_decay_;
    int32_t cursor = engine_.cursor();
    E::Interpolation ap4_read = ap4_read_, del_read = del_read_;
    while (size) {
      // Same real-time modulation cadence: 32 full-rate or 16 half-rate ticks.
      // The inner loop does not test the clock or rebuild read positions.
      constexpr unsigned interval = half_rate ? 16 : 32;
      if (engine_.UpdateModulation(cursor, interval - 1))
        PrepareReads<half_rate>(engine_.lfo_value(), ap4_read, del_read);
      const size_t until_update = (cursor - 1) & (interval - 1);
      const size_t span = std::min(size, until_update ? until_update : size_t(interval));
      size -= span;
      for (size_t i = 0; i < span; ++i) {
        float wet;
        engine_.Start(&c, cursor);
        c.Read(*in_out);
        c.Read(ap1 TAIL, kap);
        c.WriteAllPass(ap1, -kap);
        c.Read(ap2 TAIL, kap);
        c.WriteAllPass(ap2, -kap);
        c.Read(ap3 TAIL, kap);
        c.WriteAllPass(ap3, -kap);
        c.Interpolate(ap4, ap4_read, kap);
        c.WriteAllPass(ap4, -kap);
        c.Interpolate(del, del_read, rt);
        c.Lp(lp, klp);
        c.Read(dapa TAIL, -kap);
        c.WriteAllPass(dapa, kap);
        c.Read(dapb TAIL, kap);
        c.WriteAllPass(dapb, -kap);
        c.Write(del, 2.0f);
        c.Write(wet, 0.0f);
        *in_out += amount * (wet - *in_out);
        ++in_out;
      }
    }
    engine_.set_cursor(cursor);
    lp_decay_ = lp;
    ap4_read_ = ap4_read;
    del_read_ = del_read;
  }
  
  typedef FxEngine<8192, FORMAT_12_BIT> E;
  template<bool half_rate>
  static void PrepareReads(float lfo, E::Interpolation& ap4, E::Interpolation& del) {
    constexpr float rate = half_rate ? 22050.0f : 44100.0f;
    ap4.Init((400.0f * rate / 48000.0f)
        + (43.0f * rate / 48000.0f) * lfo);
    del.Init((3070.0f * rate / 48000.0f)
        + (340.0f * rate / 48000.0f) * lfo);
  }
  E engine_;
  float lp_decay_;
  E::Interpolation ap4_read_, del_read_;
  bool half_rate_;
  float down_history_[16], up_history_[8];
  float pending_odd_;
  
  DISALLOW_COPY_AND_ASSIGN(Diffuser);
};

}  // namespace plaits

#endif  // PLAITS_DSP_FX_DIFFUSER_H_
