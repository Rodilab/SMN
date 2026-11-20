// Copyright 2021 Emilie Gillet.
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
// Wave terrain synthesis.

#include "plaits/dsp/engine2/wave_terrain_engine.h"

#include <cmath>
#include <algorithm>

#include "plaits/dsp/oscillator/wavetable_oscillator.h"

namespace plaits {

using namespace std;
using namespace stmlib;

// At the maximum H target, omit the residual blend of about 0.0001.
// Other terrain boundaries do not use this near-zero shortcut.
constexpr float kLastTerrain = 8.0f - 1.0001f;
// Allow the few ulps left by repeated additions at the saturated target.
// The discarded weight is at most ~0.00011015 (0.0111 percent).
constexpr float kSingleLastTerrain = 7.0f - 0.00011f;

void WaveTerrainEngine::Init(BufferAllocator* allocator) {
  path_.Init();
  offset_ = 0.0f;
  terrain_ = 0.0f;
  temp_buffer_ = allocator->Allocate<float>(kMaxBlockSize * 4);
}

void WaveTerrainEngine::Reset() {
  
}

// The fourth Hermite field is the exact original integrated sample x0.
// Recover wrapped samples directly from Wavetable's one shared flash resource.
extern const int16_t wav_hermite_coefficients[192 * 128 * 4];

inline float IntegratedSample(const int16_t* wave, int sample) {
  return static_cast<float>(wave[((sample + 127) & 127) * 4 + 3]);
}

inline float DifferentiateWave(const int16_t* wave, int sample, float fraction) {
  // From [2a, 2b, 2c, x0], recover the original integer differences.
  // Their bounds and the even numerator are checked against the raw resource.
  const int16_t* coefficients = wave + (sample & 127) * 4;
  const int32_t curvature = int32_t(coefficients[0]) - int32_t(coefficients[1]);
  const int32_t twice_delta = int32_t(coefficients[2]) - curvature;
  // Divide before converting: keep the original delta + curvature * fraction
  // and its ARM FMA rounding, without a second floating-point product.
  return float(twice_delta / 2) + float(curvature) * fraction;
}

// Coordinates for one path point, shared by the two factory banks.
// This temporary value does not add a table or persistent voice state.
struct TerrainWTCoordinates {
  int wave;
  int sample;
  float sample_fraction;
  float wave_fraction;
};

inline TerrainWTCoordinates PrepareTerrainWT(float x, float y) {
  // The magic-circle path can slightly overshoot its nominal radius during
  // pitch changes. Bound lookup coordinates before indexing a factory bank.
  x = std::clamp(x, -1.0f, 1.0f);
  y = std::clamp(y, -1.0f, 1.0f);
  const float sample = (y + 1.0f) * 0.5f * 128.0f;
  const float wt = (x + 1.0f) * 0.5f * 63.0f;
  MAKE_INTEGRAL_FRACTIONAL(sample);
  MAKE_INTEGRAL_FRACTIONAL(wt);
  return {wt_integral, sample_integral, sample_fractional, wt_fractional};
}

inline float TerrainLookupWT(float x, float y, int bank) {
  const TerrainWTCoordinates p = PrepareTerrainWT(x, y);
  const int16_t* waves = wav_hermite_coefficients +
      (bank * 64 + p.wave) * 512;
  const float a = DifferentiateWave(waves, p.sample, p.sample_fraction);
  // At the last wave, fraction is zero: never address the next bank.
  const float b = p.wave == 63 ? a :
      DifferentiateWave(waves + 512, p.sample, p.sample_fraction);
  return (a + (b - a) * p.wave_fraction) * (1.0f / 1024.0f);
}

inline float TerrainLookupWTPair(float x, float y, int bank, float fraction) {
  const TerrainWTCoordinates p = PrepareTerrainWT(x, y);
  const int16_t* first = wav_hermite_coefficients + (bank * 64 + p.wave) * 512;
  const int16_t* second = first - 64 * 512;  // Factory terrain order: 2, 1, 0.
  const float a0 = DifferentiateWave(first, p.sample, p.sample_fraction);
  const float b0 = DifferentiateWave(second, p.sample, p.sample_fraction);
  float a1 = a0;
  float b1 = b0;
  // Share the endpoint guard too: neither bank may read its next wave at 63.
  if (p.wave != 63) {
    a1 = DifferentiateWave(first + 512, p.sample, p.sample_fraction);
    b1 = DifferentiateWave(second + 512, p.sample, p.sample_fraction);
  }
  const float a = (a0 + (a1 - a0) * p.wave_fraction) * (1.0f / 1024.0f);
  const float b = (b0 + (b1 - b0) * p.wave_fraction) * (1.0f / 1024.0f);
  return a + (b - a) * fraction;
}

// BEGIN GENERATED TERRAIN SINE COEFFICIENTS
// Exact [sample, float(next - sample)] pairs; shared immutable flash data.
// Generated by tests/smn_synth/terrain_sine_coefficients.py, never at runtime.
alignas(8) static const float terrain_sine_coefficients[512][2] = {
  {0x0.0p+0f, 0x1.921d200000000p-7f},
  {0x1.921d200000000p-7f, 0x1.920da00000000p-7f},
  {0x1.9215600000000p-6f, 0x1.91eea00000000p-7f},
  {0x1.2d86580000000p-5f, 0x1.91c0200000000p-7f},
  {0x1.91f6600000000p-5f, 0x1.9182200000000p-7f},
  {0x1.f656e80000000p-5f, 0x1.9134b00000000p-7f},
  {0x1.2d520a0000000p-4f, 0x1.90d7b00000000p-7f},
  {0x1.5f6d000000000p-4f, 0x1.906b600000000p-7f},
  {0x1.917a6c0000000p-4f, 0x1.8fef800000000p-7f},
  {0x1.c3785c0000000p-4f, 0x1.8f64500000000p-7f},
  {0x1.f564e60000000p-4f, 0x1.8ec9900000000p-7f},
  {0x1.139f0c0000000p-3f, 0x1.8e1fa00000000p-7f},
  {0x1.2c81060000000p-3f, 0x1.8d66600000000p-7f},
  {0x1.45576c0000000p-3f, 0x1.8c9d800000000p-7f},
  {0x1.5e21440000000p-3f, 0x1.8bc5a00000000p-7f},
  {0x1.76dd9e0000000p-3f, 0x1.8ade600000000p-7f},
  {0x1.8f8b840000000p-3f, 0x1.89e7e00000000p-7f},
  {0x1.a82a020000000p-3f, 0x1.88e2400000000p-7f},
  {0x1.c0b8260000000p-3f, 0x1.87cd800000000p-7f},
  {0x1.d934fe0000000p-3f, 0x1.86a9a00000000p-7f},
  {0x1.f19f980000000p-3f, 0x1.8576800000000p-7f},
  {0x1.04fb800000000p-2f, 0x1.8434c00000000p-7f},
  {0x1.111d260000000p-2f, 0x1.82e3c00000000p-7f},
  {0x1.1d34440000000p-2f, 0x1.8183c00000000p-7f},
  {0x1.2940620000000p-2f, 0x1.8015400000000p-7f},
  {0x1.35410c0000000p-2f, 0x1.7e97c00000000p-7f},
  {0x1.4135ca0000000p-2f, 0x1.7d0b400000000p-7f},
  {0x1.4d1e240000000p-2f, 0x1.7b70800000000p-7f},
  {0x1.58f9a80000000p-2f, 0x1.79c6c00000000p-7f},
  {0x1.64c7de0000000p-2f, 0x1.780ec00000000p-7f},
  {0x1.7088540000000p-2f, 0x1.7648000000000p-7f},
  {0x1.7c3a940000000p-2f, 0x1.7472c00000000p-7f},
  {0x1.87de2a0000000p-2f, 0x1.728f800000000p-7f},
  {0x1.9372a60000000p-2f, 0x1.709dc00000000p-7f},
  {0x1.9ef7940000000p-2f, 0x1.6e9dc00000000p-7f},
  {0x1.aa6c820000000p-2f, 0x1.6c8fc00000000p-7f},
  {0x1.b5d1000000000p-2f, 0x1.6a73c00000000p-7f},
  {0x1.c1249e0000000p-2f, 0x1.6849800000000p-7f},
  {0x1.cc66ea0000000p-2f, 0x1.6611800000000p-7f},
  {0x1.d797760000000p-2f, 0x1.63cbc00000000p-7f},
  {0x1.e2b5d40000000p-2f, 0x1.6178400000000p-7f},
  {0x1.edc1960000000p-2f, 0x1.5f17000000000p-7f},
  {0x1.f8ba4e0000000p-2f, 0x1.5ca8400000000p-7f},
  {0x1.01cfc80000000p-1f, 0x1.5a2c800000000p-7f},
  {0x1.07387a0000000p-1f, 0x1.57a2800000000p-7f},
  {0x1.0c97040000000p-1f, 0x1.550c800000000p-7f},
  {0x1.11eb360000000p-1f, 0x1.5268000000000p-7f},
  {0x1.1734d60000000p-1f, 0x1.4fb7800000000p-7f},
  {0x1.1c73b40000000p-1f, 0x1.4cf9800000000p-7f},
  {0x1.21a79a0000000p-1f, 0x1.4a2e800000000p-7f},
  {0x1.26d0540000000p-1f, 0x1.4757800000000p-7f},
  {0x1.2bedb20000000p-1f, 0x1.4473800000000p-7f},
  {0x1.30ff800000000p-1f, 0x1.4183000000000p-7f},
  {0x1.36058c0000000p-1f, 0x1.3e85800000000p-7f},
  {0x1.3affa20000000p-1f, 0x1.3b7d000000000p-7f},
  {0x1.3fed960000000p-1f, 0x1.3867000000000p-7f},
  {0x1.44cf320000000p-1f, 0x1.3546000000000p-7f},
  {0x1.49a44a0000000p-1f, 0x1.3218800000000p-7f},
  {0x1.4e6cac0000000p-1f, 0x1.2edf800000000p-7f},
  {0x1.53282a0000000p-1f, 0x1.2b9a800000000p-7f},
  {0x1.57d6940000000p-1f, 0x1.284a000000000p-7f},
  {0x1.5c77bc0000000p-1f, 0x1.24ee800000000p-7f},
  {0x1.610b760000000p-1f, 0x1.2187000000000p-7f},
  {0x1.6591920000000p-1f, 0x1.1e15000000000p-7f},
  {0x1.6a09e60000000p-1f, 0x1.1a98000000000p-7f},
  {0x1.6e74460000000p-1f, 0x1.170f800000000p-7f},
  {0x1.72d0840000000p-1f, 0x1.137c800000000p-7f},
  {0x1.771e760000000p-1f, 0x1.0fdf000000000p-7f},
  {0x1.7b5df20000000p-1f, 0x1.0c37000000000p-7f},
  {0x1.7f8ece0000000p-1f, 0x1.0884800000000p-7f},
  {0x1.83b0e00000000p-1f, 0x1.04c8000000000p-7f},
  {0x1.87c4000000000p-1f, 0x1.0101800000000p-7f},
  {0x1.8bc8060000000p-1f, 0x1.fa62000000000p-8f},
  {0x1.8fbcca0000000p-1f, 0x1.f2ad000000000p-8f},
  {0x1.93a2240000000p-1f, 0x1.eae6000000000p-8f},
  {0x1.9777f00000000p-1f, 0x1.e30a000000000p-8f},
  {0x1.9b3e040000000p-1f, 0x1.db1d000000000p-8f},
  {0x1.9ef43e0000000p-1f, 0x1.d31e000000000p-8f},
  {0x1.a29a7a0000000p-1f, 0x1.cb0c000000000p-8f},
  {0x1.a630920000000p-1f, 0x1.c2e8000000000p-8f},
  {0x1.a9b6620000000p-1f, 0x1.bab4000000000p-8f},
  {0x1.ad2bca0000000p-1f, 0x1.b26e000000000p-8f},
  {0x1.b090a60000000p-1f, 0x1.aa17000000000p-8f},
  {0x1.b3e4d40000000p-1f, 0x1.a1b0000000000p-8f},
  {0x1.b728340000000p-1f, 0x1.9939000000000p-8f},
  {0x1.ba5aa60000000p-1f, 0x1.90b2000000000p-8f},
  {0x1.bd7c0a0000000p-1f, 0x1.881c000000000p-8f},
  {0x1.c08c420000000p-1f, 0x1.7f77000000000p-8f},
  {0x1.c38b300000000p-1f, 0x1.76c2000000000p-8f},
  {0x1.c678b40000000p-1f, 0x1.6dff000000000p-8f},
  {0x1.c954b20000000p-1f, 0x1.652f000000000p-8f},
  {0x1.cc1f100000000p-1f, 0x1.5c50000000000p-8f},
  {0x1.ced7b00000000p-1f, 0x1.5364000000000p-8f},
  {0x1.d17e780000000p-1f, 0x1.4a6b000000000p-8f},
  {0x1.d4134e0000000p-1f, 0x1.4165000000000p-8f},
  {0x1.d696180000000p-1f, 0x1.3852000000000p-8f},
  {0x1.d906bc0000000p-1f, 0x1.2f35000000000p-8f},
  {0x1.db65260000000p-1f, 0x1.260b000000000p-8f},
  {0x1.ddb13c0000000p-1f, 0x1.1cd5000000000p-8f},
  {0x1.dfeae60000000p-1f, 0x1.1395000000000p-8f},
  {0x1.e212100000000p-1f, 0x1.0a4a000000000p-8f},
  {0x1.e426a40000000p-1f, 0x1.00f5000000000p-8f},
  {0x1.e6288e0000000p-1f, 0x1.ef2c000000000p-9f},
  {0x1.e817ba0000000p-1f, 0x1.dc5c000000000p-9f},
  {0x1.e9f4160000000p-1f, 0x1.c976000000000p-9f},
  {0x1.ebbd8c0000000p-1f, 0x1.b682000000000p-9f},
  {0x1.ed740e0000000p-1f, 0x1.a37c000000000p-9f},
  {0x1.ef178a0000000p-1f, 0x1.9066000000000p-9f},
  {0x1.f0a7f00000000p-1f, 0x1.7d40000000000p-9f},
  {0x1.f225300000000p-1f, 0x1.6a0a000000000p-9f},
  {0x1.f38f3a0000000p-1f, 0x1.56ca000000000p-9f},
  {0x1.f4e6040000000p-1f, 0x1.4378000000000p-9f},
  {0x1.f6297c0000000p-1f, 0x1.301e000000000p-9f},
  {0x1.f7599a0000000p-1f, 0x1.1cb6000000000p-9f},
  {0x1.f876500000000p-1f, 0x1.0942000000000p-9f},
  {0x1.f97f920000000p-1f, 0x1.eb8c000000000p-10f},
  {0x1.fa75580000000p-1f, 0x1.c480000000000p-10f},
  {0x1.fb57980000000p-1f, 0x1.9d60000000000p-10f},
  {0x1.fc26480000000p-1f, 0x1.7630000000000p-10f},
  {0x1.fce1600000000p-1f, 0x1.4ef4000000000p-10f},
  {0x1.fd88da0000000p-1f, 0x1.27ac000000000p-10f},
  {0x1.fe1cb00000000p-1f, 0x1.0054000000000p-10f},
  {0x1.fe9cda0000000p-1f, 0x1.b1f0000000000p-11f},
  {0x1.ff09560000000p-1f, 0x1.6320000000000p-11f},
  {0x1.ff621e0000000p-1f, 0x1.1448000000000p-11f},
  {0x1.ffa7300000000p-1f, 0x1.8ab0000000000p-12f},
  {0x1.ffd8860000000p-1f, 0x1.d9c0000000000p-13f},
  {0x1.fff6220000000p-1f, 0x1.3bc0000000000p-14f},
  {0x1.0000000000000p+0f, -0x1.3bc0000000000p-14f},
  {0x1.fff6220000000p-1f, -0x1.d9c0000000000p-13f},
  {0x1.ffd8860000000p-1f, -0x1.8ab0000000000p-12f},
  {0x1.ffa7300000000p-1f, -0x1.1448000000000p-11f},
  {0x1.ff621e0000000p-1f, -0x1.6320000000000p-11f},
  {0x1.ff09560000000p-1f, -0x1.b1f0000000000p-11f},
  {0x1.fe9cda0000000p-1f, -0x1.0054000000000p-10f},
  {0x1.fe1cb00000000p-1f, -0x1.27ac000000000p-10f},
  {0x1.fd88da0000000p-1f, -0x1.4ef4000000000p-10f},
  {0x1.fce1600000000p-1f, -0x1.7630000000000p-10f},
  {0x1.fc26480000000p-1f, -0x1.9d60000000000p-10f},
  {0x1.fb57980000000p-1f, -0x1.c480000000000p-10f},
  {0x1.fa75580000000p-1f, -0x1.eb8c000000000p-10f},
  {0x1.f97f920000000p-1f, -0x1.0942000000000p-9f},
  {0x1.f876500000000p-1f, -0x1.1cb6000000000p-9f},
  {0x1.f7599a0000000p-1f, -0x1.301e000000000p-9f},
  {0x1.f6297c0000000p-1f, -0x1.4378000000000p-9f},
  {0x1.f4e6040000000p-1f, -0x1.56ca000000000p-9f},
  {0x1.f38f3a0000000p-1f, -0x1.6a0a000000000p-9f},
  {0x1.f225300000000p-1f, -0x1.7d40000000000p-9f},
  {0x1.f0a7f00000000p-1f, -0x1.9066000000000p-9f},
  {0x1.ef178a0000000p-1f, -0x1.a37c000000000p-9f},
  {0x1.ed740e0000000p-1f, -0x1.b682000000000p-9f},
  {0x1.ebbd8c0000000p-1f, -0x1.c976000000000p-9f},
  {0x1.e9f4160000000p-1f, -0x1.dc5c000000000p-9f},
  {0x1.e817ba0000000p-1f, -0x1.ef2c000000000p-9f},
  {0x1.e6288e0000000p-1f, -0x1.00f5000000000p-8f},
  {0x1.e426a40000000p-1f, -0x1.0a4a000000000p-8f},
  {0x1.e212100000000p-1f, -0x1.1395000000000p-8f},
  {0x1.dfeae60000000p-1f, -0x1.1cd5000000000p-8f},
  {0x1.ddb13c0000000p-1f, -0x1.260b000000000p-8f},
  {0x1.db65260000000p-1f, -0x1.2f35000000000p-8f},
  {0x1.d906bc0000000p-1f, -0x1.3852000000000p-8f},
  {0x1.d696180000000p-1f, -0x1.4165000000000p-8f},
  {0x1.d4134e0000000p-1f, -0x1.4a6b000000000p-8f},
  {0x1.d17e780000000p-1f, -0x1.5364000000000p-8f},
  {0x1.ced7b00000000p-1f, -0x1.5c50000000000p-8f},
  {0x1.cc1f100000000p-1f, -0x1.652f000000000p-8f},
  {0x1.c954b20000000p-1f, -0x1.6dff000000000p-8f},
  {0x1.c678b40000000p-1f, -0x1.76c2000000000p-8f},
  {0x1.c38b300000000p-1f, -0x1.7f77000000000p-8f},
  {0x1.c08c420000000p-1f, -0x1.881c000000000p-8f},
  {0x1.bd7c0a0000000p-1f, -0x1.90b2000000000p-8f},
  {0x1.ba5aa60000000p-1f, -0x1.9939000000000p-8f},
  {0x1.b728340000000p-1f, -0x1.a1b0000000000p-8f},
  {0x1.b3e4d40000000p-1f, -0x1.aa17000000000p-8f},
  {0x1.b090a60000000p-1f, -0x1.b26e000000000p-8f},
  {0x1.ad2bca0000000p-1f, -0x1.bab4000000000p-8f},
  {0x1.a9b6620000000p-1f, -0x1.c2e8000000000p-8f},
  {0x1.a630920000000p-1f, -0x1.cb0c000000000p-8f},
  {0x1.a29a7a0000000p-1f, -0x1.d31e000000000p-8f},
  {0x1.9ef43e0000000p-1f, -0x1.db1d000000000p-8f},
  {0x1.9b3e040000000p-1f, -0x1.e30a000000000p-8f},
  {0x1.9777f00000000p-1f, -0x1.eae6000000000p-8f},
  {0x1.93a2240000000p-1f, -0x1.f2ad000000000p-8f},
  {0x1.8fbcca0000000p-1f, -0x1.fa62000000000p-8f},
  {0x1.8bc8060000000p-1f, -0x1.0101800000000p-7f},
  {0x1.87c4000000000p-1f, -0x1.04c8000000000p-7f},
  {0x1.83b0e00000000p-1f, -0x1.0884800000000p-7f},
  {0x1.7f8ece0000000p-1f, -0x1.0c37000000000p-7f},
  {0x1.7b5df20000000p-1f, -0x1.0fdf000000000p-7f},
  {0x1.771e760000000p-1f, -0x1.137c800000000p-7f},
  {0x1.72d0840000000p-1f, -0x1.170f800000000p-7f},
  {0x1.6e74460000000p-1f, -0x1.1a98000000000p-7f},
  {0x1.6a09e60000000p-1f, -0x1.1e15000000000p-7f},
  {0x1.6591920000000p-1f, -0x1.2187000000000p-7f},
  {0x1.610b760000000p-1f, -0x1.24ee800000000p-7f},
  {0x1.5c77bc0000000p-1f, -0x1.284a000000000p-7f},
  {0x1.57d6940000000p-1f, -0x1.2b9a800000000p-7f},
  {0x1.53282a0000000p-1f, -0x1.2edf800000000p-7f},
  {0x1.4e6cac0000000p-1f, -0x1.3218800000000p-7f},
  {0x1.49a44a0000000p-1f, -0x1.3546000000000p-7f},
  {0x1.44cf320000000p-1f, -0x1.3867000000000p-7f},
  {0x1.3fed960000000p-1f, -0x1.3b7d000000000p-7f},
  {0x1.3affa20000000p-1f, -0x1.3e85800000000p-7f},
  {0x1.36058c0000000p-1f, -0x1.4183000000000p-7f},
  {0x1.30ff800000000p-1f, -0x1.4473800000000p-7f},
  {0x1.2bedb20000000p-1f, -0x1.4757800000000p-7f},
  {0x1.26d0540000000p-1f, -0x1.4a2e800000000p-7f},
  {0x1.21a79a0000000p-1f, -0x1.4cf9800000000p-7f},
  {0x1.1c73b40000000p-1f, -0x1.4fb7800000000p-7f},
  {0x1.1734d60000000p-1f, -0x1.5268000000000p-7f},
  {0x1.11eb360000000p-1f, -0x1.550c800000000p-7f},
  {0x1.0c97040000000p-1f, -0x1.57a2800000000p-7f},
  {0x1.07387a0000000p-1f, -0x1.5a2c800000000p-7f},
  {0x1.01cfc80000000p-1f, -0x1.5ca8400000000p-7f},
  {0x1.f8ba4e0000000p-2f, -0x1.5f17000000000p-7f},
  {0x1.edc1960000000p-2f, -0x1.6178400000000p-7f},
  {0x1.e2b5d40000000p-2f, -0x1.63cbc00000000p-7f},
  {0x1.d797760000000p-2f, -0x1.6611800000000p-7f},
  {0x1.cc66ea0000000p-2f, -0x1.6849800000000p-7f},
  {0x1.c1249e0000000p-2f, -0x1.6a73c00000000p-7f},
  {0x1.b5d1000000000p-2f, -0x1.6c8fc00000000p-7f},
  {0x1.aa6c820000000p-2f, -0x1.6e9dc00000000p-7f},
  {0x1.9ef7940000000p-2f, -0x1.709dc00000000p-7f},
  {0x1.9372a60000000p-2f, -0x1.728f800000000p-7f},
  {0x1.87de2a0000000p-2f, -0x1.7472c00000000p-7f},
  {0x1.7c3a940000000p-2f, -0x1.7648000000000p-7f},
  {0x1.7088540000000p-2f, -0x1.780ec00000000p-7f},
  {0x1.64c7de0000000p-2f, -0x1.79c6c00000000p-7f},
  {0x1.58f9a80000000p-2f, -0x1.7b70800000000p-7f},
  {0x1.4d1e240000000p-2f, -0x1.7d0b400000000p-7f},
  {0x1.4135ca0000000p-2f, -0x1.7e97c00000000p-7f},
  {0x1.35410c0000000p-2f, -0x1.8015400000000p-7f},
  {0x1.2940620000000p-2f, -0x1.8183c00000000p-7f},
  {0x1.1d34440000000p-2f, -0x1.82e3c00000000p-7f},
  {0x1.111d260000000p-2f, -0x1.8434c00000000p-7f},
  {0x1.04fb800000000p-2f, -0x1.8576800000000p-7f},
  {0x1.f19f980000000p-3f, -0x1.86a9a00000000p-7f},
  {0x1.d934fe0000000p-3f, -0x1.87cd800000000p-7f},
  {0x1.c0b8260000000p-3f, -0x1.88e2400000000p-7f},
  {0x1.a82a020000000p-3f, -0x1.89e7e00000000p-7f},
  {0x1.8f8b840000000p-3f, -0x1.8ade600000000p-7f},
  {0x1.76dd9e0000000p-3f, -0x1.8bc5a00000000p-7f},
  {0x1.5e21440000000p-3f, -0x1.8c9d800000000p-7f},
  {0x1.45576c0000000p-3f, -0x1.8d66600000000p-7f},
  {0x1.2c81060000000p-3f, -0x1.8e1fa00000000p-7f},
  {0x1.139f0c0000000p-3f, -0x1.8ec9900000000p-7f},
  {0x1.f564e60000000p-4f, -0x1.8f64500000000p-7f},
  {0x1.c3785c0000000p-4f, -0x1.8fef800000000p-7f},
  {0x1.917a6c0000000p-4f, -0x1.906b600000000p-7f},
  {0x1.5f6d000000000p-4f, -0x1.90d7b00000000p-7f},
  {0x1.2d520a0000000p-4f, -0x1.9134b00000000p-7f},
  {0x1.f656e80000000p-5f, -0x1.9182200000000p-7f},
  {0x1.91f6600000000p-5f, -0x1.91c0200000000p-7f},
  {0x1.2d86580000000p-5f, -0x1.91eea00000000p-7f},
  {0x1.9215600000000p-6f, -0x1.920da00000000p-7f},
  {0x1.921d200000000p-7f, -0x1.921d200000000p-7f},
  {0x1.1a62640000000p-53f, -0x1.921d200000000p-7f},
  {-0x1.921d200000000p-7f, -0x1.920da00000000p-7f},
  {-0x1.9215600000000p-6f, -0x1.91eea00000000p-7f},
  {-0x1.2d86580000000p-5f, -0x1.91c0200000000p-7f},
  {-0x1.91f6600000000p-5f, -0x1.9182200000000p-7f},
  {-0x1.f656e80000000p-5f, -0x1.9134b00000000p-7f},
  {-0x1.2d520a0000000p-4f, -0x1.90d7b00000000p-7f},
  {-0x1.5f6d000000000p-4f, -0x1.906b600000000p-7f},
  {-0x1.917a6c0000000p-4f, -0x1.8fef800000000p-7f},
  {-0x1.c3785c0000000p-4f, -0x1.8f64500000000p-7f},
  {-0x1.f564e60000000p-4f, -0x1.8ec9900000000p-7f},
  {-0x1.139f0c0000000p-3f, -0x1.8e1fa00000000p-7f},
  {-0x1.2c81060000000p-3f, -0x1.8d66600000000p-7f},
  {-0x1.45576c0000000p-3f, -0x1.8c9d800000000p-7f},
  {-0x1.5e21440000000p-3f, -0x1.8bc5a00000000p-7f},
  {-0x1.76dd9e0000000p-3f, -0x1.8ade600000000p-7f},
  {-0x1.8f8b840000000p-3f, -0x1.89e7e00000000p-7f},
  {-0x1.a82a020000000p-3f, -0x1.88e2400000000p-7f},
  {-0x1.c0b8260000000p-3f, -0x1.87cd800000000p-7f},
  {-0x1.d934fe0000000p-3f, -0x1.86a9a00000000p-7f},
  {-0x1.f19f980000000p-3f, -0x1.8576800000000p-7f},
  {-0x1.04fb800000000p-2f, -0x1.8434c00000000p-7f},
  {-0x1.111d260000000p-2f, -0x1.82e3c00000000p-7f},
  {-0x1.1d34440000000p-2f, -0x1.8183c00000000p-7f},
  {-0x1.2940620000000p-2f, -0x1.8015400000000p-7f},
  {-0x1.35410c0000000p-2f, -0x1.7e97c00000000p-7f},
  {-0x1.4135ca0000000p-2f, -0x1.7d0b400000000p-7f},
  {-0x1.4d1e240000000p-2f, -0x1.7b70800000000p-7f},
  {-0x1.58f9a80000000p-2f, -0x1.79c6c00000000p-7f},
  {-0x1.64c7de0000000p-2f, -0x1.780ec00000000p-7f},
  {-0x1.7088540000000p-2f, -0x1.7648000000000p-7f},
  {-0x1.7c3a940000000p-2f, -0x1.7472c00000000p-7f},
  {-0x1.87de2a0000000p-2f, -0x1.728f800000000p-7f},
  {-0x1.9372a60000000p-2f, -0x1.709dc00000000p-7f},
  {-0x1.9ef7940000000p-2f, -0x1.6e9dc00000000p-7f},
  {-0x1.aa6c820000000p-2f, -0x1.6c8fc00000000p-7f},
  {-0x1.b5d1000000000p-2f, -0x1.6a73c00000000p-7f},
  {-0x1.c1249e0000000p-2f, -0x1.6849800000000p-7f},
  {-0x1.cc66ea0000000p-2f, -0x1.6611800000000p-7f},
  {-0x1.d797760000000p-2f, -0x1.63cbc00000000p-7f},
  {-0x1.e2b5d40000000p-2f, -0x1.6178400000000p-7f},
  {-0x1.edc1960000000p-2f, -0x1.5f17000000000p-7f},
  {-0x1.f8ba4e0000000p-2f, -0x1.5ca8400000000p-7f},
  {-0x1.01cfc80000000p-1f, -0x1.5a2c800000000p-7f},
  {-0x1.07387a0000000p-1f, -0x1.57a2800000000p-7f},
  {-0x1.0c97040000000p-1f, -0x1.550c800000000p-7f},
  {-0x1.11eb360000000p-1f, -0x1.5268000000000p-7f},
  {-0x1.1734d60000000p-1f, -0x1.4fb7800000000p-7f},
  {-0x1.1c73b40000000p-1f, -0x1.4cf9800000000p-7f},
  {-0x1.21a79a0000000p-1f, -0x1.4a2e800000000p-7f},
  {-0x1.26d0540000000p-1f, -0x1.4757800000000p-7f},
  {-0x1.2bedb20000000p-1f, -0x1.4473800000000p-7f},
  {-0x1.30ff800000000p-1f, -0x1.4183000000000p-7f},
  {-0x1.36058c0000000p-1f, -0x1.3e85800000000p-7f},
  {-0x1.3affa20000000p-1f, -0x1.3b7d000000000p-7f},
  {-0x1.3fed960000000p-1f, -0x1.3867000000000p-7f},
  {-0x1.44cf320000000p-1f, -0x1.3546000000000p-7f},
  {-0x1.49a44a0000000p-1f, -0x1.3218800000000p-7f},
  {-0x1.4e6cac0000000p-1f, -0x1.2edf800000000p-7f},
  {-0x1.53282a0000000p-1f, -0x1.2b9a800000000p-7f},
  {-0x1.57d6940000000p-1f, -0x1.284a000000000p-7f},
  {-0x1.5c77bc0000000p-1f, -0x1.24ee800000000p-7f},
  {-0x1.610b760000000p-1f, -0x1.2187000000000p-7f},
  {-0x1.6591920000000p-1f, -0x1.1e15000000000p-7f},
  {-0x1.6a09e60000000p-1f, -0x1.1a98000000000p-7f},
  {-0x1.6e74460000000p-1f, -0x1.170f800000000p-7f},
  {-0x1.72d0840000000p-1f, -0x1.137c800000000p-7f},
  {-0x1.771e760000000p-1f, -0x1.0fdf000000000p-7f},
  {-0x1.7b5df20000000p-1f, -0x1.0c37000000000p-7f},
  {-0x1.7f8ece0000000p-1f, -0x1.0884800000000p-7f},
  {-0x1.83b0e00000000p-1f, -0x1.04c8000000000p-7f},
  {-0x1.87c4000000000p-1f, -0x1.0101800000000p-7f},
  {-0x1.8bc8060000000p-1f, -0x1.fa62000000000p-8f},
  {-0x1.8fbcca0000000p-1f, -0x1.f2ad000000000p-8f},
  {-0x1.93a2240000000p-1f, -0x1.eae6000000000p-8f},
  {-0x1.9777f00000000p-1f, -0x1.e30a000000000p-8f},
  {-0x1.9b3e040000000p-1f, -0x1.db1d000000000p-8f},
  {-0x1.9ef43e0000000p-1f, -0x1.d31e000000000p-8f},
  {-0x1.a29a7a0000000p-1f, -0x1.cb0c000000000p-8f},
  {-0x1.a630920000000p-1f, -0x1.c2e8000000000p-8f},
  {-0x1.a9b6620000000p-1f, -0x1.bab4000000000p-8f},
  {-0x1.ad2bca0000000p-1f, -0x1.b26e000000000p-8f},
  {-0x1.b090a60000000p-1f, -0x1.aa17000000000p-8f},
  {-0x1.b3e4d40000000p-1f, -0x1.a1b0000000000p-8f},
  {-0x1.b728340000000p-1f, -0x1.9939000000000p-8f},
  {-0x1.ba5aa60000000p-1f, -0x1.90b2000000000p-8f},
  {-0x1.bd7c0a0000000p-1f, -0x1.881c000000000p-8f},
  {-0x1.c08c420000000p-1f, -0x1.7f77000000000p-8f},
  {-0x1.c38b300000000p-1f, -0x1.76c2000000000p-8f},
  {-0x1.c678b40000000p-1f, -0x1.6dff000000000p-8f},
  {-0x1.c954b20000000p-1f, -0x1.652f000000000p-8f},
  {-0x1.cc1f100000000p-1f, -0x1.5c50000000000p-8f},
  {-0x1.ced7b00000000p-1f, -0x1.5364000000000p-8f},
  {-0x1.d17e780000000p-1f, -0x1.4a6b000000000p-8f},
  {-0x1.d4134e0000000p-1f, -0x1.4165000000000p-8f},
  {-0x1.d696180000000p-1f, -0x1.3852000000000p-8f},
  {-0x1.d906bc0000000p-1f, -0x1.2f35000000000p-8f},
  {-0x1.db65260000000p-1f, -0x1.260b000000000p-8f},
  {-0x1.ddb13c0000000p-1f, -0x1.1cd5000000000p-8f},
  {-0x1.dfeae60000000p-1f, -0x1.1395000000000p-8f},
  {-0x1.e212100000000p-1f, -0x1.0a4a000000000p-8f},
  {-0x1.e426a40000000p-1f, -0x1.00f5000000000p-8f},
  {-0x1.e6288e0000000p-1f, -0x1.ef2c000000000p-9f},
  {-0x1.e817ba0000000p-1f, -0x1.dc5c000000000p-9f},
  {-0x1.e9f4160000000p-1f, -0x1.c976000000000p-9f},
  {-0x1.ebbd8c0000000p-1f, -0x1.b682000000000p-9f},
  {-0x1.ed740e0000000p-1f, -0x1.a37c000000000p-9f},
  {-0x1.ef178a0000000p-1f, -0x1.9066000000000p-9f},
  {-0x1.f0a7f00000000p-1f, -0x1.7d40000000000p-9f},
  {-0x1.f225300000000p-1f, -0x1.6a0a000000000p-9f},
  {-0x1.f38f3a0000000p-1f, -0x1.56ca000000000p-9f},
  {-0x1.f4e6040000000p-1f, -0x1.4378000000000p-9f},
  {-0x1.f6297c0000000p-1f, -0x1.301e000000000p-9f},
  {-0x1.f7599a0000000p-1f, -0x1.1cb6000000000p-9f},
  {-0x1.f876500000000p-1f, -0x1.0942000000000p-9f},
  {-0x1.f97f920000000p-1f, -0x1.eb8c000000000p-10f},
  {-0x1.fa75580000000p-1f, -0x1.c480000000000p-10f},
  {-0x1.fb57980000000p-1f, -0x1.9d60000000000p-10f},
  {-0x1.fc26480000000p-1f, -0x1.7630000000000p-10f},
  {-0x1.fce1600000000p-1f, -0x1.4ef4000000000p-10f},
  {-0x1.fd88da0000000p-1f, -0x1.27ac000000000p-10f},
  {-0x1.fe1cb00000000p-1f, -0x1.0054000000000p-10f},
  {-0x1.fe9cda0000000p-1f, -0x1.b1f0000000000p-11f},
  {-0x1.ff09560000000p-1f, -0x1.6320000000000p-11f},
  {-0x1.ff621e0000000p-1f, -0x1.1448000000000p-11f},
  {-0x1.ffa7300000000p-1f, -0x1.8ab0000000000p-12f},
  {-0x1.ffd8860000000p-1f, -0x1.d9c0000000000p-13f},
  {-0x1.fff6220000000p-1f, -0x1.3bc0000000000p-14f},
  {-0x1.0000000000000p+0f, 0x1.3bc0000000000p-14f},
  {-0x1.fff6220000000p-1f, 0x1.d9c0000000000p-13f},
  {-0x1.ffd8860000000p-1f, 0x1.8ab0000000000p-12f},
  {-0x1.ffa7300000000p-1f, 0x1.1448000000000p-11f},
  {-0x1.ff621e0000000p-1f, 0x1.6320000000000p-11f},
  {-0x1.ff09560000000p-1f, 0x1.b1f0000000000p-11f},
  {-0x1.fe9cda0000000p-1f, 0x1.0054000000000p-10f},
  {-0x1.fe1cb00000000p-1f, 0x1.27ac000000000p-10f},
  {-0x1.fd88da0000000p-1f, 0x1.4ef4000000000p-10f},
  {-0x1.fce1600000000p-1f, 0x1.7630000000000p-10f},
  {-0x1.fc26480000000p-1f, 0x1.9d60000000000p-10f},
  {-0x1.fb57980000000p-1f, 0x1.c480000000000p-10f},
  {-0x1.fa75580000000p-1f, 0x1.eb8c000000000p-10f},
  {-0x1.f97f920000000p-1f, 0x1.0942000000000p-9f},
  {-0x1.f876500000000p-1f, 0x1.1cb6000000000p-9f},
  {-0x1.f7599a0000000p-1f, 0x1.301e000000000p-9f},
  {-0x1.f6297c0000000p-1f, 0x1.4378000000000p-9f},
  {-0x1.f4e6040000000p-1f, 0x1.56ca000000000p-9f},
  {-0x1.f38f3a0000000p-1f, 0x1.6a0a000000000p-9f},
  {-0x1.f225300000000p-1f, 0x1.7d40000000000p-9f},
  {-0x1.f0a7f00000000p-1f, 0x1.9066000000000p-9f},
  {-0x1.ef178a0000000p-1f, 0x1.a37c000000000p-9f},
  {-0x1.ed740e0000000p-1f, 0x1.b682000000000p-9f},
  {-0x1.ebbd8c0000000p-1f, 0x1.c976000000000p-9f},
  {-0x1.e9f4160000000p-1f, 0x1.dc5c000000000p-9f},
  {-0x1.e817ba0000000p-1f, 0x1.ef2c000000000p-9f},
  {-0x1.e6288e0000000p-1f, 0x1.00f5000000000p-8f},
  {-0x1.e426a40000000p-1f, 0x1.0a4a000000000p-8f},
  {-0x1.e212100000000p-1f, 0x1.1395000000000p-8f},
  {-0x1.dfeae60000000p-1f, 0x1.1cd5000000000p-8f},
  {-0x1.ddb13c0000000p-1f, 0x1.260b000000000p-8f},
  {-0x1.db65260000000p-1f, 0x1.2f35000000000p-8f},
  {-0x1.d906bc0000000p-1f, 0x1.3852000000000p-8f},
  {-0x1.d696180000000p-1f, 0x1.4165000000000p-8f},
  {-0x1.d4134e0000000p-1f, 0x1.4a6b000000000p-8f},
  {-0x1.d17e780000000p-1f, 0x1.5364000000000p-8f},
  {-0x1.ced7b00000000p-1f, 0x1.5c50000000000p-8f},
  {-0x1.cc1f100000000p-1f, 0x1.652f000000000p-8f},
  {-0x1.c954b20000000p-1f, 0x1.6dff000000000p-8f},
  {-0x1.c678b40000000p-1f, 0x1.76c2000000000p-8f},
  {-0x1.c38b300000000p-1f, 0x1.7f77000000000p-8f},
  {-0x1.c08c420000000p-1f, 0x1.881c000000000p-8f},
  {-0x1.bd7c0a0000000p-1f, 0x1.90b2000000000p-8f},
  {-0x1.ba5aa60000000p-1f, 0x1.9939000000000p-8f},
  {-0x1.b728340000000p-1f, 0x1.a1b0000000000p-8f},
  {-0x1.b3e4d40000000p-1f, 0x1.aa17000000000p-8f},
  {-0x1.b090a60000000p-1f, 0x1.b26e000000000p-8f},
  {-0x1.ad2bca0000000p-1f, 0x1.bab4000000000p-8f},
  {-0x1.a9b6620000000p-1f, 0x1.c2e8000000000p-8f},
  {-0x1.a630920000000p-1f, 0x1.cb0c000000000p-8f},
  {-0x1.a29a7a0000000p-1f, 0x1.d31e000000000p-8f},
  {-0x1.9ef43e0000000p-1f, 0x1.db1d000000000p-8f},
  {-0x1.9b3e040000000p-1f, 0x1.e30a000000000p-8f},
  {-0x1.9777f00000000p-1f, 0x1.eae6000000000p-8f},
  {-0x1.93a2240000000p-1f, 0x1.f2ad000000000p-8f},
  {-0x1.8fbcca0000000p-1f, 0x1.fa62000000000p-8f},
  {-0x1.8bc8060000000p-1f, 0x1.0101800000000p-7f},
  {-0x1.87c4000000000p-1f, 0x1.04c8000000000p-7f},
  {-0x1.83b0e00000000p-1f, 0x1.0884800000000p-7f},
  {-0x1.7f8ece0000000p-1f, 0x1.0c37000000000p-7f},
  {-0x1.7b5df20000000p-1f, 0x1.0fdf000000000p-7f},
  {-0x1.771e760000000p-1f, 0x1.137c800000000p-7f},
  {-0x1.72d0840000000p-1f, 0x1.170f800000000p-7f},
  {-0x1.6e74460000000p-1f, 0x1.1a98000000000p-7f},
  {-0x1.6a09e60000000p-1f, 0x1.1e15000000000p-7f},
  {-0x1.6591920000000p-1f, 0x1.2187000000000p-7f},
  {-0x1.610b760000000p-1f, 0x1.24ee800000000p-7f},
  {-0x1.5c77bc0000000p-1f, 0x1.284a000000000p-7f},
  {-0x1.57d6940000000p-1f, 0x1.2b9a800000000p-7f},
  {-0x1.53282a0000000p-1f, 0x1.2edf800000000p-7f},
  {-0x1.4e6cac0000000p-1f, 0x1.3218800000000p-7f},
  {-0x1.49a44a0000000p-1f, 0x1.3546000000000p-7f},
  {-0x1.44cf320000000p-1f, 0x1.3867000000000p-7f},
  {-0x1.3fed960000000p-1f, 0x1.3b7d000000000p-7f},
  {-0x1.3affa20000000p-1f, 0x1.3e85800000000p-7f},
  {-0x1.36058c0000000p-1f, 0x1.4183000000000p-7f},
  {-0x1.30ff800000000p-1f, 0x1.4473800000000p-7f},
  {-0x1.2bedb20000000p-1f, 0x1.4757800000000p-7f},
  {-0x1.26d0540000000p-1f, 0x1.4a2e800000000p-7f},
  {-0x1.21a79a0000000p-1f, 0x1.4cf9800000000p-7f},
  {-0x1.1c73b40000000p-1f, 0x1.4fb7800000000p-7f},
  {-0x1.1734d60000000p-1f, 0x1.5268000000000p-7f},
  {-0x1.11eb360000000p-1f, 0x1.550c800000000p-7f},
  {-0x1.0c97040000000p-1f, 0x1.57a2800000000p-7f},
  {-0x1.07387a0000000p-1f, 0x1.5a2c800000000p-7f},
  {-0x1.01cfc80000000p-1f, 0x1.5ca8400000000p-7f},
  {-0x1.f8ba4e0000000p-2f, 0x1.5f17000000000p-7f},
  {-0x1.edc1960000000p-2f, 0x1.6178400000000p-7f},
  {-0x1.e2b5d40000000p-2f, 0x1.63cbc00000000p-7f},
  {-0x1.d797760000000p-2f, 0x1.6611800000000p-7f},
  {-0x1.cc66ea0000000p-2f, 0x1.6849800000000p-7f},
  {-0x1.c1249e0000000p-2f, 0x1.6a73c00000000p-7f},
  {-0x1.b5d1000000000p-2f, 0x1.6c8fc00000000p-7f},
  {-0x1.aa6c820000000p-2f, 0x1.6e9dc00000000p-7f},
  {-0x1.9ef7940000000p-2f, 0x1.709dc00000000p-7f},
  {-0x1.9372a60000000p-2f, 0x1.728f800000000p-7f},
  {-0x1.87de2a0000000p-2f, 0x1.7472c00000000p-7f},
  {-0x1.7c3a940000000p-2f, 0x1.7648000000000p-7f},
  {-0x1.7088540000000p-2f, 0x1.780ec00000000p-7f},
  {-0x1.64c7de0000000p-2f, 0x1.79c6c00000000p-7f},
  {-0x1.58f9a80000000p-2f, 0x1.7b70800000000p-7f},
  {-0x1.4d1e240000000p-2f, 0x1.7d0b400000000p-7f},
  {-0x1.4135ca0000000p-2f, 0x1.7e97c00000000p-7f},
  {-0x1.35410c0000000p-2f, 0x1.8015400000000p-7f},
  {-0x1.2940620000000p-2f, 0x1.8183c00000000p-7f},
  {-0x1.1d34440000000p-2f, 0x1.82e3c00000000p-7f},
  {-0x1.111d260000000p-2f, 0x1.8434c00000000p-7f},
  {-0x1.04fb800000000p-2f, 0x1.8576800000000p-7f},
  {-0x1.f19f980000000p-3f, 0x1.86a9a00000000p-7f},
  {-0x1.d934fe0000000p-3f, 0x1.87cd800000000p-7f},
  {-0x1.c0b8260000000p-3f, 0x1.88e2400000000p-7f},
  {-0x1.a82a020000000p-3f, 0x1.89e7e00000000p-7f},
  {-0x1.8f8b840000000p-3f, 0x1.8ade600000000p-7f},
  {-0x1.76dd9e0000000p-3f, 0x1.8bc5a00000000p-7f},
  {-0x1.5e21440000000p-3f, 0x1.8c9d800000000p-7f},
  {-0x1.45576c0000000p-3f, 0x1.8d66600000000p-7f},
  {-0x1.2c81060000000p-3f, 0x1.8e1fa00000000p-7f},
  {-0x1.139f0c0000000p-3f, 0x1.8ec9900000000p-7f},
  {-0x1.f564e60000000p-4f, 0x1.8f64500000000p-7f},
  {-0x1.c3785c0000000p-4f, 0x1.8fef800000000p-7f},
  {-0x1.917a6c0000000p-4f, 0x1.906b600000000p-7f},
  {-0x1.5f6d000000000p-4f, 0x1.90d7b00000000p-7f},
  {-0x1.2d520a0000000p-4f, 0x1.9134b00000000p-7f},
  {-0x1.f656e80000000p-5f, 0x1.9182200000000p-7f},
  {-0x1.91f6600000000p-5f, 0x1.91c0200000000p-7f},
  {-0x1.2d86580000000p-5f, 0x1.91eea00000000p-7f},
  {-0x1.9215600000000p-6f, 0x1.920da00000000p-7f},
  {-0x1.921d200000000p-7f, 0x1.921d200000000p-7f},
};
static_assert(sizeof(terrain_sine_coefficients) == 4096,
              "Terrain sine coefficient resource size changed");
// END GENERATED TERRAIN SINE COEFFICIENTS

// Original Sine assumes a nonnegative phase. Fast trajectory modulation can
// violate that assumption; periodic wrapping preserves the intended sine.
inline float TerrainSine(float phase) {
  if (phase < 0.0f) phase -= floorf(phase);
  const float index = phase * kSineLUTSize;
  const int32_t integral = static_cast<int32_t>(index);
  const float fractional = index - static_cast<float>(integral);
  const float* coefficients = terrain_sine_coefficients[integral & 511];
  return coefficients[0] + coefficients[1] * fractional;
}

// Reuse FM's immutable 8192-point table; only the final AUX sine is quantized.
extern const float fm_sub_sine[8192];
inline float TerrainAuxSine(float phase) {
  if (phase < 0.0f) phase -= floorf(phase);
  return fm_sub_sine[static_cast<int32_t>(phase * 8192.0f + 0.5f) & 8191];
}

inline float Squash(float x, float a) {
  x *= a;
  return x / (1.0f + fabsf(x));
}

template<int terrain_index>
__attribute__((always_inline)) inline float WaveTerrainEngine::TerrainFixed(float x, float y) {
  // Keep the original positive offsets and formulas. TerrainSine also
  // handles negative arguments reached during rapid trajectory modulation.
  const float k = 4.0f;
  switch (terrain_index) {
    case 0:
      {
        return (Squash(TerrainSine(k + x * 1.273f), 2.0f) - \
            TerrainSine(k + y * (x + 1.571f) * 0.637f)) * 0.57f;
      }
      break;
    case 1:
      {
        const float xy = x * y;
        return TerrainSine(k + TerrainSine(k + (x + y) * 0.637f) / (0.2f + xy * xy) * 0.159f);
      }
      break;
    case 2:
      {
        const float xy = x * y;
        return TerrainSine(k + TerrainSine(k + 2.387f * xy) / (0.350f + xy * xy) * 0.159f);
      }
      break;
    case 3:
      {
        const float xy = x * y;
        const float xys = (x - 0.25f) * (y + 0.25f);
        return TerrainSine(k + xy / (2.0f + fabsf(5.0f * xys)) * 6.366f);
      }
      break;
    case 4:
      {
        return TerrainSine(
          0.159f / (0.170f + fabsf(y - 0.25f)) + \
          0.477f / (0.350f + fabsf((x + 0.5f) * (y + 1.5f))) + k);
      }
      break;
    case 5:
    case 6:
    case 7:
      {
        return TerrainLookupWT(x, y, 2 - (terrain_index - 5));
      }
      break;
    default:
      return 0.0f;
  }
}

inline float WaveTerrainEngine::Terrain(float x, float y, int terrain_index) {
  switch (terrain_index) {
    case 0: return TerrainFixed<0>(x, y);
    case 1: return TerrainFixed<1>(x, y);
    case 2: return TerrainFixed<2>(x, y);
    case 3: return TerrainFixed<3>(x, y);
    case 4: return TerrainFixed<4>(x, y);
    case 5: return TerrainFixed<5>(x, y);
    case 6: return TerrainFixed<6>(x, y);
    case 7: return TerrainFixed<7>(x, y);
    default: return 0.0f;
  }
}

// The pair is fixed at compile time; both oversampled positions use it.
// MAIN averages both positions; AUX evaluates each neighbor at their midpoint.
template<EngineOutput output, int left, bool single>
__attribute__((always_inline)) inline float WaveTerrainEngine::RenderSample(
    float offset, float fraction, const float* path_x, const float* path_y) {
  float sum = 0.0f;
  for (size_t j = 0; j < (output == ENGINE_OUTPUT_AUX ? 1 : 2); ++j) {
    const float px = output == ENGINE_OUTPUT_AUX ?
        (path_x[0] + path_x[1]) * 0.5f : path_x[j];
    const float y = output == ENGINE_OUTPUT_AUX ?
        (path_y[0] + path_y[1]) * 0.5f : path_y[j];
    const float x = px * (1.0f - fabsf(offset)) + offset;
    float z;
    if constexpr (!single && (left == 5 || left == 6)) {
      z = TerrainLookupWTPair(x, y, 7 - left, fraction);
    } else {
      const float a = TerrainFixed<left>(x, y);
      z = a;
      if constexpr (!single) {
        const float b = TerrainFixed<left + 1>(x, y);
        z = a + (b - a) * fraction;
      }
    }
    if constexpr (output == ENGINE_OUTPUT_MAIN) sum += z;
    else sum += y + z;
  }
  if constexpr (output == ENGINE_OUTPUT_MAIN) return 0.5f * sum;
  else return TerrainAuxSine(1.0f + 0.5f * sum);
}

// Advance H by repeated additions, as in Plaits. Test the next value
// against this pair's bounds to avoid another conversion and dispatch.
// Multiplying the increment would round boundary crossings differently.
template<EngineOutput output, int left>
__attribute__((always_inline)) inline size_t WaveTerrainEngine::RenderRun(
    float& z, ParameterInterpolator& terrain, ParameterInterpolator& offset,
    const float* path_x, const float* path_y, float* out, size_t i, size_t size) {
  // Separate runs keep the single/pair choice out of the sample loop. Pure
  // surfaces have no state to advance when their contribution is omitted.
  if (z == float(left) || (left == 7 && z >= kSingleLastTerrain && z <= 7.0f)) {
    do {
      out[i] = RenderSample<output, left, true>(
          offset.Next(), 0.0f, path_x + 2 * i, path_y + 2 * i);
      if (++i == size) break;
      z = terrain.Next();
    } while (left == 7 ? (z >= kSingleLastTerrain && z <= 7.0f) : z == float(left));
    return i;
  }
  do {
    out[i] = RenderSample<output, left>(
        offset.Next(), z - float(left), path_x + 2 * i, path_y + 2 * i);
    if (++i == size) break;
    z = terrain.Next();
  } while (z > float(left) &&
      z < (left == 6 ? kSingleLastTerrain : float(left + 1)));
  return i;
}

void WaveTerrainEngine::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  RenderAtFrequency(parameters, NoteToFrequency(parameters.note),
      out, aux, size);
}

void WaveTerrainEngine::RenderAtFrequency(
    const EngineParameters& parameters, float frequency,
    float* out, float* aux, size_t size, EngineOutput output) {
  if (!size || size > kMaxBlockSize) return;
  if (output == ENGINE_OUTPUT_MAIN) {
    RenderBlock<ENGINE_OUTPUT_MAIN>(parameters, frequency, out, aux, size);
  } else if (output == ENGINE_OUTPUT_AUX) {
    RenderBlock<ENGINE_OUTPUT_AUX>(parameters, frequency, out, aux, size);
  } else {
    RenderBlock<ENGINE_OUTPUT_BOTH>(parameters, frequency, out, aux, size);
  }
}

template<EngineOutput output>
void WaveTerrainEngine::RenderBlock(
    const EngineParameters& parameters, float frequency,
    float* out, float* aux, size_t size) {
  const size_t kOversampling = 2;
  const float kScale = 1.0f / float(kOversampling);

  float* path_x = &temp_buffer_[0];
  float* path_y = &temp_buffer_[kOversampling * size];
  
  const float f0 = frequency;
  const float attenuation = max(1.0f - 8.0f * f0, 0.0f);
  const float radius = 0.1f + 0.9f * parameters.timbre * attenuation * \
      (2.0f - attenuation);

  // Use the "magic sine" algorithm to generate sin and cos functions for the
  // trajectory coordinates.
  const float inverse_size = 1.0f / static_cast<float>(size);
  path_.RenderQuadrature(
      f0 * kScale, radius, path_x, path_y, size * kOversampling,
      inverse_size * kScale);
  
  ParameterInterpolator offset(&offset_, 1.9f * parameters.morph - 1.0f, inverse_size);
  // Round the target before subtracting the previous state. This prevents
  // fused multiply-add instructions from changing H's interpolation history
  // between the specialized rendering loops.
  const volatile float terrain_target =
      min(parameters.harmonics * 1.05f, 1.0f) * kLastTerrain;
  ParameterInterpolator terrain(&terrain_, terrain_target, inverse_size);
  
  size_t i = 0;
  float z = terrain.Next();
  while (i < size) {
    MAKE_INTEGRAL_FRACTIONAL(z);
    if constexpr (output != ENGINE_OUTPUT_BOTH) {
      // Select once for a run of samples in the same pair. The helper leaves
      // z at the next unconsumed sample, including exact boundary crossings.
      const int pair = z >= kSingleLastTerrain && z <= 7.0f ? 7 : z_integral;
      switch (pair) {
        case 0:
          i = RenderRun<output, 0>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        case 1:
          i = RenderRun<output, 1>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        case 2:
          i = RenderRun<output, 2>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        case 3:
          i = RenderRun<output, 3>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        case 4:
          i = RenderRun<output, 4>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        case 5:
          i = RenderRun<output, 5>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        case 6:
          i = RenderRun<output, 6>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        case 7:
          i = RenderRun<output, 7>(z, terrain, offset, path_x, path_y, out, i, size);
          continue;
        default: break;  // Generic fallback for direct calls outside 0..7.
      }
    }
    const float x_offset = offset.Next();
    size_t ij = i * kOversampling;
    float out_s = 0.0f;
    
    if constexpr (output != ENGINE_OUTPUT_AUX) {
      for (size_t j = 0; j < kOversampling; ++j) {
        const float x = path_x[ij] * (1.0f - fabsf(x_offset)) + x_offset;
        const float y = path_y[ij];
        ++ij;

        float surface;
        if (z >= kSingleLastTerrain && z <= 7.0f) {
          surface = TerrainFixed<7>(x, y);
        } else if ((z_integral == 5 || z_integral == 6) && z_fractional != 0.0f) {
          surface = TerrainLookupWTPair(x, y, 7 - z_integral, z_fractional);
        } else {
          const float z0 = Terrain(x, y, z_integral);
          surface = z0;
          if (z_fractional != 0.0f) {
            const float z1 = Terrain(x, y, z_integral + 1);
            surface = z0 + (z1 - z0) * z_fractional;
          }
        }
        if (output != ENGINE_OUTPUT_AUX) out_s += surface;

      }
    }
    if (output != ENGINE_OUTPUT_AUX) out[i] = kScale * out_s;
    if (output != ENGINE_OUTPUT_MAIN) {
      // One surface evaluation per neighbor at the midpoint. The path itself
      // still advances twice, so MAIN/AUX switches retain the same phase/state.
      const size_t p = i * 2;
      const float px = (path_x[p] + path_x[p + 1]) * 0.5f;
      const float y = (path_y[p] + path_y[p + 1]) * 0.5f;
      const float x = px * (1.0f - fabsf(x_offset)) + x_offset;
      float surface;
      if (z >= kSingleLastTerrain && z <= 7.0f) {
        surface = TerrainFixed<7>(x, y);
      } else if ((z_integral == 5 || z_integral == 6) && z_fractional != 0.0f) {
        surface = TerrainLookupWTPair(x, y, 7 - z_integral, z_fractional);
      } else {
        const float a = Terrain(x, y, z_integral);
        surface = a;
        if (z_fractional != 0.0f) {
          const float b = Terrain(x, y, z_integral + 1);
          surface = a + (b - a) * z_fractional;
        }
      }
      const float aux_sample = TerrainAuxSine(1.0f + 0.5f * (y + surface));
      if (output == ENGINE_OUTPUT_BOTH) aux[i] = aux_sample;
      else out[i] = aux_sample;
    }
    if (++i < size) z = terrain.Next();
  }
}

}  // namespace plaits
