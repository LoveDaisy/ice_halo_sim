#ifndef CORE_COLOR_UTIL_H_
#define CORE_COLOR_UTIL_H_

#include <cstddef>

#include "util/color_data.hpp"

namespace lumice {

// Public constants previously declared in server/render.hpp — re-declared here
// so core/ TUs need not include server headers.
inline constexpr int kCmfMinWavelength = 360;
inline constexpr int kCmfMaxWavelength = 830;

// Display brightness baseline: maps the physically-derived per-pixel radiance
// to a visually reasonable [0,1] range at EV=0. The average illuminated pixel
// appears at ~5% brightness, so bright halo features (~20x average) approach
// full white. Independent of resolution and FOV — use EV (intensity_factor)
// for user-controlled brightness adjustment.
inline constexpr float kNormScale = 0.08f;

// Scatter-add `v[i]` weighted by the CIE 1931 color-matching functions at
// the given wavelength into the XYZ image. When xy == nullptr, samples write
// in-order to xyz[3*i .. 3*i+2]; otherwise xy[i] is the flat pixel index.
// No bounds check — caller must clamp xy[] to [0, W*H).
//
// Inline-defined so that scatter_accum.hpp and server/render.cpp can both
// include this header without an ODR collision.
inline void SpectrumToXyz(float wl, const float* v, const int* xy, float* xyz, size_t num = 1) {
  int wl_key = static_cast<int>(wl + 0.5f);
  if (wl_key < kCmfMinWavelength || wl_key > kCmfMaxWavelength) {
    return;
  }

  for (size_t i = 0; i < num; i++) {
    size_t idx = xy == nullptr ? i * 3 : static_cast<size_t>(xy[i]) * 3;
    xyz[idx + 0] += kCmfX[wl_key - kCmfMinWavelength] * v[i];
    xyz[idx + 1] += kCmfY[wl_key - kCmfMinWavelength] * v[i];
    xyz[idx + 2] += kCmfZ[wl_key - kCmfMinWavelength] * v[i];
  }
}

// Single-ray Y-only helper (task-336.2 component lanes).
// Returns kCmfY[wl] * w for an in-range wavelength; 0 otherwise.
// Deliberately mirrors SpectrumToXyz's wavelength clipping (rounding + range check)
// so per-component Y lanes accumulate a strict Y-slice of the same batch.
inline float SpectrumToYSingle(float wl, float w) {
  int wl_key = static_cast<int>(wl + 0.5f);
  if (wl_key < kCmfMinWavelength || wl_key > kCmfMaxWavelength) {
    return 0.0f;
  }
  return kCmfY[wl_key - kCmfMinWavelength] * w;
}

// Per-ray variant — each ray i carries its own wavelength wl_per_ray[i].
// Used by the Metal/DR-3 path where the photon's lifetime wavelength tag is
// derived from a host-uploaded wavelength pool (see metal_trace_backend.mm's
// ComputeWlPool). xy / xyz semantics mirror SpectrumToXyz above. Out-of-range
// wavelengths are silently skipped per ray.
inline void SpectrumToXyzPerRay(const float* wl_per_ray, const float* v, const int* xy, float* xyz, size_t num) {
  for (size_t i = 0; i < num; i++) {
    int wl_key = static_cast<int>(wl_per_ray[i] + 0.5f);
    if (wl_key < kCmfMinWavelength || wl_key > kCmfMaxWavelength) {
      continue;
    }
    int idx_cmf = wl_key - kCmfMinWavelength;
    size_t pidx = xy == nullptr ? i * 3 : static_cast<size_t>(xy[i]) * 3;
    xyz[pidx + 0] += kCmfX[idx_cmf] * v[i];
    xyz[pidx + 1] += kCmfY[idx_cmf] * v[i];
    xyz[pidx + 2] += kCmfZ[idx_cmf] * v[i];
  }
}

}  // namespace lumice

#endif  // CORE_COLOR_UTIL_H_
