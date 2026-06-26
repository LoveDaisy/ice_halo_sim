#ifndef CORE_BACKEND_WL_POOL_H_
#define CORE_BACKEND_WL_POOL_H_

#include <cassert>
#include <cstdint>
#include <vector>

#include "core/color_util.hpp"  // kCmfMinWavelength / kCmfMaxWavelength
#include "core/crystal.hpp"     // Crystal::GetRefractiveIndex
#include "util/color_data.hpp"  // kCmfX / kCmfY / kCmfZ
#include "util/env_knobs.hpp"   // env::WlPoolSize
#include "util/illuminant.hpp"  // GetIlluminantSpd / IlluminantType
#include "util/logger.hpp"      // Logger

namespace lumice {

// Per-ray wavelength pool (scrum-268.8 DR-3), single-sourced for the Metal and
// CUDA backends (296.6 promoted this out of metal_trace_backend.mm). Each entry
// holds the optics for one sampled wavelength; a device root-gen / host fallback
// draws a per-ray wl_idx in [0, M) and tags the ray with it for its whole
// lifetime. The MSL mirror (src/core/metal/lumice_trace.metal) and the CUDA
// device-side struct MUST match this 20-byte layout (static_assert below).
//   * n_idx / spd_weight: read on-device (refraction + ray weight).
//   * cmf_x/y/z: Metal's on-device XYZ projection. The exit-seam path (CUDA, and
//     Metal's live path) instead reconstructs the wavelength host-side from
//     wl_idx (simulator.cpp) and applies CMF in the consumer, so CUDA's device
//     pool leaves cmf_* unused — they stay in the shared struct for layout
//     parity and Metal's harness path.
struct WlEntry {
  float n_idx;
  float spd_weight;
  float cmf_x;
  float cmf_y;
  float cmf_z;
};
static_assert(sizeof(WlEntry) == 20u, "WlEntry must stay 20B (5 floats) — mirror the MSL + CUDA device structs");

// Default M = 64 (≈6.25 nm resolution across [380, 780] nm). Overridable via
// LUMICE_WL_POOL_SIZE; capped at 255 so a uint8_t wl_idx in ExitRayRecord stays
// representable end-to-end.
constexpr uint32_t kWlPoolSizeDefault = 64u;
constexpr uint32_t kWlPoolSizeMax = 255u;

inline uint32_t ResolveWlPoolSize(Logger& logger) {
  // Reads LUMICE_WL_POOL_SIZE via util/env_knobs (the single registered getenv
  // site; see doc/env-var-policy.md).
  return env::WlPoolSize(logger, kWlPoolSizeDefault, kWlPoolSizeMax);
}

inline void ComputeCmf(float wl, float& cie_x, float& cie_y, float& cie_z) {
  int wl_key = static_cast<int>(wl + 0.5f);
  if (wl_key < kCmfMinWavelength || wl_key > kCmfMaxWavelength) {
    cie_x = cie_y = cie_z = 0.0f;
    return;
  }
  int idx = wl_key - kCmfMinWavelength;
  cie_x = kCmfX[idx];
  cie_y = kCmfY[idx];
  cie_z = kCmfZ[idx];
}

// Build the per-(crystal, spectrum) WlEntry table. M >= 1 is the caller's
// responsibility. illuminant_mode == true samples M wavelengths uniformly with
// SPD weights from the standard illuminant table; otherwise the pool degenerates
// to a single entry carrying spec_wl / spec_weight (discrete-wavelength path),
// replicated across all M slots so a PCG % M lookup stays well-defined.
inline void ComputeWlPool(const Crystal& crystal, bool illuminant_mode, IlluminantType illuminant, float spec_wl,
                          float spec_weight, uint32_t M, std::vector<WlEntry>& out) {
  assert(M > 0u);
  out.resize(M);
  if (illuminant_mode) {
    for (uint32_t m = 0; m < M; ++m) {
      // Mid-point sampling across [380, 780] — preserves the simulator.cpp
      // uniform-PDF semantics (each entry covers a 400/M nm slice).
      float wl = 380.0f + (static_cast<float>(m) + 0.5f) * 400.0f / static_cast<float>(M);
      WlEntry& e = out[m];
      e.n_idx = crystal.GetRefractiveIndex(wl);
      e.spd_weight = GetIlluminantSpd(illuminant, wl);
      ComputeCmf(wl, e.cmf_x, e.cmf_y, e.cmf_z);
    }
  } else {
    WlEntry e0{};
    e0.n_idx = crystal.GetRefractiveIndex(spec_wl);
    e0.spd_weight = spec_weight;
    ComputeCmf(spec_wl, e0.cmf_x, e0.cmf_y, e0.cmf_z);
    for (uint32_t m = 0; m < M; ++m) {
      out[m] = e0;
    }
  }
}

}  // namespace lumice

#endif  // CORE_BACKEND_WL_POOL_H_
