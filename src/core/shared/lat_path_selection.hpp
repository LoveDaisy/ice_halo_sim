// Host-side latitude sampling path decision + Jacobian rejection envelope.
//
// Single source (scrum-328.2 Step 4) for the three previously mirrored copies
// of the Rayleigh/GenericReject/GaussLegacy/NoRandom branch selection and the
// envelope constant M used as the acceptance ratio cos(phi)/M:
//   - math.cpp:416-441 file-static ComputeJacobianEnvelope (host CPU sampler)
//   - metal_trace_backend.mm:186-214 ComputeJacobianEnvelopeForDeviceGen
//   - cuda_trace_backend.cu:254-273 CudaJacobianEnvelopeForDeviceGen
//
// Design invariant: this header is HOST-ONLY. It is called BEFORE kernel
// dispatch (to populate GenRootKernelParams.lat_path / lat_rejection_m), never
// from Metal shader / CUDA device code. Device kernels consume the numeric
// wire value via `GenRootKernelParams::lat_path` — see
// src/core/shared/pcg_shared.h's `lm_pcg::kLatPath*` constants for the
// device-side sink of these values. The numeric encoding (0..4) is asserted
// pairwise identical between LatPathKind and lm_pcg::kLatPath* by the two
// device-backend TUs (metal_trace_backend.mm / cuda_trace_backend.cu) that
// include both headers.
#ifndef LM_LAT_PATH_SELECTION_H_
#define LM_LAT_PATH_SELECTION_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "core/math.hpp"

namespace lumice {
namespace lat_path {

// Latitude-sampling path taxonomy — numeric values MUST match lm_pcg::kLatPath*
// in src/core/shared/pcg_shared.h (device-side). See device-backend TUs for the
// pairwise static_assert.
enum class LatPathKind : uint32_t {
  kFullSphere = 0u,
  kNoRandom = 1u,
  kRayleigh = 2u,  // kGaussian near-pole optimization
  kGaussLegacy = 3u,
  kGenericReject = 4u,  // kGaussian / kUniform / kZigzag / kLaplacian
};

constexpr uint32_t ToWireValue(LatPathKind kind) {
  return static_cast<uint32_t>(kind);
}

// Host-side rejection envelope constant M for Jacobian rejection sampling of
// latitude. `dist` uses degrees (mean/std in the AxisDistribution unit
// convention); output is dimensionless (cos-space M used as the acceptance
// ratio accept_ratio = cos(phi_rad) / M).
//
// Branch semantics (kept bit-identical to the three mirrors that were removed):
//   - kGaussian:  proposal range ~ [mean-3σ, mean+3σ] → M = cos(max(|mean|-3σ, 0)°)
//   - kZigzag:    proposal range |std·sin(2πU)+mean| → M = cos(max(|mean|-std, 0)°)
//   - kLaplacian: 5b cutoff (covers 99.3% of Laplace mass) → cos(max(|mean|-5b, 0)°)
//   - kUniform / default: M = 1
inline float ComputeJacobianEnvelope(const Distribution& dist) {
  switch (dist.type) {
    case DistributionType::kGaussian:
      return std::cos(std::max(std::abs(dist.mean) - 3.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kZigzag:
      return std::cos(std::max(std::abs(dist.mean) - dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kLaplacian:
      return std::cos(std::max(std::abs(dist.mean) - 5.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kUniform:
    default:
      return 1.0f;
  }
}

struct LatPathDecision {
  LatPathKind kind;
  float rejection_m;  // 1.0 for all paths except kGenericReject
};

// Select latitude-sampling path + rejection envelope for a given axis
// distribution. Mirrors the branch structure at
// RandomSampler::SampleSphericalPointsSph setup (math.cpp:459-475).
//
// Rayleigh threshold: colatitude_center + 3σ < 0.5° — same value used by
// math.cpp:469, metal_trace_backend.mm:1466, cuda_trace_backend.cu:307.
inline LatPathDecision SelectLatPath(const AxisDistribution& axis_dist) {
  if (axis_dist.IsFullSphereUniform()) {
    return { LatPathKind::kFullSphere, 1.0f };
  }
  auto lat_type = axis_dist.latitude_dist.type;
  if (lat_type == DistributionType::kNoRandom) {
    return { LatPathKind::kNoRandom, 1.0f };
  }
  if (lat_type == DistributionType::kGaussianLegacy) {
    return { LatPathKind::kGaussLegacy, 1.0f };
  }
  if (lat_type == DistributionType::kGaussian) {
    constexpr float kPolarThresholdRad = 0.5f * math::kDegreeToRad;
    float lat_mean_rad = axis_dist.latitude_dist.mean * math::kDegreeToRad;
    float lat_std_rad = axis_dist.latitude_dist.std * math::kDegreeToRad;
    float colatitude_center = math::kPi_2 - std::abs(lat_mean_rad);
    bool use_rayleigh = (colatitude_center + 3.0f * lat_std_rad) < kPolarThresholdRad;
    if (use_rayleigh) {
      return { LatPathKind::kRayleigh, 1.0f };
    }
    return { LatPathKind::kGenericReject, ComputeJacobianEnvelope(axis_dist.latitude_dist) };
  }
  // kUniform / kZigzag / kLaplacian
  return { LatPathKind::kGenericReject, ComputeJacobianEnvelope(axis_dist.latitude_dist) };
}

}  // namespace lat_path
}  // namespace lumice

#endif  // LM_LAT_PATH_SELECTION_H_
