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
  kGenericReject = 4u,           // kGaussian / kUniform / kZigzag / kLaplacian
  kLaplacianTightEnvelope = 5u,  // kLaplacian near-pole optimization (Gamma(2,b) proposal)
  kLutInverseCdf = 6u,           // unified area-measure inverse-CDF LUT (330.2); numeric value
                                 // MUST match lm_pcg::kLatPathLutInverseCdf (pcg_shared.h)
};

// Near-pole colatitude trigger threshold for tight-envelope area-measure
// sampling — shared between the kGaussian (Rayleigh) and kLaplacian (Gamma(2,b))
// branches of SelectLatPath. The threshold is a property of the sin(theta) <= theta
// upper bound (doc/near-pole-area-measure-sampling.md §2.2), independent of the
// proposal distribution family.
constexpr float kPolarThresholdRad = 0.5f * math::kDegreeToRad;

// Scale upper bounds for the two tight-envelope branches.
// - Gaussian sigma cap 60° (doc §附录 MC-validated).
// - Laplacian b cap 60° (scrum-328.4 Step 1 calibration exp4: safety_hit_pct=0
//   at b=90° with N=100k; mean_attempts stays <= 2 within cap. 60° chosen to
//   mirror the Gaussian cap for reader-side symmetry, not because Laplacian
//   fails at b>60°).
constexpr float kMaxTightEnvelopeSigmaRad = 60.0f * math::kDegreeToRad;
constexpr float kMaxTightEnvelopeLaplacianBRad = 60.0f * math::kDegreeToRad;

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
//   - kUniform:   proposal support is exactly [mean-std/2, mean+std/2] (uniform
//     GetUniform() maps to (u-0.5)*std+mean; see math.cpp RandomNumberGenerator::Get
//     and pcg_get_dist mirror). Tight-envelope M = cos(max(|mean|-std/2, 0)°) is
//     exact (not an approximation): when the support brackets the equator the
//     max()-clamp yields M=1 (bit-identical to the prior behavior); when the
//     support is bounded away from equator M<1 lifts the acceptance rate.
//     Full-sphere uniform (IsFullSphereUniform()==true) is routed to
//     kFullSphere in SelectLatPath and does not reach this branch.
//   - default: unknown/future type → conservative M = 1.
inline float ComputeJacobianEnvelope(const Distribution& dist) {
  switch (dist.type) {
    case DistributionType::kGaussian:
      return std::cos(std::max(std::abs(dist.mean) - 3.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kZigzag:
      return std::cos(std::max(std::abs(dist.mean) - dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kLaplacian:
      return std::cos(std::max(std::abs(dist.mean) - 5.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kUniform:
      return std::cos(std::max(std::abs(dist.mean) - 0.5f * dist.std, 0.0f) * math::kDegreeToRad);
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
// RandomSampler::SampleSphericalPointsSph setup (math.cpp:437,
// metal_trace_backend.mm:1455, cuda_trace_backend.cu:290).
//
// Rayleigh threshold: colatitude_center < 0.5° (sigma-independent). The
// tight-envelope Rayleigh sampler in pcg_shared.h::sample_lat_lon_roll is
// exact (accept = sin(theta)/theta, M=1) for any sigma up to ~60°
// (doc/near-pole-area-measure-sampling.md §附录, validated by MC), so the
// previous "colatitude_center + 3sigma < 0.5°" guard (which the tight sampler
// no longer needs) was relaxed. The prior guard misrouted typical near-pole
// Parhelia/Parry configurations (mean=90°, sigma=5°, colatitude_center=0 but
// 3sigma=15°) to kLatPathGenericReject with a ~27% acceptance rate.
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
    float lat_mean_rad = axis_dist.latitude_dist.mean * math::kDegreeToRad;
    float lat_std_rad = axis_dist.latitude_dist.std * math::kDegreeToRad;
    float colatitude_center = math::kPi_2 - std::abs(lat_mean_rad);
    bool use_rayleigh = colatitude_center < kPolarThresholdRad && lat_std_rad < kMaxTightEnvelopeSigmaRad;
    if (use_rayleigh) {
      return { LatPathKind::kRayleigh, 1.0f };
    }
    return { LatPathKind::kGenericReject, ComputeJacobianEnvelope(axis_dist.latitude_dist) };
  }
  if (lat_type == DistributionType::kLaplacian) {
    // Laplacian tight-envelope branch (scrum-328.4). Trigger uses the same
    // colatitude_center threshold as the Gaussian branch — the sin(theta) <= theta
    // upper bound is a geometric property, not proposal-family-dependent. The scale
    // (b) cap is a separate constant validated by MC in exp4 (see comment beside
    // kMaxTightEnvelopeLaplacianBRad).
    float lat_mean_rad = axis_dist.latitude_dist.mean * math::kDegreeToRad;
    float lat_scale_rad = axis_dist.latitude_dist.std * math::kDegreeToRad;  // Distribution.std carries Laplace b
    float colatitude_center = math::kPi_2 - std::abs(lat_mean_rad);
    bool use_laplacian_tight = colatitude_center < kPolarThresholdRad && lat_scale_rad < kMaxTightEnvelopeLaplacianBRad;
    if (use_laplacian_tight) {
      return { LatPathKind::kLaplacianTightEnvelope, 1.0f };
    }
    return { LatPathKind::kGenericReject, ComputeJacobianEnvelope(axis_dist.latitude_dist) };
  }
  // kUniform / kZigzag
  return { LatPathKind::kGenericReject, ComputeJacobianEnvelope(axis_dist.latitude_dist) };
}

}  // namespace lat_path
}  // namespace lumice

#endif  // LM_LAT_PATH_SELECTION_H_
