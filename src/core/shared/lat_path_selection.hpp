// Host-side latitude sampling path decision.
//
// Single source (scrum-328.2 Step 4) for the latitude path selection shared by
// the host CPU sampler (math.cpp::SampleSphericalPointsSph) and the two device
// backends (metal_trace_backend.mm / cuda_trace_backend.cu BuildGenRootParams).
//
// 330.3: the Rayleigh / Laplacian-tight-envelope / generic-rejection branches
// were retired — every non-degenerate latitude distribution now routes to the
// unified inverse-CDF area-measure LUT (kLutInverseCdf, landed 330.2). The
// Jacobian rejection envelope (ComputeJacobianEnvelope) is gone with them; the
// LUT is rejection-free so rejection_m is a constant 1.0 on every path.
//
// Design invariant: this header is HOST-ONLY. It is called BEFORE kernel
// dispatch (to populate GenRootKernelParams.lat_path), never from Metal shader
// / CUDA device code. Device kernels consume the numeric wire value via
// `GenRootKernelParams::lat_path` — see src/core/shared/pcg_shared.h's
// `lm_pcg::kLatPath*` constants for the device-side sink of these values. The
// numeric encoding is asserted pairwise identical between LatPathKind and
// lm_pcg::kLatPath* by the two device-backend TUs (metal_trace_backend.mm /
// cuda_trace_backend.cu) that include both headers.
#ifndef LM_LAT_PATH_SELECTION_H_
#define LM_LAT_PATH_SELECTION_H_

#include <cstdint>

#include "core/math.hpp"

namespace lumice {
namespace lat_path {

// Latitude-sampling path taxonomy — numeric values MUST match lm_pcg::kLatPath*
// in src/core/shared/pcg_shared.h (device-side). See device-backend TUs for the
// pairwise static_assert. Values 2/4/5 (retired Rayleigh / GenericReject /
// LaplacianTightEnvelope, 330.3) are intentionally left as gaps so the wire
// encoding of the surviving kinds does not shift.
enum class LatPathKind : uint32_t {
  kFullSphere = 0u,
  kNoRandom = 1u,
  kGaussLegacy = 3u,
  kLutInverseCdf = 6u,  // unified area-measure inverse-CDF LUT (330.2); numeric value
                        // MUST match lm_pcg::kLatPathLutInverseCdf (pcg_shared.h)
};

constexpr uint32_t ToWireValue(LatPathKind kind) {
  return static_cast<uint32_t>(kind);
}

struct LatPathDecision {
  LatPathKind kind;
  float rejection_m;  // always 1.0 (the surviving paths are all rejection-free); retained
                      // because the device wire field GenRootKernelParams::lat_rejection_m
                      // still carries it (dead post-330.3, struct layout frozen).
};

// Select latitude-sampling path for a given axis distribution. Shared single
// source for the host CPU sampler and both GPU backends (scrum-328.2 Step 4).
//
// 330.3: kGaussian / kLaplacian / kUniform / kZigzag all route to the unified
// inverse-CDF LUT (kLutInverseCdf). Only the degenerate paths keep dedicated
// handling: full-sphere uniform (kFullSphere), single deterministic orientation
// (kNoRandom), and the legacy no-Jacobian Gaussian (kGaussLegacy).
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
  // kGaussian / kLaplacian / kUniform / kZigzag → unified area-measure LUT (330.2).
  return { LatPathKind::kLutInverseCdf, 1.0f };
}

}  // namespace lat_path
}  // namespace lumice

#endif  // LM_LAT_PATH_SELECTION_H_
