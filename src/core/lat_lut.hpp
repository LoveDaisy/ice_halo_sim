#ifndef LM_LAT_LUT_H_
#define LM_LAT_LUT_H_

#include <array>
#include <cstdint>

#include "core/math.hpp"

namespace lumice {

// Precomputed inverse-CDF lookup table for unified area-measure latitude sampling
// (330.2 core-lut-sampler; design: doc/near-pole-area-measure-sampling.md + explore
// unify-orientation-sampling-cosine-measure). One branchless path replaces the near-pole
// tight-envelope (Rayleigh/Gamma) and off-pole generic-rejection latitude samplers.
//
//   theta[i]      colatitude-from-zenith nodes, UNIFORMLY spaced over
//                 [theta[0], theta[kNodes-1]] (the mass-bracketing sub-interval of [0, pi]).
//                 Consumed by lm_pcg::invert_lat_lut (fixed 8-step binary search + interp).
//   cdf[i]        strictly-increasing CDF of the folded area-measure target at theta[i].
//   flip_prob[i]  P(flip = true | colatitude in [theta[i], theta[i+1])), i in [0, kNodes-2];
//                 flip_prob[kNodes-1] is unused. flip rotates azimuth/roll by pi downstream,
//                 so it must be reproduced (for a symmetric near-pole proposal P(flip)~0.5,
//                 and a non-uniform azimuth would be wrongly un-symmetrized if flip were dropped).
//
// The target is built by DETERMINISTIC quadrature that pushes the exact per-family latitude
// proposal through the SAME fold the samplers use (lm_pcg::normalize_latitude) and weights by
// sin(theta) (= cos(latitude), the spherical area Jacobian) — so the LUT reproduces the folded
// semantics exactly, not an idealized unfolded distribution. Sampling from it is exact up to
// the node resolution (explore exp3: uniform-theta + binary search is tail-exact at N=256).
struct LatLut {
  static constexpr uint32_t kNodes = 257;  // 256 intervals: power of two -> constant 8-step search
  std::array<float, kNodes> theta{};
  std::array<float, kNodes> cdf{};
  std::array<float, kNodes> flip_prob{};
};

// Build the LUT for a latitude Distribution (mean/std in DEGREES, axis-dist convention).
// Host-only, build-time, deterministic (no RNG); amortized once per orientation distribution
// (per-ci/per-layer, cached — never per ray). Intended for the LUT-routed families
// (kGaussian / kUniform / kZigzag / kLaplacian); kNoRandom / kGaussianLegacy / full-sphere are
// handled by their own paths and are not passed here.
LatLut BuildLatLut(const Distribution& lat_dist);

}  // namespace lumice

#endif  // LM_LAT_LUT_H_
