#include "core/lat_lut.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/shared/pcg_shared.h"  // lm_pcg::normalize_latitude — single-source fold (device-matched)

namespace lumice {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;
constexpr double kDeg2Rad = kPi / 180.0;

// Fine grids: histogram bins over colatitude [0, pi] and quadrature samples of the proposal.
// Both are deterministic (no RNG). kQuad dominates build cost (~one-time per distribution).
constexpr int kFine = 4096;
constexpr int kQuad = 1 << 16;

// Deterministic per-family latitude proposal transform L(U), U in (0,1), returns RADIANS.
// Mirrors lm_pcg::pcg_get_dist / RandomNumberGenerator::Get (identical closed forms) so the LUT
// target uses the SAME proposal the samplers draw. Gaussian is handled separately (analytic
// normal-pdf quadrature) because its draw is Box-Muller, not a single-uniform inverse CDF.
double ProposalLatFromU(DistributionType type, double mean_rad, double scale_rad, double u) {
  switch (type) {
    case DistributionType::kUniform:
      return (u - 0.5) * scale_rad + mean_rad;
    case DistributionType::kZigzag:
      return std::abs(scale_rad * std::sin(u * 2.0 * kPi) + mean_rad);
    case DistributionType::kLaplacian: {
      const double sgn = (u < 0.5) ? -1.0 : 1.0;
      const double arg = std::max(1.0 - 2.0 * std::abs(u - 0.5), 1e-30);
      return mean_rad - scale_rad * sgn * std::log(arg);
    }
    default:
      return mean_rad;  // unreachable for the U-quadrature families
  }
}

// Linear interpolation of a cumulative array (size kFine+1, indexed by fine colatitude bin)
// at an arbitrary colatitude theta in [0, pi].
double LerpCum(const std::vector<double>& cum, double theta) {
  const double x = theta / (kPi / kFine);
  int i = static_cast<int>(x);
  if (i < 0) {
    return cum.front();
  }
  if (i >= kFine) {
    return cum.back();
  }
  const double f = x - i;
  return cum[i] * (1.0 - f) + cum[i + 1] * f;
}

// A degenerate but valid LUT (all mass at a single colatitude) for the guard path.
LatLut DegenerateLut(double colat) {
  LatLut lut;
  const float c = static_cast<float>(std::min(std::max(colat, 0.0), kPi));
  for (uint32_t i = 0; i < LatLut::kNodes; ++i) {
    lut.theta[i] = c;
    lut.cdf[i] = static_cast<float>(i) / static_cast<float>(LatLut::kNodes - 1);
    lut.flip_prob[i] = 0.0f;
  }
  return lut;
}

}  // namespace

LatLut BuildLatLut(const Distribution& lat_dist) {
  const double mean_rad = static_cast<double>(lat_dist.mean) * kDeg2Rad;
  const double scale_rad = static_cast<double>(lat_dist.std) * kDeg2Rad;
  const DistributionType type = lat_dist.type;
  const double dtheta = kPi / kFine;

  // --- Deterministic quadrature: area-measure mass + flipped mass over colatitude bins. ---
  std::vector<double> mass(kFine, 0.0);
  std::vector<double> flip_mass(kFine, 0.0);

  auto accumulate = [&](double lat_rad, double weight) {
    float phi_out = 0.0f;
    bool flip = false;
    lm_pcg::normalize_latitude(static_cast<float>(lat_rad), phi_out, flip);
    const double theta_z = kHalfPi - static_cast<double>(phi_out);  // colatitude-from-zenith [0, pi]
    const double w = weight * std::sin(theta_z);                    // area Jacobian (= cos(latitude))
    if (w <= 0.0) {
      return;
    }
    int bin = static_cast<int>(theta_z / dtheta);
    bin = std::min(std::max(bin, 0), kFine - 1);
    mass[bin] += w;
    if (flip) {
      flip_mass[bin] += w;
    }
  };

  if (type == DistributionType::kGaussian) {
    // Analytic normal-pdf quadrature over latitude L in [mean - 12 sigma, mean + 12 sigma].
    const double lo = mean_rad - 12.0 * scale_rad;
    const double hi = mean_rad + 12.0 * scale_rad;
    const double dL = (hi - lo) / kQuad;
    const double inv2s2 = (scale_rad > 0.0) ? 1.0 / (2.0 * scale_rad * scale_rad) : 0.0;
    for (int i = 0; i < kQuad; ++i) {
      const double L = lo + (i + 0.5) * dL;
      const double d = L - mean_rad;
      accumulate(L, std::exp(-d * d * inv2s2) * dL);  // unnormalized normal (normalized below)
    }
  } else {
    // U-quadrature: X = g(U), U ~ Uniform(0,1); each dU carries uniform weight. Exact for the
    // transform-based families (uniform / laplacian / zigzag) and reuses their exact closed form.
    const double dU = 1.0 / kQuad;
    for (int i = 0; i < kQuad; ++i) {
      const double u = (i + 0.5) * dU;
      accumulate(ProposalLatFromU(type, mean_rad, scale_rad, u), dU);
    }
  }

  // --- Cumulative arrays + total. ---
  std::vector<double> cum_mass(kFine + 1, 0.0);
  std::vector<double> cum_flip(kFine + 1, 0.0);
  for (int i = 0; i < kFine; ++i) {
    cum_mass[i + 1] = cum_mass[i] + mass[i];
    cum_flip[i + 1] = cum_flip[i] + flip_mass[i];
  }
  const double total = cum_mass[kFine];
  if (!(total > 0.0)) {
    // No area-measure mass (e.g. scale == 0). Fall back to a delta at the folded mean colatitude.
    float phi_out = 0.0f;
    bool flip = false;
    lm_pcg::normalize_latitude(static_cast<float>(mean_rad), phi_out, flip);
    return DegenerateLut(kHalfPi - static_cast<double>(phi_out));
  }

  // --- Bracket the mass: [theta_lo, theta_hi] where the CDF crosses [1e-7, 1 - 1e-7]. ---
  double theta_lo = 0.0;
  double theta_hi = kPi;
  for (int i = 0; i <= kFine; ++i) {
    if (cum_mass[i] / total >= 1e-7) {
      theta_lo = i * dtheta;
      break;
    }
  }
  for (int i = kFine; i >= 0; --i) {
    if (cum_mass[i] / total <= 1.0 - 1e-7) {
      theta_hi = i * dtheta;
      break;
    }
  }
  if (!(theta_hi > theta_lo)) {
    return DegenerateLut(0.5 * (theta_lo + theta_hi));
  }

  // --- Resample kNodes uniform-theta nodes; CDF via interp; flip_prob per node interval. ---
  LatLut lut;
  const double span = theta_hi - theta_lo;
  for (uint32_t n = 0; n < LatLut::kNodes; ++n) {
    const double t = theta_lo + span * n / (LatLut::kNodes - 1);
    lut.theta[n] = static_cast<float>(t);
    lut.cdf[n] = static_cast<float>(LerpCum(cum_mass, t) / total);
  }
  // Enforce strict monotonicity so the binary search predicate is total and interpolation never
  // divides by zero (fp32 CDF increments can underflow to bit-equal in low-density regions).
  for (uint32_t n = 1; n < LatLut::kNodes; ++n) {
    if (lut.cdf[n] <= lut.cdf[n - 1]) {
      lut.cdf[n] = std::nextafter(lut.cdf[n - 1], std::numeric_limits<float>::infinity());
    }
  }
  for (uint32_t n = 0; n + 1 < LatLut::kNodes; ++n) {
    const double t0 = lut.theta[n];
    const double t1 = lut.theta[n + 1];
    const double m = LerpCum(cum_mass, t1) - LerpCum(cum_mass, t0);
    const double fm = LerpCum(cum_flip, t1) - LerpCum(cum_flip, t0);
    lut.flip_prob[n] = (m > 0.0) ? static_cast<float>(std::min(std::max(fm / m, 0.0), 1.0)) : 0.0f;
  }
  lut.flip_prob[LatLut::kNodes - 1] = lut.flip_prob[LatLut::kNodes - 2];
  return lut;
}

}  // namespace lumice
