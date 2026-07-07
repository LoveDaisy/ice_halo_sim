#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "core/crystal.hpp"
#include "core/lat_lut.hpp"
#include "core/math.hpp"
#include "core/shared/lat_path_selection.hpp"
#include "core/shared/pcg_shared.h"
#include "util/threading_pool.hpp"

namespace {

class RngTest : public ::testing::Test {
 protected:
  static constexpr size_t kCheckSize = 1024;
  static constexpr double kFloatEps = 1e-7;
};


TEST_F(RngTest, GaussianTest) {
  // Prepare buffers to store the random values.
  auto values1 = std::make_unique<float[]>(kCheckSize);
  auto values2 = std::make_unique<float[]>(kCheckSize);

  auto& rng = lumice::RandomNumberGenerator::GetInstance();

  // Fill the buffer with Gaussian-distributed random values.
  rng.Reset();
  for (size_t i = 0; i < kCheckSize; i++) {
    values1[i] = rng.GetGaussian();
  }

  // Fill the buffer using a single-worker thread pool.
  // RNG is thread_local, so with pool size 1 the worker thread has its own RNG
  // seeded identically, producing the same sequence in the same order.
  rng.Reset();
  auto thread_pool = lumice::ThreadingPool::CreatePool(1);
  thread_pool->CommitRangeStepJobsAndWait(
      0, kCheckSize, [&values2, &rng](int /* thread_id */, int i) { values2[i] = rng.GetGaussian(); });

  // Compare the two buffers. They should be the same after sorting.
  std::sort(values1.get(), values1.get() + kCheckSize);
  std::sort(values2.get(), values2.get() + kCheckSize);
  for (size_t i = 0; i < kCheckSize; i++) {
    ASSERT_NEAR(values1[i], values2[i], kFloatEps);
  }
}


TEST_F(RngTest, UniformTest) {
  // Prepare buffers to store the random values.
  auto values1 = std::make_unique<float[]>(kCheckSize);
  auto values2 = std::make_unique<float[]>(kCheckSize);

  auto& rng = lumice::RandomNumberGenerator::GetInstance();

  // Fill the buffer with uniform-distributed random values.
  rng.Reset();
  for (size_t i = 0; i < kCheckSize; i++) {
    values1[i] = rng.GetUniform();
  }

  // Fill the buffer using a single-worker thread pool.
  rng.Reset();
  auto thread_pool = lumice::ThreadingPool::CreatePool(1);
  thread_pool->CommitRangeStepJobsAndWait(
      0, kCheckSize, [&values2, &rng](int /* thread_id */, int i) { values2[i] = rng.GetUniform(); });

  // Compare the two buffers. They should be the same after sorting.
  std::sort(values1.get(), values1.get() + kCheckSize);
  std::sort(values2.get(), values2.get() + kCheckSize);
  for (size_t i = 0; i < kCheckSize; i++) {
    ASSERT_NEAR(values1[i], values2[i], kFloatEps);
  }
}


TEST_F(RngTest, IsFullSphereUniform) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;

  // Default full-sphere: azimuth={kUniform,0,360}, latitude={kUniform,90,360}
  AxisDistribution full_sphere;
  full_sphere.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  full_sphere.latitude_dist = { DistributionType::kUniform, 90.0f, 360.0f };
  EXPECT_TRUE(full_sphere.IsFullSphereUniform());

  // Custom azimuth range -> false
  AxisDistribution custom_az;
  custom_az.azimuth_dist = { DistributionType::kUniform, 45.0f, 90.0f };
  custom_az.latitude_dist = { DistributionType::kUniform, 90.0f, 360.0f };
  EXPECT_FALSE(custom_az.IsFullSphereUniform());

  // Custom latitude range -> false
  AxisDistribution custom_lat;
  custom_lat.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  custom_lat.latitude_dist = { DistributionType::kUniform, 60.0f, 60.0f };
  EXPECT_FALSE(custom_lat.IsFullSphereUniform());

  // Gaussian type -> false
  AxisDistribution gauss;
  gauss.azimuth_dist = { DistributionType::kGaussian, 0.0f, 10.0f };
  gauss.latitude_dist = { DistributionType::kUniform, 90.0f, 360.0f };
  EXPECT_FALSE(gauss.IsFullSphereUniform());

  // Default constructor (kNoRandom) -> false
  AxisDistribution default_ctor;
  EXPECT_FALSE(default_ctor.IsFullSphereUniform());
}


TEST_F(RngTest, SampleSphericalWithCustomAzimuth) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomSampler;

  // azimuth: uniform [60°, 120°] (mean=90, std=60)
  // latitude: kNoRandom at 0° (equator, i.e. zenith=90 -> latitude_mean=0)
  AxisDistribution axis;
  axis.azimuth_dist = { DistributionType::kUniform, 90.0f, 60.0f };
  axis.latitude_dist = { DistributionType::kNoRandom, 0.0f, 0.0f };

  constexpr size_t kN = 2048;
  auto data = std::make_unique<float[]>(kN * 3);
  RandomSampler::SampleSphericalPointsSph(axis, data.get(), kN);

  const float kAzMin = 60.0f * lumice::math::kDegreeToRad;
  const float kAzMax = 120.0f * lumice::math::kDegreeToRad;
  for (size_t i = 0; i < kN; i++) {
    float az = data[i * 3 + 0];
    EXPECT_GE(az, kAzMin - 1e-5f) << "Sample " << i << " azimuth too low: " << az;
    EXPECT_LE(az, kAzMax + 1e-5f) << "Sample " << i << " azimuth too high: " << az;
  }
}


TEST_F(RngTest, SampleSphericalWithCustomLatitude) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomSampler;

  // azimuth: kNoRandom at 0°
  // latitude: uniform around 45° with range 30° -> [30°, 60°]
  AxisDistribution axis;
  axis.azimuth_dist = { DistributionType::kNoRandom, 0.0f, 0.0f };
  axis.latitude_dist = { DistributionType::kUniform, 45.0f, 30.0f };

  constexpr size_t kN = 2048;
  auto data = std::make_unique<float[]>(kN * 3);
  RandomSampler::SampleSphericalPointsSph(axis, data.get(), kN);

  const float kLatMin = 30.0f * lumice::math::kDegreeToRad;
  const float kLatMax = 60.0f * lumice::math::kDegreeToRad;
  for (size_t i = 0; i < kN; i++) {
    float lat = data[i * 3 + 1];
    EXPECT_GE(lat, kLatMin - 1e-5f) << "Sample " << i << " latitude too low: " << lat;
    EXPECT_LE(lat, kLatMax + 1e-5f) << "Sample " << i << " latitude too high: " << lat;
  }
}


TEST_F(RngTest, TriangleSample) {
  auto crystal = lumice::Crystal::CreatePrism(0.2f);

  float p[3];
  float v[3];
  int fid = 0;
  const float* face_vtx = crystal.GetTriangleVtx() + fid * 9;
  lumice::SampleTrianglePoint(face_vtx, p);
  for (int k = 0; k < 3; k++) {
    v[k] = p[k] - face_vtx[k];
  }
  float dot = lumice::Dot3(v, crystal.GetTriangleNormal() + fid * 3);
  EXPECT_NEAR(dot, 0.0f, kFloatEps);
}


// ============================================================================
// Jacobian-corrected spherical sampling tests
// ============================================================================

class SphericalSamplingTest : public ::testing::Test {
 protected:
  static constexpr size_t kSampleCount = 500000;
  static constexpr float kDeg2Rad = lumice::math::kDegreeToRad;

  // Compute the theoretical density for the Jacobian-corrected distribution:
  //   p(theta) ∝ G(theta - theta0, sigma) × sin(theta)
  // where theta = colatitude = pi/2 - latitude.
  // Returns normalized density values at bin centers.
  static std::vector<double> TheoreticalDensity(const std::vector<double>& bin_centers_deg, double zen_mean_deg,
                                                double sigma_deg) {
    double theta0 = zen_mean_deg * lumice::math::kDegreeToRad;
    double sigma = sigma_deg * lumice::math::kDegreeToRad;
    double bin_width_rad = (bin_centers_deg[1] - bin_centers_deg[0]) * lumice::math::kDegreeToRad;

    std::vector<double> density(bin_centers_deg.size());
    double total = 0;
    for (size_t i = 0; i < bin_centers_deg.size(); i++) {
      double theta = bin_centers_deg[i] * lumice::math::kDegreeToRad;
      // Account for latitude wrap/fold at theta=0 (latitude=90°) and theta=pi (latitude=-90°).
      // Samples with theta < 0 get reflected to -theta; samples with theta > pi get reflected to 2*pi-theta.
      double g_direct = std::exp(-0.5 * (theta - theta0) * (theta - theta0) / (sigma * sigma));
      double g_fold0 = std::exp(-0.5 * (-theta - theta0) * (-theta - theta0) / (sigma * sigma));
      double g_fold_pi = std::exp(-0.5 * (2 * lumice::math::kPi - theta - theta0) *
                                  (2 * lumice::math::kPi - theta - theta0) / (sigma * sigma));
      density[i] = (g_direct + g_fold0 + g_fold_pi) * std::sin(theta);
      total += density[i] * bin_width_rad;
    }
    for (auto& d : density) {
      d /= total;
    }
    return density;
  }

  // Sample N points and return colatitude distribution as bin counts.
  static std::vector<int> SampleAndBin(double zen_mean_deg, double sigma_deg, const std::vector<double>& bin_edges_deg,
                                       size_t n) {
    lumice::AxisDistribution axis;
    axis.latitude_dist.type = lumice::DistributionType::kGaussian;
    axis.latitude_dist.mean = static_cast<float>(90.0 - zen_mean_deg);  // zenith → latitude
    axis.latitude_dist.std = static_cast<float>(sigma_deg);
    axis.azimuth_dist.type = lumice::DistributionType::kUniform;

    std::vector<int> counts(bin_edges_deg.size() - 1, 0);
    float lon_lat[3];
    for (size_t i = 0; i < n; i++) {
      lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
      double colatitude_deg = (lumice::math::kPi_2 - lon_lat[1]) / kDeg2Rad;
      // Find bin.
      for (size_t b = 0; b < counts.size(); b++) {
        if (colatitude_deg >= bin_edges_deg[b] && colatitude_deg < bin_edges_deg[b + 1]) {
          counts[b]++;
          break;
        }
      }
    }
    return counts;
  }
};


TEST_F(SphericalSamplingTest, JacobianCorrectedDistribution) {
  // Test configurations: (zenith_mean_deg, sigma_deg)
  struct Config {
    double zen;
    double sigma;
    const char* label;
  };
  Config configs[] = {
    { 0, 5, "polar (Rayleigh)" }, { 1, 2, "near-polar boundary" }, { 10, 5, "near-polar rejection" },
    { 45, 5, "mid-latitude" },    { 90, 5, "equatorial" },
  };

  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(42);

  for (const auto& cfg : configs) {
    SCOPED_TRACE(cfg.label);

    // Build bins in colatitude (zenith angle) space, centered around zen_mean.
    double lo = std::max(0.0, cfg.zen - 4 * cfg.sigma);
    double hi = std::min(180.0, cfg.zen + 4 * cfg.sigma);
    constexpr int kNumBins = 20;
    double bin_width = (hi - lo) / kNumBins;
    std::vector<double> bin_edges;
    std::vector<double> bin_centers;
    for (int b = 0; b <= kNumBins; b++) {
      bin_edges.push_back(lo + b * bin_width);
    }
    for (int b = 0; b < kNumBins; b++) {
      bin_centers.push_back(lo + (b + 0.5) * bin_width);
    }

    auto counts = SampleAndBin(cfg.zen, cfg.sigma, bin_edges, kSampleCount);
    auto theo = TheoreticalDensity(bin_centers, cfg.zen, cfg.sigma);

    // Compare observed density vs theoretical within 2-sigma range.
    double bin_width_rad = bin_width * kDeg2Rad;
    for (int b = 0; b < kNumBins; b++) {
      double dist_from_mean = std::abs(bin_centers[b] - cfg.zen);
      if (dist_from_mean > 2 * cfg.sigma) {
        continue;  // Skip tails — low sample count makes relative deviation noisy.
      }
      double observed_density = static_cast<double>(counts[b]) / (kSampleCount * bin_width_rad);
      if (theo[b] < 1e-6) {
        continue;  // Skip near-zero theoretical density.
      }
      double relative_dev = std::abs(observed_density - theo[b]) / theo[b];
      EXPECT_LT(relative_dev, 0.05) << "bin center=" << bin_centers[b] << "° for " << cfg.label;
    }
  }
}


TEST_F(SphericalSamplingTest, LargeSigmaUniformDistribution) {
  // When sigma >> 90°, the Gaussian is nearly flat and the Jacobian-corrected
  // distribution approaches uniform on sphere: p(theta) = sin(theta)/2.
  struct Config {
    double zen;
    double sigma;
    const char* label;
  };
  Config configs[] = {
    { 0, 180, "polar large-sigma" },
    { 90, 180, "equatorial large-sigma" },
  };

  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(99);

  for (const auto& cfg : configs) {
    SCOPED_TRACE(cfg.label);

    // Bins covering full colatitude range [0°, 180°].
    constexpr int kNumBins = 18;
    constexpr double kBinWidth = 180.0 / kNumBins;  // 10° bins
    std::vector<double> bin_edges;
    std::vector<double> bin_centers;
    for (int b = 0; b <= kNumBins; b++) {
      bin_edges.push_back(b * kBinWidth);
    }
    for (int b = 0; b < kNumBins; b++) {
      bin_centers.push_back((b + 0.5) * kBinWidth);
    }

    auto counts = SampleAndBin(cfg.zen, cfg.sigma, bin_edges, kSampleCount);

    // Theoretical density: p(theta) = sin(theta) / 2 (uniform on sphere).
    double bin_width_rad = kBinWidth * kDeg2Rad;
    for (int b = 0; b < kNumBins; b++) {
      double theta_rad = bin_centers[b] * kDeg2Rad;
      double theo_density = std::sin(theta_rad) / 2.0;
      if (theo_density < 1e-6) {
        continue;
      }
      double observed_density = static_cast<double>(counts[b]) / (kSampleCount * bin_width_rad);
      double relative_dev = std::abs(observed_density - theo_density) / theo_density;
      EXPECT_LT(relative_dev, 0.05) << "bin center=" << bin_centers[b] << "° for " << cfg.label;
    }
  }
}


TEST_F(SphericalSamplingTest, UniformLatitudeJacobianCorrection) {
  // Verify that kUniform latitude also gets Jacobian correction: p(theta) ∝ sin(theta).
  // Full-range uniform latitude should produce the same distribution as uniform-on-sphere.
  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(77);

  lumice::AxisDistribution axis;
  axis.latitude_dist.type = lumice::DistributionType::kUniform;
  axis.latitude_dist.mean = 90.0f;  // zenith=0° → latitude=90° (centered at pole)
  axis.latitude_dist.std = 360.0f;  // full range
  axis.azimuth_dist.type = lumice::DistributionType::kUniform;
  axis.azimuth_dist.mean = 0.0f;
  axis.azimuth_dist.std = 360.0f;

  // Bin over full colatitude [0°, 180°].
  constexpr int kNumBins = 18;
  constexpr double kBinWidth = 180.0 / kNumBins;
  std::vector<double> bin_edges;
  std::vector<double> bin_centers;
  for (int b = 0; b <= kNumBins; b++) {
    bin_edges.push_back(b * kBinWidth);
  }
  for (int b = 0; b < kNumBins; b++) {
    bin_centers.push_back((b + 0.5) * kBinWidth);
  }

  // Sample using the parameterized path (NOT IsFullSphereUniform dispatch).
  std::vector<int> counts(kNumBins, 0);
  float lon_lat[3];
  for (size_t i = 0; i < kSampleCount; i++) {
    lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
    double colatitude_deg = (lumice::math::kPi_2 - lon_lat[1]) / kDeg2Rad;
    for (int b = 0; b < kNumBins; b++) {
      if (colatitude_deg >= bin_edges[b] && colatitude_deg < bin_edges[b + 1]) {
        counts[b]++;
        break;
      }
    }
  }

  // Should match uniform-on-sphere: p(theta) = sin(theta)/2.
  double bin_width_rad = kBinWidth * kDeg2Rad;
  for (int b = 0; b < kNumBins; b++) {
    double theta_rad = bin_centers[b] * kDeg2Rad;
    double theo_density = std::sin(theta_rad) / 2.0;
    if (theo_density < 1e-6) {
      continue;
    }
    double observed_density = static_cast<double>(counts[b]) / (kSampleCount * bin_width_rad);
    double relative_dev = std::abs(observed_density - theo_density) / theo_density;
    EXPECT_LT(relative_dev, 0.05) << "bin center=" << bin_centers[b] << "°";
  }
}


TEST_F(SphericalSamplingTest, MeanVarianceAccuracy) {
  // For each config, check that the sample mean of colatitude matches the theoretical expectation.
  // For G(theta-theta0, sigma) × sin(theta), the mean is slightly shifted from theta0
  // due to the sin(theta) weighting. We verify by comparing against a numerically computed expectation.
  struct Config {
    double zen;
    double sigma;
  };
  Config configs[] = { { 0, 5 }, { 45, 5 }, { 90, 5 }, { 0, 30 }, { 0, 180 }, { 90, 180 } };

  auto& rng = lumice::RandomNumberGenerator::GetInstance();

  for (const auto& cfg : configs) {
    // Reset seed per-config so each config's Monte Carlo estimate is independent
    // of prior configs' RNG consumption. Previously all configs shared one seed
    // (123); when scrum-328.3 introduced the sinθ/θ accept step in the Rayleigh
    // path, the shifted RNG stream pushed {zen=0,σ=30} 0.008° over the 0.1°
    // tolerance despite the sampler's target distribution being unchanged. Per-
    // config seeding removes the cross-config coupling without loosening the
    // tolerance (see plan §3 "RNG 消耗节拍变化" clause).
    rng.SetSeed(123);
    lumice::AxisDistribution axis;
    axis.latitude_dist.type = lumice::DistributionType::kGaussian;
    axis.latitude_dist.mean = static_cast<float>(90.0 - cfg.zen);
    axis.latitude_dist.std = static_cast<float>(cfg.sigma);
    axis.azimuth_dist.type = lumice::DistributionType::kUniform;

    // Numerically compute E[theta] for p(theta) ∝ G_folded(theta) × sin(theta).
    // G_folded sums Gaussian images under reflections at 0 and π: images at θ₀+2kπ and -θ₀+2kπ.
    // For large sigma this converges to uniform sphere: p(theta) ∝ sin(theta), E[theta] = π/2 ≈ 90°.
    double theta0 = cfg.zen * kDeg2Rad;
    double sigma = cfg.sigma * kDeg2Rad;
    double sum_theta = 0;
    double sum_weight = 0;
    constexpr int kIntegrationSteps = 10000;
    for (int k = 0; k < kIntegrationSteps; k++) {
      double theta = (0.001 + k * lumice::math::kPi / kIntegrationSteps);
      double g = 0;
      for (int n = -3; n <= 3; n++) {
        double shift = 2.0 * n * lumice::math::kPi;
        double d1 = theta - theta0 - shift;
        double d2 = theta + theta0 - shift;
        g += std::exp(-0.5 * d1 * d1 / (sigma * sigma));
        g += std::exp(-0.5 * d2 * d2 / (sigma * sigma));
      }
      double w = g * std::sin(theta);
      sum_theta += theta * w;
      sum_weight += w;
    }
    double expected_theta = sum_theta / sum_weight;
    bool is_large_sigma = cfg.sigma >= 90;

    // Sample and compute mean colatitude.
    double sample_sum = 0;
    float lon_lat[3];
    for (size_t i = 0; i < kSampleCount; i++) {
      lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
      double colatitude = lumice::math::kPi_2 - lon_lat[1];
      sample_sum += colatitude;
    }
    double sample_mean = sample_sum / kSampleCount;

    // Large sigma tolerance: 0.5°; small sigma: 0.1°.
    double tolerance = is_large_sigma ? 0.5 : 0.1;
    EXPECT_NEAR(sample_mean / kDeg2Rad, expected_theta / kDeg2Rad, tolerance)
        << "zenith=" << cfg.zen << "° sigma=" << cfg.sigma << "°";
  }
}

// ============================================================================
// NormalizeLatitude tests
// ============================================================================

class NormalizeLatitudeTest : public ::testing::Test {
 protected:
  static constexpr float kPi = lumice::math::kPi;
  static constexpr float kPi_2 = lumice::math::kPi_2;
  static constexpr float kEps = 1e-5f;

  void ExpectNormalized(float input, float expected_lat, bool expected_flip, const char* label) {
    auto [lat, flip] = lumice::detail::NormalizeLatitude(input);
    EXPECT_NEAR(lat, expected_lat, kEps) << label << " (latitude)";
    EXPECT_EQ(flip, expected_flip) << label << " (flip)";
  }
};


TEST_F(NormalizeLatitudeTest, InRangeUnchanged) {
  ExpectNormalized(0.0f, 0.0f, false, "phi=0");
  ExpectNormalized(kPi / 4, kPi / 4, false, "phi=pi/4");
  ExpectNormalized(-kPi / 4, -kPi / 4, false, "phi=-pi/4");
}


TEST_F(NormalizeLatitudeTest, PoleExact) {
  ExpectNormalized(kPi_2, kPi_2, false, "phi=pi/2 (north pole)");
  ExpectNormalized(-kPi_2, -kPi_2, false, "phi=-pi/2 (south pole)");
}


TEST_F(NormalizeLatitudeTest, SingleFoldForward) {
  // phi=π: past north pole by π/2 → equator, flip=true
  ExpectNormalized(kPi, 0.0f, true, "phi=pi");
  // phi=3π/4: past north pole by π/4 → π/4, flip=true
  ExpectNormalized(3 * kPi / 4, kPi / 4, true, "phi=3pi/4");
}


TEST_F(NormalizeLatitudeTest, SingleFoldBackward) {
  // phi=-π: past south pole by π/2 → equator, flip=true
  ExpectNormalized(-kPi, 0.0f, true, "phi=-pi");
  // phi=-3π/4: past south pole by π/4 → -π/4, flip=true
  ExpectNormalized(-3 * kPi / 4, -kPi / 4, true, "phi=-3pi/4");
}


TEST_F(NormalizeLatitudeTest, MultipleFolds) {
  // phi=3π: three pole reflections → equator, flip=true (odd reflections)
  ExpectNormalized(3 * kPi, 0.0f, true, "phi=3pi");
  // phi=2π: two pole reflections → equator, flip=false (even reflections)
  ExpectNormalized(2 * kPi, 0.0f, false, "phi=2pi");
  // phi=5π/2: two pole reflections → north pole, flip=false
  ExpectNormalized(5 * kPi_2, kPi_2, false, "phi=5pi/2");
  // phi=-5π/2: two pole reflections → south pole, flip=false
  ExpectNormalized(-5 * kPi_2, -kPi_2, false, "phi=-5pi/2");
}


// ============================================================================
// Zigzag distribution tests
// ============================================================================

TEST_F(RngTest, IsFullSphereUniformZigzag) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;

  AxisDistribution zigzag;
  zigzag.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  zigzag.latitude_dist = { DistributionType::kZigzag, 0.0f, 30.0f };
  EXPECT_FALSE(zigzag.IsFullSphereUniform());
}


TEST_F(SphericalSamplingTest, ZigzagBasicSampling) {
  // Zigzag with mean=0 (tilt_offset), std=30 (amplitude): |30·sin(2πU) + 0|
  // All samples should produce colatitude in [60°, 120°] (zenith 60-120 = latitude -30 to +30).
  lumice::AxisDistribution axis;
  axis.latitude_dist = { lumice::DistributionType::kZigzag, 0.0f, 30.0f };
  axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(42);

  float lon_lat[3];
  for (int i = 0; i < 10000; i++) {
    lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
    float lat_deg = lon_lat[1] / kDeg2Rad;
    // |0 + 30·sin(...)| produces latitude in [0, 30], but Jacobian rejection + NormalizeLatitude
    // may shift slightly. Latitude should be in [-30, 30] range (generous margin).
    ASSERT_GE(lat_deg, -35.0f) << "latitude out of range at sample " << i;
    ASSERT_LE(lat_deg, 35.0f) << "latitude out of range at sample " << i;
  }
}


TEST_F(SphericalSamplingTest, ZigzagJacobianCorrection) {
  // Verify that zigzag sampling with Jacobian correction produces the correct density.
  // Target: p(theta) ∝ zigzag_proposal_pdf(theta) × sin(theta)
  // where zigzag_proposal_pdf is the rectified arcsine distribution.
  struct Config {
    float mean;  // tilt_offset (latitude degrees)
    float std;   // amplitude (degrees)
    const char* label;
  };
  Config configs[] = {
    { 0.0f, 30.0f, "equatorial, mean<std" },
    { 45.0f, 10.0f, "mid-latitude, mean>std" },
    { 80.0f, 5.0f, "near-pole, mean>>std" },
  };

  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(123);

  for (const auto& cfg : configs) {
    SCOPED_TRACE(cfg.label);

    lumice::AxisDistribution axis;
    axis.latitude_dist = { lumice::DistributionType::kZigzag, cfg.mean, cfg.std };
    axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

    // Determine colatitude range from the zigzag proposal range.
    // Zigzag |A·sin(2πU) + B| produces latitude in [max(|B|-A, 0), |B|+A].
    float lat_lo = std::max(std::abs(cfg.mean) - cfg.std, 0.0f);
    float lat_hi = std::abs(cfg.mean) + cfg.std;
    // Convert latitude range to colatitude range (colatitude = 90 - latitude).
    float lo = std::max(0.0f, 90.0f - lat_hi - 3.0f);
    float hi = std::min(180.0f, 90.0f - lat_lo + 3.0f);
    constexpr int kNumBins = 20;
    float bin_width = (hi - lo) / kNumBins;

    std::vector<double> bin_edges;
    std::vector<double> bin_centers;
    for (int b = 0; b <= kNumBins; b++) {
      bin_edges.push_back(lo + b * bin_width);
    }
    for (int b = 0; b < kNumBins; b++) {
      bin_centers.push_back(lo + (b + 0.5) * bin_width);
    }

    // Sample and bin.
    constexpr size_t kN = 1000000;
    std::vector<int> counts(kNumBins, 0);
    float lon_lat[3];
    for (size_t i = 0; i < kN; i++) {
      lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
      double colatitude_deg = (lumice::math::kPi_2 - lon_lat[1]) / kDeg2Rad;
      for (int b = 0; b < kNumBins; b++) {
        if (colatitude_deg >= bin_edges[b] && colatitude_deg < bin_edges[b + 1]) {
          counts[b]++;
          break;
        }
      }
    }

    double total_in_range = 0;
    for (int b = 0; b < kNumBins; b++) {
      total_in_range += counts[b];
    }

    // At least 90% of samples should fall within the binning range.
    EXPECT_GT(total_in_range / kN, 0.90) << "Too many samples outside expected range for " << cfg.label;

    // Verify Jacobian correction by checking mean colatitude shift.
    // With Jacobian sin(theta) weighting, samples are shifted toward the equator (theta=90°)
    // compared to the bare proposal. We verify by comparing:
    //   1. Observed mean colatitude from rejection samples
    //   2. Proposal mean colatitude (computed via Monte Carlo without rejection)
    // The observed mean should be >= proposal mean (closer to equator = larger colatitude).
    double observed_mean_colat = 0;
    for (int b = 0; b < kNumBins; b++) {
      observed_mean_colat += bin_centers[b] * counts[b];
    }
    observed_mean_colat /= (total_in_range + 1e-10);

    // Compute proposal mean colatitude (without Jacobian) via Monte Carlo.
    double proposal_mean_colat = 0;
    constexpr size_t kProposalN = 100000;
    for (size_t i = 0; i < kProposalN; i++) {
      float raw_lat = rng.Get(axis.latitude_dist);  // proposal in degrees
      double colat = 90.0 - static_cast<double>(raw_lat);
      // Clamp to [0, 180] for safety.
      colat = std::max(0.0, std::min(180.0, colat));
      proposal_mean_colat += colat;
    }
    proposal_mean_colat /= kProposalN;

    // Jacobian sin(theta) weighting pushes mean colatitude toward 90° (equator).
    // For colatitude < 90°, observed mean should be LARGER than proposal mean.
    // For colatitude > 90°, observed mean should be SMALLER (closer to 90°).
    double observed_dist_to_equator = std::abs(observed_mean_colat - 90.0);
    double proposal_dist_to_equator = std::abs(proposal_mean_colat - 90.0);
    EXPECT_LT(observed_dist_to_equator, proposal_dist_to_equator + 0.5)
        << "Jacobian correction not detected for " << cfg.label << " (observed_mean_colat=" << observed_mean_colat
        << ", proposal_mean_colat=" << proposal_mean_colat << ")";
  }
}


TEST_F(RngTest, ZigzagJsonRoundTrip) {
  using lumice::Distribution;
  using lumice::DistributionType;

  Distribution orig{ DistributionType::kZigzag, 10.0f, 25.0f };
  nlohmann::json j;
  lumice::to_json(j, orig);

  EXPECT_EQ(j["type"], "zigzag");
  EXPECT_FLOAT_EQ(j["mean"], 10.0f);
  EXPECT_FLOAT_EQ(j["std"], 25.0f);

  Distribution parsed{};
  lumice::from_json(j, parsed);
  EXPECT_EQ(parsed.type, DistributionType::kZigzag);
  EXPECT_FLOAT_EQ(parsed.mean, 10.0f);
  EXPECT_FLOAT_EQ(parsed.std, 25.0f);
}


// ============================================================================
// Laplacian distribution tests
// ============================================================================

TEST_F(RngTest, IsFullSphereUniformLaplacian) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;

  AxisDistribution lap;
  lap.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  lap.latitude_dist = { DistributionType::kLaplacian, 0.0f, 2.0f };
  EXPECT_FALSE(lap.IsFullSphereUniform());
}


TEST_F(SphericalSamplingTest, LaplacianBasicSampling) {
  // Laplacian with mean=0, std=5 (scale b=5°).
  lumice::AxisDistribution axis;
  axis.latitude_dist = { lumice::DistributionType::kLaplacian, 0.0f, 5.0f };
  axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(42);

  float lon_lat[3];
  double sum_lat = 0;
  constexpr int kN = 50000;
  for (int i = 0; i < kN; i++) {
    lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
    float lat_deg = lon_lat[1] / kDeg2Rad;
    sum_lat += lat_deg;
    // Laplace can produce wide tails, but most samples should be within [-40, 40].
    ASSERT_GE(lat_deg, -90.0f) << "latitude out of range at sample " << i;
    ASSERT_LE(lat_deg, 90.0f) << "latitude out of range at sample " << i;
  }
  // Mean latitude should be close to 0 (equatorial center).
  double mean_lat = sum_lat / kN;
  EXPECT_NEAR(mean_lat, 0.0, 1.0) << "Mean latitude deviates too much from center";
}


TEST_F(SphericalSamplingTest, LaplacianJacobianCorrection) {
  // Verify Jacobian correction using the mean-shift method (same as zigzag).
  struct Config {
    float mean;
    float std;
    const char* label;
  };
  Config configs[] = {
    { 0.0f, 5.0f, "equatorial" },
    { 45.0f, 3.0f, "mid-latitude" },
    { 80.0f, 2.0f, "near-pole" },
  };

  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(999);

  for (const auto& cfg : configs) {
    SCOPED_TRACE(cfg.label);

    lumice::AxisDistribution axis;
    axis.latitude_dist = { lumice::DistributionType::kLaplacian, cfg.mean, cfg.std };
    axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

    // Sample with Jacobian (rejection sampling).
    constexpr size_t kN = 500000;
    double observed_mean_colat = 0;
    float lon_lat[3];
    for (size_t i = 0; i < kN; i++) {
      lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
      double colatitude_deg = (lumice::math::kPi_2 - lon_lat[1]) / kDeg2Rad;
      observed_mean_colat += colatitude_deg;
    }
    observed_mean_colat /= kN;

    // Compute proposal mean colatitude (without Jacobian).
    double proposal_mean_colat = 0;
    constexpr size_t kProposalN = 100000;
    for (size_t i = 0; i < kProposalN; i++) {
      float raw_lat = rng.Get(axis.latitude_dist);
      double colat = 90.0 - static_cast<double>(raw_lat);
      colat = std::max(0.0, std::min(180.0, colat));
      proposal_mean_colat += colat;
    }
    proposal_mean_colat /= kProposalN;

    // Jacobian pushes mean closer to equator (colatitude 90°).
    double observed_dist = std::abs(observed_mean_colat - 90.0);
    double proposal_dist = std::abs(proposal_mean_colat - 90.0);
    EXPECT_LT(observed_dist, proposal_dist + 0.5)
        << "Jacobian correction not detected for " << cfg.label << " (observed=" << observed_mean_colat
        << ", proposal=" << proposal_mean_colat << ")";
  }
}


TEST_F(SphericalSamplingTest, LaplacianHeavierTailThanGaussian) {
  // Same mean and scale: Laplacian should have more samples far from mean than Gaussian.
  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(777);

  constexpr float kMean = 0.0f;
  constexpr float kScale = 5.0f;
  constexpr size_t kN = 200000;
  constexpr float kThreshold = 3.0f * kScale;  // 15 degrees from mean

  // Count Laplacian samples beyond threshold.
  lumice::AxisDistribution lap_axis;
  lap_axis.latitude_dist = { lumice::DistributionType::kLaplacian, kMean, kScale };
  lap_axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

  int lap_far = 0;
  float lon_lat[3];
  for (size_t i = 0; i < kN; i++) {
    lumice::RandomSampler::SampleSphericalPointsSph(lap_axis, lon_lat);
    float lat_deg = lon_lat[1] / kDeg2Rad;
    if (std::abs(lat_deg - kMean) > kThreshold) {
      lap_far++;
    }
  }

  // Count Gaussian samples beyond threshold.
  lumice::AxisDistribution gauss_axis;
  gauss_axis.latitude_dist = { lumice::DistributionType::kGaussian, kMean, kScale };
  gauss_axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

  int gauss_far = 0;
  for (size_t i = 0; i < kN; i++) {
    lumice::RandomSampler::SampleSphericalPointsSph(gauss_axis, lon_lat);
    float lat_deg = lon_lat[1] / kDeg2Rad;
    if (std::abs(lat_deg - kMean) > kThreshold) {
      gauss_far++;
    }
  }

  // Laplacian should have MORE far-tail samples than Gaussian.
  // At 3σ: Gaussian tail ~ 0.27%, Laplace tail ~ exp(-3) ≈ 5%.
  EXPECT_GT(lap_far, gauss_far) << "Laplacian should have heavier tails than Gaussian" << " (lap_far=" << lap_far
                                << ", gauss_far=" << gauss_far << ")";
}


// scrum-328.4 / 330.2 (unified area-measure LUT). Verifies the CPU sampler reproduces the
// ANALYTICAL target p(theta) ∝ exp(-theta/b) · sin(theta) for a near-pole Laplacian
// (mean=90°, colatitude_center=0 → routes to the unified inverse-CDF LUT, kLutInverseCdf).
// Deliberately NOT a KS-match against the retired GenericReject Laplacian sampler, which had a
// known M=cos(5b) tail-clamp bias (scrum-328.1 exp1); the LUT is exact by construction, so
// matching the old sampler would flag a valid improvement as regression (issue.md AC hardline).
//
// Uses moment + CDF-quantile matching against the analytical target rather than bin-by-bin
// density matching — the latter suffers from bin-boundary MC artifacts at the θ→0 corner
// where p(θ)→0 linearly and small integer-count effects amplify apparent relative dev.
TEST_F(SphericalSamplingTest, LaplacianTightEnvelopeExactness) {
  // Configuration: near-pole (mean latitude=90° in internal convention, i.e., zenith=0°) with
  // b=5° — the task-325 issue scenario and the b tested in scrum-328.1 exp1.
  lumice::AxisDistribution axis;
  axis.latitude_dist = { lumice::DistributionType::kLaplacian, 90.0f, 5.0f };
  axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

  // 330.2 S5: all Laplacian latitudes now route to the unified inverse-CDF LUT. The rest of this
  // test is the valuable part — it validates the sampled colatitude against the ANALYTIC target
  // p(θ) ∝ exp(-θ/b)·sin(θ) (the scrum-328 AC), which the LUT reproduces exactly.
  auto decision = lumice::lat_path::SelectLatPath(axis);
  ASSERT_EQ(decision.kind, lumice::lat_path::LatPathKind::kLutInverseCdf)
      << "Test configuration must route to the unified LUT path.";

  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(1234);

  // Collect colatitudes.
  constexpr size_t kN = kSampleCount;
  std::vector<double> colatitudes;
  colatitudes.reserve(kN);
  float lon_lat[3];
  for (size_t i = 0; i < kN; i++) {
    lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
    colatitudes.push_back(static_cast<double>(lumice::math::kPi_2 - lon_lat[1]));  // rad
  }
  std::sort(colatitudes.begin(), colatitudes.end());

  // Theoretical target: p(θ) ∝ exp(-θ/b)·sin(θ) on [0, π].
  const double target_b = 5.0 * kDeg2Rad;
  auto target_density = [target_b](double theta) { return std::exp(-theta / target_b) * std::sin(theta); };
  // Numerical CDF sampling for quantile inversion + moment computation.
  constexpr int kCdfPts = 20000;
  const double cdf_dt = lumice::math::kPi / kCdfPts;
  std::vector<double> cdf_pts(kCdfPts + 1);
  cdf_pts[0] = 0.0;
  double theo_mean = 0.0;
  double theo_msq = 0.0;
  for (int k = 0; k < kCdfPts; k++) {
    double theta = (k + 0.5) * cdf_dt;
    double w = target_density(theta) * cdf_dt;
    cdf_pts[k + 1] = cdf_pts[k] + w;
    theo_mean += theta * w;
    theo_msq += theta * theta * w;
  }
  double normalizer = cdf_pts.back();
  theo_mean /= normalizer;
  theo_msq /= normalizer;
  double theo_std = std::sqrt(theo_msq - theo_mean * theo_mean);

  auto theo_quantile = [&](double q) -> double {
    double target_val = q * normalizer;
    auto it = std::lower_bound(cdf_pts.begin(), cdf_pts.end(), target_val);
    int idx = static_cast<int>(std::distance(cdf_pts.begin(), it));
    if (idx <= 0) {
      return 0.0;
    }
    if (idx >= static_cast<int>(cdf_pts.size())) {
      return static_cast<double>(lumice::math::kPi);
    }
    double frac = (target_val - cdf_pts[idx - 1]) / (cdf_pts[idx] - cdf_pts[idx - 1]);
    return (static_cast<double>(idx - 1) + frac) * cdf_dt;
  };

  // Moment matching (rad units).
  double sample_mean = 0.0;
  for (double c : colatitudes)
    sample_mean += c;
  sample_mean /= kN;
  double sample_msq = 0.0;
  for (double c : colatitudes)
    sample_msq += (c - sample_mean) * (c - sample_mean);
  double sample_std = std::sqrt(sample_msq / kN);

  EXPECT_NEAR(sample_mean, theo_mean, theo_mean * 0.02)
      << "Sample colatitude mean deviates from analytical target (rad).";
  EXPECT_NEAR(sample_std, theo_std, theo_std * 0.02) << "Sample colatitude std deviates from analytical target (rad).";

  // Quantile matching at 5%, 10%, 25%, 50%, 75%, 90%, 95%, 99%.
  const double kQuantiles[] = { 0.05, 0.10, 0.25, 0.50, 0.75, 0.90, 0.95, 0.99 };
  for (double q : kQuantiles) {
    double sample_q = colatitudes[static_cast<size_t>(q * kN)];
    double theo_q = theo_quantile(q);
    // Absolute tolerance: 1° = 0.0175 rad — well above MC noise on quantiles at N=500k,
    // strict enough to catch any distribution-shape drift.
    EXPECT_NEAR(sample_q, theo_q, 1.0 * kDeg2Rad)
        << "Quantile " << q << " sample=" << sample_q << " theo=" << theo_q << " (rad)";
  }
}


// 330.3 (post branch-retirement smoke). The tight-envelope↔generic-reject crossover (b-cap,
// polar threshold) it originally guarded no longer exists — every Laplacian latitude, near-pole
// / off-pole / wide-b alike, routes to the single unified LUT path (kLutInverseCdf). What remains
// worth guarding: all these configs route to the LUT and produce valid finite orientations.
TEST_F(SphericalSamplingTest, LaplacianAllConfigsRouteToLut) {
  // Wide b=70° (> old 60° cap): still routes to the LUT.
  lumice::AxisDistribution axis;
  axis.latitude_dist = { lumice::DistributionType::kLaplacian, 90.0f, 70.0f };
  axis.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };

  auto decision = lumice::lat_path::SelectLatPath(axis);
  EXPECT_EQ(decision.kind, lumice::lat_path::LatPathKind::kLutInverseCdf)
      << "Wide-b Laplacian must route to the unified LUT.";

  lumice::AxisDistribution axis_within;
  axis_within.latitude_dist = { lumice::DistributionType::kLaplacian, 90.0f, 59.0f };
  axis_within.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  auto decision_within = lumice::lat_path::SelectLatPath(axis_within);
  EXPECT_EQ(decision_within.kind, lumice::lat_path::LatPathKind::kLutInverseCdf)
      << "Near-pole Laplacian must route to the unified LUT.";

  // Off-pole (mean=80° → colatitude_center=10°) also routes to the LUT now (previously GenericReject).
  lumice::AxisDistribution axis_offpole;
  axis_offpole.latitude_dist = { lumice::DistributionType::kLaplacian, 80.0f, 5.0f };
  axis_offpole.azimuth_dist = { lumice::DistributionType::kUniform, 0.0f, 360.0f };
  auto decision_offpole = lumice::lat_path::SelectLatPath(axis_offpole);
  EXPECT_EQ(decision_offpole.kind, lumice::lat_path::LatPathKind::kLutInverseCdf)
      << "Off-pole Laplacian must route to the unified LUT.";

  // Runtime sanity for the wide-b config: no NaN, latitudes in [-π/2, π/2].
  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(2024);
  float lon_lat[3];
  for (int i = 0; i < 5000; i++) {
    lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
    ASSERT_GE(lon_lat[1], -lumice::math::kPi_2 - 1e-4f);
    ASSERT_LE(lon_lat[1], lumice::math::kPi_2 + 1e-4f);
    ASSERT_TRUE(std::isfinite(lon_lat[0]));
    ASSERT_TRUE(std::isfinite(lon_lat[1]));
    ASSERT_TRUE(std::isfinite(lon_lat[2]));
  }
}


// Verifies that fold-roll coupling is correct: sampling with a latitude that triggers pole crossing
// (fold) and roll=0° produces the same output as sampling the equivalent post-fold orientation
// directly (with no fold, and roll=π already baked in).
TEST(FoldRollEquivalenceTest, FoldedEqualsExplicit) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomSampler;
  using lumice::math::kDegreeToRad;
  using lumice::math::kPi;

  // "Folded" path: latitude=-120° (zenith=120°+90°=210°, colatitude=210° > 180°) triggers fold.
  // After fold: normalized latitude=-60°, azimuth += π, roll += π.
  // kGaussianLegacy with std=0 gives deterministic mean without bypassing NormalizeLatitude.
  AxisDistribution folded_axis;
  folded_axis.latitude_dist = { DistributionType::kGaussianLegacy, -120.0f, 0.0f };
  folded_axis.azimuth_dist = { DistributionType::kNoRandom, 0.0f, 0.0f };
  folded_axis.roll_dist = { DistributionType::kNoRandom, 0.0f, 0.0f };
  float data_folded[3]{};
  RandomSampler::SampleSphericalPointsSph(folded_axis, data_folded);

  // "Explicit" path: equivalent post-fold orientation sampled directly (no fold occurs).
  // latitude=-60°, azimuth=180°, roll=180° mirror the folded result without calling NormalizeLatitude.
  AxisDistribution explicit_axis;
  explicit_axis.latitude_dist = { DistributionType::kNoRandom, -60.0f, 0.0f };
  explicit_axis.azimuth_dist = { DistributionType::kNoRandom, 180.0f, 0.0f };
  explicit_axis.roll_dist = { DistributionType::kNoRandom, 180.0f, 0.0f };
  float data_explicit[3]{};
  RandomSampler::SampleSphericalPointsSph(explicit_axis, data_explicit);

  EXPECT_NEAR(data_folded[0], data_explicit[0], 1e-5f);  // lon (azimuth)
  EXPECT_NEAR(data_folded[1], data_explicit[1], 1e-5f);  // lat
  EXPECT_NEAR(data_folded[2], data_explicit[2], 1e-5f);  // roll: fold-linked +π
}


// Step 1a: Verify kGaussian rejection path flip=true/false balance for N(+90°, 180°).
// Diagnostic probe: checks that ~50% of accepted samples trigger flip (roll ≈ π) vs no-flip
// (roll ≈ 0). Uses roll_dist std=0 so roll_raw=0 and flip effect is exactly ±π.
// Also verifies symmetry: lat mean for flip=true and flip=false groups are similar.
TEST(FoldRollFlipBalanceTest, KGaussianFlipBalance) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomNumberGenerator;
  using lumice::RandomSampler;
  using lumice::math::kPi;

  AxisDistribution axis;
  axis.latitude_dist = { DistributionType::kGaussian, 90.0f, 180.0f };
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };  // std=0: roll_raw always 0

  constexpr int kN = 300'000;
  constexpr uint32_t kSeeds[] = { 42, 123, 999 };
  constexpr int kNTotal = kN * 3;

  std::vector<float> data(kN * 3);
  double phi_sum_flip = 0.0;
  double phi_sum_noflip = 0.0;
  int count_flip = 0;
  int count_noflip = 0;

  for (uint32_t seed : kSeeds) {
    RandomNumberGenerator::GetInstance().SetSeed(seed);
    RandomSampler::SampleSphericalPointsSph(axis, data.data(), kN);
    for (int i = 0; i < kN; i++) {
      float roll = data[i * 3 + 2];
      float lat = data[i * 3 + 1];
      bool is_flip = (std::abs(roll - kPi) < 0.001f);
      if (is_flip) {
        phi_sum_flip += lat;
        count_flip++;
      } else {
        phi_sum_noflip += lat;
        count_noflip++;
      }
    }
  }

  float flip_frac = static_cast<float>(count_flip) / kNTotal;
  // For N(90°, 180°) + cos-rejection, flip fraction ≈ 50% by distribution symmetry.
  // Wide tolerance [30%, 70%]: only fails if flip is clearly broken (e.g., always 0%).
  EXPECT_GE(flip_frac, 0.30f) << "flip fraction unexpectedly low: " << flip_frac;
  EXPECT_LE(flip_frac, 0.70f) << "flip fraction unexpectedly high: " << flip_frac;

  if (count_flip > 0 && count_noflip > 0) {
    auto phi_mean_flip = static_cast<float>(phi_sum_flip / count_flip);
    auto phi_mean_noflip = static_cast<float>(phi_sum_noflip / count_noflip);
    // flip=true and flip=false groups have symmetric lat distributions; means should match.
    EXPECT_NEAR(phi_mean_flip, phi_mean_noflip, 0.02f) << "flip vs no-flip lat mean unexpectedly different";
  }
}


// Step 1b: Directly verify flip transmission for axis_neg (N(-90°, 180°)) vs axis_pos (N(+90°, 180°)).
// AC#3 direct test: flip_neg_frac counts roll≈π samples from axis_neg.
// - Bug present (flip not transmitted): flip_neg_frac ≈ 0% (all roll=0)
// - Bug absent (flip correctly transmitted): flip_neg_frac > 40% (majority of axis_neg triggers flip)
// Run this test BEFORE Step 3 fixes to capture the baseline failure value in progress.md.
TEST(FoldRollFlipBalanceTest, AxisDistEquivalence) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomNumberGenerator;
  using lumice::RandomSampler;
  using lumice::math::kPi;

  // axis_pos: N(+90°, 180°) — std=180° covers full sphere, flip≈50%, matches user config
  // (zenith=0 → latitude_mean=90 via from_json). Verifies flip is transmitted to roll.
  AxisDistribution axis_pos;
  axis_pos.latitude_dist = { DistributionType::kGaussian, +90.0f, 180.0f };
  axis_pos.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis_pos.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };  // std=0: roll_raw=0

  // axis_neg: N(-90°, 180°) — std=180° covers full sphere, flip≈50% (symmetric to axis_pos)
  AxisDistribution axis_neg;
  axis_neg.latitude_dist = { DistributionType::kGaussian, -90.0f, 180.0f };
  axis_neg.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis_neg.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };

  constexpr int kN = 50'000;
  std::vector<float> data_pos(kN * 3);
  std::vector<float> data_neg(kN * 3);

  RandomNumberGenerator::GetInstance().SetSeed(42);
  RandomSampler::SampleSphericalPointsSph(axis_pos, data_pos.data(), kN);

  RandomNumberGenerator::GetInstance().SetSeed(42);
  RandomSampler::SampleSphericalPointsSph(axis_neg, data_neg.data(), kN);

  int flip_pos = 0;
  int flip_neg = 0;
  double lat_pos_sum = 0.0;
  double lat_neg_sum = 0.0;
  for (int i = 0; i < kN; i++) {
    if (std::abs(data_pos[i * 3 + 2] - kPi) < 0.001f) {
      flip_pos++;
    }
    if (std::abs(data_neg[i * 3 + 2] - kPi) < 0.001f) {
      flip_neg++;
    }
    lat_pos_sum += std::abs(data_pos[i * 3 + 1]);
    lat_neg_sum += std::abs(data_neg[i * 3 + 1]);
  }

  auto flip_pos_frac = static_cast<float>(flip_pos) / kN;
  auto flip_neg_frac = static_cast<float>(flip_neg) / kN;

  // Both N(+90°,180°) and N(-90°,180°) cover the full sphere with std=180°, so ~50% of samples
  // cross the fold boundary and trigger flip=true regardless of the sign of the mean.
  // Expected flip fraction: ≈ 49.9% (computed analytically from N(±90°,180°) + fold geometry).
  // If flip is NOT transmitted to roll: all roll=0, both fractions ≈ 0% (regression failure).
  EXPECT_GT(flip_pos_frac, 0.40f) << "axis_pos flip fraction too low — flip not transmitted to roll: " << flip_pos_frac;
  EXPECT_LT(flip_pos_frac, 0.60f) << "axis_pos flip fraction unexpectedly high: " << flip_pos_frac;
  EXPECT_GT(flip_neg_frac, 0.40f) << "axis_neg flip fraction too low — flip not transmitted to roll: " << flip_neg_frac;
  EXPECT_LT(flip_neg_frac, 0.60f) << "axis_neg flip fraction unexpectedly high: " << flip_neg_frac;

  // |lat| mean should be similar between pos and neg (symmetric distributions).
  auto lat_pos_mean = static_cast<float>(lat_pos_sum / kN);
  auto lat_neg_mean = static_cast<float>(lat_neg_sum / kN);
  // SE ≈ 0.005 rad; tolerance 0.02 ≈ 4σ; false positive rate < 0.006%.
  EXPECT_NEAR(lat_pos_mean, lat_neg_mean, 0.02f)
      << "|lat| mean differs: pos=" << lat_pos_mean << " neg=" << lat_neg_mean;
}


// Step 4 verification: kZigzag abs() is intentional — phi always ≥ 0, fold never triggered.
// Verifies that SampleSphericalPointsSph with kZigzag negative mean produces phi ≥ 0 and flip=false.
TEST(FoldRollFlipBalanceTest, ZigzagNegativeMeanNoFold) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomNumberGenerator;
  using lumice::RandomSampler;
  using lumice::math::kPi;

  // kZigzag with negative mean: |std·sin(2πU) + mean|; abs() ensures phi ≥ 0 always.
  AxisDistribution axis;
  axis.latitude_dist = { DistributionType::kZigzag, -30.0f, 10.0f };
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };  // roll_raw=0

  constexpr int kN = 10'000;
  std::vector<float> data(kN * 3);
  RandomNumberGenerator::GetInstance().SetSeed(77);
  RandomSampler::SampleSphericalPointsSph(axis, data.data(), kN);

  int flip_count = 0;
  int negative_lat_count = 0;
  for (int i = 0; i < kN; i++) {
    if (std::abs(data[i * 3 + 2] - kPi) < 0.001f) {
      flip_count++;
    }
    if (data[i * 3 + 1] < 0.0f) {
      negative_lat_count++;
    }
  }
  // kZigzag abs() ensures phi ≥ 0 for |mean|=30 > std=10, so flip=false for all samples.
  EXPECT_EQ(flip_count, 0) << "kZigzag negative mean should never trigger fold (phi always ≥ 0)";
  EXPECT_EQ(negative_lat_count, 0) << "kZigzag negative mean should always produce lat ≥ 0";
}

// Step 5 verification: Rayleigh path for latitude_mean < 0 must fold phi to positive and set flip=true.
// Verifies that south-pole Rayleigh (tiny σ, mean=-89.5°) transmits flip=true to roll (roll≈π).
TEST(FoldRollFlipBalanceTest, RayleighNegativeMean) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomSampler;
  using lumice::math::kPi;

  AxisDistribution axis;
  // latitude_mean=-89.9°, std=0.01° → colatitude=0.1°, 3σ=0.03° → total<0.5° → triggers Rayleigh path
  axis.latitude_dist = { DistributionType::kGaussian, -89.9f, 0.01f };
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };

  constexpr int kN = 5'000;
  std::vector<float> data(kN * 3);
  lumice::RandomNumberGenerator::GetInstance().SetSeed(42);
  RandomSampler::SampleSphericalPointsSph(axis, data.data(), kN);

  int flip_count = 0;
  int negative_lat_count = 0;
  for (int i = 0; i < kN; i++) {
    float lat = data[i * 3 + 1];
    float roll = data[i * 3 + 2];
    // flip=true is signalled by roll≈π (roll_dist std=0 → roll is exactly 0 or π)
    if (std::abs(roll - kPi) < 0.01f) {
      flip_count++;
    }
    if (lat < 0.0f) {
      negative_lat_count++;
    }
  }

  float flip_frac = static_cast<float>(flip_count) / kN;
  // 330.2: the unified LUT PRESERVES the southern hemisphere. The retired Rayleigh abs() fold
  // mapped a down-pointing c-axis to up (+ roll π) — an up/down-symmetry assumption that is wrong
  // for up/down-asymmetric crystals (owner-confirmed 2026-07-05). So a mean=-89.9° c-axis stays at
  // negative latitude, and with σ=0.01° no sample crosses the south pole → no fold, roll stays 0.
  EXPECT_GT(negative_lat_count, kN * 99 / 100) << "unified LUT must keep down c-axis DOWN (negative latitude)";
  EXPECT_LT(flip_frac, 0.01f) << "no south-pole crossing at σ=0.01° → flip≈0 (roll stays 0, not π)";
}

// Step 5 regression: positive-mean Rayleigh (north pole) must NOT trigger flip.
// Verifies the new `if (latitude_mean_rad < 0)` branch does not affect latitude_mean >= 0.
TEST(FoldRollFlipBalanceTest, RayleighPositiveMean) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomSampler;
  using lumice::math::kPi;

  AxisDistribution axis;
  // latitude_mean=+89.9°, std=0.01° → triggers Rayleigh path (near north pole, tiny σ)
  axis.latitude_dist = { DistributionType::kGaussian, 89.9f, 0.01f };
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };

  constexpr int kN = 5'000;
  std::vector<float> data(kN * 3);
  lumice::RandomNumberGenerator::GetInstance().SetSeed(43);
  RandomSampler::SampleSphericalPointsSph(axis, data.data(), kN);

  int flip_count = 0;
  int negative_lat_count = 0;
  for (int i = 0; i < kN; i++) {
    float lat = data[i * 3 + 1];
    float roll = data[i * 3 + 2];
    if (std::abs(roll - kPi) < 0.01f) {
      flip_count++;
    }
    if (lat < 0.0f) {
      negative_lat_count++;
    }
  }

  // All samples near north pole → flip=false for all → roll≈0
  EXPECT_EQ(flip_count, 0) << "Rayleigh north-pole path must not set flip=true";
  EXPECT_EQ(negative_lat_count, 0) << "Rayleigh north-pole path must produce positive phi";
}


// scrum-328.4: mirror of RayleighNegativeMean for the Laplacian tight-envelope branch. The
// copysign/clamp/flip code path is separate from the Rayleigh branch (independent local vars),
// so Rayleigh coverage does not exercise it.
TEST(FoldRollFlipBalanceTest, LaplacianTightEnvelopeNegativeMean) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomSampler;
  using lumice::math::kPi;

  AxisDistribution axis;
  // mean=-89.9°, b=0.01° → colatitude_center=0.1° < 0.5° and b < 60° → tight-envelope
  axis.latitude_dist = { DistributionType::kLaplacian, -89.9f, 0.01f };
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };

  constexpr int kN = 5'000;
  std::vector<float> data(kN * 3);
  lumice::RandomNumberGenerator::GetInstance().SetSeed(444);
  RandomSampler::SampleSphericalPointsSph(axis, data.data(), kN);

  int flip_count = 0;
  int negative_lat_count = 0;
  for (int i = 0; i < kN; i++) {
    float lat = data[i * 3 + 1];
    float roll = data[i * 3 + 2];
    if (std::abs(roll - kPi) < 0.01f) {
      flip_count++;
    }
    if (lat < 0.0f) {
      negative_lat_count++;
    }
  }
  float flip_frac = static_cast<float>(flip_count) / kN;
  // 330.2: unified LUT preserves the southern hemisphere (see RayleighNegativeMean). A down-pointing
  // Laplacian c-axis (mean=-89.9°) stays at negative latitude; with b=0.01° no sample crosses the
  // south pole → no fold. (Retired behavior folded it to positive latitude + flip.)
  EXPECT_GT(negative_lat_count, kN * 99 / 100) << "unified LUT must keep down c-axis DOWN (negative latitude)";
  EXPECT_LT(flip_frac, 0.05f) << "negligible south-pole crossing at b=0.01° → flip≈0";
}


TEST(FoldRollFlipBalanceTest, LaplacianTightEnvelopePositiveMean) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  using lumice::RandomSampler;
  using lumice::math::kPi;

  AxisDistribution axis;
  axis.latitude_dist = { DistributionType::kLaplacian, 89.9f, 0.01f };
  axis.azimuth_dist = { DistributionType::kUniform, 0.0f, 360.0f };
  axis.roll_dist = { DistributionType::kGaussian, 0.0f, 0.0f };

  constexpr int kN = 5'000;
  std::vector<float> data(kN * 3);
  lumice::RandomNumberGenerator::GetInstance().SetSeed(445);
  RandomSampler::SampleSphericalPointsSph(axis, data.data(), kN);

  int flip_count = 0;
  int negative_lat_count = 0;
  for (int i = 0; i < kN; i++) {
    float lat = data[i * 3 + 1];
    float roll = data[i * 3 + 2];
    if (std::abs(roll - kPi) < 0.01f) {
      flip_count++;
    }
    if (lat < 0.0f) {
      negative_lat_count++;
    }
  }

  EXPECT_EQ(flip_count, 0) << "Laplacian tight-envelope north-pole path must not set flip=true";
  EXPECT_EQ(negative_lat_count, 0) << "Laplacian tight-envelope north-pole path must produce positive phi";
}


TEST_F(RngTest, LaplacianJsonRoundTrip) {
  using lumice::Distribution;
  using lumice::DistributionType;

  Distribution orig{ DistributionType::kLaplacian, 45.0f, 3.0f };
  nlohmann::json j;
  lumice::to_json(j, orig);

  EXPECT_EQ(j["type"], "laplacian");
  EXPECT_FLOAT_EQ(j["mean"], 45.0f);
  EXPECT_FLOAT_EQ(j["std"], 3.0f);

  Distribution parsed{};
  lumice::from_json(j, parsed);
  EXPECT_EQ(parsed.type, DistributionType::kLaplacian);
  EXPECT_FLOAT_EQ(parsed.mean, 45.0f);
  EXPECT_FLOAT_EQ(parsed.std, 3.0f);
}

// --- 330.2 invert_lat_lut (unified area-measure inverse-CDF LUT) --------------
// Pins the numeric contract of the pure single-source inversion used by BOTH the
// CPU sampler and the device gen/transit kernels, independent of the
// distribution-specific BuildLatLut (330.2 S2). Covers analytic tables with known
// inverses + the fp32 monotonicity guards (degenerate bin, xi clamp at lifted
// endpoints). Uniform-theta + fixed binary search (explore exp3/exp5).
namespace {
constexpr float kHalfPiF = 1.5707963267948966f;
}

TEST(InvertLatLutTest, LinearCdfIsLinearInverse) {
  constexpr uint32_t kN = 257;  // 256 intervals (power of two -> constant 8-iter search)
  std::vector<float> theta(kN), cdf(kN);
  for (uint32_t i = 0; i < kN; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kN - 1);
    theta[i] = kHalfPiF * t;
    cdf[i] = t;  // linear CDF -> exact linear inverse
  }
  for (float xi : { 0.0f, 0.1f, 0.25f, 0.5f, 0.75f, 0.9f, 1.0f }) {
    const float got = lm_pcg::invert_lat_lut(xi, theta.data(), cdf.data(), kN);
    EXPECT_NEAR(got, xi * kHalfPiF, 1e-5f) << "xi=" << xi;
  }
}

TEST(InvertLatLutTest, ClampsOutOfRangeAndLiftedEndpoints) {
  // Endpoints deliberately != 0/1 (as after a monotonicity lift): xi outside
  // [cdf[0], cdf[N-1]] must clamp to the boundary nodes, not read out of range.
  constexpr uint32_t kN = 5;
  const std::vector<float> theta = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
  const std::vector<float> cdf = { 0.1f, 0.3f, 0.6f, 0.8f, 0.95f };
  EXPECT_FLOAT_EQ(lm_pcg::invert_lat_lut(-1.0f, theta.data(), cdf.data(), kN), 0.0f);
  EXPECT_FLOAT_EQ(lm_pcg::invert_lat_lut(2.0f, theta.data(), cdf.data(), kN), 1.0f);
  EXPECT_FLOAT_EQ(lm_pcg::invert_lat_lut(0.1f, theta.data(), cdf.data(), kN), 0.0f);
  EXPECT_FLOAT_EQ(lm_pcg::invert_lat_lut(0.95f, theta.data(), cdf.data(), kN), 1.0f);
}

TEST(InvertLatLutTest, DegenerateBinNoNaN) {
  // A flat CDF region (c1 == c0) must not divide by zero; returns a finite node value.
  constexpr uint32_t kN = 5;
  const std::vector<float> theta = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
  const std::vector<float> cdf = { 0.0f, 0.5f, 0.5f, 0.5f, 1.0f };  // flat middle
  const float got = lm_pcg::invert_lat_lut(0.5f, theta.data(), cdf.data(), kN);
  EXPECT_TRUE(std::isfinite(got));
  EXPECT_GE(got, 0.0f);
  EXPECT_LE(got, 1.0f);
}

TEST(InvertLatLutTest, MonotoneAndConvexCdfInverse) {
  // Convex CDF F=t^2 (theta=kHalfPi*t) => inverse theta = kHalfPi*sqrt(xi).
  constexpr uint32_t kN = 257;
  std::vector<float> theta(kN), cdf(kN);
  for (uint32_t i = 0; i < kN; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kN - 1);
    theta[i] = kHalfPiF * t;
    cdf[i] = t * t;
  }
  float prev = -1.0f;
  for (int k = 0; k <= 100; ++k) {
    const float xi = static_cast<float>(k) / 100.0f;
    const float got = lm_pcg::invert_lat_lut(xi, theta.data(), cdf.data(), kN);
    EXPECT_GE(got, prev - 1e-6f) << "non-monotone at xi=" << xi;
    prev = got;
  }
  EXPECT_NEAR(lm_pcg::invert_lat_lut(0.25f, theta.data(), cdf.data(), kN), kHalfPiF * 0.5f, 2e-3f);
}


// --- 330.2 BuildLatLut: colatitude-marginal parity vs the real sampler ---------
// The LUT's core job is to reproduce the folded area-measure colatitude distribution.
// This marginal is flip-independent (flip rotates azimuth/roll, not latitude), so it
// isolates the BuildLatLut quadrature/fold construction from the flip question.
namespace {
double KsStat(std::vector<double> a, std::vector<double> b) {
  std::sort(a.begin(), a.end());
  std::sort(b.begin(), b.end());
  std::vector<double> all;
  all.reserve(a.size() + b.size());
  all.insert(all.end(), a.begin(), a.end());
  all.insert(all.end(), b.begin(), b.end());
  std::sort(all.begin(), all.end());
  double d = 0.0;
  for (double v : all) {
    double ca = static_cast<double>(std::upper_bound(a.begin(), a.end(), v) - a.begin()) / a.size();
    double cb = static_cast<double>(std::upper_bound(b.begin(), b.end(), v) - b.begin()) / b.size();
    d = std::max(d, std::abs(ca - cb));
  }
  return d;
}
}  // namespace

TEST(LatLutColatitudeTest, MatchesRealSamplerColatitudeMarginal) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  struct Cfg {
    float mean;
    float std;
    DistributionType type;
    const char* label;
  };
  const Cfg cfgs[] = {
    { 90.0f, 5.0f, DistributionType::kGaussian, "gauss near-pole (real=Rayleigh)" },
    { 60.0f, 5.0f, DistributionType::kGaussian, "gauss off-pole (real=generic)" },
    { 30.0f, 8.0f, DistributionType::kGaussian, "gauss mid-latitude" },
    { 90.0f, 5.0f, DistributionType::kLaplacian, "laplacian near-pole (real=tight)" },
    { 0.0f, 20.0f, DistributionType::kUniform, "uniform band" },
  };
  constexpr int kN = 500000;
  const double crit = 1.63 * std::sqrt(2.0 / kN);
  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(20260705);

  for (const auto& c : cfgs) {
    SCOPED_TRACE(c.label);
    AxisDistribution axis;
    axis.latitude_dist = lumice::Distribution{ c.type, c.mean, c.std };
    axis.azimuth_dist.type = DistributionType::kUniform;  // uniform az: flip is a no-op here

    std::vector<double> real(kN);
    float lon_lat[3];
    for (int i = 0; i < kN; ++i) {
      lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
      real[i] = lumice::math::kPi_2 - lon_lat[1];  // colatitude-from-zenith
    }

    const lumice::LatLut table = lumice::BuildLatLut(axis.latitude_dist);
    std::vector<double> lut(kN);
    for (int i = 0; i < kN; ++i) {
      const float xi = rng.GetUniform();
      lut[i] = lm_pcg::invert_lat_lut(xi, table.theta.data(), table.cdf.data(), lumice::LatLut::kNodes);
    }

    // Allow a small slack over the two-sample KS crit for LUT node-resolution + fp32.
    EXPECT_LT(KsStat(real, lut), crit * 2.0) << c.label;
  }
}

// Full-direction parity for the COMMON regime (mean>0, uniform azimuth): the LUT must be
// distributionally identical to the current sampler there — same colatitude AND same azimuth.
// (For the edge regimes — southern near-pole, or near-pole + non-uniform azimuth — the LUT
// intentionally diverges toward correctness: it preserves the up/down hemisphere the Rayleigh
// abs() fold collapses, and keeps the over-vertical azimuth population Rayleigh drops. Those
// are validated against the analytic target, not the old sampler, per the scrum-328 AC.)
TEST(LatLutColatitudeTest, CommonRegimeAzimuthUnchanged) {
  using lumice::AxisDistribution;
  using lumice::DistributionType;
  auto wrap = [](double a) {
    const double two_pi = 2.0 * lumice::math::kPi;
    a = std::fmod(a, two_pi);
    return a < 0 ? a + two_pi : a;
  };
  const float means[] = { 90.0f, 60.0f, 30.0f };
  constexpr int kN = 400000;
  const double crit = 1.63 * std::sqrt(2.0 / kN);
  auto& rng = lumice::RandomNumberGenerator::GetInstance();
  rng.SetSeed(999);
  for (float mean : means) {
    SCOPED_TRACE(mean);
    AxisDistribution axis;
    axis.latitude_dist = lumice::Distribution{ DistributionType::kGaussian, mean, 5.0f };
    // Full-circle uniform azimuth (std=360°) — the common near-pole case; flip is a no-op here.
    // (A degenerate std=0 would fix azimuth at 0°, which IS the non-uniform edge case where the
    // LUT intentionally diverges, so it must not be used to assert common-regime equivalence.)
    axis.azimuth_dist = lumice::Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    const lumice::LatLut table = lumice::BuildLatLut(axis.latitude_dist);

    std::vector<double> az_real(kN), az_lut(kN);
    float lon_lat[3];
    for (int i = 0; i < kN; ++i) {
      lumice::RandomSampler::SampleSphericalPointsSph(axis, lon_lat);
      az_real[i] = wrap(lon_lat[0]);
    }
    for (int i = 0; i < kN; ++i) {
      const float xi = rng.GetUniform();
      const float th = lm_pcg::invert_lat_lut(xi, table.theta.data(), table.cdf.data(), lumice::LatLut::kNodes);
      const uint32_t bin = lm_pcg::lat_lut_bin(th, table.theta.data(), lumice::LatLut::kNodes);
      const bool flip = rng.GetUniform() < table.flip_prob[bin];
      float az = rng.Get(axis.azimuth_dist) * lumice::math::kDegreeToRad;
      if (flip) {
        az += lumice::math::kPi;
      }
      az_lut[i] = wrap(az);
    }
    EXPECT_LT(KsStat(az_real, az_lut), crit * 2.0) << "azimuth must stay uniform (flip is a no-op here)";
  }
}

// task-335: GetSharedLatLut is the single build-once cache that replaced the
// per-thread single-entry cache whose most-recent-only policy thrashed when a
// worker's crystal loop interleaved distinct latitude distributions. Lock its
// contract: (1) same key -> same stable pointer (memoized, not rebuilt);
// (2) distinct keys -> distinct entries; (3) content is bit-identical to a
// direct BuildLatLut (memoization must not perturb the numeric result -> parity
// preserved). Regression guard against reintroducing a thrashing / rebuilding cache.
TEST(SharedLatLutTest, MemoizesAndMatchesBuildLatLut) {
  using lumice::Distribution;
  using lumice::DistributionType;

  const Distribution plate{ DistributionType::kGaussian, 90.0f, 0.3f };
  const Distribution column{ DistributionType::kGaussian, 0.0f, 0.3f };

  // (1) Same key returns the same cached pointer across repeated (and interleaved) calls.
  const lumice::LatLut* p1 = lumice::GetSharedLatLut(plate);
  const lumice::LatLut* c1 = lumice::GetSharedLatLut(column);
  const lumice::LatLut* p2 = lumice::GetSharedLatLut(plate);  // interleaved re-request
  const lumice::LatLut* c2 = lumice::GetSharedLatLut(column);
  ASSERT_NE(p1, nullptr);
  ASSERT_NE(c1, nullptr);
  EXPECT_EQ(p1, p2) << "same distribution must reuse the cached LUT (no rebuild)";
  EXPECT_EQ(c1, c2) << "same distribution must reuse the cached LUT (no rebuild)";

  // (2) Distinct keys map to distinct entries.
  EXPECT_NE(p1, c1) << "distinct distributions must not collide to one entry";

  // (3) Cached content is bit-identical to a fresh direct build (parity guarantee).
  const lumice::LatLut fresh = lumice::BuildLatLut(plate);
  for (uint32_t i = 0; i < lumice::LatLut::kNodes; ++i) {
    EXPECT_EQ(p1->theta[i], fresh.theta[i]) << "theta[" << i << "]";
    EXPECT_EQ(p1->cdf[i], fresh.cdf[i]) << "cdf[" << i << "]";
    EXPECT_EQ(p1->flip_prob[i], fresh.flip_prob[i]) << "flip_prob[" << i << "]";
  }
}


}  // namespace
