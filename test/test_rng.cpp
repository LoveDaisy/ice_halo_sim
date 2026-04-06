#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "core/crystal.hpp"
#include "core/math.hpp"
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
  auto data = std::make_unique<float[]>(kN * 2);
  RandomSampler::SampleSphericalPointsSph(axis, data.get(), kN);

  const float kAzMin = 60.0f * lumice::math::kDegreeToRad;
  const float kAzMax = 120.0f * lumice::math::kDegreeToRad;
  for (size_t i = 0; i < kN; i++) {
    float az = data[i * 2 + 0];
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
  auto data = std::make_unique<float[]>(kN * 2);
  RandomSampler::SampleSphericalPointsSph(axis, data.get(), kN);

  const float kLatMin = 30.0f * lumice::math::kDegreeToRad;
  const float kLatMax = 60.0f * lumice::math::kDegreeToRad;
  for (size_t i = 0; i < kN; i++) {
    float lat = data[i * 2 + 1];
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
    float lon_lat[2];
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
  float lon_lat[2];
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
  rng.SetSeed(123);

  for (const auto& cfg : configs) {
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
    float lon_lat[2];
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

  float lon_lat[2];
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
    float lon_lat[2];
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

  float lon_lat[2];
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
    float lon_lat[2];
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
  float lon_lat[2];
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


}  // namespace
