#include <gtest/gtest.h>

#include "server/ray_num_semantics.hpp"

// task-323: ray_num was unified to mean TOTAL rays across all wavelengths; the server derives the
// per-wavelength budget via ceil(total / n_wl). This guards the non-divisible (ceil) path, which
// the e2e suite does not exercise directly.
namespace lumice {
namespace {

TEST(RayNumSemantics, CeilNonDivisible) {
  EXPECT_EQ(PerWavelengthRayNum(100, 7), 15u);  // ceil(100/7) = 15
}

TEST(RayNumSemantics, ExactDivisible) {
  EXPECT_EQ(PerWavelengthRayNum(35, 7), 5u);  // exact
}

TEST(RayNumSemantics, CeilJustAboveExact) {
  EXPECT_EQ(PerWavelengthRayNum(36, 7), 6u);  // ceil(36/7) = 6
}

TEST(RayNumSemantics, SingleWavelengthIdentity) {
  EXPECT_EQ(PerWavelengthRayNum(100, 1), 100u);  // n_wl == 1 identity
}

TEST(RayNumSemantics, ZeroWavelengthIdentity) {
  EXPECT_EQ(PerWavelengthRayNum(100, 0), 100u);  // n_wl <= 1 identity (no divide by zero)
}

}  // namespace
}  // namespace lumice
