#include <gtest/gtest.h>

#include "util/lens_fov_default.hpp"

// The two numbers doc/configuration.md's lens "Defaults" section publishes, pinned as bare
// literals on purpose: this is the one authority both config readers call, so its test must not be
// written in terms of the function under test or it would agree with any value it happened to hold.
TEST(LensFovDefault, NonGlobeLensDefaultsToNinetyDegrees) {
  EXPECT_FLOAT_EQ(lumice::LensDefaultFovDegrees(false), 90.0f);
}

TEST(LensFovDefault, GlobeDefaultsToThirtyDegrees) {
  EXPECT_FLOAT_EQ(lumice::LensDefaultFovDegrees(true), 30.0f);
}

// Usable where a constant is required, so a caller may fold it into a constexpr table.
static_assert(lumice::LensDefaultFovDegrees(false) == 90.0f, "non-globe default is 90");
static_assert(lumice::LensDefaultFovDegrees(true) == 30.0f, "globe default is 30");
