// TEMPORARY skeleton case — proves the composition_correctness_test target links, registers and is
// selected by `ctest -L composition-correctness`. Deleted as soon as the first real chain file
// lands; it asserts nothing about the product.

#include <gtest/gtest.h>

TEST(CompositionSkeleton, TargetIsWiredUp) {
  EXPECT_EQ(1 + 1, 2);
}
