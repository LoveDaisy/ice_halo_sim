// Tests for src/util/cpu_info.{hpp,cpp} — PhysicalCoreCount() invariants.

#include <thread>

#include "gtest/gtest.h"
#include "util/cpu_info.hpp"

namespace {

TEST(CpuInfoTest, PhysicalCoreCountIsAtLeastOne) {
  EXPECT_GE(lumice::PhysicalCoreCount(), 1);
}

TEST(CpuInfoTest, PhysicalCoreCountDoesNotExceedHardwareConcurrency) {
  auto hw = static_cast<int>(std::thread::hardware_concurrency());
  if (hw > 0) {
    EXPECT_LE(lumice::PhysicalCoreCount(), hw);
  } else {
    GTEST_SKIP() << "hardware_concurrency() returned 0; cannot bound PhysicalCoreCount()";
  }
}

TEST(CpuInfoTest, PhysicalCoreCountIsStable) {
  int a = lumice::PhysicalCoreCount();
  int b = lumice::PhysicalCoreCount();
  EXPECT_EQ(a, b);
}

}  // namespace
