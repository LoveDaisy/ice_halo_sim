#include <gtest/gtest.h>

#include <bitset>
#include <cstdint>

#include "util/bit_utils.hpp"

namespace lumice {
namespace {

// Compile-time proof that PopCount is genuinely constexpr — a runtime-only
// implementation would not compile here.
static_assert(PopCount(0ull) == 0, "PopCount(0) must be 0");
static_assert(PopCount(~0ull) == 64, "PopCount(~0) must be 64");
static_assert(PopCount(0x1ull) == 1, "PopCount(bit 0) must be 1");
static_assert(PopCount(0xDEADBEEFull) == 24, "PopCount(0xDEADBEEF) must be 24");

TEST(PopCount, Zero) {
  EXPECT_EQ(PopCount(0ull), 0);
}

TEST(PopCount, AllOnes) {
  EXPECT_EQ(PopCount(~0ull), 64);
}

TEST(PopCount, SingleBit) {
  EXPECT_EQ(PopCount(1ull << 0), 1);
  EXPECT_EQ(PopCount(1ull << 31), 1);
  EXPECT_EQ(PopCount(1ull << 63), 1);
}

TEST(PopCount, SparseValuesCrossValidate) {
  constexpr uint64_t kValues[] = {
    0x0ull,
    0x1ull,
    0xDEADBEEFull,
    0xF0F0F0F0F0F0F0F0ull,
    0x0F0F0F0F0F0F0F0Full,
    0xAAAAAAAAAAAAAAAAull,
    0x5555555555555555ull,
    0x8000000000000001ull,
    0xFFFFFFFF00000000ull,
    0x00000000FFFFFFFFull,
    ~0ull,
  };
  for (uint64_t x : kValues) {
    const int kExpected = static_cast<int>(std::bitset<64>(x).count());
    EXPECT_EQ(PopCount(x), kExpected) << "x=0x" << std::hex << x;
  }
}

}  // namespace
}  // namespace lumice
