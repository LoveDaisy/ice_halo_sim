// The thousands-grouping formatter the analysis panel prints its ray counts through, asserted on
// the boundary values where a grouping rule goes wrong: the empty group at exactly a multiple of
// three digits (999 / 1000), the one-digit and two-digit leading groups, and the widest number the
// type holds, whose twenty digits exercise six separators and a two-digit leading group at once.
// The strings are spelled out rather than derived, so the test cannot share a slip with the code.

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "util/thousands_format.hpp"

namespace lumice {
namespace {

TEST(ThousandsFormat, NumbersBelowAThousandCarryNoSeparator) {
  EXPECT_EQ(FormatThousands(0), "0");
  EXPECT_EQ(FormatThousands(7), "7");
  EXPECT_EQ(FormatThousands(42), "42");
  EXPECT_EQ(FormatThousands(999), "999");
}

TEST(ThousandsFormat, GroupsOfThreeFromTheRight) {
  EXPECT_EQ(FormatThousands(1000), "1,000");
  EXPECT_EQ(FormatThousands(12345), "12,345");
  EXPECT_EQ(FormatThousands(999999), "999,999");
  EXPECT_EQ(FormatThousands(1000000), "1,000,000");
  EXPECT_EQ(FormatThousands(1234567), "1,234,567");
}

TEST(ThousandsFormat, Uint64UpperBound) {
  EXPECT_EQ(FormatThousands(std::numeric_limits<std::uint64_t>::max()), "18,446,744,073,709,551,615");
}

}  // namespace
}  // namespace lumice
