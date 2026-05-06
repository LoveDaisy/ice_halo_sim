// Unit tests for src/gui/raypath_segments.hpp (multi-segment raypath helper).

#include <gtest/gtest.h>

#include "gui/raypath_segments.hpp"

namespace lumice::gui {
namespace {

// ---- SplitRaypathSegments ----

TEST(RaypathSegments, SplitEmpty) {
  EXPECT_TRUE(SplitRaypathSegments("").empty());
}

TEST(RaypathSegments, SplitSingleSegmentNoSeparator) {
  auto v = SplitRaypathSegments("3-1-5");
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0], "3-1-5");
}

TEST(RaypathSegments, SplitTrimsWhitespace) {
  auto v = SplitRaypathSegments("3-5; 1-3");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], "3-5");
  EXPECT_EQ(v[1], "1-3");
}

TEST(RaypathSegments, SplitKeepsEmptySegments) {
  // Splitter is dumb: it preserves empty segments so the validator can flag them.
  auto v = SplitRaypathSegments("3;;5");
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0], "3");
  EXPECT_TRUE(v[1].empty());
  EXPECT_EQ(v[2], "5");
}

TEST(RaypathSegments, SplitLeadingTrailingSeparators) {
  auto v1 = SplitRaypathSegments(";3");
  ASSERT_EQ(v1.size(), 2u);
  EXPECT_TRUE(v1[0].empty());
  EXPECT_EQ(v1[1], "3");

  auto v2 = SplitRaypathSegments("3;");
  ASSERT_EQ(v2.size(), 2u);
  EXPECT_EQ(v2[0], "3");
  EXPECT_TRUE(v2[1].empty());
}

// ---- ValidateRaypathTextMultiSegment ----

TEST(RaypathSegments, ValidateEmptyIsValid) {
  auto r = ValidateRaypathTextMultiSegment("", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
}

TEST(RaypathSegments, ValidateSingleSegmentDelegates) {
  // No ';' → must behave exactly like single-segment validator.
  auto r = ValidateRaypathTextMultiSegment("3-1-5", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);

  auto r2 = ValidateRaypathTextMultiSegment("3-5-", CrystalKind::kPrism);
  EXPECT_EQ(r2.state, RaypathValidation::kIncomplete);

  auto r3 = ValidateRaypathTextMultiSegment("abc", CrystalKind::kPrism);
  EXPECT_EQ(r3.state, RaypathValidation::kInvalid);
}

TEST(RaypathSegments, ValidateMultiSegmentValid) {
  auto r = ValidateRaypathTextMultiSegment("3-5; 1-3", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
}

TEST(RaypathSegments, ValidateLeadingSemicolonRejected) {
  auto r = ValidateRaypathTextMultiSegment(";3", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
}

TEST(RaypathSegments, ValidateTrailingSemicolonRejected) {
  auto r = ValidateRaypathTextMultiSegment("3-5;", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
}

TEST(RaypathSegments, ValidateConsecutiveSemicolonsRejected) {
  auto r = ValidateRaypathTextMultiSegment("3;;5", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
}

TEST(RaypathSegments, ValidateConsecutiveSemicolonsWithWhitespaceRejected) {
  // "; ;" in a multi-segment context is still empty between separators.
  auto r = ValidateRaypathTextMultiSegment("3 ; ; 5", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
}

TEST(RaypathSegments, ValidatePureSemicolonRejected) {
  auto r = ValidateRaypathTextMultiSegment(";", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
}

TEST(RaypathSegments, ValidateOneInvalidSegmentRejected) {
  // Face 51 is outside any crystal's legal range.
  auto r = ValidateRaypathTextMultiSegment("3-5; 51", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kInvalid);
}

TEST(RaypathSegments, ValidateSegmentMixedSeparators) {
  // Both '-' and ',' are accepted within a segment (legacy comma-as-dash).
  auto r = ValidateRaypathTextMultiSegment("3,5; 1-3", CrystalKind::kPrism);
  EXPECT_EQ(r.state, RaypathValidation::kValid);
}

// ---- ParseRaypathTextMultiSegment ----

TEST(RaypathSegments, ParseSingleSegment) {
  auto v = ParseRaypathTextMultiSegment("3-1-5");
  ASSERT_EQ(v.size(), 1u);
  ASSERT_EQ(v[0].size(), 3u);
  EXPECT_EQ(v[0][0], 3);
  EXPECT_EQ(v[0][1], 1);
  EXPECT_EQ(v[0][2], 5);
}

TEST(RaypathSegments, ParseMultiSegment) {
  auto v = ParseRaypathTextMultiSegment("3-5; 1-3");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], (std::vector<int>{ 3, 5 }));
  EXPECT_EQ(v[1], (std::vector<int>{ 1, 3 }));
}

TEST(RaypathSegments, ParseEmptyInput) {
  EXPECT_TRUE(ParseRaypathTextMultiSegment("").empty());
}

TEST(RaypathSegments, ParseDropsEmptySegments) {
  // Tolerant: invalid ';' patterns produce empty segments, which are dropped.
  // (Strict rejection is the validator's job.)
  auto v = ParseRaypathTextMultiSegment("3;;5");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], (std::vector<int>{ 3 }));
  EXPECT_EQ(v[1], (std::vector<int>{ 5 }));
}

TEST(RaypathSegments, ParseSegmentMixedSeparators) {
  auto v = ParseRaypathTextMultiSegment("3,5; 1-3");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], (std::vector<int>{ 3, 5 }));
  EXPECT_EQ(v[1], (std::vector<int>{ 1, 3 }));
}

}  // namespace
}  // namespace lumice::gui
