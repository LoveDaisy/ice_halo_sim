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
  auto r = ValidateRaypathTextMultiSegment("", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_VALID);
}

TEST(RaypathSegments, ValidateSingleSegmentDelegates) {
  // No ';' → must behave exactly like single-segment validator.
  auto r = ValidateRaypathTextMultiSegment("3-1-5", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_VALID);

  auto r2 = ValidateRaypathTextMultiSegment("3-5-", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r2.state, LUMICE_RAYPATH_INCOMPLETE);

  auto r3 = ValidateRaypathTextMultiSegment("abc", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r3.state, LUMICE_RAYPATH_INVALID);
}

TEST(RaypathSegments, ValidateMultiSegmentValid) {
  auto r = ValidateRaypathTextMultiSegment("3-5; 1-3", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_VALID);
}

TEST(RaypathSegments, ValidateLeadingSemicolonRejected) {
  auto r = ValidateRaypathTextMultiSegment(";3", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
}

TEST(RaypathSegments, ValidateTrailingSemicolonRejected) {
  auto r = ValidateRaypathTextMultiSegment("3-5;", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
}

TEST(RaypathSegments, ValidateConsecutiveSemicolonsRejected) {
  auto r = ValidateRaypathTextMultiSegment("3;;5", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
}

TEST(RaypathSegments, ValidateConsecutiveSemicolonsWithWhitespaceRejected) {
  // "; ;" in a multi-segment context is still empty between separators.
  auto r = ValidateRaypathTextMultiSegment("3 ; ; 5", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
}

TEST(RaypathSegments, ValidatePureSemicolonRejected) {
  auto r = ValidateRaypathTextMultiSegment(";", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
}

TEST(RaypathSegments, ValidateOneInvalidSegmentRejected) {
  // Face 51 is outside any crystal's legal range.
  auto r = ValidateRaypathTextMultiSegment("3-5; 51", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
}

TEST(RaypathSegments, ValidateSegmentRejectsLegacyComma) {
  // ',' inside a segment used to be accepted as a second spelling of '-'. It is retired: the
  // "3,5" segment is now a syntax error, and one bad segment rejects the whole text.
  auto r = ValidateRaypathTextMultiSegment("3,5; 1-3", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
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

TEST(RaypathSegments, ParseSegmentDropsLegacyCommaSegment) {
  // The "3,5" segment yields no path at all, so only the well-formed segment survives. What it
  // must NOT do is contribute a truncated {3}: std::stoi("3,5") returns 3 without throwing.
  auto v = ParseRaypathTextMultiSegment("3,5; 1-3");
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0], (std::vector<int>{ 1, 3 }));
}

// ---- ParseRaypathSegment: whole-segment rejection (the retired ',' connector) ----

TEST(RaypathSegments, ParseSegmentRejectsCommaConnectorEntirely) {
  // The flagship input. A per-token rule would not be enough here: splitting "3-5,1-2" on '-'
  // gives "3" / "5,1" / "2", and dropping only the malformed middle token returns {3, 2} — a
  // plausible-looking two-face path nobody asked for. The segment, not the token, is what is
  // invalidated.
  auto v = ParseRaypathSegment("3-5,1-2");
  EXPECT_TRUE(v.empty());
  EXPECT_NE(v, (std::vector<int>{ 3, 2 }));
}

TEST(RaypathSegments, ParseSegmentRejectsSingleCommaToken) {
  // "3,5" must not come back as {3} — std::stoi stops at the ',' and returns 3 without error,
  // which is how removing the normalization alone would trade a four-face wrong answer for a
  // one-face wrong answer.
  auto v = ParseRaypathSegment("3,5");
  EXPECT_TRUE(v.empty());
  EXPECT_NE(v, (std::vector<int>{ 3 }));
}

TEST(RaypathSegments, ParseSegmentUnchangedForDashOnlyInput) {
  // The other half of the tightening: well-formed input is untouched. Guards against the
  // rejection being over-implemented into "anything unusual yields nothing".
  EXPECT_EQ(ParseRaypathSegment("3-5-1-2"), (std::vector<int>{ 3, 5, 1, 2 }));
  EXPECT_EQ(ParseRaypathSegment("3"), (std::vector<int>{ 3 }));
}

}  // namespace
}  // namespace lumice::gui
