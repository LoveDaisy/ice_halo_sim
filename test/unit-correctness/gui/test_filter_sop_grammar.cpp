// Unit tests for the GUI sum-of-products (SoP) filter data model + AND-grammar
// (scrum filter-editor-uplift / task-gui-sop-data-model).
//
// Coverage map (ACs from task-gui-sop-data-model/issue.md + plan.md §3):
//   AC1 — express + round-trip an arbitrary sum-of-products through the grammar
//         (single raypath / multi-summand OR / cross-type OR / AND-of-factors),
//         plus the legacy → SoP converters (FromLegacyRaypath / FromLegacyEntryExit).
//   AC2 — operator== sensitivity under the AS-DESIGNED semantics (SummandText
//         compares .text only; FilterConfig compares every top-level field + param).
//   Grammar validation — ValidateSummandText accept/reject + ParseLengthSpec modes.
//
// The expectations here are written from the plan's INTENDED behavior, not by
// mirroring whatever the implementation currently returns.

#include <gtest/gtest.h>

#include <string>
#include <variant>
#include <vector>

#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "include/lumice.h"

namespace lumice::gui {
namespace {

// --- helpers ---------------------------------------------------------------

RaypathParams Rp(std::string text) {
  RaypathParams rp;
  rp.raypath_text = std::move(text);
  return rp;
}

EntryExitParams Ee(std::string entry, std::string exit = "", int mode = 0, int min_len = 1, int max_len = 1) {
  EntryExitParams ep;
  ep.entry_text = std::move(entry);
  ep.exit_text = std::move(exit);
  ep.length_mode = mode;
  ep.min_len = min_len;
  ep.max_len = max_len;
  return ep;
}

// Round-trip one summand row's factors through Format → Parse and assert the
// parsed factors are semantically equal to the originals. This is the core AC1
// grammar contract: any factor list a caller can build must survive being
// serialized to canonical text and parsed back.
void ExpectFactorsRoundTrip(const std::vector<Factor>& factors) {
  std::string text = FormatSummandText(factors);
  std::vector<Factor> reparsed = ParseSummandText(text);
  EXPECT_TRUE(reparsed == factors) << "round-trip mismatch; canonical text = \"" << text << "\"";
}

// ===========================================================================
// AC1 — express + round-trip arbitrary sum-of-products
// ===========================================================================

// (a) single raypath factor.
TEST(FilterSopAc1, SingleRaypathRoundTrip) {
  std::vector<Factor> factors{ Factor{ Rp("3-5") } };
  ExpectFactorsRoundTrip(factors);

  auto parsed = ParseSummandText("3-5");
  ASSERT_EQ(parsed.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<RaypathParams>(parsed[0]));
  EXPECT_EQ(std::get<RaypathParams>(parsed[0]).raypath_text, "3-5");
}

// (b) multi-summand OR expressed as a SumOfProducts of several rows. Each row is
// round-tripped independently (the grammar operates per-row; OR lives at the
// SoP/vector level).
TEST(FilterSopAc1, MultiSummandOrRoundTrip) {
  SumOfProducts sop{
    SummandText{ FormatSummandText({ Factor{ Rp("3-5") } }), { Factor{ Rp("3-5") } } },
    SummandText{ FormatSummandText({ Factor{ Rp("1-3") } }), { Factor{ Rp("1-3") } } },
    SummandText{ FormatSummandText({ Factor{ Rp("6-7") } }), { Factor{ Rp("6-7") } } },
  };
  ASSERT_EQ(sop.size(), 3u);
  for (const auto& row : sop) {
    ExpectFactorsRoundTrip(row.factors);
    // The stored canonical text must re-parse to the stored factor cache.
    EXPECT_TRUE(ParseSummandText(row.text) == row.factors);
  }
}

// (c) cross-type OR: one row is a raypath, another is an entry-exit factor.
TEST(FilterSopAc1, CrossTypeOrRoundTrip) {
  std::vector<Factor> raypath_row{ Factor{ Rp("3-5") } };
  std::vector<Factor> ee_row{ Factor{ Ee("2") } };

  ExpectFactorsRoundTrip(raypath_row);
  ExpectFactorsRoundTrip(ee_row);

  // The EE row's canonical text always carries the entry: type anchor.
  EXPECT_EQ(FormatSummandText(ee_row), "entry:2");
}

// (d) AND-of-factors within a single summand: raypath AND entry-exit. Also
// checks that factor order is preserved across the round-trip.
TEST(FilterSopAc1, AndOfFactorsRoundTrip) {
  std::vector<Factor> factors{ Factor{ Rp("3-5") }, Factor{ Ee("2") } };
  EXPECT_EQ(FormatSummandText(factors), "3-5 & entry:2");
  ExpectFactorsRoundTrip(factors);

  // Reversed order (EE first, then raypath) must also survive.
  std::vector<Factor> reversed{ Factor{ Ee("2") }, Factor{ Rp("3-5") } };
  EXPECT_EQ(FormatSummandText(reversed), "entry:2 & 3-5");
  ExpectFactorsRoundTrip(reversed);
}

// A raypath sandwiched between two EE fragments must split into TWO separate EE
// factors (the in-flight EE builder is flushed at each raypath token), not merge
// across the raypath. This pins the "same-row EE tokens merge / raypath breaks
// the run" rule from plan §3.3.
TEST(FilterSopAc1, RaypathBreaksEeMergeRun) {
  std::vector<Factor> factors{ Factor{ Ee("2") }, Factor{ Rp("5") }, Factor{ Ee("", "3") } };
  std::string text = FormatSummandText(factors);
  EXPECT_EQ(text, "entry:2 & 5 & entry: & exit:3");
  auto parsed = ParseSummandText(text);
  ASSERT_EQ(parsed.size(), 3u);
  EXPECT_TRUE(std::holds_alternative<EntryExitParams>(parsed[0]));
  EXPECT_TRUE(std::holds_alternative<RaypathParams>(parsed[1]));
  EXPECT_TRUE(std::holds_alternative<EntryExitParams>(parsed[2]));
  EXPECT_TRUE(parsed == factors);
}

// EE factor carrying entry + exit + length constraint round-trips as a single
// merged factor, and the length spec decodes to the right (mode, min, max).
TEST(FilterSopAc1, EntryExitWithLengthRoundTrip) {
  // range mode
  std::vector<Factor> range_row{ Factor{ Ee("2", "3", /*mode=*/3, /*min_len=*/2, /*max_len=*/4) } };
  EXPECT_EQ(FormatSummandText(range_row), "entry:2 & exit:3 & len:2-4");
  ExpectFactorsRoundTrip(range_row);

  // strict mode
  std::vector<Factor> strict_row{ Factor{ Ee("2", "", /*mode=*/1, /*min_len=*/3, /*max_len=*/3) } };
  EXPECT_EQ(FormatSummandText(strict_row), "entry:2 & len:3");
  ExpectFactorsRoundTrip(strict_row);

  // at-most mode
  std::vector<Factor> atmost_row{ Factor{ Ee("2", "", /*mode=*/2, /*min_len=*/1, /*max_len=*/5) } };
  EXPECT_EQ(FormatSummandText(atmost_row), "entry:2 & len:<=5");
  ExpectFactorsRoundTrip(atmost_row);
}

// Wildcard (empty) entry facelist round-trips thanks to the always-emitted
// entry: anchor (plan §3 default-state disambiguation).
TEST(FilterSopAc1, WildcardEntryRoundTrip) {
  std::vector<Factor> row{ Factor{ Ee("") } };
  EXPECT_EQ(FormatSummandText(row), "entry:");
  ExpectFactorsRoundTrip(row);
}

// Empty text parses to an empty factor list (a "no filter" row).
TEST(FilterSopAc1, EmptyTextParsesEmpty) {
  EXPECT_TRUE(ParseSummandText("").empty());
  EXPECT_TRUE(ParseSummandText("   ").empty());
}

// --- AC3 (degenerate-instance) legacy → SoP converters ---------------------
// These live under the AC1 umbrella per the task brief (round-trip the legacy
// shape into a SoP and back through the grammar).

TEST(FilterSopAc1, FromLegacyRaypathSplitsSemicolonIntoRows) {
  SumOfProducts sop = FromLegacyRaypath(Rp("3-5; 1-3"));
  ASSERT_EQ(sop.size(), 2u);
  EXPECT_EQ(sop[0].text, "3-5");
  EXPECT_EQ(sop[1].text, "1-3");
  // Each row's factor cache is a single raypath factor equal to its text.
  for (const auto& row : sop) {
    ASSERT_EQ(row.factors.size(), 1u);
    ASSERT_TRUE(std::holds_alternative<RaypathParams>(row.factors[0]));
    EXPECT_EQ(std::get<RaypathParams>(row.factors[0]).raypath_text, row.text);
    // Grammar-conformant: the row text re-parses to the row's factor cache.
    EXPECT_TRUE(ParseSummandText(row.text) == row.factors);
  }
}

TEST(FilterSopAc1, FromLegacyRaypathSingleSegmentIsSingleRow) {
  SumOfProducts sop = FromLegacyRaypath(Rp("3-5"));
  ASSERT_EQ(sop.size(), 1u);
  EXPECT_EQ(sop[0].text, "3-5");
}

TEST(FilterSopAc1, FromLegacyRaypathEmptyIsSingleEmptyRow) {
  // Empty legacy raypath → the "no filter" degenerate SoP: 1 row, 1 empty
  // raypath factor (matches the FilterConfig default state).
  SumOfProducts sop = FromLegacyRaypath(Rp(""));
  ASSERT_EQ(sop.size(), 1u);
  EXPECT_TRUE(sop[0].text.empty());
  ASSERT_EQ(sop[0].factors.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<RaypathParams>(sop[0].factors[0]));
  EXPECT_TRUE(std::get<RaypathParams>(sop[0].factors[0]).raypath_text.empty());
}

TEST(FilterSopAc1, FromLegacyEntryExitIsSingleRowAllModes) {
  struct Case {
    EntryExitParams ep;
    const char* expect_text;
  };
  Case cases[] = {
    { Ee("2"), "entry:2" },
    { Ee("2", "3"), "entry:2 & exit:3" },
    { Ee("2", "", 1, 4, 4), "entry:2 & len:4" },
    { Ee("2", "3", 2, 1, 6), "entry:2 & exit:3 & len:<=6" },
    { Ee("2", "3", 3, 2, 5), "entry:2 & exit:3 & len:2-5" },
  };
  for (const auto& c : cases) {
    SumOfProducts sop = FromLegacyEntryExit(c.ep);
    ASSERT_EQ(sop.size(), 1u);
    EXPECT_EQ(sop[0].text, c.expect_text);
    ASSERT_EQ(sop[0].factors.size(), 1u);
    ASSERT_TRUE(std::holds_alternative<EntryExitParams>(sop[0].factors[0]));
    EXPECT_TRUE(std::get<EntryExitParams>(sop[0].factors[0]) == c.ep);
    // The emitted text re-parses to the same EntryExitParams (bijective).
    auto reparsed = ParseSummandText(sop[0].text);
    ASSERT_EQ(reparsed.size(), 1u);
    ASSERT_TRUE(std::holds_alternative<EntryExitParams>(reparsed[0]));
    EXPECT_TRUE(std::get<EntryExitParams>(reparsed[0]) == c.ep);
  }
}

// ===========================================================================
// AC2 — operator== sensitivity (AS-DESIGNED semantics)
// ===========================================================================

// SummandText::operator== compares .text ONLY; the factors parse-cache is
// intentionally excluded. This is a documented design fact, so assert both
// directions explicitly.
TEST(FilterSopAc2, SummandTextComparesTextOnly) {
  SummandText same_text_diff_factors_a{ "3-5", {} };
  SummandText same_text_diff_factors_b{ "3-5", { Factor{ Rp("3-5") }, Factor{ Ee("2") } } };
  // DESIGN FACT: equal because .text matches, even though .factors differ wildly.
  EXPECT_TRUE(same_text_diff_factors_a == same_text_diff_factors_b);

  SummandText diff_text_a{ "3-5", {} };
  SummandText diff_text_b{ "3-6", {} };
  EXPECT_TRUE(diff_text_a != diff_text_b);
}

TEST(FilterSopAc2, IdenticalSopIsEqual) {
  FilterConfig a;
  a.SetRaypath(Rp("3-5"));
  FilterConfig b;
  b.SetRaypath(Rp("3-5"));
  EXPECT_TRUE(a == b);
}

TEST(FilterSopAc2, AddedOrRowIsNotEqual) {
  FilterConfig a;
  a.param = SumOfProducts{ SummandText{ "3-5", { Factor{ Rp("3-5") } } } };
  FilterConfig b = a;
  b.param.push_back(SummandText{ "1-3", { Factor{ Rp("1-3") } } });
  EXPECT_TRUE(a != b);
}

TEST(FilterSopAc2, RemovedOrRowIsNotEqual) {
  FilterConfig a;
  a.param = SumOfProducts{
    SummandText{ "3-5", { Factor{ Rp("3-5") } } },
    SummandText{ "1-3", { Factor{ Rp("1-3") } } },
  };
  FilterConfig b = a;
  b.param.pop_back();
  EXPECT_TRUE(a != b);
}

TEST(FilterSopAc2, ChangedRowTextIsNotEqual) {
  FilterConfig a;
  a.param = SumOfProducts{ SummandText{ "3-5 & entry:2", {} } };
  FilterConfig b = a;
  b.param[0].text = "3-5 & entry:3";
  EXPECT_TRUE(a != b);
}

// FilterConfig::operator== must reflect every top-level field plus param.
TEST(FilterSopAc2, FilterConfigFieldSensitivity) {
  FilterConfig base;
  base.name = "f";
  base.action = 0;
  base.sym_p = true;
  base.sym_b = true;
  base.sym_d = true;
  base.SetRaypath(Rp("3-5"));

  {
    FilterConfig o = base;
    o.name = "g";
    EXPECT_TRUE(base != o);
  }
  {
    FilterConfig o = base;
    o.action = 1;
    EXPECT_TRUE(base != o);
  }
  {
    FilterConfig o = base;
    o.sym_p = false;
    EXPECT_TRUE(base != o);
  }
  {
    FilterConfig o = base;
    o.sym_b = false;
    EXPECT_TRUE(base != o);
  }
  {
    FilterConfig o = base;
    o.sym_d = false;
    EXPECT_TRUE(base != o);
  }
  {
    FilterConfig o = base;
    o.SetRaypath(Rp("1-3"));  // param change
    EXPECT_TRUE(base != o);
  }
  {
    // The copy is the point: a distinct object built from `base` must compare equal.
    FilterConfig o = base;  // NOLINT(performance-unnecessary-copy-initialization)
    EXPECT_TRUE(base == o);
  }
}

// --- default construction + compat accessors (data-model contract) ---------

TEST(FilterSopModel, DefaultIsEmptyRaypathNoFilter) {
  FilterConfig f;
  EXPECT_TRUE(f.IsDegenerateSingleFactor());
  EXPECT_TRUE(f.IsRaypath());
  EXPECT_FALSE(f.IsEntryExit());
  EXPECT_TRUE(f.RaypathText().empty());
}

TEST(FilterSopModel, SetRaypathCompatWriter) {
  FilterConfig f;
  f.SetRaypath(Rp("3-5"));
  EXPECT_TRUE(f.IsRaypath());
  EXPECT_FALSE(f.IsEntryExit());
  EXPECT_EQ(f.RaypathText(), "3-5");
  ASSERT_EQ(f.param.size(), 1u);
  EXPECT_EQ(f.param[0].text, "3-5");
}

// Documented risk-1 caveat: SetRaypath transparently passes ';' multi-segment
// sugar through into the row text WITHOUT splitting into rows. This is the
// intentional compat-layer asymmetry (canonical fan-out is FromLegacyRaypath).
TEST(FilterSopModel, SetRaypathPassesSemicolonThroughVerbatim) {
  FilterConfig f;
  f.SetRaypath(Rp("3-5; 1-3"));
  EXPECT_TRUE(f.IsRaypath());
  ASSERT_EQ(f.param.size(), 1u);  // NOT split into 2 rows
  EXPECT_EQ(f.RaypathText(), "3-5; 1-3");
  EXPECT_EQ(f.param[0].text, "3-5; 1-3");
}

TEST(FilterSopModel, SetEntryExitCompatWriter) {
  FilterConfig f;
  EntryExitParams ep = Ee("2", "3", 3, 2, 5);
  f.SetEntryExit(ep);
  EXPECT_TRUE(f.IsEntryExit());
  EXPECT_FALSE(f.IsRaypath());
  EXPECT_TRUE(f.EntryExitParamsValue() == ep);
  ASSERT_EQ(f.param.size(), 1u);
  EXPECT_EQ(f.param[0].text, "entry:2 & exit:3 & len:2-5");
  // DegenerateFactor() surfaces the same EE factor.
  ASSERT_TRUE(std::holds_alternative<EntryExitParams>(f.DegenerateFactor()));
  EXPECT_TRUE(std::get<EntryExitParams>(f.DegenerateFactor()) == ep);
}

// A genuine multi-row / multi-factor SoP is NOT a degenerate single factor.
TEST(FilterSopModel, NonDegenerateSopIsNotSingleFactor) {
  FilterConfig f;
  f.param = SumOfProducts{
    SummandText{ "3-5", { Factor{ Rp("3-5") } } },
    SummandText{ "entry:2", { Factor{ Ee("2") } } },
  };
  EXPECT_FALSE(f.IsDegenerateSingleFactor());
  EXPECT_FALSE(f.IsRaypath());
  EXPECT_FALSE(f.IsEntryExit());
}

// ===========================================================================
// Grammar validation — ValidateSummandText + ParseLengthSpec
// ===========================================================================

TEST(FilterSopGrammar, ValidateEmptyIsValid) {
  EXPECT_EQ(ValidateSummandText("", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_VALID);
  EXPECT_EQ(ValidateSummandText("   ", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_VALID);
}

TEST(FilterSopGrammar, ValidateAcceptsWellFormedRows) {
  const char* good[] = {
    "3-5",
    "3-5 & entry:2",
    "entry:2 & exit:3",
    "entry:2 & exit:3 & len:2-4",
    "entry:",            // wildcard entry anchor
    "entry: & len:<=5",  // wildcard entry + length
    "3-5 & entry:2 & exit:4 & len:3",
  };
  for (const char* g : good) {
    EXPECT_EQ(ValidateSummandText(g, LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_VALID) << "expected VALID: " << g;
  }
}

TEST(FilterSopGrammar, ValidateRejectsDanglingAmpersand) {
  EXPECT_EQ(ValidateSummandText("3-5 &", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("& 3-5", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("3-5 && 2", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
}

TEST(FilterSopGrammar, ValidateRejectsUnknownPrefix) {
  // "foo:2" is not an EE token and is not a valid raypath token.
  EXPECT_EQ(ValidateSummandText("foo:2", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
}

TEST(FilterSopGrammar, ValidateRejectsDuplicateEeToken) {
  EXPECT_EQ(ValidateSummandText("entry:2 & entry:3", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("len:3 & len:4", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
}

TEST(FilterSopGrammar, ValidateRejectsBadFacePerCrystalKind) {
  // Face 13 is a pyramid-only face: legal on PYRAMID, illegal on PRISM.
  EXPECT_EQ(ValidateSummandText("13", LUMICE_CRYSTAL_PYRAMID).state, LUMICE_RAYPATH_VALID);
  EXPECT_EQ(ValidateSummandText("13", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  // Same rule applies inside an entry: facelist.
  EXPECT_EQ(ValidateSummandText("entry:13", LUMICE_CRYSTAL_PYRAMID).state, LUMICE_RAYPATH_VALID);
  EXPECT_EQ(ValidateSummandText("entry:13", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  // Face 51 is outside every crystal's range.
  EXPECT_EQ(ValidateSummandText("entry:51", LUMICE_CRYSTAL_PYRAMID).state, LUMICE_RAYPATH_INVALID);
}

TEST(FilterSopGrammar, ValidateRejectsMalformedLengthSpec) {
  EXPECT_EQ(ValidateSummandText("len:abc", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("len:0", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("len:5-2", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("len:<=0", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
}

// Regression (code-review-01 Major 2): ValidateSummandText must AGREE with
// ParseSummandText. A raypath token flushes the in-flight EE factor, so a later
// entry:/exit: begins a NEW EE factor and must NOT be rejected as a "duplicate".
// Before the fix, the validator carried EE state across the raypath and rejected
// this parser-accepted, round-trippable input.
TEST(FilterSopGrammar, ValidateAcceptsRaypathBrokenEeRun) {
  EXPECT_EQ(ValidateSummandText("entry:2 & 5 & entry:3", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_VALID);
  // ...and the parser genuinely produces two independent EE factors around the raypath.
  auto parsed = ParseSummandText("entry:2 & 5 & entry:3");
  ASSERT_EQ(parsed.size(), 3u);
  EXPECT_TRUE(std::holds_alternative<EntryExitParams>(parsed[0]));
  EXPECT_TRUE(std::holds_alternative<RaypathParams>(parsed[1]));
  EXPECT_TRUE(std::holds_alternative<EntryExitParams>(parsed[2]));
  // exit: after the raypath is likewise a fresh factor, not a duplicate.
  EXPECT_EQ(ValidateSummandText("entry:2 & 5 & exit:3", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_VALID);
}

// Regression (code-review-01 Major 1): an all-digit len token that overflows int
// must be REJECTED, not crash. Before the fix std::stoi threw an uncaught
// std::out_of_range on these inputs.
TEST(FilterSopGrammar, ValidateRejectsOverflowLengthSpec) {
  EXPECT_EQ(ValidateSummandText("len:99999999999999999999", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("len:<=99999999999999999999", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  EXPECT_EQ(ValidateSummandText("len:1-99999999999999999999", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  // The tolerant parser must not crash AND must not fabricate a factor from the
  // all-invalid EE run (code-review-02 Major 1): the whole token is skipped.
  EXPECT_TRUE(ParseSummandText("len:99999999999999999999").empty());
  int mode = 0;
  int min_len = 0;
  int max_len = 0;
  EXPECT_FALSE(detail::ParseLengthSpec("99999999999999999999", mode, min_len, max_len));
  EXPECT_FALSE(detail::ParseLengthSpec("<=99999999999999999999", mode, min_len, max_len));
  EXPECT_FALSE(detail::ParseLengthSpec("1-99999999999999999999", mode, min_len, max_len));
}

// Regression (code-review-02 Major 1): the tolerant parser SKIPS an EE token
// that fails to merge — it must never fabricate a match-everything wildcard EE.
TEST(FilterSopGrammar, ParseSkipsAllInvalidEeRun) {
  // All-invalid EE run → no factor (not a default EntryExitParams{} wildcard).
  EXPECT_TRUE(ParseSummandText("len:abc").empty());
  EXPECT_TRUE(ParseSummandText("len:0").empty());
  // Partially-valid EE run keeps the valid field, drops the invalid token.
  auto partial = ParseSummandText("entry:2 & len:abc");
  ASSERT_EQ(partial.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<EntryExitParams>(partial[0]));
  EXPECT_EQ(std::get<EntryExitParams>(partial[0]).entry_text, "2");
  EXPECT_EQ(std::get<EntryExitParams>(partial[0]).length_mode, 0);  // bad len skipped
}

// Contract lock (code-review-03 Minor 2): tolerant parser and strict validator
// INTENTIONALLY diverge on a duplicate same-field EE token — the parser keeps
// the first value ("first-wins", drops the dup) while the validator rejects the
// whole row. Pinned so this asymmetry can't silently drift (the same
// Validate/Parse-divergence root cause was a Major twice; see backlog
// "filter grammar Validate/Parse 遍历统一").
TEST(FilterSopGrammar, DuplicateEeTokenParserFirstWinsValidatorRejects) {
  // duplicate entry:
  EXPECT_EQ(ValidateSummandText("entry:2 & entry:3", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  auto dup_entry = ParseSummandText("entry:2 & entry:3");
  ASSERT_EQ(dup_entry.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<EntryExitParams>(dup_entry[0]));
  EXPECT_EQ(std::get<EntryExitParams>(dup_entry[0]).entry_text, "2");  // first wins
  // duplicate len:
  EXPECT_EQ(ValidateSummandText("len:3 & len:4", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
  auto dup_len = ParseSummandText("len:3 & len:4");
  ASSERT_EQ(dup_len.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<EntryExitParams>(dup_len[0]));
  EXPECT_EQ(std::get<EntryExitParams>(dup_len[0]).length_mode, 1);  // strict N, first wins
  EXPECT_EQ(std::get<EntryExitParams>(dup_len[0]).min_len, 3);
}

TEST(FilterSopGrammar, ParseLengthSpecStrict) {
  int mode = 0;
  int min_len = 0;
  int max_len = 0;
  ASSERT_TRUE(detail::ParseLengthSpec("3", mode, min_len, max_len));
  EXPECT_EQ(mode, 1);
  EXPECT_EQ(min_len, 3);
  EXPECT_EQ(max_len, 3);
}

TEST(FilterSopGrammar, ParseLengthSpecAtMost) {
  int mode = 0;
  int min_len = 0;
  int max_len = 0;
  ASSERT_TRUE(detail::ParseLengthSpec("<=5", mode, min_len, max_len));
  EXPECT_EQ(mode, 2);
  EXPECT_EQ(min_len, 1);
  EXPECT_EQ(max_len, 5);
}

TEST(FilterSopGrammar, ParseLengthSpecRange) {
  int mode = 0;
  int min_len = 0;
  int max_len = 0;
  ASSERT_TRUE(detail::ParseLengthSpec("2-4", mode, min_len, max_len));
  EXPECT_EQ(mode, 3);
  EXPECT_EQ(min_len, 2);
  EXPECT_EQ(max_len, 4);
}

TEST(FilterSopGrammar, ParseLengthSpecRejectsMalformed) {
  int mode = 0;
  int min_len = 0;
  int max_len = 0;
  EXPECT_FALSE(detail::ParseLengthSpec("", mode, min_len, max_len));
  EXPECT_FALSE(detail::ParseLengthSpec("0", mode, min_len, max_len));
  EXPECT_FALSE(detail::ParseLengthSpec("abc", mode, min_len, max_len));
  EXPECT_FALSE(detail::ParseLengthSpec("5-2", mode, min_len, max_len));  // min > max
  EXPECT_FALSE(detail::ParseLengthSpec("<=0", mode, min_len, max_len));
}

// ===========================================================================
// 334.3 H-A — single-row multi-raypath OR via ';' inside the AND grammar.
//
// Pre-334.3, ValidateSummandText rejected any raypath token containing ';'.
// H-A relaxes that: a raypath token MAY carry ';'-alternatives which the
// downstream FactorAlternatives/ExpandSopToClauses expand into multiple
// summands at serialization time. The grammar contract for the validator is
// now "delegate to ValidateRaypathTextMultiSegment" — leading/trailing/
// consecutive ';' still reject (empty segment), inner ';' between valid
// segments accepts.
// ===========================================================================

TEST(FilterSopGrammar, ValidateAcceptsSemicolonMultiRaypath) {
  const char* good[] = {
    "1-3;3-5",
    "1-3;3-5 & entry:2",
    "entry:2 & 1-3;3-5",
    "1,2;3-5 & entry:2",           // ',' inside a raypath segment + ';' separator
    "entry:2 & 1-3;3-5 & exit:4",  // ';' raypath sandwiched between EE factors
  };
  for (const char* g : good) {
    EXPECT_EQ(ValidateSummandText(g, LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_VALID) << "expected VALID: " << g;
  }
}

TEST(FilterSopGrammar, ValidateRejectsMalformedSemicolonRaypath) {
  // Leading / trailing / consecutive ';' are rejected as "Empty raypath
  // segment" by ValidateRaypathTextMultiSegment. Also cover ';'+'&'
  // adjacency, which must not accidentally slip through as a valid segment.
  const char* bad[] = {
    ";3-5", "3-5;", "3;;5",
    "3-5; & entry:2",  // ';' immediately before AND separator = trailing ';' in the raypath token
  };
  for (const char* b : bad) {
    EXPECT_EQ(ValidateSummandText(b, LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID) << "expected INVALID: " << b;
  }
}

TEST(FilterSopGrammar, SemicolonMultiRaypathRoundTrip) {
  // ParseSummandText stores the whole ';'-carrying token verbatim in
  // RaypathParams.raypath_text; FormatSummandText echoes it back byte for
  // byte. Fan-out into multiple summands happens later, at serialization.
  std::vector<Factor> factors{ Factor{ Rp("1-3;3-5") } };
  ExpectFactorsRoundTrip(factors);

  auto parsed = ParseSummandText("1-3;3-5");
  ASSERT_EQ(parsed.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<RaypathParams>(parsed[0]));
  EXPECT_EQ(std::get<RaypathParams>(parsed[0]).raypath_text, "1-3;3-5");

  // With an AND partner, the ';' stays inside the single raypath factor;
  // the row parses into 2 factors, not 3.
  auto with_ee = ParseSummandText("1-3;3-5 & entry:2");
  ASSERT_EQ(with_ee.size(), 2u);
  ASSERT_TRUE(std::holds_alternative<RaypathParams>(with_ee[0]));
  EXPECT_EQ(std::get<RaypathParams>(with_ee[0]).raypath_text, "1-3;3-5");
  ASSERT_TRUE(std::holds_alternative<EntryExitParams>(with_ee[1]));
}

// ===========================================================================
// 334.3 H-D — pure preview formatters. These NEVER drive filter behavior;
// they mirror the syntactic ';' fan-out for the editor's live preview so
// users can see what their row expands to before committing.
// ===========================================================================

TEST(FilterSopPreview, ExpandsSemicolonAlternatives) {
  std::vector<Factor> factors{ Factor{ Rp("1-3;3-5") }, Factor{ Ee("2") } };
  EXPECT_EQ(FormatSummandExpansionPreview(factors), "(1-3 OR 3-5) & entry:2");
}

TEST(FilterSopPreview, SingleSegmentRaypathUnchanged) {
  // No ';' → no parentheses, byte-identical to FormatSummandText output.
  std::vector<Factor> factors{ Factor{ Rp("3-5") } };
  EXPECT_EQ(FormatSummandExpansionPreview(factors), "3-5");
  EXPECT_EQ(FormatSummandExpansionPreview(factors), FormatSummandText(factors));
}

TEST(FilterSopPreview, EeCommaListNotReexpanded) {
  // EE tokens have their own comma-list expansion semantics (handled
  // downstream by ExpandSopToClauses in file_io.cpp); the preview helper
  // MUST NOT re-implement that expansion — it shows the EE token verbatim.
  std::vector<Factor> factors{ Factor{ Ee("1,2") } };
  EXPECT_EQ(FormatSummandExpansionPreview(factors), "entry:1,2");
}

TEST(FilterSopPreview, SopOverviewListsAllRows) {
  SumOfProducts sop;
  sop.emplace_back(SummandText{ std::string{ "1-3;3-5" }, { Factor{ Rp("1-3;3-5") } } });
  sop.emplace_back(SummandText{ std::string{ "entry:2" }, { Factor{ Ee("2") } } });
  const std::string preview = FormatSopExpansionPreview(sop);
  EXPECT_EQ(preview,
            "OR of 2 row(s):"
            "\n  (1-3 OR 3-5)"
            "\n  entry:2");
}

TEST(FilterSopPreview, SopOverviewEmptyRowRenderedAsWildcard) {
  // A single blank row (no factors) is the "match-all" sentinel; the SoP
  // overview must show it as "*" — same convention as the pre-334.3 card
  // tooltip, so migrating panels.cpp to FormatSopExpansionPreview is
  // byte-identical for the empty-row case.
  SumOfProducts sop;
  sop.emplace_back(SummandText{ std::string{}, {} });
  const std::string preview = FormatSopExpansionPreview(sop);
  EXPECT_EQ(preview, "OR of 1 row(s):\n  *");
}

}  // namespace
}  // namespace lumice::gui
