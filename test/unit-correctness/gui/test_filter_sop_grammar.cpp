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

// AC1 as one table: any factor list a caller can build must serialize to the canonical text below
// and parse back to the same factors. One row per shape the grammar has to express — a single
// raypath, an AND of factors in either order, an EE factor with each length mode, the wildcard
// entry, and the raypath-breaks-the-EE-run rule — because the assertion is identical for all of
// them and one case per shape was one preamble per shape.
TEST(FilterSopAc1, EveryExpressibleFactorListRoundTripsThroughItsCanonicalText) {
  struct Case {
    const char* name;
    std::vector<Factor> factors;
    const char* canonical_text;
  };
  const std::vector<Case> kCases = {
    { "single raypath", { Factor{ Rp("3-5") } }, "3-5" },
    // The EE row's canonical text always carries the entry: type anchor, which is what makes the
    // wildcard (empty facelist) row distinguishable from an empty row at all.
    { "single entry/exit", { Factor{ Ee("2") } }, "entry:2" },
    { "wildcard entry", { Factor{ Ee("") } }, "entry:" },
    // AND of factors, both orders: factor order is preserved across the round trip.
    { "raypath AND entry/exit", { Factor{ Rp("3-5") }, Factor{ Ee("2") } }, "3-5 & entry:2" },
    { "entry/exit AND raypath", { Factor{ Ee("2") }, Factor{ Rp("3-5") } }, "entry:2 & 3-5" },
    // A raypath sandwiched between two EE fragments splits into TWO separate EE factors (the
    // in-flight EE builder is flushed at each raypath token) rather than merging across it.
    { "a raypath breaks the EE merge run",
      { Factor{ Ee("2") }, Factor{ Rp("5") }, Factor{ Ee("", "3") } },
      "entry:2 & 5 & entry: & exit:3" },
    // Each length mode decodes to its own spelling.
    { "EE with a range length",
      { Factor{ Ee("2", "3", /*mode=*/3, /*min_len=*/2, /*max_len=*/4) } },
      "entry:2 & exit:3 & len:2-4" },
    { "EE with a strict length",
      { Factor{ Ee("2", "", /*mode=*/1, /*min_len=*/3, /*max_len=*/3) } },
      "entry:2 & len:3" },
    { "EE with an at-most length",
      { Factor{ Ee("2", "", /*mode=*/2, /*min_len=*/1, /*max_len=*/5) } },
      "entry:2 & len:<=5" },
  };

  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    EXPECT_EQ(FormatSummandText(c.factors), c.canonical_text);
    ExpectFactorsRoundTrip(c.factors);
  }

  // OR lives at the SoP/vector level, not in the grammar: each row is round-tripped independently,
  // and a row's stored canonical text must re-parse to its stored factor cache.
  const SumOfProducts sop{
    SummandText{ FormatSummandText({ Factor{ Rp("3-5") } }), { Factor{ Rp("3-5") } } },
    SummandText{ FormatSummandText({ Factor{ Ee("2") } }), { Factor{ Ee("2") } } },
    SummandText{ FormatSummandText({ Factor{ Rp("6-7") } }), { Factor{ Rp("6-7") } } },
  };
  for (const auto& row : sop) {
    ExpectFactorsRoundTrip(row.factors);
    EXPECT_TRUE(ParseSummandText(row.text) == row.factors);
  }

  // An empty row is "no filter": no factors at all.
  EXPECT_TRUE(ParseSummandText("").empty());
  EXPECT_TRUE(ParseSummandText("   ").empty());
}

// --- AC3 (degenerate-instance) legacy → SoP converters ---------------------
// These live under the AC1 umbrella per the task brief (round-trip the legacy
// shape into a SoP and back through the grammar).

TEST(FilterSopAc1, FromLegacyRaypathFansOutToOneRowPerSegment) {
  struct Case {
    const char* text;
    std::vector<std::string> expect_rows;
  };
  const Case kCases[] = {
    { "3-5; 1-3", { "3-5", "1-3" } },
    { "3-5", { "3-5" } },
    // Empty legacy raypath -> zero rows, which is "no filter" said in the type's own vocabulary and
    // is also the FilterConfig default. It used to produce one row holding one empty raypath factor
    // instead; that row is not "no filter" but the editor's match-all, and a filter_out entry
    // wearing it excludes every ray.
    { "", {} },
  };
  for (const Case& c : kCases) {
    SumOfProducts sop = FromLegacyRaypath(Rp(c.text));
    if (sop.size() != c.expect_rows.size()) {
      ADD_FAILURE() << c.text << ": expected " << c.expect_rows.size() << " rows, got " << sop.size();
      continue;  // no rows to index for this case; the rest still get checked
    }
    for (size_t i = 0; i < c.expect_rows.size(); ++i) {
      EXPECT_EQ(sop[i].text, c.expect_rows[i]) << c.text << " row " << i;
      if (sop[i].factors.size() != 1u) {
        ADD_FAILURE() << c.text << " row " << i << ": expected exactly 1 factor, got " << sop[i].factors.size();
        continue;  // no single factor to check for this row; the rest still get checked
      }
      if (!std::holds_alternative<RaypathParams>(sop[i].factors[0])) {
        ADD_FAILURE() << c.text << " row " << i << ": factor is not a RaypathParams";
        continue;
      }
      EXPECT_EQ(std::get<RaypathParams>(sop[i].factors[0]).raypath_text, sop[i].text);
      // Grammar-conformant: the row text re-parses to the row's factor cache. Not asked of the
      // empty row — the grammar reads "" as no factors at all, while the degenerate SoP has to
      // carry one empty raypath factor for the editor to have a row to show.
      if (!sop[i].text.empty()) {
        EXPECT_TRUE(ParseSummandText(sop[i].text) == sop[i].factors) << c.text << " row " << i;
      }
    }
  }
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
    if (sop.size() != 1u) {
      ADD_FAILURE() << c.expect_text << ": expected exactly 1 row, got " << sop.size();
      continue;  // no row to index for this case; the rest still get checked
    }
    EXPECT_EQ(sop[0].text, c.expect_text);
    if (sop[0].factors.size() != 1u) {
      ADD_FAILURE() << c.expect_text << ": expected exactly 1 factor, got " << sop[0].factors.size();
      continue;
    }
    if (!std::holds_alternative<EntryExitParams>(sop[0].factors[0])) {
      ADD_FAILURE() << c.expect_text << ": factor is not an EntryExitParams";
      continue;
    }
    EXPECT_TRUE(std::get<EntryExitParams>(sop[0].factors[0]) == c.ep);
    // The emitted text re-parses to the same EntryExitParams (bijective).
    auto reparsed = ParseSummandText(sop[0].text);
    if (reparsed.size() != 1u) {
      ADD_FAILURE() << c.expect_text << ": reparse expected exactly 1 factor, got " << reparsed.size();
      continue;
    }
    if (!std::holds_alternative<EntryExitParams>(reparsed[0])) {
      ADD_FAILURE() << c.expect_text << ": reparsed factor is not an EntryExitParams";
      continue;
    }
    EXPECT_TRUE(std::get<EntryExitParams>(reparsed[0]) == c.ep);
  }
}

// A wildcard EE (no entry, no exit, no length constraint) states no predicate at all, and
// FromLegacyEntryExit now answers that the same way FromLegacyRaypath answers an empty
// raypath_text above ("" -> {} case): zero rows, which is "no filter" in the type's own
// vocabulary. It used to produce the single-row match-all shape unconditionally — a legacy .lmc's
// entry_exit filter naming neither face nor length came back as a filter matching everything, and
// under filter_out that excluded every ray.
TEST(FilterSopAc1, FromLegacyEntryExitIsEmptyWhenNothingIsStated) {
  EXPECT_TRUE(FromLegacyEntryExit(Ee("")).empty());
  // A length constraint alone is still a real predicate, not the wildcard shape.
  EXPECT_FALSE(FromLegacyEntryExit(Ee("", "", /*mode=*/1, /*min_len=*/2, /*max_len=*/2)).empty());
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

// SoP equality follows the row list: same rows equal, and adding, removing or retexting a row
// breaks it. One case, because the four are one proposition about the same comparison.
TEST(FilterSopAc2, SopEqualityFollowsTheRowList) {
  FilterConfig a;
  a.SetRaypath(Rp("3-5"));
  FilterConfig b;
  b.SetRaypath(Rp("3-5"));
  EXPECT_TRUE(a == b);

  FilterConfig added = a;
  added.param.push_back(SummandText{ "1-3", { Factor{ Rp("1-3") } } });
  EXPECT_TRUE(a != added);

  FilterConfig removed = added;
  removed.param.pop_back();
  EXPECT_TRUE(removed == a) << "removing the added row must restore equality";
  removed.param.pop_back();
  EXPECT_TRUE(a != removed);

  FilterConfig retexted = a;
  retexted.param[0].text = "3-6";
  EXPECT_TRUE(a != retexted);
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

// A default-constructed FilterConfig states nothing, and "nothing" is the empty SoP rather than a
// row that happens to say everything.
//
// The distinction is the whole point. This used to default to one row holding one empty raypath
// factor, on the reading that "empty raypath ≡ no filter" — true in pre-variant builds, false since
// a factor with empty text became the editor's match-all. Under that default a FilterConfig nobody
// filled in commits as core's `none`, which under filter_out excludes every ray. The compat
// accessors are asserted to agree: a filter that says nothing is not a degenerate single factor, so
// IsRaypath()/IsEntryExit() — questions about WHICH single factor it is — are both false.
TEST(FilterSopModel, DefaultIsEmptySopNoFilter) {
  FilterConfig f;
  EXPECT_TRUE(f.param.empty());
  EXPECT_FALSE(f.IsDegenerateSingleFactor());
  EXPECT_FALSE(f.IsRaypath());
  EXPECT_FALSE(f.IsEntryExit());
  // RaypathText() / DegenerateFactor() are deliberately NOT called: they assert on a non-degenerate
  // SoP, and this state is the reason that assert exists.
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

// The validator over its accept/reject domain, as one table. Every row is the same question — is
// this row text well formed for this crystal kind — and one case per rule was one preamble per rule.
TEST(FilterSopGrammar, ValidateAnswersForEveryWellAndMalformedRowShape) {
  struct Case {
    const char* text;
    LUMICE_CrystalKind kind;
    LUMICE_RaypathValidationState expected;
  };
  const Case kCases[] = {
    // Blank rows are valid and stripped at commit time.
    { "", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    { "   ", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    // Well-formed shapes, including the wildcard entry anchor.
    { "3-5", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    { "3-5 & entry:2", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    { "entry:2 & exit:3", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    { "entry:2 & exit:3 & len:2-4", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    { "entry:", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    { "entry: & len:<=5", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    { "3-5 & entry:2 & exit:4 & len:3", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_VALID },
    // A dangling or doubled '&' has no factor to bind.
    { "3-5 &", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "& 3-5", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "3-5 && 2", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    // "foo:2" is neither an EE token nor a valid raypath token.
    { "foo:2", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    // One EE token of each kind per factor.
    { "entry:2 & entry:3", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "len:3 & len:4", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    // Face 13 is pyramid-only, both bare and inside an entry: facelist; face 51 is on no crystal.
    { "13", LUMICE_CRYSTAL_PYRAMID, LUMICE_RAYPATH_VALID },
    { "13", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "entry:13", LUMICE_CRYSTAL_PYRAMID, LUMICE_RAYPATH_VALID },
    { "entry:13", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "entry:51", LUMICE_CRYSTAL_PYRAMID, LUMICE_RAYPATH_INVALID },
    // Length specs: not a number, zero, an inverted range, a zero upper bound.
    { "len:abc", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "len:0", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "len:5-2", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
    { "len:<=0", LUMICE_CRYSTAL_PRISM, LUMICE_RAYPATH_INVALID },
  };
  for (const Case& c : kCases) {
    EXPECT_EQ(ValidateSummandText(c.text, c.kind).state, c.expected) << "\"" << c.text << "\" kind=" << c.kind;
  }
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

// Contract lock (task-gui-filter-grammar-statemachine-unify): after unifying
// Validate/Parse onto detail::WalkSummandEeFlush, a merge-failed EE token in
// the MIDDLE of an EE run must NOT terminate the run for the parser — the
// skeleton leaves EE state untouched on merge failure, so a subsequent valid
// EE token (`exit:3` here) still accumulates into the same factor as the
// prior valid token (`entry:2`). Validator rejects at the failed token.
// Existing "entry:2 & len:abc" (ParseSkipsAllInvalidEeRun) is a TAIL failure,
// not mid-run; this locks the mid-run case that had no coverage before.
TEST(FilterSopGrammar, MidRunEeMergeSkipDoesNotTerminateFactor) {
  // Validator: rejects at the `len:abc` token.
  auto v = ValidateSummandText("entry:2 & len:abc & exit:3", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(v.state, LUMICE_RAYPATH_INVALID);
  // Parser: skips `len:abc`, accumulates `entry:2` + `exit:3` into ONE factor
  // (not two — mid-run merge failure must not fabricate a factor boundary).
  auto parsed = ParseSummandText("entry:2 & len:abc & exit:3");
  ASSERT_EQ(parsed.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<EntryExitParams>(parsed[0]));
  const auto& ee = std::get<EntryExitParams>(parsed[0]);
  EXPECT_EQ(ee.entry_text, "2");
  EXPECT_EQ(ee.exit_text, "3");
  EXPECT_EQ(ee.length_mode, 0);  // bad len skipped, no length constraint
}

// The length-spec grammar's three accepted shapes and its rejections, as one table. Each row is a
// spelling and the (mode, min, max) triple it must produce; a rejected spelling produces nothing.
TEST(FilterSopGrammar, ParseLengthSpecReadsEveryShapeAndRejectsTheRest) {
  struct Case {
    const char* text;
    bool accepted;
    int mode;  // ignored when !accepted
    int min_len;
    int max_len;
  };
  const Case kCases[] = {
    { "3", true, 1, 3, 3 },    // strict N
    { "<=5", true, 2, 1, 5 },  // at most
    { "2-4", true, 3, 2, 4 },  // range
    { "", false, 0, 0, 0 },    { "0", false, 0, 0, 0 },
    { "abc", false, 0, 0, 0 }, { "5-2", false, 0, 0, 0 },  // min > max
    { "<=0", false, 0, 0, 0 },
  };
  for (const Case& c : kCases) {
    int mode = 0;
    int min_len = 0;
    int max_len = 0;
    EXPECT_EQ(detail::ParseLengthSpec(c.text, mode, min_len, max_len), c.accepted) << c.text;
    if (!c.accepted) {
      continue;
    }
    EXPECT_EQ(mode, c.mode) << c.text;
    EXPECT_EQ(min_len, c.min_len) << c.text;
    EXPECT_EQ(max_len, c.max_len) << c.text;
  }
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

TEST(FilterSopGrammar, ASemicolonSeparatesRaypathAlternativesButNeverStandsAlone) {
  struct Case {
    const char* text;
    LUMICE_RaypathValidationState expected;
  };
  const Case kCases[] = {
    { "1-3;3-5", LUMICE_RAYPATH_VALID },
    { "1-3;3-5 & entry:2", LUMICE_RAYPATH_VALID },
    { "entry:2 & 1-3;3-5", LUMICE_RAYPATH_VALID },
    // Not a legal mixture any more: ',' is retired as a path connector, so the "1,2" segment is
    // a syntax error even though the ';' around it is well-formed.
    { "1,2;3-5 & entry:2", LUMICE_RAYPATH_INVALID },
    { "entry:2 & 1-3;3-5 & exit:4", LUMICE_RAYPATH_VALID },  // sandwiched between EE factors
    // Leading / trailing / consecutive ';' are an empty raypath segment. The last row covers
    // ';'+'&' adjacency, which must not slip through as a valid segment.
    { ";3-5", LUMICE_RAYPATH_INVALID },
    { "3-5;", LUMICE_RAYPATH_INVALID },
    { "3;;5", LUMICE_RAYPATH_INVALID },
    { "3-5; & entry:2", LUMICE_RAYPATH_INVALID },
  };
  for (const Case& c : kCases) {
    EXPECT_EQ(ValidateSummandText(c.text, LUMICE_CRYSTAL_PRISM).state, c.expected) << c.text;
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

// The preview formatter mirrors the syntactic ';' fan-out and nothing else. EE tokens have their
// own comma-list expansion semantics downstream (ExpandSopToClauses in file_io.cpp); re-implementing
// that here would show the user an expansion the commit path does not perform.
TEST(FilterSopPreview, TheRowPreviewExpandsSemicolonAlternativesAndLeavesEverythingElseVerbatim) {
  struct Case {
    const char* name;
    std::vector<Factor> factors;
    const char* expected;
  };
  const std::vector<Case> kCases = {
    { "';' alternatives become an OR group", { Factor{ Rp("1-3;3-5") }, Factor{ Ee("2") } }, "(1-3 OR 3-5) & entry:2" },
    { "a single segment gets no parentheses", { Factor{ Rp("3-5") } }, "3-5" },
    { "an EE comma list is not re-expanded", { Factor{ Ee("1,2") } }, "entry:1,2" },
  };
  for (const Case& c : kCases) {
    EXPECT_EQ(FormatSummandExpansionPreview(c.factors), c.expected) << c.name;
  }
  // With no ';' the preview is byte-identical to the canonical text.
  EXPECT_EQ(FormatSummandExpansionPreview({ Factor{ Rp("3-5") } }), FormatSummandText({ Factor{ Rp("3-5") } }));

  // The whole-SoP overview lists every row, and renders a blank row as the "match-all" sentinel —
  // the same convention the pre-334.3 card tooltip used, so migrating panels.cpp onto this helper
  // is byte-identical for the empty-row case.
  SumOfProducts sop;
  sop.emplace_back(SummandText{ std::string{ "1-3;3-5" }, { Factor{ Rp("1-3;3-5") } } });
  sop.emplace_back(SummandText{ std::string{ "entry:2" }, { Factor{ Ee("2") } } });
  EXPECT_EQ(FormatSopExpansionPreview(sop),
            "OR of 2 row(s):"
            "\n  (1-3 OR 3-5)"
            "\n  entry:2");

  SumOfProducts blank;
  blank.emplace_back(SummandText{ std::string{}, {} });
  EXPECT_EQ(FormatSopExpansionPreview(blank), "OR of 1 row(s):\n  *");
}

// ===========================================================================
// MigrateLegacyRaypathCommaConnector — the load-time rewrite that keeps
// documents written before the ',' retirement meaning what they meant then.
//
// It is tested here rather than next to file_io because it is a pure function
// over one summand row, and because it tokenizes with the same
// detail::SplitSummandTokens / detail::IsEeToken pair the grammar above uses —
// which is the whole reason it can tell a raypath ',' from an EE facelist ','.
// ===========================================================================

TEST(FilterSopGrammar, MigrationRewritesRaypathCommaToDash) {
  // The old parser normalized ',' to '-' silently; the migration does the same rewrite out loud,
  // so the document keeps the four-face path it actually had.
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector("3-5,1-2"), "3-5-1-2");
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector("3,5"), "3-5");
}

TEST(FilterSopGrammar, MigrationLeavesEntryExitFacelistCommasAlone) {
  // An EE facelist ',' is OR syntax and always was — a different language sharing one character.
  // Rewriting it would turn "entry face 1 or 2" into a nonsense token.
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector("entry:1,2"), "entry:1,2");
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector("exit:1,2"), "exit:1,2");
}

TEST(FilterSopGrammar, MigrationSplitsTheTwoCommaMeaningsWithinOneRow) {
  // The crossing case: both kinds of ',' in one row. Only the raypath token's is rewritten.
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector("entry:1,2 & 3-5,1-2"), "entry:1,2 & 3-5-1-2");
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector("3-5,1-2 & exit:1,2"), "3-5-1-2 & exit:1,2");
}

TEST(FilterSopGrammar, MigrationRewritesInsideEverySemicolonAlternative) {
  // ';' alternatives live inside one raypath token, so a character-wise rewrite within the token
  // already covers them — no second split needed.
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector("3,5;1-2"), "3-5;1-2");
}

TEST(FilterSopGrammar, MigrationIsAVerbatimNoOpWithoutCommas) {
  // The common case is a document with no ',' at all, and it must come back byte for byte — not
  // re-joined, re-spaced or trimmed. A migration that reformats every row on every load would
  // make every load look like an edit.
  const std::string kAlreadyCanonical = "3-5 & entry:1,2";
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector(kAlreadyCanonical), kAlreadyCanonical);
  const std::string kOddSpacing = "3-5   &  len:2";
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector(kOddSpacing), kOddSpacing);
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector(""), "");
}

TEST(FilterSopGrammar, MigrationOutputPassesTheTightenedValidator) {
  // The point of the rewrite is that what it produces is accepted by the validator that just
  // stopped accepting the input. If these two ever disagree, an old document loads into a state
  // the editor refuses to commit.
  const std::string migrated = MigrateLegacyRaypathCommaConnector("entry:1,2 & 3-5,1-2");
  EXPECT_EQ(ValidateSummandText(migrated, LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_VALID);
  EXPECT_EQ(ValidateSummandText("entry:1,2 & 3-5,1-2", LUMICE_CRYSTAL_PRISM).state, LUMICE_RAYPATH_INVALID);
}

TEST(FilterSopGrammar, MigrationIsIdempotent) {
  const std::string once = MigrateLegacyRaypathCommaConnector("entry:1,2 & 3,5;1-2");
  EXPECT_EQ(MigrateLegacyRaypathCommaConnector(once), once);
}

}  // namespace
}  // namespace lumice::gui
