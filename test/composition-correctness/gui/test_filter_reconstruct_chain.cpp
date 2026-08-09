// Composition chain: turning a core filter back into something the editor can show.
//
// Units in the chain: file_io × gui_state × raypath_segments.
//
// What the collaboration produces that is observable: a config file's filters appear in the editor
// as rows the user can read and change. The wire form is a flat pool of simple filters plus a
// composition that references them by id; the editor's form is a sum of products. Rebuilding one
// from the other is the only path by which an imported file becomes editable.
//
// Two ways this goes wrong, and they fail in opposite directions. Rebuild too eagerly and a filter
// the editor cannot actually express is shown as something else — the user then edits and saves a
// document that silently no longer filters what the original did. Refuse too eagerly and a filter
// the editor CAN express is dropped, which turns "only these ray paths" into "everything" with
// nothing on screen saying so. So both the accepted and the refused shapes are pinned here, and the
// refusals are pinned as loud: an unrepresentable filter must leave a warning behind, never a
// quietly missing filter.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <variant>
#include <vector>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "lumice.h"
#include "support/scene_json_helpers.hpp"

namespace lumice::gui {
namespace {

using lumice::test::CoreJson;

// The same one-entry document the commit chain uses, minus the filter, which each case supplies.
void SeedOneEntryDocument() {
  DoNew();
  g_state.crystals.assign(1, CrystalConfig{});
  g_state.crystals[0].type = CrystalType::kPrism;
  g_state.crystals[0].height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    g_state.crystals[0].face_distance[i] = 1.0f;
  }
  g_state.filters.assign(1, FilterConfig{});
  Layer layer;
  layer.probability = 1.0f;
  EntryCard entry;
  entry.crystal_id = 0;
  entry.filter_id = 0;
  entry.proportion = 100.0f;
  layer.entries.assign(1, entry);
  g_state.layers.assign(1, layer);
}

// A hand-authored core document that references filter `main_id` from its only scattering entry.
std::string CoreDocWithFilters(const std::string& filter_array, int main_id) {
  return R"({"crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
             "filter": )" +
         filter_array + R"(,
             "scene": {"light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
                       "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": )" +
         std::to_string(main_id) + R"(}]}]}})";
}

const FilterConfig& LoadedFilter(const GuiState& s) {
  return s.filters.at(*s.layers.at(0).entries.at(0).filter_id);
}

// ---------------------------------------------------------------------------------------------
// Shapes the editor CAN express: emit, read back, and check the rows the user would see.
//
// Every row also re-emits and compares, which is the half that catches a rebuild that "looks right"
// in the editor but no longer means what the file said.

TEST(FilterReconstructChain, ExpressibleCompositionsComeBackAsEditableRows) {
  struct Case {
    const char* name;
    FilterConfig (*make)();
    std::vector<std::string> expected_rows;
  };
  const Case kCases[] = {
    // N alternatives -> N rows. Merging them back into one ';'-joined row would be semantically
    // equal and still wrong: the editor would show a filter the user cannot take apart.
    { "alternatives become one row each",
      [] {
        FilterConfig f;
        f.SetRaypath(RaypathParams{ "3-5;1-3" });
        return f;
      },
      { "3-5", "1-3" } },
    // A multi-value entry/exit pair is a Cartesian product on the wire; each pair is its own row,
    // in the order the emitter wrote them.
    { "an entry/exit product becomes one row per pair",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "3,4", "5,6" });
        return f;
      },
      { "entry:3 & exit:5", "entry:3 & exit:6", "entry:4 & exit:5", "entry:4 & exit:6" } },
    // A wildcard side is written by OMITTING the field, so reading it back exercises the
    // absent-field branch rather than a sentinel value.
    { "a wildcard side comes back as an empty anchor",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "", "5,6" });
        return f;
      },
      { "entry: & exit:5", "entry: & exit:6" } },
  };

  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    SeedOneEntryDocument();
    ClearImportComplexFilterWarning();
    g_state.filters[0] = c.make();

    const std::string emitted = CoreJson(g_state);
    if (emitted.empty()) {
      ADD_FAILURE() << c.name << ": nothing was emitted at all";
      continue;  // no document to read back for this row; the rest still get checked
    }
    GuiState loaded = InitDefaultState();
    if (!DeserializeFromJson(emitted, loaded)) {
      ADD_FAILURE() << c.name << ": the GUI's own emission failed to re-parse";
      continue;
    }
    if (!loaded.layers.at(0).entries.at(0).filter_id.has_value()) {
      ADD_FAILURE() << c.name << ": the filter did not come back at all";
      continue;
    }

    const FilterConfig& f = LoadedFilter(loaded);
    if (f.param.size() != c.expected_rows.size()) {
      ADD_FAILURE() << c.name << ": expected " << c.expected_rows.size() << " rows, got " << f.param.size();
      continue;  // row count is wrong; comparing rows one-by-one below would run out of bounds
    }
    for (size_t i = 0; i < c.expected_rows.size(); ++i) {
      EXPECT_EQ(f.param[i].text, c.expected_rows[i]) << "row " << i;
    }
    EXPECT_TRUE(PeekImportComplexFilterWarning().empty()) << "a representable filter warned anyway";
    EXPECT_EQ(nlohmann::json::parse(emitted)["filter"], nlohmann::json::parse(CoreJson(loaded))["filter"])
        << "the rebuilt filter no longer emits what the file said";
  }
}

// Two composition shapes the editor CAN express, both loaded through the same skeleton. A clause
// holding more than one term is an AND, expressed as one row with several factors — once refused
// outright, and refusing a shape the editor can show is how an imported filter silently becomes no
// filter. An empty raypath is the match-all wildcard and can sit beside a real one as separate
// alternatives — treating the empty array as "no filter" instead would drop the wildcard row and
// narrow the filter to the other alternative.
TEST(FilterReconstructChain, AnAndClauseBecomesOneRowAndAMatchAllAlternativeSurvivesBesideARealOne) {
  DoNew();
  ClearImportComplexFilterWarning();
  GuiState loaded = InitDefaultState();
  ASSERT_TRUE(
      DeserializeFromJson(CoreDocWithFilters(R"([{"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
                             {"id": 2, "type": "raypath", "action": "filter_in", "raypath": [1, 3]},
                             {"id": 3, "type": "complex", "action": "filter_in", "composition": [[1, 2]]}])",
                                             3),
                          loaded));

  ASSERT_TRUE(loaded.layers.at(0).entries.at(0).filter_id.has_value());
  const FilterConfig& f = LoadedFilter(loaded);
  ASSERT_EQ(f.param.size(), 1u) << "an AND clause is one row, not one row per term";
  ASSERT_EQ(f.param[0].factors.size(), 2u);
  EXPECT_TRUE(std::holds_alternative<RaypathParams>(f.param[0].factors[0]));
  EXPECT_TRUE(std::holds_alternative<RaypathParams>(f.param[0].factors[1]));
  EXPECT_EQ(f.param[0].text, std::string("3-5 & 1-3"));
  EXPECT_TRUE(PeekImportComplexFilterWarning().empty());

  // Compared against an explicit expectation rather than the input document: the emitter owns the
  // id space (0-based, so 1..3 renumbers to 0..2) and always writes `symmetry` where the input
  // omitted it. The structural claim — two children ANDed by one composition, in order — is what
  // matters and is asserted directly.
  const nlohmann::json expected = {
    { { "id", 0 }, { "type", "raypath" }, { "action", "filter_in" }, { "symmetry", "" }, { "raypath", { 3, 5 } } },
    { { "id", 1 }, { "type", "raypath" }, { "action", "filter_in" }, { "symmetry", "" }, { "raypath", { 1, 3 } } },
    { { "id", 2 },
      { "type", "complex" },
      { "action", "filter_in" },
      { "symmetry", "" },
      { "composition", { { 0, 1 } } } },
  };
  EXPECT_EQ(nlohmann::json::parse(CoreJson(loaded))["filter"], expected);

  GuiState wildcard = InitDefaultState();
  ASSERT_TRUE(
      DeserializeFromJson(CoreDocWithFilters(R"([{"id": 1, "type": "raypath", "action": "filter_in", "raypath": []},
                             {"id": 2, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
                             {"id": 3, "type": "complex", "action": "filter_in", "composition": [[1], [2]]}])",
                                             3),
                          wildcard));

  ASSERT_TRUE(wildcard.layers.at(0).entries.at(0).filter_id.has_value());
  const FilterConfig& w = LoadedFilter(wildcard);
  ASSERT_EQ(w.param.size(), 2u);
  ASSERT_EQ(w.param[0].factors.size(), 1u);
  ASSERT_TRUE(std::holds_alternative<RaypathParams>(w.param[0].factors[0]));
  EXPECT_TRUE(std::get<RaypathParams>(w.param[0].factors[0]).raypath_text.empty());
  EXPECT_EQ(w.param[1].text, std::string("3-5"));
  EXPECT_TRUE(PeekImportComplexFilterWarning().empty());
}

// ---------------------------------------------------------------------------------------------
// Shapes the editor CANNOT express: refuse, and say so.
//
// The failure guarded against is a SILENT miscull — the entry ends up with no filter, the run
// proceeds, and the picture contains everything the file meant to exclude.

TEST(FilterReconstructChain, UnrepresentableCompositionsAreRefusedLoudly) {
  struct Case {
    const char* name;
    const char* filters;
    int main_id;
  };
  const Case kCases[] = {
    { "a child type the editor has no factor for",
      R"([{"id": 1, "type": "direction", "action": "filter_in", "az": 30.0, "el": 15.0},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]}])",
      2 },
    { "a composition nested inside a composition",
      R"([{"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]},
          {"id": 3, "type": "complex", "action": "filter_in", "composition": [[2]]}])",
      3 },
    { "a term pointing at an id that is not in the pool",
      R"([{"id": 1, "type": "complex", "action": "filter_in", "composition": [[99]]}])", 1 },
    { "a clause with no terms in it",
      R"([{"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[]]}])",
      2 },
    // A child with no `type` at all is malformed, and the tempting reading — "no type, so match
    // everything" — is the worst possible one: it turns a narrow filter into no filter.
    { "a child with no type field",
      R"([{"id": 1, "action": "filter_in", "raypath": [3, 5]},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]}])",
      2 },
  };

  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    DoNew();
    ClearImportComplexFilterWarning();
    GuiState loaded = InitDefaultState();

    if (!DeserializeFromJson(CoreDocWithFilters(c.filters, c.main_id), loaded)) {
      ADD_FAILURE() << c.name
                    << ": the document as a whole must still open — an unreadable filter is not an unreadable file";
      continue;  // nothing loaded for this row; the rest still get checked
    }
    if (loaded.layers.size() != 1u) {
      ADD_FAILURE() << c.name << ": expected exactly one layer, got " << loaded.layers.size();
      continue;
    }
    EXPECT_FALSE(loaded.layers[0].entries.at(0).filter_id.has_value())
        << "an unrepresentable filter was materialized as something else";
    EXPECT_FALSE(PeekImportComplexFilterWarning().empty()) << "the filter vanished with nothing said about it";
    ClearImportComplexFilterWarning();
  }
}

// ---------------------------------------------------------------------------------------------
// Filters too large for the wire format at all.
//
// The ABI bounds the number of clauses and the number of terms per clause. Over either bound there
// is nothing to commit, and the contract is refusal — never a truncated filter, which would run and
// produce a picture of a filter nobody wrote.

TEST(FilterReconstructChain, OverLargeFiltersProduceNoSceneAtAll) {
  struct Case {
    const char* name;
    FilterConfig (*make)();
  };
  const Case kCases[] = {
    { "more OR alternatives than the clause bound",
      [] {
        std::string text;
        for (int i = 0; i < LUMICE_MAX_CONFIG_CLAUSES + 1; ++i) {
          text += (i ? ";" : "") + std::string("3-5");
        }
        FilterConfig f;
        f.SetRaypath(RaypathParams{ text });
        return f;
      } },
    { "more AND factors in one row than the term bound",
      [] {
        std::string text;
        for (int i = 0; i < LUMICE_MAX_CONFIG_TERMS + 1; ++i) {
          text += (i ? " & " : "") + std::to_string(i + 1) + "-" + std::to_string(i + 2);
        }
        FilterConfig f;
        f.param = SumOfProducts{ SummandText{ text, ParseSummandText(text) } };
        return f;
      } },
    // Four factors of nine alternatives each: 9^4 = 6561 clauses, over the bound, while staying
    // well under the per-clause term bound. The detection therefore has to happen on the clause
    // product — and before materializing it, or the check costs an exponential allocation.
    { "a Cartesian product over the clause bound",
      [] {
        const std::string alt = "1;2;3;4;5;6;7;8;3-4";
        const std::string text = alt + " & " + alt + " & " + alt + " & " + alt;
        SummandText row;
        row.text = text;
        row.factors = { Factor{ RaypathParams{ alt } }, Factor{ RaypathParams{ alt } }, Factor{ RaypathParams{ alt } },
                        Factor{ RaypathParams{ alt } } };
        FilterConfig f;
        f.param = SumOfProducts{ row };
        return f;
      } },
  };

  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    SeedOneEntryDocument();
    g_state.filters[0] = c.make();
    g_state.filters[0].name = "OverflowFilter";

    FilterOverflowInfo overflow;
    EXPECT_EQ(BuildScene(g_state, SceneIntent::kSimCommit, &overflow), nullptr)
        << "an over-bounds filter produced a scene, so a truncated config can reach the simulator";
    EXPECT_EQ(overflow.layer_index, 0);
    EXPECT_EQ(overflow.entry_index, 0);
    EXPECT_EQ(overflow.filter_name, std::string("OverflowFilter")) << "the user cannot be told which filter it was";
    // Export goes through the same one emitter, so it refuses too. It used to have a second emitter
    // that degraded an over-bounds filter to a bounded match-all stand-in — a file whose filter
    // meant the opposite of the one on screen.
    EXPECT_TRUE(CoreJson(g_state).empty());

    std::string json;
    std::string warning;
    EXPECT_FALSE(BuildExportJsonOrWarn(g_state, &json, &warning));
    EXPECT_TRUE(json.empty()) << "a refused export still handed back a document to write";
    EXPECT_NE(warning.find("OverflowFilter"), std::string::npos) << "the warning does not name the filter: " << warning;
  }

  // The sanity half: an ordinary filter still exports, so the assertions above are about the bound
  // rather than about export being broken.
  SeedOneEntryDocument();
  g_state.filters[0].SetRaypath(RaypathParams{ "3-5" });
  std::string json;
  std::string warning;
  EXPECT_TRUE(BuildExportJsonOrWarn(g_state, &json, &warning));
  EXPECT_FALSE(json.empty());
  EXPECT_TRUE(warning.empty());
}

// Colour classes have their own two bounds, and the report has to say WHICH one was hit — the
// dialog offers different advice for "too many classes" than for "too many refs in one class".
TEST(FilterReconstructChain, OverLargeColourClassSetsReportWhichBoundWasHit) {
  SeedOneEntryDocument();
  ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = true;

  for (int i = 0; i < LUMICE_MAX_CONFIG_COLOR_CLASSES + 1; ++i) {
    ColorClassConfig cls;
    cls.match.push_back(ref);
    g_state.raypath_color.push_back(cls);
  }
  ColorClassOverflowInfo classes_over;
  EXPECT_EQ(BuildScene(g_state, SceneIntent::kSimCommit, nullptr, &classes_over), nullptr);
  EXPECT_TRUE(classes_over.class_over_cap);
  EXPECT_EQ(classes_over.class_index, LUMICE_MAX_CONFIG_COLOR_CLASSES);

  SeedOneEntryDocument();
  ColorClassConfig fat;
  for (int i = 0; i < LUMICE_MAX_CONFIG_COLOR_REFS + 1; ++i) {
    fat.match.push_back(ref);
  }
  g_state.raypath_color.push_back(fat);
  ColorClassOverflowInfo refs_over;
  EXPECT_EQ(BuildScene(g_state, SceneIntent::kSimCommit, nullptr, &refs_over), nullptr);
  EXPECT_FALSE(refs_over.class_over_cap) << "a ref overflow was reported as a class overflow";
  EXPECT_EQ(refs_over.class_index, 0);
  EXPECT_EQ(refs_over.ref_index, LUMICE_MAX_CONFIG_COLOR_REFS);
}

// The live editor shows a clause count as the user types, and it must be the same arithmetic the
// commit path enforces — a preview that says "fine" about a filter the commit refuses is worse than
// no preview.
TEST(FilterReconstructChain, TheLiveClauseCountIsTheSameArithmeticTheCommitEnforces) {
  FilterConfig one;
  one.SetRaypath(RaypathParams{ "3-5" });
  EXPECT_FALSE(SummarizeSopExpansion(one).overflow);
  EXPECT_EQ(SummarizeSopExpansion(one).clause_count, 1u);

  FilterConfig two;
  two.SetRaypath(RaypathParams{ "3-5;1-2" });
  EXPECT_FALSE(SummarizeSopExpansion(two).overflow);
  EXPECT_EQ(SummarizeSopExpansion(two).clause_count, 2u);

  const std::string alt = "1;2;3;4;5;6;7;8;3-4";
  SummandText row;
  row.text = alt + " & " + alt + " & " + alt + " & " + alt;
  row.factors = { Factor{ RaypathParams{ alt } }, Factor{ RaypathParams{ alt } }, Factor{ RaypathParams{ alt } },
                  Factor{ RaypathParams{ alt } } };
  FilterConfig over;
  over.param = SumOfProducts{ row };
  const auto summary = SummarizeSopExpansion(over);
  EXPECT_TRUE(summary.overflow);
  // On overflow the expansion is replaced by a bounded stand-in, so the reported count is that
  // stand-in's, not the 6561 that was never built.
  EXPECT_EQ(summary.clause_count, 1u);

  // And where the offending filter is, in the words the dialog uses. Indices are 1-based on screen;
  // an unnamed filter drops the name clause instead of showing an empty pair of quotes.
  FilterOverflowInfo named;
  named.layer_index = 2;
  named.entry_index = 0;
  named.filter_name = "MyFilter";
  EXPECT_EQ(FormatOverflowLocator(named), std::string("filter \"MyFilter\", Layer 3 / Entry 1"));

  FilterOverflowInfo unnamed;
  unnamed.layer_index = 0;
  unnamed.entry_index = 4;
  EXPECT_EQ(FormatOverflowLocator(unnamed), std::string("Layer 1 / Entry 5"));
}

// The same overflow is re-detected on every debounced commit while the user keeps typing. Warning
// each time would re-open the modal and freeze the editor; never warning again would hide a
// different filter overflowing later. The gate is "is this the same message as the one in flight",
// which is the predicate the run path itself uses.
TEST(FilterReconstructChain, ARepeatedWarningIsShownOnceUntilItChangesOrClears) {
  ClearGuiWarning();
  EXPECT_TRUE(PeekGuiWarning().empty());

  SetGuiWarning("over-bounds A");
  EXPECT_EQ(PeekGuiWarning(), std::string("over-bounds A"));
  SetGuiWarning("over-bounds A");  // same message on the next debounce -> nothing re-opens
  EXPECT_EQ(PeekGuiWarning(), std::string("over-bounds A"));

  SetGuiWarning("over-bounds B");  // a different filter now overflows -> the user must be told
  EXPECT_EQ(PeekGuiWarning(), std::string("over-bounds B"));

  ClearGuiWarning();  // a commit succeeded -> the gate re-arms
  EXPECT_TRUE(PeekGuiWarning().empty());
  SetGuiWarning("over-bounds A");
  EXPECT_EQ(PeekGuiWarning(), std::string("over-bounds A")) << "after a clear, the earlier message must warn again";
  ClearGuiWarning();
}

}  // namespace
}  // namespace lumice::gui
