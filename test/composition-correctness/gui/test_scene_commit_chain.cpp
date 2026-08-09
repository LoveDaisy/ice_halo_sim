// Composition chain: the document the simulator is actually given.
//
// Units in the chain: file_io × gui_state × raypath_segments × panels (and, as the far side of the
// seam, the core scene the C API builds). panels is here for one reason and it is not scaffolding:
// the entry card's filter summary is the only place the user sees what a filter says without
// opening it, and it is computed from the same sum-of-products the commit path expands.
//
// What the collaboration produces that is observable: the scene handed to the simulator says what
// the document on screen says. BuildScene is the single emitter — the export file and the committed
// run are the same scene under two intents — so everything the user configured has to survive one
// translation, and nothing in the GUI re-checks it afterwards.
//
// When a field does not survive, the run succeeds. The picture is simply of a different
// configuration than the one on screen: a randomized crystal simulated as a fixed one, a filter
// that lets everything through, six faces drawn independently that the user grouped. The user has
// no way to tell from the result, because the result of the wrong configuration is also a plausible
// halo picture.
//
// Derived from the src call graph: app.cpp -> file_io is 20 call sites, the highest-count edge out
// of app.cpp, and the commit path is what most of them reach.

#include <gtest/gtest.h>

#include <cmath>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "gui/raypath_segments.hpp"
#include "lumice.h"
#include "support/scene_json_helpers.hpp"

namespace lumice::gui {
namespace {

using lumice::test::CommitSceneJson;
using lumice::test::CoreJson;
using lumice::test::CountDistinct;
using lumice::test::PrismFacePlaneOffsets;
using lumice::test::SceneJson;

// A one-crystal, one-filter, one-entry document. Every case below needs exactly this and differs
// only in what it puts in the filter or the crystal, so it is built once — the version of these
// cases that spelled it out inline repeated eighteen lines of it a dozen times, which is how the
// six face_distance assignments came to be the most-repeated statement in the suite.
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

FilterConfig SopFilter(const std::vector<std::string>& rows) {
  FilterConfig f;
  SumOfProducts sop;
  for (const std::string& row : rows) {
    sop.push_back(SummandText{ row, ParseSummandText(row) });
  }
  f.param = std::move(sop);
  return f;
}

// ---------------------------------------------------------------------------------------------
// The crystal side: what the user shaped is what gets simulated.

// A configured randomization must arrive at the simulator as a distribution, not as its mean. The
// pre-fix code wrapped every shape scalar in a no-randomization distribution, so a GUI-configured
// spread never reached core at all — and the resulting picture, a sharper halo, looks like a
// perfectly ordinary result.
TEST(SceneCommitChain, ConfiguredRandomizationReachesTheSimulator) {
  SeedOneEntryDocument();
  CrystalConfig& cr = g_state.crystals[0];
  cr.height = ShapeDist{ ShapeDistType::kGauss, 2.5f, 0.4f };
  cr.face_distance[0] = ShapeDist{ ShapeDistType::kUniform, 1.2f, 0.3f };
  cr.face_distance[3] = ShapeDist{ ShapeDistType::kLaplacian, 0.9f, 0.05f };

  const nlohmann::json scene = CommitSceneJson(g_state);
  ASSERT_FALSE(scene.is_null());
  const nlohmann::json& shape = scene["crystal"][0]["shape"];

  EXPECT_EQ(shape["height"]["type"].get<std::string>(), "gauss");
  EXPECT_FLOAT_EQ(shape["height"]["mean"].get<float>(), 2.5f);
  EXPECT_FLOAT_EQ(shape["height"]["std"].get<float>(), 0.4f);
  // Per-face heterogeneity survives too: one family per face, not one family for the crystal.
  EXPECT_EQ(shape["face_distance"][0]["type"].get<std::string>(), "uniform");
  EXPECT_FLOAT_EQ(shape["face_distance"][0]["std"].get<float>(), 0.3f);
  EXPECT_EQ(shape["face_distance"][3]["type"].get<std::string>(), "laplacian");
  EXPECT_FLOAT_EQ(shape["face_distance"][3]["std"].get<float>(), 0.05f);
  // An untouched face stays deterministic, which core spells as a bare number rather than a
  // distribution object. Without this the case would also pass on an emitter that randomized
  // everything.
  ASSERT_TRUE(shape["face_distance"][1].is_number());
  EXPECT_FLOAT_EQ(shape["face_distance"][1].get<float>(), 1.0f);
}

// Grouped scalars share one draw. This is a symmetry the eye cannot verify on the rendered halo, so
// it is asserted on the geometry: a C3 grouping of six randomized faces must give exactly two
// distinct face distances, and it must be faces 0/2/4 that agree rather than some other split that
// happens to produce two values.
TEST(SceneCommitChain, GroupedShapeScalarsShareOneDrawInThePreviewMesh) {
  SeedOneEntryDocument();
  CrystalConfig& cr = g_state.crystals[0];
  for (int i = 0; i < 6; ++i) {
    cr.face_distance[i] = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.1f };
  }

  // Control: six independent draws. Without it, the grouped case below would also pass on a mesh
  // builder that ignored face_distance entirely.
  LUMICE_CrystalMesh independent{};
  ASSERT_TRUE(BuildCrystalMeshData(cr, 12345, &independent));
  EXPECT_EQ(CountDistinct(PrismFacePlaneOffsets(independent), 1e-5f), 6u);

  for (int i = 0; i < 6; ++i) {
    cr.face_distance[i].sync_group = (i % 2 == 0) ? 1 : 2;
  }
  LUMICE_CrystalMesh grouped{};
  ASSERT_TRUE(BuildCrystalMeshData(cr, 12345, &grouped));
  const std::vector<float> off = PrismFacePlaneOffsets(grouped);
  EXPECT_EQ(CountDistinct(off, 1e-5f), 2u);
  EXPECT_NEAR(off[0], off[2], 1e-5f);
  EXPECT_NEAR(off[2], off[4], 1e-5f);
  EXPECT_NEAR(off[1], off[3], 1e-5f);
  EXPECT_NEAR(off[3], off[5], 1e-5f);
  EXPECT_GT(std::fabs(off[0] - off[1]), 1e-5f) << "the two groups collapsed into one";
}

// The preview's re-upload gate must see a grouping change. A hash blind to sync_group leaves the
// previous crystal on screen, which reads as "the feature does not work" rather than as a stale
// cache. Every slot individually, because "which field is invisible to the preview" is not a guess
// worth making.
TEST(SceneCommitChain, EveryGroupableSlotChangesThePreviewParamHash) {
  CrystalConfig base;
  base.type = CrystalType::kPrism;
  for (int i = 0; i < 6; ++i) {
    base.face_distance[i] = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.1f };
  }
  EXPECT_EQ(CrystalParamHash(base), CrystalParamHash(CrystalConfig{ base })) << "the hash is not stable";

  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    CrystalConfig grouped = base;
    ShapeScalarAt(grouped, slot).sync_group = 1;
    EXPECT_NE(CrystalParamHash(base), CrystalParamHash(grouped)) << "slot " << slot << " is invisible to the preview";
  }
}

// A layer probability the GUI warns about is still a value the user typed. The warning is display
// logic; it must not rewrite what reaches the simulator. Both the last layer (the value that trips
// the warning) and an earlier one, so "zeroed everything" and "zeroed only the last" both fail.
TEST(SceneCommitChain, LayerProbabilitiesReachTheSimulatorUnmodified) {
  SeedOneEntryDocument();
  g_state.layers[0].probability = 0.3f;
  Layer last = g_state.layers[0];
  last.probability = 0.45f;  // last layer, non-zero — the value the GUI warns about
  g_state.layers.push_back(last);

  const nlohmann::json scattering = CommitSceneJson(g_state)["scene"]["scattering"];
  ASSERT_EQ(scattering.size(), 2u);
  EXPECT_FLOAT_EQ(scattering[0]["prob"].get<float>(), 0.3f);
  EXPECT_FLOAT_EQ(scattering[1]["prob"].get<float>(), 0.45f) << "the display warning rewrote the committed value";
}

// ---------------------------------------------------------------------------------------------
// The filter side: what the user wrote is what gets applied.

// A filter written as several OR alternatives becomes several simple filters plus one complex that
// composes them, and the entry points at the composed one. Pointing the entry at a child instead
// would silently apply one alternative out of several.
TEST(SceneCommitChain, MultiSegmentRaypathBecomesChildFiltersPlusOneComposition) {
  SeedOneEntryDocument();
  g_state.filters[0].SetRaypath(RaypathParams{ "3-5; 1-3" });

  const nlohmann::json scene = nlohmann::json::parse(CoreJson(g_state));
  const nlohmann::json& filters = scene["filter"];
  ASSERT_EQ(filters.size(), 3u);
  EXPECT_EQ(filters[0]["type"].get<std::string>(), "raypath");
  EXPECT_EQ(filters[0]["raypath"], nlohmann::json({ 3, 5 }));
  EXPECT_EQ(filters[1]["raypath"], nlohmann::json({ 1, 3 }));
  EXPECT_EQ(filters[2]["type"].get<std::string>(), "complex");
  // A single-term clause is emitted as a bare id, so a two-clause OR of singletons is [0,1] rather
  // than [[0],[1]] — the core encoder's form, which the GUI's own emitter once disagreed with.
  EXPECT_EQ(filters[2]["composition"], nlohmann::json({ 0, 1 }));
  EXPECT_EQ(scene["scene"]["scattering"][0]["entries"][0]["filter"].get<int>(), 2);

  // The other direction of the same statement: one alternative must NOT be promoted to a
  // composition. A gratuitous complex filter is not wrong, but it is a different document than the
  // one written.
  SeedOneEntryDocument();
  g_state.filters[0].SetRaypath(RaypathParams{ "3-1-5" });
  const nlohmann::json single = nlohmann::json::parse(CoreJson(g_state))["filter"];
  ASSERT_EQ(single.size(), 1u);
  EXPECT_EQ(single[0]["type"].get<std::string>(), "raypath");
  EXPECT_EQ(single[0]["raypath"].size(), 3u);
}

// An entry/exit filter compared against a hand-written reference object rather than field by field:
// json equality is field-set equality, so this also fails on a key the emitter adds and on one it
// drops, which a list of per-field assertions does not.
TEST(SceneCommitChain, AnEntryExitFilterEmitsExactlyTheDocumentedObject) {
  SeedOneEntryDocument();
  FilterConfig f;
  f.action = 1;     // filter_out
  f.sym_p = true;   // -> "P"
  f.sym_b = false;  // omitted
  f.sym_d = true;   // -> "D"
  f.SetEntryExit(EntryExitParams{ "2", "5" });
  g_state.filters[0] = f;

  const nlohmann::json filters = nlohmann::json::parse(CoreJson(g_state))["filter"];
  ASSERT_EQ(filters.size(), 1u);
  const nlohmann::json expected = {
    { "id", 0 },          { "type", "entry_exit" }, { "action", "filter_out" },
    { "symmetry", "PD" }, { "entry", 2 },           { "exit", 5 },
  };
  EXPECT_EQ(filters[0], expected);
}

// Two ways of writing the same filter — inline ';' alternatives in one row, or one row each — must
// reach the simulator as the same thing. They are separate entry points into the same expansion, so
// nothing but this makes them agree; if they drift, a filter means one thing in the editor and
// another after a save/load that normalizes it.
TEST(SceneCommitChain, InlineAlternativesAndSeparateRowsCommitIdentically) {
  struct Pair {
    const char* name;
    std::vector<std::string> one_row;
    std::vector<std::string> many_rows;
    size_t expected_clauses;
  };
  const Pair kPairs[] = {
    { "plain alternation", { "1-3;3-5" }, { "1-3", "3-5" }, 2 },
    // The distributive case: the alternation has to distribute over its AND partner rather than
    // being applied to only one side.
    { "alternation under an AND", { "1-3;3-5 & entry:2" }, { "1-3 & entry:2", "3-5 & entry:2" }, 2 },
  };

  for (const Pair& p : kPairs) {
    SCOPED_TRACE(p.name);
    SeedOneEntryDocument();
    g_state.filters[0] = SopFilter(p.one_row);
    const nlohmann::json one = nlohmann::json::parse(CoreJson(g_state))["filter"];
    SeedOneEntryDocument();
    g_state.filters[0] = SopFilter(p.many_rows);
    const nlohmann::json many = nlohmann::json::parse(CoreJson(g_state))["filter"];

    EXPECT_EQ(one, many) << "the two spellings commit different filters";
    const nlohmann::json* composed = nullptr;
    for (const nlohmann::json& f : one) {
      if (f.contains("composition")) {
        composed = &f;
      }
    }
    if (composed == nullptr) {
      ADD_FAILURE() << p.name << ": no composition emitted at all";
      continue;  // nothing to compare the clause count against for this row; the rest still get checked
    }
    EXPECT_EQ((*composed)["composition"].size(), p.expected_clauses);
  }
}

// Every filter shape the editor can express must survive core's own reader. The GUI has one emitter,
// so emitter-vs-emitter drift is structurally impossible; what remains checkable is an expansion the
// GUI can build and the core reader cannot faithfully read back — a dropped term, a mis-shaped
// composition, a renumbered id. Re-emitting after a re-parse is what exposes that.
TEST(SceneCommitChain, EveryExpressibleFilterShapeSurvivesCoresOwnReader) {
  struct Shape {
    const char* name;
    FilterConfig (*make)();
  };
  const Shape kShapes[] = {
    { "single segment",
      [] {
        FilterConfig f;
        f.SetRaypath(RaypathParams{ "3-1-5" });
        return f;
      } },
    { "three segments",
      [] {
        FilterConfig f;
        f.SetRaypath(RaypathParams{ "3-5; 1-4; 2-6" });
        return f;
      } },
    { "one entry/exit pair",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "3", "5" });
        return f;
      } },
    { "entry/exit with a length bound",
      [] {
        EntryExitParams ee{ "3,5", "1" };
        ee.length_mode = 3;
        ee.min_len = 2;
        ee.max_len = 6;
        FilterConfig f;
        f.SetEntryExit(ee);
        return f;
      } },
    // The wildcard matrix: an empty side means "any face", which the wire form spells with a
    // sentinel. Both the single- and multi-value sides, because they take different branches.
    { "both sides wildcard",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "", "" });
        return f;
      } },
    { "wildcard entry, one exit",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "", "5" });
        return f;
      } },
    { "one entry, wildcard exit",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "3", "" });
        return f;
      } },
    { "two entries, wildcard exit",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "3,5", "" });
        return f;
      } },
    { "wildcard entry, two exits",
      [] {
        FilterConfig f;
        f.SetEntryExit(EntryExitParams{ "", "3,5" });
        return f;
      } },
    // Sum-of-products shapes: a mixed-type OR, an AND row, and an AND row whose factor carries its
    // own alternatives (so the Cartesian distribution happens inside a clause).
    { "raypath OR entry/exit", [] { return SopFilter({ "3-1-5", "entry:3 & exit:5" }); } },
    { "entry/exit AND raypath", [] { return SopFilter({ "entry:3 & 7-1" }); } },
    { "multi-value AND, plus a plain row", [] { return SopFilter({ "entry:3,4 & 7-1", "2-6" }); } },
  };

  for (const Shape& shape : kShapes) {
    SCOPED_TRACE(shape.name);
    SeedOneEntryDocument();
    g_state.filters[0] = shape.make();

    ScenePtr built = BuildScene(g_state, SceneIntent::kSimCommit);
    if (built == nullptr) {
      ADD_FAILURE() << shape.name << ": the editor built a filter the commit path refuses";
      continue;  // no scene to re-emit for this row; the rest still get checked
    }
    const nlohmann::json emitted = SceneJson(built.get());

    LUMICE_Scene* reparsed_raw = nullptr;
    const std::string text = emitted.dump();
    if (LUMICE_SceneFromJson(text.c_str(), &reparsed_raw) != LUMICE_OK) {
      ADD_FAILURE() << shape.name << ": core cannot read what the GUI wrote";
      continue;  // nothing to re-diff for this row; the rest still get checked
    }
    ScenePtr reparsed(reparsed_raw);
    const nlohmann::json again = SceneJson(reparsed.get());

    EXPECT_EQ(emitted["filter"], again["filter"]);
    // ...and so must the references into it, or the filters survive while nothing points at them.
    EXPECT_EQ(emitted["scene"]["scattering"], again["scene"]["scattering"]);
  }
}

// A filter too complicated for the editor to summarize must still summarize. The summary line on an
// entry card once called a helper that is only defined for the degenerate one-row, one-factor case,
// so writing a second OR row made the card's own text undefined behaviour.
TEST(SceneCommitChain, EveryFilterShapeHasAnEntryCardSummary) {
  const std::vector<std::vector<std::string>> kShapes = {
    { "3-5", "1-3", "2-6" },       // multi-row OR
    { "entry:3 & 7-1" },           // one row, two factors
    { "entry:3,4 & 7-1", "2-6" },  // both at once
  };
  for (const std::vector<std::string>& rows : kShapes) {
    const std::string summary = FilterSummary(std::optional<FilterConfig>{ SopFilter(rows) });
    EXPECT_FALSE(summary.empty()) << "a filter with " << rows.size() << " row(s) has no card summary";
  }
  // And the multi-row case says so, rather than showing the first row as if it were the whole
  // filter — the card is the only place the user sees that more rows exist.
  EXPECT_NE(FilterSummary(std::optional<FilterConfig>{ SopFilter({ "3-5", "1-3", "2-6" }) }).find("(+2 more)"),
            std::string::npos);
}

// The entry-exit length range has four spellings on the card, and each one is a different reader
// of the same two numbers: unconstrained hides them, exact prints one, bounded-above prints the
// bound, and a range prints both. Getting the mode right but the numbers from the wrong field
// produces a summary that is plausible and wrong, which the card gives the user no way to notice.
//
// This arrived from test/gui when the filter editor was rewritten: it had been sitting in a
// gui_test case whose own signature declared it never touched the GUI, and nothing at this layer
// covered the four spellings, so retiring it would have dropped them.
TEST(SceneCommitChain, EveryEntryExitLengthModeHasItsOwnSummarySpelling) {
  auto summary_for = [](int mode, int min_v, int max_v) {
    FilterConfig fc;
    EntryExitParams ee;
    ee.entry_text = "3";
    ee.exit_text = "5";
    ee.length_mode = mode;
    ee.min_len = min_v;
    ee.max_len = max_v;
    fc.SetEntryExit(ee);
    return FilterSummary(std::optional<FilterConfig>{ fc });
  };
  EXPECT_EQ(summary_for(0, 1, 1), "EE:3-5 In PBD");
  EXPECT_EQ(summary_for(1, 2, 2), "EE:3-5 L=2 In PBD");
  EXPECT_EQ(summary_for(2, 1, 4), "EE:3-5 L<=4 In PBD");
  EXPECT_EQ(summary_for(3, 2, 5), "EE:3-5 L=[2,5] In PBD");
}

}  // namespace
}  // namespace lumice::gui
