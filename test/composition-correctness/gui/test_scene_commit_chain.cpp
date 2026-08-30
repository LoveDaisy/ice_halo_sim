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
#include "gui/color_window.hpp"
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
// The two intents: one document, two readers, exactly one field allowed to differ.

// SceneIntent exists to carry a single divergence, and it is worth spelling out because both arms
// look wrong from the other's side. The GUI's Run path must NOT bake the exposure into the
// renderer's intensity_factor — the GUI applies EV at display time, and baking it here multiplies
// it in a second time, which is how a re-run of a coloured composite once came back at 2x. The
// config-export path MUST bake it: the CLI has no display-time exposure and the exported file has
// to reproduce the on-screen brightness on its own.
//
// The third assertion is the one that keeps this from rotting: the two documents are compared field
// by field with that one key removed, so a future intent-dependent branch anywhere else in the
// emitter fails here instead of quietly making the export a different document.
TEST(SceneCommitChain, OnlyTheExposureDiffersBetweenTheRunAndExportIntents) {
  for (const float offset : { 0.0f, 2.5f, -3.0f, 6.0f }) {
    SeedOneEntryDocument();
    g_state.renderer.exposure_offset = offset;

    nlohmann::json commit_doc = CommitSceneJson(g_state);
    if (commit_doc.is_null() || commit_doc["render"].size() != 1u) {
      // Non-fatal: an offset that fails to commit must not take the remaining offsets' reports
      // with it, and "which offsets" is what says whether the break is in the arithmetic or in
      // the commit path.
      ADD_FAILURE() << "offset " << offset << ": the run intent produced no single-renderer scene";
      continue;
    }
    EXPECT_FLOAT_EQ(commit_doc["render"][0]["intensity_factor"].get<float>(), 1.0f)
        << "offset " << offset << ": the run path baked an exposure the display path will apply again";

    // The same guard as the commit arm, and needed for the same reason rather than for symmetry's
    // sake: nlohmann's operator[] auto-vivifies a missing "render" into a null and an out-of-range
    // [0] into another, after which .get<float>() throws json::type_error. An uncaught throw here
    // leaves the loop entirely, so the offsets after this one report nothing at all — which is the
    // half of "an offset that fails must not take the others with it" the commit arm alone cannot
    // deliver. parse() itself is inside the guard's reach for the same reason: it throws on a
    // malformed emitter output, and that is a report, not a crash.
    nlohmann::json export_doc;
    try {
      export_doc = nlohmann::json::parse(CoreJson(g_state));
    } catch (const nlohmann::json::exception& e) {
      ADD_FAILURE() << "offset " << offset << ": the export intent emitted something unparseable: " << e.what();
      continue;
    }
    if (export_doc.is_null() || export_doc["render"].size() != 1u) {
      ADD_FAILURE() << "offset " << offset << ": the export intent produced no single-renderer scene";
      continue;
    }
    EXPECT_FLOAT_EQ(export_doc["render"][0]["intensity_factor"].get<float>(), std::pow(2.0f, offset))
        << "offset " << offset << ": the exported config would open darker than the preview it came from";

    commit_doc["render"][0].erase("intensity_factor");
    export_doc["render"][0].erase("intensity_factor");
    EXPECT_EQ(commit_doc, export_doc) << "offset " << offset << ": the intent reached a second field";
  }
}

// ev_mode reaches the scene on BOTH intents, carrying the document's value rather than a constant.
//
// The case above already forbids an intent split here — it compares the two documents field by
// field with only intensity_factor removed, so an ev_mode that diverged by intent fails there. What
// it cannot see is an ev_mode that never varies at all: a BuildScene that hardcoded "relative"
// would satisfy it perfectly, and every absolute document would silently render relative. So this
// case varies the document and reads the emitted word.
TEST(SceneCommitChain, TheDocumentsEvModeReachesBothIntents) {
  struct Row {
    int value;
    const char* spelling;
  };
  for (const Row& row : { Row{ 0, "relative" }, Row{ 1, "absolute" } }) {
    SeedOneEntryDocument();
    g_state.renderer.ev_mode = row.value;

    // Every failure below is non-fatal + continue, for the reason the intent case above states:
    // one spelling that fails to commit must not suppress the other's report, and "which
    // spelling" is what says whether the break is in the mapping or in the commit path.
    nlohmann::json commit_doc = CommitSceneJson(g_state);
    if (commit_doc.is_null() || commit_doc["render"].size() != 1u) {
      ADD_FAILURE() << row.spelling << ": the run intent produced no single-renderer scene";
      continue;
    }
    EXPECT_EQ(commit_doc["render"][0]["ev_mode"].get<std::string>(), row.spelling)
        << row.spelling << ": the composite preview would anchor differently from the document";

    // Guarded like its sibling above: operator[] auto-vivifies a missing key to null and
    // get<std::string>() then throws, which would leave the loop entirely.
    nlohmann::json export_doc;
    try {
      export_doc = nlohmann::json::parse(CoreJson(g_state));
    } catch (const nlohmann::json::exception& e) {
      ADD_FAILURE() << row.spelling << ": the export intent emitted something unparseable: " << e.what();
      continue;
    }
    if (export_doc.is_null() || export_doc["render"].size() != 1u) {
      ADD_FAILURE() << row.spelling << ": the export intent produced no single-renderer scene";
      continue;
    }
    EXPECT_EQ(export_doc["render"][0]["ev_mode"].get<std::string>(), row.spelling)
        << row.spelling << ": the exported config would open at a different brightness";
  }
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

// A raypath longer than the card can show is truncated at kFilterSummaryBodyChars characters plus
// an ellipsis, and that number is a measurement of the card's value column rather than "whatever
// fits" — see the note on FilterSummary in panels.cpp for what it was measured against. Asserted
// here because the truncation used to be pinned by a gui_test case that was retired with
// test_gui_interaction.cpp, and nothing replaced it — a silently-widened cut would show up as
// overrun text on a card and nowhere else.
//
// The literals below are written out rather than built from the constant: a test that computes its
// expectation the same way the code does agrees with the code by construction, including when both
// are wrong. Widening the column is a deliberate act and updating these two strings is the cost of
// it.
TEST(SceneCommitChain, ALongRaypathIsCutToTheCardsBodyLimit) {
  FilterConfig fc;
  RaypathParams rp;
  rp.raypath_text = "1-2-3-4-5-6-7-8-9-1";  // 19 characters
  fc.SetRaypath(rp);
  const std::string summary = FilterSummary(std::optional<FilterConfig>{ fc });
  EXPECT_EQ(summary.rfind("1-2-3-4-5-6-7-8-...", 0), 0u) << "got '" << summary << "'";

  // ...and one that fits is left alone, so the cut is a bound rather than an unconditional trim.
  // 15 characters: this one WAS truncated under the pre-widening limit, so it also witnesses that
  // the column really did grow rather than the test merely following it.
  RaypathParams shorter;
  shorter.raypath_text = "1-2-3-4-5-6-7-8";  // 15 characters
  fc.SetRaypath(shorter);
  const std::string untouched = FilterSummary(std::optional<FilterConfig>{ fc });
  EXPECT_EQ(untouched.find("..."), std::string::npos) << "got '" << untouched << "'";
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

// ---------------------------------------------------------------------------------------------
// A row that states no predicate states nothing, and nothing is not "everything".
//
// The editor's commit path already reads a blank row that way — it strips blank rows before
// building the filter, and a filter left with no rows is committed as no filter at all. The
// expansion path underneath disagreed: a row that parsed to zero factors became one clause holding
// a single match-all term. The two are not different spellings of the same result. Under
// `filter_in` a match-all clause ORed beside the real ones widens the filter to everything, so the
// user's other rows stop meaning anything; under `filter_out` it excludes every ray and the render
// comes back black. Neither says anything on screen, and both are indistinguishable from "the
// simulation is still warming up" until the user goes looking.
//
// The line these cases draw is between a row the user left empty and a filter a core config file
// writes ON PURPOSE to admit every ray (`"type": "none"`, or no `type` key at all). The two arrive
// as different shapes — a row with no factors at all, versus a row with one factor whose text is
// empty — and only the first is dropped. FilterReconstructChain's match-all case guards the other
// side of that line.
//
// A third shape is neither: `"type": "raypath"` with `"raypath": []`, which core matches NO ray
// with. It was once read here as the deliberate wildcard, which is backwards; it is refused at
// import now, and FilterReconstructChain pins that.

// A blank row between two real ones leaves the two real ones, not three clauses one of which
// matches everything.
TEST(SceneCommitChain, AnEmptyOrRowIsDroppedRatherThanLoweredToMatchAll) {
  SeedOneEntryDocument();
  g_state.filters[0] = SopFilter({ "3-5", "", "1-3" });

  const nlohmann::json scene = CommitSceneJson(g_state);
  ASSERT_FALSE(scene.is_null()) << "the document with a blank row did not commit at all";
  const nlohmann::json& filters = scene["filter"];
  ASSERT_EQ(filters.size(), 3u) << "expected the two written rows plus one composition, got " << filters.dump();
  EXPECT_EQ(filters[0]["raypath"], nlohmann::json({ 3, 5 }));
  EXPECT_EQ(filters[1]["raypath"], nlohmann::json({ 1, 3 }));
  EXPECT_EQ(filters[2]["type"].get<std::string>(), "complex");
  EXPECT_EQ(filters[2]["composition"], nlohmann::json({ 0, 1 }));

  // The claim the clause count alone does not make: nothing in the emitted pool matches every ray.
  // `"type": "none"` is how the commit path spells a match-all term, so that is what a lowered
  // blank row would look like here — and an empty `raypath` array is checked beside it because a
  // term wearing that shape would be the reverse mistake, matching no ray at all.
  for (const nlohmann::json& jf : filters) {
    const std::string type = jf.value("type", std::string{});
    EXPECT_NE(type, std::string("none")) << "a blank row became a match-all term: " << jf.dump();
    if (type == "raypath") {
      EXPECT_FALSE(jf["raypath"].empty()) << "a term was emitted that matches no ray at all: " << jf.dump();
    }
  }
  EXPECT_EQ(scene["scene"]["scattering"][0]["entries"][0]["filter"].get<int>(), 2);
}

// Every row blank means the filter says nothing at all, which is no filter — not a filter that
// happens to admit everything. The two differ once `action` is filter_out, and they differ in the
// document either way, so the assertion is that the entry lands in the SAME observable state as an
// entry that was never given a filter rather than in a new "empty filter" state of its own.
TEST(SceneCommitChain, AFilterWhoseRowsAreAllBlankCommitsAsNoFilter) {
  SeedOneEntryDocument();
  g_state.layers[0].entries[0].filter_id.reset();
  const nlohmann::json without = CommitSceneJson(g_state);
  ASSERT_FALSE(without.is_null()) << "the no-filter baseline did not commit";

  SeedOneEntryDocument();
  g_state.filters[0] = SopFilter({ "", "  " });
  const nlohmann::json blank = CommitSceneJson(g_state);
  ASSERT_FALSE(blank.is_null()) << "the all-blank filter did not commit at all";

  EXPECT_TRUE(blank["filter"].empty()) << "an all-blank filter still emitted filters: " << blank["filter"].dump();
  EXPECT_FALSE(blank["scene"]["scattering"][0]["entries"][0].contains("filter"))
      << "the entry still points at a filter: " << blank["scene"]["scattering"][0]["entries"][0].dump();
  EXPECT_EQ(blank["scene"]["scattering"], without["scene"]["scattering"])
      << "an all-blank filter is a third state, neither a filter nor the absence of one";
}

// The user-visible consequence, pinned as itself rather than as a clause count: a filter_out filter
// carrying one blank row must not hide every ray. Under the old lowering this document committed a
// two-clause OR whose second clause matched everything, and filter_out on "everything" renders a
// black frame with no error anywhere.
TEST(SceneCommitChain, AFilterOutRowLeftBlankDoesNotExcludeEveryRay) {
  SeedOneEntryDocument();
  FilterConfig f = SopFilter({ "3-5", "" });
  f.action = 1;  // filter_out
  g_state.filters[0] = f;

  const nlohmann::json scene = CommitSceneJson(g_state);
  ASSERT_FALSE(scene.is_null()) << "the filter_out document did not commit at all";
  const nlohmann::json& filters = scene["filter"];
  for (const nlohmann::json& jf : filters) {
    EXPECT_NE(jf.value("type", std::string{}), std::string("none"))
        << "filter_out was handed a match-all term — every ray is excluded: " << jf.dump();
  }
  // The one row that says something is the whole filter, so it needs no composition wrapping it.
  ASSERT_EQ(filters.size(), 1u) << "expected the single surviving row, got " << filters.dump();
  EXPECT_EQ(filters[0]["type"].get<std::string>(), "raypath");
  EXPECT_EQ(filters[0]["action"].get<std::string>(), "filter_out");
  EXPECT_EQ(filters[0]["raypath"], nlohmann::json({ 3, 5 }));
}

// The same statement, made about the OTHER way a filter enters the document: off disk.
//
// The three cases above are all about a filter the user built in the editor, where the blank rows
// carry no factor at all and the expansion drops them. A filter read from a `.lmc` reaches the same
// question by a different route and used to answer it differently: when the file said nothing a
// reader could turn into a predicate, the parser wrote a 1-row / 1-factor SoP holding an empty
// raypath — a row that DOES carry a factor, which is the editor's match-all, which under filter_out
// excludes every ray and renders a black frame.
//
// Each row below is a distinct branch of that parser, not a restatement: an empty `summands` array
// (the v3 shape), a legacy filter whose `raypath_text` key is absent, a filter object with nothing
// in it at all, and a `direction` filter — a type the GUI removed, so no reader can produce a
// predicate from it either. All four are the same γ-class question (450.2 adjudication 5: illegal
// input in the GUI's own format, with no core semantics to align to) and take the same answer
// (adjudication 6: drop the predicate, do not pick a value for it).
TEST(SceneCommitChain, AFilterReadFromAFileWithNoPredicateInItCommitsAsNoFilter) {
  struct Case {
    const char* name;
    const char* filter_json;
  };
  const Case kCases[] = {
    { "v3 empty summands array", R"({"summands": [], "action": "filter_out"})" },
    { "legacy filter with no raypath_text", R"({"type": "raypath", "action": "filter_out"})" },
    { "filter object with nothing in it", R"({"action": "filter_out"})" },
    { "removed direction filter", R"({"type": "direction", "action": "filter_out", "az": 30.0, "el": 15.0})" },
    // Same γ-class question as the raypath row above, on the entry_exit arm instead:
    // FromLegacyEntryExit used to build the single-row match-all shape even when
    // entry_text/exit_text/length were all absent.
    { "legacy entry_exit filter naming neither face nor length", R"({"type": "entry_exit", "action": "filter_out"})" },
  };

  // The baseline this is compared against: the same document with no `filter` key on the entry.
  // Asserted as equality with that scene rather than as a list of properties, so "no filter" cannot
  // quietly become a third state that merely looks empty from the angles a property list checks.
  const std::string kDocPrefix =
      R"({"schema_version": 3, "layers": [{"prob": 0.0, "entries": [{
          "crystal": {"type": "prism", "shape": {"height": 1.0, "face_distance": [1,1,1,1,1,1]}},
          "proportion": 100.0)";
  const std::string kDocSuffix = R"(}]}]})";

  GuiState without;
  ASSERT_TRUE(DeserializeGuiStateJson(kDocPrefix + kDocSuffix, without)) << "the no-filter baseline did not load";
  ASSERT_FALSE(without.layers.at(0).entries.at(0).filter_id.has_value());
  const nlohmann::json baseline = CommitSceneJson(without);
  ASSERT_FALSE(baseline.is_null()) << "the no-filter baseline did not commit";

  for (const Case& c : kCases) {
    GuiState loaded;
    const std::string doc = kDocPrefix + R"(, "filter": )" + c.filter_json + kDocSuffix;
    if (!DeserializeGuiStateJson(doc, loaded)) {
      ADD_FAILURE() << c.name << ": the document did not load at all";
      continue;  // nothing to inspect for this case; the rest still get checked
    }
    if (loaded.layers.size() != 1u || loaded.layers.at(0).entries.size() != 1u) {
      ADD_FAILURE() << c.name << ": the document did not load as one layer of one entry";
      continue;  // no entry to inspect for this case; the rest still get checked
    }

    // The document side: the entry points at no filter, and no filter was put in the pool for it.
    EXPECT_FALSE(loaded.layers[0].entries[0].filter_id.has_value())
        << c.name << ": the entry was given a filter the file did not describe";
    EXPECT_TRUE(loaded.filters.empty()) << c.name << ": a filter with no predicate in it entered the pool";

    // The simulator side: the scene is the one the file describes, which is the one with no filter.
    const nlohmann::json scene = CommitSceneJson(loaded);
    if (scene.is_null()) {
      ADD_FAILURE() << c.name << ": the loaded document did not commit at all";
      continue;  // no scene to read for this case; the rest still get checked
    }
    for (const nlohmann::json& jf : scene["filter"]) {
      EXPECT_NE(jf.value("type", std::string{}), std::string("none"))
          << c.name << ": filter_out was handed a match-all term — every ray is excluded: " << jf.dump();
    }
    EXPECT_TRUE(scene["filter"].empty()) << c.name << ": " << scene["filter"].dump();
    EXPECT_EQ(scene["scene"]["scattering"], baseline["scene"]["scattering"])
        << c.name << ": the entry landed in a third state, neither a filter nor the absence of one";
  }
}

// The sentinel for the shape that keeps coming back: a filter in the pool that states nothing, with
// an entry pointing at it.
//
// Every path that builds one has been closed — the editor strips blank rows before committing, and
// the loader now returns no filter rather than an empty one — so this state is not reachable from
// the product today. It is pinned anyway, because "not reachable" is a fact about the current set of
// construction sites and the shape's whole history is of being reconstructed at a new one. What is
// asserted is that the CONSUMERS are safe if it ever is: neither of them may read "says nothing" as
// "matches everything". If a fourth construction site appears, that is a bug about the entry
// pointing at a filter it should not; it must not also be a black frame.
TEST(SceneCommitChain, APoolFilterStatingNothingIsNeverReadAsMatchAll) {
  SeedOneEntryDocument();
  ASSERT_TRUE(g_state.layers.at(0).entries.at(0).filter_id.has_value())
      << "the seeded document has no filter to empty out";
  g_state.filters[0] = FilterConfig{};  // states nothing, and the entry still points at it
  g_state.filters[0].action = 1;        // filter_out — the action under which match-all hides all
  ASSERT_TRUE(g_state.filters[0].param.empty()) << "a default FilterConfig is no longer the empty statement";

  // Consumer 1, the commit path: no term matching every ray reaches the scene, and the entry
  // commits with nothing attached.
  const nlohmann::json scene = CommitSceneJson(g_state);
  ASSERT_FALSE(scene.is_null()) << "the document did not commit at all";
  for (const nlohmann::json& jf : scene["filter"]) {
    EXPECT_NE(jf.value("type", std::string{}), std::string("none"))
        << "filter_out was handed a match-all term — every ray is excluded: " << jf.dump();
  }
  EXPECT_TRUE(scene["filter"].empty()) << scene["filter"].dump();
  EXPECT_FALSE(scene["scene"]["scattering"][0]["entries"][0].contains("filter"))
      << scene["scene"]["scattering"][0]["entries"][0].dump();

  // Consumer 2, the color-class import: the same filter read as color refs must produce no ref
  // rather than one whose match_all is set — that flag is this consumer's spelling of the same
  // "matches everything" the term above would have been.
  int skipped = 0;
  const ColorClassConfig cls = BuildClassFromFilter(0, 0, g_state.filters[0], skipped);
  EXPECT_TRUE(cls.match.empty()) << "a filter that states nothing was imported as " << cls.match.size() << " ref(s)";
  EXPECT_EQ(skipped, 0) << "nothing was there to skip";
}

// ---------------------------------------------------------------------------------------------
// Excluding a crystal is exactly hand-zeroing its weight, and it does not consume the weight.

// The claim the card's participation toggle makes is an equivalence: switching a crystal off and
// re-running gives the run the user would have got by dragging that crystal's Weight to zero
// themselves — the operation it replaces. It is asserted at the seam rather than on pixels because
// that is where it is decidable. BuildScene is the single GUI->core emitter, so two documents that
// emit the same scene are the same run by construction: the engine is deterministic given the same
// input and seed, and comparing two rendered images instead would answer a noisier question at far
// greater cost.
//
// The comparison is the WHOLE scene document, not just the one proportion. `enabled` is a GUI-only
// concept translated at exactly one line, and the failure worth catching is a second translation
// appearing somewhere else — an excluded entry that also drops its filter, renumbers a crystal id,
// or vanishes from the scattering list would still satisfy an assertion written on proportion
// alone, while being a different run.
//
// Both intents are compared for the same reason the exposure case above compares both: the export
// path feeds the CLI, and an exported config that still carries the excluded crystal at full weight
// reproduces a picture the preview never showed.
TEST(SceneCommitChain, ExcludingAnEntryCommitsTheSameSceneAsZeroingItsWeightByHand) {
  // Two entries, not one: with a single entry both arms sum to zero total weight and the scene
  // would compare equal even if the emitter ignored `enabled` and wrote 42 for it. The sibling is
  // what makes the excluded entry's committed number observable.
  const auto seed_two = [](bool use_toggle) {
    SeedOneEntryDocument();
    EntryCard sibling = g_state.layers[0].entries[0];
    sibling.proportion = 58.0f;
    g_state.layers[0].entries.push_back(sibling);
    EntryCard& excluded = g_state.layers[0].entries[0];
    excluded.proportion = use_toggle ? 42.0f : 0.0f;
    excluded.enabled = use_toggle ? false : true;
  };

  seed_two(true);
  const nlohmann::json toggled = CommitSceneJson(g_state);
  const std::string toggled_export = CoreJson(g_state);
  // The weight the user typed is still theirs after the commit: BuildScene takes a const GuiState&
  // and translates on the way out, so turning the crystal back on restores 42 rather than 0. This
  // is the half of the toggle that the equivalence above cannot show — an emitter that zeroed
  // entry.proportion in place would pass every assertion below and still destroy the setting.
  EXPECT_FLOAT_EQ(g_state.layers.at(0).entries.at(0).proportion, 42.0f)
      << "committing consumed the excluded entry's stored weight";
  EXPECT_FALSE(g_state.layers.at(0).entries.at(0).enabled) << "committing flipped the toggle back on";

  seed_two(false);
  const nlohmann::json by_hand = CommitSceneJson(g_state);
  const std::string by_hand_export = CoreJson(g_state);

  ASSERT_FALSE(toggled.is_null()) << "the document with an excluded entry did not commit at all";
  ASSERT_FALSE(by_hand.is_null()) << "the hand-zeroed document did not commit at all";
  EXPECT_FLOAT_EQ(toggled["scene"]["scattering"][0]["entries"][0]["proportion"].get<float>(), 0.0f)
      << "the excluded entry reached the simulator at its stored weight: "
      << toggled["scene"]["scattering"][0]["entries"][0].dump();
  EXPECT_FLOAT_EQ(toggled["scene"]["scattering"][0]["entries"][1]["proportion"].get<float>(), 58.0f)
      << "excluding one entry rewrote its sibling's weight";
  EXPECT_EQ(toggled, by_hand) << "excluding a crystal is not the same run as zeroing its weight";
  EXPECT_EQ(toggled_export, by_hand_export) << "the two are the same run but export as different configs";
}

// A layer with every crystal excluded is a state the GUI has to survive, not one it has to prevent.
//
// The engine already handles it: PartitionCrystalRayNum returns an all-zero allocation for a
// zero total (its AllZeroProportions unit test pins that), and a zero-proportion entry never
// reaches MakeCrystal. So the disposition here is a notice, not a guard — and a notice is only
// honest if the thing it describes actually commits. What this pins is that it does: the commit
// produces a scene rather than a null, with the layer's entries all at zero.
//
// AllEntriesDisabled is checked in the same case because it is the predicate the panel's notice is
// drawn from, and the notice itself is TextColored, which no gui_test assertion can reach. Its
// empty-layer answer is pinned too: an empty layer holds no toggles, so reporting "you turned
// everything off" about it would be a message about something the user never did.
TEST(SceneCommitChain, ALayerWithEveryCrystalExcludedStillCommits) {
  SeedOneEntryDocument();
  EntryCard sibling = g_state.layers[0].entries[0];
  g_state.layers[0].entries.push_back(sibling);
  EXPECT_FALSE(AllEntriesDisabled(g_state.layers[0])) << "a fully participating layer reported as all-excluded";

  for (EntryCard& e : g_state.layers[0].entries) {
    e.enabled = false;
  }
  EXPECT_TRUE(AllEntriesDisabled(g_state.layers[0])) << "every entry is excluded and the predicate disagrees";

  const nlohmann::json scene = CommitSceneJson(g_state);
  ASSERT_FALSE(scene.is_null()) << "a layer with everything excluded refused to commit";
  const nlohmann::json& entries = scene["scene"]["scattering"][0]["entries"];
  ASSERT_EQ(entries.size(), 2u) << "the excluded entries were dropped from the scene rather than zeroed";
  for (const nlohmann::json& je : entries) {
    EXPECT_FLOAT_EQ(je["proportion"].get<float>(), 0.0f) << je.dump();
  }

  Layer empty;
  EXPECT_FALSE(AllEntriesDisabled(empty)) << "a layer with no entries at all was reported as all-excluded";
}

}  // namespace
}  // namespace lumice::gui
