// Composition chain: the document round trip.
//
// Units in the chain: file_io × gui_state × raypath_segments.
//
// What the collaboration produces that is observable: a GuiState written to the .lmc JSON and read
// back is the same document. "Same" is decided field by field here, because the failure this layer
// exists to catch is not "loading crashed" — it is one field, out of a few hundred, quietly coming
// back different. A user who hits that saves a file, reopens it, and finds a parameter changed with
// nothing on screen saying so; the natural conclusion is that they misremembered, not that the
// serializer dropped it.
//
// Derived from the src call graph, not from what the old suite happened to cover: file_io.cpp calls
// into gui_state 14 times and into raypath_segments 9 times, the two highest-count edges out of that
// translation unit.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"

namespace lumice::gui {
namespace {

// One field of the document, expressed as: how to set it to a value that is NOT the default, and
// how to read it back as a string. A round trip is correct for this field iff the readout survives.
//
// Readouts are strings rather than typed comparisons on purpose: a failure prints the two values,
// so a red run says "sync_group came back 0 0 0 0 0 0 0 0 0 0, expected 0 0 3 0 0 3 0 0 0 0"
// instead of "false is not true".
struct FieldProbe {
  const char* name;
  void (*mutate)(GuiState&);
  std::string (*read)(const GuiState&);
};

std::string JoinFloats(const float* v, int n) {
  std::string out;
  for (int i = 0; i < n; ++i) {
    out += (i ? " " : "") + std::to_string(v[i]);
  }
  return out;
}

const std::vector<FieldProbe>& FieldProbes() {
  static const std::vector<FieldProbe> kProbes = {
    { "crystal.shape_dist.type_center_spread",
      // kUniform, not kGauss: the loader deliberately downgrades non-uniform families to uniform
      // (the GUI edits uniform only), so a Gauss probe here would be asserting that a documented
      // conversion does not happen. The conversion gets its own case below, where it is the
      // subject rather than an obstacle.
      [](GuiState& s) { s.crystals.at(0).height = ShapeDist{ ShapeDistType::kUniform, 2.5f, 0.75f }; },
      [](const GuiState& s) {
        const ShapeDist& d = s.crystals.at(0).height;
        return std::to_string(static_cast<int>(d.type)) + " " + std::to_string(d.center) + " " +
               std::to_string(d.spread);
      } },
    { "crystal.sync_group",
      [](GuiState& s) {
        // Two scalars sharing a non-zero group id is the whole point of the field: core's
        // PrepareSyncGroups draws once and shares. A round trip that keeps the ids but loses
        // which scalars carry them changes the crystal population without changing any number
        // the user typed.
        //
        // Both slots must exist on THIS crystal's kind. The document is a prism, so the
        // pyramid-only scalars (UPPER_H / PRISM_H / LOWER_H) are not written at all and their
        // group ids legitimately do not come back — measured, after picking PRISM_H here first.
        ShapeScalarAt(s.crystals.at(0), LUMICE_SHAPE_SCALAR_HEIGHT).sync_group = 3;
        ShapeScalarAt(s.crystals.at(0), LUMICE_SHAPE_SCALAR_FACE_0 + 1).sync_group = 3;
      },
      [](const GuiState& s) {
        int groups[LUMICE_SHAPE_SCALAR_COUNT] = {};
        FillSyncGroupArray(s.crystals.at(0), groups);
        std::string out;
        for (int i = 0; i < LUMICE_SHAPE_SCALAR_COUNT; ++i) {
          out += (i ? " " : "") + std::to_string(groups[i]);
        }
        return out;
      } },
    { "filter.raypath_text",
      [](GuiState& s) {
        RaypathParams rp;
        rp.raypath_text = "3-5; 1-3";
        s.filters.at(0).SetRaypath(rp);
      },
      [](const GuiState& s) {
        std::string out;
        for (const SummandText& row : s.filters.at(0).param) {
          out += "[" + row.text + "]";
        }
        return out;
      } },
    { "filter.entry_exit_text",
      [](GuiState& s) {
        EntryExitParams ep;
        ep.entry_text = "3,4";
        ep.exit_text = "1";
        s.filters.at(0).SetEntryExit(ep);
      },
      [](const GuiState& s) {
        std::string out;
        for (const SummandText& row : s.filters.at(0).param) {
          out += "[" + row.text + "]";
        }
        return out;
      } },
    // The two overlay colours below are here because they had NO guard of any kind before this
    // file existed (measured while auditing the suite, not guessed). They are also the cheapest
    // possible demonstration of the key-name half of the contract: a serializer that renames its
    // key still round-trips within one process and only breaks against files already on disk.
    { "renderer.horizon_color",
      [](GuiState& s) {
        s.horizon_color[0] = 0.125f;
        s.horizon_color[1] = 0.25f;
        s.horizon_color[2] = 0.5f;
      },
      [](const GuiState& s) { return JoinFloats(s.horizon_color, 3); } },
    { "renderer.grid_color",
      [](GuiState& s) {
        s.grid_color[0] = 0.5f;
        s.grid_color[1] = 0.25f;
        s.grid_color[2] = 0.125f;
      },
      [](const GuiState& s) { return JoinFloats(s.grid_color, 3); } },
    { "renderer.zenith_nadir_color",
      [](GuiState& s) {
        s.zenith_nadir_color[0] = 0.2f;
        s.zenith_nadir_color[1] = 0.4f;
        s.zenith_nadir_color[2] = 0.6f;
      },
      [](const GuiState& s) { return JoinFloats(s.zenith_nadir_color, 3); } },
    { "overlay.sun_circle_angles",
      // NOT {22, 46}: that is already the default, so a serializer that dropped the key entirely
      // would still round-trip it. The EveryProbeWritesSomethingOtherThanTheDefault case below
      // exists because this exact mistake was made here first and passed.
      [](GuiState& s) { s.sun_circle_angles = { 9.0f, 22.0f, 46.0f }; },
      [](const GuiState& s) {
        std::string out;
        for (float a : s.sun_circle_angles) {
          out += std::to_string(a) + ";";
        }
        return out;
      } },
    { "overlay.alphas",
      [](GuiState& s) {
        s.horizon_alpha = 0.3f;
        s.grid_alpha = 0.4f;
        s.sun_circles_alpha = 0.6f;
      },
      [](const GuiState& s) {
        return std::to_string(s.horizon_alpha) + " " + std::to_string(s.grid_alpha) + " " +
               std::to_string(s.sun_circles_alpha);
      } },
  };
  return kProbes;
}

// A document with one crystal and one filter, REFERENCED by one entry card in one layer.
//
// The reference is not decoration. The pools are append-only within a session and the serializer
// drops pool slots no entry points at, so a crystal added without an entry is written out as
// nothing at all — the first version of this fixture skipped the entry and every crystal probe
// failed by throwing out of `crystals.at(0)` on the way back, which reads as a serializer bug and
// is really a malformed document.
GuiState MinimalDocument() {
  GuiState s;
  s.crystals.assign(1, CrystalConfig{});
  s.filters.assign(1, FilterConfig{});
  EntryCard entry;
  entry.crystal_id = 0;
  entry.filter_id = 0;
  Layer layer;
  layer.entries.push_back(entry);
  s.layers.assign(1, layer);
  return s;
}

// E1 — every probed field survives GuiState -> JSON -> GuiState.
//
// One loop, one EXPECT per field, deliberately not ASSERT: a fatal assertion on the first bad field
// would hide every field after it, and "which fields does the serializer drop" is exactly the
// question this case is asked to answer.
//
// The detection-power half is asserted in the same pass rather than beside it, because a probe that
// cannot fail is not a probe: each probe's mutated readout must differ from the untouched
// document's, which is the property that makes a dropped field observable instead of accidentally
// equal to what the reader defaults to.
TEST(DocumentRoundtripChain, EveryProbedFieldSurvivesJsonRoundTrip) {
  for (const FieldProbe& probe : FieldProbes()) {
    GuiState before = MinimalDocument();
    probe.mutate(before);
    const std::string expected = probe.read(before);
    EXPECT_NE(expected, probe.read(MinimalDocument()))
        << "probe " << probe.name << " writes the default value, so it would pass even if the "
        << "serializer dropped the field entirely";

    GuiState after = MinimalDocument();
    const bool ok = DeserializeGuiStateJson(SerializeGuiStateJson(before), after);
    EXPECT_TRUE(ok) << probe.name << ": DeserializeGuiStateJson rejected its own output";
    if (!ok) {
      continue;
    }
    EXPECT_EQ(probe.read(after), expected) << "field " << probe.name << " did not survive the round trip";
  }
}

// E1/E5 — the loader's deliberate downgrade is announced, not silent.
//
// The GUI edits uniform shape distributions only, so a non-uniform family arriving from a file is
// converted on load. That conversion is a real change to the document the user is about to
// simulate, which is why file_io keeps a counter for it: the notice it drives is the only thing
// standing between "your crystals were simplified" and the user never finding out.
//
// Three properties, all load-bearing: the conversion happens, the counter reports it exactly once
// (it is a Take — reading clears), and a uniform family is NOT counted. A counter that never clears
// makes the notice permanent and trains the user to dismiss it; one that never counts makes the
// change invisible; one wired to "count every crystal" would satisfy the first two and put a
// "distributions were simplified" notice on every file the user opens.
TEST(DocumentRoundtripChain, ANonUniformShapeDistIsDowngradedAndCountedAndAUniformOneIsNot) {
  for (const ShapeDistType family : { ShapeDistType::kGauss, ShapeDistType::kUniform }) {
    const bool expect_downgrade = (family != ShapeDistType::kUniform);
    GuiState before = MinimalDocument();
    before.crystals.at(0).height = ShapeDist{ family, 2.5f, 0.75f };

    TakeShapeDistDowngradeCount();  // discard anything a previous case in this binary left behind
    GuiState after = MinimalDocument();
    ASSERT_TRUE(DeserializeGuiStateJson(SerializeGuiStateJson(before), after));

    EXPECT_EQ(static_cast<int>(after.crystals.at(0).height.type), static_cast<int>(ShapeDistType::kUniform))
        << "a non-uniform family survived into a GuiState the GUI cannot edit";
    // The numbers themselves must not move: it is the family that is simplified, not the value.
    EXPECT_FLOAT_EQ(after.crystals.at(0).height.center, 2.5f);
    EXPECT_EQ(TakeShapeDistDowngradeCount() > 0, expect_downgrade)
        << "the document was changed on load with nothing to report it, or the reverse";
    EXPECT_EQ(TakeShapeDistDowngradeCount(), 0) << "the counter did not clear on read, so a notice would never go away";
  }
}

// E3 — an absent key must leave the documented default in place, and the key's NAME is part of the
// on-disk contract.
//
// Two directions, because they fail differently. Renaming the key breaks reading files already on
// disk while a same-process round trip stays green; dropping the absent-key branch makes a file
// written by an older build come back with a black horizon line instead of the red one.
TEST(DocumentRoundtripChain, AbsentOverlayColorKeysKeepTheDefaultAndTheKeyNamesAreTheContract) {
  const GuiState defaults;

  GuiState loaded = MinimalDocument();
  loaded.horizon_color[0] = 0.0f;
  loaded.grid_color[0] = 0.0f;
  EXPECT_TRUE(DeserializeGuiStateJson("{}", loaded)) << "an empty document is a valid document";
  EXPECT_EQ(JoinFloats(loaded.horizon_color, 3), JoinFloats(defaults.horizon_color, 3));
  EXPECT_EQ(JoinFloats(loaded.grid_color, 3), JoinFloats(defaults.grid_color, 3));

  // The other direction. Spelled out rather than referenced through a constant: the job here is to
  // fail when the key on disk changes, and a shared constant would move with the change.
  GuiState s = MinimalDocument();
  s.horizon_color[0] = 0.125f;
  s.grid_color[0] = 0.375f;
  const std::string json = SerializeGuiStateJson(s);
  EXPECT_NE(json.find("\"overlay_horizon_color\""), std::string::npos);
  EXPECT_NE(json.find("\"overlay_grid_color\""), std::string::npos);
}

// E2 — legacy input translates into the modern sum-of-products, and the text form round-trips.
//
// FromLegacyRaypath / FromLegacyEntryExit are the only path by which a file written before the SoP
// grammar existed becomes a filter the engine runs. If the translation is wrong the file still
// loads, the simulation still runs, and the picture is of a different filter than the one the file
// describes.
struct LegacyRaypathCase {
  const char* name;
  const char* raypath_text;
  size_t expected_rows;
  const char* expected_first_row;
};

TEST(DocumentRoundtripChain, LegacyRaypathSplitsIntoOneRowPerSegment) {
  const LegacyRaypathCase kCases[] = {
    { "single-segment", "3-5", 1, "3-5" },
    { "two-segments", "3-5; 1-3", 2, "3-5" },
    { "three-segments", "3-5;1-3;4-6", 3, "3-5" },
    // The empty case is not "no rows": FilterConfig's own default is a single row holding an
    // empty RaypathParams, and the translator has to produce that same degenerate shape or a
    // filter-less document becomes a document with a zero-row filter, which is not a valid state.
    { "empty-is-one-degenerate-row", "", 1, "" },
  };

  for (const LegacyRaypathCase& c : kCases) {
    RaypathParams rp;
    rp.raypath_text = c.raypath_text;
    const SumOfProducts sop = FromLegacyRaypath(rp);

    EXPECT_EQ(sop.size(), c.expected_rows) << c.name;
    if (sop.empty()) {
      continue;
    }
    EXPECT_EQ(sop.front().text, c.expected_first_row) << c.name;
    for (const SummandText& row : sop) {
      EXPECT_EQ(row.factors.size(), 1u) << c.name << ": a legacy segment is exactly one factor";
    }
    // The same text, read by the multi-segment parser the engine side uses: the segments have to
    // survive there too, or the editor shows rows the run does not apply. (The degenerate empty
    // row has no segment to parse, which is why the count is compared only for real text.)
    const std::vector<std::vector<int>> segments = ParseRaypathTextMultiSegment(c.raypath_text);
    if (*c.raypath_text != '\0') {
      EXPECT_EQ(segments.size(), c.expected_rows) << c.name;
    }
    for (const std::vector<int>& seg : segments) {
      EXPECT_FALSE(seg.empty()) << c.name << ": a segment that parsed to nothing would filter nothing";
    }
  }

  // An entry/exit pair takes the other legacy translator and becomes a single AND-row.
  EntryExitParams ep;
  ep.entry_text = "3,4";
  ep.exit_text = "1";
  const SumOfProducts sop = FromLegacyEntryExit(ep);
  ASSERT_EQ(sop.size(), 1u) << "an entry/exit pair is a single AND-row";
  // The row's text is what the editor shows and what the file stores; the factors are what the
  // engine runs. They have to agree, or the user edits one thing and simulates another.
  EXPECT_EQ(FormatSummandText(ParseSummandText(sop.front().text)), sop.front().text);
}

// E4 — face-number validation is crystal-kind sensitive.
//
// The same face number can be legal on a pyramid and absent from a prism. Accepting a pyramid-only
// face on a prism does not fail loudly: the filter simply never matches, and the user reads that as
// "filters do not work" rather than "that face does not exist here".
TEST(DocumentRoundtripChain, FaceNumberValidityDependsOnCrystalKind) {
  struct Case {
    const char* text;
    LUMICE_CrystalKind kind;
    bool expect_valid;
  };
  const Case kCases[] = {
    { "3", LUMICE_CRYSTAL_PRISM, true },     { "1", LUMICE_CRYSTAL_PRISM, true },
    { "13", LUMICE_CRYSTAL_PYRAMID, true },  { "13", LUMICE_CRYSTAL_PRISM, false },
    { "99", LUMICE_CRYSTAL_PYRAMID, false }, { "abc", LUMICE_CRYSTAL_PYRAMID, false },
  };

  for (const Case& c : kCases) {
    const GuiValidationResult r = GuiValidateFaceNumberText(c.text, c.kind);
    const bool valid = (r.state == LUMICE_RAYPATH_VALID);
    EXPECT_EQ(valid, c.expect_valid) << "face '" << c.text << "' on kind " << static_cast<int>(c.kind) << ": "
                                     << r.message;
    if (!valid) {
      EXPECT_FALSE(r.message.empty()) << "a rejection the user cannot read is a rejection with no cause";
    }
  }
}

}  // namespace
}  // namespace lumice::gui
