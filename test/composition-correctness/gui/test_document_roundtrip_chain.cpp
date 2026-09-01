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

#include <nlohmann/json.hpp>
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
    { "crystal.face_distance",
      // Per face, not one value broadcast: the six faces are independently editable and a writer
      // that emitted the first one six times round-trips perfectly while silently making every
      // crystal regular. Face 2 keeps a centre equal to the 1.0 default with a non-zero spread —
      // the boundary a "did the user change this" check written on the centre alone drops.
      [](GuiState& s) {
        CrystalConfig& cr = s.crystals.at(0);
        for (int i = 0; i < 6; ++i) {
          cr.face_distance[i] = ShapeDist{ ShapeDistType::kUniform, 0.8f + 0.1f * static_cast<float>(i), 0.05f };
        }
        cr.face_distance[2] = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.15f };
      },
      [](const GuiState& s) {
        std::string out;
        for (int i = 0; i < 6; ++i) {
          const ShapeDist& d = s.crystals.at(0).face_distance[i];
          out += (i ? " " : "") + std::to_string(d.center) + "/" + std::to_string(d.spread);
        }
        return out;
      } },
    { "crystal.axis_dists",
      // All three axes, each with its own family and numbers. They share one serializer, so a
      // single axis would seem to be enough; it is not, because the omit-if-default branch is
      // per axis and roll is the one whose default makes it the natural candidate to skip.
      [](GuiState& s) {
        CrystalConfig& cr = s.crystals.at(0);
        cr.zenith = AxisDist{ AxisDistType::kGauss, 25.0f, 3.0f };
        cr.azimuth = AxisDist{ AxisDistType::kUniform, 10.0f, 20.0f };
        cr.roll = AxisDist{ AxisDistType::kLaplacian, 5.0f, 2.0f };
      },
      [](const GuiState& s) {
        const CrystalConfig& cr = s.crystals.at(0);
        const auto one = [](const AxisDist& d) {
          return std::to_string(static_cast<int>(d.type)) + " " + std::to_string(d.mean) + " " + std::to_string(d.std);
        };
        return one(cr.zenith) + " | " + one(cr.azimuth) + " | " + one(cr.roll);
      } },
    { "entry.proportion",
      // How much of the population this entry is. Dropping it does not change what is drawn, only
      // how much of it, which is why nothing on screen says the file came back different.
      [](GuiState& s) { s.layers.at(0).entries.at(0).proportion = 75.0f; },
      [](const GuiState& s) { return std::to_string(s.layers.at(0).entries.at(0).proportion); } },
    { "entry.enabled",
      // Whether this entry takes part in the run at all. It is probed alongside a proportion that
      // is emphatically not zero, because the failure mode is a writer that decides `enabled` is
      // derivable from the weight: dropping the field then reloads an excluded crystal as a
      // participating one at full weight, and the run comes back with the crystal the user
      // switched off. The weight itself must survive the trip untouched — that is what makes the
      // exclusion reversible.
      [](GuiState& s) {
        s.layers.at(0).entries.at(0).enabled = false;
        s.layers.at(0).entries.at(0).proportion = 42.0f;
      },
      [](const GuiState& s) {
        const EntryCard& e = s.layers.at(0).entries.at(0);
        return std::string(e.enabled ? "on" : "off") + " @" + std::to_string(e.proportion);
      } },
    { "sun.custom_spectrum",
      // The discrete spectrum is a list, not a name, and it is the one light-source field whose
      // absence leaves a perfectly valid document that simulates a different colour of sunlight.
      [](GuiState& s) {
        s.sun.spectrum_index = kCustomSpectrumIndex;
        s.sun.custom_spectrum = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };
      },
      [](const GuiState& s) {
        std::string out = std::to_string(s.sun.spectrum_index) + ":";
        for (const auto& p : s.sun.custom_spectrum) {
          out += std::to_string(p.wavelength) + "/" + std::to_string(p.weight) + ";";
        }
        return out;
      } },
    { "view.modal_layout_vertical",
      // A view preference rather than a simulation input, and stored in the document alongside
      // them. It is here because "this field is only a preference" is exactly the reasoning under
      // which a serializer stops writing one.
      [](GuiState& s) { s.modal_layout_vertical = !GuiState{}.modal_layout_vertical; },
      [](const GuiState& s) { return std::to_string(static_cast<int>(s.modal_layout_vertical)); } },
    { "view.background",
      // All three background fields in one probe because they are one setting to the user: an
      // image, whether it is showing, and how far it is faded. Split across three probes, a
      // serializer that wrote the path and dropped the other two would report two red fields for
      // one defect; kept together, the readout says which of the three came back wrong.
      //
      // The path is a std::filesystem::path and goes through PathToU8/PathFromU8 on the way out
      // and back, which is the one field here whose round trip is more than a JSON scalar copy.
      [](GuiState& s) {
        s.bg_path = "/some/test/path.png";
        s.bg_show = true;
        s.bg_alpha = 0.7f;
        s.bg_offset_x = -0.35f;
        s.bg_offset_y = 0.2f;
        s.bg_scale = 1.75f;
      },
      [](const GuiState& s) {
        return s.bg_path.string() + " " + std::to_string(static_cast<int>(s.bg_show)) + " " +
               std::to_string(s.bg_alpha) + " " + std::to_string(s.bg_offset_x) + " " + std::to_string(s.bg_offset_y) +
               " " + std::to_string(s.bg_scale);
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
    { "filter.mixed_sum_of_products",
      // The two probes above each build their rows through a typed setter, so both produce rows of
      // one kind. A filter authored in the editor can hold rows of BOTH kinds side by side, plus an
      // AND clause inside one of them — the shape whose reader branch is separate from either
      // single-kind path.
      [](GuiState& s) {
        SumOfProducts sop;
        for (const char* row : { "entry:3,4 & 7-1", "2-6" }) {
          sop.push_back(SummandText{ row, ParseSummandText(row) });
        }
        s.filters.at(0).param = std::move(sop);
      },
      [](const GuiState& s) {
        std::string out;
        for (const SummandText& row : s.filters.at(0).param) {
          out += "[" + row.text + "]";
        }
        return out;
      } },
    { "renderer.ev_mode",
      // 1 (absolute), not 0: 0 is the default, so a serializer that dropped the key entirely
      // would round-trip it. Held as an int in the struct but written as a WORD on disk, which
      // is the half the round trip alone cannot see — the spelling case below covers that.
      [](GuiState& s) { s.renderer.ev_mode = 1; },
      [](const GuiState& s) { return std::to_string(s.renderer.ev_mode); } },
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
    { "renderer.lens_border_color",
      [](GuiState& s) {
        s.lens_border_color[0] = 0.9f;
        s.lens_border_color[1] = 0.1f;
        s.lens_border_color[2] = 0.5f;
      },
      [](const GuiState& s) { return JoinFloats(s.lens_border_color, 3); } },
    // No editor drives this field anymore (field_editor_registry.cpp registers no row for it), so
    // this probe is now the ONLY thing exercising its round trip: a document saved while it still
    // had a control must keep opening, showing, and re-saving the value unchanged.
    { "renderer.ray_color",
      [](GuiState& s) {
        s.renderer.ray_color[0] = 0.2f;
        s.renderer.ray_color[1] = 0.5f;
        s.renderer.ray_color[2] = 0.9f;
      },
      [](const GuiState& s) { return JoinFloats(s.renderer.ray_color, 3); } },
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
    // The GUI had no control for this field until the crystal edit modal grew a Name box, so
    // although both halves of its serialization have existed since the format did, nothing had
    // ever put a name into a document and read one back. The name is also what the Colours window
    // shows for a crystal, so a dropped one turns every colour class's subject back into a bare
    // pool id.
    { "crystal.name", [](GuiState& s) { s.crystals[0].name = "plate"; },
      [](const GuiState& s) { return s.crystals.empty() ? std::string("<no crystal>") : s.crystals[0].name; } },
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

// E1 — the aspect preset is spelled on disk the way files already on disk spell it, for every
// preset and both orientations.
//
// Not a row in the probe table above, and deliberately not a round trip either. The preset is
// stored as a STRING (kAspectPresetJsonNames in file_io.cpp) beside an enum, and the writer and
// the reader walk the SAME table — so a table shifted by one agrees with itself, and every preset
// round-trips perfectly within one process. That is measured rather than reasoned: the round-trip
// version of this case was written first, and shifting kAspectPresetJsonNames by one row left it
// green.
//
// What a shift does break is every file already on disk, which is why the expected spellings below
// are literals rather than reads of the table under test. Both directions are needed: the write
// side pins what lands in the file, and the read side pins what a file carrying that spelling comes
// back as — separately, because AspectPresetFromString answers an unrecognised name with kFree
// rather than an error, so a shifted table turns some saved documents into their neighbour and the
// rest into "Free", with nothing on screen saying so.
//
// aspect_portrait rides along in the same loop rather than in a second case: it is a plain bool
// with no table to misalign, and the pair is what ApplyAspectRatio consumes.
TEST(DocumentRoundtripChain, EachAspectPresetKeepsItsOnDiskSpellingInBothOrientations) {
  struct Row {
    AspectPreset preset;
    const char* json_name;
  };
  const Row kRows[] = {
    { AspectPreset::kFree, "free" },
    { AspectPreset::k16x9, "16:9" },
    { AspectPreset::k3x2, "3:2" },
    { AspectPreset::k4x3, "4:3" },
    { AspectPreset::k1x1, "1:1" },
    { AspectPreset::k2x1, "2:1" },
    { AspectPreset::kMatchBg, "match_background" },
  };
  static_assert(sizeof(kRows) / sizeof(kRows[0]) == kAspectPresetCount,
                "every AspectPreset needs a row: a preset with no expected spelling is untested");

  for (const Row& row : kRows) {
    const char* label = kAspectPresetNames[static_cast<int>(row.preset)];
    for (bool portrait : { false, true }) {
      // EXPECT, not ASSERT: a fatal assert here would return out of the whole loop and hide every
      // remaining preset, which is precisely the question this case is asked.
      GuiState before = MinimalDocument();
      before.aspect_preset = row.preset;
      before.aspect_portrait = portrait;
      const nlohmann::json written = nlohmann::json::parse(SerializeGuiStateJson(before));
      EXPECT_EQ(written.value("aspect_ratio", std::string{}), row.json_name)
          << label << " was written to disk under the wrong name";
      EXPECT_EQ(written.value("aspect_portrait", !portrait), portrait) << label << " orientation, write side";

      // Read side: a document carrying this literal spelling, and nothing else changed.
      nlohmann::json on_disk = nlohmann::json::parse(SerializeGuiStateJson(MinimalDocument()));
      on_disk["aspect_ratio"] = row.json_name;
      on_disk["aspect_portrait"] = portrait;
      GuiState after = MinimalDocument();
      EXPECT_TRUE(DeserializeGuiStateJson(on_disk.dump(), after)) << row.json_name;
      EXPECT_EQ(static_cast<int>(after.aspect_preset), static_cast<int>(row.preset))
          << "a file saying \"" << row.json_name << "\" came back as "
          << kAspectPresetNames[static_cast<int>(after.aspect_preset)];
      EXPECT_EQ(after.aspect_portrait, portrait) << row.json_name << " orientation, read side";
    }
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
  // Every family a file can carry, not just one of them: the GUI edits uniform only, and each
  // non-uniform family is a separate arm of the reader's conversion. kNoRandom is here for the
  // same reason kUniform is — it must pass through untouched and uncounted, or every file with an
  // ordinary crystal in it opens under a "your distributions were simplified" notice.
  for (const ShapeDistType family :
       { ShapeDistType::kGauss, ShapeDistType::kZigzag, ShapeDistType::kLaplacian, ShapeDistType::kGaussLegacy,
         ShapeDistType::kUniform, ShapeDistType::kNoRandom }) {
    const bool editable_here = family == ShapeDistType::kUniform || family == ShapeDistType::kNoRandom;
    const bool expect_downgrade = !editable_here;
    GuiState before = MinimalDocument();
    before.crystals.at(0).height = ShapeDist{ family, 2.5f, 0.75f };

    TakeShapeDistDowngradeCount();  // discard anything a previous case in this binary left behind
    GuiState after = MinimalDocument();
    if (!DeserializeGuiStateJson(SerializeGuiStateJson(before), after)) {
      ADD_FAILURE() << "family " << static_cast<int>(family) << ": round trip failed to deserialize";
      continue;  // no loaded document to inspect for this family; the rest still get checked
    }

    const ShapeDistType expected_type = editable_here ? family : ShapeDistType::kUniform;
    EXPECT_EQ(static_cast<int>(after.crystals.at(0).height.type), static_cast<int>(expected_type))
        << "family " << static_cast<int>(family)
        << ": a family the GUI cannot edit survived, or one it can was converted anyway";
    // The numbers themselves must not move: it is the family that is simplified, not the value.
    EXPECT_FLOAT_EQ(after.crystals.at(0).height.center, 2.5f);
    EXPECT_EQ(TakeShapeDistDowngradeCount() > 0, expect_downgrade)
        << "the document was changed on load with nothing to report it, or the reverse";
    EXPECT_EQ(TakeShapeDistDowngradeCount(), 0) << "the counter did not clear on read, so a notice would never go away";
  }
}

// The second loader downgrade that is announced rather than silent: a filter the file described no
// rule for is loaded as no filter at all.
//
// Dropping it is the right disposition — the alternative, a filter that matches everything, hides
// every ray under filter_out — but it is still a document arriving different from how it was
// written, and the user's own copy of the file still says otherwise. The counter is what turns that
// into a notice. Its three properties are the same three the shape-distribution counter has, and
// they fail in the same three ways: never counting makes the change invisible, never clearing makes
// the notice permanent, and counting documents that were loaded faithfully trains the user to
// dismiss it.
TEST(DocumentRoundtripChain, AFilterWithNoRuleInItIsDroppedAndCounted) {
  struct Case {
    const char* name;
    const char* doc;
    int expected_count;
  };
  const char* kPrism = R"("crystal": {"type": "prism", "shape": {"height": 1.0}}, "proportion": 100.0)";
  const std::string kEmptySummands =
      std::string(R"({"layers": [{"prob": 0.0, "entries": [{)") + kPrism + R"(, "filter": {"summands": []}}]}]})";
  const std::string kNoRaypathText =
      std::string(R"({"layers": [{"prob": 0.0, "entries": [{)") + kPrism + R"(, "filter": {"type": "raypath"}}]}]})";
  const std::string kTwoOfThem = std::string(R"({"layers": [{"prob": 0.0, "entries": [{)") + kPrism +
                                 R"(, "filter": {"summands": []}}, {)" + kPrism +
                                 R"(, "filter": {"type": "raypath"}}]}]})";
  const std::string kRealFilter =
      std::string(R"({"layers": [{"prob": 0.0, "entries": [{)") + kPrism + R"(, "filter": {"summands": ["3-5"]}}]}]})";
  const std::string kNoFilterKey = std::string(R"({"layers": [{"prob": 0.0, "entries": [{)") + kPrism + R"(}]}]})";

  const Case kCases[] = {
    { "an empty summands array", kEmptySummands.c_str(), 1 },
    { "a legacy form with no raypath text", kNoRaypathText.c_str(), 1 },
    // Per filter, not per document: a file with two of them has to say so, or the notice
    // under-reports exactly when there is most to report.
    { "two of them in one document", kTwoOfThem.c_str(), 2 },
    // The other half of the claim: a document that was loaded faithfully is not counted.
    { "a filter that does state a rule", kRealFilter.c_str(), 0 },
    { "an entry with no filter at all", kNoFilterKey.c_str(), 0 },
  };

  for (const Case& c : kCases) {
    TakeFilterNoPredicateDowngradeCount();  // discard anything a previous case in this binary left
    GuiState loaded;
    if (!DeserializeGuiStateJson(c.doc, loaded)) {
      ADD_FAILURE() << c.name << ": the document failed to deserialize";
      continue;  // no load to count; the rest still get checked
    }
    EXPECT_EQ(TakeFilterNoPredicateDowngradeCount(), c.expected_count)
        << c.name << ": the document was changed on load with nothing to report it, or the reverse";
    EXPECT_EQ(TakeFilterNoPredicateDowngradeCount(), 0)
        << c.name << ": the counter did not clear on read, so a notice would never go away";
  }
}

// E1 — a document written before the participation toggle existed loads as participating, on both
// read paths.
//
// The probe sweep above cannot reach this. It writes with SerializeGuiStateJson and reads back with
// DeserializeGuiStateJson, so both halves are the current version by construction, and the key is
// always present. What ships is the other direction: every .lmc already on a user's disk is missing
// this key, and the reader's answer to a missing key decides whether those documents come back
// intact or with crystals silently switched off. `false` is the value a default-initialised bool
// would have had, which is why the absent-key answer is worth pinning rather than assuming.
//
// Both branches of DeserializeGuiStateJson are exercised, not just the current one. The reader
// picks its branch on the document's shape — "layers" is the v2 inline form, "crystals" +
// "scattering" the legacy v1 pool form — and the two parse entries with separate code that shares
// no helper for this field. A patch applied to one of them alone is green on every test that only
// feeds the other, and the documents that would lose their toggle state are precisely the older
// ones, i.e. the ones the compatibility default exists for. Present-key cases are here too: an
// absent-key default of `true` is also what a reader that ignores the key entirely produces, so
// without them the two branches would pass while never reading the field at all.
TEST(DocumentRoundtripChain, AnEntryWithNoEnabledKeyLoadsAsParticipatingOnBothReadPaths) {
  struct Case {
    const char* name;
    std::string doc;
    bool expect_enabled;
    float expect_proportion;
  };

  const char* kPrism = R"("crystal": {"type": "prism", "shape": {"height": 1.0}})";
  const auto v2 = [&](const char* entry_tail) {
    return std::string(R"({"layers": [{"prob": 0.0, "entries": [{)") + kPrism + entry_tail + R"(}]}]})";
  };
  const auto v1 = [](const char* entry_body) {
    return std::string(R"({"crystals": [], "scattering": [{"prob": 0.0, "entries": [{)") + entry_body + R"(}]}]})";
  };

  const Case kCases[] = {
    { "v2 inline, no enabled key", v2(R"(, "proportion": 42.0)"), true, 42.0f },
    { "v2 inline, enabled false", v2(R"(, "proportion": 42.0, "enabled": false)"), false, 42.0f },
    { "v2 inline, enabled true", v2(R"(, "proportion": 42.0, "enabled": true)"), true, 42.0f },
    { "legacy v1 pool, no enabled key", v1(R"("crystal_id": 0, "proportion": 42.0)"), true, 42.0f },
    { "legacy v1 pool, enabled false", v1(R"("crystal_id": 0, "proportion": 42.0, "enabled": false)"), false, 42.0f },
    { "legacy v1 pool, enabled true", v1(R"("crystal_id": 0, "proportion": 42.0, "enabled": true)"), true, 42.0f },
  };

  for (const Case& c : kCases) {
    GuiState loaded;
    if (!DeserializeGuiStateJson(c.doc, loaded)) {
      ADD_FAILURE() << c.name << ": the document failed to deserialize";
      continue;  // nothing loaded to inspect; the remaining cases still get checked
    }
    if (loaded.layers.size() != 1 || loaded.layers.at(0).entries.size() != 1) {
      ADD_FAILURE() << c.name << ": expected exactly one layer with one entry";
      continue;
    }
    const EntryCard& e = loaded.layers.at(0).entries.at(0);
    EXPECT_EQ(e.enabled, c.expect_enabled) << c.name << ": the entry came back with the wrong participation state";
    // The weight rides along in every case: a reader that decided the two fields were one thing
    // would satisfy the enabled check above and still lose the number the exclusion has to
    // preserve.
    EXPECT_FLOAT_EQ(e.proportion, c.expect_proportion) << c.name << ": the stored weight did not survive the load";
  }
}

// E1 — sync_group on the crystal kind whose slots the probe sweep structurally cannot reach, with
// the emitted KEY NAMES asserted rather than only the round trip.
//
// Why this is not covered by the "crystal.sync_group" probe above, and cannot be. That probe runs
// on MinimalDocument(), whose single crystal is a prism, and the writer is type-gated exactly as
// the scalars are: a prism emits height + face_distance and never writes prism_h / upper_h /
// lower_h at all, so the three pyramid-only slots are absent from that document by construction.
// Adding them to the probe would assert that an inapplicable slot round-trips, which is a claim
// about nothing.
//
// The trap this pins is positional rather than arithmetic. LUMICE_SHAPE_SCALAR_UPPER_H is slot 1
// and PRISM_H is slot 2 — the reverse of the order the fields appear in CrystalConfig — so a
// mapping written from the struct's field order instead of from ShapeScalarAt swaps exactly these
// two and nothing else. Distinct group ids per slot are what make that swap visible; equal ones
// would round-trip perfectly through the bug.
//
// And the key names are asserted against the emitted document, not inferred from the round trip
// being stable. A round trip is self-consistent even when the GUI writes keys core cannot read:
// the GUI writer, the core reader and the C API each carry their own spelling of these names with
// no compile-time link between them, and drift in that spelling is how a hand-authored config
// silently loses its grouping (the failure mode that produced the "shape/axis key names converge
// on core's single source" work). Reading tests cover core's side; this covers what the GUI emits.
TEST(DocumentRoundtripChain, PyramidSyncGroupSlotsSurviveTheDocumentAndKeepTheirKeyNames) {
  // The parallel-array view of one crystal's ten slots, as a string, so a slot that came back under
  // the wrong index is reported as the whole array rather than as one field.
  const auto slots = [](const CrystalConfig& c) {
    int groups[LUMICE_SHAPE_SCALAR_COUNT] = {};
    FillSyncGroupArray(c, groups);
    std::string out;
    for (int i = 0; i < LUMICE_SHAPE_SCALAR_COUNT; ++i) {
      out += (i ? " " : "") + std::to_string(groups[i]);
    }
    return out;
  };

  GuiState before = MinimalDocument();
  {
    // Scoped, and the expectations below are read back out of `before` by index rather than held as
    // references: the second crystal is push_back'd into the same vector, which is free to
    // reallocate and leave any reference taken here dangling.
    CrystalConfig& prism = before.crystals.at(0);
    prism.type = CrystalType::kPrism;
    ShapeScalarAt(prism, LUMICE_SHAPE_SCALAR_HEIGHT).sync_group = 5;
    for (int i = 0; i < 6; ++i) {
      ShapeScalarAt(prism, LUMICE_SHAPE_SCALAR_FACE_0 + i).sync_group = (i % 2 == 0) ? 1 : 2;
    }
  }

  // A second crystal rather than a second document: the writer walks the pool, and one entry per
  // kind is what makes "the gate is per crystal" observable instead of "the gate is per file".
  CrystalConfig pyramid;
  pyramid.type = CrystalType::kPyramid;
  ShapeScalarAt(pyramid, LUMICE_SHAPE_SCALAR_PRISM_H).sync_group = 3;
  ShapeScalarAt(pyramid, LUMICE_SHAPE_SCALAR_UPPER_H).sync_group = 4;
  ShapeScalarAt(pyramid, LUMICE_SHAPE_SCALAR_LOWER_H).sync_group = 3;
  for (int i = 0; i < 6; ++i) {
    ShapeScalarAt(pyramid, LUMICE_SHAPE_SCALAR_FACE_0 + i).sync_group = 6 - i;
  }
  EntryCard extra;
  extra.crystal_id = static_cast<int>(before.crystals.size());
  extra.filter_id = 0;
  before.crystals.push_back(pyramid);
  before.layers.at(0).entries.push_back(extra);

  const std::string json = SerializeGuiStateJson(before);
  nlohmann::json doc;
  try {
    doc = nlohmann::json::parse(json);
  } catch (const nlohmann::json::exception& e) {
    ADD_FAILURE() << "the emitter produced something unparseable: " << e.what();
    return;
  }

  // Presence before value, in both arms: without it a writer that emits nothing fails by throwing
  // out of operator[] instead of naming the expectation that broke.
  const nlohmann::json& entries = doc["layers"][0]["entries"];
  ASSERT_EQ(entries.size(), 2u) << "the two-crystal document did not survive as two entries";
  ASSERT_TRUE(entries[0]["crystal"]["shape"].contains("sync_group"));
  ASSERT_TRUE(entries[1]["crystal"]["shape"].contains("sync_group"));

  const nlohmann::json& prism_sg = entries[0]["crystal"]["shape"]["sync_group"];
  EXPECT_EQ(prism_sg["height"].get<int>(), 5);
  EXPECT_EQ(prism_sg["face_distance"][2].get<int>(), 1);
  EXPECT_FALSE(prism_sg.contains("prism_h")) << "a slot this crystal kind does not have was written anyway";

  const nlohmann::json& pyr_sg = entries[1]["crystal"]["shape"]["sync_group"];
  EXPECT_EQ(pyr_sg["prism_h"].get<int>(), 3);
  EXPECT_EQ(pyr_sg["upper_h"].get<int>(), 4) << "upper_h and prism_h look swapped — a positional mapping";
  EXPECT_EQ(pyr_sg["lower_h"].get<int>(), 3);
  EXPECT_EQ(pyr_sg["face_distance"][0].get<int>(), 6);
  EXPECT_FALSE(pyr_sg.contains("height")) << "a slot this crystal kind does not have was written anyway";

  // ...and back.
  const std::string expected_prism = slots(before.crystals.at(0));
  const std::string expected_pyramid = slots(before.crystals.at(1));
  // Non-vacuous in the way that matters here: the two kinds must not produce the same array, or a
  // writer that emitted one crystal's groups for both would satisfy the pair.
  EXPECT_NE(expected_prism, expected_pyramid);

  GuiState after = MinimalDocument();
  ASSERT_TRUE(DeserializeGuiStateJson(json, after)) << "DeserializeGuiStateJson rejected its own output";
  ASSERT_EQ(after.layers.at(0).entries.size(), 2u);
  EXPECT_EQ(slots(CrystalOf(after, after.layers.at(0).entries.at(0))), expected_prism);
  EXPECT_EQ(slots(CrystalOf(after, after.layers.at(0).entries.at(1))), expected_pyramid);
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
    // The empty case is zero rows, and zero rows is a valid filter — it is the one that says
    // nothing, which the commit path reads as "no filter". The translator used to produce a single
    // row holding an empty RaypathParams here, on the reading that a filter-less document must not
    // become a zero-row filter. That row is not filter-less: it carries a factor, which makes it
    // the editor's match-all, so a legacy file naming no raypath at all came back as a filter that
    // matches everything — and under filter_out, one that excludes every ray.
    { "empty-is-no-rows", "", 0, "" },
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

// ---------------------------------------------------------------------------------------------
// E5 — the retired ',' raypath connector, migrated on load.
//
// ',' used to be normalized to '-' by the parser, so a document could be saved with "3-5,1-2" in
// it and meant the four-face path 3-5-1-2. Now that the validator rejects ',', such a document
// would open into a state the editor refuses to commit — unless the load rewrites it. These cases
// are about that rewrite happening on BOTH read paths and stopping exactly at the raypath token.
//
// The counter is drained before each load, not only read after: DeserializeGuiStateJson resets it
// on entry, but a case that only read it afterwards would still pass if that reset were removed
// and some earlier case had left a count behind.

const char* const kCrystalJson = R"("crystal": {"type": "prism", "shape": {"height": 1.0}}, "proportion": 100.0)";

std::string V3DocWithSummand(const std::string& summand) {
  return std::string(R"({"schema_version": 3, "layers": [{"prob": 0.0, "entries": [{)") + kCrystalJson +
         R"(, "filter": {"action": "filter_in", "summands": [")" + summand + R"("]}}]}]})";
}

std::string LegacyDocWithRaypathText(const std::string& raypath_text) {
  return std::string(R"({"schema_version": 2, "layers": [{"prob": 0.0, "entries": [{)") + kCrystalJson +
         R"(, "filter": {"type": "raypath", "action": "filter_in", "raypath_text": ")" + raypath_text + R"("}}]}]})";
}

const FilterConfig& OnlyFilter(const GuiState& s) {
  EXPECT_EQ(s.filters.size(), 1u);
  return s.filters.at(0);
}

TEST(DocumentRoundtripChain, LegacyCommaRaypathConnectorMigratesToDashOnLoadV3Summands) {
  GuiState s;
  TakeRaypathCommaMigratedCount();
  ASSERT_TRUE(DeserializeGuiStateJson(V3DocWithSummand("3-5,1-2"), s));

  const FilterConfig& f = OnlyFilter(s);
  ASSERT_EQ(f.param.size(), 1u);
  // The path the document meant is preserved; only its spelling changed.
  EXPECT_EQ(f.param[0].text, std::string("3-5-1-2"));
  EXPECT_EQ(TakeRaypathCommaMigratedCount(), 1);
}

TEST(DocumentRoundtripChain, LegacyCommaRaypathConnectorMigratesToDashOnLoadV1V2RaypathText) {
  GuiState s;
  TakeRaypathCommaMigratedCount();
  ASSERT_TRUE(DeserializeGuiStateJson(LegacyDocWithRaypathText("3-5,1-2"), s));

  // The legacy arm reaches the rewrite through FromLegacyRaypath rather than the summands loop.
  // It is a separate call site, so it needs its own evidence — the v3 case above says nothing
  // about it.
  const FilterConfig& f = OnlyFilter(s);
  ASSERT_EQ(f.param.size(), 1u);
  EXPECT_EQ(f.param[0].text, std::string("3-5-1-2"));
  EXPECT_EQ(TakeRaypathCommaMigratedCount(), 1);
}

TEST(DocumentRoundtripChain, LegacyCommaMigrationLeavesEntryExitFacelistCommaAlone) {
  GuiState s;
  TakeRaypathCommaMigratedCount();
  ASSERT_TRUE(DeserializeGuiStateJson(V3DocWithSummand("entry:1,2 & 3-5,1-2"), s));

  // The red line between the two meanings of ',' in one row: the EE facelist keeps its OR comma,
  // the raypath token loses its connector comma. A migration that walked characters instead of
  // tokens would turn "entry face 1 or 2" into "entry:1-2" and quietly change what the filter
  // matches.
  const FilterConfig& f = OnlyFilter(s);
  ASSERT_EQ(f.param.size(), 1u);
  EXPECT_EQ(f.param[0].text, std::string("entry:1,2 & 3-5-1-2"));
  EXPECT_EQ(TakeRaypathCommaMigratedCount(), 1);
}

TEST(DocumentRoundtripChain, NoCommaDocumentIsNotTouchedByMigration) {
  GuiState s;
  TakeRaypathCommaMigratedCount();
  ASSERT_TRUE(DeserializeGuiStateJson(V3DocWithSummand("3-5 & entry:1,2"), s));

  // A document with no retired connector must come back byte for byte and must not be counted:
  // the load is a no-op here, not a reformat that happens to agree.
  const FilterConfig& f = OnlyFilter(s);
  ASSERT_EQ(f.param.size(), 1u);
  EXPECT_EQ(f.param[0].text, std::string("3-5 & entry:1,2"));
  EXPECT_EQ(TakeRaypathCommaMigratedCount(), 0);
}

TEST(DocumentRoundtripChain, MigratedDocumentSurvivesTheNextRoundTrip) {
  // The end of the story the migration promises: open an old document, save it, and the ',' is
  // gone from the file for good. Without this, a load that rewrote only the in-memory state would
  // still pass every case above and write the old spelling back out.
  GuiState loaded;
  ASSERT_TRUE(DeserializeGuiStateJson(V3DocWithSummand("3-5,1-2"), loaded));

  const std::string resaved = SerializeGuiStateJson(loaded);
  EXPECT_EQ(resaved.find("3-5,1-2"), std::string::npos) << "re-saved document still carries the retired ','";

  GuiState reloaded;
  TakeRaypathCommaMigratedCount();
  ASSERT_TRUE(DeserializeGuiStateJson(resaved, reloaded));
  ASSERT_EQ(OnlyFilter(reloaded).param.size(), 1u);
  EXPECT_EQ(OnlyFilter(reloaded).param[0].text, std::string("3-5-1-2"));
  EXPECT_EQ(TakeRaypathCommaMigratedCount(), 0) << "a re-saved document should need no second migration";
}

TEST(DocumentRoundtripChain, MigrationCountDoesNotLeakFromAPreviousLoad) {
  // The count belongs to one load, and the load is what says so — not the caller's discipline in
  // draining it. Two callers reach DeserializeGuiStateJson (LoadLmcFile and the user-defaults
  // merge), and a count carried over from the first would make the second announce a migration
  // that did not happen in it.
  GuiState first;
  TakeRaypathCommaMigratedCount();
  ASSERT_TRUE(DeserializeGuiStateJson(V3DocWithSummand("3-5,1-2"), first));

  GuiState second;
  ASSERT_TRUE(DeserializeGuiStateJson(V3DocWithSummand("3-5"), second));
  EXPECT_EQ(TakeRaypathCommaMigratedCount(), 0) << "the clean load inherited the previous load's count";
}

TEST(DocumentRoundtripChain, SchemaVersionIsFour) {
  // The version the writer stamps is what dates a file's ',' for anyone who later has to decide
  // what a ',' in it meant. Asserted as a literal, not as a read of the writer's own constant.
  const nlohmann::json root = nlohmann::json::parse(SerializeGuiStateJson(MinimalDocument()));
  ASSERT_TRUE(root.contains("schema_version"));
  EXPECT_EQ(root["schema_version"].get<int>(), 4);
}

}  // namespace

// The ev_mode spelling on disk, in both directions, and for the same reason the aspect-preset case
// above is not a round trip: writer and reader share kEvModeJsonNames, so a table edited on both
// sides agrees with itself while every saved document changes meaning. The expected words here are
// literals, and they are literally core's — config/render_config.hpp's own enum table spells them
// "relative"/"absolute", and a .lmc that disagreed would load into a GUI whose exported config then
// said something else.
//
// The unknown-word row is the third direction and the one with a real failure mode behind it: an
// unrecognised value must land on relative, the SAME place a missing key lands, so a typo cannot
// silently opt a document into the absolute anchor.
TEST(DocumentRoundtripChain, EvModeIsSpelledOnDiskTheWayCoreSpellsIt) {
  struct Row {
    int value;
    const char* spelling;
  };
  for (const Row& row : { Row{ 0, "relative" }, Row{ 1, "absolute" } }) {
    GuiState doc = MinimalDocument();
    doc.renderer.ev_mode = row.value;
    const auto written = nlohmann::json::parse(SerializeGuiStateJson(doc));
    EXPECT_EQ(written["renderer"]["ev_mode"].get<std::string>(), row.spelling);

    auto on_disk = nlohmann::json::parse(SerializeGuiStateJson(MinimalDocument()));
    on_disk["renderer"]["ev_mode"] = row.spelling;
    GuiState read_back = MinimalDocument();
    if (!DeserializeGuiStateJson(on_disk.dump(), read_back)) {
      // Non-fatal per row: one unreadable spelling must not take the other spelling's report with
      // it — which of the two failed is the whole diagnostic here.
      ADD_FAILURE() << row.spelling << ": the reader rejected a document carrying this spelling";
      continue;
    }
    EXPECT_EQ(read_back.renderer.ev_mode, row.value) << row.spelling;
  }

  // A document written before the key existed, and a document written with a typo in it, must
  // reach the same place: relative.
  for (const char* variant : { "", "abolute" }) {
    auto doc = nlohmann::json::parse(SerializeGuiStateJson(MinimalDocument()));
    doc["renderer"]["ev_mode"] = "absolute";
    if (variant[0] == '\0') {
      doc["renderer"].erase("ev_mode");
    } else {
      doc["renderer"]["ev_mode"] = variant;
    }
    GuiState read_back = MinimalDocument();
    read_back.renderer.ev_mode = 1;  // seed non-default so a no-op read is visible
    const char* label = variant[0] == '\0' ? "<absent>" : variant;
    if (!DeserializeGuiStateJson(doc.dump(), read_back)) {
      ADD_FAILURE() << label << ": an unrecognised ev_mode must load, not fail the whole document";
      continue;
    }
    EXPECT_EQ(read_back.renderer.ev_mode, 0) << "variant: " << label;
  }
}

}  // namespace lumice::gui
