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
      },
      [](const GuiState& s) {
        return s.bg_path.string() + " " + std::to_string(static_cast<int>(s.bg_show)) + " " +
               std::to_string(s.bg_alpha);
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

}  // namespace
}  // namespace lumice::gui
