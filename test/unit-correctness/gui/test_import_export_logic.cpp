// The import/export cases that never need a rendered frame.
//
// test_gui_import_export.cpp is the largest file in gui_test (89 cases) and almost all of it is
// JSON / LMC serialization round-trips: build a GuiState, serialize it through the production
// path, read it back, assert on the document. None of that needs a window, a GL context or an
// ImGuiTestContext — and because gui_test is build-only in CI (no display server), none of it
// had ever run on Windows or Linux. Serialization is exactly where platform differences bite
// (float formatting, path separators, locale), so these cases are worth more here than anywhere.
//
// What stayed in gui_test: the GL / document-reset owner cluster, whose seam is
// "document switch -> ResetFrontendState -> preview texture / poller staged"
// (doc/gui-state-governance.md), plus everything that drives frames through
// GuiFunc + YieldUntilTrue or reads gui::g_preview.
//
// Two mechanical notes on the translation from the IM_* form:
//   * IM_CHECK / IM_CHECK_EQ / IM_CHECK_LT / IM_CHECK_NE / IM_CHECK_STR_EQ all RETURN from the
//     test function when they fail (imgui_te_context.h: `if (!res) return;`, and IM_CHECK_OP's
//     `_RETURN` argument is `true` for every non-`_NO_RET` form). ASSERT_*, not EXPECT_*, is
//     therefore the mapping that preserves what these cases did before the move: several of them
//     guard a lookup and then dereference or index the result on the next line.
//   * Each case starts from gui::DoNew() rather than gui_test's ResetTestState(): a fresh
//     document is the only part of that helper these cases consume, and DoNew() is what it
//     delegates to for exactly that.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "gui/raypath_segments.hpp"
#include "lumice.h"
#include "support/scene_json_helpers.hpp"

namespace gui = lumice::gui;

using lumice::test::CommitSceneJson;
using lumice::test::CoreJson;
using lumice::test::CountDistinct;
using lumice::test::PrismFacePlaneOffsets;
using lumice::test::SceneJson;

// Test 1: Core JSON path invariants — export JSON → DeserializeFromJson.
// NOTE: This is NOT a GUI-state round-trip. The export path always emits a
// fixed full-sphere renderer (lens="dual_fisheye_equal_area", fov=180, visible="full",
// background=[0,0,0]) regardless of GUI state — Core only consumes this shape.
// GUI-state round-trip is covered by renderer_new_format_roundtrip / lmc_full_roundtrip.
// exposure_offset / opacity are not asserted: exposure_offset's 2^x → log2 round-trip
// introduces floating-point precision drift.
TEST(ImportExport, json_roundtrip) {
  gui::DoNew();

  // Set up non-default state
  gui::g_state.sim.ray_num_millions = 2.5f;

  // Serialize → deserialize
  std::string json = CoreJson(gui::g_state);
  ASSERT_TRUE(!json.empty());

  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(json, loaded);
  ASSERT_TRUE(ok);

  // State-derived fields survive the round-trip.
  ASSERT_TRUE(std::abs(loaded.sim.ray_num_millions - gui::g_state.sim.ray_num_millions) < 0.01f);
  ASSERT_EQ(static_cast<int>(loaded.layers.size()), static_cast<int>(gui::g_state.layers.size()));

  // Renderer fields that Core hardcodes: lens is always dual_fisheye_equal_area
  // (kLensTypeNames[4] == "Dual Fisheye Equal Area"; keep this index in sync
  // if kLensTypeNames is reordered) and fov is always 180. These assertions lock
  // the Core-path contract itself.
  ASSERT_EQ(loaded.renderer.lens_type, 4);
  ASSERT_EQ(loaded.renderer.fov, 180.0f);
}

// layer.probability round-trip must byte-preserve
// hand-written values including the "footgun" case (last-layer prob>0). The
// GUI's disable/warning logic is display-only; it must not silently rewrite
// the loaded value. Locks: file value == deserialized state.probability.
TEST(ImportExport, last_layer_prob_roundtrip) {
  gui::DoNew();

  // Two layers, both with prob>0 (last layer would trigger the CLI
  // warning, but load must still preserve the value byte-for-byte).
  gui::g_state.layers[0].probability = 0.3f;
  gui::Layer new_layer;
  gui::EntryCard e;
  e.crystal_id = 0;
  new_layer.entries.push_back(e);
  new_layer.probability = 0.45f;  // last layer, non-zero — the footgun value
  gui::g_state.layers.push_back(std::move(new_layer));

  std::string json = CoreJson(gui::g_state);
  ASSERT_TRUE(!json.empty());

  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(json, loaded);
  ASSERT_TRUE(ok);

  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 2);
  ASSERT_EQ(loaded.layers[0].probability, 0.3f);
  ASSERT_EQ(loaded.layers[1].probability, 0.45f);  // NOT silently zeroed

  // Second serialization path (code-review Major-1): the handle commit
  // path (BuildScene -> LUMICE_CommitScene) is what actually feeds the
  // simulator. Assert it preserves the last-layer footgun prob>0
  // too — the display disable/warning logic must not zero the stored value
  // on the path that reaches core. This path is commit-only (no reverse
  // deserialize), so it is a forward-fidelity check, not a round-trip.
  // (The .lmc save path SerializeGuiStateJson uses the identical
  // `jl["prob"] = layer.probability` float primitive as the core JSON
  // above, so its prob fidelity is covered by equivalence.)
  // Feed the DESERIALIZED state (not the original g_state) so this is the
  // full end-to-end chain: file JSON -> deserialize -> loaded -> scene
  // -> core (code-review r2 Minor-1).
  const auto scene_j = CommitSceneJson(loaded);
  const auto& scattering = scene_j["scene"]["scattering"];
  ASSERT_EQ(scattering.size(), (size_t)2);
  ASSERT_EQ(scattering[0]["prob"].get<float>(), 0.3f);
  ASSERT_EQ(scattering[1]["prob"].get<float>(), 0.45f);
}

// Test 2: JSON file round-trip — write to file, read back, deserialize, verify
TEST(ImportExport, json_file_roundtrip) {
  gui::DoNew();

  gui::g_state.sim.ray_num_millions = 3.0f;
  std::string json = CoreJson(gui::g_state);

  // Not a hard-coded "/tmp/..." — there is no /tmp on Windows, so ExportConfigJson correctly
  // reports a failed open and this case died on its FIRST assertion there, never reaching the
  // round-trip it exists to check. That went unnoticed while the case lived in gui_test, which
  // CI builds but never runs.
  const std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "lumice_config_test.json";
  bool write_ok = gui::ExportConfigJson(tmp_path, json);
  ASSERT_TRUE(write_ok);

  // Read back
  std::ifstream in(tmp_path);
  ASSERT_TRUE(in.is_open());
  std::string read_json((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  in.close();

  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(read_json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_TRUE(std::abs(loaded.sim.ray_num_millions - 3.0f) < 0.01f);

  std::filesystem::remove(tmp_path);
}

// Test 3: Old format backward compat — ID-referenced crystals/scattering → layers/entries
TEST(ImportExport, old_format_compat) {
  gui::DoNew();

  // Pre-copy-model .lmc GUI-state format (verified via `git show ade8fc2^`):
  // lowercase "type", nested "shape" block, scattering entries reference crystals
  // by crystal_id. DeserializeGuiStateJson's legacy branch (the
  // root.contains("crystals") && root.contains("scattering") path) reads exactly
  // this shape. Optional fields (wedge angles, indices, axis) are omitted because
  // this test only asserts crystal type, the primary shape dimensions (height for
  // prism; prism_h/upper_h/lower_h for pyramid), proportion, filter absence and
  // sun altitude.
  std::string json = R"({
    "crystals": [
      {"id": 1, "type": "prism", "shape": {"height": 2.0}},
      {"id": 2, "type": "pyramid",
       "shape": {"prism_h": 1.0, "upper_h": 0.3, "lower_h": 0.4}}
    ],
    "scattering": [
      {"prob": 0.8, "entries": [
        {"crystal_id": 1, "proportion": 60.0},
        {"crystal_id": 2, "proportion": 40.0, "filter_id": -1}
      ]}
    ],
    "sun": {"altitude": 25.0}
  })";

  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);

  // Verify migration to copy model
  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  ASSERT_EQ(static_cast<int>(loaded.layers[0].entries.size()), 2);
  ASSERT_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[0]).type, gui::CrystalType::kPrism);
  ASSERT_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[0]).height, 2.0f);
  ASSERT_EQ(loaded.layers[0].entries[0].proportion, 60.0f);
  ASSERT_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[1]).type, gui::CrystalType::kPyramid);
  ASSERT_TRUE(!loaded.layers[0].entries[1].filter_id.has_value());
  ASSERT_EQ(loaded.sun.altitude, 25.0f);
}

// Test: crystal shape randomization reaches the COMMIT path (AC4). BuildScene is the
// GUI→core bridge (LUMICE_CommitScene's input); pre-fix it wrapped every
// shape scalar in a NO_RANDOM distribution, so a GUI-configured randomization never reached the
// simulator. This asserts the emitted distribution carries the real {type,center,spread}
// for height and a per-face-heterogeneous face_distance — the same ShapeDist → LUMICE_Distribution
// value-aligned mapping the preview uses. A NO_RANDOM distribution emits as a bare number
// (c_api.cpp DistributionToJson), everything else as {"type","center","spread"}.
TEST(ImportExport, shape_randomization_commit_path) {
  gui::DoNew();

  auto& cr = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
  cr.type = gui::CrystalType::kPrism;
  cr.height = gui::ShapeDist{ gui::ShapeDistType::kGauss, 2.5f, 0.4f };
  cr.face_distance[0] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.2f, 0.3f };
  cr.face_distance[3] = gui::ShapeDist{ gui::ShapeDistType::kLaplacian, 0.9f, 0.05f };

  const auto scene_j = CommitSceneJson(gui::g_state);
  ASSERT_TRUE(scene_j["crystal"].size() >= (size_t)1);

  const auto& shape = scene_j["crystal"][0]["shape"];
  // height: the configured distribution reaches core, not a NO_RANDOM flattening.
  ASSERT_EQ(shape["height"]["type"].get<std::string>(), std::string("gauss"));
  ASSERT_EQ(shape["height"]["mean"].get<float>(), 2.5f);
  ASSERT_EQ(shape["height"]["std"].get<float>(), 0.4f);
  // per-face heterogeneity survives the commit mapping.
  const auto& fd = shape["face_distance"];
  ASSERT_EQ(fd[0]["type"].get<std::string>(), std::string("uniform"));
  ASSERT_EQ(fd[0]["mean"].get<float>(), 1.2f);
  ASSERT_EQ(fd[0]["std"].get<float>(), 0.3f);
  ASSERT_EQ(fd[3]["type"].get<std::string>(), std::string("laplacian"));
  ASSERT_EQ(fd[3]["mean"].get<float>(), 0.9f);
  ASSERT_EQ(fd[3]["std"].get<float>(), 0.05f);
  // an untouched face stays the deterministic default (NO_RANDOM → bare number).
  ASSERT_TRUE(fd[1].is_number());
  ASSERT_EQ(fd[1].get<float>(), 1.0f);
}

// AC1: a grouping reaches the PREVIEW mesh. Grouped scalars share one draw, so a C3 grouping of
// randomized face distances must produce a mesh with exactly two distinct prism-face distances.
// White-box on the geometry — the whole point of the feature is a symmetry the eye cannot verify.
TEST(ImportExport, sync_group_reaches_preview_mesh) {
  gui::DoNew();

  auto& cr = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
  cr.type = gui::CrystalType::kPrism;
  for (int i = 0; i < 6; ++i) {
    cr.face_distance[i] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.0f, 0.1f };
  }

  // Control: six independent draws → six distinct distances. Without it the grouped case
  // below would also pass on a mesh that ignored face_distance entirely.
  LUMICE_CrystalMesh independent{};
  ASSERT_TRUE(gui::BuildCrystalMeshData(cr, 12345, &independent));
  ASSERT_EQ(CountDistinct(PrismFacePlaneOffsets(independent), 1e-5f), (size_t)6);

  // {FACE_0,FACE_2,FACE_4} and {FACE_1,FACE_3,FACE_5}: two draws, six faces.
  for (int i = 0; i < 6; ++i) {
    cr.face_distance[i].sync_group = (i % 2 == 0) ? 1 : 2;
  }
  LUMICE_CrystalMesh c3{};
  ASSERT_TRUE(gui::BuildCrystalMeshData(cr, 12345, &c3));
  const auto off = PrismFacePlaneOffsets(c3);
  ASSERT_EQ(CountDistinct(off, 1e-5f), (size_t)2);
  // Named-face form of the same statement: it is faces 0/2/4 that agree, not some other
  // three-way split that happens to give two distinct values.
  ASSERT_TRUE(std::fabs(off[0] - off[2]) <= 1e-5f);
  ASSERT_TRUE(std::fabs(off[2] - off[4]) <= 1e-5f);
  ASSERT_TRUE(std::fabs(off[1] - off[3]) <= 1e-5f);
  ASSERT_TRUE(std::fabs(off[3] - off[5]) <= 1e-5f);
  ASSERT_TRUE(std::fabs(off[0] - off[1]) > 1e-5f);
}

// AC1, the import half: a HAND-AUTHORED core config carrying sync_group loads into the GUI's
// embedded form. This is the task's headline claim ("a config file with sync_group can be
// imported by the GUI"), and it runs through the same ParseCrystal the .lmc path uses.
TEST(ImportExport, sync_group_imports_from_core_json) {
  gui::DoNew();

  const std::string core_json = R"({
    "crystal": [
      {"id": 1, "type": "prism",
       "shape": {"height": 1.0,
                 "face_distance": [1,1,1,1,1,1],
                 "sync_group": {"height": 7, "face_distance": [1,2,1,2,1,2]}}}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5},
      "ray_num": 1000,
      "max_hits": 8,
      "scattering": [
        {"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0}]}
      ]
    }
  })";

  gui::GuiState loaded = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(core_json, loaded));
  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  const auto& c = gui::CrystalOf(loaded, loaded.layers[0].entries[0]);
  ASSERT_EQ(c.height.sync_group, 7);
  for (int i = 0; i < 6; ++i) {
    ASSERT_EQ(c.face_distance[i].sync_group, (i % 2 == 0) ? 1 : 2);
  }
}

// The wedge-angle, Miller-index and axis keys, which the sync_group tests above do not reach:
// sync_group covers only the ten shape SCALARS, and lmc_full_roundtrip checks "zenith" alone.
// Every key here is a bare string literal on purpose — expressing them through
// LUMICE_ShapeWedgeAngleKeyName / LUMICE_AxisScalarKeyName would let the schema and its own test
// drift together and pass forever. Import and export are both checked because they are separate
// key lists in the code: a round-trip alone is self-consistent even when the GUI writes keys core
// cannot read.
TEST(ImportExport, wedge_angle_and_axis_keys_roundtrip) {
  gui::DoNew();

  // --- Import: explicit wedge angles + all three axis keys, each a distinct value so a
  // swapped pair (zenith read into roll, upper read into lower) cannot pass.
  const std::string core_json = R"({
    "crystal": [
      {"id": 1, "type": "pyramid",
       "shape": {"prism_h": 1.2, "upper_h": 0.1, "lower_h": 0.5,
                 "upper_wedge_angle": 33.0, "lower_wedge_angle": 22.0},
       "axis": {"zenith": {"type": "gauss", "mean": 12.0, "std": 1.0},
                "azimuth": {"type": "uniform", "mean": 34.0, "std": 2.0},
                "roll": {"type": "gauss", "mean": 56.0, "std": 3.0}}}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5},
      "ray_num": 1000,
      "max_hits": 8,
      "scattering": [
        {"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0}]}
      ]
    }
  })";

  gui::GuiState loaded = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(core_json, loaded));
  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  const auto& c = gui::CrystalOf(loaded, loaded.layers[0].entries[0]);
  ASSERT_EQ(c.type, gui::CrystalType::kPyramid);
  ASSERT_LT(std::abs(c.upper_alpha - 33.0f), 1e-3f);
  ASSERT_LT(std::abs(c.lower_alpha - 22.0f), 1e-3f);
  ASSERT_LT(std::abs(c.zenith.mean - 12.0f), 1e-3f);
  ASSERT_LT(std::abs(c.azimuth.mean - 34.0f), 1e-3f);
  ASSERT_LT(std::abs(c.azimuth.std - 2.0f), 1e-3f);
  ASSERT_LT(std::abs(c.roll.mean - 56.0f), 1e-3f);
  ASSERT_LT(std::abs(c.roll.std - 3.0f), 1e-3f);

  // --- Import, legacy arm: with the explicit angles gone, the Miller indices take over. The
  // expected degrees are the same literals the core-side MillerIndexFallback tests pin.
  const std::string indices_json = R"({
    "crystal": [
      {"id": 1, "type": "pyramid",
       "shape": {"prism_h": 1.2, "upper_h": 0.1, "lower_h": 0.5,
                 "upper_indices": [2, 0, 3], "lower_indices": [1, 0, 1]}}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5},
      "ray_num": 1000,
      "max_hits": 8,
      "scattering": [
        {"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0}]}
      ]
    }
  })";

  gui::GuiState legacy = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(indices_json, legacy));
  const auto& lc = gui::CrystalOf(legacy, legacy.layers[0].entries[0]);
  ASSERT_LT(std::abs(lc.upper_alpha - 38.57f), 0.01f);  // {2,0,3}
  ASSERT_LT(std::abs(lc.lower_alpha - 28.00f), 0.01f);  // {1,0,1}

  // --- Export: the same keys come back out, spelled the same way.
  auto& cr = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
  cr.type = gui::CrystalType::kPyramid;
  cr.upper_alpha = 33.0f;
  cr.lower_alpha = 22.0f;
  cr.zenith = gui::AxisDist{ gui::AxisDistType::kGauss, 12.0f, 1.0f };
  cr.azimuth = gui::AxisDist{ gui::AxisDistType::kUniform, 34.0f, 2.0f };
  cr.roll = gui::AxisDist{ gui::AxisDistType::kGauss, 56.0f, 3.0f };

  const auto doc = nlohmann::json::parse(gui::SerializeGuiStateJson(gui::g_state));
  const auto& crystal_j = doc["layers"][0]["entries"][0]["crystal"];
  const auto& shape = crystal_j["shape"];
  ASSERT_TRUE(shape.contains("upper_wedge_angle"));
  ASSERT_TRUE(shape.contains("lower_wedge_angle"));
  ASSERT_LT(std::abs(shape["upper_wedge_angle"].get<float>() - 33.0f), 1e-3f);
  ASSERT_LT(std::abs(shape["lower_wedge_angle"].get<float>() - 22.0f), 1e-3f);
  // Miller indices are read-only legacy: the writer converts to an angle and never emits them.
  ASSERT_TRUE(!shape.contains("upper_indices"));
  ASSERT_TRUE(!shape.contains("lower_indices"));

  const auto& axis = crystal_j["axis"];
  ASSERT_TRUE(axis.contains("zenith"));
  ASSERT_TRUE(axis.contains("azimuth"));
  ASSERT_TRUE(axis.contains("roll"));
  ASSERT_LT(std::abs(axis["zenith"]["mean"].get<float>() - 12.0f), 1e-3f);
  ASSERT_LT(std::abs(axis["azimuth"]["mean"].get<float>() - 34.0f), 1e-3f);
  ASSERT_LT(std::abs(axis["azimuth"]["std"].get<float>() - 2.0f), 1e-3f);
  ASSERT_LT(std::abs(axis["roll"]["mean"].get<float>() - 56.0f), 1e-3f);
  ASSERT_LT(std::abs(axis["roll"]["std"].get<float>() - 3.0f), 1e-3f);
}

// AC2, the other half: an all-independent crystal must emit NO "sync_group" key, so every .lmc
// written before v4.13 keeps its exact wire form. Paired with a red/green flip — asserting only
// absence would pass just as happily on a serializer that never writes the key at all.
TEST(ImportExport, sync_group_absent_when_all_independent) {
  gui::DoNew();

  auto& cr = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
  cr.type = gui::CrystalType::kPrism;
  const auto shape_of = [](const std::string& doc_str) {
    return nlohmann::json::parse(doc_str)["layers"][0]["entries"][0]["crystal"]["shape"];
  };
  ASSERT_TRUE(!shape_of(gui::SerializeGuiStateJson(gui::g_state)).contains("sync_group"));

  cr.face_distance[3].sync_group = 1;
  ASSERT_TRUE(shape_of(gui::SerializeGuiStateJson(gui::g_state)).contains("sync_group"));
}

// AC3: the preview's change detector sees a grouping change. CrystalParamHash gates the mesh
// re-upload, so a hash blind to sync_group leaves the previous crystal on screen — which reads
// as "the feature does not work" rather than as a stale cache. Both directions are checked: a
// hash that ignored the field would fail the first, one that hashed uninitialized memory the
// second.
TEST(ImportExport, sync_group_changes_param_hash) {
  gui::DoNew();

  gui::CrystalConfig base;
  base.type = gui::CrystalType::kPrism;
  for (int i = 0; i < 6; ++i) {
    base.face_distance[i] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.0f, 0.1f };
  }

  gui::CrystalConfig same = base;
  ASSERT_EQ(gui::CrystalParamHash(base), gui::CrystalParamHash(same));

  // Every slot individually: a hash built from a partial field list would pass on some and
  // fail on others, and "which field is invisible to the preview" is not a guess worth making.
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    gui::CrystalConfig grouped = base;
    gui::ShapeScalarAt(grouped, slot).sync_group = 1;
    ASSERT_NE(gui::CrystalParamHash(base), gui::CrystalParamHash(grouped));
  }
}

// Test: legacy core-JSON import with array-form spectrum.
// Regression guard for the historical silent-drop bug in file_io.cpp:
// arrays were previously discarded without warning; must now populate custom_spectrum.
TEST(ImportExport, core_json_array_spectrum_import) {
  gui::DoNew();
  // Baseline: reset state has a preset spectrum, not custom.
  ASSERT_TRUE(gui::g_state.sun.spectrum_index != gui::kCustomSpectrumIndex);

  // Build a minimal core-JSON with an array-form spectrum and import it.
  nlohmann::json root;
  nlohmann::json cr;
  cr["id"] = 1;
  cr["type"] = "prism";
  cr["shape"]["height"] = 1.0f;
  cr["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 10.0f } };
  cr["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  root["crystal"] = nlohmann::json::array({ cr });
  root["scene"]["light_source"]["type"] = "sun";
  root["scene"]["light_source"]["altitude"] = 20.0f;
  root["scene"]["light_source"]["diameter"] = 0.5f;
  root["scene"]["light_source"]["spectrum"] = nlohmann::json::array(
      { { { "wavelength", 480.0f }, { "weight", 0.9f } }, { { "wavelength", 620.0f }, { "weight", 1.0f } } });
  root["scene"]["ray_num"] = 1000;
  root["scene"]["max_hits"] = 7;
  root["scene"]["scattering"] = nlohmann::json::array(
      { { { "prob", 1.0f }, { "entries", nlohmann::json::array({ { { "crystal", 1 }, { "proportion", 1.0f } } }) } } });

  // Import via the real Core-JSON entry point (mirrors the GUI "Open .json"
  // flow: app.cpp:498 DeserializeFromJson), NOT the binary .lmc loader.
  bool load_ok = gui::DeserializeFromJson(root.dump(), gui::g_state);
  ASSERT_TRUE(load_ok);
  ASSERT_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
  ASSERT_EQ(gui::g_state.sun.custom_spectrum.size(), (size_t)2);
  ASSERT_EQ(gui::g_state.sun.custom_spectrum[0].wavelength, 480.0f);
  ASSERT_EQ(gui::g_state.sun.custom_spectrum[1].wavelength, 620.0f);
}

// Test 4a: renderer copy-model new-format round-trip.
// Fields covered (per RenderConfig in gui_state.hpp):
//   lens_type, fov, elevation, azimuth, roll, sim_resolution_index, visible,
//   background[3], ray_color[3], opacity, exposure_offset
TEST(ImportExport, renderer_new_format_roundtrip) {
  gui::DoNew();

  // Populate every RenderConfig field with a non-default value.
  auto& r = gui::g_state.renderer;
  r.lens_type = 3;
  r.fov = 115.0f;
  r.elevation = 12.0f;
  r.azimuth = -24.0f;
  r.roll = 7.5f;
  r.sim_resolution_index = 2;
  r.visible = 1;
  r.front = true;
  r.background[0] = 0.1f;
  r.background[1] = 0.2f;
  r.background[2] = 0.3f;
  r.ray_color[0] = 0.8f;
  r.ray_color[1] = 0.6f;
  r.ray_color[2] = 0.4f;
  r.opacity = 0.75f;
  r.exposure_offset = 1.5f;

  std::string json = gui::SerializeGuiStateJson(gui::g_state);
  ASSERT_TRUE(!json.empty());

  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);

  ASSERT_EQ(loaded.renderer.lens_type, 3);
  ASSERT_EQ(loaded.renderer.fov, 115.0f);
  ASSERT_EQ(loaded.renderer.elevation, 12.0f);
  ASSERT_EQ(loaded.renderer.azimuth, -24.0f);
  ASSERT_EQ(loaded.renderer.roll, 7.5f);
  ASSERT_EQ(loaded.renderer.sim_resolution_index, 2);
  ASSERT_EQ(loaded.renderer.visible, 1);
  ASSERT_EQ(loaded.renderer.front, true);
  ASSERT_EQ(loaded.renderer.background[0], 0.1f);
  ASSERT_EQ(loaded.renderer.background[1], 0.2f);
  ASSERT_EQ(loaded.renderer.background[2], 0.3f);
  ASSERT_EQ(loaded.renderer.ray_color[0], 0.8f);
  ASSERT_EQ(loaded.renderer.ray_color[1], 0.6f);
  ASSERT_EQ(loaded.renderer.ray_color[2], 0.4f);
  ASSERT_EQ(loaded.renderer.opacity, 0.75f);
  ASSERT_EQ(loaded.renderer.exposure_offset, 1.5f);
}

// Test 4b: legacy .lmc renderer format (vector + selected_renderer_id + next_renderer_id)
// must still load correctly into the new copy-model renderer field.
// Fields covered: all RenderConfig fields (same list as Test 4a).
TEST(ImportExport, renderer_legacy_format_compat) {
  gui::DoNew();

  // Historical .lmc GUI-state format: renderers: [ { id, ... } ]
  // Simulates output from before the renderer fields were inlined into the document;
  // all renderer fields non-default.
  std::string json = R"({
    "layers": [],
    "renderers": [
      {
        "id": 7,
        "lens_type": "fisheye_stereographic",
        "fov": 135.0,
        "elevation": 11.0,
        "azimuth": -21.0,
        "roll": 3.5,
        "sim_resolution": 2048,
        "visible": "lower",
        "background": [0.11, 0.22, 0.33],
        "ray_color": [0.9, 0.7, 0.5],
        "opacity": 0.8,
        "exposure_offset": -1.25
      }
    ],
    "selected_renderer_id": 7,
    "next_renderer_id": 8
  })";

  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);

  // Values must equal the first (and only) element of the legacy renderers array.
  ASSERT_EQ(loaded.renderer.lens_type, 3);  // index of "fisheye_stereographic"
  ASSERT_EQ(loaded.renderer.fov, 135.0f);
  ASSERT_EQ(loaded.renderer.elevation, 11.0f);
  ASSERT_EQ(loaded.renderer.azimuth, -21.0f);
  ASSERT_EQ(loaded.renderer.roll, 3.5f);
  ASSERT_EQ(loaded.renderer.sim_resolution_index, 2);  // 2048 → index 2 in kSimResolutions={512,1024,2048,4096}
  ASSERT_EQ(loaded.renderer.visible, 1);               // "lower" → 1
  ASSERT_EQ(loaded.renderer.front, false);             // no "front" key → default false
  ASSERT_TRUE(std::abs(loaded.renderer.background[0] - 0.11f) < 1e-5f);
  ASSERT_TRUE(std::abs(loaded.renderer.background[1] - 0.22f) < 1e-5f);
  ASSERT_TRUE(std::abs(loaded.renderer.background[2] - 0.33f) < 1e-5f);
  ASSERT_TRUE(std::abs(loaded.renderer.ray_color[0] - 0.9f) < 1e-5f);
  ASSERT_TRUE(std::abs(loaded.renderer.ray_color[1] - 0.7f) < 1e-5f);
  ASSERT_TRUE(std::abs(loaded.renderer.ray_color[2] - 0.5f) < 1e-5f);
  ASSERT_TRUE(std::abs(loaded.renderer.opacity - 0.8f) < 1e-5f);
  ASSERT_EQ(loaded.renderer.exposure_offset, -1.25f);
}

// Test 4c: Core JSON with empty render array — must not crash; renderer keeps defaults.
// Covers the boundary DeserializeFromJson gained when the renderer fields moved inline.
TEST(ImportExport, core_json_empty_render) {
  gui::DoNew();

  std::string json = R"({
    "crystal": [],
    "filter": [],
    "scene": {"scattering": [], "ray_num": 1000, "max_hits": 4,
               "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"}},
    "render": []
  })";

  gui::GuiState loaded;
  bool ok = gui::DeserializeFromJson(json, loaded);
  ASSERT_TRUE(ok);
  // Default RenderConfig field values (from gui_state.hpp).
  // Warning log path exists but is not asserted automatically; observe via logs manually.
  ASSERT_EQ(loaded.renderer.lens_type, 0);
  ASSERT_EQ(loaded.renderer.fov, 90.0f);
  ASSERT_EQ(loaded.renderer.sim_resolution_index, 1);
  ASSERT_EQ(loaded.renderer.visible, 2);
  ASSERT_EQ(loaded.renderer.front, false);
  ASSERT_EQ(loaded.renderer.opacity, 1.0f);
}

// Test 4e: legacy "visible": "front" maps to base=full + front=true
TEST(ImportExport, renderer_legacy_front_compat) {
  gui::DoNew();

  std::string json = R"({
    "layers": [],
    "renderer": {
      "lens_type": "fisheye_equal_area",
      "fov": 180.0,
      "visible": "front"
    }
  })";

  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(loaded.renderer.visible, gui::kVisibleFull);
  ASSERT_EQ(loaded.renderer.front, true);
}

// Test 4f: legacy "visible": "front" in core-JSON format (DeserializeFromJson path)
// Covers the root["render"][0]["visible"]=="front" branch at file_io.cpp:1038-1041,
// which is distinct from the GUI-JSON "renderer" object path tested by renderer_legacy_front_compat.
TEST(ImportExport, core_json_legacy_front_compat) {
  gui::DoNew();

  std::string json = R"({
    "crystal": [],
    "filter": [],
    "scene": {"scattering": [], "ray_num": 1000, "max_hits": 4,
               "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"}},
    "render": [
      {
        "lens": {"type": "fisheye_equal_area", "fov": 180.0},
        "visible": "front"
      }
    ]
  })";

  gui::GuiState loaded;
  bool ok = gui::DeserializeFromJson(json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(loaded.renderer.visible, gui::kVisibleFull);
  ASSERT_EQ(loaded.renderer.front, true);
}

// Test 4d: legacy format with multiple renderers — GUI now single-renderer, take first.
TEST(ImportExport, renderer_legacy_multi_takes_first) {
  gui::DoNew();

  std::string json = R"({
    "layers": [],
    "renderers": [
      {"id": 1, "lens_type": "linear", "fov": 90.0},
      {"id": 2, "lens_type": "fisheye_equal_area", "fov": 180.0}
    ]
  })";
  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);
  // Must equal the first entry, not the second.
  ASSERT_EQ(loaded.renderer.lens_type, 0);  // "linear" → 0
  ASSERT_EQ(loaded.renderer.fov, 90.0f);
}

// Test E — legacy `overlay_<x>` key (single
// visibility) deserializes into both line and label = legacy_value.
TEST(ImportExport, overlay_legacy_key_fallback) {
  gui::DoNew();

  std::string json = R"({
    "layers": [],
    "renderer": {"lens_type": "linear", "fov": 90.0},
    "overlay_horizon": true,
    "overlay_grid": false,
    "overlay_sun_circles": true
  })";
  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(loaded.show_horizon_line, true);
  ASSERT_EQ(loaded.show_horizon_label, true);
  ASSERT_EQ(loaded.show_grid_line, false);
  ASSERT_EQ(loaded.show_grid_label, false);
  ASSERT_EQ(loaded.show_sun_circles_line, true);
  ASSERT_EQ(loaded.show_sun_circles_label, true);
}

// Test F — new keys take precedence over the
// legacy key when both are present (mixed-key scenario for hand-edited JSON).
// Cover horizon AND grid to distinguish "all overlays handled correctly"
// from "only horizon special-cased".
TEST(ImportExport, overlay_new_keys_take_precedence) {
  gui::DoNew();

  std::string json = R"({
    "layers": [],
    "renderer": {"lens_type": "linear", "fov": 90.0},
    "overlay_horizon": true,
    "overlay_horizon_line": true,
    "overlay_horizon_label": false,
    "overlay_grid": true,
    "overlay_grid_line": false,
    "overlay_grid_label": true
  })";
  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(loaded.show_horizon_line, true);
  ASSERT_EQ(loaded.show_horizon_label, false);
  ASSERT_EQ(loaded.show_grid_line, false);
  ASSERT_EQ(loaded.show_grid_label, true);
}

// Test G — round-trip preserves all six
// line/label fields independently (no collapsing back to single key on write).
TEST(ImportExport, overlay_roundtrip_preserves_split) {
  gui::DoNew();

  gui::g_state.show_horizon_line = true;
  gui::g_state.show_horizon_label = false;
  gui::g_state.show_grid_line = false;
  gui::g_state.show_grid_label = true;
  gui::g_state.show_sun_circles_line = true;
  gui::g_state.show_sun_circles_label = true;

  std::string json = gui::SerializeGuiStateJson(gui::g_state);
  ASSERT_TRUE(!json.empty());

  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(loaded.show_horizon_line, true);
  ASSERT_EQ(loaded.show_horizon_label, false);
  ASSERT_EQ(loaded.show_grid_line, false);
  ASSERT_EQ(loaded.show_grid_label, true);
  ASSERT_EQ(loaded.show_sun_circles_line, true);
  ASSERT_EQ(loaded.show_sun_circles_label, true);
}

// Zenith/nadir overlay round-trip — 4 new fields
// (line toggle, color[3], alpha, radius_px) preserved through Serialize → Deserialize.
// Missing-key fallback covered separately by overlay_legacy_key_fallback.
TEST(ImportExport, zenith_nadir_roundtrip) {
  gui::DoNew();

  gui::g_state.show_zenith_nadir_line = true;
  gui::g_state.zenith_nadir_color[0] = 0.1f;
  gui::g_state.zenith_nadir_color[1] = 0.5f;
  gui::g_state.zenith_nadir_color[2] = 0.9f;
  gui::g_state.zenith_nadir_alpha = 0.42f;
  gui::g_state.zenith_nadir_radius_px = 12.5f;

  std::string json = gui::SerializeGuiStateJson(gui::g_state);
  ASSERT_TRUE(!json.empty());

  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(loaded.show_zenith_nadir_line, true);
  ASSERT_EQ(loaded.zenith_nadir_color[0], 0.1f);
  ASSERT_EQ(loaded.zenith_nadir_color[1], 0.5f);
  ASSERT_EQ(loaded.zenith_nadir_color[2], 0.9f);
  ASSERT_EQ(loaded.zenith_nadir_alpha, 0.42f);
  ASSERT_EQ(loaded.zenith_nadir_radius_px, 12.5f);

  // Legacy file (without these keys) must fall back to defaults — verifies
  // no exception and no spurious enabled state for users on older .lmc files.
  std::string legacy_json = R"({
    "layers": [],
    "renderer": {"lens_type": "linear", "fov": 90.0}
  })";
  gui::GuiState legacy_loaded;
  ASSERT_TRUE(gui::DeserializeGuiStateJson(legacy_json, legacy_loaded));
  ASSERT_EQ(legacy_loaded.show_zenith_nadir_line, false);
  ASSERT_EQ(legacy_loaded.zenith_nadir_color[0], 0.8f);
  ASSERT_EQ(legacy_loaded.zenith_nadir_color[1], 0.2f);
  ASSERT_EQ(legacy_loaded.zenith_nadir_color[2], 0.2f);
  ASSERT_EQ(legacy_loaded.zenith_nadir_alpha, 0.6f);
  ASSERT_EQ(legacy_loaded.zenith_nadir_radius_px, 8.0f);
}

// Multi-raypath OR, end-to-end.
// GUI raypath_text "3-5; 1-3" → export JSON → ConfigManager parses
// out 3 filters: 2 simple raypaths (ids 1, 2) + 1 complex (id 3) referencing
// them. The main filter referenced by the scattering entry is the complex.
TEST(ImportExport, multi_raypath_or_e2e) {
  gui::DoNew();

  // Build a GuiState with one entry whose filter has a multi-segment raypath.
  gui::g_state.layers.clear();
  gui::Layer layer;
  layer.probability = 0.0f;
  gui::EntryCard entry;
  gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
  gui::CrystalOf(gui::g_state, entry).height = 1.0f;
  gui::CrystalOf(gui::g_state, entry).face_distance[0] = 1.0f;
  gui::CrystalOf(gui::g_state, entry).face_distance[1] = 1.0f;
  gui::CrystalOf(gui::g_state, entry).face_distance[2] = 1.0f;
  gui::CrystalOf(gui::g_state, entry).face_distance[3] = 1.0f;
  gui::CrystalOf(gui::g_state, entry).face_distance[4] = 1.0f;
  gui::CrystalOf(gui::g_state, entry).face_distance[5] = 1.0f;
  entry.proportion = 100.0f;
  gui::FilterConfig f;
  f.SetRaypath(gui::RaypathParams{ "3-5; 1-3" });
  gui::SetFilter(gui::g_state, entry, f);
  layer.entries.push_back(entry);
  gui::g_state.layers.push_back(layer);

  // Serialize and inspect the emitted core JSON directly. NOTE: this test
  // validates the serialized JSON shape only (filter array layout, ids,
  // composition structure) — not ConfigManager round-trip semantics. If
  // round-trip behavior needs guarding in the future, add a dedicated
  // core-side test or a config-deserialization C API (see backlog).
  std::string core_json = CoreJson(gui::g_state);
  auto parsed = nlohmann::json::parse(core_json);

  // The Scene assigns filter ids itself, 0-based in insertion order (lumice.h "Incremental
  // build"). ExpandFilterToScene adds the per-clause children first (ids 0..N-1) then the
  // complex (id N), so array index == id.
  ASSERT_TRUE(parsed.contains("filter") && parsed["filter"].is_array());
  ASSERT_EQ(static_cast<int>(parsed["filter"].size()), 3);

  // Filter 0: raypath [3, 5] (segment "3-5").
  ASSERT_EQ(parsed["filter"][0]["id"].get<int>(), 0);
  ASSERT_STREQ(parsed["filter"][0]["type"].get<std::string>().c_str(), "raypath");
  ASSERT_EQ(parsed["filter"][0]["raypath"], nlohmann::json({ 3, 5 }));

  // Filter 1: raypath [1, 3] (segment "1-3").
  ASSERT_EQ(parsed["filter"][1]["id"].get<int>(), 1);
  ASSERT_STREQ(parsed["filter"][1]["type"].get<std::string>().c_str(), "raypath");
  ASSERT_EQ(parsed["filter"][1]["raypath"], nlohmann::json({ 1, 3 }));

  // Filter 2: complex. A single-term clause is emitted as a BARE id (core wire form —
  // core filter_config.cpp to_json / c_api.cpp CompositionArrayToJson), so a 2-clause
  // OR of singletons is [0, 1], not [[0], [1]]. The GUI's own emitter used to write the
  // always-nested form; going through the core encoder aligns the two.
  ASSERT_EQ(parsed["filter"][2]["id"].get<int>(), 2);
  ASSERT_STREQ(parsed["filter"][2]["type"].get<std::string>().c_str(), "complex");
  ASSERT_EQ(parsed["filter"][2]["composition"], nlohmann::json({ 0, 1 }));

  // Scattering entry should reference the complex filter (id 2).
  ASSERT_TRUE(parsed.contains("scene"));
  ASSERT_TRUE(parsed["scene"]["scattering"][0]["entries"][0]["filter"] == 2);
}

// Single-segment raypath stays a
// RaypathFilterParam (no forced upgrade to ComplexFilterParam).
TEST(ImportExport, single_raypath_no_complex_upgrade) {
  gui::DoNew();

  gui::g_state.layers.clear();
  gui::Layer layer;
  layer.probability = 0.0f;
  gui::EntryCard entry;
  gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
  gui::CrystalOf(gui::g_state, entry).height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
  }
  entry.proportion = 100.0f;
  gui::FilterConfig f;
  f.SetRaypath(gui::RaypathParams{ "3-1-5" });
  gui::SetFilter(gui::g_state, entry, f);
  layer.entries.push_back(entry);
  gui::g_state.layers.push_back(layer);

  // Inspect serialized JSON directly — same shape-only contract as the
  // multi_raypath_or_e2e test above.
  std::string core_json = CoreJson(gui::g_state);
  auto parsed = nlohmann::json::parse(core_json);

  // Exactly 1 simple raypath filter, no complex upgrade.
  ASSERT_TRUE(parsed.contains("filter") && parsed["filter"].is_array());
  ASSERT_EQ(static_cast<int>(parsed["filter"].size()), 1);
  ASSERT_STREQ(parsed["filter"][0]["type"].get<std::string>().c_str(), "raypath");
  ASSERT_EQ(static_cast<int>(parsed["filter"][0]["raypath"].size()), 3);
}

// T7: a non-degenerate
// SoP (multi-summand and/or multi-factor) must render a non-crash entry-card
// summary. FilterSummary (panels.cpp) previously called DegenerateFactor(),
// which asserts/UB on a non-degenerate SoP. Assert it returns a non-empty
// string without crashing. (Full multi-summand editor UI is 333.4.)
TEST(ImportExport, non_degenerate_sop_summary_no_crash) {
  gui::DoNew();

  auto make_sop = [](const std::vector<std::string>& rows) {
    gui::FilterConfig f;
    gui::SumOfProducts sop;
    for (const auto& row : rows) {
      sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
    }
    f.param = std::move(sop);
    return f;
  };

  // Multi-summand OR.
  {
    gui::FilterConfig f = make_sop({ "3-5", "1-3", "2-6" });
    std::string s = gui::FilterSummary(std::optional<gui::FilterConfig>{ f });
    ASSERT_TRUE(!s.empty());
    ASSERT_TRUE(s.find("(+2 more)") != std::string::npos);
  }
  // Multi-factor AND (single row, non-degenerate: 1 row but 2 factors).
  {
    gui::FilterConfig f = make_sop({ "entry:3 & 7-1" });
    std::string s = gui::FilterSummary(std::optional<gui::FilterConfig>{ f });
    ASSERT_TRUE(!s.empty());
  }
  // Mixed OR + AND.
  {
    gui::FilterConfig f = make_sop({ "entry:3,4 & 7-1", "2-6" });
    std::string s = gui::FilterSummary(std::optional<gui::FilterConfig>{ f });
    ASSERT_TRUE(!s.empty());
  }
}

// A single OR-row carrying inline ';' alternatives
// (`1-3;3-5`) must produce a serialized filter array structurally identical
// to the two-row form (`1-3` / `3-5`), because the ';' fan-out in
// ValidateRaypathTextMultiSegment + ExpandSopToClauses is meant to be the
// exact same expansion path both entry points feed into.
TEST(ImportExport, semicolon_row_equals_two_rows_composition) {
  gui::DoNew();

  auto build_and_serialize = [](const std::vector<std::string>& rows) -> nlohmann::json {
    gui::g_state.layers.clear();
    gui::g_state.filters.clear();
    gui::Layer layer;
    layer.probability = 0.0f;
    gui::EntryCard entry;
    gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
    gui::CrystalOf(gui::g_state, entry).height = 1.0f;
    for (int i = 0; i < 6; ++i) {
      gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
    }
    entry.proportion = 100.0f;
    gui::FilterConfig f;
    gui::SumOfProducts sop;
    for (const auto& row : rows) {
      sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
    }
    f.param = std::move(sop);
    gui::SetFilter(gui::g_state, entry, f);
    layer.entries.push_back(entry);
    gui::g_state.layers.push_back(layer);
    return nlohmann::json::parse(CoreJson(gui::g_state));
  };

  const auto single_row = build_and_serialize({ "1-3;3-5" });
  const auto two_rows = build_and_serialize({ "1-3", "3-5" });

  // Same filter-array size (2 simple raypaths + 1 complex = 3).
  ASSERT_TRUE(single_row.contains("filter") && single_row["filter"].is_array());
  ASSERT_TRUE(two_rows.contains("filter") && two_rows["filter"].is_array());
  ASSERT_EQ(static_cast<int>(single_row["filter"].size()), 3);
  ASSERT_EQ(static_cast<int>(two_rows["filter"].size()), 3);

  // Byte-identical filter subtrees (raypath children + complex composition).
  // Serialization order is deterministic: FactorAlternatives → ExpandSopToClauses
  // sees the same expanded summand list for both entry points, so ids and
  // composition order match exactly.
  ASSERT_TRUE(single_row["filter"] == two_rows["filter"]);
}

// Distributive form: the ';' alternation must
// distribute over an AND partner — `1-3;3-5 & entry:2` expands to two
// clauses `1-3 & entry:2` / `3-5 & entry:2`, identical to writing them
// across two separate rows.
TEST(ImportExport, semicolon_row_distributes_over_and_factor) {
  gui::DoNew();

  auto build_and_serialize = [](const std::vector<std::string>& rows) -> nlohmann::json {
    gui::g_state.layers.clear();
    gui::g_state.filters.clear();
    gui::Layer layer;
    layer.probability = 0.0f;
    gui::EntryCard entry;
    gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
    gui::CrystalOf(gui::g_state, entry).height = 1.0f;
    for (int i = 0; i < 6; ++i) {
      gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
    }
    entry.proportion = 100.0f;
    gui::FilterConfig f;
    gui::SumOfProducts sop;
    for (const auto& row : rows) {
      sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
    }
    f.param = std::move(sop);
    gui::SetFilter(gui::g_state, entry, f);
    layer.entries.push_back(entry);
    gui::g_state.layers.push_back(layer);
    return nlohmann::json::parse(CoreJson(gui::g_state));
  };

  const auto single_row = build_and_serialize({ "1-3;3-5 & entry:2" });
  const auto two_rows = build_and_serialize({ "1-3 & entry:2", "3-5 & entry:2" });

  // Two AC1 checks in one test:
  //   1. Both forms serialize to the SAME filter array (byte-identical
  //      children + composition).
  //   2. The composition has exactly 2 clauses (distributive law honored).
  ASSERT_TRUE(single_row["filter"] == two_rows["filter"]);
  // Find the complex filter in the array (composition present).
  const nlohmann::json* complex_filter = nullptr;
  for (const auto& jf : single_row["filter"]) {
    if (jf.contains("composition")) {
      complex_filter = &jf;
      break;
    }
  }
  ASSERT_TRUE(complex_filter != nullptr);
  ASSERT_EQ(static_cast<int>((*complex_filter)["composition"].size()), 2);
}

// Legacy v=1 .lmc / GUI JSON without
// `type` field falls back to RaypathParams (default raypath_text "").
TEST(ImportExport, legacy_no_type_falls_back_to_raypath) {
  gui::DoNew();

  // A v=1-style payload: filter object lacks `type` and uses raypath_text directly.
  std::string json = R"({
    "schema_version": 1,
    "layers": [
      { "prob": 0.0, "entries": [
        { "crystal": { "type": "prism", "shape": { "height": 1.0, "face_distance": [1,1,1,1,1,1] } },
          "proportion": 100.0,
          "filter": { "id": 1, "action": "filter_in", "raypath_text": "3-1-5", "sym_p": true, "sym_b": true, "sym_d": true }
        }
      ]}
    ],
    "renderer": {"lens_type": "linear", "fov": 90.0}
  })";
  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  ASSERT_EQ(static_cast<int>(loaded.layers[0].entries.size()), 1);
  ASSERT_TRUE(loaded.layers[0].entries[0].filter_id.has_value());
  const auto& f = loaded.filters[*loaded.layers[0].entries[0].filter_id];
  ASSERT_TRUE(f.IsRaypath());
  ASSERT_EQ(f.RaypathText(), std::string("3-1-5"));
}

// T4: a legacy v2 .lmc raypath filter
// with ';' multi-segment sugar upgrades losslessly to a multi-row SoP via the
// canonical FromLegacyRaypath fan-out (one OR row per segment).
TEST(ImportExport, legacy_v2_multisegment_raypath_upgrades_to_sop) {
  gui::DoNew();

  const std::string v2_lmc = R"({
    "schema_version": 2,
    "layers": [{
      "prob": 0.0,
      "entries": [{
        "crystal": {"type": "prism", "shape": {"height": 1.0}},
        "proportion": 100.0,
        "filter": {"type": "raypath", "action": "filter_in", "raypath_text": "3-5;1-3"}
      }]
    }]
  })";

  gui::GuiState restored;
  bool ok = gui::DeserializeGuiStateJson(v2_lmc, restored);
  ASSERT_TRUE(ok);
  ASSERT_TRUE(restored.layers[0].entries[0].filter_id.has_value());
  const auto& f = restored.filters[*restored.layers[0].entries[0].filter_id];
  // Split into two OR rows: "3-5" and "1-3".
  ASSERT_EQ(static_cast<int>(f.param.size()), 2);
  ASSERT_STREQ(f.param[0].text.c_str(), "3-5");
  ASSERT_STREQ(f.param[1].text.c_str(), "1-3");
}

// Per-type entry/exit — end-to-end equivalence between
// GUI EE filter → export JSON output and a hand-crafted reference
// core JSON object. Validates AC-6 (`type` / `action` / `symmetry` /
// `entry` / `exit` / `id` field-by-field equality regardless of insertion
// order via nlohmann::json::operator==).
TEST(ImportExport, entry_exit_serialize_core_equivalent) {
  gui::DoNew();

  // Build a single-entry GuiState carrying an EntryExitParams filter.
  gui::g_state.layers.clear();
  gui::Layer layer;
  layer.probability = 1.0f;

  gui::EntryCard e;
  gui::CrystalOf(gui::g_state, e).type = gui::CrystalType::kPrism;
  gui::CrystalOf(gui::g_state, e).height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    gui::CrystalOf(gui::g_state, e).face_distance[i] = 1.0f;
  }
  e.proportion = 100.0f;

  gui::FilterConfig fc;
  fc.action = 1;     // filter_out
  fc.sym_p = true;   // → "P" in symmetry suffix
  fc.sym_b = false;  // omitted
  fc.sym_d = true;   // → "D"
  fc.SetEntryExit(gui::EntryExitParams{ /*entry_text=*/"2", /*exit_text=*/"5" });
  gui::SetFilter(gui::g_state, e, fc);

  layer.entries.push_back(e);
  gui::g_state.layers.push_back(layer);

  const std::string s = CoreJson(gui::g_state);
  ASSERT_TRUE(!s.empty());

  const auto j = nlohmann::json::parse(s);
  ASSERT_TRUE(j.contains("filter"));
  ASSERT_TRUE(j["filter"].is_array());
  ASSERT_EQ(static_cast<int>(j["filter"].size()), 1);

  // Field-level assertions (also catches symmetry string ordering bugs).
  const auto& jf = j["filter"][0];
  ASSERT_STREQ(jf["type"].get<std::string>().c_str(), "entry_exit");
  ASSERT_STREQ(jf["action"].get<std::string>().c_str(), "filter_out");
  ASSERT_STREQ(jf["symmetry"].get<std::string>().c_str(), "PD");
  ASSERT_EQ(jf["entry"].get<int>(), 2);
  ASSERT_EQ(jf["exit"].get<int>(), 5);
  ASSERT_EQ(jf["id"].get<int>(), 0);  // Scene-assigned, 0-based

  // Whole-object equivalence with hand-crafted reference (json::operator==
  // is field-set + value equality, insertion-order independent).
  const nlohmann::json expected = {
    { "id", 0 },          { "type", "entry_exit" }, { "action", "filter_out" },
    { "symmetry", "PD" }, { "entry", 2 },           { "exit", 5 },
  };
  ASSERT_TRUE(jf == expected);
}

// filter-direction-hide (issue 180.2): .lmc with type="direction" filter
// must degrade to empty RaypathParams after the Direction type is removed
// from the GUI. Mirrors Crystal's unknown-type fallback path (#179).
TEST(ImportExport, direction_lmc_degrades_to_raypath) {
  gui::DoNew();

  const std::string v2_lmc = R"({
    "schema_version": 2,
    "layers": [{
      "prob": 0.0,
      "entries": [{
        "crystal": {"type": "prism", "shape": {"height": 1.0}},
        "proportion": 100.0,
        "filter": {
          "type": "direction",
          "action": "filter_in",
          "az": 30.0, "el": 15.0
        }
      }]
    }]
  })";

  gui::GuiState restored;
  bool ok = gui::DeserializeGuiStateJson(v2_lmc, restored);
  ASSERT_TRUE(ok);
  ASSERT_EQ(static_cast<int>(restored.layers[0].entries.size()), 1);

  // direction type → unknown-type fallback → empty RaypathParams
  ASSERT_TRUE(restored.layers[0].entries[0].filter_id.has_value());
  const auto& f = restored.filters[*restored.layers[0].entries[0].filter_id];
  ASSERT_TRUE(f.IsRaypath());
  ASSERT_TRUE(f.RaypathText().empty());
}

// Legacy v2 .lmc Entry-Exit filter (with
// int "entry" / "exit" fields) must load successfully and translate to
// the v3 string representation. Re-serialization writes the new
// entry_text / exit_text fields.
TEST(ImportExport, entry_exit_v2_int_translates_to_text_on_load) {
  gui::DoNew();

  const std::string v2_lmc = R"({
    "schema_version": 2,
    "layers": [{
      "prob": 0.0,
      "entries": [{
        "crystal": {"type": "prism", "shape": {"height": 1.0}},
        "proportion": 100.0,
        "filter": {
          "type": "entry_exit",
          "action": "filter_in",
          "entry": 7, "exit": 4
        }
      }]
    }]
  })";

  gui::GuiState restored;
  bool ok = gui::DeserializeGuiStateJson(v2_lmc, restored);
  ASSERT_TRUE(ok);
  ASSERT_TRUE(restored.layers[0].entries[0].filter_id.has_value());
  const auto& f = restored.filters[*restored.layers[0].entries[0].filter_id];
  ASSERT_TRUE(f.IsEntryExit());
  const auto& ee = f.EntryExitParamsValue();
  ASSERT_EQ(ee.entry_text, std::string("7"));
  ASSERT_EQ(ee.exit_text, std::string("4"));

  // Re-serialization writes the v3 sum-of-products form: type "sop" +
  // "summands" array of canonical row texts (the legacy EE int fields upgrade
  // to the "entry:<e> & exit:<x>" grammar row).
  const std::string written = gui::SerializeGuiStateJson(restored);
  const auto j = nlohmann::json::parse(written);
  const auto& jf = j["layers"][0]["entries"][0]["filter"];
  ASSERT_STREQ(jf["type"].get<std::string>().c_str(), "sop");
  ASSERT_TRUE(jf["summands"].is_array());
  ASSERT_EQ(static_cast<int>(jf["summands"].size()), 1);
  ASSERT_STREQ(jf["summands"][0].get<std::string>().c_str(), "entry:7 & exit:4");
}

// Reject -> reconstruct: a
// multi-segment raypath complex filter (N simple raypaths + 1 OR-of-singletons
// complex) reconstructs into an N-row sum-of-products (one raypath factor per
// row). Pre-uplift this merged back into a single ';'-joined RaypathParams; the
// SoP model now surfaces each OR row explicitly (semantically equivalent, and
// re-serialize byte-equivalent). No warning (fully representable).
TEST(ImportExport, complex_raypath_roundtrip) {
  gui::DoNew();
  gui::ClearImportComplexFilterWarning();

  gui::g_state.layers.clear();
  gui::Layer layer;
  layer.probability = 0.0f;
  gui::EntryCard entry;
  gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
  gui::CrystalOf(gui::g_state, entry).height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
  }
  entry.proportion = 100.0f;
  gui::FilterConfig f;
  f.SetRaypath(gui::RaypathParams{ "3-5;1-3" });
  gui::SetFilter(gui::g_state, entry, f);
  layer.entries.push_back(entry);
  gui::g_state.layers.push_back(layer);

  const std::string core_json = CoreJson(gui::g_state);
  ASSERT_TRUE(!core_json.empty());

  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(core_json, loaded);
  ASSERT_TRUE(ok);

  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  ASSERT_EQ(static_cast<int>(loaded.layers[0].entries.size()), 1);
  const auto& loaded_entry = loaded.layers[0].entries[0];
  ASSERT_TRUE(loaded_entry.filter_id.has_value());
  const auto& loaded_filter = loaded.filters[*loaded_entry.filter_id];
  // Reconstructed as a 2-row SoP: row 0 = raypath "3-5", row 1 = raypath "1-3".
  ASSERT_EQ(static_cast<int>(loaded_filter.param.size()), 2);
  ASSERT_STREQ(loaded_filter.param[0].text.c_str(), "3-5");
  ASSERT_STREQ(loaded_filter.param[1].text.c_str(), "1-3");
  // No warning should have fired for a well-formed GUI-emitted complex.
  ASSERT_TRUE(gui::PeekImportComplexFilterWarning().empty());
  // Re-serialize equivalence: the reconstructed filter emits the SAME core
  // filter array (2 simple raypaths + 1 complex).
  const auto j_orig = nlohmann::json::parse(core_json);
  const auto j_reser = nlohmann::json::parse(CoreJson(loaded));
  ASSERT_TRUE(j_orig["filter"] == j_reser["filter"]);
}

// Reject -> reconstruct: an EE
// multi-value (cartesian product) complex filter reconstructs into one EE
// factor per (entry,exit) pair, one row per clause. "3,4" x "5,6" -> 4 rows
// (3,5),(3,6),(4,5),(4,6). Pre-uplift this re-factorized into comma lists;
// the SoP model keeps each pair as its own row (re-serialize byte-equivalent).
TEST(ImportExport, complex_ee_roundtrip) {
  gui::DoNew();
  gui::ClearImportComplexFilterWarning();

  gui::g_state.layers.clear();
  gui::Layer layer;
  layer.probability = 0.0f;
  gui::EntryCard entry;
  gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
  gui::CrystalOf(gui::g_state, entry).height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
  }
  entry.proportion = 100.0f;
  gui::FilterConfig f;
  gui::EntryExitParams ee;
  ee.entry_text = "3,4";
  ee.exit_text = "5,6";
  f.SetEntryExit(ee);
  gui::SetFilter(gui::g_state, entry, f);
  layer.entries.push_back(entry);
  gui::g_state.layers.push_back(layer);

  const std::string core_json = CoreJson(gui::g_state);
  ASSERT_TRUE(!core_json.empty());
  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(core_json, loaded);
  ASSERT_TRUE(ok);

  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  const auto& loaded_entry = loaded.layers[0].entries[0];
  ASSERT_TRUE(loaded_entry.filter_id.has_value());
  const auto& loaded_filter = loaded.filters[*loaded_entry.filter_id];
  // 4 clauses -> 4 rows, each a single EE factor. Serialize order was e outer,
  // x inner: (3,5),(3,6),(4,5),(4,6).
  ASSERT_EQ(static_cast<int>(loaded_filter.param.size()), 4);
  ASSERT_STREQ(loaded_filter.param[0].text.c_str(), "entry:3 & exit:5");
  ASSERT_STREQ(loaded_filter.param[1].text.c_str(), "entry:3 & exit:6");
  ASSERT_STREQ(loaded_filter.param[2].text.c_str(), "entry:4 & exit:5");
  ASSERT_STREQ(loaded_filter.param[3].text.c_str(), "entry:4 & exit:6");
  ASSERT_TRUE(gui::PeekImportComplexFilterWarning().empty());
  const auto j_orig = nlohmann::json::parse(core_json);
  const auto j_reser = nlohmann::json::parse(CoreJson(loaded));
  ASSERT_TRUE(j_orig["filter"] == j_reser["filter"]);
}

// Reject -> reconstruct: a
// wildcard entry (empty text) crossed with multiple exits serializes to a
// complex filter (pair_count = 1 x N > 1). Reconstruct decodes the absent
// "entry" field back to an empty (wildcard) string per row — exercising
// DecodeEEFaceFromJson's absent-field branch. Two clauses -> two rows.
TEST(ImportExport, complex_ee_wildcard_roundtrip) {
  gui::DoNew();
  gui::ClearImportComplexFilterWarning();

  gui::g_state.layers.clear();
  gui::Layer layer;
  layer.probability = 0.0f;
  gui::EntryCard entry;
  gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
  gui::CrystalOf(gui::g_state, entry).height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
  }
  entry.proportion = 100.0f;
  gui::FilterConfig f;
  gui::EntryExitParams ee;
  ee.entry_text = "";  // wildcard entry
  ee.exit_text = "5,6";
  f.SetEntryExit(ee);
  gui::SetFilter(gui::g_state, entry, f);
  layer.entries.push_back(entry);
  gui::g_state.layers.push_back(layer);

  const std::string core_json = CoreJson(gui::g_state);
  ASSERT_TRUE(!core_json.empty());
  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(core_json, loaded);
  ASSERT_TRUE(ok);

  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  const auto& loaded_entry = loaded.layers[0].entries[0];
  ASSERT_TRUE(loaded_entry.filter_id.has_value());
  const auto& loaded_filter = loaded.filters[*loaded_entry.filter_id];
  // 2 clauses -> 2 rows; entry omitted (wildcard) so the canonical text is
  // "entry: & exit:<N>" (empty entry_text after the "entry:" anchor).
  ASSERT_EQ(static_cast<int>(loaded_filter.param.size()), 2);
  ASSERT_STREQ(loaded_filter.param[0].text.c_str(), "entry: & exit:5");
  ASSERT_STREQ(loaded_filter.param[1].text.c_str(), "entry: & exit:6");
  ASSERT_TRUE(gui::PeekImportComplexFilterWarning().empty());
  const auto j_orig = nlohmann::json::parse(core_json);
  const auto j_reser = nlohmann::json::parse(CoreJson(loaded));
  ASSERT_TRUE(j_orig["filter"] == j_reser["filter"]);
}

// Reject -> RECONSTRUCT: a true
// AND-of-products complex filter (a composition product with >1 child id,
// expressing AND) IS now representable — it reconstructs into a single OR row
// whose factors are the ANDed children. Pre-uplift this was loudly rejected;
// the SoP model makes it a first-class reconstruct (warning empty).
TEST(ImportExport, and_of_products_complex_reconstructs) {
  gui::DoNew();
  gui::ClearImportComplexFilterWarning();

  // Hand-authored core JSON: 2 raypath simples + 1 complex with an
  // AND-of-products composition ([[1,2]] — id 1 AND id 2 in the same clause).
  const std::string core_json = R"({
    "crystal": [
      {"id": 1, "type": "Prism", "height": 1.0,
       "face_distance": [1,1,1,1,1,1]}
    ],
    "filter": [
      {"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
      {"id": 2, "type": "raypath", "action": "filter_in", "raypath": [1, 3]},
      {"id": 3, "type": "complex", "action": "filter_in",
       "composition": [[1, 2]]}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5},
      "ray_num": 1000,
      "max_hits": 8,
      "scattering": [
        {"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 3}]}
      ]
    }
  })";

  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(core_json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  const auto& loaded_entry = loaded.layers[0].entries[0];
  // The complex reconstructs — the entry references it.
  ASSERT_TRUE(loaded_entry.filter_id.has_value());
  const auto& lf = loaded.filters[*loaded_entry.filter_id];
  // One clause with two raypath terms -> one row with two raypath factors.
  ASSERT_EQ(static_cast<int>(lf.param.size()), 1);
  ASSERT_EQ(static_cast<int>(lf.param[0].factors.size()), 2);
  ASSERT_TRUE(std::holds_alternative<gui::RaypathParams>(lf.param[0].factors[0]));
  ASSERT_TRUE(std::holds_alternative<gui::RaypathParams>(lf.param[0].factors[1]));
  ASSERT_STREQ(lf.param[0].text.c_str(), "3-5 & 1-3");
  // No warning — fully representable now.
  ASSERT_TRUE(gui::PeekImportComplexFilterWarning().empty());
  // Re-serialize equivalence: back to the same 2 raypath + 1 complex form. Compared
  // against an explicit expectation rather than the hand-authored input, because the
  // emitter owns the id space (Scene-assigned, 0-based, so the round-trip renumbers 1..3
  // to 0..2) and always writes a `symmetry` key (empty when no bits) where the input
  // omitted it. The structural claim — two raypath children AND-ed by one complex, in
  // order — is asserted directly.
  const auto j_reser = nlohmann::json::parse(CoreJson(loaded));
  const nlohmann::json expected_filters = {
    { { "id", 0 }, { "type", "raypath" }, { "action", "filter_in" }, { "symmetry", "" }, { "raypath", { 3, 5 } } },
    { { "id", 1 }, { "type", "raypath" }, { "action", "filter_in" }, { "symmetry", "" }, { "raypath", { 1, 3 } } },
    { { "id", 2 },
      { "type", "complex" },
      { "action", "filter_in" },
      { "symmetry", "" },
      { "composition", { { 0, 1 } } } },  // one clause, two AND terms -> nested array
  };
  ASSERT_TRUE(j_reser["filter"] == expected_filters);
}

// Filter reconstruct, empty-raypath co-existence:
// an empty `raypath:[]` term (match-all wildcard) co-existing with a non-empty
// raypath term as separate OR clauses of the SAME complex filter. Behavior is
// already correct (TryReconstructComplexFilter's raypath branch treats an
// empty/absent array as the match-all wildcard factor) — this pins it with a
// regression test so future reconstruct-logic changes can't silently break it.
TEST(ImportExport, complex_match_all_and_nonempty_raypath_coexist) {
  gui::DoNew();
  gui::ClearImportComplexFilterWarning();

  // Hand-authored core JSON: 2 raypath simples (one match-all via empty
  // `raypath: []`, one non-empty) OR'd via a 2-clause composition.
  const std::string core_json = R"({
    "crystal": [
      {"id": 1, "type": "Prism", "height": 1.0,
       "face_distance": [1,1,1,1,1,1]}
    ],
    "filter": [
      {"id": 1, "type": "raypath", "action": "filter_in", "raypath": []},
      {"id": 2, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
      {"id": 3, "type": "complex", "action": "filter_in",
       "composition": [[1], [2]]}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5},
      "ray_num": 1000,
      "max_hits": 8,
      "scattering": [
        {"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 3}]}
      ]
    }
  })";

  gui::GuiState loaded = gui::InitDefaultState();
  bool ok = gui::DeserializeFromJson(core_json, loaded);
  ASSERT_TRUE(ok);
  ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
  const auto& loaded_entry = loaded.layers[0].entries[0];
  // The complex reconstructs — the entry references it.
  ASSERT_TRUE(loaded_entry.filter_id.has_value());
  const auto& lf = loaded.filters[*loaded_entry.filter_id];
  // 2 clauses -> 2 rows: row 0 = match-all wildcard (empty text), row 1 = "3-5".
  ASSERT_EQ(static_cast<int>(lf.param.size()), 2);
  ASSERT_EQ(static_cast<int>(lf.param[0].factors.size()), 1);
  ASSERT_TRUE(std::holds_alternative<gui::RaypathParams>(lf.param[0].factors[0]));
  ASSERT_TRUE(std::get<gui::RaypathParams>(lf.param[0].factors[0]).raypath_text.empty());
  ASSERT_STREQ(lf.param[0].text.c_str(), "");
  ASSERT_STREQ(lf.param[1].text.c_str(), "3-5");
  // Legal form (match-all is a first-class wildcard factor) — no warning.
  ASSERT_TRUE(gui::PeekImportComplexFilterWarning().empty());
  // Re-serialize equivalence: back to the same 2 raypath + 1 complex form (explicit
  // expectation — see the sibling test above for why the hand-authored input is not the
  // comparison target). Each clause has a single term, so the composition uses the core's
  // bare-id form.
  const auto j_reser = nlohmann::json::parse(CoreJson(loaded));
  const nlohmann::json expected_filters = {
    { { "id", 0 },
      { "type", "raypath" },
      { "action", "filter_in" },
      { "symmetry", "" },
      { "raypath", nlohmann::json::array() } },
    { { "id", 1 }, { "type", "raypath" }, { "action", "filter_in" }, { "symmetry", "" }, { "raypath", { 3, 5 } } },
    { { "id", 2 },
      { "type", "complex" },
      { "action", "filter_in" },
      { "symmetry", "" },
      { "composition", { 0, 1 } } },  // two single-term clauses -> bare ids
  };
  ASSERT_TRUE(j_reser["filter"] == expected_filters);
}

// Genuinely non-representable complex inputs
// still loudly reject (GUI `Factor` has only raypath / entry_exit arms). These
// KEEP the anti-silent-miscull contract for the unsupported cases:
//   (a) child simple type not in {raypath, entry_exit} (e.g. "direction")
//   (b) a term id that points to another complex filter (nested complex)
//   (c) a term id with no matching child in the pool (dangling reference)
TEST(ImportExport, unsupported_complex_still_rejects) {
  // Drives DeserializeFromJson with a hand-authored core JSON, asserts the
  // referenced complex filter is NOT materialized and a warning fires.
  auto expect_reject = [](const std::string& core_json, int entry_filter_id) {
    (void)entry_filter_id;
    gui::DoNew();
    gui::ClearImportComplexFilterWarning();
    gui::GuiState loaded = gui::InitDefaultState();
    bool ok = gui::DeserializeFromJson(core_json, loaded);
    ASSERT_TRUE(ok);
    ASSERT_EQ(static_cast<int>(loaded.layers.size()), 1);
    const auto& e = loaded.layers[0].entries[0];
    ASSERT_TRUE(!e.filter_id.has_value());  // unsupported complex not materialized
    ASSERT_TRUE(!gui::PeekImportComplexFilterWarning().empty());
    gui::ClearImportComplexFilterWarning();
  };

  // (a) child type "direction" — unknown to the GUI Factor variant.
  expect_reject(R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
    "filter": [
      {"id": 1, "type": "direction", "action": "filter_in", "az": 30.0, "el": 15.0},
      {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 2}]}]
    }
  })",
                2);

  // (b) nested complex: a term id points to another complex filter.
  expect_reject(R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
    "filter": [
      {"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
      {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]},
      {"id": 3, "type": "complex", "action": "filter_in", "composition": [[2]]}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 3}]}]
    }
  })",
                3);

  // (c) dangling reference: term id has no matching child in the pool.
  expect_reject(R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
    "filter": [
      {"id": 1, "type": "complex", "action": "filter_in", "composition": [[99]]}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 1}]}]
    }
  })",
                1);

  // (d) empty clause: a composition product with no terms (pins the explicit
  // product.empty() reject added in TryReconstructComplexFilter).
  expect_reject(R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
    "filter": [
      {"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
      {"id": 2, "type": "complex", "action": "filter_in", "composition": [[]]}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 2}]}]
    }
  })",
                2);

  // (e) child filter with a MISSING "type" field — malformed core-JSON. Must loud-reject,
  // NOT silently rebuild as a match-all raypath.
  expect_reject(R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
    "filter": [
      {"id": 1, "action": "filter_in", "raypath": [3, 5]},
      {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]}
    ],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 2}]}]
    }
  })",
                2);
}

// 327.4 cross-check, migrated to the handle API (399.5). The original guarded two hand-written
// expansion twins (GUI -> JSON vs GUI -> C struct) against drift; that class of drift is now
// structurally impossible — the GUI has ONE emitter (BuildScene) and the export JSON is that
// scene serialized. What remains genuinely checkable, and is what this test now asserts, is
// that a filter expansion survives the JSON round-trip losslessly: BuildScene -> SceneToJson
// -> SceneFromJson -> SceneToJson must reproduce the same document. That covers the same
// failure mode from the surviving direction — an expansion the GUI can build but the core
// reader cannot faithfully parse back (bad composition shape, dropped term, id mismatch).
TEST(ImportExport, filter_expand_struct_vs_json) {
  auto run_cross_check = [](const gui::FilterConfig& f) {
    gui::g_state.filters.clear();
    gui::g_state.layers.clear();
    gui::g_state.filters.push_back(f);
    gui::Layer layer;
    layer.probability = 1.0f;
    gui::EntryCard e;
    e.crystal_id = 0;
    e.filter_id = 0;
    e.proportion = 100.0f;
    layer.entries.push_back(e);
    gui::g_state.layers.push_back(layer);

    gui::ScenePtr built = gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit);
    ASSERT_TRUE(built != nullptr);
    const nlohmann::json a_json = SceneJson(built.get());

    // Re-parse the emitted document through the core reader and re-emit.
    const std::string json_str = a_json.dump();
    LUMICE_Scene* reparsed_raw = nullptr;
    ASSERT_EQ(LUMICE_SceneFromJson(json_str.c_str(), &reparsed_raw), LUMICE_OK);
    gui::ScenePtr reparsed(reparsed_raw);
    const nlohmann::json b_json = SceneJson(reparsed.get());

    // Filters (identity + per-type arm fields + composition shape) must survive verbatim.
    ASSERT_EQ(a_json["filter"].size(), b_json["filter"].size());
    ASSERT_TRUE(a_json["filter"] == b_json["filter"]);
    // ... and so must the scattering entries that reference them by id.
    ASSERT_TRUE(a_json["scene"]["scattering"] == b_json["scene"]["scattering"]);
  };

  auto fresh_state = []() {
    gui::DoNew();
    gui::g_state.crystals.clear();
    gui::g_state.filters.clear();
    gui::g_state.layers.clear();
    gui::CrystalConfig c;
    c.type = gui::CrystalType::kPrism;
    c.height = 1.0f;
    for (int i = 0; i < 6; ++i) {
      c.face_distance[i] = 1.0f;
    }
    gui::g_state.crystals.push_back(c);
  };

  // Degenerate / type-internal-OR cases (AC4 byte-equivalence guard): built via
  // the compat SetRaypath / SetEntryExit writers, exactly as pre-uplift.
  auto cross_check = [&](gui::Factor param) {
    fresh_state();
    gui::FilterConfig f;
    if (std::holds_alternative<gui::RaypathParams>(param)) {
      f.SetRaypath(std::get<gui::RaypathParams>(std::move(param)));
    } else {
      f.SetEntryExit(std::get<gui::EntryExitParams>(std::move(param)));
    }
    run_cross_check(f);
  };

  // Full sum-of-products cases (new capability): each canonical summand text
  // becomes an OR row; ParseSummandText builds the AND-of-factors parse cache.
  auto cross_check_sop = [&](const std::vector<std::string>& rows) {
    fresh_state();
    gui::FilterConfig f;
    gui::SumOfProducts sop;
    for (const auto& row : rows) {
      sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
    }
    f.param = std::move(sop);
    run_cross_check(f);
  };

  cross_check(gui::RaypathParams{ "3-1-5" });          // single-segment -> 1 simple raypath
  cross_check(gui::RaypathParams{ "3-5; 1-4; 2-6" });  // multi-segment -> 3 simple + 1 complex
  cross_check(gui::EntryExitParams{ "3", "5" });       // single pair -> 1 simple EE
  gui::EntryExitParams ee_multi{ "3,5", "1" };         // 2x1 -> 2 simple + 1 complex
  ee_multi.length_mode = 3;
  ee_multi.min_len = 2;
  ee_multi.max_len = 6;
  cross_check(ee_multi);

  // Wildcard EE matrix: empty entry/exit strings map to LUMICE_EE_WILDCARD_SENTINEL and
  // must survive struct↔JSON cross-check on both single and multi-value sides.
  cross_check(gui::EntryExitParams{ "", "" });     // both wildcard -> 1 simple EE
  cross_check(gui::EntryExitParams{ "", "5" });    // entry wildcard + single exit -> 1 simple EE
  cross_check(gui::EntryExitParams{ "3", "" });    // single entry + exit wildcard -> 1 simple EE
  cross_check(gui::EntryExitParams{ "3,5", "" });  // 2 entry values × wildcard exit -> 2 simple + 1 complex
  cross_check(gui::EntryExitParams{ "", "3,5" });  // wildcard entry × 2 exit values -> 2 simple + 1 complex

  // Sum-of-products cases (T1):
  // 1. cross-type OR: raypath row + EE row -> 2 clauses, 1 term each, mixed type.
  cross_check_sop({ "3-1-5", "entry:3 & exit:5" });
  // 2. AND clause: one row with an EE factor AND a raypath factor -> 1 clause, 2 terms.
  cross_check_sop({ "entry:3 & 7-1" });
  // 3. cross-type OR + AND + internal multi-value (Cartesian distribution):
  //    "entry:3,4 & 7-1" -> 2 clauses (EE3&rp, EE4&rp) each 2 terms; "2-6" -> +1 clause.
  cross_check_sop({ "entry:3,4 & 7-1", "2-6" });

  // Overflow: a raypath with more than LUMICE_MAX_CONFIG_CLAUSES OR segments exceeds the
  // clause ABI bound -> BuildScene must return nullptr (graceful degradation), so app.cpp
  // keeps the prior committed state instead of committing a truncated config.
  {
    gui::DoNew();
    gui::g_state.crystals.clear();
    gui::g_state.filters.clear();
    gui::g_state.layers.clear();
    gui::CrystalConfig c;
    c.type = gui::CrystalType::kPrism;
    c.height = 1.0f;
    for (int i = 0; i < 6; ++i) {
      c.face_distance[i] = 1.0f;
    }
    gui::g_state.crystals.push_back(c);
    std::string too_many_segments;  // LUMICE_MAX_CONFIG_CLAUSES + 1 OR segments
    for (int i = 0; i < LUMICE_MAX_CONFIG_CLAUSES + 1; ++i) {
      if (i) {
        too_many_segments += ";";
      }
      too_many_segments += "3-5";
    }
    gui::FilterConfig f;
    f.name = "OverflowFilter";  // named so overflow.filter_name assertion below is meaningful
    f.SetRaypath(gui::RaypathParams{ too_many_segments });
    gui::g_state.filters.push_back(f);
    gui::Layer layer;
    layer.probability = 1.0f;
    gui::EntryCard e;
    e.crystal_id = 0;
    e.filter_id = 0;
    e.proportion = 100.0f;
    layer.entries.push_back(e);
    gui::g_state.layers.push_back(layer);
    gui::FilterOverflowInfo overflow;
    gui::ScenePtr over = gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit, &overflow);
    // Over ABI bounds -> no handle at all. The caller therefore cannot commit a half-built
    // scene even by mistake, which is what the pre-handle "no partial writes into the
    // caller's LUMICE_Config" contract was buying.
    ASSERT_TRUE(over == nullptr);
    // Overflow identity: the first (layer 0, entry 0) reference is captured with the
    // FilterConfig::name so the caller can locate the offending filter for the user.
    ASSERT_EQ(overflow.layer_index, 0);
    ASSERT_EQ(overflow.entry_index, 0);
    ASSERT_EQ(overflow.filter_name, std::string("OverflowFilter"));
    // The locator string DoRun embeds in the modal + Log message (named filter form).
    ASSERT_STREQ(gui::FormatOverflowLocator(overflow).c_str(), "filter \"OverflowFilter\", Layer 1 / Entry 1");
  }

  // Overflow (new trigger): a single OR row
  // with more than LUMICE_MAX_CONFIG_TERMS AND factors exceeds the per-clause
  // term ABI bound. Pre-uplift this was unreachable (clauses were always
  // singletons); a multi-factor AND row can now hit it. Must return false with
  // no partial write.
  {
    gui::DoNew();
    gui::g_state.crystals.clear();
    gui::g_state.filters.clear();
    gui::g_state.layers.clear();
    gui::CrystalConfig c;
    c.type = gui::CrystalType::kPrism;
    c.height = 1.0f;
    for (int i = 0; i < 6; ++i) {
      c.face_distance[i] = 1.0f;
    }
    gui::g_state.crystals.push_back(c);
    // LUMICE_MAX_CONFIG_TERMS + 1 raypath factors ANDed together in one row.
    std::string too_many_terms;
    for (int i = 0; i < LUMICE_MAX_CONFIG_TERMS + 1; ++i) {
      if (i) {
        too_many_terms += " & ";
      }
      too_many_terms += std::to_string(i + 1) + "-" + std::to_string(i + 2);
    }
    gui::FilterConfig f;
    f.param = gui::SumOfProducts{ gui::SummandText{ too_many_terms, gui::ParseSummandText(too_many_terms) } };
    gui::g_state.filters.push_back(f);
    gui::Layer layer;
    layer.probability = 1.0f;
    gui::EntryCard e;
    e.crystal_id = 0;
    e.filter_id = 0;
    e.proportion = 100.0f;
    layer.entries.push_back(e);
    gui::g_state.layers.push_back(layer);
    // term count over LUMICE_MAX_CONFIG_TERMS -> no handle produced.
    ASSERT_TRUE(gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit) == nullptr);
  }

  // Overflow: the cross-factor Cartesian is capped
  // BEFORE materialization. Multiple multi-alternative factors whose product
  // exceeds LUMICE_MAX_CONFIG_CLAUSES must be rejected gracefully without ever
  // building the (potentially exponential) clause tree.
  {
    gui::DoNew();
    gui::g_state.crystals.clear();
    gui::g_state.filters.clear();
    gui::g_state.layers.clear();
    gui::CrystalConfig c;
    c.type = gui::CrystalType::kPrism;
    c.height = 1.0f;
    for (int i = 0; i < 6; ++i) {
      c.face_distance[i] = 1.0f;
    }
    gui::g_state.crystals.push_back(c);
    // Four raypath factors ANDed, each carrying 9 ';'-OR alternatives:
    // 9 * 9 * 9 * 9 = 6561 would-be clauses > LUMICE_MAX_CONFIG_CLAUSES(4096), still under
    // the per-clause factor cap (4 factors <= LUMICE_MAX_CONFIG_TERMS=64) — so this
    // exercises the clause-product path, not the term-count path. Detection triggers on
    // the 4th expansion attempt (acc.size() 729 * 9 = 6561 > kMaxClauses), so only ~729
    // intermediate 3-element vectors materialize (cheap).
    gui::SummandText row;
    row.text = "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4";
    row.factors = {
      gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
      gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
      gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
      gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    };
    gui::FilterConfig f;
    f.param = gui::SumOfProducts{ row };
    gui::g_state.filters.push_back(f);
    gui::Layer layer;
    layer.probability = 1.0f;
    gui::EntryCard e;
    e.crystal_id = 0;
    e.filter_id = 0;
    e.proportion = 100.0f;
    layer.entries.push_back(e);
    gui::g_state.layers.push_back(layer);
    // Cartesian > LUMICE_MAX_CONFIG_CLAUSES -> no handle produced, and no exponential
    // clause tree materialized on the way there.
    ASSERT_TRUE(gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit) == nullptr);

    // Export behavior on the same over-limit state, tightened relative to the pre-handle
    // code: there is no longer a second, total JSON emitter that degrades an
    // overflowing filter to a bounded match-all stand-in. Export now goes through the one
    // BuildScene, so it REFUSES outright — strictly safer than the old stand-in, which was
    // only kept off disk by DoExportConfigJson pre-checking the commit path.
    ASSERT_TRUE(CoreJson(gui::g_state).empty());
  }
}

// Export overflow rejection: the pure BuildExportJsonOrWarn — the
// logic DoExportConfigJson delegates to — must REFUSE to produce a config for an over-limit
// filter (never silently write a semantically-opposite match-all export) and hand back a
// locator-bearing warning. Pins the critical fix into the regression gate rather than
// relying on a one-time code read.
TEST(ImportExport, export_json_rejects_overflow_filter) {
  gui::DoNew();
  gui::g_state.crystals.clear();
  gui::g_state.filters.clear();
  gui::g_state.layers.clear();
  gui::CrystalConfig c;
  c.type = gui::CrystalType::kPrism;
  c.height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    c.face_distance[i] = 1.0f;
  }
  gui::g_state.crystals.push_back(c);
  // 4 raypath factors x 9 alternatives = 6561 clauses > LUMICE_MAX_CONFIG_CLAUSES(4096).
  gui::SummandText row;
  row.text = "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4";
  row.factors = {
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
  };
  gui::FilterConfig f;
  f.name = "BigFilter";
  f.param = gui::SumOfProducts{ row };
  gui::g_state.filters.push_back(f);
  gui::Layer layer;
  layer.probability = 1.0f;
  gui::EntryCard e;
  e.crystal_id = 0;
  e.filter_id = 0;
  e.proportion = 100.0f;
  layer.entries.push_back(e);
  gui::g_state.layers.push_back(layer);

  // Overflow → refuse: no JSON produced, warning names the offending filter.
  std::string json;
  std::string warning;
  ASSERT_TRUE(!gui::BuildExportJsonOrWarn(gui::g_state, &json, &warning));
  ASSERT_TRUE(json.empty());  // out_json left untouched — nothing to write
  ASSERT_TRUE(!warning.empty());
  ASSERT_TRUE(warning.find("BigFilter") != std::string::npos);  // FormatOverflowLocator names it

  // Sanity: a valid (degenerate) filter exports fine — no warning, JSON produced.
  gui::g_state.filters[0].SetRaypath(gui::RaypathParams{ "3-5" });
  std::string ok_json;
  std::string ok_warning;
  ASSERT_TRUE(gui::BuildExportJsonOrWarn(gui::g_state, &ok_json, &ok_warning));
  ASSERT_TRUE(!ok_json.empty());
  ASSERT_TRUE(ok_warning.empty());
}

// SummarizeSopExpansion delegates
// to the same ExpandSopToClauses as the commit path so the live preview and
// the ABI enforcement point share the exact same arithmetic (no drift).
// Single-row default (blank) collapses to 1 clause, non-overflow; a 6561-
// would-be-clause row (4 × 9-alt) trips the LUMICE_MAX_CONFIG_CLAUSES=4096
// cap and reports overflow with the bounded degenerate stand-in.
TEST(ImportExport, summarize_sop_expansion_delegates_to_commit_path) {
  gui::FilterConfig f_ok;
  f_ok.SetRaypath(gui::RaypathParams{ "3-5" });
  const auto s_ok = gui::SummarizeSopExpansion(f_ok);
  ASSERT_TRUE(!s_ok.overflow);
  ASSERT_EQ(s_ok.clause_count, static_cast<size_t>(1));

  // Multi-alternative single-row: `;`-OR expands but stays well below cap.
  gui::FilterConfig f_multi;
  f_multi.SetRaypath(gui::RaypathParams{ "3-5;1-2" });
  const auto s_multi = gui::SummarizeSopExpansion(f_multi);
  ASSERT_TRUE(!s_multi.overflow);
  ASSERT_EQ(s_multi.clause_count, static_cast<size_t>(2));

  // Overflow: 4 × 9-alt = 6561 > LUMICE_MAX_CONFIG_CLAUSES(4096).
  gui::SummandText row;
  row.text = "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4";
  row.factors = {
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
    gui::Factor{ gui::RaypathParams{ "1;2;3;4;5;6;7;8;3-4" } },
  };
  gui::FilterConfig f_over;
  f_over.param = gui::SumOfProducts{ row };
  const auto s_over = gui::SummarizeSopExpansion(f_over);
  ASSERT_TRUE(s_over.overflow);
  // ExpandSopToClauses replaces the huge tree with a bounded degenerate
  // stand-in (1 clause) on overflow — the summary reflects that.
  ASSERT_EQ(s_over.clause_count, static_cast<size_t>(1));
}

// FormatOverflowLocator format contract (pure function, no GUI): the unnamed form drops the
// filter clause and keeps 1-based Layer/Entry, and the indices are presented as +1.
TEST(ImportExport, overflow_locator_format) {
  gui::FilterOverflowInfo named;
  named.layer_index = 2;
  named.entry_index = 0;
  named.filter_name = "MyFilter";
  ASSERT_STREQ(gui::FormatOverflowLocator(named).c_str(), "filter \"MyFilter\", Layer 3 / Entry 1");
  gui::FilterOverflowInfo unnamed;
  unnamed.layer_index = 0;
  unnamed.entry_index = 4;
  // Empty filter_name -> no "filter \"...\"," clause, just the position.
  ASSERT_STREQ(gui::FormatOverflowLocator(unnamed).c_str(), "Layer 1 / Entry 5");
}

// DoRun's Log-panel dedup gate (`PeekGuiWarning() != warning_msg`): a persistent overflow
// re-detected on every ~70ms debounce commit writes the Log line ONLY on the first detection
// (message unchanged -> gate closed), and re-opens (gate open) when the located filter/layer
// changes or after a successful commit clears the warning. This exercises the exact predicate
// DoRun uses so AC② (Log dedup) has automated coverage rather than only on-screen verification.
TEST(ImportExport, overflow_log_dedup_gate) {
  gui::ClearGuiWarning();
  const std::string msg_a = "This filter has too many OR segments / values to apply (limit " +
                            std::to_string(LUMICE_MAX_CONFIG_CLAUSES) + "; filter \"A\", Layer 1 / Entry 1).\nkept.";
  // First detection: gate open -> would write the Log line.
  ASSERT_TRUE(gui::PeekGuiWarning() != msg_a);
  gui::SetGuiWarning(msg_a);
  // Same overflow re-detected on the next debounce commit: gate closed -> Log NOT re-written.
  ASSERT_TRUE(gui::PeekGuiWarning() == msg_a);
  // User switches which filter overflows (new locator): gate open again -> re-logs.
  const std::string msg_b = "This filter has too many OR segments / values to apply (limit " +
                            std::to_string(LUMICE_MAX_CONFIG_CLAUSES) + "; filter \"B\", Layer 2 / Entry 1).\nkept.";
  ASSERT_TRUE(gui::PeekGuiWarning() != msg_b);
  gui::SetGuiWarning(msg_b);
  // A successful commit clears the warning: the same message would log again afterwards.
  gui::ClearGuiWarning();
  ASSERT_TRUE(gui::PeekGuiWarning() != msg_a);
  gui::ClearGuiWarning();
}

// 327.4 anti-spam: SetGuiWarning is idempotent while the same message is in-flight, so an
// over-bounds filter re-detected on every debounced commit does NOT re-open the modal and
// freeze interaction. ClearGuiWarning (a successful commit) re-arms it.
TEST(ImportExport, gui_warning_dedup) {
  gui::ClearGuiWarning();
  ASSERT_TRUE(gui::PeekGuiWarning().empty());
  gui::SetGuiWarning("over-bounds A");
  ASSERT_STREQ(gui::PeekGuiWarning().c_str(), "over-bounds A");
  gui::SetGuiWarning("over-bounds A");  // same message -> idempotent, no re-open
  ASSERT_STREQ(gui::PeekGuiWarning().c_str(), "over-bounds A");
  gui::SetGuiWarning("over-bounds B");  // different -> updates (would re-open)
  ASSERT_STREQ(gui::PeekGuiWarning().c_str(), "over-bounds B");
  gui::ClearGuiWarning();  // successful commit re-arms
  ASSERT_TRUE(gui::PeekGuiWarning().empty());
  gui::SetGuiWarning("over-bounds A");  // same message after a clear -> warns again
  ASSERT_STREQ(gui::PeekGuiWarning().c_str(), "over-bounds A");
  gui::ClearGuiWarning();
}

// BuildScene raypath_color emission
// ---------------------------------------------------------------------------
// ColorClassConfig (GUI-side) -> LUMICE_ColorClass translation.
// Uses the plain BuildScene no-imgui-interaction pattern shared by the
// sibling overflow / SoP cross-check tests above (no ItemClick / no UI).
TEST(ImportExport, raypath_color_empty_emits_zero_count) {
  gui::DoNew();
  const auto j = CommitSceneJson(gui::g_state);
  // Zero classes -> the whole raypath_color key is absent, keeping the mono/no-color scene
  // byte-identical to the pre-v4.7 wire form (the "count == 0 -> omit" isomorphism). Mode
  // is only meaningful alongside classes, so it is deliberately not emitted here either.
  ASSERT_TRUE(!j.contains("raypath_color"));
  // doc §4.8: GUI default is now painter (asserted on the state, which is where the default
  // lives — with no classes there is nothing for it to be emitted onto).
  ASSERT_EQ(gui::g_state.raypath_color_mode, LUMICE_COLOR_MODE_PAINTER);
}

TEST(ImportExport, raypath_color_single_class_raypath_ref) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  cls.color[0] = 0.8f;
  cls.color[1] = 0.3f;
  cls.color[2] = 0.1f;
  cls.combine = 0;
  cls.visible = true;
  cls.solo = false;
  gui::ColorClassRefConfig ref;
  ref.layer_idx = 0;
  ref.crystal_pool_id = 0;
  ref.match_all = false;
  ref.predicate_text = "3-5-1";
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  const auto& c = classes[0];
  ASSERT_EQ(c["color"][0].get<float>(), 0.8f);
  ASSERT_EQ(c["color"][1].get<float>(), 0.3f);
  ASSERT_EQ(c["color"][2].get<float>(), 0.1f);
  // combine/visible/solo are emitted only when non-default (any / true / false).
  ASSERT_TRUE(!c.contains("combine"));
  ASSERT_TRUE(!c.contains("visible"));
  ASSERT_TRUE(!c.contains("solo"));
  ASSERT_EQ(c["match"].size(), (size_t)1);
  ASSERT_EQ(c["match"][0]["layer"].get<int>(), 0);
  ASSERT_EQ(c["match"][0]["crystal"].get<int>(), 0);  // pool 0 -> scene crystal id 0
  ASSERT_EQ(c["match"][0]["type"].get<std::string>(), std::string("raypath"));
  ASSERT_EQ(c["match"][0]["raypath"], nlohmann::json({ 3, 5, 1 }));
}

TEST(ImportExport, raypath_color_match_all_emits_unset) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = true;
  ref.predicate_text = "3-5";  // ignored when match_all=true
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  ASSERT_EQ(classes[0]["match"].size(), (size_t)1);
  // UNSET (match-all) is encoded by the ABSENCE of a "type" field.
  ASSERT_TRUE(!classes[0]["match"][0].contains("type"));
}

TEST(ImportExport, raypath_color_empty_text_emits_unset) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = false;
  ref.predicate_text = "  ";  // whitespace-only -> match-all
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  ASSERT_TRUE(!classes[0]["match"][0].contains("type"));
}

TEST(ImportExport, raypath_color_entry_exit_predicate) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = false;
  ref.predicate_text = "entry:1 & exit:2";
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  const auto& p = classes[0]["match"][0];
  ASSERT_EQ(p["type"].get<std::string>(), std::string("entry_exit"));
  ASSERT_EQ(p["entry"].get<int>(), 1);
  ASSERT_EQ(p["exit"].get<int>(), 2);
}

TEST(ImportExport, raypath_color_combine_all_two_refs) {
  gui::DoNew();
  // Add a second layer sharing crystal pool 0 (InitDefaultState set up 1 crystal + 1 layer).
  gui::Layer layer1;
  gui::EntryCard e;
  e.crystal_id = 0;
  layer1.entries.push_back(e);
  gui::g_state.layers.push_back(layer1);

  gui::ColorClassConfig cls;
  cls.combine = 1;  // ALL
  gui::ColorClassRefConfig r0;
  r0.layer_idx = 0;
  r0.crystal_pool_id = 0;
  r0.match_all = false;
  r0.predicate_text = "3-5";
  gui::ColorClassRefConfig r1;
  r1.layer_idx = 1;
  r1.crystal_pool_id = 0;
  r1.match_all = false;
  r1.predicate_text = "1-3";
  cls.match.push_back(r0);
  cls.match.push_back(r1);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  ASSERT_EQ(classes[0]["combine"].get<std::string>(), std::string("all"));
  ASSERT_EQ(classes[0]["match"].size(), (size_t)2);
  ASSERT_EQ(classes[0]["match"][0]["layer"].get<int>(), 0);
  ASSERT_EQ(classes[0]["match"][1]["layer"].get<int>(), 1);
}

TEST(ImportExport, raypath_color_display_flags_and_mode) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  cls.visible = false;
  cls.solo = true;
  gui::ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = true;
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);
  gui::g_state.raypath_color_mode = LUMICE_COLOR_MODE_ADDITIVE;

  const auto j = CommitSceneJson(gui::g_state);
  ASSERT_EQ(j["raypath_color"]["classes"][0]["visible"].get<bool>(), false);
  ASSERT_EQ(j["raypath_color"]["classes"][0]["solo"].get<bool>(), true);
  ASSERT_EQ(j["raypath_color"]["mode"].get<std::string>(), std::string("additive"));
}

TEST(ImportExport, raypath_color_orphan_ref_skipped) {
  gui::DoNew();
  // Add a second crystal to the pool but do NOT reference it in any scattering
  // entry — pool id 1 becomes orphaned in crystal_pool_to_core.
  gui::CrystalConfig c2;
  c2.type = gui::CrystalType::kPrism;
  gui::g_state.crystals.push_back(c2);

  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig r_orphan;
  r_orphan.crystal_pool_id = 1;
  r_orphan.match_all = true;
  gui::ColorClassRefConfig r_ok;
  r_ok.crystal_pool_id = 0;
  r_ok.match_all = true;
  cls.match.push_back(r_orphan);
  cls.match.push_back(r_ok);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  ASSERT_EQ(classes[0]["match"].size(), (size_t)1);  // orphan dropped
  ASSERT_EQ(classes[0]["match"][0]["crystal"].get<int>(), 0);
}

TEST(ImportExport, raypath_color_multi_factor_predicate_skipped) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig r_bad;
  r_bad.crystal_pool_id = 0;
  r_bad.match_all = false;
  r_bad.predicate_text = "3-5 & entry:2";  // 2 factors -- not expressible as a single atom
  gui::ColorClassRefConfig r_ok;
  r_ok.crystal_pool_id = 0;
  r_ok.match_all = true;
  cls.match.push_back(r_bad);
  cls.match.push_back(r_ok);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  ASSERT_EQ(classes[0]["match"].size(), (size_t)1);  // bad-predicate ref dropped
  ASSERT_TRUE(!classes[0]["match"][0].contains("type"));
}

TEST(ImportExport, raypath_color_multi_alt_raypath_skipped) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig r_bad;
  r_bad.crystal_pool_id = 0;
  r_bad.match_all = false;
  r_bad.predicate_text = "1-3;5-7";  // ';' OR alternatives -- 1 Factor but multiple alternatives
  cls.match.push_back(r_bad);
  gui::g_state.raypath_color.push_back(cls);

  const auto j = CommitSceneJson(gui::g_state);
  const auto& classes = j["raypath_color"]["classes"];
  ASSERT_EQ(classes.size(), (size_t)1);
  ASSERT_EQ(classes[0]["match"].size(), (size_t)0);
}

TEST(ImportExport, raypath_color_class_over_cap) {
  gui::DoNew();
  for (int i = 0; i < LUMICE_MAX_CONFIG_COLOR_CLASSES + 1; i++) {
    gui::ColorClassConfig cls;
    gui::ColorClassRefConfig ref;
    ref.crystal_pool_id = 0;
    ref.match_all = true;
    cls.match.push_back(ref);
    gui::g_state.raypath_color.push_back(cls);
  }
  gui::ColorClassOverflowInfo color_overflow;
  ASSERT_TRUE(gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit, nullptr, &color_overflow) == nullptr);
  ASSERT_TRUE(color_overflow.class_over_cap);
  ASSERT_EQ(color_overflow.class_index, LUMICE_MAX_CONFIG_COLOR_CLASSES);
}

TEST(ImportExport, raypath_color_ref_over_cap) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  for (int j = 0; j < LUMICE_MAX_CONFIG_COLOR_REFS + 1; j++) {
    gui::ColorClassRefConfig ref;
    ref.crystal_pool_id = 0;
    ref.match_all = true;
    cls.match.push_back(ref);
  }
  gui::g_state.raypath_color.push_back(cls);
  gui::ColorClassOverflowInfo color_overflow;
  ASSERT_TRUE(gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit, nullptr, &color_overflow) == nullptr);
  ASSERT_TRUE(!color_overflow.class_over_cap);
  ASSERT_EQ(color_overflow.class_index, 0);
  ASSERT_EQ(color_overflow.ref_index, LUMICE_MAX_CONFIG_COLOR_REFS);
}

// Export-JSON / DeserializeFromJson raypath_color roundtrip.
// ---------------------------------------------------------------------------
// These tests exercise the export path (BuildExportJsonOrWarn). Where the original suite
// cross-checked the JSON emitter against a separate struct emitter, the two are now the same
// scene under two SceneIntents — so the cross-checks below assert the intents agree, which is
// the property that can still break (a field accidentally made intent-dependent). The one
// intended difference, renderer intensity_factor, has its own dedicated test
// (intensity_factor_ignores_exposure_offset_in_gui_run_path).
TEST(ImportExport, raypath_color_serialize_omits_when_empty) {
  gui::DoNew();
  // Empty raypath_color → JSON must not contain a "raypath_color" key so mono-only
  // configs remain byte-identical with the pre-v4.7 shape (zero-regression contract).
  const std::string js = CoreJson(gui::g_state);
  auto j = nlohmann::json::parse(js);
  ASSERT_TRUE(!j.contains("raypath_color"));
}

TEST(ImportExport, raypath_color_serialize_object_shape) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  cls.color[0] = 1.0f;
  cls.color[1] = 0.0f;
  cls.color[2] = 0.0f;
  gui::ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = false;
  ref.predicate_text = "3-5";
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);
  gui::g_state.raypath_color_mode = LUMICE_COLOR_MODE_ADDITIVE;

  auto j = nlohmann::json::parse(CoreJson(gui::g_state));
  ASSERT_TRUE(j.contains("raypath_color"));
  const auto& jrc = j["raypath_color"];
  ASSERT_TRUE(jrc.is_object());
  ASSERT_STREQ(jrc.value("mode", std::string{}).c_str(), "additive");
  ASSERT_TRUE(jrc.contains("classes") && jrc["classes"].is_array());
  ASSERT_EQ(static_cast<int>(jrc["classes"].size()), 1);
  const auto& jc = jrc["classes"][0];
  ASSERT_TRUE(jc["color"].is_array() && jc["color"].size() == 3);
  ASSERT_EQ(jc["color"][0].get<float>(), 1.0f);
  // "combine" and "visible" omitted at wire (default any / true).
  ASSERT_TRUE(!jc.contains("combine"));
  ASSERT_TRUE(!jc.contains("visible"));
  ASSERT_TRUE(!jc.contains("solo"));
  ASSERT_TRUE(jc["match"].is_array() && jc["match"].size() == 1);
  const auto& jr = jc["match"][0];
  ASSERT_EQ(jr["layer"].get<int>(), 0);
  ASSERT_EQ(jr["crystal"].get<int>(), 0);  // pool 0 -> scene crystal id 0
  ASSERT_STREQ(jr["type"].get<std::string>().c_str(), "raypath");
  ASSERT_TRUE(jr["raypath"].is_array() && jr["raypath"].size() == 2);
  ASSERT_EQ(jr["raypath"][0].get<int>(), 3);
  ASSERT_EQ(jr["raypath"][1].get<int>(), 5);

  // Default symmetry (kSymNone) MUST omit the "symmetry" key
  // (backward compat with pre-v4.9 project files that have no such field).
  ASSERT_TRUE(!jr.contains("symmetry"));
}

// Non-default symmetry surfaces as a "PBD"-subset string, and
// the struct-side LUMICE_ColorPredicate.symmetry bitmask matches. Guards
// against Emit/Fill divergence and against forgetting to write the field.
TEST(ImportExport, raypath_color_json_symmetry_non_default_pd) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  cls.color[0] = 0.1f;
  gui::ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = false;
  ref.predicate_text = "3-5";
  ref.sym_p = true;
  ref.sym_d = true;  // sym_b left false — combination is "PD" (bits 1|4 == 5).
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);

  // The commit-intent scene carries the symmetry string...
  const auto commit_j = CommitSceneJson(gui::g_state);
  ASSERT_STREQ(commit_j["raypath_color"]["classes"][0]["match"][0]["symmetry"].get<std::string>().c_str(), "PD");

  // ...and so does the exported JSON.
  auto j = nlohmann::json::parse(CoreJson(gui::g_state));
  const auto& jr = j["raypath_color"]["classes"][0]["match"][0];
  ASSERT_TRUE(jr.contains("symmetry"));
  ASSERT_STREQ(jr["symmetry"].get<std::string>().c_str(), "PD");
}

// Cross-check: JSON emit vs struct emit produce the same predicate (RAYPATH branch).
TEST(ImportExport, raypath_color_json_vs_struct_raypath) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  cls.color[0] = 0.5f;
  cls.color[1] = 0.6f;
  cls.color[2] = 0.7f;
  cls.combine = LUMICE_COLOR_COMBINE_ALL;
  cls.visible = false;
  cls.solo = true;
  gui::ColorClassRefConfig ref;
  ref.crystal_pool_id = 0;
  ref.match_all = false;
  ref.predicate_text = "1-2-3-4";
  cls.match.push_back(ref);
  gui::g_state.raypath_color.push_back(cls);

  // Commit-intent scene vs export-intent JSON.
  const auto commit_j = CommitSceneJson(gui::g_state);
  auto j = nlohmann::json::parse(CoreJson(gui::g_state));
  const auto& jr = j["raypath_color"]["classes"][0]["match"][0];

  // Class-level fields land as expected...
  ASSERT_STREQ(j["raypath_color"]["classes"][0]["combine"].get<std::string>().c_str(), "all");
  ASSERT_EQ(j["raypath_color"]["classes"][0]["visible"].get<bool>(), false);
  ASSERT_EQ(j["raypath_color"]["classes"][0]["solo"].get<bool>(), true);
  ASSERT_STREQ(jr["type"].get<std::string>().c_str(), "raypath");
  ASSERT_EQ(jr["raypath"], nlohmann::json({ 1, 2, 3, 4 }));
  // ...and the entire color block is intent-independent.
  ASSERT_TRUE(commit_j["raypath_color"] == j["raypath_color"]);
}

// Cross-check: JSON emit vs struct emit for entry_exit predicate + match-all.
TEST(ImportExport, raypath_color_json_vs_struct_entry_exit_and_match_all) {
  gui::DoNew();
  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig ee_ref;
  ee_ref.crystal_pool_id = 0;
  ee_ref.match_all = false;
  ee_ref.predicate_text = "entry:1 & exit:2";
  gui::ColorClassRefConfig all_ref;
  all_ref.crystal_pool_id = 0;
  all_ref.match_all = true;
  cls.match.push_back(ee_ref);
  cls.match.push_back(all_ref);
  gui::g_state.raypath_color.push_back(cls);

  const auto commit_j = CommitSceneJson(gui::g_state);
  auto j = nlohmann::json::parse(CoreJson(gui::g_state));
  const auto& jrefs = j["raypath_color"]["classes"][0]["match"];
  ASSERT_EQ(static_cast<int>(commit_j["raypath_color"]["classes"][0]["match"].size()), 2);
  ASSERT_EQ(static_cast<int>(jrefs.size()), 2);

  // EE ref.
  ASSERT_STREQ(jrefs[0]["type"].get<std::string>().c_str(), "entry_exit");
  ASSERT_EQ(jrefs[0].value("entry", -1), 1);
  ASSERT_EQ(jrefs[0].value("exit", -1), 2);

  // Match-all ref: UNSET is encoded as the absence of a "type" field.
  ASSERT_TRUE(!jrefs[1].contains("type"));

  // Both intents produce the same color block.
  ASSERT_TRUE(commit_j["raypath_color"] == j["raypath_color"]);
}

// Orphan / multi-factor refs are SKIPPED in JSON emit — matches the struct emit gate.
TEST(ImportExport, raypath_color_serialize_skips_orphan_and_multi_factor) {
  gui::DoNew();
  // Add an unreferenced crystal to pool → orphan pool id 1.
  gui::CrystalConfig c2;
  c2.type = gui::CrystalType::kPrism;
  gui::g_state.crystals.push_back(c2);

  gui::ColorClassConfig cls;
  gui::ColorClassRefConfig orphan;
  orphan.crystal_pool_id = 1;
  orphan.match_all = true;
  gui::ColorClassRefConfig multi_factor;
  multi_factor.crystal_pool_id = 0;
  multi_factor.match_all = false;
  multi_factor.predicate_text = "3-5 & entry:2";
  gui::ColorClassRefConfig ok;
  ok.crystal_pool_id = 0;
  ok.match_all = true;
  cls.match.push_back(orphan);
  cls.match.push_back(multi_factor);
  cls.match.push_back(ok);
  gui::g_state.raypath_color.push_back(cls);

  auto j = nlohmann::json::parse(CoreJson(gui::g_state));
  const auto& jrefs = j["raypath_color"]["classes"][0]["match"];
  ASSERT_EQ(static_cast<int>(jrefs.size()), 1);
  ASSERT_EQ(jrefs[0]["crystal"].get<int>(), 0);
  ASSERT_TRUE(!jrefs[0].contains("type"));  // ok was match_all
}

// GuiState → JSON → GuiState roundtrip preserves raypath_color content (via operator==).
TEST(ImportExport, raypath_color_json_roundtrip) {
  gui::DoNew();
  gui::ColorClassConfig cls_a;
  cls_a.color[0] = 1.0f;
  cls_a.color[1] = 0.2f;
  cls_a.color[2] = 0.3f;
  cls_a.combine = LUMICE_COLOR_COMBINE_ANY;
  cls_a.visible = true;
  cls_a.solo = false;
  cls_a.z_order = 0;  // aligns with physical index — matches load-time default.
  gui::ColorClassRefConfig ra;
  ra.layer_idx = 0;
  ra.crystal_pool_id = 0;
  ra.match_all = false;
  ra.predicate_text = "3-5-1";
  // Non-default symmetry on ra so the operator== round-trip
  // below naturally covers the new fields (catches "wrote Emit but forgot
  // Deserialize" or vice-versa).
  ra.sym_p = true;
  ra.sym_b = true;
  cls_a.match.push_back(ra);

  gui::ColorClassConfig cls_b;
  cls_b.color[0] = 0.1f;
  cls_b.color[1] = 0.8f;
  cls_b.color[2] = 0.2f;
  cls_b.combine = LUMICE_COLOR_COMBINE_ALL;
  cls_b.visible = false;
  cls_b.solo = true;
  cls_b.z_order = 1;
  gui::ColorClassRefConfig rb0;
  rb0.layer_idx = 0;
  rb0.crystal_pool_id = 0;
  rb0.match_all = true;
  gui::ColorClassRefConfig rb1;
  rb1.layer_idx = 0;
  rb1.crystal_pool_id = 0;
  rb1.match_all = false;
  rb1.predicate_text = "entry:1 & exit:2 & len:3";
  cls_b.match.push_back(rb0);
  cls_b.match.push_back(rb1);

  gui::g_state.raypath_color.push_back(cls_a);
  gui::g_state.raypath_color.push_back(cls_b);
  gui::g_state.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;

  const auto original = gui::g_state.raypath_color;
  const int original_mode = gui::g_state.raypath_color_mode;

  const std::string js = CoreJson(gui::g_state);
  gui::GuiState loaded = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(js, loaded));

  ASSERT_EQ(loaded.raypath_color_mode, original_mode);
  ASSERT_EQ(static_cast<int>(loaded.raypath_color.size()), static_cast<int>(original.size()));
  for (size_t i = 0; i < original.size(); i++) {
    ASSERT_TRUE(loaded.raypath_color[i] == original[i]);
  }
}

// Fixture-driven end-to-end roundtrip via the tracked three-arcs config.
// Deserialize → Serialize back → the round-tripped GuiState.raypath_color must equal
// the first-parse result. Structural fixed point, not a byte-diff (JSON formatting
// may differ).
TEST(ImportExport, raypath_color_fixture_roundtrip_three_arcs) {
  gui::DoNew();
  std::ifstream ifs(LUMICE_E2E_CONFIG_DIR "/raypath_color_three_arcs.json");
  ASSERT_TRUE(ifs.is_open());
  std::string js((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(!js.empty());

  gui::GuiState s1 = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(js, s1));
  ASSERT_EQ(static_cast<int>(s1.raypath_color.size()), 3);

  const std::string js2 = CoreJson(s1);
  gui::GuiState s2 = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(js2, s2));

  ASSERT_EQ(s2.raypath_color_mode, s1.raypath_color_mode);
  ASSERT_EQ(static_cast<int>(s2.raypath_color.size()), static_cast<int>(s1.raypath_color.size()));
  for (size_t i = 0; i < s1.raypath_color.size(); i++) {
    ASSERT_TRUE(s2.raypath_color[i] == s1.raypath_color[i]);
  }
}

// The GUI new-ref PBD-default (owner-preferred)
// must NOT leak into DeserializeFromJson: an old core JSON whose match[] entries
// omit the "symmetry" key must land with sym_p/sym_b/sym_d all false (struct
// default kSymNone), preserving pre-existing rendering semantics for legacy
// configs. A companion entry with explicit "symmetry": "P" pins that partial
// symmetry is stored verbatim (not silently upgraded to full PBD).
//
// This test is orthogonal to Steps 1/2 (Add Ref / Import from Filter) — it
// should stay green regardless of whether the call-site defaults ship, and
// going red here after those ship means the change accidentally touched the
// deserialization path (file_io.cpp ~2050-2058) — hard rollback signal.
TEST(ImportExport, raypath_color_deserialize_preserves_legacy_symmetry) {
  gui::DoNew();
  const std::string js = R"({
    "crystal": [{"id": 1, "type": "prism", "prism_h": 1.0}],
    "filter": [],
    "scene": {
      "light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
      "ray_num": 1000000,
      "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
    },
    "render": [{"id": 1, "lens": {"type": "dual_fisheye_equal_area", "fov": 180}, "resolution": [512, 256]}],
    "raypath_color": {"mode": "dominant", "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [
        {"layer": 0, "crystal": 1, "type": "raypath", "raypath": [3, 5]},
        {"layer": 0, "crystal": 1, "type": "raypath", "raypath": [1, 2], "symmetry": "P"}
      ]}
    ]}
  })";
  gui::GuiState loaded = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(js, loaded));
  ASSERT_EQ(static_cast<int>(loaded.raypath_color.size()), 1);
  ASSERT_EQ(static_cast<int>(loaded.raypath_color[0].match.size()), 2);

  // No "symmetry" key → struct default (all false). NOT the GUI new-ref
  // default of all true — that's the invariant this test defends.
  ASSERT_TRUE(!loaded.raypath_color[0].match[0].sym_p);
  ASSERT_TRUE(!loaded.raypath_color[0].match[0].sym_b);
  ASSERT_TRUE(!loaded.raypath_color[0].match[0].sym_d);

  // Explicit "symmetry": "P" → sym_p only, sym_b/sym_d stay false.
  ASSERT_TRUE(loaded.raypath_color[0].match[1].sym_p);
  ASSERT_TRUE(!loaded.raypath_color[0].match[1].sym_b);
  ASSERT_TRUE(!loaded.raypath_color[0].match[1].sym_d);
}

// Import via bare-array wire shape (default-mode, no {mode, classes} wrapper).
// doc §4.8: bare-array default is now painter.
TEST(ImportExport, raypath_color_bare_array_wire_import) {
  gui::DoNew();
  const std::string js = R"({
    "crystal": [{"id": 1, "type": "prism", "prism_h": 1.0}],
    "filter": [],
    "scene": {
      "light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
      "ray_num": 1000000,
      "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
    },
    "render": [{"id": 1, "lens": {"type": "dual_fisheye_equal_area", "fov": 180}, "resolution": [512, 256]}],
    "raypath_color": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
    ]
  })";
  gui::GuiState loaded = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(js, loaded));
  ASSERT_EQ(loaded.raypath_color_mode, LUMICE_COLOR_MODE_PAINTER);
  ASSERT_EQ(static_cast<int>(loaded.raypath_color.size()), 1);
  ASSERT_EQ(loaded.raypath_color[0].color[0], 1.0f);
  ASSERT_EQ(static_cast<int>(loaded.raypath_color[0].match.size()), 1);
  ASSERT_TRUE(loaded.raypath_color[0].match[0].match_all);
}

// Import: unknown crystal id in match → ref skipped, class still loaded.
TEST(ImportExport, raypath_color_import_skips_unknown_crystal_ref) {
  gui::DoNew();
  const std::string js = R"({
    "crystal": [{"id": 1, "type": "prism", "prism_h": 1.0}],
    "filter": [],
    "scene": {
      "light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
      "ray_num": 1000000,
      "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
    },
    "render": [{"id": 1, "lens": {"type": "dual_fisheye_equal_area", "fov": 180}, "resolution": [512, 256]}],
    "raypath_color": {"mode": "dominant", "classes": [
      {"color": [0.5, 0.5, 0.5], "match": [
        {"layer": 0, "crystal": 99},
        {"layer": 0, "crystal": 1}
      ]}
    ]}
  })";
  gui::GuiState loaded = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(js, loaded));
  ASSERT_EQ(static_cast<int>(loaded.raypath_color.size()), 1);
  ASSERT_EQ(static_cast<int>(loaded.raypath_color[0].match.size()), 1);
  ASSERT_EQ(loaded.raypath_color[0].match[0].crystal_pool_id, 0);
}

// AC4 single-owner gate: source-scan src/gui/app.cpp to prove the owner-managed reset
// primitives (`g_server_poller.InvalidateStagedTexture()`, `g_crystal_mesh_hash = 0`) appear
// exactly once — inside `ResetFrontendState`. If a future change re-scatters them into
// command handlers — the exact class of bug this gate exists to catch — this
// count-based assertion fails deterministically at build+test time, no on-screen regression
// required. Reads the file via LUMICE_GUI_APP_CPP_PATH (CMake compile-def) so the assertion
// stays anchored to the tracked source, not a stale copy.
TEST(ImportExport, reset_primitives_are_owner_single_source) {
  std::ifstream in(LUMICE_GUI_APP_CPP_PATH);
  ASSERT_TRUE(in.is_open());
  std::string src((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(!src.empty());

  auto count_occurrences = [&src](const std::string& needle) {
    int count = 0;
    size_t pos = 0;
    while ((pos = src.find(needle, pos)) != std::string::npos) {
      ++count;
      pos += needle.size();
    }
    return count;
  };

  // Each primitive: exactly one production call site (the ResetFrontendState body).
  // The needle strings are chosen to be unambiguous — spaces / operator forms match the
  // owner's actual code shape (see app.cpp `void ResetFrontendState(...)`).
  ASSERT_EQ(count_occurrences("g_server_poller.InvalidateStagedTexture()"), 1);
  ASSERT_EQ(count_occurrences("g_crystal_mesh_hash = 0"), 1);
}

// AC2 mechanism: `DoRevert()` invalidates `last_pushed_display_state` so the next reconcile
// re-pushes the restored display payload without further user interaction (plan §1 偏离 C,
// and preserved by the owner's kRevert branch). This is
// the CPU-level proof that the owner still routes the invalidation through — a full server
// round-trip is not needed to prove the mechanism, only that the baseline reset actually
// happens after DoRevert. Regression mode: a future change to the owner's kRevert branch
// drops `state.InvalidateEffectsBaselines()`; this test then fails because the baseline is
// still set after DoRevert.
TEST(ImportExport, dorevert_invalidates_effects_baselines_via_owner) {
  gui::DoNew();

  // Preconditions: (a) a revert snapshot exists (DoRevert's `if` guard), (b) a display-state
  // baseline exists (so we can observe the reset). Both are pure struct assignments — no
  // simulator round-trip required.
  gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
  gui::g_state.last_pushed_display_state = gui::GuiState::DisplayStateBaseline{};
  ASSERT_TRUE(gui::g_state.last_pushed_display_state.has_value());

  // Real production DoRevert() — owner's kRevert branch calls state.InvalidateEffectsBaselines().
  gui::DoRevert();

  // Postcondition: baseline cleared → next reconciler tick will re-push the full display
  // payload (plan §1 偏离 C fix). A regression that removes InvalidateEffectsBaselines from
  // the kRevert branch leaves the baseline populated and this assertion fails.
  ASSERT_TRUE(!gui::g_state.last_pushed_display_state.has_value());
}

// A document switch leaves no status-bar stats behind; a Revert keeps them.
//
// The status bar's ray / crystal / sampling-density readout describes the run of the document on
// screen, and its display gate is `stats_sim_ray_num > 0` (app_panels.cpp). A leftover value from
// the previous document satisfies that gate, and no poll follows a document switch to correct it —
// the poller-side stats-epoch gate filters INCOMING updates and never clears what is already
// displayed. So this property cannot be inferred from the poller fix; it needs its own guard.
//
// It currently holds structurally rather than by an explicit reset: every document-switch path
// replaces the whole GuiState before ResetFrontendState runs (DoNew via MakeNewDocumentState;
// DoOpen(.json) via a local new_state; DoOpen(.lmc) via DeserializeGuiStateJson, which opens with
// `state = GuiState{}`), and GuiState default-initializes all four counters to 0. That is exactly
// why this test drives the real DoNew / DoOpen entry points instead of calling ResetFrontendState
// directly: what is being pinned is the end-to-end property, not any one implementation of it. An
// added reset inside ResetFrontendState would be unreachable dead code today — and a future change
// that stops replacing the whole state on open is precisely what this case is here to catch.
TEST(ImportExport, document_switch_leaves_no_stale_status_bar_stats) {
  // Non-zero on all four, so a partial reset cannot pass by clearing only the gate field.
  auto seed_stats = []() {
    gui::g_state.stats_ray_seg_num = 4321;
    gui::g_state.stats_sim_ray_num = 200000;
    gui::g_state.stats_crystal_num = 77;
    gui::g_state.stats_orientation_num = 88;
  };
  auto expect_cleared = [](const char* which) {
    SCOPED_TRACE(which);
    EXPECT_EQ(gui::g_state.stats_ray_seg_num, 0u);
    EXPECT_EQ(gui::g_state.stats_sim_ray_num, 0u);
    EXPECT_EQ(gui::g_state.stats_crystal_num, 0u);
    EXPECT_EQ(gui::g_state.stats_orientation_num, 0u);
  };

  // (A) New document.
  seed_stats();
  gui::DoNew();
  expect_cleared("DoNew");

  // (B) Open a .lmc. This is the path worth spending a file on: unlike DoNew and the .json import,
  // DoOpen(.lmc) hands the LIVE g_state to LoadLmcFile rather than building a separate one, so it
  // is the path where a leftover field would actually survive if the deserializer stopped clearing.
  const std::filesystem::path lmc_path = std::filesystem::temp_directory_path() / "lumice_stats_switch_test.lmc";
  // RAII rather than a trailing remove(): the ASSERT_TRUE below returns from the TEST on failure,
  // which would skip a trailing cleanup and leave the file behind for every later run on this host.
  struct TempFileGuard {
    std::filesystem::path path;
    ~TempFileGuard() {
      std::error_code ec;
      std::filesystem::remove(path, ec);  // best-effort: a teardown failure must not fail the test
    }
  } lmc_guard{ lmc_path };
  ASSERT_TRUE(gui::SaveLmcFile(lmc_path, gui::g_state, gui::g_preview, /*save_texture=*/false));
  seed_stats();
  gui::DoOpen(lmc_path);
  expect_cleared("DoOpen(.lmc)");

  // (C) Revert — the carve-out, pinned deliberately rather than for symmetry: Revert restores
  // config into the SAME document and does not end the run whose numbers are on screen, so those
  // numbers still describe what is displayed. Without this assertion a later "let's clear stats on
  // every reset reason" change would silently blank a still-accurate status bar.
  gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
  seed_stats();
  gui::DoRevert();
  EXPECT_EQ(gui::g_state.stats_ray_seg_num, 4321u);
  EXPECT_EQ(gui::g_state.stats_sim_ray_num, 200000u);
  EXPECT_EQ(gui::g_state.stats_crystal_num, 77u);
  EXPECT_EQ(gui::g_state.stats_orientation_num, 88u);
}
