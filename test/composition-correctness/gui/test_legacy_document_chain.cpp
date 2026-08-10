// Composition chain: opening a document written by an older build.
//
// Units in the chain: file_io × gui_state × raypath_segments.
//
// What the collaboration produces that is observable: a file whose on-disk shape predates the
// current one still opens as the document its author saved. Every entry in the tables below is a
// wire form this project once wrote and no longer writes — a renderer stored as an array, an
// overlay stored as one flag instead of two, a filter stored as an integer pair.
//
// This is the one direction of failure a round trip cannot see. Serialize-then-deserialize is
// self-consistent by construction: rename a key, drop a legacy branch, change a default, and the
// round trip stays green while every file already on the user's disk opens wrong. Nothing errors —
// the document loads, the simulation runs, and the picture is of a different configuration than the
// one the file describes.
//
// Derived from the src call graph: file_io.cpp -> gui_state is 14 call sites and file_io.cpp ->
// raypath_segments is 9, the two highest-count edges out of that translation unit, and the legacy
// branches are where most of them sit.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <string>
#include <system_error>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"

namespace lumice::gui {
namespace {

// A legacy document plus what the loaded state must say afterwards. One row per wire form, because
// the failure being chased is "this particular old shape stopped being understood" and a row that
// bundled several would not name which.
struct LegacyDocCase {
  const char* name;
  const char* doc;
  void (*expect)(const GuiState& loaded);
};

// ---------------------------------------------------------------------------------------------
// The GUI-native (.lmc) reader: shapes this project wrote into its own files.

const LegacyDocCase kGuiNativeCases[] = {
  // v1 pool format: crystals and scattering as top-level ID-referenced pools, before entries
  // carried their crystal inline. Both crystal kinds, so a reader that handled only the first
  // cannot pass, and distinct proportions so a swapped pair cannot either.
  { "v1 pool format migrates to the copy model",
    R"({
      "crystals": [
        {"id": 1, "type": "prism", "shape": {"height": 2.0}},
        {"id": 2, "type": "pyramid", "shape": {"prism_h": 1.0, "upper_h": 0.3, "lower_h": 0.4}}
      ],
      "scattering": [
        {"prob": 0.8, "entries": [
          {"crystal_id": 1, "proportion": 60.0},
          {"crystal_id": 2, "proportion": 40.0, "filter_id": -1}
        ]}
      ],
      "sun": {"altitude": 25.0}
    })",
    [](const GuiState& s) {
      ASSERT_EQ(s.layers.size(), 1u);
      ASSERT_EQ(s.layers[0].entries.size(), 2u);
      EXPECT_EQ(CrystalOf(s, s.layers[0].entries[0]).type, CrystalType::kPrism);
      EXPECT_FLOAT_EQ(CrystalOf(s, s.layers[0].entries[0]).height.center, 2.0f);
      EXPECT_FLOAT_EQ(s.layers[0].entries[0].proportion, 60.0f);
      EXPECT_EQ(CrystalOf(s, s.layers[0].entries[1]).type, CrystalType::kPyramid);
      EXPECT_FALSE(s.layers[0].entries[1].filter_id.has_value()) << "filter_id -1 means no filter, not filter 0";
      EXPECT_FLOAT_EQ(s.sun.altitude, 25.0f);
    } },

  // Renderer as an array with a selection id, from before the renderer fields moved inline. Every
  // field non-default, so a reader that populated some and defaulted the rest is caught.
  { "renderers[] array loads into the single inline renderer",
    R"({
      "layers": [],
      "renderers": [{
        "id": 7, "lens_type": "fisheye_stereographic", "fov": 135.0,
        "elevation": 11.0, "azimuth": -21.0, "roll": 3.5,
        "sim_resolution": 2048, "visible": "lower",
        "background": [0.11, 0.22, 0.33], "ray_color": [0.9, 0.7, 0.5],
        "opacity": 0.8, "exposure_offset": -1.25
      }],
      "selected_renderer_id": 7, "next_renderer_id": 8
    })",
    [](const GuiState& s) {
      EXPECT_EQ(s.renderer.lens_type, 3) << "index of \"fisheye_stereographic\"";
      EXPECT_FLOAT_EQ(s.renderer.fov, 135.0f);
      EXPECT_FLOAT_EQ(s.renderer.elevation, 11.0f);
      EXPECT_FLOAT_EQ(s.renderer.azimuth, -21.0f);
      EXPECT_FLOAT_EQ(s.renderer.roll, 3.5f);
      EXPECT_EQ(s.renderer.sim_resolution_index, 2) << "2048 -> index 2 in kSimResolutions";
      EXPECT_EQ(s.renderer.visible, 1) << "\"lower\" -> 1";
      EXPECT_FALSE(s.renderer.front) << "no \"front\" key -> default";
      EXPECT_NEAR(s.renderer.background[0], 0.11f, 1e-5f);
      EXPECT_NEAR(s.renderer.background[2], 0.33f, 1e-5f);
      EXPECT_NEAR(s.renderer.ray_color[0], 0.9f, 1e-5f);
      EXPECT_NEAR(s.renderer.ray_color[2], 0.5f, 1e-5f);
      EXPECT_NEAR(s.renderer.opacity, 0.8f, 1e-5f);
      EXPECT_FLOAT_EQ(s.renderer.exposure_offset, -1.25f);
    } },

  // Two renderers in a file the GUI can only show one of. Taking the first is a choice; taking the
  // second silently would show a view the user never selected.
  { "renderers[] with several entries takes the first",
    R"({"layers": [], "renderers": [
        {"id": 1, "lens_type": "linear", "fov": 90.0},
        {"id": 2, "lens_type": "fisheye_equal_area", "fov": 180.0}]})",
    [](const GuiState& s) {
      EXPECT_EQ(s.renderer.lens_type, 0) << "\"linear\" -> 0";
      EXPECT_FLOAT_EQ(s.renderer.fov, 90.0f);
    } },

  // "front" was a visibility value before it became a flag beside a base visibility.
  { "visible=front becomes full + front",
    R"({"layers": [], "renderer": {"lens_type": "fisheye_equal_area", "fov": 180.0, "visible": "front"}})",
    [](const GuiState& s) {
      EXPECT_EQ(s.renderer.visible, kVisibleFull);
      EXPECT_TRUE(s.renderer.front);
    } },

  // One overlay flag became two (line and label). An old file has only the single key, and both
  // halves must follow it — a reader that lit only the line leaves labels off on every old file.
  { "single overlay flag drives both line and label",
    R"({"layers": [], "renderer": {"lens_type": "linear", "fov": 90.0},
        "overlay_horizon": true, "overlay_grid": false, "overlay_sun_circles": true})",
    [](const GuiState& s) {
      EXPECT_TRUE(s.show_horizon_line);
      EXPECT_TRUE(s.show_horizon_label);
      EXPECT_FALSE(s.show_grid_line);
      EXPECT_FALSE(s.show_grid_label);
      EXPECT_TRUE(s.show_sun_circles_line);
      EXPECT_TRUE(s.show_sun_circles_label);
    } },

  // A hand-edited file can carry both spellings. The specific keys win, or the split is undoable by
  // whoever left the old key behind. Horizon AND grid, so "only horizon was special-cased" fails.
  { "split overlay keys win over the single legacy key",
    R"({"layers": [], "renderer": {"lens_type": "linear", "fov": 90.0},
        "overlay_horizon": true, "overlay_horizon_line": true, "overlay_horizon_label": false,
        "overlay_grid": true, "overlay_grid_line": false, "overlay_grid_label": true})",
    [](const GuiState& s) {
      EXPECT_TRUE(s.show_horizon_line);
      EXPECT_FALSE(s.show_horizon_label);
      EXPECT_FALSE(s.show_grid_line);
      EXPECT_TRUE(s.show_grid_label);
    } },

  // A filter with no `type` predates the type tag entirely; it is a raypath, and its text must
  // survive. Falling back to an empty filter instead turns "only these paths" into "everything".
  { "typeless v1 filter is a raypath and keeps its text",
    R"({"schema_version": 1, "layers": [{"prob": 0.0, "entries": [{
        "crystal": {"type": "prism", "shape": {"height": 1.0, "face_distance": [1,1,1,1,1,1]}},
        "proportion": 100.0,
        "filter": {"id": 1, "action": "filter_in", "raypath_text": "3-1-5",
                   "sym_p": true, "sym_b": true, "sym_d": true}}]}],
        "renderer": {"lens_type": "linear", "fov": 90.0}})",
    [](const GuiState& s) {
      ASSERT_EQ(s.layers.size(), 1u);
      ASSERT_TRUE(s.layers[0].entries.at(0).filter_id.has_value());
      const FilterConfig& f = s.filters.at(*s.layers[0].entries[0].filter_id);
      EXPECT_TRUE(f.IsRaypath());
      EXPECT_EQ(f.RaypathText(), std::string("3-1-5"));
    } },

  // v2 sugar: one row carrying ';' alternatives becomes one OR row per segment. The engine sees the
  // same set either way; the editor does not, and a file that collapses back to one row shows the
  // user a filter they cannot take apart.
  { "v2 ';' raypath fans out to one row per segment",
    R"({"schema_version": 2, "layers": [{"prob": 0.0, "entries": [{
        "crystal": {"type": "prism", "shape": {"height": 1.0}}, "proportion": 100.0,
        "filter": {"type": "raypath", "action": "filter_in", "raypath_text": "3-5;1-3"}}]}]})",
    [](const GuiState& s) {
      ASSERT_TRUE(s.layers.at(0).entries.at(0).filter_id.has_value());
      const FilterConfig& f = s.filters.at(*s.layers[0].entries[0].filter_id);
      ASSERT_EQ(f.param.size(), 2u);
      EXPECT_EQ(f.param[0].text, std::string("3-5"));
      EXPECT_EQ(f.param[1].text, std::string("1-3"));
    } },

  // v2 entry/exit stored the faces as integers, before the text grammar existed.
  { "v2 integer entry/exit becomes the text grammar",
    R"({"schema_version": 2, "layers": [{"prob": 0.0, "entries": [{
        "crystal": {"type": "prism", "shape": {"height": 1.0}}, "proportion": 100.0,
        "filter": {"type": "entry_exit", "action": "filter_in", "entry": 7, "exit": 4}}]}]})",
    [](const GuiState& s) {
      ASSERT_TRUE(s.layers.at(0).entries.at(0).filter_id.has_value());
      const FilterConfig& f = s.filters.at(*s.layers[0].entries[0].filter_id);
      ASSERT_TRUE(f.IsEntryExit());
      EXPECT_EQ(f.EntryExitParamsValue().entry_text, std::string("7"));
      EXPECT_EQ(f.EntryExitParamsValue().exit_text, std::string("4"));
    } },

  // The Direction filter type was removed from the GUI. A file still carrying one must degrade to a
  // filter that matches everything rather than to a filter that matches nothing — and the crystal
  // beside it, which is fine, must survive either way.
  { "removed direction filter degrades to an empty raypath",
    R"({"schema_version": 2, "layers": [{"prob": 0.0, "entries": [{
        "crystal": {"type": "prism", "shape": {"height": 1.0}}, "proportion": 100.0,
        "filter": {"type": "direction", "action": "filter_in", "az": 30.0, "el": 15.0}}]}]})",
    [](const GuiState& s) {
      ASSERT_EQ(s.layers.at(0).entries.size(), 1u);
      ASSERT_TRUE(s.layers[0].entries[0].filter_id.has_value());
      const FilterConfig& f = s.filters.at(*s.layers[0].entries[0].filter_id);
      EXPECT_TRUE(f.IsRaypath());
      EXPECT_TRUE(f.RaypathText().empty());
    } },
};

// ---------------------------------------------------------------------------------------------
// The legacy core/CLI reader: shapes written by the command-line tool, or by hand.

const LegacyDocCase kCoreJsonCases[] = {
  // The same "front" migration on the OTHER entry point. It is a separate branch in file_io, so a
  // fix applied to one path leaves the other reading the file wrong.
  { "visible=front becomes full + front (core path)",
    R"({"crystal": [], "filter": [],
        "scene": {"scattering": [], "ray_num": 1000, "max_hits": 4,
                  "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"}},
        "render": [{"lens": {"type": "fisheye_equal_area", "fov": 180.0}, "visible": "front"}]})",
    [](const GuiState& s) {
      EXPECT_EQ(s.renderer.visible, kVisibleFull);
      EXPECT_TRUE(s.renderer.front);
    } },

  // An empty render array must leave the renderer alone rather than half-populate it. Stated
  // against a fresh RenderConfig so it covers every field instead of the handful a literal list
  // would, and so it cannot drift from gui_state's own defaults.
  { "empty render array leaves the renderer untouched",
    R"({"crystal": [], "filter": [],
        "scene": {"scattering": [], "ray_num": 1000, "max_hits": 4,
                  "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"}},
        "render": []})",
    [](const GuiState& s) { EXPECT_TRUE(s.renderer == RenderConfig{}); } },

  // Wedge angles used to be written as Miller indices. The degrees below are the same literals the
  // core-side fallback pins, computed independently of the loader.
  { "Miller indices become wedge angles",
    R"({"crystal": [{"id": 1, "type": "pyramid",
         "shape": {"prism_h": 1.2, "upper_h": 0.1, "lower_h": 0.5,
                   "upper_indices": [2, 0, 3], "lower_indices": [1, 0, 1]}}],
        "scene": {"light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
                  "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0}]}]}})",
    [](const GuiState& s) {
      const CrystalConfig& c = CrystalOf(s, s.layers.at(0).entries.at(0));
      EXPECT_NEAR(c.upper_alpha, 38.57f, 0.01f) << "{2,0,3}";
      EXPECT_NEAR(c.lower_alpha, 28.00f, 0.01f) << "{1,0,1}";
    } },

  // A spectrum written as an array of samples was once discarded without a word. Silently dropping
  // it simulates a different light source than the file asks for.
  { "array-form spectrum imports as a custom spectrum",
    R"({"crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.0}}],
        "scene": {"light_source": {"type": "sun", "altitude": 20.0, "diameter": 0.5,
                    "spectrum": [{"wavelength": 480.0, "weight": 0.9},
                                 {"wavelength": 620.0, "weight": 1.0}]},
                  "ray_num": 1000, "max_hits": 7,
                  "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]}})",
    [](const GuiState& s) {
      EXPECT_EQ(s.sun.spectrum_index, kCustomSpectrumIndex);
      ASSERT_EQ(s.sun.custom_spectrum.size(), 2u);
      EXPECT_FLOAT_EQ(s.sun.custom_spectrum[0].wavelength, 480.0f);
      EXPECT_FLOAT_EQ(s.sun.custom_spectrum[1].wavelength, 620.0f);
    } },

  // Explicit wedge angles and all three axis slots, each a distinct value so a swapped pair (zenith
  // read into roll, upper into lower) cannot pass.
  { "explicit wedge angles and all three axis slots import",
    R"({"crystal": [{"id": 1, "type": "pyramid",
         "shape": {"prism_h": 1.2, "upper_h": 0.1, "lower_h": 0.5,
                   "upper_wedge_angle": 33.0, "lower_wedge_angle": 22.0},
         "axis": {"zenith": {"type": "gauss", "mean": 12.0, "std": 1.0},
                  "azimuth": {"type": "uniform", "mean": 34.0, "std": 2.0},
                  "roll": {"type": "gauss", "mean": 56.0, "std": 3.0}}}],
        "scene": {"light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
                  "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0}]}]}})",
    [](const GuiState& s) {
      const CrystalConfig& c = CrystalOf(s, s.layers.at(0).entries.at(0));
      EXPECT_EQ(c.type, CrystalType::kPyramid);
      EXPECT_NEAR(c.upper_alpha, 33.0f, 1e-3f);
      EXPECT_NEAR(c.lower_alpha, 22.0f, 1e-3f);
      EXPECT_NEAR(c.zenith.mean, 12.0f, 1e-3f);
      EXPECT_NEAR(c.azimuth.mean, 34.0f, 1e-3f);
      EXPECT_NEAR(c.azimuth.std, 2.0f, 1e-3f);
      EXPECT_NEAR(c.roll.mean, 56.0f, 1e-3f);
      EXPECT_NEAR(c.roll.std, 3.0f, 1e-3f);
    } },

  // Grouped shape scalars written by hand: the headline claim that a core config carrying
  // sync_group can be opened by the GUI at all. Same ParseCrystal the .lmc path uses.
  { "hand-written sync_group imports onto the shape scalars",
    R"({"crystal": [{"id": 1, "type": "prism",
         "shape": {"height": 1.0, "face_distance": [1,1,1,1,1,1],
                   "sync_group": {"height": 7, "face_distance": [1,2,1,2,1,2]}}}],
        "scene": {"light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
                  "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0}]}]}})",
    [](const GuiState& s) {
      const CrystalConfig& c = CrystalOf(s, s.layers.at(0).entries.at(0));
      EXPECT_EQ(c.height.sync_group, 7);
      for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(c.face_distance[i].sync_group, (i % 2 == 0) ? 1 : 2) << "face " << i;
      }
    } },
};

TEST(LegacyDocumentChain, GuiNativeLegacyShapesStillOpen) {
  for (const LegacyDocCase& c : kGuiNativeCases) {
    SCOPED_TRACE(c.name);
    DoNew();
    GuiState loaded;
    if (!DeserializeGuiStateJson(c.doc, loaded)) {
      ADD_FAILURE() << c.name << ": a document this project once wrote no longer parses";
      continue;  // nothing loaded for this row; the rest still get checked
    }
    c.expect(loaded);
  }
}

TEST(LegacyDocumentChain, CoreJsonLegacyShapesStillOpen) {
  for (const LegacyDocCase& c : kCoreJsonCases) {
    SCOPED_TRACE(c.name);
    DoNew();
    GuiState loaded = InitDefaultState();
    if (!DeserializeFromJson(c.doc, loaded)) {
      ADD_FAILURE() << c.name << ": a document the CLI once wrote no longer parses";
      continue;
    }
    c.expect(loaded);
  }
}

// The write side of the same contract, for the keys whose NAMES are what an old file matches on.
// A round trip cannot see a rename; only a literal spelling can. These are spelled out rather than
// referenced through a constant on purpose — a shared constant would move with the change.
TEST(LegacyDocumentChain, TheKeysWrittenToDiskAreSpelledTheWayReadersExpect) {
  DoNew();
  CrystalConfig& cr = CrystalOf(g_state, g_state.layers.at(0).entries.at(0));
  cr.type = CrystalType::kPyramid;
  cr.upper_alpha = 33.0f;
  cr.lower_alpha = 22.0f;
  cr.zenith = AxisDist{ AxisDistType::kGauss, 12.0f, 1.0f };
  const auto crystal_of = [](const std::string& text) {
    return nlohmann::json::parse(text)["layers"][0]["entries"][0]["crystal"];
  };

  const nlohmann::json crystal = crystal_of(SerializeGuiStateJson(g_state));
  EXPECT_TRUE(crystal["shape"].contains("upper_wedge_angle"));
  EXPECT_TRUE(crystal["shape"].contains("lower_wedge_angle"));
  EXPECT_NEAR(crystal["shape"]["upper_wedge_angle"].get<float>(), 33.0f, 1e-3f);
  EXPECT_NEAR(crystal["shape"]["lower_wedge_angle"].get<float>(), 22.0f, 1e-3f);
  // Miller indices are read-only legacy: the writer converts to an angle and never emits them.
  EXPECT_FALSE(crystal["shape"].contains("upper_indices"));
  EXPECT_FALSE(crystal["shape"].contains("lower_indices"));

  for (const char* slot : { "zenith", "azimuth", "roll" }) {
    EXPECT_TRUE(crystal["axis"].contains(slot)) << slot;
  }
  EXPECT_NEAR(crystal["axis"]["zenith"]["mean"].get<float>(), 12.0f, 1e-3f);

  // A crystal with no grouping must emit no `sync_group` key at all, so every file written before
  // the key existed keeps its exact wire form. Both directions: asserting absence alone would pass
  // just as happily on a writer that never emits the key.
  EXPECT_FALSE(crystal["shape"].contains("sync_group"));
  cr.face_distance[3].sync_group = 1;
  EXPECT_TRUE(crystal_of(SerializeGuiStateJson(g_state))["shape"].contains("sync_group"));
}

// The overlay split must survive the write, not just the read: a writer that collapsed the two
// flags back to one key would lose the distinction on the next save, and the file would come back
// with labels the user had turned off.
TEST(LegacyDocumentChain, OverlayLineAndLabelStaySeparateThroughASave) {
  DoNew();
  g_state.show_horizon_line = true;
  g_state.show_horizon_label = false;
  g_state.show_grid_line = false;
  g_state.show_grid_label = true;
  g_state.show_sun_circles_line = true;
  g_state.show_sun_circles_label = true;

  GuiState loaded;
  ASSERT_TRUE(DeserializeGuiStateJson(SerializeGuiStateJson(g_state), loaded));
  EXPECT_TRUE(loaded.show_horizon_line);
  EXPECT_FALSE(loaded.show_horizon_label);
  EXPECT_FALSE(loaded.show_grid_line);
  EXPECT_TRUE(loaded.show_grid_label);
  EXPECT_TRUE(loaded.show_sun_circles_line);
  EXPECT_TRUE(loaded.show_sun_circles_label);
}

// A document written to an actual file and read back off it. Everything else here works on strings;
// this is the one case that touches the filesystem, and it exists because the path handling is
// where platform differences bite — the ancestor of this case hardcoded "/tmp/...", which does not
// exist on Windows, so it died on its first assertion there and never reached the round trip.
TEST(LegacyDocumentChain, ADocumentSurvivesAWriteToDiskAndAReadBack) {
  DoNew();
  g_state.sim.ray_num_millions = 3.0f;

  std::string json;
  ASSERT_TRUE(BuildExportJsonOrWarn(g_state, &json, nullptr));
  const std::filesystem::path path = std::filesystem::temp_directory_path() / "lumice_legacy_chain_roundtrip.json";
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code ec;
      std::filesystem::remove(path, ec);  // best-effort: a teardown failure must not fail the case
    }
  } cleanup{ path };
  ASSERT_TRUE(ExportConfigJson(path, json));

  std::ifstream in(path);
  ASSERT_TRUE(in.is_open());
  const std::string read_back((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

  GuiState loaded = InitDefaultState();
  ASSERT_TRUE(DeserializeFromJson(read_back, loaded));
  EXPECT_NEAR(loaded.sim.ray_num_millions, 3.0f, 0.01f);
}

}  // namespace
}  // namespace lumice::gui
