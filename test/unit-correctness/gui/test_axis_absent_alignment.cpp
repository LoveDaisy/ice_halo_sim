// A document that omits `crystal[].axis` entirely must mean the SAME orientation to core and to
// the GUI's loader.
//
// It did not. Core's AxisDistribution() default constructor (math.cpp) is one fixed orientation —
// zenith/azimuth/roll all kNoRandom at 0, consuming no RNG. The GUI's ParseCrystal had no `else`
// for the absent-`axis` case, so the three slots stayed at CrystalConfig{}'s own member default,
// kUniform 0/360: a full-sphere spin. Not a precision gap — the opposite physical answer, from the
// same bytes, depending on whether you ran the file through the CLI or opened it in the editor.
// That member default is right where it comes from (a NEW crystal in the editor, where "spin
// freely" is the useful starting point) and wrong as a stand-in for a value the document declined
// to state, which only core gets to define.
//
// This is a CROSS-LAYER oracle, the same shape as test_render_handedness_guard.cpp: gui_unit_test
// is the only target in the repo linking lumice_gui_obj and lumice_obj together (test/CMakeLists.txt
// documents that exemption), so it is the only place the two parsers can be compared at all. The
// point of comparing rather than asserting two hardcoded literals is that a literal pins today's
// answer on one side only — if core's default ever moves, a GUI-side literal goes on passing while
// the two sides silently diverge again. Every assertion below that matters computes its expected
// value FROM core's parse.
//
// Both sides' absolute values are also spelled out, for the same reason JsonParserParity's axis
// cases spell core's out: an equality that holds because both sides moved together is worth
// knowing about.

#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <string>

#include "config/config_manager.hpp"
#include "core/math.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"

namespace gui = lumice::gui;

namespace {

// The GUI stores zenith; core stores latitude. 90 - latitude is the whole of the conversion, and
// getting its direction backwards is a documented hazard in this repo — hence a named function
// rather than an open-coded subtraction at each of the three call sites.
float ZenithOfCore(const lumice::Distribution& latitude_dist) {
  return 90.0f - latitude_dist.center;
}

// A minimal document core accepts, whose single crystal has no `axis` key. The single-entry
// scattering block matters as much as the crystal does: the GUI pools only crystals some
// scattering entry REFERENCES (file_io.cpp's crystal_id_to_pool), and pool indices are
// first-reference order, not document ids. One crystal referenced once puts it at pool index 0.
std::string AxislessDoc(const std::string& crystal_json) {
  return std::string(R"({ "crystal": [)") + crystal_json + R"(], "filter": [],
    "scene": {
      "ray_num": 100, "max_hits": 4,
      "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
      "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ]
    },
    "render": [ { "id": 1, "resolution": [64, 32] } ] })";
}

// Parse one document with BOTH parsers and assert they land on the same orientation.
//
// `label` names the document in a failure; gtest's own line numbers point here, not at the caller,
// so without it a red says nothing about which document produced it.
void ExpectBothParsersAgreeOnFixedAxis(const std::string& text, const char* label) {
  SCOPED_TRACE(label);

  // --- core ---
  lumice::ConfigManager core_config;
  ASSERT_NO_THROW(lumice::from_json(nlohmann::json::parse(text), core_config));
  ASSERT_EQ(core_config.crystals_.count(1), 1u) << "the document is expected to carry crystal id=1";
  const lumice::AxisDistribution& core_axis = core_config.crystals_.at(1).axis_;

  // Core's own answer, spelled out. Not the property under test — the pin that says so out loud if
  // core's default ever moves, so that a cross-check passing "because both sides moved" cannot be
  // mistaken for the invariant still holding.
  EXPECT_EQ(core_axis.latitude_dist.type, lumice::DistributionType::kNoRandom);
  EXPECT_EQ(core_axis.azimuth_dist.type, lumice::DistributionType::kNoRandom);
  EXPECT_EQ(core_axis.roll_dist.type, lumice::DistributionType::kNoRandom);
  EXPECT_FLOAT_EQ(core_axis.latitude_dist.center, 90.0f) << "latitude 90 == zenith 0";
  EXPECT_FLOAT_EQ(core_axis.azimuth_dist.center, 0.0f);
  EXPECT_FLOAT_EQ(core_axis.roll_dist.center, 0.0f);

  // --- GUI ---
  gui::GuiState state = gui::InitDefaultState();
  ASSERT_TRUE(gui::DeserializeFromJson(text, state));
  ASSERT_FALSE(state.layers.empty());
  ASSERT_FALSE(state.layers[0].entries.empty());
  const gui::CrystalConfig& gui_crystal = gui::CrystalOf(state, state.layers[0].entries[0]);

  // AC1: a determinate value, i.e. NOT the kUniform 0/360 the loader used to leave here. `type` is
  // asserted alongside `std` on the plan reviewer's point: std==0 pins the numeric outcome, `type`
  // pins WHICH of the GUI's representations expresses it, and a future refactor could satisfy the
  // first while quietly changing the second.
  EXPECT_EQ(gui_crystal.zenith.type, gui::AxisDistType::kGauss);
  EXPECT_EQ(gui_crystal.azimuth.type, gui::AxisDistType::kGauss);
  EXPECT_EQ(gui_crystal.roll.type, gui::AxisDistType::kGauss);
  EXPECT_FLOAT_EQ(gui_crystal.zenith.std, 0.0f);
  EXPECT_FLOAT_EQ(gui_crystal.azimuth.std, 0.0f);
  EXPECT_FLOAT_EQ(gui_crystal.roll.std, 0.0f);

  // AC2: the same orientation, expressed as core's numbers vs the GUI's. This is the comparison
  // the task exists for — expected values computed from core's parse, never restated as literals.
  //
  // Spread is compared as well as center, and that is not symmetry for its own sake. Measured with
  // the red-state probe: with the fix reverted, the three center comparisons below stay GREEN. The
  // old wrong value was kUniform{mean 0, std 360}, whose center coincides with core's — the whole
  // divergence lives in the spread, so a cross-check on center alone passes over exactly the
  // defect it was written to catch. The type comparison cannot carry the load either: core says
  // kNoRandom and the GUI says kGauss-with-std-0 for the same physical thing, so the enums are not
  // comparable and the pins above have to state the GUI's representation as a literal. That leaves
  // spread as the one cross-parser quantity that actually goes red here.
  EXPECT_FLOAT_EQ(gui_crystal.zenith.mean, ZenithOfCore(core_axis.latitude_dist));
  EXPECT_FLOAT_EQ(gui_crystal.azimuth.mean, core_axis.azimuth_dist.center);
  EXPECT_FLOAT_EQ(gui_crystal.roll.mean, core_axis.roll_dist.center);
  EXPECT_FLOAT_EQ(gui_crystal.zenith.std, core_axis.latitude_dist.spread);
  EXPECT_FLOAT_EQ(gui_crystal.azimuth.std, core_axis.azimuth_dist.spread);
  EXPECT_FLOAT_EQ(gui_crystal.roll.std, core_axis.roll_dist.spread);
}

std::string ReadFileOrEmpty(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

}  // namespace

TEST(AxisAbsentAlignment, SyntheticPrismAgreesAcrossParsers) {
  ExpectBothParsersAgreeOnFixedAxis(AxislessDoc(R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 } })"),
                                    "synthetic prism");
}

// Pyramid too: `axis` is parsed by the same branch for both crystal types, but that branch sits
// after a type-dependent `shape` block in both parsers, and "the shape arm you did not take cannot
// disturb the axis" is a claim about two implementations here, not one.
TEST(AxisAbsentAlignment, SyntheticPyramidAgreesAcrossParsers) {
  ExpectBothParsersAgreeOnFixedAxis(
      AxislessDoc(R"({ "id": 1, "type": "pyramid", "shape": { "prism_h": 1.0, "upper_h": 0.3, "lower_h": 0.3 } })"),
      "synthetic pyramid");
}

// Drift sentinel over real tracked corpus, so the pins above cannot pass on a document shape that
// no longer resembles anything shipped.
//
// v3_config.json, and NOT examples/config_example.json, which is the file this task's issue named.
// Measured while implementing: of the four crystals in this repo that omit `axis`, only two are
// referenced by a scattering entry (v3_config.json#1 and error/missing_render.json#1). The example
// config's crystal 1 is not — its sole entry references crystal 3 — so the GUI drops it before it
// can be read back, and neither parser renders with it. v3_config.json is the one normal document
// where this fix is observable at all.
TEST(AxisAbsentAlignment, TrackedFixtureAgreesAcrossParsers) {
  const std::string text = ReadFileOrEmpty(LUMICE_TEST_FIXTURE_DIR "/v3_config.json");
  ASSERT_FALSE(text.empty()) << "cannot read " << LUMICE_TEST_FIXTURE_DIR "/v3_config.json";
  // Guard the premise rather than assume it: if someone adds an `axis` to this fixture's crystal 1,
  // this case stops being about the absent-axis path and its green would mean nothing.
  const nlohmann::json root = nlohmann::json::parse(text);
  ASSERT_FALSE(root["crystal"][0].contains("axis"))
      << "v3_config.json's crystal 1 gained an `axis` key; this sentinel needs a different document";
  ExpectBothParsersAgreeOnFixedAxis(text, "test/fixtures/v3_config.json");
}
