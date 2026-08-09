// Composition chain: the colour classes, from the editor to the scene and back.
//
// Units in the chain: file_io × gui_state × raypath_segments.
//
// What the collaboration produces that is observable: the ray paths the user picked out are the
// ones that come back coloured, in the colour they picked, after a save and a reload. A colour
// class is a predicate written in the same grammar as a filter, and it has to survive two
// translations the user never sees: into the scene the simulator receives, and into the document
// written to disk.
//
// The failure mode is quiet in both directions. A class that fails to translate is simply absent —
// the halo renders in the ordinary single colour and looks like a normal picture, so the natural
// conclusion is that the arc was not there rather than that the marking was dropped. A class that
// translates into the WRONG predicate colours a different set of rays, which is worse: the picture
// asserts something about the physics that the document never said.
//
// One structural property carries much of the weight here and is asserted on every row rather than
// once: the colour block must be IDENTICAL under both scene intents. The committed scene and the
// exported document come from one emitter under two intents, so a field that accidentally becomes
// intent-dependent gives the user a file that renders differently from the run they exported it
// from — and every other case in this file would still pass.

#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "lumice.h"
#include "support/scene_json_helpers.hpp"

namespace lumice::gui {
namespace {

using lumice::test::CommitSceneJson;
using lumice::test::CoreJson;

ColorClassRefConfig MatchAllRef(int crystal_pool_id = 0, int layer_idx = 0) {
  ColorClassRefConfig ref;
  ref.layer_idx = layer_idx;
  ref.crystal_pool_id = crystal_pool_id;
  ref.match_all = true;
  return ref;
}

ColorClassRefConfig PredicateRef(const char* text, int crystal_pool_id = 0, int layer_idx = 0) {
  ColorClassRefConfig ref;
  ref.layer_idx = layer_idx;
  ref.crystal_pool_id = crystal_pool_id;
  ref.match_all = false;
  ref.predicate_text = text;
  return ref;
}

// One case: set the document up, then say what the emitted colour block must look like. The block
// is read from the EXPORT document; the harness separately asserts the commit document agrees, so
// each row states its expectation once.
struct ColorCase {
  const char* name;
  void (*setup)();
  void (*expect)(const nlohmann::json& raypath_color);
};

const ColorCase kCases[] = {
  // A ref with a single-segment predicate: the whole shape of one class, spelled out once. Later
  // rows vary one thing each against it.
  { "a raypath predicate emits its faces and nothing it did not set",
    [] {
      ColorClassConfig cls;
      cls.color[0] = 0.8f;
      cls.color[1] = 0.3f;
      cls.color[2] = 0.1f;
      cls.match.push_back(PredicateRef("3-5-1"));
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) {
      ASSERT_EQ(rc["classes"].size(), 1u);
      const nlohmann::json& c = rc["classes"][0];
      EXPECT_FLOAT_EQ(c["color"][0].get<float>(), 0.8f);
      EXPECT_FLOAT_EQ(c["color"][1].get<float>(), 0.3f);
      EXPECT_FLOAT_EQ(c["color"][2].get<float>(), 0.1f);
      // Default-valued fields are omitted at the wire, which is what keeps a document written
      // before these fields existed byte-identical to one written now.
      EXPECT_FALSE(c.contains("combine"));
      EXPECT_FALSE(c.contains("visible"));
      EXPECT_FALSE(c.contains("solo"));
      ASSERT_EQ(c["match"].size(), 1u);
      EXPECT_EQ(c["match"][0]["layer"].get<int>(), 0);
      EXPECT_EQ(c["match"][0]["crystal"].get<int>(), 0) << "pool 0 -> scene crystal id 0";
      EXPECT_EQ(c["match"][0]["type"].get<std::string>(), "raypath");
      EXPECT_EQ(c["match"][0]["raypath"], nlohmann::json({ 3, 5, 1 }));
      EXPECT_FALSE(c["match"][0].contains("symmetry")) << "no symmetry bits set -> the key stays off the wire";
    } },

  // Match-all is encoded by the ABSENCE of a type, not by a special type value. Two ways of saying
  // it — the explicit flag, and a predicate that is only whitespace — must reach the same place, or
  // clearing a predicate text in the editor would mean something different from ticking the box.
  { "an explicit match-all emits no predicate type",
    [] {
      ColorClassConfig cls;
      ColorClassRefConfig ref = MatchAllRef();
      ref.predicate_text = "3-5";  // ignored while match_all is set
      cls.match.push_back(ref);
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) {
      ASSERT_EQ(rc["classes"][0]["match"].size(), 1u);
      EXPECT_FALSE(rc["classes"][0]["match"][0].contains("type"));
    } },
  { "a whitespace-only predicate emits no predicate type",
    [] {
      ColorClassConfig cls;
      cls.match.push_back(PredicateRef("  "));
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) {
      ASSERT_EQ(rc["classes"][0]["match"].size(), 1u);
      EXPECT_FALSE(rc["classes"][0]["match"][0].contains("type"));
    } },

  { "an entry/exit predicate emits its two faces",
    [] {
      ColorClassConfig cls;
      cls.match.push_back(PredicateRef("entry:1 & exit:2"));
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) {
      const nlohmann::json& p = rc["classes"][0]["match"][0];
      EXPECT_EQ(p["type"].get<std::string>(), "entry_exit");
      EXPECT_EQ(p["entry"].get<int>(), 1);
      EXPECT_EQ(p["exit"].get<int>(), 2);
    } },

  // Per-ref symmetry bits are a subset string. This is the only place the P/B/D selection reaches
  // the engine, and a dropped bit colours a third of the arcs it should.
  { "per-ref symmetry bits emit as a subset string",
    [] {
      ColorClassConfig cls;
      ColorClassRefConfig ref = PredicateRef("3-5");
      ref.sym_p = true;
      ref.sym_d = true;  // sym_b left off, so "PD" cannot be a hardcoded full set
      cls.match.push_back(ref);
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) { EXPECT_EQ(rc["classes"][0]["match"][0]["symmetry"].get<std::string>(), "PD"); } },

  // Two refs in one class, in different layers, combined with ALL rather than the default ANY.
  { "several refs keep their order and their combine rule",
    [] {
      Layer second;
      EntryCard e;
      e.crystal_id = 0;
      second.entries.push_back(e);
      g_state.layers.push_back(second);

      ColorClassConfig cls;
      cls.combine = LUMICE_COLOR_COMBINE_ALL;
      cls.match.push_back(PredicateRef("3-5", 0, 0));
      cls.match.push_back(PredicateRef("1-3", 0, 1));
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) {
      EXPECT_EQ(rc["classes"][0]["combine"].get<std::string>(), "all");
      ASSERT_EQ(rc["classes"][0]["match"].size(), 2u);
      EXPECT_EQ(rc["classes"][0]["match"][0]["layer"].get<int>(), 0);
      EXPECT_EQ(rc["classes"][0]["match"][1]["layer"].get<int>(), 1);
    } },

  { "non-default display flags and mode are emitted",
    [] {
      ColorClassConfig cls;
      cls.visible = false;
      cls.solo = true;
      cls.match.push_back(MatchAllRef());
      g_state.raypath_color.push_back(cls);
      g_state.raypath_color_mode = LUMICE_COLOR_MODE_ADDITIVE;
    },
    [](const nlohmann::json& rc) {
      EXPECT_FALSE(rc["classes"][0]["visible"].get<bool>());
      EXPECT_TRUE(rc["classes"][0]["solo"].get<bool>());
      EXPECT_EQ(rc["mode"].get<std::string>(), "additive");
    } },

  // Three ways a ref can be unexpressible on the wire. Each is dropped, and each is paired with a
  // good ref in the same class so "the class survived with one ref fewer" is distinguishable from
  // "the class was dropped whole".
  { "a ref to a crystal no entry uses is dropped",
    [] {
      CrystalConfig unused;
      unused.type = CrystalType::kPrism;
      g_state.crystals.push_back(unused);  // pool id 1, referenced by nothing

      ColorClassConfig cls;
      cls.match.push_back(MatchAllRef(1));
      cls.match.push_back(MatchAllRef(0));
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) {
      ASSERT_EQ(rc["classes"].size(), 1u);
      ASSERT_EQ(rc["classes"][0]["match"].size(), 1u);
      EXPECT_EQ(rc["classes"][0]["match"][0]["crystal"].get<int>(), 0);
    } },
  { "a predicate with two factors is dropped",
    [] {
      ColorClassConfig cls;
      cls.match.push_back(PredicateRef("3-5 & entry:2"));  // a colour ref is one atom, not a clause
      cls.match.push_back(MatchAllRef());
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) {
      ASSERT_EQ(rc["classes"][0]["match"].size(), 1u);
      EXPECT_FALSE(rc["classes"][0]["match"][0].contains("type")) << "the surviving ref is the match-all one";
    } },
  { "a predicate with several alternatives is dropped",
    [] {
      ColorClassConfig cls;
      cls.match.push_back(PredicateRef("1-3;5-7"));  // one factor, but more than one alternative
      g_state.raypath_color.push_back(cls);
    },
    [](const nlohmann::json& rc) { EXPECT_EQ(rc["classes"][0]["match"].size(), 0u); } },
};

TEST(RaypathColorDocumentChain, EveryClassShapeEmitsTheSameBlockUnderBothIntents) {
  for (const ColorCase& c : kCases) {
    SCOPED_TRACE(c.name);
    DoNew();
    c.setup();

    const nlohmann::json exported = nlohmann::json::parse(CoreJson(g_state));
    ASSERT_TRUE(exported.contains("raypath_color")) << "the class did not reach the exported document at all";
    c.expect(exported["raypath_color"]);

    const nlohmann::json committed = CommitSceneJson(g_state);
    ASSERT_FALSE(committed.is_null());
    EXPECT_EQ(committed["raypath_color"], exported["raypath_color"])
        << "the run and the file it was exported from would render differently";
  }
}

// No classes means no key at all, in both documents. A document with an empty colour block is not
// the same document as one written before colouring existed, and the whole point of omitting the
// key is that those two stay byte-identical.
TEST(RaypathColorDocumentChain, NoClassesMeansNoColourBlockAtAll) {
  DoNew();
  EXPECT_FALSE(nlohmann::json::parse(CoreJson(g_state)).contains("raypath_color"));
  EXPECT_FALSE(CommitSceneJson(g_state).contains("raypath_color"));
  // The mode is still a real setting with a real default; it simply has nothing to be emitted onto.
  EXPECT_EQ(g_state.raypath_color_mode, LUMICE_COLOR_MODE_PAINTER);
}

// The round trip through the document, compared with the config's own equality rather than field by
// field: a field added to the struct and to the writer but not to the reader fails here without
// anyone remembering to add a row for it.
TEST(RaypathColorDocumentChain, ClassesSurviveTheDocumentRoundTrip) {
  DoNew();
  ColorClassConfig a;
  a.color[0] = 1.0f;
  a.color[1] = 0.2f;
  a.color[2] = 0.3f;
  a.combine = LUMICE_COLOR_COMBINE_ANY;
  a.z_order = 0;
  ColorClassRefConfig ra = PredicateRef("3-5-1");
  ra.sym_p = true;
  ra.sym_b = true;
  a.match.push_back(ra);

  ColorClassConfig b;
  b.color[0] = 0.1f;
  b.color[1] = 0.8f;
  b.color[2] = 0.2f;
  b.combine = LUMICE_COLOR_COMBINE_ALL;
  b.visible = false;
  b.solo = true;
  b.z_order = 1;
  b.match.push_back(MatchAllRef());
  b.match.push_back(PredicateRef("entry:1 & exit:2 & len:3"));

  g_state.raypath_color = { a, b };
  g_state.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;
  const std::vector<ColorClassConfig> before = g_state.raypath_color;

  GuiState loaded = InitDefaultState();
  ASSERT_TRUE(DeserializeFromJson(CoreJson(g_state), loaded));
  EXPECT_EQ(loaded.raypath_color_mode, LUMICE_COLOR_MODE_PAINTER);
  ASSERT_EQ(loaded.raypath_color.size(), before.size());
  for (size_t i = 0; i < before.size(); ++i) {
    EXPECT_TRUE(loaded.raypath_color[i] == before[i]) << "class " << i << " changed on the way through";
  }
}

// The same round trip starting from a file that ships with the project rather than from a string
// literal built here. That is the point: it breaks when the shipped fixture and the importer drift
// apart, which a self-authored document cannot notice.
TEST(RaypathColorDocumentChain, AShippedConfigReachesAFixedPointThroughTheImporter) {
  DoNew();
  std::ifstream in(LUMICE_E2E_CONFIG_DIR "/raypath_color_three_arcs.json");
  ASSERT_TRUE(in.is_open());
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(text.empty());

  GuiState first = InitDefaultState();
  ASSERT_TRUE(DeserializeFromJson(text, first));
  ASSERT_EQ(first.raypath_color.size(), 3u) << "the shipped three-arc config no longer imports as three classes";

  GuiState second = InitDefaultState();
  ASSERT_TRUE(DeserializeFromJson(CoreJson(first), second));
  EXPECT_EQ(second.raypath_color_mode, first.raypath_color_mode);
  ASSERT_EQ(second.raypath_color.size(), first.raypath_color.size());
  for (size_t i = 0; i < first.raypath_color.size(); ++i) {
    EXPECT_TRUE(second.raypath_color[i] == first.raypath_color[i]) << "class " << i;
  }
}

// Reading a document that predates the fields the editor now seeds by default. The editor ticks all
// three symmetry boxes on a newly added ref; a file that says nothing about symmetry must NOT pick
// that up, or opening an old config silently widens every class in it.
TEST(RaypathColorDocumentChain, AnOldDocumentKeepsItsOwnSymmetryRatherThanTheEditorsDefault) {
  DoNew();
  GuiState loaded = InitDefaultState();
  ASSERT_TRUE(DeserializeFromJson(R"({
    "crystal": [{"id": 1, "type": "prism", "prism_h": 1.0}],
    "filter": [],
    "scene": {"light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
              "ray_num": 1000000, "max_hits": 8,
              "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]},
    "render": [{"id": 1, "lens": {"type": "dual_fisheye_equal_area", "fov": 180}, "resolution": [512, 256]}],
    "raypath_color": {"mode": "dominant", "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [
        {"layer": 0, "crystal": 1, "type": "raypath", "raypath": [3, 5]},
        {"layer": 0, "crystal": 1, "type": "raypath", "raypath": [1, 2], "symmetry": "P"}
      ]}
    ]}
  })",
                                  loaded));
  ASSERT_EQ(loaded.raypath_color.size(), 1u);
  ASSERT_EQ(loaded.raypath_color[0].match.size(), 2u);

  EXPECT_FALSE(loaded.raypath_color[0].match[0].sym_p) << "the editor's new-ref default leaked into the loader";
  EXPECT_FALSE(loaded.raypath_color[0].match[0].sym_b);
  EXPECT_FALSE(loaded.raypath_color[0].match[0].sym_d);
  // A partial selection is stored verbatim, not rounded up to the full set.
  EXPECT_TRUE(loaded.raypath_color[0].match[1].sym_p);
  EXPECT_FALSE(loaded.raypath_color[0].match[1].sym_b);
  EXPECT_FALSE(loaded.raypath_color[0].match[1].sym_d);
}

// Two import shapes with nothing in common but their fragility: a bare array (the wrapper-less form
// written before the mode existed) and a ref naming a crystal the document does not contain.
TEST(RaypathColorDocumentChain, ImportToleratesTheOlderWireShapeAndADanglingCrystalRef) {
  const std::string prologue = R"({
    "crystal": [{"id": 1, "type": "prism", "prism_h": 1.0}],
    "filter": [],
    "scene": {"light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
              "ray_num": 1000000, "max_hits": 8,
              "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]},
    "render": [{"id": 1, "lens": {"type": "dual_fisheye_equal_area", "fov": 180}, "resolution": [512, 256]}],
    "raypath_color": )";

  DoNew();
  GuiState bare = InitDefaultState();
  ASSERT_TRUE(
      DeserializeFromJson(prologue + R"([{"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}]})", bare));
  EXPECT_EQ(bare.raypath_color_mode, LUMICE_COLOR_MODE_PAINTER) << "a document with no mode takes today's default";
  ASSERT_EQ(bare.raypath_color.size(), 1u);
  EXPECT_FLOAT_EQ(bare.raypath_color[0].color[0], 1.0f);
  ASSERT_EQ(bare.raypath_color[0].match.size(), 1u);
  EXPECT_TRUE(bare.raypath_color[0].match[0].match_all) << "a ref with no predicate matches everything";

  DoNew();
  GuiState dangling = InitDefaultState();
  ASSERT_TRUE(DeserializeFromJson(prologue + R"({"mode": "dominant", "classes": [
      {"color": [0.5, 0.5, 0.5], "match": [{"layer": 0, "crystal": 99}, {"layer": 0, "crystal": 1}]}]}})",
                                  dangling));
  ASSERT_EQ(dangling.raypath_color.size(), 1u) << "one bad ref took the whole class with it";
  ASSERT_EQ(dangling.raypath_color[0].match.size(), 1u);
  EXPECT_EQ(dangling.raypath_color[0].match[0].crystal_pool_id, 0);
}

}  // namespace
}  // namespace lumice::gui
