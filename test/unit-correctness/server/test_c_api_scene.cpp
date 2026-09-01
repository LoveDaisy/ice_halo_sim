#include <gtest/gtest.h>

#include <cstring>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "include/lumice.h"
#include "server/c_api_internal.hpp"  // ConfigScratch(+Guard) + ConfigToJson + SceneRoot (internal)

// White-box tests for the LUMICE_Scene opaque handle (type + lifecycle + Add*/Set* build API).
//
// AC1 strategy: prove each Scene Add*/Set* encodes the SAME wire JSON as the established batch
// path by diffing SceneRoot() against ConfigToJson() of an equivalent single-item ConfigScratch.
// ConfigToJson is the single source of truth for that wire format (and is itself cross-checked
// against core to_json elsewhere in test_c_api.cpp), so equality here means the incremental path
// did not drift from the batch path. This is white-box, not "return code was OK".

namespace {

// RAII wrapper so a failed EXPECT/ASSERT still Destroys the scene.
struct SceneGuard {
  LUMICE_Scene* scene = LUMICE_SceneCreate();
  ~SceneGuard() { LUMICE_SceneDestroy(scene); }
  LUMICE_Scene* get() const { return scene; }
};

LUMICE_Distribution DetDist(float value) {
  return LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, value, 0.0f };
}

// Deterministic regular-hexagon prism (six unit face distances), mirroring test_c_api.cpp.
LUMICE_CrystalParam MakePrismParam(float height = 1.0f) {
  LUMICE_CrystalParam p{};
  p.type = 0;
  p.height = DetDist(height);
  for (auto& fd : p.face_distance) {
    fd = DetDist(1.0f);
  }
  return p;
}

}  // namespace

// =============== Lifecycle (AC3) ===============

TEST(SceneLifecycle, CreateReturnsNonNullEmptySkeleton) {
  SceneGuard g;
  ASSERT_NE(g.get(), nullptr);
  const auto& root = SceneRoot(g.get());
  // Empty skeleton: arrays present + empty, scene block present, raypath_color absent until touched.
  EXPECT_TRUE(root.at("crystal").is_array());
  EXPECT_EQ(root.at("crystal").size(), 0u);
  EXPECT_TRUE(root.at("filter").is_array());
  EXPECT_EQ(root.at("filter").size(), 0u);
  EXPECT_TRUE(root.at("render").is_array());
  EXPECT_TRUE(root.contains("scene"));
  EXPECT_TRUE(root.at("scene").at("scattering").is_array());
  EXPECT_FALSE(root.contains("raypath_color"));  // omitted until SetColorMode/AddColorClass
}

TEST(SceneLifecycle, DestroyNullIsNoOp) {
  LUMICE_SceneDestroy(nullptr);  // must not crash
}

TEST(SceneLifecycle, CloneIsDistinctPointer) {
  SceneGuard g;
  LUMICE_Scene* clone = LUMICE_SceneClone(g.get());
  ASSERT_NE(clone, nullptr);
  EXPECT_NE(clone, g.get());
  LUMICE_SceneDestroy(clone);
}

TEST(SceneLifecycle, CloneNullReturnsNull) {
  EXPECT_EQ(LUMICE_SceneClone(nullptr), nullptr);
}

// =============== Clone deep-copy independence (AC2) ===============

TEST(SceneClone, MutatingOriginalDoesNotAffectClone) {
  SceneGuard g;
  int id = -1;
  const LUMICE_CrystalParam p1 = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p1, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);

  LUMICE_Scene* clone = LUMICE_SceneClone(g.get());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(SceneRoot(clone).at("crystal").size(), 1u);

  // Add to the ORIGINAL; the clone must not see it.
  const LUMICE_CrystalParam p2 = MakePrismParam(2.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p2, &id), LUMICE_OK);
  EXPECT_EQ(id, 1);
  EXPECT_EQ(SceneRoot(g.get()).at("crystal").size(), 2u);
  EXPECT_EQ(SceneRoot(clone).at("crystal").size(), 1u) << "clone must be independent of original";

  LUMICE_SceneDestroy(clone);
}

TEST(SceneClone, MutatingCloneDoesNotAffectOriginal) {
  SceneGuard g;
  int id = -1;
  const LUMICE_CrystalParam p1 = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p1, &id), LUMICE_OK);

  LUMICE_Scene* clone = LUMICE_SceneClone(g.get());
  ASSERT_NE(clone, nullptr);
  // Mutate the CLONE; the original must not see it.
  const LUMICE_CrystalParam p2 = MakePrismParam(3.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(clone, &p2, &id), LUMICE_OK);
  EXPECT_EQ(SceneRoot(clone).at("crystal").size(), 2u);
  EXPECT_EQ(SceneRoot(g.get()).at("crystal").size(), 1u) << "original must be independent of clone";

  LUMICE_SceneDestroy(clone);
}

// =============== AC1: id assignment + white-box isomorphism vs ConfigToJson ===============

TEST(SceneAdd, IdsAreZeroBasedInsertionOrder) {
  SceneGuard g;
  int id = -1;
  const LUMICE_CrystalParam p = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p, &id), LUMICE_OK);
  EXPECT_EQ(id, 0);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p, &id), LUMICE_OK);
  EXPECT_EQ(id, 1);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p, &id), LUMICE_OK);
  EXPECT_EQ(id, 2);
}

TEST(SceneAdd, CrystalIgnoresCallerIdUsesInsertionOrder) {
  SceneGuard g;
  LUMICE_CrystalParam p = MakePrismParam(1.0f);
  p.id = 999;  // caller-supplied id must be IGNORED
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p, &id), LUMICE_OK);
  EXPECT_EQ(id, 0);
  EXPECT_EQ(SceneRoot(g.get()).at("crystal").at(0).at("id").get<int>(), 0);
}

TEST(SceneAdd, CrystalEncodingMatchesConfigToJson) {
  SceneGuard g;
  LUMICE_CrystalParam p = MakePrismParam(1.5f);
  p.zenith = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 90.0f, 3.0f };
  p.azimuth = DetDist(10.0f);
  p.roll = DetDist(0.0f);
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p, &id), LUMICE_OK);

  ConfigScratch config{};
  ConfigScratchGuard guard(config);
  config.crystal_count = 1;
  config.crystals[0] = p;
  config.crystals[0].id = 0;  // Scene assigns 0 by insertion order
  auto oracle = ConfigToJson(config);
  EXPECT_EQ(SceneRoot(g.get()).at("crystal").at(0), oracle.at("crystal").at(0));
}

namespace {
// Build the Scene's single-filter object and the ConfigToJson oracle's single-filter object for
// the same LUMICE_FilterParam, and assert byte-equality.
void ExpectFilterMatchesConfigToJson(const LUMICE_FilterParam& f) {
  SceneGuard g;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddFilter(g.get(), &f, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);

  ConfigScratch config{};
  ConfigScratchGuard guard(config);
  config.filter_count = 1;
  config.filters[0] = f;
  config.filters[0].id = 0;
  auto oracle = ConfigToJson(config);
  EXPECT_EQ(SceneRoot(g.get()).at("filter").at(0), oracle.at("filter").at(0));
}
}  // namespace

TEST(SceneAddFilter, NoneMatchesConfigToJson) {
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_NONE;
  ExpectFilterMatchesConfigToJson(f);
}

TEST(SceneAddFilter, RaypathMatchesConfigToJson) {
  LUMICE_FilterParam f{};
  f.action = 1;  // filter_out
  f.type = LUMICE_FILTER_TYPE_RAYPATH;
  f.raypath_count = 3;
  f.raypath[0] = 3;
  f.raypath[1] = 1;
  f.raypath[2] = 5;
  f.symmetry = 1 | 2;  // P | B
  ExpectFilterMatchesConfigToJson(f);
}

TEST(SceneAddFilter, EntryExitMatchesConfigToJson) {
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  f.ee_entry = 3;
  f.ee_exit = 5;
  f.ee_min_len = 2;
  f.ee_max_len = 8;
  ExpectFilterMatchesConfigToJson(f);
}

TEST(SceneAddFilter, EntryExitWildcardsMatchConfigToJson) {
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  f.ee_entry = -1;
  f.ee_exit = -1;
  f.ee_min_len = 1;
  f.ee_max_len = -1;
  ExpectFilterMatchesConfigToJson(f);
}

TEST(SceneAddFilter, DirectionMatchesConfigToJson) {
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_DIRECTION;
  f.dir_az = 120.0f;
  f.dir_el = -15.0f;
  f.dir_radii = 2.5f;
  ExpectFilterMatchesConfigToJson(f);
}

TEST(SceneAddFilter, CrystalMatchesConfigToJson) {
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_CRYSTAL;
  f.crystal_id = 2;
  ExpectFilterMatchesConfigToJson(f);
}

TEST(SceneAddFilter, ComplexTypeRejected) {
  // A complex filter must go through SceneAddComplexFilter, not SceneAddFilter.
  SceneGuard g;
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_COMPLEX;
  f.composition_index = 0;
  int id = -1;
  EXPECT_EQ(LUMICE_SceneAddFilter(g.get(), &f, &id), LUMICE_ERR_INVALID_CONFIG);
  EXPECT_EQ(SceneRoot(g.get()).at("filter").size(), 0u) << "rejection must leave scene unchanged";
}

TEST(SceneAddFilter, UnsetTypeRejected) {
  SceneGuard g;
  LUMICE_FilterParam f{};  // type == UNSET(0)
  int id = -1;
  EXPECT_EQ(LUMICE_SceneAddFilter(g.get(), &f, &id), LUMICE_ERR_INVALID_CONFIG);
  EXPECT_EQ(SceneRoot(g.get()).at("filter").size(), 0u);
}

TEST(SceneAddComplexFilter, MatchesConfigToJson) {
  // Build a sum-of-products: clause0 = {1}, clause1 = {2, 3}.
  const int term_counts[] = { 1, 2 };
  const int term_ids[] = { 1, 2, 3 };
  LUMICE_ComplexComposition comp{};
  ASSERT_EQ(LUMICE_CompositionSetClauses(&comp, 2, term_counts, term_ids), LUMICE_OK);

  LUMICE_FilterParam f{};
  f.action = 0;
  f.symmetry = 4;  // D
  // type/composition_index are deliberately left at complex/0; the Scene ignores them.
  f.type = LUMICE_FILTER_TYPE_COMPLEX;

  SceneGuard g;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddComplexFilter(g.get(), &f, &comp, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);

  // Oracle: a ConfigScratch with one complex filter referencing compositions[0].
  ConfigScratch config{};
  ConfigScratchGuard guard(config);
  config.filter_count = 1;
  config.filters[0] = f;
  config.filters[0].id = 0;
  config.filters[0].composition_index = 0;
  config.composition_count = 1;
  ASSERT_EQ(LUMICE_CompositionSetClauses(&config.compositions[0], 2, term_counts, term_ids), LUMICE_OK);
  auto oracle = ConfigToJson(config);
  EXPECT_EQ(SceneRoot(g.get()).at("filter").at(0), oracle.at("filter").at(0));

  LUMICE_CompositionReleaseClauses(&comp);
}

TEST(SceneAddComplexFilter, DeepCopiesCompositionBeforeReturn) {
  const int term_counts[] = { 2 };
  const int term_ids[] = { 7, 9 };
  LUMICE_ComplexComposition comp{};
  ASSERT_EQ(LUMICE_CompositionSetClauses(&comp, 1, term_counts, term_ids), LUMICE_OK);

  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_COMPLEX;
  SceneGuard g;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddComplexFilter(g.get(), &f, &comp, &id), LUMICE_OK);

  // Free the caller's composition immediately; the Scene must already own a deep copy.
  LUMICE_CompositionReleaseClauses(&comp);

  const auto& jf = SceneRoot(g.get()).at("filter").at(0);
  ASSERT_TRUE(jf.at("composition").is_array());
  ASSERT_EQ(jf.at("composition").size(), 1u);
  // Single clause with two terms -> array [7, 9].
  EXPECT_EQ(jf.at("composition").at(0), nlohmann::json({ 7, 9 }));
}

TEST(SceneAddRenderer, MatchesConfigToJson) {
  LUMICE_RenderParam r{};
  r.resolution_w = 800;
  r.resolution_h = 600;
  r.intensity_factor = 2.0f;
  r.overlap = 0.1f;

  SceneGuard g;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);

  ConfigScratch config{};
  ConfigScratchGuard guard(config);
  config.renderer_count = 1;
  config.renderers[0] = r;
  config.renderers[0].id = 0;
  auto oracle = ConfigToJson(config);
  EXPECT_EQ(SceneRoot(g.get()).at("render").at(0), oracle.at("render").at(0));
}

TEST(SceneAddScatterLayer, MatchesConfigToJson) {
  LUMICE_ScatterLayer layer{};
  layer.probability = 0.3f;
  layer.entry_count = 2;
  layer.entries[0] = LUMICE_ScatterEntry{ 0, 0.7f, -1 };
  layer.entries[1] = LUMICE_ScatterEntry{ 1, 0.3f, 2 };

  SceneGuard g;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddScatterLayer(g.get(), &layer, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);

  ConfigScratch config{};
  ConfigScratchGuard guard(config);
  config.scatter_count = 1;
  config.scattering[0] = layer;
  auto oracle = ConfigToJson(config);
  EXPECT_EQ(SceneRoot(g.get()).at("scene").at("scattering").at(0), oracle.at("scene").at("scattering").at(0));
}

TEST(SceneAddColorClass, MatchesConfigToJson) {
  LUMICE_ColorClass cls{};
  cls.color[0] = 1.0f;
  cls.color[1] = 0.5f;
  cls.color[2] = 0.0f;
  cls.combine = LUMICE_COLOR_COMBINE_ANY;
  cls.visible = 1;
  cls.solo = 0;
  cls.match_count = 1;
  cls.match[0].layer = 0;
  cls.match[0].crystal = 1;
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_RAYPATH;
  cls.match[0].predicate.raypath_count = 2;
  cls.match[0].predicate.raypath[0] = 3;
  cls.match[0].predicate.raypath[1] = 4;

  SceneGuard g;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddColorClass(g.get(), &cls, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);

  ConfigScratch config{};
  ConfigScratchGuard guard(config);
  ConfigCreateColorClasses(&config, 1);
  config.raypath_color[0] = cls;
  config.raypath_color_count = 1;
  config.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;  // Scene default = core kDefaultCompositeMode
  auto oracle = ConfigToJson(config);
  EXPECT_EQ(SceneRoot(g.get()).at("raypath_color").at("classes").at(0), oracle.at("raypath_color").at("classes").at(0));
}

TEST(SceneAddColorClass, FirstCallSeedsDefaultMode) {
  SceneGuard g;
  LUMICE_ColorClass cls{};
  cls.visible = 1;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddColorClass(g.get(), &cls, &id), LUMICE_OK);
  // Mode is seeded to the core default ("painter") when AddColorClass runs before SetColorMode.
  EXPECT_EQ(SceneRoot(g.get()).at("raypath_color").at("mode").get<std::string>(), "painter");
}

// =============== Set* contracts ===============

TEST(SceneSetColorMode, ValidModesAndInvalidRejected) {
  SceneGuard g;
  EXPECT_EQ(LUMICE_SceneSetColorMode(g.get(), LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
  EXPECT_EQ(SceneRoot(g.get()).at("raypath_color").at("mode").get<std::string>(), "dominant");
  EXPECT_EQ(LUMICE_SceneSetColorMode(g.get(), LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);
  EXPECT_EQ(SceneRoot(g.get()).at("raypath_color").at("mode").get<std::string>(), "additive");
  EXPECT_EQ(LUMICE_SceneSetColorMode(g.get(), 99), LUMICE_ERR_INVALID_CONFIG);
}

TEST(SceneSetColorMode, SeedsEmptyClassesArray) {
  SceneGuard g;
  ASSERT_EQ(LUMICE_SceneSetColorMode(g.get(), LUMICE_COLOR_MODE_PAINTER), LUMICE_OK);
  ASSERT_TRUE(SceneRoot(g.get()).at("raypath_color").at("classes").is_array());
  EXPECT_EQ(SceneRoot(g.get()).at("raypath_color").at("classes").size(), 0u);
}

TEST(SceneSetSimParams, FiniteAndInfiniteAndGeomClockOmit) {
  SceneGuard g;
  ASSERT_EQ(LUMICE_SceneSetSimParams(g.get(), 0, 1000, 15, 0), LUMICE_OK);
  {
    const auto& scene = SceneRoot(g.get()).at("scene");
    EXPECT_EQ(scene.at("ray_num").get<LUMICE_RayCount>(), 1000u);
    EXPECT_EQ(scene.at("max_hits").get<int>(), 15);
    EXPECT_FALSE(scene.contains("geom_clock")) << "geom_clock == 0 must be omitted";
  }
  // geom_clock != 0 emits it.
  ASSERT_EQ(LUMICE_SceneSetSimParams(g.get(), 1, 0, 20, 8), LUMICE_OK);
  {
    const auto& scene = SceneRoot(g.get()).at("scene");
    EXPECT_EQ(scene.at("ray_num").get<std::string>(), "infinite");
    EXPECT_EQ(scene.at("geom_clock").get<int>(), 8);
  }
  // Re-setting geom_clock back to 0 must erase it (no stale value).
  ASSERT_EQ(LUMICE_SceneSetSimParams(g.get(), 1, 0, 20, 0), LUMICE_OK);
  EXPECT_FALSE(SceneRoot(g.get()).at("scene").contains("geom_clock"));
}

TEST(SceneSetSpectrum, DiscreteWinsRegardlessOfOrder) {
  const LUMICE_SpectrumEntry entries[] = { { 450.0f, 1.0f }, { 550.0f, 2.0f } };

  // Order A: SetLightSource(string) then SetCustomSpectrum(discrete).
  {
    SceneGuard g;
    ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 30.0f, 180.0f, 0.5f, "A"), LUMICE_OK);
    ASSERT_EQ(LUMICE_SceneSetCustomSpectrum(g.get(), entries, 2), LUMICE_OK);
    EXPECT_TRUE(SceneRoot(g.get()).at("scene").at("light_source").at("spectrum").is_array());
  }
  // Order B: SetCustomSpectrum(discrete) then SetLightSource(string) — string must NOT clobber it.
  {
    SceneGuard g;
    ASSERT_EQ(LUMICE_SceneSetCustomSpectrum(g.get(), entries, 2), LUMICE_OK);
    ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 30.0f, 180.0f, 0.5f, "A"), LUMICE_OK);
    const auto& sp = SceneRoot(g.get()).at("scene").at("light_source").at("spectrum");
    ASSERT_TRUE(sp.is_array());
    EXPECT_EQ(sp.size(), 2u);
    EXPECT_FLOAT_EQ(sp.at(1).at("wavelength").get<float>(), 550.0f);
  }
}

TEST(SceneSetSpectrum, CountZeroClearsDiscrete) {
  const LUMICE_SpectrumEntry entries[] = { { 450.0f, 1.0f } };
  SceneGuard g;
  ASSERT_EQ(LUMICE_SceneSetCustomSpectrum(g.get(), entries, 1), LUMICE_OK);
  ASSERT_TRUE(SceneRoot(g.get()).at("scene").at("light_source").at("spectrum").is_array());
  ASSERT_EQ(LUMICE_SceneSetCustomSpectrum(g.get(), nullptr, 0), LUMICE_OK);
  const auto& sp = SceneRoot(g.get()).at("scene").at("light_source").at("spectrum");
  EXPECT_TRUE(sp.is_string());
  EXPECT_EQ(sp.get<std::string>(), "D65");
}

TEST(SceneSetLightSource, SceneBlockMatchesConfigToJson) {
  // A fully-populated scene block (light source string spectrum + sim params) must serialize
  // identically to the batch path.
  SceneGuard g;
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 23.5f, 90.0f, 0.53f, "D65"), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetSimParams(g.get(), 0, 500000, 12, 4), LUMICE_OK);

  ConfigScratch config{};
  ConfigScratchGuard guard(config);
  config.sun_altitude = 23.5f;
  config.sun_azimuth = 90.0f;
  config.sun_diameter = 0.53f;
  config.spectrum = "D65";
  config.spectrum_count = 0;
  config.infinite = 0;
  config.ray_num = 500000;
  config.max_hits = 12;
  config.geom_clock = 4;
  auto oracle = ConfigToJson(config);
  EXPECT_EQ(SceneRoot(g.get()).at("scene"), oracle.at("scene"));
}

// =============== AC3: negative paths (NULL args, invalid params, no side effects) ===============

TEST(SceneNegative, NullArgsReturnNullArg) {
  SceneGuard g;
  int id = -1;
  const LUMICE_CrystalParam cr = MakePrismParam(1.0f);
  EXPECT_EQ(LUMICE_SceneAddCrystal(nullptr, &cr, &id), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_SceneAddCrystal(g.get(), nullptr, &id), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_SceneAddCrystal(g.get(), &cr, nullptr), LUMICE_ERR_NULL_ARG);

  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_NONE;
  EXPECT_EQ(LUMICE_SceneAddFilter(g.get(), nullptr, &id), LUMICE_ERR_NULL_ARG);
  LUMICE_ComplexComposition comp{};
  EXPECT_EQ(LUMICE_SceneAddComplexFilter(g.get(), &f, nullptr, &id), LUMICE_ERR_NULL_ARG);

  EXPECT_EQ(LUMICE_SceneSetLightSource(nullptr, 0, 0, 0, "D65"), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_SceneSetSimParams(nullptr, 0, 0, 0, 0), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_SceneSetColorMode(nullptr, 0), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_SceneSetCustomSpectrum(nullptr, nullptr, 0), LUMICE_ERR_NULL_ARG);
}

TEST(SceneNegative, InvalidParamsRejectedWithoutSideEffect) {
  SceneGuard g;
  int id = -1;

  // Raypath count beyond the fixed array bound must be rejected (defensive, no OOB read).
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_RAYPATH;
  f.raypath_count = LUMICE_MAX_CONFIG_RAYPATH_LEN + 1;
  EXPECT_EQ(LUMICE_SceneAddFilter(g.get(), &f, &id), LUMICE_ERR_INVALID_CONFIG);
  EXPECT_EQ(SceneRoot(g.get()).at("filter").size(), 0u);

  // Invalid color-class combine is rejected and does NOT create the raypath_color key.
  LUMICE_ColorClass cls{};
  cls.visible = 1;
  cls.combine = 99;  // invalid
  EXPECT_EQ(LUMICE_SceneAddColorClass(g.get(), &cls, &id), LUMICE_ERR_INVALID_CONFIG);
  EXPECT_FALSE(SceneRoot(g.get()).contains("raypath_color")) << "failed Add must not seed the key";

  // Custom spectrum beyond the cap is rejected.
  std::vector<LUMICE_SpectrumEntry> too_many(LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES + 1,
                                             LUMICE_SpectrumEntry{ 500.0f, 1.0f });
  EXPECT_EQ(LUMICE_SceneSetCustomSpectrum(g.get(), too_many.data(), static_cast<int>(too_many.size())),
            LUMICE_ERR_INVALID_CONFIG);
}

TEST(SceneNegative, RendererSoftCapEnforced) {
  SceneGuard g;
  int id = -1;
  LUMICE_RenderParam r{};
  r.resolution_w = 100;
  r.resolution_h = 100;
  for (int i = 0; i < LUMICE_MAX_CONFIG_RENDERERS; i++) {
    ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);
    EXPECT_EQ(id, i);
  }
  // One past the soft cap must be rejected.
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_ERR_INVALID_CONFIG);
  EXPECT_EQ(SceneRoot(g.get()).at("render").size(), static_cast<size_t>(LUMICE_MAX_CONFIG_RENDERERS));
}

// v4.11: the renderer struct gained two enum-valued fields and two counted inline arrays, so
// LUMICE_SceneAddRenderer gained the matching rejections. Every one of them must leave the scene
// untouched — a partially-encoded renderer would be worse than the rejected call.
TEST(SceneNegative, RendererInvalidEnumOrGridCountRejected) {
  SceneGuard g;
  int id = -1;
  LUMICE_RenderParam base{};
  base.resolution_w = 100;
  base.resolution_h = 100;
  base.lens_type = LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA;
  base.lens_fov = 180.0f;

  LUMICE_RenderParam bad_lens = base;
  bad_lens.lens_type = 999;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &bad_lens, &id), LUMICE_ERR_INVALID_CONFIG);

  LUMICE_RenderParam bad_visible = base;
  bad_visible.visible = 7;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &bad_visible, &id), LUMICE_ERR_INVALID_CONFIG);

  LUMICE_RenderParam bad_angular_dist = base;
  bad_angular_dist.angular_dist_count = LUMICE_MAX_CONFIG_GRID_LINES + 1;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &bad_angular_dist, &id), LUMICE_ERR_INVALID_CONFIG);

  LUMICE_RenderParam negative_elevation = base;
  negative_elevation.elevation_grid_count = -1;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &negative_elevation, &id), LUMICE_ERR_INVALID_CONFIG);

  // v4.18: the meridian list is a third fixed-capacity array and gets the same bounds pass. Both
  // ends, because a count validated at one end only is the shape of the defect this checks for.
  LUMICE_RenderParam bad_longitude = base;
  bad_longitude.longitude_grid_count = LUMICE_MAX_CONFIG_GRID_LINES + 1;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &bad_longitude, &id), LUMICE_ERR_INVALID_CONFIG);

  LUMICE_RenderParam negative_longitude = base;
  negative_longitude.longitude_grid_count = -1;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &negative_longitude, &id), LUMICE_ERR_INVALID_CONFIG);

  // v4.16: ev_mode is the third enum-valued field and gets the same treatment.
  LUMICE_RenderParam bad_ev_mode = base;
  bad_ev_mode.ev_mode = 42;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &bad_ev_mode, &id), LUMICE_ERR_INVALID_CONFIG);

  EXPECT_TRUE(SceneRoot(g.get()).at("render").empty())
      << "a rejected renderer must not leave a partially-built entry behind";

  // The exact cap is accepted (the bound is inclusive).
  LUMICE_RenderParam at_cap = base;
  at_cap.angular_dist_count = LUMICE_MAX_CONFIG_GRID_LINES;
  at_cap.elevation_grid_count = LUMICE_MAX_CONFIG_GRID_LINES;
  at_cap.longitude_grid_count = LUMICE_MAX_CONFIG_GRID_LINES;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &at_cap, &id), LUMICE_OK);
}

// =============== AC1: SceneToJson / SceneFromJson round-trip (lossless) ===============

namespace {

// Serialize a scene into a std::string via the two-call buffer contract: query the length with a
// NULL buffer, then fill an exactly-sized buffer. Also asserts *out_len is stable across both calls.
std::string SceneToJsonString(const LUMICE_Scene* scene) {
  size_t len = 0;
  EXPECT_EQ(LUMICE_SceneToJson(scene, nullptr, 0, &len), LUMICE_OK);
  std::string buf(len + 1, '\0');  // +1 for the NUL the contract always writes
  size_t written_len = 0;
  EXPECT_EQ(LUMICE_SceneToJson(scene, buf.data(), buf.size(), &written_len), LUMICE_OK);
  EXPECT_EQ(written_len, len);
  buf.resize(len);  // drop the trailing NUL slot
  return buf;
}

// ToJson -> FromJson -> deep-compare roots, plus ToJson idempotence on the round-tripped handle.
// Full-root equality is intentionally strict: a difference would expose a real
// JsonToConfig/ConfigToJson gap, not a test that is too tight (see plan §7 risk 3).
void ExpectLosslessRoundTrip(LUMICE_Scene* scene) {
  const std::string json = SceneToJsonString(scene);
  LUMICE_Scene* rt = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJson(json.c_str(), &rt), LUMICE_OK);
  ASSERT_NE(rt, nullptr);
  EXPECT_EQ(SceneRoot(scene), SceneRoot(rt)) << "FromJson(ToJson(scene)) must equal the original root";
  // Idempotence: re-serializing the round-tripped handle yields byte-identical JSON (proves FromJson
  // introduced no silent field loss / reordering).
  EXPECT_EQ(json, SceneToJsonString(rt));
  LUMICE_SceneDestroy(rt);
}

LUMICE_Distribution Dist(int type, float center, float spread) {
  return LUMICE_Distribution{ type, center, spread };
}

}  // namespace

TEST(SceneRoundTrip, EmptyScene) {
  SceneGuard g;
  ExpectLosslessRoundTrip(g.get());
}

TEST(SceneRoundTrip, CrystalEveryDistributionType) {
  // Every randomizable distribution type (no_random/uniform/gauss/zigzag/laplacian/gauss_legacy)
  // must round-trip on both a shape scalar (height) and axis fields (zenith/azimuth).
  const int dist_types[] = { LUMICE_DIST_NO_RANDOM, LUMICE_DIST_UNIFORM,   LUMICE_DIST_GAUSS,
                             LUMICE_DIST_ZIGZAG,    LUMICE_DIST_LAPLACIAN, LUMICE_DIST_GAUSS_LEGACY };
  for (int dt : dist_types) {
    SceneGuard g;
    const float spread = (dt == LUMICE_DIST_NO_RANDOM) ? 0.0f : 0.5f;
    LUMICE_CrystalParam p = MakePrismParam(1.5f);
    p.height = Dist(dt, 2.0f, spread);
    p.zenith = Dist(dt, 90.0f, spread);
    p.azimuth = Dist(dt, 10.0f, spread);
    p.roll = DetDist(0.0f);
    int id = -1;
    ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p, &id), LUMICE_OK) << "dist type " << dt;
    ExpectLosslessRoundTrip(g.get());
  }
}

TEST(SceneRoundTrip, PyramidCrystal) {
  SceneGuard g;
  LUMICE_CrystalParam p{};
  p.type = 1;  // pyramid
  p.prism_h = DetDist(1.0f);
  p.upper_h = Dist(LUMICE_DIST_GAUSS, 0.4f, 0.05f);
  p.lower_h = DetDist(0.3f);
  p.upper_wedge_angle = 28.0f;
  p.lower_wedge_angle = 28.0f;
  for (auto& fd : p.face_distance) {
    fd = DetDist(1.0f);
  }
  p.zenith = Dist(LUMICE_DIST_UNIFORM, 0.0f, 5.0f);
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &p, &id), LUMICE_OK);
  ExpectLosslessRoundTrip(g.get());
}

TEST(SceneRoundTrip, SimpleFilters) {
  SceneGuard g;
  int id = -1;
  const LUMICE_CrystalParam cr = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &cr, &id), LUMICE_OK);

  LUMICE_FilterParam raypath{};
  raypath.type = LUMICE_FILTER_TYPE_RAYPATH;
  raypath.symmetry = 4;
  raypath.raypath_count = 3;
  raypath.raypath[0] = 3;
  raypath.raypath[1] = 5;
  raypath.raypath[2] = 3;
  ASSERT_EQ(LUMICE_SceneAddFilter(g.get(), &raypath, &id), LUMICE_OK);

  LUMICE_FilterParam crystal_filter{};
  crystal_filter.type = LUMICE_FILTER_TYPE_CRYSTAL;
  crystal_filter.crystal_id = 0;
  ASSERT_EQ(LUMICE_SceneAddFilter(g.get(), &crystal_filter, &id), LUMICE_OK);

  ExpectLosslessRoundTrip(g.get());
}

TEST(SceneRoundTrip, ComplexFilterComposition) {
  // A complex filter's composition references OTHER filters by id. FromJson runs the full config
  // validator, which cross-checks those references (stricter than Add-time, which only validates
  // each item's shape) — so the referenced simple filters must exist for a lossless round-trip.
  SceneGuard g;
  int id = -1;
  LUMICE_FilterParam s0{};
  s0.type = LUMICE_FILTER_TYPE_RAYPATH;
  s0.symmetry = 4;
  s0.raypath_count = 2;
  s0.raypath[0] = 3;
  s0.raypath[1] = 5;
  ASSERT_EQ(LUMICE_SceneAddFilter(g.get(), &s0, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);
  LUMICE_FilterParam s1{};
  s1.type = LUMICE_FILTER_TYPE_RAYPATH;
  s1.symmetry = 4;
  s1.raypath_count = 2;
  s1.raypath[0] = 3;
  s1.raypath[1] = 6;
  ASSERT_EQ(LUMICE_SceneAddFilter(g.get(), &s1, &id), LUMICE_OK);
  ASSERT_EQ(id, 1);
  // Sum-of-products over the two simple filters: clause0 = {0}, clause1 = {0, 1}.
  const int term_counts[] = { 1, 2 };
  const int term_ids[] = { 0, 0, 1 };
  LUMICE_ComplexComposition comp{};
  ASSERT_EQ(LUMICE_CompositionSetClauses(&comp, 2, term_counts, term_ids), LUMICE_OK);
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_COMPLEX;
  f.symmetry = 4;
  ASSERT_EQ(LUMICE_SceneAddComplexFilter(g.get(), &f, &comp, &id), LUMICE_OK);
  ASSERT_EQ(id, 2);
  LUMICE_CompositionReleaseClauses(&comp);
  ExpectLosslessRoundTrip(g.get());
}

TEST(SceneRoundTrip, RendererAndScatterLayer) {
  SceneGuard g;
  int id = -1;
  const LUMICE_CrystalParam cr = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &cr, &id), LUMICE_OK);

  LUMICE_RenderParam r{};
  r.resolution_w = 1024;
  r.resolution_h = 768;
  r.intensity_factor = 1.5f;
  r.overlap = 0.25f;
  // A zero-initialized LUMICE_RenderParam is not committable (lens_fov = 0 is an invalid FOV),
  // and this scene goes through LUMICE_SceneFromJson, which re-validates via core.
  r.lens_type = LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA;
  r.lens_fov = 180.0f;
  ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);

  // A scattering entry may name a filter, and that filter must exist — both core and the C API
  // reject a dangling reference — so declare one before referencing id 0 below.
  LUMICE_FilterParam f{};
  f.type = LUMICE_FILTER_TYPE_NONE;
  ASSERT_EQ(LUMICE_SceneAddFilter(g.get(), &f, &id), LUMICE_OK);

  LUMICE_ScatterLayer layer{};
  layer.probability = 0.4f;
  layer.entry_count = 2;
  layer.entries[0] = LUMICE_ScatterEntry{ 0, 0.6f, -1 };
  layer.entries[1] = LUMICE_ScatterEntry{ 0, 0.4f, 0 };
  ASSERT_EQ(LUMICE_SceneAddScatterLayer(g.get(), &layer, &id), LUMICE_OK);

  ExpectLosslessRoundTrip(g.get());
}

// Every lens projection must survive Scene -> JSON -> Scene. The C API <-> core lens mapping is a
// hand-written switch on both sides, so a single transposed arm would silently render a config
// through the wrong projection; a whole-corpus test cannot catch that for the projections the
// corpus happens not to use. The FOVs stay inside each type's MaxFov (core rejects the rest).
TEST(SceneRoundTrip, EveryLensTypeAndVisibleRange) {
  struct LensCase {
    int type;
    float fov;
  };
  const LensCase lenses[] = {
    { LUMICE_LENS_TYPE_LINEAR, 120.0f },
    { LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA, 200.0f },
    { LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT, 240.0f },
    { LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC, 300.0f },
    { LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA, 180.0f },
    { LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUIDISTANT, 190.0f },
    { LUMICE_LENS_TYPE_DUAL_FISHEYE_STEREOGRAPHIC, 210.0f },
    { LUMICE_LENS_TYPE_RECTANGULAR, 0.0f },  // full-sky by definition; core ignores fov
    { LUMICE_LENS_TYPE_FISHEYE_ORTHOGRAPHIC, 180.0f },
    { LUMICE_LENS_TYPE_DUAL_FISHEYE_ORTHOGRAPHIC, 150.0f },
    { LUMICE_LENS_TYPE_GLOBE, 90.0f },
  };
  const int visibles[] = { LUMICE_VISIBLE_UPPER, LUMICE_VISIBLE_LOWER, LUMICE_VISIBLE_FULL };

  for (const auto& lens : lenses) {
    for (int visible : visibles) {
      SceneGuard g;
      int id = -1;
      LUMICE_RenderParam r{};
      r.resolution_w = 256;
      r.resolution_h = 128;
      r.intensity_factor = 1.0f;
      r.lens_type = lens.type;
      r.lens_fov = lens.fov;
      r.visible = visible;
      r.ray_color[0] = r.ray_color[1] = r.ray_color[2] = -1.0f;
      r.horizon = 1;
      ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK)
          << "lens_type=" << lens.type << " visible=" << visible;
      // The round trip re-parses through core, so a mis-mapped enum surfaces either as a rejection
      // or as a root mismatch here.
      ExpectLosslessRoundTrip(g.get());
    }
  }
}

TEST(SceneRoundTrip, ColorClasses) {
  SceneGuard g;
  ASSERT_EQ(LUMICE_SceneSetColorMode(g.get(), LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
  LUMICE_ColorClass cls{};
  cls.color[0] = 0.2f;
  cls.color[1] = 0.7f;
  cls.color[2] = 0.9f;
  cls.combine = LUMICE_COLOR_COMBINE_ANY;
  cls.visible = 1;
  cls.solo = 0;
  cls.match_count = 1;
  cls.match[0].layer = 0;
  cls.match[0].crystal = 0;
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_RAYPATH;
  cls.match[0].predicate.raypath_count = 2;
  cls.match[0].predicate.raypath[0] = 3;
  cls.match[0].predicate.raypath[1] = 4;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddColorClass(g.get(), &cls, &id), LUMICE_OK);
  ExpectLosslessRoundTrip(g.get());
}

TEST(SceneRoundTrip, LightSourceDiscreteSpectrumAndSimParams) {
  SceneGuard g;
  const LUMICE_SpectrumEntry spec[] = { { 450.0f, 1.0f }, { 550.0f, 2.0f }, { 650.0f, 0.5f } };
  ASSERT_EQ(LUMICE_SceneSetCustomSpectrum(g.get(), spec, 3), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 23.5f, 90.0f, 0.53f, "D65"), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetSimParams(g.get(), 0, 500000, 12, 4), LUMICE_OK);
  ExpectLosslessRoundTrip(g.get());
}

TEST(SceneRoundTrip, LightSourceStringSpectrum) {
  SceneGuard g;
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 10.0f, 45.0f, 0.5f, "D65"), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetSimParams(g.get(), 1, 0, 8, 0), LUMICE_OK);
  ExpectLosslessRoundTrip(g.get());
}

TEST(SceneRoundTrip, RichSceneAllSubsystems) {
  // One scene exercising all nine subsystems together, then a full-root round-trip.
  SceneGuard g;
  int id = -1;

  LUMICE_CrystalParam cr = MakePrismParam(1.2f);
  cr.zenith = Dist(LUMICE_DIST_LAPLACIAN, 90.0f, 2.0f);
  cr.azimuth = Dist(LUMICE_DIST_UNIFORM, 0.0f, 10.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &cr, &id), LUMICE_OK);

  LUMICE_FilterParam raypath{};
  raypath.type = LUMICE_FILTER_TYPE_RAYPATH;
  raypath.symmetry = 4;
  raypath.raypath_count = 2;
  raypath.raypath[0] = 3;
  raypath.raypath[1] = 5;
  ASSERT_EQ(LUMICE_SceneAddFilter(g.get(), &raypath, &id), LUMICE_OK);

  const int term_counts[] = { 2 };
  const int term_ids[] = { 0, 0 };
  LUMICE_ComplexComposition comp{};
  ASSERT_EQ(LUMICE_CompositionSetClauses(&comp, 1, term_counts, term_ids), LUMICE_OK);
  LUMICE_FilterParam cf{};
  cf.type = LUMICE_FILTER_TYPE_COMPLEX;
  cf.symmetry = 4;
  ASSERT_EQ(LUMICE_SceneAddComplexFilter(g.get(), &cf, &comp, &id), LUMICE_OK);
  LUMICE_CompositionReleaseClauses(&comp);

  LUMICE_RenderParam r{};
  r.resolution_w = 800;
  r.resolution_h = 800;
  r.intensity_factor = 1.0f;
  // Every v4.11 renderer field gets a NON-default value here: this scene goes through
  // ExpectLosslessRoundTrip, so a field that fails to survive Scene -> JSON -> Scene shows up as a
  // root mismatch rather than passing on a coincidence of defaults.
  r.lens_type = LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC;
  r.lens_fov = 270.0f;
  r.lens_shift[0] = 7;
  r.lens_shift[1] = -3;
  r.view_azimuth = 30.0f;
  r.view_elevation = -12.5f;
  r.view_roll = 5.0f;
  r.visible = LUMICE_VISIBLE_LOWER;
  r.background[0] = 0.1f;
  r.background[1] = 0.2f;
  r.background[2] = 0.3f;
  r.ray_color[0] = 0.9f;
  r.ray_color[1] = 0.8f;
  r.ray_color[2] = 0.7f;
  r.horizon = 0;
  r.angular_dist_count = 2;
  r.angular_dist[0] = LUMICE_GridLine{ 0.0f, 1.5f, 0.5f, { 1.0f, 0.0f, 0.0f } };
  r.angular_dist[1] = LUMICE_GridLine{ 90.0f, 2.0f, 0.25f, { 0.0f, 1.0f, 0.0f } };
  r.elevation_grid_count = 1;
  r.elevation_grid[0] = LUMICE_GridLine{ 45.0f, 1.0f, 1.0f, { 0.0f, 0.0f, 1.0f } };
  r.longitude_grid_count = 2;
  r.longitude_grid[0] = LUMICE_GridLine{ -90.0f, 1.0f, 0.6f, { 0.5f, 0.5f, 0.0f } };
  r.longitude_grid[1] = LUMICE_GridLine{ 180.0f, 3.0f, 0.1f, { 0.0f, 0.5f, 0.5f } };
  r.zenith_nadir = 1;
  r.zenith_nadir_radius_px = 14.0f;
  r.zenith_nadir_opacity = 0.25f;
  r.zenith_nadir_color[0] = 0.1f;
  r.zenith_nadir_color[1] = 0.7f;
  r.zenith_nadir_color[2] = 0.9f;
  ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);

  LUMICE_ScatterLayer layer{};
  layer.probability = 0.5f;
  layer.entry_count = 1;
  layer.entries[0] = LUMICE_ScatterEntry{ 0, 1.0f, 0 };
  ASSERT_EQ(LUMICE_SceneAddScatterLayer(g.get(), &layer, &id), LUMICE_OK);

  ASSERT_EQ(LUMICE_SceneSetColorMode(g.get(), LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);
  LUMICE_ColorClass cls{};
  cls.color[0] = 1.0f;
  cls.combine = LUMICE_COLOR_COMBINE_ANY;
  cls.visible = 1;
  cls.match_count = 1;
  cls.match[0].layer = 0;
  cls.match[0].crystal = 0;
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_CRYSTAL;
  cls.match[0].predicate.crystal_id = 0;
  ASSERT_EQ(LUMICE_SceneAddColorClass(g.get(), &cls, &id), LUMICE_OK);

  const LUMICE_SpectrumEntry spec[] = { { 480.0f, 1.0f }, { 620.0f, 0.8f } };
  ASSERT_EQ(LUMICE_SceneSetCustomSpectrum(g.get(), spec, 2), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 20.0f, 120.0f, 0.5f, "D65"), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetSimParams(g.get(), 0, 1000000, 20, 8), LUMICE_OK);

  ExpectLosslessRoundTrip(g.get());
}

// =============== zenith / nadir marker block (v4.19) ===============
//
// The ABI half of the block is covered by the full-renderer round trip above. What is NOT covered
// there — and is the thing this block can get wrong that the line families cannot — is that its
// three appearance fields have NON-ZERO defaults on the JSON side. "Key absent" and
// "zero-initialized struct" are therefore different states, and a decoder that answers a missing
// key with zeros disagrees with core's own parser about what a pre-v4.19 document means.

namespace {
const nlohmann::json& RendererGridOf(const LUMICE_Scene* scene) {
  return SceneRoot(scene).at("render").at(0).at("grid");
}

// A committable single-renderer document, built through the API rather than written out by hand so
// the parts this file is not testing stay valid as the schema moves. `edit` is handed the parsed
// document to shape the marker block however the case needs.
std::string SceneJsonWithMarkerBlock(const std::function<void(nlohmann::json&)>& edit) {
  SceneGuard g;
  int id = -1;
  EXPECT_EQ(LUMICE_SceneSetSimParams(g.get(), 0, 1000, 8, 0), LUMICE_OK);
  EXPECT_EQ(LUMICE_SceneSetLightSource(g.get(), 20.0f, 0.0f, 0.5f, "D65"), LUMICE_OK);
  const LUMICE_CrystalParam c = MakePrismParam(1.5f);
  EXPECT_EQ(LUMICE_SceneAddCrystal(g.get(), &c, &id), LUMICE_OK);
  LUMICE_ScatterLayer layer{};
  layer.probability = 0.0f;
  layer.entry_count = 1;
  layer.entries[0] = LUMICE_ScatterEntry{ id, 1.0f, -1 };
  EXPECT_EQ(LUMICE_SceneAddScatterLayer(g.get(), &layer, &id), LUMICE_OK);
  LUMICE_RenderParam r{};
  r.resolution_w = 64;
  r.resolution_h = 64;
  r.intensity_factor = 1.0f;
  r.lens_type = LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA;
  r.lens_fov = 180.0f;
  r.ray_color[0] = r.ray_color[1] = r.ray_color[2] = -1.0f;
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);

  nlohmann::json doc = nlohmann::json::parse(SceneToJsonString(g.get()));
  edit(doc.at("render").at(0).at("grid"));
  return doc.dump();
}
}  // namespace

TEST(SceneRenderZenithNadir, StructValuesReachTheJsonKey) {
  SceneGuard g;
  int id = -1;
  LUMICE_RenderParam r{};
  r.resolution_w = 64;
  r.resolution_h = 64;
  r.intensity_factor = 1.0f;
  r.lens_type = LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA;
  r.lens_fov = 180.0f;
  r.zenith_nadir = 1;
  r.zenith_nadir_radius_px = 11.5f;
  r.zenith_nadir_opacity = 0.35f;
  r.zenith_nadir_color[0] = 0.25f;
  r.zenith_nadir_color[1] = 0.5f;
  r.zenith_nadir_color[2] = 0.75f;
  ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);

  const nlohmann::json& grid = RendererGridOf(g.get());
  ASSERT_TRUE(grid.contains("zenith_nadir"));
  const auto& z = grid.at("zenith_nadir");
  EXPECT_TRUE(z.at("enabled").get<bool>());
  EXPECT_NEAR(z.at("radius_px").get<float>(), 11.5f, 1e-5f);
  EXPECT_NEAR(z.at("opacity").get<float>(), 0.35f, 1e-5f);
  EXPECT_NEAR(z.at("color").at(2).get<float>(), 0.75f, 1e-5f);
}

TEST(SceneRenderZenithNadir, MissingKeyDecodesToCoreDefaultsNotZeros) {
  // A document written before v4.19 carries no "grid.zenith_nadir". Decoding it must leave the
  // struct on core's ZenithNadirParam defaults — off, but with radius 8 / opacity 0.6 /
  // {0.8, 0.2, 0.2} — because that is what core's own parser leaves, and the two decoders are
  // required to agree. Zeroing instead would be invisible while the marker is off and would
  // produce an invisible marker the moment anything switched it on.
  const std::string doc = SceneJsonWithMarkerBlock([](nlohmann::json& grid) { grid.erase("zenith_nadir"); });
  ASSERT_EQ(doc.find("zenith_nadir"), std::string::npos) << "the fixture must actually omit the key";

  LUMICE_Scene* scene = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJson(doc.c_str(), &scene), LUMICE_OK);
  ASSERT_NE(scene, nullptr);

  const auto& z = RendererGridOf(scene).at("zenith_nadir");
  EXPECT_FALSE(z.at("enabled").get<bool>());
  EXPECT_NEAR(z.at("radius_px").get<float>(), 8.0f, 1e-5f);
  EXPECT_NEAR(z.at("opacity").get<float>(), 0.6f, 1e-5f);
  EXPECT_NEAR(z.at("color").at(0).get<float>(), 0.8f, 1e-5f);
  EXPECT_NEAR(z.at("color").at(1).get<float>(), 0.2f, 1e-5f);
  LUMICE_SceneDestroy(scene);
}

TEST(SceneRenderZenithNadir, PartialObjectKeepsTheDefaultsItOmits) {
  // The middle state between the two above: a document that switches the marker on and says
  // nothing else must get the default appearance, not a zero-radius transparent black ring.
  const std::string doc = SceneJsonWithMarkerBlock(
      [](nlohmann::json& grid) { grid["zenith_nadir"] = nlohmann::json{ { "enabled", true } }; });

  LUMICE_Scene* scene = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJson(doc.c_str(), &scene), LUMICE_OK);
  ASSERT_NE(scene, nullptr);

  const auto& z = RendererGridOf(scene).at("zenith_nadir");
  EXPECT_TRUE(z.at("enabled").get<bool>());
  EXPECT_NEAR(z.at("radius_px").get<float>(), 8.0f, 1e-5f);
  EXPECT_NEAR(z.at("opacity").get<float>(), 0.6f, 1e-5f);
  EXPECT_NEAR(z.at("color").at(0).get<float>(), 0.8f, 1e-5f);
  LUMICE_SceneDestroy(scene);
}

// =============== AC3: serialization negative paths (error code, never crash, *out == NULL) ===============

namespace {
// A non-NULL sentinel we never dereference: proves the impl actively writes NULL on failure
// (rather than leaving the caller's pointer untouched). Casting an integer to a pointer and only
// comparing it is well-defined; it is never loaded/stored through.
LUMICE_Scene* Sentinel() {
  return reinterpret_cast<LUMICE_Scene*>(0xDEADBEEF);
}
}  // namespace

TEST(SceneSerializeNegative, ToJsonNullScene) {
  size_t len = 12345;
  EXPECT_EQ(LUMICE_SceneToJson(nullptr, nullptr, 0, &len), LUMICE_ERR_NULL_ARG);
}

// Mirrors StructFilterParse.ConfigToJsonBufferTruncationContract (test_c_api.cpp) for the Scene
// entry point: same snprintf-style caller-buffer contract, same highest-risk-path rationale
// (plan Step 2 required this test).
TEST(SceneSerializeNegative, ToJsonBufferTruncationContract) {
  SceneGuard g;
  int id = -1;
  const LUMICE_CrystalParam cr = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(g.get(), &cr, &id), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 20.0f, 120.0f, 0.5f, "D65"), LUMICE_OK);

  // Query length only (out_buf == NULL, buf_size == 0).
  size_t full_len = 0;
  EXPECT_EQ(LUMICE_SceneToJson(g.get(), nullptr, 0, &full_len), LUMICE_OK);
  EXPECT_GT(full_len, size_t{ 8 });  // full JSON is well over 8 bytes

  // Full (untruncated) reference output.
  std::string full(full_len + 1, '\0');
  ASSERT_EQ(LUMICE_SceneToJson(g.get(), full.data(), full.size(), nullptr), LUMICE_OK);

  // Truncate into a small buffer.
  char small[8];
  std::memset(small, 'X', sizeof(small));
  size_t len = 0;
  EXPECT_EQ(LUMICE_SceneToJson(g.get(), small, sizeof(small), &len), LUMICE_OK);
  EXPECT_EQ(len, full_len);                          // out_len = FULL length, not written count
  EXPECT_GE(len, sizeof(small));                     // out_len >= buf_size signals truncation
  EXPECT_EQ(small[sizeof(small) - 1], '\0');         // always NUL-terminated
  EXPECT_EQ(std::strlen(small), sizeof(small) - 1);  // wrote exactly buf_size-1 chars
  EXPECT_EQ(std::string(small, sizeof(small) - 1),   // truncated prefix matches full prefix
            std::string(full.data(), sizeof(small) - 1));
}

// SetLightSource takes an unvalidated const char* spectrum; nlohmann::json accepts the raw bytes
// at assignment time and only validates UTF-8 at dump() time. This exercises the catch path in
// LUMICE_SceneToJson (c_api.cpp:966-971) that must not let the exception cross the C ABI boundary.
TEST(SceneSerializeNegative, ToJsonInvalidUtf8SpectrumReturnsInvalidConfig) {
  SceneGuard g;
  const char invalid_utf8[] = { '\xFF', '\xFE', '\0' };  // not a valid UTF-8 byte sequence
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 20.0f, 120.0f, 0.5f, invalid_utf8), LUMICE_OK);

  size_t len = 12345;
  EXPECT_EQ(LUMICE_SceneToJson(g.get(), nullptr, 0, &len), LUMICE_ERR_INVALID_CONFIG);
}

TEST(SceneSerializeNegative, FromJsonNullArgs) {
  LUMICE_Scene* scene = Sentinel();
  EXPECT_EQ(LUMICE_SceneFromJson(nullptr, &scene), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(scene, nullptr) << "*out_scene must be NULL on failure, even for the NULL json_str path";
  EXPECT_EQ(LUMICE_SceneFromJson("{}", nullptr), LUMICE_ERR_NULL_ARG);

  scene = Sentinel();
  EXPECT_EQ(LUMICE_SceneFromJsonFile(nullptr, &scene), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(scene, nullptr) << "*out_scene must be NULL on failure, even for the NULL filename path";
  EXPECT_EQ(LUMICE_SceneFromJsonFile("x.json", nullptr), LUMICE_ERR_NULL_ARG);
}

TEST(SceneSerializeNegative, SyntaxErrorClearsOutScene) {
  LUMICE_Scene* scene = Sentinel();
  EXPECT_EQ(LUMICE_SceneFromJson("{ not valid json", &scene), LUMICE_ERR_INVALID_JSON);
  EXPECT_EQ(scene, nullptr) << "*out_scene must be NULL on failure";
}

TEST(SceneSerializeNegative, MissingCrystalField) {
  LUMICE_Scene* scene = Sentinel();
  const char* json = R"({ "scene": { "ray_num": 0, "max_hits": 0 } })";
  EXPECT_EQ(LUMICE_SceneFromJson(json, &scene), LUMICE_ERR_MISSING_FIELD);
  EXPECT_EQ(scene, nullptr);
}

TEST(SceneSerializeNegative, MissingSceneField) {
  LUMICE_Scene* scene = Sentinel();
  const char* json = R"({ "crystal": [] })";
  EXPECT_EQ(LUMICE_SceneFromJson(json, &scene), LUMICE_ERR_MISSING_FIELD);
  EXPECT_EQ(scene, nullptr);
}

TEST(SceneSerializeNegative, CrystalMissingTypeField) {
  LUMICE_Scene* scene = Sentinel();
  // Crystal entry present but missing "type".
  const char* json = R"({ "crystal": [ { "id": 0 } ], "scene": { "ray_num": 0, "max_hits": 0 } })";
  EXPECT_EQ(LUMICE_SceneFromJson(json, &scene), LUMICE_ERR_MISSING_FIELD);
  EXPECT_EQ(scene, nullptr);
}

TEST(SceneSerializeNegative, CrystalMissingIdField) {
  LUMICE_Scene* scene = Sentinel();
  // Crystal entry present but missing "id".
  const char* json = R"({ "crystal": [ { "type": "prism" } ], "scene": { "ray_num": 0, "max_hits": 0 } })";
  EXPECT_EQ(LUMICE_SceneFromJson(json, &scene), LUMICE_ERR_MISSING_FIELD);
  EXPECT_EQ(scene, nullptr);
}

TEST(SceneSerializeNegative, CrystalInvalidTypeValue) {
  LUMICE_Scene* scene = Sentinel();
  const char* json =
      R"({ "crystal": [ { "id": 0, "type": "notacrystal" } ], "scene": { "ray_num": 0, "max_hits": 0 } })";
  EXPECT_EQ(LUMICE_SceneFromJson(json, &scene), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(scene, nullptr);
}

TEST(SceneSerializeNegative, CrystalCountOverSoftCap) {
  // Build a JSON scene with one crystal beyond the soft cap, programmatically.
  nlohmann::json root;
  root["crystal"] = nlohmann::json::array();
  for (int i = 0; i < LUMICE_MAX_CONFIG_CRYSTALS + 1; i++) {
    nlohmann::json c;
    c["id"] = i;
    c["type"] = "prism";
    c["shape"]["height"] = { { "type", "no_random" }, { "center", 1.0 } };
    nlohmann::json fd = nlohmann::json::array();
    for (int k = 0; k < 6; k++) {
      fd.push_back({ { "type", "no_random" }, { "center", 1.0 } });
    }
    c["shape"]["face_distance"] = fd;
    root["crystal"].push_back(c);
  }
  root["scene"]["ray_num"] = 0;
  root["scene"]["max_hits"] = 0;

  LUMICE_Scene* scene = Sentinel();
  EXPECT_EQ(LUMICE_SceneFromJson(root.dump().c_str(), &scene), LUMICE_ERR_INVALID_CONFIG);
  EXPECT_EQ(scene, nullptr);
}

TEST(SceneSerializeNegative, FromJsonFileNotFound) {
  LUMICE_Scene* scene = Sentinel();
  EXPECT_EQ(LUMICE_SceneFromJsonFile("/path/does/not/exist/scene.json", &scene), LUMICE_ERR_FILE_NOT_FOUND);
  EXPECT_EQ(scene, nullptr);
}

// =============== LUMICE_CommitScene (AC1/AC2) ===============
//
// AC1 strategy mirrors the Add*/Set* section above: rather than asserting "return code was OK",
// these pin the commit contract itself — the out_reused sequence across a renderer-preserving
// edit versus a renderer-changing one — plus the byte-level premise underneath it: scene->root
// and ConfigToJson() are the SAME wire document (WireDocumentMatchesConfigToJsonAcrossEdits).
// That premise is what makes LUMICE_SceneFromJson correct: it parses into a ConfigScratch and
// re-encodes through ConfigToJson, so if the two encodings ever drifted, a JSON-authored scene
// and a hand-built one would commit as different documents.

namespace {

// Renderer used by every commit test below. Kept tiny: commit builds a consumer per renderer,
// and these tests care about the commit/reuse contract, not about rendering anything.
LUMICE_RenderParam MakeCommitRenderParam() {
  LUMICE_RenderParam r{};
  r.resolution_w = 64;
  r.resolution_h = 32;
  r.intensity_factor = 1.0f;
  r.overlap = 0.0f;
  // Part of "the smallest state core accepts": a zero-initialized LUMICE_RenderParam carries
  // lens_fov = 0, which core rejects as an invalid FOV, so these two are mandatory.
  r.lens_type = LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA;
  r.lens_fov = 180.0f;
  return r;
}

// Populate `scene` with the smallest state the core accepts as a committable configuration:
// one deterministic prism, one renderer, a sun, and a short finite ray budget (so the server
// terminates promptly instead of simulating forever).
void FillCommittableScene(LUMICE_Scene* scene, float sun_altitude = 20.0f) {
  int id = -1;
  const LUMICE_CrystalParam crystal = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_SceneAddCrystal(scene, &crystal, &id), LUMICE_OK);
  const LUMICE_RenderParam renderer = MakeCommitRenderParam();
  ASSERT_EQ(LUMICE_SceneAddRenderer(scene, &renderer, &id), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetLightSource(scene, sun_altitude, 0.0f, 0.5f, "D65"), LUMICE_OK);
  ASSERT_EQ(LUMICE_SceneSetSimParams(scene, 0, 100, 8, 0), LUMICE_OK);
}

// The ConfigScratch oracle for FillCommittableScene: the same state expressed through the
// internal wide struct, so the Scene encoding can be diffed against ConfigToJson's.
void FillEquivalentConfig(ConfigScratch* config, float sun_altitude = 20.0f) {
  config->crystal_count = 1;
  config->crystals[0] = MakePrismParam(1.0f);
  config->crystals[0].id = 0;
  config->renderer_count = 1;
  config->renderers[0] = MakeCommitRenderParam();
  config->renderers[0].id = 0;
  config->sun_altitude = sun_altitude;
  config->sun_azimuth = 0.0f;
  config->sun_diameter = 0.5f;
  config->spectrum = "D65";
  config->infinite = 0;
  config->ray_num = 100;
  config->max_hits = 8;
  config->geom_clock = 0;
}

// RAII server so a failed ASSERT still tears the server down.
struct ServerGuard {
  LUMICE_Server* server = LUMICE_CreateServer();
  ~ServerGuard() {
    LUMICE_StopServer(server);
    LUMICE_DestroyServer(server);
  }
  LUMICE_Server* get() const { return server; }
};

}  // namespace

TEST(SceneCommit, NullServerReturnsNullArg) {
  SceneGuard g;
  int reused = -1;
  EXPECT_EQ(LUMICE_CommitScene(nullptr, g.get(), &reused), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(reused, -1) << "out_reused must be untouched on the error path";
}

TEST(SceneCommit, NullSceneReturnsNullArg) {
  ServerGuard s;
  ASSERT_NE(s.get(), nullptr);
  int reused = -1;
  EXPECT_EQ(LUMICE_CommitScene(s.get(), nullptr, &reused), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(reused, -1) << "out_reused must be untouched on the error path";
}

TEST(SceneCommit, OutReusedNullDoesNotCrash) {
  SceneGuard g;
  ASSERT_NO_FATAL_FAILURE(FillCommittableScene(g.get()));
  ServerGuard s;
  ASSERT_NE(s.get(), nullptr);
  // out_reused is optional: NULL must commit normally.
  EXPECT_EQ(LUMICE_CommitScene(s.get(), g.get(), nullptr), LUMICE_OK);
}

TEST(SceneCommit, FirstCommitFromJsonNotReused) {
  // Exercise the full JSON authoring route the API is meant to be used through:
  // SceneFromJson (399.3) produces the handle, CommitScene consumes it.
  SceneGuard src;
  ASSERT_NO_FATAL_FAILURE(FillCommittableScene(src.get()));
  const std::string json = SceneRoot(src.get()).dump();

  LUMICE_Scene* scene = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJson(json.c_str(), &scene), LUMICE_OK);
  ASSERT_NE(scene, nullptr);

  ServerGuard s;
  ASSERT_NE(s.get(), nullptr);
  int reused = -1;
  EXPECT_EQ(LUMICE_CommitScene(s.get(), scene, &reused), LUMICE_OK);
  EXPECT_EQ(reused, 0) << "first commit has no consumers to reuse";
  LUMICE_SceneDestroy(scene);
}

TEST(SceneCommit, CommitDoesNotConsumeScene) {
  // The handle stays owned by the caller and stays usable: commit, mutate, commit the SAME
  // handle again. (Ownership is the part of the header contract a caller most easily gets wrong.)
  SceneGuard g;
  ASSERT_NO_FATAL_FAILURE(FillCommittableScene(g.get()));
  ServerGuard s;
  ASSERT_NE(s.get(), nullptr);
  ASSERT_EQ(LUMICE_CommitScene(s.get(), g.get(), nullptr), LUMICE_OK);
  EXPECT_EQ(SceneRoot(g.get()).at("crystal").size(), 1u) << "commit must not mutate or clear the scene";
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 25.0f, 0.0f, 0.5f, "D65"), LUMICE_OK);
  EXPECT_EQ(LUMICE_CommitScene(s.get(), g.get(), nullptr), LUMICE_OK);
}

TEST(SceneCommit, ReusesConsumersOnNonRendererChange) {
  // Reuse judgement is the core correctness point of this entry point: the server may keep its
  // consumers across a commit that does not change the renderer set/layout, so a live preview
  // buffer is not torn. Method mirrors StructFilterComplex.ComplexFilterCommitReusesOnNonRendererChange
  // in test_c_api.cpp, which locks the same contract for the legacy struct path.
  SceneGuard g;
  ASSERT_NO_FATAL_FAILURE(FillCommittableScene(g.get()));
  ServerGuard s;
  ASSERT_NE(s.get(), nullptr);

  int reused = -1;
  ASSERT_EQ(LUMICE_CommitScene(s.get(), g.get(), &reused), LUMICE_OK);
  EXPECT_EQ(reused, 0) << "first commit builds consumers";

  // Change a non-renderer field only.
  ASSERT_EQ(LUMICE_SceneSetLightSource(g.get(), 25.0f, 0.0f, 0.5f, "D65"), LUMICE_OK);
  reused = -1;
  ASSERT_EQ(LUMICE_CommitScene(s.get(), g.get(), &reused), LUMICE_OK);
  EXPECT_EQ(reused, 1) << "consumers must be reused when the renderer set is unchanged";
}

TEST(SceneCommit, WireDocumentMatchesConfigToJsonAcrossEdits) {
  // The byte-level premise the whole handle path rests on: a Scene built through Add*/Set*
  // encodes to exactly what ConfigToJson emits for the equivalent config state — before AND
  // after an edit. LUMICE_SceneFromJson relies on it directly (it parses into a ConfigScratch
  // and re-encodes through ConfigToJson into a handle root), so a drift here means a
  // JSON-authored scene and a hand-built one would reach the core as different documents.
  //
  // The commit half is asserted alongside it so the equality is not just an encoder fact:
  // the same scene state must commit OK and report the renderer-preserving reuse signal.
  SceneGuard scene;
  ASSERT_NO_FATAL_FAILURE(FillCommittableScene(scene.get()));

  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  FillEquivalentConfig(&config);

  ASSERT_EQ(SceneRoot(scene.get()), ConfigToJson(config)) << "scene->root must match ConfigToJson byte for byte";

  ServerGuard scene_server;
  ASSERT_NE(scene_server.get(), nullptr);

  int scene_reused = -1;
  EXPECT_EQ(LUMICE_CommitScene(scene_server.get(), scene.get(), &scene_reused), LUMICE_OK);
  EXPECT_EQ(scene_reused, 0);

  // Identical non-renderer edit on both sides.
  ASSERT_EQ(LUMICE_SceneSetLightSource(scene.get(), 25.0f, 0.0f, 0.5f, "D65"), LUMICE_OK);
  config.sun_altitude = 25.0f;
  ASSERT_EQ(SceneRoot(scene.get()), ConfigToJson(config)) << "edited states must still match";

  scene_reused = -1;
  EXPECT_EQ(LUMICE_CommitScene(scene_server.get(), scene.get(), &scene_reused), LUMICE_OK);
  EXPECT_EQ(scene_reused, 1);
}

TEST(SceneCommit, DoesNotReuseConsumersOnRendererChange) {
  // Counter-pole of ReusesConsumersOnNonRendererChange, and the reason that test means anything:
  // out_reused is a real signal, not a constant 1 after the first commit. Adding a renderer
  // changes the renderer set, so the consumers must be rebuilt.
  SceneGuard g;
  ASSERT_NO_FATAL_FAILURE(FillCommittableScene(g.get()));
  ServerGuard s;
  ASSERT_NE(s.get(), nullptr);

  int reused = -1;
  ASSERT_EQ(LUMICE_CommitScene(s.get(), g.get(), &reused), LUMICE_OK);
  ASSERT_EQ(reused, 0);

  int id = -1;
  const LUMICE_RenderParam extra = MakeCommitRenderParam();
  ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &extra, &id), LUMICE_OK);
  reused = -1;
  ASSERT_EQ(LUMICE_CommitScene(s.get(), g.get(), &reused), LUMICE_OK);
  EXPECT_EQ(reused, 0) << "renderer-set change must rebuild consumers";
}

TEST(SceneCommit, EmptySceneIsRejected) {
  // Error-code mapping path: a freshly created (crystal-less) scene is not a committable
  // configuration, and the core rejection must surface as a mapped C error rather than OK.
  SceneGuard g;
  ASSERT_NE(g.get(), nullptr);
  ServerGuard s;
  ASSERT_NE(s.get(), nullptr);
  int reused = -1;
  EXPECT_NE(LUMICE_CommitScene(s.get(), g.get(), &reused), LUMICE_OK);
  EXPECT_EQ(reused, -1) << "out_reused must be untouched when the commit fails";
}

// =============== v4.17 grid.angular_dist rename (C API side) ===============
// The C API carries its own JSON decoder, independent of core's ParseRenderConfig. The rename's
// alias rule therefore has to be pinned twice; test_json.cpp pins the core half with the same four
// propositions. Keys are bare string literals for the same reason as there — expressing them via
// the code under test would let the wire format and its test drift together.

TEST(SceneGridAngularDist, EncoderWritesOnlyTheNewKey) {
  SceneGuard g;
  int id = -1;
  LUMICE_RenderParam r{};
  r.resolution_w = 128;
  r.resolution_h = 128;
  r.intensity_factor = 1.0f;
  r.angular_dist_count = 1;
  r.angular_dist[0] = LUMICE_GridLine{ 22.0f, 1.2f, 0.4f, { 1.0f, 1.0f, 1.0f } };
  ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);

  const auto& jr = SceneRoot(g.get()).at("render").at(0);
  ASSERT_TRUE(jr.contains("grid"));
  ASSERT_TRUE(jr["grid"].contains("angular_dist"));
  // Absent, not merely equal: emitting both spellings would keep every reader working while
  // making the retired key permanent in files this version writes.
  EXPECT_FALSE(jr["grid"].contains("central"));
  ASSERT_EQ(jr["grid"]["angular_dist"].size(), 1u);
  EXPECT_NEAR(jr["grid"]["angular_dist"][0]["value"].get<float>(), 22.0f, 1e-5f);
}

namespace {

// A minimal but VALID scene document carrying one angular-distance line, produced by the encoder
// itself rather than hand-written: the alias tests below rewrite one key inside it, so everything
// except that key is guaranteed to be a document this decoder already accepts.
std::string SceneJsonWithOneAngularDistLine(float value_deg, float opacity) {
  SceneGuard g;
  int id = -1;
  EXPECT_EQ(LUMICE_SceneSetSimParams(g.get(), 0, 1000, 8, 0), LUMICE_OK);
  EXPECT_EQ(LUMICE_SceneSetLightSource(g.get(), 20.0f, 0.0f, 0.5f, "D65"), LUMICE_OK);
  const LUMICE_CrystalParam c = MakePrismParam(1.5f);
  EXPECT_EQ(LUMICE_SceneAddCrystal(g.get(), &c, &id), LUMICE_OK);
  LUMICE_ScatterLayer layer{};
  layer.probability = 0.0f;
  layer.entry_count = 1;
  layer.entries[0] = LUMICE_ScatterEntry{ id, 1.0f, -1 };
  EXPECT_EQ(LUMICE_SceneAddScatterLayer(g.get(), &layer, &id), LUMICE_OK);
  LUMICE_RenderParam r{};
  r.resolution_w = 128;
  r.resolution_h = 128;
  r.intensity_factor = 1.0f;
  r.lens_type = LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA;
  r.lens_fov = 180.0f;
  r.angular_dist_count = 1;
  r.angular_dist[0] = LUMICE_GridLine{ value_deg, 1.2f, opacity, { 1.0f, 1.0f, 1.0f } };
  EXPECT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);
  return SceneToJsonString(g.get());
}

// Replace the first occurrence of `from` with `to`. Used to turn the encoder's new spelling back
// into the retired one, i.e. to manufacture a pre-rename document.
std::string ReplaceFirst(std::string text, const std::string& from, const std::string& to) {
  const size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << "expected to find " << from;
  if (at != std::string::npos) {
    text.replace(at, from.size(), to);
  }
  return text;
}

}  // namespace

TEST(SceneGridAngularDist, DecoderReadsTheLegacyCentralAlias) {
  // The same document a pre-rename version of this code would have written.
  const std::string legacy =
      ReplaceFirst(SceneJsonWithOneAngularDistLine(22.0f, 0.4f), "\"angular_dist\"", "\"central\"");
  ASSERT_NE(legacy.find("\"central\""), std::string::npos);

  LUMICE_Scene* scene = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJson(legacy.c_str(), &scene), LUMICE_OK) << legacy;
  ASSERT_NE(scene, nullptr);

  // Re-serializing normalizes the alias away: what went in as "central" comes back out under the
  // new spelling, with its values intact.
  const auto& jr = SceneRoot(scene).at("render").at(0);
  ASSERT_TRUE(jr["grid"].contains("angular_dist"));
  EXPECT_FALSE(jr["grid"].contains("central"));
  ASSERT_EQ(jr["grid"]["angular_dist"].size(), 1u);
  EXPECT_NEAR(jr["grid"]["angular_dist"][0]["value"].get<float>(), 22.0f, 1e-5f);
  EXPECT_NEAR(jr["grid"]["angular_dist"][0]["opacity"].get<float>(), 0.4f, 1e-5f);
  LUMICE_SceneDestroy(scene);
}

TEST(SceneGridAngularDist, DecoderPrefersTheNewKeyWhenBothAppear) {
  // Both spellings present and disagreeing: the new key wins, and the two lists are not merged.
  const std::string both = ReplaceFirst(SceneJsonWithOneAngularDistLine(46.0f, 0.9f), "\"grid\":{",
                                        "\"grid\":{\"central\":[{\"value\":22.0,\"width\":1.0,\"opacity\":0.4,"
                                        "\"color\":[1.0,1.0,1.0]}],");

  LUMICE_Scene* scene = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJson(both.c_str(), &scene), LUMICE_OK);
  ASSERT_NE(scene, nullptr);

  const auto& jr = SceneRoot(scene).at("render").at(0);
  ASSERT_EQ(jr["grid"]["angular_dist"].size(), 1u) << "the two lists must not be concatenated";
  EXPECT_NEAR(jr["grid"]["angular_dist"][0]["value"].get<float>(), 46.0f, 1e-5f);
  LUMICE_SceneDestroy(scene);
}
