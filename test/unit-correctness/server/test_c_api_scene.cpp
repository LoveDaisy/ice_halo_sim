#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "include/lumice.h"
#include "include/lumice_config_scope.hpp"  // lumice::ConfigOwningGuard for the LUMICE_Config oracle
#include "server/c_api_internal.hpp"        // ConfigToJson + SceneRoot (test-only exposures)

// White-box tests for the LUMICE_Scene opaque handle (type + lifecycle + Add*/Set* build API).
//
// AC1 strategy: prove each Scene Add*/Set* encodes the SAME wire JSON as the established batch
// path by diffing SceneRoot() against ConfigToJson() of an equivalent single-item LUMICE_Config.
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

  LUMICE_Config config{};
  lumice::ConfigOwningGuard guard(config);
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

  LUMICE_Config config{};
  lumice::ConfigOwningGuard guard(config);
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

  // Oracle: a LUMICE_Config with one complex filter referencing compositions[0].
  LUMICE_Config config{};
  lumice::ConfigOwningGuard guard(config);
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
  r.opacity = 0.5f;
  r.intensity_factor = 2.0f;
  r.overlap = 0.1f;

  SceneGuard g;
  int id = -1;
  ASSERT_EQ(LUMICE_SceneAddRenderer(g.get(), &r, &id), LUMICE_OK);
  ASSERT_EQ(id, 0);

  LUMICE_Config config{};
  lumice::ConfigOwningGuard guard(config);
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

  LUMICE_Config config{};
  lumice::ConfigOwningGuard guard(config);
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

  LUMICE_Config config{};
  lumice::ConfigOwningGuard guard(config);
  LUMICE_ConfigCreateColorClasses(&config, 1);
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

  LUMICE_Config config{};
  lumice::ConfigOwningGuard guard(config);
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
