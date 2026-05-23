// Unit tests for GuiState::ConfigSnapshot field-sync behavior.
// Complements the sizeof() guard in gui_state.hpp by verifying round-trip semantics.

#include <gtest/gtest.h>

#include "gui/gui_state.hpp"

namespace lumice::gui {
namespace {

// Helper: build a GuiState with non-default values across every configuration field.
// ID-pool model: crystals/filters live in dedicated pools; entries hold ids.
GuiState MakeModifiedState() {
  GuiState s = InitDefaultState();

  // Reset state: clear default pool slot + default layer, then rebuild.
  s.crystals.clear();
  s.filters.clear();
  s.layers.clear();

  // Pool slot 0: custom prism
  CrystalConfig c0;
  c0.name = "custom-prism";
  c0.type = CrystalType::kPrism;
  c0.height = 2.5f;
  c0.face_distance[0] = 1.3f;
  c0.zenith = AxisDist{ AxisDistType::kGauss, 30.0f, 5.0f };
  s.crystals.push_back(c0);

  // Pool slot 1: pyramid
  CrystalConfig c1;
  c1.type = CrystalType::kPyramid;
  c1.prism_h = 1.8f;
  c1.upper_h = 0.4f;
  c1.lower_h = 0.3f;
  c1.upper_alpha = 32.0f;
  c1.lower_alpha = 28.0f;
  s.crystals.push_back(c1);

  // Filter pool slot 0
  FilterConfig f;
  f.param = RaypathParams{ "3-1-5" };
  f.sym_p = false;
  s.filters.push_back(f);

  Layer layer;
  layer.probability = 0.42f;
  EntryCard e0;
  e0.crystal_id = 0;
  e0.proportion = 70.0f;
  layer.entries.push_back(e0);

  EntryCard e1;
  e1.crystal_id = 1;
  e1.filter_id = 0;
  e1.proportion = 30.0f;
  layer.entries.push_back(e1);
  s.layers.push_back(layer);

  // Mutate sun, sim, renderer (copy model: single renderer embedded directly).
  s.sun = SunConfig{ 35.0f, 0.6f, 4 };
  s.sim = SimConfig{ 7.5f, 12, true };

  s.renderer.lens_type = 3;
  s.renderer.fov = 110.0f;
  s.renderer.elevation = 15.0f;
  s.renderer.azimuth = -20.0f;
  s.renderer.exposure_offset = 0.5f;

  return s;
}

TEST(ConfigSnapshot, FromCapturesAllConfigFields) {
  GuiState s = MakeModifiedState();
  auto snap = GuiState::ConfigSnapshot::From(s);

  ASSERT_EQ(snap.layers.size(), s.layers.size());
  ASSERT_EQ(snap.layers[0].entries.size(), s.layers[0].entries.size());
  ASSERT_EQ(snap.crystals.size(), s.crystals.size());
  ASSERT_EQ(snap.filters.size(), s.filters.size());
  const int snap_cid0 = snap.layers[0].entries[0].crystal_id;
  const int src_cid0 = s.layers[0].entries[0].crystal_id;
  EXPECT_EQ(snap.crystals[snap_cid0].name, s.crystals[src_cid0].name);
  EXPECT_FLOAT_EQ(snap.crystals[snap_cid0].height, s.crystals[src_cid0].height);
  EXPECT_FLOAT_EQ(snap.crystals[snap_cid0].face_distance[0], 1.3f);
  EXPECT_EQ(snap.crystals[snap_cid0].zenith.type, AxisDistType::kGauss);
  EXPECT_FLOAT_EQ(snap.crystals[snap_cid0].zenith.mean, 30.0f);
  EXPECT_FLOAT_EQ(snap.crystals[snap_cid0].zenith.std, 5.0f);
  EXPECT_FLOAT_EQ(snap.layers[0].probability, s.layers[0].probability);

  EXPECT_FLOAT_EQ(snap.sun.altitude, 35.0f);
  EXPECT_EQ(snap.sun.spectrum_index, 4);
  EXPECT_FLOAT_EQ(snap.sim.ray_num_millions, 7.5f);
  EXPECT_EQ(snap.sim.max_hits, 12);
  EXPECT_TRUE(snap.sim.infinite);

  EXPECT_EQ(snap.renderer.lens_type, 3);
  EXPECT_FLOAT_EQ(snap.renderer.fov, 110.0f);
  EXPECT_FLOAT_EQ(snap.renderer.elevation, 15.0f);
  EXPECT_FLOAT_EQ(snap.renderer.azimuth, -20.0f);
  EXPECT_FLOAT_EQ(snap.renderer.exposure_offset, 0.5f);
}

// Spot-check (not exhaustive): this verifies a representative whitelist of runtime /
// view-preference fields survives ApplyTo. Completeness depends on the field-sync
// scope documented in gui_state.hpp. Adding a new runtime field does NOT fail this
// test automatically — review discipline remains the backstop for that classification.
TEST(ConfigSnapshot, ApplyToRestoresConfigFieldsAndPreservesRuntimeState) {
  GuiState source = MakeModifiedState();
  auto snap = GuiState::ConfigSnapshot::From(source);

  // Build a target state that differs in BOTH config AND runtime fields.
  GuiState target = InitDefaultState();
  target.sun.altitude = 99.0f;  // Will be overwritten by ApplyTo.

  // Runtime / view-preference fields: these must NOT be touched by ApplyTo.
  target.dirty = true;
  target.sim_state = GuiState::SimState::kSimulating;
  target.stats_ray_seg_num = 123456;
  target.stats_sim_ray_num = 78910;
  target.snapshot_intensity = 0.5f;
  target.effective_pixels = 42;
  target.texture_upload_count = 1001;
  target.intensity_locked = true;
  target.aspect_preset = AspectPreset::k16x9;
  target.aspect_portrait = true;
  target.bg_show = true;
  target.bg_alpha = 0.7f;
  target.show_horizon_line = true;
  target.show_horizon_label = true;
  target.gui_log_level = 1;
  target.core_log_level = 2;
  target.log_to_file = true;
  target.log_panel_open = true;
  target.right_panel_collapsed = true;
  target.save_texture = false;

  snap.ApplyTo(target);

  // Config fields: match source.
  ASSERT_EQ(target.layers.size(), source.layers.size());
  EXPECT_FLOAT_EQ(target.sun.altitude, source.sun.altitude);
  EXPECT_FLOAT_EQ(target.sim.ray_num_millions, source.sim.ray_num_millions);
  EXPECT_EQ(target.renderer.lens_type, source.renderer.lens_type);
  EXPECT_FLOAT_EQ(target.renderer.fov, source.renderer.fov);
  EXPECT_FLOAT_EQ(target.renderer.exposure_offset, source.renderer.exposure_offset);

  // Runtime / view fields: unchanged.
  EXPECT_TRUE(target.dirty);
  EXPECT_EQ(target.sim_state, GuiState::SimState::kSimulating);
  EXPECT_EQ(target.stats_ray_seg_num, 123456u);
  EXPECT_EQ(target.stats_sim_ray_num, 78910u);
  EXPECT_FLOAT_EQ(target.snapshot_intensity, 0.5f);
  EXPECT_EQ(target.effective_pixels, 42);
  EXPECT_EQ(target.texture_upload_count, 1001u);
  EXPECT_TRUE(target.intensity_locked);
  EXPECT_EQ(target.aspect_preset, AspectPreset::k16x9);
  EXPECT_TRUE(target.aspect_portrait);
  EXPECT_TRUE(target.bg_show);
  EXPECT_FLOAT_EQ(target.bg_alpha, 0.7f);
  EXPECT_TRUE(target.show_horizon_line);
  EXPECT_TRUE(target.show_horizon_label);
  EXPECT_EQ(target.gui_log_level, 1);
  EXPECT_EQ(target.core_log_level, 2);
  EXPECT_TRUE(target.log_to_file);
  EXPECT_TRUE(target.log_panel_open);
  EXPECT_TRUE(target.right_panel_collapsed);
  EXPECT_FALSE(target.save_texture);
}

TEST(ConfigSnapshot, RoundTripFromThenApplyRestoresConfig) {
  GuiState original = MakeModifiedState();
  auto snap = GuiState::ConfigSnapshot::From(original);

  GuiState restored = InitDefaultState();
  snap.ApplyTo(restored);

  // Key fields should survive the From → ApplyTo cycle.
  ASSERT_EQ(restored.layers.size(), original.layers.size());
  EXPECT_EQ(restored.layers[0].entries.size(), original.layers[0].entries.size());
  ASSERT_EQ(restored.crystals.size(), original.crystals.size());
  ASSERT_EQ(restored.filters.size(), original.filters.size());
  const auto& restored_e1 = restored.layers[0].entries[1];
  EXPECT_EQ(restored.crystals[restored_e1.crystal_id].type, CrystalType::kPyramid);
  ASSERT_TRUE(restored_e1.filter_id.has_value());
  EXPECT_EQ(restored.filters[*restored_e1.filter_id].RaypathText(), "3-1-5");
  EXPECT_FALSE(restored.filters[*restored_e1.filter_id].sym_p);
  EXPECT_FLOAT_EQ(restored.sun.altitude, original.sun.altitude);
  EXPECT_EQ(restored.sim.infinite, original.sim.infinite);
  EXPECT_EQ(restored.renderer.lens_type, original.renderer.lens_type);
  EXPECT_FLOAT_EQ(restored.renderer.fov, original.renderer.fov);
  // Nested crystal/axis fields also survive the From → ApplyTo cycle.
  const auto& restored_e0 = restored.layers[0].entries[0];
  EXPECT_FLOAT_EQ(restored.crystals[restored_e0.crystal_id].face_distance[0], 1.3f);
  EXPECT_EQ(restored.crystals[restored_e0.crystal_id].zenith.type, AxisDistType::kGauss);
  EXPECT_FLOAT_EQ(restored.crystals[restored_e0.crystal_id].zenith.mean, 30.0f);
}

// ID-pool round-trip: build state with shared crystal_id across two entries,
// take snapshot, mutate live pool, ApplyTo, verify pool + entry-ref ids both
// restore correctly (identity-based sharing survives the round trip).
TEST(ConfigSnapshot, RoundTripPoolAndEntries) {
  GuiState s = InitDefaultState();
  // Two entries sharing crystal pool slot 0.
  CrystalConfig c = s.crystals[0];
  c.height = 1.5f;
  s.crystals[0] = c;
  EntryCard e_extra;
  e_extra.crystal_id = 0;
  e_extra.proportion = 25.0f;
  s.layers[0].entries.push_back(e_extra);
  ASSERT_EQ(s.layers[0].entries.size(), 2u);

  auto snap = GuiState::ConfigSnapshot::From(s);

  // Mutate the live pool + push a filter entry, then revert via ApplyTo.
  s.crystals[0].height = 99.0f;
  s.filters.push_back(FilterConfig{});
  s.layers[0].entries[1].filter_id = 0;

  snap.ApplyTo(s);

  EXPECT_FLOAT_EQ(s.crystals[0].height, 1.5f);
  EXPECT_EQ(s.layers[0].entries[0].crystal_id, 0);
  EXPECT_EQ(s.layers[0].entries[1].crystal_id, 0);
  EXPECT_FALSE(s.layers[0].entries[1].filter_id.has_value());
  EXPECT_EQ(s.filters.size(), 0u);
}

// Mirror the production sizeof() guard at test scope as an extra reminder on the
// baseline platform. Platform-gated because std::vector size varies across stdlibs.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(GuiState::ConfigSnapshot) == 160,
              "Test mirror: ConfigSnapshot size changed; update From/ApplyTo in gui_state.hpp");
#endif

}  // namespace
}  // namespace lumice::gui
