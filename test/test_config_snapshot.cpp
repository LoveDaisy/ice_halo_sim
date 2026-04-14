// Unit tests for GuiState::ConfigSnapshot field-sync behavior.
// Complements the sizeof() guard in gui_state.hpp by verifying round-trip semantics.

#include <gtest/gtest.h>

#include "gui/gui_state.hpp"

namespace lumice::gui {
namespace {

// Helper: build a GuiState with non-default values across every configuration field.
GuiState MakeModifiedState() {
  GuiState s = InitDefaultState();

  // Mutate layers: replace default single-entry layer and add a pyramid entry with filter.
  s.layers.clear();
  Layer layer;
  layer.probability = 0.42f;
  EntryCard e0;
  e0.crystal.name = "custom-prism";
  e0.crystal.type = CrystalType::kPrism;
  e0.crystal.height = 2.5f;
  e0.crystal.face_distance[0] = 1.3f;
  e0.crystal.zenith = AxisDist{ AxisDistType::kGauss, 30.0f, 5.0f };
  e0.proportion = 70.0f;
  layer.entries.push_back(e0);

  EntryCard e1;
  e1.crystal.type = CrystalType::kPyramid;
  e1.crystal.prism_h = 1.8f;
  e1.crystal.upper_h = 0.4f;
  e1.crystal.lower_h = 0.3f;
  e1.crystal.upper_alpha = 32.0f;
  e1.crystal.lower_alpha = 28.0f;
  FilterConfig f;
  f.raypath_text = "3-1-5";
  f.sym_p = false;
  e1.filter = f;
  e1.proportion = 30.0f;
  layer.entries.push_back(e1);
  s.layers.push_back(layer);

  // Mutate sun, sim, renderers, and id counters.
  s.sun = SunConfig{ 35.0f, 0.6f, 4 };
  s.sim = SimConfig{ 7.5f, 12, true };

  s.renderers.clear();
  RenderConfig r;
  r.id = 7;
  r.lens_type = 3;
  r.fov = 110.0f;
  r.elevation = 15.0f;
  r.azimuth = -20.0f;
  r.exposure_offset = 0.5f;
  s.renderers.push_back(r);
  s.selected_renderer = 0;
  s.next_renderer_id = 8;

  return s;
}

TEST(ConfigSnapshot, FromCapturesAllConfigFields) {
  GuiState s = MakeModifiedState();
  auto snap = GuiState::ConfigSnapshot::From(s);

  ASSERT_EQ(snap.layers.size(), s.layers.size());
  ASSERT_EQ(snap.layers[0].entries.size(), s.layers[0].entries.size());
  EXPECT_EQ(snap.layers[0].entries[0].crystal.name, s.layers[0].entries[0].crystal.name);
  EXPECT_FLOAT_EQ(snap.layers[0].entries[0].crystal.height, s.layers[0].entries[0].crystal.height);
  EXPECT_FLOAT_EQ(snap.layers[0].entries[0].crystal.face_distance[0], 1.3f);
  EXPECT_EQ(snap.layers[0].entries[0].crystal.zenith.type, AxisDistType::kGauss);
  EXPECT_FLOAT_EQ(snap.layers[0].entries[0].crystal.zenith.mean, 30.0f);
  EXPECT_FLOAT_EQ(snap.layers[0].entries[0].crystal.zenith.std, 5.0f);
  EXPECT_FLOAT_EQ(snap.layers[0].probability, s.layers[0].probability);

  EXPECT_FLOAT_EQ(snap.sun.altitude, 35.0f);
  EXPECT_EQ(snap.sun.spectrum_index, 4);
  EXPECT_FLOAT_EQ(snap.sim.ray_num_millions, 7.5f);
  EXPECT_EQ(snap.sim.max_hits, 12);
  EXPECT_TRUE(snap.sim.infinite);

  ASSERT_EQ(snap.renderers.size(), 1u);
  EXPECT_EQ(snap.renderers[0].id, 7);
  EXPECT_EQ(snap.renderers[0].lens_type, 3);
  EXPECT_FLOAT_EQ(snap.renderers[0].fov, 110.0f);
  EXPECT_EQ(snap.selected_renderer, 0);
  EXPECT_EQ(snap.next_renderer_id, 8);
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
  target.show_horizon = true;
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
  ASSERT_EQ(target.renderers.size(), source.renderers.size());
  EXPECT_EQ(target.renderers[0].id, source.renderers[0].id);
  EXPECT_EQ(target.selected_renderer, source.selected_renderer);
  EXPECT_EQ(target.next_renderer_id, source.next_renderer_id);

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
  EXPECT_TRUE(target.show_horizon);
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
  EXPECT_EQ(restored.layers[0].entries[1].crystal.type, CrystalType::kPyramid);
  EXPECT_TRUE(restored.layers[0].entries[1].filter.has_value());
  EXPECT_EQ(restored.layers[0].entries[1].filter->raypath_text, "3-1-5");
  EXPECT_FALSE(restored.layers[0].entries[1].filter->sym_p);
  EXPECT_FLOAT_EQ(restored.sun.altitude, original.sun.altitude);
  EXPECT_EQ(restored.sim.infinite, original.sim.infinite);
  EXPECT_EQ(restored.renderers[0].id, original.renderers[0].id);
  // Nested crystal/axis fields also survive the From → ApplyTo cycle.
  EXPECT_FLOAT_EQ(restored.layers[0].entries[0].crystal.face_distance[0], 1.3f);
  EXPECT_EQ(restored.layers[0].entries[0].crystal.zenith.type, AxisDistType::kGauss);
  EXPECT_FLOAT_EQ(restored.layers[0].entries[0].crystal.zenith.mean, 30.0f);
}

// Mirror the production sizeof() guard at test scope as an extra reminder on the
// baseline platform. Platform-gated because std::vector size varies across stdlibs.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(GuiState::ConfigSnapshot) == 80,
              "Test mirror: ConfigSnapshot size changed; update From/ApplyTo in gui_state.hpp");
#endif

}  // namespace
}  // namespace lumice::gui
