// Tests for ComputeOverlayLabels Front-mode hemisphere culling.
// Direct (non-interactive) tests: instantiate OverlayLabelInput, call
// ComputeOverlayLabels, inspect result labels.

#include <vector>

#include "gui/overlay_labels.hpp"
#include "test_gui_shared.hpp"

namespace {

// Convenience: build an OverlayLabelInput with grid enabled, sun circles disabled.
lumice::gui::OverlayLabelInput MakeGridOnly(int visible, int lens_type, float elevation, float azimuth) {
  lumice::gui::OverlayLabelInput in{};
  in.lens_type = lens_type;
  in.fov = 120.0f;
  in.elevation = elevation;
  in.azimuth = azimuth;
  in.roll = 0.0f;
  in.visible = visible;
  in.show_horizon = false;
  in.show_grid = true;
  in.show_sun_circles = false;
  in.sun_dir[0] = 0.0f;
  in.sun_dir[1] = 0.0f;
  in.sun_dir[2] = -1.0f;
  in.sun_circle_count = 0;
  in.sun_circle_angles = nullptr;
  in.horizon_color[0] = in.horizon_color[1] = in.horizon_color[2] = 1.0f;
  in.grid_color[0] = in.grid_color[1] = in.grid_color[2] = 1.0f;
  in.sun_circles_color[0] = in.sun_circles_color[1] = in.sun_circles_color[2] = 1.0f;
  in.grid_alpha = 1.0f;
  in.sun_circles_alpha = 1.0f;
  return in;
}

// Convenience: input with sun circles only (single 22° circle), Linear lens.
lumice::gui::OverlayLabelInput MakeSunOnly(int visible, const float sun_dir[3], const float* circle_angles, int count) {
  lumice::gui::OverlayLabelInput in = MakeGridOnly(visible, /*lens=Linear*/ 0, /*elev*/ 0.0f, /*az*/ 0.0f);
  in.show_grid = false;
  in.show_sun_circles = true;
  in.sun_dir[0] = sun_dir[0];
  in.sun_dir[1] = sun_dir[1];
  in.sun_dir[2] = sun_dir[2];
  in.sun_circle_count = count;
  in.sun_circle_angles = circle_angles;
  return in;
}

int CountSunCircleLabels(const std::vector<lumice::gui::OverlayLabel>& labels) {
  int n = 0;
  for (const auto& l : labels) {
    if (l.group == 1)
      ++n;  // group 1 = sun circles
  }
  return n;
}

int CountGridLabels(const std::vector<lumice::gui::OverlayLabel>& labels) {
  int n = 0;
  for (const auto& l : labels) {
    if (l.group == 0)
      ++n;
  }
  return n;
}

}  // namespace

void RegisterOverlayLabelTests(ImGuiTestEngine* engine) {
  // Test A: Front-mode rectangular grid produces strictly fewer labels than Full-mode.
  // Rectangular (lens_type=7) maps the full sphere to the viewport rectangle, so edge sampling
  // exercises grid crossings across the entire sphere — cleanest baseline for hemisphere-cull comparison.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_labels", "front_rectangular_grid_culls_back");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      auto in_full = MakeGridOnly(/*visible=*/2, /*lens=Rectangular*/ 7, /*elev*/ 0.0f, /*az*/ 0.0f);
      auto in_front = MakeGridOnly(/*visible=*/3, /*lens=Rectangular*/ 7, /*elev*/ 0.0f, /*az*/ 0.0f);

      std::vector<lumice::gui::OverlayLabel> labels_full;
      std::vector<lumice::gui::OverlayLabel> labels_front;
      lumice::gui::ComputeOverlayLabels(in_full, 0.0f, 0.0f, 800.0f, 400.0f, labels_full);
      lumice::gui::ComputeOverlayLabels(in_front, 0.0f, 0.0f, 800.0f, 400.0f, labels_front);

      int n_full = CountGridLabels(labels_full);
      int n_front = CountGridLabels(labels_front);

      IM_CHECK_GT(n_full, 0);        // baseline sanity
      IM_CHECK_LT(n_front, n_full);  // Front strictly fewer (back hemisphere labels culled)
    };
  }

  // Test B: Front-mode + sun behind camera → sun-circle interior labels suppressed.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_labels", "front_sun_circle_back_suppressed");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      // forward (elev=0,az=0,roll=0) = (-1,0,0); sun_dir=(1,0,0) places the sun directly behind.
      const float sun_back[3] = { 1.0f, 0.0f, 0.0f };
      const float angle = 5.0f;  // small circle stays inside the viewport (interior labels path)
      auto in = MakeSunOnly(/*visible=*/3, sun_back, &angle, 1);

      std::vector<lumice::gui::OverlayLabel> labels;
      lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 800.0f, 600.0f, labels);

      // All 4 interior labels lie within ±5° of sun_dir in the back hemisphere → all culled.
      IM_CHECK_EQ(CountSunCircleLabels(labels), 0);
    };
  }

  // Test C: Front-mode + sun in front → sun-circle interior labels retained.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_labels", "front_sun_circle_front_retained");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      // sun_dir = forward = (-1,0,0); 4 interior labels all in front hemisphere.
      const float sun_front[3] = { -1.0f, 0.0f, 0.0f };
      const float angle = 5.0f;
      auto in = MakeSunOnly(/*visible=*/3, sun_front, &angle, 1);

      std::vector<lumice::gui::OverlayLabel> labels;
      lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, 800.0f, 600.0f, labels);

      // Expect at least some interior labels (the projection may also drop some by viewport bounds,
      // but at least 1 should survive for a small 5° circle near the optical axis).
      IM_CHECK_GT(CountSunCircleLabels(labels), 0);
    };
  }

  // Test D: Non-Front modes — sun-circle interior labels behavior unchanged.
  // Regression guard for is_visible_front returning true unconditionally outside Front mode.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_labels", "non_front_sun_circle_unchanged");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      // sun_dir = forward = (-1,0,0); same scenario as Test C but in Full / Upper / Lower modes.
      // Per F4 in plan: interior block historically had no is_visible filtering, so the four labels
      // must be produced identically regardless of mode. If is_visible_front mistakenly culled in
      // non-Front modes, n_full/n_upper/n_lower would drop below n_baseline = 4. Sun must be front-
      // facing so labels actually pass WorldDirToPixel; otherwise WorldDirToPixel suppresses them
      // first and the assertion degenerates to 0 == 0 (no regression detection).
      const float sun_front[3] = { -1.0f, 0.0f, 0.0f };
      const float angle = 5.0f;

      std::vector<lumice::gui::OverlayLabel> labels_full;
      std::vector<lumice::gui::OverlayLabel> labels_upper;
      std::vector<lumice::gui::OverlayLabel> labels_lower;
      auto in_full = MakeSunOnly(/*visible=Full*/ 2, sun_front, &angle, 1);
      auto in_upper = MakeSunOnly(/*visible=Upper*/ 0, sun_front, &angle, 1);
      auto in_lower = MakeSunOnly(/*visible=Lower*/ 1, sun_front, &angle, 1);

      lumice::gui::ComputeOverlayLabels(in_full, 0.0f, 0.0f, 800.0f, 600.0f, labels_full);
      lumice::gui::ComputeOverlayLabels(in_upper, 0.0f, 0.0f, 800.0f, 600.0f, labels_upper);
      lumice::gui::ComputeOverlayLabels(in_lower, 0.0f, 0.0f, 800.0f, 600.0f, labels_lower);

      int n_full = CountSunCircleLabels(labels_full);
      int n_upper = CountSunCircleLabels(labels_upper);
      int n_lower = CountSunCircleLabels(labels_lower);
      IM_CHECK_GT(n_full, 0);  // baseline sanity (non-zero so regression check is meaningful)
      IM_CHECK_EQ(n_full, n_upper);
      IM_CHECK_EQ(n_full, n_lower);
    };
  }
}
