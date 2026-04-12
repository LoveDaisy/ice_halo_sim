#include "test_gui_shared.hpp"

void RegisterSmokeTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "gui_smoke", "default_state");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Verify default state: 1 crystal, 1 renderer, 1 scattering layer
    IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.renderers.size()), 1);
    IM_CHECK_EQ(gui::g_state.selected_crystal, 0);
    IM_CHECK_EQ(gui::g_state.selected_renderer, 0);
    IM_CHECK_EQ(gui::g_state.dirty, false);
    IM_CHECK_EQ(gui::g_state.sim_state, gui::GuiState::SimState::kIdle);
  };
}
