#include "test_gui_shared.hpp"

void RegisterSmokeTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "gui_smoke", "default_state");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Verify default state: 1 layer with 1 entry, embedded renderer at default values
    IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);
    IM_CHECK_EQ(gui::g_state.renderer.lens_type, 0);
    IM_CHECK_EQ(gui::g_state.dirty, false);
    IM_CHECK_EQ(gui::g_state.sim_state, gui::GuiState::SimState::kIdle);
  };
}
