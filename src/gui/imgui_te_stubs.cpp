// Stub implementations of ImGui Test Engine hooks.
// When IMGUI_ENABLE_TEST_ENGINE is defined, ImGui calls these hooks.
// The real implementations live in imgui_test_engine (linked by LumiceGUITests).
// LumiceGUI links these no-op stubs instead.

#include "imgui.h"
#include "imgui_internal.h"

void ImGuiTestEngineHook_ItemAdd(ImGuiContext*, ImGuiID, const ImRect&, const ImGuiLastItemData*) {}
void ImGuiTestEngineHook_ItemInfo(ImGuiContext*, ImGuiID, const char*, ImGuiItemStatusFlags) {}
void ImGuiTestEngineHook_Log(ImGuiContext*, const char*, ...) {}
