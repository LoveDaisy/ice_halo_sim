// The theme claims every ImGuiCol_ slot there is.
//
// The proposition is about omission, which is why it needs a case of its own: a slot the palette
// never mentions does not look wrong at review time, it looks like nothing at all, and then ships
// ImGui's default colour under this app's name. It happened — the run-progress bar drew in the
// default amber (0.90, 0.70, 0.00) for as long as the theme existed, i.e. in the warning grade
// this app reserves for "this needs your attention", because ImGuiCol_PlotHistogram was not in the
// palette's list.
//
// How it is checked, and why not the obvious way. Comparing the live style against a pristine
// StyleColorsDark() would be the shorter test and it would be wrong in both directions: two slots
// (BorderShadow, TableRowBg) are deliberately assigned the very value dark already had, so they
// would read as unclaimed, and any future slot whose chosen value happens to coincide would join
// them. Filling a scratch style with a value no palette entry can produce and requiring that none
// of it survives ApplyStyle() asks the actual question — "was this slot written to" — with no
// dependency on WHAT was written.
//
// It also covers slots that do not exist yet: an ImGui upgrade that adds one gets a red here
// rather than a quiet default in the next screenshot.

#include "gui/theme.hpp"
#include "test_gui_shared.hpp"

namespace {

// Outside the range any colour can take (channels are 0..1), so no palette entry can produce it
// and no slot can be "claimed" by coincidence.
constexpr ImVec4 kUnclaimed(-1.0f, -2.0f, -3.0f, -4.0f);

bool IsUnclaimed(const ImVec4& c) {
  return c.x == kUnclaimed.x && c.y == kUnclaimed.y && c.z == kUnclaimed.z && c.w == kUnclaimed.w;
}

}  // namespace

void RegisterThemeCoverageTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "theme_coverage", "the_palette_claims_every_color_slot");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    ImGuiStyle probe;
    for (int i = 0; i < ImGuiCol_COUNT; ++i) {
      probe.Colors[i] = kUnclaimed;
    }

    lumice::gui::ApplyStyle(probe);

    // Names are reported rather than indices: an index moves with every ImGui upgrade, and the
    // whole point of a red here is that someone can go add the missing line.
    std::string unclaimed;
    for (int i = 0; i < ImGuiCol_COUNT; ++i) {
      if (IsUnclaimed(probe.Colors[i])) {
        unclaimed += std::string(" ") + ImGui::GetStyleColorName(i);
      }
    }
    ctx->LogInfo("slots=%d unclaimed:[%s]", ImGuiCol_COUNT, unclaimed.c_str());
    IM_CHECK_STR_EQ(unclaimed.c_str(), "");

    // The live style is the thing the app actually draws with, and it is reached by a different
    // path (ApplyVisualLanguage at startup). Anchoring the one slot whose default leaked into the
    // product keeps this case honest about the defect it came from: whatever the accent is, it is
    // not that amber.
    const ImVec4& hist = ImGui::GetStyle().Colors[ImGuiCol_PlotHistogram];
    IM_CHECK(!(hist.x == 0.90f && hist.y == 0.70f && hist.z == 0.00f));
    // Same accent as the check marks, which is what makes it the theme's accent rather than a
    // second blue that merely looks like it.
    const ImVec4& check = ImGui::GetStyle().Colors[ImGuiCol_CheckMark];
    IM_CHECK_EQ(hist.x, check.x);
    IM_CHECK_EQ(hist.y, check.y);
    IM_CHECK_EQ(hist.z, check.z);
    IM_CHECK_EQ(hist.w, check.w);
  };
}
