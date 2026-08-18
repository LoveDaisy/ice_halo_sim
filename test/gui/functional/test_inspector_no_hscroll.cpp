// The form column never scrolls sideways (doc/gui-visual-language.md §4.7).
//
// What this suite is for. A vertical scrollbar on the inspector is ordinary — a page can be taller
// than its half of the column and the user scrolls it. A HORIZONTAL one is not: it says some single
// row is wider than the panel, and because the panel is one column shared by six pages, that row
// has made every other page's width its business. That is the mechanism behind the two complaints
// this scrum started from — "too wide to look precise" and "not wide enough" are the same defect
// seen from two sides, because with nothing carrying an intrinsic width the longest row is what
// decides the column (scrum §0). So the rule is stated as a property of the column rather than as a
// per-row review habit: no page, at any width the user can drag the column to, may overflow it.
//
// Why a live frame. `ImGuiWindow::ScrollMax.x` is the measurement, and it exists only in a rendered
// frame. It is computed in Begin() from the previous frame's ContentSize regardless of whether the
// window carries ImGuiWindowFlags_HorizontalScrollbar (imgui.cpp, "Lock down maximum scrolling") —
// the inspector does not, so an overflow here is invisible rather than scrollable, which is exactly
// why it needs a mechanical detector instead of an eye.
//
// The two widths. The default is what a fresh install shows. The narrow one is a deliberate audit
// width, well under the default, standing in for "the user dragged the separator in": a page that
// only fits at the default width would make the separator a trap rather than a control. Both are
// driven through gui::ResizePanelNode, the same entry point the column's own collapse uses.
//
// What a user sees when these break: a row of buttons or a sentence running off the right edge of
// the inspector with no scrollbar to bring it back, and every page in the column widened to make
// room for one of them.

#include <string>

#include "gui/dock_layout.hpp"
#include "gui/gui_constants.hpp"
#include "imgui_internal.h"
#include "test_gui_shared.hpp"

namespace {

// Narrower than any default this scrum is likely to land on, and narrow enough that the axis
// preset row — the widest single row the inspector draws — has to wrap to fit. It is an audit
// width, not a supported minimum: nothing stops the user dragging further, and the pages below
// simply have to keep their content inside whatever they are given.
constexpr float kNarrowColumnWidth = 300.0f;

// Wide enough that the six presets and their lead-in fit on one line with room to spare — the
// other end of the preset row's wrap, and the only way to tell "it wrapped because it had to"
// from "it always draws that way".
constexpr float kWideColumnWidth = 600.0f;

// Sets the document column's width for the duration of a scope and puts it back afterwards.
//
// A destructor rather than two statements around the checks: ResetTestState deliberately does NOT
// rebuild the dock layout (see its note in test_gui_main.cpp), so a case that leaves the column
// narrowed hands every case after it a different panel width — and the failures that causes land
// nowhere near this file. The parent split node is what gets resized; ImGui distributes a split
// node's size down to both halves on the same frame, so the tree and the inspector move together.
struct ScopedColumnWidth {
  ScopedColumnWidth(ImGuiTestContext* ctx, float width) : ctx_(ctx) {
    node_ = gui::GetPanelNodeIds().left;
    height_ = gui::GetPanelNodeHeight(node_);
    original_width_ = gui::GetPanelNodeWidth(node_);
    gui::ResizePanelNode(node_, ImVec2(width, height_));
    ctx_->Yield(4);
  }

  ~ScopedColumnWidth() {
    gui::ResizePanelNode(node_, ImVec2(original_width_, height_));
    ctx_->Yield(4);
  }

  ScopedColumnWidth(const ScopedColumnWidth&) = delete;
  ScopedColumnWidth& operator=(const ScopedColumnWidth&) = delete;

 private:
  ImGuiTestContext* ctx_ = nullptr;
  ImGuiID node_ = 0;
  float height_ = 0.0f;
  float original_width_ = 0.0f;
};

// The six pages the inspector can show. The three crystal tabs are three pages of one selection,
// which is why they are listed by tab rather than by selection kind — the column shows one of six
// things, and "which of the six" is what this table enumerates.
struct InspectorPage {
  const char* name;
  void (*Open)(ImGuiTestContext* ctx);
};

const InspectorPage kInspectorPages[] = {
  { "sun",
    [](ImGuiTestContext* ctx) {
      gui::g_state.SelectSun();
      ctx->Yield(3);
    } },
  { "camera",
    [](ImGuiTestContext* ctx) {
      gui::g_state.SelectCamera();
      ctx->Yield(3);
    } },
  { "layer",
    [](ImGuiTestContext* ctx) {
      gui::g_state.SelectLayer(0);
      ctx->Yield(3);
    } },
  { "crystal", [](ImGuiTestContext* ctx) { OpenCrystalTab(ctx); } },
  { "axis", [](ImGuiTestContext* ctx) { OpenAxisTab(ctx); } },
  { "filter", [](ImGuiTestContext* ctx) { OpenFilterTab(ctx); } },
};

// Opens one page and reports (non-fatally) if the inspector ended up scrollable sideways.
//
// Non-fatal on purpose: the caller walks all six pages, and a fatal assert would return out of the
// case on the first one — hiding the other five, which is the difference between "the axis page
// overflows" and "some page overflows" when reading the failure. The caller stops driving after a
// report all the same (a page left half-opened is not a state the next page's checks mean
// anything in).
void CheckPageFitsColumn(ImGuiTestContext* ctx, const InspectorPage& page, const char* width_label) {
  page.Open(ctx);
  // ScrollMax.x is computed in Begin() from the PREVIOUS frame's ContentSize, so the page has to
  // have been submitted at least once at this width before the number means anything.
  ctx->Yield(3);

  ImGuiWindow* inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
  if (inspector == nullptr) {
    IM_ERRORF("the inspector window is not up on the %s page (%s)", page.name, width_label);
    return;
  }
  if (inspector->ScrollMax.x != 0.0f) {
    IM_ERRORF("the %s page overflows the %s column by %.1f px (window %.1f px wide)", page.name, width_label,
              static_cast<double>(inspector->ScrollMax.x), static_cast<double>(inspector->Size.x));
  }
}

// y of a preset button's top edge, or -1 when the engine cannot see it. Two buttons share a row iff
// they report the same value. Routed through InspectorItemInfo rather than ctx->ItemInfo so the
// lookup is scoped to the inspector (and survives a page tall enough to clip the row), and so a
// button that ends up scrolled out of view reads as found rather than as missing.
float PresetButtonTop(ImGuiTestContext* ctx, const char* label) {
  const ImGuiTestItemInfo info = InspectorItemInfo(ctx, ("**/" + std::string(label)).c_str());
  return info.ID != 0 ? info.RectFull.Min.y : -1.0f;
}

// Every preset the axis page offers, in the order the row draws them.
const char* const kPresetLabels[] = { "Column", "Plate", "Parry", "Lowitz", "Random", "Custom" };

// How many lines the preset row occupies, or 0 when a preset could not be found at all — the
// distinction matters, because "the row is one line" and "the row is one line because the last
// three buttons were clipped away" are the pass and the failure of the same measurement.
//
// Counts transitions rather than distinct values: the buttons are walked in draw order, so a new
// line is exactly a y that differs from the previous button's.
int CountPresetRows(ImGuiTestContext* ctx, const char* width_label) {
  int rows = 1;
  float previous_y = -1.0f;
  for (const char* label : kPresetLabels) {
    const float y = PresetButtonTop(ctx, label);
    if (y < 0.0f) {
      IM_ERRORF("preset %s is not on the axis page (%s column)", label, width_label);
      return 0;
    }
    if (previous_y >= 0.0f && y != previous_y) {
      ++rows;
    }
    previous_y = y;
  }
  return rows;
}

}  // namespace

void RegisterInspectorNoHScrollTests(ImGuiTestEngine* engine) {
  // AC1. The default width is the one a fresh install shows and the one every reference capture is
  // shot at, so an overflow here is what every user sees.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "inspector_no_hscroll", "no_page_overflows_the_default_column");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      // Premise: the column really is at its default width, i.e. no earlier case left the splitter
      // somewhere else. Without this the case still passes when the column is twice as wide as the
      // product ships it — which is the one condition under which it proves nothing.
      IM_CHECK_EQ(gui::GetPanelNodeWidth(gui::GetPanelNodeIds().left), gui::kLeftPanelWidth);

      for (const InspectorPage& page : kInspectorPages) {
        CheckPageFitsColumn(ctx, page, "default-width");
        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // AC1, the other half. Same six pages with the column dragged in: a page that fits only at the
  // default width has made the separator a trap.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "inspector_no_hscroll", "no_page_overflows_a_narrowed_column");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      const ScopedColumnWidth narrowed(ctx, kNarrowColumnWidth);
      // Premise: the resize took. A no-op ResizePanelNode would leave every check below running at
      // the default width and passing for the wrong reason.
      IM_CHECK_EQ(gui::GetPanelNodeWidth(gui::GetPanelNodeIds().left), kNarrowColumnWidth);

      for (const InspectorPage& page : kInspectorPages) {
        CheckPageFitsColumn(ctx, page, "narrowed");
        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // AC2. The positive half of the preset row's story: `ScrollMax.x == 0` alone would also be
  // satisfied by a row that silently dropped its last buttons, or by one that never had to wrap
  // because the column was wide enough — so the wrap itself is asserted, in both directions. At the
  // default width the six presets are one row; narrowed, at least one of them has moved down.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "inspector_no_hscroll", "the_preset_row_wraps_rather_than_widening");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      OpenAxisTab(ctx);
      ctx->Yield(3);

      // Both probes are widths this case sets itself rather than the shipping default, and
      // deliberately so: what is being asserted is that the row RESPONDS to width, which stays a
      // true statement whatever the default is later tuned to. (Whether the default in particular
      // overflows is the sibling case's job, and it reads the default from the constant.)
      {
        const ScopedColumnWidth widened(ctx, kWideColumnWidth);
        const int rows = CountPresetRows(ctx, "widened");
        IM_CHECK_EQ(rows, 1);
      }

      // Narrowed, the row has to break. What must NOT happen is the row staying on one line and
      // pushing the column's content past its right edge, which is what the sibling case measures;
      // here the wrap is observed directly, so a future "fix" that clipped the row instead of
      // wrapping it — dropping the buttons that did not fit — cannot pass both (CountPresetRows
      // reports a missing button rather than counting the rows it can still see).
      {
        const ScopedColumnWidth narrowed(ctx, kNarrowColumnWidth);
        const int rows = CountPresetRows(ctx, "narrowed");
        IM_CHECK_GT(rows, 1);
      }
    };
  }

  // The filter page's over-cap warning is the widest sentence the inspector can draw, and it is
  // drawn only in a state no other case in this file reaches — the default document's filter page
  // never shows it. Without this case the sentence's wrapping is untested, which is how it got to
  // sit beside the clause count on one unbreakable line in the first place.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "inspector_no_hscroll", "the_over_cap_warning_wraps_inside_the_column");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      OpenFilterTab(ctx);
      ctx->Yield(2);

      // Four factors of nine alternatives each: 6561 clauses, past LUMICE_MAX_CONFIG_CLAUSES. Same
      // input as filter_editor's over-cap case, which asserts the editor survives drawing it; this
      // one asserts the column does.
      ctx->ItemInputValue("**/##row_text_0",
                          "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4");
      ctx->Yield(4);

      ImGuiWindow* inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(inspector != nullptr);
      IM_CHECK_EQ(inspector->ScrollMax.x, 0.0f);

      {
        const ScopedColumnWidth narrowed(ctx, kNarrowColumnWidth);
        ImGuiWindow* narrowed_inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
        IM_CHECK(narrowed_inspector != nullptr);
        IM_CHECK_EQ(narrowed_inspector->ScrollMax.x, 0.0f);
      }

      ctx->Yield(2);
    };
  }
}
