// The right panel's Overlay group: the auxiliary lines drawn over the preview and the angle list
// behind the Angular Dist. row's fold.
//
// What this suite is for. `RenderOverlaysTab` (src/gui/app_panels.cpp) draws five rows of the same
// record — colour, name, line, text label, opacity — as one ImGui table, and its one non-obvious
// property is a LAYOUT one: the columns have to line up across rows whose names have very different
// widths, and the longest name must not be cut off. That is only checkable against a rendered
// frame, and it is the kind of thing that silently stops being true when a name is added or a
// column's width budget grows. It matters more here than it would in a wider container: the group
// lives in a 300 px panel, so the name column is what every other column's declared width leaves
// over, and there is no slack to absorb a change.
//
// It used to be checkable in a weaker form: the checkboxes were placed at an x computed from the
// widest name, and the cases below asserted that the anchors agreed. They now assert against the
// table's own column rectangles, which is a stronger reading of the same claim — an anchor can be
// consistent across rows and still overlap a name, whereas a column cannot.
//
// The angle editor behind the Angular Dist. row's fold is the other half: it is a popup that only
// exists while a sun-circle overlay is on, and its preset buttons have to know which angles are
// already in the list, or the user gets duplicates that draw on top of each other.
//
// Deliberately NOT here, with where each lives instead. Where the labels are PLACED around the sky
// is unit-correctness/gui/test_overlay_labels.cpp; the overlay colours reaching the renderer is
// composition-correctness/gui/test_document_roundtrip_chain.cpp; the View and Display groups above
// this one are functional/test_view_display_controls.cpp.
//
// One proposition is recorded rather than covered: that overlay labels are drawn on the preview
// window's own draw list, and are therefore OCCLUDED by a modal rather than floating over it (P39).
// The observable is which draw list received the commands, and the test engine offers no reading of
// that — what it would take is a committed-pixel comparison with a modal open over a labelled
// preview, i.e. a fifth reference group.
//
// What a user sees when these break: the row's name clipped mid-word, a checkbox sitting under a
// name, or a 22-degree halo circle added twice and drawn at double brightness.

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"
// imgui_internal.h is normally an anti-pattern in this suite. One claim below has no public
// reading: what text a table's HEADER draws. A blank header submits no addressable item, and the
// word this one stopped drawing ("Overlay") is still on screen one line above it as the group's
// CollapsingHeader — so a "**/Overlay" lookup answers about the wrong widget whether the header is
// blank or not. TableGetColumnName reads the name the column was actually set up with.
#include "imgui_internal.h"
#include "test_gui_shared.hpp"

namespace {

// RAII pairing for ImGuiTestContext::SetRef, for the same mechanical reason ScopedPopups exists
// (see its comment in test_gui_shared.hpp): a fatal IM_CHECK expands to `return` in the enclosing
// lambda, so a tail-of-function `SetRef("")` only runs when the case passes — and the assertions
// it sits behind are exactly the ones a real regression would trip. Without this, a genuine UI
// regression here would leave the ref pinned to "//##RightPanel" for the rest of the test process,
// turning one real failure into a cascade of unrelated false negatives in later cases.
struct ScopedRef {
  ScopedRef(ImGuiTestContext* ctx, const char* ref) : ctx_(ctx) { ctx_->SetRef(ref); }
  ~ScopedRef() { ctx_->SetRef(""); }

  ScopedRef(const ScopedRef&) = delete;
  ScopedRef& operator=(const ScopedRef&) = delete;

 private:
  ImGuiTestContext* ctx_;
};

// The five rows, by the id suffix their widgets carry.
const char* const kRows[] = { "horizon", "grid", "sun_circles", "zenith_nadir", "lens_border" };

// The name each row displays. Mirrored rather than read from the panel because the panel's row table
// is file-local to app_panels.cpp — and because a name that changed there without changing here is
// precisely the event the width check below exists to catch. "Angular Dist." is abbreviated in the
// panel too, and deliberately: spelled out it does not fit this panel's name column.
const char* NameOfRow(const std::string& row) {
  if (row == "horizon") {
    return "Horizon";
  }
  if (row == "grid") {
    return "Grid";
  }
  if (row == "sun_circles") {
    return "Angular Dist.";
  }
  if (row == "zenith_nadir") {
    return "Zenith/Nadir";
  }
  return "Lens Border";
}

}  // namespace

void RegisterOverlayControlTests(ImGuiTestEngine* engine) {
  // P32's layout half, in its table form. Two claims, and they are different: every row's Line
  // checkbox starts at the same x (a column is a column), and no row's NAME runs into it (the name
  // column is wide enough for the longest name there is). The second is the one the table exists
  // for — the previous layout satisfied the first while clipping "Angular Distance".
  //
  // Asserted as a relation between the items rather than against pixel constants, so it stays true
  // on a platform whose glyphs render wider instead of quietly shipping an overlap.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_controls", "the_columns_line_up_and_no_name_is_cut_off");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      // Named ref, not the default: the Overlay group is the last one in a scrollable panel, and a
      // wildcard lookup resolves an item by its LABEL — the engine records no label for a clipped
      // item, so a row scrolled out of view reads exactly like a missing one. The engine recovers by
      // panning the window, but only when the ref names the window to pan.
      const ScopedRef panel_ref(ctx, "//##RightPanel");

      float line_x = -1.0f;
      for (const char* row : kRows) {
        const ImGuiTestItemInfo line = ctx->ItemInfo(("**/##" + std::string(row) + "_line").c_str());
        // The swatch's id is REPRODUCED rather than searched for, because a "**/" search cannot
        // reach it however it is spelled: ColorButton never calls IMGUI_TEST_ENGINE_ITEM_INFO, so
        // it registers no debug label, and a wildcard matches by label (same quirk, and the same
        // fix, as test_run_lifecycle.cpp's "$$0/##color/##ColorButton" and test_color_window.cpp's
        // ComboPick). Three seeds, outermost first: the window the row was submitted into — taken
        // from the checkbox rather than derived; the table's override id, which every cell widget
        // hashes against instead of the window's (imgui_tables.cpp); and ColorEdit3's PushID of its
        // own label, under which it submits ImGui's "##ColorButton".
        ImGuiID color_id = 0;
        if (line.Window != nullptr) {
          const ImGuiID table_id = ImGui::GetIDWithSeed("##OverlaysTable", nullptr, line.Window->ID);
          const ImGuiID swatch_group =
              ImGui::GetIDWithSeed(("##" + std::string(row) + "_color").c_str(), nullptr, table_id);
          color_id = ImGui::GetIDWithSeed("##ColorButton", nullptr, swatch_group);
        }
        const ImGuiTestItemInfo color = ctx->ItemInfo(color_id, ImGuiTestOpFlags_NoError);
        if (line.ID == 0 || color.ID == 0) {
          IM_ERRORF("row %s: the Line checkbox or the colour swatch is missing", row);
          break;
        }
        if (line_x < 0.0f) {
          line_x = line.RectFull.Min.x;
        } else if (line.RectFull.Min.x != line_x) {
          IM_ERRORF("row %s: Line starts at x=%.1f, the first row measured %.1f", row,
                    static_cast<double>(line.RectFull.Min.x), static_cast<double>(line_x));
          break;
        }
        // The name sits between the swatch and the Line column, and it is the widest thing in that
        // gap that has to fit. Reading the text width rather than a rendered rect because
        // ImGui::TextUnformatted submits no addressable item: a clipped name would not report a
        // narrower rect, it would report nothing at all.
        const float name_w = ImGui::CalcTextSize(NameOfRow(row)).x;
        const float name_room = line.RectFull.Min.x - color.RectFull.Max.x;
        if (name_room < name_w) {
          IM_ERRORF("row %s: %.1f px between the swatch and Line, the name needs %.1f", row,
                    static_cast<double>(name_room), static_cast<double>(name_w));
          break;
        }
      }
      IM_CHECK_GT(line_x, 0.0f);  // a run of five misses would leave this unset

      // "Empty cell IS the information" (doc/gui-visual-language.md §4.4): the marker row carries no
      // text label, so its Label cell holds nothing — not a disabled checkbox, not a dash. The lens
      // border row says the same thing for the same reason: what it draws is a circle, not a word.
      // Asserted against the three rows that DO have one, so the claim is about these two rows and
      // not about the id being spelled some other way.
      IM_CHECK_EQ(ctx->ItemInfo("**/##zenith_nadir_label", ImGuiTestOpFlags_NoError).ID, (ImGuiID)0);
      IM_CHECK_EQ(ctx->ItemInfo("**/##lens_border_label", ImGuiTestOpFlags_NoError).ID, (ImGuiID)0);
      IM_CHECK_NE(ctx->ItemInfo("**/##horizon_label").ID, (ImGuiID)0);
      IM_CHECK_NE(ctx->ItemInfo("**/##grid_label").ID, (ImGuiID)0);
      IM_CHECK_NE(ctx->ItemInfo("**/##sun_circles_label").ID, (ImGuiID)0);

      // The header row. The name column draws none, because the group's own CollapsingHeader
      // already says "Overlay" one line above it and a word repeated directly under itself reads as
      // a second, different thing. Stated together with the four that DO draw one, so "blank"
      // cannot be satisfied by a header row that failed to render at all. The table is found by the
      // same three-seed id the swatch lookup above reconstructs.
      const ImGuiTestItemInfo any_row = ctx->ItemInfo("**/##horizon_line");
      ImGuiTable* table = nullptr;
      if (any_row.Window != nullptr) {
        table = ImGui::TableFindByID(ImGui::GetIDWithSeed("##OverlaysTable", nullptr, any_row.Window->ID));
      }
      IM_CHECK(table != nullptr);
      // Everything from "##" on is id, not text — what a header draws is what comes before it.
      auto header_text = [table](int column_n) {
        const char* name = ImGui::TableGetColumnName(table, column_n);
        const char* id_part = std::strstr(name, "##");
        return std::string(name, id_part != nullptr ? static_cast<size_t>(id_part - name) : std::strlen(name));
      };
      IM_CHECK(header_text(1).empty());
      IM_CHECK_EQ(header_text(2), std::string("Line"));
      IM_CHECK_EQ(header_text(3), std::string("Label"));
      IM_CHECK_EQ(header_text(4), std::string("Alpha"));
    };
  }

  // The other half of "empty cell is the information": the fold column. Only two of the five rows
  // have a field the others lack, and only those two offer the button — a fold button on every row
  // would say the opposite of what this layout is for.
  //
  // Read in the DEFAULT document state, both sun-circle switches off, and that is the point: which
  // rows offer a fold is a property of the ROWS — of which of them owns a field the others lack —
  // and not of any other control's current value. Opening the one this case is named for is part of
  // the same claim: a button that is offered but leads nowhere is not an offer.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_controls", "only_the_rows_with_an_extra_field_offer_a_fold");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(3);
      IM_CHECK(!gui::g_state.show_sun_circles_line);
      IM_CHECK(!gui::g_state.show_sun_circles_label);

      {
        // Named ref for the reason the case above gives: without it a fold button scrolled out of
        // view would satisfy the negative checks as readily as one that is not offered. Scoped to
        // just this block because the editor it opens is a different window, which a
        // panel-prefixed ref would exclude.
        const ScopedRef panel_ref(ctx, "//##RightPanel");

        IM_CHECK(!ctx->ItemExists("**/###horizon_fold"));
        IM_CHECK(!ctx->ItemExists("**/###grid_fold"));
        IM_CHECK(ctx->ItemExists("**/###sun_circles_fold"));
        IM_CHECK(ctx->ItemExists("**/###zenith_nadir_fold"));
        // No fold: unlike the marker pair, the lens border owns no field of its own — the shader
        // derives the circle from the lens, the FOV and the viewport, so there is nothing to edit.
        IM_CHECK(!ctx->ItemExists("**/###lens_border_fold"));

        ctx->ItemClick("**/###sun_circles_fold");
      }
      ctx->Yield(3);
      // A preset button, i.e. the angle editor really is on screen and not merely a button that
      // consumed a click.
      IM_CHECK(ctx->ItemExists("**/9\xc2\xb0"));

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
    };
  }

  // P32, restated. Reaching the angle editor is a property of the Angular Dist. row itself — the
  // row owns a field the others lack, so it offers a fold — and not of whether the circles happen
  // to be drawn at the moment. All four combinations of the row's two switches, because "reachable
  // regardless" is a claim about the whole square and not about the one corner a new document
  // opens in.
  //
  // It used to be the opposite claim: the button appeared only once a sun-circle overlay was on.
  // Stacked vertically that condition had nothing to be read against; in the table its cell sits
  // directly above an unconditional fold on the Zenith/Nadir row, and an empty cell there reads as
  // the feature having gone missing rather than as a considered condition. There is deliberately
  // no !ItemExists assertion left in this case — the invariant is that the fold is offered in
  // every state, so any surviving negative check would be re-asserting the gate somewhere else.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "overlay_controls", "the_angle_editor_is_reachable_regardless_of_the_circles_toggle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(3);
      IM_CHECK(!gui::g_state.show_sun_circles_line);
      IM_CHECK(!gui::g_state.show_sun_circles_label);

      // Both helpers name the right panel for their lookup, for the reason the cases above give: a
      // fold button scrolled out of view is indistinguishable from one that is not offered unless
      // the ref names the window the engine may pan. Neither helper reports a failure itself — it
      // returns, and the caller checks — so a failure is fatal to the CASE instead of merely
      // ending the helper and leaving the rest of the case driving an invalid state.
      auto click_in_panel = [ctx](const char* item) {
        const ScopedRef panel_ref(ctx, "//##RightPanel");
        ctx->ItemClick(item);
      };
      auto editor_opens = [ctx]() -> bool {
        {
          const ScopedRef panel_ref(ctx, "//##RightPanel");
          if (!ctx->ItemExists("**/###sun_circles_fold")) {
            return false;
          }
          ctx->ItemClick("**/###sun_circles_fold");
        }
        ctx->Yield(3);
        // Released before the lookup: the editor is a popup, a different window.
        const bool opened = ctx->ItemExists("**/9\xc2\xb0");
        ctx->KeyPress(ImGuiKey_Escape);
        ctx->Yield(2);
        return opened;
      };

      IM_CHECK(editor_opens());  // neither switch: the state a new document opens in

      click_in_panel("**/##sun_circles_line");
      ctx->Yield(3);
      IM_CHECK(gui::g_state.show_sun_circles_line);
      IM_CHECK(editor_opens());  // lines only

      click_in_panel("**/##sun_circles_label");
      ctx->Yield(3);
      IM_CHECK(gui::g_state.show_sun_circles_label);
      IM_CHECK(editor_opens());  // lines and labels

      click_in_panel("**/##sun_circles_line");
      ctx->Yield(3);
      IM_CHECK(!gui::g_state.show_sun_circles_line);
      IM_CHECK(editor_opens());  // labels only

      click_in_panel("**/##sun_circles_label");
      ctx->Yield(2);
    };
  }

  // P32. The preset buttons are the fast path into the angle list, and each one has to know whether
  // its angle is already there — a second 22-degree circle is not a second circle, it is the same
  // circle drawn twice at double opacity. Adding also keeps the list sorted, so the delete buttons
  // below it stay in the order the user reads.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "overlay_controls", "an_angle_preset_disables_once_it_is_in_the_sorted_list");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      gui::g_state.show_sun_circles_line = true;
      gui::g_state.sun_circle_angles.clear();
      ctx->Yield(3);

      // Named ref for the fold button only, for the reason given in the case above. Scoped to just
      // this click is deliberate: the presets and the delete rows live in the popup, a different
      // window, which a panel-prefixed ref would exclude — so the ref must not still be set once we
      // get there.
      {
        const ScopedRef panel_ref(ctx, "//##RightPanel");
        ctx->ItemClick("**/###sun_circles_fold");
      }
      ctx->Yield(3);

      // All four presets the editor offers (9 / 22 / 28 / 46 degrees), deliberately clicked out of
      // order so "sorted" is a claim about the insert rather than about the order they arrived in.
      // The list is mirrored rather than iterated because the panel's own array is file-local to
      // app_panels.cpp; a preset added there without a row here shows up as the count check below
      // disagreeing, not as a silent gap.
      const char* const kDegrees[] = { "46\xc2\xb0", "9\xc2\xb0", "28\xc2\xb0", "22\xc2\xb0" };
      for (const char* deg : kDegrees) {
        const std::string ref = std::string("**/") + deg;
        if (IsDisabled(ctx->ItemInfo(ref.c_str()))) {
          IM_ERRORF("preset %s was already disabled before it was clicked", deg);
          continue;
        }
        ctx->ItemClick(ref.c_str());
        ctx->Yield(2);
        if (!IsDisabled(ctx->ItemInfo(ref.c_str()))) {
          IM_ERRORF("preset %s is still offered after it was added", deg);
        }

        if (ctx->IsError()) {
          break;
        }
      }

      IM_CHECK_EQ(gui::g_state.sun_circle_angles.size(), (size_t)4);
      IM_CHECK(std::is_sorted(gui::g_state.sun_circle_angles.begin(), gui::g_state.sun_circle_angles.end()));
      IM_CHECK_EQ(gui::g_state.sun_circle_angles.front(), 9.0f);
      IM_CHECK_EQ(gui::g_state.sun_circle_angles.back(), 46.0f);

      // The per-row delete removes THAT angle, not the last one — the rows are drawn in list order,
      // so removing row 0 must take the smallest.
      ctx->ItemClick("**/x##del_0");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.sun_circle_angles.size(), (size_t)3);
      IM_CHECK_EQ(gui::g_state.sun_circle_angles.front(), 22.0f);
      // ...and the preset it freed is on offer again.
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/9\xc2\xb0")));

      ctx->KeyPress(ImGuiKey_Escape);  // dismiss the popup
      ctx->Yield(2);
      gui::g_state.show_sun_circles_line = false;
      ctx->Yield(2);
    };
  }

  // The marker radius, the other field behind a fold. Same proposition as the angle list above —
  // that folding a minority field away did not put it out of reach — and it needs stating
  // separately because the two folds hold entirely different editors.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_controls", "the_marker_radius_is_editable_behind_its_fold");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(3);

      {
        // Named ref so the negative check reads "not submitted inline" and not "scrolled past" —
        // see the case above. Released before the popup, which is a different window.
        const ScopedRef panel_ref(ctx, "//##RightPanel");
        // The selector lost SliderWithInput's "_input" half along with the control: the radius is
        // the same DragFloatField the alpha cells use now, and that submits ONE item whose id is
        // "##" + the label it was handed.
        IM_CHECK(!ctx->ItemExists("**/##Radius##zenith_nadir"));  // folded away, not shown inline
        ctx->ItemClick("**/###zenith_nadir_fold");
      }
      ctx->Yield(3);

      // Both ends of the declared domain (2..20 px), so the popup's control is the registry's
      // control and not a second opinion about the range.
      ctx->ItemInputValue("**/##Radius##zenith_nadir", 100.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, 20.0f);
      ctx->ItemInputValue("**/##Radius##zenith_nadir", -5.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, 2.0f);

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
    };
  }

  // The five alpha cells, at both ends of each declared domain. Literals throughout, for the reason
  // spelled out at the head of functional/test_scene_controls.cpp: the call site reads the registry,
  // so asking the registry what to expect would compare one line of code against itself.
  //
  // The cell is a single DragFloat now rather than a [slider][input] pair, and ItemInputValue drives
  // it through its text-entry path — which is exactly the path that only clamps because
  // DragFloatField passes ImGuiSliderFlags_AlwaysClamp and then clamps again unconditionally, so
  // this is a check on that helper as much as on the domain.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_controls", "the_overlay_alphas_clamp_to_their_declared_domains");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      const ScopedRef panel_ref(ctx, "//##RightPanel");

      float* const kSlots[] = {
        &gui::g_state.horizon_alpha,      &gui::g_state.grid_alpha,        &gui::g_state.sun_circles_alpha,
        &gui::g_state.zenith_nadir_alpha, &gui::g_state.lens_border_alpha,
      };
      for (size_t i = 0; i < IM_ARRAYSIZE(kRows); ++i) {
        const std::string ref = "**/##" + std::string(kRows[i]) + "_alpha";
        ctx->ItemInputValue(ref.c_str(), 100.0f);
        ctx->Yield();
        if (*kSlots[i] != 1.0f) {
          IM_ERRORF("%s: clamped to %f, expected the 1.0 maximum", ref.c_str(), static_cast<double>(*kSlots[i]));
        }
        if (ctx->IsError()) {
          break;
        }
        ctx->ItemInputValue(ref.c_str(), -100.0f);
        ctx->Yield();
        if (*kSlots[i] != 0.0f) {
          IM_ERRORF("%s: clamped to %f, expected the 0.0 minimum", ref.c_str(), static_cast<double>(*kSlots[i]));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }
}
