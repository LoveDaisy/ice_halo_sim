// The right panel's Overlay group: the auxiliary lines drawn over the preview and the angle list
// behind the sun-circle row.
//
// What this suite is for. `RenderRightPanel`'s overlay section (src/gui/app_panels.cpp) draws three
// rows of the same shape plus a zenith/nadir row, and its one non-obvious property is a LAYOUT one:
// the Line and Label checkboxes are anchored at fixed x so the two columns line up across rows
// whose names have very different widths. That is only checkable against a rendered frame, and it
// is the kind of thing that silently stops being true when a name is added — the widest name today
// is "Angular Distance" and the anchors are derived from it.
//
// The angle editor behind "Edit Angles..." is the other half: it is a popup that only exists while
// a sun-circle overlay is on, and its preset buttons have to know which angles are already in the
// list, or the user gets duplicates that draw on top of each other.
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
// What a user sees when these break: a "Label" checkbox sitting under the word "Angular Distance",
// or a 22-degree halo circle added twice and drawn at double brightness.

#include <algorithm>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {}  // namespace

void RegisterOverlayControlTests(ImGuiTestEngine* engine) {
  // P32's layout half. The three rows carry names of different widths and their checkboxes are
  // placed with an explicit SameLine offset rather than by flow, so "the columns line up" is a
  // claim about that offset being wide enough for the longest name — and the longest name is the
  // one that stops fitting first.
  //
  // Asserted as a relation between the items rather than against a pixel constant, so it stays true
  // on a platform whose glyphs render wider instead of quietly shipping an overlap.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_controls", "the_line_and_label_columns_line_up_across_the_rows");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      const char* const kRows[] = { "horizon", "grid", "sun_circles" };
      float line_x = -1.0f;
      float label_x = -1.0f;
      for (const char* row : kRows) {
        const ImGuiTestItemInfo line = ctx->ItemInfo(("**/Line##" + std::string(row)).c_str());
        const ImGuiTestItemInfo label = ctx->ItemInfo(("**/Label##" + std::string(row)).c_str());
        if (line.ID == 0 || label.ID == 0) {
          IM_ERRORF("row %s: Line or Label is missing", row);
          continue;
        }
        if (line_x < 0.0f) {
          line_x = line.RectFull.Min.x;
          label_x = label.RectFull.Min.x;
        }
        if (line.RectFull.Min.x != line_x) {
          IM_ERRORF("row %s: Line starts at x=%.1f, the first row measured %.1f", row,
                    static_cast<double>(line.RectFull.Min.x), static_cast<double>(line_x));
        }
        if (label.RectFull.Min.x != label_x) {
          IM_ERRORF("row %s: Label starts at x=%.1f, the first row measured %.1f", row,
                    static_cast<double>(label.RectFull.Min.x), static_cast<double>(label_x));
        }
        // The two columns are distinct and in order — an anchor that collapsed would satisfy the
        // equalities above on its own.
        if (label.RectFull.Min.x <= line.RectFull.Max.x) {
          IM_ERRORF("row %s: Label at %.1f overlaps Line ending at %.1f", row,
                    static_cast<double>(label.RectFull.Min.x), static_cast<double>(line.RectFull.Max.x));
        }

        if (ctx->IsError()) {
          break;
        }
      }
      IM_CHECK_GT(line_x, 0.0f);  // a run of three misses would leave this unset

      // The zenith/nadir row has no Label column but shares the Line anchor, which is what keeps it
      // reading as part of the same table rather than as a stray checkbox.
      IM_CHECK_EQ(ctx->ItemInfo("**/##zenith_nadir_line").RectFull.Min.x, line_x);
    };
  }

  // P32. The angle editor is offered only while a sun-circle overlay is actually being drawn —
  // editing the angle list of something invisible is a control with no feedback.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "overlay_controls", "the_angle_editor_is_offered_only_while_the_circles_show");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(3);
      IM_CHECK(!gui::g_state.show_sun_circles_line);
      IM_CHECK(!gui::g_state.show_sun_circles_label);
      IM_CHECK(!ctx->ItemExists("**/Edit Angles...##overlay"));

      // Either switch is enough — the lines and the labels are two ways of showing the same set.
      ctx->ItemClick("**/Line##sun_circles");
      ctx->Yield(3);
      IM_CHECK(gui::g_state.show_sun_circles_line);
      IM_CHECK(ctx->ItemExists("**/Edit Angles...##overlay"));

      ctx->ItemClick("**/Line##sun_circles");
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists("**/Edit Angles...##overlay"));

      ctx->ItemClick("**/Label##sun_circles");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists("**/Edit Angles...##overlay"));

      ctx->ItemClick("**/Label##sun_circles");
      ctx->Yield(3);
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
      ctx->Yield(3);
      gui::g_state.show_sun_circles_line = true;
      gui::g_state.sun_circle_angles.clear();
      ctx->Yield(3);

      ctx->ItemClick("**/Edit Angles...##overlay");
      ctx->Yield(3);

      // All four presets the panel offers (9 / 22 / 28 / 46 degrees), deliberately clicked out of
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

  // The overlay group's sliders, at both ends of each declared domain. Literals throughout, for the
  // reason spelled out at the head of functional/test_scene_controls.cpp: the call site reads the
  // registry, so asking the registry what to expect would compare one line of code against itself.
  //
  // The zenith/nadir pair is the reason this case scrolls. With every group expanded those two rows
  // sit below the fold of the right panel, and a clipped item is never submitted — so the engine
  // cannot find them by id at all. The panel is handed back at the top afterwards: gui_test is one
  // process and a scroll position outlives the case that set it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_controls", "the_overlay_sliders_clamp_to_their_declared_domains");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Slider {
        const char* input;
        float* slot;
        float lo;
        float hi;
        bool below_the_fold;
      };
      ResetTestState();
      ctx->Yield(3);

      const Slider kSliders[] = {
        { "**/##Alpha##horizon_input", &gui::g_state.horizon_alpha, 0.0f, 1.0f, false },
        { "**/##Alpha##grid_input", &gui::g_state.grid_alpha, 0.0f, 1.0f, false },
        { "**/##Alpha##sun_circles_input", &gui::g_state.sun_circles_alpha, 0.0f, 1.0f, false },
        { "**/##Alpha##zenith_nadir_input", &gui::g_state.zenith_nadir_alpha, 0.0f, 1.0f, true },
        // The one overlay slider whose domain is neither [0, 1] nor symmetric.
        { "**/##Radius##zenith_nadir_input", &gui::g_state.zenith_nadir_radius_px, 2.0f, 20.0f, true },
      };

      bool scrolled = false;
      for (const Slider& s : kSliders) {
        if (s.below_the_fold && !scrolled) {
          ctx->ScrollToBottom("//##RightPanel");
          ctx->Yield(2);
          scrolled = true;
        }
        ctx->ItemInputValue(s.input, s.hi + 100.0f);
        ctx->Yield();
        if (*s.slot != s.hi) {
          IM_ERRORF("%s: clamped to %f, expected the %f maximum", s.input, static_cast<double>(*s.slot),
                    static_cast<double>(s.hi));
        }
        if (ctx->IsError()) {
          break;
        }
        ctx->ItemInputValue(s.input, s.lo - 100.0f);
        ctx->Yield();
        if (*s.slot != s.lo) {
          IM_ERRORF("%s: clamped to %f, expected the %f minimum", s.input, static_cast<double>(*s.slot),
                    static_cast<double>(s.lo));
        }

        if (ctx->IsError()) {
          break;
        }
      }

      ctx->ScrollToTop("//##RightPanel");
      ctx->Yield(2);
    };
  }
}
