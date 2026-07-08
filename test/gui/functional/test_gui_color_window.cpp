// task-342.3 Step 10: gui_test coverage for the Colors window's pure helpers.
//
// These tests exercise the exposed helpers in color_window.hpp directly (no
// ImGui UI drive). Together they nail down:
//   - plan §3 decision 1: z_order and physical vector index stay decoupled —
//     SwapZOrder swaps scalars, never vector entries.
//   - CompactZOrder rebuilds a permutation after delete-class leaves a gap.
//   - plan §3 decision 3: ValidateSingleAtomText rejects multi-factor and
//     multi-alternative text (AND-inside-atom or ; -union).
//   - AC5 decoupling: BuildClassFromFilter clones filter rows into fresh
//     refs by value (no shared filter_id), skipping multi-factor AND rows.
//
// The UI-driven paths (color swatch → LUMICE_SetRaypathColors call, empty-arc
// warning icon rendering, ItemClick-style flows) are left for the AC6 owner
// on-screen manual pass; direct helpers cover the invariants that a UI test
// would only reach indirectly and much more slowly.

#include "gui/color_window.hpp"
#include "gui/raypath_segments.hpp"
#include "test_gui_shared.hpp"

void RegisterColorWindowTests(ImGuiTestEngine* engine) {
  // SwapZOrder swaps only the z_order scalars; vector entries stay pinned
  // to their physical index so the C-API lane binding (GetColorClassLaneY(i))
  // keeps pointing at the same class.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "swap_zorder_does_not_reorder_vector");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig a;
      a.color[0] = 1.0f;
      a.color[1] = 0.0f;
      a.color[2] = 0.0f;
      a.z_order = 0;
      gui::ColorClassConfig b;
      b.color[0] = 0.0f;
      b.color[1] = 1.0f;
      b.color[2] = 0.0f;
      b.z_order = 1;
      gui::g_state.raypath_color.push_back(a);
      gui::g_state.raypath_color.push_back(b);

      gui::SwapZOrder(gui::g_state, 0, 1);

      // Vector index 0 must still be the red class; only z_order was flipped.
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[0], 1.0f);
      IM_CHECK_EQ(gui::g_state.raypath_color[1].color[1], 1.0f);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, 0);
    };
  }

  // Out-of-range and self-swap are no-ops (defensive: the caller is UI code
  // that iterates on user input; a bad index should not corrupt state).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "swap_zorder_ignores_bad_indices");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      c.z_order = 7;
      gui::g_state.raypath_color.push_back(c);

      gui::SwapZOrder(gui::g_state, 0, 0);     // self-swap
      gui::SwapZOrder(gui::g_state, 0, 42);    // out-of-range
      gui::SwapZOrder(gui::g_state, 99, 100);  // both out-of-range
      IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, 7);
    };
  }

  // CompactZOrder rebuilds z_order to [0, N) preserving the user's visible
  // order. Called after delete-class which leaves a hole (e.g. deleting the
  // middle class in [z=0, z=1, z=2] leaves [z=0, z=2] which is not a
  // permutation LUMICE_SetRaypathColors will accept).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "compact_zorder_fills_gaps");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      auto make_cls = [](int z) {
        gui::ColorClassConfig c;
        c.z_order = z;
        return c;
      };
      gui::g_state.raypath_color.push_back(make_cls(5));  // physical 0
      gui::g_state.raypath_color.push_back(make_cls(2));  // physical 1
      gui::g_state.raypath_color.push_back(make_cls(9));  // physical 2

      gui::CompactZOrder(gui::g_state);

      // The visible order (by ascending z) was phys=1, phys=0, phys=2 —
      // compaction assigns 0, 1, 2 in that same rank order.
      IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, 0);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[2].z_order, 2);
    };
  }

  // Delete-in-the-middle simulates the RenderColorWindow post-erase step:
  // erase(1) removes the middle physical class; CompactZOrder then packs the
  // remaining z_order values into [0, N).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "compact_zorder_after_delete_middle");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      auto make_cls = [](int z) {
        gui::ColorClassConfig c;
        c.z_order = z;
        return c;
      };
      gui::g_state.raypath_color.push_back(make_cls(0));
      gui::g_state.raypath_color.push_back(make_cls(1));
      gui::g_state.raypath_color.push_back(make_cls(2));

      // Simulate delete of physical index 1 (per RenderColorWindow pending_delete).
      gui::g_state.raypath_color.erase(gui::g_state.raypath_color.begin() + 1);
      gui::CompactZOrder(gui::g_state);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 2);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, 0);
      IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, 1);
    };
  }

  // ValidateSingleAtomText — plan §3 decision 3 gate. Empty is treated as
  // whole-crystal (valid); a single Factor with a single alternative is valid;
  // multi-factor "A & B" AND-inside-atom is rejected; multi-alternative "A;B"
  // is rejected (a LUMICE_ColorPredicate is a single atom, cross-ref
  // combine:all is the only AND path).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "validate_single_atom_accepts_empty");
    t->TestFunc = [](ImGuiTestContext*) {
      auto v = gui::ValidateSingleAtomText("");
      IM_CHECK_EQ(v.state, LUMICE_RAYPATH_VALID);
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "validate_single_atom_accepts_single_raypath");
    t->TestFunc = [](ImGuiTestContext*) {
      auto v = gui::ValidateSingleAtomText("3-5-1");
      IM_CHECK_EQ(v.state, LUMICE_RAYPATH_VALID);
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "validate_single_atom_rejects_multi_factor");
    t->TestFunc = [](ImGuiTestContext*) {
      auto v = gui::ValidateSingleAtomText("3-5 & entry:2");
      IM_CHECK_EQ(v.state, LUMICE_RAYPATH_INVALID);
      IM_CHECK(!v.message.empty());
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "validate_single_atom_rejects_semicolon_or");
    t->TestFunc = [](ImGuiTestContext*) {
      // A ';' inside a single ref would be a summand OR-separator (SoP wire
      // notation) — the base ValidateSummandText already rejects it before
      // the single-atom gate has a chance to look at it, which is exactly
      // right: the color window's per-ref textbox represents ONE atom, and
      // cross-ref OR is expressed with combine:any across refs.
      auto v = gui::ValidateSingleAtomText("3-5;7-9");
      IM_CHECK_EQ(v.state, LUMICE_RAYPATH_INVALID);
    };
  }

  // BuildClassFromFilter — AC5 "Import from filter" backbone. Verifies:
  //   (a) each single-factor SoP row lands as one ref with the same text
  //       (empty → match_all = true; non-empty → predicate_text mirrors);
  //   (b) multi-factor AND rows are skipped and counted in skipped_rows
  //       (LUMICE_ColorPredicate is a single atom, no intra-atom AND);
  //   (c) combine defaults to LUMICE_COLOR_COMBINE_ANY (rows OR'd together —
  //       blueprint case B).
  //   (d) the produced ColorClassConfig owns its match[] by value: mutating
  //       the source filter's SoP after import does not touch the class.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "build_class_from_filter_single_atoms");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::FilterConfig f;
      f.name = "src_filter";
      // Fill f.param with three rows: two single-atom, one AND row.
      f.param.clear();
      f.param.push_back(
          gui::SummandText{ std::string{ "3-5" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });
      f.param.push_back(
          gui::SummandText{ std::string{ "1-2-3" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });
      // AND row: 2 factors — Build must skip it.
      f.param.push_back(gui::SummandText{
          std::string{ "5-7 & entry:2" },
          std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} }, gui::Factor{ gui::EntryExitParams{} } } });

      int skipped = -1;
      gui::ColorClassConfig cls = gui::BuildClassFromFilter(0, 7, f, skipped);

      IM_CHECK_EQ(skipped, 1);
      IM_CHECK_EQ(cls.combine, LUMICE_COLOR_COMBINE_ANY);
      IM_CHECK_EQ(static_cast<int>(cls.match.size()), 2);
      IM_CHECK_EQ(cls.match[0].layer_idx, 0);
      IM_CHECK_EQ(cls.match[0].crystal_pool_id, 7);
      IM_CHECK(!cls.match[0].match_all);
      IM_CHECK_EQ(cls.match[0].predicate_text, "3-5");
      IM_CHECK_EQ(cls.match[1].predicate_text, "1-2-3");

      // (d) AC5 decoupling: mutating the filter after import must not touch
      // the class. Change the source filter row text and verify the class's
      // ref text stays as it was at import time.
      f.param[0].text = "MUTATED";
      IM_CHECK_EQ(cls.match[0].predicate_text, "3-5");
    };
  }

  // BuildClassFromFilter — empty text on a single-factor row means "the whole
  // crystal" (mirrors the ref's match_all semantics), used when a filter row
  // has no meaningful raypath text but was inserted as a placeholder.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "build_class_from_filter_empty_text_is_match_all");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::FilterConfig f;
      f.param.clear();
      f.param.push_back(
          gui::SummandText{ std::string{}, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });

      int skipped = -1;
      gui::ColorClassConfig cls = gui::BuildClassFromFilter(1, 3, f, skipped);
      IM_CHECK_EQ(skipped, 0);
      IM_CHECK_EQ(static_cast<int>(cls.match.size()), 1);
      IM_CHECK(cls.match[0].match_all);
      IM_CHECK_EQ(cls.match[0].layer_idx, 1);
      IM_CHECK_EQ(cls.match[0].crystal_pool_id, 3);
    };
  }
}
