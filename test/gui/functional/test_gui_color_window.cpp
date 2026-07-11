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

#include <chrono>
#include <thread>

#include "IconsFontAwesome6.h"  // ICON_FA_* selectors for locating icon-prefixed buttons.
#include "gui/color_window.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "test_gui_shared.hpp"

namespace {

// Minimal single-class raypath_color config so a real server has exactly one
// active color class to commit against. Mirrors kColorConfig in
// test_gui_composite_preview.cpp; recreated here (not shared) because that
// fixture lives in a TU-private anon namespace and is not linkable across the
// gui_test target (same rationale as that file's own header comment).
const char* kSingleClassConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [128, 64],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "opacity": 1.0, "intensity_factor": 1.0
  }],
  "raypath_color": {
    "mode": "dominant",
    "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
    ]
  }
})";

bool RunToIdleWithData(LUMICE_Server* server, const char* json) {
  if (LUMICE_CommitConfig(server, json) != LUMICE_OK) {
    return false;
  }
  for (int waited = 0; waited < 5000; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[1]{};
      LUMICE_GetRawXyzResults(server, xyz, 1);
      if (xyz[0].has_valid_data) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

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
      // "1-3;5-7" uses only legal PRISM face numbers (1-8), so the base
      // ValidateSummandText / face-range checks pass — the rejection here
      // must come from the alternative-count check (CountFactorAlternatives
      // == 2), NOT a coincidental face-number-out-of-range failure. This
      // exercises the same gate FillColorPredicate enforces at commit time
      // (code-review-01 Major 2): a ';' inside a single ref is a
      // summand-level OR-separator that expands to >1 alternative, which a
      // single-atom LUMICE_ColorPredicate cannot carry — cross-ref OR is
      // expressed with combine:any across refs instead.
      auto v = gui::ValidateSingleAtomText("1-3;5-7");
      IM_CHECK_EQ(v.state, LUMICE_RAYPATH_INVALID);
      IM_CHECK(!v.message.empty());
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

  // task-348.1 Step 1 fix — PollColorClassSignal resize semantics.
  // Pre-fix: `assign(n, 0)` unconditionally zeroed every class on every call, so
  // `state.raypath_color.push_back()` immediately made every pre-existing class
  // read signal==0 → "no rays matched" warning on ALL classes for one frame.
  // Post-fix: `resize(n, 1)` preserves existing signals and defaults new entries
  // to 1 (settling / no warning). This test pins the resize invariant directly
  // (no need for a real server, no C-API roundtrip, no throttle timing).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "poll_signal_resize_preserves_existing_and_defaults_new");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      // Seed g_state with 2 classes and a caller-owned buffer that already carries
      // "known matched" for both. The buffer stands in for the shared cache the
      // orchestration wrapper would hold across frames.
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.raypath_color.push_back(c);
      std::vector<int> flags = { 1, 1 };

      // Simulate "user added a third class in this frame": grow raypath_color
      // to 3 WITHOUT a server commit. server=nullptr forces the early return
      // AFTER resize, so we exercise the resize path in isolation.
      gui::g_state.raypath_color.push_back(c);
      gui::PollColorClassSignal(gui::g_state, nullptr, flags);

      // Pre-fix: [0,0,0]. Post-fix: existing [0]/[1] preserved as 1; new [2] defaults to 1.
      IM_CHECK_EQ(static_cast<int>(flags.size()), 3);
      IM_CHECK_EQ(flags[0], 1);
      IM_CHECK_EQ(flags[1], 1);
      IM_CHECK_EQ(flags[2], 1);
    };
  }

  // Shrink path: user deletes a class. resize(n, 1) still applies — extra entries
  // beyond the new size are dropped; kept prefix retains its values.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "poll_signal_resize_shrinks_and_keeps_prefix");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      std::vector<int> flags = { 1, 1, 0 };  // caller cache larger than state

      gui::PollColorClassSignal(gui::g_state, nullptr, flags);

      IM_CHECK_EQ(static_cast<int>(flags.size()), 1);
      IM_CHECK_EQ(flags[0], 1);
    };
  }

  // The two tests above drive PollColorClassSignal with server=nullptr, which
  // returns before ever calling LUMICE_GetColorClassSignal —
  // they exercise the resize path but never the "server rejects class_count"
  // branch (issue.md ② root cause 2: the 500 ms throttled poll hits the server's
  // strict class_count check during the settling window between a GUI-side
  // push_back and the server picking up the committed class table). This test
  // drives that branch for real: commit a 1-class config to a real server, then
  // grow g_state.raypath_color to 2 WITHOUT committing, so PollColorClassSignal's
  // `n` (2) mismatches the server's active class count (1) and
  // LUMICE_GetColorClassSignal returns LUMICE_ERR_INVALID_CONFIG.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "poll_signal_preserves_on_real_server_class_count_mismatch");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      LUMICE_Server* server = LUMICE_CreateServer();
      IM_CHECK(server != nullptr);
      const bool ok = RunToIdleWithData(server, kSingleClassConfig);
      IM_CHECK(ok);
      if (!ok) {
        LUMICE_DestroyServer(server);
        return;
      }

      // Local state grows to 2 classes; the server still only knows about 1
      // (no commit happened), so this mirrors the real add-class settling window.
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.raypath_color.push_back(c);

      // Sentinel values distinguishable from both a real signal (0/1) and the
      // resize-appended default (1): if the mismatch branch is skipped and the
      // real poll result gets moved into out_flags, these would be overwritten.
      std::vector<int> flags = { 7, 7 };
      gui::PollColorClassSignal(gui::g_state, server, flags);

      IM_CHECK_EQ(static_cast<int>(flags.size()), 2);
      IM_CHECK_EQ(flags[0], 7);
      IM_CHECK_EQ(flags[1], 7);

      LUMICE_DestroyServer(server);
    };
  }

  // Aggregate predicate for the top-bar warning (task-348.1 Step 3). Same
  // semantics as the per-row warning in RenderColorWindow: warn only when the
  // user has configured at least one class with non-empty match[] AND every
  // such class currently reports no signal.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "all_configured_unmatched_returns_false_when_empty_pool");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      std::vector<int> flags;
      IM_CHECK(!gui::AllConfiguredColorClassesUnmatched(gui::g_state, flags));
    };
  }
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "all_configured_unmatched_returns_false_when_no_match_configured");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      // Two classes, both with empty match[] (user added them but hasn't wired refs yet).
      gui::ColorClassConfig c;
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.raypath_color.push_back(c);
      std::vector<int> flags = { 0, 0 };
      // No configured refs anywhere ⇒ nothing to warn about, top bar stays quiet.
      IM_CHECK(!gui::AllConfiguredColorClassesUnmatched(gui::g_state, flags));
    };
  }
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "all_configured_unmatched_true_when_every_configured_class_is_zero");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      gui::ColorClassRefConfig r;
      c.match.push_back(r);
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.raypath_color.push_back(c);
      std::vector<int> flags = { 0, 0 };
      IM_CHECK(gui::AllConfiguredColorClassesUnmatched(gui::g_state, flags));
    };
  }
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "all_configured_unmatched_false_when_any_configured_class_has_signal");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      gui::ColorClassRefConfig r;
      c.match.push_back(r);
      gui::g_state.raypath_color.push_back(c);
      gui::g_state.raypath_color.push_back(c);
      std::vector<int> flags = { 0, 1 };
      IM_CHECK(!gui::AllConfiguredColorClassesUnmatched(gui::g_state, flags));
    };
  }
  {
    // Mixed pool: class[0] configured but silent, class[1] empty match. The
    // top-bar warning should fire (class[0] alone is enough — class[1] adds
    // nothing to warn about since it has no configured refs).
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "all_configured_unmatched_ignores_empty_match_classes");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig configured;
      gui::ColorClassRefConfig r;
      configured.match.push_back(r);
      gui::ColorClassConfig empty_cls;
      gui::g_state.raypath_color.push_back(configured);
      gui::g_state.raypath_color.push_back(empty_cls);
      std::vector<int> flags = { 0, 0 };
      IM_CHECK(gui::AllConfiguredColorClassesUnmatched(gui::g_state, flags));
    };
  }

  // A configured class whose index falls outside signal_flags (caller's cache
  // hasn't grown to match state.raypath_color yet) must be treated as "unknown",
  // not as a confirmed no-signal — matching
  // PollColorClassSignal's own "we don't know yet" default (resize(n, 1)). The
  // only configured class is out of range: pre-fix, `any_configured` was set
  // before the bounds check and the missing entry could never disqualify the
  // warning, so this would have returned true (false-positive warn). Post-fix
  // it must return false — there is no known data, so nothing to warn about yet.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "all_configured_unmatched_treats_out_of_range_index_as_unknown");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig configured;
      gui::ColorClassRefConfig r;
      configured.match.push_back(r);
      gui::g_state.raypath_color.push_back(configured);
      std::vector<int> flags;  // empty: index 0 is out of range
      IM_CHECK(!gui::AllConfiguredColorClassesUnmatched(gui::g_state, flags));
    };
  }
  // A genuinely known no-signal class must still trigger the warning even when a
  // second, out-of-range (unknown) class is also configured — the fix must not
  // over-correct into "any unknown entry suppresses the whole aggregate".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window",
                                    "all_configured_unmatched_true_when_known_entry_unmatched_despite_unknown_peer");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig configured;
      gui::ColorClassRefConfig r;
      configured.match.push_back(r);
      gui::g_state.raypath_color.push_back(configured);
      gui::g_state.raypath_color.push_back(configured);
      std::vector<int> flags = { 0 };  // index 0 known + unmatched; index 1 out of range (unknown)
      IM_CHECK(gui::AllConfiguredColorClassesUnmatched(gui::g_state, flags));
    };
  }

  // task-list-row-ergonomics ④: SetRefMatchAll must NOT clear predicate_text.
  // Root fix — pre-fix the whole checkbox's `.clear()` path wiped the field so
  // un-checking whole left the row blank. AC2 machine gate.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "set_ref_match_all_true_preserves_predicate_text");
    t->TestFunc = [](ImGuiTestContext*) {
      gui::ColorClassRefConfig ref;
      ref.predicate_text = "3-5";
      ref.match_all = false;
      gui::SetRefMatchAll(ref, true);
      IM_CHECK(ref.match_all);
      IM_CHECK_EQ(ref.predicate_text, std::string("3-5"));
    };
  }
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "set_ref_match_all_false_restores_editability_with_original_text");
    t->TestFunc = [](ImGuiTestContext*) {
      gui::ColorClassRefConfig ref;
      ref.predicate_text = "3-5";
      ref.match_all = true;  // as if user had checked "whole"
      gui::SetRefMatchAll(ref, false);
      IM_CHECK(!ref.match_all);
      IM_CHECK_EQ(ref.predicate_text, std::string("3-5"));
    };
  }

  // task-list-row-ergonomics ③: HandleEyeClick — plain click only touches
  // `visible`, Alt+click enforces exclusive solo. AC3 machine gate.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "handle_eye_click_plain_click_toggles_visible_only");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig a;
      a.visible = true;
      a.solo = false;
      gui::ColorClassConfig b = a;
      gui::g_state.raypath_color.push_back(a);
      gui::g_state.raypath_color.push_back(b);

      gui::HandleEyeClick(gui::g_state.raypath_color, 0, /*alt_down=*/false);

      IM_CHECK(!gui::g_state.raypath_color[0].visible);
      IM_CHECK(gui::g_state.raypath_color[1].visible);
      IM_CHECK(!gui::g_state.raypath_color[0].solo);
      IM_CHECK(!gui::g_state.raypath_color[1].solo);
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "handle_eye_click_alt_sets_exclusive_solo");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      for (int i = 0; i < 3; i++) {
        gui::ColorClassConfig c;
        c.visible = true;
        c.solo = false;
        gui::g_state.raypath_color.push_back(c);
      }
      gui::HandleEyeClick(gui::g_state.raypath_color, 1, /*alt_down=*/true);

      IM_CHECK(!gui::g_state.raypath_color[0].solo);
      IM_CHECK(gui::g_state.raypath_color[1].solo);
      IM_CHECK(!gui::g_state.raypath_color[2].solo);
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "handle_eye_click_alt_second_click_clears_solo");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      for (int i = 0; i < 3; i++) {
        gui::ColorClassConfig c;
        c.visible = true;
        c.solo = false;
        gui::g_state.raypath_color.push_back(c);
      }
      // First Alt+click: idx=1 becomes solo.
      gui::HandleEyeClick(gui::g_state.raypath_color, 1, /*alt_down=*/true);
      IM_CHECK(gui::g_state.raypath_color[1].solo);
      // Second Alt+click on the same idx clears every solo (compositor's any_solo
      // becomes false, falls back to per-visible composition).
      gui::HandleEyeClick(gui::g_state.raypath_color, 1, /*alt_down=*/true);
      IM_CHECK(!gui::g_state.raypath_color[0].solo);
      IM_CHECK(!gui::g_state.raypath_color[1].solo);
      IM_CHECK(!gui::g_state.raypath_color[2].solo);
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "handle_eye_click_alt_switching_target_moves_exclusively");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      for (int i = 0; i < 3; i++) {
        gui::ColorClassConfig c;
        c.visible = true;
        c.solo = false;
        gui::g_state.raypath_color.push_back(c);
      }
      gui::g_state.raypath_color[0].solo = true;  // seed: someone else already solo'd
      gui::HandleEyeClick(gui::g_state.raypath_color, 2, /*alt_down=*/true);
      // Even though phys=2 was not previously solo, the seeded phys=0 must be
      // cleared — the "clear all → set target" ordering (not just "toggle self")
      // enforces the at-most-one invariant across the full class list.
      IM_CHECK(!gui::g_state.raypath_color[0].solo);
      IM_CHECK(!gui::g_state.raypath_color[1].solo);
      IM_CHECK(gui::g_state.raypath_color[2].solo);
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "handle_eye_click_out_of_range_is_noop");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::ColorClassConfig c;
      c.visible = true;
      c.solo = false;
      gui::g_state.raypath_color.push_back(c);

      gui::HandleEyeClick(gui::g_state.raypath_color, 99, /*alt_down=*/false);
      gui::HandleEyeClick(gui::g_state.raypath_color, 99, /*alt_down=*/true);

      IM_CHECK(gui::g_state.raypath_color[0].visible);
      IM_CHECK(!gui::g_state.raypath_color[0].solo);
    };
  }

  // BuildClassFromFilter — a row whose single Factor still expands to more
  // than one alternative ("1-3;5-7", the same ';' OR-separator case as
  // validate_single_atom_rejects_semicolon_or) must be skipped like an
  // AND-row, not silently imported as a legal single-atom ref that
  // FillColorPredicate would then drop at the next commit (code-review-01
  // Minor 1, same root cause as Major 2). Uses gui::ParseSummandText for
  // `.factors` (rather than the dummy single-Factor placeholder the other
  // cases here use) so CountFactorAlternatives sees the real ';' content.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "build_class_from_filter_skips_multi_alt_row");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::FilterConfig f;
      f.param.clear();
      const std::string text = "1-3;5-7";
      f.param.push_back(gui::SummandText{ text, gui::ParseSummandText(text) });
      f.param.push_back(
          gui::SummandText{ std::string{ "3-5" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });

      int skipped = -1;
      gui::ColorClassConfig cls = gui::BuildClassFromFilter(0, 2, f, skipped);

      IM_CHECK_EQ(skipped, 1);
      IM_CHECK_EQ(static_cast<int>(cls.match.size()), 1);
      IM_CHECK_EQ(cls.match[0].predicate_text, "3-5");
    };
  }

  // task-349.2 Step 1 (AC1 white-box lockdown): structural color-class edits on
  // top of a completed simulation must surface as SimState::kModified through
  // the shared dirty → ReconcileSimState pipeline (the same route the main-
  // scene "changed since last run" ⚠ + Revert ride on). Driven by REAL UI
  // clicks (not a hand poke of state.dirty) so the test can catch a future
  // rewrite that quietly bypasses MarkFilterDirty on any color-window control.
  //
  // Seed pattern (kDone + committed_epoch>0 + run_intent=kLoaded + dirty=false)
  // mirrors p1_edit/ok_no_change_preserves_state so the reconcile base pins to
  // kDone and the subsequent dirty edit flips it to kModified.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_class_via_ui_marks_modified");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // T1: the reconciler diffs against last_committed_state; without a baseline it is a no-op
      // before the first commit (mirrors real Run-then-edit flow). Seed the baseline BEFORE the
      // edit so the subsequent Add Class becomes a diff — under the OLD widget-side
      // MarkFilterDirty call this seeding was unnecessary; T1 makes the baseline a first-class
      // input. (toggle_whole_via_ui_marks_modified below already followed this pattern.)
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Click the "+ Add Class" button in the Colors window. `**/` wildcard
      // walks any window path, so we don't need to hardcode the window ID.
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 1);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // The top-bar Revert affordance is the user-visible surface of kModified;
      // presence here proves the ReconcileSimState → RenderTopBar wire is live
      // for a color-driven dirty (not just a main-scene edit).
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));

      // Close the Colors window so it does not overlay the next test's clicks.
      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // Whole-crystal checkbox is the #2 case from issue.md: a display-affecting
  // structural edit whose current implementation is `SetRefMatchAll(ref, ...);
  // state.MarkFilterDirty();` — must surface as kModified for the same reason
  // as Add Class.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "toggle_whole_via_ui_marks_modified");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed one color class with one ref (match_all=false) so the "whole"
      // checkbox is unchecked and toggling it flips to true. Do this BEFORE
      // pinning kDone so the seeding does not itself dirty the reconcile input.
      gui::ColorClassConfig cls;
      cls.color[0] = 1.0f;
      cls.visible = true;
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = gui::g_state.layers[0].entries[0].crystal_id;
      ref.match_all = false;
      cls.match.push_back(ref);
      gui::g_state.raypath_color.push_back(cls);
      // Baseline snapshot for the config-vs-snapshot dirty check inside
      // ReconcileSimState (dirty is set explicitly below anyway; snapshot just
      // ensures we start from a clean reconciled kDone before the UI edit).
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Expand every tree node in the Colors window so RenderRefRow (and its
      // "whole" checkbox) becomes part of the frame's item table. ItemOpenAll
      // walks the window's item tree; wildcard-locating `##body` under the
      // class's PushID(phys=int) is fragile because PushID(int) hashes an
      // integer, not the string "0".
      ctx->ItemOpenAll("//" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      ctx->ItemClick("**/whole");
      ctx->Yield(2);
      ctx->SetRef("");

      IM_CHECK(gui::g_state.raypath_color[0].match[0].match_all);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));

      // Close the Colors window so the next test's frame does not still show
      // an overlay on top of top-bar controls it needs to click.
      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-349.2 Step 2 (§3.4 Revert-completeness fix): after +Add Class flips
  // sim_state to kModified, clicking the top-bar Revert button must restore
  // raypath_color to its pre-edit size AND settle sim_state back to a
  // non-Modified state. Both are required: pre-fix, ConfigSnapshot silently
  // omitted raypath_color, so Revert cleared `dirty` (settling sim_state) but
  // left the added class behind — a check on sim_state alone would have
  // reported "green" while the color edit stealthily remained.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_class_then_revert_restores_color_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed a single, non-default color class as the "last-committed" baseline
      // (with fingerprint fields the assertion can distinguish from a default-
      // constructed junk class if the round-trip drops content silently).
      gui::ColorClassConfig baseline;
      baseline.color[0] = 0.25f;
      baseline.color[1] = 0.5f;
      baseline.color[2] = 0.75f;
      baseline.combine = 1;
      baseline.visible = false;
      gui::g_state.raypath_color.push_back(baseline);
      // Snapshot the baseline INTO last_committed_state, mirroring what
      // FillLumiceConfig + CommitConfigStruct do after a successful commit.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      // Pin reconcile base to kDone so a subsequent dirty edit surfaces as
      // kModified. Same seed shape as the two tests above.
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // Structural edit: +Add Class → raypath_color grows to 2, sim_state → kModified.
      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      // Click Revert. Post-fix: raypath_color shrinks back to 1 (the baseline
      // is restored from last_committed_state), sim_state settles back to a
      // non-Modified state (kDone under the current reconcile). Pre-fix, only
      // the sim_state assertion would pass and the size assertion would fail —
      // that is the entire point of this regression guard.
      ctx->ItemClick("##TopBar/Revert");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 1);
      IM_CHECK_NE(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // Content-level check: the baseline's fingerprint survived — proves
      // ApplyTo copied the raypath_color vector by value rather than default-
      // initializing an empty replacement (belt + suspenders with the
      // unit-correctness test on ConfigSnapshot itself).
      IM_CHECK_EQ(gui::g_state.raypath_color[0].combine, 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].visible, false);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[1], 0.5f);

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }

  // task-349.2 Step 3 (#6, AC2 machine gate): when there are configured color
  // classes with matches, the top-bar Colored button + Colors-window "Enable
  // colors" checkbox must NOT be BeginDisabled(). This is the "wiring is
  // present" check — 7 pure unit tests above already cover the predicate
  // `AllConfiguredColorClassesUnmatched`; here we drive the real UI with a
  // configured class + non-empty signal and confirm the two controls stay
  // enabled. The complementary "predicate=true → disabled" state depends on
  // driving the internal 500 ms signal cache to zeros, which requires a real
  // 0-match server config (plan §7 risk 4 called this out as potentially
  // flaky); we cover it via the pure predicate unit tests + owner on-screen
  // AC4 pass, not a fragile end-to-end here.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "enable_controls_stay_enabled_when_predicate_reports_matches");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Seed one configured class (non-empty match) so the top-bar Colored
      // button (which is gated on !raypath_color.empty()) actually renders.
      gui::ColorClassConfig cls;
      cls.color[0] = 1.0f;
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = 0;
      ref.match_all = true;
      cls.match.push_back(ref);
      gui::g_state.raypath_color.push_back(cls);
      gui::g_state.color_window_open = true;
      ctx->Yield(4);

      // With server=nullptr, RefreshColorClassSignals resizes the cache to
      // (n, 1) — all classes report "matched / unknown", predicate returns
      // false, controls stay enabled. After 349.3 (#4) the top-bar toggle is a
      // plain-text Checkbox again, so the widget id is label-dependent
      // ("Colored" vs "Full Spectrum"). ResetTestState() / DoNew() leave
      // last_uploaded_as_composite at its default `false`, so the initial label
      // is "Full Spectrum".
      auto top_info = ctx->ItemInfo("##TopBar/Full Spectrum##CompositePreviewToggle");
      IM_CHECK((top_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);

      ctx->SetRef("//" ICON_FA_PALETTE " Colors");
      auto win_info = ctx->ItemInfo("**/Enable colors");
      IM_CHECK((win_info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      ctx->SetRef("");

      gui::g_state.color_window_open = false;
      ctx->Yield(2);
    };
  }
}
