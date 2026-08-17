// The top bar's execution cluster — what THIS RUN is asked to do, and whether the picture on
// screen still answers to the document.
//
// What this suite is for. The cluster (RenderExecutionCluster, src/gui/app_panels.cpp) holds the
// ray budget, max hits, the backend toggle, the Run/Stop slot and the "Changed - re-run" chip.
// Three of its propositions can only be settled by a real frame:
//   - the ray budget's ∞ detent is a POSITION on a slider track, so what a drag can and cannot
//     reach is a property of the rendered widget and of nothing else;
//   - the chip is submitted on every frame and hidden by alpha + BeginDisabled rather than by
//     omission, so "is it lit" is a question about an item's flags in a frame; and
//   - the backend toggle is absent, not greyed, on a machine with no GPU — which only a submitted
//     (or unsubmitted) item can say.
//
// Deliberately NOT here, with where each lives instead. Whether the field-tier table itself is
// right is unit-correctness/gui/test_state_reconcile.cpp; what the reconciler does with a dirty
// document is composition-correctness/gui/test_run_lifecycle_chain.cpp; the Run slot's fixed width
// across its three labels is functional/test_shell_chrome.cpp; what a completed run puts on screen
// is functional/test_run_lifecycle.cpp. The sun and spectrum controls that stayed in the right
// panel are functional/test_scene_controls.cpp.
//
// What a user sees when these break: a ray budget whose largest value they cannot enter at all, an
// "until stopped" run they asked for as a number, or a stale picture with nothing on screen saying
// so.

#include <chrono>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/server_poller.hpp"  // LUMICE_CreateServer / StopServer / DestroyServer
#include "test_gui_shared.hpp"

namespace {

// The ray budget's two halves are separate items: the detent lives on the slider, the exact number
// on the input box. Every case below that says something about one of them addresses that one.
const char* const kRaysSlider = "**/##Rays(M)_slider";
const char* const kRaysInput = "**/##Rays(M)_input";
const char* const kMaxHits = "**/##Max hits_input";
const char* const kUseGpu = "**/Use GPU";
const char* const kChip = "##TopBar/" ICON_FA_CIRCLE_EXCLAMATION " Changed - re-run";

// The declared domain of sim.ray_num_millions, as literals. Asking the registry what to expect
// would compare one line of code against itself; what a user can actually land on is a property of
// the control, which is what this suite is here to measure.
constexpr float kRaysMin = 0.1f;
constexpr float kRaysMax = 100.0f;

// The state a case borrows to fake "a run happened" without paying for a server, handed back on
// every exit path. A fatal IM_CHECK expands to a `return`, so teardown written at the end of a case
// body runs only when the case PASSES — and in a case built around run_intent → sim_state, the
// checks that would skip it are precisely the ones a real regression trips. ResetTestState() in the
// next case rebuilds these anyway; what the guard buys is that the case gives back what it borrowed
// at the place it took it.
struct ScopedFakedRun {
  ~ScopedFakedRun() {
    gui::g_state.run_intent = gui::RunIntent::kNone;
    gui::g_state.last_committed_state.reset();
    gui::g_state.dirty = false;
  }
};

// Owns a real server for the length of one case, for the two cases that need DoRun to do anything
// (it returns immediately when g_server is null). Same shape and same reason as ScopedFakedRun.
struct ScopedServer {
  ScopedServer() { gui::g_server = LUMICE_CreateServer(); }
  ~ScopedServer() {
    gui::g_server_poller.Stop();
    gui::JoinPendingStop();
    if (gui::g_server != nullptr) {
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_stop_inflight.store(false);
    gui::g_state.run_intent = gui::RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
    gui::g_state.dirty = false;
  }

  ScopedServer(const ScopedServer&) = delete;
  ScopedServer& operator=(const ScopedServer&) = delete;

  bool ok() const { return gui::g_server != nullptr; }
};

// A held mouse button that is released even when a check in the middle of a drag returns early.
// The sweep case below asserts WHILE the button is down — that is the whole point of it — and a
// plain MouseDown/.../MouseUp pair would skip its own release on exactly the runs that fail.
class ScopedMouseDown {
 public:
  ScopedMouseDown(ImGuiTestContext* ctx, const ImVec2& pos) : ctx_(ctx) {
    ctx_->MouseMoveToPos(pos);
    ctx_->MouseDown(0);
    ctx_->Yield(2);
  }

  ScopedMouseDown(const ScopedMouseDown&) = delete;
  ScopedMouseDown& operator=(const ScopedMouseDown&) = delete;

  ~ScopedMouseDown() { Release(); }

  void MoveTo(const ImVec2& pos) const {
    ctx_->MouseMoveToPos(pos);
    ctx_->Yield(2);
  }

  void Release() {
    if (released_) {
      return;
    }
    released_ = true;
    ctx_->MouseUp(0);
    ctx_->Yield(2);
  }

 private:
  ImGuiTestContext* ctx_;
  bool released_ = false;
};

// Put the app into "there is a completed result, and the document matches it". kLoaded is the
// intent a .lmc with a baked texture arrives in — a terminal kDone with no server behind it — which
// is exactly the precondition the chip is defined against, and the cheapest way to reach it.
void SeedCompletedResult(ImGuiTestContext* ctx) {
  gui::g_state.run_intent = gui::RunIntent::kLoaded;
  gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
  gui::g_state.dirty = false;
  ctx->Yield(3);
}

// Whether the chip is currently offering itself to be clicked. The chip is submitted every frame
// regardless (alpha 0 + BeginDisabled when there is nothing to re-run), so its item flags are what
// carries the signal — and both the alpha and the disabled state are written from the one
// `modified` boolean in the same block, so reading either reads the whole statement.
bool ChipIsLit(ImGuiTestContext* ctx) {
  const ImGuiTestItemInfo info = ctx->ItemInfo(kChip, ImGuiTestOpFlags_NoError);
  return info.ID != 0 && !IsDisabled(info);
}

}  // namespace

void RegisterExecutionClusterTests(ImGuiTestEngine* engine) {
  // AC1. The checkbox is gone, and the mode it used to carry is a position on the ray-budget
  // slider. Both halves are asserted: an absent checkbox with no detent behind it would be a
  // removed feature rather than a moved one.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "execution_cluster", "the_infinite_checkbox_is_gone_and_the_detent_carries_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK(!ctx->ItemExists("**/Infinite rays"));
      IM_CHECK(!gui::g_state.sim.infinite);

      // Drag the handle past the right end of the track. The detent is the last band of it, so
      // "further right than the track goes" lands in the detent whatever its width is.
      ctx->ItemDragWithDelta(kRaysSlider, ImVec2(2000.0f, 0.0f));
      ctx->Yield(2);
      IM_CHECK(gui::g_state.sim.infinite);

      // And it says so in words. The slider's display string is not addressable through the item
      // API, so what is checked here is the state it stands for; the words themselves are pinned by
      // the top bar's own pixel reference.
      ctx->ItemDragWithDelta(kRaysSlider, ImVec2(-2000.0f, 0.0f));
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.sim.infinite);
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, kRaysMin);
    };
  }

  // AC2, first half — the red state, stated as strongly as it can be: sweeping the pointer across
  // EVERY pixel of the track, with the button held, never leaves the budget at its largest finite
  // value. This is not "the maximum is hard to hit"; the detent's boundary is closed on the
  // infinite side, so the finite band maps onto [min, max) and max is unreachable by dragging at
  // all. A change that made the top of the track snap to max would turn this red — which is the
  // point, because that change would also make ∞ mean "a very large number".
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "execution_cluster", "dragging_alone_cannot_reach_the_largest_finite_total");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const ImGuiTestItemInfo slider = ctx->ItemInfo(kRaysSlider);
      IM_CHECK_NE(slider.ID, 0u);
      const float x0 = slider.RectFull.Min.x;
      const float x1 = slider.RectFull.Max.x;
      const float y = (slider.RectFull.Min.y + slider.RectFull.Max.y) * 0.5f;
      IM_CHECK_GT(x1 - x0, 20.0f);  // a degenerate track would make the sweep vacuous

      bool saw_infinite = false;
      bool saw_finite = false;
      ScopedMouseDown drag(ctx, ImVec2(x0 + 1.0f, y));
      for (float x = x0; x <= x1; x += 1.0f) {
        drag.MoveTo(ImVec2(x, y));
        if (gui::g_state.sim.infinite) {
          saw_infinite = true;
        } else {
          saw_finite = true;
          if (gui::g_state.sim.ray_num_millions >= kRaysMax) {
            IM_ERRORF("dragging to x=%.1f left a finite budget of %f, at or above the maximum %f",
                      static_cast<double>(x), static_cast<double>(gui::g_state.sim.ray_num_millions),
                      static_cast<double>(kRaysMax));
          }
        }
        if (ctx->IsError()) {
          break;
        }
      }
      drag.Release();

      // Non-vacuity: the sweep has to have visited both bands, or "never reached the maximum" would
      // be satisfied by a slider that never moved.
      IM_CHECK(saw_finite);
      IM_CHECK(saw_infinite);
    };
  }

  // AC2, second half — and the input box reaches it, which is the whole reason the detent is
  // allowed to swallow the top of the domain. Typing the number is also what leaves the detent:
  // naming a ray total IS the statement that the run has one.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "execution_cluster", "typing_reaches_the_largest_finite_total_and_leaves_the_detent");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // From inside the detent, so the exit is part of what is measured.
      ctx->ItemDragWithDelta(kRaysSlider, ImVec2(2000.0f, 0.0f));
      ctx->Yield(2);
      IM_CHECK(gui::g_state.sim.infinite);
      // The box is live while the detent is engaged — the property AC2 turns on. A BeginDisabled
      // wrapper here (which is how this control used to work) makes the maximum unreachable by any
      // route at all.
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kRaysInput)));

      ctx->ItemInputValue(kRaysInput, kRaysMax);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, kRaysMax);
      IM_CHECK(!gui::g_state.sim.infinite);
    };
  }

  // The budget survives a detour through the detent and back. A control that zeroed the total on
  // the way past would be indistinguishable from this one until the user came back.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "execution_cluster", "a_detour_through_the_detent_gives_the_ray_total_back");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemInputValue(kRaysInput, 5.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 5.0f);

      ctx->ItemDragWithDelta(kRaysSlider, ImVec2(2000.0f, 0.0f));
      ctx->Yield(2);
      IM_CHECK(gui::g_state.sim.infinite);
      // Still 5.0, and this is the assertion the case exists for. A slider writes a value from the
      // pointer's absolute position on every frame of a drag, so the naive merge of the checkbox
      // into the track destroys the user's number on the way to the detent — they would arrive at
      // "until stopped" with ~99 M behind it. The budget is a mode change away, not an edit.
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 5.0f);

      // And leaving by typing gives the typed number, not the retained one: naming a total is an
      // instruction, not a request to be reminded what the old total was.
      ctx->ItemInputValue(kRaysInput, 12.0f);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.sim.infinite);
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, 12.0f);
    };
  }

  // The declared domain, at both ends, through the input box. Same treatment the sun sliders get in
  // functional/test_scene_controls.cpp, and for the same reason: what a typed number lands on is a
  // property of the call site.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "execution_cluster", "the_ray_total_clamps_typed_values_to_its_domain");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemInputValue(kRaysInput, kRaysMin);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, kRaysMin);
      ctx->ItemInputValue(kRaysInput, kRaysMax);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, kRaysMax);
      ctx->ItemInputValue(kRaysInput, 1000.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, kRaysMax);
      ctx->ItemInputValue(kRaysInput, -1.0f);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.ray_num_millions, kRaysMin);
    };
  }

  // P83, moved with the control. The int slider is a different widget family from the float ones
  // and reads its bounds from the same registry, so it gets the same treatment: literals, both ends.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "execution_cluster", "max_hits_clamps_typed_values_to_its_domain");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemInputValue(kMaxHits, 12);
      ctx->Yield();
      IM_CHECK_EQ(gui::g_state.sim.max_hits, 12);
      ctx->ItemInputValue(kMaxHits, 100000);
      ctx->Yield();
      const int at_max = gui::g_state.sim.max_hits;
      IM_CHECK_LT(at_max, 100000);  // it was clamped by something
      ctx->ItemInputValue(kMaxHits, -5);
      ctx->Yield();
      IM_CHECK_GT(gui::g_state.sim.max_hits, 0);
      IM_CHECK_LT(gui::g_state.sim.max_hits, at_max);
    };
  }

  // P84, moved with the control. The GPU toggle is not a control that greys out on a machine
  // without a GPU — it is not drawn at all, because a checkbox whose only outcome is a silent
  // fallback to the CPU is worse than no checkbox. Which branch is under test is decided by the
  // same probe the cluster uses, so this case says something true on a machine with a backend and
  // on one without; asserting only the branch this developer's machine happens to take would make
  // it a no-op elsewhere.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "execution_cluster", "the_gpu_toggle_is_absent_rather_than_greyed_without_a_backend");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const bool have_backend =
          LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL) || LUMICE_IsBackendAvailable(LUMICE_BACKEND_CUDA);
      if (!have_backend) {
        IM_CHECK(!ctx->ItemExists(kUseGpu));
        return;
      }

      IM_CHECK(ctx->ItemExists(kUseGpu));
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kUseGpu)));
      const bool before = gui::g_state.use_gpu_backend;
      gui::g_state.dirty = false;
      ctx->ItemClick(kUseGpu);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.use_gpu_backend, !before);
      // The backend choice rebuilds the server on the next Run, so it has to read as a change.
      IM_CHECK(gui::g_state.dirty);

      // ...and it is unreachable while a run is in flight, since an in-flight stop still holds the
      // server the switch would rebuild. run_intent is what makes the state stick: the harness main
      // loop re-derives sim_state every frame, so a bare sim_state write is gone by the next one.
      const ScopedFakedRun intent_guard;
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield(3);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state),
                  static_cast<int>(gui::GuiState::SimState::kSimulating));  // the premise held
      IM_CHECK(IsDisabled(ctx->ItemInfo(kUseGpu)));
    };
  }

  // AC3. The chip's predicate is the field→tier classifier and nothing else. Six fields, three from
  // each side of the line: editing a re-sim tier field after a result exists lights the chip;
  // editing a view/display tier field never does, however many times it is edited.
  //
  // The fields are named individually rather than swept from the tier table, deliberately. Reading
  // the table to decide what to expect would compare the classifier against itself and pass for any
  // chip wired to any predicate; these six are literals, so a chip that quietly grew a list of its
  // own has to disagree with one of them.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "execution_cluster", "the_dirty_chip_follows_the_field_tier_classifier");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Field {
        const char* name;
        void (*edit)();
        bool expect_lit;
      };
      const Field kFields[] = {
        // Re-sim tiers (kStructSoft / kStructHard in gui_state_tiers.hpp), one per top-level entry
        // so three different rows of the table are exercised rather than three leaves of one.
        { "sun.altitude", [] { gui::g_state.sun.altitude += 5.0f; }, true },
        { "sim.max_hits", [] { gui::g_state.sim.max_hits += 1; }, true },
        // Resolution, not fov: the tier table's "renderer" row is one entry for a struct whose
        // fields do not all invalidate a result, and the commit baseline captures only the
        // re-sim-eligible projection of it (RenderConfigResimFields, gui_state.hpp) — fov, lens and
        // the camera angles are view state and correctly leave a finished run alone. The chip
        // follows the projection, which is the finer and the operative statement of the two.
        { "renderer.sim_resolution_index",
          [] { gui::g_state.renderer.sim_resolution_index = gui::g_state.renderer.sim_resolution_index + 1; }, true },
        // View tier — display-time preferences, which by construction cannot invalidate a result.
        { "bg_alpha", [] { gui::g_state.bg_alpha = 0.25f; }, false },
        { "show_horizon_line", [] { gui::g_state.show_horizon_line = !gui::g_state.show_horizon_line; }, false },
        { "grid_alpha", [] { gui::g_state.grid_alpha = 0.75f; }, false },
      };

      for (const Field& f : kFields) {
        ResetTestState();
        const ScopedFakedRun guard;
        ctx->Yield(2);
        SeedCompletedResult(ctx);
        if (ChipIsLit(ctx)) {
          IM_ERRORF("%s: the chip was already lit before the edit", f.name);
          break;  // the case's premise is gone; continuing would report noise
        }

        f.edit();
        ctx->Yield(3);
        // Read once, before the report: a second ChipIsLit(ctx) inside the message would run after
        // IM_ERRORF has already put the context into its error state, where every ImGuiTestContext
        // action no-ops — so the message would print the value of a query that never ran.
        const bool lit = ChipIsLit(ctx);
        if (lit != f.expect_lit) {
          IM_ERRORF("%s: editing it left the chip %s, expected %s", f.name, lit ? "lit" : "dark",
                    f.expect_lit ? "lit" : "dark");
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // AC4. The chip cannot appear during a run, and that is a definition rather than an omission:
  // ReconcileSimState only produces kModified from kDone, so an edit made while a run is in flight
  // is auto-committed to the running server instead of being reported as a difference from a
  // finished result. The chip answers "is the result on screen stale", and during a run there is no
  // finished result for it to be stale against.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "execution_cluster", "the_dirty_chip_cannot_appear_during_a_run");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedFakedRun guard;
      ctx->Yield(2);

      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield(3);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state),
                  static_cast<int>(gui::GuiState::SimState::kSimulating));  // the premise held
      IM_CHECK(!ChipIsLit(ctx));

      // A sim-tier edit — the one that lights the chip on a finished result — while the run is in
      // flight. dirty may well go true; the chip must not.
      gui::g_state.sun.altitude += 5.0f;
      ctx->Yield(3);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kSimulating));
      IM_CHECK(!ChipIsLit(ctx));
    };
  }

  // Clicking the chip runs — the same call the Run button makes, not a variant of it. Needs a real
  // server, because DoRun returns immediately without one and the case would assert nothing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "execution_cluster", "clicking_the_dirty_chip_starts_a_run");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedServer server;
      IM_CHECK(server.ok());
      ctx->Yield(2);

      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = 0.5f;  // small enough to finish quickly
      gui::g_state.renderer.sim_resolution_index = 0;
      gui::DoRun(/*user_initiated=*/true);  // the synchronous commit is what mints the baseline
      IM_CHECK(gui::g_state.last_committed_state.has_value());
      gui::g_state.run_intent = gui::RunIntent::kLoaded;  // stand in for "the run finished"
      gui::g_state.dirty = false;
      ctx->Yield(3);
      IM_CHECK(!ChipIsLit(ctx));

      gui::g_state.sun.altitude += 5.0f;
      ctx->Yield(3);
      IM_CHECK(ChipIsLit(ctx));

      ctx->ItemClick(kChip);
      ctx->Yield(3);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(gui::RunIntent::kRunning));
      // Teardown is the guard's; see ScopedServer for why it cannot be written here.
    };
  }

  // Revert is a second action beside the chip, not folded into it. The chip re-runs with the new
  // configuration; Revert throws the new configuration away. A merge of the two would leave no way
  // to do the second, so both must be present and both must be reachable in the same state.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "execution_cluster", "the_chip_and_revert_are_two_separate_actions");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedFakedRun guard;
      ctx->Yield(2);
      SeedCompletedResult(ctx);

      const float before = gui::g_state.sun.altitude;
      gui::g_state.sun.altitude = before + 5.0f;
      ctx->Yield(3);
      IM_CHECK(ChipIsLit(ctx));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("##TopBar/Revert")));

      ctx->ItemClick("##TopBar/Revert");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.sun.altitude, before);
      IM_CHECK(!ChipIsLit(ctx));
    };
  }
}
