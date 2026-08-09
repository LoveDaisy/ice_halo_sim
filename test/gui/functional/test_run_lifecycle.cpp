// The run lifecycle as the user drives it: the button that starts and stops a simulation, the one
// that throws away edits since the last run, and the modal that says a configuration did not fit.
//
// What needs a real frame here and what does not. The state machine itself — which
// (intent, committed epoch, observation, dirty) tuple maps to which displayed state — is a pure
// function, and every row of it is pinned one layer down in
// composition-correctness/gui/test_run_lifecycle_chain.cpp; the warning modal's
// once-per-episode de-duplication likewise, in test_run_warning_chain.cpp; the conditions under
// which the poller rescues a completed run's final frame past the preview quality gate, in
// unit-correctness/gui/test_server_poller.cpp. Nothing here re-asserts any of them. What is left
// is what only a live toolbar can answer: that a state reaches a control the user can press, that
// pressing it does what the label promises, and that a finished run survives what the user does
// next.
//
// Every case drives the reconcile INPUTS (run_intent, the stop latch, dirty, a real DoRun) and
// never sim_state, which has a single owner — SyncFromPoller assigns it from ReconcileSimState on
// every tick, so a direct write does not survive to the frame that would draw it, and a case
// written that way silently observes the wrong state.
//
// What a user sees when these break: Run and Stop stop agreeing with what the backend is doing, so
// the button either does nothing or restarts a run that had already finished; the Revert
// affordance shifts the toolbar under the cursor as it appears and disappears; a finite run
// finishes and the preview stays blank; a warning re-opens on every 70 ms commit while a slider is
// dragged and the window stops responding.

#include <chrono>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/gui_constants.hpp"  // kIdleHeartbeatIntervalMs — the completed run's slow poll interval
#include "gui/server_poller.hpp"
#include "test_gui_shared.hpp"

namespace {

using SimState = gui::GuiState::SimState;
using RunIntent = gui::RunIntent;

const char* const kRunBtn = "##TopBar/" ICON_FA_PLAY " Run";
const char* const kStopBtn = "##TopBar/" ICON_FA_STOP " Stop";
const char* const kStoppingBtn = "##TopBar/" ICON_FA_STOP " Stopping...";
const char* const kRevertBtn = "##TopBar/Revert";
const char* const kNewBtn = "##TopBar/New";

bool IsDisabled(const ImGuiTestItemInfo& info) {
  return (info.ItemFlags & ImGuiItemFlags_Disabled) != 0;
}

// A live server for the duration of one case, torn down on EVERY exit path. Held by an object
// because IM_CHECK* expands to `return`, so a failing assertion between creation and a hand-written
// teardown would otherwise leave a running server and a poller thread for the next registered case
// to inherit (ResetTestState() only nulls the pointer).
//
// JoinPendingStop() runs before the reset as well as after: a stop offloaded by an earlier case may
// still be holding that case's server on the background thread, and ResetTestState() would drop the
// pointer out from under it.
struct ScopedRunScene {
  ScopedRunScene() {
    gui::JoinPendingStop();
    ResetTestState();
    gui::g_server = LUMICE_CreateServer();
    gui::g_server_is_gpu = false;  // re-establish the backend-toggle detection invariant
  }

  ~ScopedRunScene() {
    gui::g_server_poller.Stop();
    gui::JoinPendingStop();
    if (gui::g_server != nullptr) {
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_server_is_gpu = false;
    // The stop latch is the one field here that has no other owner: ResetTestState() rebuilds the
    // document through DoNew(), which zeroes the four GuiState fields below, but g_stop_inflight is
    // an app.cpp global that no reset touches. A case that leaves it set makes every later case's
    // kStopping intent stick, since SyncFromPoller only promotes kStopping→kStopped while no stop
    // is in flight. The four after it are defensive overlap with that rebuild, kept so this object
    // is a complete statement of what it borrowed rather than a list of what ResetTestState happens
    // to miss today.
    gui::g_stop_inflight.store(false);
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
    gui::g_state.dirty = false;
  }

  ScopedRunScene(const ScopedRunScene&) = delete;
  ScopedRunScene& operator=(const ScopedRunScene&) = delete;

  bool ok() const { return gui::g_server != nullptr; }
};

// Smallest scene that still produces a real frame: the lowest simulation resolution and a finite
// budget. `millions` is dyadic so ExpectedSimRayNum can name the count exactly (see its note in
// test_gui_shared.hpp).
void SeedFiniteRun(float millions) {
  gui::g_state.sim.infinite = false;
  gui::g_state.sim.ray_num_millions = millions;
  gui::g_state.sim.max_hits = 8;
  gui::g_state.renderer.sim_resolution_index = 0;
}

void SeedInfiniteRun() {
  gui::g_state.sim.infinite = true;
  gui::g_state.sim.max_hits = 8;
  gui::g_state.renderer.sim_resolution_index = 0;
}

// Pump frames until `pred` holds or the wall-clock budget runs out. Wall clock rather than a frame
// count because --fixed-dt decouples the two: it injects a constant frame dt and skips the
// frame-limit sleep, so an iteration count says nothing about how many real seconds the simulation
// thread has been given. The budget bounds a hang; it is not an expectation.
template <typename Pred>
bool DriveUntil(ImGuiTestContext* ctx, Pred pred, int timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    ctx->Yield();
  }
  return true;
}

bool ReachedDoneAndLatched(ImGuiTestContext* ctx) {
  return DriveUntil(
      ctx,
      [] { return gui::g_state.sim_state == SimState::kDone && gui::g_state.run_intent == RunIntent::kRunCompleted; },
      20);
}

// One row of the toolbar-state table: a name to report and the reconcile inputs that put the top
// bar into that state.
struct SlotCase {
  const char* name;
  void (*apply)();
  const char* expected_slot;  // the one of the three labels that must occupy the Run/Stop slot
  bool revert_enabled;
};

// Restart trio body. `seed` runs before the first Run, `edit` between the first upload and the
// re-commit. Shared because the three cases differ only in those two statements, and a table would
// have folded three historically independent registrations into one reported result.
void DriveRestart(ImGuiTestContext* ctx, void (*seed)(), void (*edit)(), int restart_timeout_ms) {
  ScopedRunScene scene;
  IM_CHECK(scene.ok());
  seed();
  SeedInfiniteRun();

  gui::DoRun(/*user_initiated=*/true);
  // Wait for the FIRST upload before capturing the baseline: a restart is observable only as a new
  // upload on top of an existing one, and a baseline captured before anything was painted would be
  // satisfied by the run's own first frame.
  IM_CHECK(WaitForSimRestartAtLeast(ctx, 0, 10000));
  // Then let the commits already in flight land before the baseline is read. Without this, an
  // upload belonging to the FIRST run can arrive just after the read and satisfy the "one more
  // upload" test on its own — the edit below would then be credited with a repaint it did not
  // cause, and the case would be green whatever the restart path did. Of the three cases this
  // helper serves, only the ray-budget one carried this drain before; it guards all three equally
  // and there was never a reason the other two were exempt, so it is applied here rather than
  // per case. It is deliberately BEFORE the baseline read — the "do not yield between the baseline
  // and DoRun" rule the old cases stated is about the window after it, which is still respected.
  ctx->Yield(3);
  const unsigned long long baseline = gui::g_state.texture_upload_count;

  edit();
  gui::DoRun(/*user_initiated=*/true);
  IM_CHECK(WaitForSimRestartAtLeast(ctx, baseline, restart_timeout_ms));
}

}  // namespace

void RegisterRunLifecycleTests(ImGuiTestEngine* engine) {
  // The Run/Stop slot and the Revert affordance say what the run state is (P2, P4, P5).
  //
  // The three labels share one slot, so "Stop is shown" and "Run is not" are different claims and
  // both are made: a slot that drew both would be a layout bug, and a slot that drew neither would
  // pass a test that only looked for the one it expected.
  //
  // The kStopping row is the one state with no purely static input: SyncFromPoller promotes a
  // kStopping intent to kStopped the moment no stop is actually in flight. Holding g_stop_inflight
  // — the same latch DoStop sets and the background drain clears — is what keeps the row on screen
  // for more than the single frame the promotion allows, and it drives the same input the product
  // does rather than pinning sim_state by hand. The drained transition itself is exercised against
  // a real server in stop_paints_immediately_and_settles_once_the_backend_drains below.
  ImGuiTest* t_slot = IM_REGISTER_TEST(engine, "run_lifecycle", "the_run_slot_and_revert_follow_the_run_state");
  t_slot->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);

    static const SlotCase kCases[] = {
      { "never run",
        [] {
          gui::g_state.run_intent = RunIntent::kNone;
          gui::g_state.dirty = false;
          gui::g_stop_inflight.store(false);
        },
        kRunBtn, false },
      { "simulating",
        [] {
          gui::g_state.run_intent = RunIntent::kRunning;
          gui::g_state.dirty = false;
          gui::g_stop_inflight.store(false);
        },
        kStopBtn, false },
      { "stop draining",
        [] {
          gui::g_state.run_intent = RunIntent::kStopping;
          gui::g_state.dirty = false;
          gui::g_stop_inflight.store(true);
        },
        kStoppingBtn, false },
      { "stopped",
        [] {
          gui::g_state.run_intent = RunIntent::kStopped;
          gui::g_state.dirty = false;
          gui::g_stop_inflight.store(false);
        },
        kRunBtn, false },
      { "stopped, then edited",
        [] {
          gui::g_state.run_intent = RunIntent::kStopped;
          gui::g_state.dirty = true;
          gui::g_stop_inflight.store(false);
        },
        kRunBtn, true },
    };
    static const char* const kAllSlots[] = { kRunBtn, kStopBtn, kStoppingBtn };

    // Collected and reported after the loop, not asserted inside it: IM_CHECK* expands to `return`
    // in the enclosing function, so a fatal assert on row 1 would hide rows 2..5 — the exact defect
    // scripts/check_loop_fatal_asserts.py exists to stop.
    std::string wrong_slot;
    std::string wrong_revert;
    for (const SlotCase& c : kCases) {
      c.apply();
      ctx->Yield(3);
      for (const char* slot : kAllSlots) {
        const bool present = ctx->ItemExists(slot);
        if (present != (slot == c.expected_slot)) {
          wrong_slot += std::string(" ") + c.name + (present ? ":unexpected " : ":missing ") + slot;
        }
      }
      const ImGuiTestItemInfo revert = ctx->ItemInfo(kRevertBtn, ImGuiTestOpFlags_NoError);
      if (revert.ID == 0) {
        wrong_revert += std::string(" ") + c.name + ":absent";
      } else if (IsDisabled(revert) == c.revert_enabled) {
        wrong_revert += std::string(" ") + c.name + (c.revert_enabled ? ":disabled" : ":enabled");
      }
    }
    ctx->LogInfo("slot:[%s] revert:[%s]", wrong_slot.c_str(), wrong_revert.c_str());
    IM_CHECK_STR_EQ(wrong_slot.c_str(), "");
    IM_CHECK_STR_EQ(wrong_revert.c_str(), "");

    // P5's other half: the Revert area is always rendered, so the toolbar must not re-flow as it
    // becomes usable. The pair compared here is the clean/edited pair — both draw "Run" in the slot,
    // so the run slot's own width is constant by construction and the only variable left is the
    // Revert group. (That the slot's width is the same across its THREE labels is a separate
    // proposition and is stated where the top bar's chrome lives, not here.)
    kCases[3].apply();  // stopped, clean
    ctx->Yield(3);
    const ImRect revert_clean = ctx->ItemInfo(kRevertBtn).RectFull;
    const ImRect new_clean = ctx->ItemInfo(kNewBtn).RectFull;
    kCases[4].apply();  // stopped, edited
    ctx->Yield(3);
    IM_CHECK_EQ(ctx->ItemInfo(kRevertBtn).RectFull.Min.x, revert_clean.Min.x);
    IM_CHECK_EQ(ctx->ItemInfo(kNewBtn).RectFull.Min.x, new_clean.Min.x);

    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.dirty = false;
    gui::g_stop_inflight.store(false);
  };

  // Pressing the button is what the proposition is about (P2). The case above establishes that a
  // state reaches the right label; this one establishes that the label is wired to the action, over
  // a real server, through ImGui's own click plumbing rather than a direct DoRun/DoStop call.
  ImGuiTest* t_press = IM_REGISTER_TEST(engine, "run_lifecycle", "pressing_run_starts_a_run_and_the_slot_becomes_stop");
  t_press->TestFunc = [](ImGuiTestContext* ctx) {
    ScopedRunScene scene;
    IM_CHECK(scene.ok());
    SeedInfiniteRun();  // infinite, so the run is still going when Stop is pressed
    ctx->Yield(2);

    ctx->ItemClick(kRunBtn);
    ctx->Yield(2);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunning));
    IM_CHECK(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kSimulating; }, 10));
    IM_CHECK(ctx->ItemExists(kStopBtn));
    IM_CHECK(!ctx->ItemExists(kRunBtn));

    // Stop is optimistic: the intent flips synchronously on the click and the drain is offloaded,
    // so this assertion does not depend on how fast the background thread runs.
    ctx->ItemClick(kStopBtn);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kStopping));

    gui::JoinPendingStop();
    IM_CHECK(DriveUntil(ctx, [] { return gui::g_state.run_intent == RunIntent::kStopped; }, 10));
    ctx->Yield(2);
    IM_CHECK(ctx->ItemExists(kRunBtn));  // the slot is offering to run again
  };

  // Revert throws away the edits made since the last run and puts the result back on screen (P5).
  ImGuiTest* t_revert = IM_REGISTER_TEST(engine, "run_lifecycle", "pressing_revert_restores_the_committed_config");
  t_revert->TestFunc = [](ImGuiTestContext* ctx) {
    ScopedRunScene scene;
    IM_CHECK(scene.ok());
    SeedFiniteRun(0.125f);

    gui::DoRun(/*user_initiated=*/true);  // the commit is what mints the Revert baseline
    IM_CHECK(ReachedDoneAndLatched(ctx));
    IM_CHECK(gui::g_state.last_committed_state.has_value());

    const float committed_altitude = gui::g_state.sun.altitude;
    gui::g_state.sun.altitude = committed_altitude + 7.0f;
    ctx->Yield(3);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kModified));
    IM_CHECK(!IsDisabled(ctx->ItemInfo(kRevertBtn)));

    ctx->ItemClick(kRevertBtn);
    ctx->Yield(3);
    IM_CHECK_EQ(gui::g_state.sun.altitude, committed_altitude);
    IM_CHECK(!gui::g_state.dirty);
    // And the toolbar settles back onto the result it already had, rather than onto "never run".
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
  };

  // The warning modal fires once per episode and re-arms only when the condition is cleared (P48).
  //
  // The arithmetic behind that — which triggers re-fire on a deliberate Run and which do not — is a
  // table in composition-correctness/gui/test_run_warning_chain.cpp, which deliberately consumes
  // the pending flag by hand rather than rendering. This case is the other half of that split: that
  // a rendered frame turns the flag into a modal at all, that OK closes it, and that OK does NOT
  // re-arm it — the last being the whole anti-spam mechanism, since a dismissed message stays in
  // flight precisely so the next debounced commit finds it already reported.
  ImGuiTest* t_warn = IM_REGISTER_TEST(engine, "run_lifecycle", "a_warning_opens_once_and_reopens_only_when_cleared");
  t_warn->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();  // clears any in-flight warning
    ctx->Yield(2);
    const std::string msg = "this configuration does not fit";

    gui::SetGuiWarning(msg);
    ctx->Yield(2);
    IM_CHECK(ImGui::IsPopupOpen("Warning"));

    ctx->ItemClick("Warning/OK");
    ctx->Yield(2);
    IM_CHECK(!ImGui::IsPopupOpen("Warning"));
    IM_CHECK_STR_EQ(gui::PeekGuiWarning().c_str(), msg.c_str());  // dismissed, still in flight

    gui::SetGuiWarning(msg);  // the same condition, re-detected on the next commit
    ctx->Yield(2);
    IM_CHECK(!ImGui::IsPopupOpen("Warning"));

    gui::ClearGuiWarning();  // a commit succeeded — the gate re-arms
    gui::SetGuiWarning(msg);
    ctx->Yield(2);
    IM_CHECK(ImGui::IsPopupOpen("Warning"));

    ctx->ItemClick("Warning/OK");
    ctx->Yield(2);
    gui::ClearGuiWarning();
  };

  // A finite run reaches its terminal display state with its result on screen, and the idle
  // heartbeat does not pull either back off (the "backend finished but the GUI still says
  // Simulating" dead state, and the blank-preview-after-completion one beside it).
  //
  // The preview assertions are the frame-needing half of that second defect: a completed run whose
  // final frame never reaches the texture leaves the user looking at nothing, whatever the status
  // bar says. Only the half, though, and deliberately so. The gate this once tripped over rejects
  // snapshots below a ray threshold, and a budget under that threshold is not a state this GUI will
  // hold: the field domain for sim.ray_num_millions starts at 0.1 (100,000 rays, against an
  // uncalibrated floor of 5,000), and setting a smaller value has the frame-tail reconciler clamp
  // it back up — measured, not assumed: a run seeded at 0.002 traces its 2,000 rays and completes,
  // but the clamped field then differs from the committed snapshot forever, so the state settles on
  // Modified and never on Done. So the rescue branch cannot be reached from a configuration the
  // editor can produce, and its conditions are pinned where they can be: directly, without a
  // window, in unit-correctness/gui/test_server_poller.cpp.
  //
  // Two clocks meet at the end of a completed run and the question is whether they interfere:
  // --fixed-dt pins the frame pump's dt to 1/60s while the poller's self-paused heartbeat is a
  // steady_clock wait on its own thread. They are structurally unrelated, but "structurally
  // unrelated" is an argument, and this suite exists to check arguments against frames.
  //
  // Asserted in the direction that can actually fail. "No tick fired" would be a statement about
  // how fast this machine got from completion to here — true on a fast run, a flake on a slow one.
  // What matters is that the injected dt does not starve the heartbeat, so ticks do appear in real
  // time, and that those ticks do not disturb the state the reconcile already settled on.
  ImGuiTest* t_done = IM_REGISTER_TEST(engine, "run_lifecycle", "a_finite_run_reaches_done_and_the_heartbeat_holds");
  t_done->TestFunc = [](ImGuiTestContext* ctx) {
    ScopedRunScene scene;
    IM_CHECK(scene.ok());
    SeedFiniteRun(0.5f);
#if defined(__APPLE__)
    // The one case that runs the real GPU route (MaybeReconstructServerForBackend → Metal single
    // engine): the completion edge is backend plumbing, and this is where a backend that never
    // reports COMPLETED would show up.
    gui::g_state.use_gpu_backend = true;
#endif

    gui::DoRun(/*user_initiated=*/true);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunning));
    IM_CHECK(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kDone; }, 20));
    IM_CHECK_EQ(static_cast<unsigned long long>(gui::g_state.stats_sim_ray_num), ExpectedSimRayNum(0.5f));
    IM_CHECK_GT(gui::g_state.texture_upload_count, 0ull);
    IM_CHECK(gui::g_preview.HasTexture());

    const uint64_t ticks_at_done = gui::g_server_poller.HeartbeatTickCountForTest();
    const auto hb_start = std::chrono::steady_clock::now();
    const auto hb_budget = std::chrono::milliseconds(4 * gui::kIdleHeartbeatIntervalMs);
    IM_CHECK(DriveUntil(
        ctx,
        [&] {
          return gui::g_server_poller.HeartbeatTickCountForTest() != ticks_at_done ||
                 std::chrono::steady_clock::now() - hb_start >= hb_budget;
        },
        20));
    IM_CHECK_GT(gui::g_server_poller.HeartbeatTickCountForTest(), ticks_at_done);
    ctx->Yield();  // one more SyncFromPoller AFTER a heartbeat tick published
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
  };

  // Stop is non-blocking: it paints immediately and settles once the backend has drained.
  //
  // Determinism without sleeps: run_intent is written synchronously by DoStop, and the terminal
  // state is only checked after JoinPendingStop() has drained the background thread.
  ImGuiTest* t_stop = IM_REGISTER_TEST(engine, "run_lifecycle", "stop_paints_immediately_and_settles_after_drain");
  t_stop->TestFunc = [](ImGuiTestContext* ctx) {
    ScopedRunScene scene;
    IM_CHECK(scene.ok());
    SeedInfiniteRun();  // still running when the Stop arrives — the transition the feature targets

    gui::DoRun(/*user_initiated=*/true);
    IM_CHECK(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kSimulating; }, 10));

    gui::DoStop();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kStopping));
    IM_CHECK(gui::g_stop_inflight.load());
    // Re-entry while draining is a no-op (P4's other half — the disabled button blocks it at the UI
    // layer, and DoStop refuses it again underneath, because the future is single-owner and must
    // not be overwritten while pending).
    gui::DoStop();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kStopping));

    gui::JoinPendingStop();
    IM_CHECK(!gui::g_stop_inflight.load());  // the background thread returned ⇒ the backend drained

    // The terminal display takes two frames: one advances the intent kStopping→kStopped, the next
    // reconciles it to the terminal state. Frames are driven through the real main loop rather than
    // by calling SyncFromPoller() from this coroutine thread, which has no current GL context — a
    // freshly staged, not-yet-uploaded payload would take the upload branch there and dereference a
    // null GL dispatch table.
    IM_CHECK(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kDone; }, 10));
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kStopped));
  };

  // A completed run latches, and nothing the display side does afterwards disturbs it.
  //
  // The user-visible bug: after a finite run completed, toggling any of colour / visible / solo /
  // z-order / composite mode made the toolbar flash Run→Stop and the status bar say "Simulating…".
  // Two mechanisms had to hold together — the poller must not publish an invalid observation across
  // a display-time refresh wake, and a completed intent must be structurally immune to a late
  // invalid observation if one arrives anyway.
  //
  // The edits are driven through the real widgets (a colour-picker popup, a plain click, a held
  // modifier, an arrow button, a combo) rather than by writing the fields. The widgets ARE pure
  // field-writers, so a field poke reaches the same reconciler — but only the widget path exercises
  // the multi-frame interaction around it: a popup that is open across frames, a frame where the
  // mouse is down but the click has not registered, a modifier held while another item is clicked.
  // The direct-field-write sibling this replaces asserted the same invariant over the same five
  // categories through the strictly weaker path.
  //
  // Each edit is followed by a multi-frame scan rather than one assertion, because the claim is
  // "does not flash": a single post-click check would step straight over a one-frame regression.
  ImGuiTest* t_latch = IM_REGISTER_TEST(engine, "run_lifecycle", "display_edits_via_real_ui_do_not_disturb_done");
  t_latch->TestFunc = [](ImGuiTestContext* ctx) {
    ScopedRunScene scene;
    IM_CHECK(scene.ok());
    SeedFiniteRun(0.5f);

    // Two match-all classes on the default crystal, seeded BEFORE the run so the committed config
    // carries them and the display-push lane is actually exercised (an empty raypath_color
    // short-circuits the display diff). z_order ascends with rank so the wildcard lookups below —
    // which resolve to the FIRST matching item in this frame's submission order — deterministically
    // hit the rank-0 row without a fragile PushID-based literal path.
    gui::ColorClassRefConfig ref0;
    ref0.layer_idx = 0;
    ref0.crystal_pool_id = gui::g_state.layers[0].entries[0].crystal_id;
    ref0.match_all = true;
    gui::ColorClassConfig c0;
    c0.color[0] = 1.0f;
    c0.visible = true;
    c0.z_order = 0;
    c0.match.push_back(ref0);
    gui::ColorClassConfig c1;
    c1.color[1] = 1.0f;
    c1.visible = true;
    c1.z_order = 1;
    c1.match.push_back(ref0);
    gui::g_state.raypath_color.push_back(c0);
    gui::g_state.raypath_color.push_back(c1);

    gui::DoRun(/*user_initiated=*/true);
    IM_CHECK(ReachedDoneAndLatched(ctx));

    // The lifecycle clock must not advance either: a display-time edit that re-committed would
    // restart the simulation the user just finished.
    const uint64_t baseline_epoch = gui::g_state.committed_epoch;
    // Which frame of which edit went wrong is the whole diagnostic here, so the disturbed frames
    // are collected and reported together rather than asserted inside the scan — an assert there
    // would return out of this case on the first one and take the remaining edits with it.
    auto scan = [&](const char* edit) {
      std::string disturbed;
      for (int frame = 0; frame < 5; ++frame) {
        ctx->Yield();
        if (gui::g_state.sim_state != SimState::kDone || gui::g_state.run_intent != RunIntent::kRunCompleted ||
            gui::g_state.committed_epoch != baseline_epoch) {
          disturbed += " " + std::string(edit) + ":frame" + std::to_string(frame) + ":state" +
                       std::to_string(static_cast<int>(gui::g_state.sim_state)) + ":intent" +
                       std::to_string(static_cast<int>(gui::g_state.run_intent)) + ":epoch" +
                       std::to_string(gui::g_state.committed_epoch);
        }
      }
      IM_CHECK_STR_EQ(disturbed.c_str(), "");
    };

    gui::g_state.color_window_open = true;
    ctx->Yield(2);
    // ImGui keys a window's position and size by title, so both survive across cases in one process
    // — and another suite deliberately moves this window off to the left and never puts it back,
    // which can clip this row's leftmost controls out of view. Pin them.
    ctx->WindowMove("//" ICON_FA_PALETTE " Colors", ImVec2(50, 50));
    ctx->WindowResize("//" ICON_FA_PALETTE " Colors", ImVec2(720, 480));
    ctx->Yield(2);
    ctx->SetRef("//" ICON_FA_PALETTE " Colors");

    // Colour: the swatch opens a picker popup; clicking the middle of its saturation/value square
    // is never the pure-red corner it starts from, so the click always produces an edit. Two
    // ID quirks stack here: `##color` is the PushID scope ColorEdit3 opens around itself (its only
    // clickable child is ImGui's own "##ColorButton"), and ColorButton never reports a debug label,
    // so a wildcard search by label can never find it. The literal path sidesteps both — `$$0`
    // reproduces PushID(0)'s int hash, and literal resolution looks the item up by ID.
    ctx->ItemClick("$$0/##color/##ColorButton");
    ctx->Yield();
    ctx->ItemClick("**/sv");
    ctx->PopupCloseAll();
    scan("color");
    IM_CHECK_NE(gui::g_state.raypath_color[0].color[0], 1.0f);  // sanity: the click landed

    IM_CHECK(gui::g_state.raypath_color[0].visible);
    ctx->ItemClick("**/" ICON_FA_EYE);
    scan("visible");
    IM_CHECK(!gui::g_state.raypath_color[0].visible);

    // Solo is Alt+click. Rank 0 now draws the crossed-out eye (visible was just turned off) while
    // rank 1 still draws the plain one, so this lookup is unambiguous by construction.
    ctx->KeyDown(ImGuiMod_Alt);
    ctx->ItemClick("**/" ICON_FA_EYE_SLASH);
    ctx->KeyUp(ImGuiMod_Alt);
    scan("solo");
    IM_CHECK(gui::g_state.raypath_color[0].solo);

    const int z0_before = gui::g_state.raypath_color[0].z_order;
    const int z1_before = gui::g_state.raypath_color[1].z_order;
    ctx->ItemClick("**/" ICON_FA_ARROW_DOWN "##down");
    scan("z_order");
    IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, z1_before);
    IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, z0_before);

    // Composite mode: the default is already painter, so a click on "painter" would observe no
    // transition at all. Establish dominant first, then observe the move back.
    ctx->ComboClick("##ColorMode/dominant");
    scan("mode:dominant");
    IM_CHECK_EQ(gui::g_state.raypath_color_mode, 0);
    ctx->ComboClick("##ColorMode/painter");
    scan("mode:painter");
    IM_CHECK_EQ(gui::g_state.raypath_color_mode, 2);

    // Finally, the latch against the observation that used to demote it: an invalid publish through
    // the live poller, the exact shape a display-time refresh wake once produced. The reconcile's
    // answer for (latched completion, invalid observation) is pinned a layer down; what is pinned
    // here is that the real poller's reset path reaches SyncFromPoller without demoting anything.
    gui::g_server_poller.PublishValidResetForTest();
    ctx->Yield(2);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunCompleted));

    ctx->SetRef("");
    gui::g_state.color_window_open = false;
    gui::g_state.raypath_color.clear();
    ctx->Yield(2);
  };

  // Editing the scene while a run is going and pressing Run again repaints from the new
  // configuration. Three separate registrations rather than one table: they are three historically
  // independent guards, and folding them into one function body would report one result for three
  // failure modes.
  //
  // The signal is texture_upload_count, which is monotonic — "the preview was painted again" is the
  // user-visible end of a restart, and it is the one observation that cannot be satisfied by the
  // commit alone.
  ImGuiTest* t_crystal = IM_REGISTER_TEST(engine, "run_lifecycle", "a_crystal_edit_restarts_the_run_and_repaints");
  t_crystal->TestFunc = [](ImGuiTestContext* ctx) {
    DriveRestart(
        ctx, [] {},
        [] {
          gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height = 2.5f;
          gui::g_state.dirty = true;
        },
        1500);
  };

  // The filter row takes longer than the crystal row on purpose: a filter edit is a structural one,
  // so it raises the epoch floor and fences the old generation's textures until the re-commit mints
  // a newer epoch. This is the end-to-end exercise of that anti-flicker gate — the fence itself is
  // pinned in composition-correctness/gui/test_run_lifecycle_chain.cpp.
  ImGuiTest* t_filter = IM_REGISTER_TEST(engine, "run_lifecycle", "a_filter_edit_restarts_the_run_and_repaints");
  t_filter->TestFunc = [](ImGuiTestContext* ctx) {
    DriveRestart(
        ctx,
        [] {
          gui::FilterConfig f;
          f.SetRaypath(gui::RaypathParams{ "3-1-5" });
          gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
        },
        [] {
          gui::g_state.filters[static_cast<size_t>(*gui::g_state.layers[0].entries[0].filter_id)].MutableRaypathText() =
              "3-1-5-7";
          gui::g_state.MarkStructHardDirty();
        },
        2000);
  };

  ImGuiTest* t_budget = IM_REGISTER_TEST(engine, "run_lifecycle", "ending_an_infinite_run_restarts_and_repaints");
  t_budget->TestFunc = [](ImGuiTestContext* ctx) {
    DriveRestart(
        ctx, [] {},
        [] {
          gui::g_state.sim.infinite = false;
          gui::g_state.sim.ray_num_millions = 0.5f;  // small enough that the new run can finish
          gui::g_state.dirty = true;
        },
        2000);
  };
}
