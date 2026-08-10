// The file commands in the top bar, and the two modals standing between the user and losing work.
//
// What this suite is for. Three things here can only be answered by a real toolbar and real
// clicks. The first is which commands are reachable in which state — New and Open close while a
// simulation runs, the Save menu deliberately does not, and each item inside it is gated on its own
// precondition rather than on one shared "busy" flag. The second is the prompt chain: an unsaved
// document being replaced can route through TWO modals in sequence, and every branch of the second
// one decides the fate of an action queued by the first. The third is what a document switch leaves
// behind on screen, which is a question about GL pixels and a poller that stages frames of its own,
// and therefore about a running frame loop.
//
// Deliberately NOT here. Whether a saved document comes back the same is a serializer question and
// lives in composition-correctness/gui/test_document_roundtrip_chain.cpp and
// test_legacy_document_chain.cpp; which reset primitives a document switch owns is
// test_document_switch_chain.cpp; what an import is entitled to warn about is
// test_filter_reconstruct_chain.cpp. Nothing below re-asserts any of them — the cases here start
// from "the file layer works" and ask what the user can reach and what stays on screen.
//
// What a user sees when these break: Save greys out mid-simulation so a finished render cannot be
// written out; "Don't Save" silently keeps the document, or Cancel throws it away; a prompt appears
// with no way out; a config is imported and quietly simplified with nothing said about it; a new
// document opens showing the previous one's halo, which reads as "the file did not load".

#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "gui/export_fbo_renderer.hpp"  // RenderExportToRgba — the non-proxy view of the sim layer
#include "gui/gui_state.hpp"
#include "gui/server_poller.hpp"
#include "support/scoped_result_frame.hpp"
#include "test_gui_shared.hpp"

namespace {

// A frame budget for anything that has to cross to the render thread and back. Bounded rather than
// unbounded so a wiring regression fails at the assertion that follows, and at that line, instead
// of hanging the suite.
constexpr int kSettleYieldLimit = 60;

template <typename Fn>
void YieldUntilTrue(ImGuiTestContext* ctx, int max_yields, Fn&& condition) {
  for (int i = 0; i < max_yields && !condition(); ++i) {
    ctx->Yield();
  }
}

bool FileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

// ---------------------------------------------------------------------------------------------
// GL scaffolding.
//
// Uploading a texture and reading the framebuffer back both need the GL context, which is current
// only on the render thread; TestFunc runs on the test engine's coroutine worker without it. So the
// cases describe an operation, the GuiFunc performs it on the next frame, and the answer comes back
// as a centre pixel. GuiFuncs are plain function pointers — ImGuiTestGuiFunc rejects capturing
// lambdas.
// ---------------------------------------------------------------------------------------------

struct GlOp {
  bool requested = false;
  bool done = false;

  // Inputs, applied in this order within one frame.
  bool upload_stale = false;
  bool upload_bg = false;
  bool clear = false;
  bool upload_fresh_after_clear = false;
  bool bg_enabled = false;

  // Outputs.
  bool export_ok = false;
  unsigned char center_r = 0;
  unsigned char center_g = 0;
  unsigned char center_b = 0;

  void Reset() { *this = GlOp{}; }
};

GlOp g_gl_op;

// Fills chosen so ONE centre-pixel read tells four states apart with no tolerance: the previous
// scene's pixels, the black a clear leaves, a background showing through, and a fresh upload.
constexpr unsigned char kStaleMagenta[3] = { 0xFF, 0x00, 0xFF };
constexpr unsigned char kFreshCyan[3] = { 0x00, 0xFF, 0xFF };
constexpr unsigned char kBgGreen[3] = { 0x00, 0xFF, 0x00 };

constexpr int kProbeW = 32;
constexpr int kProbeH = 16;

void FillSolid(std::vector<unsigned char>& buf, int w, int h, const unsigned char rgb[3]) {
  buf.resize(static_cast<size_t>(w) * h * 3);
  for (int i = 0; i < w * h; ++i) {
    buf[i * 3 + 0] = rgb[0];
    buf[i * 3 + 1] = rgb[1];
    buf[i * 3 + 2] = rgb[2];
  }
}

// Runs the requested uploads/clears and then ONE render, in a single frame. Single-frame matters
// for the interleaving case: a clear followed by an upload with no Render() in between is exactly
// the sequence in which a deferred blank could overwrite pixels that arrived after it.
void GlOpGuiFunc(ImGuiTestContext*) {
  if (!g_gl_op.requested || g_gl_op.done) {
    return;
  }
  constexpr int kTexW = 16;
  constexpr int kTexH = 8;
  std::vector<unsigned char> pixels;
  if (g_gl_op.upload_stale) {
    FillSolid(pixels, kTexW, kTexH, kStaleMagenta);
    gui::g_preview.UploadTexture(pixels.data(), kTexW, kTexH);
  }
  if (g_gl_op.upload_bg) {
    FillSolid(pixels, kTexW, kTexH, kBgGreen);
    gui::g_preview.UploadBgTexture(pixels.data(), kTexW, kTexH);
  }
  if (g_gl_op.clear) {
    gui::g_preview.ClearTexture();
  }
  if (g_gl_op.upload_fresh_after_clear) {
    FillSolid(pixels, kTexW, kTexH, kFreshCyan);
    gui::g_preview.UploadTexture(pixels.data(), kTexW, kTexH);
  }

  // The equirectangular projection fills the whole frame with texture samples, so a uniformly
  // coloured source gives a uniformly coloured result and the centre pixel is representative —
  // no fisheye cutout to fall into.
  gui::PreviewParams params;
  gui::ConfigureEquirectExportParams(params);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // 8-bit RGB mode: the texture is sampled as authored
  if (g_gl_op.bg_enabled) {
    params.bg.enabled = true;
    params.bg.alpha = 0.0f;  // shader is bg*(1-alpha) + sim*alpha, so alpha=0 is pure background
    params.bg.aspect = static_cast<float>(kTexW) / static_cast<float>(kTexH);
  }
  auto rgba = gui::RenderExportToRgba(gui::g_preview, params, kProbeW, kProbeH, std::nullopt);
  g_gl_op.export_ok = !rgba.empty();
  if (g_gl_op.export_ok) {
    const size_t off = (static_cast<size_t>(kProbeH / 2) * kProbeW + kProbeW / 2) * 4;
    g_gl_op.center_r = rgba[off + 0];
    g_gl_op.center_g = rgba[off + 1];
    g_gl_op.center_b = rgba[off + 2];
  }
  g_gl_op.done = true;
}

// Ask for a render with no GL operations in front of it: what does the sim layer show right now.
void SampleCenterPixel(ImGuiTestContext* ctx) {
  g_gl_op.Reset();
  g_gl_op.requested = true;
  YieldUntilTrue(ctx, kSettleYieldLimit, [] { return g_gl_op.done; });
}

// ---------------------------------------------------------------------------------------------
// A previous document, really on screen and really staged in the poller.
//
// The four document-entry cases below each need BOTH halves of "the previous document is still
// here": pixels in the GL texture, and a composite snapshot the poller is holding and would
// re-upload on the next sync. Clearing only the first is precisely the bug that recurred — the
// screen goes blank for a frame and the previous scene comes straight back.
// ---------------------------------------------------------------------------------------------

// A single prism in the wire format LUMICE_SceneFromJson reads, carrying one match-all red colour
// class so the run produces a composite (a colourless run stages no composite payload at all).
const char* const kColoredConfig = R"({
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
  LUMICE_Scene* scene = nullptr;
  if (LUMICE_SceneFromJson(json, &scene) != LUMICE_OK) {
    return false;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  if (err != LUMICE_OK) {
    return false;
  }
  for (int waited = 0; waited < 5000; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[2]{};
      lumice::test::ScopedResultFrame frame(server);
      LUMICE_FrameGetRawXyz(frame.get(), xyz, 1);
      if (xyz[0].has_valid_data) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// A server whose result is staged in the GLOBAL poller, torn down on every exit path. Held by an
// object because IM_CHECK expands to `return`: a failure between creation and a hand-written
// teardown would leave a running server and a polling thread for the next case to inherit.
struct ScopedStagedDocument {
  LUMICE_Server* server = nullptr;
  bool staged = false;

  explicit ScopedStagedDocument(ImGuiTestContext* ctx) {
    server = LUMICE_CreateServer();
    if (server == nullptr || !RunToIdleWithData(server, kColoredConfig)) {
      return;
    }
    // The global poller, not a local one: production SyncFromPoller reads only from g_server_poller,
    // so a composite has to be published there for the main thread to pick it up and upload it.
    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);
    YieldUntilTrue(ctx, kSettleYieldLimit, [] { return gui::g_preview.HasTexture(); });
    const auto snap = gui::g_server_poller.LoadSnapshot();
    staged = gui::g_preview.HasTexture() && snap != nullptr && snap->payload != nullptr;
  }

  ~ScopedStagedDocument() {
    gui::g_server_poller.Stop();
    if (server != nullptr) {
      LUMICE_DestroyServer(server);
      server = nullptr;
    }
  }

  ScopedStagedDocument(const ScopedStagedDocument&) = delete;
  ScopedStagedDocument& operator=(const ScopedStagedDocument&) = delete;

  unsigned long long staged_serial() const {
    const auto snap = gui::g_server_poller.LoadSnapshot();
    return snap != nullptr ? snap->texture_serial : 0;
  }
};

// Both halves of "nothing of the previous document is left", asserted together because either one
// alone passes on a fix that only did the other. The pixel read is the non-proxy evidence: a
// CPU-side flag flipping to false is what the first two attempts at this bug already satisfied.
void ExpectPreviousDocumentGone(ImGuiTestContext* ctx, unsigned long long serial_before) {
  IM_CHECK(!gui::g_preview.HasTexture());

  const auto snap = gui::g_server_poller.LoadSnapshot();
  IM_CHECK(snap != nullptr);
  IM_CHECK(snap->payload == nullptr);
  // The serial is deliberately NOT reset: it is what a genuinely new frame is compared against, so
  // rewinding it here would make the next real upload look like one already on screen.
  IM_CHECK_EQ(snap->texture_serial, serial_before);

  SampleCenterPixel(ctx);
  IM_CHECK(g_gl_op.export_ok);
  IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
  IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
  IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);
}

// ---------------------------------------------------------------------------------------------
// The prompt chain.
// ---------------------------------------------------------------------------------------------

// Put the document into "edited since the last run and not yet saved", the state both prompts
// exist for. run_intent rather than a direct sim_state write: ReconcileSimState re-derives
// sim_state from the intent every frame, so a direct write does not survive the next Yield.
void SeedModifiedDocument(ImGuiTestContext* ctx, const std::string& path) {
  gui::g_state.current_file_path = path;
  gui::g_state.layers[0].entries.push_back(gui::EntryCard{});
  gui::g_state.run_intent = gui::RunIntent::kLoaded;
  gui::g_state.dirty = true;
  ctx->Yield();
}

// New -> Unsaved prompt -> Save, arriving at the Save-Modified prompt. Three frames on the last
// hop: the click handler calls DoSave() mid-frame, which raises the flag that
// RenderSaveModifiedPopup — called right after RenderUnsavedPopup in both the product's loop and
// this harness's — consumes to open the second modal.
void DriveNewIntoTheModifiedPrompt(ImGuiTestContext* ctx) {
  ctx->ItemClick("##TopBar/New");
  ctx->Yield(2);
  ctx->ItemClick("Unsaved Changes/Save");
  ctx->Yield(3);
}

}  // namespace

void RegisterFileOpsTests(ImGuiTestEngine* engine) {
  // -------------------------------------------------------------------------------------------
  // What is reachable, and when (P6, P7, P8).
  // -------------------------------------------------------------------------------------------

  // While a run is in flight, New and Open would throw away the run; Save would not. The asymmetry
  // is the proposition: a finished or in-progress render must stay exportable, and the obvious
  // implementation — one BeginDisabled around the whole file group — gets it wrong in the direction
  // that costs the user their picture.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "a_running_simulation_closes_new_and_open_but_not_save");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK(!IsDisabled(ctx->ItemInfo("##TopBar/New")));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("##TopBar/Open")));

      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield(2);
      IM_CHECK(IsDisabled(ctx->ItemInfo("##TopBar/New")));
      IM_CHECK(IsDisabled(ctx->ItemInfo("##TopBar/Open")));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("##TopBar/Save")));

      gui::g_state.run_intent = gui::RunIntent::kNone;
      ctx->Yield(2);
    };
  }

  // Six items behind one button, and each one's precondition is its own: writing the document needs
  // the simulation not to be running; a screenshot needs something on screen to shoot; the two
  // panorama exports need a live server to re-render from; exporting the config needs nothing at
  // all. The failure this pins is the cheap fix — gate them all on the same flag — which silently
  // takes away the three commands that were still legitimate.
  //
  // Read through ItemInfo rather than by clicking: every one of these items opens a native file
  // dialog, which blocks and cannot be driven from a test.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "each_save_menu_item_is_gated_on_its_own_precondition");
    t->GuiFunc = [](ImGuiTestContext*) {
      // A texture, uploaded on the render thread, so the Screenshot gate has both states to show.
      if (g_export_test.upload_requested && !g_export_test.upload_done) {
        InitSynthTexture();
        gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
        g_export_test.upload_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // One row of the gate table: an item, and whether it should be reachable right now.
      struct Item {
        const char* path;
        bool expect_enabled;
      };
      // Open the menu, read every gate, close it again.
      //
      // The ref is pointed at the popup first: a "**/" path resolves by label, and the menu's own
      // "Save" item shares its label with the top bar button that opened it — searched from the
      // root, "**/Save" finds the button, which is never disabled, and the gate reads as passing
      // whatever the menu does. Only SetRef understands "$FOCUSED", so the popup has to become the
      // ref before the search rather than inside it.
      //
      // Reported per item rather than asserted fatally, so a run names every gate that drifted
      // instead of only the first — and so the menu is always closed on the way out. A fatal
      // assertion here returns with the popup still up, and an open popup eats every click the
      // next case makes.
      const auto check = [](ImGuiTestContext* c, const char* scenario, const Item* items, int n) {
        c->ItemClick("##TopBar/Save");
        c->Yield(2);
        c->SetRef("//$FOCUSED");
        for (int i = 0; i < n; ++i) {
          const bool disabled = IsDisabled(c->ItemInfo(items[i].path));
          if (disabled == items[i].expect_enabled) {
            IM_ERRORF("%s: '%s' is %s and should not be", scenario, items[i].path,
                      disabled ? "unreachable" : "reachable");
          }
        }
        c->SetRef("");
        c->PopupCloseAll();
        c->Yield(2);
      };

      // Idle, no texture, no server: writing the document is fine, the two panoramas and the
      // screenshot have nothing to work from, the config export is unconditional.
      IM_CHECK(gui::g_server == nullptr);
      IM_CHECK(!gui::g_preview.HasTexture());
      const Item kIdleNoTexture[] = {
        { "**/Save", true },
        { "**/Save Copy", true },
        { "**/Screenshot...", false },
        { "**/Dual Fisheye Equal Area...", false },
        { "**/Equirectangular...", false },
        { "**/Config JSON...", true },
      };
      check(ctx, "idle, nothing rendered", kIdleNoTexture, IM_ARRAYSIZE(kIdleNoTexture));

      // A rendered preview unlocks the screenshot and nothing else — the panoramas re-render from
      // the server, not from the texture.
      g_export_test.Reset();
      g_export_test.upload_requested = true;
      YieldUntilTrue(ctx, kSettleYieldLimit, [] { return g_export_test.upload_done; });
      IM_CHECK(gui::g_preview.HasTexture());
      const Item kWithTexture[] = {
        { "**/Screenshot...", true },
        { "**/Dual Fisheye Equal Area...", false },
        { "**/Equirectangular...", false },
      };
      check(ctx, "a preview is on screen", kWithTexture, IM_ARRAYSIZE(kWithTexture));

      // Running: the document writes close, the read-only exports stay open. This is the same
      // asymmetry the case above states at the button, restated here per menu item because the two
      // are separate BeginDisabled scopes and only one of them is what the user reaches for.
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield(2);
      const Item kRunning[] = {
        { "**/Save", false },
        { "**/Save Copy", false },
        { "**/Screenshot...", true },
        { "**/Config JSON...", true },
      };
      check(ctx, "a run is in flight", kRunning, IM_ARRAYSIZE(kRunning));
      gui::g_state.run_intent = gui::RunIntent::kNone;
      ctx->Yield();
    };
  }

  // The two checkboxes in the Save menu are preferences, not commands: they must read back checked
  // the next time the menu opens. A menu item bound to a copy of the flag toggles happily and
  // forgets, and the user finds out when the file they saved has no texture in it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "the_save_menu_toggles_are_remembered_across_openings");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const bool texture_before = gui::g_state.save_texture;
      IM_CHECK(!gui::g_state.screenshot_include_overlay);  // documented default: a plain screenshot

      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);
      ctx->ItemClick("**/Include Overlay in Screenshot");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.screenshot_include_overlay);

      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);
      ctx->ItemClick("**/Include Texture in .lmc");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.save_texture, !texture_before);

      // Reopen and read the marks back: both edits survived, and the second did not reset the first.
      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.screenshot_include_overlay);
      IM_CHECK_EQ(gui::g_state.save_texture, !texture_before);
      ctx->ItemClick("**/Include Overlay in Screenshot");  // back off, and observe it move
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.screenshot_include_overlay);
    };
  }

  // -------------------------------------------------------------------------------------------
  // The unsaved-changes prompt (P49).
  // -------------------------------------------------------------------------------------------

  // Cancel means "forget I asked": the document is untouched AND the queued New is dropped. The
  // second half is what makes Cancel safe — a queued action left behind fires on the next prompt
  // the user answers, which is a New the user never asked for.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "cancelling_the_unsaved_prompt_keeps_the_document");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.layers[0].entries.push_back(gui::EntryCard{});
      gui::g_state.dirty = true;
      ctx->Yield();

      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);
      ctx->ItemClick("Unsaved Changes/Cancel");
      ctx->Yield(2);

      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
    };
  }

  // Don't Save runs the queued action immediately, discarding the edits — the branch with no
  // recovery, and the one a user picks fastest.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "dont_save_discards_the_edits_and_runs_the_queued_action");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.layers[0].entries.push_back(gui::EntryCard{});
      gui::g_state.dirty = true;
      ctx->Yield();

      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);
      ctx->ItemClick("Unsaved Changes/Don't Save");
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
    };
  }

  // Save, on a document with nothing to warn about, writes and then lets the queued New through —
  // in that order. The file is read back rather than merely stat'ed, because "the New ran first"
  // and "the save wrote the wrong document" are the same size on disk.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "file_ops", "saving_from_the_unsaved_prompt_writes_before_the_queued_action");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::string path = GuiTestTempPath("lumice_unsaved_save.lmc").string();
      std::remove(path.c_str());
      gui::g_state.current_file_path = path;
      auto& crystal = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
      crystal.type = gui::CrystalType::kPyramid;
      crystal.prism_h = 3.5f;
      gui::g_state.dirty = true;
      ctx->Yield();

      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);
      ctx->ItemClick("Unsaved Changes/Save");
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));

      gui::GuiState saved;
      std::vector<unsigned char> tex;
      int tex_w = 0;
      int tex_h = 0;
      IM_CHECK(gui::LoadLmcFile(path, saved, tex, tex_w, tex_h));
      IM_CHECK_EQ(gui::CrystalOf(saved, saved.layers[0].entries[0]).type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::CrystalOf(saved, saved.layers[0].entries[0]).prism_h, 3.5f);

      std::remove(path.c_str());
    };
  }

  // -------------------------------------------------------------------------------------------
  // The Save-Modified prompt (P50, P51, P52).
  //
  // Saving a document that has been edited since its last run would write a file whose baked
  // preview does not match its own config, and clear the "re-run me" cue on the way. The prompt
  // stops that, and each of its three buttons has to decide what happens to an action the FIRST
  // prompt queued — which is the part that was implemented three times and drifted.
  // -------------------------------------------------------------------------------------------

  // The gate itself: Save on a modified document opens the prompt and touches nothing. Driven
  // through DoSave() directly rather than through the menu, because the menu item opens a native
  // file dialog; the button wiring is driven for real two cases down.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "saving_a_modified_document_asks_first_and_writes_nothing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::string path = GuiTestTempPath("lumice_savemod_gate.lmc").string();
      std::remove(path.c_str());
      gui::g_state.current_file_path = path;
      gui::g_state.sim_state = gui::GuiState::SimState::kModified;
      gui::g_state.dirty = true;

      IM_CHECK(!gui::g_show_save_modified_popup);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kNone));

      gui::DoSave();

      IM_CHECK(gui::g_show_save_modified_popup);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kSave));
      IM_CHECK(!FileExists(path));
      // dirty must survive: only a real serialization is allowed to clear it.
      IM_CHECK(gui::g_state.dirty);

      gui::g_show_save_modified_popup = false;
      gui::g_pending_save_kind = gui::PendingSaveKind::kNone;
    };
  }

  // ...and the other side of the gate, which is what keeps it from being "always ask": a document
  // that has never been run has nothing stale to warn about, so Save writes straight through.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "saving_an_unrun_document_does_not_ask");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::string path = GuiTestTempPath("lumice_savemod_bypass.lmc").string();
      std::remove(path.c_str());
      gui::g_state.current_file_path = path;
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
      gui::g_state.dirty = true;

      gui::DoSave();

      IM_CHECK(!gui::g_show_save_modified_popup);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kNone));
      IM_CHECK(FileExists(path));
      IM_CHECK(!gui::g_state.dirty);

      std::remove(path.c_str());
    };
  }

  // The full chain through real buttons: New raises the unsaved prompt, its Save routes into the
  // modified prompt instead of writing, and only "Save anyway" — the one branch that actually
  // persists something — lets the queued New through afterwards.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "save_anyway_writes_and_then_releases_the_queued_action");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const std::string path = GuiTestTempPath("lumice_chain_anyway.lmc").string();
      std::remove(path.c_str());
      SeedModifiedDocument(ctx, path);

      DriveNewIntoTheModifiedPrompt(ctx);
      IM_CHECK(ImGui::IsPopupOpen("Save Modified Config"));
      IM_CHECK(!FileExists(path));
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);  // the New has not run

      ctx->ItemClick("Save Modified Config/Save anyway");
      ctx->Yield(2);

      IM_CHECK(FileExists(path));
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);  // ...and now it has

      std::remove(path.c_str());
    };
  }

  // Cancel on the second prompt must abort the queued New as well. Proceeding without it would
  // discard exactly the edit the first prompt was raised to protect — silently, since by then the
  // user has answered two dialogs and the last thing they pressed was Cancel.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "cancelling_the_modified_prompt_aborts_the_queued_action");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const std::string path = GuiTestTempPath("lumice_chain_cancel.lmc").string();
      std::remove(path.c_str());
      SeedModifiedDocument(ctx, path);

      DriveNewIntoTheModifiedPrompt(ctx);
      ctx->ItemClick("Save Modified Config/Cancel");
      ctx->Yield(2);

      IM_CHECK(!FileExists(path));
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
    };
  }

  // "Run first" is the third exit, and it is the one with a precondition: there has to be a server
  // to run on. It also abandons the queued New — a run persists nothing, so proceeding would still
  // throw the edit away. Both halves here because the enable gate alone would pass on a button
  // that then did the wrong thing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "run_first_needs_a_server_and_abandons_the_queued_action");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const std::string path = GuiTestTempPath("lumice_chain_runfirst.lmc").string();
      std::remove(path.c_str());
      SeedModifiedDocument(ctx, path);

      // No server yet: the button is drawn, and unreachable.
      IM_CHECK(gui::g_server == nullptr);
      DriveNewIntoTheModifiedPrompt(ctx);
      IM_CHECK(ImGui::IsPopupOpen("Save Modified Config"));
      IM_CHECK(IsDisabled(ctx->ItemInfo("Save Modified Config/Run first")));

      // Give it one, without leaving the prompt.
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);
      gui::g_server_is_gpu = false;
      ctx->Yield(2);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("Save Modified Config/Run first")));

      ctx->ItemClick("Save Modified Config/Run first");
      ctx->Yield(2);

      // A run was asked for; nothing was written; and the New is gone rather than pending.
      IM_CHECK(!ImGui::IsPopupOpen("Save Modified Config"));
      IM_CHECK(!FileExists(path));
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kNone));
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNone));
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      gui::g_server_poller.Stop();
      gui::JoinPendingStop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
    };
  }

  // Escape is not an exit. Dear ImGui routes Escape to a popup's close path only when the topmost
  // popup is NOT flagged modal, and BeginPopupModal always sets that flag — so this prompt, like
  // every modal in the app, cannot be dismissed that way. Pinned with a real key press rather than
  // left as a comment: if a future Dear ImGui changes it, the prompt starts closing here with the
  // queued action and the pending save kind still set, and this case is what says so.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "escape_does_not_dismiss_the_modified_prompt");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const std::string path = GuiTestTempPath("lumice_chain_escape.lmc").string();
      std::remove(path.c_str());
      SeedModifiedDocument(ctx, path);

      DriveNewIntoTheModifiedPrompt(ctx);
      IM_CHECK(ImGui::IsPopupOpen("Save Modified Config"));
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNew));

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);

      IM_CHECK(ImGui::IsPopupOpen("Save Modified Config"));
      IM_CHECK(!FileExists(path));
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_action), static_cast<int>(gui::PendingAction::kNew));
      IM_CHECK_EQ(static_cast<int>(gui::g_pending_save_kind), static_cast<int>(gui::PendingSaveKind::kSave));

      // Leave through a real button: an open modal swallows the next case's clicks.
      ctx->ItemClick("Save Modified Config/Save anyway");
      ctx->Yield(2);
      IM_CHECK(!ImGui::IsPopupOpen("Save Modified Config"));
      std::remove(path.c_str());
    };
  }

  // -------------------------------------------------------------------------------------------
  // The import warning (P47).
  // -------------------------------------------------------------------------------------------

  // An imported config the GUI cannot fully represent is simplified on load, and the user has to be
  // told once. Three properties in one case because they are one behaviour: several warnings from
  // one import arrive as one modal with the messages joined, and OK closes it for good — the queue
  // is consumed, not merely displayed, so it does not reopen on the next frame forever.
  //
  // What is warned about, and when, is settled in composition-correctness/gui — the message is
  // pushed here directly through the same entry point the importer uses.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "the_import_warning_appears_once_and_joins_its_messages");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(!ImGui::IsPopupOpen("Import Warning"));

      gui::SetImportComplexFilterWarning("first thing that did not fit");
      gui::SetImportComplexFilterWarning("second thing that did not fit");
      // Both messages are still queued: nothing is consumed until a frame draws the modal.
      IM_CHECK(gui::PeekImportComplexFilterWarning().find("first") != std::string::npos);
      IM_CHECK(gui::PeekImportComplexFilterWarning().find("second") != std::string::npos);

      ctx->Yield(2);
      IM_CHECK(ImGui::IsPopupOpen("Import Warning"));
      // One modal, and the queue drained into it — a second warning must not be waiting to pop
      // again the moment this one closes.
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());

      ctx->ItemClick("Import Warning/OK");
      ctx->Yield(2);
      IM_CHECK(!ImGui::IsPopupOpen("Import Warning"));

      // And it stays closed: the trigger is the queued message, not a flag that survives the modal.
      ctx->Yield(4);
      IM_CHECK(!ImGui::IsPopupOpen("Import Warning"));
    };
  }

  // -------------------------------------------------------------------------------------------
  // What a document switch leaves on screen (the recurring one).
  //
  // Four entry paths and one carve-out. This defect came back three times, each time through a
  // different half of the same seam: the on-screen texture, and the composite the poller has staged
  // and would re-upload on the next sync. Each case below asserts BOTH halves for its own path, so
  // no path is covered by a proxy for another path's evidence.
  // -------------------------------------------------------------------------------------------

  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "file_ops", "opening_a_json_config_leaves_nothing_of_the_previous_document");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedStagedDocument previous(ctx);
      IM_CHECK(previous.staged);
      const auto serial = previous.staged_serial();

      // A plain config with no colour classes, so the document being opened cannot be confused
      // with the coloured one already on screen.
      std::string json;
      IM_CHECK(gui::BuildExportJsonOrWarn(gui::InitDefaultState(), &json, nullptr));
      const std::string path = GuiTestTempPath("lumice_switch_open.json").string();
      IM_CHECK(gui::ExportConfigJson(path, json));

      gui::DoOpen(path);  // CPU-only on this branch; the GL blank is consumed by the next render
      ExpectPreviousDocumentGone(ctx, serial);

      std::remove(path.c_str());
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "opening_an_lmc_without_a_baked_preview_leaves_nothing_behind");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const std::string path = GuiTestTempPath("lumice_switch_blank.lmc").string();
      IM_CHECK(gui::SaveLmcFile(path, gui::g_state, gui::g_preview, /*save_texture=*/false));

      ScopedStagedDocument previous(ctx);
      IM_CHECK(previous.staged);
      const auto serial = previous.staged_serial();

      gui::DoOpen(path);
      ExpectPreviousDocumentGone(ctx, serial);

      std::remove(path.c_str());
    };
  }

  // The one path whose postcondition is NOT a blank screen: an .lmc carrying a baked preview has to
  // show it. This is the case that stops "fence everything on every switch" from passing — and it
  // still owes the other half of the seam, since the staged composite must be dropped or the next
  // sync overwrites the baked image with the previous document's.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "opening_an_lmc_with_a_baked_preview_shows_the_baked_one");
    static bool s_open_done = false;
    static std::string s_open_path;
    t->GuiFunc = [](ImGuiTestContext* ctx) {
      GlOpGuiFunc(ctx);
      // UploadTexture is a GL call, so this branch of DoOpen has to run on the render thread.
      if (!s_open_done && !s_open_path.empty()) {
        gui::DoOpen(s_open_path);
        s_open_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_open_done = false;
      s_open_path.clear();

      // Bake a distinctive payload in through the CPU-side seam SaveLmcFile serializes, so the
      // file's pixels are exactly the colour asserted below.
      constexpr int kBakedW = 12;
      constexpr int kBakedH = 6;
      std::vector<unsigned char> baked;
      FillSolid(baked, kBakedW, kBakedH, kFreshCyan);
      gui::g_preview.UpdateCpuTextureData(baked.data(), kBakedW, kBakedH);
      const std::string path = GuiTestTempPath("lumice_switch_baked.lmc").string();
      IM_CHECK(gui::SaveLmcFile(path, gui::g_state, gui::g_preview, /*save_texture=*/true));

      ScopedStagedDocument previous(ctx);
      IM_CHECK(previous.staged);
      const auto serial = previous.staged_serial();

      s_open_path = path;
      YieldUntilTrue(ctx, kSettleYieldLimit, [] { return s_open_done; });
      IM_CHECK(s_open_done);

      // The staged composite is fenced, exactly as on the branches that blank the screen...
      const auto snap = gui::g_server_poller.LoadSnapshot();
      IM_CHECK(snap != nullptr);
      IM_CHECK(snap->payload == nullptr);
      IM_CHECK_EQ(snap->texture_serial, serial);

      // ...and what is on screen is the file's own preview, not black and not the previous one.
      IM_CHECK(gui::g_preview.HasTexture());
      SampleCenterPixel(ctx);
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);

      std::remove(path.c_str());
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "a_new_document_leaves_nothing_of_the_previous_one");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedStagedDocument previous(ctx);
      IM_CHECK(previous.staged);
      const auto serial = previous.staged_serial();

      gui::DoNew();
      ExpectPreviousDocumentGone(ctx, serial);
    };
  }

  // Revert is the carve-out and is pinned beside the four rather than left implied: it restores
  // configuration into the SAME document and does not end the run whose picture is on screen, so
  // that picture must survive. Without this, a later "clear the preview on every reset reason"
  // change would blank a still-accurate render and nothing would object.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "reverting_keeps_the_preview_it_did_not_invalidate");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // A snapshot to restore, or DoRevert's body is guarded out and the case asserts nothing.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.upload_stale = true;  // "stale" only by reuse of the fixture: this IS the current view
      YieldUntilTrue(ctx, kSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);

      gui::DoRevert();

      SampleCenterPixel(ctx);
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);
    };
  }

  // -------------------------------------------------------------------------------------------
  // The blanking primitive the four paths above share.
  //
  // ClearTexture does not touch GL: it raises a flag the next render consumes. Three cases because
  // three different things can go wrong with a deferred write, and each has been one of them.
  // -------------------------------------------------------------------------------------------

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "clearing_the_preview_reaches_the_pixels_not_just_the_flag");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.upload_stale = true;
      YieldUntilTrue(ctx, kSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);  // really in GL storage, not just CPU
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);

      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.clear = true;
      YieldUntilTrue(ctx, kSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);
    };
  }

  // The background is a separate layer and must not go with it. A user who loaded a sky photo and
  // opened a new document should still see the photo — losing it is collateral damage from the fix,
  // and it looks exactly like the fix working.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "clearing_the_preview_leaves_the_background_image");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.upload_stale = true;
      g_gl_op.upload_bg = true;
      g_gl_op.clear = true;
      g_gl_op.bg_enabled = true;
      YieldUntilTrue(ctx, kSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      // Pure background. Not magenta (the sim layer drawn over it) and not black (the background
      // cleared along with it).
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);
    };
  }

  // The ordering hazard a deferred write creates: clear, then upload, with no render in between.
  // The upload is the newer write and must win — otherwise the pending blank lands on top of it and
  // the freshly loaded document's own preview is what gets erased.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "file_ops", "an_upload_after_a_clear_wins_over_the_pending_blank");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.upload_stale = true;
      g_gl_op.clear = true;
      g_gl_op.upload_fresh_after_clear = true;
      YieldUntilTrue(ctx, kSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);
    };
  }
}
