// The Colors window and its two top-bar mirrors, driven through real frames and real clicks.
//
// What this suite is for: `src/gui/color_window.cpp` draws one window whose every control writes a
// GuiState field and nothing else — the frame-tail reconciler is the sole owner of the effects
// those writes produce. That split is exactly what a headless test cannot see. Falsifying "clicking
// this control lands on the structural lane rather than the display lane", "this row's icon agrees
// with what the compositor actually does", or "these two mirrors of the same predicate cannot
// disagree" all require a real frame to have drawn the control and a real click to have reached it.
// Anything here that stops needing a frame belongs one layer down, in
// test/unit-correctness/gui/test_color_window_logic.cpp, which already owns the pure predicates
// (EffectiveVisible / AnySolo / NoVisibleMatchedColorClass / CompactZOrder / SwapZOrder /
// ValidateSingleAtomText / BuildClassFromFilter / HandleEyeClick).
//
// What a user sees when these break, roughly in the order the cases appear: the Colors button does
// not bring the window up, or silently overwrites a colored/full-spectrum choice the user just
// made; the colored toggle claims a state the preview is not in; a control that cannot do anything
// right now still looks clickable (or, worse, one of the two mirrors looks clickable while the
// other does not); a class edit does not mark the document modified, so Revert never offers itself
// and the next Run silently drops the edit; reordering rows moves the wrong pair; the row jitters
// sideways every time an eye is toggled; an eye says "showing" for a class the composite has
// already excluded.
//
// Two propositions from the catalogue are deliberately NOT covered here, and the reason is
// mechanical rather than a judgement call. The top-bar aggregate pip and the per-row warning
// triangle are both drawn with ImGui::TextUnformatted, which calls ItemAdd() with id == 0; the
// test-engine registration hook sits inside that function's `if (id != 0)` branch, so neither
// glyph is addressable by any ItemExists / ItemInfo lookup. Their shared predicate
// (NoVisibleMatchedColorClass) is covered one layer down, and the state both indicators are
// derived from is covered here by empty_composite_disables_both_mirrors. Pixel-level coverage
// would need a new visual reference group, which is a scope decision, not an oversight.
//
// The disabled-state tooltip text being shared between the two mirrors (kColorsDisabledNoMatchTooltip)
// is likewise a single-constant fact at the source level; no frame can distinguish two call sites
// that reference the same constant from two that happen to spell the same string. What a frame can
// distinguish is whether the two mirrors are disabled *together*, which is what this suite asserts.

#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"           // DoRun / DoRevert / g_server
#include "gui/color_window.hpp"  // HandleEyeClick + the signal-cache test seams
#include "gui/file_io.hpp"       // BuildScene — the committed-scene leg of the PBD default
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "support/scoped_result_frame.hpp"
#include "test_gui_shared.hpp"

namespace {

const char* const kColorsWindowRef = "//" ICON_FA_PALETTE " Colors";

// Single-class raypath_color config for the real-server cases below. Recreated here rather than
// shared: the equivalent fixtures in the sibling suites live in TU-private anonymous namespaces
// and are not linkable across the gui_test target.
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

LUMICE_ErrorCode CommitJsonConfig(LUMICE_Server* server, const char* json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json, &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

bool RunToIdleWithData(LUMICE_Server* server, const char* json) {
  if (CommitJsonConfig(server, json) != LUMICE_OK) {
    return false;
  }
  for (int waited = 0; waited < 5000; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[1]{};
      lumice::test::ScopedResultFrame frame_xyz(server);
      LUMICE_FrameGetRawXyz(frame_xyz.get(), xyz, 1);
      if (xyz[0].has_valid_data) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// Open the Colors window with its geometry pinned, then point ctx at it.
//
// The pinning is not cosmetic. ImGui keys a window's position and size by title, so they survive
// across cases in one process — and another suite deliberately moves this very window off to the
// left so it occludes a crystal card, and never puts it back. Whichever case opens it next
// inherits that position and can find its leftmost row controls (the z_order arrows) clipped out
// of view. Pinning makes every lookup below independent of what ran first.
void OpenColorsWindow(ImGuiTestContext* ctx) {
  gui::g_state.color_window_open = true;
  ctx->Yield(2);
  ctx->WindowMove(kColorsWindowRef, ImVec2(50, 50));
  ctx->WindowResize(kColorsWindowRef, ImVec2(760, 520));
  ctx->Yield(2);
  ctx->SetRef(kColorsWindowRef);
}

void CloseColorsWindow(ImGuiTestContext* ctx) {
  ctx->SetRef("");
  gui::g_state.color_window_open = false;
  ctx->Yield(2);
}

// Pin the reconciler's diff base to "a finished run with nothing pending", so that the very next
// edit is the only difference it can observe and therefore the only thing that can flip
// sim_state to kModified. Callers seed the classes they want treated as already-committed BEFORE
// calling this.
void SeedCommittedBaseline(ImGuiTestContext* ctx) {
  gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
  gui::g_state.run_intent = gui::RunIntent::kLoaded;
  gui::g_state.sim_state = gui::GuiState::SimState::kDone;
  gui::g_state.committed_epoch = 5;
  gui::g_state.display_epoch_floor = 0;
  gui::g_state.dirty = false;
  ctx->Yield(4);
}

gui::ColorClassRefConfig MakeRef(int layer_idx, int crystal_pool_id, bool match_all) {
  gui::ColorClassRefConfig ref;
  ref.layer_idx = layer_idx;
  ref.crystal_pool_id = crystal_pool_id;
  ref.match_all = match_all;
  return ref;
}

// A class with one ref against the default document's only placement.
gui::ColorClassConfig MakeMatchAllClass(float r, float g, float b, int z_order) {
  gui::ColorClassConfig cls;
  cls.color[0] = r;
  cls.color[1] = g;
  cls.color[2] = b;
  cls.visible = true;
  cls.solo = false;
  cls.z_order = z_order;
  cls.match.push_back(MakeRef(0, gui::g_state.layers[0].entries[0].crystal_id, /*match_all=*/true));
  return cls;
}

// Click an item inside whichever popup is currently up, then point the ref back at the window.
//
// The two-step is needed because a wildcard path resolves its prefix with ImHashDecoratedPath,
// which does not understand the "$FOCUSED" variable — only SetRef() does. So the popup has to
// become the ref before the wildcard search, not inside it.
void ClickInFocusedPopup(ImGuiTestContext* ctx, const std::string& label) {
  ctx->SetRef("//$FOCUSED");
  ctx->ItemClick(("**/" + label).c_str());
  ctx->SetRef(kColorsWindowRef);
}

// Open a combo and pick one of its entries.
//
// Two quirks make this a hand-rolled helper. ctx->ComboClick() splits its argument at the FIRST
// '/', so it cannot address a combo that sits inside a PushID scope — which every combo in this
// window except ##ColorMode does. And `combo_ref` must be a LITERAL path, never a "**/" wildcard:
// ImGui::BeginCombo() never calls IMGUI_TEST_ENGINE_ITEM_INFO(), so a combo has no registered
// debug label and a wildcard search (which matches by label) can never find one. Same quirk as
// ColorButton, and the same fix — name the item by its id path.
void ComboPick(ImGuiTestContext* ctx, const char* combo_ref, const char* entry) {
  ctx->ItemClick(combo_ref);
  ClickInFocusedPopup(ctx, entry);
}

// The top-bar colored toggle's widget id embeds its own label, which flips with the ground-truth
// state — so the id a lookup must use depends on what the test just asserted about that state.
std::string TopBarToggleRef(bool composite_now) {
  return std::string("##TopBar/") + (composite_now ? "Colored" : "Full Spectrum") + "##CompositePreviewToggle";
}

// Drive a finite CPU run to completion. Returns false on timeout so the caller can fail at its own
// assertion line rather than inside a helper.
bool RunToDone(ImGuiTestContext* ctx, int timeout_sec = 20) {
  gui::DoRun(/*user_initiated=*/true);
  IM_CHECK_RETV(gui::g_state.run_intent == gui::RunIntent::kRunning, false);
  const auto start = std::chrono::steady_clock::now();
  while (gui::g_state.sim_state != gui::GuiState::SimState::kDone ||
         gui::g_state.run_intent != gui::RunIntent::kRunCompleted) {
    ctx->Yield();
    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() >
        timeout_sec) {
      return false;
    }
  }
  return true;
}

// Sum two channels over the server's current composite. Stops the poller first so its background
// DoSnapshot() cannot race this synchronous read — the same serialization every direct composite
// read in this repo needs.
void ReadCompositeChannelSums(int channel_a, int channel_b, unsigned long long& sum_a, unsigned long long& sum_b) {
  gui::g_server_poller.Stop();
  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_comp(gui::g_server);
  IM_CHECK_SILENT(LUMICE_FrameGetComposite(frame_comp.get(), comp, LUMICE_MAX_RENDER_RESULTS) == LUMICE_OK);
  IM_CHECK_SILENT(comp[0].img_buffer != nullptr);
  const size_t nbytes = static_cast<size_t>(comp[0].img_width) * static_cast<size_t>(comp[0].img_height) * 3;
  sum_a = 0;
  sum_b = 0;
  for (size_t i = 0; i + 2 < nbytes; i += 3) {
    sum_a += comp[0].img_buffer[i + static_cast<size_t>(channel_a)];
    sum_b += comp[0].img_buffer[i + static_cast<size_t>(channel_b)];
  }
}

// Fresh CPU server + default document, for the two cases that need a real sim to observe a
// display-time push landing on the server.
void BeginRealServerScene(float ray_num_millions) {
  gui::g_server_poller.Stop();
  gui::g_server = LUMICE_CreateServer();
  IM_CHECK_SILENT(gui::g_server != nullptr);
  // Deliberately CPU-only: the device-fused GPU path does not populate the raypath-color lanes,
  // so forcing GPU here would make every composite read below vacuous rather than wrong.
  gui::g_server_is_gpu = false;
  gui::g_state = gui::InitDefaultState();
  gui::g_state.sim.infinite = false;
  gui::g_state.sim.ray_num_millions = ray_num_millions;
  gui::g_state.sim.max_hits = 8;
}

void EndRealServerScene() {
  gui::g_server_poller.Stop();
  if (gui::g_server != nullptr) {
    LUMICE_StopServer(gui::g_server);
    LUMICE_DestroyServer(gui::g_server);
    gui::g_server = nullptr;
  }
  gui::g_server_is_gpu = false;
  gui::g_state.run_intent = gui::RunIntent::kNone;
  gui::g_state.committed_epoch = 0;
  gui::g_state.display_epoch_floor = 0;
}

}  // namespace

void RegisterColorWindowTests(ImGuiTestEngine* engine) {
  // ---------------------------------------------------------------------------------------------
  // Top-bar surface: the two controls that live in ##TopBar but belong to this window's story.
  // ---------------------------------------------------------------------------------------------

  // The arming rule is deliberately edge-triggered, and the edge is the only thing worth testing:
  // opening the window with nothing configured turns the colored preview on as a convenience, but
  // once it is open the user's own choice must survive every subsequent frame. A per-frame
  // implementation of the same convenience would pass a single-frame check and take the user's
  // choice away on the next one.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "colors_button_arms_colored_on_open_edge_only");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(gui::g_state.raypath_color.empty());
      IM_CHECK(!gui::g_state.color_window_open);

      ctx->ItemClick("##TopBar/" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.color_window_open);
      IM_CHECK(gui::g_state.show_composite_preview);  // armed on the false->true edge

      // The user turns it back off while the window stays open. Nothing may put it back.
      gui::g_state.show_composite_preview = false;
      ctx->Yield(8);
      IM_CHECK(!gui::g_state.show_composite_preview);

      ctx->ItemClick("##TopBar/" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.color_window_open);
    };
  }

  // The Colors button takes a second, styled branch once any class exists: three PushStyleColor
  // calls that must be matched by a PopStyleColor(3). Whether the predicate picks that branch is
  // settled one layer down (test_color_window_logic.cpp asserts both of its answers); what only a
  // frame can settle is that the branch still produces a working button rather than an
  // unbalanced style stack — which is why this case clicks it in the tinted state instead of
  // re-asserting the predicate. The colour values themselves are pixels, and out of reach here.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "colors_button_still_works_in_its_tinted_branch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(gui::g_state.raypath_color.empty());  // untinted branch
      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_PALETTE " Colors"));

      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));
      // Four frames is the ordinary settle used throughout this file, and it is safe here for a
      // reason the absence assertions above do not enjoy: everything below asserts PRESENCE, and
      // the engine's two-frame grace window can only ever make an item easier to find, never
      // harder. Any settle ≥ 1 frame would do; 4 just keeps it uniform with its neighbours.
      ctx->Yield(4);  // tinted branch now active

      ctx->ItemClick("##TopBar/" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.color_window_open);  // the styled button is still a live control
      ctx->ItemClick("##TopBar/" ICON_FA_PALETTE " Colors");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.color_window_open);

      // Its neighbours on the same row still resolve — an unbalanced push/pop pair does not
      // move items, but it does leave the rest of the bar drawing under a colour that was never
      // meant for it, and a stack underflow is what would take the neighbours down with it.
      IM_CHECK(ctx->ItemExists(TopBarToggleRef(/*composite_now=*/false).c_str()));
      IM_CHECK(ctx->ItemExists("##TopBar/" ICON_FA_GEAR " Settings"));
    };
  }

  // Two claims that only a drawn top bar can settle: the toggle does not exist at all until a
  // class does, and once it exists its label reports the ground truth (what was last uploaded),
  // not the preference the user asked for. Those two can differ for a frame, and a label bound to
  // the preference would tell the user the preview is colored while it is not.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "colored_toggle_appears_with_classes_and_reads_ground_truth");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Six frames, not the usual two, before asserting ABSENCE: the engine keeps an item
      // findable for two frames after it was last submitted (ImGuiTestEngine_FindItemInfo's
      // `TimestampMain + 2 >= FrameCount`), so a two-frame settle sits exactly on the boundary
      // and an earlier case's toggle is still resolvable. This does not weaken the assertion —
      // a toggle that really rendered without classes would be re-submitted every frame and
      // never age out.
      ctx->Yield(6);
      IM_CHECK(!ctx->ItemExists(TopBarToggleRef(/*composite_now=*/false).c_str()));
      IM_CHECK(!ctx->ItemExists(TopBarToggleRef(/*composite_now=*/true).c_str()));

      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));
      ctx->Yield(4);
      IM_CHECK(ctx->ItemExists(TopBarToggleRef(/*composite_now=*/false).c_str()));

      // Ground truth flips without the preference moving: the label must follow the former.
      gui::g_state.show_composite_preview = false;
      gui::g_state.last_uploaded_as_composite = true;
      ctx->Yield(4);
      IM_CHECK(ctx->ItemExists(TopBarToggleRef(/*composite_now=*/true).c_str()));
      IM_CHECK(!ctx->ItemExists(TopBarToggleRef(/*composite_now=*/false).c_str()));
    };
  }

  // The two mirrors of "the composite would be empty right now" — the top-bar toggle and the
  // window's own Enable colors checkbox — are wired to one shared predicate precisely so they
  // cannot disagree. Before that merge, each had its own trigger and a user could meet a greyed
  // checkbox beside an enabled toggle. Hiding every matched class is the branch that reaches the
  // predicate without needing a server to report zero matches.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "empty_composite_disables_both_mirrors");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      for (int i = 0; i < 2; i++) {
        gui::ColorClassConfig cls = MakeMatchAllClass(1.0f, 1.0f, 1.0f, i);
        cls.visible = false;
        gui::g_state.raypath_color.push_back(cls);
      }
      OpenColorsWindow(ctx);

      // With server == nullptr the signal cache resizes to "matched / unknown" for every class, so
      // the only thing making the composite empty here is that both matched classes are hidden.
      ctx->SetRef("");
      IM_CHECK(IsDisabled(ctx->ItemInfo(TopBarToggleRef(/*composite_now=*/false).c_str())));
      ctx->SetRef(kColorsWindowRef);
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/Enable colors")));

      // Un-hiding one class is enough to make the composite non-empty; both must release together.
      gui::g_state.raypath_color[0].visible = true;
      ctx->Yield(4);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/Enable colors")));
      ctx->SetRef("");
      IM_CHECK(!IsDisabled(ctx->ItemInfo(TopBarToggleRef(/*composite_now=*/false).c_str())));

      CloseColorsWindow(ctx);
    };
  }

  // Same read/write split as the top-bar mirror, asserted on the in-window control: the check mark
  // reports the ground truth, and the click writes the preference through the shared writer.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "enable_colors_reads_ground_truth_and_writes_preference");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));
      gui::g_state.last_uploaded_as_composite = true;
      gui::g_state.show_composite_preview = false;
      OpenColorsWindow(ctx);

      // Reading the preference instead of the ground truth would show this unchecked.
      IM_CHECK(ctx->ItemIsChecked("**/Enable colors"));

      const bool pref_before = gui::g_state.show_composite_preview;
      ctx->ItemClick("**/Enable colors");
      ctx->Yield(2);
      IM_CHECK_NE(gui::g_state.show_composite_preview, pref_before);

      CloseColorsWindow(ctx);
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Header row: composite mode, import, add, remove-all.
  // ---------------------------------------------------------------------------------------------

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "composite_mode_combo_writes_each_mode");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenColorsWindow(ctx);

      // Every mode is exercised and the failures are collected, then asserted once: a fatal
      // assert inside the loop would stop at the first bad mode and leave the rest unevaluated,
      // and "the combo writes the wrong index only for the last entry" is exactly the shape that
      // hides behind an early exit.
      struct ModeCase {
        const char* label;
        int expected;
      };
      const ModeCase kModes[] = { { "dominant", LUMICE_COLOR_MODE_DOMINANT },
                                  { "additive", LUMICE_COLOR_MODE_ADDITIVE },
                                  { "painter", LUMICE_COLOR_MODE_PAINTER } };
      std::string wrong;
      for (const auto& m : kModes) {
        ComboPick(ctx, "##ColorMode", m.label);
        ctx->Yield(2);
        if (gui::g_state.raypath_color_mode != m.expected) {
          wrong += std::string(" ") + m.label + "->" + std::to_string(gui::g_state.raypath_color_mode) + "(want " +
                   std::to_string(m.expected) + ")";
        }
      }
      IM_CHECK_STR_EQ(wrong.c_str(), "");

      CloseColorsWindow(ctx);
    };
  }

  // Import is the one header control whose availability depends on the rest of the document. With
  // no filtered placement there is nothing to import, and an enabled button would open an empty
  // popup — which reads as "the feature is broken" rather than "you have no filters".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "import_from_filter_is_disabled_without_a_filtered_entry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenColorsWindow(ctx);
      IM_CHECK(gui::g_state.filters.empty());
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/" ICON_FA_FILE_IMPORT " Import from filter")));

      gui::FilterConfig f;
      f.name = "fx";
      f.SetRaypath(gui::RaypathParams{ "3-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      ctx->Yield(4);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/" ICON_FA_FILE_IMPORT " Import from filter")));

      CloseColorsWindow(ctx);
    };
  }

  // Importing clones the filter's rows into a fresh class by value. The clone, not the reference,
  // is the point: a class that aliased the filter would silently follow later filter edits, and
  // the colouring would drift away from what the user set up without any edit to the class.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "import_from_filter_clones_rows_into_a_new_class");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.crystals[0].name = "cr";
      gui::FilterConfig f;
      f.name = "fx";
      f.SetRaypath(gui::RaypathParams{ "3-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      OpenColorsWindow(ctx);

      ctx->ItemClick("**/" ICON_FA_FILE_IMPORT " Import from filter");
      ctx->Yield(2);
      // "L1" (1-based layer) and the crystal rendered as the shared identity string
      // `#0 · cr · Prism` rather than the bare name.
      ClickInFocusedPopup(ctx, "L1 \xc2\xb7 #0 \xc2\xb7 cr \xc2\xb7 Prism \xc2\xb7 fx##imp_0");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 1);
      const auto& cls = gui::g_state.raypath_color[0];
      IM_CHECK_EQ(cls.combine, LUMICE_COLOR_COMBINE_ANY);
      IM_CHECK_EQ(static_cast<int>(cls.match.size()), 1);
      IM_CHECK_STR_EQ(cls.match[0].predicate_text.c_str(), "3-5");
      IM_CHECK(!cls.match[0].match_all);

      // By value: editing the filter afterwards must not reach the imported class.
      gui::g_state.filters[0].MutableRaypathText() = "1-2";
      ctx->Yield(2);
      IM_CHECK_STR_EQ(gui::g_state.raypath_color[0].match[0].predicate_text.c_str(), "3-5");

      CloseColorsWindow(ctx);
    };
  }

  // Add Class has to do two separable things, and only one of them is visible in the vector: seed a
  // usable class (white, visible, appended at the bottom of the priority stack), and land on the
  // structural lane so the document reads as modified. If the reconciler routed it onto the
  // display lane instead, the class would appear, the top bar would offer no Revert, and the next
  // Run would quietly commit without it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_class_seeds_a_visible_class_and_marks_modified");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      SeedCommittedBaseline(ctx);
      OpenColorsWindow(ctx);

      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[0], 1.0f);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[1], 1.0f);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[2], 1.0f);
      IM_CHECK(gui::g_state.raypath_color[0].visible);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, 0);

      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 2);
      IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, 1);  // appended at the bottom of the stack

      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // The Revert affordance is the user-visible end of that lane; its presence proves the wire
      // from a color edit through the reconciler to the top bar is live.
      ctx->SetRef("");
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));

      CloseColorsWindow(ctx);
    };
  }

  // Remove All is disabled with nothing to remove, so an idle press cannot dirty a clean document;
  // with classes present it clears them through the same structural lane as a single-row delete.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "remove_all_is_inert_when_empty_and_clears_otherwise");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      SeedCommittedBaseline(ctx);
      OpenColorsWindow(ctx);
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/" ICON_FA_TRASH " Remove All")));

      for (int i = 0; i < 3; i++) {
        ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
        ctx->Yield(2);
      }
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 3);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/" ICON_FA_TRASH " Remove All")));

      ctx->ItemClick("**/" ICON_FA_TRASH " Remove All");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 0);
      // Misrouting the clear onto the display lane would clear the three adds' kModified flag and
      // the UI would claim there is nothing to commit while the vector went from 3 to 0.
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // Still rendered while disabled, so the row does not reflow when the last class goes.
      IM_CHECK(ctx->ItemExists("**/" ICON_FA_TRASH " Remove All"));

      CloseColorsWindow(ctx);
    };
  }

  // Revert has to restore the colour state, not merely clear the dirty flag. Before ConfigSnapshot
  // carried raypath_color, Revert settled sim_state and left the added class in place — so a check
  // on sim_state alone reported green while the edit stealthily survived. Both halves are asserted
  // here for that reason.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_class_then_revert_restores_color_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::ColorClassConfig baseline = MakeMatchAllClass(0.25f, 0.5f, 0.75f, /*z_order=*/0);
      baseline.combine = 1;
      baseline.visible = false;
      gui::g_state.raypath_color.push_back(baseline);
      SeedCommittedBaseline(ctx);
      OpenColorsWindow(ctx);

      ctx->ItemClick("**/" ICON_FA_PLUS " Add Class");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      ctx->SetRef("");
      ctx->ItemClick("##TopBar/Revert");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 1);
      IM_CHECK_NE(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // Fingerprint fields: proves the snapshot restored the class by value instead of dropping in
      // a default-constructed replacement that merely has the right count.
      IM_CHECK_EQ(gui::g_state.raypath_color[0].combine, 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].visible, false);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[1], 0.5f);

      CloseColorsWindow(ctx);
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Row controls: priority, visibility, deletion.
  // ---------------------------------------------------------------------------------------------

  // z_order and the physical vector index are two different orders on purpose: the compositor
  // binds a class's lane to its vector slot, so reordering must move the scalars and leave the
  // vector alone. Rows are keyed by PushID(physical index), which is what lets this case name a
  // row by its slot and assert where it landed on screen.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "rows_render_in_z_order_while_the_vector_stays_put");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/2));
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 1.0f, 0.0f, /*z_order=*/0));
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 0.0f, 1.0f, /*z_order=*/1));
      OpenColorsWindow(ctx);

      const float y_phys0 = ctx->ItemInfo("**/$$0/" ICON_FA_XMARK "##cls_del").RectFull.Min.y;
      const float y_phys1 = ctx->ItemInfo("**/$$1/" ICON_FA_XMARK "##cls_del").RectFull.Min.y;
      const float y_phys2 = ctx->ItemInfo("**/$$2/" ICON_FA_XMARK "##cls_del").RectFull.Min.y;
      // Screen order must follow z_order (1, 2, 0), not the vector order (0, 1, 2).
      IM_CHECK(y_phys1 < y_phys2);
      IM_CHECK(y_phys2 < y_phys0);

      // And the vector itself has not been permuted: slot 0 is still the red class.
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[0], 1.0f);
      IM_CHECK_EQ(gui::g_state.raypath_color[1].color[1], 1.0f);
      IM_CHECK_EQ(gui::g_state.raypath_color[2].color[2], 1.0f);

      CloseColorsWindow(ctx);
    };
  }

  // The arrows swap with the neighbour in RANK order, which is only distinguishable from the
  // physical neighbour when the two orders disagree — so this case sets them up to disagree. The
  // ends are disabled because there is nothing to swap with; a live arrow there would swap a row
  // with itself and read as "the button does nothing".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "priority_arrows_are_capped_at_the_ends_and_swap_by_rank");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Ranks: phys1 (z=0) top, phys2 (z=1) middle, phys0 (z=2) bottom.
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/2));
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 1.0f, 0.0f, /*z_order=*/0));
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 0.0f, 1.0f, /*z_order=*/1));
      OpenColorsWindow(ctx);

      IM_CHECK(IsDisabled(ctx->ItemInfo("**/$$1/" ICON_FA_ARROW_UP "##up")));  // top row
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/$$1/" ICON_FA_ARROW_DOWN "##down")));
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/$$0/" ICON_FA_ARROW_DOWN "##down")));  // bottom row
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/$$0/" ICON_FA_ARROW_UP "##up")));

      // Raise the middle row (phys2). Its rank neighbour is phys1; its PHYSICAL neighbour is
      // phys1 as well going down and phys0 going up — so the discriminating outcome is that phys0
      // (the bottom row, physically adjacent to phys2) must not move.
      const int z_phys0_before = gui::g_state.raypath_color[0].z_order;
      ctx->ItemClick("**/$$2/" ICON_FA_ARROW_UP "##up");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.raypath_color[2].z_order, 0);
      IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, z_phys0_before);

      CloseColorsWindow(ctx);
    };
  }

  // The eye glyph changes between two icons of different advance width. The button's width is
  // pinned to the wider of the two so the controls after it hold still; without that, every
  // visibility toggle nudges the delete button sideways and a user aiming at it hits the row's
  // warning glyph instead.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "eye_button_width_is_pinned_so_the_row_does_not_jitter");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));
      OpenColorsWindow(ctx);

      const float x_visible = ctx->ItemInfo("**/$$0/" ICON_FA_XMARK "##cls_del").RectFull.Min.x;
      ctx->ItemClick("**/$$0/" ICON_FA_EYE);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.raypath_color[0].visible);  // sanity: the click landed
      const float x_hidden = ctx->ItemInfo("**/$$0/" ICON_FA_XMARK "##cls_del").RectFull.Min.x;
      IM_CHECK_EQ(x_visible, x_hidden);

      CloseColorsWindow(ctx);
    };
  }

  // Storage semantics and display semantics of visibility are deliberately different: solo hides
  // peers in the compositor without touching their `visible` flag. The eye therefore has to read
  // the same effective predicate the compositor uses, or the window says "this class is showing"
  // about a class the composite has already dropped.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "eye_icon_shows_slash_on_peers_when_another_class_solo");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      for (int i = 0; i < 2; i++) {
        gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 1.0f, 1.0f, i));
      }
      gui::HandleEyeClick(gui::g_state.raypath_color, /*phys=*/1, /*alt_down=*/true);
      IM_CHECK(gui::g_state.raypath_color[0].visible);  // storage untouched on the peer
      IM_CHECK(!gui::g_state.raypath_color[0].solo);
      IM_CHECK(gui::g_state.raypath_color[1].solo);
      OpenColorsWindow(ctx);

      // Two-sided on both rows: an implementation that ignored solo would show a plain eye on $$0.
      IM_CHECK(ctx->ItemExists("**/$$0/" ICON_FA_EYE_SLASH));
      IM_CHECK(!ctx->ItemExists("**/$$0/" ICON_FA_EYE));
      IM_CHECK(ctx->ItemExists("**/$$1/" ICON_FA_EYE));
      IM_CHECK(!ctx->ItemExists("**/$$1/" ICON_FA_EYE_SLASH));

      gui::HandleEyeClick(gui::g_state.raypath_color, /*phys=*/1, /*alt_down=*/true);  // clear solo
      ctx->Yield(4);
      IM_CHECK(ctx->ItemExists("**/$$0/" ICON_FA_EYE));
      IM_CHECK(!ctx->ItemExists("**/$$0/" ICON_FA_EYE_SLASH));

      CloseColorsWindow(ctx);
    };
  }

  // The same button carries two actions distinguished only by a modifier key, which is exactly the
  // kind of wiring that a real input event settles and a direct call to the handler cannot.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "eye_click_toggles_visible_and_alt_click_solos");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      for (int i = 0; i < 2; i++) {
        gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 1.0f, 1.0f, i));
      }
      OpenColorsWindow(ctx);

      ctx->ItemClick("**/$$0/" ICON_FA_EYE);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.raypath_color[0].visible);
      IM_CHECK(!gui::g_state.raypath_color[0].solo);  // a plain click must not solo

      ctx->KeyDown(ImGuiMod_Alt);
      ctx->ItemClick("**/$$1/" ICON_FA_EYE);
      ctx->KeyUp(ImGuiMod_Alt);
      ctx->Yield(2);
      IM_CHECK(gui::g_state.raypath_color[1].solo);
      IM_CHECK(!gui::g_state.raypath_color[0].solo);  // solo is exclusive

      ctx->KeyDown(ImGuiMod_Alt);
      ctx->ItemClick("**/$$1/" ICON_FA_EYE);
      ctx->KeyUp(ImGuiMod_Alt);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.raypath_color[1].solo);  // alt+click again restores all

      CloseColorsWindow(ctx);
    };
  }

  // Deleting a class leaves a hole in the z_order values, and the display push contract requires a
  // permutation. Compaction runs in the widget for that reason; without it the surviving classes'
  // priorities are whatever the hole left behind and the server rejects the push.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "deleting_a_class_compacts_z_order_into_a_permutation");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 1.0f, 0.0f, /*z_order=*/1));
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 0.0f, 1.0f, /*z_order=*/2));
      OpenColorsWindow(ctx);

      ctx->ItemClick("**/$$1/" ICON_FA_XMARK "##cls_del");  // delete the middle-priority class
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 2);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[0], 1.0f);  // red survived
      IM_CHECK_EQ(gui::g_state.raypath_color[1].color[2], 1.0f);  // blue survived
      // Relative order preserved, values compacted to [0, n).
      IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, 0);
      IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, 1);

      CloseColorsWindow(ctx);
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Class body: combine, ref rows, symmetry.
  // ---------------------------------------------------------------------------------------------

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "combine_combo_switches_between_any_and_all");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));
      SeedCommittedBaseline(ctx);
      OpenColorsWindow(ctx);
      ctx->ItemOpenAll(kColorsWindowRef);
      ctx->Yield(2);

      ComboPick(ctx, "$$0/##body/##combine", "all");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].combine, LUMICE_COLOR_COMBINE_ALL);
      // Membership is a structural property of the class, so the edit must reach the re-sim lane.
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      ComboPick(ctx, "$$0/##body/##combine", "any");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].combine, LUMICE_COLOR_COMBINE_ANY);

      CloseColorsWindow(ctx);
    };
  }

  // A ref names a placement, so its two combos are not independent: choosing a layer that does not
  // contain the currently-named crystal has to re-point the crystal, or the ref keeps a pool id
  // that means nothing in its new layer and matches nothing at commit time.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "layer_switch_repoints_the_crystal_into_that_layer");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Layer 1 holds a different crystal, so switching layers has to move the pool id.
      gui::g_state.crystals[0].name = "c0";
      gui::CrystalConfig second;
      second.name = "c1";
      gui::g_state.crystals.push_back(second);
      gui::Layer layer1;
      gui::EntryCard entry1;
      entry1.crystal_id = 1;
      layer1.entries.push_back(entry1);
      gui::g_state.layers.push_back(layer1);

      gui::ColorClassConfig cls;
      cls.visible = true;
      cls.match.push_back(MakeRef(0, 0, /*match_all=*/true));
      gui::g_state.raypath_color.push_back(cls);
      OpenColorsWindow(ctx);
      ctx->ItemOpenAll(kColorsWindowRef);
      ctx->Yield(2);

      // "Layer 2", not "Layer 1": the combo's labels are 1-based (DisplayLayerNumber), and the
      // layer this case means is index 1 — the one holding the OTHER crystal, which is the whole
      // point of the case. This edit is a semantic re-point, not a cosmetic relabel: before the
      // numbering was unified, "Layer 1" here addressed index 1, and after it addresses index 0.
      ComboPick(ctx, "$$0/##body/$$0/##layer", "Layer 2");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].match[0].layer_idx, 1);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].match[0].crystal_pool_id, 1);

      CloseColorsWindow(ctx);
    };
  }

  // Checking "whole" freezes the predicate field rather than hiding it, and keeps the text. Both
  // halves matter: a field that vanishes reflows the row, and text that is dropped cannot be got
  // back by un-checking — the user has to retype what they had.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "whole_freezes_the_predicate_without_dropping_its_text");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::ColorClassConfig cls;
      cls.visible = true;
      auto ref = MakeRef(0, gui::g_state.layers[0].entries[0].crystal_id, /*match_all=*/false);
      ref.predicate_text = "3-5";
      cls.match.push_back(ref);
      gui::g_state.raypath_color.push_back(cls);
      SeedCommittedBaseline(ctx);
      OpenColorsWindow(ctx);
      ctx->ItemOpenAll(kColorsWindowRef);
      ctx->Yield(2);

      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##pred")));
      ctx->ItemClick("**/whole");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].match_all);
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/##pred")));  // frozen, still present
      IM_CHECK_STR_EQ(gui::g_state.raypath_color[0].match[0].predicate_text.c_str(), "3-5");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));

      ctx->ItemClick("**/whole");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.raypath_color[0].match[0].match_all);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##pred")));
      IM_CHECK_STR_EQ(gui::g_state.raypath_color[0].match[0].predicate_text.c_str(), "3-5");

      CloseColorsWindow(ctx);
    };
  }

  // The three symmetry bits are written by one shared widget, so a single click already proves the
  // wiring. What the other two clicks prove is different and not implied by the first: each bit
  // has to participate in the class's equality operator, or the reconciler's diff never observes
  // the second and third click as a change at all and the edit is silently lost at commit.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "each_symmetry_bit_is_independently_observed");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::ColorClassConfig cls;
      cls.visible = true;
      auto ref = MakeRef(0, gui::g_state.layers[0].entries[0].crystal_id, /*match_all=*/false);
      ref.predicate_text = "3-5";  // a valid single atom, so `dirty` below reflects only our clicks
      cls.match.push_back(ref);
      gui::g_state.raypath_color.push_back(cls);
      SeedCommittedBaseline(ctx);
      OpenColorsWindow(ctx);
      ctx->ItemOpenAll(kColorsWindowRef);
      ctx->Yield(2);

      struct SymCase {
        const char* item;
        bool gui::ColorClassRefConfig::*field;
      };
      const SymCase kBits[] = { { "**/P##color_ref", &gui::ColorClassRefConfig::sym_p },
                                { "**/B##color_ref", &gui::ColorClassRefConfig::sym_b },
                                { "**/D##color_ref", &gui::ColorClassRefConfig::sym_d } };
      // Collected, then asserted once after the loop: stopping at the first bad bit would leave
      // the other two unevaluated, and "one of the three is missing from operator==" is precisely
      // the per-bit defect this case exists to separate.
      std::string not_set;
      std::string not_observed;
      for (const auto& bit : kBits) {
        gui::g_state.dirty = false;
        ctx->ItemClick(bit.item);
        ctx->Yield(2);
        if (!(gui::g_state.raypath_color[0].match[0].*bit.field)) {
          not_set += std::string(" ") + bit.item;
        }
        if (!gui::g_state.dirty) {
          not_observed += std::string(" ") + bit.item;
        }
      }
      IM_CHECK_STR_EQ(not_set.c_str(), "");
      IM_CHECK_STR_EQ(not_observed.c_str(), "");
      // Per-field, not a shared tri-state: the earlier bits survive the later clicks.
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_p);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_b);
      IM_CHECK(gui::g_state.raypath_color[0].match[0].sym_d);

      CloseColorsWindow(ctx);
    };
  }

  // A GUI-created ref defaults to P|B|D, and the default has to be applied at the call site rather
  // than in the struct — deserialization of a legacy config without symmetry keys must stay
  // un-symmetric. The committed-scene leg is what makes this more than a field check: it proves
  // the default survives the whole BuildScene path rather than being dropped on the way out.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "add_ref_defaults_to_pbd_and_reaches_the_scene");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::ColorClassConfig cls;
      cls.color[0] = 1.0f;
      cls.visible = true;
      gui::g_state.raypath_color.push_back(cls);  // no refs yet, so Add Ref has a target
      SeedCommittedBaseline(ctx);
      OpenColorsWindow(ctx);
      ctx->ItemOpenAll(kColorsWindowRef);
      ctx->Yield(2);

      ctx->ItemClick("**/" ICON_FA_PLUS " Add Ref");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color[0].match.size()), 1);
      const auto& added = gui::g_state.raypath_color[0].match[0];
      IM_CHECK_EQ(added.layer_idx, 0);
      IM_CHECK(added.match_all);
      IM_CHECK(added.sym_p);
      IM_CHECK(added.sym_b);
      IM_CHECK(added.sym_d);
      // The struct default stays false; only this call site opts in.
      IM_CHECK(!gui::ColorClassRefConfig{}.sym_p);

      gui::ScenePtr scene = gui::BuildScene(gui::g_state, gui::SceneIntent::kSimCommit);
      IM_CHECK(scene != nullptr);
      size_t json_len = 0;
      IM_CHECK_EQ(LUMICE_SceneToJson(scene.get(), nullptr, 0, &json_len), LUMICE_OK);
      std::string scene_buf(json_len + 1, '\0');
      IM_CHECK_EQ(LUMICE_SceneToJson(scene.get(), scene_buf.data(), scene_buf.size(), nullptr), LUMICE_OK);
      scene_buf.resize(json_len);
      const auto scene_j = nlohmann::json::parse(scene_buf);
      IM_CHECK_STR_EQ(scene_j["raypath_color"]["classes"][0]["match"][0]["symmetry"].get<std::string>().c_str(), "PBD");

      CloseColorsWindow(ctx);
    };
  }

  // Deleting one ref must take that ref and no other. The rows are drawn in a loop with a deferred
  // erase, which is the shape that most easily deletes by the wrong index.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "deleting_a_ref_removes_only_that_row");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      gui::ColorClassConfig cls;
      cls.visible = true;
      for (const char* text : { "3-5", "1-2", "4-6" }) {
        auto ref = MakeRef(0, gui::g_state.layers[0].entries[0].crystal_id, /*match_all=*/false);
        ref.predicate_text = text;
        cls.match.push_back(ref);
      }
      gui::g_state.raypath_color.push_back(cls);
      OpenColorsWindow(ctx);
      ctx->ItemOpenAll(kColorsWindowRef);
      ctx->Yield(2);

      // Full literal path: class PushID(phys) / the "##body" tree scope / ref PushID(idx).
      ctx->ItemClick("$$0/##body/$$1/" ICON_FA_XMARK "##ref_del");  // the middle ref
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.raypath_color[0].match.size()), 2);
      IM_CHECK_STR_EQ(gui::g_state.raypath_color[0].match[0].predicate_text.c_str(), "3-5");
      IM_CHECK_STR_EQ(gui::g_state.raypath_color[0].match[1].predicate_text.c_str(), "4-6");

      CloseColorsWindow(ctx);
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Signal cache. These four need the ImGui clock (the throttle reads ImGui::GetTime()) but not a
  // drawn control, which is why they yield a frame and then work through the exported seams
  // instead of driving widgets. They stay in this target because gui_unit_test has no ImGui
  // context to read that clock from.
  // ---------------------------------------------------------------------------------------------

  // The 500 ms poll can land inside the window between a GUI-side push_back and the server picking
  // up the committed class table, and the server rejects the mismatched count. Treating that
  // rejection as "we know the answer and it is zero" is what once made every pre-existing class
  // flash a no-match warning the instant a new class was added.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "color_window", "poll_signal_preserves_on_real_server_class_count_mismatch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield();
      LUMICE_Server* server = LUMICE_CreateServer();
      IM_CHECK(server != nullptr);
      const bool ok = RunToIdleWithData(server, kSingleClassConfig);
      IM_CHECK(ok);
      if (!ok) {
        LUMICE_DestroyServer(server);
        return;
      }

      // Local state grows to two classes while the server still knows about one.
      gui::g_state.raypath_color.push_back(gui::ColorClassConfig{});
      gui::g_state.raypath_color.push_back(gui::ColorClassConfig{});

      // Sentinels distinguishable from both a real signal (0/1) and the resize default (1).
      std::vector<int> flags = { 7, 7 };
      gui::PollColorClassSignal(gui::g_state, server, flags);
      IM_CHECK_EQ(static_cast<int>(flags.size()), 2);
      IM_CHECK_EQ(flags[0], 7);
      IM_CHECK_EQ(flags[1], 7);

      LUMICE_DestroyServer(server);
    };
  }

  // The cached flags describe one (server, committed_epoch) domain. A commit bumps the epoch, and
  // serving the previous generation's flags for up to one throttle interval reads to the user as a
  // pip still claiming "matched" after the thing it described stopped existing.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "refresh_signals_invalidates_on_committed_epoch_bump");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield();
      gui::g_state.raypath_color.push_back(gui::ColorClassConfig{});
      gui::g_state.committed_epoch = 0;

      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      uint64_t epoch = 0;
      size_t flags_size = 0;
      float t1 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(nullptr, &epoch, &flags_size, &t1);
      IM_CHECK_EQ(static_cast<int>(epoch), 0);
      IM_CHECK_EQ(static_cast<int>(flags_size), 1);
      IM_CHECK(t1 > -1000.0f);  // the initial poll fired

      // Well inside the throttle window: without invalidation this call is a no-op.
      gui::g_state.committed_epoch = 5;
      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      float t2 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(nullptr, &epoch, nullptr, &t2);
      IM_CHECK_EQ(static_cast<int>(epoch), 5);
      IM_CHECK(t2 >= t1);
    };
  }

  // The other half of the same domain key: a backend swap destroys the server and mints a new one,
  // and the old pointer's flags describe a server that no longer exists.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "refresh_signals_invalidates_on_server_pointer_change");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield();
      gui::g_state.raypath_color.push_back(gui::ColorClassConfig{});

      // Two real servers: the refresh eventually calls into the C API with whichever pointer it
      // holds, so a fabricated one would be undefined behaviour rather than a test.
      LUMICE_Server* srv_a = LUMICE_CreateServer();
      LUMICE_Server* srv_b = LUMICE_CreateServer();
      IM_CHECK(srv_a != nullptr);
      IM_CHECK(srv_b != nullptr);
      IM_CHECK(srv_a != srv_b);

      gui::RefreshColorClassSignals(gui::g_state, srv_a);
      LUMICE_Server* cached = nullptr;
      gui::GetColorClassSignalCacheKeysForTest(&cached, nullptr, nullptr, nullptr);
      IM_CHECK_EQ(cached, srv_a);

      gui::RefreshColorClassSignals(gui::g_state, srv_b);
      gui::GetColorClassSignalCacheKeysForTest(&cached, nullptr, nullptr, nullptr);
      IM_CHECK_EQ(cached, srv_b);

      LUMICE_DestroyServer(srv_a);
      LUMICE_DestroyServer(srv_b);
    };
  }

  // The complement of the two above, and the reason they cannot simply poll every call: with the
  // domain unchanged the throttle must hold, or the debounce contract the C API documents is gone
  // and an O(W*H*classes*consumers) query runs every frame.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "refresh_signals_steady_state_honors_throttle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield();
      gui::g_state.raypath_color.push_back(gui::ColorClassConfig{});
      gui::g_state.committed_epoch = 42;

      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      float t1 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(nullptr, nullptr, nullptr, &t1);

      gui::RefreshColorClassSignals(gui::g_state, nullptr);
      float t2 = 0.0f;
      gui::GetColorClassSignalCacheKeysForTest(nullptr, nullptr, nullptr, &t2);
      IM_CHECK_EQ(t2, t1);
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Real-timing cases. These three drive a live simulation and wait on wall-clock batch
  // accumulation, which the correctness pool's fixed frame dt starves. scripts/build.sh routes
  // them to the real-timing pool BY NAME, and scripts/regen_gui_test_refs.py carries the same
  // filter string (check_policies.py's gui-test-suite-args-sync keeps the two in step) — so
  // renaming any of the three means updating both sides.
  // ---------------------------------------------------------------------------------------------

  // Priority is a display-time property, and re-running must not quietly restore whatever priority
  // was committed. Two fully-overlapping match-all classes in painter mode make the winner depend
  // on priority alone: the lower z_order paints over the other regardless of brightness.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "zorder_priority_persists_across_rerun");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      BeginRealServerScene(/*ray_num_millions=*/0.5f);
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));  // red on top
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 0.0f, 1.0f, /*z_order=*/1));  // blue below
      gui::g_state.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;

      IM_CHECK(RunToDone(ctx));
      unsigned long long r = 0;
      unsigned long long b = 0;
      ReadCompositeChannelSums(0, 2, r, b);
      IM_CHECK(r + b > 0u);  // sanity: something landed
      IM_CHECK(r > b);

      // Drag-reorder equivalent: promote blue. Display-time only, no re-sim.
      gui::g_state.raypath_color[0].z_order = 1;
      gui::g_state.raypath_color[1].z_order = 0;
      ctx->Yield(3);
      ReadCompositeChannelSums(0, 2, r, b);
      IM_CHECK(b > r);  // sanity: the swap reached the server before the re-run

      // Re-run with no further colour edit. A stale effects baseline would let the commit fall
      // back to the committed order and silently undo the user's reordering.
      IM_CHECK(RunToDone(ctx));
      ReadCompositeChannelSums(0, 2, r, b);
      IM_CHECK(b > r);

      EndRealServerScene();
    };
  }

  // Revert restores GuiState, but the colours the user sees live on the server. Restoring only the
  // former leaves the preview showing the edit the user just reverted.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "revert_repushes_server_display_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      BeginRealServerScene(/*ray_num_millions=*/0.5f);
      gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));
      gui::g_state.raypath_color_mode = LUMICE_COLOR_MODE_DOMINANT;

      IM_CHECK(RunToDone(ctx));
      unsigned long long r = 0;
      unsigned long long g = 0;
      ReadCompositeChannelSums(0, 1, r, g);
      IM_CHECK(r > g);  // committed baseline: red

      gui::g_state.raypath_color[0].color[0] = 0.0f;  // display-time edit: red -> green
      gui::g_state.raypath_color[0].color[1] = 1.0f;
      ctx->Yield(3);
      ReadCompositeChannelSums(0, 1, r, g);
      IM_CHECK(g > r);  // sanity: the edit reached the server before the revert

      gui::DoRevert();
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[0], 1.0f);
      IM_CHECK_EQ(gui::g_state.raypath_color[0].color[1], 0.0f);
      ctx->Yield(3);
      ReadCompositeChannelSums(0, 1, r, g);
      IM_CHECK(r > g);  // and the server followed it back

      EndRealServerScene();
    };
  }

  // The device colour-class cap is enforced where the GPU backend actually runs, not at commit —
  // so the warning cannot come from DoRun and has to be observed by polling. The second half is
  // the reset: a later clean run must clear the tally, or the warning follows the user into a
  // configuration that never had the problem.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "color_window", "gpu_color_class_overflow_surfaces_async_warning");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::ClearGuiWarning();
      gui::g_server = LUMICE_CreateServer();
      IM_CHECK(gui::g_server != nullptr);
      gui::g_state.use_gpu_backend = true;  // the three colour-degrade caps are GPU-only

      // 20 classes against a device cap of 16: the excess four are dropped at BeginSession. Each
      // carries a whole-crystal ref so it is a real class that reaches the core class table.
      constexpr int kNumClasses = 20;
      for (int c = 0; c < kNumClasses; ++c) {
        gui::ColorClassConfig cls =
            MakeMatchAllClass((c % 3 == 0) ? 1.0f : 0.0f, (c % 3 == 1) ? 1.0f : 0.0f, (c % 3 == 2) ? 1.0f : 0.0f, c);
        cls.combine = 0;
        gui::g_state.raypath_color.push_back(cls);
      }
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = 0.02f;

      const unsigned long long baseline_uploads = gui::g_state.texture_upload_count;
      gui::DoRun(/*user_initiated=*/true);
      IM_CHECK(WaitForSimRestartAtLeast(ctx, baseline_uploads, /*timeout_ms=*/8000));
      for (int i = 0; i < 20 && gui::PeekGuiWarning().empty(); ++i) {
        ctx->Yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      const std::string warning = gui::PeekGuiWarning();
      IM_CHECK(!warning.empty());
      IM_CHECK(warning.find("color") != std::string::npos || warning.find("Color") != std::string::npos);
      IM_CHECK(warning.find("degraded") != std::string::npos);
      IM_CHECK(warning.find("color class") != std::string::npos);

      // Exact count, and via the async path: the component tally stays 0, so this is not a
      // predicate drop wearing the same message.
      LUMICE_ColorOverflowInfo color_over{};
      IM_CHECK(LUMICE_GetColorOverflowInfo(gui::g_server, &color_over) == LUMICE_OK);
      IM_CHECK_EQ(color_over.color_class_overflow_count, 4);
      IM_CHECK_EQ(color_over.component_overflow_count, 0);

      // Dismiss the modal this warning opened. The harness renders the warning popup (it mirrors
      // the product's frame loop), and ClearGuiWarning only re-arms the trigger — it does not close
      // a popup that is already on screen, so an undismissed one would block input for whichever
      // case runs next.
      ctx->PopupCloseAll();
      gui::ClearGuiWarning();
      gui::g_state.raypath_color.clear();
      const unsigned long long baseline_uploads2 = gui::g_state.texture_upload_count;
      gui::DoRun(/*user_initiated=*/true);
      IM_CHECK(WaitForSimRestartAtLeast(ctx, baseline_uploads2, /*timeout_ms=*/8000));
      for (int i = 0; i < 20; ++i) {
        ctx->Yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      LUMICE_ColorOverflowInfo color_over2{};
      IM_CHECK(LUMICE_GetColorOverflowInfo(gui::g_server, &color_over2) == LUMICE_OK);
      IM_CHECK_EQ(color_over2.color_class_overflow_count, 0);
      IM_CHECK(gui::PeekGuiWarning().empty());

      gui::ClearGuiWarning();
      gui::g_state.raypath_color.clear();
      gui::g_state.use_gpu_backend = false;
      gui::g_state.dirty = false;
      EndRealServerScene();
    };
  }
}
