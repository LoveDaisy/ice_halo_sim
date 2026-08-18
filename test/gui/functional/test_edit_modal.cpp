// The Edit Entry modal — the one window where a crystal, its orientation and its filter are edited.
//
// What this suite is for. `edit_modals.cpp` draws a container that exists in two forms (a staged
// `BeginPopupModal` whose OK is the only commit, and an Immediate `Begin` that writes every frame),
// a persistent crystal preview beside a three-tab body, and a shape table whose rows carry a
// randomization checkbox and a sync-group swatch. Almost nothing here can be settled without a real
// frame: whether a tab kept its selection, whether the window actually went away, whether a value
// that was propagated through a sync group survived the next frame's clamp, and whether a column is
// wide enough for the text ImGui silently ellipsizes. Those are the questions below.
//
// Deliberately NOT here, with where each lives instead. Whether a filter's row text parses, what a
// blank row lowers to, and when OK is gated are functional/test_filter_editor.cpp; the axis-preset
// classifier and the factory-vs-user zenith it resolves to are
// unit-correctness/gui/test_axis_presets.cpp; whether an edited document survives a save and reload
// is composition-correctness/gui/test_document_roundtrip_chain.cpp; the page's committed pixel
// layout is visual/test_gui_crystal_inspector_layout.cpp, whose four reference images this file
// must not disturb. Nothing below restates any of them.
//
// One thing this file asserts that reads like core's job: the sync-group leader. It is here because
// the GUI carries its own mirror of core's rule (it must, to show the user the value a join will
// hand out before any commit happens), the two have drifted apart once already, and the only place
// the drift is observable is the moment a row joins a group in this table.
//
// What a user sees when these break: an OK that silently discards what they typed, a Cancel that
// does not, a preview showing the previous crystal's pose, a grouped row that snaps back to a
// different number one frame after they set it, or a column of parameter names reading "Prism…".
//
// One convention, applied without exception below rather than case by case: every case that opens
// the modal declares a `ScopedPopups` (test_gui_shared.hpp) right after its ResetTestState().
// The rule is "opens the modal", not "opens the modal and has a fatal check before closing it" —
// the narrower reading would make a reader work out, per case, why this one has a guard and that
// one does not, and would silently become wrong the day somebody adds an assertion. The guard is
// what closes the modal when a case leaves early; the ItemClick(OK/Cancel/Close) still written at
// the end of most bodies is the pass path, and stays because it drives the real control.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/crystal_preview.hpp"  // BuildCrystalMeshData — core's own answer to "which member led"
#include "gui/dock_layout.hpp"      // kDocumentInspectorWindowName — the page's host window
#include "gui/file_io.hpp"          // DeserializeFromJson / BuildExportJsonOrWarn
#include "gui/panels.hpp"
// imgui_internal.h is normally an anti-pattern. Three claims here have no public reading: window
// z-order (ImGuiContext::Windows), whether the staged form is a real modal
// (GetTopMostPopupModal), and whether a table column's text fits (ImGuiTableColumn's layout
// bookkeeping). The relied-on semantics are documented in the convention block at the top of
// src/gui/app_panels.cpp; an ImGui upgrade that changes either must update both.
#include "imgui_internal.h"
#include "support/scene_json_helpers.hpp"  // CommitSceneJson / PrismFacePlaneOffsets / CountDistinct
#include "test_gui_shared.hpp"

using lumice::test::CommitSceneJson;
using lumice::test::CountDistinct;
using lumice::test::PrismFacePlaneOffsets;

namespace {

const char* const kHeightInput = "**/##Height##modal_cr";


gui::CrystalConfig& EntryCrystal() {
  return gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
}

// Whether a tab's rendered label carries the trailing " *". Read off the label ImGui actually
// submitted rather than off the buffer-vs-snapshot comparison the marker is derived from — the
// point of the marker is that the user can see it.
bool TabIsDirty(ImGuiTestContext* ctx, const char* tab_ref) {
  const ImGuiTestItemInfo info = InspectorItemInfo(ctx, tab_ref);
  return info.ID != 0 && std::strstr(info.DebugLabel, "*") != nullptr;
}

// One logical filter-presence edit from a "finite rays done" baseline, returning the
// effect-observable triple. A free function rather than a capturing lambda because
// ImGuiTest::TestFunc is a raw function pointer typedef.
//
// This used to take an `immediate` flag and the two cases below compared the staged path's triple
// against the immediate path's. There is now one path, so that comparison would be a tautology —
// the two calls would run the same code and agree with themselves. What is asserted instead is the
// effect itself, in absolute terms: adding or removing a filter is a STRUCTURAL change, so it must
// mark the document dirty and drop the accumulated display rather than leaving a picture on screen
// that no longer corresponds to the scene.
struct CommitOutcome {
  bool dirty;
  gui::GuiState::SimState sim_state;
  unsigned long long display_epoch_floor;
};

CommitOutcome RunFilterPresenceToggle(ImGuiTestContext* ctx, bool start_with_filter) {
  ResetTestState();
  const ScopedPopups popup_guard(ctx);
  if (start_with_filter) {
    gui::FilterConfig f;
    f.SetRaypath(gui::RaypathParams{ "3-1-5" });
    gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
  }
  gui::g_state.run_intent = gui::RunIntent::kLoaded;
  gui::g_state.sim_state = gui::GuiState::SimState::kDone;
  gui::g_state.snapshot_intensity = 0.5f;
  gui::g_state.committed_epoch = 5;
  gui::g_state.display_epoch_floor = 0;
  gui::g_state.dirty = false;
  gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
  ctx->Yield(2);

  OpenFilterTab(ctx);
  ctx->Yield(4);
  if (start_with_filter) {
    // Scrolled to first: the shared filter controls sit below the token hint, which wraps to a
    // second line whenever the column is narrow enough — and a clipped item is invisible to
    // ItemClick (see InspectorItemInfo's declaration). This helper predates InspectorItemClick
    // below, so it reaches for the same scroll directly rather than through it.
    ScrollInspectorTo(ctx, "**/Remove Filter##filter");
    ctx->ItemClick("**/Remove Filter##filter");
  } else {
    ctx->ItemInputValue("**/##row_text_0", "3-1-5");
  }
  ctx->Yield(4);

  return CommitOutcome{ gui::g_state.dirty, gui::g_state.sim_state, gui::g_state.display_epoch_floor };
}

// Act on an inspector control after bringing it on screen.
//
// The crystal page is taller than the document column, so its lower half — the whole Face Distance
// section — starts clipped. A clipped item is invisible to ItemClick exactly as it is to ItemExists
// (the reason is spelled out at InspectorItemInfo's declaration in test_gui_shared.hpp: ImGui never
// submits a culled table row, so the engine is never told the widget's name). The scroll is
// therefore part of the action rather than a nicety, and these two wrappers are where it lives so
// that no case has to remember. ScrollInspectorTo returns immediately when the item is already on
// screen, so the rows above the fold pay nothing; a failure to find it is left to the ItemClick /
// ItemInputValue below to report, which keeps the error message the one a reader expects.
void InspectorItemClick(ImGuiTestContext* ctx, const char* ref) {
  ScrollInspectorTo(ctx, ref);
  ctx->ItemClick(ref);
}

void InspectorItemInputValue(ImGuiTestContext* ctx, const char* ref, float value) {
  ScrollInspectorTo(ctx, ref);
  ctx->ItemInputValue(ref, value);
}

// The id of one axis row's distribution combo.
//
// It cannot be reached the way every other widget in this file is. ImGui::Combo submits its preview
// button through BeginCombo, which never calls the test engine's item-info hook, so the engine's
// registry has no entry for it and a label wildcard can never match. What the registry DOES have is
// the Mean slider on the same row, and RenderAxisDist wraps the whole row in one PushID(label) with
// no further scope of its own — so that slider's ParentID is exactly the id-stack top the combo was
// submitted under, and hashing "##dist" against it reproduces the combo's id.
//
// Derived rather than spelled out because the alternative is a literal path through the tab body's
// BeginChild, whose name ImGui generates; that would pin this file to ImGui's child-naming scheme.
ImGuiID AxisDistComboId(ImGuiTestContext* ctx, const char* row_label) {
  const ImGuiTestItemInfo mean = InspectorItemInfo(ctx, (std::string("**/") + row_label + "/##Mean").c_str());
  IM_CHECK_RETV(mean.ID != 0, 0);
  return ImHashStr("##dist", 0, mean.ParentID);
}

// The topmost window that is not a BeginChild pane. The modal opens two child panes of its own
// (##modal_left_pane / ##modal_right_pane) which sit after their parent in g.Windows, so a bare
// g.Windows.back() would name a child rather than the window whose z-order is under test.
ImGuiWindow* TopmostRootWindow() {
  ImGuiContext* g = ImGui::GetCurrentContext();
  for (int i = g->Windows.Size - 1; i >= 0; --i) {
    if ((g->Windows[i]->Flags & ImGuiWindowFlags_ChildWindow) == 0) {
      return g->Windows[i];
    }
  }
  return nullptr;
}

}  // namespace

void RegisterEditModalTests(ImGuiTestEngine* engine) {
  // ===================================================================================
  // The container: what kind of window each mode opens, and how it closes.
  // ===================================================================================

  // ===================================================================================
  // Commit semantics: what the per-frame commit writes.
  // ===================================================================================

  // Attaching or clearing a filter is a STRUCTURAL edit: the rays already accumulated were traced
  // against a different scene, so the document must go dirty and the picture on screen must be
  // dropped rather than left standing as a result of a scene that no longer exists. Add and remove
  // are kept apart because they enter ApplyBuffersToEntry through different branches (creation vs
  // clearing).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "adding_a_filter_marks_the_document_and_drops_the_display");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const CommitOutcome out = RunFilterPresenceToggle(ctx, /*start_with_filter=*/false);
      IM_CHECK(out.dirty);
      IM_CHECK_EQ(static_cast<int>(out.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      // The baseline set display_epoch_floor to 0 against a committed_epoch of 5, so a floor that
      // moved is the accumulated display being discarded rather than shown on.
      IM_CHECK_GT(out.display_epoch_floor, 0u);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "removing_a_filter_marks_the_document_and_drops_the_display");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const CommitOutcome out = RunFilterPresenceToggle(ctx, /*start_with_filter=*/true);
      IM_CHECK(out.dirty);
      IM_CHECK_EQ(static_cast<int>(out.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK_GT(out.display_epoch_floor, 0u);
    };
  }

  // P108. An Immediate edit lands on the entry the same frame — no OK involved.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "an_immediate_edit_reaches_the_entry_at_once");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      const float orig_h = EntryCrystal().height.center;

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemInputValue(kHeightInput, orig_h + 2.5f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().height, orig_h + 2.5f);

      ctx->Yield(2);
    };
  }

  // The distinction Immediate mode exists for: a crystal tweak keeps accumulating onto the picture
  // already on screen, while a filter change cannot (it changes which rays exist at all). One case
  // rather than two because the whole claim is that the two edits differ — a build that cleared on
  // both, or on neither, has to fail somewhere in here.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "edit_modal", "an_immediate_crystal_edit_keeps_the_display_a_filter_edit_clears_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      ctx->Yield();
      const float orig_h = EntryCrystal().height.center;

      // No mode to switch into: the page commits every frame, which is what this case was checking
      // the Immediate checkbox to obtain.
      OpenCrystalTab(ctx);
      ctx->Yield(4);

      ctx->ItemInputValue(kHeightInput, orig_h + 1.0f);
      ctx->Yield(2);
      IM_CHECK(gui::g_state.dirty);
      IM_CHECK_GT(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, 0u);

      // The entry had no filter at open, so the first raypath edit IS the filter creation.
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "3-1-5");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, gui::g_state.committed_epoch);

      ctx->Yield(2);
    };
  }

  // The model-layer invariant that survives the missing OK gate: FilterConfig never holds a
  // non-valid raypath. Staged mode enforces it by disabling OK (functional/test_filter_editor.cpp);
  // Immediate mode has no gate to disable, so ApplyBuffersToEntry has to refuse the write itself.
  //
  // Structured to avoid a vacuous pass: a valid baseline is committed first (so "unchanged" means
  // something), each rejected value is followed by asserting the LAST GOOD text is still there
  // (not merely that some filter exists), and a valid value afterwards proves the guard did not
  // wedge the entry permanently.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "edit_modal", "an_invalid_raypath_never_reaches_the_entry_in_immediate_mode");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemClick("**/###filter_tab");
      ctx->Yield(4);

      auto raypath_text = []() {
        const auto& id = gui::g_state.layers[0].entries[0].filter_id;
        return id.has_value() ? gui::g_state.filters[*id].RaypathText() : std::string("<none>");
      };

      ctx->ItemInputValue("**/##row_text_0", "3-1");
      ctx->Yield(2);
      IM_CHECK_STR_EQ(raypath_text().c_str(), "3-1");

      // "abc" is syntactically invalid; "3-" is incomplete. ValidateSummandText separates them, and
      // the commit guard treats both the same way (`state != kValid`), so both are driven.
      for (const char* rejected : { "abc", "3-" }) {
        ctx->ItemInputValue("**/##row_text_0", rejected);
        ctx->Yield(2);
        if (raypath_text() != "3-1") {
          IM_ERRORF("typing \"%s\" changed the committed raypath to \"%s\"", rejected, raypath_text().c_str());
        }

        if (ctx->IsError()) {
          break;
        }
      }

      ctx->ItemInputValue("**/##row_text_0", "3-1-5");
      ctx->Yield(2);
      IM_CHECK_STR_EQ(raypath_text().c_str(), "3-1-5");

      // Empty is not "invalid" — it is the documented way to say "no filter".
      ctx->ItemInputValue("**/##row_text_0", "");
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->Yield(2);
    };
  }

  // ===================================================================================
  // The tab bar and the persistent preview pane.
  // ===================================================================================

  // P87 / P106. The preview lives in a persistent left pane OUTSIDE the tab body, which is the
  // whole reason it can hold a pose across a tab switch. Both halves of that are asserted: the pane
  // is reachable from every tab, and a rotation applied on it survives switching away and back.
  // A tab body that had swallowed the preview would fail the first; a pane rebuilt on tab change
  // would fail the second.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "the_preview_pane_is_one_widget_shared_by_every_tab");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      IM_CHECK(InspectorItemExists(ctx, "**/##modal_preview_interact"));
      // The right pane resolved to a non-zero width, or the tab bar would not have been submitted.
      IM_CHECK(InspectorItemExists(ctx, "**/###crystal_tab"));

      float before[16];
      std::memcpy(before, gui::g_crystal_rotation, sizeof before);
      ctx->ItemDragWithDelta("**/##modal_preview_interact", ImVec2(60.0f, 0.0f));
      ctx->Yield(2);
      IM_CHECK(std::memcmp(before, gui::g_crystal_rotation, sizeof before) != 0);

      float dragged[16];
      std::memcpy(dragged, gui::g_crystal_rotation, sizeof dragged);
      for (const char* tab : { "**/###axis_tab", "**/###filter_tab", "**/###crystal_tab" }) {
        ctx->ItemClick(tab);
        ctx->Yield(2);
        if (!InspectorItemExists(ctx, "**/##modal_preview_interact")) {
          IM_ERRORF("the preview pane is not reachable from %s", tab);
        }

        if (ctx->IsError()) {
          break;
        }
      }
      IM_CHECK(std::memcmp(dragged, gui::g_crystal_rotation, sizeof dragged) == 0);

      ctx->Yield(2);
    };
  }

  // ===================================================================================
  // The crystal preview's pose, and the three things allowed to reset it.
  // ===================================================================================

  // P85 / P88. Opening a DIFFERENT crystal snaps the preview to that crystal's own default pose;
  // opening the SAME one again keeps whatever the user dragged. The judge is crystal_id rather than
  // (layer, entry), which is what lets two entries linked to one pool slot share a viewing history.
  //
  // Driven through the selection directly rather than by clicking a row, so the assertion isolates
  // the reset logic
  // from the card router's indirection — and the card-to-card switch WITHOUT an intervening
  // Cancel/OK is the exact path the bug reproduced on.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "edit_modal", "opening_another_crystal_snaps_the_preview_to_its_default_pose");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      const gui::AxisDist az_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      gui::CrystalConfig column;
      column.zenith = gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f };
      column.azimuth = az_full;
      column.roll = roll_full;
      IM_CHECK_EQ(static_cast<int>(gui::ClassifyAxisPreset(column.zenith, column.azimuth, column.roll)),
                  static_cast<int>(gui::AxisPreset::kColumn));

      gui::EntryCard e_column;
      e_column.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(column);
      gui::g_state.layers[0].entries.push_back(e_column);
      ctx->Yield(2);

      const auto& cr0 = EntryCrystal();
      gui::AxisDist params0[3] = { cr0.zenith, cr0.azimuth, cr0.roll };
      float expected0[16] = { 0 };
      gui::DefaultPreviewRotation(gui::ClassifyAxisPreset(cr0.zenith, cr0.azimuth, cr0.roll), params0, expected0);
      gui::AxisDist params1[3] = { column.zenith, column.azimuth, column.roll };
      float expected1[16] = { 0 };
      gui::DefaultPreviewRotation(gui::AxisPreset::kColumn, params1, expected1);

      auto expect_pose = [](const char* where, const float* expected) {
        for (int i = 0; i < 16; ++i) {
          if (gui::g_crystal_rotation[i] != expected[i]) {
            IM_ERRORF("%s: index %d actual=%f expected=%f", where, i, static_cast<double>(gui::g_crystal_rotation[i]),
                      static_cast<double>(expected[i]));
          }
        }
      };

      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(3);
      expect_pose("entry 0 on first selection", expected0);

      if (ctx->IsError()) {
        return;
      }

      // Drag, so that entry 1 opening on ITS default is distinguishable from a leftover pose.
      gui::ApplyTrackballRotation(60.0f, 0.0f);
      IM_CHECK(std::memcmp(gui::g_crystal_rotation, expected0, sizeof expected0) != 0);

      gui::g_state.SelectCrystal(0, 1);
      ctx->Yield(3);
      expect_pose("entry 1 after a direct row-to-row switch", expected1);

      if (ctx->IsError()) {
        return;
      }

      // Re-selecting the SAME crystal is idempotent: the drag survives. This is what separates the
      // crystal_id judge from an "always reset" fix. Driven by leaving the page and coming back,
      // since re-selecting the row already selected is not a state change at all — the page would
      // not even notice it, which would make this half of the case vacuous.
      gui::ApplyTrackballRotation(30.0f, 20.0f);
      float after_drag[16];
      std::memcpy(after_drag, gui::g_crystal_rotation, sizeof after_drag);
      IM_CHECK(std::memcmp(after_drag, expected1, sizeof expected1) != 0);
      gui::g_state.SelectSun();
      ctx->Yield(3);
      gui::g_state.SelectCrystal(0, 1);
      ctx->Yield(3);
      IM_CHECK(std::memcmp(gui::g_crystal_rotation, after_drag, sizeof after_drag) == 0);

      ctx->Yield(2);
    };
  }

  // P88. Reset View and the entry-card thumbnail must derive their matrix from one source
  // (DefaultPreviewRotation), or the modal shows a crystal from an angle the card never does. All
  // six presets are walked because kCustom takes the chain-formula branch that the four canonical
  // ones do not.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "reset_view_lands_on_the_same_pose_the_card_thumbnail_uses");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      auto verify = [&](const char* label, gui::AxisPreset preset, gui::AxisDist zenith, gui::AxisDist azimuth,
                        gui::AxisDist roll) {
        ResetTestState();
        const ScopedPopups popup_guard(ctx);
        ctx->Yield(2);
        auto& cr = EntryCrystal();
        cr.zenith = zenith;
        cr.azimuth = azimuth;
        cr.roll = roll;
        IM_CHECK_EQ(static_cast<int>(gui::ClassifyAxisPreset(cr.zenith, cr.azimuth, cr.roll)),
                    static_cast<int>(preset));

        OpenCrystalTab(ctx);
        ctx->Yield(3);
        ctx->ItemClick("**/Reset View##modal");
        ctx->Yield(2);
        // Snapshot BEFORE Cancel: Cancel restores the pose saved at open time, so a read afterwards
        // would observe the pre-open matrix instead of Reset View's result.
        float observed[16];
        std::memcpy(observed, gui::g_crystal_rotation, sizeof observed);
        ctx->Yield(2);

        gui::AxisDist params[3] = { zenith, azimuth, roll };
        float expected[16] = { 0 };
        gui::DefaultPreviewRotation(preset, params, expected);
        for (int i = 0; i < 16; ++i) {
          if (observed[i] != expected[i]) {
            IM_ERRORF("preset=%s index=%d actual=%f expected=%f", label, i, static_cast<double>(observed[i]),
                      static_cast<double>(expected[i]));
          }
        }
      };

      const gui::AxisDist az_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_locked{ gui::AxisDistType::kGauss, 0.0f, 1.0f };

      verify("Plate", gui::AxisPreset::kPlate, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 1.0f }, az_full,
             roll_full);
      if (ctx->IsError()) {
        return;
      }
      verify("Column", gui::AxisPreset::kColumn, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f }, az_full,
             roll_full);
      if (ctx->IsError()) {
        return;
      }
      verify("Parry", gui::AxisPreset::kParry, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f }, az_full,
             roll_locked);
      if (ctx->IsError()) {
        return;
      }
      verify("Lowitz", gui::AxisPreset::kLowitz, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 40.0f }, az_full,
             roll_locked);
      if (ctx->IsError()) {
        return;
      }
      verify("Random", gui::AxisPreset::kRandom, az_full, az_full, roll_full);
      if (ctx->IsError()) {
        return;
      }
      verify("Custom", gui::AxisPreset::kCustom, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 20.0f }, az_full,
             gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 20.0f });
    };
  }

  // P94 / P95. Picking a preset in the Axis tab must move the MODAL preview too, not just the card
  // thumbnail outside it — the bug was a user changing preset and seeing the crystal they were
  // looking at stay put.
  //
  // The expected distributions are inlined from edit_modals.cpp's file-scope kAxisPresets, which is
  // not exported. A change there must be mirrored here; that is the cost of the table being private.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "an_axis_preset_button_resets_the_modal_preview");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const gui::AxisDist az_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_free{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      const gui::AxisDist roll_locked{ gui::AxisDistType::kGauss, 0.0f, 1.0f };

      auto verify = [&](const char* label, gui::AxisPreset preset, gui::AxisDist zenith, gui::AxisDist azimuth,
                        gui::AxisDist roll) {
        ResetTestState();
        const ScopedPopups popup_guard(ctx);
        ctx->Yield(2);
        OpenCrystalTab(ctx);
        ctx->Yield(3);
        ctx->ItemClick("**/###axis_tab");
        ctx->Yield(2);
        ctx->ItemClick(("**/" + std::string(label)).c_str());
        ctx->Yield(2);

        float observed[16];
        std::memcpy(observed, gui::g_crystal_rotation, sizeof observed);
        gui::AxisDist params[3] = { zenith, azimuth, roll };
        float expected[16] = { 0 };
        gui::DefaultPreviewRotation(preset, params, expected);
        ctx->Yield(2);

        for (int i = 0; i < 16; ++i) {
          if (observed[i] != expected[i]) {
            IM_ERRORF("preset=%s index=%d actual=%f expected=%f", label, i, static_cast<double>(observed[i]),
                      static_cast<double>(expected[i]));
          }
        }
      };

      verify("Column", gui::AxisPreset::kColumn, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f }, az_full,
             roll_free);
      if (ctx->IsError()) {
        return;
      }
      verify("Plate", gui::AxisPreset::kPlate, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 1.0f }, az_full,
             roll_free);
      if (ctx->IsError()) {
        return;
      }
      verify("Parry", gui::AxisPreset::kParry, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f }, az_full,
             roll_locked);
      if (ctx->IsError()) {
        return;
      }
      verify("Lowitz", gui::AxisPreset::kLowitz, gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 40.0f }, az_full,
             roll_locked);
      if (ctx->IsError()) {
        return;
      }
      verify("Random", gui::AxisPreset::kRandom, az_full, az_full, roll_free);
      if (ctx->IsError()) {
        return;
      }
      verify("Custom", gui::AxisPreset::kCustom, gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 20.0f }, az_full,
             gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 20.0f });
    };
  }

  // P95's negative half. Only the preset buttons and Reset View may overwrite the pose; an ordinary
  // axis-distribution edit must leave it alone, or dragging a Std slider would keep snapping the
  // crystal back while the user is looking at it. The two halves of SliderWithInput write the same
  // bound field, so the text path standing in for the drag path cannot hide a difference.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "an_axis_slider_edit_leaves_the_preview_pose_alone");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      OpenCrystalTab(ctx);
      ctx->Yield(3);

      // Snapshot AFTER the drag, so the baseline is the dragged pose rather than the open-time one.
      gui::ApplyTrackballRotation(20.0f, 0.0f);
      ctx->Yield(1);
      float before[16];
      std::memcpy(before, gui::g_crystal_rotation, sizeof before);

      ctx->ItemClick("**/###axis_tab");
      ctx->Yield(2);
      ctx->ItemInputValue("**/Zenith/##Mean", 45.0f);
      ctx->Yield(2);

      // ...and again BEFORE Cancel, for the same reason as in the Reset View case above.
      float after[16];
      std::memcpy(after, gui::g_crystal_rotation, sizeof after);
      ctx->Yield(2);

      for (int i = 0; i < 16; ++i) {
        if (before[i] != after[i]) {
          IM_ERRORF("an axis edit moved the preview at index %d: before=%f after=%f", i, static_cast<double>(before[i]),
                    static_cast<double>(after[i]));
        }
      }
    };
  }

  // P58. The distribution combo re-shapes the row beneath it: the second slider is a different
  // control per type (Std / Range / Amplitude / Scale), and the stored value is pulled into the new
  // type's range on the switch. Both are asserted — a combo that only rewrote the label would leave
  // a 150° "Amplitude" on a control whose maximum is 90.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "the_distribution_combo_reshapes_the_row_beneath_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      auto& cr = EntryCrystal();
      cr.zenith = gui::AxisDist{ gui::AxisDistType::kGauss, 0.0f, 150.0f };  // legal for Gauss (max 180)
      ctx->Yield(2);

      OpenAxisTab(ctx);
      ctx->Yield(4);
      ctx->ItemClick("**/###axis_tab");
      ctx->Yield(2);
      IM_CHECK(InspectorItemExists(ctx, "**/Zenith/##Std"));
      IM_CHECK(!InspectorItemExists(ctx, "**/Zenith/##Amplitude"));

      ctx->ItemClick(AxisDistComboId(ctx, "Zenith"));
      ctx->Yield(2);
      ctx->ItemClick("**/Zigzag");  // the popup's Selectables ARE registered by label
      ctx->Yield(3);

      IM_CHECK(InspectorItemExists(ctx, "**/Zenith/##Amplitude"));
      IM_CHECK(!InspectorItemExists(ctx, "**/Zenith/##Std"));
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(EntryCrystal().zenith.type), static_cast<int>(gui::AxisDistType::kZigzag));
      IM_CHECK_EQ(EntryCrystal().zenith.std, 90.0f);  // clamped into Zigzag's range on the switch
    };
  }

  // ===================================================================================
  // The shape table: rows, domains, randomization, and the width budget.
  // ===================================================================================

  // P90. Prism draws one height row; Pyramid replaces it with the five that describe a capped
  // column. Asserted by which controls exist, since that is what the user can reach.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "switching_crystal_type_swaps_the_row_set");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      IM_CHECK(InspectorItemExists(ctx, kHeightInput));
      IM_CHECK(!InspectorItemExists(ctx, "**/##Prism H##modal_cr"));

      ctx->ItemClick("**/Pyramid##modal");
      ctx->Yield(3);
      IM_CHECK(!InspectorItemExists(ctx, kHeightInput));
      for (const char* row : { "**/##Prism H##modal_cr", "**/##Upper H##modal_cr", "**/##Lower H##modal_cr",
                               "**/##Upper A##modal_cr", "**/##Lower A##modal_cr" }) {
        if (!InspectorItemExists(ctx, row)) {
          IM_ERRORF("pyramid row missing: %s", row);
        }
        if (ctx->IsError()) {
          break;
        }
      }

      // P89: a wedge-angle row carries Param and Value only. Its Sync / Rand / Spread cells are
      // pushed but left empty, so the grid stays rectangular and the headers stay over their
      // columns — which is why the sync swatch exists on an H row and not on a wedge row.
      IM_CHECK(InspectorItemExists(ctx, "**/##sync_Upper H##modal_cr"));
      IM_CHECK(!InspectorItemExists(ctx, "**/##sync_Upper A##modal_cr"));
      IM_CHECK(!InspectorItemExists(ctx, "**/##rnd_Upper A##modal_cr"));

      ctx->ItemClick("**/Prism##modal");
      ctx->Yield(3);
      IM_CHECK(InspectorItemExists(ctx, kHeightInput));
      ctx->Yield(2);
    };
  }

  // P57 as the shape rows see it. Each row declares a domain and a mapping in slider_mapping.hpp,
  // and what the user actually lands on when typing a number is the clamp. Four separate cases used
  // to stand here, one per row and bound; they are the same act (type an out-of-domain value, OK,
  // read back) over a table of rows, so the table is the honest shape.
  //
  // The input path applies std::clamp only — it does NOT exercise the non-linear drag mapping. The
  // kLog vs kLinear distinction is still caught, though, because the two rows below have different
  // bounds and a row that lost its declared range would land on the wrong number here.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "each_shape_row_clamps_typed_values_to_its_declared_domain");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Row {
        gui::CrystalType type;
        const char* input;
        float typed;
        float expected;
        const char* why;
      };
      // Height is kLog over [0.01, 100]; the pyramid H rows are kLinear over [0, 1].
      const Row kRows[] = {
        { gui::CrystalType::kPrism, "**/##Height##modal_cr", 0.0f, 0.01f, "kLog cannot reach zero" },
        { gui::CrystalType::kPyramid, "**/##Upper H##modal_cr", 0.0f, 0.0f, "kLinear starts at zero" },
        { gui::CrystalType::kPyramid, "**/##Upper H##modal_cr", 1.5f, 1.0f, "clamped to the upper bound" },
        { gui::CrystalType::kPyramid, "**/##Upper H##modal_cr", 0.5f, 0.5f, "an in-range value is identity" },
      };

      for (const Row& row : kRows) {
        ResetTestState();
        const ScopedPopups popup_guard(ctx);
        ctx->Yield(2);
        EntryCrystal().type = row.type;
        ctx->Yield();

        OpenCrystalTab(ctx);
        ctx->Yield(3);
        ctx->ItemInputValue(row.input, row.typed);
        ctx->Yield();
        ctx->Yield(2);

        const float got =
            (row.type == gui::CrystalType::kPrism) ? EntryCrystal().height.center : EntryCrystal().upper_h.center;
        // Reported rather than asserted fatally, for the message: which row disagrees, and by what,
        // is the diagnostic. It does not make the sweep finish — see the note on ctx->IsError() in
        // test_gui_shared.hpp — which is why the loop stops itself below.
        if (std::fabs(got - row.expected) > 1e-6f) {
          IM_ERRORF("%s: typed %f, expected %f, got %f (%s)", row.input, static_cast<double>(row.typed),
                    static_cast<double>(row.expected), static_cast<double>(got), row.why);
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P64 / P65. Randomization is off for a new crystal, turning it on picks Uniform with a spread
  // proportional to the centre, and turning it off collapses the spread back to zero so that
  // equality and round-tripping stay clean. The Spread cell is rendered in both states (greyed
  // rather than absent), which is asserted on either side of the toggle.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "edit_modal", "randomizing_a_row_defaults_to_uniform_and_zeroes_on_release");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      IM_CHECK_EQ(EntryCrystal().height.type, gui::ShapeDistType::kNoRandom);
      for (int i = 0; i < 6; ++i) {
        if (EntryCrystal().face_distance[i].type != gui::ShapeDistType::kNoRandom) {
          IM_ERRORF("face index %d is randomized on a brand-new crystal", i);
        }
      }

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      IM_CHECK((InspectorItemInfo(ctx, "**/##spread_Height##modal_cr").ItemFlags & ImGuiItemFlags_Disabled) != 0);

      // The checkbox is text-less — the table header names the column — so its id is the suffix.
      InspectorItemClick(ctx, "**/##rnd_Height##modal_cr");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().height.type, gui::ShapeDistType::kUniform);
      IM_CHECK_GT(EntryCrystal().height.spread, 0.0f);
      IM_CHECK((InspectorItemInfo(ctx, "**/##spread_Height##modal_cr").ItemFlags & ImGuiItemFlags_Disabled) == 0);

      InspectorItemClick(ctx, "**/##rnd_Height##modal_cr");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().height.type, gui::ShapeDistType::kNoRandom);
      IM_CHECK_EQ(EntryCrystal().height.spread, 0.0f);

      ctx->Yield(2);
    };
  }

  // The property table has no "apply to all faces" control — the owner rejected one — so each of the
  // six face rows is an independent distribution. This is the positive guard for that decision: a
  // re-added broadcast would flip all six at once and fail here. Several frames are pumped after the
  // two clicks so a broadcast that arrived passively on a later frame is caught too.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "each_face_row_randomizes_on_its_own");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");  // default-collapsed
      ctx->Yield(2);

      for (int i = 0; i < 6; ++i) {
        if (EntryCrystal().face_distance[i].type != gui::ShapeDistType::kNoRandom) {
          IM_ERRORF("face index %d is randomized before any row was toggled", i);
        }
      }

      InspectorItemClick(ctx, "**/##rnd_Face 3##modal_fd");
      ctx->Yield(2);
      InspectorItemClick(ctx, "**/##rnd_Face 4##modal_fd");
      ctx->Yield(2);
      ctx->Yield(6);

      IM_CHECK_EQ(EntryCrystal().face_distance[0].type, gui::ShapeDistType::kUniform);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].type, gui::ShapeDistType::kUniform);
      for (int i = 2; i < 6; ++i) {
        if (EntryCrystal().face_distance[i].type != gui::ShapeDistType::kNoRandom) {
          IM_ERRORF("face index %d was randomized by a click on another row", i);
        }
      }

      // ImGui persists a CollapsingHeader's open state per window across cases, so leaving this
      // section open would change the modal's layout for every later case in this process.
      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);
    };
  }

  // P92, all three of its clauses. Reset All restores the shape fields (including their sync-group
  // ids, which it clears by assigning whole ShapeDist values rather than by naming them), leaves
  // name / type / axis alone, and writes only the buffer — so a Cancel afterwards leaves the entry
  // exactly as it was.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "reset_all_restores_the_shape_and_only_the_buffer");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      // A non-default baseline, so "back to defaults" has something to differ from.
      OpenCrystalTab(ctx);
      ctx->Yield(3);
      ctx->ItemInputValue(kHeightInput, 5.0f);
      ctx->Yield();
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().height, 5.0f);

      // Sync groups have no control of their own on these rows, so they are injected straight into
      // the entry, from where the modal copies them into its buffer like any other shape state. Both
      // a scalar and two face slots: a rewrite that reset per-component instead of per-ShapeDist
      // would leave exactly these behind and nothing else in the suite would notice.
      EntryCrystal().height.sync_group = 3;
      EntryCrystal().face_distance[2].sync_group = 1;
      EntryCrystal().face_distance[4].sync_group = 1;
      // Captured for the "Reset All touches the SHAPE and nothing else" assertions at the end:
      // name, type and the three axis distributions must come through untouched.
      const gui::CrystalConfig before_modal = EntryCrystal();

      // The "Reset All then Cancel leaves the entry untouched" half of this case is gone with
      // Cancel: there is no uncommitted state for a discard to throw away, so the proposition has
      // no subject rather than a changed answer. What is left is the half that was always the
      // point — Reset All puts every shape field, including the sync groups, back to its default.
      OpenCrystalTab(ctx);
      ctx->Yield(3);
      ctx->ItemInputValue(kHeightInput, 2.0f);
      ctx->Yield();
      ctx->ItemClick("**/Reset All##modal_cr");
      ctx->Yield(3);

      const gui::CrystalConfig defaults;
      const auto& cr = EntryCrystal();
      IM_CHECK_EQ(cr.height, defaults.height);
      IM_CHECK_EQ(cr.prism_h, defaults.prism_h);
      IM_CHECK_EQ(cr.upper_h, defaults.upper_h);
      IM_CHECK_EQ(cr.lower_h, defaults.lower_h);
      IM_CHECK_EQ(cr.upper_alpha, defaults.upper_alpha);
      IM_CHECK_EQ(cr.lower_alpha, defaults.lower_alpha);
      // Reported per face rather than asserted fatally: which faces survived Reset All is the
      // diagnostic, and a fatal assert would stop at the first one.
      for (int i = 0; i < 6; ++i) {
        if (!(cr.face_distance[i] == defaults.face_distance[i])) {
          IM_ERRORF("face index %d was not reset", i);
        }
        // Sync group spelled out separately from ShapeDist::operator== above, so a failure names
        // the field rather than just "the distribution differs".
        if (cr.face_distance[i].sync_group != 0) {
          IM_ERRORF("face index %d kept sync group %d", i, cr.face_distance[i].sync_group);
        }
      }
      IM_CHECK_EQ(cr.height.sync_group, 0);
      // ...and the things Reset All must NOT touch.
      IM_CHECK_EQ(cr.name, before_modal.name);
      IM_CHECK_EQ(cr.type, before_modal.type);
      IM_CHECK(cr.zenith == before_modal.zenith);
      IM_CHECK(cr.azimuth == before_modal.azimuth);
      IM_CHECK(cr.roll == before_modal.roll);
    };
  }

  // P62 / P110. The Sync column is fixed-width and Value is the only stretch column, so every pixel
  // Sync takes comes out of the value control. The failure this catches is invisible in any state
  // assertion, and what it looks like changed when the [slider][input] pair became one DragFloat:
  // the pair used to OVERFLOW its cell once the slider hit PrepareSliderLayout's 40 px floor,
  // whereas a single control sized to the cell cannot overflow — it silently clips the number it
  // is showing instead. So the floor below is stated in terms of the text the control must render.
  // Measured in the worst case the page supports — a Pyramid with Face Distance expanded, in a
  // column narrower than the modal this replaced.
  //
  // The modal's companion claim — that the content fits its fixed-height pane without a scrollbar —
  // is gone with the pane. The page is a docked window the user can resize and its content scrolls
  // by design, so a scrollbar there is the feature rather than the regression.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "the_sync_column_leaves_the_slider_room");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      EntryCrystal().type = gui::CrystalType::kPyramid;

      // Every check below is fatal and every one of them is what a real narrow-layout regression
      // trips, so the modal this case pops has to be handed back by an object rather than by the
      // ItemClose/ItemClick pair at the end of the body.
      const ScopedPopups popup_guard(ctx);

      ctx->Yield(2);
      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->Yield(6);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(4);

      ImGuiWindow* win = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(win != nullptr);
      // Narrower than the modal this replaced (420 px), which is why the column budget below is
      // the binding constraint rather than a formality: the page lives in the document column and
      // takes its width from the dock node.
      IM_CHECK_EQ(win->Size.x, gui::kLeftPanelWidth);

      const auto value_ctl = InspectorItemInfo(ctx, "**/##Face 3##modal_fd");
      const auto swatch = InspectorItemInfo(ctx, "**/##sync_Face 3##modal_fd");
      const auto rand_check = InspectorItemInfo(ctx, "**/##rnd_Face 3##modal_fd");
      // A face distance renders as "0.000" at its widest, plus the frame's own padding. Below this
      // the cell is too narrow to show the value it holds — the modern shape of "Sync ate the
      // Value column", and the reason the number is a text measurement rather than a constant.
      const float min_readable = ImGui::CalcTextSize("0.000").x + ImGui::GetStyle().FramePadding.x * 2.0f;
      IM_CHECK_GT(value_ctl.RectFull.GetWidth(), min_readable);
      // The control must end before the next column's content begins. Column order is
      // Param | Value | Sync | Rand | Spread, so the swatch is what it has to clear — and the Rand
      // checkbox is checked too, since the ordering is itself part of the claim.
      IM_CHECK_LT(value_ctl.RectFull.Max.x, swatch.RectFull.Min.x);
      IM_CHECK_LT(swatch.RectFull.Max.x, rand_check.RectFull.Min.x);
      // The swatch is square and one frame tall, so it cannot be what drives row height.
      IM_CHECK_EQ(swatch.RectFull.GetWidth(), swatch.RectFull.GetHeight());
      IM_CHECK_LE(swatch.RectFull.GetHeight(), ImGui::GetFrameHeight());

      // The "does the content fit its fixed-height pane without a scrollbar" half of this case is
      // gone with the modal. The pane it named was the popup's own BeginChild, sized so the tallest
      // crystal layout fit inside a window that had to stay on screen. The page has no such pane:
      // it is a docked window the user can resize, and its content scrolls by design — a scrollbar
      // there is the feature, not the regression, so there is nothing left to assert.

      // Collapse the Face Distance node again: a tree node's open flag lives in the host window's
      // ImGui storage, which outlives ResetTestState, so the next case would otherwise start with
      // it expanded.
      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(4);
      ctx->Yield(2);
    };
  }

  // The other half of the width budget: the four fixed columns are sized by TEXT, not by their
  // controls — Param by its longest row label, Sync and Rand by their own headers, which are wider
  // than the swatch and the checkbox. Both clipping modes are SILENT (ImGui ellipsizes a header and
  // hard-clips a row label without complaint), so nothing else would notice.
  //
  // The criterion is ImGui's own layout bookkeeping rather than a pixel constant:
  // ContentMaxXHeadersIdeal is where a header's text would end unclipped, ContentMaxXUnfrozen the
  // same for the cells, and WorkMaxX the column's content region. Comparing the three is font- and
  // DPI-adaptive by construction — a platform whose glyphs render wider fails here instead of
  // shipping a clipped table.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "the_fixed_columns_fit_their_text");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Pyramid has the widest Param labels ("Prism H", "Upper A", "Lower A").
      EntryCrystal().type = gui::CrystalType::kPyramid;

      // One layout, where there were two. The modal offered a side-by-side arrangement and a
      // stacked one and this case checked the column budget in both; the page is a fixed-width
      // column, so the stacked arrangement is the only one that means anything and the other has
      // no window to be measured in. The lambda is kept rather than inlined because its checks are
      // fatal, and a fatal assert returns out of the enclosing function — keeping the body in a
      // callee is what stops one failure from also skipping the teardown below it.
      auto check_layout = [ctx]() {
        const ScopedPopups popup_guard(ctx);
        ctx->Yield(2);
        OpenCrystalTab(ctx);
        ctx->Yield(6);
        ctx->ItemOpen("**/Face Distance##modal");
        ctx->Yield(4);

        ImGuiWindow* win = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
        IM_CHECK(win != nullptr);
        IM_CHECK_EQ(win->Size.x, gui::kLeftPanelWidth);

        // Both shape tables, found by their column signature rather than by an id we would have to
        // reproduce through the child-window stack.
        ImGuiContext& g = *ImGui::GetCurrentContext();
        int tables_checked = 0;
        for (int n = 0; n < g.Tables.GetMapSize(); ++n) {
          ImGuiTable* table = g.Tables.TryGetMapData(n);
          if (table == nullptr || table->ColumnsCount != gui::kShapeTableColumnCount ||
              table->LastFrameActive < g.FrameCount - 1) {
            continue;
          }
          const char* col0 = ImGui::TableGetColumnName(table, 0);
          if (col0 == nullptr || std::strcmp(col0, "Param") != 0) {
            continue;  // some other table that happens to have the same column count
          }
          ++tables_checked;
          for (int c = 0; c < table->ColumnsCount; ++c) {
            const ImGuiTableColumn& column = table->Columns[c];
            const char* name = ImGui::TableGetColumnName(table, c);
            if (std::strcmp(name, "Value") == 0) {
              continue;  // the stretch column absorbs the slack; the others are what is budgeted
            }
            const float ideal = ImMax(column.ContentMaxXHeadersIdeal, column.ContentMaxXUnfrozen);
            if (ideal > column.WorkMaxX) {
              IM_ERRORF("column %s clips: needs %.1f, has %.1f", name, static_cast<double>(ideal - column.WorkMinX),
                        static_cast<double>(column.WorkMaxX - column.WorkMinX));
            }
          }
        }
        IM_CHECK_EQ(tables_checked, 2);  // params + Face Distance; a miss must fail, not skip

        ctx->ItemClose("**/Face Distance##modal");
        ctx->Yield(2);
        ctx->Yield(2);
      };

      check_layout();
    };
  }

  // ===================================================================================
  // The Sync column: shape-scalar groups, built by clicking.
  // ===================================================================================
  //
  // Every case below drives the real widgets — swatch button, then popup item — rather than the
  // helpers underneath. The point of this column is that a user can build a grouping by clicking,
  // and a test that called the helper directly would keep passing after the popup stopped being
  // reachable.
  //
  // The ids are stable by construction: the swatch is "##sync_<row label>" with the group number
  // DRAWN on it rather than being part of the label, and the popup items are anchored on
  // "###sync_none" / "###sync_group_N" / "###sync_new", so the membership text they display can
  // change without moving the id out from under these paths.

  // Joining snapshots the group's value onto the joining row; editing ANY member afterwards writes
  // through to the whole group. Both halves, plus the scope: an ungrouped row is untouched.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "joining_a_sync_group_snapshots_it_and_edits_propagate");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);

      // Face 5 (face_distance[2]) opens a group of its own; nothing else is grouped, so "+ New
      // group" hands out id 1.
      InspectorItemClick(ctx, "**/##sync_Face 5##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_new");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].sync_group, 1);
      // A brand-new group has no other member, so joining must not disturb the row's own value.
      IM_CHECK_EQ(EntryCrystal().face_distance[2].center, 1.0f);

      // Give the group a distinctive distribution, so the snapshot below is checked on all three
      // fields rather than on a centre that happened to match already.
      InspectorItemClick(ctx, "**/##rnd_Face 5##modal_fd");
      ctx->Yield(2);
      InspectorItemInputValue(ctx, "**/##Face 5##modal_fd", 1.5f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].center, 1.5f);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].type, gui::ShapeDistType::kUniform);
      const float leader_spread = EntryCrystal().face_distance[2].spread;
      IM_CHECK(leader_spread > 0.0f);

      IM_CHECK_EQ(EntryCrystal().face_distance[4].center, 1.0f);  // differs from the leader pre-join
      InspectorItemClick(ctx, "**/##sync_Face 7##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_1");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[4].sync_group, 1);
      IM_CHECK_EQ(EntryCrystal().face_distance[4].center, 1.5f);
      IM_CHECK_EQ(EntryCrystal().face_distance[4].type, gui::ShapeDistType::kUniform);
      IM_CHECK_EQ(EntryCrystal().face_distance[4].spread, leader_spread);

      // Propagation runs from either end: the owner chose writable subordinates over greyed ones.
      InspectorItemInputValue(ctx, "**/##Face 5##modal_fd", 0.75f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].center, 0.75f);
      IM_CHECK_EQ(EntryCrystal().face_distance[4].center, 0.75f);
      InspectorItemInputValue(ctx, "**/##Face 7##modal_fd", 1.25f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[4].center, 1.25f);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].center, 1.25f);

      // Scoped to the group, not to "every face row".
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 1.0f);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].sync_group, 0);

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);
    };
  }

  // Leaving keeps the value the group imposed — there is no shadow copy of what the row held before
  // it joined — and does not disturb the group it left.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "leaving_a_sync_group_keeps_the_value_it_was_given");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);

      // A two-member group holding a non-default value, so "unchanged on leave" is a claim about a
      // value the group actually imposed.
      InspectorItemClick(ctx, "**/##sync_Face 3##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_new");
      ctx->Yield(2);
      InspectorItemInputValue(ctx, "**/##Face 3##modal_fd", 1.75f);
      ctx->Yield(2);
      InspectorItemClick(ctx, "**/##sync_Face 4##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_1");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].sync_group, 1);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].center, 1.75f);

      InspectorItemClick(ctx, "**/##sync_Face 4##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_none");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].sync_group, 0);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].center, 1.75f);  // NOT the 1.0 it had before joining
      IM_CHECK_EQ(EntryCrystal().face_distance[0].sync_group, 1);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 1.75f);

      // ...and it really stopped following: editing the ex-leader leaves it where it is.
      InspectorItemInputValue(ctx, "**/##Face 3##modal_fd", 0.5f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 0.5f);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].center, 1.75f);

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);
    };
  }

  // The widget only ever sees the scalars the CURRENT crystal type draws, so a group id sitting on
  // the other type's scalar has to survive a type switch untouched and must not be handed out again.
  // This is the "no commensurability check" design implemented as the smallest possible behaviour; a
  // later tidy-up that cleared such ids would silently change it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "a_group_on_the_other_types_scalar_stays_dormant");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      // height is prism-only; give it a group, then edit the crystal as a pyramid.
      EntryCrystal().height.sync_group = 4;
      EntryCrystal().type = gui::CrystalType::kPrism;

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemClick("**/Pyramid##modal");
      ctx->Yield(3);

      InspectorItemClick(ctx, "**/##sync_Upper H##modal_cr");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_new");
      ctx->Yield(2);
      // "+ New group" must not hand out 4 — the dormant prism group still holds it, and reusing it
      // would silently merge the two on a switch back.
      IM_CHECK_EQ(EntryCrystal().upper_h.sync_group, 5);
      // Slot-order trap: it is upper_h that moved, not prism_h (slot 1 vs slot 2, the reverse of the
      // rows' visual order). A positional slot mapping fails exactly here.
      IM_CHECK_EQ(EntryCrystal().prism_h.sync_group, 0);
      IM_CHECK_EQ(EntryCrystal().lower_h.sync_group, 0);
      IM_CHECK_EQ(EntryCrystal().height.sync_group, 4);

      ctx->ItemClick("**/Prism##modal");
      ctx->Yield(3);
      IM_CHECK_EQ(EntryCrystal().height.sync_group, 4);
      IM_CHECK_EQ(EntryCrystal().upper_h.sync_group, 5);

      ctx->Yield(2);
    };
  }

  // P61. Re-picking the group a row is already in must not re-snapshot it — the popup's
  // "if (dist.sync_group != group)" guard is load-bearing.
  //
  // The starting state is a group whose members hold DIFFERENT distributions. That is not contrived:
  // the GUI passes hand-authored sync_group values through verbatim (core's from_json owns
  // normalization, on commit), and it is the only state in which "re-select does nothing" and
  // "re-select snapshots from the leader" are distinguishable at all. A group built by clicking has
  // equal members by construction, and the case would pass just as happily with the guard deleted.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "reselecting_the_current_sync_group_changes_nothing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      EntryCrystal().face_distance[0] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.6f, 0.3f };
      EntryCrystal().face_distance[0].sync_group = 1;  // the leader, lowest slot
      EntryCrystal().face_distance[1] = gui::ShapeDist{ gui::ShapeDistType::kNoRandom, 0.9f, 0.0f };
      EntryCrystal().face_distance[1].sync_group = 1;
      const gui::ShapeDist leader_before = EntryCrystal().face_distance[0];
      const gui::ShapeDist member_before = EntryCrystal().face_distance[1];

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);

      InspectorItemClick(ctx, "**/##sync_Face 4##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_1");
      ctx->Yield(2);
      IM_CHECK(EntryCrystal().face_distance[1] == member_before);
      IM_CHECK(EntryCrystal().face_distance[0] == leader_before);

      // Positive control on the same row: picking a DIFFERENT group does snapshot. Without it, a
      // widget that ignored every popup click would pass the assertions above.
      InspectorItemClick(ctx, "**/##sync_Face 5##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_new");
      ctx->Yield(2);
      InspectorItemClick(ctx, "**/##sync_Face 4##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_2");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].sync_group, 2);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].center, EntryCrystal().face_distance[2].center);

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);
    };
  }

  // The leader rule, checked across the two ways "lowest slot index" can disagree with what a naive
  // search finds. lumice.h defines a group's owner as its lowest-indexed applicable member, over
  // every slot the crystal type has; the GUI snapshots from that same slot so that what the user
  // sees on join is the value core will draw with.
  //   group 1 — slot order vs ROW order: Upper H is slot 1, Lower H slot 3, and Upper H is drawn
  //             BELOW Prism H. A search following the visual order, or CrystalConfig's field order,
  //             lands on the wrong one.
  //   group 2 — slot order vs REACHABLE order: Prism H (slot 2) is the group's lowest member and has
  //             no Sync control of its own. It is the leader all the same. Electing the lowest slot
  //             the user can REACH instead would hand out face_distance[1]'s 1.5 while core draws
  //             with 0.75, i.e. the table would show a distribution the simulation overwrites.
  // Both groups are seeded with members that DISAGREE — the only configuration in which "which
  // member is the leader" is observable at all.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "edit_modal", "the_sync_leader_is_the_lowest_slot_not_the_lowest_reachable_row");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      auto& cr = EntryCrystal();
      cr.type = gui::CrystalType::kPyramid;
      cr.upper_h = gui::ShapeDist{ gui::ShapeDistType::kUniform, 0.4f, 0.08f };
      cr.upper_h.sync_group = 1;  // slot 1 — group 1's leader
      cr.lower_h = gui::ShapeDist{ gui::ShapeDistType::kNoRandom, 0.9f, 0.0f };
      cr.lower_h.sync_group = 1;  // slot 3 — the foil
      // Prism H's value is inside a face slider's [0, 2] range on purpose, so a wrong leader shows
      // up as the wrong NUMBER rather than as a clamp artifact that could be explained away.
      cr.prism_h = gui::ShapeDist{ gui::ShapeDistType::kNoRandom, 0.75f, 0.0f };
      cr.prism_h.sync_group = 2;
      cr.face_distance[1] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.5f, 0.3f };
      cr.face_distance[1].sync_group = 2;  // slot 5 — the lowest slot the user can reach

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);

      // Face 3 (slot 4) joins group 1: it must take upper_h's distribution, not lower_h's. Every
      // face slot is above all three H slots, so no face can be the leader here.
      InspectorItemClick(ctx, "**/##sync_Face 3##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_1");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].sync_group, 1);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 0.4f);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].type, gui::ShapeDistType::kUniform);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].spread, 0.08f);

      // Face 5 (slot 6) joins group 2: the leader is prism_h, not the higher-indexed
      // face_distance[1] that happens to be the lowest member carrying a Sync control.
      InspectorItemClick(ctx, "**/##sync_Face 5##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_2");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].sync_group, 2);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].center, 0.75f);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].type, gui::ShapeDistType::kNoRandom);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].spread, 0.0f);

      // Joining READS the group; it does not write it. Every seeded member is untouched and the two
      // groups are still internally disagreeing.
      IM_CHECK_EQ(EntryCrystal().upper_h.center, 0.4f);
      IM_CHECK_EQ(EntryCrystal().lower_h.center, 0.9f);
      IM_CHECK_EQ(EntryCrystal().prism_h.center, 0.75f);
      IM_CHECK_EQ(EntryCrystal().prism_h.sync_group, 2);
      IM_CHECK_EQ(EntryCrystal().face_distance[1].center, 1.5f);

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);
    };
  }

  // A hand-authored config MAY group a scalar the GUI offers no Sync control for — lumice.h runs no
  // commensurability check and the GUI is not a second authority on what is legal. Once such a group
  // exists, core treats that scalar as an ordinary member (it is applicable, so normalization elects
  // it and writes over the rest of the group from it), and the GUI must report and propagate it the
  // same way. The missing Sync control means one thing only: the user cannot make THIS row join a
  // group from here.
  //
  // All the consequences are walked in one case because the failure mode is a HALF-applied
  // predicate: hiding the swatch is the affordance, and if the membership list, the leader search or
  // the propagation followed it too, the table would show a distribution core overwrites on commit —
  // the row silently reverting after the user edited it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "a_row_without_a_sync_control_is_still_a_full_member");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      // Exactly two members, so the popup's membership label stays inside
      // ImGuiTestItemInfo::DebugLabel's 32-character buffer and the string checks below are not
      // reading a truncation.
      const std::string core_json = R"({
        "crystal": [
          {"id": 1, "type": "prism",
           "shape": {"height": 1.0,
                     "face_distance": [1,1,1,1,1,1],
                     "sync_group": {"height": 1, "face_distance": [1,0,0,0,0,0]}}}
        ],
        "scene": {
          "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"},
          "ray_num": 1000,
          "max_hits": 8,
          "scattering": [
            {"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0}]}
          ]
        }
      })";
      // DeserializeFromJson assigns a fresh GuiState. That used to need repairing afterwards: the
      // modal carried two display-tier layout flags that the import reset to their struct defaults,
      // handing the clicks below a differently-shaped window. The page has no such flags — its
      // geometry is the dock node's — so the import needs no fix-up.
      IM_CHECK(gui::DeserializeFromJson(core_json, gui::g_state));
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().type, gui::CrystalType::kPrism);
      IM_CHECK_EQ(EntryCrystal().height.sync_group, 1);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].sync_group, 1);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);

      // (1) No Sync control on the Height row — with two positive controls beside it, so a false
      // pass is impossible: the row itself is still there and editable, and a row that IS syncable
      // still has its swatch.
      IM_CHECK(!InspectorItemExists(ctx, "**/##sync_Height##modal_cr"));
      IM_CHECK(InspectorItemExists(ctx, kHeightInput));
      IM_CHECK(InspectorItemExists(ctx, "**/##sync_Face 3##modal_fd"));

      // (2) The membership list names the group's real members, Height included — it describes the
      // group the user is about to join, and Height is the member whose value that group carries.
      // Read off the popup item's visible label rather than re-derived.
      InspectorItemClick(ctx, "**/##sync_Face 3##modal_fd");
      ctx->Yield(2);
      const ImGuiTestItemInfo group_item = InspectorItemInfo(ctx, "**/###sync_group_1");
      IM_CHECK_NE(group_item.ID, (ImGuiID)0);
      IM_CHECK(std::strstr(group_item.DebugLabel, "Face 3") != nullptr);  // the list is really read
      IM_CHECK(std::strstr(group_item.DebugLabel, "Height") != nullptr);  // ...and Height is in it
      // Re-selecting the group the row is already in is a documented no-op, so this only dismisses
      // the popup.
      ctx->ItemClick("**/###sync_group_1");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].sync_group, 1);

      // (3) An edit to either row moves the other. face→height is the destination-side scope (core
      // normalizes Height too, so leaving it behind would only mean core rewrites the face on
      // commit); height→face is the source-side scope — the row has no swatch, but it is the
      // group's leader, and an edit to the leader is exactly the edit the whole group must follow.
      // Both values sit inside BOTH sliders' ranges (Height's [0.01, 100] and a face's [0, 2]), so
      // nothing here is a clamp; cross-range clamping has its own case.
      InspectorItemInputValue(ctx, "**/##Face 3##modal_fd", 0.75f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 0.75f);
      IM_CHECK_EQ(EntryCrystal().height.center, 0.75f);
      ctx->ItemInputValue(kHeightInput, 1.8f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().height.center, 1.8f);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 1.8f);

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);

      // (4) The group id was never the GUI's to clean up: it survives the walkthrough in state AND
      // through the production export path, read back rather than matched as a substring.
      IM_CHECK_EQ(EntryCrystal().height.sync_group, 1);
      std::string exported;
      IM_CHECK(gui::BuildExportJsonOrWarn(gui::g_state, &exported, nullptr));
      gui::GuiState reloaded = gui::InitDefaultState();
      IM_CHECK(gui::DeserializeFromJson(exported, reloaded));
      const auto& rc = gui::CrystalOf(reloaded, reloaded.layers[0].entries[0]);
      IM_CHECK_EQ(rc.height.sync_group, rc.face_distance[0].sync_group);
      IM_CHECK_NE(rc.height.sync_group, 0);
    };
  }

  // The mechanism that pins "the GUI's leader IS core's leader", as opposed to a comment asking the
  // next person to keep them in step.
  //
  // The two rules are two implementations of one sentence in lumice.h, nothing structural forces
  // them together, and they HAVE drifted apart once: a GUI-side affordance narrowing (some rows get
  // no Sync control) leaked into the leader search, so for a hand-authored group the table handed
  // out a distribution core discarded on commit. A test that hardcoded the expected leader could not
  // catch the recurrence in general — it would freeze today's GUI-side belief, and if CORE's rule is
  // what moves, the frozen literal moves with the GUI and the test stays green while the two diverge.
  //
  // So the expected value is measured from core at run time. BuildCrystalMeshData goes through the
  // public mesh API, whose param→JSON→param round trip runs the real normalization before sampling
  // geometry; feed it the disagreeing group and it returns the mesh core would actually draw.
  // Compare that against one control mesh per candidate leader — the same crystal with the group
  // flattened onto that candidate — and exactly one matches. That names core's leader without
  // reading a line of GUI code, and the GUI then has to agree with it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "the_guis_sync_leader_is_the_one_core_draws_with");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      // The owner's reproduction verbatim: prism height (slot 0, no Sync control) grouped with
      // face_distance[0] (slot 4), the two disagreeing.
      constexpr float kHeightValue = 1.0f;
      constexpr float kFaceValue = 0.75f;
      auto& cr = EntryCrystal();
      cr.type = gui::CrystalType::kPrism;
      cr.height = gui::ShapeDist{ gui::ShapeDistType::kNoRandom, kHeightValue, 0.0f };
      cr.height.sync_group = 1;
      cr.face_distance[0] = gui::ShapeDist{ gui::ShapeDistType::kNoRandom, kFaceValue, 0.0f };
      cr.face_distance[0].sync_group = 1;

      auto build = [](const gui::CrystalConfig& c, LUMICE_CrystalMesh* out) {
        // kNoRandom throughout, so the seed selects nothing and the draw is deterministic; passing
        // the same literal every time keeps that explicit rather than incidental.
        return gui::BuildCrystalMeshData(c, gui::kPreviewFixedSampleSeed, out);
      };
      auto same_mesh = [](const LUMICE_CrystalMesh& a, const LUMICE_CrystalMesh& b) {
        return a.vertex_count == b.vertex_count && a.edge_count == b.edge_count &&
               a.triangle_count == b.triangle_count &&
               std::memcmp(a.vertices, b.vertices, sizeof(float) * a.vertex_count * 3) == 0 &&
               std::memcmp(a.edges, b.edges, sizeof(int) * a.edge_count * 2) == 0 &&
               std::memcmp(a.triangles, b.triangles, sizeof(int) * a.triangle_count * 3) == 0;
      };

      gui::CrystalConfig flat_height = EntryCrystal();
      flat_height.height.center = kHeightValue;
      flat_height.face_distance[0].center = kHeightValue;
      flat_height.height.sync_group = 0;
      flat_height.face_distance[0].sync_group = 0;
      gui::CrystalConfig flat_face = flat_height;
      flat_face.height.center = kFaceValue;
      flat_face.face_distance[0].center = kFaceValue;

      LUMICE_CrystalMesh mesh_core{};
      LUMICE_CrystalMesh mesh_flat_height{};
      LUMICE_CrystalMesh mesh_flat_face{};
      IM_CHECK(build(EntryCrystal(), &mesh_core));
      IM_CHECK(build(flat_height, &mesh_flat_height));
      IM_CHECK(build(flat_face, &mesh_flat_face));

      const bool core_chose_height = same_mesh(mesh_core, mesh_flat_height);
      const bool core_chose_face = same_mesh(mesh_core, mesh_flat_face);
      // Exactly one: both true would mean the mesh cannot tell the candidates apart and the oracle
      // proves nothing; both false would mean core picked something neither control models, and
      // reading a leader value out of that would be an invention.
      IM_CHECK(core_chose_height != core_chose_face);
      const float core_leader_value = core_chose_height ? kHeightValue : kFaceValue;

      // Face 5 joining group 1 snapshots from whatever the GUI thinks the leader is, so the value
      // that lands in the row IS the GUI's answer — read through a real click rather than by calling
      // the predicate.
      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);
      InspectorItemClick(ctx, "**/##sync_Face 5##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_1");
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].sync_group, 1);
      IM_CHECK_EQ(EntryCrystal().face_distance[2].center, core_leader_value);

      // The other half: with the GUI writing the whole group, an edit leaves every member equal, so
      // core's normalization finds nothing to override — no silent rewrite of the row the user just
      // typed into. Checked against core the same way: after the edit, the grouped config's mesh
      // must equal the mesh of the same values carrying no groups at all.
      InspectorItemInputValue(ctx, "**/##Face 3##modal_fd", 1.25f);
      ctx->Yield(2);
      gui::CrystalConfig after_edit_ungrouped = EntryCrystal();
      for (int s = 0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
        gui::ShapeScalarAt(after_edit_ungrouped, s).sync_group = 0;
      }
      LUMICE_CrystalMesh mesh_after_edit{};
      LUMICE_CrystalMesh mesh_after_edit_ungrouped{};
      IM_CHECK(build(EntryCrystal(), &mesh_after_edit));
      IM_CHECK(build(after_edit_ungrouped, &mesh_after_edit_ungrouped));
      IM_CHECK(same_mesh(mesh_after_edit, mesh_after_edit_ungrouped));

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);
    };
  }

  // Grouping scalars whose sliders have DIFFERENT ranges is permitted, and Upper H [0, 1] with a
  // face distance [0, 2] is where that shows. What the group settles on is the tighter bound.
  //
  // The property worth pinning is not the number 1.0 — it is that the loop TERMINATES. Propagation
  // is triggered by "this row's value changed", and a clamp is a change, so a value no member can
  // hold is fed back and forth between rows. The EDITED row is the wide one and the clamping row is
  // drawn ABOVE it, so the clamp lands on the frame after the edit — the direction that genuinely
  // needs a second pass through the table, and the one an implementation that converges only because
  // the rows happen to be in the right order would fail.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "a_sync_group_across_two_ranges_settles_on_the_tighter_one");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);
      EntryCrystal().type = gui::CrystalType::kPyramid;

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);
      InspectorItemClick(ctx, "**/##sync_Upper H##modal_cr");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_new");
      ctx->Yield(2);
      InspectorItemClick(ctx, "**/##sync_Face 3##modal_fd");
      ctx->Yield(2);
      ctx->ItemClick("**/###sync_group_1");
      ctx->Yield(2);
      // Face 3 snapshotted Upper H's default 0.2, which is inside both ranges — nothing to clamp yet.
      IM_CHECK_EQ(EntryCrystal().upper_h.center, 0.2f);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 0.2f);

      // 2.0 is legal for a face distance and outside Upper H's [0, 1].
      InspectorItemInputValue(ctx, "**/##Face 3##modal_fd", 2.0f);
      ctx->Yield(2);
      IM_CHECK_EQ(EntryCrystal().upper_h.center, 1.0f);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 1.0f);
      // ...and it stays there: no oscillation between the two clamps over the following frames.
      ctx->Yield(8);
      IM_CHECK_EQ(EntryCrystal().upper_h.center, 1.0f);
      IM_CHECK_EQ(EntryCrystal().face_distance[0].center, 1.0f);

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(2);
    };
  }

  // The end-to-end claim the whole column exists for: a grouping BUILT BY CLICKING reaches the
  // geometry the simulator traces. That a sync_group already present in a document survives a save
  // and reaches the simulator is settled off-screen, in composition-correctness; this starts from an
  // all-independent crystal, drives the actual widgets, and then asserts on the mesh.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "edit_modal", "a_sync_group_built_by_clicking_reaches_the_geometry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      const ScopedPopups popup_guard(ctx);
      ctx->Yield(2);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);

      // Randomize all six faces first. Without randomization every face keeps the same default 1.0
      // and "grouped faces are equal" would hold for a build that ignored sync_group entirely.
      for (int i = 0; i < 6; ++i) {
        char rnd_id[64];
        std::snprintf(rnd_id, sizeof(rnd_id), "**/##rnd_Face %d##modal_fd", i + 3);
        ctx->ItemClick(rnd_id);
        ctx->Yield(2);
      }

      // Faces 3/5/7 → group 1, faces 4/6/8 → group 2. Each group is opened with "+ New group" on its
      // first member and joined through "###sync_group_N" after that.
      const int kGroupMembers[2][3] = { { 3, 5, 7 }, { 4, 6, 8 } };
      for (int g = 0; g < 2; ++g) {
        for (int m = 0; m < 3; ++m) {
          char swatch[64];
          std::snprintf(swatch, sizeof(swatch), "**/##sync_Face %d##modal_fd", kGroupMembers[g][m]);
          ctx->ItemClick(swatch);
          ctx->Yield(2);
          if (m == 0) {
            ctx->ItemClick("**/###sync_new");
          } else {
            char item[64];
            std::snprintf(item, sizeof(item), "**/###sync_group_%d", g + 1);
            ctx->ItemClick(item);
          }
          ctx->Yield(2);
        }
      }

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->Yield(3);

      // Two separate claims: the committed entry carries the grouping the clicks described, and the
      // scene handed to core carries the same array. A grouping can be perfectly correct in the
      // GUI's own data and still be dropped on the way to the simulator.
      //
      // Reported per face rather than asserted fatally: which faces disagree is what says whether
      // the grouping was lost or merely mis-assigned.
      const auto& cr = EntryCrystal();
      const auto scene_j = CommitSceneJson(gui::g_state);
      const auto& sg = scene_j["crystal"][0]["shape"]["sync_group"]["face_distance"];
      for (int i = 0; i < 6; ++i) {
        const int expected = (i % 2 == 0) ? 1 : 2;
        if (cr.face_distance[i].sync_group != expected) {
          IM_ERRORF("face %d: committed sync_group is %d, expected %d", i, cr.face_distance[i].sync_group, expected);
        }
        if (cr.face_distance[i].type != gui::ShapeDistType::kUniform) {
          IM_ERRORF("face %d: randomization was dropped on commit", i);
        }
        if (sg[i].get<int>() != expected) {
          IM_ERRORF("face %d: the scene handed to core says sync_group %d, expected %d", i, sg[i].get<int>(), expected);
        }
      }

      // White-box on the geometry: two draws, six faces. The eye cannot verify this symmetry in a
      // halo image, which is the entire reason the feature exists.
      LUMICE_CrystalMesh mesh{};
      IM_CHECK(gui::BuildCrystalMeshData(cr, 12345, &mesh));
      const auto off = PrismFacePlaneOffsets(mesh);
      IM_CHECK_EQ(CountDistinct(off, 1e-5f), (size_t)2);
      IM_CHECK(std::fabs(off[0] - off[2]) <= 1e-5f);
      IM_CHECK(std::fabs(off[2] - off[4]) <= 1e-5f);
      IM_CHECK(std::fabs(off[1] - off[3]) <= 1e-5f);
      IM_CHECK(std::fabs(off[3] - off[5]) <= 1e-5f);
      // The two groups drew separately: a mesh collapsing all six to one value would satisfy every
      // equality above.
      IM_CHECK(std::fabs(off[0] - off[1]) > 1e-5f);

      // Control on the same randomized crystal: ungrouped, the six faces draw six distinct values.
      // Without it, "2 distinct" would also pass on a build that ignored face randomization.
      gui::CrystalConfig ungrouped = cr;
      for (int i = 0; i < 6; ++i) {
        ungrouped.face_distance[i].sync_group = 0;
      }
      LUMICE_CrystalMesh independent{};
      IM_CHECK(gui::BuildCrystalMeshData(ungrouped, 12345, &independent));
      IM_CHECK_EQ(CountDistinct(PrismFacePlaneOffsets(independent), 1e-5f), (size_t)6);
    };
  }
}
