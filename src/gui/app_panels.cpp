#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/aspect_ratio_rules.hpp"
#include "gui/color_window.hpp"
#include "gui/composite_exposure_push.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/defaults_panel.hpp"
#include "gui/destructive_style.hpp"
#include "gui/dock_layout.hpp"
#include "gui/edit_modals.hpp"
#include "gui/field_editor_registry.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_ev_auto.hpp"
#include "gui/gui_logger.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/panels.hpp"
#include "gui/semantic_colors.hpp"
#include "gui/sim_state_rules.hpp"
#include "gui/sun_circle_rules.hpp"
#include "gui/theme.hpp"
#include "gui/window_sizing.hpp"  // SplitViewportForDisplayStrip — the preview/strip split's single owner
#include "imgui.h"
#include "util/path_utils.hpp"  // PathToU8 — the pending export path is shown in the overwrite prompt

// =============================================================================
// GUI window z-order convention (task-gui-window-zorder, scrum-gui-polish-v12)
// -----------------------------------------------------------------------------
// ImGui z-order is decided by two ASYMMETRIC mechanisms (imgui.cpp 6550-6553
// for creation; 12776 for click splice; 12839-12842 for focus flag check):
//   1. First-time Begin (creation): if the window has
//      ImGuiWindowFlags_NoBringToFrontOnFocus, ImGui calls
//      g.Windows.push_front -> the window goes to the FRONT of g.Windows,
//      which is the BOTTOM of the visual z-order (rendered first).
//      Without the flag: g.Windows.push_back -> goes to the BACK of
//      g.Windows = TOP visually. This is the design intent: the flag
//      means "this window is background, never raise it".
//   2. Click / FocusWindow: BringWindowToDisplayFront splices the window to
//      the BACK of g.Windows (visually TOP) — UNLESS the window has
//      NoBringToFrontOnFocus, in which case the splice is skipped.
//
// Practical implication: NoBringToFrontOnFocus simultaneously
//   (a) places the window at the BOTTOM of g.Windows on creation, and
//   (b) freezes it there against click side-effects.
// Windows without the flag float ABOVE the NoBringToFrontOnFocus cluster.
//
// Layered model (from bottom to top):
//
//   Layer 4 (Top, ImGui-managed):
//     - Tooltip / DragDrop overlay (ImGui internal, automatic top)
//     - GetForegroundDrawList (debug overlay; not used in this project)
//
//   Layer 3 (Floating; default raise behavior — NO NoBringToFrontOnFocus,
//            so push_back on creation -> floats above the background cluster):
//     - "Unsaved Changes" (BeginPopupModal), and the other popups beside it:
//       "Save Modified Config", "Overwrite Config File", "Import Warning",
//       "Warning", "Custom Spectrum", the Settings panel.
//       The per-entry "Edit Entry" window used to head this list in both its
//       forms; it is gone — the crystal / axis / filter editors are a page of
//       "##DocumentInspector" in the background cluster below.
//     - "##LogPanel" — user-toggleable; raisable on click; sits naturally
//       above the document column / display strip cluster.
//     - ICON_FA_PALETTE " Colors" (color_window.cpp:508) — user-toggleable
//       floating window. Manual click detection in the background cluster
//       (e.g. RenderEntryCard's IsMouseHoveringRect path) MUST gate on
//       IsWindowHovered() or !io.WantCaptureMouse to avoid click-through
//       when Colors covers the panels beneath it
//       (task-color-window-mouse-capture / 346.2).
//
//   Background cluster (NoBringToFrontOnFocus, push_front on creation
//                       -> bottom of g.Windows):
//     - "##DockHost" — the transparent host carrying the main DockSpace
//       (dock_layout.cpp). Draws nothing itself; the dockspace paints the
//       panel background over everything except its central node.
//     - "##DocumentTree" / "##DocumentInspector" — docked
//       INTO that dockspace. Their position and size come from their dock
//       nodes; they no longer carry NoMove/NoResize because they no longer
//       place themselves. The two share the document column, one above the
//       other, with a native separator between them.
//     - "##TopBar" / "##StatusBar" — fixed top/bottom bars, outside the
//       dockspace, still placed by SetNextPanelGeometry.
//     - "##PreviewPanel" — transparent (NoBackground) and pinned over the
//       dockspace's deliberately-empty central node; the OpenGL preview
//       shader is rendered into this region between ImGui::Render and
//       SwapBuffers in main.cpp.
//     - "##DisplayStrip" — the Grade / Overlays / Components tabs, opaque and
//       pinned to the BOTTOM of that same central node; the preview gives up
//       exactly this band (SplitViewportForDisplayStrip, window_sizing.hpp).
//       Fixed-geometry chrome like the bars, not a dock node — hence NoDocking.
//     Within this cluster, push_front means the LATEST Begin'd window ends
//     up at index 0 (bottom). Visual order within the cluster is therefore
//     the REVERSE of main.cpp Render* call order. Cluster members do not
//     overlap each other, so this internal ordering has no visual effect —
//     and for the three that are now dock nodes / the central node, docking
//     enforces that non-overlap rather than the call order merely happening
//     to produce it. Measured, not assumed: shell_chrome's
//     "the_log_panel_can_come_forward_over_the_side_panels" still reports the
//     log panel above the docked left panel in g.Windows.
//
// -----------------------------------------------------------------------------
// CHECKLIST when adding a new ImGui::Begin window (in this file or elsewhere):
//   1. Register its layer (Background cluster / Layer 3 / Layer 4) in the
//      model above.
//   2. If background cluster: flags MUST include
//      ImGuiWindowFlags_NoBringToFrontOnFocus. The window will be pushed to
//      the front of g.Windows on creation (bottom visually) and frozen
//      there against click splices. Within the cluster, visual stacking is
//      the REVERSE of main.cpp Render* call order — call later to render
//      LOWER. Usually irrelevant because cluster members do not overlap.
//   3. If Layer 3 (floating): do NOT add NoBringToFrontOnFocus. The window
//      will be push_back'd on creation and naturally float above the
//      background cluster, and clicks will splice it to the very top.
//   4. Decide whether it may be docked. Anything that is not one of the
//      dockspace's own panels needs ImGuiWindowFlags_NoDocking: dragged into
//      the DockSpace it would take space from the default layout and have no
//      way back except View -> Reset Layout.
//   5. Code-review must reject any new Begin not registered here, or any
//      main.cpp Render* call order that contradicts this model.
//
// SCOPE of this convention:
// File-level soft constraint. No global automation gate; enforcement relies on
// code-review human inspection. (Whether to promote to CLAUDE.md or a
// clang-tidy check is left to task-closeout decision.)
// =============================================================================

namespace lumice::gui {

using SimState = GuiState::SimState;

namespace {

// The leading label of a field in a HORIZONTAL row — the display strip's inline answer to the
// inspector's PropertyRow. Same visual contract (dimmed text, top-aligned with the frame of the
// control it names, control immediately to its right); what it does not have is the property
// table's fixed label column, because there is no column of rows here to align down.
//
// A nested two-column BeginPropertyTable per field would produce the same pixels and claim an
// alignment that does not exist — the strip's fields sit side by side, so no two of them share a
// vertical line to be aligned on. The label column tier (kPropertyLabelColWidth) is deliberately
// NOT consumed here for the same reason: padding "EV" out to 60 px would put the number that far
// from its own name, which is the proximity trade doc/gui-visual-language.md §5 records as
// falsified.
void InlineFieldLabel(const char* text) {
  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled("%s", text);
  ImGui::SameLine();
}

// With ImGuiConfigFlags_ViewportsEnable (gui-polish-v15), window positions
// and ForegroundDrawList coordinates are in absolute OS screen space, not
// relative to the main GLFW window. All fixed-layout panels must anchor to
// the main viewport's origin to stay inside the host window.
inline ImVec2 MainVpPos(float x, float y) {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  return ImVec2(vp->Pos.x + x, vp->Pos.y + y);
}

// The divider between two clusters of a horizontal chrome row: a one-pixel vertical rule at the
// theme's separator value, drawn where a TextDisabled("|") used to be printed.
//
// Why not the pipe character. A "|" is a glyph, so its weight, its height and the air on either
// side of it are the font's decisions, not the layout's — it sits on the text baseline, it is as
// dark as the dimmed text grade, and at 15 px Roboto it is tall enough to read as a character in a
// sentence. A row of them reads as content with punctuation in it rather than as groups with
// structure between them. The rule is drawn instead: inset from the row's top and bottom, at
// ImGuiCol_Separator (the same low-contrast white the horizontal separators use, i.e. no new
// colour), and one device pixel wide regardless of what the body font does next.
//
// Call it between two items of a SameLine run; it claims its own layout slot and leaves the cursor
// on the same line, so a call site replaces the three-line SameLine / TextDisabled / SameLine
// sequence one-for-one. kHairlineWidth is what a caller measuring a cluster must budget for it.
constexpr float kHairlineWidth = 1.0f;
constexpr float kHairlineInsetY = 3.0f;

void Hairline() {
  ImGui::SameLine();
  const float h = ImGui::GetFrameHeight();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::Dummy(ImVec2(kHairlineWidth, h));
  // +0.5 puts the 1 px line on the pixel centre so it does not land as two half-covered columns.
  ImGui::GetWindowDrawList()->AddLine(ImVec2(p.x + 0.5f, p.y + kHairlineInsetY),
                                      ImVec2(p.x + 0.5f, p.y + h - kHairlineInsetY),
                                      ImGui::GetColorU32(ImGuiCol_Separator));
  ImGui::SameLine();
}

// The divider between two segments of the status bar. A middle dot, not the pipe the row used to
// print: the segments there are readouts rather than groups of controls, and a dot between two
// short phrases separates them without drawing a line through a 28 px row. Same call shape as
// Hairline() — it claims its own slot and leaves the cursor on the line — and kMiddleDot is what a
// caller measuring a cluster must budget for it.
constexpr const char* kMiddleDot = "\xC2\xB7";  // U+00B7, inside the body font's default range

void MiddleDot() {
  ImGui::SameLine();
  ImGui::TextDisabled("%s", kMiddleDot);
  ImGui::SameLine();
}

// Widths of the shapes a chrome row lays out, for a cluster that must be measured before it is
// drawn — which is what right-aligning a run of SameLine items costs, since the run's starting x
// depends on its total width. Each mirrors the geometry of the ImGui call it is named after
// (Button: text + FramePadding.x*2; Checkbox: the square plus, when there is a label, ItemInnerSpacing
// and the label; Text: the text). They exist as named functions rather than open-coded arithmetic
// at the call site because a measurement that drifts from its widget is invisible until the cluster
// lands a few pixels off the window edge.
float ButtonWidth(const char* label) {
  return ImGui::CalcTextSize(label, nullptr, true).x + ImGui::GetStyle().FramePadding.x * 2.0f;
}

float CheckboxWidth(const char* label) {
  const float text_w = ImGui::CalcTextSize(label, nullptr, true).x;
  return ImGui::GetFrameHeight() + (text_w > 0.0f ? ImGui::GetStyle().ItemInnerSpacing.x + text_w : 0.0f);
}

float TextWidth(const char* text) {
  return ImGui::CalcTextSize(text).x;
}

// Pin a chrome panel to the main viewport so ImGui never promotes it to an
// independent OS viewport. Without SetNextWindowViewport, panels that sit at
// the viewport edge (e.g. status bar at the bottom row) may be promoted,
// which makes them appear covered by the host window or float outside it.
inline void SetNextPanelGeometry(float x, float y, float w, float h) {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + x, vp->Pos.y + y));
  ImGui::SetNextWindowSize(ImVec2(w, h));
  ImGui::SetNextWindowViewport(vp->ID);
}

// ================================================================================================
// Execution cluster — the top bar's SECOND row, rendered by RenderExecutionCluster below.
//
// Why it is a block of its own rather than more statements in RenderTopBar: the first row is static
// chrome (buttons that open things), while this row is the only part of the top bar that reads
// simulation state and writes document fields. Keeping the two apart is what lets a reader answer
// "what in the top bar can change while a run is in flight" by looking at one function.
//
// What belongs here is decided by field lifecycle, not by convenience: these controls say how hard
// THIS RUN goes and whether the picture on screen still corresponds to the document. Nothing saved
// with the scene belongs in this row — that is the document column's job
// (doc/gui-layout-architecture.md §1/§3).
// ================================================================================================

// The two numeric controls' widths now come from the shared token table (kRaysControlWidth /
// kCompactFieldWidth, gui_constants.hpp), which is also where the "the row totals ~985 px, so it
// fits inside kMinWindowWidth" arithmetic that picked them is recorded.
//
// Fixed slot for the run-progress readout, so the row does not shift as the run's state changes.
constexpr float kProgressSlotWidth = 130.0f;
// Its thickness. A rule, not a box: thin enough that the shape cannot be mistaken for a frame with
// something typed in it (which is how the full-height bar with a centred percentage read), thick
// enough to stay visible at the row's scale. The percentage it used to carry is in the tooltip.
constexpr float kProgressBarHeight = 4.0f;

void RenderExecutionCluster() {
  const auto& style = ImGui::GetStyle();
  const bool simulating = IsSimulating(g_state.sim_state);
  const bool stopping = IsStopping(g_state.sim_state);

  // ---- Run / Stop ----
  // Fixed width (max of ALL three labels, incl. "Stopping…") to prevent layout shift on toggle.
  const char* kRunLabel = ICON_FA_PLAY " Run";
  const char* kStopLabel = ICON_FA_STOP " Stop";
  const char* kStoppingLabel = ICON_FA_STOP " Stopping...";
  float run_stop_width = std::max({ ImGui::CalcTextSize(kRunLabel).x, ImGui::CalcTextSize(kStopLabel).x,
                                    ImGui::CalcTextSize(kStoppingLabel).x }) +
                         style.FramePadding.x * 2;
  if (simulating) {
    // Stop is a destructive action in the same sense as delete/remove — it shares their palette
    // rather than keeping a second, slightly different red of its own.
    PushDestructiveStyle();
    if (ImGui::Button(kStopLabel, ImVec2(run_stop_width, 0))) {
      DoStop();
    }
    PopDestructiveStyle();
  } else if (stopping) {
    // Async Stop in flight: greyed, disabled "Stopping…" (DoStop is idempotent, but the disabled
    // button makes the in-flight state unambiguous and blocks re-entry at the UI layer).
    ImGui::BeginDisabled();
    ImGui::Button(kStoppingLabel, ImVec2(run_stop_width, 0));
    ImGui::EndDisabled();
  } else {
    PushGoodButtonStyle();
    if (ImGui::Button(kRunLabel, ImVec2(run_stop_width, 0))) {
      DoRun(/*user_initiated=*/true);
    }
    PopGoodButtonStyle();
  }

  // ---- Dirty chip + Revert ----
  // Always rendered for stable layout, hidden when not modified: Alpha=0 + BeginDisabled leaves the
  // area invisible and non-interactive while it still occupies layout space. The hidden area
  // intercepts clicks, which is harmless in this horizontal toolbar context.
  //
  // The chip and Revert are two actions, not one: the chip re-runs with the new configuration, and
  // Revert throws the new configuration away. Merging them would leave no way to do the second.
  //
  // The predicate is `IsModified(sim_state)` and nothing else — deliberately NOT a second list of
  // "which fields count as simulation input". IsModified is fed by ReconcileSimState, whose dirty
  // flag is DiffAgainstCommitBaseline's verdict over the field→tier table (gui_state_tiers.hpp), so
  // the chip and the tier classifier are the same statement read twice. Anything that would make
  // the chip disagree with the classifier is a bug in the classifier, and belongs there.
  //
  // A consequence of that predicate worth stating, because it is a behavior and not an oversight:
  // the chip cannot appear during a run. ReconcileSimState only produces kModified from kDone, so
  // kSimulating/kStopping are never downgraded to it; edits made while a run is in flight are
  // auto-committed to the running server instead (the throttled commit in main.cpp), which is what
  // makes "modified relative to the last completed run" the only question the chip answers.
  const bool modified = IsModified(g_state.sim_state);
  if (!modified) {
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.0f);
  }
  ImGui::BeginDisabled(!modified);
  ImGui::SameLine();
  PushWarningButtonStyle();
  if (ImGui::Button(ICON_FA_CIRCLE_EXCLAMATION " Changed - re-run") && modified) {
    // Same call as the Run button above, not a variant of it: the chip is a second entry point to
    // running, placed where the user is already looking when they notice the result is stale.
    DoRun(/*user_initiated=*/true);
  }
  PopWarningButtonStyle();
  // The wording is source-agnostic on purpose — "configuration changed", not "you added a color
  // class". Main-scene edits and color-class edits reach kModified through the same
  // ReconcileSimState pipeline, so naming either source would be wrong half the time. Shown only
  // when modified, since the row is BeginDisabled(alpha=0) otherwise.
  if (modified && ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Configuration changed since the last run.\n"
        "Click to re-simulate, or Revert to discard the changes.");
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Revert") && modified) {  // `&& modified`: redundant safety guard over BeginDisabled
    DoRevert();
  }
  ImGui::EndDisabled();
  if (!modified) {
    ImGui::PopStyleVar();
  }

  Hairline();

  // ---- Ray budget ----
  RaysBudgetControl(g_state, kRaysControlWidth);

  Hairline();

  // ---- Max hits ----
  // One DragInt, not the [slider][input] pair this used to be: the field's whole domain is a few
  // dozen values, and ctrl+click still types an exact one, so the pair's second half was paying
  // ~90 px of a 1024-px row for a way to enter a number the drag can already reach. Unlike the ray
  // budget beside it, this field has no non-numeric end that needs a track to put a detent on.
  //
  // The label leads the control here, as it does everywhere else in this row, rather than trailing
  // it the way SliderIntWithInput's own label did.
  const FieldEditorConstraint hits_c = ConstraintFor("sim.max_hits", g_state);
  InlineFieldLabel("Max hits");
  ImGui::SetNextItemWidth(kCompactFieldWidth);
  DragIntField("Max hits", &g_state.sim.max_hits, static_cast<int>(hits_c.min_value),
               static_cast<int>(hits_c.max_value));
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Maximum number of crystal face hits per ray path");
  }

  // ---- Backend toggle ----
  // GPU backend toggle (Metal on Apple, CUDA on NVIDIA). Marked dirty explicitly so the next
  // Apply/Run reconstructs the server for the chosen backend (MaybeReconstructServerForBackend in
  // app.cpp) — CPU N-worker vs GPU single engine are different orchestration topologies, so the
  // server is rebuilt and the accumulated image resets on toggle. Falls back to CPU silently if the
  // active config is not GPU-compatible.
  // use_gpu_backend is intentionally excluded from ConfigSnapshot (session/view field, see
  // gui_state.hpp field-sync scope comment), so it cannot participate in the reconciler auto-diff —
  // the manual MarkDirty call below is the T0 documented exception.
  // Runtime gate: only show the checkbox when a GPU backend is actually available (Metal device on
  // Apple / NVIDIA device + usable CUDA on Windows-Linux), so it never appears on CPU-only hosts or
  // machines with very old hardware / broken GPU drivers, where selecting it would otherwise fail
  // in EnsureDevice. The probe is cached, so the per-frame cost is a plain memory read.
  if (LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL) || LUMICE_IsBackendAvailable(LUMICE_BACKEND_CUDA)) {
    Hairline();
    // Disable the toggle while busy (simulating OR async Stop draining): the backend switch
    // reconstructs the server on the next DoRun, and an in-flight stop still holds it (R1).
    const bool busy = IsBusy(g_state.sim_state);
    ImGui::BeginDisabled(busy);
    if (Checkbox("Use GPU", &g_state.use_gpu_backend)) {
      g_state.MarkDirty();
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Use the GPU for simulation (falls back to CPU if incompatible)");
    }
  }

  // ---- Run progress ----
  // A 4 px rule, filled with the accent, over a neutral track. Two readings it had to lose, both
  // of which were accidents rather than decisions: it drew in ImGui's default amber, because
  // ImGuiCol_PlotHistogram was never claimed by the theme and amber is this app's WARNING grade
  // (semantic_colors.hpp), so a healthy run read as a problem; and a full-height frame with a
  // percentage centred in it reads as a text field the user could type into. Accent is the right
  // grade here under "emphasis only while an interaction is in progress"
  // (doc/gui-visual-language.md §4.3) — a run in flight is that case.
  //
  // The infinite tier now gets the same rule, in ImGui's indeterminate mode. It used to get NO
  // slot at all, and the argument for that was entirely about TEXT: a bar with no denominator has
  // to lie with a percentage, and the ray-budget control three slots to the left already prints
  // "until stopped", so a second copy read as a rendering fault. Neither half survives this shape
  // — the rule carries no number and no words, so it repeats nothing and claims no denominator.
  // What it does say is "this is running", which for an unbounded run is the one thing the row
  // cannot otherwise show: the ray budget sits still and the percentage that would move does not
  // exist. It animates only while the run is actually in flight; idle, the track sits empty rather
  // than sliding forever under a stopped simulation.
  //
  // The percentage moved into the tooltip. At 4 px ImGui's overlay text is clipped to unreadable,
  // and the two cannot both be had — a bar tall enough to hold 15 px type is the frame this shape
  // was getting away from.
  {
    Hairline();
    const bool infinite = g_state.sim.infinite;
    const double target = static_cast<double>(g_state.sim.ray_num_millions) * 1e6;
    const double done = static_cast<double>(g_state.stats_sim_ray_num);
    const float finite_fraction = target > 0.0 ? static_cast<float>(std::clamp(done / target, 0.0, 1.0)) : 0.0f;
    // ImGui reads a negative fraction as "indeterminate, animated by this value" — hence the clock.
    const float fraction =
        infinite ? (IsBusy(g_state.sim_state) ? -static_cast<float>(ImGui::GetTime()) : 0.0f) : finite_fraction;

    // The track colour is pushed locally rather than set in the theme: the global FrameBg is the
    // blue an input field is filled with, which is precisely the reading this shape had to lose,
    // and changing it globally would repaint every real input in the app. ChildBg is the palette's
    // neutral step above the window background — a groove, not a second widget.
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetStyleColorVec4(ImGuiCol_ChildBg));
    // Centre the rule in the row: at 4 px it would otherwise hang off the top of a 21 px line.
    // Safe because this is the last item in the row — nothing after it inherits the offset.
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (ImGui::GetFrameHeight() - kProgressBarHeight) * 0.5f);
    ImGui::ProgressBar(fraction, ImVec2(kProgressSlotWidth, kProgressBarHeight), "");
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) {
      if (infinite) {
        ImGui::SetTooltip("Running until stopped: this run has no ray budget to measure progress against.");
      } else {
        ImGui::SetTooltip("Rays traced so far against this run's ray budget: %.0f%%",
                          static_cast<double>(finite_fraction) * 100.0);
      }
    }
  }
}
}  // namespace

void RenderTopBar(float window_width) {
  SetNextPanelGeometry(0, 0, window_width, kTopBarHeight);
  ImGui::Begin("##TopBar", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);

  // ---- Row 1: chrome. Documents, features, windows — nothing here reads simulation state except
  // the `busy` gate on the file operations, and nothing here writes a document field.
  //
  // Left-panel collapse toggle owns the leftmost slot of the top bar so it can never overlap with
  // panel-internal headers.
  {
    const char* left_toggle_label = g_state.left_panel_collapsed ? ICON_FA_CHEVRON_RIGHT "##left_panel_toggle" :
                                                                   ICON_FA_CHEVRON_LEFT "##left_panel_toggle";
    if (ImGui::Button(left_toggle_label)) {
      g_state.left_panel_collapsed = !g_state.left_panel_collapsed;
    }
    Hairline();
  }

  // `busy` gates the file operations: New/Open/Save stay disabled while the backend is still
  // draining an async Stop (kStopping), not just while simulating.
  const bool busy = IsBusy(g_state.sim_state);

  // File operations — New/Open disabled while busy (simulating OR async Stop draining); Save menu
  // itself stays enabled so read-only exports (Screenshot / Dual Fisheye Equal Area /
  // Equirectangular / Config JSON) remain reachable.
  ImGui::BeginDisabled(busy);
  if (ImGui::Button("New")) {
    CheckUnsavedAndDo(PendingAction::kNew);
  }
  ImGui::SameLine();
  if (ImGui::Button("Open")) {
    CheckUnsavedAndDo(PendingAction::kOpen);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  {
    if (ImGui::Button("Save")) {
      ImGui::OpenPopup("SaveMenu");
    }
    if (ImGui::BeginPopup("SaveMenu")) {
      bool no_texture = !g_preview.HasTexture();
      bool has_server = g_server != nullptr && g_state.sim_state != GuiState::SimState::kIdle;
      ImGui::BeginDisabled(busy);
      if (ImGui::MenuItem("Save")) {
        DoSave();
      }
      if (ImGui::MenuItem("Save Copy")) {
        DoSaveAs();
      }
      ImGui::EndDisabled();
      ImGui::Separator();
      if (ImGui::MenuItem("Screenshot...", nullptr, false, !no_texture)) {
        DoExportPreviewPng();
      }
      if (ImGui::MenuItem("Dual Fisheye Equal Area...", nullptr, false, has_server)) {
        DoExportDualFisheyeEqualAreaPng();
      }
      if (ImGui::MenuItem("Equirectangular...", nullptr, false, has_server)) {
        DoExportEquirectangularPng();
      }
      if (ImGui::MenuItem("Config JSON...")) {
        DoExportConfigJson();
      }
      ImGui::Separator();
      // Personal defaults are NOT in this menu. They used to be, on the reasoning that "what a new
      // document starts from" is a file-scope decision and so belongs with the other export
      // commands. The reasoning was sound and the placement still failed the only test that
      // matters: a user looking for their settings does not open a Save menu to find them, and in
      // practice did not. The entry is now a Settings button in the top bar — see RenderTopBar.
      ImGui::MenuItem("Include Texture in .lmc", nullptr, &g_state.save_texture);
      ImGui::MenuItem("Include Overlay in Screenshot", nullptr, &g_state.screenshot_include_overlay);
      ImGui::EndPopup();
    }
  }

  // ---- The row's right cluster: what OPENS things (windows, menus, the settings modal) ----
  //
  // It is flush to the window's right edge, with the file operations left where they are and open
  // space between the two. That space is the grouping: two runs of buttons at opposite ends of a
  // row read as two kinds of control without a divider having to say so, which is what the row was
  // doing with pipe characters before.
  //
  // Right-aligning a SameLine run means knowing where it starts, which means measuring it before
  // drawing it. Every conditional item is measured under the same predicate it is drawn under —
  // the two lists below and further down must stay in step, and a cluster measured for one state
  // and drawn in another lands off the edge by exactly the width of whatever was missed. That is
  // what test/gui/functional/test_shell_chrome.cpp's alignment case is for; it exercises the
  // conditional members in both of their states rather than only the default one.
  const bool has_color_classes = !g_state.raypath_color.empty();
  bool composite_now = false;
  bool composite_empty = false;
  std::string composite_toggle_id;
  if (has_color_classes) {
    // The shared signal cache is read BEFORE the toggle is rendered, so the control can be wrapped
    // in BeginDisabled() when the composite would be empty — otherwise it appears "unclickable /
    // not responding" with no visual explanation. Reading it here costs nothing: the poll behind it
    // is throttled to 500 ms and is unaffected by call-site order, and the Colors window and the
    // aggregate pip read the same source, so all three cannot disagree.
    //
    // "The composite would be empty" is one predicate (NoVisibleMatchedColorClass) covering two
    // ways of getting there: no rays match any configured class, OR every matching class is
    // currently hidden (visible=false, or solo'd out by another class). It has a single owner,
    // shared with the Colors-window Enable checkbox, for that reason — two indicators computing
    // "empty" separately is two chances to say different things about one composite.
    composite_now = g_state.last_uploaded_as_composite;
    const std::vector<int>& signal_flags = RefreshColorClassSignals(g_state, g_server);
    composite_empty = NoVisibleMatchedColorClass(g_state, signal_flags);
    composite_toggle_id = std::string(composite_now ? "Colored" : "Full Spectrum") + "##CompositePreviewToggle";
  }
  {
    const ImGuiStyle& style = ImGui::GetStyle();
    const float gap = style.ItemSpacing.x;
    float cluster_w = ButtonWidth(ICON_FA_PALETTE " Colors");
    if (has_color_classes) {
      cluster_w += gap + CheckboxWidth(composite_toggle_id.c_str());
      if (composite_empty) {
        cluster_w += gap + TextWidth(ICON_FA_TRIANGLE_EXCLAMATION);
      }
    }
    cluster_w += gap + kHairlineWidth + gap + ButtonWidth(ICON_FA_GEAR " Settings");
    cluster_w += gap + ButtonWidth("View");
    ImGui::SameLine(ImGui::GetWindowWidth() - cluster_w - style.WindowPadding.x);
  }

  // task-345.5 (⑥): dedicated "feature button" group, immediately right of
  // New/Open/Save. Colors is the first occupant; future cross-cutting toggles
  // unrelated to file I/O or panel layout should land here rather than
  // competing for status-bar space.
  //
  // task-gui-feedback-affordances Step 1 (AC2): visually distinguish "has color
  // classes" vs "no color classes" so the topbar signals whether coloring is
  // configured before the window is opened. Derived state (empty check on
  // raypath_color) — no new state source. Blue/purple tint avoids collision
  // with Run (green) / Stop (red).
  const bool tint_colors_button = ShouldTintColorsButton(g_state.raypath_color.empty());
  if (tint_colors_button) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.30f, 0.35f, 0.65f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.40f, 0.45f, 0.75f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.25f, 0.28f, 0.55f, 1.0f));
  }
  if (ImGui::Button(ICON_FA_PALETTE " Colors")) {
    // task-348.3 AC3 (⑦): apply the "default enable on open with no classes" rule
    // ONLY on the false→true transition of color_window_open. Doing it here (inside
    // the click branch, guarded by `opening`) — not per-frame inside RenderColorWindow
    // — is what makes the behavior "memory-preserving": the user can still manually
    // toggle Colored off after opening, and subsequent focus changes / clicks that do
    // not close-then-reopen the window will not overwrite that choice.
    const bool opening = !g_state.color_window_open;
    g_state.color_window_open = !g_state.color_window_open;
    if (opening && ShouldDefaultEnableColorsOnOpen(g_state.raypath_color.empty())) {
      g_state.show_composite_preview = true;
    }
  }
  if (tint_colors_button) {
    ImGui::PopStyleColor(3);
  }

  // task-colored-toggle-to-topbar (346.3): colored/full-spectrum display-time
  // toggle, relocated from the status bar (task-345.4) to sit next to Colors.
  // Gated identically (raypath_color non-empty) so it renders only when at
  // least one color class exists (AC4: no color classes ⇒ no checkbox,
  // matching pre-346.3 status-bar behavior). Label/checked-state reads
  // GROUND TRUTH (last_uploaded_as_composite), click writes the user
  // preference (show_composite_preview) — same read/write split as
  // gui_state.hpp:794-803. The checkbox living in ##TopBar (a window
  // independent of the Colors window's own render call) is itself the
  // persistent "currently in colored mode" marker required by AC3: closing
  // Colors does not touch this window.
  if (has_color_classes) {
    ImGui::SameLine();
    // task-349.3 (#4): revert 348.3 icon-only Button back to a plain-text Checkbox
    // (no ICON_FA_PALETTE prefix) so this display-time toggle reads visually
    // distinct from the icon-bearing "Colors" open-window button one slot to the
    // left (owner-rejected: two adjacent palette-icon controls confused which was
    // the toggle vs. the window opener). Semantic split unchanged from 345.4/346.3:
    // label + `checked` read GROUND TRUTH (last_uploaded_as_composite), click
    // writes via shared ToggleCompositePreview(g_state) — the shared writer was
    // introduced in 348.3 and stays after the widget-shape revert (a12: two
    // control sites, one write path).
    //
    // The state this block reads (composite_now / composite_empty / the label, hence the widget's
    // width) is computed above, where the cluster is measured: the measurement and the drawing
    // must see the same values, and the toggle's label alternates between "Colored" and
    // "Full Spectrum", which are not the same width.
    //
    // Style tokens: Checkbox renders as frame background + check mark, not a
    // button surface — accent must go on FrameBg/FrameBgHovered/CheckMark. Using
    // ImGuiCol_Button here would silently no-op (this was the 346.3→348.3 pitfall
    // recorded in learnings/code-quality.md; reverting the widget type must
    // re-swap the token set).
    bool checked = composite_now;
    if (composite_now) {
      ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.35f, 0.55f, 0.85f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.45f, 0.65f, 0.95f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    }
    ImGui::BeginDisabled(composite_empty);
    if (Checkbox(composite_toggle_id.c_str(), &checked)) {
      ToggleCompositePreview(g_state);
    }
    ImGui::EndDisabled();
    if (composite_now) {
      ImGui::PopStyleColor(3);
    }
    // Tooltip logic (both enabled and disabled cases): AllowWhenDisabled so the
    // BeginDisabled() wrapper does not eat the hover. When disabled, show the
    // "no matches" reason (a12: shared string with the Colors-window mirror);
    // when enabled, show the existing "Currently: Colored / Full Spectrum" hint.
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      if (composite_empty) {
        ImGui::SetTooltip("%s", kColorsDisabledNoMatchTooltip);
      } else {
        ImGui::SetTooltip(
            "Toggle colored composite / full-spectrum preview.\n"
            "Display-time only -- does not re-simulate or discard color classes.\n"
            "%s",
            composite_now ? "Currently: Colored" : "Currently: Full Spectrum");
      }
    }

    // task-348.1 Step 3 (① 反馈缺失): when every configured color class has no
    // matching rays, the Colored composite is empty and the button above appears
    // "unclickable / non-responding" (last_uploaded_as_composite never flips true).
    // Surface an aggregate warning pip here so the user sees WHY nothing changes —
    // reads the same shared signal cache as the Colors window's per-row warnings
    // (single source, a12), so both indicators agree by construction. Silent when
    // no class has non-empty match[] (matches per-row semantics).
    //
    // task-349.2 Step 3: the pip stays alongside the disabled button (both driven
    // by composite_empty) — the two indicators are complementary, not redundant:
    // the button greying is the immediate visual cue "cannot toggle now", the
    // pip is the persistent per-row / aggregate warning surface.
    if (composite_empty) {
      ImGui::SameLine();
      ImGui::PushStyleColor(ImGuiCol_Text, WarningTextColor());
      ImGui::TextUnformatted(ICON_FA_TRIANGLE_EXCLAMATION);
      ImGui::PopStyleColor();
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "No visible color class currently matches any rays -- the composite would be empty.\n"
            "Either no rays match (they may be blocked by a physical filter), or every matching\n"
            "class is currently hidden. Open the Colors window to inspect per-class details.");
      }
    }
  }

  // Personal defaults, promoted out of the Save menu. The panel it opens edits what a NEW document
  // starts from, so by file-scope logic it belonged with the export commands — but it was reached
  // by nobody who had not been told where it was, which is the only measure a settings entry has.
  //
  // Two deliberate choices, both worth knowing before this button gets tidied away again:
  //   - It is separated from the Colors group by a hairline — the only divider left inside this
  //     cluster — because the two are different kinds of control. Colors and the Colored checkbox
  //     are toggles that change what the viewport shows; this opens a modal that edits persistent
  //     preferences. Sharing a run of buttons with no break would let a modal launcher read as one
  //     more view toggle. View needs no second hairline: it opens a menu about the window's own
  //     layout, which is the same kind of thing this button opens.
  //   - It is NOT gated on `busy`. Reading and editing personal defaults is independent of whether
  //     a simulation is running, same as Colors — a settings entry that disappears while the thing
  //     the user is watching runs is a settings entry they cannot find when they think to look.
  // The panel's preset library is one section inside it, so retuning a preset is still reachable;
  // it no longer has a menu item of its own pointing straight at that section.
  Hairline();
  if (ImGui::Button(ICON_FA_GEAR " Settings")) {
    OpenDefaultsPanel(g_state, DefaultsPanelSection::kSettings);
  }

  // View menu. It exists because panel geometry became user-editable (the panels are dock nodes and
  // their splitters are draggable, and the arrangement persists across runs) — "I dragged the layout
  // into a state I cannot undo" is the one objection to persisting it, and this is the answer to it.
  // Same popup-button shape as Save above rather than a real menu bar, which the top bar has never
  // had.
  ImGui::SameLine();
  if (ImGui::Button("View")) {
    ImGui::OpenPopup("ViewMenu");
  }
  if (ImGui::BeginPopup("ViewMenu")) {
    if (ImGui::MenuItem("Reset Layout")) {
      RequestDockLayoutReset();
      // A reset that left the column collapsed would not be a reset: collapse is view state, and the
      // rebuilt layout restores the column to its default width regardless. Clearing the flag here
      // keeps the marker and the geometry from disagreeing. The collapse-tracking in
      // RenderDocumentTree sees this as an ordinary expand and asks for the width the rebuild
      // already produced, so the two agree rather than fight.
      g_state.left_panel_collapsed = false;
    }
    ImGui::EndPopup();
  }

  // ---- Row 2: the execution cluster. No SameLine, so it starts on a fresh line; kTopBarHeight is
  // sized for exactly these two rows (gui_constants.hpp).
  RenderExecutionCluster();

  ImGui::End();
}

namespace {
constexpr float kCollapseBtnSize = 20.0f;

// The expand button of a collapsed side panel: an ordinary ImGui button, vertically centred in the
// panel's own (now kCollapseBtnSize-wide) window. Call between that window's Begin and End.
//
// It used to be drawn straight to the ForegroundDrawList, with hit-testing done by hand and gated on
// `!io.WantCaptureMouse` — a stand-in for "no floating window is over the strip", which worked only
// because nothing else was submitted where the strip sat. Under docking the strip's rectangle always
// belongs to a dock node, so that gate is false whenever the pointer is on the button and the panel
// could never be brought back. Letting ImGui hit-test a real widget answers the original question
// (is something above this?) properly, by z-order, instead of by proxy.
void RenderCollapsedStrip(const char* btn_label, bool* collapsed) {
  // Centre the square button in the strip; the window itself supplies the strip's background, so
  // there is nothing left to draw by hand.
  const float btn_y = (ImGui::GetWindowHeight() - kCollapseBtnSize) * 0.5f;
  ImGui::SetCursorPos(ImVec2(0.0f, btn_y));
  if (ImGui::Button(btn_label, ImVec2(kCollapseBtnSize, kCollapseBtnSize))) {
    *collapsed = false;
  }
}

// Which collapse state has already been written to which node. Per panel, kept by the caller.
//
// `restore_extent` is what the node measured along the folding axis the moment it was folded, and
// it is why unfolding does not have to guess. For the side panels the guess would merely be stale
// (a dragged width snapping back to the default); for the document column's halves there is no
// constant to guess WITH — their expanded height is a fraction of a column whose own height is the
// window's. Zero means "never folded on this node", which is the state after a layout rebuild hands
// out new IDs, and then the caller's default is the only answer available.
struct PanelCollapseTracker {
  ImGuiID node_id = 0;
  bool applied = false;
  float restore_extent = 0.0f;
};

// Which extent of a dock node a collapse writes. The document column's two halves fold along the
// other axis from the side panels, and the arithmetic is otherwise identical — see ApplyPanelCollapse.
enum class CollapseAxis { kWidth, kHeight };

// A collapsed panel's extent along the folding axis. Width: the strip that holds the chevron that
// brings it back. Height: enough for the half's header row, which is what stays visible when a half
// folds — the column keeps saying what is folded, where a bare 20 px strip would not.
constexpr float kFoldedHalfHeight = 26.0f;

// A panel's collapsed/expanded state is the size of its dock node along one axis. Four things about
// the shape of this:
//   - It writes only on a transition, never every frame. A per-frame DockBuilderSetNodeSize would
//     silently undo a splitter drag on the very next frame, i.e. the panels would look resizable and
//     not be.
//   - The node ID is read fresh from dock_layout on every call, never cached: a Reset Layout
//     rebuilds the tree and hands out new IDs, and resizing the old one would silently do nothing.
//   - When the node changes (first frame, or a rebuild), the marker is seeded from the layout rather
//     than from a default. The interactive app persists the dock tree but not GuiState, so a session
//     that quit with a panel collapsed comes back with a strip-wide node and a flag that says
//     expanded; a marker starting at "expanded" would agree with the flag, see no transition, and
//     leave the panel rendering its full content inside a 20 px column.
//   - The height it writes back comes from the NODE, not from the calling window. For the right
//     panel the two are the same number; for the document column they are not — its node is the
//     split parent of the tree and the inspector, and the tree window's own height is one half of
//     it. Writing that half back would shrink the column a little further on every collapse. A
//     node that cannot be measured (height 0, i.e. the id no longer resolves) leaves the marker
//     untouched so the transition is retried rather than swallowed.
// One implementation for both axes rather than a width version and a near-copy height version.
// Nothing in the body was specific to width: the folding extent and the extent carried through
// unchanged are the only two quantities, and which of them is x is a parameter. (plan §3 point 3
// asked for this evaluation explicitly — the merge is clean, so there is no second copy to keep in
// step.)
void ApplyPanelCollapse(CollapseAxis axis, ImGuiID node_id, bool collapsed, float expanded_extent,
                        PanelCollapseTracker* tracker) {
  if (node_id == 0) {
    return;
  }
  const bool horizontal = axis == CollapseAxis::kWidth;
  const float folded_extent = horizontal ? kCollapseBtnSize : kFoldedHalfHeight;
  const float along = horizontal ? GetPanelNodeWidth(node_id) : GetPanelNodeHeight(node_id);
  if (tracker->node_id != node_id) {
    tracker->node_id = node_id;
    tracker->applied = along <= folded_extent;
  }
  if (collapsed == tracker->applied) {
    return;
  }
  const float across = horizontal ? GetPanelNodeHeight(node_id) : GetPanelNodeWidth(node_id);
  if (across <= 0.0f) {
    return;
  }
  tracker->applied = collapsed;
  float new_along;
  if (collapsed) {
    tracker->restore_extent = along;
    new_along = folded_extent;
  } else {
    new_along = tracker->restore_extent > folded_extent ? tracker->restore_extent : expanded_extent;
  }
  ResizePanelNode(node_id, horizontal ? ImVec2(new_along, across) : ImVec2(across, new_along));
}

// Shared by both side panels. NoMove / NoResize are gone: position and size are the dock node's job
// now, and leaving them on would be a claim about geometry this code no longer makes.
// NoBringToFrontOnFocus is kept — the panels stay in the background cluster described at the top of
// this file, below every floating window.
constexpr ImGuiWindowFlags kSidePanelBaseFlags =
    ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus;

// A document-column half's section header, and the control that folds it. Returns true on the frame
// the user asked to toggle the fold.
//
// It doubles as the page title, which for the inspector is load-bearing: the inspector is the only
// thing on screen that says which item the controls below are editing — the tree's highlight says
// it too, but the tree can be scrolled away from the selected row, or folded flat, while the
// inspector still shows it.
//
// The whole row is the hit target rather than just the chevron (doc/gui-layout-architecture.md §5:
// clicking the section header folds the section). A full-width Selectable gives that, and gives the
// hover highlight that says the row is a target at all.
//
// `id` is a fixed string per half, kept out of the visible label with `###`, so the identity does
// not move when the chevron flips or the crystal page retitles itself on every selection.
bool RenderHalfFoldHeader(const char* icon, const char* text, const char* id, bool folded) {
  char label[96];
  snprintf(label, sizeof(label), "%s  %s  %s###%s", folded ? ICON_FA_CHEVRON_RIGHT : ICON_FA_CHEVRON_DOWN, icon, text,
           id);
  const bool toggled = ImGui::Selectable(label);
  ImGui::Separator();
  return toggled;
}

// Height to give a half when it unfolds and the tracker has nothing remembered — the default split
// of whatever the column measures right now. Not a constant, because the column's height is the
// window's: a fixed number would be wrong on every window size but one.
float DefaultHalfHeight(bool inspector) {
  const float column_h = GetPanelNodeHeight(GetPanelNodeIds().left);
  const float ratio = inspector ? kDocumentInspectorHeightRatio : 1.0f - kDocumentInspectorHeightRatio;
  return column_h * ratio;
}
}  // namespace

void RenderDocumentTree() {
  static PanelCollapseTracker s_collapse;

  if (g_state.left_panel_collapsed) {
    // The tree window is still submitted, holding the strip-wide column, rather than skipped: a
    // docked window that stops submitting makes its node invisible, and the neighbours take the
    // space back the same frame — the strip would end up on top of the preview instead of beside
    // it. The INSPECTOR is skipped while collapsed (see RenderDocumentInspector), and that is the
    // same mechanism used deliberately: its neighbour is the tree, one node over, so the space it
    // gives up stays inside the column and the strip holds one chevron rather than two.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin(kDocumentTreeWindowName, nullptr,
                 kSidePanelBaseFlags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    // Collapse resizes the column's PARENT node, not this window's own node — see
    // DockPanelNodeIds. Consequence, by design: a resize written here lands on the dock node
    // starting next frame, like every other transition-triggered ResizePanelNode call.
    ApplyPanelCollapse(CollapseAxis::kWidth, GetPanelNodeIds().left, true, kLeftPanelWidth, &s_collapse);
    RenderCollapsedStrip(ICON_FA_CHEVRON_RIGHT, &g_state.left_panel_collapsed);
    ImGui::End();
    return;
  }

  // Pick-mode: Esc cancels (read here before any ImGui::Begin so the key event
  // isn't consumed by inner widgets first).
  bool pick_active_at_entry = g_state.pick_link_source.has_value();
  if (pick_active_at_entry && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
    g_state.pick_link_source.reset();
  }
  // Remember whether pick was active at the start of this frame so we can
  // detect "pick just completed" at the bottom and re-select the source entry.
  std::optional<GuiState::EntryRef> pick_source_at_entry =
      pick_active_at_entry ? g_state.pick_link_source : std::nullopt;

  // Arming pick unfolds the halves, because the tree's rows ARE the click targets pick is waiting
  // for. Without this, "Link to..." pressed from a page while the tree is folded arms a mode whose
  // only exit is Esc — the thing the user was told to click is not on screen.
  if (pick_active_at_entry) {
    g_state.FoldDocumentHalves(false, false);
  }

  ImGui::Begin(kDocumentTreeWindowName, nullptr,
               kSidePanelBaseFlags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ApplyPanelCollapse(CollapseAxis::kWidth, GetPanelNodeIds().left, false, kLeftPanelWidth, &s_collapse);

  // The half's own fold, distinct from the whole-column collapse handled above: this one gives the
  // height to the inspector rather than to the preview.
  static PanelCollapseTracker s_fold;
  if (RenderHalfFoldHeader(ICON_FA_LIST, "Document", "tree_fold", g_state.document_tree_folded)) {
    g_state.FoldDocumentHalves(!g_state.document_tree_folded, false);
  }
  ApplyPanelCollapse(CollapseAxis::kHeight, GetPanelNodeIds().document_tree, g_state.document_tree_folded,
                     DefaultHalfHeight(/*inspector=*/false), &s_fold);
  if (g_state.document_tree_folded) {
    // Everything below is skipped, header included in what stays: the node is now a strip the
    // height of that one row, and submitting rows into it would only give the window a scrollbar
    // over content nobody asked to see.
    ImGui::End();
    return;
  }

  // Pick-mode hint bar — render above the scroll area so the user always sees
  // the active-pick state and the Esc instruction. The actual click target is
  // each entry row (handled inside RenderEntryRow).
  if (pick_active_at_entry) {
    const auto& src = *g_state.pick_link_source;
    ImGui::PushStyleColor(ImGuiCol_Text, WarningTextColor());
    ImGui::TextWrapped("Pick mode: click an entry to share crystal/filter from Layer %d / Entry %d (Esc to cancel)",
                       src.layer_idx, src.entry_idx);
    ImGui::PopStyleColor();
    ImGui::Separator();
  }

  // ---- Layout: tree rows (scroll) + toolbar ----
  float avail_h = ImGui::GetContentRegionAvail().y;
  auto& style = ImGui::GetStyle();
  float toolbar_h = ImGui::GetFrameHeight() + style.ItemSpacing.y;
  float rows_h = std::max(0.0f, avail_h - toolbar_h);

  // Process thumbnail update queue before rendering rows
  g_thumbnail_cache.ProcessUpdateQueue(g_state, kMaxThumbnailUpdatesPerFrame);

  // ---- Row scroll area (fills panel above the toolbar) ----
  ImGui::BeginChild("##TreeScroll", ImVec2(0, rows_h), ImGuiChildFlags_None);
  RenderDocumentTreeRows(g_state);
  ImGui::EndChild();

  // ---- Bottom toolbar: add layer only (per-layer delete lives on the header row) ----
  ImGui::Spacing();
  if (ImGui::SmallButton("+ Layer")) {
    // Bind the new layer's seed entry to a fresh pool slot — see panels.cpp's
    // "+ Crystal" handler for the same rationale (avoid implicit link to slot 0).
    EntryCard new_entry;
    new_entry.crystal_id = static_cast<int>(g_state.crystals.size());
    g_state.crystals.emplace_back();
    Layer new_layer;
    new_layer.entries.push_back(new_entry);
    // Footgun #2 guard: the layer that WAS the last layer is about to become
    // an intermediate layer. If its prob is (near-)zero, no rays will reach the
    // new layer — the newly added layer would be silently dead. Promote to a
    // sensible continuation default so the user sees rays in the new layer by
    // default. This is a user-initiated state transition (not a load), so it
    // is not covered by the "don't silently rewrite loaded values" rule.
    // Zero-detection MUST use IsProbZero (same helper as panels.cpp) — a raw
    // == 0.0f would let a slider-dragged 1e-7 sneak past this promotion.
    if (!g_state.layers.empty() && IsProbZero(g_state.layers.back().probability)) {
      g_state.layers.back().probability = kDefaultContinuationProb;
    }
    g_state.layers.push_back(std::move(new_layer));
    g_thumbnail_cache.OnLayerStructureChanged();
  }

  // Pick-mode cancel: blank area / panel-switch click.
  // If pick is still active after the rows are rendered (no row's Selectable consumed the click), a
  // left mouse click anywhere cancels pick. Covers clicking blank space in the tree, the inspector,
  // the right panel, or any non-row widget. Esc was handled at frame start.
  if (g_state.pick_link_source.has_value() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    g_state.pick_link_source.reset();
    pick_source_at_entry.reset();  // suppress the spurious re-select below
  }

  // Pick-mode completion: if pick was active at frame entry but is now reset (cleared by
  // RenderEntryRow's pick-click handler), put the selection back on the SOURCE entry so the user
  // resumes editing where they started — the inspector is showing whatever the selection names, and
  // leaving it on the row that was merely clicked as a model would silently move the edit target.
  // The source entry's crystal_id was just re-bound to the clicked row's crystal, so also reset the
  // singleton trackball view to that crystal's default orientation — otherwise the inspector's
  // preview keeps the old crystal's rotation while the thumbnail (which always renders from the
  // entry's axis distribution) shows the new one. Cancel paths (Esc / blank-area click) clear
  // pick_source_at_entry and skip this branch, so the view reset only fires when a link was
  // actually applied.
  if (pick_source_at_entry.has_value() && !g_state.pick_link_source.has_value()) {
    const auto& src = *pick_source_at_entry;
    const auto& editing_entry = g_state.layers[src.layer_idx].entries[src.entry_idx];
    ResetCrystalViewToCrystal(g_state.crystals[editing_entry.crystal_id]);
    g_state.SelectCrystal(src.layer_idx, src.entry_idx);
  }

  ImGui::End();
}

namespace {

// The inspector's Camera page: which lens the sky is projected through, which hemisphere is
// shown, and where the observer is pointed. This used to be the right panel's "View" group. It
// moved because these fields describe the DOCUMENT — they are saved with the scene and a change
// to any of them is a different picture of the same simulation — which is the line the whole
// column is drawn along (doc/gui-layout-architecture.md §2). Only the host changed: every gate
// below is still the field editor registry's answer, not this call site's.
void RenderCameraControls() {
  auto& r = g_state.renderer;

  ImGui::SeparatorText("Lens");
  if (BeginPropertyTable("##cam_lens")) {
    // Use BeginCombo + Selectable to honour kLensTypePresentationOrder (gui_state.hpp).
    // The enum value (r.lens_type) is preserved unchanged; only the display order differs.
    PropertyRow("Lens Type");
    if (ImGui::BeginCombo("##Lens Type##view", kLensTypeNames[r.lens_type])) {
      for (int idx : kLensTypePresentationOrder) {
        bool selected = (r.lens_type == idx);
        if (ImGui::Selectable(kLensTypeNames[idx], selected)) {
          // The lens switch and its pose fix-ups live in gui_state.hpp, shared with the defaults
          // panel's per-row lens editor. .lmc loading and tests bypass both controls by writing
          // lens_type directly, so they keep their fov.
          ApplyLensTypeSelection(r, idx);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    // Domain, format and disabled-when all come from the field editor registry rather than being
    // written here — same for every control below whose field is a registered document leaf. The
    // bound is the lens' own MaxFov (the registry calls LUMICE_MaxFov), and `enabled` is the
    // full-sky gate that used to be spelled `BeginDisabled(full_sky)` at this line.
    const FieldEditorConstraint fov_c = ConstraintFor("renderer.fov", g_state);
    PropertyRow("FOV");
    ImGui::BeginDisabled(!fov_c.enabled);
    DragFloatField("FOV##view", &r.fov, static_cast<float>(fov_c.min_value), static_cast<float>(fov_c.max_value),
                   fov_c.fmt, fov_c.scale);
    ImGui::EndDisabled();
    EndPropertyTable();
  }

  const bool is_globe = (r.lens_type == kLensTypeGlobe);

  ImGui::SeparatorText("Visibility");
  // Same registry query as the FOV control above, for the same reason. What used to stand here was
  // a hand-paired nest — `BeginDisabled()` under `full_sky` on the outside, `BeginDisabled(
  // is_globe)` under `!full_sky` on the inside — whose NET effect each widget saw had to be read
  // off the interleaving of four `if`s. The two gates are already registered (renderer.visible →
  // NotUnderFullSky, renderer.front → NotUnderFullSkyOrGlobe), and each already folds the
  // full-sky case in, so the call site needs no nesting: one flat pair per field.
  if (BeginPropertyTable("##cam_visibility")) {
    // A composite row: the control column holds the whole hemisphere choice rather than a single
    // widget. The row shape is unchanged — one label on the left, everything it names on the right
    // — which is what lets a group of buttons share a left edge with the scalars around it.
    const FieldEditorConstraint visible_c = ConstraintFor("renderer.visible", g_state);
    // Label-less: the section header one line up already says "Visibility", and a row that repeats
    // its own section's word says nothing the second time. The row still exists so the choice keeps
    // the control column's left edge rather than starting at the panel margin.
    PropertyRow("");
    ImGui::BeginDisabled(!visible_c.enabled);
    ImGui::RadioButton("Upper##visible", &r.visible, kVisibleUpper);
    ImGui::SameLine();
    ImGui::RadioButton("Full##visible", &r.visible, kVisibleFull);
    ImGui::SameLine();
    ImGui::RadioButton("Lower##visible", &r.visible, kVisibleLower);
    ImGui::EndDisabled();
    // Between the two pairs rather than inside either: SameLine only moves the draw cursor for the
    // next widget, so it is unaffected by — and does not affect — the disabled stack.
    ImGui::SameLine(0, 20);
    const FieldEditorConstraint front_c = ConstraintFor("renderer.front", g_state);
    ImGui::BeginDisabled(!front_c.enabled);
    Checkbox("Front##visible", &r.front);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Show front hemisphere only\n(combine with Upper/Full/Lower)");
    }
    ImGui::EndDisabled();
    EndPropertyTable();
  }

  ImGui::SeparatorText("Pose");
  if (is_globe) {
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "In Globe lens, Az/El/Roll control the observer's orbit\n"
          "around the sphere, not the camera's own attitude.\n"
          "Roll is locked to 0 in this mode (the control is greyed out).");
    }
  }
  if (BeginPropertyTable("##cam_pose")) {
    // The elevation limit backs off one degree from the pole under Globe; that, like the full-sky
    // gate the two share, is the registry's to state. Both entries carry the SAME gate
    // (NotUnderFullSky), so which of the two `enabled` values wraps the pair cannot matter.
    const FieldEditorConstraint el_c = ConstraintFor("renderer.elevation", g_state);
    const FieldEditorConstraint az_c = ConstraintFor("renderer.azimuth", g_state);
    ImGui::BeginDisabled(!el_c.enabled);
    PropertyRow("Elevation");
    DragFloatField("Elevation##view", &r.elevation, static_cast<float>(el_c.min_value),
                   static_cast<float>(el_c.max_value), el_c.fmt, el_c.scale);
    PropertyRow("Azimuth");
    DragFloatField("Azimuth##view", &r.azimuth, static_cast<float>(az_c.min_value), static_cast<float>(az_c.max_value),
                   az_c.fmt, az_c.scale);
    ImGui::EndDisabled();
    // roll's gate is the wider one (full-sky OR globe) — again read, not restated.
    const FieldEditorConstraint roll_c = ConstraintFor("renderer.roll", g_state);
    PropertyRow("Roll");
    ImGui::BeginDisabled(!roll_c.enabled);
    DragFloatField("Roll##view", &r.roll, static_cast<float>(roll_c.min_value), static_cast<float>(roll_c.max_value),
                   roll_c.fmt, roll_c.scale);
    ImGui::EndDisabled();
    EndPropertyTable();
  }

  ImGui::Separator();
  float btn_w = ImGui::CalcTextSize("Reset").x + ImGui::GetStyle().FramePadding.x * 2.0f;
  float avail = ImGui::GetContentRegionAvail().x;
  if (avail > btn_w) {
    ImGui::SameLine(avail - btn_w);
  }
  if (ImGui::SmallButton("Reset##view")) {
    ViewDefaults d = DefaultViewParamsFor(r.lens_type);
    r.fov = d.fov;
    r.elevation = d.elevation;
    r.azimuth = d.azimuth;
    r.roll = d.roll;
  }
}

}  // namespace

void RenderDocumentInspector() {
  // Not submitted while the column is collapsed. Unlike the tree — whose window has to stay
  // submitted so the strip keeps a node beside the preview rather than on top of it — the
  // inspector's neighbour is the tree, one node over inside the same column. Dropping it hands
  // its height to the tree, which is the whole 20 px strip, so the collapsed column shows one
  // chevron instead of two stacked ones with a separator between them.
  if (g_state.left_panel_collapsed) {
    return;
  }

  ImGui::Begin(kDocumentInspectorWindowName, nullptr, kSidePanelBaseFlags);

  // A selection is a pair of indices into vectors the user can shrink, so it can name something
  // that no longer exists. The delete paths clear it, but that is a claim about every writer being
  // careful; range-checking HERE is a claim about this reader, and it is the one that decides
  // whether a stale selection is a blank page or a crash.
  GuiState::DocumentSelection sel = g_state.selection;
  const int layer_count = static_cast<int>(g_state.layers.size());
  const bool layer_in_range = sel.layer_idx >= 0 && sel.layer_idx < layer_count;
  if ((sel.kind == GuiState::SelectionKind::kLayer || sel.kind == GuiState::SelectionKind::kCrystal) &&
      !layer_in_range) {
    sel.kind = GuiState::SelectionKind::kNone;
  }
  if (sel.kind == GuiState::SelectionKind::kCrystal && !g_state.HasValidCrystalSelection()) {
    sel.kind = GuiState::SelectionKind::kNone;
  }

  // A page swap starts at the top of the new page. The window is one persistent ImGui window
  // showing four different documents' worth of controls, so without this it opens the next page at
  // whatever offset the last one was left at — reliably hiding the top of a page whenever the
  // previous one was taller, which is exactly when the user was last scrolling.
  static GuiState::DocumentSelection s_shown;
  if (sel != s_shown) {
    s_shown = sel;
    ImGui::SetScrollY(0.0f);
  }

  // The header names the page, so it is composed before anything is drawn — including in the folded
  // case, where it is the only thing drawn and therefore the only thing still saying what the
  // column has selected.
  const char* icon = ICON_FA_CIRCLE_INFO;
  char title[64] = "Inspector";
  switch (sel.kind) {
    case GuiState::SelectionKind::kSun:
      icon = ICON_FA_SUN;
      snprintf(title, sizeof(title), "Sun");
      break;
    case GuiState::SelectionKind::kCamera:
      icon = ICON_FA_CAMERA;
      snprintf(title, sizeof(title), "Camera");
      break;
    case GuiState::SelectionKind::kLayer:
      icon = ICON_FA_LAYER_GROUP;
      snprintf(title, sizeof(title), "Layer %d", sel.layer_idx + 1);
      break;
    case GuiState::SelectionKind::kCrystal: {
      const auto& entry = g_state.layers[sel.layer_idx].entries[sel.entry_idx];
      const CrystalConfig& cr = g_state.crystals[entry.crystal_id];
      icon = ICON_FA_GEM;
      snprintf(title, sizeof(title), "%s  ·  L%d/%d", cr.type == CrystalType::kPrism ? "Prism" : "Pyramid",
               sel.layer_idx + 1, sel.entry_idx + 1);
      break;
    }
    case GuiState::SelectionKind::kNone:
      break;
  }

  static PanelCollapseTracker s_fold;
  if (RenderHalfFoldHeader(icon, title, "inspector_fold", g_state.document_inspector_folded)) {
    g_state.FoldDocumentHalves(false, !g_state.document_inspector_folded);
  }
  ApplyPanelCollapse(CollapseAxis::kHeight, GetPanelNodeIds().document_inspector, g_state.document_inspector_folded,
                     DefaultHalfHeight(/*inspector=*/true), &s_fold);
  if (g_state.document_inspector_folded) {
    ImGui::End();
    return;
  }

  switch (sel.kind) {
    case GuiState::SelectionKind::kSun:
      RenderSunControls(g_state);
      break;
    case GuiState::SelectionKind::kCamera:
      RenderCameraControls();
      break;
    case GuiState::SelectionKind::kLayer:
      // The erase is the caller's, not the page's — see RenderLayerInspector. Doing it here also
      // keeps it after the tree has finished iterating this frame's layers.
      if (RenderLayerInspector(g_state, sel.layer_idx)) {
        g_state.layers.erase(g_state.layers.begin() + sel.layer_idx);
        g_thumbnail_cache.OnLayerStructureChanged();
        g_state.SelectNone();
      }
      break;
    case GuiState::SelectionKind::kCrystal:
      RenderCrystalInspector(g_state, sel.layer_idx, sel.entry_idx);
      break;
    case GuiState::SelectionKind::kNone:
      ImGui::TextDisabled("Select an item in the tree above.");
      break;
  }

  ImGui::End();
}

namespace {

// The rectangle the preview and the display strip share: the dockspace's central node, which
// dock_layout keeps permanently empty so the GL shader drawn between ImGui::Render and SwapBuffers
// shows through it. Taking the rect from the node instead of recomputing it from the panel-width
// constants is what makes both windows follow a splitter drag; the constants only describe the
// DEFAULT layout now.
//
// The fallback covers the frames before the first BuildDefaultDockLayout call (and any state where
// the tree has not been split): the pre-docking arithmetic, which is exactly right for the default
// layout — not a stale rect carried over from an earlier frame.
//
// Shared by the two windows rather than computed in each: they are stacked inside this one band, so
// a fallback that drifted between them would put a seam in a place only one of them knows about.
struct CentralBand {
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
};

CentralBand GetCentralBand(float window_width, float window_height) {
  CentralBand band;
  band.x = g_state.left_panel_collapsed ? kCollapseBtnSize : kLeftPanelWidth;
  band.y = kTopBarHeight;
  band.w = window_width - band.x;
  band.h = window_height - kTopBarHeight - kStatusBarHeight;

  ImVec2 central_pos;
  ImVec2 central_size;
  if (GetCentralNodeRect(&central_pos, &central_size)) {
    band.x = central_pos.x;
    band.y = central_pos.y;
    band.w = central_size.x;
    band.h = central_size.y;
  }
  return band;
}

// One frame's auxiliary-line overlay, derived from the document alone — the sun, the renderer pose
// and the show_*_line view preferences. It reads nothing from the simulation lifecycle, and that is
// load-bearing rather than incidental: it is what lets a stale on-screen image and a live
// coordinate system disagree on purpose. The pixels are the last run's, the lines are the current
// document's, and the offset between them IS the "this result is out of date" signal, read at the
// same moment as the top bar's dirty chip (doc/gui-layout-architecture.md §4).
//
// Deliberately pure: the empty state's own presentation (half intensity, forced sky coordinate
// system) is a post-processing step the caller applies to the returned value. Pushing it in here as a couple of
// mode parameters would make every other call site pass constants that only exist to say "I am not
// the empty state".
OverlayDecoration BuildOverlayDecoration(const GuiState& st, const ViewProjection& vp, int vp_w, int vp_h) {
  OverlayDecoration ov;

  // Line flags only — labels are a separate path via BuildOverlayLabelInput.
  ov.show_horizon = st.show_horizon_line;
  ov.show_grid = st.show_grid_line;
  ov.show_sun_circles = st.show_sun_circles_line;
  std::copy(std::begin(st.horizon_color), std::end(st.horizon_color), std::begin(ov.horizon_color));
  std::copy(std::begin(st.grid_color), std::end(st.grid_color), std::begin(ov.grid_color));
  std::copy(std::begin(st.sun_circles_color), std::end(st.sun_circles_color), std::begin(ov.sun_circles_color));
  ov.horizon_alpha = st.horizon_alpha;
  ov.grid_alpha = st.grid_alpha;
  ov.sun_circles_alpha = st.sun_circles_alpha;
  ov.grid_step = ComputeGridStep(st.renderer.fov);

  // Sun direction in world space (azimuth fixed at 0, only altitude matters). Computed once and
  // used twice — as the shader's u_sun_dir for the angular-distance circles, and as the input to
  // the marker's forward projection below. Two copies of this derivation would have to agree
  // forever for the circles to stay centred on the dot.
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  const float sa = st.sun.altitude * kDeg2Rad;
  const float sun_world_dir[3] = { -std::cos(sa), 0.0f, -std::sin(sa) };
  std::copy(std::begin(sun_world_dir), std::end(sun_world_dir), std::begin(ov.sun_dir));
  ov.sun_circle_count = std::min(static_cast<int>(st.sun_circle_angles.size()), kMaxSunCircles);
  for (int i = 0; i < ov.sun_circle_count; i++) {
    ov.sun_circle_angles[i] = st.sun_circle_angles[i];
  }

  // Zenith / Nadir pixel-space marker. zenith world dir = (0,0,-1), nadir = (0,0,+1)
  // (see preview_renderer.cpp:overlayAuxLines altitude convention).
  ov.show_zenith_nadir = st.show_zenith_nadir_line;
  std::copy(std::begin(st.zenith_nadir_color), std::end(st.zenith_nadir_color), std::begin(ov.zenith_nadir_color));
  ov.zenith_nadir_alpha = st.zenith_nadir_alpha;
  ov.zenith_nadir_radius_px = st.zenith_nadir_radius_px;
  constexpr float kZenithWorldDir[3] = { 0.f, 0.f, -1.f };
  constexpr float kNadirWorldDir[3] = { 0.f, 0.f, 1.f };
  const auto zpos = ProjectWorldDirToScreen(vp, kZenithWorldDir, vp_w, vp_h);
  const auto npos = ProjectWorldDirToScreen(vp, kNadirWorldDir, vp_w, vp_h);
  ov.zenith_screen_pos[0] = zpos[0];
  ov.zenith_screen_pos[1] = zpos[1];
  ov.nadir_screen_pos[0] = npos[0];
  ov.nadir_screen_pos[1] = npos[1];

  // The sun's own pixel position. Placed unconditionally — show_sun_marker stays false here and is
  // the caller's to set, so a call site that wants the marker never has to also remember to ask for
  // the position, and a test can read the position without turning the marker on.
  const auto spos = ProjectWorldDirToScreen(vp, sun_world_dir, vp_w, vp_h);
  ov.sun_marker_screen_pos[0] = spos[0];
  ov.sun_marker_screen_pos[1] = spos[1];

  return ov;
}

// The intensities the empty state's CPU-drawn marks are drawn at — the ones the shader would have
// used, handed over by ApplyEmptyStatePresentation as it mutes its own copy of them. Returned
// rather than recomputed at the draw site so the scaling below has exactly one owner.
struct EmptyStateStrokes {
  float sun_circles_alpha = 0.0f;
  float sun_marker_alpha = 0.0f;
};

// How much of the user's overlay intensity the empty state keeps. Half is where the prototype
// landed: enough to read the sky as "framed and waiting" rather than as a rendered result, which is
// the whole distinction the empty state is drawing. A starting point, not a derived constant.
constexpr float kEmptyStateOverlayAlphaScale = 0.5f;

// Turn a document's overlay into the empty state's version of itself: every line the empty state
// owes the user, dimmer.
//
// Which lines those are is not a taste call — doc/gui-layout-architecture.md §4 enumerates the
// empty-state sky coordinate system as horizon, angular-distance circles and sun marker, so those
// three are forced on here regardless of the show_*_line toggles. The grid and the zenith/nadir
// markers are NOT in that enumeration, so they keep following the toggles, in the empty state
// exactly as over a result. The asymmetry is the point rather than an oversight: a show_*_line
// toggle means "do not clutter the image I rendered", a preference about a RESULT. An empty state
// has no result to clutter, and the one thing it is for — telling the user where the 22 degree halo
// will land before they press Run — is the very line the default-off sun_circles toggle would
// suppress.
//
// Forcing happens on the caller's per-frame copy of the decoration, never on GuiState: the argument
// is the local value RenderPreviewPanel just derived via BuildOverlayDecoration, and this function
// is handed no GuiState to write back to. Leaving the empty state and returning to a result
// therefore restores the user's own toggles by construction, not by anyone remembering to undo
// anything.
//
// The marker's alpha is NOT scaled. The four alphas above are user settings, chosen for legibility
// on top of a rendered image, so halving them is what "dimmer than that" means. The marker has no
// user setting and never appears over an image — its OverlayDecoration default IS its empty-state
// value, and halving it would only be halving a number this task chose one line earlier.
//
// Two of the four alphas are not dimmed but ZEROED, and that is a handover rather than a removal:
// the angular-distance circles and the sun marker are still forced ON one line below, but the
// EMPTY STATE'S versions of them are a dashed circle with a degree label and a cross-hair, and
// neither shape is expressible in overlayAuxLines() — the shader draws a solid ring and a filled
// dot, and it cannot draw text at all. So the empty state draws those two itself, on the CPU, in
// DrawEmptyStateInstrument below; the alphas it would have drawn them at are returned here and
// handed to that function, and the shader is muted so the two paths cannot double-draw.
//
// Reading only this function, alpha == 0 looks like "invisible". It is not: it means "not the
// shader's to draw this time". The visible counterpart lives in RenderPreviewPanel's empty-state
// branch, which carries the reverse pointer back here.
EmptyStateStrokes ApplyEmptyStatePresentation(OverlayDecoration* ov) {
  ov->horizon_alpha *= kEmptyStateOverlayAlphaScale;
  ov->grid_alpha *= kEmptyStateOverlayAlphaScale;
  ov->zenith_nadir_alpha *= kEmptyStateOverlayAlphaScale;
  ov->show_horizon = true;
  ov->show_sun_circles = true;
  ov->show_sun_marker = true;

  EmptyStateStrokes strokes;
  strokes.sun_circles_alpha = ov->sun_circles_alpha * kEmptyStateOverlayAlphaScale;
  strokes.sun_marker_alpha = ov->sun_marker_alpha;
  ov->sun_circles_alpha = 0.0f;
  ov->sun_marker_alpha = 0.0f;
  return strokes;
}

// The empty state's instrument marks: everything the shader was just muted for, plus the two
// labels it never could have drawn.
//
// What this draws, and why here rather than in the shader (doc/gui-layout-architecture.md §4 for
// what the empty state owes the user, the prototype for the form):
//   - each angular-distance circle as a DASHED ring with its angle written beside it ("22°"),
//   - the sun as a CROSS-HAIR rather than a filled dot,
//   - the horizon line — drawn by the shader, unchanged — labelled HORIZON at its left end.
// overlayAuxLines() draws solid rings and a filled dot and has no text at all, so all three shapes
// would have meant editing that fragment shader. That shader is the pixel source of the committed
// lens_proj reference group, whose scenes are re-shot on a change to it; the empty state is not
// among them and has no business forcing that. Drawing on the CPU keeps the change where its own
// evidence is.
//
// Every mark projects through overlay_labels.hpp's WorldDirToPixel — the same forward the labels
// over a rendered result use. Not a copy of it: a second projection would put the "22°" text and
// the ring it names on two different lenses' worth of maths the first time either changed.
//
// Colours are the ones already in play, at the intensities ApplyEmptyStatePresentation handed over:
// the circles keep the document's own sun_circles_color, the cross keeps the marker's colour, and
// both labels take ImGuiCol_TextDisabled — the palette's dim text tier (theme.cpp text_dim), the
// same grade every other secondary reading in this app is set in. No new hue is introduced here.

// Samples per angular-distance ring. 96 puts a vertex every 3.75° of arc, which reads as a circle
// rather than a polygon at any viewport this panel gets, and leaves the dash rhythm below enough
// segments to be a rhythm.
constexpr int kEmptyStateCircleSegments = 96;
// The dash rhythm, in segments: three drawn, three skipped. Deliberately coarse — the point is that
// the ring reads as a MEASUREMENT overlaid on the sky rather than as something in the picture.
constexpr int kEmptyStateDashOnSegments = 3;
constexpr int kEmptyStateDashOffSegments = 3;
// Half the length of each arm of the sun's cross-hair, in ImGui pixels. Slightly larger than the
// filled dot it replaces (OverlayDecoration::sun_marker_radius_px = 5) so the two forms carry
// about the same visual weight.
constexpr float kEmptyStateSunCrossArmPx = 8.0f;
// Gap between a mark and the text naming it.
constexpr float kEmptyStateLabelGapPx = 4.0f;

// One frame's world → preview-window projection, bound once so every mark below is placed by the
// same lens, pose and rectangle.
struct EmptyStateProjection {
  float view_matrix[9] = {};
  int lens_type = 0;
  float fov = 0.0f;
  float res_x = 0.0f, res_y = 0.0f;                          // viewport in FRAMEBUFFER pixels
  float vp_x = 0.0f, vp_y = 0.0f, vp_w = 0.0f, vp_h = 0.0f;  // the same rect in ImGui screen space

  // Screen position of a world direction, or false if it is behind the camera, outside this lens's
  // projection domain, or off the edge of the viewport. The y flip and the framebuffer → screen
  // scaling are the same two lines ComputeOverlayLabels uses on its own samples.
  bool ToScreen(const float dir[3], ImVec2* out) const {
    const ProjectedPixel fp = WorldDirToPixel(dir[0], dir[1], dir[2], res_x, res_y, lens_type, fov, view_matrix);
    if (!fp.valid) {
      return false;
    }
    const float hw = res_x * 0.5f;
    const float hh = res_y * 0.5f;
    if (std::fabs(fp.px) > hw || std::fabs(fp.py) > hh) {
      return false;
    }
    out->x = vp_x + (fp.px + hw) / res_x * vp_w;
    out->y = vp_y + (hh - fp.py) / res_y * vp_h;
    return true;
  }
};

// Place a label above `anchor`, kept inside the viewport by the same clamp the overlay labels use.
void DrawEmptyStateLabel(ImDrawList* dl, const EmptyStateProjection& proj, ImVec2 anchor, const char* text,
                         ImU32 color) {
  const ImVec2 size = ImGui::CalcTextSize(text);
  ImVec2 pos(anchor.x - size.x * 0.5f, anchor.y - kEmptyStateLabelGapPx - size.y);
  pos = detail::ClampLabelPosToViewport(pos, size, proj.vp_x, proj.vp_y, proj.vp_w, proj.vp_h);
  dl->AddText(pos, color, text);
}

void DrawEmptyStateInstrument(const EmptyStateProjection& proj, const OverlayDecoration& ov,
                              const EmptyStateStrokes& strokes, EmptyStateInstrument* out) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImU32 label_col = ImGui::GetColorU32(ImGuiCol_TextDisabled);
  const ImU32 circle_col = ImGui::ColorConvertFloat4ToU32(
      ImVec4(ov.sun_circles_color[0], ov.sun_circles_color[1], ov.sun_circles_color[2], strokes.sun_circles_alpha));
  const ImU32 cross_col = ImGui::ColorConvertFloat4ToU32(
      ImVec4(ov.sun_marker_color[0], ov.sun_marker_color[1], ov.sun_marker_color[2], strokes.sun_marker_alpha));

  // An orthonormal basis of the plane perpendicular to the sun, so a ring at angular distance θ is
  // cos(θ)·sun + sin(θ)·(cos t·u + sin t·v). The sun's azimuth is fixed at 0 in this app
  // (BuildOverlayDecoration), i.e. sun_dir.y is identically zero, so (0,1,0) is perpendicular to it
  // for every altitude and the usual "pick a reference axis that is not parallel" fallback has no
  // case to cover. v then points along the meridian, which is what puts the degree label at the top
  // of the ring rather than at an angle that moves with the sun.
  const float* sun = ov.sun_dir;
  const float u[3] = { 0.0f, 1.0f, 0.0f };
  const float v[3] = { sun[1] * u[2] - sun[2] * u[1], sun[2] * u[0] - sun[0] * u[2], sun[0] * u[1] - sun[1] * u[0] };

  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  constexpr float kTwoPi = 2.0f * 3.14159265358979323846f;
  constexpr int kDashPeriod = kEmptyStateDashOnSegments + kEmptyStateDashOffSegments;

  for (int c = 0; c < ov.sun_circle_count; c++) {
    const float angle_deg = ov.sun_circle_angles[c];
    const float theta = angle_deg * kDeg2Rad;
    const float ct = std::cos(theta);
    const float st = std::sin(theta);

    ImVec2 pts[kEmptyStateCircleSegments + 1];
    bool on_screen[kEmptyStateCircleSegments + 1];
    for (int i = 0; i <= kEmptyStateCircleSegments; i++) {
      const float t = kTwoPi * static_cast<float>(i) / kEmptyStateCircleSegments;
      const float cos_t = std::cos(t);
      const float sin_t = std::sin(t);
      const float dir[3] = { ct * sun[0] + st * (cos_t * u[0] + sin_t * v[0]),
                             ct * sun[1] + st * (cos_t * u[1] + sin_t * v[1]),
                             ct * sun[2] + st * (cos_t * u[2] + sin_t * v[2]) };
      on_screen[i] = proj.ToScreen(dir, &pts[i]);
    }

    int drawn_segments = 0;
    for (int i = 0; i < kEmptyStateCircleSegments; i++) {
      if (i % kDashPeriod >= kEmptyStateDashOnSegments) {
        continue;  // the gap half of the rhythm
      }
      if (!on_screen[i] || !on_screen[i + 1]) {
        continue;  // an arc that leaves the frame simply stops, as the shader's ring does
      }
      dl->AddLine(pts[i], pts[i + 1], circle_col);
      drawn_segments++;
    }
    if (drawn_segments == 0) {
      continue;  // this ring is entirely off screen — no ring, and nothing to label
    }
    out->dashed_circles++;

    // The label goes at t = 90°, i.e. straight up the meridian from the sun — a fixed parameter ON
    // THE RING rather than a fixed pixel offset, so it is re-projected every frame and follows the
    // sun and the lens instead of drifting off the ring the first time either moves.
    //
    // When that point is off screen the label walks around the ring to the nearest sample that is
    // not, in either direction. That is not the curve-centric label placement machinery
    // (overlay-label-placement.md) and deliberately so — no visibility model, no collision pass,
    // no scoring; it reads the on_screen[] flags this loop already computed and stops at the first
    // true. Without it the 46° ring loses its label at the default pose, since its top is above the
    // frame — the ring is drawn and unnamed, which is the one thing the degree labels exist to fix.
    const int anchor_index = kEmptyStateCircleSegments / 4;  // t = 90°
    int label_index = -1;
    for (int d = 0; d <= kEmptyStateCircleSegments / 2 && label_index < 0; d++) {
      const int forward = (anchor_index + d) % kEmptyStateCircleSegments;
      const int backward = (anchor_index - d + kEmptyStateCircleSegments) % kEmptyStateCircleSegments;
      if (on_screen[forward]) {
        label_index = forward;
      } else if (on_screen[backward]) {
        label_index = backward;
      }
    }
    if (label_index < 0) {
      continue;  // the whole ring is off screen (its dashes came from the wrap-around sample only)
    }
    const ImVec2 label_anchor = pts[label_index];
    const bool integral = std::fabs(angle_deg - std::round(angle_deg)) < 0.05f;
    char buf[32];
    std::snprintf(buf, sizeof(buf), integral ? "%.0f\xC2\xB0" : "%.1f\xC2\xB0", angle_deg);
    DrawEmptyStateLabel(dl, proj, label_anchor, buf, label_col);
    // Bounds: ov.sun_circle_count is already min(angles.size(), kMaxSunCircles) at its only
    // producer (BuildOverlayDecoration), which is the same bound the ring loop reads angles under.
    out->degree_label_pos[c][0] = label_anchor.x;
    out->degree_label_pos[c][1] = label_anchor.y;
    out->degree_labels++;
  }

  // The sun itself: a cross-hair, which says "this is where it WILL be" in a way a filled dot —
  // the shape a rendered sun actually has — does not.
  ImVec2 sun_pos;
  if (proj.ToScreen(sun, &sun_pos)) {
    dl->AddLine(ImVec2(sun_pos.x - kEmptyStateSunCrossArmPx, sun_pos.y),
                ImVec2(sun_pos.x + kEmptyStateSunCrossArmPx, sun_pos.y), cross_col);
    dl->AddLine(ImVec2(sun_pos.x, sun_pos.y - kEmptyStateSunCrossArmPx),
                ImVec2(sun_pos.x, sun_pos.y + kEmptyStateSunCrossArmPx), cross_col);
    out->sun_cross = true;
    out->sun_cross_pos[0] = sun_pos.x;
    out->sun_cross_pos[1] = sun_pos.y;
  }

  // HORIZON, at the left end of the horizon line — the end the prototype labels, and the one a
  // reader gets to first. Which azimuth that is depends on where the camera points and which lens
  // it looks through, so it is FOUND rather than computed: walk the altitude-0 circle, keep the
  // visible sample furthest to the left. A closed-form azimuth would have to re-derive the sign
  // convention of the view matrix and would still be wrong for the full-sky lenses, whose horizon
  // does not run left-to-right across the frame at all.
  ImVec2 left_end;
  bool found_horizon = false;
  for (int i = 0; i < kEmptyStateCircleSegments; i++) {
    const float az = kTwoPi * static_cast<float>(i) / kEmptyStateCircleSegments;
    const float dir[3] = { -std::cos(az), -std::sin(az), 0.0f };
    ImVec2 p;
    if (!proj.ToScreen(dir, &p)) {
      continue;
    }
    if (!found_horizon || p.x < left_end.x) {
      left_end = p;
      found_horizon = true;
    }
  }
  if (found_horizon) {
    DrawEmptyStateLabel(dl, proj, left_end, "HORIZON", label_col);
    out->horizon_label = true;
  }
}

}  // namespace

void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height) {
  const CentralBand band = GetCentralBand(window_width, window_height);
  // The strip takes the bottom of the band; what is left is the preview. Complementary by
  // construction — see SplitViewportForDisplayStrip (window_sizing.hpp).
  const ViewportStripSplit split = SplitViewportForDisplayStrip(band.y, band.h, kDisplayStripHeight);
  float panel_x = band.x;
  float panel_y = band.y;
  float panel_width = band.w;
  float panel_height = split.preview_h;
  SetNextPanelGeometry(panel_x, panel_y, panel_width, panel_height);
  // NoDocking: this window is pinned to the central node, it is not docked INTO it. Letting the user
  // dock it would fill the central node, and imgui only punches the passthru hole while that node is
  // empty — the preview would paint itself out of existence. (It is also unreachable by dragging
  // today, having neither a title bar nor NoMove cleared; the flag states the constraint rather than
  // leaving it to be re-derived.)
  ImGui::Begin("##PreviewPanel", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDocking |
                   ImGuiWindowFlags_NoBringToFrontOnFocus);

  // Renderer invariants (previously in RenderViewBar, runs every frame).
  // Copy-model: GuiState always owns a single renderer, no vector/index bookkeeping needed.
  {
    auto& r = g_state.renderer;
    float max_fov = LUMICE_MaxFov(static_cast<LUMICE_LensType>(r.lens_type));
    r.fov = std::min(r.fov, max_fov);
    if (LensIsFullSky(r.lens_type)) {  // Full-sky lenses: force view angles to zero
      r.elevation = 0.0f;
      r.azimuth = 0.0f;
      r.roll = 0.0f;
    }
  }

  float preview_height = panel_height;

  // The viewport rectangle, the view projection and the overlay are computed for BOTH states, not
  // only the one that has pixels. An empty preview draws the same sky coordinate system through the
  // same shader, so it needs the same rectangle and the same projection; what it does not need is
  // the exposure, the background image and the drag surface. Those three — not the geometry — are
  // where the two states actually part company, and the branch below is now only about them.
  //
  // Compute viewport in framebuffer pixels (for HiDPI).
  int fb_w = 0;
  int fb_h = 0;
  glfwGetFramebufferSize(window, &fb_w, &fb_h);
  float scale_x = static_cast<float>(fb_w) / window_width;
  float scale_y = static_cast<float>(fb_h) / window_height;

  auto& rc = g_state.renderer;
  auto& pp = g_preview_vp.params;

  // Store viewport for deferred rendering. Always active: with the empty state drawing its own
  // coordinate system, there is no longer a frame in which the panel is on screen and has nothing
  // for PreviewRenderer::Render to do (which guards a degenerate rectangle itself).
  g_preview_vp.active = true;
  g_preview_vp.vp_x = static_cast<int>(panel_x * scale_x);
  // OpenGL Y is bottom-up: measure from the window's bottom edge to the panel's bottom edge. This
  // used to read kStatusBarHeight directly, which was the same number only while the panel was
  // guaranteed to end exactly at the status bar.
  g_preview_vp.vp_y = static_cast<int>((window_height - (panel_y + panel_height)) * scale_y);
  g_preview_vp.vp_w = static_cast<int>(panel_width * scale_x);
  g_preview_vp.vp_h = static_cast<int>(preview_height * scale_y);
  pp.view_proj = BuildPreviewViewProjFromRenderer(rc);
  // Overlap parameters for dual fisheye texture sampling. Frame-invariant constants, so they are
  // set once for both states rather than left holding whichever frame last passed through the
  // branch below.
  pp.source.max_abs_dz = kDualFisheyeOverlap;
  pp.source.r_scale = 1.0f / std::sqrt(1.0f + kDualFisheyeOverlap);
  // Auxiliary line overlay (line flags only — labels are handled separately via
  // BuildOverlayLabelInput below). Read from the document every frame in both states, which is what
  // keeps a stale image and a live coordinate system able to disagree — see BuildOverlayDecoration.
  pp.overlay = BuildOverlayDecoration(g_state, pp.view_proj, g_preview_vp.vp_w, g_preview_vp.vp_h);
  // Zeroed on EVERY frame, not only on empty ones: it records what the empty state's instrument
  // drew, so a result frame has to leave it saying "nothing", not saying whatever the last empty
  // frame said.
  g_preview_vp.empty_state = EmptyStateInstrument{};

  if (g_preview.HasTexture() || g_preview.HasBackground()) {
    float ev_total = rc.exposure_offset + g_state.ev_auto;
    pp.exposure.intensity_factor = std::pow(2.0f, ev_total);

    // task-347 (Fix B) DECOUPLE: the composite path is now server-side self-
    // anchored on participating-P99 (see doc/ev-pipeline-architecture.md §6.6).
    // GUI must push ONLY the manual `exposure_offset` here — folding `ev_auto`
    // back in would multiply the auto-anchor by the auto-EV a second time and
    // reintroduce the double-count that Fix B is meant to eliminate. The mono
    // shader-uniform path above still uses `ev_total = exposure_offset +
    // ev_auto` because the mono pipeline anchors off the mono ExposureScale
    // (integral over the whole image), not off a self-anchored P99.
    //
    // Push guard logic (value-changed OR off->on edge) lives in
    // ShouldPushCompositeExposure (gui/composite_exposure_push.hpp,
    // code-review round 1 Major #1 + Minor #1) so the four branches are
    // independently unit-testable rather than only reachable through a full
    // ImGui frame. See that header for the off->on rationale (plan-review
    // Minor #2).
    {
      static float s_last_pushed_ev = std::numeric_limits<float>::quiet_NaN();
      static bool s_last_composite_active = false;
      constexpr float kCompositeEvPushEpsilon = 1e-4f;
      // Fixed +1.0 EV (2x) brightness boost baked into the composite-mode
      // exposure baseline (task-color-ev-boost / A1). owner-observed perceptual
      // gap: single-color composite reads dimmer than full-spectrum at the same
      // manual exposure offset. Stacked on top of rc.exposure_offset (baseline,
      // not override). Independent of Fix B's "no ev_auto" constraint — this is
      // a static perceptual compensation, not an auto-anchored term.
      constexpr float kColorModeEvBoost = 1.0f;
      const bool composite_active = g_server != nullptr && !g_state.raypath_color.empty();
      // Fix B: push value == manual EV offset only (no ev_auto).
      // + kColorModeEvBoost: static composite-mode brightness baseline (see above).
      const float composite_ev_push = rc.exposure_offset + kColorModeEvBoost;
      if (lumice::gui::ShouldPushCompositeExposure(composite_active, s_last_composite_active, composite_ev_push,
                                                   s_last_pushed_ev, kCompositeEvPushEpsilon)) {
        LUMICE_SetCompositeExposure(g_server, composite_ev_push);
        // Wake a paused poller so the next frame can pick up the freshly re-baked
        // composite even after a finite sim has completed (same rationale as the
        // color-window PushDisplayState path — see task-345.2 (③) in color_window.cpp).
        // No-op when the poller is already running. WakeForRefresh (not WakeForRestart)
        // preserves valid=true across the wake edge — same display-time-inert contract
        // as color/visible/solo/z_order edits (task-color-migration §3 D3).
        g_server_poller.WakeForRefresh(g_server);
        s_last_pushed_ev = composite_ev_push;
      }
      s_last_composite_active = composite_active;
    }
    float norm_intensity = g_state.snapshot_intensity;
    pp.exposure.intensity_scale = norm_intensity > 0 ? pp.exposure.intensity_factor / norm_intensity : 0.0f;
    pp.bg.enabled = g_state.bg_show && g_preview.HasBackground();
    pp.bg.alpha = g_state.bg_alpha;
    pp.bg.aspect = g_preview.GetBgAspect();

    // Overlay labels at viewport edges (drawn on the preview window's draw list so
    // modals correctly occlude them). BuildOverlayLabelInput is shared with
    // DoExportPreviewPng (off-screen FBO path) so both call sites produce
    // byte-identical OverlayLabelInput for a given state.
    if (g_state.show_horizon_label || g_state.show_grid_label || g_state.show_sun_circles_label) {
      OverlayLabelInput label_input = BuildOverlayLabelInput(g_state, rc);

      // Viewport rect in absolute OS screen coordinates. DrawOverlayLabels emits to
      // ImGui::GetWindowDrawList(), and with ImGuiConfigFlags_ViewportsEnable (gui-polish-v15)
      // draw list coordinates are absolute screen space, not relative to the host GLFW window.
      // Anchor (panel_x, panel_y) through MainVpPos() so labels stay glued to the
      // preview viewport when the host window is dragged or sits on a non-primary monitor.
      // Note: the export_fbo_renderer.cpp path passes (0, 0, w, h) intentionally — it owns a
      // self-allocated ImDrawList targeting an off-screen FBO and must NOT add this offset.
      ImVec2 vp_origin = MainVpPos(panel_x, panel_y);
      float vp_sx = vp_origin.x;
      float vp_sy = vp_origin.y;
      float vp_sw = panel_width;
      float vp_sh = preview_height;

      static std::vector<OverlayLabel> labels;
      ComputeOverlayLabels(label_input, vp_sx, vp_sy, vp_sw, vp_sh, labels);
      DrawOverlayLabels(labels, vp_sx, vp_sy, vp_sw, vp_sh);
    }

    // Mouse interaction: orbit with drag, FOV with scroll.
    // Disabled for lenses in kFullSkyLensTypes (dual fisheye 4-6, rectangular 7,
    // dual orthographic 9): their shader path skips the view matrix so view
    // angles + FOV have no visual effect.
    bool full_sky = LensIsFullSky(rc.lens_type);
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("##preview_interact", avail);

    if (!full_sky) {
      bool is_hovered = ImGui::IsItemHovered();
      bool is_active = ImGui::IsItemActive();

      ImGuiIO& io = ImGui::GetIO();
      if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        ImVec2 delta = io.MouseDelta;
        // Sensitivity is the lens's angular resolution at the frame center, so one pixel of
        // drag moves the content one pixel whatever the FOV and viewport are. A fixed deg/px
        // was ~180x too fast at fov=1° and ~70x too slow at fov=179°. vp_w/vp_h are
        // framebuffer pixels (set above from the DPI scale), the same units
        // ProjectWorldDirToScreen works in, which is what makes this DPI-correct for free.
        float gain = ComputeDragGainDegPerPixel(rc.lens_type, rc.fov, g_preview_vp.vp_w, g_preview_vp.vp_h);
        // Globe is outside-in: u_view_matrix * (0,0,D) yields camera world
        // position, so +az/+el move the camera and the sphere drifts in the
        // opposite direction. Flip both signs so a right/down drag moves the
        // sphere right/down, matching the inside-out lenses' feel.
        if (rc.lens_type == kLensTypeGlobe) {
          rc.azimuth += delta.x * gain;
          rc.elevation -= delta.y * gain;
        } else {
          rc.azimuth -= delta.x * gain;
          rc.elevation += delta.y * gain;
        }
        // Wrap azimuth into [-180, 180] so dragging past the slider's clamp
        // boundary continues seamlessly instead of getting pinned at ±180.
        if (rc.azimuth > 180.0f) {
          rc.azimuth -= 360.0f;
        } else if (rc.azimuth < -180.0f) {
          rc.azimuth += 360.0f;
        }
        // Globe lens needs a tighter clamp to avoid view-matrix degeneracy at
        // ±90°; inside-out lenses keep the existing ±90° range.
        float el_lim = (rc.lens_type == kLensTypeGlobe) ? 89.0f : 90.0f;
        rc.elevation = std::max(-el_lim, std::min(el_lim, rc.elevation));
      }

      if (is_hovered && io.MouseWheel != 0.0f) {
        float fov_max = LUMICE_MaxFov(static_cast<LUMICE_LensType>(rc.lens_type));
        rc.fov -= io.MouseWheel * 5.0f;
        rc.fov = std::max(1.0f, std::min(fov_max, rc.fov));
      }
    }
  } else {
    // Nothing has been rendered and nothing has been loaded. The view is still a view, though: the
    // document already says which way the camera points, how wide the lens is and where the sun
    // will be, so the panel draws that — dimmed, and with the sun marked — instead of a black
    // rectangle. The empty state stops being "there is nothing here" and becomes "this is framed,
    // press Run" (doc/gui-layout-architecture.md §4).
    //
    // Exposure and background stay at their defaults: with no texture uploaded, PreviewRenderer
    // samples its 1x1 blank in sRGB mode, where neither is read. The drag surface is deliberately
    // NOT submitted — orbiting an empty view is a separate question from previewing one, and this
    // panel keeps its "the surface exists only when there is something to interact with" contract.
    // The circles and the sun marker come back from here with their shader alphas at zero and their
    // show_* flags forced ON — the empty state draws those two itself, right below, in a form the
    // shader has no way to express (see ApplyEmptyStatePresentation for the full handover). The
    // intensities it would have used come back as the return value.
    const EmptyStateStrokes strokes = ApplyEmptyStatePresentation(&pp.overlay);
    pp.exposure = Exposure{};
    pp.bg = Background::Disabled();

    EmptyStateProjection proj;
    BuildViewMatrix(rc.elevation, rc.azimuth, rc.roll, proj.view_matrix);
    proj.lens_type = rc.lens_type;
    proj.fov = rc.fov;
    proj.res_x = static_cast<float>(g_preview_vp.vp_w);
    proj.res_y = static_cast<float>(g_preview_vp.vp_h);
    // Same anchoring as the result state's overlay labels: draw-list coordinates are absolute OS
    // screen space under ImGuiConfigFlags_ViewportsEnable, so the panel rect goes through
    // MainVpPos rather than being used as window-local.
    const ImVec2 vp_origin = MainVpPos(panel_x, panel_y);
    proj.vp_x = vp_origin.x;
    proj.vp_y = vp_origin.y;
    proj.vp_w = panel_width;
    proj.vp_h = preview_height;
    if (proj.res_x > 0.0f && proj.res_y > 0.0f) {
      DrawEmptyStateInstrument(proj, pp.overlay, strokes, &g_preview_vp.empty_state);
      g_preview_vp.empty_state.drawn = true;
    }

    // Instructional, not descriptive: it names the next action rather than restating what the user
    // can already see (doc/gui-visual-language.md).
    //
    // Set in three runs rather than one, because the sentence is two statements at two weights: the
    // sky IS framed (a description of what is already on screen, and so dim — the same
    // ImGuiCol_TextDisabled grade every other secondary reading uses), and Run is the thing to do
    // next, so it carries the body text's own weight. The app has one font at one weight, so
    // emphasis here is contrast, not a bold face; brightening the one word that names a control the
    // user can go and press is the whole of it.
    const char* kHintLead = "Sky is framed. Press ";
    const char* kHintKeyword = "Run";
    const char* kHintTail = " to expose.";
    const ImVec2 lead_size = ImGui::CalcTextSize(kHintLead);
    const ImVec2 keyword_size = ImGui::CalcTextSize(kHintKeyword);
    const ImVec2 tail_size = ImGui::CalcTextSize(kHintTail);
    const float hint_width = lead_size.x + keyword_size.x + tail_size.x;
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImGui::SetCursorPos(ImVec2((avail.x - hint_width) * 0.5f, (avail.y - lead_size.y) * 0.5f));
    ImGui::TextDisabled("%s", kHintLead);
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::TextUnformatted(kHintKeyword);
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::TextDisabled("%s", kHintTail);
  }

  ImGui::End();
}

namespace {

// Width of the sim-tier marker bar, in pixels. Thin on purpose: the old form filled the whole input
// with rust, which read as "this control is broken" rather than "editing it costs you the run".
constexpr float kSimTierEdgeWidth = 3.0f;

// Armed by ResetDisplayStripSelectionForTest, consumed by the Grade tab's flags on the next frame.
// Which tab is selected lives in ImGui's TabBar, not here, so "put it back on Grade" cannot be an
// assignment — it has to be a request the next render honours (same shape, and the same reason, as
// edit_modals.cpp's g_pending_tab_select).
bool g_strip_select_grade = false;

// The sim-tier marker: a bar along the LEADING edge of the control just submitted, saying "editing
// this re-runs the simulation and discards the accumulated rays" (doc/gui-visual-language.md §7,
// doc/gui-layout-architecture.md §4).
//
// It is the same statement as the top bar's dirty chip, read at a different moment. The chip
// reports that a sim-tier field HAS been edited — IsModified, fed by DiffAgainstCommitBaseline over
// GuiState::ConfigSnapshot, whose RenderConfig half is RenderConfigResimFields (gui_state.hpp).
// This marker says which control would produce that. The two cannot share one runtime value (one is
// a property of a field, the other of the document's state), so what keeps them from drifting is
// that both name the same registry: marking a control whose field RenderConfigResimFields does not
// carry would light up a control the chip never answers for. That registry holds four fields
// (sim_resolution_index, background, ray_color, opacity), of which sim_resolution_index is the only
// one this strip offers a control for — the other three are reachable today only through the
// Settings panel's field registry. The classifier side of it is pinned by
// unit-correctness/gui/test_state_reconcile.cpp's "renderer.sim_resolution_index" row.
//
// Call directly after the control it marks — it reads ImGui's last-item rectangle, and it does not
// disturb it, so an IsItemHovered() tooltip after this call still belongs to the control.
void MarkSimTierEdge() {
  const ImVec2 lo = ImGui::GetItemRectMin();
  const ImVec2 hi = ImGui::GetItemRectMax();
  ImGui::GetWindowDrawList()->AddRectFilled(lo, ImVec2(lo.x + kSimTierEdgeWidth, hi.y),
                                            ImGui::ColorConvertFloat4ToU32(WarningTextColor()),
                                            ImGui::GetStyle().FrameRounding, ImDrawFlags_RoundCornersLeft);
}

// Every tab's body goes in a child region of its own. The strip is a FIXED height (see
// kDisplayStripHeight) so that switching tabs cannot move the viewport's bottom edge; a tab whose
// content does not fit therefore has to scroll inside the strip rather than resize it. Without the
// child, an over-tall tab would simply be clipped, with nothing on screen saying so.
void BeginStripTabContent(const char* id) {
  ImGui::BeginChild(id, ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
}

// The Grade tab: how the accumulated result is rendered and shown — the former right panel's
// Display group, minus its own collapsing header (the tab is the group boundary now).
//
// Laid out across the strip rather than down it: three groups side by side, each two rows tall.
// The strip is wide and short by construction, and the old one-control-per-row column would have
// made it four rows tall — which is height taken from the viewport for a shape that does not need
// it.
//
// The columns are SizingFixedFit, not SizingStretchSame, and that is the whole of this row's width
// story. Under StretchSame every column got a third of the strip's width whatever it held, and the
// controls inside then asked for "the column minus a label" — which is how a combo offering five
// resolutions ended up ~600 px wide on a wide window. Nobody chose that number; it was the
// viewport's, read through two containers (doc/gui-visual-language.md §2). FixedFit inverts the
// direction: each column is as wide as the widest row inside it, and each control states its own
// width from the token table. The cost is that the three groups no longer divide the strip evenly
// and may leave slack at the right — accepted deliberately, because the alternative buys even
// columns by re-attaching every control to the window width.
//
// Within a column the two rows stack vertically (they always have), so a column's width is the max
// of its rows' widths, not their sum: column 1 is set by "Resolution [combo]" rather than by EV.
void RenderGradeTab(GLFWwindow* window) {
  // Copy-model renderer: GuiState always owns a valid renderer by default construction.
  auto& r = g_state.renderer;

  // ONE sub-heading for the whole group (doc/gui-visual-language.md §4.6): the three it replaces
  // ("Rendering" / "Aspect Ratio" / "Background") each headed a single row, which is less than a
  // sub-heading has to earn. What they separated is now separated by the columns themselves.
  ImGui::SeparatorText("Rendering");

  if (ImGui::BeginTable("##GradeLayout", 3, ImGuiTableFlags_SizingFixedFit)) {
    ImGui::TableNextRow();

    // ---- Column 1: the image the simulation renders, and how bright it is shown.
    ImGui::TableSetColumnIndex(0);
    InlineFieldLabel("Resolution");
    ImGui::SetNextItemWidth(kToolbarComboWidth);
    ImGui::Combo("##resolution_display", &r.sim_resolution_index, kSimResolutionLabels, kSimResolutionCount);
    MarkSimTierEdge();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Re-runs simulation; accumulated rays reset");
    }
    InlineFieldLabel("EV");
    const FieldEditorConstraint ev_c = ConstraintFor("renderer.exposure_offset", g_state);
    ImGui::SetNextItemWidth(kCompactFieldWidth);
    DragFloatField("EV##display", &r.exposure_offset, static_cast<float>(ev_c.min_value),
                   static_cast<float>(ev_c.max_value), ev_c.fmt, ev_c.scale);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Exposure value offset for display brightness");
    }

    // ---- Column 2: the aspect ratio the preview region is fitted to.
    ImGui::TableSetColumnIndex(1);
    int preset_idx = static_cast<int>(g_state.aspect_preset);
    const char* preview_label = kAspectPresetNames[preset_idx];
    // The orientation button MODIFIES the preset, so it shares the preset's row (AC4 /
    // doc/gui-visual-language.md §4.6). It used to sit on a row of its own, which read as a second,
    // independent control.
    const char* flip_label = g_state.aspect_portrait ? "Portrait" : "Landscape";
    InlineFieldLabel("Aspect");
    ImGui::SetNextItemWidth(kAspectPresetComboWidth);
    if (ImGui::BeginCombo("##aspect_preset_display", preview_label)) {
      for (int i = 0; i < kAspectPresetCount; i++) {
        bool disabled = AspectPresetOptionDisabled(static_cast<AspectPreset>(i), g_preview.HasBackground());
        ImGui::BeginDisabled(disabled);
        bool selected = (i == preset_idx);
        if (ImGui::Selectable(kAspectPresetNames[i], selected)) {
          g_state.aspect_preset = static_cast<AspectPreset>(i);
          ApplyAspectRatio(window, g_state.aspect_preset, g_state.aspect_portrait);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
        ImGui::EndDisabled();
      }
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(AspectFlipDisabled(g_state.aspect_preset));
    if (ImGui::Button(flip_label)) {
      g_state.aspect_portrait = !g_state.aspect_portrait;
      ApplyAspectRatio(window, g_state.aspect_preset, g_state.aspect_portrait);
    }
    ImGui::EndDisabled();

    // Screen-too-small warning: rendered only when the requested aspect could
    // not be honored AND the user is still on a non-Free preset (the
    // ApplyAspectRatio path already clears aspect_clamp on Free / kMatchBg-no-bg,
    // but we re-check here so a stale signal from a missed callback path
    // cannot leak through).
    if (g_state.aspect_clamp.was_clamped && g_state.aspect_preset != AspectPreset::kFree) {
      // Disabled Selectable for the static header (ImGui::Text* widgets are
      // emitted with id=0 so they cannot be located by the GUI test engine;
      // disabled Selectable still calls ItemAdd with a real ID derived from
      // the label, so it is addressable while remaining non-interactive).
      // Dynamic ratio detail follows as a plain Text below.
      ImGui::PushStyleColor(ImGuiCol_Text, WarningTextColor());
      ImGui::Selectable("Screen too small for this aspect", false, ImGuiSelectableFlags_Disabled);
      ImGui::Text("preview ~%.2f:1, export %.2f:1", g_state.aspect_clamp.achieved_preview_ratio,
                  g_state.aspect_clamp.requested_preview_ratio);
      ImGui::PopStyleColor();
    }

    // ---- Column 3: the background image shown under the result.
    ImGui::TableSetColumnIndex(2);
    if (ImGui::Button("Load Bg##display")) {
      DoLoadBackground(window);
    }
    ImGui::SameLine();
    bool no_bg = !g_preview.HasBackground();
    ImGui::BeginDisabled(no_bg);
    if (ImGui::Button("Clear##display_bg")) {
      DoClearBackground();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(no_bg);
    Checkbox("Show##display_bg", &g_state.bg_show);
    // bg_alpha's own gate (WhenBackgroundShown) already covers BOTH "no image loaded" and "image
    // hidden", so it subsumes the outer BeginDisabled(no_bg) this sits inside. The outer one stays
    // because it also wraps the Show checkbox, which is not this field's control; a doubly-pushed
    // disabled state is idempotent.
    const FieldEditorConstraint bg_alpha_c = ConstraintFor("bg_alpha", g_state);
    ImGui::BeginDisabled(!bg_alpha_c.enabled);
    InlineFieldLabel("Alpha");
    ImGui::SetNextItemWidth(kCompactFieldWidth);
    DragFloatField("Alpha##display", &g_state.bg_alpha, static_cast<float>(bg_alpha_c.min_value),
                   static_cast<float>(bg_alpha_c.max_value), bg_alpha_c.fmt, bg_alpha_c.scale);
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    ImGui::EndTable();
  }
}

// The angle list behind the Angular Distance row's fold: presets, a custom-angle input, and the
// current list with per-entry delete.
//
// A named function rather than statements inside the row loop. A table row IS a loop body here, so
// a popup written inline would be built once per row — four popups sharing one name, of which the
// last one submitted wins. Having exactly one construction site is a property worth being able to
// check by reading, not by trusting the loop's shape to stay what it is today.
void RenderSunCirclesAnglePopup() {
  bool at_limit = SunCirclesAtLimit(g_state.sun_circle_angles.size());

  // Preset buttons
  const float presets[] = { 9.0f, 22.0f, 28.0f, 46.0f };
  for (float p : presets) {
    const bool already = SunCircleAlreadyPresent(g_state.sun_circle_angles, p);
    char label[16];
    std::snprintf(label, sizeof(label), "%.0f\xc2\xb0", p);
    ImGui::BeginDisabled(already || at_limit);
    if (ImGui::Button(label)) {
      g_state.sun_circle_angles.push_back(p);
      std::sort(g_state.sun_circle_angles.begin(), g_state.sun_circle_angles.end());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
  }
  ImGui::NewLine();

  // Custom angle input. This popup hangs off the Overlays tab's Angular Distance row, so it is
  // inside the display strip's width-token scope even though it is a separate ImGui window: a
  // single numeric field in a horizontal row is exactly what kCompactFieldWidth names. The 60.0f
  // it replaces was within 2 px of the tier anyway.
  static float custom_angle = 22.0f;
  ImGui::PushItemWidth(kCompactFieldWidth);
  ImGui::InputFloat("##custom_angle", &custom_angle, 0.0f, 0.0f, "%.1f");
  ImGui::PopItemWidth();
  ImGui::SameLine();
  ImGui::BeginDisabled(at_limit);
  if (ImGui::Button("+##add_circle")) {
    custom_angle = ClampSunCircleAngle(custom_angle);
    g_state.sun_circle_angles.push_back(custom_angle);
    std::sort(g_state.sun_circle_angles.begin(), g_state.sun_circle_angles.end());
  }
  ImGui::EndDisabled();

  // Current list with delete buttons
  ImGui::Separator();
  int remove_idx = -1;
  for (int i = 0; i < static_cast<int>(g_state.sun_circle_angles.size()); i++) {
    ImGui::Text("%.1f\xc2\xb0", g_state.sun_circle_angles[i]);
    ImGui::SameLine();
    char del_label[32];
    std::snprintf(del_label, sizeof(del_label), "x##del_%d", i);
    if (ImGui::SmallButton(del_label)) {
      remove_idx = i;
    }
  }
  if (remove_idx >= 0) {
    g_state.sun_circle_angles.erase(g_state.sun_circle_angles.begin() + remove_idx);
  }
}

// The pixel radius behind the Zenith/Nadir row's fold. It is the one field only that row has, and
// giving it a column of its own would have cost every other row an empty cell (and the name column
// the width) to say something about one of the four — the fold is what buys "Angular Distance" its
// uncut name (doc/gui-visual-language.md §4.4).
void RenderZenithNadirRadiusPopup() {
  const FieldEditorConstraint zn_r_c = ConstraintFor("overlay_zenith_nadir_radius_px", g_state);
  SliderWithInput("Radius##zenith_nadir", &g_state.zenith_nadir_radius_px, static_cast<float>(zn_r_c.min_value),
                  static_cast<float>(zn_r_c.max_value), zn_r_c.fmt, zn_r_c.scale);
}

// What a row's fold holds, when it holds anything. Two of the four overlays have a field the others
// do not, and they are not the same field.
enum class OverlayFold {
  kNone,
  kSunCircleAngles,
  kZenithNadirRadius,
};

// One auxiliary line, as the table reads it. Every row answers the same questions — colour, name,
// line, text label, opacity — which is exactly why the table is the right shape for them
// (doc/gui-visual-language.md §4.4). `label` is null for a row that draws no text label at all;
// that cell is then left EMPTY rather than filled with a disabled control or an explanatory word,
// because "this one has no text" is what an empty cell in a labelled column already says.
struct OverlayRowSpec {
  const char* name;
  const char* color_id;
  float* color;
  const char* line_id;
  bool* line;
  const char* label_id;  // null ⇒ no text label for this overlay; the cell stays empty.
  bool* label;
  const char* alpha_id;
  const char* alpha_field;
  float* alpha;
  // Triple hash, unlike every other id above, and not a slip: the fold button's label carries a
  // visible glyph, and ImGui hashes the WHOLE label for "glyph##suffix" — the id would then contain
  // the icon codepoint, so renaming the icon would silently rename the item. "###suffix" hashes the
  // suffix alone. Null for a row with no fold.
  const char* fold_id;
  OverlayFold fold;
};

// The Overlays tab: the four auxiliary lines drawn over the preview, as one table.
//
// It replaces four stacked two-row blocks that repeated the word "Alpha" four times and anchored
// their checkboxes at an x computed from the width of the longest name — an arrangement in which
// adding a name longer than "Angular Distance" silently overlapped the Line column.
void RenderOverlaysTab() {
  const OverlayRowSpec rows[] = {
    { "Horizon", "##horizon_color", g_state.horizon_color, "##horizon_line", &g_state.show_horizon_line,
      "##horizon_label", &g_state.show_horizon_label, "##horizon_alpha", "overlay_horizon_alpha",
      &g_state.horizon_alpha, nullptr, OverlayFold::kNone },
    { "Grid", "##grid_color", g_state.grid_color, "##grid_line", &g_state.show_grid_line, "##grid_label",
      &g_state.show_grid_label, "##grid_alpha", "overlay_grid_alpha", &g_state.grid_alpha, nullptr,
      OverlayFold::kNone },
    { "Angular Distance", "##sun_circles_color", g_state.sun_circles_color, "##sun_circles_line",
      &g_state.show_sun_circles_line, "##sun_circles_label", &g_state.show_sun_circles_label, "##sun_circles_alpha",
      "overlay_sun_circles_alpha", &g_state.sun_circles_alpha, "###sun_circles_fold", OverlayFold::kSunCircleAngles },
    // The marker pair: pixel-space dots at the zenith and the nadir. No text label — hence a null
    // label id — and the only row with a radius.
    { "Zenith/Nadir", "##zenith_nadir_color", g_state.zenith_nadir_color, "##zenith_nadir_line",
      &g_state.show_zenith_nadir_line, nullptr, nullptr, "##zenith_nadir_alpha", "overlay_zenith_nadir_alpha",
      &g_state.zenith_nadir_alpha, "###zenith_nadir_fold", OverlayFold::kZenithNadirRadius },
  };

  const float swatch_w = ImGui::GetFrameHeight();
  const float check_w = ImGui::GetFrameHeight();
  const float fold_w = ImGui::GetFrameHeight();
  constexpr float kAlphaColWidth = 90.0f;

  // Name is the ONLY stretching column. That is the mechanism behind "the name is not cut off":
  // every other column states the width it needs, and whatever is left goes to the names — rather
  // than the names getting what is left over after an x anchor derived from the longest of them.
  constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg;
  // Capped width, not the strip's full width. The strip is as wide as the viewport (~1200 px on the
  // default window), and a table stretched across all of it puts a row's name at one end and the
  // checkboxes that belong to it at the other, with a screen's worth of empty row in between —
  // measured on a capture before this cap existed. The cap is a maximum, not a fixed width: below
  // it the table still shrinks with the window, so the Name column's stretch keeps doing the job
  // the fixed columns' declared widths leave it (see the note above kFlags).
  constexpr float kMaxTableWidth = 560.0f;
  const ImVec2 outer_size(std::min(ImGui::GetContentRegionAvail().x, kMaxTableWidth), 0.0f);
  if (!ImGui::BeginTable("##OverlaysTable", 6, kFlags, outer_size)) {
    return;
  }
  ImGui::TableSetupColumn("##color", ImGuiTableColumnFlags_WidthFixed, swatch_w);
  ImGui::TableSetupColumn("Overlay", ImGuiTableColumnFlags_WidthStretch);
  ImGui::TableSetupColumn("Line", ImGuiTableColumnFlags_WidthFixed, std::max(check_w, ImGui::CalcTextSize("Line").x));
  ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, std::max(check_w, ImGui::CalcTextSize("Label").x));
  ImGui::TableSetupColumn("Alpha", ImGuiTableColumnFlags_WidthFixed, kAlphaColWidth);
  ImGui::TableSetupColumn("##fold", ImGuiTableColumnFlags_WidthFixed, fold_w);
  ImGui::TableHeadersRow();

  for (const OverlayRowSpec& row : rows) {
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    ImGui::ColorEdit3(row.color_id, row.color, ImGuiColorEditFlags_NoInputs);

    ImGui::TableSetColumnIndex(1);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(row.name);

    ImGui::TableSetColumnIndex(2);
    Checkbox(row.line_id, row.line);

    // Empty cell, on purpose — see OverlayRowSpec::label.
    if (row.label != nullptr) {
      ImGui::TableSetColumnIndex(3);
      Checkbox(row.label_id, row.label);
    }

    ImGui::TableSetColumnIndex(4);
    const FieldEditorConstraint alpha_c = ConstraintFor(row.alpha_field, g_state);
    // A single DragFloat rather than the [slider][input] pair the panel used: the pair needs about
    // twice this cell's width, and the width it would take comes straight off the name column
    // (doc/gui-visual-language.md §7 records the cell-sized single control as the verified form).
    // AlwaysClamp so the ctrl+click text entry honours the registry's domain like the drag does.
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::DragFloat(row.alpha_id, row.alpha, 0.005f, static_cast<float>(alpha_c.min_value),
                     static_cast<float>(alpha_c.max_value), alpha_c.fmt, ImGuiSliderFlags_AlwaysClamp);

    if (row.fold == OverlayFold::kNone) {
      continue;  // Empty fold cell: this overlay has no field the others lack.
    }
    ImGui::TableSetColumnIndex(5);
    // The angle list is offered only while the circles are actually drawn — editing the angles of
    // something invisible is a control with no feedback. The radius has no such gate: the markers'
    // own Line checkbox is right there in the same row.
    const bool circles_shown = g_state.show_sun_circles_line || g_state.show_sun_circles_label;
    if (row.fold == OverlayFold::kSunCircleAngles && !circles_shown) {
      continue;
    }
    if (ImGui::SmallButton((std::string(ICON_FA_ELLIPSIS) + row.fold_id).c_str())) {
      ImGui::OpenPopup(row.fold_id);
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(row.fold == OverlayFold::kSunCircleAngles ? "Edit angles" : "Marker radius");
    }
    if (ImGui::BeginPopup(row.fold_id)) {
      if (row.fold == OverlayFold::kSunCircleAngles) {
        RenderSunCirclesAnglePopup();
      } else {
        RenderZenithNadirRadiusPopup();
      }
      ImGui::EndPopup();
    }
  }

  ImGui::EndTable();
}

}  // namespace

void ResetDisplayStripSelectionForTest() {
  g_strip_select_grade = true;
}

void RenderDisplayStrip(GLFWwindow* window, float window_width, float window_height) {
  const CentralBand band = GetCentralBand(window_width, window_height);
  const ViewportStripSplit split = SplitViewportForDisplayStrip(band.y, band.h, kDisplayStripHeight);

  SetNextPanelGeometry(band.x, split.strip_y, band.w, split.strip_h);
  // Fixed-geometry chrome, like the top bar and the status bar and unlike the document column: it is
  // glued to the viewport's bottom edge because that is where it says something about the picture
  // above it (doc/gui-layout-architecture.md §4). NoDocking states that — dragged into the dockspace
  // it would take space from the layout with no way back except View -> Reset Layout. A background
  // (no NoBackground flag) is deliberate: the strip is chrome, not a hole onto the GL preview.
  ImGui::Begin("##DisplayStrip", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoDocking |
                   ImGuiWindowFlags_NoBringToFrontOnFocus);

  if (ImGui::BeginTabBar("##DisplayStripTabs")) {
    const ImGuiTabItemFlags grade_flags = g_strip_select_grade ? ImGuiTabItemFlags_SetSelected : 0;
    g_strip_select_grade = false;
    if (ImGui::BeginTabItem("Grade", nullptr, grade_flags)) {
      BeginStripTabContent("##GradeTab");
      RenderGradeTab(window);
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Overlays")) {
      BeginStripTabContent("##OverlaysTab");
      RenderOverlaysTab();
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    // A reserved slot, and deliberately nothing more. What goes here is the per-raypath colour
    // analysis of doc/gui-custom-spectrum-and-raypath-color.md — a legend of one row per light-path
    // component, the same shape as the Overlays table above it. The slot exists now because the
    // strip's edge and height were chosen to fit that future tenant (see kDisplayStripHeight and
    // the strip's bottom-edge placement, doc/gui-layout-architecture.md §4/§6); leaving it out
    // would have made "does this layout hold it" unanswerable until the day it lands.
    if (ImGui::BeginTabItem("Components")) {
      BeginStripTabContent("##ComponentsTab");
      ImGui::TextDisabled("Light-path component analysis lands here.");
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::End();
}

void RenderStatusBar(float window_width, float window_height) {
  SetNextPanelGeometry(0, window_height - kStatusBarHeight, window_width, kStatusBarHeight);
  ImGui::Begin("##StatusBar", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);

  const ImGuiStyle& style = ImGui::GetStyle();
  const float spacing = style.ItemSpacing.x;

  // The row reads left to right as "what is happening / to what / in which file", and right to
  // left as "how much has been traced / where the log is". The two counters below therefore belong
  // to the right cluster, and a right-aligned run of items has to know its own width before it can
  // place its first item — so they are formatted here, before anything is drawn, and the block that
  // draws them further down consumes exactly these strings. Measuring one string and drawing
  // another is how a cluster like this ends up hanging off the window edge.
  std::string total_rays;
  std::string sampling;
  if (g_state.stats_sim_ray_num > 0) {
    const LUMICE_RayCount n = g_state.stats_sim_ray_num;
    char buf[64];
    // "Total" is the summed count over all discrete wavelengths. `ray_num` is itself the requested
    // total, so the actual traced count reported here is ceil(ray_num / N_wavelengths) x
    // N_wavelengths (it may overshoot by fewer than N).
    if (n >= 1'000'000'000ULL) {
      snprintf(buf, sizeof(buf), "Total rays: %.1f x10^9", n / 1e9);
    } else if (n >= 1'000'000ULL) {
      snprintf(buf, sizeof(buf), "Total rays: %.1f x10^6", n / 1e6);
    } else {
      snprintf(buf, sizeof(buf), "Total rays: %.1f x10^3", n / 1e3);
    }
    total_rays = buf;

    // Sampling density. Deliberately under the SAME "a run has happened" gate as "Total rays" and
    // under no additional condition of its own: whether a dimension is randomized is exactly what
    // this readout is for, so hiding it when a dimension is fixed would suppress the answer in the
    // case the user most needs it ("1 per 5.4 x10^6 rays" IS the explanation for an over-sharp
    // render).
    //
    // Plain text on purpose: no progress bar, no colour grading, no check/cross. Neither counter
    // has a "good" value -- a low shape count is correct for a fixed shape and expected on the GPU
    // route -- so any better/worse styling here would manufacture false alarms.
    //
    // NOTE: this segment builds its text in a pure function (app.cpp) while "Total rays" above
    // formats inline. The inconsistency is deliberate, not an oversight -- do NOT "unify" it by
    // inlining this one. `ImGui::Text`/`TextUnformatted` submit an item ID of 0, so a test cannot
    // address the rendered string through the item API; extracting the text lets the string itself
    // be asserted, with a separate pixel test proving it reaches the framebuffer. A future status
    // bar segment that wants test coverage should follow THIS pattern rather than the inline one.
    sampling =
        FormatSamplingSegment(g_state.stats_crystal_num, g_state.stats_orientation_num, g_state.stats_sim_ray_num);
  }

  // ---- Left cluster: state, scene, file ----
  //
  // The state leads with a filled dot in its own colour. The word alone had to carry the state
  // through colour applied to the text itself, which is the weakest place to put it: at 15 px a
  // coloured word is a few dozen tinted pixels competing with everything else on the row, and the
  // reader has to be looking AT it to see the colour. A dot is a solid disc of that colour with
  // nothing else in it -- it is the same information, made legible peripherally, and it costs one
  // glyph.
  {
    const char* label = nullptr;
    ImVec4 color(1.0f, 1.0f, 1.0f, 1.0f);
    switch (g_state.sim_state) {
      case SimState::kIdle:
        label = ICON_FA_CIRCLE " Ready";
        color = GoodTextColor();
        break;
      case SimState::kSimulating:
        label = ICON_FA_CIRCLE " Simulating...";
        color = ImVec4(1.0f, 0.8f, 0.0f, 1.0f);
        break;
      case SimState::kStopping:
        label = ICON_FA_CIRCLE " Stopping...";
        color = ImVec4(1.0f, 0.6f, 0.0f, 1.0f);
        break;
      case SimState::kDone:
        label = ICON_FA_CIRCLE " Done";
        color = ImVec4(0.3f, 0.7f, 1.0f, 1.0f);
        break;
      case SimState::kModified:
        label = ICON_FA_CIRCLE " Modified";
        color = WarningTextColor();
        break;
    }
    if (label != nullptr) {
      ImGui::TextColored(color, "%s", label);
    }
  }

  MiddleDot();

  // Sim resolution + lens info (renderer is always embedded in GuiState).
  {
    const auto& rc = g_state.renderer;
    const int res = kSimResolutions[rc.sim_resolution_index];
    ImGui::TextDisabled("%dx%d  %s  FOV:%.0f", res, res / 2, kLensTypeNames[rc.lens_type], rc.fov);
  }

  MiddleDot();

  if (g_state.current_file_path.empty()) {
    ImGui::TextDisabled("No file");
  } else {
    const auto filename = g_state.current_file_path.filename().u8string();
    if (g_state.dirty) {
      ImGui::Text("%s *", filename.c_str());
    } else {
      ImGui::Text("%s", filename.c_str());
    }
  }

  // ---- Right cluster: the run's counters, then the Log toggle, flush to the right edge ----
  //
  // The counters sit here rather than in the left run because they are the only segments of this
  // row whose WIDTH moves on their own -- a ray count crosses a magnitude and the text under it
  // changes length. Left-ordered, that shifted every segment after them; against the right edge,
  // the growth happens into the gap in the middle of the row, where there is nothing to push.
  //
  // The colored/full-spectrum mode toggle that used to live at this end moved to the top bar next
  // to the Colors button, which is why the Log button is the only control left down here.
  {
    const char* log_label = g_state.log_panel_open ? ICON_FA_CHEVRON_DOWN " Log" : ICON_FA_CHEVRON_RIGHT " Log";
    // SmallButton's own frame padding is (FramePadding.x, 0), so the width is the label plus the
    // horizontal padding twice, same as a full-height button.
    float cluster_w = TextWidth(log_label) + style.FramePadding.x * 2.0f;
    if (!total_rays.empty()) {
      cluster_w += TextWidth(total_rays.c_str()) + spacing + TextWidth(kMiddleDot) + spacing +
                   TextWidth(sampling.c_str()) + spacing;
    }
    ImGui::SameLine(ImGui::GetWindowWidth() - cluster_w - style.WindowPadding.x);

    if (!total_rays.empty()) {
      ImGui::TextDisabled("%s", total_rays.c_str());
      MiddleDot();
      ImGui::TextDisabled("%s", sampling.c_str());
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", FormatSamplingTooltip(g_state.stats_crystal_num, g_state.stats_orientation_num,
                                                      g_state.stats_sim_ray_num)
                                    .c_str());
      }
      ImGui::SameLine();
    }
    if (ImGui::SmallButton(log_label)) {
      g_state.log_panel_open = !g_state.log_panel_open;
    }
  }

  ImGui::End();
}

// Pending message text for the Import Warning modal. Filled by
// SetImportComplexFilterWarning from the JSON import path; consumed (and
// cleared) by RenderImportWarningPopup when the user dismisses the modal.
namespace {
std::string g_pending_import_warning;
}  // namespace

void SetImportComplexFilterWarning(const std::string& msg) {
  if (!g_pending_import_warning.empty()) {
    g_pending_import_warning += "\n";
  }
  g_pending_import_warning += msg;
}

std::string PeekImportComplexFilterWarning() {
  return g_pending_import_warning;
}

void ClearImportComplexFilterWarning() {
  g_pending_import_warning.clear();
}

void RenderImportWarningPopup() {
  static std::string active_msg;
  if (!g_pending_import_warning.empty()) {
    active_msg = std::move(g_pending_import_warning);
    // A moved-from std::string is valid-but-unspecified, not guaranteed empty;
    // clear() makes the trigger false next frame so the popup opens once.
    g_pending_import_warning.clear();
    ImGui::OpenPopup("Import Warning");
  }

  if (ImGui::BeginPopupModal("Import Warning", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("Some settings in the imported config could not be fully represented in the GUI:");
    ImGui::Separator();
    ImGui::TextUnformatted(active_msg.c_str());
    ImGui::Separator();
    ImGui::TextUnformatted("They were simplified on load. Edit the config file / CLI directly to keep the originals.");
    if (ImGui::Button("OK", ImVec2(80, 0))) {
      active_msg.clear();
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

// See app.hpp for why this sentence is a named constant and what is asserted about it.
const char* const kExportOverwriteWarningText =
    "Exporting replaces it with what the GUI can express, so anything in it the GUI\n"
    "cannot represent will be lost. Save as .lmc instead to keep this project\n"
    "without touching that file.";

// Export config JSON, onto a path that already holds a file. Same modal idiom as
// RenderSaveModifiedPopup above (BeginPopupModal + explicit buttons, so Escape and click-outside
// cannot dismiss it — see the note there for why that holds for every modal in this app), because
// the same thing is being protected: a write the user cannot undo.
void RenderExportOverwriteConfirmPopup() {
  if (g_show_export_overwrite_confirm_popup) {
    ImGui::OpenPopup("Overwrite Config File");
    g_show_export_overwrite_confirm_popup = false;
  }

  if (ImGui::BeginPopupModal("Overwrite Config File", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextDisabled("A file already exists at:");
    ImGui::TextUnformatted(PathToU8(g_pending_export_json_path).c_str());
    ImGui::Separator();
    ImGui::TextUnformatted(kExportOverwriteWarningText);
    ImGui::Separator();

    if (ImGui::Button("Overwrite", ImVec2(120, 0))) {
      ConfirmPendingConfigJsonExport();
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      CancelPendingConfigJsonExport();
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

// Generic GUI warning modal (not import-specific). Fires ONCE per distinct warning episode:
// SetGuiWarning is idempotent while the same message is in-flight, so a persistent condition
// re-detected on every debounced commit (e.g. an over-bounds filter re-checked on each 70ms
// DoRun while the user drags an unrelated slider) does NOT re-open the modal and freeze
// interaction. ClearGuiWarning (called on a successful commit) re-arms it so a later
// re-occurrence of the same condition warns again.
//
// Identity contract (the message text IS the episode key): callers MUST guarantee that one
// logical warning event always produces byte-identical text, and that distinct events use
// distinct text. Today the only caller (DoRun over-bounds) uses a constant string, so this
// holds. Before a second caller reuses this modal, promote the key from message content to an
// explicit warning-id/source enum (text becomes display-only) to avoid text collisions
// (two events, same wording -> new warning swallowed) or wording drift (one event, changed
// wording -> spurious re-open).
std::string g_gui_warning_current;   // "" = none; else the message currently in-flight
bool g_gui_warning_trigger = false;  // request OpenPopup on the next render

void SetGuiWarning(const std::string& msg) {
  if (msg == g_gui_warning_current) {
    return;  // this exact warning is already showing / dismissed — do not re-open (anti-spam)
  }
  g_gui_warning_current = msg;
  g_gui_warning_trigger = true;
}

void ClearGuiWarning() {
  g_gui_warning_current.clear();
  g_gui_warning_trigger = false;
}

// Test accessor: the message currently in-flight ("" if none).
std::string PeekGuiWarning() {
  return g_gui_warning_current;
}

// Test accessor: is a modal re-open pending on the next render? See app.hpp.
bool IsGuiWarningPending() {
  return g_gui_warning_trigger;
}

namespace internal_test {
void ConsumeGuiWarningPending() {
  g_gui_warning_trigger = false;
}
}  // namespace internal_test

void RenderGuiWarningPopup() {
  if (g_gui_warning_trigger) {
    g_gui_warning_trigger = false;
    ImGui::OpenPopup("Warning");
  }
  if (ImGui::BeginPopupModal("Warning", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted(g_gui_warning_current.c_str());
    ImGui::Separator();
    if (ImGui::Button("OK", ImVec2(80, 0))) {
      // Keep g_gui_warning_current set so the same persistent condition, re-detected on the
      // next debounced commit, is deduped (does not re-open). ClearGuiWarning (on a successful
      // commit) re-arms it.
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

// Executes the New/Open/Quit action queued in g_pending_action (if any) and
// clears it. Shared by the three button branches below and in
// RenderSaveModifiedPopup that resume a deferred action once it is safe to do
// so (code-review-02 M2: was three independent copies of the same 4-branch
// switch; consolidated to a single call site).
void ResolvePendingAction(GLFWwindow* window) {
  switch (g_pending_action) {
    case PendingAction::kNew:
      DoNew();
      break;
    case PendingAction::kOpen:
      DoOpen();
      break;
    case PendingAction::kQuit:
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      break;
    default:
      break;
  }
  g_pending_action = PendingAction::kNone;
}

void RenderUnsavedPopup(GLFWwindow* window) {
  if (g_show_unsaved_popup) {
    ImGui::OpenPopup("Unsaved Changes");
    g_show_unsaved_popup = false;
  }

  if (ImGui::BeginPopupModal("Unsaved Changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("You have unsaved changes. Save before continuing?");
    ImGui::Separator();

    if (ImGui::Button("Save", ImVec2(80, 0))) {
      // task-cleanup-hardening AC4 code-review-01 M1: route through the
      // kModified gate (DoSave) instead of bypassing it via PerformSave. If
      // sim_state == kModified, DoSave() defers to RenderSaveModifiedPopup and
      // leaves g_pending_action queued — its three branches below decide
      // whether the deferred New/Open/Quit actually runs. If not modified,
      // DoSave() falls through to PerformSave() synchronously, same as before.
      DoSave();
      if (!g_show_save_modified_popup) {
        ResolvePendingAction(window);
      }
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Don't Save", ImVec2(100, 0))) {
      ResolvePendingAction(window);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      g_pending_action = PendingAction::kNone;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

// task-cleanup-hardening AC4: RenderSaveModifiedPopup — the "Save while
// sim_state == kModified" prompt. The owner ruling (issue.md §偏离-E) is that
// silently serializing "stale preview + fresh config + dirty=false" and
// clearing Modified is a bug: the .lmc records a preview that does NOT match
// its own config, and the user loses the visual warning that a re-run is
// needed. This popup gates that flow — the user picks one of three:
//   - "Run first"  : DoRun() to produce a fresh preview matching the current
//                    config, then close the popup; the user re-invokes Save
//                    when ready. Disabled if no live server (kIdle).
//   - "Save anyway": PerformSave / PerformSaveAs (bypass the check). Freezes
//                    the last committed run's preview into the .lmc — legit
//                    when the user knowingly wants to snapshot pre-edit state.
//   - "Cancel"     : Clears the pending save kind and closes; no side effect.
//
// code-review-01 M1: this popup can also be reached via RenderUnsavedPopup's
// "Save" button (DoSave() defers here when kModified), which leaves a
// g_pending_action (New/Open/Quit) queued. Only "Save anyway" — the branch
// that actually performs a serialization — resumes that deferred action;
// "Run first" and "Cancel" both clear it, since neither persists anything and
// silently proceeding with New/Open/Quit would discard the edit the original
// Unsaved-changes prompt was protecting. When this popup is opened directly
// from the top-bar Save button, g_pending_action is always kNone already, so
// clearing/switching on it here is a no-op in that path.
void RenderSaveModifiedPopup(GLFWwindow* window) {
  // window is used by the "Save anyway" branch's chained kQuit case (see
  // code-review-01 M1 doc comment above).
  //
  // code-review-02 M1 investigated whether Escape can dismiss this modal
  // without going through any of the three buttons below, which would leave
  // g_pending_save_kind / g_pending_action stale for a later, unrelated Save
  // to misfire on. Verified NOT reachable in this codebase: Dear ImGui's
  // NavUpdateCancelRequest() (imgui.cpp) only routes Escape to
  // ClosePopupToLevel() when the topmost open popup does NOT have
  // ImGuiWindowFlags_Modal set — BeginPopupModal always sets that flag, so
  // Escape never reaches the close path for ANY modal in this app,
  // regardless of p_open or window flags (io.ConfigFlags has
  // ImGuiConfigFlags_NavEnableKeyboard on in both main.cpp:126 and
  // test_gui_main.cpp:264, so the nav-active precondition is satisfied — the
  // modal exclusion is what actually blocks it). Modals also block
  // click-outside-to-close by design. The only exits are the three buttons.
  // gui_test file_ops/escape_does_not_dismiss_the_modified_prompt
  // (test/gui/functional/test_file_ops.cpp) pins this via a real
  // ctx->KeyPress(ImGuiKey_Escape) against the live popup.
  if (g_show_save_modified_popup) {
    ImGui::OpenPopup("Save Modified Config");
    g_show_save_modified_popup = false;
  }

  if (ImGui::BeginPopupModal("Save Modified Config", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("The config has been modified since the last simulation run.");
    ImGui::Text("The on-screen preview reflects the previous config, not the current one.");
    ImGui::Separator();

    // "Run first" is only meaningful when there is a live server to run on and no run is already
    // inflight. The second half is the SAME "busy" notion the top bar gates New/Open/Save on —
    // shared through sim_state_rules.hpp rather than restated here, which is what the note that
    // used to sit at this line ("single-source would be nicer but the top bar's enable predicate
    // is inlined and not exported") asked for. The two gates are still distinct predicates: this
    // one additionally requires a live server.
    const bool can_run = CanRunFromModal(g_server != nullptr, g_state.sim_state);
    ImGui::BeginDisabled(!can_run);
    if (ImGui::Button("Run first", ImVec2(100, 0))) {
      DoRun(/*user_initiated=*/true);
      g_pending_save_kind = PendingSaveKind::kNone;
      // Abort any chained New/Open/Quit (see doc comment above) — Run doesn't
      // persist anything, so proceeding now would still discard the edit.
      g_pending_action = PendingAction::kNone;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Save anyway", ImVec2(120, 0))) {
      switch (g_pending_save_kind) {
        case PendingSaveKind::kSave:
          PerformSave();
          break;
        case PendingSaveKind::kSaveAs:
          PerformSaveAs();
          break;
        case PendingSaveKind::kNone:
          break;
      }
      g_pending_save_kind = PendingSaveKind::kNone;
      // Resume the New/Open/Quit deferred by RenderUnsavedPopup's Save button
      // (see doc comment above) — a real save just happened, so it's safe to
      // proceed. No-op when this popup was opened directly (g_pending_action
      // is kNone in that path).
      ResolvePendingAction(window);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      g_pending_save_kind = PendingSaveKind::kNone;
      // Abort any chained New/Open/Quit — see doc comment above.
      g_pending_action = PendingAction::kNone;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

void RenderLogPanel(float window_width, float window_height) {
  if (!g_imgui_log_sink) {
    return;
  }

  constexpr float kLogPanelHeight = 250.0f;

  if (!g_state.log_panel_open) {
    return;
  }

  SetNextPanelGeometry(0, window_height - kLogPanelHeight - kStatusBarHeight, window_width, kLogPanelHeight);
  // ##LogPanel intentionally does NOT carry NoBringToFrontOnFocus: it
  // belongs to Layer 3 (floating, raisable) per the z-order convention block
  // at the top of this file. ImGui creates NoBringToFrontOnFocus windows via
  // push_front (= bottom of g.Windows) and others via push_back (= top), so
  // adding the flag here would push LogPanel into the background cluster
  // BELOW the document column / display strip — the opposite of the desired stacking.
  // NoDocking: this panel keeps its own fixed geometry above the status bar. Without the flag a user
  // could drag it into the main DockSpace, where it would take space away from the panels the
  // default layout is built from and never come back on its own.
  ImGui::Begin(
      "##LogPanel", &g_state.log_panel_open,
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking);

  // Config controls row
  static const char* const kLevelNames[] = { "Trace", "Debug", "Verbose", "Info", "Warning", "Error", "Off" };
  static const LUMICE_LogLevel kLevelMap[] = { LUMICE_LOG_TRACE, LUMICE_LOG_DEBUG,   LUMICE_LOG_VERBOSE,
                                               LUMICE_LOG_INFO,  LUMICE_LOG_WARNING, LUMICE_LOG_ERROR,
                                               LUMICE_LOG_OFF };

  ImGui::TextDisabled("GUI");
  ImGui::SameLine();
  ImGui::PushItemWidth(80);
  if (ImGui::Combo("##GuiLevel", &g_state.gui_log_level, kLevelNames, 7)) {
    SetGuiLogLevel(static_cast<spdlog::level::level_enum>(g_state.gui_log_level));
  }
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::TextDisabled("Core");
  ImGui::SameLine();
  ImGui::PushItemWidth(80);
  if (ImGui::Combo("##CoreLevel", &g_state.core_log_level, kLevelNames, 7)) {
    if (g_server) {
      LUMICE_SetLogLevel(g_server, kLevelMap[g_state.core_log_level]);
    }
  }
  ImGui::PopItemWidth();

  ImGui::SameLine();
  if (Checkbox("File", &g_state.log_to_file)) {
    if (g_file_log_sink) {
      g_file_log_sink->set_level(g_state.log_to_file ? spdlog::level::trace : spdlog::level::off);
    }
  }
  if (g_state.log_to_file) {
    ImGui::SameLine();
    ImGui::TextDisabled("%s", g_log_file_path.c_str());
  }

  ImGui::SameLine(ImGui::GetWindowWidth() - 60);
  if (ImGui::Button("Clear")) {
    g_imgui_log_sink->Clear();
  }

  ImGui::Separator();

  // Log content area with auto-scroll
  ImGui::BeginChild("LogContent", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);

  auto entry_count = g_imgui_log_sink->Size();
  ImGuiListClipper clipper;
  clipper.Begin(static_cast<int>(entry_count));

  // We need random access — collect visible entries via ForEachEntry with index filtering.
  // For simplicity and correctness with clipper, read all entries once per frame into a local cache.
  // This is acceptable because the deque is bounded to 4096 entries.
  struct CachedEntry {
    spdlog::level::level_enum level;
    const char* text;
  };
  static std::vector<LogEntry> frame_cache;
  frame_cache.clear();
  frame_cache.reserve(entry_count);
  g_imgui_log_sink->ForEachEntry([](size_t /*idx*/, const LogEntry& e) { frame_cache.push_back(e); });

  while (clipper.Step()) {
    for (int i = clipper.DisplayStart; i < clipper.DisplayEnd && i < static_cast<int>(frame_cache.size()); i++) {
      const auto& entry = frame_cache[i];
      ImVec4 color;
      switch (entry.level) {
        case spdlog::level::trace:
        case spdlog::level::debug:
          color = ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
          break;
        case spdlog::level::warn:
          color = WarningTextColor();
          break;
        case spdlog::level::err:
        case spdlog::level::critical:
          color = DestructiveTextColor();
          break;
        default:
          color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
          break;
      }
      ImGui::PushStyleColor(ImGuiCol_Text, color);
      ImGui::TextUnformatted(entry.message.c_str());
      ImGui::PopStyleColor();
    }
  }

  // Auto-scroll to bottom when new entries arrive
  if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20.0f) {
    ImGui::SetScrollHereY(1.0f);
  }

  ImGui::EndChild();
  ImGui::End();
}

}  // namespace lumice::gui
