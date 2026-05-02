#include "gui/edit_modals.hpp"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

#include "config/raypath_validation.hpp"
#include "gui/app.hpp"
#include "gui/axis_presets.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/crystal_renderer.hpp"
#include "gui/face_number_overlay.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "gui/raypath_segments.hpp"
#include "gui/window_sizing.hpp"
#include "imgui.h"

namespace lumice::gui {

namespace {

// Resets the Crystal tab's shape parameters in `c` to CrystalConfig{} defaults.
// Explicitly enumerates the 7 fields rather than `c = CrystalConfig{}` + restore
// metadata: a future field added to CrystalConfig is *not* silently swept into
// the reset (the developer must add a line here or accept the omission).
// Does not touch name / type / zenith / azimuth / roll — type lives in a radio
// button above, name is layer-scoped metadata, axis lives in a sibling tab.
void ResetCrystalShapeParams(CrystalConfig& c) {
  CrystalConfig defaults;
  c.height = defaults.height;
  c.prism_h = defaults.prism_h;
  c.upper_h = defaults.upper_h;
  c.lower_h = defaults.lower_h;
  c.upper_alpha = defaults.upper_alpha;
  c.lower_alpha = defaults.lower_alpha;
  std::copy(std::begin(defaults.face_distance), std::end(defaults.face_distance), std::begin(c.face_distance));
}

}  // namespace

// ============================================================
// Modal state (file scope — not shared via gui_state.hpp)
// ============================================================

enum class ActiveModal { kNone, kOpen };
enum class ActiveTab { kCrystal, kAxis, kFilter };

// Minimum width applied to the unified edit popup. Covers the two-column
// layout: the runtime left-pane width is
//   `kModalPreviewImageSize + 2×WindowPadding.x + 4`
// computed dynamically; ≈340 under the default style (WindowPadding.x=8),
// ≈348 with HiDPI themes that raise padding. The floor of 820 adds the
// right TabBar content minimum (~432) + child spacing (~16) + popup window
// padding (~32). AlwaysAutoResize still governs height; the constraint only
// adds a width floor so that both columns render without clipping.
constexpr float kEditModalMinWidth = 820.0f;

// Vertical layout (modal_layout_vertical=true) relaxes the horizontal floor
// since the tab content is stacked below the preview rather than beside it.
// Height is content-driven (AlwaysAutoResize): the bottom pane uses the same
// fixed height as the horizontal right pane (kPreviewChildHeight), so the
// vertical modal is exactly 2× the horizontal modal height plus chrome.
constexpr float kEditModalMinWidthVertical = 360.0f;
constexpr float kEditModalMinHeightVertical = 0.0f;

static ActiveModal g_active_modal = ActiveModal::kNone;
static int g_modal_layer_idx = -1;
static int g_modal_entry_idx = -1;

// Active tab is updated each frame inside the corresponding BeginTabItem true-branch
// (ImGui doesn't auto-write user state). The OpenEditModal path always sets it
// explicitly together with g_pending_tab_select=true to drive first-frame selection;
// at other times its value reflects the ImGui TabBar's currently active tab.
static ActiveTab g_active_tab = ActiveTab::kCrystal;
static bool g_pending_tab_select = false;

// Edit buffers
static CrystalConfig g_crystal_buf;
static AxisDist g_axis_buf[3];  // zenith, azimuth, roll

// Filter modal: type discriminator + per-type buffers (issue editor-modal-type-selection).
// Each FilterEditType keeps its own sub-buffer so switching type is non-destructive
// (T6 contract: "切到 EE 再切回 raypath, 之前输入仍在"). Commit assembles a
// FilterConfig from g_filter_top (shared fields) + active type's sub-buffer.
enum class FilterEditType { kRaypath = 0, kEntryExit = 1, kDirection = 2, kCrystal = 3 };
static FilterEditType g_filter_active_type = FilterEditType::kRaypath;
static FilterEditType g_filter_active_type_snapshot = FilterEditType::kRaypath;

// Shared filter fields (name / action / sym_*) — uses FilterConfig as the
// container so existing operator== works, but the `param` field is never
// mutated: it stays at default-constructed RaypathParams{} in both g_filter_top
// and its snapshot, so equality reduces to the shared fields only. Per-type
// payloads live in the dedicated *_params buffers below.
//
// INVARIANT: g_filter_top.param must remain default-constructed RaypathParams{}
// at all times. The split is "top fields here, payload in *_params"; writing to
// `param` would silently break the dirty-compare reduction since FilterConfig::
// operator== compares param too. If subtype expansion (#4-#6) introduces a real
// shared field beyond name/action/sym_*, prefer extracting a dedicated struct
// over reusing FilterConfig.
static FilterConfig g_filter_top;
static FilterConfig g_filter_top_snapshot;

// Per-type sub-buffers — switching type is non-destructive (each retains state).
static RaypathParams g_raypath_params;
static EntryExitParams g_ee_params;
static DirectionParams g_dir_params;
static CrystalParams g_crystal_params;
static RaypathParams g_raypath_params_snapshot;
static EntryExitParams g_ee_params_snapshot;
static DirectionParams g_dir_params_snapshot;
static CrystalParams g_crystal_params_snapshot;

// ImGui InputText backing store for the raypath text input. The raypath
// sub-buffer's raypath_text is synced FROM this char array (canonical source)
// at every commit / dirty-compare site (mirrors the pre-task convention).
static char g_raypath_buf[256];

// Initial-present flag captured at modal open. Used by ApplyBuffersToEntry so
// an untouched OK on a previously-empty filter does not silently materialize a
// default-constructed filter into entry.filter. Combined with the "empty
// raypath → nullopt" commit rule this also closes the "Remove button" path
// without a separate state bit.
static bool g_filter_initial_present = false;

// Snapshots captured on OpenEditModal for per-tab dirty-mark computation.
// Filter already has its own snapshot above (used for a separate purpose in
// CommitAllBuffers). Crystal / Axis were previously not snapshotted; they are
// now needed so the tab label can append " *" when the in-flight buffer
// diverges from the value at modal-open time.
static CrystalConfig g_crystal_buf_snapshot;
static AxisDist g_axis_buf_snapshot[3];

// Crystal modal: trackball state saved on open, restored on Cancel
static float g_saved_rotation[16];
static float g_saved_zoom;

// Crystal modal: mesh hash for edit buffer change detection
static int g_modal_mesh_hash = 0;

// Flag: the OpenPopup call must happen exactly once per modal open,
// on the frame following the EditRequest.
static bool g_pending_open = false;

// Flag: Immediate↔Staged switch is a close+reopen cycle that must bypass
// the Staged Cancel trackball-restore path in HandlePopupClosed; set by the
// checkbox handler in RenderEditModals, consumed on the next-frame close.
static bool g_pending_mode_switch = false;

// ============================================================
// Wedge angle presets (same as in panels.cpp SliderWithPreset for alpha)
// ============================================================

namespace {

struct ValuePreset {
  const char* label;
  float value;
};

constexpr ValuePreset kWedgePresets[] = {
  { "{1,0,-1,1} 28.0\xc2\xb0", 28.0f },
  { "{2,0,-2,1} 47.3\xc2\xb0", 47.3f },
  { "{1,0,-1,0} 90.0\xc2\xb0", 90.0f },
  { "{1,0,-1,2} 14.7\xc2\xb0", 14.7f },
};
constexpr int kWedgePresetCount = 4;

// Render a slider + input + preset dropdown for wedge angle.
// Local copy for modal use — panels.cpp::SliderWithPreset calls MarkDirty internally
// and cannot be reused in edit-buffer context. Both share the same visual layout.
bool SliderWithPresetEdit(const char* label, float* value, float min_val, float max_val, const char* fmt,
                          SliderScale scale, const ValuePreset* presets, int preset_count) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  constexpr float kArrowBtnWidth = 20.0f;
  float input_w = kInputWidth - kArrowBtnWidth;

  const char* hash_pos = strstr(label, "##");
  if (hash_pos) {
    auto len = static_cast<size_t>(hash_pos - label);
    if (len >= sizeof(display_buf))
      len = sizeof(display_buf) - 1;
    memcpy(display_buf, label, len);
    display_buf[len] = '\0';
  } else {
    snprintf(display_buf, sizeof(display_buf), "%s", label);
  }
  snprintf(slider_id, sizeof(slider_id), "##%s_slider", label);
  snprintf(input_id, sizeof(input_id), "##%s_input", label);

  float spacing = ImGui::GetStyle().ItemSpacing.x;
  float avail_w = ImGui::GetContentRegionAvail().x;
  float slider_w = avail_w - kInputWidth - kArrowBtnWidth - kLabelColWidth - spacing * 3;
  if (slider_w < 40.0f)
    slider_w = 40.0f;

  bool changed = false;

  ImGui::PushItemWidth(slider_w);
  // ImGuiSliderFlags_NoInput: see panels.cpp::RenderNonlinearSlider for rationale.
  if (scale == SliderScale::kSqrt && min_val >= 0.0f) {
    float sqrt_val = std::sqrt(std::max(*value, 0.0f));
    float sqrt_max = std::sqrt(max_val);
    if (ImGui::SliderFloat(slider_id, &sqrt_val, 0.0f, sqrt_max, "", ImGuiSliderFlags_NoInput)) {
      *value = sqrt_val * sqrt_val;
      changed = true;
    }
  } else {
    changed |= ImGui::SliderFloat(slider_id, value, min_val, max_val, fmt, ImGuiSliderFlags_NoInput);
  }
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(input_w);
  changed |= ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

  char popup_id[64];
  snprintf(popup_id, sizeof(popup_id), "##%s_presets", label);
  ImGui::SameLine(0, 0);
  char arrow_id[64];
  snprintf(arrow_id, sizeof(arrow_id), "##%s_arrow", label);
  if (ImGui::ArrowButton(arrow_id, ImGuiDir_Down)) {
    ImGui::OpenPopup(popup_id);
  }
  if (ImGui::BeginPopup(popup_id)) {
    for (int i = 0; i < preset_count; i++) {
      bool selected = std::abs(*value - presets[i].value) < 0.05f;
      if (ImGui::Selectable(presets[i].label, selected)) {
        *value = presets[i].value;
        changed = true;
      }
    }
    ImGui::EndPopup();
  }

  *value = std::clamp(*value, min_val, max_val);

  ImGui::SameLine();
  ImGui::TextUnformatted(display_buf);
  return changed;
}

}  // namespace

namespace {

// Re-snapshot all three buffers as the new dirty-compare baseline. Used by
// OpenEditModal (initial snapshot) and the Immediate→Staged mode switch
// (fresh baseline so dirty-mark starts from zero). Field coverage mirrors
// OpenEditModal lines 210-215 (see plan §F10). Does NOT touch trackball
// save — that retains the Open-time baseline.
//
// Any new edit-buffer field added in the future must be appended here AND
// in ApplyBuffersToEntry to keep the dirty-compare / commit paths symmetric.
void SnapshotAllBuffers(const GuiState& state) {
  // Sync UI raypath char buffer back into the raypath sub-buffer before
  // snapshotting, so the snapshot reflects the current edit state.
  g_raypath_params.raypath_text = g_raypath_buf;
  g_filter_top_snapshot = g_filter_top;
  g_raypath_params_snapshot = g_raypath_params;
  g_ee_params_snapshot = g_ee_params;
  g_dir_params_snapshot = g_dir_params;
  g_crystal_params_snapshot = g_crystal_params;
  g_filter_active_type_snapshot = g_filter_active_type;
  g_crystal_buf_snapshot = g_crystal_buf;
  g_axis_buf_snapshot[0] = g_axis_buf[0];
  g_axis_buf_snapshot[1] = g_axis_buf[1];
  g_axis_buf_snapshot[2] = g_axis_buf[2];
  const int ly = g_modal_layer_idx;
  const int en = g_modal_entry_idx;
  if (ly >= 0 && ly < static_cast<int>(state.layers.size()) && en >= 0 &&
      en < static_cast<int>(state.layers[ly].entries.size())) {
    g_filter_initial_present = state.layers[ly].entries[en].filter.has_value();
  }
}

// Mark the next combo's popup viewport as TopMost so it shares NSWindow level
// with the modal when the modal is detached into its own OS viewport. Without
// this, combo popups default to normal level (0) while the detached modal sits
// at NSFloatingWindowLevel (3, set via the modal's own SetNextWindowClass /
// ImGuiViewportFlags_TopMost in RenderEditModals), causing the popup to render
// behind the modal — invisible and click-throughable. Must be called before
// every modal-internal `BeginCombo` / `Combo` / `RenderAxisDist` call site.
//
// MAINTAINER: any new Combo / BeginCombo inside modal rendering functions
// (RenderCrystalPreviewPane / RenderCrystalModal / RenderAxisModal /
// RenderFilterModal) MUST be preceded by a call to this helper. Forgetting
// the call has no compile-time error and silently regresses to the original
// bug — only visible in detached-modal state which CI cannot reproduce
// (hidden GLFW window pins GetMainViewport()->Pos to (0,0)).
//
// Mechanism: BeginCombo internally backs up and restores g.NextWindowData (see
// imgui_widgets.cpp:1837/1906), so the flags set here propagate through to the
// combo popup's `Begin` call inside `BeginComboPopup`. Validated against
// macOS GLFW backend (CGWindowListCopyWindowInfo reports popup layer=3 after
// applying this; without it layer=0). See ocornut/imgui#6216.
//
// UPGRADE NOTE: re-verify the NextWindowData backup/restore path in
// imgui_widgets.cpp::BeginCombo when upgrading ImGui past v1.91.8-docking.
// `ImGuiWindowClass.ViewportFlagsOverrideSet` is alpha API (per ocornut in
// #7105) and the backup/restore pair (lines 1837/1906 above) may shift across
// versions; if combo popups regress to layer=0 after an upgrade, audit those
// two sites first.
void SetNextComboPopupTopMost() {
  ImGuiWindowClass wc;
  wc.ViewportFlagsOverrideSet = ImGuiViewportFlags_TopMost;
  ImGui::SetNextWindowClass(&wc);
}

}  // namespace

// ============================================================
// OpenEditModal — called from RenderLeftPanel on EditRequest
// ============================================================

void OpenEditModal(const EditRequest& req, GuiState& state) {
  if (req.target == EditTarget::kNone) {
    return;
  }

  // Validate index
  int ly = req.layer_idx;
  int en = req.entry_idx;
  if (ly < 0 || ly >= static_cast<int>(state.layers.size())) {
    return;
  }
  if (en < 0 || en >= static_cast<int>(state.layers[ly].entries.size())) {
    return;
  }

  auto& entry = state.layers[ly].entries[en];
  g_modal_layer_idx = ly;
  g_modal_entry_idx = en;

  // Initialize all three buffers (regardless of req.target) so any tab the user
  // switches to shows the entry's current values. Modal-level OK commits all
  // three atomically; Cancel discards all.
  g_crystal_buf = entry.crystal;
  g_axis_buf[0] = entry.crystal.zenith;
  g_axis_buf[1] = entry.crystal.azimuth;
  g_axis_buf[2] = entry.crystal.roll;
  // Filter buffers: split top-level shared fields and per-type sub-buffer.
  // `entry.filter.value_or(FilterConfig{})` defaults to a Raypath alternative
  // with empty text — preserves the pre-task semantics for nullopt entries.
  const FilterConfig src = entry.filter.value_or(FilterConfig{});
  g_filter_top = FilterConfig{};
  g_filter_top.name = src.name;
  g_filter_top.action = src.action;
  g_filter_top.sym_p = src.sym_p;
  g_filter_top.sym_b = src.sym_b;
  g_filter_top.sym_d = src.sym_d;
  // Reset all per-type buffers to defaults; load the active alternative.
  // Default `g_filter_active_type` to kRaypath as a fallback so a future
  // FilterParamVariant alternative not handled below leaves the modal in a
  // safe state rather than carrying over the previous session's discriminator.
  g_raypath_params = {};
  g_ee_params = {};
  g_dir_params = {};
  g_crystal_params = {};
  g_filter_active_type = FilterEditType::kRaypath;
  if (std::holds_alternative<RaypathParams>(src.param)) {
    g_raypath_params = std::get<RaypathParams>(src.param);
    g_filter_active_type = FilterEditType::kRaypath;
  } else if (std::holds_alternative<EntryExitParams>(src.param)) {
    g_ee_params = std::get<EntryExitParams>(src.param);
    g_filter_active_type = FilterEditType::kEntryExit;
  } else if (std::holds_alternative<DirectionParams>(src.param)) {
    g_dir_params = std::get<DirectionParams>(src.param);
    g_filter_active_type = FilterEditType::kDirection;
  } else if (std::holds_alternative<CrystalParams>(src.param)) {
    g_crystal_params = std::get<CrystalParams>(src.param);
    g_filter_active_type = FilterEditType::kCrystal;
  }
  snprintf(g_raypath_buf, sizeof(g_raypath_buf), "%s", g_raypath_params.raypath_text.c_str());
  // Snapshot the dirty-compare baseline (Crystal/Axis/Filter buffers +
  // filter_initial_present). Trackball save is initialized separately below —
  // it's Open-time state, not snapshot.
  SnapshotAllBuffers(state);

  // Save trackball state for Cancel restoration
  std::memcpy(g_saved_rotation, g_crystal_rotation, sizeof(g_saved_rotation));
  g_saved_zoom = g_crystal_zoom;
  g_modal_mesh_hash = 0;  // Force mesh update on first frame

  switch (req.target) {
    case EditTarget::kCrystal:
      g_active_tab = ActiveTab::kCrystal;
      break;
    case EditTarget::kAxis:
      g_active_tab = ActiveTab::kAxis;
      break;
    case EditTarget::kFilter:
      g_active_tab = ActiveTab::kFilter;
      break;
    default:
      return;
  }

  g_active_modal = ActiveModal::kOpen;
  g_pending_open = true;
  g_pending_tab_select = true;
}


// ============================================================
// Crystal Modal
// ============================================================

static void HandleCrystalPreviewInteraction(bool hovered, bool active) {
  ImGuiIO& io = ImGui::GetIO();
  if (active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    ApplyTrackballRotation(io.MouseDelta.x, io.MouseDelta.y);
  }
  if (hovered) {
    ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
    if (io.MouseWheel != 0.0f) {
      g_crystal_zoom *= (1.0f - io.MouseWheel * 0.1f);
      g_crystal_zoom = std::max(0.5f, std::min(10.0f, g_crystal_zoom));
    }
  }
}

// Constant: 3D preview image size inside the left pane (also the FBO display
// size). WindowPadding is applied by the surrounding child; see the dynamic
// child-size computation in RenderEditModals.
constexpr float kModalPreviewImageSize = 320.0f;

// Render the persistent crystal preview pane (3D image + drag interaction +
// style selector + reset view). Called from the left BeginChild during modal
// rendering so the preview stays visible across Crystal / Axis / Filter tabs.
static void RenderCrystalPreviewPane(GuiState& /*state*/) {
  auto& cr = g_crystal_buf;

  // Update mesh if crystal params changed
  int hash = CrystalParamHash(cr);
  if (hash != g_modal_mesh_hash) {
    int result = BuildAndUploadCrystalMesh(cr);
    if (result != 0) {
      g_modal_mesh_hash = result;
    }
  }

  // Render to FBO (F8: only caller of g_crystal_renderer.Render in production;
  // no double-write with panels.cpp / thumbnail_cache.cpp).
  auto crystal_style = static_cast<CrystalStyle>(g_crystal_style);
  g_crystal_renderer.Render(g_crystal_rotation, g_crystal_zoom, crystal_style);

  // -- 3D Preview (horizontally centered inside the left pane) --
  auto tex_id = static_cast<ImTextureID>(g_crystal_renderer.GetTextureId());
  float avail_w = ImGui::GetContentRegionAvail().x;
  float offset_x = (avail_w - kModalPreviewImageSize) * 0.5f;
  if (offset_x > 0.0f) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_x);
  }
  ImVec2 preview_pos = ImGui::GetCursorScreenPos();
  ImGui::Image(tex_id, ImVec2(kModalPreviewImageSize, kModalPreviewImageSize), ImVec2(0, 1), ImVec2(1, 0));

  // Face-number overlay: use the same rotation/zoom/size used by the GL render
  // pass above so screen coords align pixel-for-pixel with the FBO texture.
  if (const auto* m = GetLastCrystalMesh(); m != nullptr) {
    float mvp[16];
    CrystalRenderer::ComputeMvp(g_crystal_rotation, g_crystal_zoom, static_cast<int>(kModalPreviewImageSize),
                                static_cast<int>(kModalPreviewImageSize), mvp);
    DrawFaceNumberOverlay(m->vertices, m->vertex_count, m->triangles, m->triangle_count, m->face_numbers,
                          g_crystal_rotation, mvp, g_crystal_zoom, preview_pos,
                          ImVec2(kModalPreviewImageSize, kModalPreviewImageSize), ImGui::GetWindowDrawList(),
                          crystal_style);
  }

  // Overlay InvisibleButton to consume mouse clicks and prevent modal window drag.
  ImGui::SetCursorScreenPos(preview_pos);
  ImGui::InvisibleButton("##modal_preview_interact", ImVec2(kModalPreviewImageSize, kModalPreviewImageSize));
  HandleCrystalPreviewInteraction(ImGui::IsItemHovered(), ImGui::IsItemActive());

  // Style combo + Reset View (single row — Combo / SameLine / SmallButton).
  // SetNextComboPopupTopMost: see RenderAxisModal for rationale (combo popups
  // need same NSWindow level as the detached modal viewport).
  ImGui::PushItemWidth(120.0f);
  SetNextComboPopupTopMost();
  ImGui::Combo("##ModalCrystalStyle", &g_crystal_style, kCrystalStyleNames, kCrystalStyleCount);
  ImGui::PopItemWidth();
  ImGui::SameLine();
  if (ImGui::SmallButton("Reset View##modal")) {
    // Reset to the default view derived from the current axis preset + edit-buffer
    // distribution params. g_axis_buf reflects the user's live edits; for kCustom
    // this drives a chain-formula default that matches the outer card thumbnail.
    ResetCrystalView(ClassifyAxisPreset(g_axis_buf[0], g_axis_buf[1], g_axis_buf[2]), g_axis_buf);
  }
}

// Crystal tab body: type radio + parameter sliders + face distance tree.
// Preview / style / reset live in the persistent left pane (see
// RenderCrystalPreviewPane) so they remain visible on Axis / Filter tabs too.
static void RenderCrystalModal(GuiState& /*state*/) {
  auto& cr = g_crystal_buf;

  // -- Crystal type --
  int type_int = static_cast<int>(cr.type);
  if (ImGui::RadioButton("Prism##modal", &type_int, 0)) {
    cr.type = CrystalType::kPrism;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Pyramid##modal", &type_int, 1)) {
    cr.type = CrystalType::kPyramid;
  }

  ImGui::Spacing();

  // -- Parameters --
  // See gui/slider_mapping.hpp for the three-H-mapping conventions.
  if (cr.type == CrystalType::kPrism) {
    SliderWithInput("Height##modal_cr", &cr.height, 0.01f, 100.0f, "%.2f", SliderScale::kLog);
  } else {
    SliderWithInput("Prism H##modal_cr", &cr.prism_h, 0.0f, 100.0f, "%.4f", SliderScale::kLogLinear);
    SliderWithInput("Upper H##modal_cr", &cr.upper_h, 0.0f, 1.0f, "%.3f", SliderScale::kLinear);
    SliderWithInput("Lower H##modal_cr", &cr.lower_h, 0.0f, 1.0f, "%.3f", SliderScale::kLinear);
    SliderWithPresetEdit("Upper A##modal_cr", &cr.upper_alpha, 0.1f, 90.0f, "%.1f", SliderScale::kLinear, kWedgePresets,
                         kWedgePresetCount);
    SliderWithPresetEdit("Lower A##modal_cr", &cr.lower_alpha, 0.1f, 90.0f, "%.1f", SliderScale::kLinear, kWedgePresets,
                         kWedgePresetCount);
  }

  // -- Face distance --
  if (ImGui::TreeNode("Face Distance##modal")) {
    for (int i = 0; i < 6; i++) {
      char label[32];
      snprintf(label, sizeof(label), "Face %d##modal_fd", i + 3);
      SliderWithInput(label, &cr.face_distance[i], 0.0f, 2.0f, "%.3f");
    }
    ImGui::TreePop();
  }

  // -- Reset All (Crystal tab) --
  // Resets shape parameters to defaults. Preserves type/name/axis (axis lives
  // in a sibling tab; the OK/Cancel atomicity contract still applies — Reset
  // All only mutates g_crystal_buf, OK commits, Cancel discards).
  ImGui::Spacing();
  if (ImGui::Button("Reset All##modal_cr", ImVec2(120, 0))) {
    ResetCrystalShapeParams(cr);
  }

  // OK / Cancel handled at modal level (RenderEditModals).
}


// ============================================================
// Axis Modal
// ============================================================

// Preset table: label + id + default (zenith, azimuth, roll). Table-driven so
// that the active preset (via ClassifyAxisPreset) can be highlighted by looping
// over the same entries used to write defaults.
struct AxisPresetEntry {
  const char* label;
  AxisPreset id;
  AxisDist zenith;
  AxisDist azimuth;
  AxisDist roll;
};

static constexpr AxisDist kAzFullUniform{ AxisDistType::kUniform, 0.0f, 360.0f };
static constexpr AxisDist kRollFreeUniform{ AxisDistType::kUniform, 0.0f, 360.0f };
static constexpr AxisDist kRollLockedGauss{ AxisDistType::kGauss, 0.0f, 1.0f };

static constexpr AxisPresetEntry kAxisPresets[] = {
  { "Column", AxisPreset::kColumn, { AxisDistType::kGauss, 90.0f, 1.0f }, kAzFullUniform, kRollFreeUniform },
  { "Plate", AxisPreset::kPlate, { AxisDistType::kGauss, 0.0f, 1.0f }, kAzFullUniform, kRollFreeUniform },
  { "Parry", AxisPreset::kParry, { AxisDistType::kGauss, 90.0f, 1.0f }, kAzFullUniform, kRollLockedGauss },
  // Lowitz default zenith uses Gauss (v11 内测反馈：gauss 更符合物理直觉; classifier 仍接受 zigzag).
  { "Lowitz", AxisPreset::kLowitz, { AxisDistType::kGauss, 0.0f, 40.0f }, kAzFullUniform, kRollLockedGauss },
  { "Random", AxisPreset::kRandom, kAzFullUniform, kAzFullUniform, kRollFreeUniform },
  { "Custom",
    AxisPreset::kCustom,
    { AxisDistType::kGauss, 90.0f, 20.0f },
    kAzFullUniform,
    { AxisDistType::kGauss, 0.0f, 20.0f } },
};

static void RenderAxisModal(GuiState& /*state*/) {
  // Preset buttons — active preset (inferred from current g_axis_buf) is highlighted
  // using the theme's ButtonActive color so it adapts to light/dark style changes.
  AxisPreset active = ClassifyAxisPreset(g_axis_buf[0], g_axis_buf[1], g_axis_buf[2]);
  const ImVec4 active_color = ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive);

  ImGui::Text("Presets:");
  for (const auto& entry : kAxisPresets) {
    ImGui::SameLine();
    bool highlighted = entry.id == active;
    if (highlighted) {
      ImGui::PushStyleColor(ImGuiCol_Button, active_color);
    }
    if (ImGui::SmallButton(entry.label)) {
      g_axis_buf[0] = entry.zenith;
      g_axis_buf[1] = entry.azimuth;
      g_axis_buf[2] = entry.roll;
      // Drive modal preview to the preset's default view (fixes the bug where
      // switching preset only updated the outer card thumbnail). Same source
      // as Reset View / thumbnail — see DefaultPreviewRotation.
      ResetCrystalView(entry.id, g_axis_buf);
    }
    if (highlighted) {
      ImGui::PopStyleColor();
    }
  }

  ImGui::Separator();

  // Axis distribution controls (zenith: 0-180, azimuth: 0-360, roll: 0-360).
  // Return values intentionally ignored: modal operates on edit buffer, dirty state
  // is only committed on OK button press (not on each slider change).
  //
  // SetNextComboPopupTopMost() before each combo: see helper definition for
  // mechanism + maintainer rules. Cross-file contract assumption:
  // RenderAxisDist (panels.cpp) must not call Begin/BeginChild before its
  // internal Combo, otherwise the WindowClass we just queued would be consumed
  // prematurely by that intermediate Begin instead of the combo popup. Verified
  // for the current implementation; if RenderAxisDist is ever refactored to
  // wrap its body in BeginChild for layout purposes, this propagation will
  // silently break (popup regresses to layer=0).
  SetNextComboPopupTopMost();
  RenderAxisDist("Zenith", g_axis_buf[0], 0.0f, 180.0f);
  SetNextComboPopupTopMost();
  RenderAxisDist("Azimuth", g_axis_buf[1], 0.0f, 360.0f);
  SetNextComboPopupTopMost();
  RenderAxisDist("Roll", g_axis_buf[2], 0.0f, 360.0f);

  // OK / Cancel handled at modal level (RenderEditModals).
}


// ============================================================
// Filter Modal
// ============================================================

// Forward declaration — CurrentValidationKind() is defined in the anonymous
// namespace further down (next to ApplyBuffersToEntry). Single source of truth
// for "what crystal kind should the raypath validator use"; both this Filter
// sub-panel and the modal-level OK gate consume it.
namespace {
lumice::CrystalKind CurrentValidationKind();
}

static void RenderRaypathSubpanel() {
  // Sync InputText backing buffer into the raypath sub-buffer before validation
  // so the displayed hint reflects the current edit state.
  g_raypath_params.raypath_text = g_raypath_buf;
  const auto result = ValidateRaypathTextMultiSegment(g_raypath_params.raypath_text, CurrentValidationKind());
  const auto validation = result.state;

  ImVec4 border_color;
  switch (validation) {
    case RaypathValidation::kValid:
      border_color = ImVec4(0.2f, 0.8f, 0.2f, 1.0f);
      break;
    case RaypathValidation::kIncomplete:
      border_color = ImVec4(0.9f, 0.8f, 0.1f, 1.0f);
      break;
    case RaypathValidation::kInvalid:
      border_color = ImVec4(0.9f, 0.2f, 0.2f, 1.0f);
      break;
  }

  // Row 1: Raypath text input (top-emphasized).
  ImGui::PushStyleColor(ImGuiCol_FrameBg,
                        ImVec4(border_color.x * 0.3f, border_color.y * 0.3f, border_color.z * 0.3f, 0.5f));
  ImGui::InputText("Raypath##filter_modal", g_raypath_buf, sizeof(g_raypath_buf));
  ImGui::PopStyleColor();

  // Validation hint immediately under the InputText.
  switch (validation) {
    case RaypathValidation::kValid:
      if (g_raypath_params.raypath_text.empty()) {
        ImGui::TextDisabled("No raypath filter (match all)");
      } else {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "Valid");
      }
      break;
    case RaypathValidation::kIncomplete:
      ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.1f, 1.0f), "Incomplete (still typing?)");
      break;
    case RaypathValidation::kInvalid: {
      const char* msg = result.message.empty() ? "Invalid raypath" : result.message.c_str();
      // TextWrapped so multi-segment "Segment N: ..." messages don't overflow
      // the modal's narrow vertical layout.
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
      ImGui::TextWrapped("%s", msg);
      ImGui::PopStyleColor();
      break;
    }
  }

  // Multi-raypath OR hint (issue range item 3): clarify the new ';'-separated syntax.
  ImGui::TextDisabled("e.g. 3-5 or 3-5; 1-3");
}

static void RenderEntryExitStub() {
  ImGui::TextDisabled("Entry-Exit filter — coming soon");
}

static void RenderDirectionStub() {
  ImGui::TextDisabled("Direction filter — coming soon");
}

static void RenderCrystalFilterStub() {
  ImGui::TextDisabled("Crystal filter — coming soon");
}

// Shared filter controls (Action radio + P/B/D), rendered after the
// type-specific dispatch. Always uses g_filter_top so type switches don't
// reset shared user preferences.
static void RenderSharedFilterControls() {
  // Action: two RadioButtons (was Combo pre-task; aligns with Crystal tab style).
  if (ImGui::RadioButton("Filter In##filter_action", g_filter_top.action == 0)) {
    g_filter_top.action = 0;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Filter Out##filter_action", g_filter_top.action == 1)) {
    g_filter_top.action = 1;
  }

  ImGui::Checkbox("P##filter_modal", &g_filter_top.sym_p);
  ImGui::SameLine();
  ImGui::Checkbox("B##filter_modal", &g_filter_top.sym_b);
  ImGui::SameLine();
  ImGui::Checkbox("D##filter_modal", &g_filter_top.sym_d);
}

// Remove Filter button — clears the raypath InputText backing buffer.
// Disabled when the raypath is empty; further wrapped in BeginDisabled(true)
// when active type is a stub (the outer dispatch wrapper handles that).
static void RenderRemoveFilterButton() {
  const bool raypath_empty = (g_raypath_buf[0] == '\0');
  ImGui::BeginDisabled(raypath_empty);
  if (ImGui::Button("Remove Filter##filter", ImVec2(120, 0))) {
    g_raypath_buf[0] = '\0';
    g_raypath_params.raypath_text.clear();
  }
  ImGui::EndDisabled();
}

static void RenderFilterModal(GuiState& /*state*/) {
  // Type radio — 4 options, SameLine horizontal (style-aligned with Crystal
  // tab's Prism/Pyramid radios). Boolean form (RadioButton(label, active))
  // matches the Action radio below and the Crystal-tab Prism/Pyramid radios,
  // avoiding a "ImGui writes a temp int, branch writes the enum" double-write.
  // On vertical layout (~360px) we keep all four on one row; no runtime
  // adaptive wrap to avoid extra test dimensions.
  if (ImGui::RadioButton("Raypath##filter_type", g_filter_active_type == FilterEditType::kRaypath)) {
    g_filter_active_type = FilterEditType::kRaypath;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Entry-Exit##filter_type", g_filter_active_type == FilterEditType::kEntryExit)) {
    g_filter_active_type = FilterEditType::kEntryExit;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Direction##filter_type", g_filter_active_type == FilterEditType::kDirection)) {
    g_filter_active_type = FilterEditType::kDirection;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Crystal##filter_type", g_filter_active_type == FilterEditType::kCrystal)) {
    g_filter_active_type = FilterEditType::kCrystal;
  }

  ImGui::Spacing();

  // Stub types render the entire body inside BeginDisabled(true) so the user
  // immediately sees that nothing below is interactive (B1: 整段灰显). The
  // OK button at the modal level gets a parallel disable path in
  // RenderEditModals so the user cannot commit a stub-type filter.
  const bool is_stub = (g_filter_active_type != FilterEditType::kRaypath);
  ImGui::BeginDisabled(is_stub);

  // Type-specific dispatch. `default:` assert ensures any future
  // FilterEditType extension (#4-#6) is handled explicitly rather than
  // silently dropped — mirrors the project convention of "exhaustive switch +
  // assert tripwire" recorded in scratchpad/learnings.md.
  switch (g_filter_active_type) {
    case FilterEditType::kRaypath:
      RenderRaypathSubpanel();
      break;
    case FilterEditType::kEntryExit:
      RenderEntryExitStub();
      break;
    case FilterEditType::kDirection:
      RenderDirectionStub();
      break;
    case FilterEditType::kCrystal:
      RenderCrystalFilterStub();
      break;
    default:
      assert(false && "unhandled FilterEditType in RenderFilterModal dispatch");
      break;
  }

  ImGui::Spacing();

  // Shared controls (Action radio + P/B/D).
  RenderSharedFilterControls();

  ImGui::Spacing();

  // Remove Filter — only meaningful for raypath; nested BeginDisabled is fine
  // (ImGui treats it as a logical OR of the disable stack).
  RenderRemoveFilterButton();

  ImGui::EndDisabled();

  // OK / Cancel handled at modal level (RenderEditModals).
}


// ============================================================
// Public API
// ============================================================

// True when the unified edit modal is open (any tab). Used by visual-smoke
// tests to skip FBO drive while the modal owns the FBO via its per-frame
// g_crystal_renderer.Render() call. Since the preview pane is now always
// visible (shared across Crystal / Axis / Filter tabs), the gate condition
// is modal-open rather than crystal-tab-active.
bool IsEditModalOpen() {
  return g_active_modal == ActiveModal::kOpen;
}

EditModalTarget GetEditModalTarget() {
  if (!IsEditModalOpen()) {
    return { -1, -1 };
  }
  return { g_modal_layer_idx, g_modal_entry_idx };
}

void ResetModalState() {
  g_active_modal = ActiveModal::kNone;
  g_active_tab = ActiveTab::kCrystal;
  g_pending_tab_select = false;
  // modal_immediate_mode is GuiState-owned; the test harness's DoNew-based
  // reset overwrites g_state, but we also clear it here for belt-and-braces
  // (test lambdas that fail mid-flight may skip their own cleanup tails).
  g_state.modal_immediate_mode = false;
  g_modal_layer_idx = -1;
  g_modal_entry_idx = -1;
  g_crystal_buf = {};
  g_axis_buf[0] = {};
  g_axis_buf[1] = {};
  g_axis_buf[2] = {};
  g_filter_top = {};
  g_filter_top_snapshot = {};
  g_filter_active_type = FilterEditType::kRaypath;
  g_filter_active_type_snapshot = FilterEditType::kRaypath;
  g_raypath_params = {};
  g_ee_params = {};
  g_dir_params = {};
  g_crystal_params = {};
  g_raypath_params_snapshot = {};
  g_ee_params_snapshot = {};
  g_dir_params_snapshot = {};
  g_crystal_params_snapshot = {};
  g_filter_initial_present = false;
  g_raypath_buf[0] = '\0';
  std::memset(g_saved_rotation, 0, sizeof(g_saved_rotation));
  g_saved_zoom = 1.0f;
  g_pending_open = false;
  g_pending_mode_switch = false;
  g_modal_mesh_hash = 0;
}

namespace {

// Validation kind based on the in-flight Crystal-tab buffer (not the entry),
// because the user may switch crystal type before committing — the validator
// should match what will actually be written on OK. Single source of truth
// for the Filter sub-panel and the modal-level OK gate.
lumice::CrystalKind CurrentValidationKind() {
  return (g_crystal_buf.type == CrystalType::kPrism) ? lumice::CrystalKind::kPrism : lumice::CrystalKind::kPyramid;
}

// Filter-tab dirty predicate. Active-type-guarded form per plan §7 risk 6:
// when the active type is X, only X's sub-buffer is compared against its
// snapshot. This makes the dirty mark robust against future per-type input
// controls (#4-#6 sub-tasks) — typing in EE while raypath is active must not
// flip the dirty bit on either side until the user actually switches type.
//
// Used by both the tab-label dirty mark in RenderEditModals AND the
// `buf_changed` gate inside ApplyBuffersToEntry (commit decision). Keeping the
// definition single-source means future sub-types only need an additional
// case here; both consumers pick up the change automatically.
bool IsFilterDirty() {
  if (g_filter_active_type != g_filter_active_type_snapshot) {
    return true;
  }
  if (g_filter_top != g_filter_top_snapshot) {
    return true;
  }
  switch (g_filter_active_type) {
    case FilterEditType::kRaypath:
      return g_raypath_params != g_raypath_params_snapshot;
    case FilterEditType::kEntryExit:
      return g_ee_params != g_ee_params_snapshot;
    case FilterEditType::kDirection:
      return g_dir_params != g_dir_params_snapshot;
    case FilterEditType::kCrystal:
      return g_crystal_params != g_crystal_params_snapshot;
    default:
      assert(false && "unhandled FilterEditType in IsFilterDirty");
      return false;
  }
}

// Result of applying edit buffers back to the entry. Used by both commit paths
// to decide which dirty-notification side effects to fire.
struct ApplyBuffersResult {
  bool valid;           // false if g_modal_layer_idx / g_modal_entry_idx out of range
  bool entry_changed;   // entry != old_entry after apply
  bool filter_changed;  // entry.filter != old_entry.filter after apply
};

// Single source of truth for buffer→entry field assignment. Both
// CommitAllBuffers (Staged OK) and CommitAllBuffersImmediate (Immediate
// per-frame) delegate here; the two commits only differ in which
// MarkDirty / MarkFilterDirty calls they fire based on the returned flags.
//
// Adding a new edit-buffer field? Update this function AND SnapshotAllBuffers
// in the same change (the pair drives both commit path and dirty-compare baseline).
ApplyBuffersResult ApplyBuffersToEntry(GuiState& state) {
  const int ly = g_modal_layer_idx;
  const int en = g_modal_entry_idx;
  if (ly < 0 || ly >= static_cast<int>(state.layers.size()) || en < 0 ||
      en >= static_cast<int>(state.layers[ly].entries.size())) {
    return { false, false, false };
  }
  auto& entry = state.layers[ly].entries[en];
  // Diff-gate: snapshot before applying buffers so callers can skip render-invalidation
  // when nothing actually changed (e.g. Staged OK without any edits must not
  // clear the rendered image or arm Revert).
  const EntryCard old_entry = entry;
  // g_crystal_buf carries the Crystal-tab edits; overlay axis edits from the
  // Axis tab so the unified commit reflects both.
  entry.crystal = g_crystal_buf;
  entry.crystal.zenith = g_axis_buf[0];
  entry.crystal.azimuth = g_axis_buf[1];
  entry.crystal.roll = g_axis_buf[2];
  // Sync the ImGui InputText backing buffer back into the raypath sub-buffer
  // before any commit decision; g_raypath_buf is the canonical source for the
  // raypath text. Mirrors the dirty-compare pattern in RenderEditModals.
  g_raypath_params.raypath_text = g_raypath_buf;

  // Stub-type defensive path: OK button is disabled on stub types, so this
  // branch is not user-reachable. Leave entry.filter unchanged so a
  // hypothetical entry into this branch (e.g. via Immediate-mode commit while
  // a stub is selected) cannot silently overwrite the existing filter with
  // half-built default data.
  if (g_filter_active_type != FilterEditType::kRaypath) {
    return { true, entry != old_entry, entry.filter != old_entry.filter };
  }

  // Buffer-changed predicate: defined as IsFilterDirty() in this TU so the
  // commit decision and the tab-label dirty mark stay in sync — adding a new
  // FilterEditType only needs one place to be touched.
  const bool buf_changed = IsFilterDirty();

  if (g_raypath_params.raypath_text.empty()) {
    // Empty raypath ≡ "no filter" at the UI layer (the Remove button is just
    // a shortcut for "backspace the textbox empty"). This also subsumes the
    // old "Remove intent" path and the default-PBD leak: a default-constructed
    // FilterConfig has empty raypath, so untouched no-filter entries stay
    // nullopt here.
    entry.filter = std::nullopt;
  } else if (g_filter_initial_present || buf_changed) {
    // Model-layer invariant: FilterConfig must never hold an invalid raypath,
    // regardless of commit mode. Staged mode already enforces this via the OK
    // button's disabled gate (ok_disabled check on the Staged OK path);
    // Immediate mode per-frame commits reach this branch with potentially
    // invalid text, so the guard here is the single model-layer gate that
    // makes the invariant hold in both modes. kIncomplete is treated same as
    // kInvalid — matches the Staged OK disjunction `v.state != kValid`,
    // preventing half-typed input from poisoning the renderer. Multi-segment
    // OR (";") is supported via ValidateRaypathTextMultiSegment.
    auto v = ValidateRaypathTextMultiSegment(g_raypath_params.raypath_text, CurrentValidationKind());
    if (v.state == RaypathValidation::kValid) {
      // Materialize FilterConfig from top fields + active raypath sub-buffer.
      FilterConfig out;
      out.name = g_filter_top.name;
      out.action = g_filter_top.action;
      out.sym_p = g_filter_top.sym_p;
      out.sym_b = g_filter_top.sym_b;
      out.sym_d = g_filter_top.sym_d;
      out.param = g_raypath_params;
      entry.filter = out;
    }
  }
  return { true, entry != old_entry, entry.filter != old_entry.filter };
}

// Staged OK path: any entry change clears the display + restarts simulation
// (existing semantics — OK implies user commits and sim will re-run).
void CommitAllBuffers(GuiState& state) {
  const auto r = ApplyBuffersToEntry(state);
  if (!r.valid || !r.entry_changed) {
    return;
  }
  g_thumbnail_cache.Invalidate(g_modal_layer_idx, g_modal_entry_idx);
  state.MarkDirty();
  state.MarkFilterDirty();
  g_crystal_mesh_hash = -1;
}

// Immediate path: crystal/axis edits only MarkDirty; MarkFilterDirty (which
// clears snapshot_intensity and locks upload via intensity_locked=true) is
// gated on filter actually changing. This is what allows infinite-rays
// accumulation to persist while the user drags a crystal slider.
void CommitAllBuffersImmediate(GuiState& state) {
  const auto r = ApplyBuffersToEntry(state);
  if (!r.valid || !r.entry_changed) {
    return;
  }
  g_thumbnail_cache.Invalidate(g_modal_layer_idx, g_modal_entry_idx);
  state.MarkDirty();
  if (r.filter_changed) {
    state.MarkFilterDirty();
  }
  g_crystal_mesh_hash = -1;
}

// Centralized cleanup for every path that causes BeginPopup* to return false
// on the frame after a close was requested. Three branches:
//   1. mode-switch close+reopen: consume g_pending_mode_switch, arm
//      g_pending_open, keep g_active_modal = kOpen, do NOT restore trackball
//      (we want to resurface the modal on the next frame with all state
//      intact).
//   2. Immediate normal close (bottom Close / Esc / click-outside): changes
//      are already committed every frame, so just clear g_active_modal.
//      Trackball edits are user-intentional in Immediate mode — don't revert.
//   3. Staged close (Cancel button / title-bar × / Esc): revert trackball to
//      the Open-time snapshot and clear g_active_modal. This branch also
//      covers the deleted-entry race (the in-body guard calls
//      CloseCurrentPopup when indices go out of range); the trackball restore
//      is idempotent and harmless when the entry is already gone.
void HandlePopupClosed(GuiState& state) {
  if (g_pending_mode_switch) {
    g_pending_mode_switch = false;
    g_pending_open = true;
    // Replay the OpenEditModal tab-select mechanic so the newly reopened
    // popup forces BeginTabItem(g_active_tab) onto SetSelected on its first
    // rendered frame. Without this, ImGui's TabBar silently defaults to the
    // first tab after the ID stack is regenerated post-close+reopen.
    g_pending_tab_select = true;
    return;
  }
  if (state.modal_immediate_mode) {
    g_active_modal = ActiveModal::kNone;
    return;
  }
  std::memcpy(g_crystal_rotation, g_saved_rotation, sizeof(g_crystal_rotation));
  g_crystal_zoom = g_saved_zoom;
  g_crystal_mesh_hash = -1;
  g_active_modal = ActiveModal::kNone;
}

// Renders the layout-toggle button, the TabBar, and the three tab bodies.
// Called from both the horizontal (right child) and vertical (bottom child)
// layout branches with identical arguments; layout-dependent geometry is
// handled by the caller's BeginChild sizing.
void RenderModalTabBar(GuiState& state, const char* crystal_label, const char* axis_label, const char* filter_label,
                       ImGuiTabItemFlags crystal_flags, ImGuiTabItemFlags axis_flags, ImGuiTabItemFlags filter_flags) {
  // Note: H/V layout toggle relocated to the bottom button row alongside the
  // Immediate checkbox for visual consistency (both are view-preference
  // toggles; gui-polish-v15 round 2 UX feedback). See RenderEditModals below.
  if (ImGui::BeginTabBar("##edit_modal_tabs")) {
    if (ImGui::BeginTabItem(crystal_label, nullptr, crystal_flags)) {
      g_active_tab = ActiveTab::kCrystal;
      RenderCrystalModal(state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(axis_label, nullptr, axis_flags)) {
      g_active_tab = ActiveTab::kAxis;
      RenderAxisModal(state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(filter_label, nullptr, filter_flags)) {
      g_active_tab = ActiveTab::kFilter;
      RenderFilterModal(state);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    // Consumed pending selection after BeginTabItem reads the flag.
    g_pending_tab_select = false;
  }
}

}  // namespace

void RenderEditModals(GuiState& state, GLFWwindow* window) {
  // Deferred OpenPopup: only Staged mode uses the popup stack. Immediate mode
  // drives visibility directly via g_active_modal → title_x_open (see below),
  // so it neither opens nor consults the popup stack.
  if (g_pending_open && g_active_modal == ActiveModal::kOpen) {
    if (!state.modal_immediate_mode) {
      ImGui::OpenPopup("Edit Entry");
    }
    g_pending_open = false;
  }

  // Index validity guard: if the target entry was deleted while modal is open, close it.
  if (g_active_modal == ActiveModal::kOpen) {
    bool valid = g_modal_layer_idx >= 0 && g_modal_layer_idx < static_cast<int>(state.layers.size()) &&
                 g_modal_entry_idx >= 0 &&
                 g_modal_entry_idx < static_cast<int>(state.layers[g_modal_layer_idx].entries.size());
    if (!valid) {
      g_active_modal = ActiveModal::kNone;
    }
  }

  // Size constraints: clamp modal max to the workarea of the monitor containing
  // the window center, so it cannot overflow a small secondary display. When
  // the helper cannot identify a monitor (headless tests, nullptr window), fall
  // back to an unbounded max rather than a primary-monitor default (avoids the
  // multi-monitor "primary bias" anti-pattern).
  const float min_w = state.modal_layout_vertical ? kEditModalMinWidthVertical : kEditModalMinWidth;
  const float min_h = state.modal_layout_vertical ? kEditModalMinHeightVertical : 0.0f;
  // Snap window width when the user toggles H↔V. SetNextWindowSizeConstraints
  // alone only bounds the allowed range; an already-sized window stays at its
  // current width if that value is within the new range. Explicit
  // SetNextWindowSize on the toggle frame forces the width to the new layout's
  // minimum; height 0 means "auto-fit to content" (AlwaysAutoResize semantics).
  static bool s_prev_modal_layout_vertical = state.modal_layout_vertical;
  if (s_prev_modal_layout_vertical != state.modal_layout_vertical) {
    s_prev_modal_layout_vertical = state.modal_layout_vertical;
    ImGui::SetNextWindowSize(ImVec2(min_w, 0.0f));
  }
  MonitorRect mon{};
  if (GetCurrentMonitorWorkArea(window, &mon)) {
    auto max_w = std::max(min_w, static_cast<float>(mon.w - kWindowDecorationMargin));
    auto max_h = std::max(static_cast<float>(kMinWindowHeight), static_cast<float>(mon.h - kWindowDecorationMargin));
    ImGui::SetNextWindowSizeConstraints(ImVec2(min_w, min_h), ImVec2(max_w, max_h));
  } else {
    ImGui::SetNextWindowSizeConstraints(ImVec2(min_w, min_h), ImVec2(FLT_MAX, FLT_MAX));
  }
  // Mode dispatch: Staged → BeginPopupModal (blocks background, exposes title-bar ×
  // via p_open). Immediate → ImGui::Begin (regular window — external clicks pass
  // through, hover/focus work on background UI, window stays visible until an
  // explicit close via the bottom Close button or the title-bar ×).
  //
  // title_x_open initialization is load-bearing for Immediate: Begin uses it as
  // p_open, so code-driven close paths (Close button, title-bar ×, mode switch)
  // set g_active_modal = kNone → next frame title_x_open = false → Begin returns
  // false → cleanup via HandlePopupClosed's !window_open branch. If this were
  // kept as `true` unconditionally, code-driven close in Immediate would fail
  // silently (window re-renders every frame with no assertion). Staged branch
  // is neutral: BeginPopupModal's visibility is governed by the popup stack,
  // p_open only controls whether the title-bar × renders.
  // Dual semantics:
  //   Immediate: doubles as Begin's p_open — drives window visibility. Initial
  //              value (g_active_modal==kOpen) → true while modal is open,
  //              → false the frame after any code-driven close (Close button /
  //              title ×), cleanly triggering !window_open cleanup.
  //   Staged:    governs only whether the title-bar × glyph renders;
  //              BeginPopupModal's visibility is determined by the popup stack.
  bool title_x_open = (g_active_modal == ActiveModal::kOpen);
  bool window_open = false;
  // Pin the dispatch mode for this frame. The Immediate-mode checkbox
  // (rendered later in body) may flip state.modal_immediate_mode mid-frame,
  // but End/EndPopup must pair with the container we actually opened here.
  const bool dispatched_immediate = state.modal_immediate_mode;
  // H0 outer guard: the Immediate path uses ImGui::Begin, which — unlike
  // BeginPopupModal — keeps a registered ImGuiWindow in g.Windows and
  // continues rendering a tomb-stone title bar even when *p_open==false,
  // until the window is fully garbage-collected across frames. The standard
  // ImGui pattern for closing a window is to skip the entire Begin/End block
  // at the call site. Guard only the Immediate branch; Staged
  // BeginPopupModal on an empty popup stack already returns false without
  // leaving a stray title bar. The !g_pending_mode_switch clause preserves
  // the Staged→Immediate mode-switch consume path further down (the body
  // branch that arms g_pending_tab_select needs Begin to run). Once this
  // guard is in effect, the inner race-case exit
  // (dispatched_immediate && g_active_modal != kOpen → End+return) is
  // unreachable; it is retained as an idempotent safety net for deleted-
  // entry paths and will become reachable again only if this guard is ever
  // relaxed.
  if (dispatched_immediate && !title_x_open && !g_pending_mode_switch) {
    return;
  }
  if (dispatched_immediate) {
    // Keep the window always-on-top when dragged out to an independent OS
    // viewport. Without this, losing focus to the main GLFW window would
    // let the host window cover the detached modal. ViewportFlagsOverrideSet
    // applies only when the window actually becomes its own viewport; it is
    // a no-op while docked to the main viewport.
    ImGuiWindowClass window_class;
    window_class.ViewportFlagsOverrideSet = ImGuiViewportFlags_TopMost;
    ImGui::SetNextWindowClass(&window_class);
    // Center the modal on the main viewport ONLY on the very first creation
    // of this window within the process. After that, ImGui's in-memory
    // window settings remember the user's dragged position across close/
    // reopen cycles. ImGuiCond_Appearing would re-center every time the
    // window re-appears, which is not what we want.
    const ImGuiViewport* main_vp = ImGui::GetMainViewport();
    ImVec2 center(main_vp->Pos.x + main_vp->Size.x * 0.5f, main_vp->Pos.y + main_vp->Size.y * 0.5f);
    ImGui::SetNextWindowPos(center, ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    // NoDocking: with ImGuiConfigFlags_ViewportsEnable, this flag prevents docking into
    // the main window while still allowing the window to live in its own OS viewport when
    // dragged outside. Without NoDocking, users could accidentally dock the editor into a
    // main-window split which is not the intended layout.
    window_open =
        ImGui::Begin("Edit Entry", &title_x_open,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse);
  } else {
    window_open = ImGui::BeginPopupModal("Edit Entry", &title_x_open, ImGuiWindowFlags_AlwaysAutoResize);
  }
  if (!window_open) {
    // ImGui contract: Begin always pairs with End regardless of return value.
    // BeginPopupModal when it returns false must NOT be paired with EndPopup.
    if (dispatched_immediate) {
      ImGui::End();
    }
    // Either the container was never opened, or it was closed on the previous
    // frame. If the latter (or a mode switch is in flight), run centralized
    // cleanup so mode switch / trackball restore / g_active_modal reset
    // converge through HandlePopupClosed.
    if (g_active_modal == ActiveModal::kOpen || g_pending_mode_switch) {
      HandlePopupClosed(state);
    }
    return;
  }
  // Title-bar × close:
  //   Staged: CloseCurrentPopup → next-frame !window_open → HandlePopupClosed's
  //           Staged-cancel branch (trackball restore + g_active_modal reset).
  //           Cancel button / × / Esc share this single path.
  //   Immediate: no popup-stack entry to close — directly reset g_active_modal.
  //           Next frame Begin returns false → !window_open cleanup branch.
  if (!title_x_open) {
    if (dispatched_immediate) {
      g_active_modal = ActiveModal::kNone;
    } else {
      ImGui::CloseCurrentPopup();
    }
  }
  // Race case: the entry was deleted (index guard above set kNone) or user
  // just clicked the title-bar × in Immediate mode. Close the container body
  // for this frame; next frame's !window_open path runs HandlePopupClosed.
  if (g_active_modal != ActiveModal::kOpen) {
    if (dispatched_immediate) {
      ImGui::End();
    } else {
      ImGui::CloseCurrentPopup();
      ImGui::EndPopup();
    }
    return;
  }

  // Staged → Immediate mode-switch consume: because Begin (unlike
  // BeginPopupModal on an empty stack) returns true on Frame N+1 with
  // title_x_open=true, !window_open never fires → HandlePopupClosed cannot
  // observe the switch. Consume g_pending_mode_switch inline here (body path).
  // This is the structural inverse of the Immediate → Staged direction, where
  // BeginPopupModal on an empty popup stack returns false and cleanup flows
  // through HandlePopupClosed naturally. Known asymmetry: Immediate inbound
  // (body consume) vs Staged inbound (HandlePopupClosed consume) — a direct
  // consequence of ImGui's popup stack vs regular window dispatch split.
  // Counterpart for Immediate → Staged direction: HandlePopupClosed's
  // mode-switch branch (arms g_pending_open + g_pending_tab_select, keeps
  // g_active_modal=kOpen), invoked from the !window_open path on Frame N+1
  // when BeginPopupModal sees an empty popup stack.
  // Invariant: never touch g_active_modal here (it must stay kOpen until a
  // user-driven close; see F10 in plan).
  if (dispatched_immediate && g_pending_mode_switch) {
    g_pending_mode_switch = false;
    g_pending_tab_select = true;
  }

  // Snapshot the SetSelected flags BEFORE any BeginTabItem body runs —
  // each body writes g_active_tab, which would otherwise overwrite the
  // pending selection before later tabs (e.g. Filter) read it.
  const ImGuiTabItemFlags crystal_flags = (g_pending_tab_select && g_active_tab == ActiveTab::kCrystal) ?
                                              ImGuiTabItemFlags_SetSelected :
                                              ImGuiTabItemFlags_None;
  const ImGuiTabItemFlags axis_flags = (g_pending_tab_select && g_active_tab == ActiveTab::kAxis) ?
                                           ImGuiTabItemFlags_SetSelected :
                                           ImGuiTabItemFlags_None;
  const ImGuiTabItemFlags filter_flags = (g_pending_tab_select && g_active_tab == ActiveTab::kFilter) ?
                                             ImGuiTabItemFlags_SetSelected :
                                             ImGuiTabItemFlags_None;

  // Per-tab dirty detection. The label picks up a trailing " *" when the
  // in-flight buffer differs from the snapshot taken at modal-open. All
  // labels share a fixed `###` suffix — with three hashes ImGui derives the
  // internal ID purely from the `###suffix` portion, so the display string
  // can vary ("Crystal" vs "Crystal *") without changing the tab's hash
  // (otherwise the tab would lose its SelectedTabId the moment dirty flips,
  // falling back to the first tab and hiding the user's work-in-progress).
  // filter_dirty: delegate to IsFilterDirty() so the tab-label "*" and the
  // commit-time `buf_changed` predicate share a single source of truth. Sync
  // the InputText backing first so post-keystroke / post-Remove diffs are
  // reflected on the same frame.
  g_raypath_params.raypath_text = g_raypath_buf;
  const bool crystal_dirty = g_crystal_buf != g_crystal_buf_snapshot;
  const bool axis_dirty = g_axis_buf[0] != g_axis_buf_snapshot[0] || g_axis_buf[1] != g_axis_buf_snapshot[1] ||
                          g_axis_buf[2] != g_axis_buf_snapshot[2];
  const bool filter_dirty = IsFilterDirty();
  // Immediate mode: changes apply every frame, so "dirty" is not a meaningful
  // state — suppress the * mark on all tabs regardless of buffer vs snapshot.
  const bool show_dirty = !state.modal_immediate_mode;
  const char* crystal_label = (show_dirty && crystal_dirty) ? "Crystal *###crystal_tab" : "Crystal###crystal_tab";
  const char* axis_label = (show_dirty && axis_dirty) ? "Axis *###axis_tab" : "Axis###axis_tab";
  const char* filter_label = (show_dirty && filter_dirty) ? "Filter *###filter_tab" : "Filter###filter_tab";

  // Layout dispatch. Horizontal: preview on left, tab bar on right (default).
  // Vertical: preview on top (full width), tab bar below (full width, scrollable).
  // Both share the same preview child height and the same RenderModalTabBar body;
  // only the container geometry and the SameLine/no-SameLine differ.
  const ImGuiStyle& style = ImGui::GetStyle();
  const float kPreviewChildW_H = kModalPreviewImageSize + style.WindowPadding.x * 2.0f + 4.0f;
  const float kToolRow = ImGui::GetFrameHeightWithSpacing();
  const float kVPad = style.WindowPadding.y * 2.0f + style.ItemSpacing.y;
  const float kPreviewChildHeight = kModalPreviewImageSize + kToolRow + kVPad;
  bool vertical = state.modal_layout_vertical;

  if (vertical) {
    // Upper pane: preview, full modal width, fixed height.
    ImGui::BeginChild("##modal_top_pane", ImVec2(-FLT_MIN, kPreviewChildHeight), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    RenderCrystalPreviewPane(state);
    ImGui::EndChild();
    // Lower pane: tab bar + body. Height matches the horizontal right pane
    // so V-layout modal is visually a stacked version of H-layout (user
    // feedback gui-polish-v15 round 2). Use default scrollbar behavior
    // (shown only when content overflows); AlwaysVerticalScrollbar would
    // leave the track visible even when content fits — unnecessary noise
    // for tabs whose natural height already matches the pane.
    ImGui::BeginChild("##modal_bottom_pane", ImVec2(-FLT_MIN, kPreviewChildHeight), ImGuiChildFlags_None,
                      ImGuiWindowFlags_None);
    RenderModalTabBar(state, crystal_label, axis_label, filter_label, crystal_flags, axis_flags, filter_flags);
    ImGui::EndChild();
  } else {
    // Horizontal: existing layout. Left pane NoScrollbar + NoScrollWithMouse — height
    // budget covers content; right pane sizes off remaining width.
    const float right_w = std::max(320.0f, ImGui::GetContentRegionAvail().x - kPreviewChildW_H - style.ItemSpacing.x);
    ImGui::BeginChild("##modal_left_pane", ImVec2(kPreviewChildW_H, kPreviewChildHeight), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    RenderCrystalPreviewPane(state);
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("##modal_right_pane", ImVec2(right_w, kPreviewChildHeight), ImGuiChildFlags_None);
    RenderModalTabBar(state, crystal_label, axis_label, filter_label, crystal_flags, axis_flags, filter_flags);
    ImGui::EndChild();
  }

  // Immediate mode: push buffer→entry every frame (diff-gated inside
  // CommitAllBuffersImmediate; no-op when nothing changed). Esc / click-outside
  // close are default BeginPopup behaviors — no explicit handling needed.
  if (state.modal_immediate_mode) {
    CommitAllBuffersImmediate(state);
  }

  ImGui::Separator();
  // Bottom row: mode-specific buttons + Immediate mode checkbox anchored
  // to the right edge.
  if (state.modal_immediate_mode) {
    // Immediate: single Close (no Cancel semantics — changes are already applied).
    // No CloseCurrentPopup: we are inside ImGui::Begin (regular window), the
    // popup stack has no "Edit Entry" entry. Setting g_active_modal=kNone
    // drives the next-frame cleanup via title_x_open=false → Begin returns
    // false → !window_open branch.
    if (ImGui::Button("Close##edit_modal", ImVec2(80, 0))) {
      g_active_modal = ActiveModal::kNone;
    }
  } else {
    // Staged: OK / Cancel.
    // OK gate: empty raypath is treated as "no filter" (ApplyBuffersToEntry
    // writes nullopt), so validation only applies when the user actually
    // typed something. This subsumes the previous "Remove intent" gating
    // branch without a dedicated flag.
    bool ok_disabled = false;
    const char* ok_tooltip = nullptr;
    // Stub-type gate (issue editor-modal-type-selection): three new filter
    // types render placeholder UI only; OK must be disabled until the user
    // switches back to Raypath.
    if (g_filter_active_type != FilterEditType::kRaypath) {
      ok_disabled = true;
      ok_tooltip = "Filter type is not yet implemented — switch to Raypath to commit";
    } else {
      g_raypath_params.raypath_text = g_raypath_buf;
      if (!g_raypath_params.raypath_text.empty()) {
        const auto v = ValidateRaypathTextMultiSegment(g_raypath_params.raypath_text, CurrentValidationKind());
        if (v.state != RaypathValidation::kValid) {
          ok_disabled = true;
          ok_tooltip = "Filter raypath invalid — fix it in the Filter tab";
        }
      }
    }

    if (ok_disabled) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("OK##edit_modal", ImVec2(80, 0))) {
      CommitAllBuffers(state);
      g_active_modal = ActiveModal::kNone;
      ImGui::CloseCurrentPopup();
    }
    if (ok_disabled) {
      ImGui::EndDisabled();
    }
    if (ok_disabled && ok_tooltip != nullptr && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("%s", ok_tooltip);
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel##edit_modal", ImVec2(80, 0))) {
      // Cleanup (trackball restore, g_active_modal reset) is delegated to
      // HandlePopupClosed on the next frame via the !window_open path, so
      // Cancel / title-bar × / Esc all share a single code path.
      ImGui::CloseCurrentPopup();
    }
  }

  // View-preference toggles (Vertical layout + Immediate), right-aligned on the
  // button row. Both are checkboxes for visual consistency: neither marks the
  // file dirty, both affect UI presentation only.
  ImGui::SameLine();
  constexpr float kViewToggleGroupWidth = 210.0f;  // "Vertical" + "Immediate" checkboxes + padding
  const float avail = ImGui::GetContentRegionAvail().x;
  if (avail > kViewToggleGroupWidth) {
    ImGui::Dummy(ImVec2(avail - kViewToggleGroupWidth, 0));
    ImGui::SameLine();
  }
  // Vertical layout toggle (view preference — does NOT mark the file dirty).
  // Checked = stacked layout (preview on top); unchecked = side-by-side.
  ImGui::Checkbox("Vertical##layout_toggle", &state.modal_layout_vertical);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%s", state.modal_layout_vertical ? "Stacked layout (preview on top)" :
                                                          "Side-by-side layout (preview on left)");
  }
  ImGui::SameLine();
  // ImGui::Checkbox returns true only on the frame the user actually toggled
  // the value, so checking the return alone is sufficient to detect a change.
  if (ImGui::Checkbox("Immediate##edit_modal", &state.modal_immediate_mode)) {
    g_pending_mode_switch = true;
    if (state.modal_immediate_mode) {
      // Staged → Immediate: commit in-flight buffer to state so any pending
      // edits become the live baseline. Use the Immediate path (not
      // CommitAllBuffers) to avoid MarkFilterDirty clearing the display on
      // crystal-only changes — that would zero infinite-rays accumulation
      // at the exact moment the user wants to start observing live changes.
      CommitAllBuffersImmediate(state);
      // Current frame is still inside BeginPopupModal (dispatch at frame-start
      // used the old mode). CloseCurrentPopup keeps the popup stack clean;
      // Frame N+1 enters the Immediate Begin branch and consumes
      // g_pending_mode_switch inline (see race-case guard + inline consume).
      ImGui::CloseCurrentPopup();
    } else {
      // Immediate → Staged: re-snapshot the current buffer state as the new
      // dirty-compare baseline (dirty-mark starts from zero going forward).
      SnapshotAllBuffers(state);
      // Current frame is still inside ImGui::Begin (dispatch at frame-start
      // used the old mode). Do NOT call CloseCurrentPopup — the popup stack
      // has no "Edit Entry" entry to close. Flow:
      //   Frame N+1: Staged BeginPopupModal on empty stack → returns false →
      //              !window_open → HandlePopupClosed's mode-switch branch
      //              keeps g_active_modal=kOpen, arms g_pending_open +
      //              g_pending_tab_select.
      //   Frame N+2: L713 reopen gate fires → OpenPopup + BeginPopupModal
      //              return true same frame → modal visible with tab replay.
      // Invariant: do NOT touch g_active_modal here (must stay kOpen for the
      // reopen gate to fire in Frame N+2).
    }
  }

  if (dispatched_immediate) {
    ImGui::End();
  } else {
    ImGui::EndPopup();
  }
}

}  // namespace lumice::gui
