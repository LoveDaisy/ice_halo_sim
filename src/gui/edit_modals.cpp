#include "gui/edit_modals.hpp"

#include <algorithm>
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
static FilterConfig g_filter_buf;
static char g_raypath_buf[256];
// Snapshot of g_filter_buf at modal open + initial presence flag. Used by
// ApplyBuffersToEntry to decide whether to write entry.filter at all: when the
// entry had no filter originally AND the user did not touch any field, OK
// must leave entry.filter as nullopt (otherwise default-constructed values
// like sym_p=b=d=true would silently turn into a "* In PBD" filter that
// blocks all rays). Combined with the "empty raypath → nullopt" commit rule
// this also closes the "Remove button" path without a separate state bit.
static FilterConfig g_filter_buf_snapshot;
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
  // Sync UI raypath char buffer back into g_filter_buf before snapshotting,
  // so g_filter_buf_snapshot.raypath_text reflects the current edit state
  // (FilterConfig::operator== compares raypath_text; plan F10 detail).
  g_filter_buf.raypath_text = g_raypath_buf;
  g_filter_buf_snapshot = g_filter_buf;
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
  g_filter_buf = entry.filter.value_or(FilterConfig{});
  snprintf(g_raypath_buf, sizeof(g_raypath_buf), "%s", g_filter_buf.raypath_text.c_str());
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
  ImGui::PushItemWidth(120.0f);
  ImGui::Combo("##ModalCrystalStyle", &g_crystal_style, kCrystalStyleNames, kCrystalStyleCount);
  ImGui::PopItemWidth();
  ImGui::SameLine();
  if (ImGui::SmallButton("Reset View##modal")) {
    ResetCrystalView();
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
    }
    if (highlighted) {
      ImGui::PopStyleColor();
    }
  }

  ImGui::Separator();

  // Axis distribution controls (zenith: 0-180, azimuth: 0-360, roll: 0-360).
  // Return values intentionally ignored: modal operates on edit buffer, dirty state
  // is only committed on OK button press (not on each slider change).
  RenderAxisDist("Zenith", g_axis_buf[0], 0.0f, 180.0f);
  RenderAxisDist("Azimuth", g_axis_buf[1], 0.0f, 360.0f);
  RenderAxisDist("Roll", g_axis_buf[2], 0.0f, 360.0f);

  // OK / Cancel handled at modal level (RenderEditModals).
}


// ============================================================
// Filter Modal
// ============================================================

static void RenderFilterModal(GuiState& /*state*/) {
  // Validation kind is derived from g_crystal_buf (the in-flight buffer of the
  // Crystal tab) rather than the entry, because the user may switch the type
  // in the Crystal tab before committing — the validator should match what
  // will actually be written on OK.
  const lumice::CrystalKind kind =
      (g_crystal_buf.type == CrystalType::kPrism) ? lumice::CrystalKind::kPrism : lumice::CrystalKind::kPyramid;

  // Action combo
  ImGui::Combo("Action##filter_modal", &g_filter_buf.action, kFilterActionNames, kFilterActionCount);

  // Raypath text input with validation color feedback.
  // g_filter_buf is a detached copy of the filter; it is only written back to
  // the entry on OK (see below), so non-kValid input cannot leak into the
  // model via sibling dirty triggers.
  g_filter_buf.raypath_text = g_raypath_buf;
  const auto result = ValidateRaypathText(g_filter_buf.raypath_text, kind);
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

  ImGui::PushStyleColor(ImGuiCol_FrameBg,
                        ImVec4(border_color.x * 0.3f, border_color.y * 0.3f, border_color.z * 0.3f, 0.5f));
  ImGui::InputText("Raypath##filter_modal", g_raypath_buf, sizeof(g_raypath_buf));
  ImGui::PopStyleColor();

  // Validation hint
  switch (validation) {
    case RaypathValidation::kValid:
      if (g_filter_buf.raypath_text.empty()) {
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
      ImGui::TextColored(ImVec4(0.9f, 0.2f, 0.2f, 1.0f), "%s", msg);
      break;
    }
  }

  // Symmetry checkboxes
  ImGui::Checkbox("P##filter_modal", &g_filter_buf.sym_p);
  ImGui::SameLine();
  ImGui::Checkbox("B##filter_modal", &g_filter_buf.sym_b);
  ImGui::SameLine();
  ImGui::Checkbox("D##filter_modal", &g_filter_buf.sym_d);

  ImGui::Spacing();
  // Remove Filter — a plain edit action equivalent to the user backspacing the
  // raypath textbox to empty. action / sym_p / sym_b / sym_d are intentionally
  // preserved so a subsequent re-type of a raypath keeps the user's P/B/D
  // preferences; the "empty raypath → nullopt" rule inside ApplyBuffersToEntry
  // is what actually drops the filter from the entry on commit.
  const bool raypath_empty = (g_raypath_buf[0] == '\0');
  ImGui::BeginDisabled(raypath_empty);
  if (ImGui::Button("Remove Filter##filter", ImVec2(120, 0))) {
    g_raypath_buf[0] = '\0';
    g_filter_buf.raypath_text.clear();
  }
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
  g_filter_buf = {};
  g_filter_buf_snapshot = {};
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
// because the user may switch crystal type before committing.
lumice::CrystalKind CurrentValidationKind() {
  return (g_crystal_buf.type == CrystalType::kPrism) ? lumice::CrystalKind::kPrism : lumice::CrystalKind::kPyramid;
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
  // Sync the ImGui InputText backing buffer back into the struct before any
  // commit decision; g_raypath_buf is the canonical source for the raypath
  // text (see plan §F6 — this sync pattern is mirrored at L205/L513/L911/OK
  // gating).
  g_filter_buf.raypath_text = g_raypath_buf;
  if (g_filter_buf.raypath_text.empty()) {
    // Empty raypath ≡ "no filter" at the UI layer (the Remove button is just
    // a shortcut for "backspace the textbox empty"). This also subsumes the
    // old "Remove intent" path and the default-PBD leak: a default-constructed
    // FilterConfig has empty raypath, so untouched no-filter entries stay
    // nullopt here.
    entry.filter = std::nullopt;
  } else if (g_filter_initial_present || g_filter_buf != g_filter_buf_snapshot) {
    // Model-layer invariant: FilterConfig must never hold an invalid raypath,
    // regardless of commit mode. Staged mode already enforces this via the OK
    // button's disabled gate (ok_disabled check on the Staged OK path);
    // Immediate mode per-frame commits reach this branch with potentially
    // invalid text, so the guard here is the single model-layer gate that
    // makes the invariant hold in both modes. kIncomplete is treated same as
    // kInvalid — matches the Staged OK disjunction `v.state != kValid`,
    // preventing half-typed input from poisoning the renderer.
    auto v = ValidateRaypathText(g_filter_buf.raypath_text, CurrentValidationKind());
    if (v.state == RaypathValidation::kValid) {
      entry.filter = g_filter_buf;
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
  // Layout toggle button (view preference — does NOT mark the file dirty).
  // ASCII H / V labels avoid font fallback issues on platforms lacking Unicode arrows.
  const char* toggle_label = state.modal_layout_vertical ? "V##layout_toggle" : "H##layout_toggle";
  if (ImGui::SmallButton(toggle_label)) {
    state.modal_layout_vertical = !state.modal_layout_vertical;
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%s", state.modal_layout_vertical ?
                                "Vertical layout (preview on top)\nClick to switch to Horizontal" :
                                "Horizontal layout (preview on left)\nClick to switch to Vertical");
  }
  ImGui::SameLine();

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
  MonitorRect mon{};
  if (GetCurrentMonitorWorkArea(window, &mon)) {
    auto max_w = std::max(kEditModalMinWidth, static_cast<float>(mon.w - kWindowDecorationMargin));
    auto max_h = std::max(static_cast<float>(kMinWindowHeight), static_cast<float>(mon.h - kWindowDecorationMargin));
    ImGui::SetNextWindowSizeConstraints(ImVec2(kEditModalMinWidth, 0), ImVec2(max_w, max_h));
  } else {
    ImGui::SetNextWindowSizeConstraints(ImVec2(kEditModalMinWidth, 0), ImVec2(FLT_MAX, FLT_MAX));
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
  FilterConfig filter_cmp = g_filter_buf;
  filter_cmp.raypath_text = g_raypath_buf;
  // filter_dirty uses the in-flight buffer vs its open-time snapshot. Remove
  // button just clears g_raypath_buf, so filter_cmp.raypath_text ("") diverges
  // from snapshot (which holds the original raypath_text) and the * mark
  // lights up. A snapshot with empty raypath that the user leaves empty
  // stays equal, so dirty remains false — no false-positive mark.
  const bool crystal_dirty = g_crystal_buf != g_crystal_buf_snapshot;
  const bool axis_dirty = g_axis_buf[0] != g_axis_buf_snapshot[0] || g_axis_buf[1] != g_axis_buf_snapshot[1] ||
                          g_axis_buf[2] != g_axis_buf_snapshot[2];
  const bool filter_dirty = filter_cmp != g_filter_buf_snapshot;
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
    // Lower pane: tab bar + body, full width, remaining height with scrollbar.
    ImGui::BeginChild("##modal_bottom_pane", ImVec2(-FLT_MIN, -FLT_MIN), ImGuiChildFlags_None,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar);
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
    g_filter_buf.raypath_text = g_raypath_buf;
    if (!g_filter_buf.raypath_text.empty()) {
      const auto v = ValidateRaypathText(g_filter_buf.raypath_text, CurrentValidationKind());
      if (v.state != RaypathValidation::kValid) {
        ok_disabled = true;
        ok_tooltip = "Filter raypath invalid — fix it in the Filter tab";
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

  // Immediate-mode toggle checkbox, right-aligned on the button row.
  ImGui::SameLine();
  constexpr float kCheckboxApproxWidth = 110.0f;  // "Immediate" label + checkbox square + padding
  const float avail = ImGui::GetContentRegionAvail().x;
  if (avail > kCheckboxApproxWidth) {
    ImGui::Dummy(ImVec2(avail - kCheckboxApproxWidth, 0));
    ImGui::SameLine();
  }
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
