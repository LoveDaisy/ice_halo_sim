#include "gui/edit_modals.hpp"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
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

// Filter modal — H5 sum-of-products editor (task-composition-editor-ui / 333.4).
// One row per OR summand; each row is a single small-domain AND text box driven
// by ValidateSummandText / ParseSummandText (raypath_segments.hpp). The prior
// "FilterEditType + Raypath/EntryExit sub-panels + per-type buffers" scheme is
// gone — every row can independently be raypath, entry-exit, or an AND mix.
constexpr size_t kSummandRowBufSize = 256;
// UI soft cap (kMaxSummandRows): prevents unbounded row growth from the "+ Add"
// button before the real ABI-layer limits kick in. The authoritative overflow
// gates live in file_io.cpp::FillLumiceConfig (ExpandSopToClauses → clauses vec
// with LUMICE_kMaxComplexFilterClauses cap) and BuildExportJsonOrWarn; those
// remain the last-word validators. 16 is the OR-summand row count and comfortably
// fits typical multi-branch filters (a real config usually has ≤ 4 rows).
constexpr size_t kMaxSummandRows = 16;

struct SummandRowBuf {
  uint64_t uid;
  char text[kSummandRowBufSize];
};

static std::vector<SummandRowBuf> g_summand_rows;
static SumOfProducts g_summand_rows_snapshot;
static uint64_t g_next_summand_row_uid = 0;

// Shared filter fields (name / action / sym_*). `param` is never mutated on
// g_filter_top — it stays at the default-constructed SoP so operator== reduces
// to the top fields (payload lives in g_summand_rows).
static FilterConfig g_filter_top;
static FilterConfig g_filter_top_snapshot;

// Initial-present flag captured at modal open. Used by ApplyBuffersToEntry so
// an untouched OK on a previously-empty filter does not silently materialize a
// default-constructed filter into entry.filter. Combined with the
// "effectively-empty → nullopt" commit rule this also closes the "Remove
// Filter" button path together with g_filter_remove_intent.
static bool g_filter_initial_present = false;
static bool g_filter_remove_intent = false;

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
// Wedge angle presets for edit modal
// ============================================================

namespace {

struct ValuePreset {
  const char* label;
  float value;
};

// Label precision must match the fmt passed to SliderWithPresetEdit (currently "%.3f").
constexpr ValuePreset kWedgePresets[] = {
  { "{1,0,-1,1} 28.000\xc2\xb0", 28.0f },
  { "{2,0,-2,1} 47.300\xc2\xb0", 47.3f },
  { "{1,0,-1,0} 90.000\xc2\xb0", 90.0f },
  { "{1,0,-1,2} 14.700\xc2\xb0", 14.7f },
};
constexpr int kWedgePresetCount = 4;

// Render a slider + input + preset dropdown for wedge angle.
// Edit-buffer context: cannot call MarkDirty internally, so this is a standalone impl.
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
  float slider_w = avail_w - kInputWidth - kLabelColWidth - spacing * 2;  // mirrors PrepareSliderLayout
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

// Build a SumOfProducts from the current row buffers. Each row's text is
// re-parsed via ParseSummandText (tolerant parser: invalid tokens are dropped
// so a mid-typing row still produces a coherent Factor vector).
SumOfProducts BuildSopFromRows(const std::vector<SummandRowBuf>& rows) {
  SumOfProducts out;
  out.reserve(rows.size());
  for (const auto& row : rows) {
    std::string text = row.text;
    auto factors = ParseSummandText(text);
    out.push_back(SummandText{ std::move(text), std::move(factors) });
  }
  return out;
}

// Load an incoming SoP into the row buffers, resetting the uid counter. Always
// ensures at least one row exists (mirrors FilterConfig's ≥1 row invariant).
void SetRowsFromSop(const SumOfProducts& sop) {
  g_summand_rows.clear();
  g_next_summand_row_uid = 0;
  if (sop.empty()) {
    SummandRowBuf row{};
    row.uid = g_next_summand_row_uid++;
    row.text[0] = '\0';
    g_summand_rows.push_back(row);
    return;
  }
  for (const auto& s : sop) {
    SummandRowBuf row{};
    row.uid = g_next_summand_row_uid++;
    snprintf(row.text, sizeof(row.text), "%s", s.text.c_str());
    g_summand_rows.push_back(row);
  }
}

// Re-snapshot all three buffers as the new dirty-compare baseline. Used by
// OpenEditModal (initial snapshot) and the Immediate→Staged mode switch
// (fresh baseline so dirty-mark starts from zero). Does NOT touch trackball
// save — that retains the Open-time baseline.
//
// Any new edit-buffer field added in the future must be appended here AND
// in ApplyBuffersToEntry to keep the dirty-compare / commit paths symmetric.
void SnapshotAllBuffers(const GuiState& state) {
  g_filter_top_snapshot = g_filter_top;
  g_summand_rows_snapshot = BuildSopFromRows(g_summand_rows);
  g_crystal_buf_snapshot = g_crystal_buf;
  g_axis_buf_snapshot[0] = g_axis_buf[0];
  g_axis_buf_snapshot[1] = g_axis_buf[1];
  g_axis_buf_snapshot[2] = g_axis_buf[2];
  const int ly = g_modal_layer_idx;
  const int en = g_modal_entry_idx;
  if (ly >= 0 && ly < static_cast<int>(state.layers.size()) && en >= 0 &&
      en < static_cast<int>(state.layers[ly].entries.size())) {
    g_filter_initial_present = state.layers[ly].entries[en].filter_id.has_value();
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
  //
  // Reallocation note: do NOT call state.crystals.reserve() here. External
  // callers may hold const references into the pool (e.g. tests that bind
  // `auto& cr = state.crystals[...]` across a modal-open boundary); growing
  // the vector would invalidate those. The modal's own reads below copy into
  // local buffers immediately, so no long-lived pool reference is needed.
  // Pick-mode push_back paths (Link / Unlink / Duplicate) happen *after* the
  // modal closes, when no external references are held.
  const CrystalConfig& src_crystal = state.crystals[entry.crystal_id];
  g_crystal_buf = src_crystal;
  g_axis_buf[0] = src_crystal.zenith;
  g_axis_buf[1] = src_crystal.azimuth;
  g_axis_buf[2] = src_crystal.roll;
  // Filter buffers (H5 sum-of-products): top-level shared fields (name / action
  // / sym_*) go into g_filter_top; per-row SoP text goes into g_summand_rows,
  // keyed by stable uid. Default-constructed FilterConfig carries a 1-row empty
  // SoP so "no filter" opens with exactly one blank row (matches the pre-task
  // "empty raypath ≡ no filter" UX).
  const FilterConfig src = entry.filter_id.has_value() ? state.filters[*entry.filter_id] : FilterConfig{};
  g_filter_top = FilterConfig{};
  g_filter_top.name = src.name;
  g_filter_top.action = src.action;
  g_filter_top.sym_p = src.sym_p;
  g_filter_top.sym_b = src.sym_b;
  g_filter_top.sym_d = src.sym_d;
  SetRowsFromSop(src.param);
  g_filter_remove_intent = false;
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
    DrawFaceNumberOverlay(m, g_crystal_rotation, mvp, g_crystal_zoom, preview_pos,
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
    SliderWithPresetEdit("Upper A##modal_cr", &cr.upper_alpha, 0.1f, 90.0f, "%.3f", SliderScale::kLinear, kWedgePresets,
                         kWedgePresetCount);
    SliderWithPresetEdit("Lower A##modal_cr", &cr.lower_alpha, 0.1f, 90.0f, "%.3f", SliderScale::kLinear, kWedgePresets,
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
LUMICE_CrystalKind CurrentValidationKind();
}

static ImVec4 ValidationFrameBgColor(LUMICE_RaypathValidationState state) {
  switch (state) {
    case LUMICE_RAYPATH_VALID:
      return ImVec4(0.06f, 0.24f, 0.06f, 0.5f);
    case LUMICE_RAYPATH_INCOMPLETE:
      return ImVec4(0.27f, 0.24f, 0.03f, 0.5f);
    case LUMICE_RAYPATH_INVALID:
      return ImVec4(0.27f, 0.06f, 0.06f, 0.5f);
  }
  return ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
}

namespace {
// Destructive (red) button palette for delete/remove buttons in this TU
// (spectrum-row delete, summand-row delete). RGB values kept in sync with
// panels.cpp's file-local kBtnDestructive*/PushDestructiveStyle — 2 independent
// definitions synchronized by convention, not via a shared header. If a third
// TU ever needs this palette, promote it to a shared header rather than
// replicating a third copy.
void PushDestructiveStyle() {
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.22f, 0.22f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.30f, 0.30f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.60f, 0.15f, 0.15f, 1.0f));
}
void PopDestructiveStyle() {
  ImGui::PopStyleColor(3);
}
}  // namespace

// Row-list editor for the sum-of-products (H5). Each row is one OR summand
// expressed in the small AND grammar (`3-5 & entry:2 & len:3-5`), validated
// with ValidateSummandText / ParseSummandText (raypath_segments.hpp). Stable
// per-row `uid` is baked into both the InputText and the delete-button IDs so
// removing a middle row cannot re-collide ImGui's internal id stack with a
// leftover buffer.
static void RenderSummandRowList() {
  const auto kind = CurrentValidationKind();
  size_t delete_idx = static_cast<size_t>(-1);
  const bool can_delete_any = g_summand_rows.size() > 1;

  // Reserve exactly the width the trailing "x" SmallButton needs (glyph + horizontal
  // FramePadding on each side) plus one ItemSpacing for SameLine(). Formula mirrors
  // panels.cpp's crystal-card hover-button sizing so per-row layout stays consistent
  // across fonts/DPI without a hard-coded magic number.
  const float frame_pad_x = ImGui::GetStyle().FramePadding.x;
  const float item_spacing_x = ImGui::GetStyle().ItemSpacing.x;
  const float del_btn_w = ImGui::CalcTextSize(ICON_FA_XMARK).x + frame_pad_x * 2.0f;
  const float row_item_width = -(del_btn_w + item_spacing_x);

  for (size_t i = 0; i < g_summand_rows.size(); ++i) {
    auto& row = g_summand_rows[i];
    ImGui::PushID(static_cast<int>(row.uid));

    const auto v = ValidateSummandText(row.text, kind);
    // Empty row keeps a neutral FrameBg (wildcard, matches "no filter" semantic).
    const bool is_empty = (row.text[0] == '\0');
    if (!is_empty) {
      ImGui::PushStyleColor(ImGuiCol_FrameBg, ValidationFrameBgColor(v.state));
    }
    char text_id[64];
    snprintf(text_id, sizeof(text_id), "##row_text_%llu", static_cast<unsigned long long>(row.uid));
    ImGui::PushItemWidth(row_item_width);  // leave room for the trailing "x" delete button
    ImGui::InputText(text_id, row.text, sizeof(row.text));
    ImGui::PopItemWidth();
    if (!is_empty) {
      ImGui::PopStyleColor();
    }

    ImGui::SameLine();
    char del_id[64];
    snprintf(del_id, sizeof(del_id), ICON_FA_XMARK "##row_delete_%llu", static_cast<unsigned long long>(row.uid));
    // Match crystal-card / spectrum-row convention: red destructive style when
    // enabled, greyed-out (not tinted red) via BeginDisabled when the last row
    // may not be removed.
    if (can_delete_any) {
      PushDestructiveStyle();
    } else {
      ImGui::BeginDisabled();
    }
    if (ImGui::SmallButton(del_id)) {
      delete_idx = i;
    }
    if (can_delete_any) {
      PopDestructiveStyle();
    } else {
      ImGui::EndDisabled();
    }

    // Per-row inline validation hint (first non-valid across the list is
    // enough to gate OK; still show every offending row so the user can fix
    // them in any order).
    if (!is_empty) {
      switch (v.state) {
        case LUMICE_RAYPATH_VALID:
          break;  // silent when valid
        case LUMICE_RAYPATH_INCOMPLETE:
          ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.1f, 1.0f), "Row %zu: incomplete", i + 1);
          break;
        case LUMICE_RAYPATH_INVALID: {
          const char* msg = v.message.empty() ? "Invalid" : v.message.c_str();
          ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
          ImGui::TextWrapped("Row %zu: %s", i + 1, msg);
          ImGui::PopStyleColor();
          break;
        }
      }
    }

    ImGui::PopID();
  }

  if (delete_idx != static_cast<size_t>(-1)) {
    g_summand_rows.erase(g_summand_rows.begin() + static_cast<std::ptrdiff_t>(delete_idx));
  }

  // Add-row button: capped at kMaxSummandRows (soft UI cap; hard cap enforced by
  // FillLumiceConfig / BuildExportJsonOrWarn at the ABI boundary).
  const bool at_cap = g_summand_rows.size() >= kMaxSummandRows;
  ImGui::BeginDisabled(at_cap);
  if (ImGui::Button("+ Add OR row##summand_add", ImVec2(140, 0))) {
    SummandRowBuf row{};
    row.uid = g_next_summand_row_uid++;
    row.text[0] = '\0';
    g_summand_rows.push_back(row);
  }
  ImGui::EndDisabled();
  if (at_cap && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Maximum OR rows reached (%zu)", kMaxSummandRows);
  }

  // Static hint — includes ';' example post-334.3 (H-A). Kept as one line so
  // it does not compete with the live-preview block below.
  ImGui::TextDisabled("e.g. 3-5  or  1-3;3-5 (OR)  or  entry:2 & exit:4  or  3-5 & len:2-3");

  // Token help icon: transparent SmallButton acts as a stable hover target
  // (mirrors RenderSharedFilterControls' `kDTooltipText` icon pattern).
  ImGui::SameLine();
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0, 0, 0, 0));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0, 0, 0, 0));
  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
  ImGui::SmallButton(ICON_FA_CIRCLE_INFO "##summand_token_help");
  ImGui::PopStyleVar();
  ImGui::PopStyleColor(4);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Token syntax:\n"
        "  3-5           face path (a raypath token)\n"
        "  1-3;3-5       raypath OR alternatives (';' distributes over '&')\n"
        "  entry:2       enter through face 2\n"
        "  exit:4        exit through face 4\n"
        "  len:2-3       ray length range\n"
        "  entry:1,2     comma-list = 'entry:1 OR entry:2' (expanded at filter apply)\n"
        "  & (AND)       within a row\n"
        "  new row       = OR alternative (sum-of-products)\n"
        "  blank row     = match all rays (no filter)");
  }

  // Live preview of the sum-of-products the current row buffers would
  // expand to on commit. Uses the SAME parse/format helpers as the commit
  // path (BuildSopFromRows -> FormatSopExpansionPreview) so what the user
  // sees IS what will be serialized. Skipped when the row set is the
  // trivial "single blank row" (match-all) state — otherwise the preview
  // just repeats the hint line above with no new information.
  const bool show_live_preview =
      g_summand_rows.size() > 1 || (g_summand_rows.size() == 1 && g_summand_rows[0].text[0] != '\0');
  if (show_live_preview) {
    SumOfProducts live_sop = BuildSopFromRows(g_summand_rows);
    // Row count here is small (soft cap = kMaxSummandRows, typically single
    // digits). Per-frame re-parse cost is negligible for immediate-mode UI;
    // no caching needed. If the cap ever grows materially, revisit.
    ImGui::Separator();
    ImGui::TextDisabled("Preview:");
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(FormatSopExpansionPreview(live_sop).c_str());
    ImGui::PopTextWrapPos();
  }
}

// Returns true when the current entry's axis config satisfies D-symmetry
// conditions: azimuth uniform 360° AND roll mean a multiple of 30°.
// Mirrors core detail::IsDApplicable (src/core/crystal.cpp). Keep in sync.
static bool IsDApplicableGuiAxis(const AxisDist& az, const AxisDist& roll) {
  const bool az_sym = az.type == AxisDistType::kUniform && std::fabs(az.std - 360.0f) < 1e-3f;
  const float rem = std::fmod(std::fmod(roll.mean, 30.0f) + 30.0f, 30.0f);
  const bool roll_ok = rem < 1e-3f || std::fabs(rem - 30.0f) < 1e-3f;
  return az_sym && roll_ok;
}

static constexpr const char* kDTooltipText =
    "D applies when azimuth = uniform 360\xc2\xb0 and roll mean is a multiple of 30\xc2\xb0.\n"
    "Current config does not meet this condition, so D has no effect.";

// Shared filter controls (Action radio + P/B/D), rendered after the
// type-specific dispatch. Always uses g_filter_top so type switches don't
// reset shared user preferences.
static void RenderSharedFilterControls(bool d_applicable) {
  // Action: two RadioButtons (was Combo pre-task; aligns with Crystal tab style).
  // Always rendered — filter_in / filter_out semantics apply to every type.
  if (ImGui::RadioButton("Filter In##filter_action", g_filter_top.action == 0)) {
    g_filter_top.action = 0;
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Show only rays matching the filter");
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Filter Out##filter_action", g_filter_top.action == 1)) {
    g_filter_top.action = 1;
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Hide rays matching the filter");
  }

  // P/B/D Checkboxes: unconditional under H5 (every row is raypath / EE / an
  // AND mix — both types consume crystal symmetry at the core layer). The
  // pre-H5 `sym_active` gate was already effectively constant since only two
  // discriminator values existed; H5 simply removes the discriminator.
  ImGui::Checkbox("P##filter_modal", &g_filter_top.sym_p);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Prism-face reflection symmetry");
  }
  ImGui::SameLine();
  ImGui::Checkbox("B##filter_modal", &g_filter_top.sym_b);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Basal-face reflection symmetry");
  }
  ImGui::SameLine();
  ImGui::Checkbox("D##filter_modal", &g_filter_top.sym_d);
  if (!d_applicable) {
    ImGui::SameLine();
    // SmallButton with transparent styling acts as a hover target for the
    // tooltip (TextDisabled lacks a stable item ID needed by the test engine).
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
    ImGui::SmallButton(ICON_FA_CIRCLE_INFO "##d_tooltip_icon");
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(4);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", kDTooltipText);
    }
  }
}

// Remove Filter — arms g_filter_remove_intent so ApplyBuffersToEntry writes
// `filter_id = nullopt` on OK regardless of row validation state, and resets
// the row buffers to a single blank row (matches the "empty ≡ no filter" UX).
// Delayed-commit pattern (intent flag preserved across Cancel restore path) —
// see imgui-modal-test.md.
static void RenderRemoveFilterButton() {
  if (ImGui::Button("Remove Filter##filter", ImVec2(120, 0))) {
    g_summand_rows.clear();
    g_next_summand_row_uid = 0;
    SummandRowBuf row{};
    row.uid = g_next_summand_row_uid++;
    row.text[0] = '\0';
    g_summand_rows.push_back(row);
    g_filter_remove_intent = true;
  }
}

static void RenderFilterModal() {
  RenderSummandRowList();

  ImGui::Spacing();

  bool d_applicable = false;
  {
    const int ly = g_modal_layer_idx;
    const int en = g_modal_entry_idx;
    if (ly >= 0 && ly < static_cast<int>(g_state.layers.size()) && en >= 0 &&
        en < static_cast<int>(g_state.layers[ly].entries.size())) {
      const auto& entry_ref = g_state.layers[ly].entries[en];
      const auto& cr = g_state.crystals[entry_ref.crystal_id];
      d_applicable = IsDApplicableGuiAxis(cr.azimuth, cr.roll);
    }
  }
  RenderSharedFilterControls(d_applicable);

  ImGui::Spacing();

  RenderRemoveFilterButton();

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

EditTarget GetActiveTabAsEditTarget() {
  switch (g_active_tab) {
    case ActiveTab::kCrystal:
      return EditTarget::kCrystal;
    case ActiveTab::kAxis:
      return EditTarget::kAxis;
    case ActiveTab::kFilter:
      return EditTarget::kFilter;
  }
  return EditTarget::kCrystal;
}

namespace {
// Defined further down alongside the spectrum-modal statics (same anonymous namespace, this TU).
void ResetSpectrumModalStateGlobals();
}  // namespace

void ResetModalState() {
  ResetSpectrumModalStateGlobals();  // clear the spectrum-editor statics too (Major: test isolation)
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
  g_summand_rows.clear();
  g_summand_rows_snapshot.clear();
  g_next_summand_row_uid = 0;
  g_filter_initial_present = false;
  g_filter_remove_intent = false;
  std::memset(g_saved_rotation, 0, sizeof(g_saved_rotation));
  g_saved_zoom = 1.0f;
  g_pending_open = false;
  g_pending_mode_switch = false;
  g_modal_mesh_hash = 0;
}

bool IsCurrentModalDApplicable() {
  const int ly = g_modal_layer_idx;
  const int en = g_modal_entry_idx;
  if (ly < 0 || ly >= static_cast<int>(g_state.layers.size())) {
    return false;
  }
  if (en < 0 || en >= static_cast<int>(g_state.layers[ly].entries.size())) {
    return false;
  }
  const auto& entry_ref = g_state.layers[ly].entries[en];
  const auto& cr = g_state.crystals[entry_ref.crystal_id];
  return IsDApplicableGuiAxis(cr.azimuth, cr.roll);
}

namespace {

// Validation kind based on the in-flight Crystal-tab buffer (not the entry),
// because the user may switch crystal type before committing — the validator
// should match what will actually be written on OK. Single source of truth
// for the Filter sub-panel and the modal-level OK gate.
LUMICE_CrystalKind CurrentValidationKind() {
  return (g_crystal_buf.type == CrystalType::kPrism) ? LUMICE_CRYSTAL_PRISM : LUMICE_CRYSTAL_PYRAMID;
}

// Filter-tab dirty predicate. Compares the shared top fields (name / action /
// sym_*) and the materialized SoP (row text list) against their open-time
// snapshots. Since SummandText::operator== only compares `.text`, the parsed
// factors cache is intentionally excluded — text is the single canonical form.
//
// Used by both the tab-label dirty mark in RenderEditModals AND the
// `buf_changed` gate inside ApplyBuffersToEntry (commit decision) so both
// consumers share one definition.
bool IsFilterDirty() {
  if (g_filter_top != g_filter_top_snapshot) {
    return true;
  }
  return BuildSopFromRows(g_summand_rows) != g_summand_rows_snapshot;
}

// Result of applying edit buffers back to the entry. Used by both commit paths
// to decide which dirty-notification side effects to fire.
//
// ID-pool model: "entry_changed" covers both pool-content mutation (writing the
// edited buffer back to state.crystals[entry.crystal_id]) AND entry-ref id
// changes; the commit path computes it by comparing the pool slot before vs
// after the write, plus the entry's own id fields.
struct ApplyBuffersResult {
  bool valid;           // false if g_modal_layer_idx / g_modal_entry_idx out of range
  bool entry_changed;   // crystal/axis/filter content or entry ids changed
  bool filter_changed;  // filter pool content or entry.filter_id changed
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

  // Diff-gate (ID-pool model): the entry struct itself only carries ids; the
  // crystal/filter content lives in the pool. Compare pool-slot content (and
  // entry's filter_id) before vs after the write to detect "anything changed"
  // for MarkDirty / render-invalidation decisions.
  const EntryCard old_entry = entry;
  const CrystalConfig old_crystal = state.crystals[entry.crystal_id];
  const std::optional<FilterConfig> old_filter =
      entry.filter_id.has_value() ? std::optional<FilterConfig>{ state.filters[*entry.filter_id] } : std::nullopt;

  // Write Crystal-tab buffer back into the pool slot referenced by entry.
  // Every entry sharing entry.crystal_id will observe this on next render.
  CrystalConfig& pool_crystal = state.crystals[entry.crystal_id];
  pool_crystal = g_crystal_buf;
  pool_crystal.zenith = g_axis_buf[0];
  pool_crystal.azimuth = g_axis_buf[1];
  pool_crystal.roll = g_axis_buf[2];

  // Local helper: propagate a filter_id change from `entry` to all entries
  // that were "linked" with it before the change — i.e., entries sharing the
  // same (crystal_id, old_filter_id) pair. Preserves the "linked group is an
  // atomic share unit" semantic when a filter is added or removed: editing
  // one card's filter must also flip the linked siblings' filter_id so the
  // group stays coherent (otherwise the fa-link badge disappears the moment
  // a filter is added to a previously filter-less linked group). Mutating
  // pool slot contents in-place (filter edit on an already-bound slot) is
  // unaffected — siblings see the new content via the shared filter_id.
  auto propagate_filter_id_to_linked = [&](int cid, std::optional<int> old_filter_id) {
    if (entry.filter_id == old_filter_id) {
      return;  // nothing changed; in-place edit, siblings already see it
    }
    for (auto& layer : state.layers) {
      for (auto& other : layer.entries) {
        if (other.crystal_id == cid && other.filter_id == old_filter_id) {
          other.filter_id = entry.filter_id;
        }
      }
    }
  };

  // Local helper: write the materialized FilterConfig into the pool, updating
  // entry.filter_id (existing slot or new append). When append is needed
  // (entry had no filter), propagate the new filter_id to entries that were
  // linked with `entry` at (crystal_id, None) so the group stays coherent.
  auto write_filter_to_pool = [&](const FilterConfig& f) {
    if (entry.filter_id.has_value()) {
      state.filters[*entry.filter_id] = f;
    } else {
      const std::optional<int> old_filter_id = entry.filter_id;
      entry.filter_id = static_cast<int>(state.filters.size());
      state.filters.push_back(f);
      propagate_filter_id_to_linked(entry.crystal_id, old_filter_id);
    }
  };

  // H5 sum-of-products commit. Three exit paths:
  //   1. Explicit Remove Filter (intent flag): drop filter_id unconditionally,
  //      skip row validation.
  //   2. Effectively-empty SoP (no non-blank rows): drop filter_id — matches
  //      the pre-task "empty raypath ≡ no filter" UX under H5 semantics.
  //   3. At least one non-blank row, all rows validate as kValid: materialize
  //      the (blank-stripped) SoP into the pool. Rows that are kIncomplete /
  //      kInvalid gate the write out (mirrors the pre-task per-type kValid gate).
  if (g_filter_remove_intent) {
    const std::optional<int> old_filter_id = entry.filter_id;
    entry.filter_id = std::nullopt;
    propagate_filter_id_to_linked(entry.crystal_id, old_filter_id);
    g_filter_remove_intent = false;
  } else {
    const bool buf_changed = IsFilterDirty();
    SumOfProducts sop = BuildSopFromRows(g_summand_rows);
    // Drop blank / whitespace-only summand rows before materializing. A blank
    // row carries no predicate and must NOT lower to a match-all clause (which
    // would make an OR filter a silent no-op, or under filter_out hide every
    // ray — the black-render footgun). This generalizes the pre-task
    // "empty ≡ no filter" UX from a single blank row to interior / extra blank
    // rows in a multi-row SoP. Blank rows validate as kValid, so they never
    // gate OK; stripping them here is the single point that keeps a forgotten
    // empty row from corrupting the committed filter.
    sop.erase(
        std::remove_if(sop.begin(), sop.end(), [](const SummandText& s) { return TrimRaypathSegment(s.text).empty(); }),
        sop.end());
    if (sop.empty()) {
      // No non-blank rows ≡ no filter.
      const std::optional<int> old_filter_id = entry.filter_id;
      entry.filter_id = std::nullopt;
      propagate_filter_id_to_linked(entry.crystal_id, old_filter_id);
    } else if (g_filter_initial_present || buf_changed) {
      const auto kind = CurrentValidationKind();
      bool all_valid = true;
      for (const auto& row : g_summand_rows) {
        const auto v = ValidateSummandText(row.text, kind);
        if (v.state != LUMICE_RAYPATH_VALID) {
          all_valid = false;
          break;
        }
      }
      if (all_valid) {
        FilterConfig out;
        out.name = g_filter_top.name;
        out.action = g_filter_top.action;
        out.sym_p = g_filter_top.sym_p;
        out.sym_b = g_filter_top.sym_b;
        out.sym_d = g_filter_top.sym_d;
        out.param = std::move(sop);
        write_filter_to_pool(out);
      }
    }
  }

  // Recompute filter content post-write to compare against the pre-write
  // snapshot. The entry-ref filter_id may have flipped (set/cleared) and/or
  // its pool slot content may have changed.
  const std::optional<FilterConfig> new_filter =
      entry.filter_id.has_value() ? std::optional<FilterConfig>{ state.filters[*entry.filter_id] } : std::nullopt;
  const bool crystal_changed = pool_crystal != old_crystal;
  const bool filter_changed = new_filter != old_filter;
  const bool entry_changed = crystal_changed || filter_changed || entry != old_entry;
  return { true, entry_changed, filter_changed };
}

// Staged OK path: any entry change clears the display + restarts simulation
// (existing semantics — OK implies user commits and sim will re-run).
void CommitAllBuffers(GuiState& state) {
  const auto r = ApplyBuffersToEntry(state);
  if (!r.valid || !r.entry_changed) {
    return;
  }
  // Invalidate by crystal_id: all entries sharing this crystal get a fresh thumbnail.
  g_thumbnail_cache.Invalidate(state.layers[g_modal_layer_idx].entries[g_modal_entry_idx].crystal_id);
  state.MarkDirty();
  state.MarkFilterDirty();
  g_crystal_mesh_hash = -1;
}

// Immediate path: crystal/axis edits only MarkDirty; MarkFilterDirty (which
// clears snapshot_intensity and raises the display epoch floor to fence stale
// old-generation textures) is gated on filter actually changing. This is what
// allows infinite-rays accumulation to persist while the user drags a crystal slider.
void CommitAllBuffersImmediate(GuiState& state) {
  const auto r = ApplyBuffersToEntry(state);
  if (!r.valid || !r.entry_changed) {
    return;
  }
  g_thumbnail_cache.Invalidate(state.layers[g_modal_layer_idx].entries[g_modal_entry_idx].crystal_id);
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
      RenderFilterModal();
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

  // ---- Sharing status row (Link to... / Unlink) ----
  // Header above the tab bar showing how many entries share this card's
  // (crystal_id, filter_id) pair. "Link to..." starts pick-mode; "Unlink"
  // forks the pool slots so this entry becomes independent.
  {
    const int ly = g_modal_layer_idx;
    const int en = g_modal_entry_idx;
    if (ly >= 0 && ly < static_cast<int>(state.layers.size()) && en >= 0 &&
        en < static_cast<int>(state.layers[ly].entries.size())) {
      const auto& cur_entry = state.layers[ly].entries[en];
      const int shared_with = CountEntriesSharing(state, cur_entry.crystal_id, cur_entry.filter_id) - 1;
      if (shared_with > 0) {
        ImGui::TextDisabled("Shared with %d other entr%s", shared_with, shared_with == 1 ? "y" : "ies");
      } else {
        ImGui::TextDisabled("Not shared");
      }
      ImGui::SameLine();
      // "Link to..." — commit current buffer, arm pick-mode, close modal.
      if (ImGui::SmallButton("Link to...##share")) {
        CommitAllBuffersImmediate(state);
        state.pick_link_source = GuiState::EntryRef{ ly, en };
        g_active_modal = ActiveModal::kNone;
        if (!state.modal_immediate_mode) {
          ImGui::CloseCurrentPopup();
        }
      }
      if (shared_with > 0) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Unlink##share")) {
          // Commit pending edits BEFORE forking pool slots so the active edit
          // doesn't carry over to the other entries we were sharing with.
          CommitAllBuffersImmediate(state);
          if (UnlinkEntryFromPool(state, ly, en)) {
            // Re-read the just-cloned slot into the modal buffer so g_crystal_buf
            // (etc.) match the new entry, not the stale shared slot.
            const auto& fresh_entry = state.layers[ly].entries[en];
            const CrystalConfig& src_crystal = state.crystals[fresh_entry.crystal_id];
            g_crystal_buf = src_crystal;
            g_axis_buf[0] = src_crystal.zenith;
            g_axis_buf[1] = src_crystal.azimuth;
            g_axis_buf[2] = src_crystal.roll;
            SnapshotAllBuffers(state);
            state.MarkDirty();
          }
        }
      }
      ImGui::Separator();
    }
  }

  // Per-tab dirty detection. The label picks up a trailing " *" when the
  // in-flight buffer differs from the snapshot taken at modal-open. All
  // labels share a fixed `###` suffix — with three hashes ImGui derives the
  // internal ID purely from the `###suffix` portion, so the display string
  // can vary ("Crystal" vs "Crystal *") without changing the tab's hash
  // (otherwise the tab would lose its SelectedTabId the moment dirty flips,
  // falling back to the first tab and hiding the user's work-in-progress).
  // filter_dirty: delegate to IsFilterDirty() so the tab-label "*" and the
  // commit-time `buf_changed` predicate share a single source of truth. Row
  // text edits go directly through InputText → row.text (no separate char
  // buffer to mirror any more under H5), so no pre-sync is needed here.
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
    // ok_tooltip_storage owns the row-index-templated string; ok_tooltip points
    // into it (or into a static literal). SetTooltip("%s", ok_tooltip) is only
    // called after `.c_str()` is valid for the duration of the frame — the
    // storage outlives the IsItemHovered check below.
    std::string ok_tooltip_storage;
    const char* ok_tooltip = nullptr;
    if (!g_filter_remove_intent) {
      const auto kind = CurrentValidationKind();
      for (size_t i = 0; i < g_summand_rows.size(); ++i) {
        const auto v = ValidateSummandText(g_summand_rows[i].text, kind);
        if (v.state == LUMICE_RAYPATH_VALID) {
          continue;
        }
        ok_disabled = true;
        if (v.state == LUMICE_RAYPATH_INCOMPLETE) {
          ok_tooltip_storage = "Row " + std::to_string(i + 1) + ": finish typing (incomplete)";
        } else {
          const char* msg = v.message.empty() ? "invalid" : v.message.c_str();
          ok_tooltip_storage = "Row " + std::to_string(i + 1) + ": " + msg;
        }
        ok_tooltip = ok_tooltip_storage.c_str();
        break;
      }
    }

    if (ok_disabled) {
      ImGui::BeginDisabled();
    }
    // Use the wider of the two labels for both buttons so the OK/Cancel pair
    // stays visually balanced regardless of glyph-width differences.
    const char* kOkLabel = ICON_FA_CHECK " OK##edit_modal";
    const char* kCancelLabel = ICON_FA_XMARK " Cancel##edit_modal";
    float ok_cancel_width =
        std::max(ImGui::CalcTextSize(kOkLabel, nullptr, true).x, ImGui::CalcTextSize(kCancelLabel, nullptr, true).x) +
        ImGui::GetStyle().FramePadding.x * 2.0f;
    if (ImGui::Button(kOkLabel, ImVec2(ok_cancel_width, 0))) {
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
    if (ImGui::Button(kCancelLabel, ImVec2(ok_cancel_width, 0))) {
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
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Apply parameter changes to simulation in real-time");
  }

  if (dispatched_immediate) {
    ImGui::End();
  } else {
    ImGui::EndPopup();
  }
}

// ========== Custom Spectrum Modal ==========
//
// Separate from the per-entry Crystal/Axis/Filter modal above: spectrum is a global light-source
// property with no entry index, so it does not share g_modal_layer_idx / g_active_modal state.
// See plan.md §3 #2 for the rationale (avoid coupling the per-entry index invariants).

namespace {

bool g_spectrum_modal_open_pending = false;
std::vector<WlWeight> g_spectrum_edit_buf;
// NOTE: no g_spectrum_prev_index / g_spectrum_modal_active. spectrum_index is committed to
// kCustomSpectrumIndex ONLY inside the OK button (transactionally with custom_spectrum), so Escape /
// Cancel / click-outside all leave spectrum_index at its prior valid value — no per-exit-path restore
// bookkeeping is needed.

// Hard-coded uniform starting point (equal-weight probes across visible band). Not the actual
// illuminant SPD — the GUI cannot include core light_config headers per the API boundary rule
// (AGENTS.md). Real SPD resampling would need a new C API function; see plan.md §2 default #1.
std::vector<WlWeight> BuildPresetSeed() {
  constexpr int kSeedCount = 9;
  constexpr float kLo = 400.0f, kHi = 720.0f;
  std::vector<WlWeight> out;
  out.reserve(kSeedCount);
  for (int i = 0; i < kSeedCount; i++) {
    float t = static_cast<float>(i) / static_cast<float>(kSeedCount - 1);
    out.push_back({ kLo + t * (kHi - kLo), 1.0f });
  }
  return out;
}

// Reset spectrum-modal statics on test teardown (called from ResetModalState via forward decl).
// These globals live in a later anonymous-namespace block than ResetModalState; both blocks are the
// same anonymous namespace in this TU, so the forward declaration below resolves here.
void ResetSpectrumModalStateGlobals() {
  g_spectrum_modal_open_pending = false;
  g_spectrum_edit_buf.clear();
}

}  // namespace

void OpenSpectrumModal(GuiState& state) {
  g_spectrum_modal_open_pending = true;
  // Seed the edit buffer with the current custom spectrum, if any; otherwise the preset seed.
  if (!state.sun.custom_spectrum.empty()) {
    g_spectrum_edit_buf = state.sun.custom_spectrum;
  } else {
    g_spectrum_edit_buf = BuildPresetSeed();
  }
}

void RenderSpectrumModal(GuiState& state) {
  if (g_spectrum_modal_open_pending) {
    ImGui::OpenPopup("Custom Spectrum##spectrum_modal");
    g_spectrum_modal_open_pending = false;
  }

  ImGui::SetNextWindowSize(ImVec2(480, 0), ImGuiCond_Appearing);
  // p_open (re-armed to true each frame) renders the title-bar × for style parity with the Edit Entry
  // modal; clicking it sets title_x_open false and is handled as a Cancel-equivalent below.
  bool title_x_open = true;
  if (!ImGui::BeginPopupModal("Custom Spectrum##spectrum_modal", &title_x_open, ImGuiWindowFlags_AlwaysAutoResize)) {
    return;
  }

  ImGui::TextUnformatted("Discrete wavelength/weight list");
  ImGui::Separator();

  // No in-modal "import preset": this editor does discrete wavelength editing only. Preset spectra
  // (D65 etc.) are chosen from the Sun-panel Spectrum combo, not here — that avoids a misleading
  // in-modal preset picker that couldn't reproduce a real illuminant SPD anyway (GUI can't include
  // core SPD headers per the API boundary). A fresh custom spectrum still opens seeded with a uniform
  // grid (BuildPresetSeed in OpenSpectrumModal) as an editable starting point.

  // Editable rows.
  int remove_idx = -1;
  for (int i = 0; i < static_cast<int>(g_spectrum_edit_buf.size()); i++) {
    ImGui::PushID(i);
    ImGui::SetNextItemWidth(80);
    ImGui::InputFloat("wl(nm)##wl", &g_spectrum_edit_buf[i].wavelength, 0.0f, 0.0f, "%.1f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80);
    ImGui::InputFloat("weight##wt", &g_spectrum_edit_buf[i].weight, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    PushDestructiveStyle();
    const bool remove_clicked = ImGui::SmallButton(ICON_FA_XMARK "##rm");
    PopDestructiveStyle();
    if (remove_clicked) {
      remove_idx = i;
    }
    ImGui::PopID();
  }
  if (remove_idx >= 0) {
    g_spectrum_edit_buf.erase(g_spectrum_edit_buf.begin() + remove_idx);
  }

  // Add row.
  const int cur_count = static_cast<int>(g_spectrum_edit_buf.size());
  const bool add_disabled = cur_count >= kSpectrumHardMax;
  if (add_disabled) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button(ICON_FA_PLUS " Add row")) {
    float next_wl = 550.0f;
    if (!g_spectrum_edit_buf.empty()) {
      next_wl = std::clamp(g_spectrum_edit_buf.back().wavelength + 40.0f, 380.0f, 780.0f);
    }
    g_spectrum_edit_buf.push_back({ next_wl, 1.0f });
  }
  if (add_disabled) {
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  ImGui::Text("%d / %d entries", cur_count, kSpectrumHardMax);

  if (cur_count > kSpectrumSoftWarnCount) {
    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f),
                       "Warning: %d > %d wavelengths — per-wavelength sampling becomes noisier.", cur_count,
                       kSpectrumSoftWarnCount);
  }

  ImGui::Separator();

  // Action row: OK / Cancel (dialog-terminating) on the left; Reset (a non-terminating edit action)
  // right-aligned on the same row so it reads as a distinct class and resists being mis-clicked as a
  // third confirm button.
  const bool ok_disabled = g_spectrum_edit_buf.empty();
  if (ok_disabled) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button(ICON_FA_CHECK " OK##spec_ok", ImVec2(80, 0))) {
    // Sanitize obviously-invalid manual input before it reaches the sim: clamp wavelength to the
    // visible band and weight to non-negative.
    for (auto& e : g_spectrum_edit_buf) {
      e.wavelength = std::clamp(e.wavelength, 380.0f, 780.0f);
      e.weight = std::max(e.weight, 0.0f);
    }
    // Single transactional commit of the custom-spectrum state: this is the ONLY site that sets
    // spectrum_index to kCustomSpectrumIndex, keeping the invariant (index==custom ⟹ buf non-empty)
    // — OK is gated on a non-empty buffer (ok_disabled) above.
    state.sun.custom_spectrum = g_spectrum_edit_buf;
    state.sun.spectrum_index = kCustomSpectrumIndex;
    state.MarkDirty();
    ImGui::CloseCurrentPopup();
  }
  if (ok_disabled) {
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Need at least one wavelength row before OK.");
    }
  }
  ImGui::SameLine();
  if (ImGui::Button(ICON_FA_XMARK " Cancel##spec_cancel", ImVec2(80, 0))) {
    // Nothing to restore: spectrum_index was never mutated on open (only OK commits it), so Cancel /
    // Escape / click-outside all naturally leave it at the prior valid preset. Just discard the buffer.
    ImGui::CloseCurrentPopup();
  }

  // Reset##spec_reset, right-aligned on the same row. Reset-to-default (not revert-to-open-time):
  // re-seed with the same uniform preset a freshly opened custom spectrum starts from. Only mutates
  // the edit buffer — OK stays the sole commit point (see g_spectrum_edit_buf contract above).
  // When the window is too narrow to right-align without overlapping Cancel, fall back to normal
  // same-line placement (this converges in one frame under AlwaysAutoResize).
  ImGui::SameLine();
  constexpr float kResetBtnW = 80.0f;
  const float reset_x = ImGui::GetWindowContentRegionMax().x - kResetBtnW;
  if (reset_x > ImGui::GetCursorPosX()) {
    ImGui::SetCursorPosX(reset_x);
  }
  if (ImGui::Button("Reset##spec_reset", ImVec2(kResetBtnW, 0))) {
    g_spectrum_edit_buf = BuildPresetSeed();
  }

  // Title-bar × = Cancel-equivalent: discard the edit buffer without committing. spectrum_index was
  // never mutated on open, so there is nothing to restore — same exit path as Cancel/Escape.
  if (!title_x_open) {
    ImGui::CloseCurrentPopup();
  }

  ImGui::EndPopup();
}

}  // namespace lumice::gui
