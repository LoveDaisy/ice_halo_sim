#include "gui/panels.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/axis_presets.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/destructive_style.hpp"
#include "gui/edit_modals.hpp"
#include "gui/field_editor_registry.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"  // FormatSummandText (non-degenerate SoP summary)
#include "gui/semantic_colors.hpp"
#include "gui/shape_scalar_domain.hpp"
#include "gui/slider_mapping.hpp"
#include "gui/theme.hpp"
#include "imgui.h"
#include "lumice.h"

namespace lumice::gui {

namespace {
// Render a slider with nonlinear scale mapping (sqrt/log/loglinear/linear).
// Must be called between PushItemWidth/PopItemWidth. Does NOT clamp — caller must clamp after.
// Note: `fmt` is only used for kLinear mode; nonlinear modes display a blank slider label.
static bool RenderNonlinearSlider(const char* slider_id, float* value, float min_val, float max_val, const char* fmt,
                                  SliderScale scale) {
  bool changed = false;
  // ImGuiSliderFlags_NoInput disables Ctrl+Click → InputText across all
  // SliderFloat / SliderInt calls to work around a macOS Screen Recording bug
  // that pollutes modifier state and traps sliders in text-edit mode. Users
  // enter values via the paired InputFloat on the right side of SliderWithInput.
  if (scale == SliderScale::kSqrt && min_val >= 0.0f) {
    float sqrt_val = std::sqrt(std::max(*value, 0.0f));
    float sqrt_max = std::sqrt(max_val);
    if (ImGui::SliderFloat(slider_id, &sqrt_val, 0.0f, sqrt_max, "", ImGuiSliderFlags_NoInput)) {
      // The *Snapped inverses here are load-bearing, not tidiness: at a slider stop the plain
      // round trip lands a few ULP short of the bound, which core's absolute-epsilon predicates
      // read as a different value entirely. See slider_mapping.hpp for the full account.
      *value = slider_mapping::SqrtNormToValueSnapped(sqrt_val, sqrt_max, max_val);
      changed = true;
    }
  } else if (scale == SliderScale::kLog && min_val > 0.0f) {
    float norm = slider_mapping::LogValueToNorm(*value, min_val, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "", ImGuiSliderFlags_NoInput)) {
      *value = slider_mapping::LogNormToValueSnapped(norm, min_val, max_val);
      changed = true;
    }
  } else if (scale == SliderScale::kLogLinear && min_val == 0.0f) {
    float norm = slider_mapping::LogLinearValueToNorm(*value, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "", ImGuiSliderFlags_NoInput)) {
      *value = slider_mapping::LogLinearNormToValueSnapped(norm, max_val);
      changed = true;
    }
  } else {
    // Linear: no *Snapped counterpart, and none is needed. The three branches above snap because
    // they round-trip the value through a transform and its inverse, which lands a few ULP short
    // of the bound at a stop. This branch hands `value` to ImGui untransformed, and ImGui itself
    // special-cases the extents -- ScaleValueFromRatioT (imgui_widgets.cpp) opens with
    // `if (t <= 0.0f ...) return v_min; if (t >= 1.0f) return v_max;`, so a stop yields the bound
    // bit-for-bit. Verified against the vendored copy rather than inferred from the other
    // branches, because it matters here: the Mean slider runs through this branch, and its value
    // feeds the same family of absolute-epsilon predicates (IsRollMeanAtMultipleOf30, and the
    // 90-degree latitude center inside IsFullSphereUniform) that a few ULP would flip exactly as
    // the Range slider's drift once flipped IsFullSphereUniform.
    changed |= ImGui::SliderFloat(slider_id, value, min_val, max_val, fmt, ImGuiSliderFlags_NoInput);
  }
  return changed;
}

// ---- Edit request state ----
EditRequest g_edit_request;

}  // namespace

// Infer axis orientation preset name from crystal config.
// Matching logic lives in axis_presets.hpp (shared with edit_modals.cpp; unit-tested).
std::string AxisPresetName(const CrystalConfig& c) {
  return AxisPresetLabel(ClassifyAxisPreset(c.zenith, c.azimuth, c.roll));
}

// ---- ID-pool sharing helpers (task-gui-linked-entries) ----

int CountEntriesSharing(const GuiState& state, int crystal_id, const std::optional<int>& filter_id) {
  int count = 0;
  for (const auto& layer : state.layers) {
    for (const auto& e : layer.entries) {
      if (e.crystal_id == crystal_id && e.filter_id == filter_id) {
        ++count;
      }
    }
  }
  return count;
}

bool UnlinkEntryFromPool(GuiState& state, int layer_idx, int entry_idx) {
  if (layer_idx < 0 || layer_idx >= static_cast<int>(state.layers.size())) {
    return false;
  }
  auto& entries = state.layers[layer_idx].entries;
  if (entry_idx < 0 || entry_idx >= static_cast<int>(entries.size())) {
    return false;
  }
  auto& e = entries[entry_idx];
  // Only fork pool slots that are currently shared (ref count ≥ 2 on the
  // <crystal_id, filter_id> pair). Already-unique entries are a no-op.
  if (CountEntriesSharing(state, e.crystal_id, e.filter_id) < 2) {
    return false;
  }
  CrystalConfig cloned_crystal = state.crystals[e.crystal_id];
  e.crystal_id = static_cast<int>(state.crystals.size());
  state.crystals.push_back(std::move(cloned_crystal));
  if (e.filter_id.has_value()) {
    FilterConfig cloned_filter = state.filters[*e.filter_id];
    e.filter_id = static_cast<int>(state.filters.size());
    state.filters.push_back(std::move(cloned_filter));
  }
  return true;
}

bool ApplyPickLink(GuiState& state, GuiState::EntryRef source, GuiState::EntryRef target) {
  if (source.layer_idx < 0 || source.layer_idx >= static_cast<int>(state.layers.size())) {
    return false;
  }
  if (target.layer_idx < 0 || target.layer_idx >= static_cast<int>(state.layers.size())) {
    return false;
  }
  const auto& source_entries = state.layers[source.layer_idx].entries;
  auto& target_entries = state.layers[target.layer_idx].entries;
  if (source.entry_idx < 0 || source.entry_idx >= static_cast<int>(source_entries.size())) {
    return false;
  }
  if (target.entry_idx < 0 || target.entry_idx >= static_cast<int>(target_entries.size())) {
    return false;
  }
  // Capture source ids before we mutate the target (in case source==target).
  const int src_cid = source_entries[source.entry_idx].crystal_id;
  const std::optional<int> src_fid = source_entries[source.entry_idx].filter_id;
  auto& t = target_entries[target.entry_idx];
  if (t.crystal_id == src_cid && t.filter_id == src_fid) {
    return false;  // already shared, no-op
  }
  t.crystal_id = src_cid;
  t.filter_id = src_fid;
  return true;
}

namespace {

// Append " <In|Out>[ <sym>]" suffix shared by every filter type's summary.
std::string FilterSummarySuffix(const FilterConfig& fc) {
  std::string suffix = (fc.action == 0) ? " In" : " Out";
  std::string sym;
  if (fc.sym_p) {
    sym += "P";
  }
  if (fc.sym_b) {
    sym += "B";
  }
  if (fc.sym_d) {
    sym += "D";
  }
  if (!sym.empty()) {
    suffix += " " + sym;
  }
  return suffix;
}

}  // namespace

// Generate filter summary text from filter config.
//
// Format dispatched by FilterParamVariant alternative:
//   - Raypath:    "<raypath_text or *> <In|Out>[ <sym>]"        (e.g. "3-1-5 In PBD")
//   - EntryExit:  "EE:<entry>-><exit> <In|Out>[ <sym>]"
//
// 12-character truncation on the raypath body keeps the card text inside the row
// it shares with the Edit button. Pinned by
// `SceneCommitChain.ALongRaypathIsCutToTwelveCharactersOnTheCard` in
// test/composition-correctness/gui/test_scene_commit_chain.cpp. Other types
// emit short prefixes that fit comfortably without truncation; if they ever
// need truncation, add it per-type.
//
// Externally linked (declared in panels.hpp) so unit / GUI tests can assert
// the rendered summary string directly. Implementation depends on
// FilterSummarySuffix above (file-internal helper) — this is fine even
// though the helper sits inside an anonymous namespace closed just above:
// both live in the same TU.
std::string FilterSummary(const std::optional<FilterConfig>& f) {
  if (!f.has_value()) {
    return "None";
  }
  const auto& fc = f.value();

  std::string body;
  if (fc.IsDegenerateSingleFactor()) {
    body = std::visit(
        [](const auto& p) -> std::string {
          using T = std::decay_t<decltype(p)>;
          if constexpr (std::is_same_v<T, RaypathParams>) {
            if (p.raypath_text.empty()) {
              return "*";
            }
            if (p.raypath_text.size() > 12) {
              return p.raypath_text.substr(0, 12) + "...";
            }
            return p.raypath_text;
          } else if constexpr (std::is_same_v<T, EntryExitParams>) {
            // Format each end as "*" (wildcard / empty), the raw text (single
            // value), or "{a,b,...}" (multi-value list). Length suffix encodes
            // the four mode choices so the summary roundtrips with the edit
            // modal's dropdown.
            auto format_side = [](const std::string& t) -> std::string {
              if (t.empty()) {
                return "*";
              }
              if (t.find(',') == std::string::npos) {
                return t;
              }
              return std::string("{") + t + "}";
            };
            std::string body = std::string("EE:") + format_side(p.entry_text) + "-" + format_side(p.exit_text);
            switch (p.length_mode) {
              case 1:
                body += " L=" + std::to_string(p.min_len);
                break;
              case 2:
                body += " L<=" + std::to_string(p.max_len);
                break;
              case 3:
                body += " L=[" + std::to_string(p.min_len) + "," + std::to_string(p.max_len) + "]";
                break;
              case 0:
              default:
                break;
            }
            return body;
          } else {
            return "*";
          }
        },
        fc.DegenerateFactor());
  } else {
    // Non-degenerate sum-of-products (multiple OR rows and/or AND factors).
    // DegenerateFactor() would assert/UB here, so summarize without it: show the
    // first row's canonical text (truncated) + "(+N more)" when more rows exist.
    // This is the minimal non-crash display (333.3); the full multi-summand
    // editor UI is 333.4.
    std::string first = fc.param.empty() ? std::string{} : FormatSummandText(fc.param[0].factors);
    if (first.empty()) {
      first = "*";
    }
    if (first.size() > 12) {
      first = first.substr(0, 12) + "...";
    }
    body = first;
    if (fc.param.size() > 1) {
      body += " (+" + std::to_string(fc.param.size() - 1) + " more)";
    }
  }

  return body + FilterSummarySuffix(fc);
}

namespace {

// Card layout: height is driven by ImGuiChildFlags_AutoResizeY so font/theme
// changes adapt automatically (kThumbnailSize lives in gui_constants.hpp).
// Destructive-button palette (delete/remove) lives in gui/destructive_style.hpp
// after task-color-window-controls-polish promoted the previously file-local copy.

}  // namespace


// Compute slider width and prepare IDs for the [slider] [input] Label layout.
// Writes slider_id and input_id buffers, returns the computed slider width.
// When `reserve_label_col` is false the trailing text label is omitted (table-cell
// mode): the width no longer reserves kLabelColWidth nor its extra SameLine spacing,
// so the [slider][input] pair fills the whole cell (GetContentRegionAvail() ==
// column width inside a BeginTable cell).
static float PrepareSliderLayout(const char* label, char* display_label_out, size_t display_buf_size, char* slider_id,
                                 size_t slider_id_size, char* input_id, size_t input_id_size,
                                 bool reserve_label_col = true) {
  // Strip ImGui ID suffix (e.g. "Azimuth##view" → display "Azimuth")
  const char* hash_pos = strstr(label, "##");
  if (hash_pos) {
    auto len = static_cast<size_t>(hash_pos - label);
    if (len >= display_buf_size)
      len = display_buf_size - 1;
    memcpy(display_label_out, label, len);
    display_label_out[len] = '\0';
  } else {
    snprintf(display_label_out, display_buf_size, "%s", label);
  }

  snprintf(slider_id, slider_id_size, "##%s_slider", label);
  snprintf(input_id, input_id_size, "##%s_input", label);

  float spacing = ImGui::GetStyle().ItemSpacing.x;
  float avail_w = ImGui::GetContentRegionAvail().x;
  // With the trailing label: subtract kLabelColWidth + 2 SameLine spacings
  // (slider→input, input→label). Without it: only the slider→input spacing.
  float slider_w =
      reserve_label_col ? (avail_w - kInputWidth - kLabelColWidth - spacing * 2) : (avail_w - kInputWidth - spacing);
  if (slider_w < 40.0f)
    slider_w = 40.0f;
  return slider_w;
}

// Render the label text after slider + input.
static void FinishSliderLayout(const char* display_label) {
  ImGui::SameLine();
  ImGui::TextUnformatted(display_label);
}

bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt, SliderScale scale,
                     bool trailing_label, bool* committed, bool* active) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id), trailing_label);

  const float old_value = *value;

  ImGui::PushItemWidth(slider_w);
  RenderNonlinearSlider(slider_id, value, min_val, max_val, fmt, scale);
  const bool slider_committed = ImGui::IsItemDeactivatedAfterEdit();
  const bool slider_active = ImGui::IsItemActive();
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  ImGui::InputFloat(input_id, value, 0, 0, fmt);
  const bool input_committed = ImGui::IsItemDeactivatedAfterEdit();
  const bool input_active = ImGui::IsItemActive();
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  if (committed) {
    *committed = slider_committed || input_committed;
  }
  if (active) {
    *active = slider_active || input_active;
  }

  if (trailing_label) {
    FinishSliderLayout(display_buf);
  }
  return *value != old_value;
}

bool DragFloatField(const char* label, float* value, float min_val, float max_val, const char* fmt, SliderScale scale) {
  char drag_id[80];
  snprintf(drag_id, sizeof(drag_id), "##%s", label);

  ImGuiSliderFlags flags = ImGuiSliderFlags_AlwaysClamp;
  if (scale != SliderScale::kLinear) {
    flags |= ImGuiSliderFlags_Logarithmic;
  }
  // Full domain per kDragTrackReferenceWidth pixels of drag, in both modes: ImGui divides a
  // logarithmic drag's delta by (max - min) before applying it, which cancels the numerator here.
  const float speed = (max_val - min_val) / kDragTrackReferenceWidth;
  const float old_value = *value;
  ImGui::DragFloat(drag_id, value, speed, min_val, max_val, fmt, flags);

  // ImGui does not give DragFloat a resize cursor on its own (unlike a window border or a table
  // column boundary), so the affordance has to be set explicitly for hover and drag alike.
  if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
  }

  // Unconditional, and NOT redundant with ImGuiSliderFlags_AlwaysClamp: that flag constrains the
  // values the widget itself produces, and deliberately leaves a value it was handed out of range
  // alone. SliderWithInput ends with exactly this line, and things depend on it — a .lmc written by
  // hand, or a lens switch that narrows a bound under a value that was legal a frame ago (the
  // globe's elevation limit), reach the field without going through any control, and the control is
  // what pulls them back in. Without it the page renders an out-of-domain value as if it were fine.
  *value = std::clamp(*value, min_val, max_val);
  // Same return contract as SliderWithInput: "the value is not what it was", clamp included, since
  // a clamp is a change the caller has to commit like any other.
  return *value != old_value;
}

// SliderInt + InputInt + label text, same layout as SliderWithInput.
// Returns true if value changed.
bool SliderIntWithInput(const char* label, int* value, int min_val, int max_val, bool trailing_label, bool* committed,
                        bool* active) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id), trailing_label);

  const int old_value = *value;

  ImGui::PushItemWidth(slider_w);
  ImGui::SliderInt(slider_id, value, min_val, max_val, "%d", ImGuiSliderFlags_NoInput);
  const bool slider_committed = ImGui::IsItemDeactivatedAfterEdit();
  const bool slider_active = ImGui::IsItemActive();
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  ImGui::InputInt(input_id, value, 0, 0);
  const bool input_committed = ImGui::IsItemDeactivatedAfterEdit();
  const bool input_active = ImGui::IsItemActive();
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  if (committed) {
    *committed = slider_committed || input_committed;
  }
  if (active) {
    *active = slider_active || input_active;
  }

  FinishSliderLayout(display_buf);
  return *value != old_value;
}

// ---- Shared combo-popup fix (see panels.hpp) ----
//
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
// RenderFilterModal, all in edit_modals.cpp) MUST be preceded by a call to
// this helper. Forgetting the call has no compile-time error and silently
// regresses to the original bug — only visible in detached-modal state which
// CI cannot reproduce (hidden GLFW window pins GetMainViewport()->Pos to (0,0)).
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

// ---- Axis distribution controls (shared with edit modals) ----
//
// CALLER CONTRACT: when invoked from a context where the parent window may
// become a detached OS viewport (currently: Edit Entry modal in multi-viewport
// mode), the caller must precede this call with `SetNextComboPopupTopMost()`.
// Without that, the internal `##dist` Combo's popup defaults to NSWindow
// layer=0 and gets rendered behind the modal at layer=3.
//
// IMPLEMENTATION CONTRACT: this function MUST NOT introduce a `Begin` /
// `BeginChild` call before the Combo at line 297 — doing so would consume
// the caller's queued NextWindowData (WindowClass) prematurely. If layout
// wrapping is ever required, the SetNextComboPopupTopMost call must be moved
// inside RenderAxisDist (after any wrapping Begin/BeginChild, immediately
// before the Combo).
bool RenderAxisDist(const char* label, AxisDist& axis, float mean_min, float mean_max) {
  bool changed = false;
  ImGui::PushID(label);
  ImGui::Text("%s", label);
  ImGui::SameLine(100);

  int dist_type = static_cast<int>(axis.type);
  auto prev_type = axis.type;

  if (ImGui::Combo("##dist", &dist_type, kAxisDistTypeComboItems)) {
    axis.type = static_cast<AxisDistType>(dist_type);
    changed = true;
  }

  // Clamp std to new range when switching type.
  if (axis.type != prev_type) {
    float max_std = 0.0f;
    switch (axis.type) {
      case AxisDistType::kGauss:
        max_std = 180.0f;
        break;
      case AxisDistType::kUniform:
        max_std = 360.0f;
        break;
      case AxisDistType::kZigzag:
        max_std = 90.0f;
        break;
      case AxisDistType::kLaplacian:
        max_std = 90.0f;
        break;
      case AxisDistType::kGaussLegacy:
        max_std = 180.0f;
        break;
      default:
        max_std = 180.0f;
        break;
    }
    axis.std = std::min(axis.std, max_std);
  }

  changed |= SliderWithInput("Mean", &axis.mean, mean_min, mean_max);

  switch (axis.type) {
    case AxisDistType::kGauss:
    case AxisDistType::kGaussLegacy:
      changed |= SliderWithInput("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kUniform:
      changed |= SliderWithInput("Range", &axis.std, 0.0f, 360.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kZigzag:
      changed |= SliderWithInput("Amplitude", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kLaplacian:
      changed |= SliderWithInput("Scale", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt);
      break;
    default:
      changed |= SliderWithInput("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt);
      break;
  }

  ImGui::PopID();
  return changed;
}


void ShapeTableParamLabel(const char* label) {
  const char* hash_pos = strstr(label, "##");
  if (hash_pos) {
    ImGui::TextUnformatted(label, hash_pos);
  } else {
    ImGui::TextUnformatted(label);
  }
}


// ---- Sync column: shape-scalar sync groups (the Sync cell of a shape-table row) ----
namespace {

// Row names indexed by LUMICE_SHAPE_SCALAR_*, used in the popup's membership lists. The order is
// the SLOT index space, not CrystalConfig's field order: UPPER_H is slot 1 and PRISM_H slot 2 (see
// the SLOT-ORDER TRAP note in gui_state.hpp), so this table reads "Upper H" before "Prism H" even
// though the modal draws Prism H first. The strings match what ShapeTableParamLabel prints for the
// corresponding rows, because a membership list naming rows the user cannot find is worse than none.
const char* const kShapeScalarLabels[LUMICE_SHAPE_SCALAR_COUNT] = {
  "Height", "Upper H", "Prism H", "Lower H", "Face 3", "Face 4", "Face 5", "Face 6", "Face 7", "Face 8",
};

// Group colors are DERIVED from the group number, never stored (D7): a stored color would have to
// round-trip through JSON, the C API and the .lmc format for a purely cosmetic property. Low
// saturation so a filled cell stays readable behind the group number and does not fight the table's
// own row striping. Groups beyond the table length wrap — a 7th group reusing group 1's color is a
// cosmetic collision, not an error state, and the number in the swatch remains unambiguous (this is
// also why the number is drawn at all: color alone is not accessible).
const ImVec4 kSyncGroupPalette[] = {
  ImVec4(0.36f, 0.55f, 0.75f, 1.00f),  // blue
  ImVec4(0.75f, 0.55f, 0.36f, 1.00f),  // amber
  ImVec4(0.45f, 0.68f, 0.48f, 1.00f),  // green
  ImVec4(0.70f, 0.48f, 0.66f, 1.00f),  // orchid
  ImVec4(0.42f, 0.66f, 0.70f, 1.00f),  // teal
  ImVec4(0.76f, 0.52f, 0.48f, 1.00f),  // terracotta
};
constexpr int kSyncGroupPaletteCount = static_cast<int>(sizeof(kSyncGroupPalette) / sizeof(kSyncGroupPalette[0]));

// Swatch fill for a non-zero group. Group 0 (independent) has no color — it renders as an outline.
ImVec4 SyncGroupColor(int group) {
  return kSyncGroupPalette[(group - 1) % kSyncGroupPaletteCount];
}

// Is this scalar drawn as a row for `type`? Mirrors RenderCrystalModal's type branch (Prism draws
// Height; Pyramid draws Prism/Upper/Lower H; the six faces are drawn by both) — the same condition,
// promoted from an implicit `if` in the renderer to a predicate the Sync widget can also ask.
//
// The question "does this shape-scalar slot physically exist on this crystal type" is core's to
// answer, and this forwards it verbatim: LUMICE_IsShapeScalarApplicable reads the one slot table in
// crystal_config.cpp that Canonicalize and Normalize also read. The GUI holds no copy of it. That
// identity — not an agreement between two hand-kept tables — is what lets the MIRROR side of the Sync
// widget (leader election, propagation, membership listing) reproduce core's rule rather than
// approximate it; see FindGroupLeaderSlot.
//
// A group number on a scalar belonging to the OTHER crystal type is therefore not visible, and lies
// dormant across a type switch — not cleared, not shown, not joined. That matches core, which zeroes
// exactly those slots in CanonicalizeSyncGroups, and it is the smallest GUI-side behavior that
// neither invents a cross-type editing model nor destroys data the user may switch back to.
bool IsShapeScalarVisible(CrystalType type, int slot) {
  const auto kind = (type == CrystalType::kPrism) ? LUMICE_CRYSTAL_PRISM : LUMICE_CRYSTAL_PYRAMID;
  return LUMICE_IsShapeScalarApplicable(kind, slot) != 0;
}

// Can the user CREATE a sync membership on this scalar? Strictly narrower than IsShapeScalarVisible
// (syncable ⊂ visible): sharing one drawn value only means something between scalars that are
// commensurable — same unit, same semantics. Counting each type's randomizable scalars leaves exactly
// two rows with nothing to pair with:
//   - prism  HEIGHT  — the only axial length; the six face distances are dimensionless ratios.
//   - pyramid PRISM_H — a c/a-axis length; upper_h/lower_h are 0..1 relative heights (those two ARE
//     each other's pair, and are exactly the use case the generalized sync group exists for).
// So a Sync control on those two rows would offer an operation that can never be meaningful.
//
// SCOPE — this predicate governs the CREATION side only: whether the row draws a Sync swatch, and
// hence whether the user can open the picker on it to join a group or start a new one. It must NOT
// reach the side of the widget that REPORTS existing state (leader election, propagation, membership
// listing): core is the sole authority on sync semantics and the GUI mirrors it verbatim, so a
// GUI-only affordance narrowing that also changed what the GUI reports would make the table show a
// distribution the simulation does not use. That is exactly the divergence this narrowing once
// caused, and the separation is now load-bearing rather than advisory — the mirror side is written
// against IsShapeScalarVisible and does not read this predicate at all.
//
// Barring these two rows is therefore a UI affordance only: storage still permits such a group
// (lumice.h runs no commensurability check), a hand-authored config carrying one keeps it, and the
// GUI reports it faithfully — it simply offers no way to build one. NextUnusedSyncGroup deliberately
// does NOT use this predicate either: new ids must not collide with a dormant one.
bool IsShapeScalarSyncable(CrystalType type, int slot) {
  return IsShapeScalarVisible(type, slot) && slot != LUMICE_SHAPE_SCALAR_HEIGHT && slot != LUMICE_SHAPE_SCALAR_PRISM_H;
}

// The slot whose value the group carries: the lowest-indexed applicable member. This is not a GUI
// approximation of the core rule — it IS the core rule (lumice.h LUMICE_CrystalParam::sync_group:
// "the group's first applicable member (lowest LUMICE_SHAPE_SCALAR_* index) consumes the RNG and
// owns the distribution"), evaluated over the same applicable set: core's NormalizeSyncGroupsImpl
// takes the argmin over the slots its slot table admits, and IsShapeScalarVisible queries that very
// table through the C API (see its comment). So the value the user sees snap in on join is the value
// core will actually draw
// with. `exclude_slot` keeps a row from electing itself as its own leader while it is joining.
//
// The scan MUST stay scoped by IsShapeScalarVisible and never by IsShapeScalarSyncable. Electing a
// leader over the narrower set answers a different question — "which member can the user edit" — and
// for a hand-authored group such as height + face_0 it returns face_0 where core returns height. The
// table would then display, and hand out on join, a distribution core overwrites on commit: the row
// silently reverts and the rendered halo comes from a value never shown. A GUI predicate may
// constrain what the user can CREATE; it may not change what the GUI REPORTS about state that
// already exists. Groups containing a row with no Sync control are unreachable to the picker but
// perfectly legal in storage, and this function is how they get reported truthfully.
// Returns -1 when the group has no other member (or group == 0).
int FindGroupLeaderSlot(const CrystalConfig& cr, int group, int exclude_slot) {
  if (group == 0) {
    return -1;
  }
  for (int s = 0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
    if (s == exclude_slot || !IsShapeScalarVisible(cr.type, s)) {
      continue;
    }
    if (ShapeScalarAt(cr, s).sync_group == group) {
      return s;
    }
  }
  return -1;
}

// Group id for "+ New group": one past the largest id anywhere in the crystal. Scans ALL ten slots,
// including ones the current type does not draw, so a new group cannot collide with a dormant group
// belonging to the other crystal type. Ids abandoned along the way are not recycled — core
// renumbers on parse anyway (canonicalization), and reusing a number the user just emptied would
// re-color a row they had visually separated.
int NextUnusedSyncGroup(const CrystalConfig& cr) {
  int max_group = 0;
  for (int s = 0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
    max_group = std::max(max_group, ShapeScalarAt(cr, s).sync_group);
  }
  return max_group + 1;
}

// Copy `slot`'s distribution onto the rest of its group (type, center and spread — NOT the group id
// itself). Called after an edit to any row that is in a group: a subordinate row stays editable and
// writing through it updates the whole group, which the owner picked over greying subordinates out.
// Gated at BOTH ends by IsShapeScalarVisible — the source row as well as the destinations — because
// core's normalization writes the leader's distribution over EVERY applicable member of the group,
// with no notion of a member that is exempt. Propagating over the narrower syncable set would leave
// the group internally disagreeing in exactly the members core then rewrites, so a commit would
// silently move a row the user did not touch (and log the leader-override warning). Writing the whole
// group instead makes core's normalize a no-op: it finds the members already equal, overrides
// nothing, warns about nothing, and the GUI and the simulation hold the same state.
//
// The rows with no Sync control are thus still group writers and still written to. That is the
// mirror side of the widget, and it is not in tension with the affordance narrowing: what a row
// cannot do is JOIN a group (IsShapeScalarSyncable, at the swatch). Once it is in one — which only a
// hand-authored config can arrange — it is a member like any other, and the Param-name tint in
// RenderShapeDistTableRow is what keeps the value movement from being unexplained on screen.
void PropagateToSyncGroup(CrystalConfig& cr, int slot) {
  const ShapeDist src = ShapeScalarAt(cr, slot);
  if (src.sync_group == 0 || !IsShapeScalarVisible(cr.type, slot)) {
    return;
  }
  for (int s = 0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
    if (s == slot || !IsShapeScalarVisible(cr.type, s)) {
      continue;
    }
    ShapeDist& dst = ShapeScalarAt(cr, s);
    if (dst.sync_group != src.sync_group) {
      continue;
    }
    dst.type = src.type;
    dst.center = src.center;
    dst.spread = src.spread;
  }
}

// Join `slot` to `group`, snapshotting the group leader's distribution into it. Joining is visibly
// destructive of the row's own value — that self-explains the semantics better than a confirmation
// dialog would, and Cancel on the modal is the undo. A group with no other member (a fresh "+ New
// group") has no leader, so the row keeps its value and simply becomes the group's first member.
void JoinSyncGroup(CrystalConfig& cr, int slot, int group) {
  ShapeDist& dist = ShapeScalarAt(cr, slot);
  dist.sync_group = group;
  const int leader = FindGroupLeaderSlot(cr, group, slot);
  if (leader < 0) {
    return;
  }
  const ShapeDist& src = ShapeScalarAt(cr, leader);
  dist.type = src.type;
  dist.center = src.center;
  dist.spread = src.spread;
}

// Comma-separated names of `group`'s members as the current type draws them, empty if the group has
// none. Scoped by IsShapeScalarVisible, so a row that is drawn but carries no Sync control (a prism
// Height in a hand-authored group) IS listed: the popup item is a description of the group the user
// is about to join, and omitting a member whose value the group carries would understate what
// joining costs. A member belonging to the other crystal type stays absent — it is not drawn at all,
// and core does not count it either (CanonicalizeSyncGroups zeroes those slots).
std::string SyncGroupMemberList(const CrystalConfig& cr, int group) {
  std::string members;
  for (int s = 0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
    if (!IsShapeScalarVisible(cr.type, s) || ShapeScalarAt(cr, s).sync_group != group) {
      continue;
    }
    if (!members.empty()) {
      members += ", ";
    }
    members += kShapeScalarLabels[s];
  }
  return members;
}

// Draw the group swatch: filled + numbered for a group, hollow outline for independent. Purely
// visual — the click is owned by the caller's Button, which this paints over.
void DrawSyncSwatch(const ImVec2& p_min, float side, int group) {
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  if (group == 0) {
    // Independent: a hollow outline. Distinct at a glance from any filled group cell, and it still
    // reads as a clickable target rather than an empty cell.
    draw_list->AddRect(p_min, ImVec2(p_min.x + side, p_min.y + side), ImGui::GetColorU32(ImGuiCol_TextDisabled), 2.0f);
    return;
  }
  // Number centered in the swatch, in whichever of black/white contrasts with the fill. Derived
  // from the fill's luminance rather than fixed, so a later palette edit cannot silently produce
  // an unreadable cell.
  const ImVec4 fill = SyncGroupColor(group);
  const float luma = 0.299f * fill.x + 0.587f * fill.y + 0.114f * fill.z;
  const ImU32 text_col = luma > 0.55f ? IM_COL32(0, 0, 0, 255) : IM_COL32(255, 255, 255, 255);
  char num[16];
  snprintf(num, sizeof(num), "%d", group);
  const ImVec2 text_size = ImGui::CalcTextSize(num);
  draw_list->AddText(ImVec2(p_min.x + (side - text_size.x) * 0.5f, p_min.y + (side - text_size.y) * 0.5f), text_col,
                     num);
}

// Body of the group picker popup (caller owns Begin/EndPopup). Returns true if the row's group
// changed. Item ids are anchored with "###" so they stay put as membership text changes.
bool RenderSyncPopupItems(CrystalConfig& cr, int slot) {
  ShapeDist& dist = ShapeScalarAt(cr, slot);
  bool changed = false;

  // "None" first: leaving a group is selecting None, because a group is an equivalence relation
  // and not an object — there is nothing to delete, and the last member leaving is what makes a
  // group disappear (D7). Leaving does NOT restore the pre-join value: no shadow copy is kept,
  // and the modal's Cancel is the undo path.
  if (ImGui::Selectable("None###sync_none") && dist.sync_group != 0) {
    dist.sync_group = 0;
    changed = true;
  }

  // Existing groups, with their membership spelled out: the decision the user is making is "join
  // the group that has Face 3 and Face 5 in it", not "join group 1".
  bool listed_any = false;
  const int group_upper_bound = NextUnusedSyncGroup(cr);  // exclusive: one past the largest id in use
  for (int group = 1; group < group_upper_bound; ++group) {
    const std::string members = SyncGroupMemberList(cr, group);
    if (members.empty()) {
      continue;
    }
    listed_any = true;
    // "###sync_group_N" anchors the id to the group number alone: the visible part carries the
    // membership list, which changes as rows join and leave, and an id derived from it would change
    // with it.
    char item[192];
    snprintf(item, sizeof(item), "%d  (%s)###sync_group_%d", group, members.c_str(), group);
    // Re-selecting the row's current group is a no-op, not a re-snapshot: the row IS the value the
    // group would hand it back, and treating it as a join would be a needless self-overwrite.
    if (ImGui::Selectable(item) && dist.sync_group != group) {
      JoinSyncGroup(cr, slot, group);
      changed = true;
    }
  }
  if (listed_any) {
    ImGui::Separator();
  }

  // An explicit item rather than a side effect of clicking through the swatch: "make a new group"
  // is a distinct intent from "pick an existing one" (D7 settled the popup as the single path).
  if (ImGui::Selectable("+  New group###sync_new")) {
    JoinSyncGroup(cr, slot, NextUnusedSyncGroup(cr));
    changed = true;
  }
  return changed;
}

// Sync cell: a group-colored square button that opens the group picker. Returns true if the row's
// group (and hence, via the snapshot, possibly its value) changed.
//
// The button's visible label is EMPTY and its id is "##sync_<label>" — the group number is drawn
// on top by hand rather than being part of the label. That is not decoration: ImGui hashes the
// whole id string (only "###" restarts the hash — imgui.cpp ImHashStr), so a number in the label
// would make the widget's id change every time the user regrouped, and every test path addressing
// it would drift out from under them. Same reason the swatch is a Button and not a ColorButton:
// Button registers with the test engine, so the cell is reachable by a real click.
bool RenderSyncCell(const char* label, CrystalConfig& cr, int slot) {
  const int group = ShapeScalarAt(cr, slot).sync_group;

  char btn_id[96];
  snprintf(btn_id, sizeof(btn_id), "##sync_%s", label);

  // Square, exactly one frame tall: the swatch must not be what sets the row height, or the modal's
  // fixed-height content pane (edit_modals.cpp kModalContentHeight, budgeted in rows) would need a
  // scrollbar once Face Distance is expanded.
  const float side = ImGui::GetFrameHeight();
  const ImVec2 p_min = ImGui::GetCursorScreenPos();

  const ImVec4 fill = group != 0 ? SyncGroupColor(group) : ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  ImGui::PushStyleColor(ImGuiCol_Button, fill);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(fill.x, fill.y, fill.z, group != 0 ? 0.80f : 0.25f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(fill.x, fill.y, fill.z, group != 0 ? 0.60f : 0.40f));
  const bool clicked = ImGui::Button(btn_id, ImVec2(side, side));
  ImGui::PopStyleColor(3);
  DrawSyncSwatch(p_min, side, group);

  char popup_id[96];
  snprintf(popup_id, sizeof(popup_id), "##sync_popup_%s", label);
  if (clicked) {
    ImGui::OpenPopup(popup_id);
  }
  bool changed = false;
  if (ImGui::BeginPopup(popup_id)) {
    changed = RenderSyncPopupItems(cr, slot);
    ImGui::EndPopup();
  }
  return changed;
}

}  // namespace

bool RenderShapeDistTableRow(const char* label, CrystalConfig& cr, int slot) {
  const ShapeScalarDomain& domain = ShapeScalarDomainFor(slot);  // single domain authority
  const float center_min = domain.min_value;
  const float center_max = domain.max_value;
  const char* center_fmt = domain.fmt;
  const SliderScale center_scale = domain.scale;
  ShapeDist& dist = ShapeScalarAt(cr, slot);  // single mapping authority (gui_state.hpp)
  bool changed = false;
  // No PushID wrapper: the center slider keeps its original `label`-derived id (so existing
  // ItemInputValue paths — e.g. "##Height##modal_cr_input" — stay valid), and each extra widget
  // gets a `label`-suffixed unique id so multiple rows do not collide and GUI tests can target each.
  ImGui::TableNextRow();

  // The two sides of the Sync widget, kept as separate variables on purpose (see the predicates'
  // comments): `syncable` is the CREATION affordance and gates the Sync cell alone; `visible` is the
  // MIRROR scope core's rules are evaluated over, and gates the Param-name tint below.
  const bool syncable = IsShapeScalarSyncable(cr.type, slot);
  const bool visible = IsShapeScalarVisible(cr.type, slot);

  // Col 0 — Parameter name. A grouped row also gets its name cell tinted in the group's color (at
  // low alpha, so the text stays readable): together with the filled Sync swatch this makes group
  // membership pre-attentive — the eye finds the rows that move together without reading numbers.
  // CellBg draws above the table's RowBg striping, so the two do not fight.
  //
  // Tinting on `visible`, not `syncable`, is what keeps that promise complete: a row with no Sync
  // control can still be a group member (hand-authored), and PropagateToSyncGroup does move its
  // value with the group. Tinting on the narrower predicate would leave that one row changing with
  // nothing on screen to attribute it to — the tint IS the visible cause. The cell has no swatch, so
  // it still reads as "in this group, but not editable from here".
  //
  // `visible` is true at every present call site (edit_modals.cpp only emits a row inside the type
  // branch that draws it), so the guard is currently a tautology. It is spelled out anyway: this is
  // the mirror scope, naming it keeps the two sides of the widget symmetrical at the point of use,
  // and a caller that ever emits rows unconditionally does not silently start tinting dormant groups.
  ImGui::TableNextColumn();
  if (visible && dist.sync_group != 0) {
    const ImVec4 tint = SyncGroupColor(dist.sync_group);
    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, ImGui::GetColorU32(ImVec4(tint.x, tint.y, tint.z, 0.28f)));
  }
  ShapeTableParamLabel(label);

  // Col 1 — center value: slider + input, filling the (stretch) Value column. trailing_label=false
  // because the name already occupies Col 0.
  ImGui::TableNextColumn();
  changed |= SliderWithInput(label, &dist.center, center_min, center_max, center_fmt, center_scale, false);

  // Col 2 — sync group swatch + picker popup. Ahead of Rand/Spread because sync is not a property of
  // randomization: it constrains the row's final value and is equally usable with Rand off. Left
  // BLANK for a non-syncable scalar, the same way the wedge rows leave their inapplicable columns
  // blank (edit_modals.cpp RenderWedgeTableRow) — the column is still advanced so the grid stays a
  // rectangle.
  ImGui::TableNextColumn();
  const bool group_changed = syncable && RenderSyncCell(label, cr, slot);

  // Col 3 — Randomize checkbox. The GUI is uniform-only, so enabling sets type=Uniform; NO_RANDOM is
  // expressed via this checkbox being off. Text-less (the header names the column); the ## suffix
  // embeds `label` so the id is unique.
  ImGui::TableNextColumn();
  char ck_id[96];
  snprintf(ck_id, sizeof(ck_id), "##rnd_%s", label);
  bool randomize = dist.type != ShapeDistType::kNoRandom;
  if (Checkbox(ck_id, &randomize)) {
    if (randomize) {
      dist.type = ShapeDistType::kUniform;                          // GUI edits uniform only
      dist.spread = kShapeDistDefaultSpreadFraction * dist.center;  // default spread heuristic
    } else {
      dist.type = ShapeDistType::kNoRandom;
      dist.spread = 0.0f;  // spread is meaningless for NO_RANDOM; zero it so operator==/round-trip stay clean
    }
    changed = true;
  }

  // Col 4 — spread input. Rendered even when not randomized, but wrapped in BeginDisabled so it greys
  // out ("available to enable") and cannot be interacted with while dist is NO_RANDOM (spread==0),
  // preserving the disabled-state semantics. A plain number input (not a slider — saves horizontal
  // space in the narrow vertical layout). Same units as center; clamped to [0, center_max].
  ImGui::BeginDisabled(!randomize);
  ImGui::TableNextColumn();
  char sp_id[96];
  snprintf(sp_id, sizeof(sp_id), "##spread_%s", label);
  ImGui::SetNextItemWidth(-FLT_MIN);
  if (ImGui::InputFloat(sp_id, &dist.spread, 0, 0, center_fmt)) {
    dist.spread = std::clamp(dist.spread, 0.0f, center_max);
    changed = true;
  }

  ImGui::EndDisabled();

  // A value edit on a row that is in a group writes through to the whole group. Evaluated after ALL
  // the cells, hence after the Sync cell, so an edit made in the same frame the row joined a group
  // propagates the joined value rather than the discarded one — the reason this lives at the end of
  // the function rather than next to the Value column, and the reason moving the Sync cell earlier in
  // the column order does not disturb it. Joining itself does not go through here: that path
  // snapshots FROM the leader (JoinSyncGroup), it does not push TO the group.
  if (changed) {
    PropagateToSyncGroup(cr, slot);
  }
  changed |= group_changed;

  // Make kShapeTableColumnCount load-bearing at the consumption site, not just a comment: assert the
  // row consumed exactly that many columns. Bumping the constant + adding a TableSetupColumn without
  // updating this helper then trips here instead of silently misaligning the grid (the drift mode the
  // constant exists to stop; ImGui does not assert on a column-count mismatch on its own).
  IM_ASSERT(ImGui::TableGetColumnIndex() + 1 == kShapeTableColumnCount &&
            "RenderShapeDistTableRow must advance exactly kShapeTableColumnCount columns");
  return changed;
}


// ---- Selection and edit accessors ----

const EditRequest& GetEditRequest() {
  return g_edit_request;
}

void ResetEditRequest() {
  g_edit_request = EditRequest{};
}

void ResetPendingDeleteState() {
  g_edit_request = EditRequest{};
}


// ========== Entry Card ==========

bool RenderEntryCard(GuiState& state, int layer_idx, int entry_idx) {
  auto& entry = state.layers[layer_idx].entries[entry_idx];

  // ---- Pick-mode: detect whether we're targeting this card ----
  // When state.pick_link_source is set, every card becomes a click-target for
  // completing the eyedropper share. Source card and already-shared cards are
  // disabled (would be a no-op). The actual click handling happens at the
  // bottom of RenderEntryCard via an InvisibleButton overlay that covers the
  // card body.
  const bool pick_active = state.pick_link_source.has_value();
  bool pick_target_disabled = false;
  if (pick_active) {
    const auto& src_ref = *state.pick_link_source;
    if (src_ref.layer_idx == layer_idx && src_ref.entry_idx == entry_idx) {
      pick_target_disabled = true;  // can't link to self
    } else if (src_ref.layer_idx >= 0 && src_ref.layer_idx < static_cast<int>(state.layers.size()) &&
               src_ref.entry_idx >= 0 &&
               src_ref.entry_idx < static_cast<int>(state.layers[src_ref.layer_idx].entries.size())) {
      const auto& src_entry = state.layers[src_ref.layer_idx].entries[src_ref.entry_idx];
      if (entry.crystal_id == src_entry.crystal_id && entry.filter_id == src_entry.filter_id) {
        pick_target_disabled = true;  // already shared
      }
    }
  }

  ImGui::PushID(entry_idx);

  // Active highlight: when the unified edit modal is bound to this entry,
  // thicken the child border and tint it with the focus accent color so the
  // user can trace which card the open modal corresponds to. The lifecycle is
  // strictly tied to IsEditModalOpen() — close paths (OK / Cancel / auto-close
  // via index-validity guard) flip the gate, no extra reset needed here.
  bool active = false;
  if (IsEditModalOpen()) {
    auto target = GetEditModalTarget();
    active = (target.layer_idx == layer_idx && target.entry_idx == entry_idx);
  }
  if (active) {
    // Plan B fallback: if the NavHighlight token's contrast against
    // ImGuiCol_Border becomes insufficient under a future theme, replace the
    // pushed color with IM_COL32(80, 160, 255, 255) per plan §7 Risk 1
    // (cool-blue accent, no family clash with red Delete or neutral Duplicate).
    ImGui::PushStyleColor(ImGuiCol_Border, ImGui::GetStyleColorVec4(ImGuiCol_NavHighlight));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, kActiveCardBorder);
  }

  // ---- Co-shared highlight ----
  // If an edit modal is open on a *different* card whose (crystal_id, filter_id)
  // matches this card's, render a co-shared border tint so the user sees which
  // cards their in-flight edit will affect. Distinct from `active` (modal owns
  // *this* card) — uses a warmer accent to differentiate.
  bool co_shared = false;
  if (!active && IsEditModalOpen()) {
    auto target = GetEditModalTarget();
    if (target.layer_idx >= 0 && target.layer_idx < static_cast<int>(state.layers.size()) && target.entry_idx >= 0 &&
        target.entry_idx < static_cast<int>(state.layers[target.layer_idx].entries.size())) {
      const auto& tgt_entry = state.layers[target.layer_idx].entries[target.entry_idx];
      if (entry.crystal_id == tgt_entry.crystal_id && entry.filter_id == tgt_entry.filter_id) {
        co_shared = true;
      }
    }
  }
  if (co_shared) {
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.65f, 0.2f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, kActiveCardBorder);
  }

  ImGui::BeginChild("##card", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);

  // Previous-frame hover state controls the alpha of the hover-action buttons
  // (always-render + alpha transition to keep click paths stable).
  ImGuiID hover_persist_id = ImGui::GetID("##card_hover_persist");
  bool hover_prev = ImGui::GetStateStorage()->GetBool(hover_persist_id, false);

  // Use frame-height spacing so each row reserves room for the Edit button (taller than text);
  // otherwise Row 0-2 buttons would overlap the Weight slider in Row 3.
  float row_h = ImGui::GetFrameHeightWithSpacing();
  float spacing_x = ImGui::GetStyle().ItemSpacing.x;
  float spacing_y = ImGui::GetStyle().ItemSpacing.y;

  // Thumbnail display size matches the right column's content height:
  // top of Row 0 (= thumb_pos.y) to bottom of Row 3 (= row_h*4 - spacing_y).
  // Square (W=H) so the crystal aspect is preserved. kThumbnailSize is the FBO
  // render resolution; ImGui scales the texture to thumb_display_size on draw.
  float thumb_display_size = row_h * 4.0f - spacing_y;

  // Left column: crystal thumbnail (or grey placeholder if not yet rendered)
  ImVec2 thumb_pos = ImGui::GetCursorScreenPos();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  auto thumb_tex = g_thumbnail_cache.GetTexture(entry.crystal_id);
  ImVec2 thumb_br(thumb_pos.x + thumb_display_size, thumb_pos.y + thumb_display_size);
  // An excluded entry draws its thumbnail dimmed so the card reads as "out" at a
  // glance, from across the panel, without having to find the toggle glyph.
  const ImU32 thumb_tint = entry.enabled ? IM_COL32(255, 255, 255, 255) : IM_COL32(255, 255, 255, 70);
  if (thumb_tex != 0) {
    // OpenGL texture Y-axis is flipped relative to ImGui: uv0=(0,1) uv1=(1,0)
    draw_list->AddImage(static_cast<ImTextureID>(thumb_tex), thumb_pos, thumb_br, ImVec2(0, 1), ImVec2(1, 0),
                        thumb_tint);
  } else {
    draw_list->AddRectFilled(thumb_pos, thumb_br,
                             entry.enabled ? IM_COL32(60, 60, 60, 255) : IM_COL32(45, 45, 45, 255));
  }
  draw_list->AddRect(thumb_pos, thumb_br, IM_COL32(100, 100, 100, 255));

  // ---- Participation toggle (always visible) ----
  // Semantics: "exclude this crystal, then re-run". Turning it off does NOT subtract this
  // crystal from the current image — it hands its share of the fixed total ray count back to
  // its siblings on the next run, so the rest of the scene gets MORE samples. That is a
  // different operation from the Colors panel's per-component eye, which is display-time
  // subtraction, so the glyph here deliberately is not an eye.
  //
  // Placement: overlaid on the thumbnail's top-left corner. Unlike Delete/Duplicate this is not
  // a hover-revealed action — "does this card count?" has to be as scannable as the Weight
  // number — and the thumbnail corner is the only always-free real estate on the card that does
  // not push the right column's four rows out of their shared three-column alignment.
  {
    // "###" and not "##": the visible glyph flips with the state, and with "##" the id hashes the
    // whole label, so the button would become a different widget the instant it is clicked —
    // losing ImGui's active-id continuity and leaving no stable path for a test to address. Same
    // reasoning as the defaults panel's state-dependent cells.
    char toggle_id[48];
    snprintf(toggle_id, sizeof(toggle_id), "%s###enabled_%d_%d", entry.enabled ? ICON_FA_TOGGLE_ON : ICON_FA_TOGGLE_OFF,
             layer_idx, entry_idx);
    constexpr float kTogglePad = 3.0f;
    ImGui::SetCursorScreenPos(ImVec2(thumb_pos.x + kTogglePad, thumb_pos.y + kTogglePad));
    if (!entry.enabled) {
      // Theme-owned disabled tone, not a semantic_colors.hpp grade: "excluded" is a state of
      // this control, not a judgement about the crystal (see that header's scope note).
      ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    }
    const bool toggle_clicked = ImGui::SmallButton(toggle_id);
    if (!entry.enabled) {
      ImGui::PopStyleColor();
    }
    if (toggle_clicked) {
      entry.enabled = !entry.enabled;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", entry.enabled ?
                                  "Participating. Click to exclude this crystal from the next run — its share of "
                                  "the total ray count goes to the other crystals (they get more samples)." :
                                  "Excluded from the next run. Its Weight is kept and takes effect again when you "
                                  "turn it back on.");
    }
  }

  // Right column — layout matches SliderWithInput's three-column model:
  //   [text / slider (text_w)] [Edit button / input (kInputWidth)] [row label (kLabelColWidth)]
  // so Row 1-3 align column boundaries with Row 4 automatically.
  float right_x = thumb_pos.x + thumb_display_size + spacing_x;
  float avail_w = ImGui::GetContentRegionAvail().x - thumb_display_size - spacing_x;
  float text_w = std::max(40.0f, avail_w - kInputWidth - kLabelColWidth - spacing_x * 2);

  auto emit_row = [&](int row_idx, const char* text_content, const char* btn_id, EditTarget target,
                      const char* row_label, bool clip_text, const char* tooltip = nullptr) {
    ImVec2 line_start(right_x, thumb_pos.y + row_h * static_cast<float>(row_idx));
    ImGui::SetCursorScreenPos(line_start);
    if (clip_text) {
      ImVec2 clip_min = line_start;
      ImVec2 clip_max(line_start.x + text_w, line_start.y + ImGui::GetTextLineHeight() + 2.0f);
      ImGui::PushClipRect(clip_min, clip_max, true);
      ImGui::TextUnformatted(text_content);
      ImGui::PopClipRect();
    } else {
      ImGui::TextUnformatted(text_content);
    }
    // Optional hover tooltip — shows the full multi-row SoP for non-degenerate
    // filters where the summary line is inherently lossy (only the first row
    // + "(+N more)" fits). Follows the same "TextUnformatted then IsItemHovered"
    // pattern as the fa-link badge below.
    if (tooltip != nullptr && ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", tooltip);
    }
    ImGui::SameLine();
    ImGui::SetCursorScreenPos(ImVec2(line_start.x + text_w + spacing_x, line_start.y));
    if (ImGui::Button(btn_id, ImVec2(kInputWidth, 0))) {
      g_edit_request = { target, layer_idx, entry_idx };
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(row_label);
  };

  // Row 1: Crystal type (resolved from pool)
  const CrystalConfig& crystal_ref = state.crystals[entry.crystal_id];
  const char* type_name = (crystal_ref.type == CrystalType::kPrism) ? "Prism" : "Pyramid";
  emit_row(0, type_name, "Edit##cr", EditTarget::kCrystal, "Crystal", false);

  // Row 2: Axis preset (resolved from pool)
  std::string preset = AxisPresetName(crystal_ref);
  emit_row(1, preset.c_str(), "Edit##ax", EditTarget::kAxis, "Axis", false);

  // Row 3: Filter summary (may exceed text_w — clip so it doesn't overlap the Edit button).
  // For non-degenerate SoP (>1 row or >1 factor), build a tooltip listing every
  // row's canonical text so users can see the full predicate without opening
  // the modal.
  std::optional<FilterConfig> filter_opt;
  if (entry.filter_id.has_value()) {
    filter_opt = state.filters[*entry.filter_id];
  }
  std::string filter_text = FilterSummary(filter_opt);
  std::string filter_tooltip_storage;
  const char* filter_tooltip = nullptr;
  if (filter_opt.has_value() && !filter_opt->IsDegenerateSingleFactor()) {
    // Card tooltip visibility is intentionally gated by IsDegenerateSingleFactor()
    // (i.e. only shown for genuinely non-degenerate multi-row / multi-factor
    // filters), whereas the editor-side live preview uses a different, wider
    // gate (any non-blank row). The formatting is shared via
    // FormatSopExpansionPreview so both call sites cannot drift, but the
    // visibility policy stays deliberately different — see edit_modals.cpp
    // RenderSummandRowList for the editor gate rationale.
    filter_tooltip_storage = gui::FormatSopExpansionPreview(filter_opt->param);
    filter_tooltip = filter_tooltip_storage.c_str();
  }
  emit_row(2, filter_text.c_str(), "Edit##fi", EditTarget::kFilter, "Filter", true, filter_tooltip);

  // Row 4: Weight — reuse SliderWithInput for [slider][input] layout
  ImGui::SetCursorScreenPos(ImVec2(right_x, thumb_pos.y + row_h * 3.0f));
  char prop_label[32];
  snprintf(prop_label, sizeof(prop_label), "Weight##prop_%d_%d", layer_idx, entry_idx);
  // Greyed out while excluded so it is visible that the number is currently inert — same
  // disable-when-inert convention as the layer prob slider. BeginDisabled only blocks
  // interaction; entry.proportion keeps its value, which is what makes the toggle reversible.
  ImGui::BeginDisabled(!entry.enabled);
  SliderWithInput(prop_label, &entry.proportion, 0.0f, 100.0f, "%.1f");
  ImGui::EndDisabled();
  if (!entry.enabled && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Weight is inert while this crystal is excluded. The value is kept.");
  }

  // Hover action buttons: stacked vertically at the right edge of the card —
  // Delete (×) on top, Duplicate (D) below, separated by kHoverBtnGap. Alpha is
  // driven by previous-frame hover state; buttons are always in the ImGui tree
  // so click paths remain stable, only visibility transitions.
  //
  // Fast-swipe mitigation: vertical stacking (v6/card-layout-v2) prevents a
  // single horizontal swipe from crossing the always-hit-tested Delete button,
  // which was the original backlog concern. Confirm dialog / undo intentionally
  // avoided — v5 verified that BeginDisabled(!hover_prev) and clicked+hover_prev
  // both break imgui_test_engine MouseMove+Yield+ItemClick timing.
  //
  // AutoResizeY first-frame drift (backlog Minor 3): ImGuiChildFlags_AutoResizeY
  // only auto-fits the Y dimension; X is driven by the parent layout and stable
  // on the first frame. The Y coordinates (del_y/dup_y) anchor to card_top, not
  // WindowSize.y, so they are also frame-stable. No positional fix needed.
  //
  // Coordinate strategy:
  //   x: card_right - btn_w - kHoverBtnPad
  //   delete y: card_top + kHoverBtnPad
  //   duplicate y: delete_y + btn_h + kHoverBtnGap
  char dup_id[32];
  char del_id[32];
  snprintf(dup_id, sizeof(dup_id), ICON_FA_COPY "##dup_%d_%d", layer_idx, entry_idx);
  snprintf(del_id, sizeof(del_id), ICON_FA_XMARK "##del_%d_%d", layer_idx, entry_idx);

  float frame_pad_x = ImGui::GetStyle().FramePadding.x;
  float dup_glyph_w = ImGui::CalcTextSize(ICON_FA_COPY).x;
  float del_glyph_w = ImGui::CalcTextSize(ICON_FA_XMARK).x;
  float btn_w = std::max(dup_glyph_w, del_glyph_w) + frame_pad_x * 2.0f;
  float btn_h = ImGui::GetFrameHeight();
  constexpr float kHoverBtnPad = 2.0f;
  ImVec2 card_win_pos = ImGui::GetWindowPos();
  ImVec2 card_win_sz = ImGui::GetWindowSize();
  float btn_x = card_win_pos.x + card_win_sz.x - btn_w - kHoverBtnPad;
  float del_y = card_win_pos.y + kHoverBtnPad;
  float dup_y = del_y + btn_h + kHoverBtnGap;

  ImGui::PushStyleVar(ImGuiStyleVar_Alpha, hover_prev ? 1.0f : 0.0f);
  // Delete button (top): red when enabled (destructive action); auto-greyed when
  // disabled (only one entry in layer — cannot remove last entry).
  ImGui::SetCursorScreenPos(ImVec2(btn_x, del_y));
  bool can_delete_entry = state.layers[layer_idx].entries.size() > 1;
  if (can_delete_entry) {
    PushDestructiveStyle();
  } else {
    ImGui::BeginDisabled();
  }
  bool delete_clicked = ImGui::SmallButton(del_id);
  if (can_delete_entry) {
    PopDestructiveStyle();
  } else {
    ImGui::EndDisabled();
  }
  // Duplicate button (below)
  ImGui::SetCursorScreenPos(ImVec2(btn_x, dup_y));
  bool dup_clicked = ImGui::SmallButton(dup_id);
  ImGui::PopStyleVar();

  if (dup_clicked) {
    // Duplicate = clone-to-pool: append new CrystalConfig (and new FilterConfig
    // if present) so the dup'd entry is fully independent. Capture pool copies
    // BEFORE push_back to avoid dangling references if vector reallocates.
    CrystalConfig cloned_crystal = state.crystals[entry.crystal_id];
    std::optional<FilterConfig> cloned_filter;
    if (entry.filter_id.has_value()) {
      cloned_filter = state.filters[*entry.filter_id];
    }
    EntryCard new_entry;
    new_entry.crystal_id = static_cast<int>(state.crystals.size());
    new_entry.proportion = entry.proportion;
    // Duplicate builds the clone field-by-field rather than copying the struct, so every
    // EntryCard field needs a line here. Carrying `enabled` over keeps duplicate a pure clone:
    // without it, duplicating an excluded card would hand back a participating one.
    new_entry.enabled = entry.enabled;
    state.crystals.push_back(std::move(cloned_crystal));
    if (cloned_filter.has_value()) {
      new_entry.filter_id = static_cast<int>(state.filters.size());
      state.filters.push_back(std::move(*cloned_filter));
    }
    auto& entries = state.layers[layer_idx].entries;
    entries.push_back(new_entry);
    g_thumbnail_cache.OnLayerStructureChanged();
  }

  // Persist hover state for next frame (computed while still inside the child
  // window so widget hover does not disqualify it).
  bool hover_now = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
  ImGui::GetStateStorage()->SetBool(hover_persist_id, hover_now);

  // ---- fa-link badge (static sharing indicator) ----
  // Drawn AFTER all inner widgets but BEFORE the pick-mode overlay so it shows
  // through the click-through invisible button. Predicate: (crystal_id,
  // filter_id) is shared by 2+ entries across all layers (this card included).
  {
    const int shared = CountEntriesSharing(state, entry.crystal_id, entry.filter_id);
    if (shared >= 2) {
      // Anchor: third slot in the right-edge column, directly below the
      // hover-revealed Delete (top) / Duplicate (middle) buttons. Stays
      // visible regardless of hover (it's a persistent state indicator, not
      // an action). Horizontally centered within the column for visual
      // alignment with the buttons above.
      const float glyph_w = ImGui::CalcTextSize(ICON_FA_LINK).x;
      const float badge_x = btn_x + (btn_w - glyph_w) * 0.5f;
      const float badge_y = dup_y + btn_h + kHoverBtnGap;
      ImGui::SetCursorScreenPos(ImVec2(badge_x, badge_y));
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 1.0f, 1.0f));
      ImGui::TextUnformatted(ICON_FA_LINK);
      ImGui::PopStyleColor();
      if (ImGui::IsItemHovered()) {
        std::string list;
        for (int li = 0; li < static_cast<int>(state.layers.size()); ++li) {
          for (int ei = 0; ei < static_cast<int>(state.layers[li].entries.size()); ++ei) {
            const auto& other = state.layers[li].entries[ei];
            if (other.crystal_id == entry.crystal_id && other.filter_id == entry.filter_id) {
              if (li == layer_idx && ei == entry_idx) {
                continue;  // skip self in the tooltip list
              }
              if (!list.empty()) {
                list += "\n";
              }
              list += "Layer " + std::to_string(li) + " / Entry " + std::to_string(ei);
            }
          }
        }
        if (list.empty()) {
          list = "(no other entries)";
        }
        ImGui::SetTooltip("Shared with:\n%s", list.c_str());
      }
    }
  }

  // ---- Card-area click handling ----
  // Detect a click anywhere inside the card area via a non-layout-affecting
  // rect query. We previously used SetCursorScreenPos(card_win_pos) +
  // InvisibleButton(card_win_sz), but that combination drives an AutoResizeY
  // feedback loop: GetWindowSize() includes the child's padding, the button
  // advances the cursor by `pos + size`, AutoResizeY grows the child to fit,
  // next frame GetWindowSize() returns the new larger size, and the card
  // expands unboundedly while pick mode is active.
  //
  // !IsAnyItemHovered() preserves hit priority for the inner widgets
  // (Edit / Duplicate / Delete): when one of them is hovered, the click is
  // routed to it and pick-mode cancellation runs via the blank-area handler
  // in app_panels.cpp instead.
  if (pick_active) {
    if (!pick_target_disabled) {
      const ImVec2 card_max(card_win_pos.x + card_win_sz.x, card_win_pos.y + card_win_sz.y);
      // IsWindowHovered() gates against floating windows (e.g. Colors) covering the card:
      // ImGui's FindHoveredWindow already resolved z-order for this frame, and returns false
      // here when the ##card child is not the top-most window under the cursor. AND with the
      // existing rect test keeps no-overlap behavior identical (task-color-window-mouse-capture).
      if (ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(card_win_pos, card_max) &&
          ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::IsAnyItemHovered()) {
        // "Link A to B" semantics: A (the entry whose modal opened the picker —
        // pick_link_source) adopts B's (the clicked card's) crystal/filter ids.
        // So in ApplyPickLink(source, target) the *clicked card* is the source
        // (model) and pick_link_source is the target (modified).
        const auto pick_source_ref = *state.pick_link_source;
        ApplyPickLink(state, GuiState::EntryRef{ layer_idx, entry_idx }, pick_source_ref);
        state.pick_link_source.reset();
        // Effects derived centrally by ReconcileGuiEffects: rebinding entry.filter_id
        // shows up as a `layers` diff (soft), and filter presence-toggle (nullopt↔some)
        // is caught by AnyEntryFilterPresenceChanged (hard) — see gui_state_reconcile.cpp.
        // Pure some(A)→some(B) rebinding stays soft, matching pre-migration behavior.
        // No explicit Invalidate: editing entry now shares clicked card's
        // crystal_id; that crystal already has a cache entry from this frame.
      }
    }
  } else {
    const ImVec2 card_max(card_win_pos.x + card_win_sz.x, card_win_pos.y + card_win_sz.y);
    // See pick-mode branch above for IsWindowHovered() rationale.
    if (ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(card_win_pos, card_max) &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::IsAnyItemHovered()) {
      g_edit_request = { EditTarget::kCard, layer_idx, entry_idx };
    }
  }

  ImGui::EndChild();  // ##card — must be unconditional

  if (active) {
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
  }
  if (co_shared) {
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
  }

  ImGui::PopID();
  return delete_clicked;
}


// ========== Layer ==========

void RenderLayer(GuiState& state, int layer_idx) {
  auto& layer = state.layers[layer_idx];

  ImGui::PushID(layer_idx);

  char header_label[32];
  snprintf(header_label, sizeof(header_label), "Layer %d", layer_idx + 1);

  bool header_open =
      ImGui::CollapsingHeader(header_label, ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_AllowItemOverlap);

  // Right-aligned delete button on the header row. Only enabled when more than
  // one layer exists (the scattering model requires at least one layer).
  char layer_del_id[32];
  snprintf(layer_del_id, sizeof(layer_del_id), ICON_FA_XMARK "##layer_%d", layer_idx);
  bool can_delete_layer = state.layers.size() > 1;
  float layer_del_w = ImGui::CalcTextSize(ICON_FA_XMARK).x + ImGui::GetStyle().FramePadding.x * 2.0f;
  ImGui::SameLine(ImGui::GetContentRegionMax().x - layer_del_w);
  if (can_delete_layer) {
    PushDestructiveStyle();
  } else {
    ImGui::BeginDisabled();
  }
  bool layer_delete_clicked = ImGui::SmallButton(layer_del_id);
  if (can_delete_layer) {
    PopDestructiveStyle();
  } else {
    ImGui::EndDisabled();
  }
  if (layer_delete_clicked && can_delete_layer) {
    state.layers.erase(state.layers.begin() + layer_idx);
    g_thumbnail_cache.OnLayerStructureChanged();
    ImGui::PopID();
    return;  // Skip rendering the rest; layer has been erased.
  }

  if (header_open) {
    // Multi-scatter probability slider — four states covering both footguns:
    //   (a) last layer & prob≈0  → slider disabled (locked at correct 0)
    //   (b) last layer & prob>0  → slider enabled + warning icon (came from a
    //       hand-written config; we don't silently rewrite the file value, so
    //       let the user drag it back to 0)
    //   (c) non-last layer & prob≈0 → slider enabled + warning icon (the next
    //       layer will receive no rays — footgun #2)
    //   (d) otherwise → plain slider
    // Zero-detection uses IsProbZero (epsilon) rather than == 0.0f — slider
    // drags can produce sub-step floats that would sneak past a strict check.
    char prob_id[32];
    snprintf(prob_id, sizeof(prob_id), "Prob.##layer_%d", layer_idx);
    bool is_last_layer = layer_idx == static_cast<int>(state.layers.size()) - 1;
    bool prob_is_zero = IsProbZero(layer.probability);
    bool disable_slider = is_last_layer && prob_is_zero;
    ImGui::BeginDisabled(disable_slider);
    ImGui::BeginGroup();
    SliderWithInput(prob_id, &layer.probability, 0.0f, 1.0f, "%.2f");
    ImGui::EndGroup();
    ImGui::EndDisabled();
    const char* prob_tip = nullptr;
    if (disable_slider) {
      prob_tip = "Last layer: all filter-pass rays are effective output; prob does not apply.";
    } else if (is_last_layer && !prob_is_zero) {
      prob_tip =
          "Last layer has prob > 0: that fraction of rays will be discarded (no next layer to receive them). Set to 0.";
    } else if (!is_last_layer && prob_is_zero) {
      prob_tip = "prob = 0 means no rays reach the next scattering layer (it will be effectively dead).";
    } else {
      prob_tip = "Fraction of rays continuing to the next layer.";
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("%s", prob_tip);
    }
    bool show_warning_icon = (is_last_layer && !prob_is_zero) || (!is_last_layer && prob_is_zero);
    if (show_warning_icon) {
      ImGui::SameLine();
      ImGui::TextColored(WarningTextColor(), ICON_FA_CIRCLE_EXCLAMATION);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", prob_tip);
      }
    }

    // Every crystal in this layer excluded: the layer contributes nothing to the next run.
    // Non-blocking on purpose — it is a legitimate transient state while the user toggles cards
    // one at a time, and the engine already handles it (PartitionCrystalRayNum returns an
    // all-zero allocation for a zero total; see its AllZeroProportions unit test), so there is
    // nothing to guard against, only something to point out.
    if (AllEntriesDisabled(layer)) {
      // Its own line rather than SameLine on the prob row: that row may already be showing a
      // CIRCLE_EXCLAMATION for the prob footguns, and two identical glyphs side by side saying
      // different things is worse than no icon at all.
      ImGui::TextColored(WarningTextColor(),
                         ICON_FA_CIRCLE_EXCLAMATION " All crystals excluded — layer produces no rays");
    }

    // Render entry cards with deferred deletion
    int pending_delete_entry = -1;
    for (int i = 0; i < static_cast<int>(layer.entries.size()); i++) {
      ImGui::Spacing();
      bool del = RenderEntryCard(state, layer_idx, i);
      if (del) {
        pending_delete_entry = i;
      }
    }

    // Deferred delete
    if (pending_delete_entry >= 0 && layer.entries.size() > 1) {
      layer.entries.erase(layer.entries.begin() + pending_delete_entry);
      g_thumbnail_cache.OnLayerStructureChanged();
    }

    // Add entry button. Bind the new entry to a fresh pool slot so it is
    // independent by default — using EntryCard's default (crystal_id = 0)
    // would silently link the new entry to whichever entry already references
    // slot 0, making +Crystal look like "implicit Link to entry 0".
    ImGui::Spacing();
    char add_id[32];
    snprintf(add_id, sizeof(add_id), "+ Crystal##layer_%d", layer_idx);
    if (ImGui::SmallButton(add_id)) {
      EntryCard new_entry;
      new_entry.crystal_id = static_cast<int>(state.crystals.size());
      state.crystals.emplace_back();
      layer.entries.push_back(new_entry);
      g_thumbnail_cache.OnLayerStructureChanged();
    }
  }

  ImGui::PopID();
}


// ========== Scattering Section (layer management) ==========

void RenderScatteringSection(GuiState& state) {
  for (int i = 0; i < static_cast<int>(state.layers.size()); i++) {
    RenderLayer(state, i);
    ImGui::Spacing();
  }
}


// ========== Scene Controls (rendered in the right panel Scene group) ==========

void RenderSceneControls(GuiState& state) {
  ImGui::SeparatorText("Sun");
  ImGui::BeginGroup();
  // AC2 migration path (scrum-gui-state-reconcile T0): widget only writes state.sun.altitude; the
  // resulting dirty is derived by ReconcileGuiEffects in SyncFromPoller (diff on state.sun vs.
  // last_committed_state.sun). The pre-migration DIRTY_IF wrapper is retired at this call site.
  // Domain and format read from the field editor registry, not written here. `fmt` is passed
  // explicitly even though it currently equals SliderWithInput's own default: relying on the
  // default would leave the display precision as a second statement this call site makes on its
  // own, which is the thing being removed.
  const FieldEditorConstraint alt_c = ConstraintFor("sun.altitude", state);
  SliderWithInput("Altitude", &state.sun.altitude, static_cast<float>(alt_c.min_value),
                  static_cast<float>(alt_c.max_value), alt_c.fmt, alt_c.scale);
  ImGui::EndGroup();
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Sun elevation angle above the horizon");
  }
  ImGui::BeginGroup();
  // AC2 migration path: same rationale as sun.altitude above.
  const FieldEditorConstraint dia_c = ConstraintFor("sun.diameter", state);
  SliderWithInput("Diameter", &state.sun.diameter, static_cast<float>(dia_c.min_value),
                  static_cast<float>(dia_c.max_value), dia_c.fmt, dia_c.scale);
  ImGui::EndGroup();
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Angular diameter of the sun disk");
  }
  ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
  // Combo carries kSpectrumCount presets + "Custom..." tail (item count = kSpectrumComboItemCount).
  // Built once from kSpectrumNames so adding/renaming a preset only requires editing kSpectrumNames.
  static const char* const* kSpectrumComboItems = [] {
    static const char* items[kSpectrumComboItemCount];
    for (int i = 0; i < kSpectrumCount; i++)
      items[i] = kSpectrumNames[i];
    items[kSpectrumCount] = "Custom...";
    return items;
  }();
  // Bind the combo to a local, NOT state.sun.spectrum_index: picking "Custom..." must not commit the
  // custom index before the modal confirms. spectrum_index is only advanced to kCustomSpectrumIndex
  // inside the modal's OK (single transactional commit), so dismissing via Escape / click-outside /
  // Cancel leaves it at the prior valid preset. The combo re-reads spectrum_index each frame, so it
  // shows the prior preset while the editor is open and flips to "Custom..." once OK commits.
  int combo_sel = state.sun.spectrum_index;
  if (ImGui::Combo("Spectrum", &combo_sel, kSpectrumComboItems, kSpectrumComboItemCount)) {
    if (combo_sel == kCustomSpectrumIndex) {
      OpenSpectrumModal(state);  // intent-only: open the editor; OK is the sole commit point
    } else {
      state.sun.spectrum_index = combo_sel;
      // Keep custom_spectrum intact — it is only read when spectrum_index==kCustomSpectrumIndex, so
      // switching to a preset and back restores the user's edits instead of silently discarding them.
    }
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Light source spectrum for wavelength-dependent refraction.\n"
        "\"Custom...\" opens an editor for a discrete wavelength/weight list.");
  }
  ImGui::PopItemWidth();
  if (state.sun.spectrum_index == kCustomSpectrumIndex) {
    if (ImGui::SmallButton("Edit spectrum...##spectrum_edit")) {
      OpenSpectrumModal(state);  // re-open editor for the existing custom spectrum
    }
  }

  ImGui::SeparatorText("Simulation");
  ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
  Checkbox("Infinite rays", &state.sim.infinite);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Run simulation continuously until manually stopped");
  }
  ImGui::PopItemWidth();
  ImGui::BeginGroup();
  // One call, not one per branch. The two branches were identical apart from the BeginDisabled
  // wrapper, and `enabled` (the registry's "infinite rays is on, so no ray total applies") is
  // exactly the condition they split on — so the split has nothing left to express, and collapsing
  // it removes the only way this field could still hold two constraints that disagree.
  const FieldEditorConstraint rays_c = ConstraintFor("sim.ray_num_millions", state);
  ImGui::BeginDisabled(!rays_c.enabled);
  SliderWithInput("Rays(M)", &state.sim.ray_num_millions, static_cast<float>(rays_c.min_value),
                  static_cast<float>(rays_c.max_value), rays_c.fmt, rays_c.scale);
  ImGui::EndDisabled();
  ImGui::EndGroup();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(
        "Total rays across all spectrum wavelengths, in millions.\n"
        "The server distributes the total to each wavelength as\n"
        "ceil(total / N_wavelengths) per wavelength.");
  }
  ImGui::BeginGroup();
  // An int field has no fmt/scale to read — SliderIntWithInput takes neither.
  const FieldEditorConstraint hits_c = ConstraintFor("sim.max_hits", state);
  SliderIntWithInput("Max hits", &state.sim.max_hits, static_cast<int>(hits_c.min_value),
                     static_cast<int>(hits_c.max_value));
  ImGui::EndGroup();
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Maximum number of crystal face hits per ray path");
  }

  // GPU backend toggle (Metal on Apple, CUDA on NVIDIA). Marked dirty explicitly so
  // the next Apply/Run reconstructs the server for the chosen backend
  // (MaybeReconstructServerForBackend in app.cpp) — CPU N-worker vs GPU single
  // engine are different orchestration topologies, so the server is rebuilt and the
  // accumulated image resets on toggle. Falls back to CPU silently if the active
  // config is not GPU-compatible.
  // use_gpu_backend is intentionally excluded from ConfigSnapshot (session/view field,
  // see gui_state.hpp field-sync scope comment), so it cannot participate in the
  // reconciler auto-diff — the manual MarkDirty call below is the T0 documented exception.
  // Runtime gate: only show the checkbox when a GPU backend is actually available
  // (Metal device on Apple / NVIDIA device + usable CUDA on Windows-Linux), so it
  // never appears on CPU-only hosts or machines with very old hardware / broken GPU
  // drivers, where selecting it would otherwise fail in EnsureDevice. The probe is
  // cached, so the per-frame cost is a plain memory read.
  // ImGui::Checkbox renders its label to the right and ignores the item-width
  // stack, so no PushItemWidth wrapper is needed here.
  if (LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL) || LUMICE_IsBackendAvailable(LUMICE_BACKEND_CUDA)) {
    // Disable the toggle while busy (simulating OR async Stop draining): the backend switch
    // reconstructs the server on the next DoRun, and an in-flight stop still holds it (R1).
    bool busy = state.sim_state == GuiState::SimState::kSimulating || state.sim_state == GuiState::SimState::kStopping;
    if (busy) {
      ImGui::BeginDisabled();
    }
    if (Checkbox("Use GPU", &state.use_gpu_backend)) {
      state.MarkDirty();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Use the GPU for simulation (falls back to CPU if incompatible)");
    }
    if (busy) {
      ImGui::EndDisabled();
    }
  }
}

}  // namespace lumice::gui
