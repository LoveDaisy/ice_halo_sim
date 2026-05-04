#include "gui/panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "config/render_config.hpp"
#include "gui/app.hpp"
#include "gui/axis_presets.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/slider_mapping.hpp"
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
      *value = sqrt_val * sqrt_val;
      changed = true;
    }
  } else if (scale == SliderScale::kLog && min_val > 0.0f) {
    float norm = slider_mapping::LogValueToNorm(*value, min_val, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "", ImGuiSliderFlags_NoInput)) {
      *value = slider_mapping::LogNormToValue(norm, min_val, max_val);
      changed = true;
    }
  } else if (scale == SliderScale::kLogLinear && min_val == 0.0f) {
    float norm = slider_mapping::LogLinearValueToNorm(*value, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "", ImGuiSliderFlags_NoInput)) {
      *value = slider_mapping::LogLinearNormToValue(norm, max_val);
      changed = true;
    }
  } else {
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
//   - EntryExit:  "EE:<entry>→<exit> <In|Out>[ <sym>]"
//   - Direction:  "DIR:<az>°/<el>° r=<radii>° <In|Out>[ <sym>]"
//   - Crystal:    "CR:#<id> <In|Out>[ <sym>]"
//
// 12-character truncation is preserved for the raypath body so the existing
// test_gui_interaction "1-2-3-4-5-6-..." case stays bit-exact. Other types
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

  std::string body = std::visit(
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
          // text fallback ("?") so a half-typed config still renders
          // something meaningful in the entry card summary.
          const std::string& e = p.entry_text.empty() ? std::string{ "?" } : p.entry_text;
          const std::string& x = p.exit_text.empty() ? std::string{ "?" } : p.exit_text;
          return "EE:" + e + "\xe2\x86\x92" + x;
        } else if constexpr (std::is_same_v<T, DirectionParams>) {
          char buf[64];
          snprintf(buf, sizeof(buf), "DIR:%g\xc2\xb0/%g\xc2\xb0", p.az, p.el);
          return buf;
        } else {
          return "*";
        }
      },
      fc.param);

  return body + FilterSummarySuffix(fc);
}

namespace {

// Helper: wrap ImGui control and mark dirty on change
#define DIRTY_IF(expr) \
  if (expr)            \
  state.MarkDirty()

// Card layout: height is driven by ImGuiChildFlags_AutoResizeY so font/theme
// changes adapt automatically (kThumbnailSize lives in gui_constants.hpp).

// Destructive action button palette (delete/remove). Shared by entry-card and
// layer-header "x" buttons so the visual language stays consistent.
constexpr ImVec4 kBtnDestructiveNormal{ 0.70f, 0.22f, 0.22f, 1.0f };
constexpr ImVec4 kBtnDestructiveHovered{ 0.85f, 0.30f, 0.30f, 1.0f };
constexpr ImVec4 kBtnDestructiveActive{ 0.60f, 0.15f, 0.15f, 1.0f };

void PushDestructiveStyle() {
  ImGui::PushStyleColor(ImGuiCol_Button, kBtnDestructiveNormal);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kBtnDestructiveHovered);
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, kBtnDestructiveActive);
}

void PopDestructiveStyle() {
  ImGui::PopStyleColor(3);
}

}  // namespace


// Compute slider width and prepare IDs for the [slider] [input] Label layout.
// Writes slider_id and input_id buffers, returns the computed slider width.
static float PrepareSliderLayout(const char* label, char* display_label_out, size_t display_buf_size, char* slider_id,
                                 size_t slider_id_size, char* input_id, size_t input_id_size) {
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
  float slider_w = avail_w - kInputWidth - kLabelColWidth - spacing * 2;
  if (slider_w < 40.0f)
    slider_w = 40.0f;
  return slider_w;
}

// Render the label text after slider + input.
static void FinishSliderLayout(const char* display_label) {
  ImGui::SameLine();
  ImGui::TextUnformatted(display_label);
}

bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt,
                     SliderScale scale) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id));

  bool changed = false;

  ImGui::PushItemWidth(slider_w);
  changed |= RenderNonlinearSlider(slider_id, value, min_val, max_val, fmt, scale);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  changed |= ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  FinishSliderLayout(display_buf);
  return changed;
}

// SliderInt + InputInt + label text, same layout as SliderWithInput.
// Returns true if value changed.
static bool SliderIntWithInput(const char* label, int* value, int min_val, int max_val) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id));

  bool changed = false;

  ImGui::PushItemWidth(slider_w);
  changed |= ImGui::SliderInt(slider_id, value, min_val, max_val, "%d", ImGuiSliderFlags_NoInput);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  changed |= ImGui::InputInt(input_id, value, 0, 0);
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  FinishSliderLayout(display_buf);
  return changed;
}

struct ValuePreset {
  const char* label;
  float value;
};

// Slider + InputFloat with dropdown arrow for presets, laid out as: [slider] [input▼] Label
// Returns true if value changed.
static bool SliderWithPreset(const char* label, float* value, float min_val, float max_val, const char* fmt,
                             SliderScale scale, const ValuePreset* presets, int preset_count) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  constexpr float kArrowBtnWidth = 20.0f;
  float input_w = kInputWidth - kArrowBtnWidth;

  // Prepare layout (manually, since input area is split into input + arrow)
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

  // Slider (nonlinear scale support)
  ImGui::PushItemWidth(slider_w);
  changed |= RenderNonlinearSlider(slider_id, value, min_val, max_val, fmt, scale);
  ImGui::PopItemWidth();

  // Input
  ImGui::SameLine();
  ImGui::PushItemWidth(input_w);
  changed |= ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

  // Arrow button + preset popup
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

  FinishSliderLayout(display_buf);
  return changed;
}


// ---- Axis distribution controls (shared with edit modals) ----
//
// CALLER CONTRACT: when invoked from a context where the parent window may
// become a detached OS viewport (currently: Edit Entry modal in multi-viewport
// mode), the caller must precede this call with `SetNextComboPopupTopMost()`
// (see edit_modals.cpp). Without that, the internal `##dist` Combo's popup
// defaults to NSWindow layer=0 and gets rendered behind the modal at layer=3.
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

  static_assert(static_cast<int>(AxisDistType::kCount) == 5, "Update combo items when adding new AxisDistType");
  if (ImGui::Combo("##dist", &dist_type, "Gauss\0Uniform\0Zigzag\0Laplacian\0Gauss (legacy)\0")) {
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

  ImGui::BeginChild("##card", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);

  // Previous-frame hover state controls the alpha of the hover-action buttons
  // (always-render + alpha transition to keep click paths stable).
  ImGuiID hover_persist_id = ImGui::GetID("##card_hover_persist");
  bool hover_prev = ImGui::GetStateStorage()->GetBool(hover_persist_id, false);

  // Use frame-height spacing so each row reserves room for the Edit button (taller than text);
  // otherwise Row 0-2 buttons would overlap the prop. slider in Row 3.
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
  auto thumb_tex = g_thumbnail_cache.GetTexture(layer_idx, entry_idx);
  ImVec2 thumb_br(thumb_pos.x + thumb_display_size, thumb_pos.y + thumb_display_size);
  if (thumb_tex != 0) {
    // OpenGL texture Y-axis is flipped relative to ImGui: uv0=(0,1) uv1=(1,0)
    draw_list->AddImage(static_cast<ImTextureID>(thumb_tex), thumb_pos, thumb_br, ImVec2(0, 1), ImVec2(1, 0));
  } else {
    draw_list->AddRectFilled(thumb_pos, thumb_br, IM_COL32(60, 60, 60, 255));
  }
  draw_list->AddRect(thumb_pos, thumb_br, IM_COL32(100, 100, 100, 255));

  // Right column — layout matches SliderWithInput's three-column model:
  //   [text / slider (text_w)] [Edit button / input (kInputWidth)] [row label (kLabelColWidth)]
  // so Row 1-3 align column boundaries with Row 4 automatically.
  float right_x = thumb_pos.x + thumb_display_size + spacing_x;
  float avail_w = ImGui::GetContentRegionAvail().x - thumb_display_size - spacing_x;
  float text_w = std::max(40.0f, avail_w - kInputWidth - kLabelColWidth - spacing_x * 2);

  auto emit_row = [&](int row_idx, const char* text_content, const char* btn_id, EditTarget target,
                      const char* row_label, bool clip_text) {
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
    ImGui::SameLine();
    ImGui::SetCursorScreenPos(ImVec2(line_start.x + text_w + spacing_x, line_start.y));
    if (ImGui::Button(btn_id, ImVec2(kInputWidth, 0))) {
      g_edit_request = { target, layer_idx, entry_idx };
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(row_label);
  };

  // Row 1: Crystal type
  const char* type_name = (entry.crystal.type == CrystalType::kPrism) ? "Prism" : "Pyramid";
  emit_row(0, type_name, "Edit##cr", EditTarget::kCrystal, "Crystal", false);

  // Row 2: Axis preset
  std::string preset = AxisPresetName(entry.crystal);
  emit_row(1, preset.c_str(), "Edit##ax", EditTarget::kAxis, "Axis", false);

  // Row 3: Filter summary (may exceed text_w — clip so it doesn't overlap the Edit button)
  std::string filter_text = FilterSummary(entry.filter);
  emit_row(2, filter_text.c_str(), "Edit##fi", EditTarget::kFilter, "Filter", true);

  // Row 4: Proportion — reuse SliderWithInput for [slider][input] "prop." layout
  ImGui::SetCursorScreenPos(ImVec2(right_x, thumb_pos.y + row_h * 3.0f));
  char prop_label[32];
  snprintf(prop_label, sizeof(prop_label), "prop.##prop_%d_%d", layer_idx, entry_idx);
  if (SliderWithInput(prop_label, &entry.proportion, 0.0f, 100.0f, "%.1f")) {
    state.MarkDirty();
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
  // Glyphs: "D" for Duplicate (less ambiguous than "+", which reads as "Add"),
  // "\xC3\x97" (U+00D7 ×) for Delete. Both ASCII / Latin-1 — covered by ImGui's
  // default Proggy Clean font; replacing them with proper SVG icons is tracked
  // in backlog as "GUI icon font integration (FontAwesome)".
  snprintf(dup_id, sizeof(dup_id), "D##dup_%d_%d", layer_idx, entry_idx);
  snprintf(del_id, sizeof(del_id), "\xC3\x97##del_%d_%d", layer_idx, entry_idx);

  float frame_pad_x = ImGui::GetStyle().FramePadding.x;
  float dup_glyph_w = ImGui::CalcTextSize("D").x;
  float del_glyph_w = ImGui::CalcTextSize("\xC3\x97").x;
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
    auto& entries = state.layers[layer_idx].entries;
    entries.push_back(entry);  // deep copy
    g_thumbnail_cache.OnLayerStructureChanged();
    state.MarkDirty();
  }

  // Persist hover state for next frame (computed while still inside the child
  // window so widget hover does not disqualify it).
  bool hover_now = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
  ImGui::GetStateStorage()->SetBool(hover_persist_id, hover_now);

  ImGui::EndChild();  // ##card — must be unconditional

  if (active) {
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
  snprintf(layer_del_id, sizeof(layer_del_id), "x##layer_%d", layer_idx);
  bool can_delete_layer = state.layers.size() > 1;
  float layer_del_w = ImGui::CalcTextSize("x").x + ImGui::GetStyle().FramePadding.x * 2.0f;
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
    state.MarkDirty();
    ImGui::PopID();
    return;  // Skip rendering the rest; layer has been erased.
  }

  if (header_open) {
    // Multi-scatter probability slider
    char prob_id[32];
    snprintf(prob_id, sizeof(prob_id), "Prob.##layer_%d", layer_idx);
    bool single_layer = state.layers.size() <= 1;
    ImGui::BeginDisabled(single_layer);
    ImGui::BeginGroup();
    DIRTY_IF(SliderWithInput(prob_id, &layer.probability, 0.0f, 1.0f, "%.2f"));
    ImGui::EndGroup();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      const char* prob_tip = single_layer ?
                                 "Fraction of rays continuing to the next layer.\nAlways 0 for a single layer." :
                                 "Fraction of rays continuing to the next layer";
      ImGui::SetTooltip("%s", prob_tip);
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
      state.MarkDirty();
    }

    // Add entry button
    ImGui::Spacing();
    char add_id[32];
    snprintf(add_id, sizeof(add_id), "+ Crystal##layer_%d", layer_idx);
    if (ImGui::SmallButton(add_id)) {
      layer.entries.emplace_back();
      g_thumbnail_cache.OnLayerStructureChanged();
      state.MarkDirty();
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
  DIRTY_IF(SliderWithInput("Altitude", &state.sun.altitude, -90.0f, 90.0f));
  DIRTY_IF(SliderWithInput("Diameter", &state.sun.diameter, 0.1f, 5.0f));
  ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
  DIRTY_IF(ImGui::Combo("Spectrum", &state.sun.spectrum_index, kSpectrumNames, kSpectrumCount));
  ImGui::PopItemWidth();

  ImGui::SeparatorText("Simulation");
  ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
  DIRTY_IF(ImGui::Checkbox("Infinite rays", &state.sim.infinite));
  ImGui::PopItemWidth();
  if (!state.sim.infinite) {
    DIRTY_IF(SliderWithInput("Rays(M)", &state.sim.ray_num_millions, 0.1f, 100.0f));
  } else {
    ImGui::BeginDisabled();
    SliderWithInput("Rays(M)", &state.sim.ray_num_millions, 0.1f, 100.0f);
    ImGui::EndDisabled();
  }
  DIRTY_IF(SliderIntWithInput("Max hits", &state.sim.max_hits, 1, 20));
}

#undef DIRTY_IF

}  // namespace lumice::gui
