#include "gui/panels.hpp"

#include <algorithm>
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
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"  // FormatSummandText (non-degenerate SoP summary)
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

  const float old_value = *value;

  ImGui::PushItemWidth(slider_w);
  RenderNonlinearSlider(slider_id, value, min_val, max_val, fmt, scale);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  FinishSliderLayout(display_buf);
  return *value != old_value;
}

// SliderInt + InputInt + label text, same layout as SliderWithInput.
// Returns true if value changed.
static bool SliderIntWithInput(const char* label, int* value, int min_val, int max_val) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id));

  const int old_value = *value;

  ImGui::PushItemWidth(slider_w);
  ImGui::SliderInt(slider_id, value, min_val, max_val, "%d", ImGuiSliderFlags_NoInput);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  ImGui::InputInt(input_id, value, 0, 0);
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  FinishSliderLayout(display_buf);
  return *value != old_value;
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

// Default spread fraction applied when a shape field's randomization is first enabled: the
// distribution spread starts at 0.2 × center (i.e. roughly ±10% for uniform's center±spread/2).
// An arbitrary but reasonable seed value; the user adjusts from there.
constexpr float kShapeDistDefaultSpreadFraction = 0.2f;

bool RenderShapeDist(const char* label, ShapeDist& dist, float center_min, float center_max, const char* center_fmt,
                     SliderScale center_scale) {
  bool changed = false;
  ImGui::PushID(label);

  // Center is always editable (deterministic value / distribution center).
  changed |= SliderWithInput(label, &dist.center, center_min, center_max, center_fmt, center_scale);

  // Randomize toggle. NO_RANDOM is expressed ONLY via this checkbox — the type combo lists just
  // the five randomized types, so there is no redundant "manually pick no_random" path.
  bool randomize = dist.type != ShapeDistType::kNoRandom;
  if (ImGui::Checkbox("Randomize", &randomize)) {
    if (randomize) {
      dist.type = ShapeDistType::kUniform;                          // owner-defined default type
      dist.spread = kShapeDistDefaultSpreadFraction * dist.center;  // default spread heuristic
    } else {
      dist.type = ShapeDistType::kNoRandom;
      dist.spread = 0.0f;  // spread is meaningless for NO_RANDOM; zero it so operator==/round-trip stay clean
    }
    changed = true;
  }

  if (dist.type != ShapeDistType::kNoRandom) {
    // Combo lists the five randomized types; index 0..4 maps to kUniform..kGaussLegacy (enum
    // values 1..5), so combo_index = static_cast<int>(type) - 1.
    static_assert(static_cast<int>(ShapeDistType::kCount) == 6,
                  "Update RenderShapeDist combo when adding a ShapeDistType");
    int combo_index = static_cast<int>(dist.type) - 1;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    // Keep this combo's popup above a detached OS-viewport modal (mirrors SetNextComboPopupTopMost
    // in edit_modals.cpp). Self-contained here — set immediately before the Combo — so the fix does
    // not depend on the caller and holds even though the combo only renders when randomized.
    ImGuiWindowClass topmost;
    topmost.ViewportFlagsOverrideSet = ImGuiViewportFlags_TopMost;
    ImGui::SetNextWindowClass(&topmost);
    if (ImGui::Combo("##shapedist", &combo_index, "Uniform\0Gauss\0Zigzag\0Laplacian\0Gauss (legacy)\0")) {
      dist.type = static_cast<ShapeDistType>(combo_index + 1);
      changed = true;
    }

    // Spread slider — same units as center (a distance/height ratio), so the range tracks the
    // center's max. Type-specific label mirrors RenderAxisDist for user familiarity.
    const char* spread_label = "Std";
    if (dist.type == ShapeDistType::kUniform) {
      spread_label = "Range";
    } else if (dist.type == ShapeDistType::kZigzag) {
      spread_label = "Amplitude";
    } else if (dist.type == ShapeDistType::kLaplacian) {
      spread_label = "Scale";
    }
    changed |= SliderWithInput(spread_label, &dist.spread, 0.0f, center_max, center_fmt, SliderScale::kSqrt);
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
  SliderWithInput(prop_label, &entry.proportion, 0.0f, 100.0f, "%.1f");

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
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), ICON_FA_CIRCLE_EXCLAMATION);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", prob_tip);
      }
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
  SliderWithInput("Altitude", &state.sun.altitude, -90.0f, 90.0f);
  ImGui::EndGroup();
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Sun elevation angle above the horizon");
  }
  ImGui::BeginGroup();
  // AC2 migration path: same rationale as sun.altitude above.
  SliderWithInput("Diameter", &state.sun.diameter, 0.1f, 5.0f);
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
  ImGui::Checkbox("Infinite rays", &state.sim.infinite);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Run simulation continuously until manually stopped");
  }
  ImGui::PopItemWidth();
  ImGui::BeginGroup();
  if (!state.sim.infinite) {
    SliderWithInput("Rays(M)", &state.sim.ray_num_millions, 0.1f, 100.0f);
  } else {
    ImGui::BeginDisabled();
    SliderWithInput("Rays(M)", &state.sim.ray_num_millions, 0.1f, 100.0f);
    ImGui::EndDisabled();
  }
  ImGui::EndGroup();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(
        "Total rays across all spectrum wavelengths, in millions.\n"
        "The server distributes the total to each wavelength as\n"
        "ceil(total / N_wavelengths) per wavelength.");
  }
  ImGui::BeginGroup();
  SliderIntWithInput("Max hits", &state.sim.max_hits, 1, 64);
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
    if (ImGui::Checkbox("Use GPU", &state.use_gpu_backend)) {
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
