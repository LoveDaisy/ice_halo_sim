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

// Copy `label` up to its "##id" suffix into `out`, so a widget id and the text drawn for it
// can be one argument. Same rule as ImGui's own label parsing.
static void StripImGuiIdSuffix(const char* label, char* out, size_t out_size) {
  const char* hash_pos = strstr(label, "##");
  if (hash_pos) {
    auto len = static_cast<size_t>(hash_pos - label);
    if (len >= out_size) {
      len = out_size - 1;
    }
    memcpy(out, label, len);
    out[len] = '\0';
  } else {
    snprintf(out, out_size, "%s", label);
  }
}

// ---- Edit request state ----

}  // namespace

// ---- Property rows (see panels.hpp for the shape and the rationale) ----

bool BeginPropertyTable(const char* id) {
  // No borders and no RowBg: the alignment of the two columns is the whole visual mechanism,
  // and drawing the grid that produces it would add one box per row to a panel whose entire
  // theme decision was to stop drawing boxes (theme.cpp, FrameBorderSize = 0).
  // No scroll flags, deliberately: with ScrollX/ScrollY ImGui::BeginTable opens a child window
  // (imgui_tables.cpp, `use_child_window`), which would consume a queued SetNextWindowClass —
  // the mechanism RenderAxisDist's combo-popup contract depends on.
  if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX)) {
    return false;
  }
  ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed, kPropertyLabelColWidth);
  ImGui::TableSetupColumn("##control", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  return true;
}

void PropertyRow(const char* label) {
  char display_buf[64];
  StripImGuiIdSuffix(label, display_buf, sizeof(display_buf));

  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  // Right-align within the label cell. Overflow (a label wider than the token) is left to run
  // into the cell's clip rect rather than being centred or ellipsised: it is a calibration bug
  // in the token, and test_property_row.cpp is what reports it.
  const float avail = ImGui::GetContentRegionAvail().x;
  const float text_w = ImGui::CalcTextSize(display_buf).x;
  if (avail > text_w) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - text_w));
  }
  // The control across the row is framed and the label is not, so without this the label sits a
  // FramePadding.y above the text inside the control it names.
  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled("%s", display_buf);

  ImGui::TableNextColumn();
  ImGui::SetNextItemWidth(-FLT_MIN);
}

void EndPropertyTable() {
  ImGui::EndTable();
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

  // Unconditional, and NOT redundant with ImGuiSliderFlags_AlwaysClamp: that flag constrains the
  // values the widget itself produces, and deliberately leaves a value it was handed out of range
  // alone. The retired [slider][input] pair ended with exactly this line, and things depend on it
  // — a .lmc written by hand, or a lens switch that narrows a bound under a value that was legal a
  // frame ago (the globe's elevation limit), reach the field without going through any control, and
  // the control is what pulls them back in. Without it the page renders an out-of-domain value as
  // if it were fine.
  *value = std::clamp(*value, min_val, max_val);
  // Same return contract as SliderWithInput: "the value is not what it was", clamp included, since
  // a clamp is a change the caller has to commit like any other.
  return *value != old_value;
}

bool DragIntField(const char* label, int* value, int min_val, int max_val, const char* fmt) {
  char drag_id[80];
  snprintf(drag_id, sizeof(drag_id), "##%s", label);

  // Full domain per kDragTrackReferenceWidth pixels of drag, matching DragFloatField. The speed
  // is a float even for an int drag — ImGui accumulates the sub-unit remainder across frames, so a
  // domain narrower than the reference width still traverses in exactly that many pixels rather
  // than snapping one step per pixel.
  const float speed = static_cast<float>(max_val - min_val) / kDragTrackReferenceWidth;
  const int old_value = *value;
  ImGui::DragInt(drag_id, value, speed, min_val, max_val, fmt, ImGuiSliderFlags_AlwaysClamp);

  // Unconditional, for the same reason as DragFloatField's: see the note there.
  *value = std::clamp(*value, min_val, max_val);
  return *value != old_value;
}

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

// ---- Document-tree row meta ("info scent") ----
//
// The four functions below format the secondary value each tree row shows at its right edge. They
// are free functions taking domain values rather than inline snprintf calls at the render sites for
// one reason: the text is drawn with a bare ImGui::TextDisabled, which carries no item id, so a
// gui_test cannot read it back through ItemInfo. Splitting the formatting out is the same move
// FilterSummary and FormatSamplingSegment already make in this repo — the test asserts the string
// this function returns and separately drives the real widget to prove the render site reads live
// state.
//
// The degree sign is written as its UTF-8 bytes rather than a literal character, matching
// overlay_labels.cpp: the font atlas is built over the default (Latin-1) glyph range, so the glyph
// is present, but keeping the escape form makes the source encoding-independent.
std::string FormatSunTreeMeta(float altitude) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%.1f\xC2\xB0", static_cast<double>(altitude));
  return buf;
}

// lens_type indexes kLensTypeNames directly, as every other consumer of the field does
// (app_panels.cpp's status bar, field_editor_registry.cpp's combo): the value is only ever written
// by a combo over that same array, so its domain is closed.
std::string FormatCameraTreeMeta(int lens_type) {
  return kLensTypeNames[lens_type];
}

std::string FormatLayerTreeMeta(float probability) {
  char buf[32];
  snprintf(buf, sizeof(buf), "P %.2f", static_cast<double>(probability));
  return buf;
}

std::string FormatCrystalTreeMeta(float proportion) {
  char buf[32];
  snprintf(buf, sizeof(buf), "w %.0f", static_cast<double>(proportion));
  return buf;
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
//
// `total_width > 0` overrides the "fill the content region" default. A horizontal toolbar row (the
// top bar's execution cluster) has no meaningful content region: everything after a SameLine still
// reports the distance to the window's right edge, so a slider sized from it would swallow the rest
// of the row. Passing an explicit width is the only way to place these controls side by side; the
// 0.0f default keeps every panel/table call site byte-for-byte unchanged.
static float PrepareSliderLayout(const char* label, char* display_label_out, size_t display_buf_size, char* slider_id,
                                 size_t slider_id_size, char* input_id, size_t input_id_size,
                                 bool reserve_label_col = true, float total_width = 0.0f) {
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
  float avail_w = total_width > 0.0f ? total_width : ImGui::GetContentRegionAvail().x;
  // With the trailing label: subtract kLabelColWidth + 2 SameLine spacings
  // (slider→input, input→label). Without it: only the slider→input spacing.
  float slider_w =
      reserve_label_col ? (avail_w - kInputWidth - kLabelColWidth - spacing * 2) : (avail_w - kInputWidth - spacing);
  if (slider_w < 40.0f)
    slider_w = 40.0f;
  return slider_w;
}

// Render the label text after slider + input. Dim, like every other field label in this app
// (PropertyRow's label column, app_panels.cpp's InlineFieldLabel): a name that never changes is
// what you read once to find the control, not what you come back to read.
static void FinishSliderLayout(const char* display_label) {
  ImGui::SameLine();
  ImGui::TextDisabled("%s", display_label);
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

// SliderInt + InputInt + label text, same layout as SliderWithInput.
// Returns true if value changed.
bool SliderIntWithInput(const char* label, int* value, int min_val, int max_val, bool trailing_label, bool* committed,
                        bool* active, float total_width) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id), trailing_label, total_width);

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

// ---- Ray budget: slider with an ∞ detent, plus the input box that is the only way to the top ----

namespace {
// Fraction of the ray-budget slider's track that carries the finite domain. The remaining tail is
// the ∞ detent. 0.88 leaves the detent about an eighth of the track — wide enough to be a target a
// user can hit deliberately, narrow enough that it does not eat the useful part of the domain.
//
// The boundary is deliberately CLOSED on the detent side (`pos >= kRaysFiniteSpan` means infinite),
// which makes the largest finite value strictly unreachable by dragging: the slider maps
// [0, kRaysFiniteSpan) onto [min, max), so every position it can report as finite is below max.
// That is not an accident of pixel quantisation to be tuned away — it is the property the input box
// exists to complement, and the one the AC2 pair of tests pins from both sides. ∞ changes the
// TERMINATION SEMANTICS rather than being "a very large number"
// (doc/gui-visual-language.md §4.5), so a detent that swallows the top of the numeric domain is
// correct only as long as the number itself stays typeable.
constexpr float kRaysFiniteSpan = 0.88f;
}  // namespace

bool RaysBudgetControl(GuiState& state, float total_width) {
  // The registry stays the single owner of the domain and the display format. Its applicability
  // predicate ("infinite rays is on, so no ray total applies") is deliberately NOT consumed here:
  // it means "grey this row out", which is right for a generic one-row-per-field editor (the
  // defaults panel renders it that way) and wrong for this control, whose whole point is that the
  // input box stays live while the detent is engaged.
  const FieldEditorConstraint rays_c = ConstraintFor("sim.ray_num_millions", state);
  const auto min_v = static_cast<float>(rays_c.min_value);
  const auto max_v = static_cast<float>(rays_c.max_value);

  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  // reserve_label_col=false: the trailing "Rays(M)" is still drawn (FinishSliderLayout below), but
  // it is not given a fixed column — this control lives in a horizontal toolbar, where the fixed
  // column exists to align nothing.
  const float slider_w = PrepareSliderLayout("Rays(M)", display_buf, sizeof(display_buf), slider_id, sizeof(slider_id),
                                             input_id, sizeof(input_id), /*reserve_label_col=*/false, total_width);

  const bool old_infinite = state.sim.infinite;
  const float old_value = state.sim.ray_num_millions;

  // The slider is driven through a normalized position rather than the field itself, because the
  // track carries two different kinds of thing (a number, and a mode) and only a position can
  // address both. sim.infinite + ray_num_millions remain the sole truth: `pos` is recomputed from
  // them every frame and never stored.
  float pos = old_infinite ? 1.0f : kRaysFiniteSpan * (old_value - min_v) / (max_v - min_v);

  // ImGui prints the slider's display string through ImFormatString(buf, size, fmt, value), so a
  // string with no conversion specifier is emitted verbatim — which is how the detent shows words
  // where the rest of the track shows a number.
  char slider_text[64];
  if (old_infinite) {
    snprintf(slider_text, sizeof(slider_text), "until stopped");
  } else {
    snprintf(slider_text, sizeof(slider_text), rays_c.fmt, static_cast<double>(old_value));
  }

  // The ray total the user had before this drag started, restored if the drag ends up in the
  // detent. Without it, reaching ∞ by dragging would destroy the number on the way past: an ImGui
  // slider writes a value from the pointer's absolute position on every frame of a drag, so a user
  // going from 5 M to "until stopped" would arrive with ~99 M in the input box and no way back to
  // the 5 they chose. That property — turn the budget off and on again and get it back — was the
  // "Infinite rays" checkbox's, and it does not stop mattering because the checkbox is gone.
  //
  // Widget-local drag scratch, not a second source of truth: it is written only while this slider
  // is being held, read only to undo what the same drag wrote, and never consulted otherwise.
  // There is one ray budget, hence one static.
  static float s_finite_before_drag = 0.0f;

  ImGui::PushItemWidth(slider_w);
  // NoInput: Ctrl+click text entry would write the raw normalized position into the field, bypassing
  // the mapping entirely. The input box beside it is the text path.
  const bool slider_moved = ImGui::SliderFloat(slider_id, &pos, 0.0f, 1.0f, slider_text,
                                               ImGuiSliderFlags_NoInput | ImGuiSliderFlags_AlwaysClamp);
  // Queried after the call, so `old_value` is still the value from before this frame's click — on
  // the activation frame ImGui has already jumped the position to wherever the pointer landed, but
  // it cannot have touched the field, which only this block writes.
  if (ImGui::IsItemActivated()) {
    s_finite_before_drag = old_infinite ? old_value : std::clamp(old_value, min_v, max_v);
  }
  if (slider_moved) {
    if (pos >= kRaysFiniteSpan) {
      state.sim.infinite = true;
      state.sim.ray_num_millions = s_finite_before_drag;
    } else {
      state.sim.infinite = false;
      state.sim.ray_num_millions = min_v + (pos / kRaysFiniteSpan) * (max_v - min_v);
    }
  }
  ImGui::PopItemWidth();
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Total rays across all spectrum wavelengths, in millions.\n"
        "The server distributes the total to each wavelength as\n"
        "ceil(total / N_wavelengths) per wavelength.\n"
        "Drag to the far right for \"until stopped\" -- that is a termination\n"
        "mode, not a large number. Type the exact maximum in the box instead.");
  }

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  ImGui::InputFloat(input_id, &state.sim.ray_num_millions, 0, 0, rays_c.fmt);
  const bool input_committed = ImGui::IsItemDeactivatedAfterEdit();
  ImGui::PopItemWidth();
  state.sim.ray_num_millions = std::clamp(state.sim.ray_num_millions, min_v, max_v);
  if (input_committed) {
    // Typing a ray total IS the explicit statement that the run has one, so it leaves the detent.
    // This is also the only route to the largest finite value, which the detent has swallowed.
    state.sim.infinite = false;
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Exact ray total. Typing here leaves \"until stopped\".");
  }

  FinishSliderLayout(display_buf);
  return state.sim.infinite != old_infinite || state.sim.ray_num_millions != old_value;
}

// ---- Shared combo-popup fix (see panels.hpp) ----
//
// Mark the next combo's popup viewport as TopMost so it shares NSWindow level
// with the modal when the modal is detached into its own OS viewport. Without
// this, combo popups default to normal level (0) while the detached modal sits
// at NSFloatingWindowLevel (3, set via the modal's own SetNextWindowClass /
// ImGuiViewportFlags_TopMost on a detached modal), causing the popup to render
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
// CALLER CONTRACT (layout): this emits property ROWS, not a self-contained block — the caller owns
// the BeginPropertyTable / EndPropertyTable around it. That is not an accident of factoring: with
// one table per axis, the three axes would each compute their own column widths, and the label
// column would only line up by coincidence. One table for all three is what makes them one form.
//
// CALLER CONTRACT (combo popup): when invoked from a context where the parent window may become a
// detached OS viewport (currently: the inspector page in multi-viewport mode), the caller must
// precede this call with `SetNextComboPopupTopMost()`. Without that, the internal `##dist` Combo's
// popup defaults to NSWindow layer=0 and gets rendered behind the modal at layer=3.
//
// IMPLEMENTATION CONTRACT: this function MUST NOT introduce a `Begin` / `BeginChild` call before
// the `##dist` Combo — doing so would consume the caller's queued NextWindowData (WindowClass)
// prematurely. If layout wrapping is ever required, the SetNextComboPopupTopMost call must be moved
// inside RenderAxisDist (after any wrapping Begin/BeginChild, immediately before the Combo).
// The PropertyRow below is NOT such a wrap: it emits TableNextRow / TableNextColumn / a text item
// / SetNextItemWidth, none of which touch NextWindowData. The enclosing BeginPropertyTable is not
// one either — ImGui's BeginTable only opens a child window when ScrollX/ScrollY is set
// (imgui_tables.cpp, `use_child_window`), and BeginPropertyTable deliberately sets neither. Both
// checked against the vendored ImGui when the property rows landed.
bool RenderAxisDist(const char* label, AxisDist& axis, float mean_min, float mean_max) {
  bool changed = false;
  ImGui::PushID(label);
  // The axis name labels the distribution-TYPE row: "what kind of spread does Zenith have", with
  // its parameters on the rows below it.
  PropertyRow(label);

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

  PropertyRow("Mean");
  changed |= DragFloatField("Mean", &axis.mean, mean_min, mean_max);

  // The spread parameter is one field under four names — which name it wears is the distribution's
  // to say, so the row is emitted inside the switch rather than labelled after it.
  switch (axis.type) {
    case AxisDistType::kGauss:
    case AxisDistType::kGaussLegacy:
      PropertyRow("Std");
      changed |= DragFloatField("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kUniform:
      PropertyRow("Range");
      changed |= DragFloatField("Range", &axis.std, 0.0f, 360.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kZigzag:
      PropertyRow("Amplitude");
      changed |= DragFloatField("Amplitude", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kLaplacian:
      PropertyRow("Scale");
      changed |= DragFloatField("Scale", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt);
      break;
    default:
      PropertyRow("Std");
      changed |= DragFloatField("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt);
      break;
  }

  ImGui::PopID();
  return changed;
}


void ShapeTableParamLabel(const char* label) {
  // Dim: this is the shape table's label column, which plays the part PropertyRow's label cell
  // plays everywhere else. TextDisabled takes no length-delimited overload, so the "##" cut is made
  // into a buffer first rather than by passing an end pointer.
  const char* hash_pos = strstr(label, "##");
  if (hash_pos == nullptr) {
    ImGui::TextDisabled("%s", label);
    return;
  }
  char display_buf[64];
  const std::size_t len = std::min(static_cast<std::size_t>(hash_pos - label), sizeof(display_buf) - 1);
  std::memcpy(display_buf, label, len);
  display_buf[len] = '\0';
  ImGui::TextDisabled("%s", display_buf);
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

  // Col 1 — center value: one DragFloat filling the (stretch) Value column. The name already
  // occupies Col 0, so the control carries no label of its own.
  ImGui::TableNextColumn();
  ImGui::SetNextItemWidth(-FLT_MIN);
  changed |= DragFloatField(label, &dist.center, center_min, center_max, center_fmt, center_scale);

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


// ---- Selection reset ----

// Kept as a no-op rather than deleted, and the name is now doubly wrong — it reset neither a
// pending delete (that went years ago) nor, since the edit modal retired, an edit request. What it
// still is, is the one symbol every gui_test teardown calls to say "the left panel owns nothing
// across cases". Deleting it means editing every one of those teardowns; leaving it means the day
// this panel acquires cross-case state again, there is already a place for it. That day is the
// only thing that justifies the symbol, so if it has not come by the next cleanup pass, delete it.
void ResetPendingDeleteState() {}


// ========== Document tree ==========
//
// The left column's master half (doc/gui-layout-architecture.md §2). What used to be here was a
// list of four-line cards, each carrying three "Edit" buttons that opened a blocking modal. The
// cards are gone: a tree row's job is to NAME an item and to say which one is being edited, and
// everything that used to need a button now happens because the row is selected and the inspector
// below is showing it.
//
// Row identity and the two gestures that must not collide. A layer row can be folded (its
// crystals appear or disappear) and it can be selected (the inspector shows its probability).
// These are different questions and ImGui already separates them:
// ImGuiTreeNodeFlags_OpenOnArrow routes a click on the triangle to folding and a click anywhere
// else on the row to selection, with ImGui::IsItemToggledOpen() telling the two apart afterwards.
// Rolling our own hit test here would be re-deriving that.

namespace {

// A tree row is one frame tall, and its thumbnail is a square of that height. Deriving the row
// height from the font rather than fixing it in pixels is what keeps the tree legible if the
// theme's font size changes — the same reason the cards used GetFrameHeightWithSpacing().
float TreeRowHeight() {
  return ImGui::GetFrameHeight();
}

// Draw the crystal thumbnail for `crystal_id` as a `side`-pixel square at the current cursor, and
// advance the cursor past it. A crystal whose thumbnail has not been rendered yet gets the same
// grey placeholder the cards used, so a row never collapses to a different height while the
// thumbnail queue catches up.
void DrawRowThumbnail(int crystal_id, float side) {
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 br(p.x + side, p.y + side);
  if (const auto tex = g_thumbnail_cache.GetTexture(crystal_id); tex != 0) {
    // OpenGL texture Y-axis is flipped relative to ImGui: uv0=(0,1) uv1=(1,0)
    dl->AddImage(static_cast<ImTextureID>(tex), p, br, ImVec2(0, 1), ImVec2(1, 0));
  } else {
    dl->AddRectFilled(p, br, IM_COL32(60, 60, 60, 255));
  }
  dl->AddRect(p, br, IM_COL32(100, 100, 100, 255));
  ImGui::Dummy(ImVec2(side, side));
}

// Whether this entry may be clicked to complete an in-flight pick. Two entries cannot be linked to
// themselves, and an entry already sharing the source's ids would be a no-op — both were disabled
// as click targets on the cards and stay disabled here.
bool PickTargetDisabled(const GuiState& state, int layer_idx, int entry_idx) {
  if (!state.pick_link_source.has_value()) {
    return false;
  }
  const auto& src_ref = *state.pick_link_source;
  if (src_ref.layer_idx == layer_idx && src_ref.entry_idx == entry_idx) {
    return true;  // can't link to self
  }
  if (src_ref.layer_idx < 0 || src_ref.layer_idx >= static_cast<int>(state.layers.size()) || src_ref.entry_idx < 0 ||
      src_ref.entry_idx >= static_cast<int>(state.layers[src_ref.layer_idx].entries.size())) {
    return false;
  }
  const auto& src_entry = state.layers[src_ref.layer_idx].entries[src_ref.entry_idx];
  const auto& entry = state.layers[layer_idx].entries[entry_idx];
  return entry.crystal_id == src_entry.crystal_id && entry.filter_id == src_entry.filter_id;
}

// A fixed, index-free row for one of the document's singletons. Returns nothing: the click writes
// the selection directly, because there is no deferred-deletion dance to sequence it with.
//
// `meta` is the dimmed secondary value drawn at the row's right edge. It is drawn as absolutely
// positioned text OVER the Selectable rather than appended to `label`, so the row's id — which
// gui_test paths name literally ("**/" ICON_FA_SUN " Sun") — stays a constant while the value it
// previews changes every frame. This is the same overlay-on-Selectable handling RenderEntryRow uses
// for its badges; RenderLayer below instead extends its existing SameLine chain, because that row
// already positions its delete button that way. The two mechanisms are each matched to the row they
// live in, deliberately — not an inconsistency to unify.
void RenderSingletonRow(GuiState& state, const char* label, GuiState::SelectionKind kind, const char* meta) {
  const bool selected = state.selection.kind == kind;
  const ImVec2 row_pos = ImGui::GetCursorScreenPos();
  if (ImGui::Selectable(label, selected)) {
    state.selection = GuiState::DocumentSelection{ kind, -1, -1 };
  }
  if (meta == nullptr || meta[0] == '\0') {
    return;
  }
  // Save the cursor the Selectable left behind and restore it after: everything below relies on the
  // normal row flow, and a stray absolute position here would land the next row on top of this one.
  const ImVec2 next_pos = ImGui::GetCursorScreenPos();
  const float right_edge = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
  const float meta_w = ImGui::CalcTextSize(meta).x;
  ImGui::SetCursorScreenPos(ImVec2(right_edge - meta_w, row_pos.y));
  ImGui::TextDisabled("%s", meta);
  ImGui::SetCursorScreenPos(next_pos);
}

}  // namespace

bool RenderEntryRow(GuiState& state, int layer_idx, int entry_idx) {
  auto& entry = state.layers[layer_idx].entries[entry_idx];
  const bool pick_active = state.pick_link_source.has_value();
  const bool pick_disabled = PickTargetDisabled(state, layer_idx, entry_idx);

  ImGui::PushID(entry_idx);

  const float row_h = TreeRowHeight();
  const float spacing_x = ImGui::GetStyle().ItemSpacing.x;
  const ImVec2 row_pos = ImGui::GetCursorScreenPos();

  // The row's hit target is a full-width Selectable drawn FIRST, with AllowOverlap so the
  // thumbnail, the labels and the hover buttons that follow can all sit on top of it and still be
  // hit-tested in preference to it. Drawing it first also means its selected/hovered fill is the
  // row's background rather than a rectangle over the content.
  //
  // While a pick is in flight, a row that cannot be the target is disabled rather than merely
  // inert: a click landing on it would otherwise fall through to the blank-area handler and cancel
  // the pick, which reads as "the click did nothing and also lost my pick mode".
  const bool selected = state.selection.kind == GuiState::SelectionKind::kCrystal &&
                        state.selection.layer_idx == layer_idx && state.selection.entry_idx == entry_idx;
  if (pick_active && pick_disabled) {
    ImGui::BeginDisabled();
  }
  // The label carries the indices even though PushID above already makes the id unique. The
  // difference is addressability: an id that differs only by a pushed integer cannot be named in a
  // gui_test path, so every row would answer to the same wildcard and the first one would win. The
  // buttons on the card this row replaced carried their indices for the same reason.
  char row_id[32];
  snprintf(row_id, sizeof(row_id), "##row_%d_%d", layer_idx, entry_idx);
  ImGui::Selectable(row_id, selected, ImGuiSelectableFlags_AllowOverlap, ImVec2(0.0f, row_h));
  // BOTH things a click on this row can mean are read off the PRESS, and they have to be the same
  // event for the two of them to be exclusive.
  //
  // Completing a pick has to be the press. The blank-area cancel in RenderDocumentTree tests
  // IsMouseClicked, which is a press, and it runs after these rows in the same frame: a
  // release-triggered completion is therefore always beaten to it by the cancel, and every pick
  // ends as "nothing happened and I lost pick mode". The cards this row replaced hit-tested the
  // press for the same reason, which is why that defect only appeared when the hand-rolled hit test
  // became a real widget.
  //
  // Selecting then has to be the press as well — not because selection cares, but because mixing
  // the two events makes them non-exclusive across a frame boundary. ImGui delivers press and
  // release on separate frames, and pick mode ends on the press: so the release of the very click
  // that completed a link arrives with pick_active already false and falls into the select branch,
  // moving the selection onto the row that was merely clicked as a MODEL. The link is applied and
  // the user is then editing the wrong entry — silently, and only when press and release happen to
  // straddle two frames, which is why it survived a suite that passed.
  const bool row_pressed = ImGui::IsItemClicked(ImGuiMouseButton_Left);
  if (pick_active && pick_disabled) {
    ImGui::EndDisabled();
  }
  if (pick_active) {
    if (row_pressed) {
      // "Link A to B" semantics: A (the entry whose inspector opened the picker — pick_link_source)
      // adopts B's (the clicked row's) crystal/filter ids. So in ApplyPickLink(source, target) the
      // CLICKED row is the source (model) and pick_link_source is the target (modified).
      //
      // Effects are derived centrally by ReconcileGuiEffects: rebinding entry.filter_id shows up as
      // a `layers` diff (soft), and a filter presence-toggle (nullopt↔some) is caught by
      // AnyEntryFilterPresenceChanged (hard) — see gui_state_reconcile.cpp. Pure some(A)→some(B)
      // rebinding stays soft, matching pre-migration behavior.
      const auto pick_source_ref = *state.pick_link_source;
      ApplyPickLink(state, GuiState::EntryRef{ layer_idx, entry_idx }, pick_source_ref);
      state.pick_link_source.reset();
    }
  } else if (row_pressed) {
    state.SelectCrystal(layer_idx, entry_idx);
  }
  const bool row_hovered = ImGui::IsItemHovered();

  // ---- Row content, drawn over the Selectable ----
  ImGui::SetCursorScreenPos(row_pos);
  DrawRowThumbnail(entry.crystal_id, row_h);
  ImGui::SameLine(0.0f, spacing_x);

  // Identity text: the two things that distinguish one crystal from another at a glance, which is
  // what the card's first two rows said with a label column each. Vertically centred against the
  // thumbnail rather than sitting at its top edge.
  const CrystalConfig& crystal_ref = state.crystals[entry.crystal_id];
  const char* type_name = (crystal_ref.type == CrystalType::kPrism) ? "Prism" : "Pyramid";
  const std::string preset = AxisPresetName(crystal_ref);
  const float text_y = row_pos.y + (row_h - ImGui::GetTextLineHeight()) * 0.5f;
  ImGui::SetCursorScreenPos(ImVec2(ImGui::GetCursorScreenPos().x, text_y));
  ImGui::Text("%s · %s", type_name, preset.c_str());

  // ---- Right-edge badges and hover actions ----
  // Laid out from the right edge inward so the identity text keeps whatever is left. Delete and
  // duplicate are revealed by hover (alpha, not omission — the click paths stay in the ImGui tree
  // so hit-testing does not change between frames, which is what kept the cards' fast-swipe
  // mitigation stable); the link badge and the filter badge are persistent state indicators and
  // are always visible.
  const float frame_pad_x = ImGui::GetStyle().FramePadding.x;
  const float btn_w =
      std::max(ImGui::CalcTextSize(ICON_FA_COPY).x, ImGui::CalcTextSize(ICON_FA_XMARK).x) + frame_pad_x * 2.0f;
  const ImVec2 win_pos = ImGui::GetWindowPos();
  const float right_edge = win_pos.x + ImGui::GetWindowContentRegionMax().x;
  float x = right_edge - btn_w;
  const float btn_y = row_pos.y + (row_h - ImGui::GetFrameHeight()) * 0.5f;

  char dup_id[32];
  char del_id[32];
  snprintf(dup_id, sizeof(dup_id), ICON_FA_COPY "##dup_%d_%d", layer_idx, entry_idx);
  snprintf(del_id, sizeof(del_id), ICON_FA_XMARK "##del_%d_%d", layer_idx, entry_idx);

  bool delete_clicked = false;
  const bool can_delete_entry = state.layers[layer_idx].entries.size() > 1;
  ImGui::PushStyleVar(ImGuiStyleVar_Alpha, row_hovered ? 1.0f : 0.0f);
  ImGui::SetCursorScreenPos(ImVec2(x, btn_y));
  if (can_delete_entry) {
    PushDestructiveStyle();
  } else {
    ImGui::BeginDisabled();
  }
  delete_clicked = ImGui::SmallButton(del_id);
  if (can_delete_entry) {
    PopDestructiveStyle();
  } else {
    ImGui::EndDisabled();
  }
  x -= btn_w + kHoverBtnGap;
  ImGui::SetCursorScreenPos(ImVec2(x, btn_y));
  const bool dup_clicked = ImGui::SmallButton(dup_id);
  ImGui::PopStyleVar();

  if (dup_clicked) {
    // Duplicate = clone-to-pool: append new CrystalConfig (and new FilterConfig if present) so the
    // dup'd entry is fully independent. Capture pool copies BEFORE push_back to avoid dangling
    // references if the vector reallocates.
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
    state.layers[layer_idx].entries.push_back(new_entry);
    g_thumbnail_cache.OnLayerStructureChanged();
  }

  // Sharing badge: (crystal_id, filter_id) referenced by 2+ entries across all layers, this one
  // included. Persistent, not hover-revealed — it says something about the document, not about
  // what the pointer is over.
  const int shared = CountEntriesSharing(state, entry.crystal_id, entry.filter_id);
  if (shared >= 2) {
    x -= ImGui::CalcTextSize(ICON_FA_LINK).x + kHoverBtnGap * 2.0f;
    ImGui::SetCursorScreenPos(ImVec2(x, text_y));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 1.0f, 1.0f));
    ImGui::TextUnformatted(ICON_FA_LINK);
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) {
      std::string list;
      for (int li = 0; li < static_cast<int>(state.layers.size()); ++li) {
        for (int ei = 0; ei < static_cast<int>(state.layers[li].entries.size()); ++ei) {
          const auto& other = state.layers[li].entries[ei];
          if (other.crystal_id != entry.crystal_id || other.filter_id != entry.filter_id) {
            continue;
          }
          if (li == layer_idx && ei == entry_idx) {
            continue;  // skip self in the tooltip list
          }
          if (!list.empty()) {
            list += "\n";
          }
          list += "Layer " + std::to_string(li) + " / Entry " + std::to_string(ei);
        }
      }
      if (list.empty()) {
        list = "(no other entries)";
      }
      ImGui::SetTooltip("Shared with:\n%s", list.c_str());
    }
  }

  // Filter badge: the card spent a whole row on the filter's summary text; a compact row cannot,
  // and the inspector's Filter tab is one click away. What the row still has to say is WHETHER a
  // filter is attached at all — a crystal quietly filtered down to nothing looks, in the render,
  // exactly like a crystal that is not contributing for physical reasons. The summary itself moves
  // to the badge's tooltip, where it is complete rather than clipped.
  if (entry.filter_id.has_value()) {
    x -= ImGui::CalcTextSize(ICON_FA_FILTER).x + kHoverBtnGap * 2.0f;
    ImGui::SetCursorScreenPos(ImVec2(x, text_y));
    ImGui::TextUnformatted(ICON_FA_FILTER);
    if (ImGui::IsItemHovered()) {
      const auto& fc = state.filters[*entry.filter_id];
      if (fc.IsDegenerateSingleFactor()) {
        ImGui::SetTooltip("%s", FilterSummary(std::optional<FilterConfig>{ fc }).c_str());
      } else {
        ImGui::SetTooltip("%s", gui::FormatSopExpansionPreview(fc.param).c_str());
      }
    }
  }

  // Weight, right-aligned before the badges. Read-only here: the slider it replaces lives on the
  // inspector's crystal page, because a drag target inside a one-line selectable row competes with
  // the row's own click.
  {
    // Formatted by the same FormatCrystalTreeMeta the other three tree rows' meta comes from, so
    // the four rows' secondary values have one owner rather than one format string each. The "w "
    // prefix is what makes a bare number read as a weight without a click.
    const std::string weight_text = FormatCrystalTreeMeta(entry.proportion);
    x -= ImGui::CalcTextSize(weight_text.c_str()).x + kHoverBtnGap * 2.0f;
    ImGui::SetCursorScreenPos(ImVec2(x, text_y));
    ImGui::TextDisabled("%s", weight_text.c_str());
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Weight (relative ray share within this layer)");
    }
  }

  // Restore the layout cursor: everything after the Selectable was positioned absolutely, so the
  // row must hand the next row a cursor that sits below it rather than wherever the last badge
  // happened to land.
  ImGui::SetCursorScreenPos(ImVec2(row_pos.x, row_pos.y + row_h + ImGui::GetStyle().ItemSpacing.y));

  ImGui::PopID();
  return delete_clicked;
}


// ========== Layer ==========

void RenderLayer(GuiState& state, int layer_idx) {
  auto& layer = state.layers[layer_idx];

  ImGui::PushID(layer_idx);

  char header_label[32];
  snprintf(header_label, sizeof(header_label), "Layer %d", layer_idx + 1);

  // OpenOnArrow is what keeps folding and selecting apart: the triangle folds, the rest of the row
  // selects. SpanAvailWidth makes the selectable region the whole row rather than just the label's
  // text, so the two gestures partition the row instead of leaving dead space between them.
  const bool selected =
      state.selection.kind == GuiState::SelectionKind::kLayer && state.selection.layer_idx == layer_idx;
  ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
                             ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_DefaultOpen |
                             ImGuiTreeNodeFlags_AllowOverlap;
  if (selected) {
    flags |= ImGuiTreeNodeFlags_Selected;
  }
  const bool header_open = ImGui::TreeNodeEx("##layer", flags, "%s", header_label);
  // IsItemToggledOpen() is the discriminator ImGui provides for exactly this: it is true on the
  // frame the arrow (or a double click) changed the fold state, and false when the click was a
  // plain selection. Without it, folding a layer would also select it.
  if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
    state.SelectLayer(layer_idx);
  }

  // Right-aligned delete button on the header row. Only enabled when more than
  // one layer exists (the scattering model requires at least one layer).
  bool can_delete_layer = state.layers.size() > 1;
  float layer_del_w = ImGui::CalcTextSize(ICON_FA_XMARK).x + ImGui::GetStyle().FramePadding.x * 2.0f;

  // The layer's info scent — its multi-scatter probability — sits just left of the delete button.
  // It extends the SameLine chain this row already uses rather than being positioned absolutely
  // like the singleton rows' meta: the delete button below is placed by an absolute SameLine offset
  // that this insertion leaves numerically untouched, so the button does not move.
  const std::string layer_meta = FormatLayerTreeMeta(layer.probability);
  const float layer_meta_w = ImGui::CalcTextSize(layer_meta.c_str()).x;
  ImGui::SameLine(ImGui::GetContentRegionMax().x - layer_del_w - ImGui::GetStyle().ItemSpacing.x - layer_meta_w);
  ImGui::TextDisabled("%s", layer_meta.c_str());

  ImGui::SameLine(ImGui::GetContentRegionMax().x - layer_del_w);
  if (can_delete_layer) {
    PushDestructiveStyle();
  } else {
    ImGui::BeginDisabled();
  }
  char layer_del_id[32];
  snprintf(layer_del_id, sizeof(layer_del_id), ICON_FA_XMARK "##layer_%d", layer_idx);
  bool layer_delete_clicked = ImGui::SmallButton(layer_del_id);
  if (can_delete_layer) {
    PopDestructiveStyle();
  } else {
    ImGui::EndDisabled();
  }
  if (layer_delete_clicked && can_delete_layer) {
    state.layers.erase(state.layers.begin() + layer_idx);
    g_thumbnail_cache.OnLayerStructureChanged();
    // The selection may now name an entry in a layer that no longer exists, or — worse, because it
    // stays in range — an entry that shifted under it. Clearing is the only answer that cannot be
    // wrong; re-pointing it at "the layer that took this index" would be inventing an intent the
    // user did not express.
    if (state.selection.layer_idx == layer_idx || state.selection.layer_idx > layer_idx) {
      state.SelectNone();
    }
    if (header_open) {
      ImGui::TreePop();
    }
    ImGui::PopID();
    return;  // Skip rendering the rest; layer has been erased.
  }

  if (header_open) {
    // Render entry rows with deferred deletion
    int pending_delete_entry = -1;
    for (int i = 0; i < static_cast<int>(layer.entries.size()); i++) {
      bool del = RenderEntryRow(state, layer_idx, i);
      if (del) {
        pending_delete_entry = i;
      }
    }

    // Deferred delete
    if (pending_delete_entry >= 0 && layer.entries.size() > 1) {
      layer.entries.erase(layer.entries.begin() + pending_delete_entry);
      g_thumbnail_cache.OnLayerStructureChanged();
      // Same reasoning as the layer-delete case above: an entry index at or past the deleted one
      // now names a different entry.
      if (state.selection.kind == GuiState::SelectionKind::kCrystal && state.selection.layer_idx == layer_idx &&
          state.selection.entry_idx >= pending_delete_entry) {
        state.SelectNone();
      }
    }

    // Add entry button. Bind the new entry to a fresh pool slot so it is
    // independent by default — using EntryCard's default (crystal_id = 0)
    // would silently link the new entry to whichever entry already references
    // slot 0, making +Crystal look like "implicit Link to entry 0".
    char add_id[32];
    snprintf(add_id, sizeof(add_id), "+ Crystal##layer_%d", layer_idx);
    if (ImGui::SmallButton(add_id)) {
      EntryCard new_entry;
      new_entry.crystal_id = static_cast<int>(state.crystals.size());
      state.crystals.emplace_back();
      layer.entries.push_back(new_entry);
      g_thumbnail_cache.OnLayerStructureChanged();
      // Selecting what was just created is the point of creating it — otherwise the inspector goes
      // on showing the previous crystal and the new one has to be hunted for in the tree.
      state.SelectCrystal(layer_idx, static_cast<int>(layer.entries.size()) - 1);
    }
    ImGui::TreePop();
  }

  ImGui::PopID();
}


// ========== Layer inspector page ==========

bool RenderLayerInspector(GuiState& state, int layer_idx) {
  auto& layer = state.layers[layer_idx];

  ImGui::PushID(layer_idx);

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
  const bool is_last_layer = layer_idx == static_cast<int>(state.layers.size()) - 1;
  const bool prob_is_zero = IsProbZero(layer.probability);
  const bool disable_slider = is_last_layer && prob_is_zero;
  const bool show_warning_icon = (is_last_layer && !prob_is_zero) || (!is_last_layer && prob_is_zero);
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
  char prob_id[32];
  snprintf(prob_id, sizeof(prob_id), "Prob.##layer_%d", layer_idx);
  if (BeginPropertyTable("##layer_props")) {
    PropertyRow("Prob.");
    if (show_warning_icon) {
      // Named exception to "the control fills its column": the warning glyph shares the cell, so
      // the control gives back exactly the glyph's own advance plus one spacing. Reserved rather
      // than a fixed number so it tracks the font — same formula as the filter editor's delete
      // column (RenderSummandRowList, edit_modals.cpp).
      ImGui::SetNextItemWidth(-(ImGui::CalcTextSize(ICON_FA_CIRCLE_EXCLAMATION).x + ImGui::GetStyle().ItemSpacing.x));
    }
    ImGui::BeginDisabled(disable_slider);
    DragFloatField(prob_id, &layer.probability, 0.0f, 1.0f, "%.2f");
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("%s", prob_tip);
    }
    if (show_warning_icon) {
      ImGui::SameLine();
      ImGui::TextColored(WarningTextColor(), ICON_FA_CIRCLE_EXCLAMATION);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", prob_tip);
      }
    }
    EndPropertyTable();
  }

  // How many crystals this layer holds. The tree above shows them one per row, but a folded layer
  // shows none, and this page is reachable with the layer folded.
  const int entry_count = static_cast<int>(layer.entries.size());
  ImGui::TextDisabled("%d crystal%s", entry_count, entry_count == 1 ? "" : "s");

  ImGui::Separator();

  // Deleting the layer from here rather than only from its tree row: the inspector is where the
  // layer's own properties live, and the tree row's × is easy to miss at one frame of height.
  // Returned to the caller rather than erased in place — the caller owns the selection fix-up, and
  // doing it here would leave `layer` dangling for the rest of this function.
  const bool can_delete_layer = state.layers.size() > 1;
  if (!can_delete_layer) {
    ImGui::BeginDisabled();
  } else {
    PushDestructiveStyle();
  }
  const bool delete_clicked = ImGui::SmallButton(ICON_FA_XMARK " Delete layer");
  if (!can_delete_layer) {
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("The scattering model requires at least one layer.");
    }
  } else {
    PopDestructiveStyle();
  }

  ImGui::PopID();
  return delete_clicked && can_delete_layer;
}


// ========== Document tree rows ==========

void RenderDocumentTreeRows(GuiState& state) {
  // The document's two singletons come first, in the order the light travels: the sun makes the
  // rays, the camera sees them, and everything between is the scattering stack below.
  //
  // The dimmed right-edge value is the row's "info scent": the tree says WHICH knobs exist, and
  // this says where each is currently set, so the common check needs no click into the inspector.
  const std::string sun_meta = FormatSunTreeMeta(state.sun.altitude);
  const std::string camera_meta = FormatCameraTreeMeta(state.renderer.lens_type);
  RenderSingletonRow(state, ICON_FA_SUN " Sun", GuiState::SelectionKind::kSun, sun_meta.c_str());
  RenderSingletonRow(state, ICON_FA_CAMERA " Camera", GuiState::SelectionKind::kCamera, camera_meta.c_str());
  ImGui::Separator();

  for (int i = 0; i < static_cast<int>(state.layers.size()); i++) {
    RenderLayer(state, i);
    // A layer erased mid-loop shortens the vector under us; re-read rather than trusting the
    // cached bound. (RenderLayer returns early after erasing, so at most one goes per frame.)
    if (i >= static_cast<int>(state.layers.size())) {
      break;
    }
  }
}


// ========== Sun controls (the inspector's Sun page) ==========

void RenderSunControls(GuiState& state) {
  if (!BeginPropertyTable("##sun_props")) {
    return;
  }
  // AC2 migration path (scrum-gui-state-reconcile T0): widget only writes state.sun.altitude; the
  // resulting dirty is derived by ReconcileGuiEffects in SyncFromPoller (diff on state.sun vs.
  // last_committed_state.sun). The pre-migration DIRTY_IF wrapper is retired at this call site.
  // Domain and format read from the field editor registry, not written here. `fmt` is passed
  // explicitly even though it currently equals DragFloatField's own default: relying on the
  // default would leave the display precision as a second statement this call site makes on its
  // own, which is the thing being removed.
  const FieldEditorConstraint alt_c = ConstraintFor("sun.altitude", state);
  PropertyRow("Altitude");
  DragFloatField("Altitude", &state.sun.altitude, static_cast<float>(alt_c.min_value),
                 static_cast<float>(alt_c.max_value), alt_c.fmt, alt_c.scale);
  // The tooltip now hangs off the control alone rather than a group spanning control + label: the
  // label lives in the other column, and a group cannot span two table cells.
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Sun elevation angle above the horizon");
  }
  // AC2 migration path: same rationale as sun.altitude above.
  const FieldEditorConstraint dia_c = ConstraintFor("sun.diameter", state);
  PropertyRow("Diameter");
  DragFloatField("Diameter", &state.sun.diameter, static_cast<float>(dia_c.min_value),
                 static_cast<float>(dia_c.max_value), dia_c.fmt, dia_c.scale);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Angular diameter of the sun disk");
  }
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
  PropertyRow("Spectrum");
  if (ImGui::Combo("##Spectrum", &combo_sel, kSpectrumComboItems, kSpectrumComboItemCount)) {
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
  if (state.sun.spectrum_index == kCustomSpectrumIndex) {
    // A label-less row rather than a button below the table: the button acts on the row above it,
    // so lining it up under that row's control is what says so.
    PropertyRow("");
    if (ImGui::SmallButton("Edit spectrum...##spectrum_edit")) {
      OpenSpectrumModal(state);  // re-open editor for the existing custom spectrum
    }
  }
  EndPropertyTable();

  // The "Simulation" group that used to sit here — Infinite rays / Rays(M) / Max hits / Use GPU —
  // now lives in the top bar's execution cluster (RenderExecutionCluster, app_panels.cpp). It was
  // moved because those fields answer "how hard does THIS RUN go", which is the Run button's
  // question, not the document's: everything left in this function is saved with the scene, and
  // nothing that was moved is (see doc/gui-layout-architecture.md §1/§3). Do not add run-budget
  // controls back here.
}

}  // namespace lumice::gui
