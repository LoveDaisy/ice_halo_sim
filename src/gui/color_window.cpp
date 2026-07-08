// task-342.3 Steps 5-9: Colors window.
//
// Non-modal floating panel driving the GuiState::raypath_color pool. Two write
// paths keep re-simulation vs. display-only edits decoupled (plan §3 decision 2):
//
//   Display-time (immediate, no epoch bump)      → LUMICE_SetRaypathColors
//     color / visible / solo / z_order / composite mode
//
//   Structural (dirty→next debounce commit)      → state.MarkFilterDirty()
//     predicate text / match_all toggle / combine any↔all / add/remove class or ref
//
// z_order (display) is strictly decoupled from the physical vector index
// (plan §3 decision 1). Reordering (up/down buttons) swaps z_order values
// only; the vector stays put so GetColorClassLaneY(i) keeps binding to the
// same class. NEVER std::swap raypath_color[] entries directly.

#include "gui/color_window.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/gui_logger.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "imgui.h"
#include "include/lumice.h"

namespace lumice::gui {

namespace {

// Rebuild the display-only arrays that LUMICE_SetRaypathColors consumes and
// push them to the server. Called on every display-time edit (see write-path
// table in file header). LUMICE_SetRaypathColors is whole-table — a single
// row change still forces a full rebuild of classes[]/z_order[].
//
// Returns true on LUMICE_OK; on failure the caller may want to log — we only
// warn once here rather than pollute the log with a per-frame error stream.
bool PushDisplayState(const GuiState& state, LUMICE_Server* server) {
  if (server == nullptr || state.raypath_color.empty()) {
    return false;
  }
  const int n = static_cast<int>(state.raypath_color.size());
  std::vector<LUMICE_ColorClassDisplay> classes(static_cast<size_t>(n));
  std::vector<int> z_order(static_cast<size_t>(n));
  for (int i = 0; i < n; i++) {
    const auto& cls = state.raypath_color[i];
    LUMICE_ColorClassDisplay& d = classes[static_cast<size_t>(i)];
    d.color[0] = cls.color[0];
    d.color[1] = cls.color[1];
    d.color[2] = cls.color[2];
    d.visible = cls.visible ? 1 : 0;
    d.solo = cls.solo ? 1 : 0;
    z_order[static_cast<size_t>(i)] = cls.z_order;
  }
  const LUMICE_ErrorCode ec =
      LUMICE_SetRaypathColors(server, classes.data(), n, z_order.data(), state.raypath_color_mode);
  if (ec != LUMICE_OK) {
    // Most likely cause: user structural edit hasn't debounced yet, so the
    // server's committed class_count doesn't match state.raypath_color.size().
    // The next commit + subsequent PushDisplayState will heal it — this warning
    // is expected during the settling window and should not be spammed.
    GUI_LOG_DEBUG("[Colors] LUMICE_SetRaypathColors returned {} (class_count={}); waiting for next commit.",
                  static_cast<int>(ec), n);
    return false;
  }
  return true;
}

// Poll GetColorClassSignal into a caller-owned buffer sized to
// state.raypath_color.size(). Buffer is resized here so callers can pass an
// empty vector on the first call.
void PollColorClassSignal(const GuiState& state, LUMICE_Server* server, std::vector<int>& out_flags) {
  const int n = static_cast<int>(state.raypath_color.size());
  out_flags.assign(static_cast<size_t>(n), 0);
  if (server == nullptr || n == 0) {
    return;
  }
  const LUMICE_ErrorCode ec = LUMICE_GetColorClassSignal(server, out_flags.data(), n);
  if (ec != LUMICE_OK) {
    // Same class_count mismatch story as PushDisplayState — settle silently.
    std::fill(out_flags.begin(), out_flags.end(), 0);
  }
}

// Human-readable summary of a class's match[] used for the collapsed row label.
// Rebuilt from state on every render (a05 减法 — no cache field).
std::string BuildClassSummary(const GuiState& state, const ColorClassConfig& cls) {
  if (cls.match.empty()) {
    return "(no members)";
  }
  const char* sep = (cls.combine == LUMICE_COLOR_COMBINE_ALL) ? " AND " : " OR ";
  std::string out;
  for (size_t i = 0; i < cls.match.size(); i++) {
    if (i > 0) {
      out += sep;
    }
    const auto& ref = cls.match[i];
    char layer_tag[16];
    std::snprintf(layer_tag, sizeof(layer_tag), "L%d", ref.layer_idx);
    out += layer_tag;
    if (ref.crystal_pool_id >= 0 && static_cast<size_t>(ref.crystal_pool_id) < state.crystals.size()) {
      const auto& cr = state.crystals[static_cast<size_t>(ref.crystal_pool_id)];
      out += "·";
      out += cr.name.empty() ? std::string("crystal#") + std::to_string(ref.crystal_pool_id) : cr.name;
    } else {
      out += "·<missing>";
    }
    if (ref.match_all) {
      out += " · whole";
    } else if (!ref.predicate_text.empty()) {
      out += " · ";
      out += ref.predicate_text;
    }
  }
  return out;
}

// Reassign z_order[] to a compact permutation [0, size) preserving the current
// user-visible ordering. Called after delete-class (which may leave holes) so
// the invariant expected by LUMICE_SetRaypathColors (z_order is a permutation)
// keeps holding.
void CompactZOrder(GuiState& state) {
  const size_t n = state.raypath_color.size();
  std::vector<size_t> phys_by_rank(n);
  for (size_t i = 0; i < n; i++) {
    phys_by_rank[i] = i;
  }
  std::stable_sort(phys_by_rank.begin(), phys_by_rank.end(),
                   [&](size_t a, size_t b) { return state.raypath_color[a].z_order < state.raypath_color[b].z_order; });
  for (size_t rank = 0; rank < n; rank++) {
    state.raypath_color[phys_by_rank[rank]].z_order = static_cast<int>(rank);
  }
}

// Swap the z_order values of two physical classes (identified by vector index).
// Guarded by CompactZOrder(): callers expect the pre-swap z_order values to be
// a permutation, which the compaction step ensures on every entry.
void SwapZOrder(GuiState& state, size_t a, size_t b) {
  if (a >= state.raypath_color.size() || b >= state.raypath_color.size() || a == b) {
    return;
  }
  std::swap(state.raypath_color[a].z_order, state.raypath_color[b].z_order);
}

// Import a GUI filter's SoP as ref[]s in a fresh ColorClassConfig. Multi-factor
// AND rows are skipped with a queued warning (LUMICE_ColorPredicate is a single
// atom; combine:all is cross-ref, not intra-ref). combine defaults to `any`
// (blueprint case B: "UI filter = one color class = OR of rows").
ColorClassConfig BuildClassFromFilter(int layer_idx, int crystal_pool_id, const FilterConfig& f, int& skipped_rows) {
  ColorClassConfig cls;
  cls.combine = LUMICE_COLOR_COMBINE_ANY;
  skipped_rows = 0;
  for (const auto& row : f.param) {
    if (row.factors.size() != 1) {
      skipped_rows++;
      continue;
    }
    ColorClassRefConfig ref;
    ref.layer_idx = layer_idx;
    ref.crystal_pool_id = crystal_pool_id;
    ref.match_all = row.text.empty();
    ref.predicate_text = row.text;
    cls.match.push_back(ref);
  }
  return cls;
}

// Aggregate every (layer, entry) placement in the scene as an import target.
struct PlacementRef {
  int layer_idx;
  int entry_idx;
  int crystal_pool_id;
  std::optional<int> filter_id;
};

std::vector<PlacementRef> CollectPlacementsWithFilter(const GuiState& state) {
  std::vector<PlacementRef> out;
  for (size_t li = 0; li < state.layers.size(); li++) {
    const auto& layer = state.layers[li];
    for (size_t ei = 0; ei < layer.entries.size(); ei++) {
      const auto& entry = layer.entries[ei];
      if (!entry.filter_id.has_value()) {
        continue;
      }
      PlacementRef p;
      p.layer_idx = static_cast<int>(li);
      p.entry_idx = static_cast<int>(ei);
      p.crystal_pool_id = entry.crystal_id;
      p.filter_id = entry.filter_id;
      out.push_back(p);
    }
  }
  return out;
}

// The user-side crystal picker enumerates all crystals actually referenced by
// entries in the given layer, deduped. Falls back to the empty list when the
// layer has no entries — the row keeps whatever crystal_pool_id was pre-set.
std::vector<int> CrystalPoolsInLayer(const GuiState& state, int layer_idx) {
  std::vector<int> out;
  if (layer_idx < 0 || static_cast<size_t>(layer_idx) >= state.layers.size()) {
    return out;
  }
  for (const auto& entry : state.layers[static_cast<size_t>(layer_idx)].entries) {
    if (std::find(out.begin(), out.end(), entry.crystal_id) == out.end()) {
      out.push_back(entry.crystal_id);
    }
  }
  return out;
}

const char* CrystalDisplayName(const GuiState& state, int pool_id) {
  static thread_local std::string buf;
  if (pool_id < 0 || static_cast<size_t>(pool_id) >= state.crystals.size()) {
    buf = "<missing>";
    return buf.c_str();
  }
  const auto& cr = state.crystals[static_cast<size_t>(pool_id)];
  if (!cr.name.empty()) {
    return cr.name.c_str();
  }
  buf = std::string("crystal#") + std::to_string(pool_id);
  return buf.c_str();
}

// Push item-scoped color states for the "empty arc" warning icon (amber-ish).
void PushWarningStyle() {
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.2f, 1.0f));
}
void PopWarningStyle() {
  ImGui::PopStyleColor();
}

// -------------------- predicate editor helpers --------------------

// Validate the user's per-ref text under the "single atom" (single Factor,
// single alternative) rule expressed by decision 3 in plan §3. Returns
// (state, message) so the row can render a red-tinted border + tooltip.
GuiValidationResult ValidateSingleAtomText(const std::string& text) {
  const std::string trimmed = TrimRaypathSegment(text);
  if (trimmed.empty()) {
    return GuiValidationResult{ LUMICE_RAYPATH_VALID, {} };
  }
  // Use PRISM as a permissive baseline for face-number legality: the color
  // window's ref carries no crystal-kind info at edit time (it points to a
  // pool id, and multiple placements of different kinds may share a class).
  // LUMICE_IsLegalFace guards the physical raypath filter separately at
  // commit time; here we only need syntactic validation + the single-atom rule.
  auto base = ValidateSummandText(trimmed, LUMICE_CRYSTAL_PRISM);
  if (base.state != LUMICE_RAYPATH_VALID) {
    return base;
  }
  const auto factors = ParseSummandText(trimmed);
  if (factors.size() != 1) {
    return GuiValidationResult{ LUMICE_RAYPATH_INVALID, "single-atom only — use combine:all across refs for AND" };
  }
  return GuiValidationResult{ LUMICE_RAYPATH_VALID, {} };
}

// -------------------- window body --------------------

void RenderCompositeModeCombo(GuiState& state, LUMICE_Server* server) {
  static const char* const kModeNames[] = { "dominant", "additive", "painter" };
  int mode = state.raypath_color_mode;
  if (mode < 0 || mode >= 3) {
    mode = 0;
  }
  ImGui::TextUnformatted("Composite:");
  ImGui::SameLine();
  ImGui::PushItemWidth(120);
  if (ImGui::Combo("##ColorMode", &mode, kModeNames, 3)) {
    state.raypath_color_mode = mode;
    PushDisplayState(state, server);
  }
  ImGui::PopItemWidth();
}

void RenderImportFromFilterUI(GuiState& state) {
  const auto placements = CollectPlacementsWithFilter(state);
  const bool disabled = placements.empty();
  if (disabled) {
    ImGui::BeginDisabled();
  }
  const bool clicked = ImGui::Button(ICON_FA_FILE_IMPORT " Import from filter");
  if (disabled) {
    ImGui::EndDisabled();
  }
  if (disabled) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("No entries with a filter in the scene.");
    }
    return;
  }
  if (clicked) {
    ImGui::OpenPopup("Import from filter");
  }
  if (ImGui::BeginPopup("Import from filter")) {
    for (size_t i = 0; i < placements.size(); i++) {
      const auto& p = placements[i];
      const char* crystal_name = CrystalDisplayName(state, p.crystal_pool_id);
      const auto& f = state.filters[static_cast<size_t>(*p.filter_id)];
      char label[256];
      std::snprintf(label, sizeof(label), "L%d · %s · %s##imp_%zu", p.layer_idx, crystal_name,
                    f.name.empty() ? "filter" : f.name.c_str(), i);
      if (ImGui::Selectable(label)) {
        int skipped = 0;
        ColorClassConfig new_cls = BuildClassFromFilter(p.layer_idx, p.crystal_pool_id, f, skipped);
        new_cls.z_order = static_cast<int>(state.raypath_color.size());
        state.raypath_color.push_back(new_cls);
        state.MarkFilterDirty();
        if (skipped > 0) {
          char msg[256];
          std::snprintf(msg, sizeof(msg),
                        "%d AND-composite row(s) in filter '%s' were skipped when imported as a color class "
                        "(color predicates are single-atom; use combine:all across multiple refs instead).",
                        skipped, f.name.c_str());
          SetImportComplexFilterWarning(msg);
        }
        ImGui::CloseCurrentPopup();
      }
    }
    ImGui::EndPopup();
  }
}

// Render one member ref (structural edits go through MarkFilterDirty).
void RenderRefRow(GuiState& state, ColorClassConfig& cls, size_t ref_idx, bool& delete_this_ref) {
  ImGui::PushID(static_cast<int>(ref_idx));
  auto& ref = cls.match[ref_idx];

  // Layer combo.
  const int layer_count = static_cast<int>(state.layers.size());
  std::vector<std::string> layer_labels(static_cast<size_t>(layer_count));
  std::vector<const char*> layer_ptrs(static_cast<size_t>(layer_count));
  for (int i = 0; i < layer_count; i++) {
    layer_labels[static_cast<size_t>(i)] = "Layer " + std::to_string(i);
    layer_ptrs[static_cast<size_t>(i)] = layer_labels[static_cast<size_t>(i)].c_str();
  }
  int layer_idx = std::clamp(ref.layer_idx, 0, std::max(0, layer_count - 1));
  ImGui::PushItemWidth(90);
  if (ImGui::Combo("##layer", &layer_idx, layer_ptrs.data(), layer_count)) {
    ref.layer_idx = layer_idx;
    // Reset crystal choice to the new layer's first placement so we never
    // leave a stale (invalid-for-this-layer) pool id in the ref.
    const auto pools = CrystalPoolsInLayer(state, layer_idx);
    if (!pools.empty() && std::find(pools.begin(), pools.end(), ref.crystal_pool_id) == pools.end()) {
      ref.crystal_pool_id = pools.front();
    }
    state.MarkFilterDirty();
  }
  ImGui::PopItemWidth();

  // Crystal combo (deduped placements in the selected layer).
  ImGui::SameLine();
  const auto pools = CrystalPoolsInLayer(state, ref.layer_idx);
  std::vector<std::string> crystal_labels(pools.size());
  std::vector<const char*> crystal_ptrs(pools.size());
  int current_choice = -1;
  for (size_t i = 0; i < pools.size(); i++) {
    crystal_labels[i] = CrystalDisplayName(state, pools[i]);
    crystal_ptrs[i] = crystal_labels[i].c_str();
    if (pools[i] == ref.crystal_pool_id) {
      current_choice = static_cast<int>(i);
    }
  }
  ImGui::PushItemWidth(120);
  if (!pools.empty()) {
    if (current_choice < 0) {
      current_choice = 0;
    }
    if (ImGui::Combo("##crystal", &current_choice, crystal_ptrs.data(), static_cast<int>(pools.size()))) {
      ref.crystal_pool_id = pools[static_cast<size_t>(current_choice)];
      state.MarkFilterDirty();
    }
  } else {
    ImGui::TextDisabled("<no placements>");
  }
  ImGui::PopItemWidth();

  // Whole-crystal checkbox.
  ImGui::SameLine();
  if (ImGui::Checkbox("whole", &ref.match_all)) {
    if (ref.match_all) {
      ref.predicate_text.clear();
    }
    state.MarkFilterDirty();
  }

  // Predicate text (only when not whole-crystal).
  if (!ref.match_all) {
    ImGui::SameLine();
    ImGui::PushItemWidth(180);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s", ref.predicate_text.c_str());
    const auto validation = ValidateSingleAtomText(ref.predicate_text);
    const bool invalid = validation.state != LUMICE_RAYPATH_VALID;
    if (invalid) {
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.9f, 0.3f, 0.3f, 1.0f));
      ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
    }
    if (ImGui::InputText("##pred", buf, sizeof(buf))) {
      // Only stamp state dirty when the new text also validates OK; keep the
      // last-good text unchanged on transient invalid input (mirrors the
      // filter editor convention: preserve the last committed atom rather
      // than propagate a half-typed line to the next debounce commit).
      const std::string new_text = buf;
      const auto v2 = ValidateSingleAtomText(new_text);
      ref.predicate_text = new_text;  // let the user see what they typed
      if (v2.state == LUMICE_RAYPATH_VALID) {
        state.MarkFilterDirty();
      }
    }
    if (invalid) {
      ImGui::PopStyleVar();
      ImGui::PopStyleColor();
      if (ImGui::IsItemHovered() && !validation.message.empty()) {
        ImGui::SetTooltip("%s", validation.message.c_str());
      }
    }
    ImGui::PopItemWidth();
  }

  // Delete this ref.
  ImGui::SameLine();
  if (ImGui::SmallButton(ICON_FA_XMARK "##ref_del")) {
    delete_this_ref = true;
  }

  ImGui::PopID();
}

// -------------------- polling state --------------------

// Poll interval is intentionally coarse — LUMICE_GetColorClassSignal is
// O(W*H*class_count*consumers), matching the "debounce cadence, not per-frame"
// contract in lumice.h. 500 ms is fast enough for interactive editing feedback
// and far below any user perception threshold.
constexpr float kSignalPollIntervalSec = 0.5f;

struct WindowLocalState {
  std::vector<int> signal_flags;
  float last_poll_time = -1000.0f;
};

WindowLocalState& GetLocalState() {
  static WindowLocalState s;
  return s;
}

}  // namespace

void RenderColorWindow(GuiState& state, LUMICE_Server* server) {
  if (!state.color_window_open) {
    return;
  }

  ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));

  if (!ImGui::Begin(ICON_FA_PALETTE " Colors", &state.color_window_open,
                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking)) {
    ImGui::End();
    return;
  }

  // Header row: composite mode + import + add.
  RenderCompositeModeCombo(state, server);
  ImGui::SameLine();
  RenderImportFromFilterUI(state);
  ImGui::SameLine();
  if (ImGui::Button(ICON_FA_PLUS " Add Class")) {
    ColorClassConfig c;
    c.color[0] = 1.0f;
    c.color[1] = 1.0f;
    c.color[2] = 1.0f;
    c.visible = true;
    c.z_order = static_cast<int>(state.raypath_color.size());
    state.raypath_color.push_back(c);
    state.MarkFilterDirty();
  }

  ImGui::Separator();

  // Debounced signal poll for AC4 empty-arc warning.
  auto& local = GetLocalState();
  const float now = static_cast<float>(ImGui::GetTime());
  if (now - local.last_poll_time > kSignalPollIntervalSec) {
    PollColorClassSignal(state, server, local.signal_flags);
    local.last_poll_time = now;
  }
  if (local.signal_flags.size() != state.raypath_color.size()) {
    local.signal_flags.assign(state.raypath_color.size(), 0);
  }

  // Sort classes by z_order for display (physical vector order stays put).
  const size_t n = state.raypath_color.size();
  std::vector<size_t> phys_by_rank(n);
  for (size_t i = 0; i < n; i++) {
    phys_by_rank[i] = i;
  }
  std::stable_sort(phys_by_rank.begin(), phys_by_rank.end(),
                   [&](size_t a, size_t b) { return state.raypath_color[a].z_order < state.raypath_color[b].z_order; });

  std::optional<size_t> pending_delete;
  std::optional<std::pair<size_t, size_t>> pending_zswap;  // (phys_a, phys_b)

  for (size_t rank = 0; rank < phys_by_rank.size(); rank++) {
    const size_t phys = phys_by_rank[rank];
    auto& cls = state.raypath_color[phys];
    ImGui::PushID(static_cast<int>(phys));

    // z_order up/down (plan §292: v1 fallback for drag-reorder). Swap z_order
    // values with the neighbor at rank±1 (NOT the neighbor in physical order).
    const bool at_top = (rank == 0);
    const bool at_bot = (rank + 1 == phys_by_rank.size());
    if (at_top) {
      ImGui::BeginDisabled();
    }
    if (ImGui::SmallButton(ICON_FA_ARROW_UP "##up")) {
      pending_zswap = std::make_pair(phys, phys_by_rank[rank - 1]);
    }
    if (at_top) {
      ImGui::EndDisabled();
    }
    ImGui::SameLine();
    if (at_bot) {
      ImGui::BeginDisabled();
    }
    if (ImGui::SmallButton(ICON_FA_ARROW_DOWN "##down")) {
      pending_zswap = std::make_pair(phys, phys_by_rank[rank + 1]);
    }
    if (at_bot) {
      ImGui::EndDisabled();
    }

    // Color swatch.
    ImGui::SameLine();
    if (ImGui::ColorEdit3("##color", cls.color, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel)) {
      PushDisplayState(state, server);
    }

    // Visible / solo.
    ImGui::SameLine();
    const char* eye = cls.visible ? ICON_FA_EYE : ICON_FA_EYE_SLASH;
    if (ImGui::SmallButton(eye)) {
      cls.visible = !cls.visible;
      PushDisplayState(state, server);
    }
    ImGui::SameLine();
    bool solo = cls.solo;
    if (ImGui::Checkbox("solo", &solo)) {
      cls.solo = solo;
      PushDisplayState(state, server);
    }

    // Empty warning (AC4).
    if (phys < local.signal_flags.size() && !cls.match.empty() && local.signal_flags[phys] == 0) {
      ImGui::SameLine();
      PushWarningStyle();
      ImGui::TextUnformatted(ICON_FA_TRIANGLE_EXCLAMATION);
      PopWarningStyle();
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("No rays matched this class (a physical filter may be blocking them).");
      }
    }

    // Delete.
    ImGui::SameLine();
    if (ImGui::SmallButton(ICON_FA_XMARK "##cls_del")) {
      pending_delete = phys;
    }

    // Summary + tree expand for the ref editor.
    ImGui::SameLine();
    const std::string summary = BuildClassSummary(state, cls);
    const bool open = ImGui::TreeNodeEx("##body", ImGuiTreeNodeFlags_SpanAvailWidth, "%s", summary.c_str());
    if (open) {
      // combine any/all
      static const char* const kCombineNames[] = { "any", "all" };
      int combine_val = std::clamp(cls.combine, 0, 1);
      ImGui::TextUnformatted("Combine:");
      ImGui::SameLine();
      ImGui::PushItemWidth(80);
      if (ImGui::Combo("##combine", &combine_val, kCombineNames, 2)) {
        cls.combine = combine_val;
        state.MarkFilterDirty();
      }
      ImGui::PopItemWidth();

      // Refs.
      std::optional<size_t> ref_to_delete;
      for (size_t ri = 0; ri < cls.match.size(); ri++) {
        bool del = false;
        RenderRefRow(state, cls, ri, del);
        if (del) {
          ref_to_delete = ri;
        }
      }
      if (ImGui::SmallButton(ICON_FA_PLUS " Add Ref")) {
        ColorClassRefConfig r;
        // Default to layer 0's first crystal placement so the row is
        // always in a self-consistent state on creation.
        r.layer_idx = 0;
        const auto pools = CrystalPoolsInLayer(state, 0);
        r.crystal_pool_id = pools.empty() ? 0 : pools.front();
        r.match_all = true;
        cls.match.push_back(r);
        state.MarkFilterDirty();
      }
      if (ref_to_delete.has_value()) {
        cls.match.erase(cls.match.begin() + static_cast<std::ptrdiff_t>(*ref_to_delete));
        state.MarkFilterDirty();
      }

      ImGui::TreePop();
    }

    ImGui::PopID();
  }

  if (pending_zswap.has_value()) {
    SwapZOrder(state, pending_zswap->first, pending_zswap->second);
    PushDisplayState(state, server);
  }
  if (pending_delete.has_value()) {
    state.raypath_color.erase(state.raypath_color.begin() + static_cast<std::ptrdiff_t>(*pending_delete));
    CompactZOrder(state);
    state.MarkFilterDirty();
  }

  ImGui::End();
}

}  // namespace lumice::gui
