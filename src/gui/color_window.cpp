// task-342.3 Steps 5-9: Colors window.
//
// Non-modal floating panel driving the GuiState::raypath_color pool. task-color-migration
// (T1) migrated all edit sites onto the T0 field-tier reconciler pattern: widgets ONLY write
// GuiState fields; the frame-tail ReconcileGuiEffects (gui_state_reconcile.cpp) diffs
// live-vs-baseline every frame and routes effects onto their proper channels via
// ApplyGuiEffects. There are NO `state.MarkStructHardDirty()` / `PushDisplayState(...)` calls in
// this file's widget code anymore — those are effects, and the reconciler is the sole owner
// (single-writer discipline, doc/gui-state-governance.md 支柱 2).
//
// Routing (which channel a widget mutation lands on) is derived from the ColorClassConfig
// split introduced in M1:
//   Structural (re-sim; dirty → next commit): combine/match on ColorClassStructState, plus
//   vector cardinality (Add/Delete class/ref, Import from filter).
//   Display-time (no epoch bump / no dirty): color/visible/solo/z_order on
//   ColorClassDisplayState, plus raypath_color_mode.
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
#include "gui/destructive_style.hpp"
#include "gui/gui_logger.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "gui/symmetry_ui.hpp"
#include "imgui.h"
#include "include/lumice.h"

namespace lumice::gui {

// Rebuild the display-only arrays that LUMICE_SetRaypathColors consumes and
// push them to the server. task-color-migration (T1): exported via
// color_window.hpp so the frame-tail reconciler + DoRun/DoRevert repush
// discipline can drive it (widgets themselves no longer call this directly —
// they only write GuiState fields; the reconciler diffs and pushes).
// LUMICE_SetRaypathColors is whole-table — a single row change still forces a
// full rebuild of classes[]/z_order[].
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
  // task-345.2 (③): after a finite sim completes, ServerPoller self-pauses (server_poller.cpp)
  // and no one drives DoSnapshot() anymore, so the LUMICE_SetRaypathColors() call above sets
  // snapshot_dirty_ but that flag would never get consumed — the preview stays on the old colors
  // until the user restarts the sim. Wake the poller here so it runs ONE more PollOnce() cycle
  // to materialize a fresh composite with the new colors, then self-pauses again (its own
  // COMPLETED-edge self-pause fires at the end of that same PollOnce, see server_poller.cpp).
  // No epoch bump, no accumulator reset, no sim restart — display-time semantics preserved
  // (322 lifecycle clock decoupling, doc/gui-preview-lifecycle-architecture.md I1–I6).
  //
  // If the poller is already running (infinite sim / mid-run edit), WakeForRefresh is a
  // zero-overhead no-op (server_poller.hpp). WakeForRefresh (not WakeForRestart) is
  // mandatory here: display-time refresh must not publish valid=false, else SyncFromPoller
  // observes an invalid snapshot and ReconcileSimState pulls a completed sim back into
  // kSimulating — task-color-migration AC1 activity bug root cause (a).
  g_server_poller.WakeForRefresh(server);
  return true;
}

namespace {

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

}  // namespace

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

// task-list-row-ergonomics ④: setter that toggles match_all without clearing
// predicate_text. file_io.cpp:1337 FillColorPredicate reads match_all FIRST,
// so retaining stale text under match_all=true does NOT contaminate commit
// output (whole-crystal is emitted). Toggling back restores the same text.
void SetRefMatchAll(ColorClassRefConfig& ref, bool match_all) {
  ref.match_all = match_all;
}

// task-356.3 — see color_window.hpp for the doc comment. Pure predicate on the
// ref alone; caller BeginDisabled-freezes the P/B/D row when this returns false.
bool IsRefSymmetryEditable(const ColorClassRefConfig& ref) {
  return !ref.match_all;
}

// task-list-row-ergonomics ③: eye-icon click handler with Alt+click = solo.
// Plain click toggles `visible` only. Alt+click enforces exclusive solo (or
// clears solo when the clicked class is already solo'd, restoring per-visible
// composition). Out-of-range is a defensive no-op. The compositor's solo/visible
// dispatch (`GatherActiveClasses` in ColorClassTable) is unchanged — this only
// shapes the UI-driven state so the solo set has size 0 or 1.
void HandleEyeClick(std::vector<ColorClassConfig>& classes, size_t phys, bool alt_down) {
  if (phys >= classes.size()) {
    return;
  }
  if (!alt_down) {
    classes[phys].visible = !classes[phys].visible;
    return;
  }
  const bool was_solo = classes[phys].solo;
  for (auto& c : classes) {
    c.solo = false;
  }
  if (!was_solo) {
    classes[phys].solo = true;
  }
}

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
  // Mirrors FillColorPredicate's authoritative gate exactly (factors.size()==1 AND
  // CountFactorAlternatives==1): a text like "1-3;5-7" parses to a single Factor (no ' & ')
  // but that Factor still expands to 2 alternatives via the ';' OR-separator, which
  // LUMICE_ColorPredicate cannot carry. Without this second check the row would look valid
  // here and then get silently dropped by FillColorPredicate at the next commit
  // (code-review-01 Major).
  if (factors.size() != 1 || CountFactorAlternatives(factors[0]) != 1) {
    return GuiValidationResult{ LUMICE_RAYPATH_INVALID, "single-atom only — use combine:all across refs for AND" };
  }
  return GuiValidationResult{ LUMICE_RAYPATH_VALID, {} };
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
    // Same single-atom gate as ValidateSingleAtomText / FillColorPredicate: a row whose
    // (cached) Factor resolves to more than one alternative (e.g. "1-3;5-7") must be skipped
    // here too, or it silently disappears at the next structural commit instead of triggering
    // the Step 8 import-skip warning (code-review-01 Minor 1, same root cause as Major 2).
    if (row.factors.size() != 1 || CountFactorAlternatives(row.factors[0]) != 1) {
      skipped_rows++;
      continue;
    }
    ColorClassRefConfig ref;
    ref.layer_idx = layer_idx;
    ref.crystal_pool_id = crystal_pool_id;
    ref.match_all = row.text.empty();
    ref.predicate_text = row.text;
    // task-color-default-pbd: GUI-created ref defaults to P|B|D symmetry (owner-preferred).
    // Only applied at this call site — struct default in ColorClassRefConfig stays false to
    // preserve deserialization semantics for legacy configs missing `sym_*` keys.
    ref.sym_p = true;
    ref.sym_b = true;
    ref.sym_d = true;
    cls.match.push_back(ref);
  }
  return cls;
}

namespace {

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

// -------------------- window body --------------------

// T1: no `server` parameter — display push derived by the frame-tail reconciler from the
// raypath_color_mode diff; this widget only writes the field.
void RenderCompositeModeCombo(GuiState& state) {
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
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "How overlapping color classes combine per pixel:\n"
        "  dominant -- brightest participating class wins\n"
        "  additive -- colors sum (clamped to white)\n"
        "  painter  -- highest-priority class with any signal wins, regardless of brightness");
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
        // T1: structural add (vector cardinality change) → reconciler routes to hard-reset lane.
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

// Render one member ref (structural edits go through MarkStructHardDirty).
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
    // T1: structural (ColorClassStructState.match) → reconciler routes to hard-reset lane.
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
      // T1: structural → reconciler.
    }
  } else {
    ImGui::TextDisabled("<no placements>");
  }
  ImGui::PopItemWidth();

  // Whole-crystal checkbox. task-list-row-ergonomics ④: SetRefMatchAll keeps
  // predicate_text so the InputText below can freeze (BeginDisabled) instead
  // of vanishing, and restore the same text when the user un-checks whole.
  ImGui::SameLine();
  bool whole = ref.match_all;
  if (ImGui::Checkbox("whole", &whole)) {
    SetRefMatchAll(ref, whole);
    // T1: structural → reconciler.
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Match every raypath through the whole crystal (ignores the predicate text\n"
        "below, which stays but is frozen while this is checked).");
  }

  // Predicate text — always rendered so its layout does not disappear when
  // whole is checked; BeginDisabled greys it out under match_all. Suppress the
  // invalid red border while frozen: the text is not participating in the
  // filter, so a "please fix" red frame would be misleading visual noise.
  ImGui::SameLine();
  ImGui::PushItemWidth(180);
  char buf[256];
  std::snprintf(buf, sizeof(buf), "%s", ref.predicate_text.c_str());
  const auto validation = ValidateSingleAtomText(ref.predicate_text);
  const bool invalid = !ref.match_all && validation.state != LUMICE_RAYPATH_VALID;
  if (invalid) {
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.9f, 0.3f, 0.3f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
  }
  if (ref.match_all) {
    ImGui::BeginDisabled();
  }
  if (ImGui::InputText("##pred", buf, sizeof(buf))) {
    // T1: reconciler routes any change to `ref.predicate_text` through struct-part diff to
    // hard-reset (RaypathColorStructChanged in gui_state_reconcile.cpp), unconditionally —
    // there is no separate imperative MarkStructHardDirty call left to gate on validity the way
    // the pre-migration code did. So gate the WRITE itself: only commit `buf` into the diffed
    // field once it validates as a single atom (code-review round-1 Major-2), mirroring the
    // pre-migration "only MarkStructHardDirty when valid" behavior and avoiding a hard-reset (and
    // its user-visible flicker) on every keystroke of a transient invalid predicate.
    // ImGui's InputText keeps its own live edit buffer for an active item — reasserting `buf`
    // from `ref.predicate_text` at the top of next frame (the std::snprintf above) does not
    // clobber what the user is mid-typing, so this is safe even while the field stays uncommitted.
    const std::string typed = std::string(buf);
    if (ValidateSingleAtomText(typed).state == LUMICE_RAYPATH_VALID) {
      ref.predicate_text = typed;
    }
  }
  if (ref.match_all) {
    ImGui::EndDisabled();
  }
  if (invalid) {
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
  }
  // Single SetTooltip for the item — syntax hint always; the validation error
  // (if any) is appended below it. AllowWhenDisabled so users still see the
  // hint while whole freezes the field (mirrors the up/down arrow tooltips
  // that also survive their at_top / at_bot disabled states).
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    static constexpr const char* kSyntaxHint =
        "Same token syntax as the filter editor (e.g. 3-5, entry:2, exit:4, len:2-3).\n"
        "Single atom only here -- no ';' OR / '&' AND; add another ref + Combine below for that.";
    if (invalid && !validation.message.empty()) {
      ImGui::SetTooltip("%s\n\n%s", kSyntaxHint, validation.message.c_str());
    } else {
      ImGui::SetTooltip("%s", kSyntaxHint);
    }
  }
  ImGui::PopItemWidth();

  // task-356.3 — per-ref P/B/D symmetry checkboxes.
  //
  // D-applicability is computed from THIS ref's bound crystal (not a modal-scoped
  // "current" crystal like the filter modal has), because different refs in the
  // same color class can point to different crystals (scrum.md 决策 1 per-ref).
  // Bounds-check the crystal id so a stale pool reference (crystal removed after
  // the ref was created — parallels the `<no placements>` fallback earlier in
  // this function) degrades gracefully to "D not applicable" without a crash.
  ImGui::SameLine();
  bool d_applicable = false;
  if (ref.crystal_pool_id >= 0 && static_cast<size_t>(ref.crystal_pool_id) < state.crystals.size()) {
    const auto& cr = state.crystals[static_cast<size_t>(ref.crystal_pool_id)];
    d_applicable = IsDApplicableGuiAxis(cr.azimuth, cr.roll);
  }
  const bool sym_editable = IsRefSymmetryEditable(ref);
  if (!sym_editable) {
    ImGui::BeginDisabled();
  }
  // The ref_idx (via PushID at RenderRefRow top) already disambiguates rows,
  // so a per-row-static suffix is enough. Symmetry writes here land on
  // ColorClassStructState via the reconciler's frame-tail diff — no explicit
  // MarkStructHardDirty call needed (see gui_state_reconcile.cpp
  // RaypathColorStructChanged; the operator== extension in gui_state.hpp is
  // what wires new fields into that diff).
  RenderSymmetryCheckboxes(ref.sym_p, ref.sym_b, ref.sym_d, d_applicable, "color_ref");
  if (!sym_editable) {
    ImGui::EndDisabled();
  }

  // Delete this ref.
  ImGui::SameLine();
  PushDestructiveStyle();
  if (ImGui::SmallButton(ICON_FA_XMARK "##ref_del")) {
    delete_this_ref = true;
  }
  PopDestructiveStyle();

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
  // task-cleanup-hardening (S5): cache-invalidation keys. The polled signal_flags
  // belong to a specific (server, committed_epoch) domain — a backend swap destroys
  // the old server and mints a fresh one whose committed_epoch resets to 0 (see
  // GuiState::ResetDisplayGenerationForBackendSwap), and any struct commit bumps
  // committed_epoch. Carrying the previous domain's flags across such a boundary
  // would display up to kSignalPollIntervalSec (500 ms) of stale server signal —
  // visually, a color-class pip claiming "matched" while the new backend has not
  // yet been polled. RefreshColorClassSignals compares (last_server, last_committed_epoch)
  // against the current pair and, on any mismatch, forces an immediate re-poll so the
  // next frame sees fresh flags (AC2).
  //
  // Hook rationale (reviewer Minor-2/3): we deliberately do NOT reuse
  // GuiState::ResetDisplayGenerationForBackendSwap — that owner drives the
  // display-generation epoch/floor/texture, a different domain from the signal-cache
  // liveness. Comparing the keys at poll entry keeps the two domains independent and
  // single-owner per doc/gui-state-governance.md 支柱 2 (per-channel single serializer).
  LUMICE_Server* last_server = nullptr;
  uint64_t last_committed_epoch = 0;
};

WindowLocalState& GetLocalState() {
  static WindowLocalState s;
  return s;
}

}  // namespace

// task-348.1 fix: poll GetColorClassSignal into a caller-owned buffer sized to
// state.raypath_color.size(). Resize semantics: new entries default to 1
// ("settling / no warning") instead of the pre-fix 0 ("no rays matched"). On
// C-API rejection (class_count mismatch during the debounce window between
// GUI-side push_back and server-side commit), out_flags is LEFT UNCHANGED —
// the previously observed signal is preserved rather than clobbered to 0.
// This closes the "add a class → every pre-existing class instantly shows the
// no-match warning" bug (② in issue.md): resize no longer wipes old signals,
// and mismatch no longer treats "we don't know yet" as "we know it's zero".
void PollColorClassSignal(const GuiState& state, LUMICE_Server* server, std::vector<int>& out_flags) {
  const size_t n = state.raypath_color.size();
  out_flags.resize(n, 1);
  if (server == nullptr || n == 0) {
    return;
  }
  std::vector<int> tmp(n, 0);
  const LUMICE_ErrorCode ec = LUMICE_GetColorClassSignal(server, tmp.data(), static_cast<int>(n));
  if (ec == LUMICE_OK) {
    out_flags = std::move(tmp);
  }
  // ec != LUMICE_OK: settling window (server hasn't picked up the new class_count yet).
  // Leave out_flags with resize-preserved prior values; the next successful poll heals.
}

// Orchestration wrapper: 500 ms throttled poll + resize + shared cache. Same-frame
// idempotent (subsequent calls hit the throttle and just resize + return); it is
// therefore safe for both RenderColorWindow (per-row) and RenderTopBar (aggregate)
// to call this in the same frame. The trailing resize guarantees the returned
// vector always matches state.raypath_color.size() even in the frames between
// two throttled polls (a fresh add/remove between windows must not leave the
// cache out-of-sync). Returns a copy: the vector is at most a handful of ints
// (one per color class), so the copy is negligible, and it keeps the internal
// throttle cache from being exposed by reference to cross-module callers
// (app_panels.cpp's top-bar pip) whose lifetime/threading assumptions this file
// cannot enforce.
std::vector<int> RefreshColorClassSignals(const GuiState& state, LUMICE_Server* server) {
  auto& local = GetLocalState();
  const float now = static_cast<float>(ImGui::GetTime());

  // task-cleanup-hardening (S5): cache-invalidation on backend swap or epoch bump.
  // A change in (server, committed_epoch) means the flags in cache describe a
  // domain that no longer exists (destroyed server) or a stale generation (pre-commit
  // signal). Clear them and force an immediate re-poll so the next display frame
  // shows the new domain's flags, not the old.
  const bool domain_changed = (server != local.last_server) || (state.committed_epoch != local.last_committed_epoch);
  if (domain_changed) {
    local.signal_flags.clear();
    local.last_poll_time = -1000.0f;  // force poll below the throttle
    local.last_server = server;
    local.last_committed_epoch = state.committed_epoch;
  }

  if (now - local.last_poll_time > kSignalPollIntervalSec) {
    PollColorClassSignal(state, server, local.signal_flags);
    local.last_poll_time = now;
  }
  local.signal_flags.resize(state.raypath_color.size(), 1);
  return local.signal_flags;
}

// task-cleanup-hardening S5: test-only accessors (see color_window.hpp).
void GetColorClassSignalCacheKeysForTest(LUMICE_Server** server_out, uint64_t* epoch_out, size_t* flags_size_out,
                                         float* last_poll_time_out) {
  const auto& local = GetLocalState();
  if (server_out != nullptr) {
    *server_out = local.last_server;
  }
  if (epoch_out != nullptr) {
    *epoch_out = local.last_committed_epoch;
  }
  if (flags_size_out != nullptr) {
    *flags_size_out = local.signal_flags.size();
  }
  if (last_poll_time_out != nullptr) {
    *last_poll_time_out = local.last_poll_time;
  }
}

void ResetColorClassSignalCacheForTest() {
  auto& local = GetLocalState();
  local.signal_flags.clear();
  local.last_poll_time = -1000.0f;
  local.last_server = nullptr;
  local.last_committed_epoch = 0;
}

// task-fix-color-window-visibility-consistency: single owner for
// "composite would be empty right now". Merges the two prior triggers
// (all-configured-classes-unmatched AND matched-but-hidden) behind one
// predicate so the Enable-colors checkbox greying + top-bar pip + row-level
// warning cannot disagree. Return-true iff no configured class is
// simultaneously matched AND effectively visible. See color_window.hpp
// docstring for the "unknown index" caveat.
bool NoVisibleMatchedColorClass(const GuiState& state, const std::vector<int>& signal_flags) {
  const bool any_solo = AnySolo(state.raypath_color);
  bool any_configured_known = false;
  for (size_t i = 0; i < state.raypath_color.size(); i++) {
    const auto& cls = state.raypath_color[i];
    if (cls.match.empty()) {
      continue;
    }
    // Out-of-range index means the caller's cache hasn't caught up with
    // state.raypath_color yet (same "we don't know yet" window PollColorClassSignal's
    // resize(n, 1) models) -- treat as unknown, so it neither counts toward
    // any_configured_known nor disqualifies the aggregate.
    if (i >= signal_flags.size()) {
      continue;
    }
    any_configured_known = true;
    if (signal_flags[i] != 0 && EffectiveVisible(cls, any_solo)) {
      return false;
    }
  }
  return any_configured_known;
}

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

  // task-349.2 Step 3 (#6): fetch the shared signal cache BEFORE the Enable
  // colors checkbox so we can wrap that checkbox (and its top-bar mirror in
  // RenderTopBar) in BeginDisabled() when every configured class matches zero
  // rays. Same throttled source as the top-bar and the per-row pips below;
  // moving the fetch earlier does not change the poll cost (single 500 ms
  // throttle) and does not race the per-row pip logic that reads
  // `signal_flags[phys]` further down — that logic still sees the same cache.
  const std::vector<int>& signal_flags = RefreshColorClassSignals(state, server);
  const bool composite_empty = NoVisibleMatchedColorClass(state, signal_flags);
  const bool any_solo = AnySolo(state.raypath_color);

  // Header row: composite mode + import + add + Enable colors (right-aligned).
  RenderCompositeModeCombo(state);
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
    // T1: structural cardinality change → reconciler routes to hard-reset lane.
  }

  // Bulk clear — one-shot for users who imported N filters and want a clean
  // slate. Cleared vector is a structural cardinality change, so the reconciler
  // routes it through the same T1 hard-reset lane as single-row erase (see
  // gui_state_reconcile.cpp RaypathColorStructChanged). Disabled when there is
  // nothing to remove so an idle press cannot dirty the state.
  ImGui::SameLine();
  const bool no_classes = state.raypath_color.empty();
  if (no_classes) {
    ImGui::BeginDisabled();
  }
  PushDestructiveStyle();
  if (ImGui::Button(ICON_FA_TRASH " Remove All")) {
    state.raypath_color.clear();
    // T1: structural cardinality change → reconciler routes to hard-reset lane
    // (same channel as the single-row delete path below).
  }
  PopDestructiveStyle();
  if (no_classes) {
    ImGui::EndDisabled();
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Remove every raypath-color class in one click.");
  }

  // task-349.3 (#3): "Enable colors" mirrors the top-bar Colored toggle inside
  // the window and lives on the same header row as "+ Add Class", right-aligned
  // to read as a distinct control group from the mode/import/add cluster on the
  // left (owner on-screen feedback after 348.3 landed the top-bar icon toggle).
  // Semantics unchanged from 348.3 AC2 (⑥) / 349.2 Step 3 (#6): checked reads
  // GROUND TRUTH (last_uploaded_as_composite), click writes via shared
  // ToggleCompositePreview(); when all_unmatched, wrap in BeginDisabled and
  // swap the tooltip to kColorsDisabledNoMatchTooltip (shared with the top-bar
  // mirror so the two disabled cues cannot drift).
  {
    ImGui::SameLine();
    const char* label = "Enable colors";
    const ImGuiStyle& style = ImGui::GetStyle();
    const float w = ImGui::CalcTextSize(label).x + ImGui::GetFrameHeight() + style.ItemInnerSpacing.x;
    const float right_edge = ImGui::GetWindowContentRegionMax().x;
    if (right_edge - w > ImGui::GetCursorPosX()) {
      ImGui::SetCursorPosX(right_edge - w);
    }
    bool checked = state.last_uploaded_as_composite;
    if (composite_empty) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Checkbox(label, &checked)) {
      ToggleCompositePreview(state);
    }
    if (composite_empty) {
      ImGui::EndDisabled();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      if (composite_empty) {
        ImGui::SetTooltip("%s", kColorsDisabledNoMatchTooltip);
      } else {
        ImGui::SetTooltip(
            "Toggle colored composite / full-spectrum preview.\n"
            "Synced with the top-bar toggle -- same display-time preference.");
      }
    }
  }

  ImGui::Separator();

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

  // Precompute the eye-icon button width once so it does not change as the
  // per-row glyph flips between ICON_FA_EYE and ICON_FA_EYE_SLASH (they have
  // different advance widths; without a fixed width the whole row shifts
  // horizontally on every toggle — AC2 zero-jitter requirement).
  // FramePadding.y is pushed to 0 at render time so the height matches the
  // surrounding SmallButtons on the same row.
  const ImGuiStyle& row_style = ImGui::GetStyle();
  const float eye_btn_w = std::max(ImGui::CalcTextSize(ICON_FA_EYE).x, ImGui::CalcTextSize(ICON_FA_EYE_SLASH).x) +
                          row_style.FramePadding.x * 2.0f;

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
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Raise priority -- wins ties in Dominant mode and paints over lower classes in Painter mode.");
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
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("Lower priority (opposite of the arrow above).");
    }

    // Color swatch.
    ImGui::SameLine();
    if (ImGui::ColorEdit3("##color", cls.color, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel)) {
      // T1: display-only (ColorClassDisplayState.color) → reconciler routes to need_display_push.
    }

    // Visible / solo — task-list-row-ergonomics ③: the standalone solo column
    // is removed; solo now rides on the eye icon via Alt+click. Plain click
    // toggles `visible`; Alt+click enforces exclusive solo (or clears it).
    // task-fix-color-window-visibility-consistency: read EffectiveVisible (not
    // raw `cls.visible`) so a solo'd class hides the eye on peers whose visible
    // flag stayed true — mirrors compositor GatherActiveClasses:55 so the UI
    // cannot say "this class is showing" while the composite has excluded it.
    ImGui::SameLine();
    const char* eye = EffectiveVisible(cls, any_solo) ? ICON_FA_EYE : ICON_FA_EYE_SLASH;
    // Fixed-width Button (with FramePadding.y=0 to match SmallButton height on
    // this row) so toggling ICON_FA_EYE ↔ ICON_FA_EYE_SLASH does not shift the
    // trailing controls (warning triangle, delete x). See eye_btn_w above.
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(row_style.FramePadding.x, 0.0f));
    if (ImGui::Button(eye, ImVec2(eye_btn_w, 0.0f))) {
      HandleEyeClick(state.raypath_color, phys, ImGui::GetIO().KeyAlt);
      // T1: display-only (visible/solo) → reconciler routes to need_display_push.
    }
    ImGui::PopStyleVar();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Click: show/hide this class in the composite (display-time only).\n"
          "Alt+Click: solo this class (hide all others); Alt+Click again to restore all.");
    }

    // Empty warning (AC4).
    if (phys < signal_flags.size() && !cls.match.empty() && signal_flags[phys] == 0) {
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
    PushDestructiveStyle();
    if (ImGui::SmallButton(ICON_FA_XMARK "##cls_del")) {
      pending_delete = phys;
    }
    PopDestructiveStyle();

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
        // T1: structural (ColorClassStructState.combine) → reconciler routes to hard-reset.
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "any -- a ray belongs to this class if it matches AT LEAST ONE ref below (OR)\n"
            "all -- a ray belongs to this class only if it matches EVERY ref below (AND)");
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
        // task-color-default-pbd: GUI-created ref defaults to P|B|D symmetry (owner-preferred).
        r.sym_p = true;
        r.sym_b = true;
        r.sym_d = true;
        cls.match.push_back(r);
        // T1: structural (match[] append) → reconciler routes to hard-reset lane.
      }
      if (ref_to_delete.has_value()) {
        cls.match.erase(cls.match.begin() + static_cast<std::ptrdiff_t>(*ref_to_delete));
        // T1: structural → reconciler.
      }

      ImGui::TreePop();
    }

    ImGui::PopID();
  }

  if (pending_zswap.has_value()) {
    SwapZOrder(state, pending_zswap->first, pending_zswap->second);
    // T1: display-only (ColorClassDisplayState.z_order) → reconciler routes to need_display_push.
  }
  if (pending_delete.has_value()) {
    state.raypath_color.erase(state.raypath_color.begin() + static_cast<std::ptrdiff_t>(*pending_delete));
    CompactZOrder(state);
    // T1: structural (vector cardinality change) → reconciler routes to hard-reset lane.
    // CompactZOrder is a data-shape maintenance step (keeps z_order a permutation), NOT an
    // effect — kept in the widget.
  }

  ImGui::End();
}

}  // namespace lumice::gui
