#ifndef LUMICE_GUI_STATE_TIERS_HPP
#define LUMICE_GUI_STATE_TIERS_HPP

// Declarative field-tier registry for GuiState (scrum-gui-state-reconcile T0 geodetics).
//
// The tier classifies "how does a mutation to this field propagate to downstream effects":
//   - kStructHard : re-sim + immediate display clear + raise display_epoch_floor
//   - kStructSoft : re-sim carry-forward (dirty=true; existing texture stays until new one arrives)
//   - kDisplay    : display push (PushDisplayState / display-time toggle; no dirty/epoch touch)
//   - kView       : pure client-side view preference (background, overlays, panel layout, logs)
//   - kSession    : runtime/session-only, not persisted, not dirty
//
// The `auto_diff_excluded` bit on an entry says: this field is registered under a tier BUT the
// generic diff reconciler (ReconcileGuiEffects) does NOT auto-derive effects from it — the field
// has bespoke handling elsewhere. Currently the sole use is `raypath_color`: it is nominally
// kStructHard (edits ARE structural), but its ColorClassConfig sub-fields mix structural
// (combine/match) and display-only (color/visible/solo/z_order) semantics; auto-diffing the whole
// vector as one hard-tier struct would spuriously fire re-sim on pure display edits until the
// scrum's later task (raypath_color migration) splits ColorClassConfig into structural + display
// sub-structs. Until then the field keeps its existing MarkFilterDirty call sites.
//
// `kDerivedFieldsExcludeList` is a separate registry for fields that are outputs / runtime-derived
// state (sim_state, epoch bookkeeping, stats readbacks, auto-EV runtime, etc.) — they are NOT
// governed by a tier because the reconciler must not touch them.
//
// Governance / gate:
//   scripts/check_policies.py :: check_gui_state_field_tier_registration walks GuiState and asserts
//   every top-level field is registered in EXACTLY one of these two tables. Adding a new GuiState
//   field without updating this file is a policy violation.

namespace lumice::gui {

enum class FieldTier {
  kStructHard,
  kStructSoft,
  kDisplay,
  kView,
  kSession,
};

struct FieldTierEntry {
  const char* name;
  FieldTier tier;
  bool auto_diff_excluded;
};

// clang-format off
inline constexpr FieldTierEntry kFieldTierTable[] = {
    // ==== T-struct·soft: re-sim carry-forward ====================================================
    { "crystals",                   FieldTier::kStructSoft, false },
    { "layers",                     FieldTier::kStructSoft, false },
    { "sun",                        FieldTier::kStructSoft, false },
    { "sim",                        FieldTier::kStructSoft, false },
    { "renderer",                   FieldTier::kStructSoft, false },
    // use_gpu_backend: registered kStructSoft for governance coverage, but NOT in ConfigSnapshot's
    // From/ApplyTo (view/session field intentionally excluded from Revert baseline). No baseline →
    // no diff possible; legacy DIRTY_IF wrapper (panels.cpp:1067) drives dirty. See SUMMARY.md AC1
    // for the second predicted exception found during M2 implementation (per plan §7 risk 5 门槛).
    { "use_gpu_backend",            FieldTier::kStructSoft, true  },

    // ==== T-struct·hard: re-sim + display clear + epoch floor bump ==============================
    { "filters",                    FieldTier::kStructHard, false },
    // raypath_color: nominally kStructHard, excluded from auto-diff until T1 splits
    // ColorClassConfig into structural (combine/match) vs display (color/visible/solo/z_order)
    // sub-structs. Diffing the whole vector as one hard-tier struct today would spuriously fire
    // re-sim on pure display-time color/visibility edits. See doc/gui-state-governance.md T1.
    { "raypath_color",              FieldTier::kStructHard, true  },

    // ==== T-display: display-time push, no dirty/epoch touch ====================================
    { "raypath_color_mode",         FieldTier::kDisplay,    false },

    // ==== T-view: pure client-side view preference ==============================================
    // Aspect ratio
    { "aspect_preset",              FieldTier::kView,       false },
    { "aspect_portrait",            FieldTier::kView,       false },
    // Background image overlay
    { "bg_path",                    FieldTier::kView,       false },
    { "bg_show",                    FieldTier::kView,       false },
    { "bg_alpha",                   FieldTier::kView,       false },
    // Auxiliary-line overlays (horizon / grid / sun-circles)
    { "show_horizon_line",          FieldTier::kView,       false },
    { "show_horizon_label",         FieldTier::kView,       false },
    { "show_grid_line",             FieldTier::kView,       false },
    { "show_grid_label",            FieldTier::kView,       false },
    { "show_sun_circles_line",      FieldTier::kView,       false },
    { "show_sun_circles_label",     FieldTier::kView,       false },
    { "sun_circle_angles",          FieldTier::kView,       false },
    { "horizon_color",              FieldTier::kView,       false },
    { "grid_color",                 FieldTier::kView,       false },
    { "sun_circles_color",          FieldTier::kView,       false },
    { "horizon_alpha",              FieldTier::kView,       false },
    { "grid_alpha",                 FieldTier::kView,       false },
    { "sun_circles_alpha",          FieldTier::kView,       false },
    // Zenith / Nadir marker
    { "show_zenith_nadir_line",     FieldTier::kView,       false },
    { "zenith_nadir_color",         FieldTier::kView,       false },
    { "zenith_nadir_alpha",         FieldTier::kView,       false },
    { "zenith_nadir_radius_px",     FieldTier::kView,       false },
    // Panel layout
    { "left_panel_collapsed",       FieldTier::kView,       false },
    { "right_panel_collapsed",      FieldTier::kView,       false },
    { "modal_layout_vertical",      FieldTier::kView,       false },
    // Log panel
    { "gui_log_level",              FieldTier::kView,       false },
    { "core_log_level",             FieldTier::kView,       false },
    { "log_to_file",                FieldTier::kView,       false },
    { "log_panel_open",             FieldTier::kView,       false },

    // ==== T-session: runtime/session-only, not persisted, not dirty =============================
    { "pick_link_source",           FieldTier::kSession,    false },
    { "color_window_open",          FieldTier::kSession,    false },
    { "current_file_path",          FieldTier::kSession,    false },
    { "save_texture",               FieldTier::kSession,    false },
    { "screenshot_include_overlay", FieldTier::kSession,    false },
    { "modal_immediate_mode",       FieldTier::kSession,    false },
};

// Derived / runtime-produced fields: NOT governed by a tier. The reconciler and effect layer must
// not touch them. Listed here so the policy gate can verify every top-level GuiState field is
// accounted for (governance-union check).
inline constexpr const char* kDerivedFieldsExcludeList[] = {
    // Simulation lifecycle (derived by ReconcileSimState)
    "sim_state",
    "run_intent",
    "committed_epoch",
    "display_epoch_floor",
    "last_uploaded_texture_serial",
    // Stats readbacks (populated from poller snapshots)
    "stats_ray_seg_num",
    "stats_sim_ray_num",
    "snapshot_intensity",
    "effective_pixels",
    "texture_upload_count",
    // Auto-EV runtime (populated by ComputeEvAuto in SyncFromPoller)
    "p99_raw_y",
    "ev_auto",
    "target_white",
    // Display-time composite/xyz toggle (own single-writer contract, orthogonal to sim/dirty)
    "show_composite_preview",
    "last_uploaded_as_composite",
    // Revert baseline
    "last_committed_state",
    // Dirty flag (this is the effect output, not an input to the reconciler)
    "dirty",
    // Runtime-derived aspect clamp info (populated by ApplyAspectRatio)
    "aspect_clamp",
};
// clang-format on

}  // namespace lumice::gui

#endif  // LUMICE_GUI_STATE_TIERS_HPP
