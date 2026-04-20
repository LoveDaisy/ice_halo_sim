#ifndef LUMICE_GUI_STATE_HPP
#define LUMICE_GUI_STATE_HPP

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace lumice::gui {

// Crystal type
enum class CrystalType { kPrism, kPyramid };

// Axis distribution type for crystal orientation.
// Values must stay contiguous 0..N-1 — ImGui RadioButton relies on static_cast<int>.
enum class AxisDistType { kGauss, kUniform, kZigzag, kLaplacian, kGaussLegacy, kCount };

// Aspect ratio presets for preview window
enum class AspectPreset { kFree, k16x9, k3x2, k4x3, k1x1, kMatchBg };
inline const char* const kAspectPresetNames[] = { "Free", "16:9", "3:2", "4:3", "1:1", "Match Background" };
constexpr int kAspectPresetCount = 6;
static_assert(sizeof(kAspectPresetNames) / sizeof(kAspectPresetNames[0]) == kAspectPresetCount,
              "kAspectPresetNames must match kAspectPresetCount");

struct AxisDist {
  AxisDistType type = AxisDistType::kUniform;
  float mean = 0.0f;
  float std = 0.0f;  // Gauss: standard deviation; Uniform: full range; Zigzag: amplitude; Laplacian: scale

  friend bool operator==(const AxisDist& a, const AxisDist& b) {
    return a.type == b.type && a.mean == b.mean && a.std == b.std;
  }
  friend bool operator!=(const AxisDist& a, const AxisDist& b) { return !(a == b); }
};

// GUI-only data structure: crystal geometry + axis distribution.
struct CrystalConfig {
  std::string name;
  CrystalType type = CrystalType::kPrism;

  // Prism
  float height = 1.0f;

  // Pyramid
  float prism_h = 1.0f;
  float upper_h = 0.2f;
  float lower_h = 0.2f;
  float upper_alpha = 28.0f;  // Wedge angle (degrees). Default ≈ atan(√3/2 / 1.629) * 180/π, i.e. Miller {1,0,-1,1}
  float lower_alpha = 28.0f;

  // Face distance (common to Prism and Pyramid — distance from center to each of the 6 prism faces)
  float face_distance[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

  // Axis distribution (all default to uniform full rotation)
  AxisDist zenith{ AxisDistType::kUniform, 0.0f, 360.0f };
  AxisDist azimuth{ AxisDistType::kUniform, 0.0f, 360.0f };
  AxisDist roll{ AxisDistType::kUniform, 0.0f, 360.0f };

  friend bool operator==(const CrystalConfig& a, const CrystalConfig& b) {
    return a.name == b.name && a.type == b.type && a.height == b.height && a.prism_h == b.prism_h &&
           a.upper_h == b.upper_h && a.lower_h == b.lower_h && a.upper_alpha == b.upper_alpha &&
           a.lower_alpha == b.lower_alpha &&
           std::equal(std::begin(a.face_distance), std::end(a.face_distance), std::begin(b.face_distance)) &&
           a.zenith == b.zenith && a.azimuth == b.azimuth && a.roll == b.roll;
  }
  friend bool operator!=(const CrystalConfig& a, const CrystalConfig& b) { return !(a == b); }
};

struct SunConfig {
  float altitude = 20.0f;
  float diameter = 0.5f;
  int spectrum_index = 2;  // Index into kSpectrumNames: 0=D50,1=D55,2=D65,3=D75,4=A,5=E
};

struct SimConfig {
  float ray_num_millions = 5.0f;
  int max_hits = 8;
  bool infinite = false;
};

// Lens type names (order must match Core's LensParam::LensType enum)
inline const char* const kLensTypeNames[] = {
  "Linear",
  "Fisheye Equal Area",
  "Fisheye Equidistant",
  "Fisheye Stereographic",
  "Dual Fisheye Equal Area",
  "Dual Fisheye Equidistant",
  "Dual Fisheye Stereographic",
  "Rectangular",
};
constexpr int kLensTypeCount = 8;

// Dual fisheye overlap: max |sky.z| for the overlap zone.
// = sin(5°) ≈ 0.0872. Each hemisphere extends 5° past the equator.
// This constant defines the GUI's internal texture format overlap. It is passed to core via
// FillLumiceConfig/SerializeCoreConfig; core reads overlap from RenderConfig (no hardcoded value).
constexpr float kDualFisheyeOverlap = 0.0872f;

inline const char* const kSpectrumNames[] = { "D50", "D55", "D65", "D75", "A", "E" };
constexpr int kSpectrumCount = 6;

inline const char* const kVisibleNames[] = { "Upper", "Lower", "Full", "Front" };
constexpr int kVisibleCount = 4;  // must stay in sync with fragment shader u_visible range

inline const int kSimResolutions[] = { 512, 1024, 2048, 4096 };
constexpr int kSimResolutionCount = 4;

struct RenderConfig {
  int lens_type = 0;  // Index into kLensTypeNames
  float fov = 90.0f;
  float elevation = 0.0f;
  float azimuth = 0.0f;
  float roll = 0.0f;
  int sim_resolution_index = 1;  // Index into kSimResolutions (default 1024)
  int visible = 2;               // Index into kVisibleNames (0=upper, 1=lower, 2=full, 3=front)
  float background[3] = { 0.0f, 0.0f, 0.0f };
  float ray_color[3] = { 1.0f, 1.0f, 1.0f };
  float opacity = 1.0f;
  float exposure_offset = 0.0f;  // EV: intensity_factor = 2^exposure_offset

  bool operator==(const RenderConfig& o) const {
    return lens_type == o.lens_type && fov == o.fov && elevation == o.elevation && azimuth == o.azimuth &&
           roll == o.roll && sim_resolution_index == o.sim_resolution_index && visible == o.visible &&
           std::equal(background, background + 3, o.background) && std::equal(ray_color, ray_color + 3, o.ray_color) &&
           opacity == o.opacity && exposure_offset == o.exposure_offset;
  }
  bool operator!=(const RenderConfig& o) const { return !(*this == o); }
};

// Filter action
inline const char* const kFilterActionNames[] = { "Filter In", "Filter Out" };
constexpr int kFilterActionCount = 2;

constexpr char kRaypathSep = '-';
constexpr const char* kRaypathSepStr = "-";

// GUI-only data structure: raypath filter configuration.
struct FilterConfig {
  std::string name;
  int action = 0;            // 0=filter_in, 1=filter_out
  std::string raypath_text;  // Dash-separated int list, e.g. "3-1-5-7-4"; comma also accepted for back-compat
  bool sym_p = true;
  bool sym_b = true;
  bool sym_d = true;

  // Used by edit_modals.cpp to detect whether the user actually modified the
  // filter buffer (so an untouched OK on a previously-empty filter doesn't
  // silently materialize a default-constructed filter into entry.filter).
  //
  // ⚠️ When adding a new field above, also:
  //   1. Compare it here in operator==
  //   2. Update file_io.cpp serialization (Serialize/ParseFilterFromGuiJson)
  //   3. Update edit_modals.cpp Filter tab UI to expose it
  friend bool operator==(const FilterConfig& a, const FilterConfig& b) {
    return a.name == b.name && a.action == b.action && a.raypath_text == b.raypath_text && a.sym_p == b.sym_p &&
           a.sym_b == b.sym_b && a.sym_d == b.sym_d;
  }
  friend bool operator!=(const FilterConfig& a, const FilterConfig& b) { return !(a == b); }
};

// GUI-only data structure: one crystal+filter entry card in the layer model.
//
// operator== exists so CommitAllBuffers (edit_modals.cpp) can skip render-invalidation
// when the OK button doesn't actually change the entry. Adding a field here without
// updating operator== would silently break that gate; the static_assert below catches
// such omissions at compile time (update both together).
struct EntryCard {
  CrystalConfig crystal;
  std::optional<FilterConfig> filter;
  float proportion = 100.0f;

  friend bool operator==(const EntryCard& a, const EntryCard& b) {
    return a.crystal == b.crystal && a.filter == b.filter && a.proportion == b.proportion;
  }
  friend bool operator!=(const EntryCard& a, const EntryCard& b) { return !(a == b); }
};
// Apple Silicon + libc++ only — std::string SSO buffer differs between libc++
// (24 bytes) and libstdc++ (32 bytes), so the absolute sizeof differs per
// platform. Pinning the Apple build is enough to catch an accidental field
// addition during local dev; Linux/Windows CI still compiles the struct.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(EntryCard) == 192,
              "EntryCard size changed (check CrystalConfig/AxisDist/EntryCard operator== for new fields)");
#endif

struct Layer {
  float probability = 0.0f;  // Probability of multi-scatter continuation (0 = single scatter), range [0,1]
  std::vector<EntryCard> entries;
};

struct GuiState {
  // Layers (copy-model: each entry owns its crystal/filter definition)
  std::vector<Layer> layers;

  // Scene
  SunConfig sun;
  SimConfig sim;

  // Renderer (copy model: GuiState owns a single renderer directly).
  // Single-renderer enforced by the GUI; if multi-renderer is ever needed, revisit.
  RenderConfig renderer;

  // Aspect ratio (view preference, not simulation parameter — does not call MarkDirty)
  AspectPreset aspect_preset = AspectPreset::kFree;
  bool aspect_portrait = false;

  // Background image overlay (view preference — does not call MarkDirty)
  std::filesystem::path bg_path;
  bool bg_show = false;
  float bg_alpha = 1.0f;

  // Auxiliary line overlay (view preference — does not call MarkDirty, not in ConfigSnapshot)
  bool show_horizon = false;
  bool show_grid = false;
  bool show_sun_circles = false;
  std::vector<float> sun_circle_angles = { 22.0f, 46.0f };
  float horizon_color[3] = { 0.8f, 0.2f, 0.2f };
  float grid_color[3] = { 1.0f, 1.0f, 1.0f };
  float sun_circles_color[3] = { 1.0f, 0.9f, 0.3f };
  float horizon_alpha = 0.6f;
  float grid_alpha = 0.3f;
  float sun_circles_alpha = 0.5f;

  // File management
  std::filesystem::path current_file_path;
  bool dirty = false;
  bool save_texture = true;  // Whether to include texture in .lmc save (UI-only, not serialized)

  // Screenshot overlay toggle — when true, Screenshot export composites overlay
  // labels onto the off-screen export FBO via RenderExportToRgba. UI-only, not
  // serialized. (Pre gui-polish-v10 this flag also queued a deferred default-framebuffer
  // readback via pending_screenshot; that mechanism was retired — see SUMMARY.)
  bool screenshot_include_overlay = false;

  void MarkDirty() {
    dirty = true;
    if (sim_state == SimState::kDone) {
      sim_state = SimState::kModified;
    }
  }

  // Mark a filter-related change. Immediately clears the display (shader renders black
  // via intensity_scale=0) and locks uploads until the next CommitConfig restart, preventing
  // stale data from the old simulation from overwriting the cleared state.
  void MarkFilterDirty() {
    MarkDirty();
    snapshot_intensity = 0;
    intensity_locked = true;
  }

  // When true, SyncFromPoller skips texture upload to prevent old simulation data from
  // overwriting a filter-change-triggered display clear. Unlocked by CommitConfig restart.
  bool intensity_locked = false;

  // Panel state (view preference — does not call MarkDirty)
  // not persisted to .lmc (unlike right_panel_collapsed)
  bool left_panel_collapsed = false;
  bool right_panel_collapsed = false;

  // Log panel state (view preference — does not call MarkDirty)
  int gui_log_level = 3;   // Index into log level names: 0=trace,1=debug,2=verbose,3=info,4=warning,5=error,6=off
  int core_log_level = 3;  // Same mapping
  bool log_to_file = false;
  bool log_panel_open = false;

  // Simulation state
  enum class SimState { kIdle, kSimulating, kDone, kModified };
  SimState sim_state = SimState::kIdle;

  // Stats from last poll
  unsigned long stats_ray_seg_num = 0;
  unsigned long stats_sim_ray_num = 0;
  float snapshot_intensity = 0;            // Per-pixel landed intensity for XYZ→RGB normalization
  int effective_pixels = 0;                // Non-zero pixel count for adaptive normalization
  int norm_mode = 0;                       // 0=absolute, 1=adaptive (not exposed in UI)
  unsigned long texture_upload_count = 0;  // Cumulative texture uploads (diagnostic counter)

  // Last committed config snapshot (for Revert — config fields only, no runtime state).
  //
  // Field-sync scope (audited 2026-04):
  //   Fields mirrored here must be the subset of GuiState classified as "configuration"
  //   (i.e. those reached by MarkDirty, contributing to the dirty/Revert lifecycle).
  //   View preferences (aspect_preset, bg_*, horizon/grid/sun circles, log levels,
  //   left_panel_collapsed, right_panel_collapsed), runtime state (sim_state, stats_*, snapshot_intensity,
  //   intensity_locked, texture_upload_count), and file management (current_file_path,
  //   dirty, save_texture) are intentionally excluded.
  //
  // Protection model (plan.md S1):
  //   - sizeof(ConfigSnapshot) guard below fires when THIS struct's fields change,
  //     forcing the developer to update From()/ApplyTo() bodies below.
  //   - It does NOT fire when GuiState gains a new configuration field — that requires
  //     discipline (code review + the field-sync audit comment above). Stronger
  //     protection was deferred (see plan.md S5b) as disproportionate to risk.
  struct ConfigSnapshot {
    std::vector<Layer> layers;
    SunConfig sun;
    SimConfig sim;
    RenderConfig renderer;

    // Build a snapshot from the configuration fields of `state`. Implementation is
    // out-of-class (after GuiState is complete) because GuiState is incomplete here.
    static ConfigSnapshot From(const GuiState& state);

    // Restore configuration fields of `state` from this snapshot.
    // IMPORTANT: pure field assignment, no GUI side effects. Callers that need
    // OnLayerStructureChanged(), MarkDirty(), sim_state transitions, etc. must
    // invoke them AFTER ApplyTo() returns.
    void ApplyTo(GuiState& state) const;
  };
  static_assert(std::is_copy_constructible_v<ConfigSnapshot>, "ConfigSnapshot must be copyable");
  static_assert(std::is_copy_assignable_v<ConfigSnapshot>, "ConfigSnapshot must be copy-assignable");
  std::optional<ConfigSnapshot> last_committed_state;
};

// Size guard for ConfigSnapshot. If any field changes here, From/ApplyTo below must
// be audited for matching changes. Apple Silicon + libc++ only (std::vector size varies
// across stdlib implementations).
#if defined(__APPLE__) && defined(__aarch64__)
// Size updated after renderer copy-model migration (task-renderer-inline): previous size (80)
// referenced vector<RenderConfig>+2 ints; new layout embeds a single RenderConfig inline.
static_assert(sizeof(GuiState::ConfigSnapshot) == 112,
              "GuiState::ConfigSnapshot size changed; audit From()/ApplyTo() implementations below");
#endif

inline GuiState::ConfigSnapshot GuiState::ConfigSnapshot::From(const GuiState& state) {
  // Explicit per-field assignment (symmetric with ApplyTo). Avoids aggregate
  // initialization so that when ConfigSnapshot gains a field, the sizeof guard
  // above fires AND reviewers see the obvious gap between From and ApplyTo.
  ConfigSnapshot s;
  s.layers = state.layers;
  s.sun = state.sun;
  s.sim = state.sim;
  s.renderer = state.renderer;
  return s;
}

inline void GuiState::ConfigSnapshot::ApplyTo(GuiState& state) const {
  state.layers = layers;
  state.sun = sun;
  state.sim = sim;
  state.renderer = renderer;
}

inline GuiState InitDefaultState() {
  GuiState s;

  // One default layer with one entry (prism, random orientation, no filter)
  Layer layer;
  EntryCard entry;
  // CrystalConfig defaults: prism, height=1, uniform random orientation
  layer.entries.push_back(entry);
  s.layers.push_back(layer);

  // Default renderer is the default-constructed GuiState::renderer; no ID or index needed.

  return s;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_STATE_HPP
