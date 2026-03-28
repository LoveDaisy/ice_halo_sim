#ifndef LUMICE_GUI_STATE_HPP
#define LUMICE_GUI_STATE_HPP

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lumice::gui {

// Crystal type
enum class CrystalType { kPrism, kPyramid };

// Axis distribution type (matches PRD: Gauss or Uniform only for MVP)
enum class AxisDistType { kGauss, kUniform };

// Aspect ratio presets for preview window
enum class AspectPreset { kFree, k16x9, k3x2, k4x3, k1x1, kMatchBg };
inline const char* const kAspectPresetNames[] = { "Free", "16:9", "3:2", "4:3", "1:1", "Match Background" };
constexpr int kAspectPresetCount = 6;
static_assert(sizeof(kAspectPresetNames) / sizeof(kAspectPresetNames[0]) == kAspectPresetCount,
              "kAspectPresetNames must match kAspectPresetCount");

struct AxisDist {
  AxisDistType type = AxisDistType::kUniform;
  float mean = 0.0f;
  float std = 0.0f;  // Gauss: standard deviation; Uniform: half-range
};

struct CrystalConfig {
  int id = 0;
  CrystalType type = CrystalType::kPrism;

  // Prism
  float height = 1.0f;

  // Pyramid
  float prism_h = 1.0f;
  float upper_h = 0.2f;
  float lower_h = 0.2f;
  int upper_indices[3] = { 1, 0, 1 };
  int lower_indices[3] = { 1, 0, 1 };

  // Axis distribution (all default to uniform full rotation)
  AxisDist zenith{ AxisDistType::kUniform, 0.0f, 360.0f };
  AxisDist azimuth{ AxisDistType::kUniform, 0.0f, 360.0f };
  AxisDist roll{ AxisDistType::kUniform, 0.0f, 360.0f };
};

struct SunConfig {
  float altitude = 20.0f;
  float azimuth = 0.0f;
  float diameter = 0.5f;
  int spectrum_index = 2;  // Index into kSpectrumNames: 0=D50,1=D55,2=D65,3=D75,4=A,5=E
};

struct SimConfig {
  float ray_num_millions = 1.0f;
  int max_hits = 8;
  bool infinite = false;
};

struct ScatterEntry {
  int crystal_id = -1;  // -1 = not selected
  float proportion = 100.0f;
  int filter_id = -1;  // -1 = None
};

struct ScatterLayer {
  float probability = 0.0f;  // Probability of multi-scatter continuation (0 = single scatter)
  std::vector<ScatterEntry> entries;
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

inline const char* const kSpectrumNames[] = { "D50", "D55", "D65", "D75", "A", "E" };
constexpr int kSpectrumCount = 6;

inline const char* const kVisibleNames[] = { "Upper", "Lower", "Full" };
constexpr int kVisibleCount = 3;

inline const int kSimResolutions[] = { 512, 1024, 2048, 4096 };
constexpr int kSimResolutionCount = 4;

struct RenderConfig {
  int id = 0;
  int lens_type = 0;  // Index into kLensTypeNames
  float fov = 90.0f;
  float elevation = 0.0f;
  float azimuth = 0.0f;
  float roll = 0.0f;
  int sim_resolution_index = 1;  // Index into kSimResolutions (default 1024)
  int visible = 2;               // Index into kVisibleNames (0=upper, 1=lower, 2=full)
  float background[3] = { 0.0f, 0.0f, 0.0f };
  float ray_color[3] = { 1.0f, 1.0f, 1.0f };
  float opacity = 1.0f;
  float exposure_offset = 0.0f;  // EV: intensity_factor = 2^exposure_offset

  bool operator==(const RenderConfig& o) const {
    return id == o.id && lens_type == o.lens_type && fov == o.fov && elevation == o.elevation && azimuth == o.azimuth &&
           roll == o.roll && sim_resolution_index == o.sim_resolution_index && visible == o.visible &&
           std::equal(background, background + 3, o.background) && std::equal(ray_color, ray_color + 3, o.ray_color) &&
           opacity == o.opacity && exposure_offset == o.exposure_offset;
  }
  bool operator!=(const RenderConfig& o) const { return !(*this == o); }
};

// Filter action
inline const char* const kFilterActionNames[] = { "Filter In", "Filter Out" };
constexpr int kFilterActionCount = 2;

struct FilterConfig {
  int id = 0;
  int action = 0;            // 0=filter_in, 1=filter_out
  std::string raypath_text;  // Comma-separated int list, e.g. "3,1,5,7,4"
  bool sym_p = true;
  bool sym_b = true;
  bool sym_d = true;
};

struct GuiState {
  // Crystals
  std::vector<CrystalConfig> crystals;
  int selected_crystal = -1;

  // Scene
  SunConfig sun;
  SimConfig sim;
  std::vector<ScatterLayer> scattering;

  // Renderers
  std::vector<RenderConfig> renderers;
  int selected_renderer = -1;

  // Filters
  std::vector<FilterConfig> filters;
  int selected_filter = -1;

  // ID counters
  int next_crystal_id = 1;
  int next_renderer_id = 1;
  int next_filter_id = 1;

  // Aspect ratio (view preference, not simulation parameter — does not call MarkDirty)
  AspectPreset aspect_preset = AspectPreset::kFree;
  bool aspect_portrait = false;

  // Background image overlay (view preference — does not call MarkDirty)
  std::filesystem::path bg_path;
  bool bg_show = false;
  float bg_alpha = 1.0f;

  // File management
  std::filesystem::path current_file_path;
  bool dirty = false;
  bool save_texture = true;  // Whether to include texture in .lmc save (UI-only, not serialized)

  void MarkDirty() {
    dirty = true;
    if (sim_state == SimState::kDone) {
      sim_state = SimState::kModified;
    }
  }

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

  // Last committed config snapshot (for Revert — config fields only, no runtime state)
  struct ConfigSnapshot {
    std::vector<CrystalConfig> crystals;
    int selected_crystal = -1;
    SunConfig sun;
    SimConfig sim;
    std::vector<ScatterLayer> scattering;
    std::vector<RenderConfig> renderers;
    int selected_renderer = -1;
    std::vector<FilterConfig> filters;
    int selected_filter = -1;
    int next_crystal_id = 1;
    int next_renderer_id = 1;
    int next_filter_id = 1;
  };
  std::optional<ConfigSnapshot> last_committed_state;
};

inline GuiState InitDefaultState() {
  GuiState s;

  // One default crystal (prism, random orientation)
  CrystalConfig c;
  c.id = s.next_crystal_id++;
  s.crystals.push_back(c);
  s.selected_crystal = 0;

  // One default renderer
  RenderConfig r;
  r.id = s.next_renderer_id++;
  s.renderers.push_back(r);
  s.selected_renderer = 0;

  // One default scattering layer with one entry
  ScatterLayer layer;
  layer.probability = 0.0f;
  ScatterEntry entry;
  entry.crystal_id = c.id;
  layer.entries.push_back(entry);
  s.scattering.push_back(layer);

  return s;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_STATE_HPP
