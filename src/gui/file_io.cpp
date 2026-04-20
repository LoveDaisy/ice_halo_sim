#include "gui/file_io.hpp"

#include <nfd.h>
#include <stb_image.h>
#include <stb_image_write.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

#include "gui/app.hpp"
#include "gui/export_fbo_renderer.hpp"
#include "gui/gl_capture.hpp"
#include "gui/gl_common.h"
#include "gui/gui_logger.hpp"
#include "gui/gui_state.hpp"
#include "gui/preview_renderer.hpp"
#include "util/path_utils.hpp"

namespace lumice::gui {

using json = nlohmann::json;

// Convert Miller index (i1, i4) to wedge angle in degrees. Returns default (28.0) if i1 == 0.
static float MillerToAlpha(int i1, int i4) {
  constexpr float kSqrt3_2 = 0.866025403784f;
  constexpr float kIceCrystalC = 1.629f;
  constexpr float kRadToDeg = 57.2957795131f;
  if (i1 == 0) {
    return 28.0f;
  }
  return std::atan(kSqrt3_2 * i4 / i1 / kIceCrystalC) * kRadToDeg;
}

// Lens type JSON names (shared by Core config and GuiState JSON)
static const char* kLensTypeJsonNames[] = { "linear",
                                            "fisheye_equal_area",
                                            "fisheye_equidistant",
                                            "fisheye_stereographic",
                                            "dual_fisheye_equal_area",
                                            "dual_fisheye_equidistant",
                                            "dual_fisheye_stereographic",
                                            "rectangular" };

static const char* kVisibleJsonNames[] = { "upper", "lower", "full", "front" };
static_assert(sizeof(kVisibleJsonNames) / sizeof(kVisibleJsonNames[0]) == kVisibleCount,
              "kVisibleJsonNames must match kVisibleCount");
static const char* kAspectPresetJsonNames[] = { "free", "16:9", "3:2", "4:3", "1:1", "match_background" };
static_assert(sizeof(kAspectPresetJsonNames) / sizeof(kAspectPresetJsonNames[0]) == kAspectPresetCount,
              "kAspectPresetJsonNames must match kAspectPresetCount");


// ========== Shared helpers ==========

std::vector<int> ParseRaypathText(const std::string& text) {
  std::vector<int> result;
  // Normalize: replace ',' with '-' so both separators are accepted
  std::string normalized = text;
  for (auto& c : normalized) {
    if (c == ',')
      c = '-';
  }
  std::istringstream iss(normalized);
  std::string token;
  while (std::getline(iss, token, '-')) {
    if (token.empty())
      continue;
    try {
      int val = std::stoi(token);
      if (val < 0)
        continue;
      result.push_back(val);
    } catch (...) {
    }
  }
  return result;
}

static const char* AxisDistTypeToString(AxisDistType t) {
  switch (t) {
    case AxisDistType::kGauss:
      return "gauss";
    case AxisDistType::kUniform:
      return "uniform";
    case AxisDistType::kZigzag:
      return "zigzag";
    case AxisDistType::kLaplacian:
      return "laplacian";
    case AxisDistType::kGaussLegacy:
      return "gauss_legacy";
    default:
      GUI_LOG_ERROR("[FileIO] Unknown AxisDistType: {}", static_cast<int>(t));
      return "gauss";
  }
}

static json SerializeAxisDist(const AxisDist& a) {
  json j;
  j["type"] = AxisDistTypeToString(a.type);
  j["mean"] = a.mean;
  j["std"] = a.std;
  return j;
}

// Field-sync guard for SerializeCrystal.
// If CrystalConfig gains/loses a field, sizeof changes and this fires. Developer must then
// audit BOTH SerializeCrystal (below) and FillCrystalParam (further down this file) for
// same-file sync. Platform-gated because std::string size varies across C++ stdlib impls;
// this mirror is a "Apple Silicon + libc++ reminder", not a cross-platform contract.
//
// Audited fields (2026-04 snapshot):
//   name (serialize: yes, c-api: no), type (yes/yes), height (prism-only/yes),
//   prism_h, upper_h, lower_h (pyramid-only/yes),
//   upper_alpha, lower_alpha (pyramid-only/yes),
//   face_distance[6] (conditional/yes), zenith, azimuth, roll (yes/yes).
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(CrystalConfig) == 112,
              "CrystalConfig size changed; audit SerializeCrystal and FillCrystalParam for new/renamed fields");
#endif
static json SerializeCrystal(const CrystalConfig& c, int id) {
  json j;
  j["id"] = id;
  if (!c.name.empty()) {
    j["name"] = c.name;
  }

  if (c.type == CrystalType::kPrism) {
    j["type"] = "prism";
    j["shape"]["height"] = c.height;
  } else {
    j["type"] = "pyramid";
    j["shape"]["prism_h"] = c.prism_h;
    j["shape"]["upper_h"] = c.upper_h;
    j["shape"]["lower_h"] = c.lower_h;
    j["shape"]["upper_wedge_angle"] = c.upper_alpha;
    j["shape"]["lower_wedge_angle"] = c.lower_alpha;
  }

  // face_distance: only write when non-default (not all 1.0)
  bool is_default_fd = true;
  for (int i = 0; i < 6; i++) {
    if (std::abs(c.face_distance[i] - 1.0f) > 1e-6f) {
      is_default_fd = false;
      break;
    }
  }
  if (!is_default_fd) {
    j["shape"]["face_distance"] = { c.face_distance[0], c.face_distance[1], c.face_distance[2],
                                    c.face_distance[3], c.face_distance[4], c.face_distance[5] };
  }

  j["axis"]["zenith"] = SerializeAxisDist(c.zenith);
  j["axis"]["azimuth"] = SerializeAxisDist(c.azimuth);
  j["axis"]["roll"] = SerializeAxisDist(c.roll);

  return j;
}

static json SerializeFilterForGui(const FilterConfig& f, int id) {
  json j;
  j["id"] = id;
  if (!f.name.empty()) {
    j["name"] = f.name;
  }
  j["action"] = f.action == 0 ? "filter_in" : "filter_out";
  j["raypath_text"] = f.raypath_text;
  j["sym_p"] = f.sym_p;
  j["sym_b"] = f.sym_b;
  j["sym_d"] = f.sym_d;
  return j;
}

static json SerializeFilterForCore(const FilterConfig& f, int id) {
  json j;
  j["id"] = id;
  j["type"] = "raypath";
  j["action"] = f.action == 0 ? "filter_in" : "filter_out";

  j["raypath"] = ParseRaypathText(f.raypath_text);

  std::string sym;
  if (f.sym_p)
    sym += "P";
  if (f.sym_b)
    sym += "B";
  if (f.sym_d)
    sym += "D";
  if (!sym.empty()) {
    j["symmetry"] = sym;
  }

  return j;
}

static AxisDistType ParseAxisDistType(const std::string& t) {
  if (t == "gauss")
    return AxisDistType::kGauss;
  if (t == "uniform")
    return AxisDistType::kUniform;
  if (t == "zigzag")
    return AxisDistType::kZigzag;
  if (t == "laplacian")
    return AxisDistType::kLaplacian;
  if (t == "gauss_legacy")
    return AxisDistType::kGaussLegacy;
  GUI_LOG_ERROR("[FileIO] Unknown axis dist type '{}', falling back to gauss", t);
  return AxisDistType::kGauss;
}

static AxisDist ParseAxisDist(const json& j) {
  AxisDist a;
  if (j.is_number()) {
    a.type = AxisDistType::kGauss;
    a.mean = j.get<float>();
    a.std = 0.0f;
  } else if (j.is_object()) {
    auto t = j.value("type", "gauss");
    a.type = ParseAxisDistType(t);
    a.mean = j.value("mean", 0.0f);
    a.std = j.value("std", 0.0f);
  }
  return a;
}

static CrystalConfig ParseCrystal(const json& j) {
  CrystalConfig c;
  c.name = j.value("name", std::string{});

  auto type_str = j.value("type", "prism");
  c.type = (type_str == "pyramid") ? CrystalType::kPyramid : CrystalType::kPrism;

  if (j.contains("shape")) {
    auto& s = j.at("shape");
    if (c.type == CrystalType::kPrism) {
      if (s.contains("height")) {
        if (s["height"].is_number()) {
          c.height = s["height"].get<float>();
        } else if (s["height"].is_object()) {
          c.height = s["height"].value("mean", 1.0f);
        }
      }
    } else {
      c.prism_h = s.value("prism_h", 1.0f);
      c.upper_h = s.value("upper_h", 0.0f);
      c.lower_h = s.value("lower_h", 0.0f);
      // Wedge angle: prefer "upper_wedge_angle", fallback to "upper_indices" conversion
      if (s.contains("upper_wedge_angle") && s["upper_wedge_angle"].is_number()) {
        c.upper_alpha = s["upper_wedge_angle"].get<float>();
      } else if (s.contains("upper_indices") && s["upper_indices"].is_array() && s["upper_indices"].size() == 3) {
        c.upper_alpha = MillerToAlpha(s["upper_indices"][0].get<int>(), s["upper_indices"][2].get<int>());
      }
      if (s.contains("lower_wedge_angle") && s["lower_wedge_angle"].is_number()) {
        c.lower_alpha = s["lower_wedge_angle"].get<float>();
      } else if (s.contains("lower_indices") && s["lower_indices"].is_array() && s["lower_indices"].size() == 3) {
        c.lower_alpha = MillerToAlpha(s["lower_indices"][0].get<int>(), s["lower_indices"][2].get<int>());
      }
    }
    // face_distance: common to both Prism and Pyramid
    if (s.contains("face_distance") && s["face_distance"].is_array()) {
      size_t n = std::min(s["face_distance"].size(), static_cast<size_t>(6));
      for (size_t i = 0; i < n; i++) {
        auto& elem = s["face_distance"][i];
        if (elem.is_number()) {
          c.face_distance[i] = elem.get<float>();
        } else if (elem.is_object()) {
          c.face_distance[i] = elem.value("mean", 1.0f);
        }
      }
    }
  }

  if (j.contains("axis")) {
    auto& ax = j.at("axis");
    if (ax.contains("zenith"))
      c.zenith = ParseAxisDist(ax["zenith"]);
    if (ax.contains("azimuth"))
      c.azimuth = ParseAxisDist(ax["azimuth"]);
    if (ax.contains("roll"))
      c.roll = ParseAxisDist(ax["roll"]);
  }

  return c;
}

static int LensTypeFromString(const std::string& s) {
  for (int i = 0; i < kLensTypeCount; i++) {
    if (s == kLensTypeJsonNames[i])
      return i;
  }
  return 0;  // default: linear
}

static int VisibleFromString(const std::string& s) {
  for (int i = 0; i < kVisibleCount; i++) {
    if (s == kVisibleJsonNames[i])
      return i;
  }
  return 2;  // default: full
}

static int SpectrumFromString(const std::string& s) {
  for (int i = 0; i < kSpectrumCount; i++) {
    if (s == kSpectrumNames[i])
      return i;
  }
  return 2;  // default: D65
}

static AspectPreset AspectPresetFromString(const std::string& s) {
  for (int i = 0; i < kAspectPresetCount; i++) {
    if (s == kAspectPresetJsonNames[i]) {
      return static_cast<AspectPreset>(i);
    }
  }
  return AspectPreset::kFree;  // default: free
}

static int SimResolutionIndexFromValue(int value) {
  for (int i = 0; i < kSimResolutionCount; i++) {
    if (kSimResolutions[i] == value)
      return i;
  }
  return 1;  // default: 1024
}

// ========== GUI JSON Renderer Helpers ==========
// Shared between SerializeGuiStateJson and DeserializeGuiStateJson.

static json SerializeRendererForGui(const RenderConfig& r) {
  json jr;
  jr["lens_type"] = kLensTypeJsonNames[r.lens_type];
  jr["fov"] = r.fov;
  jr["elevation"] = r.elevation;
  jr["azimuth"] = r.azimuth;
  jr["roll"] = r.roll;
  jr["sim_resolution"] = kSimResolutions[r.sim_resolution_index];
  jr["visible"] = kVisibleJsonNames[r.visible];
  jr["background"] = { r.background[0], r.background[1], r.background[2] };
  jr["ray_color"] = { r.ray_color[0], r.ray_color[1], r.ray_color[2] };
  jr["opacity"] = r.opacity;
  jr["exposure_offset"] = r.exposure_offset;
  return jr;
}

static RenderConfig ParseRendererFromGuiJson(const json& jr) {
  RenderConfig r;
  r.lens_type = LensTypeFromString(jr.value("lens_type", "linear"));
  r.fov = jr.value("fov", RenderConfig{}.fov);
  r.elevation = jr.value("elevation", RenderConfig{}.elevation);
  r.azimuth = jr.value("azimuth", RenderConfig{}.azimuth);
  r.roll = jr.value("roll", RenderConfig{}.roll);
  r.sim_resolution_index = SimResolutionIndexFromValue(jr.value("sim_resolution", 1024));
  r.visible = VisibleFromString(jr.value("visible", "full"));
  if (jr.contains("background") && jr["background"].is_array() && jr["background"].size() == 3) {
    for (int i = 0; i < 3; i++)
      r.background[i] = jr["background"][i].get<float>();
  }
  if (jr.contains("ray_color") && jr["ray_color"].is_array() && jr["ray_color"].size() == 3) {
    for (int i = 0; i < 3; i++)
      r.ray_color[i] = jr["ray_color"][i].get<float>();
  }
  r.opacity = jr.value("opacity", RenderConfig{}.opacity);
  r.exposure_offset = jr.value("exposure_offset", RenderConfig{}.exposure_offset);
  return r;
}

// ========== GUI JSON Filter Helper ==========
// Shared between new-format and old-format .lmc deserialization paths.
// Both paths use identical JSON keys: name, action, raypath_text, sym_p, sym_b, sym_d.

static FilterConfig ParseFilterFromGuiJson(const json& jf) {
  FilterConfig f;
  f.name = jf.value("name", std::string{});
  auto action_str = jf.value("action", "filter_in");
  f.action = (action_str == "filter_out") ? 1 : 0;
  f.raypath_text = jf.value("raypath_text", FilterConfig{}.raypath_text);
  f.sym_p = jf.value("sym_p", FilterConfig{}.sym_p);
  f.sym_b = jf.value("sym_b", FilterConfig{}.sym_b);
  f.sym_d = jf.value("sym_d", FilterConfig{}.sym_d);
  return f;
}


// ========== Core Config Serialization (for LUMICE_CommitConfig) ==========

std::string SerializeCoreConfig(const GuiState& state) {
  json root;

  // Flatten layers into crystal/filter/scattering arrays with dynamically assigned IDs
  int next_crystal_id = 1;
  int next_filter_id = 1;
  root["crystal"] = json::array();
  root["filter"] = json::array();

  json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = state.sun.altitude;
  scene["light_source"]["diameter"] = state.sun.diameter;
  if (state.sun.spectrum_index >= 0 && state.sun.spectrum_index < kSpectrumCount) {
    scene["light_source"]["spectrum"] = kSpectrumNames[state.sun.spectrum_index];
  } else {
    scene["light_source"]["spectrum"] = "D65";
  }

  if (state.sim.infinite) {
    scene["ray_num"] = "infinite";
  } else {
    auto ray_num = static_cast<size_t>(state.sim.ray_num_millions * 1e6f);
    scene["ray_num"] = ray_num;
  }
  scene["max_hits"] = state.sim.max_hits;

  scene["scattering"] = json::array();
  for (auto& layer : state.layers) {
    json jl;
    jl["prob"] = layer.probability;
    jl["entries"] = json::array();
    for (auto& entry : layer.entries) {
      // Assign temporary crystal ID and serialize
      int cid = next_crystal_id++;
      auto jc = SerializeCrystal(entry.crystal, cid);
      root["crystal"].push_back(jc);

      json je;
      je["crystal"] = cid;
      je["proportion"] = entry.proportion;

      // Filter: only if present
      if (entry.filter) {
        int fid = next_filter_id++;
        auto jf = SerializeFilterForCore(*entry.filter, fid);
        root["filter"].push_back(jf);
        je["filter"] = fid;
      }

      jl["entries"].push_back(je);
    }
    scene["scattering"].push_back(jl);
  }
  root["scene"] = scene;

  // Render — Core always produces dual equal-area fisheye texture (full-globe, equal-area).
  // NOTE: GUI enforces single renderer; if multi-renderer support is added, revisit this
  // fixed id and loop-of-one structure.
  root["render"] = json::array();
  {
    const auto& r = state.renderer;
    json jr;
    jr["id"] = 1;
    jr["lens"]["type"] = "dual_fisheye_equal_area";
    jr["lens"]["fov"] = 180.0f;

    int res = kSimResolutions[r.sim_resolution_index];
    jr["resolution"] = { res * 2, res };

    jr["view"]["elevation"] = 0.0f;
    jr["view"]["azimuth"] = 0.0f;
    jr["view"]["roll"] = 0.0f;

    jr["visible"] = "full";
    jr["background"] = { 0.0f, 0.0f, 0.0f };
    jr["opacity"] = r.opacity;
    jr["intensity_factor"] = std::pow(2.0f, r.exposure_offset);
    jr["norm_mode"] = state.norm_mode;
    jr["overlap"] = kDualFisheyeOverlap;

    root["render"].push_back(jr);
  }

  return root.dump(2);
}


// ========== Fill LUMICE_Config C struct (for LUMICE_CommitConfigStruct) ==========

static void FillAxisDist(const AxisDist& src, LUMICE_AxisDist* dst) {
  static_assert(static_cast<int>(AxisDistType::kCount) == 5, "Update FillAxisDist when adding new AxisDistType");
  switch (src.type) {
    case AxisDistType::kGauss:
      dst->type = LUMICE_AXIS_DIST_GAUSS;
      break;
    case AxisDistType::kUniform:
      dst->type = LUMICE_AXIS_DIST_UNIFORM;
      break;
    case AxisDistType::kZigzag:
      dst->type = LUMICE_AXIS_DIST_ZIGZAG;
      break;
    case AxisDistType::kLaplacian:
      dst->type = LUMICE_AXIS_DIST_LAPLACIAN;
      break;
    case AxisDistType::kGaussLegacy:
      dst->type = LUMICE_AXIS_DIST_GAUSS_LEGACY;
      break;
    default:
      dst->type = LUMICE_AXIS_DIST_GAUSS;
      break;
  }
  dst->mean = src.mean;
  dst->std = src.std;
}

// Helper: fill a LUMICE_CrystalParam from GUI CrystalConfig with a given ID.
// Field-sync guard: see the static_assert(sizeof(CrystalConfig) == 112) near
// SerializeCrystal above. One copy guards both functions (same TU, identical
// condition); this comment keeps the pairing obvious to readers.
static void FillCrystalParam(const CrystalConfig& c, int id, LUMICE_CrystalParam* dst) {
  dst->id = id;
  dst->type = c.type == CrystalType::kPrism ? 0 : 1;
  dst->height = c.height;
  dst->prism_h = c.prism_h;
  dst->upper_h = c.upper_h;
  dst->lower_h = c.lower_h;
  dst->upper_wedge_angle = c.upper_alpha;
  dst->lower_wedge_angle = c.lower_alpha;
  std::copy(std::begin(c.face_distance), std::end(c.face_distance), dst->face_distance);
  FillAxisDist(c.zenith, &dst->zenith);
  FillAxisDist(c.azimuth, &dst->azimuth);
  FillAxisDist(c.roll, &dst->roll);
}

// Helper: fill a LUMICE_FilterParam from GUI FilterConfig with a given ID
static void FillFilterParam(const FilterConfig& f, int id, LUMICE_FilterParam* dst) {
  dst->id = id;
  dst->action = f.action;
  dst->symmetry = (f.sym_p ? 1 : 0) | (f.sym_b ? 2 : 0) | (f.sym_d ? 4 : 0);
  auto rp = ParseRaypathText(f.raypath_text);
  dst->raypath_count = static_cast<int>(std::min(rp.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_RAYPATH_LEN)));
  for (int k = 0; k < dst->raypath_count; k++) {
    dst->raypath[k] = rp[k];
  }
}

void FillLumiceConfig(const GuiState& state, LUMICE_Config* out) {
  std::memset(out, 0, sizeof(LUMICE_Config));

  // Flatten layers into crystals/filters/scattering with dynamically assigned IDs
  int next_crystal_id = 1;
  int next_filter_id = 1;
  int crystal_idx = 0;
  int filter_idx = 0;

  out->scatter_count =
      static_cast<int>(std::min(state.layers.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_SCATTER_LAYERS)));
  for (int i = 0; i < out->scatter_count; i++) {
    const auto& layer = state.layers[i];
    auto& dst_layer = out->scattering[i];
    dst_layer.probability = layer.probability;
    dst_layer.entry_count =
        static_cast<int>(std::min(layer.entries.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_SCATTER_ENTRIES)));
    for (int k = 0; k < dst_layer.entry_count; k++) {
      const auto& entry = layer.entries[k];

      // Assign crystal (truncate entries if crystal array is full)
      if (crystal_idx >= LUMICE_MAX_CONFIG_CRYSTALS) {
        dst_layer.entry_count = k;
        break;
      }
      int cid = next_crystal_id++;
      FillCrystalParam(entry.crystal, cid, &out->crystals[crystal_idx++]);
      dst_layer.entries[k].crystal_id = cid;
      dst_layer.entries[k].proportion = entry.proportion;

      // Assign filter (if present; omit if filter array is full)
      if (entry.filter && filter_idx < LUMICE_MAX_CONFIG_FILTERS) {
        int fid = next_filter_id++;
        FillFilterParam(*entry.filter, fid, &out->filters[filter_idx++]);
        dst_layer.entries[k].filter_id = fid;
      } else {
        dst_layer.entries[k].filter_id = -1;
      }
    }
  }
  out->crystal_count = crystal_idx;
  out->filter_count = filter_idx;

  // Renderer — single renderer always emitted with fixed id=1.
  // NOTE: GUI enforces single renderer; if multi-renderer support is added, revisit this
  // fixed id and count=1.
  out->renderer_count = 1;
  {
    const auto& r = state.renderer;
    auto& dst = out->renderers[0];
    dst.id = 1;
    int res = kSimResolutions[r.sim_resolution_index];
    dst.resolution_w = res * 2;
    dst.resolution_h = res;
    dst.opacity = r.opacity;
    dst.intensity_factor = std::pow(2.0f, r.exposure_offset);
    dst.norm_mode = state.norm_mode;
    dst.overlap = kDualFisheyeOverlap;
  }

  // Scene: light source
  out->sun_altitude = state.sun.altitude;
  out->sun_azimuth = 0.0f;
  out->sun_diameter = state.sun.diameter;
  if (state.sun.spectrum_index >= 0 && state.sun.spectrum_index < kSpectrumCount) {
    out->spectrum = kSpectrumNames[state.sun.spectrum_index];
  } else {
    out->spectrum = "D65";
  }

  // Scene: simulation
  out->infinite = state.sim.infinite ? 1 : 0;
  out->ray_num = static_cast<unsigned long>(state.sim.ray_num_millions * 1e6f);
  out->max_hits = state.sim.max_hits;
}


// ========== Core JSON Deserialization (for JSON import) ==========
// Handles Core config JSON format: root has "crystal"/"filter"/"scene"/"render" keys.
// Crystal/filter are referenced by ID in scene.scattering entries; this function converts
// the ID references into the copy-model (EntryCard/Layer) by looking up each scatter entry's
// crystal/filter ID in the parsed maps.
// NOTE: This is NOT the .lmc GUI state format — see DeserializeGuiStateJson for that.

bool DeserializeFromJson(const std::string& json_str, GuiState& state) {
  json root;
  try {
    root = json::parse(json_str);
  } catch (...) {
    return false;
  }

  state = GuiState{};

  // Parse crystals and filters into temporary ID-indexed maps for scatter entry lookup
  std::map<int, CrystalConfig> crystal_map;
  if (root.contains("crystal") && root["crystal"].is_array()) {
    for (auto& jc : root["crystal"]) {
      int id = jc.value("id", 0);
      crystal_map[id] = ParseCrystal(jc);
    }
  }

  std::map<int, FilterConfig> filter_map;
  if (root.contains("filter") && root["filter"].is_array()) {
    for (auto& jf : root["filter"]) {
      auto type_str = jf.value("type", "");
      if (type_str != "raypath")
        continue;
      FilterConfig f;
      int id = jf.value("id", 0);
      f.name = jf.value("name", std::string{});
      auto action_str = jf.value("action", "filter_in");
      f.action = (action_str == "filter_out") ? 1 : 0;
      if (jf.contains("raypath") && jf["raypath"].is_array()) {
        std::string text;
        for (size_t i = 0; i < jf["raypath"].size(); i++) {
          if (i > 0)
            text += kRaypathSepStr;
          text += std::to_string(jf["raypath"][i].get<int>());
        }
        f.raypath_text = text;
      }
      auto sym = jf.value("symmetry", "");
      f.sym_p = (sym.find('P') != std::string::npos);
      f.sym_b = (sym.find('B') != std::string::npos);
      f.sym_d = (sym.find('D') != std::string::npos);
      filter_map[id] = f;
    }
  }

  // Scene
  if (root.contains("scene")) {
    auto& js = root["scene"];

    if (js.contains("light_source")) {
      auto& jl = js["light_source"];
      state.sun.altitude = jl.value("altitude", 20.0f);
      state.sun.diameter = jl.value("diameter", 0.5f);
      if (jl.contains("spectrum") && jl["spectrum"].is_string()) {
        state.sun.spectrum_index = SpectrumFromString(jl["spectrum"].get<std::string>());
      }
    }

    if (js.contains("ray_num")) {
      if (js["ray_num"].is_string() && js["ray_num"].get<std::string>() == "infinite") {
        state.sim.infinite = true;
      } else if (js["ray_num"].is_number()) {
        state.sim.ray_num_millions = static_cast<float>(js["ray_num"].get<size_t>()) / 1e6f;
      }
    }
    state.sim.max_hits = js.value("max_hits", 8);

    // Convert ID-referenced scattering to copy-model layers
    if (js.contains("scattering") && js["scattering"].is_array()) {
      for (auto& jlayer : js["scattering"]) {
        Layer layer;
        layer.probability = jlayer.value("prob", 1.0f);
        if (jlayer.contains("entries") && jlayer["entries"].is_array()) {
          for (auto& je : jlayer["entries"]) {
            EntryCard entry;
            int crystal_id = je.value("crystal", -1);
            if (crystal_map.count(crystal_id)) {
              entry.crystal = crystal_map[crystal_id];
            }
            entry.proportion = je.value("proportion", 1.0f);
            int filter_id = je.value("filter", -1);
            if (filter_id >= 0 && filter_map.count(filter_id)) {
              entry.filter = filter_map[filter_id];
            }
            layer.entries.push_back(entry);
          }
        }
        state.layers.push_back(layer);
      }
    }
  }

  // Render — GUI uses single renderer; ignore additional array entries and malformed keys.
  // If root["render"] is missing, not an array, or empty, keep state.renderer at defaults.
  if (root.contains("render") && root["render"].is_array() && !root["render"].empty()) {
    const auto& jr = root["render"][0];
    RenderConfig r;

    if (jr.contains("lens")) {
      r.lens_type = LensTypeFromString(jr["lens"].value("type", "linear"));
      r.fov = jr["lens"].value("fov", 90.0f);
    }

    if (jr.contains("resolution") && jr["resolution"].is_array() && jr["resolution"].size() == 2) {
      int h = jr["resolution"][1].get<int>();
      for (int i = 0; i < kSimResolutionCount; i++) {
        if (kSimResolutions[i] >= h) {
          r.sim_resolution_index = i;
          break;
        }
      }
    }

    if (jr.contains("view")) {
      r.elevation = jr["view"].value("elevation", 0.0f);
      r.azimuth = jr["view"].value("azimuth", 0.0f);
      r.roll = jr["view"].value("roll", 0.0f);
    }

    r.visible = VisibleFromString(jr.value("visible", "upper"));

    if (jr.contains("background") && jr["background"].is_array() && jr["background"].size() == 3) {
      for (int i = 0; i < 3; i++)
        r.background[i] = jr["background"][i].get<float>();
    }
    if (jr.contains("ray_color") && jr["ray_color"].is_array() && jr["ray_color"].size() == 3) {
      for (int i = 0; i < 3; i++)
        r.ray_color[i] = jr["ray_color"][i].get<float>();
    }
    r.opacity = jr.value("opacity", 1.0f);
    float ifactor = jr.value("intensity_factor", 1.0f);
    r.exposure_offset = std::log2(std::max(ifactor, 1e-6f));

    state.renderer = r;
  } else {
    GUI_LOG_WARNING("[GUI] DeserializeFromJson: no render entries; using default renderer");
  }

  return true;
}


// ========== Full GuiState JSON Serialization (for .lmc file) ==========

std::string SerializeGuiStateJson(const GuiState& state) {
  json root;

  // Layers (copy model: each entry embeds its crystal/filter)
  root["layers"] = json::array();
  int ser_crystal_id = 1;
  int ser_filter_id = 1;
  for (auto& layer : state.layers) {
    json jl;
    jl["prob"] = layer.probability;
    jl["entries"] = json::array();
    for (auto& entry : layer.entries) {
      json je;
      je["crystal"] = SerializeCrystal(entry.crystal, ser_crystal_id++);
      je["proportion"] = entry.proportion;
      if (entry.filter) {
        je["filter"] = SerializeFilterForGui(*entry.filter, ser_filter_id++);
      }
      jl["entries"].push_back(je);
    }
    root["layers"].push_back(jl);
  }

  // Sun
  json sun;
  sun["altitude"] = state.sun.altitude;
  sun["diameter"] = state.sun.diameter;
  if (state.sun.spectrum_index >= 0 && state.sun.spectrum_index < kSpectrumCount) {
    sun["spectrum"] = kSpectrumNames[state.sun.spectrum_index];
  } else {
    sun["spectrum"] = "D65";
  }
  root["sun"] = sun;

  // Sim
  json sim;
  sim["ray_num_millions"] = state.sim.ray_num_millions;
  sim["max_hits"] = state.sim.max_hits;
  sim["infinite"] = state.sim.infinite;
  root["sim"] = sim;

  // Renderer (copy model: single renderer embedded directly)
  root["renderer"] = SerializeRendererForGui(state.renderer);

  // Aspect ratio (view preference)
  auto preset_idx = static_cast<int>(state.aspect_preset);
  root["aspect_ratio"] = kAspectPresetJsonNames[preset_idx];
  root["aspect_portrait"] = state.aspect_portrait;

  // Background image overlay
  root["bg_path"] = PathToU8(state.bg_path);
  root["bg_show"] = state.bg_show;
  root["bg_alpha"] = state.bg_alpha;

  // Auxiliary line overlay
  root["overlay_horizon"] = state.show_horizon;
  root["overlay_grid"] = state.show_grid;
  root["overlay_sun_circles"] = state.show_sun_circles;
  root["overlay_sun_circle_angles"] = state.sun_circle_angles;
  root["overlay_horizon_color"] = { state.horizon_color[0], state.horizon_color[1], state.horizon_color[2] };
  root["overlay_grid_color"] = { state.grid_color[0], state.grid_color[1], state.grid_color[2] };
  root["overlay_sun_circles_color"] = { state.sun_circles_color[0], state.sun_circles_color[1],
                                        state.sun_circles_color[2] };
  root["overlay_horizon_alpha"] = state.horizon_alpha;
  root["overlay_grid_alpha"] = state.grid_alpha;
  root["overlay_sun_circles_alpha"] = state.sun_circles_alpha;

  // Panel state
  root["right_panel_collapsed"] = state.right_panel_collapsed;

  // Normalization mode (display preference)
  root["norm_mode"] = state.norm_mode;

  return root.dump(2);
}


// ========== Full GuiState JSON Deserialization ==========

bool DeserializeGuiStateJson(const std::string& json_str, GuiState& state) {
  json root;
  try {
    root = json::parse(json_str);
  } catch (...) {
    return false;
  }

  state = GuiState{};

  // Layers: new copy-model format ("layers" key) with backward compat for old ID-referenced format
  if (root.contains("layers") && root["layers"].is_array()) {
    for (auto& jl : root["layers"]) {
      Layer layer;
      layer.probability = jl.value("prob", 0.0f);
      if (jl.contains("entries") && jl["entries"].is_array()) {
        for (auto& je : jl["entries"]) {
          EntryCard entry;
          if (je.contains("crystal")) {
            entry.crystal = ParseCrystal(je["crystal"]);
          }
          entry.proportion = je.value("proportion", 100.0f);
          if (je.contains("filter") && !je["filter"].is_null()) {
            entry.filter = ParseFilterFromGuiJson(je["filter"]);
          }
          layer.entries.push_back(entry);
        }
      }
      state.layers.push_back(layer);
    }
  } else if (root.contains("crystals") && root.contains("scattering")) {
    // Backward compat: old .lmc format with ID-referenced crystals/filters/scattering
    std::map<int, CrystalConfig> crystal_map;
    if (root["crystals"].is_array()) {
      for (auto& jc : root["crystals"]) {
        int id = jc.value("id", 0);
        crystal_map[id] = ParseCrystal(jc);
      }
    }
    std::map<int, FilterConfig> filter_map;
    if (root.contains("filters") && root["filters"].is_array()) {
      for (auto& jf : root["filters"]) {
        int id = jf.value("id", 0);
        filter_map[id] = ParseFilterFromGuiJson(jf);
      }
    }
    if (root["scattering"].is_array()) {
      for (auto& jl : root["scattering"]) {
        Layer layer;
        layer.probability = jl.value("prob", 0.0f);
        if (jl.contains("entries") && jl["entries"].is_array()) {
          for (auto& je : jl["entries"]) {
            EntryCard entry;
            int crystal_id = je.value("crystal_id", -1);
            if (crystal_map.count(crystal_id)) {
              entry.crystal = crystal_map[crystal_id];
            }
            entry.proportion = je.value("proportion", 100.0f);
            int filter_id = je.value("filter_id", -1);
            if (filter_id >= 0 && filter_map.count(filter_id)) {
              entry.filter = filter_map[filter_id];
            }
            layer.entries.push_back(entry);
          }
        }
        state.layers.push_back(layer);
      }
    }
  }

  // Sun
  if (root.contains("sun")) {
    auto& js = root["sun"];
    state.sun.altitude = js.value("altitude", 20.0f);
    state.sun.diameter = js.value("diameter", 0.5f);
    state.sun.spectrum_index = SpectrumFromString(js.value("spectrum", "D65"));
  }

  // Sim
  if (root.contains("sim")) {
    auto& js = root["sim"];
    state.sim.ray_num_millions = js.value("ray_num_millions", SimConfig{}.ray_num_millions);
    state.sim.max_hits = js.value("max_hits", SimConfig{}.max_hits);
    state.sim.infinite = js.value("infinite", SimConfig{}.infinite);
  }

  // Renderer (copy model).
  // New format: root["renderer"] is a single object.
  // Legacy format: root["renderers"] is an array; take first element; ignore
  // selected_renderer_id/next_renderer_id (no longer meaningful).
  if (root.contains("renderer") && root["renderer"].is_object()) {
    state.renderer = ParseRendererFromGuiJson(root["renderer"]);
  } else if (root.contains("renderers") && root["renderers"].is_array() && !root["renderers"].empty()) {
    state.renderer = ParseRendererFromGuiJson(root["renderers"][0]);
  } else {
    // Neither new-format "renderer" object nor legacy "renderers" array with data found.
    // Symmetric with DeserializeFromJson's empty-render-array branch, so that malformed or
    // legacy files leave an observable trace for debugging.
    GUI_LOG_WARNING("[GUI] DeserializeGuiStateJson: no renderer key found; using default renderer");
  }

  // Aspect ratio (view preference, defaults to Free for old files)
  state.aspect_preset = AspectPresetFromString(root.value("aspect_ratio", "free"));
  state.aspect_portrait = root.value("aspect_portrait", false);

  // Background image overlay (backward compatible: missing fields use defaults)
  state.bg_path = PathFromU8(root.value("bg_path", std::string{}));
  state.bg_show = root.value("bg_show", false);
  state.bg_alpha = root.value("bg_alpha", 1.0f);

  // Auxiliary line overlay (backward compatible: missing fields use defaults)
  state.show_horizon = root.value("overlay_horizon", false);
  state.show_grid = root.value("overlay_grid", false);
  state.show_sun_circles = root.value("overlay_sun_circles", false);
  if (root.contains("overlay_sun_circle_angles") && root["overlay_sun_circle_angles"].is_array()) {
    state.sun_circle_angles.clear();
    for (const auto& v : root["overlay_sun_circle_angles"]) {
      if (v.is_number() && static_cast<int>(state.sun_circle_angles.size()) < kMaxSunCircles) {
        float angle = std::clamp(v.get<float>(), 0.1f, 180.0f);
        state.sun_circle_angles.push_back(angle);
      }
    }
    std::sort(state.sun_circle_angles.begin(), state.sun_circle_angles.end());
  }
  auto read_color3 = [&root](const char* key, float* out) {
    if (root.contains(key) && root[key].is_array() && root[key].size() == 3) {
      for (int i = 0; i < 3; i++) {
        out[i] = root[key][i].get<float>();
      }
    }
  };
  read_color3("overlay_horizon_color", state.horizon_color);
  read_color3("overlay_grid_color", state.grid_color);
  read_color3("overlay_sun_circles_color", state.sun_circles_color);
  state.horizon_alpha = root.value("overlay_horizon_alpha", 0.6f);
  state.grid_alpha = root.value("overlay_grid_alpha", 0.3f);
  state.sun_circles_alpha = root.value("overlay_sun_circles_alpha", 0.5f);

  // Panel state
  state.right_panel_collapsed = root.value("right_panel_collapsed", false);

  // Normalization mode (display preference, default absolute for backward compat with old .lmc files)
  state.norm_mode = root.value("norm_mode", 0);

  return true;
}


// ========== .lmc Binary File I/O ==========

// Header: 44 bytes, little-endian
// magic[4] = "LMC\0"
// version: uint32 = 1
// flags: uint32 (bit 0: has_texture)
// json_offset: uint64
// json_size: uint64
// tex_offset: uint64
// tex_size: uint64

static constexpr uint32_t kLmcMagic = 0x00434D4C;  // "LMC\0" as little-endian uint32
static constexpr uint32_t kLmcVersion = 1;
static constexpr uint32_t kLmcHeaderSize = 44;
static constexpr uint32_t kLmcFlagHasTexture = 0x1;

static void WriteU32(std::ofstream& out, uint32_t val) {
  out.write(reinterpret_cast<const char*>(&val), sizeof(val));
}

static void WriteU64(std::ofstream& out, uint64_t val) {
  out.write(reinterpret_cast<const char*>(&val), sizeof(val));
}

static bool ReadU32(std::ifstream& in, uint32_t& val) {
  return static_cast<bool>(in.read(reinterpret_cast<char*>(&val), sizeof(val)));
}

static bool ReadU64(std::ifstream& in, uint64_t& val) {
  return static_cast<bool>(in.read(reinterpret_cast<char*>(&val), sizeof(val)));
}

// stb_image_write callback: appends to std::vector<unsigned char>
static void StbWriteCallback(void* context, void* data, int size) {
  auto* buf = static_cast<std::vector<unsigned char>*>(context);
  auto* bytes = static_cast<unsigned char*>(data);
  buf->insert(buf->end(), bytes, bytes + size);
}

bool SaveLmcFile(const std::filesystem::path& path, const GuiState& state, const PreviewRenderer& preview,
                 bool save_texture) {
  std::string json_payload = SerializeGuiStateJson(state);

  // Encode texture to PNG in memory if requested
  std::vector<unsigned char> png_data;
  bool has_texture = false;
  if (save_texture && preview.HasTexture() && preview.GetTextureData() != nullptr) {
    int w = preview.GetTextureWidth();
    int h = preview.GetTextureHeight();
    int result = stbi_write_png_to_func(StbWriteCallback, &png_data, w, h, 3, preview.GetTextureData(), w * 3);
    if (result != 0) {
      has_texture = true;
    }
  }

  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return false;
  }

  // Compute offsets
  uint64_t json_offset = kLmcHeaderSize;
  auto json_size = static_cast<uint64_t>(json_payload.size());
  uint64_t tex_offset = has_texture ? json_offset + json_size : 0;
  auto tex_size = static_cast<uint64_t>(png_data.size());

  // Write header
  uint32_t flags = has_texture ? kLmcFlagHasTexture : 0;
  WriteU32(out, kLmcMagic);
  WriteU32(out, kLmcVersion);
  WriteU32(out, flags);
  WriteU64(out, json_offset);
  WriteU64(out, json_size);
  WriteU64(out, tex_offset);
  WriteU64(out, tex_size);

  // Write JSON payload
  out.write(json_payload.data(), static_cast<std::streamsize>(json_payload.size()));

  // Write texture PNG
  if (has_texture) {
    out.write(reinterpret_cast<const char*>(png_data.data()), static_cast<std::streamsize>(png_data.size()));
  }

  return out.good();
}

bool LoadLmcFile(const std::filesystem::path& path, GuiState& state, std::vector<unsigned char>& tex_data, int& tex_w,
                 int& tex_h) {
  tex_data.clear();
  tex_w = 0;
  tex_h = 0;

  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    GUI_LOG_ERROR("[LMC] Cannot open file: {}", PathToU8(path));
    return false;
  }

  // Read header
  uint32_t magic = 0;
  uint32_t version = 0;
  uint32_t flags = 0;
  uint64_t json_offset = 0;
  uint64_t json_size = 0;
  uint64_t tex_offset = 0;
  uint64_t tex_size = 0;

  if (!ReadU32(in, magic) || !ReadU32(in, version) || !ReadU32(in, flags) || !ReadU64(in, json_offset) ||
      !ReadU64(in, json_size) || !ReadU64(in, tex_offset) || !ReadU64(in, tex_size)) {
    GUI_LOG_ERROR("[LMC] Failed to read header");
    return false;
  }

  if (magic != kLmcMagic) {
    GUI_LOG_ERROR("[LMC] Invalid magic: 0x{:08x}", magic);
    return false;
  }

  if (version != kLmcVersion) {
    GUI_LOG_ERROR("[LMC] Unsupported version: {}", version);
    return false;
  }

  // Read JSON
  std::string json_payload(json_size, '\0');
  in.seekg(static_cast<std::streamoff>(json_offset));
  in.read(json_payload.data(), static_cast<std::streamsize>(json_size));
  if (!in) {
    GUI_LOG_ERROR("[LMC] Failed to read JSON section");
    return false;
  }

  if (!DeserializeGuiStateJson(json_payload, state)) {
    GUI_LOG_ERROR("[LMC] Failed to parse JSON");
    return false;
  }

  // Read texture if present
  bool flag_has_tex = (flags & kLmcFlagHasTexture) != 0;
  if (flag_has_tex) {
    if (tex_size == 0) {
      GUI_LOG_ERROR("[LMC] Texture flag set but size is 0");
      return false;
    }
    std::vector<unsigned char> png_buf(tex_size);
    in.seekg(static_cast<std::streamoff>(tex_offset));
    in.read(reinterpret_cast<char*>(png_buf.data()), static_cast<std::streamsize>(tex_size));
    if (!in) {
      GUI_LOG_ERROR("[LMC] Failed to read texture section");
      return false;
    }

    int channels = 0;
    unsigned char* decoded =
        stbi_load_from_memory(png_buf.data(), static_cast<int>(png_buf.size()), &tex_w, &tex_h, &channels, 3);
    if (!decoded) {
      GUI_LOG_ERROR("[LMC] Failed to decode texture PNG");
      return false;
    }
    size_t byte_count = static_cast<size_t>(tex_w) * tex_h * 3;
    tex_data.assign(decoded, decoded + byte_count);
    stbi_image_free(decoded);
  }

  return true;
}


// ========== Export Preview ==========

// Shared PNG writer: takes an RGBA8 top-down buffer and writes it as a PNG file.
// Centralizes stbi_write_png so callers (this module, app.cpp DoExportPreviewPng)
// share one error-handling convention.
bool WriteRgbaBufferToPng(const std::filesystem::path& path, int w, int h, const std::vector<unsigned char>& rgba) {
  if (path.empty() || w <= 0 || h <= 0 ||
      rgba.size() != static_cast<size_t>(w) * static_cast<size_t>(h) * 4) {
    return false;
  }
  auto u8path = path.u8string();
  return stbi_write_png(u8path.c_str(), w, h, 4, rgba.data(), w * 4) != 0;
}

// Thin wrapper over RenderExportToRgba: kept for binary-compatible callers in
// test/gui/ (test_gui_export, test_gui_visual, test_gui_bg). Consolidated with
// the overlay path in DoExportPreviewPng — the FBO+renderer logic lives once, in
// export_fbo_renderer.cpp.
bool ExportPreviewPng(const std::filesystem::path& path, PreviewRenderer& renderer, const PreviewViewport& vp) {
  if (vp.vp_w <= 0 || vp.vp_h <= 0 || !renderer.HasTexture()) {
    return false;
  }
  auto rgba = RenderExportToRgba(renderer, vp.params, vp.vp_w, vp.vp_h, std::nullopt);
  if (rgba.empty()) {
    return false;
  }
  return WriteRgbaBufferToPng(path, vp.vp_w, vp.vp_h, rgba);
}


bool ExportDefaultFramebufferRegionPng(const std::filesystem::path& path, int x, int y, int w, int h) {
  if (path.empty() || w <= 0 || h <= 0) {
    return false;
  }
  std::vector<unsigned char> pixels;
  if (!ReadbackGlRegionToRgba(x, y, w, h, pixels)) {
    return false;
  }
  auto u8path = path.u8string();
  return stbi_write_png(u8path.c_str(), w, h, 4, pixels.data(), w * 4) != 0;
}


// ========== Export Dual Fisheye Equal Area ==========

bool ExportDualFisheyeEqualAreaPng(const std::filesystem::path& path, const unsigned char* data, int width,
                                   int height) {
  if (path.empty() || !data || width <= 0 || height <= 0) {
    return false;
  }
  auto u8path = path.u8string();
  int result = stbi_write_png(u8path.c_str(), width, height, 3, data, width * 3);
  return result != 0;
}

// ========== Export Equirectangular ==========

bool ExportEquirectangularPng(const std::filesystem::path& path, const unsigned char* data, int width, int height) {
  if (path.empty() || !data || width <= 0 || height <= 0) {
    return false;
  }
  auto u8path = path.u8string();
  int result = stbi_write_png(u8path.c_str(), width, height, 3, data, width * 3);
  return result != 0;
}

// ========== Export Config JSON ==========

bool ExportConfigJson(const std::filesystem::path& path, const std::string& json_str) {
  if (path.empty()) {
    return false;
  }
  std::ofstream out(path);
  if (!out.is_open()) {
    return false;
  }
  out << json_str;
  return out.good();
}

// ========== File Dialogs ==========

std::filesystem::path ShowOpenDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_items[2] = { { "Lumice", "lmc" }, { "JSON Config", "json" } };
  nfdresult_t result = NFD_OpenDialog(&out_path, filter_items, 2, nullptr);
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowSaveDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "Lumice", "lmc" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "project.lmc");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportPngDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "PNG Image", "png" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "preview.png");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportDualFisheyeEqualAreaDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "PNG Image", "png" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "dual_fisheye_equal_area.png");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportEquirectangularDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "PNG Image", "png" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "equirectangular.png");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportJsonDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "JSON Config", "json" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "config.json");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowOpenImageDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_items[1] = { { "Images", "png,jpg,jpeg,bmp" } };
  nfdresult_t result = NFD_OpenDialog(&out_path, filter_items, 1, nullptr);
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

}  // namespace lumice::gui
