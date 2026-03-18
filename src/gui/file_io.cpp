#include "gui/file_io.hpp"

#include <nfd.h>
#include <stb_image.h>
#include <stb_image_write.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

#include "gui/app.hpp"
#include "gui/gl_common.h"
#include "gui/gui_state.hpp"
#include "gui/preview_renderer.hpp"

namespace lumice::gui {

using json = nlohmann::json;

// Lens type JSON names (shared by Core config and GuiState JSON)
static const char* kLensTypeJsonNames[] = { "linear",
                                            "fisheye_equal_area",
                                            "fisheye_equidistant",
                                            "fisheye_stereographic",
                                            "dual_fisheye_equal_area",
                                            "dual_fisheye_equidistant",
                                            "dual_fisheye_stereographic",
                                            "rectangular" };

static const char* kVisibleJsonNames[] = { "upper", "lower", "full" };
static const char* kAspectPresetJsonNames[] = { "free", "16:9", "3:2", "4:3", "1:1", "match_background" };
static_assert(sizeof(kAspectPresetJsonNames) / sizeof(kAspectPresetJsonNames[0]) == kAspectPresetCount,
              "kAspectPresetJsonNames must match kAspectPresetCount");


// ========== Shared helpers ==========

static json SerializeAxisDist(const AxisDist& a) {
  json j;
  j["type"] = (a.type == AxisDistType::kGauss) ? "gauss" : "uniform";
  j["mean"] = a.mean;
  j["std"] = a.std;
  return j;
}

static json SerializeCrystal(const CrystalConfig& c) {
  json j;
  j["id"] = c.id;

  if (c.type == CrystalType::kPrism) {
    j["type"] = "prism";
    j["shape"]["height"] = c.height;
  } else {
    j["type"] = "pyramid";
    j["shape"]["prism_h"] = c.prism_h;
    j["shape"]["upper_h"] = c.upper_h;
    j["shape"]["lower_h"] = c.lower_h;
    j["shape"]["upper_indices"] = { c.upper_indices[0], c.upper_indices[1], c.upper_indices[2] };
    j["shape"]["lower_indices"] = { c.lower_indices[0], c.lower_indices[1], c.lower_indices[2] };
  }

  j["axis"]["zenith"] = SerializeAxisDist(c.zenith);
  j["axis"]["azimuth"] = SerializeAxisDist(c.azimuth);
  j["axis"]["roll"] = SerializeAxisDist(c.roll);

  return j;
}

static json SerializeFilterForGui(const FilterConfig& f) {
  json j;
  j["id"] = f.id;
  j["action"] = f.action == 0 ? "filter_in" : "filter_out";
  j["raypath_text"] = f.raypath_text;
  j["sym_p"] = f.sym_p;
  j["sym_b"] = f.sym_b;
  j["sym_d"] = f.sym_d;
  return j;
}

static json SerializeFilterForCore(const FilterConfig& f) {
  json j;
  j["id"] = f.id;
  j["type"] = "raypath";
  j["action"] = f.action == 0 ? "filter_in" : "filter_out";

  std::vector<int> raypath;
  std::istringstream iss(f.raypath_text);
  std::string token;
  while (std::getline(iss, token, ',')) {
    try {
      int val = std::stoi(token);
      raypath.push_back(val);
    } catch (...) {
    }
  }
  j["raypath"] = raypath;

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

static AxisDist ParseAxisDist(const json& j) {
  AxisDist a;
  if (j.is_number()) {
    a.type = AxisDistType::kGauss;
    a.mean = j.get<float>();
    a.std = 0.0f;
  } else if (j.is_object()) {
    auto t = j.value("type", "gauss");
    a.type = (t == "uniform") ? AxisDistType::kUniform : AxisDistType::kGauss;
    a.mean = j.value("mean", 0.0f);
    a.std = j.value("std", 0.0f);
  }
  return a;
}

static CrystalConfig ParseCrystal(const json& j) {
  CrystalConfig c;
  c.id = j.value("id", 0);

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
      if (s.contains("upper_indices") && s["upper_indices"].is_array() && s["upper_indices"].size() == 3) {
        for (int i = 0; i < 3; i++)
          c.upper_indices[i] = s["upper_indices"][i].get<int>();
      }
      if (s.contains("lower_indices") && s["lower_indices"].is_array() && s["lower_indices"].size() == 3) {
        for (int i = 0; i < 3; i++)
          c.lower_indices[i] = s["lower_indices"][i].get<int>();
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

// Find index in a vector by ID, returns fallback if not found
template <typename T>
static int FindIndexById(const std::vector<T>& vec, int id, int fallback) {
  for (size_t i = 0; i < vec.size(); i++) {
    if (vec[i].id == id)
      return static_cast<int>(i);
  }
  return fallback;
}


// ========== Core Config Serialization (for LUMICE_CommitConfig) ==========

std::string SerializeCoreConfig(const GuiState& state) {
  json root;

  // Crystals
  root["crystal"] = json::array();
  for (auto& c : state.crystals) {
    root["crystal"].push_back(SerializeCrystal(c));
  }

  // Filters
  root["filter"] = json::array();
  for (auto& f : state.filters) {
    root["filter"].push_back(SerializeFilterForCore(f));
  }

  // Scene
  json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = state.sun.altitude;
  scene["light_source"]["azimuth"] = state.sun.azimuth;
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
  for (auto& layer : state.scattering) {
    json jl;
    jl["prob"] = layer.probability;
    jl["entries"] = json::array();
    for (auto& entry : layer.entries) {
      json je;
      je["crystal"] = entry.crystal_id >= 0 ? entry.crystal_id : 1;
      je["proportion"] = entry.proportion;
      if (entry.filter_id >= 0) {
        je["filter"] = entry.filter_id;
      }
      jl["entries"].push_back(je);
    }
    scene["scattering"].push_back(jl);
  }
  root["scene"] = scene;

  // Render — Core always produces a full equirectangular texture.
  root["render"] = json::array();
  for (auto& r : state.renderers) {
    json jr;
    jr["id"] = r.id;
    jr["lens"]["type"] = "rectangular";
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

    root["render"].push_back(jr);
  }

  return root.dump(2);
}


// ========== Fill LUMICE_Config C struct (for LUMICE_CommitConfigStruct) ==========

static void FillAxisDist(const AxisDist& src, LUMICE_AxisDist* dst) {
  dst->type = src.type == AxisDistType::kGauss ? 0 : 1;
  dst->mean = src.mean;
  dst->std = src.std;
}

void FillLumiceConfig(const GuiState& state, LUMICE_Config* out) {
  std::memset(out, 0, sizeof(LUMICE_Config));

  // Crystals
  out->crystal_count =
      static_cast<int>(std::min(state.crystals.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_CRYSTALS)));
  for (int i = 0; i < out->crystal_count; i++) {
    const auto& c = state.crystals[i];
    auto& dst = out->crystals[i];
    dst.id = c.id;
    dst.type = c.type == CrystalType::kPrism ? 0 : 1;
    dst.height = c.height;
    dst.prism_h = c.prism_h;
    dst.upper_h = c.upper_h;
    dst.lower_h = c.lower_h;
    std::copy(std::begin(c.upper_indices), std::end(c.upper_indices), dst.upper_indices);
    std::copy(std::begin(c.lower_indices), std::end(c.lower_indices), dst.lower_indices);
    FillAxisDist(c.zenith, &dst.zenith);
    FillAxisDist(c.azimuth, &dst.azimuth);
    FillAxisDist(c.roll, &dst.roll);
  }

  // Filters
  out->filter_count = static_cast<int>(std::min(state.filters.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_FILTERS)));
  for (int i = 0; i < out->filter_count; i++) {
    const auto& f = state.filters[i];
    auto& dst = out->filters[i];
    dst.id = f.id;
    dst.action = f.action;
    dst.symmetry = (f.sym_p ? 1 : 0) | (f.sym_b ? 2 : 0) | (f.sym_d ? 4 : 0);
    // Parse raypath_text
    std::istringstream iss(f.raypath_text);
    std::string token;
    dst.raypath_count = 0;
    while (std::getline(iss, token, ',') && dst.raypath_count < LUMICE_MAX_CONFIG_RAYPATH_LEN) {
      try {
        dst.raypath[dst.raypath_count++] = std::stoi(token);
      } catch (...) {
      }
    }
  }

  // Renderers
  out->renderer_count =
      static_cast<int>(std::min(state.renderers.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_RENDERERS)));
  for (int i = 0; i < out->renderer_count; i++) {
    const auto& r = state.renderers[i];
    auto& dst = out->renderers[i];
    dst.id = r.id;
    int res = kSimResolutions[r.sim_resolution_index];
    dst.resolution_w = res * 2;
    dst.resolution_h = res;
    dst.opacity = r.opacity;
    dst.intensity_factor = std::pow(2.0f, r.exposure_offset);
  }

  // Scene: light source
  out->sun_altitude = state.sun.altitude;
  out->sun_azimuth = state.sun.azimuth;
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

  // Scene: scattering
  out->scatter_count =
      static_cast<int>(std::min(state.scattering.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_SCATTER_LAYERS)));
  for (int i = 0; i < out->scatter_count; i++) {
    const auto& layer = state.scattering[i];
    auto& dst = out->scattering[i];
    dst.probability = layer.probability;
    dst.entry_count =
        static_cast<int>(std::min(layer.entries.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_SCATTER_ENTRIES)));
    for (int k = 0; k < dst.entry_count; k++) {
      const auto& e = layer.entries[k];
      dst.entries[k].crystal_id = e.crystal_id;
      dst.entries[k].proportion = e.proportion;
      dst.entries[k].filter_id = e.filter_id;
    }
  }
}


// ========== Core JSON Deserialization (for DoRevert) ==========

bool DeserializeFromJson(const std::string& json_str, GuiState& state) {
  json root;
  try {
    root = json::parse(json_str);
  } catch (...) {
    return false;
  }

  state = GuiState{};
  int max_id = 0;

  // Crystals
  if (root.contains("crystal") && root["crystal"].is_array()) {
    for (auto& jc : root["crystal"]) {
      auto c = ParseCrystal(jc);
      max_id = std::max(max_id, c.id);
      state.crystals.push_back(c);
    }
  }
  state.next_crystal_id = max_id + 1;
  if (!state.crystals.empty()) {
    state.selected_crystal = 0;
  }

  // Filters (only raypath for MVP, skip others)
  max_id = 0;
  if (root.contains("filter") && root["filter"].is_array()) {
    for (auto& jf : root["filter"]) {
      auto type_str = jf.value("type", "");
      if (type_str != "raypath")
        continue;
      FilterConfig f;
      f.id = jf.value("id", 0);
      auto action_str = jf.value("action", "filter_in");
      f.action = (action_str == "filter_out") ? 1 : 0;
      if (jf.contains("raypath") && jf["raypath"].is_array()) {
        std::string text;
        for (size_t i = 0; i < jf["raypath"].size(); i++) {
          if (i > 0)
            text += ",";
          text += std::to_string(jf["raypath"][i].get<int>());
        }
        f.raypath_text = text;
      }
      auto sym = jf.value("symmetry", "");
      f.sym_p = (sym.find('P') != std::string::npos);
      f.sym_b = (sym.find('B') != std::string::npos);
      f.sym_d = (sym.find('D') != std::string::npos);
      max_id = std::max(max_id, f.id);
      state.filters.push_back(f);
    }
  }
  state.next_filter_id = max_id + 1;
  if (!state.filters.empty()) {
    state.selected_filter = 0;
  }

  // Scene
  if (root.contains("scene")) {
    auto& js = root["scene"];

    if (js.contains("light_source")) {
      auto& jl = js["light_source"];
      state.sun.altitude = jl.value("altitude", 20.0f);
      state.sun.azimuth = jl.value("azimuth", 0.0f);
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

    if (js.contains("scattering") && js["scattering"].is_array()) {
      for (auto& jlayer : js["scattering"]) {
        ScatterLayer layer;
        layer.probability = jlayer.value("prob", 1.0f);
        if (jlayer.contains("entries") && jlayer["entries"].is_array()) {
          for (auto& je : jlayer["entries"]) {
            ScatterEntry entry;
            entry.crystal_id = je.value("crystal", -1);
            entry.proportion = je.value("proportion", 1.0f);
            entry.filter_id = je.value("filter", -1);
            layer.entries.push_back(entry);
          }
        }
        state.scattering.push_back(layer);
      }
    }
  }

  // Render
  max_id = 0;
  if (root.contains("render") && root["render"].is_array()) {
    for (auto& jr : root["render"]) {
      RenderConfig r;
      r.id = jr.value("id", 0);
      max_id = std::max(max_id, r.id);

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

      state.renderers.push_back(r);
    }
  }
  state.next_renderer_id = max_id + 1;
  if (!state.renderers.empty()) {
    state.selected_renderer = 0;
  }

  return true;
}


// ========== Full GuiState JSON Serialization (for .lmc file) ==========

std::string SerializeGuiStateJson(const GuiState& state) {
  json root;

  // Crystals
  root["crystals"] = json::array();
  for (auto& c : state.crystals) {
    root["crystals"].push_back(SerializeCrystal(c));
  }

  // Filters
  root["filters"] = json::array();
  for (auto& f : state.filters) {
    root["filters"].push_back(SerializeFilterForGui(f));
  }

  // Sun
  json sun;
  sun["altitude"] = state.sun.altitude;
  sun["azimuth"] = state.sun.azimuth;
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

  // Scattering
  root["scattering"] = json::array();
  for (auto& layer : state.scattering) {
    json jl;
    jl["prob"] = layer.probability;
    jl["entries"] = json::array();
    for (auto& entry : layer.entries) {
      json je;
      je["crystal_id"] = entry.crystal_id;
      je["proportion"] = entry.proportion;
      je["filter_id"] = entry.filter_id;
      jl["entries"].push_back(je);
    }
    root["scattering"].push_back(jl);
  }

  // Renderers
  root["renderers"] = json::array();
  for (auto& r : state.renderers) {
    json jr;
    jr["id"] = r.id;
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
    root["renderers"].push_back(jr);
  }

  // Selected items (by ID, not index)
  auto id_or_neg1 = [](const auto& vec, int idx) -> int {
    if (idx >= 0 && idx < static_cast<int>(vec.size()))
      return vec[idx].id;
    return -1;
  };
  root["selected_crystal_id"] = id_or_neg1(state.crystals, state.selected_crystal);
  root["selected_renderer_id"] = id_or_neg1(state.renderers, state.selected_renderer);
  root["selected_filter_id"] = id_or_neg1(state.filters, state.selected_filter);

  // ID counters
  root["next_crystal_id"] = state.next_crystal_id;
  root["next_renderer_id"] = state.next_renderer_id;
  root["next_filter_id"] = state.next_filter_id;

  // Aspect ratio (view preference)
  auto preset_idx = static_cast<int>(state.aspect_preset);
  root["aspect_ratio"] = kAspectPresetJsonNames[preset_idx];
  root["aspect_portrait"] = state.aspect_portrait;

  // Background image overlay
  root["bg_path"] = state.bg_path;
  root["bg_show"] = state.bg_show;
  root["bg_alpha"] = state.bg_alpha;

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

  // Crystals
  if (root.contains("crystals") && root["crystals"].is_array()) {
    for (auto& jc : root["crystals"]) {
      state.crystals.push_back(ParseCrystal(jc));
    }
  }

  // Filters
  if (root.contains("filters") && root["filters"].is_array()) {
    for (auto& jf : root["filters"]) {
      FilterConfig f;
      f.id = jf.value("id", 0);
      auto action_str = jf.value("action", "filter_in");
      f.action = (action_str == "filter_out") ? 1 : 0;
      f.raypath_text = jf.value("raypath_text", std::string{});
      f.sym_p = jf.value("sym_p", true);
      f.sym_b = jf.value("sym_b", true);
      f.sym_d = jf.value("sym_d", true);
      state.filters.push_back(f);
    }
  }

  // Sun
  if (root.contains("sun")) {
    auto& js = root["sun"];
    state.sun.altitude = js.value("altitude", 20.0f);
    state.sun.azimuth = js.value("azimuth", 0.0f);
    state.sun.diameter = js.value("diameter", 0.5f);
    state.sun.spectrum_index = SpectrumFromString(js.value("spectrum", "D65"));
  }

  // Sim
  if (root.contains("sim")) {
    auto& js = root["sim"];
    state.sim.ray_num_millions = js.value("ray_num_millions", 1.0f);
    state.sim.max_hits = js.value("max_hits", 8);
    state.sim.infinite = js.value("infinite", false);
  }

  // Scattering
  if (root.contains("scattering") && root["scattering"].is_array()) {
    for (auto& jl : root["scattering"]) {
      ScatterLayer layer;
      layer.probability = jl.value("prob", 0.0f);
      if (jl.contains("entries") && jl["entries"].is_array()) {
        for (auto& je : jl["entries"]) {
          ScatterEntry entry;
          entry.crystal_id = je.value("crystal_id", -1);
          entry.proportion = je.value("proportion", 100.0f);
          entry.filter_id = je.value("filter_id", -1);
          layer.entries.push_back(entry);
        }
      }
      state.scattering.push_back(layer);
    }
  }

  // Renderers
  if (root.contains("renderers") && root["renderers"].is_array()) {
    for (auto& jr : root["renderers"]) {
      RenderConfig r;
      r.id = jr.value("id", 0);
      r.lens_type = LensTypeFromString(jr.value("lens_type", "linear"));
      r.fov = jr.value("fov", 90.0f);
      r.elevation = jr.value("elevation", 0.0f);
      r.azimuth = jr.value("azimuth", 0.0f);
      r.roll = jr.value("roll", 0.0f);
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
      r.opacity = jr.value("opacity", 1.0f);
      r.exposure_offset = jr.value("exposure_offset", 0.0f);
      state.renderers.push_back(r);
    }
  }

  // ID counters
  state.next_crystal_id = root.value("next_crystal_id", 1);
  state.next_renderer_id = root.value("next_renderer_id", 1);
  state.next_filter_id = root.value("next_filter_id", 1);

  // Selected items (by ID → find index)
  int sel_crystal_id = root.value("selected_crystal_id", -1);
  int sel_renderer_id = root.value("selected_renderer_id", -1);
  int sel_filter_id = root.value("selected_filter_id", -1);

  state.selected_crystal = FindIndexById(state.crystals, sel_crystal_id, state.crystals.empty() ? -1 : 0);
  state.selected_renderer = FindIndexById(state.renderers, sel_renderer_id, state.renderers.empty() ? -1 : 0);
  state.selected_filter = FindIndexById(state.filters, sel_filter_id, state.filters.empty() ? -1 : 0);

  // Aspect ratio (view preference, defaults to Free for old files)
  state.aspect_preset = AspectPresetFromString(root.value("aspect_ratio", "free"));
  state.aspect_portrait = root.value("aspect_portrait", false);

  // Background image overlay (backward compatible: missing fields use defaults)
  state.bg_path = root.value("bg_path", std::string{});
  state.bg_show = root.value("bg_show", false);
  state.bg_alpha = root.value("bg_alpha", 1.0f);

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

bool SaveLmcFile(const std::string& path, const GuiState& state, const PreviewRenderer& preview, bool save_texture) {
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

bool LoadLmcFile(const std::string& path, GuiState& state, std::vector<unsigned char>& tex_data, int& tex_w,
                 int& tex_h) {
  tex_data.clear();
  tex_w = 0;
  tex_h = 0;

  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    fprintf(stderr, "[LMC] Cannot open file: %s\n", path.c_str());
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
    fprintf(stderr, "[LMC] Failed to read header\n");
    return false;
  }

  if (magic != kLmcMagic) {
    fprintf(stderr, "[LMC] Invalid magic: 0x%08x\n", magic);
    return false;
  }

  if (version != kLmcVersion) {
    fprintf(stderr, "[LMC] Unsupported version: %u\n", version);
    return false;
  }

  // Read JSON
  std::string json_payload(json_size, '\0');
  in.seekg(static_cast<std::streamoff>(json_offset));
  in.read(json_payload.data(), static_cast<std::streamsize>(json_size));
  if (!in) {
    fprintf(stderr, "[LMC] Failed to read JSON section\n");
    return false;
  }

  if (!DeserializeGuiStateJson(json_payload, state)) {
    fprintf(stderr, "[LMC] Failed to parse JSON\n");
    return false;
  }

  // Read texture if present
  bool flag_has_tex = (flags & kLmcFlagHasTexture) != 0;
  if (flag_has_tex) {
    if (tex_size == 0) {
      fprintf(stderr, "[LMC] Texture flag set but size is 0\n");
      return false;
    }
    std::vector<unsigned char> png_buf(tex_size);
    in.seekg(static_cast<std::streamoff>(tex_offset));
    in.read(reinterpret_cast<char*>(png_buf.data()), static_cast<std::streamsize>(tex_size));
    if (!in) {
      fprintf(stderr, "[LMC] Failed to read texture section\n");
      return false;
    }

    int channels = 0;
    unsigned char* decoded =
        stbi_load_from_memory(png_buf.data(), static_cast<int>(png_buf.size()), &tex_w, &tex_h, &channels, 3);
    if (!decoded) {
      fprintf(stderr, "[LMC] Failed to decode texture PNG\n");
      return false;
    }
    size_t byte_count = static_cast<size_t>(tex_w) * tex_h * 3;
    tex_data.assign(decoded, decoded + byte_count);
    stbi_image_free(decoded);
  }

  return true;
}


// ========== Export Preview ==========

bool ExportPreviewPng(const char* path, PreviewRenderer& renderer, const PreviewViewport& vp) {
  int w = vp.vp_w;
  int h = vp.vp_h;
  if (w <= 0 || h <= 0 || !renderer.HasTexture()) {
    return false;
  }

  // Save current FBO binding
  GLint prev_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);

  // Create temporary FBO + renderbuffer
  GLuint fbo = 0;
  GLuint rbo = 0;
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glGenRenderbuffers(1, &rbo);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, w, h);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbo);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    glBindFramebuffer(GL_FRAMEBUFFER, prev_fbo);
    glDeleteRenderbuffers(1, &rbo);
    glDeleteFramebuffers(1, &fbo);
    return false;
  }

  // Render preview into FBO
  renderer.Render(0, 0, w, h, vp.params);

  // Read pixels
  std::vector<unsigned char> pixels(static_cast<size_t>(w) * h * 4);
  glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

  // Cleanup GL resources
  glBindFramebuffer(GL_FRAMEBUFFER, prev_fbo);
  glDeleteRenderbuffers(1, &rbo);
  glDeleteFramebuffers(1, &fbo);

  // Flip Y axis (OpenGL bottom-up → image top-down)
  int row_bytes = w * 4;
  std::vector<unsigned char> row(row_bytes);
  for (int y = 0; y < h / 2; y++) {
    unsigned char* top = pixels.data() + y * row_bytes;
    unsigned char* bottom = pixels.data() + (h - 1 - y) * row_bytes;
    std::memcpy(row.data(), top, row_bytes);
    std::memcpy(top, bottom, row_bytes);
    std::memcpy(bottom, row.data(), row_bytes);
  }

  // Save as RGBA PNG
  int result = stbi_write_png(path, w, h, 4, pixels.data(), w * 4);
  return result != 0;
}


// ========== File Dialogs ==========

std::string ShowOpenDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "Lumice", "lmc" } };
  nfdresult_t result = NFD_OpenDialog(&out_path, filter_item, 1, nullptr);
  std::string path;
  if (result == NFD_OKAY && out_path) {
    path = out_path;
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::string ShowSaveDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "Lumice", "lmc" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "project.lmc");
  std::string path;
  if (result == NFD_OKAY && out_path) {
    path = out_path;
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}


std::string ShowExportPngDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "PNG Image", "png" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "preview.png");
  std::string path;
  if (result == NFD_OKAY && out_path) {
    path = out_path;
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}


std::string ShowOpenImageDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_items[1] = { { "Images", "png,jpg,jpeg,bmp" } };
  nfdresult_t result = NFD_OpenDialog(&out_path, filter_items, 1, nullptr);
  std::string path;
  if (result == NFD_OKAY && out_path) {
    path = out_path;
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

}  // namespace lumice::gui
