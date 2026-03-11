#include "gui/file_io.hpp"

#include <nfd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "gui/gui_state.hpp"

namespace lumice::gui {

using json = nlohmann::json;

// ========== Serialization (GuiState → JSON) ==========

static json SerializeAxisDist(const AxisDist& a) {
  json j;
  if (a.type == AxisDistType::kGauss) {
    j["type"] = "gauss";
  } else {
    j["type"] = "uniform";
  }
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

static json SerializeFilter(const FilterConfig& f) {
  json j;
  j["id"] = f.id;
  j["type"] = "raypath";
  j["action"] = f.action == 0 ? "filter_in" : "filter_out";

  // Parse raypath text to int array
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

std::string SerializeToJson(const GuiState& state) {
  json root;

  // Crystals
  root["crystal"] = json::array();
  for (auto& c : state.crystals) {
    root["crystal"].push_back(SerializeCrystal(c));
  }

  // Filters
  root["filter"] = json::array();
  for (auto& f : state.filters) {
    root["filter"].push_back(SerializeFilter(f));
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
  // Lens type, FOV, view rotation, and visible are frontend-only (shader) parameters.
  root["render"] = json::array();
  for (auto& r : state.renderers) {
    json jr;
    jr["id"] = r.id;

    // Always rectangular (equirectangular) for Core
    jr["lens"]["type"] = "rectangular";
    jr["lens"]["fov"] = 180.0f;  // ignored by rectangular but required by schema

    int res = kSimResolutions[r.sim_resolution_index];
    jr["resolution"] = { res * 2, res };  // equirectangular 2:1

    // No view rotation — Core renders the full sky
    jr["view"]["elevation"] = 0.0f;
    jr["view"]["azimuth"] = 0.0f;
    jr["view"]["roll"] = 0.0f;

    // Always full visible — shader handles hemisphere filtering
    jr["visible"] = "full";
    jr["background"] = { 0.0f, 0.0f, 0.0f };
    // Omit ray_color so Core uses true color (spectral rendering).
    // Core treats ray_color[0] < 0 as true color; default is [-1,-1,-1].
    jr["opacity"] = r.opacity;
    jr["intensity_factor"] = std::pow(2.0f, r.exposure_offset);

    root["render"].push_back(jr);
  }

  return root.dump(2);
}


// ========== Deserialization (JSON → GuiState) ==========

static AxisDist ParseAxisDist(const json& j) {
  AxisDist a;
  if (j.is_number()) {
    // Scalar means fixed value (Gauss with std=0)
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

static FilterConfig ParseFilter(const json& j) {
  FilterConfig f;
  f.id = j.value("id", 0);

  auto action_str = j.value("action", "filter_in");
  f.action = (action_str == "filter_out") ? 1 : 0;

  // Raypath
  if (j.contains("raypath") && j["raypath"].is_array()) {
    std::string text;
    for (size_t i = 0; i < j["raypath"].size(); i++) {
      if (i > 0)
        text += ",";
      text += std::to_string(j["raypath"][i].get<int>());
    }
    f.raypath_text = text;
  }

  // Symmetry
  auto sym = j.value("symmetry", "");
  f.sym_p = (sym.find('P') != std::string::npos);
  f.sym_b = (sym.find('B') != std::string::npos);
  f.sym_d = (sym.find('D') != std::string::npos);

  return f;
}

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
        continue;  // MVP: only raypath
      auto f = ParseFilter(jf);
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

    // Light source
    if (js.contains("light_source")) {
      auto& jl = js["light_source"];
      state.sun.altitude = jl.value("altitude", 20.0f);
      state.sun.azimuth = jl.value("azimuth", 0.0f);
      state.sun.diameter = jl.value("diameter", 0.5f);

      if (jl.contains("spectrum") && jl["spectrum"].is_string()) {
        auto sp = jl["spectrum"].get<std::string>();
        for (int i = 0; i < kSpectrumCount; i++) {
          if (sp == kSpectrumNames[i]) {
            state.sun.spectrum_index = i;
            break;
          }
        }
      }
    }

    // Simulation
    if (js.contains("ray_num")) {
      if (js["ray_num"].is_string() && js["ray_num"].get<std::string>() == "infinite") {
        state.sim.infinite = true;
      } else if (js["ray_num"].is_number()) {
        state.sim.ray_num_millions = static_cast<float>(js["ray_num"].get<size_t>()) / 1e6f;
      }
    }
    state.sim.max_hits = js.value("max_hits", 8);

    // Scattering
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
  static const char* kLensTypeJsonNames[] = { "linear",
                                              "fisheye_equal_area",
                                              "fisheye_equidistant",
                                              "fisheye_stereographic",
                                              "dual_fisheye_equal_area",
                                              "dual_fisheye_equidistant",
                                              "dual_fisheye_stereographic",
                                              "rectangular" };
  if (root.contains("render") && root["render"].is_array()) {
    for (auto& jr : root["render"]) {
      RenderConfig r;
      r.id = jr.value("id", 0);
      max_id = std::max(max_id, r.id);

      if (jr.contains("lens")) {
        auto lens_str = jr["lens"].value("type", "linear");
        for (int i = 0; i < kLensTypeCount; i++) {
          if (lens_str == kLensTypeJsonNames[i]) {
            r.lens_type = i;
            break;
          }
        }
        r.fov = jr["lens"].value("fov", 90.0f);
      }

      if (jr.contains("resolution") && jr["resolution"].is_array() && jr["resolution"].size() == 2) {
        int h = jr["resolution"][1].get<int>();
        // Find closest sim_resolution_index
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

      auto vis_str = jr.value("visible", "upper");
      static const char* kVisibleJsonNames[] = { "upper", "lower", "full" };
      for (int i = 0; i < kVisibleCount; i++) {
        if (vis_str == kVisibleJsonNames[i]) {
          r.visible = i;
          break;
        }
      }

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


// ========== File I/O ==========

std::string ShowOpenDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "JSON", "json" } };
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
  nfdfilteritem_t filter_item[1] = { { "JSON", "json" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "config.json");
  std::string path;
  if (result == NFD_OKAY && out_path) {
    path = out_path;
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

bool ReadFileToString(const std::string& path, std::string& out) {
  std::ifstream ifs(path);
  if (!ifs.is_open())
    return false;
  std::ostringstream oss;
  oss << ifs.rdbuf();
  out = oss.str();
  return true;
}

bool WriteStringToFile(const std::string& path, const std::string& content) {
  std::ofstream ofs(path);
  if (!ofs.is_open())
    return false;
  ofs << content;
  return ofs.good();
}

}  // namespace lumice::gui
