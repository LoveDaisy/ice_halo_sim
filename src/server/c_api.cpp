#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <utility>
#include <vector>

#include "core/geo3d.hpp"
#include "include/lumice.h"
#include "server/server.hpp"
#include "util/callback_sink.hpp"
#include "util/logger.hpp"
#include "util/path_utils.hpp"

namespace ns = lumice;

// Convert Miller index (i1, i4) to wedge angle in degrees. Returns 28.0 (default) if i1 == 0.
static float MillerToAlpha(int i1, int i4) {
  constexpr float kSqrt3_2 = 0.866025403784f;
  constexpr float kIceCrystalC = 1.629f;
  constexpr float kRadToDeg = 57.2957795131f;
  if (i1 == 0)
    return 28.0f;
  return std::atan(kSqrt3_2 * i4 / i1 / kIceCrystalC) * kRadToDeg;
}

// =============== Internal Helpers ===============
struct LUMICE_Server_ {
  std::unique_ptr<ns::Server> server_;
};


static LUMICE_ErrorCode MapErrorCode(ns::ErrorCode code) {
  switch (code) {
    case ns::ErrorCode::kSuccess:
      return LUMICE_OK;
    case ns::ErrorCode::kInvalidJson:
      return LUMICE_ERR_INVALID_JSON;
    case ns::ErrorCode::kInvalidConfig:
      return LUMICE_ERR_INVALID_CONFIG;
    case ns::ErrorCode::kMissingField:
      return LUMICE_ERR_MISSING_FIELD;
    case ns::ErrorCode::kInvalidValue:
      return LUMICE_ERR_INVALID_VALUE;
    case ns::ErrorCode::kServerNotReady:
    case ns::ErrorCode::kServerError:
    default:
      return LUMICE_ERR_SERVER;
  }
}


// =============== Server Lifecycle ===============
LUMICE_Server* LUMICE_CreateServer() {
  auto* s = new LUMICE_Server;
  s->server_ = std::make_unique<ns::Server>();
  return s;
}


LUMICE_Server* LUMICE_CreateServerEx(const LUMICE_ServerConfig* config) {
  auto* s = new LUMICE_Server;
  int num_workers = (config != nullptr) ? config->num_workers : 0;
  s->server_ = std::make_unique<ns::Server>(num_workers);
  return s;
}


void LUMICE_DestroyServer(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  server->server_->Terminate();
  delete server;
}


// =============== Logging ===============
void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level) {
  if (!server) {
    return;
  }
  static constexpr ns::LogLevel kLevelMap[] = {
    ns::LogLevel::kTrace,    // LUMICE_LOG_TRACE
    ns::LogLevel::kDebug,    // LUMICE_LOG_DEBUG
    ns::LogLevel::kVerbose,  // LUMICE_LOG_VERBOSE
    ns::LogLevel::kInfo,     // LUMICE_LOG_INFO
    ns::LogLevel::kWarning,  // LUMICE_LOG_WARNING
    ns::LogLevel::kError,    // LUMICE_LOG_ERROR
    ns::LogLevel::kOff,      // LUMICE_LOG_OFF
  };
  if (level >= LUMICE_LOG_TRACE && level <= LUMICE_LOG_OFF) {
    auto mapped = kLevelMap[level];
    server->server_->SetLogLevel(mapped);
    ns::GetGlobalLogger().SetLevel(mapped);
  }
}


void LUMICE_SetLogCallback(LUMICE_LogCallback callback) {
  auto& sink = ns::GetCallbackSink();
  sink->SetCallback(callback);

  // Add callback sink to shared dist_sink on first call (idempotent check via static flag).
  // Set our custom formatter since sinks added after Logger::set_formatter don't inherit it.
  static bool registered = false;
  if (!registered) {
    sink->set_formatter(ns::CreateLumiceFormatter(ns::kLogPattern));
    ns::GetSharedSink()->add_sink(sink);
    registered = true;
  }
}


// =============== Configuration ===============
LUMICE_ErrorCode LUMICE_CommitConfig(LUMICE_Server* server, const char* config_str) {
  if (!server || !config_str) {
    return LUMICE_ERR_NULL_ARG;
  }

  auto err = server->server_->CommitConfig(std::string(config_str));
  if (err) {
    LOG_ERROR("Failed to commit configuration: {}", err.message);
    if (!err.field.empty()) {
      LOG_ERROR("Error field: {}", err.field);
    }
    return MapErrorCode(err.code);
  }
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename) {
  if (!server || !filename) {
    return LUMICE_ERR_NULL_ARG;
  }

  auto err = server->server_->CommitConfigFromFile(lumice::PathFromU8(filename));
  if (err) {
    LOG_ERROR("Failed to load configuration from file '{}': {}", filename, err.message);
    if (!err.field.empty()) {
      LOG_ERROR("Error field: {}", err.field);
    }
    return MapErrorCode(err.code);
  }
  return LUMICE_OK;
}


// =============== Configuration (C struct) ===============
static nlohmann::json AxisDistToJson(const LUMICE_AxisDist& d) {
  nlohmann::json j;
  switch (d.type) {
    case LUMICE_AXIS_DIST_GAUSS:
      j["type"] = "gauss";
      break;
    case LUMICE_AXIS_DIST_UNIFORM:
      j["type"] = "uniform";
      break;
    case LUMICE_AXIS_DIST_ZIGZAG:
      j["type"] = "zigzag";
      break;
    case LUMICE_AXIS_DIST_LAPLACIAN:
      j["type"] = "laplacian";
      break;
    default:
      LOG_ERROR("Unknown LUMICE_AxisDist.type: {}", d.type);
      j["type"] = "gauss";
      break;
  }
  j["mean"] = d.mean;
  j["std"] = d.std;
  return j;
}

static nlohmann::json ConfigToJson(const LUMICE_Config& c) {
  using json = nlohmann::json;
  json root;

  // Crystals
  root["crystal"] = json::array();
  for (int i = 0; i < c.crystal_count; i++) {
    const auto& cr = c.crystals[i];
    json j;
    j["id"] = cr.id;
    if (cr.type == 0) {
      j["type"] = "prism";
      j["shape"]["height"] = cr.height;
    } else {
      j["type"] = "pyramid";
      j["shape"]["prism_h"] = cr.prism_h;
      j["shape"]["upper_h"] = cr.upper_h;
      j["shape"]["lower_h"] = cr.lower_h;
      j["shape"]["upper_wedge_angle"] = cr.upper_wedge_angle;
      j["shape"]["lower_wedge_angle"] = cr.lower_wedge_angle;
    }
    // face_distance: only write when non-default
    bool is_default_fd = true;
    for (int fi = 0; fi < 6; fi++) {
      if (std::abs(cr.face_distance[fi] - 1.0f) > 1e-6f) {
        is_default_fd = false;
        break;
      }
    }
    if (!is_default_fd) {
      j["shape"]["face_distance"] = { cr.face_distance[0], cr.face_distance[1], cr.face_distance[2],
                                      cr.face_distance[3], cr.face_distance[4], cr.face_distance[5] };
    }
    j["axis"]["zenith"] = AxisDistToJson(cr.zenith);
    j["axis"]["azimuth"] = AxisDistToJson(cr.azimuth);
    j["axis"]["roll"] = AxisDistToJson(cr.roll);
    root["crystal"].push_back(j);
  }

  // Filters
  root["filter"] = json::array();
  for (int i = 0; i < c.filter_count; i++) {
    const auto& f = c.filters[i];
    json j;
    j["id"] = f.id;
    j["type"] = "raypath";
    j["action"] = f.action == 0 ? "filter_in" : "filter_out";
    json rp = json::array();
    for (int k = 0; k < f.raypath_count; k++) {
      rp.push_back(f.raypath[k]);
    }
    j["raypath"] = rp;
    std::string sym;
    if (f.symmetry & 1)
      sym += "P";
    if (f.symmetry & 2)
      sym += "B";
    if (f.symmetry & 4)
      sym += "D";
    if (!sym.empty()) {
      j["symmetry"] = sym;
    }
    root["filter"].push_back(j);
  }

  // Scene
  json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = c.sun_altitude;
  scene["light_source"]["azimuth"] = c.sun_azimuth;
  scene["light_source"]["diameter"] = c.sun_diameter;
  scene["light_source"]["spectrum"] = c.spectrum ? c.spectrum : "D65";

  if (c.infinite) {
    scene["ray_num"] = "infinite";
  } else {
    scene["ray_num"] = c.ray_num;
  }
  scene["max_hits"] = c.max_hits;

  scene["scattering"] = json::array();
  for (int i = 0; i < c.scatter_count; i++) {
    const auto& layer = c.scattering[i];
    json jl;
    jl["prob"] = layer.probability;
    jl["entries"] = json::array();
    for (int k = 0; k < layer.entry_count; k++) {
      const auto& e = layer.entries[k];
      json je;
      je["crystal"] = e.crystal_id >= 0 ? e.crystal_id : 1;
      je["proportion"] = e.proportion;
      if (e.filter_id >= 0) {
        je["filter"] = e.filter_id;
      }
      jl["entries"].push_back(je);
    }
    scene["scattering"].push_back(jl);
  }
  root["scene"] = scene;

  // Renderers — Core always produces dual equal-area fisheye texture (full-globe, equal-area).
  // GUI shader reprojects from this format to the user's display projection.
  root["render"] = json::array();
  for (int i = 0; i < c.renderer_count; i++) {
    const auto& r = c.renderers[i];
    json jr;
    jr["id"] = r.id;
    jr["lens"]["type"] = "dual_fisheye_equal_area";
    jr["lens"]["fov"] = 180.0f;
    jr["resolution"] = { r.resolution_w, r.resolution_h };
    jr["view"]["elevation"] = 0.0f;
    jr["view"]["azimuth"] = 0.0f;
    jr["view"]["roll"] = 0.0f;
    jr["visible"] = "full";
    jr["background"] = { 0.0f, 0.0f, 0.0f };
    jr["opacity"] = r.opacity;
    jr["intensity_factor"] = r.intensity_factor;
    jr["norm_mode"] = r.norm_mode;
    root["render"].push_back(jr);
  }

  return root;
}

LUMICE_ErrorCode LUMICE_CommitConfigStruct(LUMICE_Server* server, const LUMICE_Config* config, int* out_reused) {
  if (!server || !config) {
    return LUMICE_ERR_NULL_ARG;
  }

  // Bounds check
  if (config->crystal_count > LUMICE_MAX_CONFIG_CRYSTALS || config->filter_count > LUMICE_MAX_CONFIG_FILTERS ||
      config->renderer_count > LUMICE_MAX_CONFIG_RENDERERS ||
      config->scatter_count > LUMICE_MAX_CONFIG_SCATTER_LAYERS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }

  auto config_json = ConfigToJson(*config);
  bool reused = false;
  auto err = server->server_->CommitConfig(config_json, &reused);
  if (err) {
    LOG_ERROR("Failed to commit configuration (struct): {}", err.message);
    return MapErrorCode(err.code);
  }
  if (out_reused) {
    *out_reused = reused ? 1 : 0;
  }
  return LUMICE_OK;
}


// =============== Configuration Parsing (JSON -> LUMICE_Config) ===============
// Symmetric inverse of ConfigToJson. Decomposed into per-section helpers.

static LUMICE_ErrorCode JsonToAxisDist(const nlohmann::json& j, LUMICE_AxisDist* out) {
  if (!j.is_object() || !j.contains("type") || !j.contains("mean") || !j.contains("std")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  auto type_str = j.at("type").get<std::string>();
  if (type_str == "gauss") {
    out->type = LUMICE_AXIS_DIST_GAUSS;
  } else if (type_str == "uniform") {
    out->type = LUMICE_AXIS_DIST_UNIFORM;
  } else if (type_str == "zigzag") {
    out->type = LUMICE_AXIS_DIST_ZIGZAG;
  } else if (type_str == "laplacian") {
    out->type = LUMICE_AXIS_DIST_LAPLACIAN;
  } else {
    return LUMICE_ERR_INVALID_VALUE;
  }
  out->mean = j.at("mean").get<float>();
  out->std = j.at("std").get<float>();
  return LUMICE_OK;
}

static const char* MapSpectrumString(const std::string& s) {
  static const std::map<std::string, const char*> kSpectrumMap = {
    { "D65", "D65" },
    { "D50", "D50" },
    { "A", "A" },
    { "E", "E" },
  };
  auto it = kSpectrumMap.find(s);
  return it != kSpectrumMap.end() ? it->second : nullptr;
}

static LUMICE_ErrorCode JsonToCrystal(const nlohmann::json& cj, LUMICE_CrystalParam* cr) {
  if (!cj.contains("id") || !cj.contains("type")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  cr->id = cj.at("id").get<int>();

  auto type_str = cj.at("type").get<std::string>();
  if (type_str == "prism") {
    cr->type = 0;
    if (!cj.contains("shape") || !cj.at("shape").contains("height")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    cr->height = cj.at("shape").at("height").get<float>();
  } else if (type_str == "pyramid") {
    cr->type = 1;
    if (!cj.contains("shape")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    const auto& shape = cj.at("shape");
    if (!shape.contains("prism_h") || !shape.contains("upper_h") || !shape.contains("lower_h")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    cr->prism_h = shape.at("prism_h").get<float>();
    cr->upper_h = shape.at("upper_h").get<float>();
    cr->lower_h = shape.at("lower_h").get<float>();
    // Wedge angle: prefer "upper_wedge_angle", fallback to "upper_indices" conversion
    if (shape.contains("upper_wedge_angle") && shape.at("upper_wedge_angle").is_number()) {
      cr->upper_wedge_angle = shape.at("upper_wedge_angle").get<float>();
    } else if (shape.contains("upper_indices") && shape.at("upper_indices").is_array() &&
               shape.at("upper_indices").size() == 3) {
      cr->upper_wedge_angle =
          MillerToAlpha(shape.at("upper_indices")[0].get<int>(), shape.at("upper_indices")[2].get<int>());
    }
    if (shape.contains("lower_wedge_angle") && shape.at("lower_wedge_angle").is_number()) {
      cr->lower_wedge_angle = shape.at("lower_wedge_angle").get<float>();
    } else if (shape.contains("lower_indices") && shape.at("lower_indices").is_array() &&
               shape.at("lower_indices").size() == 3) {
      cr->lower_wedge_angle =
          MillerToAlpha(shape.at("lower_indices")[0].get<int>(), shape.at("lower_indices")[2].get<int>());
    }
  } else {
    return LUMICE_ERR_INVALID_VALUE;
  }

  // face_distance: array of 6 or absent (default all 1.0f)
  if (cj.contains("shape") && cj.at("shape").contains("face_distance")) {
    const auto& fd = cj.at("shape").at("face_distance");
    if (!fd.is_array() || fd.size() != 6) {
      return LUMICE_ERR_INVALID_VALUE;
    }
    for (int k = 0; k < 6; k++) {
      cr->face_distance[k] = fd[k].get<float>();
    }
  } else {
    for (int k = 0; k < 6; k++) {
      cr->face_distance[k] = 1.0f;
    }
  }

  // Axis distributions
  if (!cj.contains("axis")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  const auto& axis = cj.at("axis");
  for (const auto& [name, dest] : std::vector<std::pair<const char*, LUMICE_AxisDist*>>{
           { "zenith", &cr->zenith }, { "azimuth", &cr->azimuth }, { "roll", &cr->roll } }) {
    if (!axis.contains(name)) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    auto err = JsonToAxisDist(axis.at(name), dest);
    if (err != LUMICE_OK) {
      return err;
    }
  }
  return LUMICE_OK;
}

static LUMICE_ErrorCode JsonToFilter(const nlohmann::json& fj, LUMICE_FilterParam* f) {
  if (!fj.contains("id") || !fj.contains("type") || !fj.contains("action")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  f->id = fj.at("id").get<int>();

  if (fj.at("type").get<std::string>() != "raypath") {
    return LUMICE_ERR_INVALID_VALUE;
  }

  auto action_str = fj.at("action").get<std::string>();
  if (action_str == "filter_in") {
    f->action = 0;
  } else if (action_str == "filter_out") {
    f->action = 1;
  } else {
    return LUMICE_ERR_INVALID_VALUE;
  }

  if (fj.contains("raypath") && fj.at("raypath").is_array()) {
    const auto& rp = fj.at("raypath");
    f->raypath_count = static_cast<int>(rp.size());
    if (f->raypath_count > LUMICE_MAX_CONFIG_RAYPATH_LEN) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    for (int k = 0; k < f->raypath_count; k++) {
      f->raypath[k] = rp[k].get<int>();
    }
  }

  // Symmetry: string "PBD" -> bitmask
  f->symmetry = 0;
  if (fj.contains("symmetry")) {
    for (char ch : fj.at("symmetry").get<std::string>()) {
      if (ch == 'P') {
        f->symmetry |= 1;
      } else if (ch == 'B') {
        f->symmetry |= 2;
      } else if (ch == 'D') {
        f->symmetry |= 4;
      }
    }
  }
  return LUMICE_OK;
}

static LUMICE_ErrorCode JsonToScene(const nlohmann::json& scene, LUMICE_Config* out) {
  // Light source
  if (scene.contains("light_source")) {
    const auto& ls = scene.at("light_source");
    if (ls.contains("altitude")) {
      out->sun_altitude = ls.at("altitude").get<float>();
    }
    if (ls.contains("azimuth")) {
      out->sun_azimuth = ls.at("azimuth").get<float>();
    }
    if (ls.contains("diameter")) {
      out->sun_diameter = ls.at("diameter").get<float>();
    }
    if (ls.contains("spectrum")) {
      const auto& sp = ls.at("spectrum");
      if (!sp.is_string()) {
        return LUMICE_ERR_INVALID_VALUE;  // Array-form spectrum not supported by LUMICE_Config
      }
      out->spectrum = MapSpectrumString(sp.get<std::string>());
      if (!out->spectrum) {
        return LUMICE_ERR_INVALID_VALUE;
      }
    } else {
      out->spectrum = "D65";
    }
  } else {
    out->spectrum = "D65";
  }

  // Ray num
  if (scene.contains("ray_num")) {
    const auto& rn = scene.at("ray_num");
    if (rn.is_string() && rn.get<std::string>() == "infinite") {
      out->infinite = 1;
      out->ray_num = 0;
    } else if (rn.is_number()) {
      out->infinite = 0;
      out->ray_num = rn.get<unsigned long>();
    } else {
      return LUMICE_ERR_INVALID_VALUE;
    }
  }

  if (scene.contains("max_hits")) {
    out->max_hits = scene.at("max_hits").get<int>();
  }

  // Scattering
  if (scene.contains("scattering") && scene.at("scattering").is_array()) {
    const auto& scat = scene.at("scattering");
    if (static_cast<int>(scat.size()) > LUMICE_MAX_CONFIG_SCATTER_LAYERS) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    out->scatter_count = static_cast<int>(scat.size());
    for (int i = 0; i < out->scatter_count; i++) {
      const auto& lj = scat[i];
      auto& layer = out->scattering[i];
      if (lj.contains("prob")) {
        layer.probability = lj.at("prob").get<float>();
      }
      if (lj.contains("entries") && lj.at("entries").is_array()) {
        const auto& entries = lj.at("entries");
        if (static_cast<int>(entries.size()) > LUMICE_MAX_CONFIG_SCATTER_ENTRIES) {
          return LUMICE_ERR_INVALID_CONFIG;
        }
        layer.entry_count = static_cast<int>(entries.size());
        for (int k = 0; k < layer.entry_count; k++) {
          const auto& ej = entries[k];
          auto& e = layer.entries[k];
          if (ej.contains("crystal")) {
            e.crystal_id = ej.at("crystal").get<int>();
          }
          if (ej.contains("proportion")) {
            e.proportion = ej.at("proportion").get<float>();
          }
          e.filter_id = ej.contains("filter") ? ej.at("filter").get<int>() : -1;
        }
      }
    }
  }
  return LUMICE_OK;
}

static LUMICE_ErrorCode JsonToRenderers(const nlohmann::json& render_arr, LUMICE_Config* out) {
  if (static_cast<int>(render_arr.size()) > LUMICE_MAX_CONFIG_RENDERERS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  out->renderer_count = static_cast<int>(render_arr.size());
  for (int i = 0; i < out->renderer_count; i++) {
    const auto& rj = render_arr[i];
    auto& r = out->renderers[i];
    if (rj.contains("id")) {
      r.id = rj.at("id").get<int>();
    }
    if (rj.contains("resolution") && rj.at("resolution").is_array() && rj.at("resolution").size() == 2) {
      r.resolution_w = rj.at("resolution")[0].get<int>();
      r.resolution_h = rj.at("resolution")[1].get<int>();
    }
    if (rj.contains("opacity")) {
      r.opacity = rj.at("opacity").get<float>();
    }
    if (rj.contains("intensity_factor")) {
      r.intensity_factor = rj.at("intensity_factor").get<float>();
    }
    if (rj.contains("norm_mode")) {
      r.norm_mode = rj.at("norm_mode").get<int>();
    }
    // lens, view, visible, background fields are ignored (not representable in LUMICE_Config)
  }
  return LUMICE_OK;
}

static LUMICE_ErrorCode JsonToConfig(const nlohmann::json& root, LUMICE_Config* out) {
  std::memset(out, 0, sizeof(LUMICE_Config));
  out->spectrum = "D65";  // Safe default (memset leaves nullptr)

  // Crystals (required)
  if (!root.contains("crystal") || !root.at("crystal").is_array()) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  const auto& crystals = root.at("crystal");
  if (static_cast<int>(crystals.size()) > LUMICE_MAX_CONFIG_CRYSTALS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  out->crystal_count = static_cast<int>(crystals.size());
  for (int i = 0; i < out->crystal_count; i++) {
    auto err = JsonToCrystal(crystals[i], &out->crystals[i]);
    if (err != LUMICE_OK) {
      return err;
    }
  }

  // Filters (optional)
  if (root.contains("filter") && root.at("filter").is_array()) {
    const auto& filters = root.at("filter");
    if (static_cast<int>(filters.size()) > LUMICE_MAX_CONFIG_FILTERS) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    out->filter_count = static_cast<int>(filters.size());
    for (int i = 0; i < out->filter_count; i++) {
      auto err = JsonToFilter(filters[i], &out->filters[i]);
      if (err != LUMICE_OK) {
        return err;
      }
    }
  }

  // Scene (required)
  if (!root.contains("scene")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  auto err = JsonToScene(root.at("scene"), out);
  if (err != LUMICE_OK) {
    return err;
  }

  // Renderers (optional)
  if (root.contains("render") && root.at("render").is_array()) {
    err = JsonToRenderers(root.at("render"), out);
    if (err != LUMICE_OK) {
      return err;
    }
  }

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_ParseConfigString(const char* json_str, LUMICE_Config* out) {
  if (!json_str || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  try {
    auto root = nlohmann::json::parse(json_str);
    return JsonToConfig(root, out);
  } catch (const nlohmann::json::parse_error&) {
    return LUMICE_ERR_INVALID_JSON;
  } catch (const nlohmann::json::exception&) {
    return LUMICE_ERR_INVALID_VALUE;
  }
}


LUMICE_ErrorCode LUMICE_ParseConfigFile(const char* filename, LUMICE_Config* out) {
  if (!filename || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  std::ifstream file(lumice::PathFromU8(filename));
  if (!file.is_open()) {
    return LUMICE_ERR_FILE_NOT_FOUND;
  }

  try {
    auto root = nlohmann::json::parse(file);
    return JsonToConfig(root, out);
  } catch (const nlohmann::json::parse_error&) {
    return LUMICE_ERR_INVALID_JSON;
  } catch (const nlohmann::json::exception&) {
    return LUMICE_ERR_INVALID_VALUE;
  }
}


// =============== Results ===============
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  auto render_results = server->server_->GetRenderResults();
  int count = static_cast<int>(render_results.size());
  if (count > max_count) {
    count = max_count;
  }

  for (int i = 0; i < count; i++) {
    out[i].renderer_id = render_results[i].renderer_id_;
    out[i].img_width = render_results[i].img_width_;
    out[i].img_height = render_results[i].img_height_;
    out[i].img_buffer = render_results[i].img_buffer_;
  }

  // Sentinel
  std::memset(&out[count], 0, sizeof(LUMICE_RenderResult));

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_GetRawXyzResults(LUMICE_Server* server, LUMICE_RawXyzResult* out, int max_count) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  auto results = server->server_->GetRawXyzResults();
  int count = static_cast<int>(results.size());
  if (count > max_count) {
    count = max_count;
  }

  for (int i = 0; i < count; i++) {
    out[i].renderer_id = results[i].renderer_id_;
    out[i].img_width = results[i].img_width_;
    out[i].img_height = results[i].img_height_;
    out[i].xyz_buffer = results[i].xyz_buffer_;
    out[i].snapshot_intensity = results[i].snapshot_intensity_;
    out[i].intensity_factor = results[i].intensity_factor_;
    out[i].has_valid_data = results[i].has_valid_data_ ? 1 : 0;
    out[i].snapshot_generation = results[i].snapshot_generation_;
    out[i].effective_pixels = results[i].effective_pixels_;
  }

  // Sentinel
  std::memset(&out[count], 0, sizeof(LUMICE_RawXyzResult));

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  int count = 0;
  if (max_count > 0) {
    auto stats_result = server->server_->GetStatsResult();
    if (stats_result.has_value()) {
      out[0].ray_seg_num = stats_result->ray_seg_num_;
      out[0].sim_ray_num = stats_result->sim_ray_num_;
      out[0].crystal_num = stats_result->crystal_num_;
      count = 1;
    }
  }

  // Sentinel
  std::memset(&out[count], 0, sizeof(LUMICE_StatsResult));

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_GetCachedStats(LUMICE_Server* server, LUMICE_StatsResult* out) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  auto result = server->server_->GetCachedStatsResult();
  if (result.has_value()) {
    out->ray_seg_num = result->ray_seg_num_;
    out->sim_ray_num = result->sim_ray_num_;
    out->crystal_num = result->crystal_num_;
  } else {
    std::memset(out, 0, sizeof(LUMICE_StatsResult));
  }

  return LUMICE_OK;
}


// =============== State & Control ===============
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  if (server->server_->IsIdle()) {
    *out = LUMICE_SERVER_IDLE;
  } else {
    *out = LUMICE_SERVER_RUNNING;
  }

  return LUMICE_OK;
}


void LUMICE_StopServer(LUMICE_Server* server) {
  if (!server) {
    return;
  }

  server->server_->Stop();
}


// =============== Crystal Mesh ===============
LUMICE_ErrorCode LUMICE_GetCrystalMesh(LUMICE_Server* /*server*/, const char* crystal_json, LUMICE_CrystalMesh* out) {
  if (!crystal_json || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  // Parse JSON
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(crystal_json);
  } catch (...) {
    return LUMICE_ERR_INVALID_JSON;
  }

  if (!j.contains("type") || !j.contains("shape")) {
    return LUMICE_ERR_MISSING_FIELD;
  }

  // Create mesh based on crystal type
  auto type_str = j.at("type").get<std::string>();
  const auto& shape = j.at("shape");
  ns::Mesh mesh;

  // Parse face_distance if present (common to both prism and pyramid)
  float dist[6]{ 1, 1, 1, 1, 1, 1 };
  if (shape.contains("face_distance") && shape["face_distance"].is_array()) {
    size_t n = std::min(shape["face_distance"].size(), static_cast<size_t>(6));
    for (size_t i = 0; i < n; i++) {
      if (shape["face_distance"][i].is_number()) {
        dist[i] = shape["face_distance"][i].get<float>();
      }
    }
  }

  try {
    if (type_str == "prism") {
      float h = shape.value("height", 1.0f);
      mesh = ns::CreatePrismMesh(h, dist);
    } else if (type_str == "pyramid") {
      float prism_h = shape.value("prism_h", 1.0f);
      float upper_h = shape.value("upper_h", 0.0f);
      float lower_h = shape.value("lower_h", 0.0f);
      // Prefer wedge_angle, fallback to upper_indices, then default
      if (shape.contains("upper_wedge_angle") && shape.contains("lower_wedge_angle")) {
        float ua = shape["upper_wedge_angle"].get<float>();
        float la = shape["lower_wedge_angle"].get<float>();
        mesh = ns::CreatePyramidMesh(ua, la, upper_h, prism_h, lower_h, dist);
      } else if (shape.contains("upper_indices") && shape.contains("lower_indices")) {
        float ua = MillerToAlpha(shape["upper_indices"][0].get<int>(), shape["upper_indices"][2].get<int>());
        float la = MillerToAlpha(shape["lower_indices"][0].get<int>(), shape["lower_indices"][2].get<int>());
        mesh = ns::CreatePyramidMesh(ua, la, upper_h, prism_h, lower_h, dist);
      } else {
        mesh = ns::CreatePyramidMesh(upper_h, prism_h, lower_h);
      }
    } else {
      return LUMICE_ERR_INVALID_VALUE;
    }
  } catch (...) {
    return LUMICE_ERR_INVALID_CONFIG;
  }

  // Fill vertices
  auto vtx_cnt = static_cast<int>(mesh.GetVtxCnt());
  if (vtx_cnt > LUMICE_MAX_CRYSTAL_VERTICES) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  out->vertex_count = vtx_cnt;
  std::memcpy(out->vertices, mesh.GetVtxPtr(0), vtx_cnt * 3 * sizeof(float));

  // Fill triangles for surface rendering
  auto tri_cnt = mesh.GetTriangleCnt();
  if (static_cast<int>(tri_cnt) > LUMICE_MAX_CRYSTAL_TRIANGLES) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  const int* tri = mesh.GetTrianglePtr(0);
  out->triangle_count = static_cast<int>(tri_cnt);
  std::memcpy(out->triangles, tri, tri_cnt * 3 * sizeof(int));

  const float* vtx = mesh.GetVtxPtr(0);

  // Build edge → triangle list
  using Edge = std::pair<int, int>;
  std::map<Edge, std::vector<size_t>> edge_tris;
  for (size_t i = 0; i < tri_cnt; i++) {
    int v0 = tri[i * 3 + 0];
    int v1 = tri[i * 3 + 1];
    int v2 = tri[i * 3 + 2];
    edge_tris[{ std::min(v0, v1), std::max(v0, v1) }].push_back(i);
    edge_tris[{ std::min(v1, v2), std::max(v1, v2) }].push_back(i);
    edge_tris[{ std::min(v0, v2), std::max(v0, v2) }].push_back(i);
  }

  // Compute triangle normals and filter edges
  auto ComputeTriNormal = [&](size_t t, float* n) {
    const float* a = vtx + tri[t * 3 + 0] * 3;
    const float* b = vtx + tri[t * 3 + 1] * 3;
    const float* c = vtx + tri[t * 3 + 2] * 3;
    float e1[3]{ b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    float e2[3]{ c[0] - a[0], c[1] - a[1], c[2] - a[2] };
    n[0] = e1[1] * e2[2] - e1[2] * e2[1];
    n[1] = e1[2] * e2[0] - e1[0] * e2[2];
    n[2] = e1[0] * e2[1] - e1[1] * e2[0];
    float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len > 1e-6f) {
      n[0] /= len;
      n[1] /= len;
      n[2] /= len;
    }
  };

  // Collect dihedral/boundary edges with their adjacent face normals
  struct EdgeInfo {
    Edge edge;
    float n0[3];
    float n1[3];
  };
  std::vector<EdgeInfo> edge_infos;
  for (const auto& [edge, tris] : edge_tris) {
    if (tris.size() == 1) {
      // Boundary edge — store same normal for both sides
      EdgeInfo info;
      info.edge = edge;
      ComputeTriNormal(tris[0], info.n0);
      std::memcpy(info.n1, info.n0, 3 * sizeof(float));
      edge_infos.push_back(info);
    } else if (tris.size() >= 2) {
      // Include if adjacent triangles have different normals (dihedral edge)
      float n0[3], n1[3];
      ComputeTriNormal(tris[0], n0);
      ComputeTriNormal(tris[1], n1);
      float dot = n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2];
      if (dot < 1.0f - 1e-3f) {
        EdgeInfo info;
        info.edge = edge;
        std::memcpy(info.n0, n0, 3 * sizeof(float));
        std::memcpy(info.n1, n1, 3 * sizeof(float));
        edge_infos.push_back(info);
      }
    }
  }

  auto edge_cnt = static_cast<int>(edge_infos.size());
  if (edge_cnt > LUMICE_MAX_CRYSTAL_EDGES) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  out->edge_count = edge_cnt;
  for (int i = 0; i < edge_cnt; i++) {
    out->edges[i * 2 + 0] = edge_infos[i].edge.first;
    out->edges[i * 2 + 1] = edge_infos[i].edge.second;
    std::memcpy(&out->edge_face_normals[i * 6 + 0], edge_infos[i].n0, 3 * sizeof(float));
    std::memcpy(&out->edge_face_normals[i * 6 + 3], edge_infos[i].n1, 3 * sizeof(float));
  }

  return LUMICE_OK;
}
