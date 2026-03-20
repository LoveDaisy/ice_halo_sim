#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <utility>
#include <vector>

#include "core/geo3d.hpp"
#include "include/lumice.h"
#include "server/server.hpp"
#include "util/logger.hpp"

namespace ns = lumice;

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


void LUMICE_DestroyServer(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  server->server_->Terminate();
  delete server;
}


// =============== Logging ===============
void LUMICE_InitLogger(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  ns::InitGlobalLogger();
}


void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level) {
  if (!server) {
    return;
  }
  static constexpr ns::LogLevel kLevelMap[] = {
    ns::LogLevel::kTrace,    // LUMICE_LOG_TRACE
    ns::LogLevel::kDebug,    // LUMICE_LOG_DEBUG
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

  auto err = server->server_->CommitConfigFromFile(filename);
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
  j["type"] = d.type == 0 ? "gauss" : "uniform";
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
      j["shape"]["upper_indices"] = { cr.upper_indices[0], cr.upper_indices[1], cr.upper_indices[2] };
      j["shape"]["lower_indices"] = { cr.lower_indices[0], cr.lower_indices[1], cr.lower_indices[2] };
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

  // Renderers — Core always produces full equirectangular texture.
  // GUI-variable fields come from the struct; fixed fields are hardcoded here.
  root["render"] = json::array();
  for (int i = 0; i < c.renderer_count; i++) {
    const auto& r = c.renderers[i];
    json jr;
    jr["id"] = r.id;
    jr["lens"]["type"] = "rectangular";
    jr["lens"]["fov"] = 180.0f;
    jr["resolution"] = { r.resolution_w, r.resolution_h };
    jr["view"]["elevation"] = 0.0f;
    jr["view"]["azimuth"] = 0.0f;
    jr["view"]["roll"] = 0.0f;
    jr["visible"] = "full";
    jr["background"] = { 0.0f, 0.0f, 0.0f };
    jr["opacity"] = r.opacity;
    jr["intensity_factor"] = r.intensity_factor;
    root["render"].push_back(jr);
  }

  return root;
}

LUMICE_ErrorCode LUMICE_CommitConfigStruct(LUMICE_Server* server, const LUMICE_Config* config) {
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
  auto err = server->server_->CommitConfig(config_json);
  if (err) {
    LOG_ERROR("Failed to commit configuration (struct): {}", err.message);
    return MapErrorCode(err.code);
  }
  return LUMICE_OK;
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

  try {
    if (type_str == "prism") {
      float h = shape.value("height", 1.0f);
      mesh = ns::CreatePrismMesh(h);
    } else if (type_str == "pyramid") {
      float prism_h = shape.value("prism_h", 1.0f);
      float upper_h = shape.value("upper_h", 0.0f);
      float lower_h = shape.value("lower_h", 0.0f);
      mesh = ns::CreatePyramidMesh(upper_h, prism_h, lower_h);
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
