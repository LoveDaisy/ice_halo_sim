#include <cstring>
#include <memory>

#include "include/icehalo.h"
#include "server/server.hpp"
#include "util/logger.hpp"

namespace ns = lumice;

// =============== Internal Helpers ===============
struct HS_HaloSimServer_ {
  std::unique_ptr<ns::Server> server_;
};


static HS_ErrorCode MapErrorCode(ns::ErrorCode code) {
  switch (code) {
    case ns::ErrorCode::kSuccess:
      return HS_OK;
    case ns::ErrorCode::kInvalidJson:
      return HS_ERR_INVALID_JSON;
    case ns::ErrorCode::kInvalidConfig:
      return HS_ERR_INVALID_CONFIG;
    case ns::ErrorCode::kMissingField:
      return HS_ERR_MISSING_FIELD;
    case ns::ErrorCode::kInvalidValue:
      return HS_ERR_INVALID_VALUE;
    case ns::ErrorCode::kServerNotReady:
    case ns::ErrorCode::kServerError:
    default:
      return HS_ERR_SERVER;
  }
}


// =============== Server Lifecycle ===============
HS_HaloSimServer* HS_CreateServer() {
  auto* s = new HS_HaloSimServer;
  s->server_ = std::make_unique<ns::Server>();
  return s;
}


void HS_DestroyServer(HS_HaloSimServer* server) {
  if (!server) {
    return;
  }
  server->server_->Terminate();
  delete server;
}


// =============== Logging ===============
void HS_InitLogger(HS_HaloSimServer* server) {
  if (!server) {
    return;
  }
  ns::InitGlobalLogger();
}


void HS_SetLogLevel(HS_HaloSimServer* server, HS_LogLevel level) {
  if (!server) {
    return;
  }
  static constexpr ns::LogLevel kLevelMap[] = {
    ns::LogLevel::kTrace,    // HS_LOG_TRACE
    ns::LogLevel::kDebug,    // HS_LOG_DEBUG
    ns::LogLevel::kInfo,     // HS_LOG_INFO
    ns::LogLevel::kWarning,  // HS_LOG_WARNING
    ns::LogLevel::kError,    // HS_LOG_ERROR
    ns::LogLevel::kOff,      // HS_LOG_OFF
  };
  if (level >= HS_LOG_TRACE && level <= HS_LOG_OFF) {
    auto mapped = kLevelMap[level];
    server->server_->SetLogLevel(mapped);
    ns::GetGlobalLogger().SetLevel(mapped);
  }
}


// =============== Configuration ===============
HS_ErrorCode HS_CommitConfig(HS_HaloSimServer* server, const char* config_str) {
  if (!server || !config_str) {
    return HS_ERR_NULL_ARG;
  }

  auto err = server->server_->CommitConfig(config_str);
  if (err) {
    LOG_ERROR("Failed to commit configuration: {}", err.message);
    if (!err.field.empty()) {
      LOG_ERROR("Error field: {}", err.field);
    }
    return MapErrorCode(err.code);
  }
  return HS_OK;
}


HS_ErrorCode HS_CommitConfigFromFile(HS_HaloSimServer* server, const char* filename) {
  if (!server || !filename) {
    return HS_ERR_NULL_ARG;
  }

  auto err = server->server_->CommitConfigFromFile(filename);
  if (err) {
    LOG_ERROR("Failed to load configuration from file '{}': {}", filename, err.message);
    if (!err.field.empty()) {
      LOG_ERROR("Error field: {}", err.field);
    }
    return MapErrorCode(err.code);
  }
  return HS_OK;
}


// =============== Results ===============
HS_ErrorCode HS_GetRenderResults(HS_HaloSimServer* server, HS_RenderResult* out, int max_count) {
  if (!server || !out) {
    return HS_ERR_NULL_ARG;
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
  std::memset(&out[count], 0, sizeof(HS_RenderResult));

  return HS_OK;
}


HS_ErrorCode HS_GetStatsResults(HS_HaloSimServer* server, HS_StatsResult* out, int max_count) {
  if (!server || !out) {
    return HS_ERR_NULL_ARG;
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
  std::memset(&out[count], 0, sizeof(HS_StatsResult));

  return HS_OK;
}


// =============== State & Control ===============
HS_ErrorCode HS_QueryServerState(HS_HaloSimServer* server, HS_ServerState* out) {
  if (!server || !out) {
    return HS_ERR_NULL_ARG;
  }

  if (server->server_->IsIdle()) {
    *out = HS_SERVER_IDLE;
  } else {
    *out = HS_SERVER_RUNNING;
  }

  return HS_OK;
}


void HS_StopServer(HS_HaloSimServer* server) {
  if (!server) {
    return;
  }

  server->server_->Stop();
}
