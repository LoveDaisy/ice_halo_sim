#include "include/cserver.h"

#include <cstddef>
#include <memory>
#include <variant>
#include <vector>

#include "include/server.hpp"

namespace ns = icehalo::v3;

// =============== C API ===============
struct HS_HaloSimServer_ {
  std::unique_ptr<ns::Server> server_;
};

struct HS_SimResult_ {
  std::vector<ns::Result> results_;
  size_t curr_idx_;
};


// =============== Creation & Destroy ===============
HS_HaloSimServer* HS_CreateServer() {
  auto* s = new HS_HaloSimServer;
  s->server_ = std::unique_ptr<ns::Server>{ new ns::Server{} };
  return s;
}


void HS_DestroyServer(HS_HaloSimServer* server) {
  if (!server) {
    return;
  }
  delete server;
}


// =============== Control ===============
HS_ServerState HS_QueryServerState(HS_HaloSimServer* server) {
  if (!server) {
    return HS_SERVER_NOT_READY;
  }

  if (server->server_->IsIdle()) {
    return HS_SERVER_IDLE;
  } else {
    return HS_SERVER_RUNNING;
  }
}


void HS_CommitConfig(HS_HaloSimServer* server, const char* config_str) {
  if (!server) {
    return;
  }

  // C API ignores errors for backward compatibility
  // Error handling can be added in future C API updates
  server->server_->CommitConfig(config_str);
}


void HS_StopServer(HS_HaloSimServer* server) {
  if (!server) {
    return;
  }

  server->server_->Stop();
}


// =============== Result ===============
HS_SimResult* HS_GetAllResults(HS_HaloSimServer* server) {
  if (!server) {
    return nullptr;
  }

  auto* res = new HS_SimResult;
  res->results_ = server->server_->GetResults();
  res->curr_idx_ = 0;
  return res;
}


int HS_HasNextResult(HS_SimResult* result) {
  if (!result) {
    return 0;
  }

  return result->curr_idx_ < result->results_.size();
}


HS_SimResult* HS_GetNextResult(HS_SimResult* result) {
  if (!result) {
    return nullptr;
  }

  result->curr_idx_++;
  return result;
}


struct ResultTypeIndicator {
  HS_SimResultType operator()(const ns::NoneResult& /* res */) { return HS_RESULT_NONE; }

  HS_SimResultType operator()(const ns::StatsResult& /* res */) { return HS_RESULT_STATS; }

  HS_SimResultType operator()(const ns::RenderResult& /* res */) { return HS_RESULT_RENDER; }
};

HS_SimResultType HS_QueryResultType(HS_SimResult* result) {
  if (!result || result->curr_idx_ >= result->results_.size()) {
    return HS_RESULT_NONE;
  }

  return std::visit(ResultTypeIndicator{}, result->results_[result->curr_idx_]);
}


HS_RenderResult HS_GetRenderResult(HS_SimResult* result) {
  if (!result || result->curr_idx_ >= result->results_.size()) {
    return HS_RenderResult{};
  }

  if (const auto* p = std::get_if<ns::RenderResult>(&(result->results_[result->curr_idx_]))) {
    HS_RenderResult res{};
    res.renderer_id_ = p->renderer_id_;
    res.img_width_ = p->img_width_;
    res.img_height_ = p->img_height_;
    res.img_buffer_ = p->img_buffer_;
    return res;
  } else {
    return HS_RenderResult{};
  }
}


HS_StatsResult HS_GetStatsResult(HS_SimResult* result) {
  if (!result || result->curr_idx_ >= result->results_.size()) {
    return HS_StatsResult{};
  }

  if (const auto* p = std::get_if<ns::StatsResult>(&(result->results_[result->curr_idx_]))) {
    HS_StatsResult res{};
    res.crystal_num_ = p->crystal_num_;
    res.ray_seg_num_ = p->ray_seg_num_;
    res.sim_ray_num_ = p->sim_ray_num_;
    return res;
  } else {
    return HS_StatsResult{};
  }
}

void HS_DeleteAllResults(HS_SimResult* result) {
  delete result;
}
