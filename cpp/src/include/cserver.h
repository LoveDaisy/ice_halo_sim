#ifndef ICEHALOSIM_CSERVER_H_
#define ICEHALOSIM_CSERVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#pragma GCC visibility push(default)

typedef struct HS_HaloSimServer_ HS_HaloSimServer;

typedef enum HS_ServerState_ {
  HS_SERVER_IDLE,
  HS_SERVER_RUNNING,
  HS_SERVER_NOT_READY,
} HS_ServerState;

typedef struct HS_SimResult_ HS_SimResult;

typedef enum HS_SimResultType_ {
  HS_RESULT_NONE,
  HS_RESULT_RENDER,
  HS_RESULT_STATS,
} HS_SimResultType;

typedef struct HS_RenderResult_ {
  int renderer_id_;
  int img_width_;
  int img_height_;
  const unsigned char* img_buffer_;  // Read-only buffer, managed by Server
} HS_RenderResult;

typedef struct HS_StatsResult_ {
  unsigned long ray_seg_num_;
  unsigned long sim_ray_num_;
  unsigned long crystal_num_;
} HS_StatsResult;


// =============== Creation & Destroy ===============
HS_HaloSimServer* HS_CreateServer();
void HS_DestroyServer(HS_HaloSimServer* server);

// =============== Control ===============
HS_ServerState HS_QueryServerState(HS_HaloSimServer* server);
void HS_CommitConfig(HS_HaloSimServer* server, const char* config_str);
void HS_StopServer(HS_HaloSimServer* server);

// =============== Result ===============
HS_SimResult* HS_GetAllResults(HS_HaloSimServer* server);
int HS_HasNextResult(HS_SimResult* result);
HS_SimResult* HS_GetNextResult(HS_SimResult* result);
HS_SimResultType HS_QueryResultType(HS_SimResult* result);
HS_RenderResult HS_GetRenderResult(HS_SimResult* result);
HS_StatsResult HS_GetStatsResult(HS_SimResult* result);
void HS_DeleteAllResults(HS_SimResult* result);

#pragma GCC visibility pop

#ifdef __cplusplus
}
#endif

#endif  // ICEHALOSIM_CSERVER_H_
