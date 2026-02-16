#ifndef ICEHALO_H_
#define ICEHALO_H_

#ifdef __cplusplus
extern "C" {
#endif

// 符号可见性：配合 CMakeLists.txt 中 Release 构建的 -fvisibility=hidden，
// 仅导出此 pragma 块内的 C API 函数，其余内部符号均隐藏。
// NOTE: 当前为 GCC/Clang 专用。若需支持 MSVC 共享库（DLL），应替换为逐函数
// HS_API 宏（构建时 __declspec(dllexport)，使用时 __declspec(dllimport)）。
#pragma GCC visibility push(default)

// =============== Constants ===============
#define HS_MAX_RENDER_RESULTS 16
#define HS_MAX_STATS_RESULTS 1

// =============== Opaque Types ===============
typedef struct HS_HaloSimServer_ HS_HaloSimServer;

// =============== Error Codes ===============
typedef enum HS_ErrorCode_ {
  HS_OK = 0,
  HS_ERR_NULL_ARG,
  HS_ERR_INVALID_JSON,
  HS_ERR_INVALID_CONFIG,
  HS_ERR_MISSING_FIELD,
  HS_ERR_INVALID_VALUE,
  HS_ERR_FILE_NOT_FOUND,
  HS_ERR_SERVER,
} HS_ErrorCode;

// =============== Log Levels ===============
typedef enum HS_LogLevel_ {
  HS_LOG_TRACE,
  HS_LOG_DEBUG,
  HS_LOG_INFO,
  HS_LOG_WARNING,
  HS_LOG_ERROR,
  HS_LOG_OFF,
} HS_LogLevel;

// =============== Server State ===============
typedef enum HS_ServerState_ {
  HS_SERVER_IDLE,
  HS_SERVER_RUNNING,
  HS_SERVER_NOT_READY,
} HS_ServerState;

// =============== Result Structs ===============
typedef struct HS_RenderResult_ {
  int renderer_id;
  int img_width;
  int img_height;
  const unsigned char* img_buffer;  // Read-only, managed by Server.
                                    // Valid until next HS_GetRenderResults() or HS_CommitConfig().
                                    // Sentinel: img_buffer == NULL
} HS_RenderResult;

typedef struct HS_StatsResult_ {
  unsigned long ray_seg_num;
  unsigned long sim_ray_num;
  unsigned long crystal_num;
  // Sentinel: all zeros (sim_ray_num == 0)
} HS_StatsResult;

// =============== Server Lifecycle ===============
HS_HaloSimServer* HS_CreateServer(void);
void HS_DestroyServer(HS_HaloSimServer* server);

// =============== Logging ===============
void HS_InitLogger(HS_HaloSimServer* server);
void HS_SetLogLevel(HS_HaloSimServer* server, HS_LogLevel level);

// =============== Configuration ===============
HS_ErrorCode HS_CommitConfig(HS_HaloSimServer* server, const char* config_str);
HS_ErrorCode HS_CommitConfigFromFile(HS_HaloSimServer* server, const char* filename);

// =============== Results ===============
// Unified pattern: (server, out, max_count) -> HS_ErrorCode, sentinel-terminated.
// out array size must be at least max_count + 1 (for sentinel slot).

// Fill render results into out array, sentinel-terminated (img_buffer == NULL).
HS_ErrorCode HS_GetRenderResults(HS_HaloSimServer* server, HS_RenderResult* out, int max_count);

// Fill stats results into out array, sentinel-terminated (sim_ray_num == 0).
HS_ErrorCode HS_GetStatsResults(HS_HaloSimServer* server, HS_StatsResult* out, int max_count);

// =============== State & Control ===============
HS_ErrorCode HS_QueryServerState(HS_HaloSimServer* server, HS_ServerState* out);
void HS_StopServer(HS_HaloSimServer* server);

#pragma GCC visibility pop

#ifdef __cplusplus
}
#endif

#endif  // ICEHALO_H_
