#ifndef LUMICE_H_
#define LUMICE_H_

#ifdef __cplusplus
extern "C" {
#endif

// 符号可见性：配合 CMakeLists.txt 中 Release 构建的 -fvisibility=hidden，
// 仅导出此 pragma 块内的 C API 函数，其余内部符号均隐藏。
// NOTE: 当前为 GCC/Clang 专用。若需支持 MSVC 共享库（DLL），应替换为逐函数
// LUMICE_API 宏（构建时 __declspec(dllexport)，使用时 __declspec(dllimport)）。
#if !defined(_MSC_VER)
#pragma GCC visibility push(default)
#endif

// =============== Constants ===============
#define LUMICE_MAX_RENDER_RESULTS 16
#define LUMICE_MAX_STATS_RESULTS 1

// =============== Opaque Types ===============
typedef struct LUMICE_Server_ LUMICE_Server;

// =============== Error Codes ===============
typedef enum LUMICE_ErrorCode_ {
  LUMICE_OK = 0,
  LUMICE_ERR_NULL_ARG,
  LUMICE_ERR_INVALID_JSON,
  LUMICE_ERR_INVALID_CONFIG,
  LUMICE_ERR_MISSING_FIELD,
  LUMICE_ERR_INVALID_VALUE,
  LUMICE_ERR_FILE_NOT_FOUND,
  LUMICE_ERR_SERVER,
} LUMICE_ErrorCode;

// =============== Log Levels ===============
typedef enum LUMICE_LogLevel_ {
  LUMICE_LOG_TRACE,
  LUMICE_LOG_DEBUG,
  LUMICE_LOG_INFO,
  LUMICE_LOG_WARNING,
  LUMICE_LOG_ERROR,
  LUMICE_LOG_OFF,
} LUMICE_LogLevel;

// =============== Server State ===============
typedef enum LUMICE_ServerState_ {
  LUMICE_SERVER_IDLE,
  LUMICE_SERVER_RUNNING,
  LUMICE_SERVER_NOT_READY,
} LUMICE_ServerState;

// =============== Result Structs ===============
typedef struct LUMICE_RenderResult_ {
  int renderer_id;
  int img_width;
  int img_height;
  const unsigned char* img_buffer;  // Read-only, managed by Server.
                                    // Valid until next LUMICE_GetRenderResults() or LUMICE_CommitConfig().
                                    // Sentinel: img_buffer == NULL
} LUMICE_RenderResult;

typedef struct LUMICE_RawRenderResult_ {
  int renderer_id;
  int img_width;
  int img_height;
  const float* xyz_buffer;       // XYZ float data, 3 floats/pixel.
                                 // Valid until next LUMICE_PrepareAllSnapshots(),
                                 // LUMICE_Get*Results(), or LUMICE_CommitConfig() call.
  float total_intensity;         // Normalization factor for tone mapping
} LUMICE_RawRenderResult;

typedef struct LUMICE_StatsResult_ {
  unsigned long ray_seg_num;
  unsigned long sim_ray_num;
  unsigned long crystal_num;
  // Sentinel: all zeros (sim_ray_num == 0)
} LUMICE_StatsResult;

// =============== Server Lifecycle ===============
LUMICE_Server* LUMICE_CreateServer(void);
void LUMICE_DestroyServer(LUMICE_Server* server);

// =============== Logging ===============
void LUMICE_InitLogger(LUMICE_Server* server);
void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level);

// =============== Configuration ===============
LUMICE_ErrorCode LUMICE_CommitConfig(LUMICE_Server* server, const char* config_str);
LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename);

// =============== Results ===============
// Unified pattern: (server, out, max_count) -> LUMICE_ErrorCode, sentinel-terminated.
// out array size must be at least max_count + 1 (for sentinel slot).
// Callers must call LUMICE_PrepareAllSnapshots() before any Get*Results() calls.

// Snapshot all consumer data under lock. Must be called once before reading results.
LUMICE_ErrorCode LUMICE_PrepareAllSnapshots(LUMICE_Server* server);

// Fill render results into out array, sentinel-terminated (img_buffer == NULL).
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);

// Fill raw (XYZ float) render results into out array, sentinel-terminated (xyz_buffer == NULL).
// Used by the GUI for GPU-side tone mapping — skips the CPU XYZ→sRGB conversion.
LUMICE_ErrorCode LUMICE_GetRawRenderResults(LUMICE_Server* server, LUMICE_RawRenderResult* out, int max_count);

// Fill stats results into out array, sentinel-terminated (sim_ray_num == 0).
LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count);

// =============== State & Control ===============
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out);
void LUMICE_StopServer(LUMICE_Server* server);

// =============== Crystal Mesh ===============
// Get crystal wireframe mesh for 3D preview.
// crystal_json is a single crystal config JSON string, e.g.:
//   {"type": "prism", "shape": {"height": 1.0}}
//   {"type": "pyramid", "shape": {"prism_h": 1.0, "upper_h": 0.5, "lower_h": 0.5}}
// Caller allocates LUMICE_CrystalMesh on stack, Core fills vertex/edge data.
// server may be NULL (not used, reserved for future).

#define LUMICE_MAX_CRYSTAL_VERTICES 128
#define LUMICE_MAX_CRYSTAL_EDGES 256
#define LUMICE_MAX_CRYSTAL_TRIANGLES 128

typedef struct LUMICE_CrystalMesh_ {
  float vertices[LUMICE_MAX_CRYSTAL_VERTICES * 3];  // [x0,y0,z0, x1,y1,z1, ...]
  int vertex_count;
  int edges[LUMICE_MAX_CRYSTAL_EDGES * 2];  // [v0,v1, v2,v3, ...] vertex index pairs
  int edge_count;
  int triangles[LUMICE_MAX_CRYSTAL_TRIANGLES * 3];  // [v0,v1,v2, ...] for surface rendering
  int triangle_count;
  // Per-edge adjacent face normals for back-face culling.
  // Edge i has two face normals: [i*6..i*6+2] and [i*6+3..i*6+5].
  // Boundary edges store the same normal twice.
  float edge_face_normals[LUMICE_MAX_CRYSTAL_EDGES * 6];
} LUMICE_CrystalMesh;

LUMICE_ErrorCode LUMICE_GetCrystalMesh(LUMICE_Server* server, const char* crystal_json, LUMICE_CrystalMesh* out);

#if !defined(_MSC_VER)
#pragma GCC visibility pop
#endif

#ifdef __cplusplus
}
#endif

#endif  // LUMICE_H_
