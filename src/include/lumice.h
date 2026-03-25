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

typedef struct LUMICE_RawXyzResult_ {
  int renderer_id;
  int img_width;
  int img_height;
  const float* xyz_buffer;                 // Read-only XYZ float data, 3 floats per pixel.
                                           // Valid until next LUMICE_GetRawXyzResults() or LUMICE_CommitConfig().
                                           // Sentinel: xyz_buffer == NULL
  float snapshot_intensity;                // Per-pixel landed intensity (= landed_ray_weights / total_pixels)
  float intensity_factor;                  // Per-renderer intensity factor (2^EV)
  int has_valid_data;                      // Non-zero once simulation has produced data (reset on CommitConfig/Stop)
  unsigned long long snapshot_generation;  // Increments on each new snapshot; compare to detect data changes
} LUMICE_RawXyzResult;

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

// Log callback: receives all Core log messages. Called from Core logging threads.
// Parameters: level, logger name (e.g. "Server", "Simulator"), pre-formatted message.
// The callback must be thread-safe.
typedef void (*LUMICE_LogCallback)(LUMICE_LogLevel level, const char* logger_name, const char* message);

// Register a log callback. Pass NULL to disable. Must be called BEFORE LUMICE_CreateServer()
// for full coverage, but can also be called later (subsequent log messages will be forwarded).
void LUMICE_SetLogCallback(LUMICE_LogCallback callback);

// =============== Configuration (JSON string) ===============
LUMICE_ErrorCode LUMICE_CommitConfig(LUMICE_Server* server, const char* config_str);
LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename);

// =============== Configuration (C struct — bypasses JSON string serialization) ===============
// GUI fills this struct directly from UI state, avoiding JSON dump/parse overhead.
// Cross-references use integer IDs (crystal_id, filter_id), resolved internally by Core.

#define LUMICE_MAX_CONFIG_CRYSTALS 16
#define LUMICE_MAX_CONFIG_FILTERS 16
#define LUMICE_MAX_CONFIG_RENDERERS 4
#define LUMICE_MAX_CONFIG_SCATTER_LAYERS 8
#define LUMICE_MAX_CONFIG_SCATTER_ENTRIES 16
#define LUMICE_MAX_CONFIG_RAYPATH_LEN 32

typedef struct LUMICE_AxisDist_ {
  int type;    // 0=gauss, 1=uniform
  float mean;  // degrees
  float std;   // gauss: standard deviation; uniform: half-range (degrees)
} LUMICE_AxisDist;

typedef struct LUMICE_CrystalParam_ {
  int id;
  int type;  // 0=prism, 1=pyramid

  // Prism
  float height;

  // Pyramid
  float prism_h;
  float upper_h;
  float lower_h;
  int upper_indices[3];
  int lower_indices[3];

  // Axis distributions
  LUMICE_AxisDist zenith;
  LUMICE_AxisDist azimuth;
  LUMICE_AxisDist roll;
} LUMICE_CrystalParam;

typedef struct LUMICE_FilterParam_ {
  int id;
  int action;    // 0=filter_in, 1=filter_out
  int symmetry;  // bitmask: 1=P, 2=B, 4=D
  int raypath[LUMICE_MAX_CONFIG_RAYPATH_LEN];
  int raypath_count;
} LUMICE_FilterParam;

typedef struct LUMICE_ScatterEntry_ {
  int crystal_id;
  float proportion;
  int filter_id;  // -1 = none
} LUMICE_ScatterEntry;

typedef struct LUMICE_ScatterLayer_ {
  float probability;
  LUMICE_ScatterEntry entries[LUMICE_MAX_CONFIG_SCATTER_ENTRIES];
  int entry_count;
} LUMICE_ScatterLayer;

typedef struct LUMICE_RenderParam_ {
  int id;
  int resolution_w;
  int resolution_h;
  float opacity;
  float intensity_factor;
} LUMICE_RenderParam;

typedef struct LUMICE_Config_ {
  // Crystals
  LUMICE_CrystalParam crystals[LUMICE_MAX_CONFIG_CRYSTALS];
  int crystal_count;

  // Filters
  LUMICE_FilterParam filters[LUMICE_MAX_CONFIG_FILTERS];
  int filter_count;

  // Renderers
  LUMICE_RenderParam renderers[LUMICE_MAX_CONFIG_RENDERERS];
  int renderer_count;

  // Scene: light source
  float sun_altitude;
  float sun_azimuth;
  float sun_diameter;
  const char* spectrum;  // e.g. "D65", "D50", "A", "E"

  // Scene: simulation
  int infinite;           // 1=infinite rays, 0=finite
  unsigned long ray_num;  // only used when infinite==0
  int max_hits;

  // Scene: scattering
  LUMICE_ScatterLayer scattering[LUMICE_MAX_CONFIG_SCATTER_LAYERS];
  int scatter_count;
} LUMICE_Config;

// out_reused: if non-NULL, set to 1 if consumers were reused (no buffer realloc), 0 if rebuilt.
LUMICE_ErrorCode LUMICE_CommitConfigStruct(LUMICE_Server* server, const LUMICE_Config* config, int* out_reused);

// =============== Results ===============
// Unified pattern: (server, out, max_count) -> LUMICE_ErrorCode, sentinel-terminated.
// out array size must be at least max_count + 1 (for sentinel slot).

// Fill render results into out array, sentinel-terminated (img_buffer == NULL).
// Returns sRGB uint8 image data (CPU-converted from XYZ).
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);

// Fill raw XYZ results into out array, sentinel-terminated (xyz_buffer == NULL).
// Returns unconverted XYZ float data + intensity scalars for GPU-side conversion.
LUMICE_ErrorCode LUMICE_GetRawXyzResults(LUMICE_Server* server, LUMICE_RawXyzResult* out, int max_count);

// Fill stats results into out array, sentinel-terminated (sim_ray_num == 0).
// Note: triggers DoSnapshot internally (includes PostSnapshot XYZ→RGB conversion).
LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count);

// Get cached stats without triggering DoSnapshot/PostSnapshot.
// Returns the stats from the most recent snapshot (updated by LUMICE_GetRawXyzResults).
// Returns all-zero struct if no snapshot has been taken yet.
LUMICE_ErrorCode LUMICE_GetCachedStats(LUMICE_Server* server, LUMICE_StatsResult* out);

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
