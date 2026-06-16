#ifndef LUMICE_H_
#define LUMICE_H_

#include <stddef.h>

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
  LUMICE_ERR_UNKNOWN,
} LUMICE_ErrorCode;

// =============== Log Levels ===============
typedef enum LUMICE_LogLevel_ {
  LUMICE_LOG_TRACE,
  LUMICE_LOG_DEBUG,
  LUMICE_LOG_VERBOSE,
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
  const float* xyz_buffer;   // Read-only XYZ float data, 3 floats per pixel.
                             // Valid until next LUMICE_GetRawXyzResults() or LUMICE_CommitConfig().
                             // Sentinel: xyz_buffer == NULL
  float snapshot_intensity;  // Per-pixel landed intensity (landed_ray_weights / (kNormScale * total_pixels))
  float intensity_factor;    // Per-renderer intensity factor (2^EV)
  int has_valid_data;        // Non-zero once simulation has produced data (reset on CommitConfig/Stop)
  unsigned long long snapshot_generation;  // Increments on each new snapshot; compare to detect data changes
  int effective_pixels;                    // Non-zero pixel count (for stats display)
} LUMICE_RawXyzResult;

typedef struct LUMICE_StatsResult_ {
  unsigned long ray_seg_num;
  unsigned long sim_ray_num;
  unsigned long crystal_num;
  // Sentinel: all zeros (sim_ray_num == 0)
} LUMICE_StatsResult;

// =============== Server Configuration ===============
typedef struct LUMICE_ServerConfig_ {
  int num_workers;        // CPU route only: worker count (0 = PhysicalCoreCount()).
                          // Ignored on the GPU/Metal route (always one engine, task-268.7).
  unsigned int sim_seed;  // Deterministic seed for the worker RNG. 0 = random (default).
  int preferred_backend;  // LUMICE_BACKEND_CPU (0, multi-worker) or LUMICE_BACKEND_METAL
                          // (1, single engine). Fixes the route at construction; the worker
                          // count follows it. env LUMICE_TRACE_BACKEND overrides. The GUI
                          // reconstructs the server when the Metal checkbox toggles.
} LUMICE_ServerConfig;

// =============== Server Lifecycle ===============
LUMICE_Server* LUMICE_CreateServer(void);
LUMICE_Server* LUMICE_CreateServerEx(const LUMICE_ServerConfig* config);
void LUMICE_DestroyServer(LUMICE_Server* server);

// =============== Logging ===============
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
LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename);  // filename must be UTF-8

// =============== Configuration (C struct — bypasses JSON string serialization) ===============
// GUI fills this struct directly from UI state, avoiding JSON dump/parse overhead.
// Cross-references use integer IDs (crystal_id, filter_id), resolved internally by Core.

#define LUMICE_MAX_CONFIG_CRYSTALS 256
#define LUMICE_MAX_CONFIG_FILTERS 256
#define LUMICE_MAX_CONFIG_RENDERERS 4
#define LUMICE_MAX_CONFIG_SCATTER_LAYERS 8
#define LUMICE_MAX_CONFIG_SCATTER_ENTRIES 256
#define LUMICE_MAX_CONFIG_RAYPATH_LEN 32

// Axis distribution type constants for LUMICE_AxisDist.type
#define LUMICE_AXIS_DIST_GAUSS 0
#define LUMICE_AXIS_DIST_UNIFORM 1
#define LUMICE_AXIS_DIST_ZIGZAG 2
#define LUMICE_AXIS_DIST_LAPLACIAN 3
#define LUMICE_AXIS_DIST_GAUSS_LEGACY 4

typedef struct LUMICE_AxisDist_ {
  int type;    // LUMICE_AXIS_DIST_GAUSS / UNIFORM / ZIGZAG / LAPLACIAN / GAUSS_LEGACY
  float mean;  // degrees
  float std;   // gauss/gauss_legacy: std dev; uniform: full range; zigzag: amplitude; laplacian: scale (degrees)
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
  float upper_wedge_angle;  // degrees, angle between pyramidal face and c-axis
  float lower_wedge_angle;  // degrees

  // Face distance (distance from center to each of the 6 prism faces).
  // Default: all 1.0f (regular hexagonal prism). Caller must initialize; memset(0) gives invalid geometry.
  float face_distance[6];

  // Axis distributions. Sampled values feed the rotation chain (angles in degrees)
  // R = Rz(azimuth - 180°) * Ry(-zenith) * Rz(roll); see doc/coordinate-convention.md.
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

// BREAKING (v4.3): norm_mode field removed; struct layout changed. Callers must recompile against this header.
typedef struct LUMICE_RenderParam_ {
  int id;
  int resolution_w;
  int resolution_h;
  float opacity;
  float intensity_factor;
  float overlap;  // Dual fisheye overlap zone |sky.z| threshold (sin value). 0 = no overlap.
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

// =============== Configuration Parsing (JSON -> LUMICE_Config) ===============
// Parse JSON into LUMICE_Config struct, enabling load-modify-commit workflows.
//
// Input format: accepts JSON in the format produced by LUMICE_CommitConfigStruct round-trip
// (i.e., the subset of the full config format that LUMICE_Config can represent).
// Specifically:
//   - crystal: height/face_distance as scalars/arrays, axis as {type, mean, std} objects
//   - filter: only type="raypath" supported; other types return LUMICE_ERR_INVALID_VALUE
//   - render: lens/view/visible/background fields are ignored; only id/resolution/opacity/
//     intensity_factor/overlap are parsed
//   - spectrum: only string enumerations ("D65","D50","A","E"); arrays return LUMICE_ERR_INVALID_VALUE
//
// The spectrum field in the output struct points to static storage; the caller must not free it.
// If the caller replaces spectrum with a custom string for CommitConfigStruct, the caller
// manages that string's lifetime.
//
// For full-format JSON (user-written configs with distribution parameters, complex filters, etc.),
// use LUMICE_CommitConfig() or LUMICE_CommitConfigFromFile() instead.

// Parse a JSON string into LUMICE_Config. Returns LUMICE_ERR_INVALID_JSON on parse failure.
LUMICE_ErrorCode LUMICE_ParseConfigString(const char* json_str, LUMICE_Config* out);

// Parse a JSON file into LUMICE_Config. filename must be UTF-8 encoded.
// Returns LUMICE_ERR_FILE_NOT_FOUND if the file cannot be opened.
LUMICE_ErrorCode LUMICE_ParseConfigFile(const char* filename, LUMICE_Config* out);

// =============== Results ===============
// See doc/capi-lifecycle-architecture.md §5 for sentinel contract.
// Unified pattern: (server, out, max_count) -> LUMICE_ErrorCode, sentinel-terminated.
// Sentinel is written at out[count] only when count < max_count.
// When the array is full (count == max_count), no sentinel slot is written;
// callers relying on sentinel iteration must value-initialize the array
// (e.g., Type arr[N + 1]{}) and pass max_count = N. Minimum array size:
// max_count + 1 for sentinel iteration, max_count for direct index access.

// Fill render results into out array (img_buffer == NULL sentinel when count < max_count).
// Returns sRGB uint8 image data (CPU-converted from XYZ).
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);

// Fill raw XYZ results into out array (xyz_buffer == NULL sentinel when count < max_count).
// Returns unconverted XYZ float data + intensity scalars for GPU-side conversion.
LUMICE_ErrorCode LUMICE_GetRawXyzResults(LUMICE_Server* server, LUMICE_RawXyzResult* out, int max_count);

// Fill stats results into out array (sim_ray_num == 0 sentinel when count < max_count).
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
#define LUMICE_MAX_CRYSTAL_FACES 24
#define LUMICE_MAX_CRYSTAL_FACE_VTXPOOL 192

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
  // Per-triangle face number (matches raypath filter numbering convention):
  //   basal = 1/2; prism = 3..8; upper pyramidal = 13..18; lower = 23..28.
  // -1 for unrecognized orientations (kInvalidId in core).
  int face_numbers[LUMICE_MAX_CRYSTAL_TRIANGLES];
  // Per-face polygon topology (CCW ordered vertex indices when viewed from outside).
  // face_vtx_pool[face_vtx_offsets[i] .. face_vtx_offsets[i]+face_vtx_counts[i]-1]
  // gives the CCW vertex indices for face i. face_count=0 means not populated.
  int face_count;
  int face_numbers_by_face[LUMICE_MAX_CRYSTAL_FACES];
  int face_vtx_offsets[LUMICE_MAX_CRYSTAL_FACES];
  int face_vtx_counts[LUMICE_MAX_CRYSTAL_FACES];
  int face_vtx_pool[LUMICE_MAX_CRYSTAL_FACE_VTXPOOL];
} LUMICE_CrystalMesh;

LUMICE_ErrorCode LUMICE_GetCrystalMesh(LUMICE_Server* server, const char* crystal_json, LUMICE_CrystalMesh* out);

// =============== Config ID Range ===============
// Maximum value for LUMICE config IDs (matches core IdType = uint16_t max).
// GUI code should clamp user-editable IDs to [0, LUMICE_MAX_ID].
#define LUMICE_MAX_ID 65535

// =============== Crystal Kind ===============
// Coarse crystal classification used for raypath face-number validation.
// GUI uses this to determine which face numbers are legal for a given crystal.
typedef enum LUMICE_CrystalKind_ {
  LUMICE_CRYSTAL_PRISM,    // Basal + prism lateral faces (1,2,3-8)
  LUMICE_CRYSTAL_PYRAMID,  // All faces including upper/lower pyramidal (1,2,3-8,13-18,23-28)
} LUMICE_CrystalKind;

// Returns non-zero if `face` is a legal face number for the given crystal kind.
int LUMICE_IsLegalFace(LUMICE_CrystalKind kind, int face);

// =============== Raypath Validation ===============
// Validation state for raypath text input (GUI border color + OK gate).
typedef enum LUMICE_RaypathValidationState_ {
  LUMICE_RAYPATH_VALID,       // All tokens valid; safe to submit
  LUMICE_RAYPATH_INCOMPLETE,  // Trailing/leading separator; user still typing
  LUMICE_RAYPATH_INVALID,     // Non-numeric tokens or illegal face numbers
} LUMICE_RaypathValidationState;

// Validate a raypath text string (dash- or comma-separated face indices) against
// both syntax rules and face-number legality for the given crystal kind.
// Used by GUI for raypath filter input validation.
// out_msg: human-readable error description (empty on kValid/kIncomplete).
//          Caller provides buffer; recommended size = 256.
// Returns LUMICE_ERR_NULL_ARG if text, out_state, or out_msg is NULL.
LUMICE_ErrorCode LUMICE_ValidateRaypathText(const char* text, LUMICE_CrystalKind kind,
                                            LUMICE_RaypathValidationState* out_state, char* out_msg,
                                            size_t msg_buf_size);

// =============== Lens Type ===============
// Lens projection type. Values match Core's LensParam::LensType enum (index 0-10).
// Used by GUI to look up per-lens FOV limits without including config/render_config.hpp.
typedef enum LUMICE_LensType_ {
  LUMICE_LENS_LINEAR = 0,
  LUMICE_LENS_FISHEYE_EQUAL_AREA = 1,
  LUMICE_LENS_FISHEYE_EQUIDISTANT = 2,
  LUMICE_LENS_FISHEYE_STEREOGRAPHIC = 3,
  LUMICE_LENS_DUAL_FISHEYE_EQUAL_AREA = 4,
  LUMICE_LENS_DUAL_FISHEYE_EQUIDISTANT = 5,
  LUMICE_LENS_DUAL_FISHEYE_STEREOGRAPHIC = 6,
  LUMICE_LENS_RECTANGULAR = 7,
  LUMICE_LENS_FISHEYE_ORTHOGRAPHIC = 8,
  LUMICE_LENS_DUAL_FISHEYE_ORTHOGRAPHIC = 9,
  LUMICE_LENS_GLOBE = 10,
} LUMICE_LensType;

// Returns the maximum valid FOV (degrees) for the given lens type.
// Used by GUI to clamp the FOV slider upper bound when the user switches lens type.
float LUMICE_MaxFov(LUMICE_LensType type);

// =============== Color Conversion ===============
// Batch XYZ float -> sRGB uint8 conversion with per-pixel intensity scale.
// xyz_in:          flat array of XYZ tristimulus values, 3 floats per pixel
//                  (length = pixel_count * 3).
// out:             caller-allocated uint8 buffer of length pixel_count * 3.
// pixel_count:     number of pixels to convert.
// intensity_scale: scalar applied per-pixel to XYZ before XYZ->sRGB conversion.
// Returns LUMICE_ERR_NULL_ARG if xyz_in or out is NULL; LUMICE_OK otherwise.
LUMICE_ErrorCode LUMICE_XyzToSrgbUint8(const float* xyz_in, unsigned char* out, int pixel_count, float intensity_scale);

// =============== Preferred Trace Backend ===============
// Stable backend identifiers. Future backends (e.g. CUDA) append new positive
// values; 0 stays CPU so default zero-init = legacy behavior.
#define LUMICE_BACKEND_CPU 0
#define LUMICE_BACKEND_METAL 1

// Set preferred trace backend for this server.
//   backend = LUMICE_BACKEND_CPU   (default): legacy CPU path.
//   backend = LUMICE_BACKEND_METAL          : request Metal; falls back to CPU
//                                             if incompatible or unavailable
//                                             on this platform.
// Takes effect on the next simulation start (after LUMICE_CommitConfig). The
// env-var LUMICE_TRACE_BACKEND, when set, overrides this preference:
//   - "cpu_backend"            forces CPU unconditionally (ignores this pref).
//   - "metal"                  forces Metal (Apple) regardless of this pref.
//   - unset / "" / "legacy"    defers to this API preference.
// (i.e. an empty or "legacy" env-var no longer forces CPU once this pref is
//  set to LUMICE_BACKEND_METAL — use "cpu_backend" to hard-pin CPU in CI.)
// On non-Apple platforms LUMICE_BACKEND_METAL is silently treated as CPU.
void LUMICE_SetPreferredBackend(LUMICE_Server* server, int backend);

#if !defined(_MSC_VER)
#pragma GCC visibility pop
#endif

#ifdef __cplusplus
}
#endif

#endif  // LUMICE_H_
