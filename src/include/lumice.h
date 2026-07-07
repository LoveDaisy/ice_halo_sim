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

// =============== Simulation Lifecycle ===============
// Explicit single-source lifecycle truth (replaces disambiguation via bare
// LUMICE_SERVER_IDLE + has_valid_data + stats>0). LUMICE_QueryServerState is a
// projection of this enum: RUNNING -> RUNNING, IDLE|COMPLETED -> IDLE.
//   - COMPLETED = a finite run drained clean (includes zero-output / all-filter-
//     rejected convergence).
//   - IDLE      = never run, or reset (post-Stop) with no data consumed.
//   - RUNNING   = pending work / workers active. Infinite runs stay RUNNING
//     forever (never COMPLETED); Stop returns them to IDLE.
typedef enum LUMICE_SimLifecycle_ {
  LUMICE_LIFECYCLE_IDLE = 0,
  LUMICE_LIFECYCLE_RUNNING,
  LUMICE_LIFECYCLE_COMPLETED,
} LUMICE_SimLifecycle;

// {lifecycle, epoch} snapshot of the backend lifecycle truth.
//   lifecycle: one of LUMICE_SimLifecycle.
//   epoch:     monotonic generation counter, ++ on each reset-causing commit.
//              0 before any successful commit. Read back after a synchronous
//              commit to learn the just-minted epoch.
typedef struct LUMICE_SimLifecycleResult_ {
  int lifecycle;
  unsigned long long epoch;
} LUMICE_SimLifecycleResult;

// =============== Ray Count Type ===============
// 64-bit count type for ray / ray-segment / crystal totals. Must stay >= 64-bit:
// `unsigned long` is only 32-bit on Windows (LLP64), which silently truncated
// totals above 2^32 (the status-bar ray-count rollover reported by Windows users).
// Used for every field that carries an actual ray-count value across the C API.
typedef unsigned long long LUMICE_RayCount;
#if defined(__cplusplus)
static_assert(sizeof(LUMICE_RayCount) >= 8, "LUMICE_RayCount must be 64-bit (Windows unsigned long is 32-bit)");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(LUMICE_RayCount) >= 8, "LUMICE_RayCount must be 64-bit (Windows unsigned long is 32-bit)");
#endif

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
  unsigned long long epoch;                // Lifecycle epoch at snapshot time (committed_epoch_); 1.5 display keying
} LUMICE_RawXyzResult;

typedef struct LUMICE_StatsResult_ {
  LUMICE_RayCount ray_seg_num;
  LUMICE_RayCount sim_ray_num;
  LUMICE_RayCount crystal_num;
  // Sentinel: all zeros (sim_ray_num == 0)
} LUMICE_StatsResult;

// =============== Server Configuration ===============
typedef struct LUMICE_ServerConfig_ {
  int num_workers;        // CPU route only: worker count (0 = PhysicalCoreCount()).
                          // Ignored on the GPU/Metal route (always one engine, task-268.7).
  unsigned int sim_seed;  // Deterministic seed for the worker RNG. 0 = random (default).
  int preferred_backend;  // LUMICE_BACKEND_CPU (0, multi-worker), LUMICE_BACKEND_METAL
                          // (1, single engine on Apple) or LUMICE_BACKEND_CUDA (2, future).
                          // Fixes the route at construction; the worker count follows it.
                          // env LUMICE_TRACE_BACKEND overrides. The GUI reconstructs the
                          // server when the backend selection changes.
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
// Discrete-spectrum entry cap. Mirrors core wl_pool.hpp::kWlPoolSizeMax (255).
#define LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES 255
// Complex (sum-of-products) filter composition bounds. See LUMICE_ComplexComposition.
// These are ABI ceilings baked into the struct layout; widen (breaking bump) if needed.
#define LUMICE_MAX_CONFIG_COMPLEX 32  // max complex-filter composition records per config
#define LUMICE_MAX_CONFIG_CLAUSES 16  // max OR clauses per complex filter
#define LUMICE_MAX_CONFIG_TERMS 8     // max AND terms per clause
// Raypath color-class (Design 2, task-342.2) ABI bounds. Same "widen (breaking bump)" rule as
// LUMICE_MAX_CONFIG_COMPLEX / _CLAUSES / _TERMS. LUMICE_MAX_CONFIG_COLOR_CLASSES upper-aligns
// with core ComponentTable::kMaxBits (64), the deduped predicate-atom budget of a scene.
// LUMICE_MAX_CONFIG_COLOR_REFS is a per-class ref-count ceiling with generous headroom over
// the OR/AND expansion seen in practice.
#define LUMICE_MAX_CONFIG_COLOR_CLASSES 64
#define LUMICE_MAX_CONFIG_COLOR_REFS 32

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

// Filter type discriminant for LUMICE_FilterParam.type.
// 0 = UNSET is a deliberate zero-init guard: a struct built via memset/aggregate
// initialization without an explicit type lands on UNSET and is rejected at commit
// (LUMICE_ERR_INVALID_CONFIG) rather than being silently treated as "none". Callers
// that want the no-op "none" filter must set LUMICE_FILTER_TYPE_NONE explicitly.
#define LUMICE_FILTER_TYPE_UNSET 0
#define LUMICE_FILTER_TYPE_NONE 1
#define LUMICE_FILTER_TYPE_RAYPATH 2
#define LUMICE_FILTER_TYPE_ENTRY_EXIT 3
#define LUMICE_FILTER_TYPE_DIRECTION 4
#define LUMICE_FILTER_TYPE_CRYSTAL 5
// Reserved: complex (sum-of-products) filter reference encoding lands in a follow-up.
// Until then a filter with this type has no ConfigToJson case and is rejected at commit.
#define LUMICE_FILTER_TYPE_COMPLEX 6

// BREAKING (v4.5): LUMICE_FilterParam extended from raypath-only to a 5-arm tagged
// union (None/Raypath/EntryExit/Direction/Crystal). Layout changed; callers must
// recompile against this header. `type` selects the active arm; arm-specific fields are
// prefixed by arm (raypath_*, ee_*, dir_*, crystal_*). Field naming/units mirror core
// config/filter_config.hpp. -1 sentinels encode optional fields.
typedef struct LUMICE_FilterParam_ {
  int id;
  int action;  // 0=filter_in, 1=filter_out
  // Symmetry is a common field for ALL filter types (mirrors core FilterConfig.symmetry_,
  // emitted by filter_config.cpp::to_json before the per-type fields), not raypath-only.
  int symmetry;  // bitmask: 1=P, 2=B, 4=D
  int type;      // LUMICE_FILTER_TYPE_* (UNSET=0 is rejected at commit)

  // Raypath arm (type == LUMICE_FILTER_TYPE_RAYPATH)
  int raypath[LUMICE_MAX_CONFIG_RAYPATH_LEN];
  int raypath_count;

  // EntryExit arm (type == LUMICE_FILTER_TYPE_ENTRY_EXIT). -1 sentinels below.
  // NOTE: ee_min_len/ee_max_len mirror core's to_json emit conditions (min_len emitted
  // only when > 1; max_len only when >= 0). An out-of-contract value like ee_min_len == 0
  // is therefore emitted as "absent" and normalized to the core default (1) at commit
  // rather than rejected here. Callers must supply ee_min_len >= 1.
  // -1 is the ONLY sentinel: any other negative value is undefined (treated as wildcard/
  // absent, not rejected). These fields are int (matching the raypath[] convention); core
  // stores IdType(uint16_t) for entry/exit and size_t for the lengths, so keep entry/exit
  // in [0, 65535] and lengths reasonably small.
  int ee_entry;    // entry face id; -1 = wildcard (any entry face)
  int ee_exit;     // exit face id;  -1 = wildcard (any exit face)
  int ee_min_len;  // path length lower bound (>= 1)
  int ee_max_len;  // path length upper bound; -1 = no upper bound

  // Direction arm (type == LUMICE_FILTER_TYPE_DIRECTION). Degrees.
  float dir_az;     // azimuth (lon)
  float dir_el;     // elevation (lat)
  float dir_radii;  // angular radius (scalar, not an array)

  // Crystal arm (type == LUMICE_FILTER_TYPE_CRYSTAL)
  int crystal_id;

  // Complex arm (type == LUMICE_FILTER_TYPE_COMPLEX): index into
  // LUMICE_Config.compositions[] holding this filter's sum-of-products.
  int composition_index;
} LUMICE_FilterParam;

// Sum-of-products composition for a complex filter, referenced by
// LUMICE_FilterParam.composition_index. The outer level is an OR over clauses; each clause
// is an AND over its terms; each term (clauses[c][t]) is the ID of a SIMPLE filter in the
// same config's filters[] pool (referenced by id — reorder-robust — not by array index;
// a term may never reference another complex filter, matching core config semantics).
// INVARIANT: compositions[] is rebuilt wholesale together with its host LUMICE_Config;
// records are not individually reordered (composition_index is a pool index valid only
// within one config snapshot). Consumers (e.g. GUI) must respect this whole-rebuild model.
typedef struct LUMICE_ComplexComposition_ {
  int clauses[LUMICE_MAX_CONFIG_CLAUSES][LUMICE_MAX_CONFIG_TERMS];  // simple-filter IDs
  int term_counts[LUMICE_MAX_CONFIG_CLAUSES];                       // number of terms in each clause
  int clause_count;
} LUMICE_ComplexComposition;

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

typedef struct LUMICE_SpectrumEntry_ {
  float wavelength;  // nm
  float weight;      // relative weight (unnormalized; core normalizes)
} LUMICE_SpectrumEntry;
#if defined(__cplusplus)
static_assert(sizeof(LUMICE_SpectrumEntry) == 2 * sizeof(float),
              "LUMICE_SpectrumEntry must be tightly packed (2 floats, no padding) for ABI stability");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(LUMICE_SpectrumEntry) == 2 * sizeof(float),
               "LUMICE_SpectrumEntry must be tightly packed (2 floats, no padding) for ABI stability");
#endif

// =============== Raypath Color Classes (task-342.2, BREAKING v4.7) ===============
// Design 2 (2026-07-08, doc/gui-custom-spectrum-and-raypath-color.md §4.0): each color class
// is decoupled from the physical filter. A class has an RGB color + a set of "match" refs;
// each ref is a placement-scoped predicate {layer, crystal, predicate} that decides which
// surviving rays get color-tagged. Predicate types are a NARROWED reuse of LUMICE_FilterParam
// (raypath / entry_exit / direction / crystal / none) — no id, action, symmetry, composition,
// complex.

// A predicate is a match rule, not a filter. Field naming mirrors the equivalent arms of
// LUMICE_FilterParam. type selects the active arm:
//   LUMICE_FILTER_TYPE_UNSET (0) — DELIBERATELY DIFFERENT from LUMICE_FilterParam's zero-init
//     guard: for a color PREDICATE, UNSET means "match-all whole-crystal" (aligns with core's
//     RaypathColorRef default `NoneFilterParam{}`, whose wire form is "no `type` key"). The
//     UNSET-reject convention on LUMICE_FilterParam guards against silently defaulting a
//     physical filter to no-op; a color predicate has no such physical-safety risk, so
//     zero-init reasonably means "whole-crystal color tag on this placement".
//   LUMICE_FILTER_TYPE_{NONE, RAYPATH, ENTRY_EXIT, DIRECTION, CRYSTAL} — same field semantics
//     as the LUMICE_FilterParam arms; see there.
//   LUMICE_FILTER_TYPE_COMPLEX is REJECTED (Design 2 color predicates are single-atom).
typedef struct LUMICE_ColorPredicate_ {
  int type;  // LUMICE_FILTER_TYPE_* (UNSET=0 means match-all; COMPLEX rejected at commit)

  // Raypath arm
  int raypath[LUMICE_MAX_CONFIG_RAYPATH_LEN];
  int raypath_count;

  // EntryExit arm. -1 sentinels; ee_min_len semantics mirror LUMICE_FilterParam.
  int ee_entry;
  int ee_exit;
  int ee_min_len;
  int ee_max_len;

  // Direction arm (degrees)
  float dir_az;
  float dir_el;
  float dir_radii;

  // Crystal arm
  int crystal_id;
} LUMICE_ColorPredicate;

// One placement-scoped color ref = the atom `{layer, crystal_id, predicate}`. Fields carry
// the same identifiers used elsewhere in scene config (scattering layer index, crystal id).
typedef struct LUMICE_ColorClassRef_ {
  int layer;    // scattering layer index (0-based)
  int crystal;  // crystal id
  LUMICE_ColorPredicate predicate;
} LUMICE_ColorClassRef;

// Combine strategy over the match[] refs (mirrors core ColorClassCombine).
#define LUMICE_COLOR_COMBINE_ANY 0
#define LUMICE_COLOR_COMBINE_ALL 1

// One color class = an RGB color, a boolean combine over its refs, per-class display-time
// visibility. A class carries `match[]` refs (semantic bits, decides which rays contribute)
// and display-time appearance (color, visible, solo — mutable via LUMICE_SetRaypathColors
// without re-simulation). match[]/combine are STRUCTURAL: changing them re-simulates.
//
// WARNING (A4): visible/solo are plain 0/1 booleans; zero-initializing `LUMICE_ColorClass{}`
// lands visible=0 (INVISIBLE), which is the OPPOSITE of the core JSON default `true`.
// Callers must explicitly set visible=1 for the class to appear in composited output — the
// class is otherwise silently omitted from the compositor. This mirrors LUMICE_FILTER_TYPE_UNSET's
// "zero-init requires explicit follow-up" discipline; the GUI FillLumiceConfig equivalent for
// task-3 must not forget this.
typedef struct LUMICE_ColorClass_ {
  float color[3];                                            // linear RGB in [0, 1]
  int combine;                                               // LUMICE_COLOR_COMBINE_ANY / _ALL
  int visible;                                               // 0 = hidden, non-zero = visible (see WARNING above)
  int solo;                                                  // non-zero = restrict composite to solo'd classes
  LUMICE_ColorClassRef match[LUMICE_MAX_CONFIG_COLOR_REFS];  // predicate atoms
  int match_count;
} LUMICE_ColorClass;

// Composite modes for the display-time compositor (mirrors core CompositeMode / the JSON
// "mode" field: "dominant" | "additive" | "painter"). Default dominant matches the wire
// default; painter uses the class list's z-order (see LUMICE_SetRaypathColors).
#define LUMICE_COLOR_MODE_DOMINANT 0
#define LUMICE_COLOR_MODE_ADDITIVE 1
#define LUMICE_COLOR_MODE_PAINTER 2

// BREAKING (v4.3): norm_mode field removed; struct layout changed. Callers must recompile against this header.
typedef struct LUMICE_RenderParam_ {
  int id;
  int resolution_w;
  int resolution_h;
  float opacity;
  float intensity_factor;
  float overlap;  // Dual fisheye overlap zone |sky.z| threshold (sin value). 0 = no overlap.
} LUMICE_RenderParam;

// BREAKING (v4.4): added spectrum_entries[]/spectrum_count for custom discrete spectrum.
// Layout changed; callers must recompile against this header. spectrum_count > 0 selects the
// discrete-list path and overrides the spectrum string field (mirrors filters/filter_count).
// BREAKING (v4.6): added compositions[]/composition_count for complex (sum-of-products)
// filters. Layout changed; callers must recompile. A filters[] entry with type ==
// LUMICE_FILTER_TYPE_COMPLEX indexes into compositions[] via its composition_index.
// BREAKING (v4.7): added raypath_color[]/raypath_color_count/raypath_color_mode for
// per-raypath color classes (Design 2). Layout changed; callers must recompile.
// raypath_color_count == 0 → no color classes configured (mono-only, zero regression).
typedef struct LUMICE_Config_ {
  // Crystals
  LUMICE_CrystalParam crystals[LUMICE_MAX_CONFIG_CRYSTALS];
  int crystal_count;

  // Filters
  LUMICE_FilterParam filters[LUMICE_MAX_CONFIG_FILTERS];
  int filter_count;

  // Complex-filter compositions (referenced by filters[].composition_index for COMPLEX type).
  LUMICE_ComplexComposition compositions[LUMICE_MAX_CONFIG_COMPLEX];
  int composition_count;

  // Renderers
  LUMICE_RenderParam renderers[LUMICE_MAX_CONFIG_RENDERERS];
  int renderer_count;

  // Scene: light source
  float sun_altitude;
  float sun_azimuth;
  float sun_diameter;
  const char* spectrum;  // e.g. "D65", "D50", "A", "E"; ignored when spectrum_count > 0
  // Discrete custom spectrum. When spectrum_count > 0, the spectrum string is ignored and this
  // list is used instead. spectrum_count == 0 means "use the spectrum string".
  LUMICE_SpectrumEntry spectrum_entries[LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES];
  int spectrum_count;

  // Scene: simulation
  int infinite;             // 1=infinite rays, 0=finite
  LUMICE_RayCount ray_num;  // only used when infinite==0
  int max_hits;

  // Scene: scattering
  LUMICE_ScatterLayer scattering[LUMICE_MAX_CONFIG_SCATTER_LAYERS];
  int scatter_count;

  // Raypath color classes (Design 2, task-342.2). raypath_color_count == 0 disables the
  // color path entirely — mono LUMICE_GetRenderResults output is byte-identical to the
  // pre-v4.7 struct layout. Non-zero enables per-raypath color: LUMICE_GetCompositeResults
  // returns one composite RGB image per colored renderer.
  LUMICE_ColorClass raypath_color[LUMICE_MAX_CONFIG_COLOR_CLASSES];
  int raypath_color_count;
  int raypath_color_mode;  // LUMICE_COLOR_MODE_DOMINANT / _ADDITIVE / _PAINTER
} LUMICE_Config;

// Bound the LUMICE_Config growth: this struct is passed by value from callers that may put
// it on the stack (unit tests, GUI wrappers). Static assertion fails loudly if a future
// field addition explodes the size. Bound is a hard ceiling; keep it low enough that
// Windows' 1 MB default stack still leaves reasonable headroom.
#if defined(__cplusplus)
static_assert(sizeof(LUMICE_Config) <= 768u * 1024u,
              "LUMICE_Config exceeded its 768 KB ABI ceiling — either shrink a field or bump the ceiling deliberately");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(
    sizeof(LUMICE_Config) <= 768u * 1024u,
    "LUMICE_Config exceeded its 768 KB ABI ceiling — either shrink a field or bump the ceiling deliberately");
#endif

// out_reused: if non-NULL, set to 1 if consumers were reused (no buffer realloc), 0 if rebuilt.
LUMICE_ErrorCode LUMICE_CommitConfigStruct(LUMICE_Server* server, const LUMICE_Config* config, int* out_reused);

// =============== Raypath Color Display-Time Setter (task-342.2) ===============
// Display-time appearance of one color class (mutable without re-simulation): the RGB
// color, the visible/solo toggles. Structural fields (match[]/combine) live on
// LUMICE_ColorClass and require re-simulation to change (LUMICE_CommitConfig{,Struct}).
// WARNING (same footgun as LUMICE_ColorClass): visible/solo are applied verbatim, so a
// zero-initialized LUMICE_ColorClassDisplay{} has visible==0 (INVISIBLE). Callers MUST set
// visible=1 explicitly for every class they want shown, or the next composite hides them.
typedef struct LUMICE_ColorClassDisplay_ {
  float color[3];
  int visible;
  int solo;
} LUMICE_ColorClassDisplay;

// Update display-time appearance of the committed color classes WITHOUT restarting the
// simulation. Colors, visibility, solo, z-order, composite mode — none of these touch the
// accumulator, the epoch, or the consumer set. The compositor re-runs on the SAME
// already-accumulated per-class Y-lanes and produces new pixel output on the next
// LUMICE_GetCompositeResults call.
//
// classes[i] targets committed color class i (physical index in raypath_color[]; the
// server's active class table). class_count MUST equal the current raypath_color_count of
// the committed config, otherwise LUMICE_ERR_INVALID_CONFIG is returned — a count mismatch
// signals the caller changed member structure and must re-commit the config.
//
// z_order: OPTIONAL — pass NULL to leave existing z-order unchanged. When non-NULL, z_order
// MUST be a permutation of [0, class_count): z_order[i] is the NEW drawing rank of class i
// (the ranks are the integers 0..class_count-1 in some order — the natural output of a GUI
// drag-reorder). The compositor sorts ascending, so rank 0 (the LOWEST rank) draws first and
// therefore lands on top for painter mode / wins dominant ties (first-drawn wins). A z_order
// that is not a valid
// permutation (out-of-range or duplicate rank, e.g. {0,0,1}) returns LUMICE_ERR_INVALID_CONFIG
// and leaves all state unchanged (all-or-nothing).
//
// mode: composite mode (LUMICE_COLOR_MODE_DOMINANT / _ADDITIVE / _PAINTER). Values outside
// this range return LUMICE_ERR_INVALID_VALUE.
//
// Thread safety: display-time only; safe relative to OTHER display-time getters (Get*Results,
// LUMICE_GetSimLifecycle, etc.). NOT thread-safe with concurrent LUMICE_CommitConfig{,Struct}
// — the existing single-owner CommitConfig rule (doc/capi-lifecycle-architecture.md §4) still
// applies to this setter.
LUMICE_ErrorCode LUMICE_SetRaypathColors(LUMICE_Server* server, const LUMICE_ColorClassDisplay* classes,
                                         int class_count, const int* z_order, int mode);

// =============== Configuration Parsing (JSON -> LUMICE_Config) ===============
// Parse JSON into LUMICE_Config struct, enabling load-modify-commit workflows.
//
// Input format: accepts JSON in the format produced by LUMICE_CommitConfigStruct round-trip
// (i.e., the subset of the full config format that LUMICE_Config can represent).
// Specifically:
//   - crystal: height/face_distance as scalars/arrays, axis as {type, mean, std} objects
//   - filter: parse (JSON -> struct) supports all 6 types (none / raypath / entry_exit /
//     direction / crystal / complex), mirroring the LUMICE_ConfigToJson emit; unknown type
//     strings return LUMICE_ERR_INVALID_VALUE. Complex filters populate compositions[] and
//     set composition_index (resolved in a second pass; each composition term must reference
//     an existing simple filter, else LUMICE_ERR_INVALID_CONFIG). Parse does lossless field
//     mapping only; per-value semantic validation (e.g. entry_exit min_len >= 1) fires at
//     commit time in core.
//   - render: lens/view/visible/background fields are ignored; only id/resolution/opacity/
//     intensity_factor/overlap are parsed
//   - spectrum: string enumerations ("D65","D55","D50","D75","A","E") populate the spectrum
//     field; JSON arrays of {wavelength, weight} populate spectrum_entries[]/spectrum_count
//     (up to LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES; larger arrays return LUMICE_ERR_INVALID_CONFIG).
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

// Serialize a LUMICE_Config into its JSON string form — the inverse of
// LUMICE_ParseConfigString and the explicit serialization half of the typed-struct API
// (for save / round-trip / debugging; distinct from the commit path).
//
// Buffer contract (snprintf-style, no cross-ABI allocation): writes at most buf_size bytes
// into out_buf, always NUL-terminated when buf_size > 0. out_buf may be NULL (or buf_size 0)
// to query the length without writing. If out_len is non-NULL it receives the full JSON
// length EXCLUDING the NUL terminator; the output was truncated iff out_len >= buf_size.
// Returns LUMICE_ERR_NULL_ARG if config is NULL, LUMICE_ERR_INVALID_CONFIG if the config
// cannot be serialized (e.g. a filter with an unset/invalid type).
LUMICE_ErrorCode LUMICE_ConfigToJson(const LUMICE_Config* config, char* out_buf, size_t buf_size, size_t* out_len);

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

// Fill per-raypath composite results into out array (img_buffer == NULL sentinel
// when count < max_count). Reuses LUMICE_RenderResult: a composite is an sRGB uint8
// W*H*3 image, one per colored renderer. Empty (out[0] sentinel) when no
// `raypath_color` is configured — the mono LUMICE_GetRenderResults path is
// unaffected. Lifetime: img_buffer is read-only and server-owned. Unlike the mono
// LUMICE_GetRenderResults buffer (which aliases a per-consumer image buffer), the
// composite buffer is genuinely freed and rebuilt whenever a fresh snapshot is
// materialized — so it is only guaranteed valid until the NEXT LUMICE_Get*Results()
// call (render/composite/stats/xyz, any of which may trigger a snapshot) or
// LUMICE_CommitConfig(). Copy the pixels before the next Get*Results if you need them.
LUMICE_ErrorCode LUMICE_GetCompositeResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);

// Fill raw XYZ results into out array (xyz_buffer == NULL sentinel when count < max_count).
// Returns unconverted XYZ float data + intensity scalars for GPU-side conversion.
LUMICE_ErrorCode LUMICE_GetRawXyzResults(LUMICE_Server* server, LUMICE_RawXyzResult* out, int max_count);

// Fill stats results into out array (sim_ray_num == 0 sentinel when count < max_count).
// Note: triggers DoSnapshot internally (includes PostSnapshot XYZ→RGB conversion).
LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count);

// Get cached stats without triggering DoSnapshot/PostSnapshot.
// Returns the stats from the most recent snapshot (updated by LUMICE_GetRawXyzResults).
// Returns all-zero struct if no snapshot has been taken yet.
// NOTE vs LUMICE_GetSimRayCount: this reads a CACHE that only refreshes when
// something else takes a snapshot (e.g. a GUI poller's LUMICE_GetRawXyzResults);
// with no such driver (e.g. a headless benchmark) it can stay stale/zero. Use
// LUMICE_GetSimRayCount when you need a live sim_ray_num with no external trigger.
LUMICE_ErrorCode LUMICE_GetCachedStats(LUMICE_Server* server, LUMICE_StatsResult* out);

// Cheap O(1) live accumulated sim ray count — no snapshot, no render, no XYZ copy.
// For progress polling (e.g. the --benchmark drain loop) that needs sim_ray_num
// every iteration but not a rendered image. Unlike LUMICE_GetStatsResults, this
// does NOT trigger DoSnapshot/PostSnapshot; and unlike LUMICE_GetCachedStats it
// reads the running counter directly, so it needs no external snapshot driver to
// stay fresh. Writes 0 if no StatsConsumer (or none produced yet). (task-317)
LUMICE_ErrorCode LUMICE_GetSimRayCount(LUMICE_Server* server, LUMICE_RayCount* out);

// =============== State & Control ===============
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out);

// Read the explicit simulation lifecycle + current epoch (single-source truth).
// LUMICE_QueryServerState is a projection of this. After a synchronous commit,
// call this to read back the just-minted epoch (no commit-signature change).
LUMICE_ErrorCode LUMICE_GetSimLifecycle(LUMICE_Server* server, LUMICE_SimLifecycleResult* out);

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
  // Area-weighted unit-length face normals, lockstep with face_numbers_by_face /
  // face_vtx_offsets / face_vtx_counts: slot [i*3..i*3+2] is the unit normal of
  // face i for i in [0, face_count). Slots beyond face_count are unspecified.
  float face_normals[LUMICE_MAX_CRYSTAL_FACES * 3];
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
#define LUMICE_BACKEND_CUDA 2

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

// Query whether a trace backend is available on this machine at runtime.
//   backend = LUMICE_BACKEND_CPU   : always returns 1.
//   backend = LUMICE_BACKEND_METAL : returns 1 iff Apple build AND a Metal
//                                    device is present at runtime; 0 otherwise
//                                    (non-Apple, or Mac without Metal device).
//   other values                   : returns 0.
// Result is cached after the first call; safe to call from any thread and from
// per-frame GUI code.
// To add a new backend (e.g. CUDA): append LUMICE_BACKEND_CUDA above and add a
// matching branch here; CPU / Metal semantics are unchanged.
int LUMICE_IsBackendAvailable(int backend);

// Query whether a server built with `preferred_backend` would take the GPU
// single-engine route (worker_count=1) on this machine. Unlike
// LUMICE_IsBackendAvailable (which only reports device presence), this also honors
// the `LUMICE_TRACE_BACKEND` env override, which wins over `preferred_backend` — so
// e.g. `LUMICE_TRACE_BACKEND=cuda` with preferred_backend=CPU returns 1 (iff an
// eligible CUDA device exists). Same resolution the server uses to size worker_count.
// Intended for the CLI `--benchmark` dual-pass: the GPU route is single-engine, so
// its "single" (warmup) vs "multi" (steady) passes are NOT parallel — callers use
// this to collapse the GPU benchmark to one steady pass. Returns 1 (GPU route) or 0.
int LUMICE_WillUseGpuRoute(int preferred_backend);

#if !defined(_MSC_VER)
#pragma GCC visibility pop
#endif

#ifdef __cplusplus
}
#endif

#endif  // LUMICE_H_
