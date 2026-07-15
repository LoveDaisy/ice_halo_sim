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

// task-gui-feedback-affordances Step 7 (AC1): summary of how many color-
// classification predicates / symmetry groups the CORE dropped during the
// most recent commit. Used by the GUI DoRun path to surface a modal warning
// that coloring will be truncated (the filter / geometry / raypath tracing
// path is UNAFFECTED — only color assignment degrades).
//
// component_overflow_count is set SYNCHRONOUSLY inside CommitConfig (predicates
// past the 64-bit ComponentTable budget were assigned kNoBit).
// symmetry_group_overflow_count is RESERVED for a follow-up task — the CPU/
// Metal/CUDA `EnsureFilterBuffers` per-slot symmetry-group cap
// (kColorMaxGroupsPerSlot=4) fires asynchronously on the worker thread's
// first batch, so surfacing it requires poll infrastructure not yet wired.
// Currently always 0; the field is kept in the ABI struct so future clients
// do not need to re-widen it later.
typedef struct LUMICE_ColorOverflowInfo_ {
  int component_overflow_count;
  int symmetry_group_overflow_count;
} LUMICE_ColorOverflowInfo;

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
  // task-345.3: composite-only auto-EV anchor. Populated by
  // LUMICE_GetCompositeResults / LUMICE_GetRawXyzAndCompositeResults's composite_out —
  // MEANINGFUL ONLY on the composite path. LUMICE_GetRenderResults (mono/full-spectrum)
  // leaves this at 0 and consumers must ignore it there. Composite path: P99 over the
  // union of NON-ZERO UNEXPOSED (raw lane) Y values across every participating color
  // class (the anchor the GUI's auto-EV feeds into ComputeEvAuto for composite display).
  // 0 on the composite path means no participating class carried any positive Y this
  // snapshot (all-black composite / all classes hidden). See doc/ev-pipeline-architecture.md
  // §2.4 for why this field is composite-only.
  float composite_p99_y;
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
// LUMICE_MAX_CONFIG_COMPLEX remains an ABI ceiling baked into the LUMICE_Config layout
// (bounds the inline compositions[] array). LUMICE_MAX_CONFIG_CLAUSES / _TERMS are no
// longer inline-array dimensions (v4.9, task-host-abi-cpu-caps): clause/term storage is
// heap-allocated via LUMICE_CompositionSetClauses, and these two constants are pure
// sanity ceilings that cap a single filter's OR/AND fan-out (defensive DoS bound against
// malformed .lmc/JSON input). Widen (breaking bump) if needed.
#define LUMICE_MAX_CONFIG_COMPLEX 32    // max complex-filter composition records per config
#define LUMICE_MAX_CONFIG_CLAUSES 4096  // sanity ceiling: max OR clauses per complex filter
#define LUMICE_MAX_CONFIG_TERMS 64      // sanity ceiling: max AND terms per clause
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
// is an AND over its terms; each term is the ID of a SIMPLE filter in the same config's
// filters[] pool (referenced by id — reorder-robust — not by array index; a term may never
// reference another complex filter, matching core config semantics).
// INVARIANT: compositions[] is rebuilt wholesale together with its host LUMICE_Config;
// records are not individually reordered (composition_index is a pool index valid only
// within one config snapshot). Consumers (e.g. GUI) must respect this whole-rebuild model.
//
// BREAKING (v4.9, task-host-abi-cpu-caps): storage layout changed from inline
// `clauses[16][8]` / `term_counts[16]` to a pair of owned heap pointers with a
// clause-major flat encoding. Rationale: at 16×8 inline the ceiling was too low for
// real "OR of several hundred raypaths" use cases; naively widening the inline array
// (e.g. to 4096×64) would push LUMICE_Config well past its 160 KB stack budget and
// re-run the raypath_color[64] stack-overflow that task-344 already fixed on the
// color path. Callers now populate one record via LUMICE_CompositionSetClauses and
// must release via LUMICE_CompositionReleaseClauses (or via LUMICE_ConfigReleaseCompositions
// / the C++ RAII guard lumice::ConfigOwningGuard). Do NOT copy this struct by value —
// term_ids/term_counts are owning pointers; aliasing copies would double-free on double
// Release (enforced by scripts/check_policies.py's `no-config-by-value-copy` gate).
//
// Fields:
//   term_ids     — owned; flat array of simple-filter IDs, clause-major
//                  (clause 0's terms, then clause 1's terms, …). Length = sum(term_counts[0..clause_count)).
//   term_counts  — owned; term_counts[c] is the AND-term count of clause c. Length = clause_count.
//   clause_count — number of OR clauses in this composition. 0 is a valid "OR of nothing" state
//                  (both pointers nullptr); >0 requires both pointers non-null.
//
// Use LUMICE_CompositionClauseTerms to iterate a specific clause without recomputing the offset.
typedef struct LUMICE_ComplexComposition_ {
  int* term_ids;     // owned; flat AND-term simple-filter IDs, clause-major (see block comment)
  int* term_counts;  // owned; term_counts[c] = clause c's AND-term count; length == clause_count
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
// (raypath / entry_exit / direction / crystal / none) — no id, action, composition, complex.
// Per-ref symmetry (P/B/D bitmask) is carried as a common field on the predicate (task-356.2,
// v4.9): matching semantics mirror the physical filter's symmetry (both feed the same
// Crystal::ReduceRaypath expansion on the core side).

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
//
// BREAKING (v4.9): added `symmetry` field to LUMICE_ColorPredicate. Layout changed; callers
// must recompile against this header.
typedef struct LUMICE_ColorPredicate_ {
  // Symmetry is a common field for ALL predicate arms (mirrors LUMICE_FilterParam.symmetry /
  // core RaypathColorRef.symmetry_), not raypath-only. Bitmask: 1=P, 2=B, 4=D; 0=kSymNone
  // (literal single-orientation match — default, wire-omitted; see RaypathColorRef::to_json).
  int symmetry;
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
// "mode" field: "dominant" | "additive" | "painter"). Default painter matches the wire
// default (doc §4.8); painter uses the class list's
// z-order (see LUMICE_SetRaypathColors).
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
// BREAKING (v4.8): raypath_color[] is no longer an inline array — it is now a heap-allocated
// `LUMICE_ColorClass*` owned by this struct. The inline `LUMICE_ColorClass[64]` layout was
// ~354 KB and pushed `LUMICE_Config` to 467 KB, which overflowed the ImGui test engine's
// ~512 KB thread stack when two configs coexisted in one function. Callers now allocate via
// `LUMICE_ConfigCreateColorClasses` and must release via `LUMICE_ConfigReleaseColorClasses`
// (or via a C++ RAII guard such as `lumice::ConfigOwningGuard`; see
// `src/include/lumice_config_scope.hpp`). Layout changed; callers must recompile.
// BREAKING (v4.9, task-host-abi-cpu-caps): LUMICE_ComplexComposition's clause/term storage
// moved off the inline layout onto a pair of owning heap pointers (term_ids / term_counts).
// The outer `compositions[LUMICE_MAX_CONFIG_COMPLEX]` inline array is retained; only the
// intra-record clause/term storage was heap-ified — which SHRINKS sizeof(LUMICE_Config)
// (each record drops from ~580 B inline to ~24 B ptr+count). Callers populate a record via
// LUMICE_CompositionSetClauses and release per-record via LUMICE_CompositionReleaseClauses
// or config-wide via LUMICE_ConfigReleaseCompositions; the C++ RAII guard
// `lumice::ConfigOwningGuard` releases both raypath_color AND compositions in one shot.
//
// Do not copy this struct by value (assignment, by-value parameter, or storing it in a
// container) — `raypath_color` AND each `compositions[i].term_ids/term_counts` are owning
// heap pointers; copies alias the same allocations and cause double-free on double Release.
// Pass `LUMICE_Config*` or `const LUMICE_Config&` instead. See
// LUMICE_ConfigCreateColorClasses / LUMICE_ConfigReleaseColorClasses,
// LUMICE_CompositionSetClauses / LUMICE_ConfigReleaseCompositions.
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

  // Raypath color classes (Design 2, task-342.2; BREAKING v4.8: heap-allocated pointer).
  // raypath_color_count == 0 disables the color path entirely — mono
  // LUMICE_GetRenderResults output is byte-identical to the pre-v4.7 struct layout.
  // Non-zero enables per-raypath color: LUMICE_GetCompositeResults returns one composite
  // RGB image per colored renderer.
  //
  // Ownership: raypath_color is owned by this LUMICE_Config and must be allocated /
  // released only through LUMICE_ConfigCreateColorClasses / LUMICE_ConfigReleaseColorClasses.
  // Zero-initializing this struct (e.g. `LUMICE_Config cfg{};`) leaves raypath_color ==
  // nullptr / raypath_color_count == 0, which is the "no color classes" state and is safe
  // to release (Release is idempotent / null-safe).
  LUMICE_ColorClass* raypath_color;
  int raypath_color_count;
  int raypath_color_mode;  // LUMICE_COLOR_MODE_DOMINANT / _ADDITIVE / _PAINTER
} LUMICE_Config;

// Bound the LUMICE_Config growth: this struct is passed by value from callers that may put
// it on the stack (unit tests, GUI wrappers). Static assertion fails loudly if a future
// field addition explodes the size. Bound is a hard ceiling; keep it low enough that
// Windows' 1 MB default stack still leaves reasonable headroom.
//
// v4.8 (task-344): tightened from 768 KB to 160 KB after raypath_color[64] was moved off
// the inline layout onto the heap. Measured sizeof(LUMICE_Config) drops from 467 KB to
// ~113 KB (plan §3 point 5); the 160 KB ceiling keeps ~40 % headroom for future fields
// while still catching accidental re-inlining or unbounded array additions early.
// v4.9 (task-host-abi-cpu-caps): LUMICE_ComplexComposition's inline `clauses[16][8]` /
// `term_counts[16]` (~580 B/record × 32 records ≈ 18.6 KB) moved off inline onto per-record
// heap pointers (24 B/record × 32 records = 768 B); measured sizeof(LUMICE_Config) drops from
// ~113 KB to ~96 KB (~98 280 B on this platform), so the 160 KB ceiling still holds with even
// more headroom than pre-v4.9.
#if defined(__cplusplus)
static_assert(sizeof(LUMICE_Config) <= 160u * 1024u,
              "LUMICE_Config exceeded its 160 KB ABI ceiling — either shrink a field or bump the ceiling deliberately");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(
    sizeof(LUMICE_Config) <= 160u * 1024u,
    "LUMICE_Config exceeded its 160 KB ABI ceiling — either shrink a field or bump the ceiling deliberately");
#endif

// out_reused: if non-NULL, set to 1 if consumers were reused (no buffer realloc), 0 if rebuilt.
LUMICE_ErrorCode LUMICE_CommitConfigStruct(LUMICE_Server* server, const LUMICE_Config* config, int* out_reused);

// =============== Raypath Color Classes lifecycle (task-344, BREAKING v4.8) ===============
// Allocate `count` zero-initialized LUMICE_ColorClass entries and attach them to `cfg`.
// On success, `cfg->raypath_color` points to the new array, `cfg->raypath_color_count` is
// set to `count`, and the returned pointer aliases `cfg->raypath_color` for the caller to
// fill.
//
// Semantics:
//   - `cfg == nullptr` → returns nullptr (no side effect).
//   - `count < 0` or `count > LUMICE_MAX_CONFIG_COLOR_CLASSES` → returns nullptr; `cfg`
//     is left untouched.
//   - `count == 0` → releases any existing allocation, sets pointer to nullptr and
//     count to 0, returns nullptr. This is the "no color classes" state (mono-only).
//   - `count > 0` → this call is create-or-replace: if `cfg->raypath_color` was already
//     non-null (e.g. a prior Create call), it is released first, then a fresh
//     zero-initialized array of `count` entries is allocated. Safe to call multiple times
//     on the same `cfg` with different counts.
//   - Allocator is calloc/free; the returned pointer must eventually be released via
//     LUMICE_ConfigReleaseColorClasses (or its C++ RAII wrapper).
//
// Ownership contract: whether the caller invokes Create directly OR indirectly triggers
// allocation via LUMICE_ParseConfigString / LUMICE_ParseConfigFile (both allocate
// implicitly based on the parsed JSON), the caller MUST call Release once done with
// `cfg`. In C++ code, prefer the `lumice::ConfigOwningGuard` RAII wrapper in
// `src/include/lumice_config_scope.hpp` (also releases compositions).
LUMICE_ColorClass* LUMICE_ConfigCreateColorClasses(LUMICE_Config* cfg, int count);

// Release the raypath_color allocation owned by `cfg`, if any. Idempotent and null-safe:
//   - `cfg == nullptr` → no-op.
//   - `cfg->raypath_color == nullptr` → clears count only, no free.
//   - Otherwise → free(cfg->raypath_color), then nullptr / count=0.
// Safe to call multiple times; safe to call on a freshly zero-initialized LUMICE_Config.
void LUMICE_ConfigReleaseColorClasses(LUMICE_Config* cfg);

// =============== Complex-Composition storage lifecycle (task-host-abi-cpu-caps, BREAKING v4.9) ===============
// Populate `comp` in one shot from an application-owned (clause_count, term_counts[], term_ids[])
// triple. This is the ONLY supported writer for the composition storage — direct field writes
// (previously legal against the old inline `clauses[16][8]`) are no longer defined.
//
// Unlike LUMICE_ConfigCreateColorClasses's two-phase allocate-then-fill-in-place pattern, this
// API is a one-shot value write (Set): every production caller already holds the complete
// (clause_count, term_counts, term_ids) triple on the stack before calling, so there is no
// caller-visible "allocated but not yet populated" intermediate state to expose. Reach for this
// same shape (Set over Create) for a future owning field only if callers likewise assemble the
// full value before writing it in.
//
// Semantics:
//   - `comp == nullptr` → returns LUMICE_ERR_NULL_ARG.
//   - `clause_count < 0` or `clause_count > LUMICE_MAX_CONFIG_CLAUSES` → returns
//     LUMICE_ERR_INVALID_CONFIG, `comp` left untouched.
//   - `clause_count > 0 && term_counts == nullptr` → LUMICE_ERR_NULL_ARG.
//   - Any `term_counts[i] < 0` or `term_counts[i] > LUMICE_MAX_CONFIG_TERMS` →
//     LUMICE_ERR_INVALID_CONFIG (full clause-count validation runs before any allocation;
//     `comp` untouched on rejection).
//   - `term_ids == nullptr` is only rejected (LUMICE_ERR_NULL_ARG) once `sum(term_counts[0..
//     clause_count))` (total term count) is computed and found > 0 — i.e. `term_ids` may
//     legitimately be null when every clause has 0 terms; see LUMICE_CompositionClauseTerms's
//     doc comment for the resulting storage state.
//   - `clause_count == 0` → release any existing allocation and land in the "OR of nothing"
//     state (term_ids/term_counts nullptr, clause_count 0). term_counts/term_ids inputs
//     are ignored in this branch.
//   - `clause_count > 0` → this call is CREATE-OR-REPLACE: if `comp` already held a prior
//     allocation, it is released before the new one is allocated. On OOM, `comp` falls back
//     to the "OR of nothing" state (fully-cleared, safe to Release again) and the function
//     returns LUMICE_ERR_INVALID_CONFIG — mirrors LUMICE_ConfigCreateColorClasses's OOM policy.
//   - `term_ids` must be a clause-major flat array of length `sum(term_counts[0..clause_count))`.
//     Elements are simple-filter IDs; reference-integrity checks (each id resolves to an
//     existing SIMPLE filter in the enclosing LUMICE_Config) are the CALLER's responsibility
//     (the c_api / GUI writers already run this pre-check; see LUMICE_CommitConfigStruct).
//   - Allocator is calloc/free; callers must eventually release via
//     LUMICE_CompositionReleaseClauses (or its config-wide sibling / RAII guard).
LUMICE_ErrorCode LUMICE_CompositionSetClauses(LUMICE_ComplexComposition* comp, int clause_count, const int* term_counts,
                                              const int* term_ids);

// Release the term_ids / term_counts allocations owned by `comp`, if any. Idempotent and null-safe:
//   - `comp == nullptr` → no-op.
//   - Otherwise → free both pointers (either may already be null), then leave `comp` in the
//     "OR of nothing" state (both nullptr, clause_count=0).
void LUMICE_CompositionReleaseClauses(LUMICE_ComplexComposition* comp);

// Release the composition storage owned by every record in `cfg->compositions[0..composition_count)`.
// Does NOT touch `composition_count` or the inline compositions[] array itself — those remain
// part of the LUMICE_Config's own layout. Idempotent and null-safe:
//   - `cfg == nullptr` → no-op.
//   - Otherwise → iterates and calls LUMICE_CompositionReleaseClauses on each record.
// Callers who reuse a `LUMICE_Config` across successive Parse / Fill invocations must call
// this before the memset that clears the struct (otherwise the record pointers leak); the
// C++ RAII guard `lumice::ConfigOwningGuard` calls it on scope exit alongside
// LUMICE_ConfigReleaseColorClasses.
void LUMICE_ConfigReleaseCompositions(LUMICE_Config* cfg);

// Read-only convenience accessor: return a pointer to clause `clause_index`'s first AND-term
// inside `comp->term_ids` (i.e. the address of `term_ids[prefix_sum(term_counts[0..clause_index))]`)
// and write `term_counts[clause_index]` into `*out_term_count`. Encapsulates the prefix-sum
// offset math so callers (ConfigToJson emit, tests, GUI diagnostics) don't recompute it.
//
// Semantics:
//   - `comp == nullptr`                                       → returns nullptr, `*out_term_count`
//                                                                 set to 0 if non-null.
//   - `clause_index < 0` or `clause_index >= comp->clause_count` → returns nullptr, `*out_term_count`
//                                                                 set to 0 if non-null.
//   - Otherwise → `*out_term_count` (if non-null) is always set to `term_counts[clause_index]`.
//     The returned pointer is nullptr whenever `comp->term_ids` itself is null — which is the
//     legitimate state when every clause in the composition has 0 terms (LUMICE_CompositionSetClauses
//     skips that allocation entirely in that case); this applies even to a 0-term clause, so callers
//     must not unconditionally dereference a non-null out_term_count as "safe to iterate".
//     When `comp->term_ids` is non-null, the returned pointer (including for a 0-term clause) is a
//     valid address into that buffer for the (possibly empty) slice.
const int* LUMICE_CompositionClauseTerms(const LUMICE_ComplexComposition* comp, int clause_index, int* out_term_count);

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

// task-345.3: display-time EV for the composite (raypath_color) path only.
// `ev_total` is applied as `2^ev_total` inside the composite bake — a single scalar shared
// by every participating color class (per-class renormalization stays structurally excluded;
// the mono / non-composite path is unaffected).
//
// No accumulator reset / no epoch bump / no sim restart — the setter just flips the internal
// snapshot_dirty_ flag, so the next LUMICE_GetCompositeResults() (or GetRawXyzAndComposite)
// rebuilds the composite with the new EV. Callers that already keep the poller running (a
// live sim, or a display-time refresh triggered by other setters) get the new brightness on
// the next poll; callers that stopped the poller must wake it (mirrors the
// LUMICE_SetRaypathColors + poller-wake pattern used by the GUI).
//
// Thread safety: display-time only; safe relative to other display-time getters (Get*Results,
// LUMICE_GetSimLifecycle, LUMICE_SetRaypathColors, etc.). NOT thread-safe with concurrent
// LUMICE_CommitConfig{,Struct} (same single-owner rule as the rest of the display-time surface).
LUMICE_ErrorCode LUMICE_SetCompositeExposure(LUMICE_Server* server, float ev_total);

// Per-color-class empty-arc detector (task-342.3 AC4). For each committed color class, reports
// whether the class has any non-zero pixel in its snapshot Y-lane on any active RenderConsumer
// — i.e. whether it has captured any rays yet. Intended for GUI empty-arc warnings when a
// physical filter has silently blocked all rays that would have matched the class predicate.
//
// out_flags is a caller-owned buffer of length class_count. On success, out_flags[i] = 1 iff
// class i has signal, 0 otherwise. class_count MUST equal the current raypath_color_count of
// the committed config, otherwise LUMICE_ERR_INVALID_CONFIG. class_count == 0 is a valid no-op
// (returns LUMICE_OK; out_flags is not touched).
//
// Reads the frozen snapshot state (no DoSnapshot trigger); callers relying on freshness should
// query LUMICE_GetCompositeResults / LUMICE_GetRawXyzResults first. O(W*H * class_count *
// consumers) scan; intended for infrequent polls (commit-debounce cadence, ~1 Hz), not per
// render frame.
LUMICE_ErrorCode LUMICE_GetColorClassSignal(LUMICE_Server* server, int* out_flags, int class_count);

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

// task-345.2: atomic combined getter — fills raw XYZ + composite results from a
// SINGLE server-side snapshot. Composite results are guaranteed to belong to
// xyz_out[0].snapshot_generation by construction (structural, not
// probabilistic). Prefer this over calling LUMICE_GetRawXyzResults() +
// LUMICE_GetCompositeResults() separately when the caller needs both in one
// coherent view (e.g. the GUI poller under an active simulation, where
// concurrent batch churn between two independent calls otherwise pairs xyz
// with a newer-generation composite).
//
// Sentinels: xyz_out[xyz_count].xyz_buffer == NULL when xyz_count < xyz_max_count;
// composite_out[composite_count].img_buffer == NULL when composite_count < composite_max_count.
// Composite is empty (composite_out[0] sentinel) when no `raypath_color` is configured.
//
// Lifetime: same as the individual getters — pointers stay valid until the
// NEXT LUMICE_Get*Results() call (which may trigger a new snapshot) or
// LUMICE_CommitConfig(). Copy the pixels before the next Get*Results if you
// need them.
LUMICE_ErrorCode LUMICE_GetRawXyzAndCompositeResults(LUMICE_Server* server, LUMICE_RawXyzResult* xyz_out,
                                                     int xyz_max_count, LUMICE_RenderResult* composite_out,
                                                     int composite_max_count);

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

// task-gui-feedback-affordances Step 7 (AC1): synchronous readback of the
// most recent commit's color-classification overflow counters (see the
// LUMICE_ColorOverflowInfo doc block above). The GUI DoRun path calls this
// after CommitConfigStruct returns OK; a non-zero component_overflow_count
// triggers a modal "coloring degraded" prompt. LUMICE_OK on success;
// LUMICE_ERR_NULL_ARG if server or out is null.
LUMICE_ErrorCode LUMICE_GetColorOverflowInfo(LUMICE_Server* server, LUMICE_ColorOverflowInfo* out);

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
