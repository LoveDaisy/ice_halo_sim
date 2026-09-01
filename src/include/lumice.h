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
// ABI version, encoded as major*100 + minor (v4.10 -> 410). Before this macro existed, the
// v4.3~v4.9 BREAKING bumps lived only in prose comments, so a caller linking against a header
// newer/older than the .so it loads had NO compile-time guard — a layout mismatch was silent UB.
// Callers can now pin the ABI they were built against, e.g.:
//   static_assert(LUMICE_API_VERSION >= 412, "Lumice header too old for this integration");
// Bump on every BREAKING change to the public symbol set / struct layout.
//
// BREAKING (v4.12): the wide `LUMICE_Config` value struct and everything that only existed to
// feed it are REMOVED. Gone: LUMICE_Config; the three commit entry points
// LUMICE_CommitConfig / LUMICE_CommitConfigFromFile / LUMICE_CommitConfigStruct;
// LUMICE_ParseConfigString / LUMICE_ParseConfigFile / LUMICE_ConfigToJson;
// LUMICE_ConfigCreateColorClasses / LUMICE_ConfigReleaseColorClasses /
// LUMICE_ConfigReleaseCompositions; and the C++ RAII header src/include/lumice_config_scope.hpp.
// LUMICE_Scene (added v4.11-era, see the "Scene (opaque handle)" section) is now the ONLY way to
// describe and commit a configuration: build with SceneCreate + Add*/Set* or parse with
// LUMICE_SceneFromJson / _FromJsonFile, then commit with LUMICE_CommitScene. No shim, no alias —
// callers of the removed symbols fail to compile, which is the intended migration signal.
// The LUMICE_MAX_CONFIG_* ceilings survive the struct: they are now the Scene's Add*/Set*
// per-kind soft caps, not the dimensions of an inline array.
//
// BREAKING (v4.14): LUMICE_StatsResult gains a fourth field, `orientation_num`
// — the count of distinct crystal ORIENTATIONS sampled, previously invisible
// (only the geometry count was reported, and on the commonest halo setup, a
// fixed shape under a random axis, that count is 1 no matter how richly the
// orientation was sampled). This is an APPEND, not a removal or a reorder, so
// existing field offsets are unchanged; but sizeof(LUMICE_StatsResult) grows
// from 24 to 32, so a caller that was NOT recompiled and passes an array of the
// old struct to LUMICE_FrameGetStats will have it written past the end.
// Recompile against this header.
//
// BREAKING (v4.15): the six server-taking result getters are REMOVED and replaced by the
// LUMICE_ResultFrame handle. Gone: LUMICE_GetRenderResults / LUMICE_GetCompositeResults /
// LUMICE_GetRawXyzResults / LUMICE_GetRawXyzAndCompositeResults / LUMICE_GetStatsResults /
// LUMICE_GetCachedStats. Read results by acquiring a frame — LUMICE_AcquireResultFrame, then
// LUMICE_FrameGetRender / _FrameGetComposite / _FrameGetRawXyz / _FrameGetStats, then
// LUMICE_ReleaseResultFrame. The buffers a frame hands out stay valid until THAT frame is
// released, instead of until the next getter call on the server; that is the whole point of the
// change — the old contract let one caller's read invalidate another's still-in-use pixels.
// Two old contracts disappear with their functions, both now structural: the combined
// xyz+composite getter existed to guarantee one generation, which any two reads off one frame
// have by construction; and the cached-stats getter's "may be stale, never triggers a snapshot"
// mode is gone — a frame carries the stats of the snapshot it is.
//
// BREAKING (v4.16): LUMICE_RenderParam loses its `opacity` field; struct layout changed.
// See the struct's own BREAKING note for why it went rather than gained an implementation.
// Recompile against this header.
// BREAKING (v4.16): LUMICE_RenderParam gains a trailing field, `ev_mode`
// (LUMICE_EV_MODE_RELATIVE / _ABSOLUTE), selecting which anchor the exposure scale is measured
// against. This is an APPEND, so every existing field keeps its offset; but sizeof() grows, so a
// caller that was NOT recompiled hands the API a shorter struct and the new field is read past
// the end of it. Recompile against this header.
// Note the accompanying DEFAULT change, which is a behavior break independent of the ABI one: a
// config with no "ev_mode" key now renders RELATIVE (anchored to the frame's own P99, i.e. what
// the GUI displays), where the v4.15-era CLI was unconditionally absolute. RELATIVE == 0 keeps
// that default reachable from a zero-initialized struct.
#define LUMICE_API_VERSION 416
#define LUMICE_MAX_RENDER_RESULTS 16
#define LUMICE_MAX_STATS_RESULTS 1

// =============== Opaque Types ===============
typedef struct LUMICE_Server_ LUMICE_Server;
// Opaque, incrementally-built scene container. See the "Scene (opaque handle)" section further
// down for its lifecycle + Add*/Set* build API. Declared here alongside the other opaque handle
// types; the build functions live below the leaf POD structs they consume.
typedef struct LUMICE_Scene_ LUMICE_Scene;
// Opaque, immutable, reference-counted snapshot of ALL results (mono render, composite,
// raw XYZ, stats) as of one server-side snapshot. Acquiring one gives the caller a real
// share of the data's lifetime: every buffer reachable through it stays valid until the
// caller releases the frame, no matter what the server does meanwhile. See the "Results"
// section below for the acquire/release + read functions.
typedef struct LUMICE_ResultFrame_ LUMICE_ResultFrame;

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

// {drained_epoch, current_epoch} snapshot of the CONSUMER-side drain contract.
// The current epoch's data is fully drained iff drained_epoch == current_epoch.
//
// Why this is not folded into the lifecycle above: LUMICE_LIFECYCLE_COMPLETED /
// LUMICE_SERVER_IDLE are derived from producer-side predicates only (no
// simulator busy, no pending scenes, scene generation done). None of them asks
// whether the consumer thread has finished draining its queue, so the
// accumulated statistics (LUMICE_FrameGetStats) can still be a PARTIAL total at
// the instant the server first reports IDLE — measured as orientation_num 19616
// vs 20000, a whole number of dispatch grains short. A reader that needs final
// totals must wait for THIS signal, not for IDLE.
//   drained_epoch:  highest epoch whose data the consumer has fully drained.
//                   Monotonic; 0 before any epoch has drained.
//   current_epoch:  == LUMICE_SimLifecycleResult.epoch, sampled in the same call
//                   so the two are compared without a second round trip.
// An infinite run never drains (production never ends), which is the same shape
// as its lifecycle never reaching COMPLETED. Stopping a server does NOT publish
// a drain: LUMICE_StopServer discards whatever is still queued, so "stopped" is
// deliberately distinguishable from "drained".
typedef struct LUMICE_DrainResult_ {
  unsigned long long drained_epoch;
  unsigned long long current_epoch;
} LUMICE_DrainResult;

// Summary of how many raypath-color assignments the CORE dropped for the most
// recent committed config, because some capacity was exceeded. Used by the GUI
// to surface a modal warning that coloring will be truncated (the filter /
// geometry / raypath tracing path is UNAFFECTED — only color assignment
// degrades). All counts are per committed config, not cumulative across runs.
//
// Two surfacing timings, by field:
//   component_overflow_count is set SYNCHRONOUSLY inside CommitConfig (color
//     predicates past the 64-bit ComponentTable budget were assigned kNoBit).
//     Backend-independent (host-side gate table). Read once in the DoRun path.
//   The remaining three are GPU-ONLY device buffer-layout caps that fire on the
//     worker thread's FIRST batch (asynchronous), so the GUI must POLL them each
//     tick rather than read them once at DoRun. Populated via the server's
//     ConsumeData path; the CPU backend has no such caps and reports 0:
//       symmetry_group_overflow_count — kColorMaxGroupsPerSlot (per gate slot)
//       or_summand_overflow_count     — kDeviceFilterMaxOrClauses (per color group)
//       color_class_overflow_count    — kMaxColorClassesDevice (per session)
// See task-color-degrade-gui-surfacing.
typedef struct LUMICE_ColorOverflowInfo_ {
  int component_overflow_count;
  int symmetry_group_overflow_count;
  int or_summand_overflow_count;
  int color_class_overflow_count;
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
  const unsigned char* img_buffer;  // Read-only view into the LUMICE_ResultFrame it was obtained
                                    // from. Valid until that frame is released with
                                    // LUMICE_ReleaseResultFrame().
                                    // Sentinel: img_buffer == NULL
  // Composite-only auto-EV anchor. Populated by LUMICE_FrameGetComposite —
  // MEANINGFUL ONLY on the composite path. LUMICE_FrameGetRender (mono/full-spectrum)
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
  const float* xyz_buffer;   // Read-only XYZ float data, 3 floats per pixel. A view into the
                             // LUMICE_ResultFrame it was obtained from; valid until that frame
                             // is released with LUMICE_ReleaseResultFrame().
                             // Sentinel: xyz_buffer == NULL
  float snapshot_intensity;  // Per-pixel landed intensity (landed_ray_weights / (kNormScale * total_pixels))
  float intensity_factor;    // Per-renderer intensity factor (2^EV)
  int has_valid_data;        // Non-zero once simulation has produced data (reset on CommitConfig/Stop)
  unsigned long long snapshot_generation;  // Increments on each new snapshot; compare to detect data changes
  int effective_pixels;                    // Non-zero pixel count (for stats display)
  // Total spectral energy the light source EMITTED into this snapshot: the sum over every
  // simulated batch of (per-ray emission weight x rays emitted). Raw total, NOT divided by
  // kNormScale * total_pixels the way snapshot_intensity above is.
  //
  // Distinct from snapshot_intensity in what it measures, not just in scaling: this counts
  // what went IN, that counts what came out and landed on a pixel. They differ by everything
  // that removes a ray -- filters, absorption, rays that miss the lens. This is the quantity
  // the renderer normalizes by, which is what makes its output scale absolute; a consumer can
  // reproduce that scale as
  //     scale = intensity_factor * kNormScale * total_pixels / emitted_energy
  // and two scenes normalized this way are directly comparable in brightness.
  //
  // Occupies alignment padding that already existed before `epoch`, so sizeof(LUMICE_RawXyzResult)
  // is unchanged and `epoch` keeps its offset.
  float emitted_energy;
  unsigned long long epoch;  // Lifecycle epoch at snapshot time (committed_epoch_); 1.5 display keying
} LUMICE_RawXyzResult;

typedef struct LUMICE_StatsResult_ {
  LUMICE_RayCount ray_seg_num;
  LUMICE_RayCount sim_ray_num;
  LUMICE_RayCount crystal_num;  // Distinct crystal GEOMETRIES sampled this run.
  // Distinct crystal ORIENTATIONS sampled this run. A separate quantity, not a
  // rescaling of crystal_num: the shape and the axis of a crystal setting are
  // independent, and the commonest halo configuration — a fixed shape under a
  // random axis — yields one geometry and a great many orientations. Expect this
  // to be far larger than crystal_num: orientation is redrawn per ray with no
  // reuse, while geometry is reused across a batch of rays.
  //
  // NOT comparable across backends, and not usable as a parity assertion: the
  // CPU and GPU routes sample at different densities and that difference is
  // precisely what the number exposes.
  LUMICE_RayCount orientation_num;
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

// =============== Configuration bounds ===============
// Per-kind soft capacity ceilings enforced by the Scene build API (LUMICE_SceneAdd* /
// LUMICE_SceneSet*) and by the JSON readers behind LUMICE_SceneFromJson / _FromJsonFile.
// They are pure validation bounds — nothing is dimensioned by them any more (the wide
// LUMICE_Config value struct whose inline arrays they used to size was removed in v4.12).
// Cross-references between items use integer IDs (crystal_id, filter_id) assigned by the
// Scene's Add* calls and resolved internally by Core.

#define LUMICE_MAX_CONFIG_CRYSTALS 256
#define LUMICE_MAX_CONFIG_FILTERS 256
#define LUMICE_MAX_CONFIG_RENDERERS 4
#define LUMICE_MAX_CONFIG_SCATTER_LAYERS 8
#define LUMICE_MAX_CONFIG_SCATTER_ENTRIES 256
#define LUMICE_MAX_CONFIG_RAYPATH_LEN 32
// Discrete-spectrum entry cap. Mirrors core wl_pool.hpp::kWlPoolSizeMax (255).
#define LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES 255
// Complex (sum-of-products) filter composition bounds. See LUMICE_ComplexComposition.
// All three are sanity ceilings against malformed .lmc/JSON input: LUMICE_MAX_CONFIG_COMPLEX
// caps complex-filter records per scene, LUMICE_MAX_CONFIG_CLAUSES / _TERMS cap a single
// filter's OR/AND fan-out. Clause/term storage is heap-allocated via
// LUMICE_CompositionSetClauses (v4.9). Widen (breaking bump) if needed.
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
// Per-renderer grid-line ceiling (central_grid[] / elevation_grid[] inline arrays in
// LUMICE_RenderParam). Same "widen (breaking bump)" rule as the constants above. 64 matches the
// order of magnitude of the other sanity ceilings; the shipped corpus peaks at 1 central line.
#define LUMICE_MAX_CONFIG_GRID_LINES 64

// BREAKING (v4.10): LUMICE_AxisDist renamed+widened to
// LUMICE_Distribution and now serves ANY randomizable scalar (axis angles AND crystal shape
// quantities), mirroring core's single `Distribution` type (src/core/math.hpp). The distribution
// type constants were renamed LUMICE_AXIS_DIST_* -> LUMICE_DIST_* AND their numeric values were
// reordered so that NO_RANDOM == 0 (see the zero-init contract note below). Fields renamed
// mean/std -> center/spread (matching core's neutral center/spread naming). Callers must recompile.
//
// Distribution type constants for LUMICE_Distribution.type. Values deliberately match core
// DistributionType's enum order (src/core/math.hpp) so "zero-init == not random" holds in both
// layers with the same integer. The C API translates via a hand-written JSON string switch
// (c_api.cpp), NOT an integer cast, so the values need only stay self-consistent here.
#define LUMICE_DIST_NO_RANDOM 0
#define LUMICE_DIST_UNIFORM 1
#define LUMICE_DIST_GAUSS 2
#define LUMICE_DIST_ZIGZAG 3
#define LUMICE_DIST_LAPLACIAN 4
#define LUMICE_DIST_GAUSS_LEGACY 5

// A randomizable scalar. `center`/`spread` roles by type (照抄 core src/core/math.hpp):
//   NO_RANDOM     center = the deterministic value itself; spread = unused
//   UNIFORM       center = mean;                           spread = full range
//   GAUSS         center = mean;                           spread = std dev
//   ZIGZAG        center = mean;                           spread = amplitude
//   LAPLACIAN     center = mean;                           spread = scale
//   GAUSS_LEGACY  center = mean;                           spread = std dev (legacy no-Jacobian)
// Units are field-dependent (axis angles: degrees; face_distance: dimensionless ratio) — the
// distribution type is unit-agnostic, exactly as core's Distribution.
//
// ZERO-INIT CONTRACT: LUMICE_DIST_NO_RANDOM == 0 is a design promise. After `LUMICE_Distribution
// d{}` (or memset(0)), `type == NO_RANDOM` and the scalar is `center == 0` — i.e. NOT random.
// This fixes the pre-v4.10 trap where LUMICE_AXIS_DIST_GAUSS == 0 made a zero-inited struct a
// "std=0 gauss" instead of "not random".
typedef struct LUMICE_Distribution_ {
  int type;      // LUMICE_DIST_NO_RANDOM / UNIFORM / GAUSS / ZIGZAG / LAPLACIAN / GAUSS_LEGACY
  float center;  // role depends on type (see table above)
  float spread;  // role depends on type (see table above); unused for NO_RANDOM
} LUMICE_Distribution;

// Index space for LUMICE_CrystalParam.sync_group[] — one slot per randomizable shape scalar.
// A prism only owns HEIGHT + the six faces, a pyramid only owns UPPER_H/PRISM_H/LOWER_H + the six
// faces; a slot that does not apply to the crystal type is simply never read for that type.
//
// ⚠️ The order mirrors core ShapeScalar (src/config/crystal_config.hpp) VERBATIM, and that order is
// the RNG draw order — which is what makes "a group's leader = its lowest-index applicable member"
// identical to "the member drawn first", so no second ordering definition is needed on either side.
// It is therefore DELIBERATELY NOT the field declaration order of LUMICE_CrystalParam below
// (height / prism_h / upper_h / lower_h — note UPPER_H and PRISM_H are swapped relative to it).
// Do not "tidy up" the divergence: aligning these two orders silently redefines which member of a
// mixed group owns the distribution. The core header carries the same warning.
#define LUMICE_SHAPE_SCALAR_HEIGHT 0   // .height — prism only
#define LUMICE_SHAPE_SCALAR_UPPER_H 1  // .upper_h — pyramid only
#define LUMICE_SHAPE_SCALAR_PRISM_H 2  // .prism_h — pyramid only
#define LUMICE_SHAPE_SCALAR_LOWER_H 3  // .lower_h — pyramid only
#define LUMICE_SHAPE_SCALAR_FACE_0 4   // .face_distance[0] — both types
#define LUMICE_SHAPE_SCALAR_FACE_1 5
#define LUMICE_SHAPE_SCALAR_FACE_2 6
#define LUMICE_SHAPE_SCALAR_FACE_3 7
#define LUMICE_SHAPE_SCALAR_FACE_4 8
#define LUMICE_SHAPE_SCALAR_FACE_5 9
#define LUMICE_SHAPE_SCALAR_COUNT 10

// BREAKING (v4.10): the five shape scalars below
// (height/prism_h/upper_h/lower_h/face_distance[6]) were promoted from bare float to
// LUMICE_Distribution so a randomizable shape can be expressed through the C struct path (they
// map to core PrismCrystalParam.h_/d_[6] and PyramidCrystalParam.h_prs_/h_pyr_u_/h_pyr_l_, all
// already Distribution in core). upper_wedge_angle/lower_wedge_angle stay bare float, mirroring
// core's wedge_angle_u_/l_ (not Distribution). Layout changed; callers must recompile.
//
// BREAKING (v4.13): `sync_group[LUMICE_SHAPE_SCALAR_COUNT]` appended at the end (and the ten
// LUMICE_SHAPE_SCALAR_* index constants added above). Core has expressed shape-scalar sync groups
// since v4.12, but this struct had no slot for them, so the C API — the only path a config file,
// the GUI or a python caller ever takes — dropped the declaration on the floor: core always
// received all-zero and nothing warned. Layout changed; callers must recompile. Behavior does not:
// a zero-initialized struct means every scalar independent, exactly as before.
typedef struct LUMICE_CrystalParam_ {
  int id;
  int type;  // 0=prism, 1=pyramid

  // Prism
  LUMICE_Distribution height;

  // Pyramid
  LUMICE_Distribution prism_h;
  LUMICE_Distribution upper_h;
  LUMICE_Distribution lower_h;
  float upper_wedge_angle;  // degrees, angle between pyramidal face and c-axis (bare float, not Distribution)
  float lower_wedge_angle;  // degrees

  // Face distance (distance from center to each of the 6 prism faces), each a distribution.
  // Zero-init makes each element {NO_RANDOM, center=0, spread=0} == degenerate (zero-distance)
  // geometry: this is the SAME "caller must initialize" contract the pre-v4.10 bare float[6]{}
  // already carried, not a new trap. Regular hexagonal prism default = six {NO_RANDOM, 1.0f, 0.0f}.
  LUMICE_Distribution face_distance[6];

  // Axis distributions. Sampled values feed the rotation chain (angles in degrees)
  // R = Rz(azimuth - 180°) * Ry(-zenith) * Rz(roll); see doc/coordinate-convention.md.
  LUMICE_Distribution zenith;
  LUMICE_Distribution azimuth;
  LUMICE_Distribution roll;

  // Shape-scalar sync groups (v4.13), indexed by LUMICE_SHAPE_SCALAR_*: 0 = independent,
  // 1..N = group id. Members of one group share a SINGLE random draw — the group's first
  // applicable member (lowest LUMICE_SHAPE_SCALAR_* index) consumes the RNG and owns the
  // distribution; the rest reuse its value without consuming anything, and a member whose own
  // distribution differs is overwritten (with a warning, not silently). Mirrors core
  // PrismCrystalParam/PyramidCrystalParam::sync_group_ (src/config/crystal_config.hpp).
  //
  // ZERO-INIT CONTRACT: after `LUMICE_CrystalParam cp{}` (or memset(0)) every scalar is
  // independent — IDENTICAL to pre-v4.13 behavior, and the serialized JSON stays byte-identical
  // (the "sync_group" key is emitted only when something is actually synced). No existing caller
  // needs to change anything.
  //
  // Group ids are canonicalized by core on parse: singleton groups collapse to 0 and surviving
  // groups are renumbered 1..N by first appearance, so {2,1,2,1,2,1} and {1,2,1,2,1,2} are the
  // same partition and compare equal. Ids need not be dense or ordered on the way in.
  int sync_group[LUMICE_SHAPE_SCALAR_COUNT];
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

  // Complex arm (type == LUMICE_FILTER_TYPE_COMPLEX). Historically an index into the removed
  // LUMICE_Config's compositions[] pool; on the Scene path the composition is passed alongside
  // the filter to LUMICE_SceneAddComplexFilter, which IGNORES this field.
  int composition_index;
} LUMICE_FilterParam;

// Sum-of-products composition for a complex filter, passed to LUMICE_SceneAddComplexFilter
// alongside the filter it belongs to. The outer level is an OR over clauses; each clause
// is an AND over its terms; each term is the ID of a SIMPLE filter in the same scene
// (referenced by id — reorder-robust — not by array index; a term may never reference another
// complex filter, matching core config semantics).
//
// BREAKING (v4.9, task-host-abi-cpu-caps): storage layout changed from inline
// `clauses[16][8]` / `term_counts[16]` to a pair of owned heap pointers with a
// clause-major flat encoding. Rationale: at 16×8 inline the ceiling was too low for
// real "OR of several hundred raypaths" use cases, and naively widening the inline array
// (e.g. to 4096×64) would have blown the stack budget of the wide value struct that carried
// these records at the time. Callers populate one record via LUMICE_CompositionSetClauses and
// must release via LUMICE_CompositionReleaseClauses. Do NOT copy this struct by value —
// term_ids/term_counts are owning pointers; aliasing copies would double-free on double
// Release (enforced by scripts/check_policies.py's `no-config-by-value-copy` gate).
// LUMICE_SceneAddComplexFilter deep-copies the record, so a caller's composition can be
// released as soon as that call returns.
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
// "zero-init requires explicit follow-up" discipline: every writer that assembles a color class
// (GUI scene builder, JSON reader, hand-written caller) must set it.
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

// Lens projection kinds. Values mirror the declaration order of core LensParam::LensType, but the
// C API<->core mapping is an explicit switch, so a future reorder on either side cannot silently
// alias one projection onto another.
#define LUMICE_LENS_TYPE_LINEAR 0
#define LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA 1
#define LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT 2
#define LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC 3
#define LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA 4
#define LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUIDISTANT 5
#define LUMICE_LENS_TYPE_DUAL_FISHEYE_STEREOGRAPHIC 6
#define LUMICE_LENS_TYPE_RECTANGULAR 7
#define LUMICE_LENS_TYPE_FISHEYE_ORTHOGRAPHIC 8
#define LUMICE_LENS_TYPE_DUAL_FISHEYE_ORTHOGRAPHIC 9
#define LUMICE_LENS_TYPE_GLOBE 10

// Which anchor the exposure scale is measured against (mirrors core RenderConfig::EvMode).
//   RELATIVE — anchor to the frame's own P99. The image keeps its look as ray_num grows, but the
//              config alone does not determine output brightness (ray_num co-determines it).
//   ABSOLUTE — anchor to the EMITTED energy, so two simulations at the same EV are comparable.
// RELATIVE == 0 is the default: a zero-initialized LUMICE_RenderParam asks for the mode that
// reproduces what the GUI displays, which is also what a config with no "ev_mode" key means.
#define LUMICE_EV_MODE_RELATIVE 0
#define LUMICE_EV_MODE_ABSOLUTE 1

// Which half of the celestial sphere the renderer draws (mirrors core RenderConfig::VisibleRange).
#define LUMICE_VISIBLE_UPPER 0
#define LUMICE_VISIBLE_LOWER 1
#define LUMICE_VISIBLE_FULL 2

// One overlay grid line (mirrors core GridLineParam). `value` is the azimuth (central grid) or
// elevation (elevation grid) in degrees; the rest is appearance.
typedef struct LUMICE_GridLine_ {
  float value;
  float width;
  float opacity;
  float color[3];
} LUMICE_GridLine;

// BREAKING (v4.3): norm_mode field removed; struct layout changed. Callers must recompile against this header.
// BREAKING (v4.11): extended from the 6-field projection-agnostic subset to the full renderer
// description (lens / lens_shift / view / visible / background / ray_color / grid /
// horizon). Before this, those fields had no home in the struct, so every C API entry
// point that re-encodes a renderer (LUMICE_SceneFromJson/File, LUMICE_SceneAddRenderer)
// silently replaced them with a hardcoded
// dual_fisheye_equal_area/fov180/view000/visible=full/black-background renderer — a config could
// parse cleanly and then be simulated with a projection the caller never asked for. Callers must
// recompile.
// BREAKING (v4.16): opacity field removed; struct layout changed. The core RenderConfig field it
// mirrored had no drawing consumer anywhere in the tree since the first commit — it parsed,
// serialized and compared, but never reached a pixel, so every caller setting it was configuring
// nothing. Removed rather than implemented: the renderer composites into a single image with no
// layer to be transparent against. Callers must recompile.
//
// WARNING: a zero-initialized `LUMICE_RenderParam{}` is NOT a committable state — lens_fov = 0 is
// rejected as an invalid FOV for every lens type. Callers must set at least lens_type/lens_fov
// explicitly (same "zero-init requires explicit follow-up" discipline as LUMICE_ColorClass's
// visible field and LUMICE_FILTER_TYPE_UNSET). No implicit non-zero default is baked in on
// purpose: an implicit default silently substituted for the caller's intent is exactly the defect
// this version fixes.
typedef struct LUMICE_RenderParam_ {
  int id;
  int resolution_w;
  int resolution_h;
  float intensity_factor;
  float overlap;   // Dual fisheye overlap zone |sky.z| threshold (sin value). 0 = no overlap.
  int lens_type;   // LUMICE_LENS_TYPE_*
  float lens_fov;  // degrees; valid range depends on lens_type (core MaxFov)
  int lens_shift[2];
  float view_azimuth;
  float view_elevation;
  float view_roll;
  int visible;  // LUMICE_VISIBLE_*
  // Linear RGB — it is added to the halo's radiance before the sRGB transfer curve, so it has to
  // live in the same space the addition does. The JSON "background" key is sRGB instead (what a
  // color picker shows); both JSON parsers convert at their boundary, so a caller writing this
  // struct directly passes linear while a caller writing JSON writes sRGB.
  float background[3];
  // Fixed ray tint in linear RGB, or {-1,-1,-1} (core's default sentinel) for "use the natural
  // spectral color". Zero-init means an all-black tint, NOT the sentinel.
  float ray_color[3];
  // Non-zero = draw a line along the celestial horizon (altitude 0). Opt-in: core's
  // RenderConfig::horizon_ defaults to false, so a zero-initialized struct asks for
  // no annotation, which is what the JSON path also gives a config with no "grid" object.
  int horizon;
  // PARSED BUT NOT RENDERED. Both lists are validated, round-tripped through JSON and compared,
  // and no code draws either — a scene that sets them produces exactly the image it would produce
  // without them. They are kept because the far target ("a CLI re-render equals what the GUI
  // showed, annotations included") needs them, and the blocker is not the drawing code but a model
  // mismatch: this schema names every line individually while the GUI derives one FOV-adaptive
  // step and one shared colour, plus a separate list of sun angular-distance circles. Reconciling
  // the two is a design decision, so the fields stay and say so. horizon above is the
  // one member of this group that does draw.
  LUMICE_GridLine central_grid[LUMICE_MAX_CONFIG_GRID_LINES];
  int central_grid_count;
  LUMICE_GridLine elevation_grid[LUMICE_MAX_CONFIG_GRID_LINES];
  int elevation_grid_count;
  // ADDED (v4.16): LUMICE_EV_MODE_*. Appended at the end of the struct, and RELATIVE == 0 so a
  // zero-initialized param keeps the documented default rather than silently opting into the
  // absolute anchor.
  int ev_mode;
} LUMICE_RenderParam;

// =============== Scene (opaque handle) ===============
// LUMICE_Scene (opaque type declared up top) is THE configuration container of this API and the
// only way to describe a simulation. It is built incrementally: adding a subsystem is adding a
// function (never an ABI break to existing callers), there is no MAX_* compile-time ceiling on
// the number of items, and there is exactly one commit entry point. The handle owns all its
// state; callers manage it exclusively through the lifecycle functions.
//
// The Add*/Set* family covers nine subsystems. The leaf POD structs above (LUMICE_CrystalParam /
// FilterParam / ComplexComposition / RenderParam / ScatterLayer / SpectrumEntry / ColorClass)
// pass in by const pointer — the Scene deep-copies every input value immediately, so the
// caller's leaf struct may be a stack temporary that is discarded/reused right after the call
// returns (no "must outlive commit" lifetime reasoning). Type + lifecycle + leaf writes are
// joined by the serialization half (SceneToJson / SceneFromJson / SceneFromJsonFile, below) and
// by the commit entry point (LUMICE_CommitScene, below).
//
// v4.12 removed the wide `LUMICE_Config` value struct this family replaced, along with its three
// commit entry points and its parse/serialize/ownership helpers. See the BREAKING note at
// LUMICE_API_VERSION for the full removed-symbol list.
//
// Naming: this family uses Noun-Verb order (SceneCreate / SceneAddCrystal / …), coexisting with
// the repo's existing Verb-Noun names (LUMICE_CreateServer / LUMICE_DestroyServer). Both are
// accepted conventions; neither is "the new standard".

// ---------- Lifecycle ----------
// Allocate an empty scene. The caller owns the returned handle and MUST eventually pass it to
// LUMICE_SceneDestroy. Returns NULL only on allocation failure.
LUMICE_Scene* LUMICE_SceneCreate(void);

// Deep-copy `scene` into a brand-new, fully independent handle (no aliasing with the original).
// This is the value the old wide-struct semantics really bought — atomic modal edit / Cancel —
// now a single call instead of a ~128 KB stack copy. Mutating the clone never affects the
// original and vice versa; each must be Destroyed independently. Returns NULL if `scene` is
// NULL or on allocation failure.
LUMICE_Scene* LUMICE_SceneClone(const LUMICE_Scene* scene);

// Release a scene handle. NULL-safe no-op (same contract as LUMICE_DestroyServer). Each handle
// must be Destroyed exactly once; destroying the same handle twice is undefined behavior (this
// mirrors LUMICE_DestroyServer and every other handle in this API — there is no double-free
// sentinel).
void LUMICE_SceneDestroy(LUMICE_Scene* scene);

// ---------- Incremental build: leaf POD in, sequential id out ----------
// Every Add* appends one item and writes its 0-based sequence id (its index among items of the
// same kind, in insertion order) to *out_id. The Scene assigns this id itself and IGNORES any
// `.id` field on the incoming POD. Cross-referencing fields the caller constructs later
// (LUMICE_FilterParam.crystal_id, LUMICE_ScatterEntry.crystal_id/filter_id, a composition's
// term ids) MUST use these returned out_id values, not a caller-chosen id.
//
// Validation is Add-time: each call validates the one item's shape/enums before writing, and
// returns an error code (never throws across the C ABI). On any validation failure the scene is
// left unchanged (no partial write). Returns LUMICE_ERR_NULL_ARG when scene / the input pointer
// / out_id is NULL; LUMICE_ERR_INVALID_CONFIG on an invalid item (bad enum, out-of-range count,
// or exceeding the soft per-kind capacity given by the matching LUMICE_MAX_CONFIG_*).
LUMICE_ErrorCode LUMICE_SceneAddCrystal(LUMICE_Scene* scene, const LUMICE_CrystalParam* crystal, int* out_id);
// SceneAddFilter handles the SIMPLE filter arms only (none/raypath/entry_exit/direction/crystal);
// a filter with type == LUMICE_FILTER_TYPE_COMPLEX is rejected (LUMICE_ERR_INVALID_CONFIG) —
// use LUMICE_SceneAddComplexFilter for those.
LUMICE_ErrorCode LUMICE_SceneAddFilter(LUMICE_Scene* scene, const LUMICE_FilterParam* filter, int* out_id);
// Add a complex (sum-of-products) filter in one call: the filter identity plus its composition.
// The Scene DEEP-COPIES composition->term_ids / term_counts into its own state immediately, so
// the caller's LUMICE_ComplexComposition (and its heap arrays) can be released/reused right
// after this returns. `filter->type` and `filter->composition_index` are ignored (type is
// forced to complex; the composition is taken from `composition`, not looked up by index).
LUMICE_ErrorCode LUMICE_SceneAddComplexFilter(LUMICE_Scene* scene, const LUMICE_FilterParam* filter,
                                              const LUMICE_ComplexComposition* composition, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddRenderer(LUMICE_Scene* scene, const LUMICE_RenderParam* renderer, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddScatterLayer(LUMICE_Scene* scene, const LUMICE_ScatterLayer* layer, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddColorClass(LUMICE_Scene* scene, const LUMICE_ColorClass* color_class, int* out_id);

// ---------- Scalar / whole-group settings, grouped by subsystem ----------
// Each Set* is idempotent (last write wins) and callable in any order. Returns
// LUMICE_ERR_NULL_ARG for a NULL scene, LUMICE_ERR_INVALID_CONFIG / _INVALID_VALUE for an
// invalid value.
//
// Light source + spectrum interact: a discrete spectrum (SceneSetCustomSpectrum with count > 0)
// takes precedence over the `spectrum` string, matching the core config's own rule.
// So SceneSetLightSource does NOT overwrite a discrete spectrum already set, and the two
// call orders (SetLightSource then SetCustomSpectrum, or the reverse) converge to the same
// result. SceneSetCustomSpectrum with count == 0 clears the discrete spectrum (falls back to the
// default "D65" string).
LUMICE_ErrorCode LUMICE_SceneSetLightSource(LUMICE_Scene* scene, float sun_altitude, float sun_azimuth,
                                            float sun_diameter, const char* spectrum);
LUMICE_ErrorCode LUMICE_SceneSetCustomSpectrum(LUMICE_Scene* scene, const LUMICE_SpectrumEntry* entries, int count);
LUMICE_ErrorCode LUMICE_SceneSetSimParams(LUMICE_Scene* scene, int infinite, LUMICE_RayCount ray_num, int max_hits,
                                          int geom_clock);
LUMICE_ErrorCode LUMICE_SceneSetColorMode(LUMICE_Scene* scene, int raypath_color_mode);

// ---------- Serialization: decoupled from commit ----------
// These are the JSON authoring half of the handle API and are DELIBERATELY independent of
// LUMICE_Server: SceneFromJson / SceneFromJsonFile only produce a handle, never commit or
// re-simulate (that stays the exclusive job of the commit entry points). Round-trip is lossless:
// SceneFromJson(ToJson(scene)) is semantically equal to the original.
//
// SceneToJson serializes `scene` into `out_buf` using an snprintf-style buffer contract: pass
// out_buf == NULL (or buf_size == 0) to query the length only; on a
// too-small buffer the output is truncated but always NUL-terminated, and *out_len (when non-NULL)
// always reports the full, untruncated length. Returns LUMICE_ERR_NULL_ARG for a NULL scene,
// LUMICE_ERR_INVALID_CONFIG if the scene cannot be serialized (e.g. a Set* call was fed a string
// that is not valid UTF-8).
LUMICE_ErrorCode LUMICE_SceneToJson(const LUMICE_Scene* scene, char* out_buf, size_t buf_size, size_t* out_len);

// SceneFromJson / SceneFromJsonFile parse and validate a full scene JSON document and, on success,
// allocate a brand-new handle written to *out_scene (the caller owns it and MUST eventually pass
// it to LUMICE_SceneDestroy). On ANY failure *out_scene is set to NULL and no handle is leaked, so
// the caller never Destroys a handle that was not produced. Error-code semantics:
// LUMICE_ERR_NULL_ARG (NULL json_str/filename or out_scene), LUMICE_ERR_INVALID_JSON (syntax
// error), LUMICE_ERR_MISSING_FIELD (a required field absent), LUMICE_ERR_INVALID_VALUE /
// LUMICE_ERR_INVALID_CONFIG (bad enum / value / exceeds a LUMICE_MAX_CONFIG_* soft cap),
// LUMICE_ERR_FILE_NOT_FOUND (SceneFromJsonFile: file cannot be opened).
LUMICE_ErrorCode LUMICE_SceneFromJson(const char* json_str, LUMICE_Scene** out_scene);
LUMICE_ErrorCode LUMICE_SceneFromJsonFile(const char* filename, LUMICE_Scene** out_scene);

// ---------- Commit ----------
// Commit `scene` to `server`. This is the ONE and only commit entry point of the API: v4.12
// removed the three legacy LUMICE_Config commit functions, and every configuration — however it
// was authored — reaches the core through here.
//
// Commit is DELIBERATELY decoupled from serialization: this takes a handle, never a JSON string
// or a file path. To commit JSON, first build a handle with LUMICE_SceneFromJson /
// LUMICE_SceneFromJsonFile, then pass it here.
//
// `scene` is neither consumed nor destroyed: it is read as const, the server keeps no reference
// to it, and the caller still owns it and must eventually call LUMICE_SceneDestroy. The same
// handle may be committed repeatedly (edit via Add*/Set*, commit again).
//
// `out_reused` is OPTIONAL (may be NULL). When non-NULL it receives 1 if the server reused the
// existing consumers/renderers across this commit (no renderer-layout change), 0 if they were
// rebuilt.
//
// Errors: LUMICE_ERR_NULL_ARG (NULL server or scene); otherwise whatever the core commit rejects
// the scene with — LUMICE_ERR_INVALID_CONFIG
// / _MISSING_FIELD / _INVALID_VALUE / _INVALID_JSON for a configuration the core refuses, and
// LUMICE_ERR_SERVER for a server-side failure. On any error *out_reused is left untouched. Note
// that no whole-scene re-validation happens here: each Add*/Set* call already validated its own
// input, so what this can still surface is cross-field/semantic rejection from the core.
LUMICE_ErrorCode LUMICE_CommitScene(LUMICE_Server* server, const LUMICE_Scene* scene, int* out_reused);

// =============== Complex-Composition storage lifecycle (task-host-abi-cpu-caps, BREAKING v4.9) ===============
// Populate `comp` in one shot from an application-owned (clause_count, term_counts[], term_ids[])
// triple. This is the ONLY supported writer for the composition storage — direct field writes
// (previously legal against the old inline `clauses[16][8]`) are no longer defined.
//
// This is a one-shot value write (Set), not a two-phase allocate-then-fill-in-place: every
// production caller already holds the complete
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
//     returns LUMICE_ERR_INVALID_CONFIG.
//   - `term_ids` must be a clause-major flat array of length `sum(term_counts[0..clause_count))`.
//     Elements are simple-filter IDs; reference-integrity checks (each id resolves to an
//     existing SIMPLE filter in the same scene) are the CALLER's responsibility
//     (the c_api / GUI writers already run this pre-check).
//   - Allocator is calloc/free; callers must eventually release via
//     LUMICE_CompositionReleaseClauses (or its config-wide sibling / RAII guard).
LUMICE_ErrorCode LUMICE_CompositionSetClauses(LUMICE_ComplexComposition* comp, int clause_count, const int* term_counts,
                                              const int* term_ids);

// Release the term_ids / term_counts allocations owned by `comp`, if any. Idempotent and null-safe:
//   - `comp == nullptr` → no-op.
//   - Otherwise → free both pointers (either may already be null), then leave `comp` in the
//     "OR of nothing" state (both nullptr, clause_count=0).
void LUMICE_CompositionReleaseClauses(LUMICE_ComplexComposition* comp);

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
// LUMICE_ColorClass and require re-simulation to change (LUMICE_CommitScene).
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
// frame acquired with LUMICE_AcquireResultFrame.
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
// Thread safety: display-time only; safe relative to OTHER display-time readers
// (LUMICE_AcquireResultFrame, LUMICE_GetSimLifecycle, etc.). NOT thread-safe with a concurrent
// LUMICE_CommitScene — the existing single-owner commit rule
// (doc/capi-lifecycle-architecture.md §4) still applies to this setter.
LUMICE_ErrorCode LUMICE_SetRaypathColors(LUMICE_Server* server, const LUMICE_ColorClassDisplay* classes,
                                         int class_count, const int* z_order, int mode);

// task-345.3: display-time EV for the composite (raypath_color) path only.
// `ev_total` is applied as `2^ev_total` inside the composite bake — a single scalar shared
// by every participating color class (per-class renormalization stays structurally excluded;
// the mono / non-composite path is unaffected).
//
// No accumulator reset / no epoch bump / no sim restart — the setter just flips the internal
// snapshot_dirty_ flag, so the next acquired result frame rebuilds the composite with the new EV.
// Callers that already keep the poller running (a live sim, or a display-time refresh triggered
// by other setters) get the new brightness on the next poll; callers that stopped the poller must
// wake it (mirrors the LUMICE_SetRaypathColors + poller-wake pattern used by the GUI).
//
// Thread safety: display-time only; safe relative to other display-time readers
// (LUMICE_AcquireResultFrame, LUMICE_GetSimLifecycle, LUMICE_SetRaypathColors, etc.). NOT
// thread-safe with concurrent LUMICE_CommitScene (same single-owner rule as the rest of the
// display-time surface).
LUMICE_ErrorCode LUMICE_SetCompositeExposure(LUMICE_Server* server, float ev_total);

// Display-time background colour for the composite (raypath_color) path only. `background_linear` is a caller-owned
// array of 3 floats, ADDITIVE **linear** RGB — the same convention the render config's `background` carries internally
// (see doc/configuration.md: JSON/picker values are sRGB, the struct side is linear). A caller holding a picker's sRGB
// triple must pre-convert it with lumice::SrgbToLinearRgb (src/util/color_space.hpp, an inline header function — see
// LUMICE_XyzToSrgbUint8WithBackground for the same note) before calling. The value is added inside the composite bake
// to every pixel the lens actually images — outside the image circle, and in the hemisphere `visible` excludes, nothing
// is painted, matching what the mono path does with the committed config's background.
//
// The mono / non-composite path is unaffected: it keeps taking its background from the committed
// scene. Pushing the SAME colour through both is what makes toggling raypath colour on and off
// leave the background pixels unchanged.
//
// No accumulator reset / no epoch bump / no sim restart — the setter just flips the internal
// snapshot_dirty_ flag, so the next acquired result frame rebuilds the composite with the new
// background. Callers that already keep the poller running get it on the next poll; callers that
// stopped the poller must wake it (mirrors the LUMICE_SetCompositeExposure + poller-wake pattern
// used by the GUI). All-zero (the server default) is an algebraic no-op, so a caller that never
// calls this sees byte-identical composites.
//
// Returns LUMICE_ERR_NULL_ARG if `server` or `background_linear` is NULL.
//
// Thread safety: display-time only; safe relative to other display-time readers
// (LUMICE_AcquireResultFrame, LUMICE_GetSimLifecycle, LUMICE_SetCompositeExposure, etc.). NOT
// thread-safe with concurrent LUMICE_CommitScene (same single-owner rule as the rest of the
// display-time surface).
LUMICE_ErrorCode LUMICE_SetCompositeBackground(LUMICE_Server* server, const float* background_linear);

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
// acquire a result frame first. O(W*H * class_count *
// consumers) scan; intended for infrequent polls (commit-debounce cadence, ~1 Hz), not per
// render frame.
LUMICE_ErrorCode LUMICE_GetColorClassSignal(LUMICE_Server* server, int* out_flags, int class_count);

// =============== Results ===============
// See doc/capi-lifecycle-architecture.md §5 for sentinel contract.
// Unified pattern: (server, out, max_count) -> LUMICE_ErrorCode, sentinel-terminated.
// Sentinel is written at out[count] only when count < max_count.
// When the array is full (count == max_count), no sentinel slot is written;
// callers relying on sentinel iteration must value-initialize the array
// (e.g., Type arr[N + 1]{}) and pass max_count = N. Minimum array size:
// max_count + 1 for sentinel iteration, max_count for direct index access.

// ---------- Result frame (opaque handle) ----------
// Every result read goes through a frame. Acquire one, read whatever kinds of result you
// need out of it, release it:
//
//     LUMICE_ResultFrame* frame = NULL;
//     if (LUMICE_AcquireResultFrame(server, &frame) == LUMICE_OK) {
//       LUMICE_RawXyzResult xyz[2] = {0};
//       LUMICE_FrameGetRawXyz(frame, xyz, 1);
//       /* xyz[0].xyz_buffer stays valid until the Release below */
//       LUMICE_ReleaseResultFrame(frame);
//     }
//
// WHY a handle rather than plain getters: the buffers these results point at are owned by
// the server, and the server keeps producing new snapshots. Without a handle a reader holds
// a pointer with no share of its lifetime — the next snapshot frees or rewrites the memory
// under it, which is a use-after-free, not merely a stale read. Holding a frame is what
// makes the borrow safe, and it is why the read functions below take a frame instead of a
// server.
//
// Two frames are independent: acquiring a second one does not affect the first, and a
// caller may hold as many as it likes.
//
// Materializes a pending snapshot (like the getters it replaces), then writes a new frame
// handle to *out_frame. The caller owns that handle and MUST eventually pass it to
// LUMICE_ReleaseResultFrame. Returns LUMICE_ERR_NULL_ARG if `server` or `out_frame` is NULL.
// On success *out_frame is never NULL, even before the first snapshot — such a frame simply
// reads as "no results" (every FrameGet* writes its sentinel / all-zero struct).
LUMICE_ErrorCode LUMICE_AcquireResultFrame(LUMICE_Server* server, LUMICE_ResultFrame** out_frame);

// Release a frame handle. NULL-safe no-op (same contract as LUMICE_DestroyServer). Each
// handle must be Released exactly once; releasing the same handle twice is undefined
// behavior (this mirrors LUMICE_DestroyServer and every other handle in this API — there is
// no double-free sentinel).
//
// Failure mode if you forget: the frame — and only that frame — leaks. It cannot corrupt
// memory or affect any other reader, because a frame is immutable and separately
// reference-counted; and a leak is what ASan/LSan/valgrind already report, so no
// project-specific machinery is needed to find one. After the Release, every pointer read
// out of that frame is dangling; copy what you still need first.
void LUMICE_ReleaseResultFrame(LUMICE_ResultFrame* frame);

// Read functions. Same (out, max_count) array shape and same sentinel contract as the
// server-taking getters, so only the first argument differs. Any two reads from the SAME
// frame describe the same snapshot generation by construction — no separate "combined"
// getter is needed to pair them. All return LUMICE_ERR_NULL_ARG on a NULL frame/out.

// Raw XYZ float data + intensity scalars (xyz_buffer == NULL sentinel).
LUMICE_ErrorCode LUMICE_FrameGetRawXyz(const LUMICE_ResultFrame* frame, LUMICE_RawXyzResult* out, int max_count);

// Per-raypath composite sRGB images, one per colored renderer (img_buffer == NULL sentinel).
// Empty (out[0] sentinel) when no `raypath_color` is configured — the mono path below is
// unaffected. composite_p99_y is meaningful here (see its field docs).
LUMICE_ErrorCode LUMICE_FrameGetComposite(const LUMICE_ResultFrame* frame, LUMICE_RenderResult* out, int max_count);

// Mono / full-spectrum sRGB uint8 images (img_buffer == NULL sentinel). composite_p99_y is
// left at 0 on this path and must be ignored.
LUMICE_ErrorCode LUMICE_FrameGetRender(const LUMICE_ResultFrame* frame, LUMICE_RenderResult* out, int max_count);

// Simulation statistics. Single value, so no max_count: writes an all-zero struct when the
// frame carries no stats (e.g. acquired before the first snapshot).
LUMICE_ErrorCode LUMICE_FrameGetStats(const LUMICE_ResultFrame* frame, LUMICE_StatsResult* out);

// Cheap O(1) live accumulated sim ray count — no snapshot, no render, no XYZ copy.
// For progress polling (e.g. the --benchmark drain loop) that needs sim_ray_num
// every iteration but not a rendered image. Unlike acquiring a result frame, this
// does NOT trigger DoSnapshot/PostSnapshot, and it reads the running counter
// directly, so it needs no external snapshot driver to stay fresh.
// Writes 0 if no StatsConsumer (or none produced yet).
LUMICE_ErrorCode LUMICE_GetSimRayCount(LUMICE_Server* server, LUMICE_RayCount* out);

// =============== State & Control ===============
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out);

// Read the explicit simulation lifecycle + current epoch (single-source truth).
// LUMICE_QueryServerState is a projection of this. After a synchronous commit,
// call this to read back the just-minted epoch (no commit-signature change).
LUMICE_ErrorCode LUMICE_GetSimLifecycle(LUMICE_Server* server, LUMICE_SimLifecycleResult* out);

// Read the consumer-side drain status (see LUMICE_DrainResult for the contract).
// Cheap O(1) atomic read — same cost class as LUMICE_GetSimRayCount: no snapshot,
// no render, no lock. Safe to call every poll iteration.
//
// Intended use: a reader that wants FINAL accumulated totals polls until
//   out.drained_epoch == out.current_epoch
// and only then acquires a result frame. Waiting for LUMICE_SERVER_IDLE alone is
// not sufficient and never was.
LUMICE_ErrorCode LUMICE_GetDrainStatus(LUMICE_Server* server, LUMICE_DrainResult* out);

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
// Caller allocates LUMICE_CrystalMesh on stack, Core fills vertex/edge data.

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

// Sample one concrete crystal shape from `crystal`'s distributions and build its
// preview mesh. Preview and simulation share the SAME LUMICE_CrystalParam, so there
// is no stringify step and no precision divergence between what is previewed and what
// is simulated. Sampling runs through the core single-source sampler (ns::MakeCrystal),
// so Gaussian/Laplacian/zigzag/uniform semantics and the ring-0 negative-d policy are
// never re-implemented on the GUI side.
//
// Contract:
//   - Deterministic: identical `crystal` + identical `sample_seed` => bit-identical
//     `out` mesh (the basis for T5 preview-animation / thumbnail determinism).
//   - `sample_seed` is a NO-OP for a fully non-random crystal (every shape field
//     LUMICE_DIST_NO_RANDOM): MakeCrystal never touches the RNG, so any seed yields the
//     same mesh (the basis for "preview holds still when std=0").
//   - Degenerate input never crashes: a randomized crystal may sample a shape the
//     closed-form validation gate rejects; that yields an empty-but-valid mesh
//     (all *_count == 0) and LUMICE_OK, never a SIGSEGV.
//   - The mesh is in the crystal's LOCAL frame; the axis distributions
//     (zenith/azimuth/roll) are NOT consumed here (orientation is applied at render time).
//
// Returns LUMICE_ERR_NULL_ARG if `crystal` or `out` is NULL; LUMICE_ERR_INVALID_VALUE
// for an unknown crystal->type; LUMICE_ERR_INVALID_CONFIG if the shape cannot be parsed.
LUMICE_ErrorCode LUMICE_GetCrystalMesh(const LUMICE_CrystalParam* crystal, unsigned long long sample_seed,
                                       LUMICE_CrystalMesh* out);

// =============== Annotation Overlay ===============
// Where a view's auxiliary lines land, in pixels: the celestial horizon, parallels (constant
// altitude), meridians (constant azimuth), circles of constant angular distance from a direction
// (the sun, in every use so far), and the zenith / nadir points. GEOMETRY AND LABEL ANCHORS ONLY —
// colour, line width, glyphs and collision avoidance belong to whoever draws. That split is the
// point: the GUI preview and the CLI renderer draw the same lines their own way, and neither one
// re-derives where they are.
//
// ⚠ THIS IS NOT A PER-FRAME CALL. Every mask costs a width*height inverse-projection sweep —
// single-digit milliseconds from 1024x1024 upward even with the internal row-parallel split, which
// is a whole 60 fps frame budget. Call it once when the view SETTLES and cache the result; a
// caller driving an interactive control must debounce, or freeze the annotation for the duration
// of a drag. The contract is stated here because it is a property of the computation, not of any
// one caller's discipline.
//
// Sanity ceilings on the request lists. As with the LUMICE_MAX_CONFIG_* family these guard against
// malformed input rather than expressing a design limit; a request past one is rejected with
// LUMICE_ERR_INVALID_VALUE rather than truncated.
#define LUMICE_MAX_ANNOTATION_LINES 360
#define LUMICE_MAX_ANNOTATION_CIRCLES 64

// Which family a label belongs to. The consumer decides appearance from this; core encodes none.
#define LUMICE_ANNOTATION_HORIZON 0
#define LUMICE_ANNOTATION_ELEVATION 1
#define LUMICE_ANNOTATION_LONGITUDE 2
#define LUMICE_ANNOTATION_ANGULAR_DIST 3

// Longest label text core produces, including the terminating NUL. Values are at most
// "-180.0" plus a two-byte UTF-8 degree sign.
#define LUMICE_ANNOTATION_LABEL_MAX 16

// The view an overlay is computed for. Separate from LUMICE_RenderParam on purpose: this carries
// `front`, which the renderer has no field for, and it describes a pure computation with no Scene
// or Server lifetime around it. `width`/`height` are the CANVAS the answer is expressed in, which
// need not be the render resolution — a GUI panel showing a re-projected all-sky texture passes
// its own on-screen pixel size and gets anchors in that space.
typedef struct LUMICE_AnnotationView_ {
  int width;
  int height;
  int lens_type;   // LUMICE_LENS_TYPE_*
  float lens_fov;  // degrees
  int lens_shift[2];
  float overlap;  // dual-fisheye overlap zone |sky.z| threshold (sin value); 0 = none
  float view_azimuth;
  float view_elevation;
  float view_roll;
  int visible;  // LUMICE_VISIBLE_*
  // Non-zero clips everything to the camera-facing hemisphere, ON TOP OF `visible`. The two are
  // independent: `visible` says which half of the sky exists, `front` says the viewer only wants
  // what is in front of them.
  int front;
} LUMICE_AnnotationView;

// What to draw. Angle lists are caller-owned and read only for the duration of the call (the same
// borrow rule as the Scene family's leaf structs), so a stack array is fine. A NULL list with a
// zero count means "none of that category".
typedef struct LUMICE_AnnotationRequest_ {
  LUMICE_AnnotationView view;

  // The celestial horizon (altitude 0). Its own flag rather than a 0 entry in `elevation_deg`,
  // because every consumer so far colours it separately.
  int horizon;

  const float* elevation_deg;  // parallels, degrees
  int elevation_count;
  const float* longitude_deg;  // meridians, degrees
  int longitude_count;

  // Circles of constant angular distance from `reference_dir`. The direction need not be
  // normalized; a zero vector falls back to the zenith.
  const float* angular_dist_deg;
  int angular_dist_count;
  float reference_dir[3];

  // Report where zenith and nadir land. Points, not curves: they carry no mask and no text,
  // because a marker's glyph is the consumer's vocabulary, not core's.
  int zenith_nadir;

  // Zero skips the curve walk. The masks alone are several times cheaper than masks plus anchors,
  // and a consumer that draws no text has no use for the anchors.
  int want_labels;
} LUMICE_AnnotationRequest;

// One label: where to put it, what it says, and which curve it came from.
typedef struct LUMICE_AnnotationLabel_ {
  float px;  // canvas pixel, x right
  float py;  // canvas pixel, y down
  int kind;  // LUMICE_ANNOTATION_*
  // Index into the request list this label's curve came from, or -1 for the horizon (which comes
  // from no list). Lets a consumer map a label back to its line without parsing the text.
  int index;
  float value_deg;
  char text[LUMICE_ANNOTATION_LABEL_MAX];
} LUMICE_AnnotationLabel;

// The result. The caller allocates this struct (stack is fine); core allocates what the pointers
// point at, and LUMICE_ReleaseAnnotationOverlay frees it. Every pointer below is owned by core and
// stays valid until that call — the same acquire/release discipline LUMICE_Scene and
// LUMICE_ResultFrame use, with the same rule: exactly one Release per successful Compute, and
// nothing dereferenced afterwards.
typedef struct LUMICE_AnnotationOverlay_ {
  int width;
  int height;

  // 1 where the lens images a piece of sky this request may annotate: imaged, inside `visible`,
  // and inside the front hemisphere when `front` is set. Row-major width*height, indexed
  // py * width + px — the same layout LUMICE_RawXyzResult uses. Every mask below is a subset of
  // this one. NULL only when the view is degenerate.
  const unsigned char* drawable;

  // One mask per annotation CATEGORY, each the union of that category's lines, NULL when the
  // category was not requested. Per category rather than per line because that is the granularity
  // a consumer colours at; a per-line mask set would be tens of megabytes at 4K.
  const unsigned char* horizon;
  const unsigned char* elevation;
  const unsigned char* longitude;
  const unsigned char* angular_dist;

  // Marker positions, valid only when the request asked for them AND the point is on the canvas
  // and inside the requested hemisphere.
  float zenith_px;
  float zenith_py;
  int zenith_valid;
  float nadir_px;
  float nadir_py;
  int nadir_valid;

  const LUMICE_AnnotationLabel* labels;
  int label_count;

  // Opaque handle to the storage the pointers above live in. Do not read, write, copy or free it;
  // pass this struct to LUMICE_ReleaseAnnotationOverlay exactly once instead. Copying the struct
  // copies the handle, so only ONE copy may be released — treat it as a move, not a value.
  void* storage;
} LUMICE_AnnotationOverlay;

// Compute the overlay for one view. `*out` is fully overwritten on success and left untouched on
// failure, so a failed call leaves nothing to release.
//
// Contract:
//   - Pure and deterministic: identical `request` => identical output. No Server, no Scene, no
//     global state, and safe to call from any thread (including concurrently with a running
//     simulation — it shares nothing with one).
//   - A degenerate view (width or height <= 0) is not an error: it yields an overlay with
//     width = height = 0, every pointer NULL and label_count = 0, which still must be Released.
//   - The masks and the anchors agree by construction: both come from one inverse sweep and one
//     forward, the same forward the trace backends run.
//
// Returns LUMICE_ERR_NULL_ARG if `request` or `out` is NULL, or a list pointer is NULL with a
// non-zero count; LUMICE_ERR_INVALID_VALUE for an unknown lens_type / visible, a negative count,
// or a count past LUMICE_MAX_ANNOTATION_LINES / _CIRCLES; LUMICE_ERR_UNKNOWN on allocation
// failure.
LUMICE_ErrorCode LUMICE_ComputeAnnotationOverlay(const LUMICE_AnnotationRequest* request,
                                                 LUMICE_AnnotationOverlay* out);

// Release the storage a successful LUMICE_ComputeAnnotationOverlay allocated, and NULL out the
// pointers so a double release is a no-op rather than a double free. NULL-safe, and safe on an
// already-released or zero-initialized struct (same wording as LUMICE_SceneDestroy: calling it on
// a live overlay exactly once is required; calling it on anything else does nothing).
void LUMICE_ReleaseAnnotationOverlay(LUMICE_AnnotationOverlay* overlay);

// =============== Config ID Range ===============
// Maximum value for LUMICE config IDs (matches core IdType = uint16_t max).
// GUI code should clamp user-editable IDs to [0, LUMICE_MAX_ID].
#define LUMICE_MAX_ID 65535

// =============== Crystal Kind ===============
// Coarse crystal classification used for raypath face-number validation.
// GUI uses this to determine which face numbers are legal for a given crystal.
// A value matching neither enumerator is REJECTED, not guessed at: every entry taking this type
// (LUMICE_IsLegalFace, LUMICE_IsShapeScalarApplicable, LUMICE_ShapeScalarSyncKeyName) answers its
// own negative — 0, 0, and NULL respectively. Note this is the same answer those functions give
// for a legitimate kind paired with an out-of-range face/slot, so it says "no", not "you passed
// garbage"; a caller computing a kind rather than writing a literal still gains nothing by
// skipping its own validation.
// This replaced an earlier lenient contract under which such a value fell through to
// LUMICE_CRYSTAL_PYRAMID. The fall-through was documented and deliberate, but a caller census
// found nothing relying on it, and it had a second cost the reject does not: it silently absorbed
// enum expansion too. The implementations now switch over the enumerators with no `default:`
// label, so adding a third kind warns (-Wswitch) at each site instead of quietly mapping it onto
// Pyramid.
typedef enum LUMICE_CrystalKind_ {
  LUMICE_CRYSTAL_PRISM,    // Basal + prism lateral faces (1,2,3-8)
  LUMICE_CRYSTAL_PYRAMID,  // All faces including upper/lower pyramidal (1,2,3-8,13-18,23-28)
} LUMICE_CrystalKind;

// Returns non-zero if `face` is a legal face number for the given crystal kind.
int LUMICE_IsLegalFace(LUMICE_CrystalKind kind, int face);

// Returns non-zero if shape-scalar `slot` (a LUMICE_SHAPE_SCALAR_* index) physically exists on
// this crystal kind: a prism has .height + the six .face_distance, a pyramid has
// .upper_h/.prism_h/.lower_h + the six .face_distance. Out-of-range slots answer zero.
//
// This is core's own applicability table, not a second copy of it — the same one canonicalization
// scopes itself by when it zeroes a group declared on a slot the type does not have. Ask it rather
// than reimplementing the rule: a GUI-side copy that drifted from core is what once made the
// crystal table display a distribution the simulation did not use.
int LUMICE_IsShapeScalarApplicable(LUMICE_CrystalKind kind, int slot);

// Returns the JSON key naming shape-scalar `slot` — both inside a crystal's `shape` object and
// inside its `shape.sync_group` sub-map, which name each scalar identically. NULL when the slot
// does not apply to this kind (or is out of range). The returned string is static storage — do
// not free it.
//
// All six face slots share the one key "face_distance", whose value is a 6-element array; write it
// once, not once per face. Use this instead of spelling the key names out: they are core's schema,
// a layer that misspells one silently drops the field rather than reporting an error.
//
// The name still says "sync" for compatibility with v4.13, which shipped it: the contract has not
// changed, only the set of callers that ask.
const char* LUMICE_ShapeScalarSyncKeyName(LUMICE_CrystalKind kind, int slot);

// Returns the JSON key inside a crystal's `shape` object holding a pyramidal wedge angle, in
// degrees — the upper one when `upper` is non-zero, the lower one otherwise. Never NULL; static
// storage, do not free.
//
// Takes a plain flag rather than a kind + slot pair because there is nothing for a kind to select:
// both wedge angles exist only on a pyramid, and a caller is already inside a pyramid branch
// before it needs the key. Same reason there is no LUMICE_AXIS_/LUMICE_SHAPE_SCALAR_-style index
// constant to go with it — two states, no third.
const char* LUMICE_ShapeWedgeAngleKeyName(int upper);

// Returns the JSON key inside a crystal's `shape` object holding a pyramidal face's Miller
// indices — the legacy read-side spelling of the quantity LUMICE_ShapeWedgeAngleKeyName names. A
// parser converts these three indices into an angle when the explicit wedge-angle key is absent;
// write paths never emit it. Never NULL; static storage, do not free.
const char* LUMICE_ShapeIndicesKeyName(int upper);

// =============== Axis Scalars ===============
// Index space for the three distributions of a crystal's `axis` object, for LUMICE_AxisScalarKeyName.
//
// Named like the LUMICE_SHAPE_SCALAR_* indices and used the same way, but the two models are NOT
// parallel: an axis has no crystal kind, hence no applicability concept — all three always exist.
// The order is the serialization order only; unlike the shape scalars it is not an RNG draw order.
#define LUMICE_AXIS_SCALAR_ZENITH 0   // .zenith
#define LUMICE_AXIS_SCALAR_AZIMUTH 1  // .azimuth
#define LUMICE_AXIS_SCALAR_ROLL 2     // .roll
#define LUMICE_AXIS_SCALAR_COUNT 3

// Returns the JSON key naming axis-scalar `slot` (a LUMICE_AXIS_SCALAR_* index) inside a crystal's
// `axis` object, or NULL when `slot` is out of range. Static storage — do not free it.
//
// Note LUMICE_AXIS_SCALAR_ZENITH names the wire quantity, which is the complement of core's
// internal latitude (zenith = 90 - latitude): the key names the file format, not the field.
const char* LUMICE_AxisScalarKeyName(int slot);

// Returns non-zero when D (the sigma-d mirror) is applicable to a crystal whose axis has this
// azimuth distribution and this roll anchor. D needs the azimuth to be uniform over a full turn
// and the roll anchor to sit on a multiple of 30 degrees; an axis failing either condition has its
// D flag silently ignored by the engine.
//
// Arguments are the three raw quantities the rule reads and no others:
//   azimuth_dist_type      one of LUMICE_DIST_*, from the azimuth LUMICE_Distribution's .type
//   azimuth_full_range_deg that distribution's .spread (for LUMICE_DIST_UNIFORM, the full width)
//   roll_anchor_deg        the roll LUMICE_Distribution's .center, read type-erased -- it is a
//                          tilt offset for ZIGZAG and an interval midpoint for UNIFORM, so do not
//                          read the name as "the statistical mean of the roll angle"
//
// This is core's own predicate, not a second copy of it. Ask it rather than transcribing the rule:
// a GUI-side transcription is exactly what once let a checkbox report D as live while the engine
// had already dropped it, the two having drifted to different float tolerances (1e-3 against
// 1e-5) on a difference of 3.05e-5.
int LUMICE_IsDApplicable(int azimuth_dist_type, float azimuth_full_range_deg, float roll_anchor_deg);

// =============== Raypath Validation ===============
// Validation state for raypath text input (GUI border color + OK gate).
typedef enum LUMICE_RaypathValidationState_ {
  LUMICE_RAYPATH_VALID,       // All tokens valid; safe to submit
  LUMICE_RAYPATH_INCOMPLETE,  // Trailing/leading separator; user still typing
  LUMICE_RAYPATH_INVALID,     // Non-numeric tokens or illegal face numbers
} LUMICE_RaypathValidationState;

// Validate a raypath text string (dash-separated face indices, e.g. "3-5") against
// both syntax rules and face-number legality for the given crystal kind.
// ',' is retired legacy syntax: a text containing one is LUMICE_RAYPATH_INVALID with a
// dedicated out_msg naming '-' (join faces on one path) and ';' (separate paths).
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

// Same conversion with an additive background composited into it — the sibling an editor needs to
// bake a frame that matches what the renderer put on screen, since the renderer paints the sky
// behind the halo and a bake without it produces a different picture from the same data.
//
// background_linear: LINEAR RGB, 3 floats, added to the halo's radiance AFTER the XYZ->RGB matrix
//                    and BEFORE the clamp and the sRGB transfer curve. That placement is the whole
//                    contract: it is what makes a pixel carrying no halo energy come back as
//                    exactly the sRGB triple a color picker showed. Adding after the curve instead
//                    gamma-encodes the color a second time (0.2 would render as byte 123, not 51).
//                    Linear because that is the space the addition means something in — the same
//                    convention LUMICE_RenderParam::background uses, and the same reason both JSON
//                    parsers convert at their boundary. A C++ caller inside this codebase gets here
//                    from a picker value via lumice::SrgbToLinearRgb (src/util/color_space.hpp, an
//                    inline header function — no separate C API for this conversion); an external
//                    C API consumer applies the standard sRGB EOTF inverse itself.
// Returns LUMICE_ERR_NULL_ARG if any pointer argument is NULL; LUMICE_OK otherwise.
LUMICE_ErrorCode LUMICE_XyzToSrgbUint8WithBackground(const float* xyz_in, unsigned char* out, int pixel_count,
                                                     float intensity_scale, const float* background_linear);

// =============== EV Auto Anchor ===============
// P99 anchor of the auto-EV pipeline (doc/ev-pipeline-architecture.md §2.2/§2.5).
// When downsample_factor > 1 the Y channel is box-summed onto a
// (img_width/f) x (img_height/f) coarse grid, the P99 is taken over the non-zero coarse bins
// and divided by f^2, so the result is a **fine-equivalent** P99 rather than a true per-pixel Y
// statistic. Falls back to the fine per-pixel P99 when downsample_factor <= 1 or the coarse grid
// collapses to zero dimensions.
//
// The coarse and fine paths are not two precisions of one statistic: on a sparse scene they were
// measured 64x apart and respond to sample count with very different slopes, so which one a
// caller picks changes auto-EV by several stops. Pick deliberately.
//
// xyz_data is a borrowed view of at least img_width*img_height*3 floats (3 floats/pixel,
// Y = channel 1), read only for the duration of the call; a raw pointer carries no length, so the
// dimensions passed in are the only bound this function has. Returns 0 if no positive Y entries
// exist.
float LUMICE_ComputeP99Y(const float* xyz_data, int img_width, int img_height, int downsample_factor);

// P99-anchored auto-EV in stops: log2(target_linear / (p99_raw_y / snapshot_intensity)), clamped
// to [-6, 6]. target_linear is the sRGB reverse transform of target_white (0-255 scale). Feed it
// the value LUMICE_ComputeP99Y returned, with the FINE snapshot_intensity even when that P99 came
// from the coarse path — the /f^2 above is what makes the two consistent. Returns 0 if
// snapshot_intensity or p99_raw_y is non-positive.
float LUMICE_ComputeEvAuto(float p99_raw_y, float snapshot_intensity, float target_white);

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
// Takes effect on the next simulation start (after LUMICE_CommitScene). The
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
