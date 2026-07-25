[中文版](c_api_zh.md)

# C API Documentation

This document provides detailed instructions on how to use the C API.

## Overview

Lumice provides a complete C interface for easy integration with other languages. The C interface wraps the C++ implementation and exposes a concise API.

### Header File

```c
#include "lumice.h"
```

### Link Library

Link against the `lumice` static library.

### Design Principles

- **Unified error codes**: Most APIs return `LUMICE_ErrorCode`, with actual outputs passed via pointer parameters
- **Array + sentinel pattern**: Result retrieval APIs use fixed-size arrays with sentinel termination, requiring no heap allocation
- **Zero-copy**: The `img_buffer` in render results points directly to the internal buffer, with no copy overhead

## API Reference

### Constants

```c
#define LUMICE_API_VERSION 412        // ABI version, encoded major*100 + minor (v4.12)
#define LUMICE_MAX_RENDER_RESULTS 16  // Maximum capacity of the render result array
#define LUMICE_MAX_STATS_RESULTS 1    // Maximum capacity of the stats result array
```

`LUMICE_API_VERSION` lets a caller pin the ABI it was compiled against and fail loudly on a
mismatch instead of hitting silent UB from a struct-layout drift, e.g.:

```c
static_assert(LUMICE_API_VERSION >= 412, "Lumice header too old for this integration");
```

It is bumped on every BREAKING change to the public symbol set or struct layout.

**v4.12 is such a break.** The wide `LUMICE_Config` value struct and everything that existed only
to feed it were removed: `LUMICE_Config` itself, the three commit entry points
(`LUMICE_CommitConfig` / `LUMICE_CommitConfigFromFile` / `LUMICE_CommitConfigStruct`),
`LUMICE_ParseConfigString` / `LUMICE_ParseConfigFile` / `LUMICE_ConfigToJson`, the
`LUMICE_ConfigCreateColorClasses` / `LUMICE_ConfigReleaseColorClasses` /
`LUMICE_ConfigReleaseCompositions` ownership helpers, and the C++ RAII header
`src/include/lumice_config_scope.hpp`. `LUMICE_Scene` replaces all of them. There is no shim and
no alias: code calling a removed symbol fails to compile, which is the intended migration signal.

### Data Types

#### LUMICE_Server

Server handle, an opaque pointer type.

```c
typedef struct LUMICE_Server_ LUMICE_Server;
```

#### LUMICE_Scene

Scene handle, an opaque pointer type. This is the configuration container of the API — see
[Configuration](#configuration) for its full lifecycle.

```c
typedef struct LUMICE_Scene_ LUMICE_Scene;
```

#### LUMICE_ErrorCode

Error code enum. The return type of most APIs.

```c
typedef enum LUMICE_ErrorCode_ {
  LUMICE_OK = 0,             // Success
  LUMICE_ERR_NULL_ARG,       // NULL argument
  LUMICE_ERR_INVALID_JSON,   // Invalid JSON format
  LUMICE_ERR_INVALID_CONFIG, // Invalid configuration content
  LUMICE_ERR_MISSING_FIELD,  // Missing required field
  LUMICE_ERR_INVALID_VALUE,  // Invalid field value
  LUMICE_ERR_FILE_NOT_FOUND, // File not found / cannot be opened
  LUMICE_ERR_SERVER,         // Internal server error
} LUMICE_ErrorCode;
```

#### LUMICE_ServerState

Server state enum.

```c
typedef enum LUMICE_ServerState_ {
  LUMICE_SERVER_IDLE,      // Idle
  LUMICE_SERVER_RUNNING,   // Running
  LUMICE_SERVER_NOT_READY, // Not ready
} LUMICE_ServerState;
```

#### LUMICE_LogLevel

Log level enum.

```c
typedef enum LUMICE_LogLevel_ {
  LUMICE_LOG_TRACE,    // Trace
  LUMICE_LOG_DEBUG,    // Debug
  LUMICE_LOG_INFO,     // Info (default)
  LUMICE_LOG_WARNING,  // Warning
  LUMICE_LOG_ERROR,    // Error
  LUMICE_LOG_OFF,      // Logging disabled
} LUMICE_LogLevel;
```

#### LUMICE_RenderResult

Render result structure.

```c
typedef struct LUMICE_RenderResult_ {
  int renderer_id;                   // Renderer ID
  int img_width;                     // Image width (pixels)
  int img_height;                    // Image height (pixels)
  const unsigned char* img_buffer;   // Image data buffer (RGB format, read-only)
} LUMICE_RenderResult;
```

**Notes**:
- The image data pointed to by `img_buffer` is managed internally by the library and does not need to be freed manually
- `img_buffer` remains valid until the next call to `LUMICE_GetRenderResults()` or `LUMICE_CommitScene()`
- If you need to retain the data long-term, you should `memcpy` it yourself
- Image data is in RGB format, with 3 bytes per pixel (R, G, B)
- Image data size = `img_width * img_height * 3` bytes
- Sentinel marker: `img_buffer == NULL`

#### LUMICE_StatsResult

Statistics result structure.

```c
typedef struct LUMICE_StatsResult_ {
  unsigned long ray_seg_num;   // Number of ray segments
  unsigned long sim_ray_num;   // Number of simulated rays
  unsigned long crystal_num;   // Number of crystals
} LUMICE_StatsResult;
```

**Notes**:
- Sentinel marker: all zeros (`sim_ray_num == 0`)

#### LUMICE_ServerConfig

Server configuration structure for `LUMICE_CreateServerEx()`.

```c
typedef struct LUMICE_ServerConfig_ {
  int num_workers;        // Number of simulator worker threads. 0 = default (hardware_concurrency - 2)
  unsigned int sim_seed;  // Deterministic seed for worker RNGs. 0 = random (default).
                          // Non-zero collapses to 1 worker for bit-stable results.
} LUMICE_ServerConfig;
```

**Notes**:
- Zero-initialized struct (`= {0}`) is equivalent to default behavior (auto worker count, random seed)
- When `sim_seed != 0`, the server forces `num_workers = 1` to ensure deterministic ray tracing results
- `sim_seed == 0` is "random" but **not fully unseeded** since 260.6: the root-ray PCG stream derives its
  `effective_seed_` from a global atomic counter (reproducible per process by `Simulator` construction order),
  while the host `rng_` used for crystal geometry stays `time ^ thread_id` random. So "0 = random" holds at the
  whole-render level, but the root-ray substream is deterministic given construction order — relevant when
  diffing two runs that build simulators in the same sequence.

### Server Lifecycle

#### LUMICE_CreateServer

Creates a server instance.

```c
LUMICE_Server* LUMICE_CreateServer(void);
```

**Return value**:
- On success: returns a server handle pointer
- On failure: returns `NULL`

**Notes**:
- The returned handle must be freed using `LUMICE_DestroyServer()`

#### LUMICE_CreateServerEx

Creates a server instance with custom configuration.

```c
LUMICE_Server* LUMICE_CreateServerEx(const LUMICE_ServerConfig* config);
```

**Parameters**:
- `config`: pointer to a `LUMICE_ServerConfig` struct; passing `NULL` is equivalent to `LUMICE_CreateServer()`

**Return value**:
- On success: returns a server handle pointer
- On failure: returns `NULL`

**Notes**:
- The returned handle must be freed using `LUMICE_DestroyServer()`
- Use this instead of `LUMICE_CreateServer()` when you need to set a deterministic seed or control worker count

#### LUMICE_DestroyServer

Destroys a server instance.

```c
void LUMICE_DestroyServer(LUMICE_Server* server);
```

**Parameters**:
- `server`: server handle pointer; passing `NULL` is safe

**Notes**:
- Destroying the server stops all ongoing processing
- The handle must not be used after destruction

### Logging

#### LUMICE_SetLogLevel

Sets the log level.

```c
void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level);
```

### Configuration

A configuration lives in a `LUMICE_Scene` — an opaque, incrementally-built handle. There is
exactly one way to configure a server: build or parse a scene, then commit it.

```
   LUMICE_SceneCreate + Add*/Set*  ─┐
                                    ├─→  LUMICE_Scene  ──→  LUMICE_CommitScene(server, scene)
   LUMICE_SceneFromJson{,File}     ─┘         │
                                              └──→  LUMICE_SceneToJson  (save / inspect)
```

Authoring (JSON or programmatic) and committing are deliberately separate: `SceneFromJson` never
touches a server, and `CommitScene` never takes a document. A handle is caller-owned, may be
committed repeatedly, and must eventually be destroyed.

#### JSON schema strictness (`LUMICE_SceneFromJson` / `LUMICE_SceneFromJsonFile`)

All C API entry points that read a config document share one parser, and that parser accepts
**exactly what the simulator's own config parser accepts** — same required keys, same defaults for
omitted optional keys. This is a hard contract, not a best effort: the handle path is the only way
to feed a config into the library, so a parser that is stricter would reject configs the CLI reads
today, and one that is looser would silently accept a document the commit stage then refuses.

The contract is enforced mechanically rather than by review: `JsonParserParity.*` in
`test/unit-correctness/server/test_json_parser_parity.cpp` runs both parsers over every config
file tracked in the repository (enumerated from disk, so new files are covered automatically) and
fails on any input one accepts and the other rejects, or any input both accept but decode to
different values.

Two deliberate exceptions:

- **Invalid enum strings are rejected, not silently reinterpreted.** Where the core parser maps an
  unrecognized `type` / `action` / `spectrum` string onto a plausible-looking fallback, the C API
  returns `LUMICE_ERR_INVALID_VALUE` instead — a typo should surface, not quietly change the
  simulation.
- **Value-range checks fire at commit, not at parse.** Bounds such as `max_hits ∈ [1, 64]` stay
  single-source in the core and are reported by `LUMICE_CommitScene`, so a parse that succeeds is
  not by itself a promise that the commit will. End to end the document is still rejected; only
  the reporting point differs.

Separately from the core's own rules, the C API enforces per-kind soft capacity ceilings
(`LUMICE_MAX_CONFIG_*`, listed in `lumice.h`) at `Add*` / parse time — e.g. at most
`LUMICE_MAX_CONFIG_COLOR_REFS` match refs on one color class. Exceeding one returns
`LUMICE_ERR_INVALID_CONFIG`.

Renderers round-trip in full. Up to v4.10 a renderer's `lens` / `lens_shift` / `view` /
`visible` / `background` / `ray_color` / `grid` / `celestial_outline` had no fields in
`LUMICE_RenderParam`, so they were dropped on parse and replaced with a hardcoded
`dual_fisheye_equal_area` / fov 180 / view {0,0,0} / `visible=full` / black-background renderer on
re-serialization — a document could parse cleanly and then be simulated through a projection the
caller never asked for. v4.11 widened the struct to the full renderer description, and the parity
test now compares renderers with core's own `RenderConfig::operator==`, with no field whitelist.

#### Scene lifecycle

```c
LUMICE_Scene* LUMICE_SceneCreate(void);
LUMICE_Scene* LUMICE_SceneClone(const LUMICE_Scene* scene);
void          LUMICE_SceneDestroy(LUMICE_Scene* scene);
```

- `SceneCreate` returns an empty handle, or `NULL` on allocation failure. The caller owns it and
  must eventually pass it to `SceneDestroy`.
- `SceneClone` deep-copies a scene into a fully independent handle (no aliasing). This is what
  the old wide-struct value semantics really bought — atomic modal edit with a Cancel path — now
  one call instead of a six-figure-byte stack copy. Each handle must be destroyed independently.
  Returns `NULL` if `scene` is `NULL` or on allocation failure.
- `SceneDestroy` is a `NULL`-safe no-op. Each handle must be destroyed exactly once; destroying
  twice is undefined behavior (same contract as `LUMICE_DestroyServer`).

#### Building a scene: `LUMICE_SceneAdd*`

```c
LUMICE_ErrorCode LUMICE_SceneAddCrystal(LUMICE_Scene*, const LUMICE_CrystalParam*, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddFilter(LUMICE_Scene*, const LUMICE_FilterParam*, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddComplexFilter(LUMICE_Scene*, const LUMICE_FilterParam*,
                                              const LUMICE_ComplexComposition*, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddRenderer(LUMICE_Scene*, const LUMICE_RenderParam*, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddScatterLayer(LUMICE_Scene*, const LUMICE_ScatterLayer*, int* out_id);
LUMICE_ErrorCode LUMICE_SceneAddColorClass(LUMICE_Scene*, const LUMICE_ColorClass*, int* out_id);
```

Every `Add*` appends one item and writes its 0-based sequence id — its index among items of the
same kind, in insertion order — to `*out_id`.

- **The scene assigns ids; it ignores any `.id` field on the incoming struct.** Cross-referencing
  fields you build later (`LUMICE_FilterParam.crystal_id`, `LUMICE_ScatterEntry.crystal_id` /
  `.filter_id`, a composition's term ids) MUST use the returned `out_id` values, never an id you
  chose yourself.
- **Every input is deep-copied immediately.** The leaf struct you pass in may be a stack temporary
  that you reuse or discard as soon as the call returns — there is no "must outlive the commit"
  lifetime rule anywhere in this API. `SceneAddComplexFilter` also copies the composition's
  `term_ids` / `term_counts` heap arrays, so you may release them right after it returns.
- **Validation is per-call.** Each `Add*` validates that one item's shape and enums before
  writing; on failure the scene is left completely unchanged (no partial write).
- `SceneAddFilter` handles the SIMPLE arms only (`none` / `raypath` / `entry_exit` / `direction` /
  `crystal`). A filter with `type == LUMICE_FILTER_TYPE_COMPLEX` is rejected — use
  `SceneAddComplexFilter`, which takes the filter identity and its composition together and
  ignores `filter->type` / `filter->composition_index`.

**Return value** (all of them):
- `LUMICE_OK`: success, `*out_id` written.
- `LUMICE_ERR_NULL_ARG`: `scene`, the input pointer, or `out_id` is `NULL`.
- `LUMICE_ERR_INVALID_CONFIG`: invalid item — bad enum, out-of-range count, or the scene already
  holds the matching `LUMICE_MAX_CONFIG_*` capacity of that kind.

#### Scene settings: `LUMICE_SceneSet*`

```c
LUMICE_ErrorCode LUMICE_SceneSetLightSource(LUMICE_Scene*, float sun_altitude, float sun_azimuth,
                                            float sun_diameter, const char* spectrum);
LUMICE_ErrorCode LUMICE_SceneSetCustomSpectrum(LUMICE_Scene*, const LUMICE_SpectrumEntry*, int count);
LUMICE_ErrorCode LUMICE_SceneSetSimParams(LUMICE_Scene*, int infinite, LUMICE_RayCount ray_num,
                                          int max_hits, int geom_clock);
LUMICE_ErrorCode LUMICE_SceneSetColorMode(LUMICE_Scene*, int raypath_color_mode);
```

Each `Set*` is idempotent (last write wins) and callable in any order. Returns
`LUMICE_ERR_NULL_ARG` for a `NULL` scene, `LUMICE_ERR_INVALID_CONFIG` / `LUMICE_ERR_INVALID_VALUE`
for an invalid value.

Light source and spectrum interact, and the interaction is order-independent by design: a discrete
spectrum (`SetCustomSpectrum` with `count > 0`) takes precedence over the `spectrum` string, so
`SetLightSource` does NOT overwrite a discrete spectrum already set. Both call orders converge to
the same result. `SetCustomSpectrum` with `count == 0` clears the discrete spectrum and falls back
to the string (default `"D65"`).

`SetSimParams` takes `infinite` (1 = run forever, 0 = stop after `ray_num` rays), `max_hits`, and
`geom_clock` (geometry-resampling clock K; 0 = let the core derive a default).

#### Serialization: `LUMICE_SceneToJson` / `SceneFromJson` / `SceneFromJsonFile`

```c
LUMICE_ErrorCode LUMICE_SceneToJson(const LUMICE_Scene* scene, char* out_buf, size_t buf_size,
                                    size_t* out_len);
LUMICE_ErrorCode LUMICE_SceneFromJson(const char* json_str, LUMICE_Scene** out_scene);
LUMICE_ErrorCode LUMICE_SceneFromJsonFile(const char* filename, LUMICE_Scene** out_scene);
```

These are the JSON authoring half, and they never touch a `LUMICE_Server`: parsing produces a
handle, nothing more. Round-trip is lossless — `SceneFromJson(SceneToJson(scene))` is semantically
equal to the original. The document format is the same one the CLI reads; see
[Configuration Documentation](configuration.md).

`SceneToJson` uses an snprintf-style caller buffer with no cross-ABI allocation:

- Pass `out_buf == NULL` (or `buf_size == 0`) to query the length without writing.
- On a too-small buffer the output is truncated but **always** NUL-terminated.
- `*out_len` (when non-`NULL`) always reports the full, untruncated length, so the output was
  truncated iff `*out_len >= buf_size`. The usual two-call pattern is: query, allocate
  `*out_len + 1`, call again.
- Returns `LUMICE_ERR_NULL_ARG` for a `NULL` scene, `LUMICE_ERR_INVALID_CONFIG` if the scene
  cannot be serialized (e.g. a `Set*` was handed a string that is not valid UTF-8).

`SceneFromJson` / `SceneFromJsonFile` allocate a brand-new handle into `*out_scene` on success —
you own it and must eventually `SceneDestroy` it. **On any failure `*out_scene` is set to `NULL`
and no handle is leaked**, so you never destroy a handle that was not produced.

**Return value**:
- `LUMICE_OK`: success, `*out_scene` owns a new handle.
- `LUMICE_ERR_NULL_ARG`: `json_str` / `filename` or `out_scene` is `NULL`.
- `LUMICE_ERR_INVALID_JSON`: syntax error.
- `LUMICE_ERR_MISSING_FIELD`: a required field is absent.
- `LUMICE_ERR_INVALID_VALUE` / `LUMICE_ERR_INVALID_CONFIG`: bad enum or value, or a
  `LUMICE_MAX_CONFIG_*` soft cap exceeded.
- `LUMICE_ERR_FILE_NOT_FOUND` (`SceneFromJsonFile` only): the file cannot be opened.

#### LUMICE_CommitScene

Commits a scene to a server. This is the **only** commit entry point in the API.

```c
LUMICE_ErrorCode LUMICE_CommitScene(LUMICE_Server* server, const LUMICE_Scene* scene,
                                    int* out_reused);
```

**Parameters**:
- `server`: server handle.
- `scene`: scene handle. Read as `const`; **neither consumed nor destroyed**. The server keeps no
  reference to it, you still own it, and the same handle may be committed repeatedly (edit via
  `Add*`/`Set*`, commit again).
- `out_reused`: optional (may be `NULL`). When non-`NULL`, receives `1` if the server reused the
  existing consumers/renderers across this commit (no renderer-layout change), `0` if they were
  rebuilt.

**Return value**:
- `LUMICE_OK`: success — the server stops the current run and starts the new one immediately.
- `LUMICE_ERR_NULL_ARG`: `server` or `scene` is `NULL`.
- `LUMICE_ERR_INVALID_CONFIG` / `LUMICE_ERR_MISSING_FIELD` / `LUMICE_ERR_INVALID_VALUE` /
  `LUMICE_ERR_INVALID_JSON`: the core refused the configuration.
- `LUMICE_ERR_SERVER`: server-side failure.

On any error `*out_reused` is left untouched. Note that no whole-scene re-validation happens here:
every `Add*`/`Set*` already validated its own input, so what this can still surface is
cross-field / semantic rejection from the core (e.g. `max_hits` out of range).

#### Per-raypath color classes (`raypath_color`) and `LUMICE_SetRaypathColors`

A scene carries an optional set of *color classes*: each has an RGB color plus a set of
placement-scoped *match* predicates `{layer, crystal, predicate}` that decide which surviving rays
get color-tagged. The composite mode (`LUMICE_COLOR_MODE_DOMINANT` / `_ADDITIVE` / `_PAINTER`) is
a scene setting. A scene with no color classes disables color entirely — the mono
`LUMICE_GetRenderResults` output and the emitted JSON are byte-identical to a config that never
mentioned the feature.

```c
LUMICE_ErrorCode LUMICE_SceneAddColorClass(LUMICE_Scene*, const LUMICE_ColorClass*, int* out_id);
LUMICE_ErrorCode LUMICE_SceneSetColorMode(LUMICE_Scene*, int raypath_color_mode);
```

**There is no ownership contract to honor.** `SceneAddColorClass` deep-copies the class you hand
it, and the scene owns its color-class storage for as long as the handle lives; `SceneDestroy`
releases everything. Up to v4.11 this was the API's sharpest edge — `LUMICE_Config.raypath_color`
was a caller-owned heap pointer with `Create`/`Release` functions, implicit-allocation paths, a
"do not copy the struct by value" rule and a C++ RAII guard to make early returns safe. All of
that is gone with the struct.

> **WARNING** — `LUMICE_ColorClass.visible` is a plain 0/1 field applied verbatim, so a
> zero-initialized `LUMICE_ColorClass{}` has `visible == 0` (INVISIBLE), the opposite of the core
> JSON default. Set `visible = 1` explicitly for every class you want shown, or the compositor
> silently omits it.

Two disjoint change paths — pick by whether the *members* change:

- **Structure change** (`match[]` refs / `combine`) → **re-simulation**. Rebuild the scene's color
  classes and re-commit with `LUMICE_CommitScene`. Read the composite images via
  `LUMICE_GetCompositeResults` (one sRGB `LUMICE_RenderResult` per colored renderer).
- **Appearance change** (RGB / visible / solo / z-order / composite mode) →
  **no re-simulation**:

```c
LUMICE_ErrorCode LUMICE_SetRaypathColors(LUMICE_Server* server,
                                         const LUMICE_ColorClassDisplay* classes,
                                         int class_count, const int* z_order, int mode);
```

**Parameters**:
- `classes`: per-class appearance patch (color, visible, solo). `class_count`
  must equal the committed `raypath_color_count`.
- `z_order`: optional (`NULL` = unchanged); when set, must be a permutation of
  `[0, class_count)` where `z_order[i]` is the new drawing rank of class `i`.
- `mode`: `LUMICE_COLOR_MODE_*`.

**Return value**:
- `LUMICE_OK`: success — the next `LUMICE_GetCompositeResults` re-composites the
  already-accumulated data with the new appearance. The simulation epoch and
  accumulator are untouched (no restart).
- `LUMICE_ERR_NULL_ARG`: `server` is `NULL`, or `classes` is `NULL` with
  `class_count > 0`.
- `LUMICE_ERR_INVALID_VALUE`: `mode` out of range.
- `LUMICE_ERR_INVALID_CONFIG`: `class_count` mismatch (structure changed —
  re-commit the config) or `z_order` is not a valid permutation.

#### LUMICE_GetColorClassSignal (AC4 empty-arc detector)

```c
LUMICE_ErrorCode LUMICE_GetColorClassSignal(LUMICE_Server* server, int* out_flags, int class_count);
```

For each committed color class, reports whether it has captured any rays yet —
i.e. whether its snapshot Y-lane has any non-zero pixel on any active
`RenderConsumer`. Intended for GUI empty-arc warnings when a physical filter
has silently blocked all rays that would have matched a color class's
predicate (a footgun the color-tag/physical-filter decoupling in §4.0 of
`doc/gui-custom-spectrum-and-raypath-color.md` makes possible: a color
predicate can reference a raypath the physical filter already excludes).

**Parameters**:
- `out_flags`: caller-owned buffer of length `class_count`. On success,
  `out_flags[i] = 1` iff class `i` has signal, `0` otherwise.
- `class_count`: must equal the committed `raypath_color_count`; `0` is a
  valid no-op (`out_flags` untouched).

**Return value**:
- `LUMICE_OK`: success.
- `LUMICE_ERR_NULL_ARG`: `server` is `NULL`, or `out_flags` is `NULL` with
  `class_count > 0`.
- `LUMICE_ERR_INVALID_CONFIG`: `class_count` mismatch.

Reads the frozen snapshot state (no `DoSnapshot` trigger) — callers relying on
freshness should query `LUMICE_GetCompositeResults` / `LUMICE_GetRawXyzResults`
first. `O(W*H * class_count * consumers)` scan; intended for infrequent polls
(commit-debounce cadence, ~1 Hz), not per render frame — the GUI color window
throttles its poll to 500ms (`kSignalPollIntervalSec` in `color_window.cpp`).

### Retrieving Results

Result retrieval uses a unified array + sentinel pattern:
- Function signature: `LUMICE_ErrorCode LUMICE_GetXxxResults(server, out, max_count)`
- The `out` array must be at least `max_count + 1` in size (to include the sentinel entry)
- A zero-valued sentinel entry immediately follows the valid results
- The caller loops until it encounters the sentinel

#### LUMICE_GetRenderResults

Retrieves render results.

```c
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);
```

**Parameters**:
- `server`: server handle pointer
- `out`: output array, at least `max_count + 1` in size
- `max_count`: maximum number of results (recommended: `LUMICE_MAX_RENDER_RESULTS`)

**Return value**:
- `LUMICE_OK`: success
- `LUMICE_ERR_NULL_ARG`: `server` or `out` is `NULL`

**Sentinel**: the trailing entry has `img_buffer == NULL`

#### LUMICE_GetStatsResults

Retrieves statistics results.

```c
LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count);
```

**Parameters**:
- `server`: server handle pointer
- `out`: output array, at least `max_count + 1` in size
- `max_count`: maximum number of results (recommended: `LUMICE_MAX_STATS_RESULTS`)

**Return value**:
- `LUMICE_OK`: success
- `LUMICE_ERR_NULL_ARG`: `server` or `out` is `NULL`

**Sentinel**: the trailing entry has `sim_ray_num == 0`

### State and Control

#### LUMICE_QueryServerState

Queries the server state.

```c
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out);
```

**Parameters**:
- `server`: server handle pointer
- `out`: pointer to receive the state

**Return value**:
- `LUMICE_OK`: success, state written to `*out`
- `LUMICE_ERR_NULL_ARG`: `server` or `out` is `NULL`

#### LUMICE_StopServer

Stops the server.

```c
void LUMICE_StopServer(LUMICE_Server* server);
```

**Parameters**:
- `server`: server handle pointer; passing `NULL` is safe

**Notes**:
- After stopping, you can still submit new configurations
- Stopping does not release server resources; call `LUMICE_DestroyServer()` to release them

## Usage Examples

### Basic Example

```c
#include "lumice.h"
#include <stdio.h>

int main() {
    // 1. Create server
    LUMICE_Server* server = LUMICE_CreateServer();
    if (!server) {
        fprintf(stderr, "Failed to create server\n");
        return 1;
    }

    // 2. Parse the config file into a scene handle, commit it, then release the handle.
    //    (The server keeps no reference to the scene, so it can go away right after commit.)
    LUMICE_Scene* scene = NULL;
    if (LUMICE_SceneFromJsonFile("config.json", &scene) != LUMICE_OK) {
        LUMICE_DestroyServer(server);
        return 1;
    }
    LUMICE_ErrorCode commit_err = LUMICE_CommitScene(server, scene, NULL);
    LUMICE_SceneDestroy(scene);
    if (commit_err != LUMICE_OK) {
        LUMICE_DestroyServer(server);
        return 1;
    }

    // 4. Poll for results
    while (1) {
        usleep(1000000);  // 1 second

        // Get render results
        LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1];
        if (LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) == LUMICE_OK) {
            for (int i = 0; renders[i].img_buffer != NULL; i++) {
                printf("Render[%02d]: %dx%d\n",
                       renders[i].renderer_id, renders[i].img_width, renders[i].img_height);
            }
        }

        // Get stats results
        LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1];
        if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) == LUMICE_OK) {
            for (int i = 0; stats[i].sim_ray_num != 0; i++) {
                printf("Stats: rays=%lu, crystals=%lu\n",
                       stats[i].sim_ray_num, stats[i].crystal_num);
            }
        }

        // Check if processing is complete
        LUMICE_ServerState state;
        if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
            break;
        }
    }

    // 5. Destroy server
    LUMICE_DestroyServer(server);
    return 0;
}
```

### Full Example (with Error Handling)

```c
#include "lumice.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <config.json>\n", argv[0]);
        return 1;
    }

    // 1. Create server
    LUMICE_Server* server = LUMICE_CreateServer();
    if (!server) {
        fprintf(stderr, "Error: Failed to create server\n");
        return 1;
    }

    // 2. Set log level (optional, default is INFO)
    LUMICE_SetLogLevel(server, LUMICE_LOG_INFO);

    // 3. Load configuration: file -> scene handle -> commit -> release handle.
    LUMICE_Scene* scene = NULL;
    LUMICE_ErrorCode err = LUMICE_SceneFromJsonFile(argv[1], &scene);
    if (err != LUMICE_OK) {
        fprintf(stderr, "Error: Failed to parse config (error code: %d)\n", err);
        LUMICE_DestroyServer(server);
        return 1;
    }
    err = LUMICE_CommitScene(server, scene, NULL);
    LUMICE_SceneDestroy(scene);   // committed or not, the handle is ours to free
    if (err != LUMICE_OK) {
        fprintf(stderr, "Error: Failed to commit config (error code: %d)\n", err);
        LUMICE_DestroyServer(server);
        return 1;
    }

    // 4. Poll for results
    while (1) {
        usleep(1000000);

        // Get render results
        LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1];
        if (LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) == LUMICE_OK) {
            for (int i = 0; renders[i].img_buffer != NULL; i++) {
                printf("Render[%02d]: %dx%d, buffer=%p\n",
                       renders[i].renderer_id, renders[i].img_width, renders[i].img_height,
                       (const void*)renders[i].img_buffer);

                // Save image (example)
                char filename[256];
                snprintf(filename, sizeof(filename), "output_%d_%dx%d.raw",
                         renders[i].renderer_id, renders[i].img_width, renders[i].img_height);
                FILE* img_file = fopen(filename, "wb");
                if (img_file) {
                    size_t img_size = (size_t)renders[i].img_width * renders[i].img_height * 3;
                    fwrite(renders[i].img_buffer, 1, img_size, img_file);
                    fclose(img_file);
                }
            }
        }

        // Get stats results
        LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1];
        if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) == LUMICE_OK) {
            for (int i = 0; stats[i].sim_ray_num != 0; i++) {
                printf("Stats: rays=%lu, segments=%lu, crystals=%lu\n",
                       stats[i].sim_ray_num, stats[i].ray_seg_num, stats[i].crystal_num);
            }
        }

        // Check if processing is complete
        LUMICE_ServerState state;
        if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
            break;
        }
    }

    // 5. Clean up
    LUMICE_DestroyServer(server);
    return 0;
}
```

## Error Handling

### LUMICE_ErrorCode Error Codes

All configuration, query, and result retrieval APIs return `LUMICE_ErrorCode`. Best practice:

```c
LUMICE_Scene* scene = NULL;
LUMICE_ErrorCode err = LUMICE_SceneFromJsonFile("config.json", &scene);
if (err != LUMICE_OK) {
    switch (err) {
        case LUMICE_ERR_NULL_ARG:
            fprintf(stderr, "Null argument\n");
            break;
        case LUMICE_ERR_FILE_NOT_FOUND:
            fprintf(stderr, "Config file not found\n");
            break;
        case LUMICE_ERR_INVALID_JSON:
            fprintf(stderr, "Invalid JSON format\n");
            break;
        default:
            fprintf(stderr, "Error code: %d\n", err);
            break;
    }
}
```

### Error Handling Best Practices

1. **Check error codes**: Always check the return value of all functions that return `LUMICE_ErrorCode`
2. **Check creation results**: `LUMICE_CreateServer()` returns `NULL` on failure
3. **Resource cleanup**: Ensure `LUMICE_DestroyServer()` is called on all exit paths

## Thread Safety

### Thread-Safe APIs

The following APIs are **thread-safe**:
- `LUMICE_QueryServerState()`: can be safely called from multiple threads
- `LUMICE_GetRenderResults()` / `LUMICE_GetStatsResults()`: can be safely called from multiple threads

### Non-Thread-Safe APIs

The following APIs are **not thread-safe** and must not be called simultaneously from multiple threads:
- `LUMICE_CreateServer()` / `LUMICE_DestroyServer()`: server lifecycle management
- `LUMICE_CommitScene()`: configuration submission
- `LUMICE_Scene*` handle mutation (`LUMICE_SceneAdd*` / `LUMICE_SceneSet*` / `LUMICE_SceneDestroy`
  on one handle) — a handle has no internal locking. Distinct handles are independent, so building
  two scenes on two threads is fine.
- `LUMICE_StopServer()`: server shutdown

### Multithreading Recommendations

1. **Single server, multiple threads**: Use a mutex to protect non-thread-safe operations
2. **Multiple servers**: Use a separate server instance per thread

## Integration with Other Languages

### Python Integration (using ctypes)

```python
import ctypes
import json

# Load library
lib = ctypes.CDLL('./liblumice.so')  # Linux
# lib = ctypes.CDLL('./liblumice.dylib')  # macOS

# Define types
class LUMICE_RenderResult(ctypes.Structure):
    _fields_ = [
        ('renderer_id', ctypes.c_int),
        ('img_width', ctypes.c_int),
        ('img_height', ctypes.c_int),
        ('img_buffer', ctypes.POINTER(ctypes.c_ubyte)),
    ]

class LUMICE_StatsResult(ctypes.Structure):
    _fields_ = [
        ('ray_seg_num', ctypes.c_ulong),
        ('sim_ray_num', ctypes.c_ulong),
        ('crystal_num', ctypes.c_ulong),
    ]

LUMICE_MAX_RENDER_RESULTS = 16
LUMICE_MAX_STATS_RESULTS = 1

# Define function signatures
lib.LUMICE_CreateServer.restype = ctypes.c_void_p
lib.LUMICE_DestroyServer.argtypes = [ctypes.c_void_p]
lib.LUMICE_SceneFromJsonFile.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
lib.LUMICE_SceneFromJsonFile.restype = ctypes.c_int
lib.LUMICE_CommitScene.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
lib.LUMICE_CommitScene.restype = ctypes.c_int
lib.LUMICE_SceneDestroy.argtypes = [ctypes.c_void_p]
lib.LUMICE_SceneDestroy.restype = None
lib.LUMICE_GetRenderResults.argtypes = [ctypes.c_void_p, ctypes.POINTER(LUMICE_RenderResult), ctypes.c_int]
lib.LUMICE_GetRenderResults.restype = ctypes.c_int
lib.LUMICE_GetStatsResults.argtypes = [ctypes.c_void_p, ctypes.POINTER(LUMICE_StatsResult), ctypes.c_int]
lib.LUMICE_GetStatsResults.restype = ctypes.c_int
lib.LUMICE_QueryServerState.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
lib.LUMICE_QueryServerState.restype = ctypes.c_int
lib.LUMICE_StopServer.argtypes = [ctypes.c_void_p]

# Usage example
def simulate(config_file):
    server = lib.LUMICE_CreateServer()
    if not server:
        raise RuntimeError("Failed to create server")

    try:
        scene = ctypes.c_void_p()
        err = lib.LUMICE_SceneFromJsonFile(config_file.encode('utf-8'), ctypes.byref(scene))
        if err != 0:
            raise RuntimeError(f"Failed to parse config (error: {err})")
        try:
            err = lib.LUMICE_CommitScene(server, scene, None)
            if err != 0:
                raise RuntimeError(f"Failed to commit config (error: {err})")
        finally:
            lib.LUMICE_SceneDestroy(scene)

        import time
        while True:
            time.sleep(1)

            # Get render results
            renders = (LUMICE_RenderResult * (LUMICE_MAX_RENDER_RESULTS + 1))()
            if lib.LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) == 0:
                for r in renders:
                    if not r.img_buffer:
                        break
                    print(f"Render[{r.renderer_id:02d}]: {r.img_width}x{r.img_height}")

            # Check state
            state = ctypes.c_int()
            if lib.LUMICE_QueryServerState(server, ctypes.byref(state)) == 0 and state.value == 0:
                break
    finally:
        lib.LUMICE_DestroyServer(server)
```

### Rust Integration

```rust
use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_uchar, c_ulong, c_void};

const LUMICE_MAX_RENDER_RESULTS: usize = 16;
const LUMICE_MAX_STATS_RESULTS: usize = 1;

#[repr(C)]
struct LUMICE_RenderResult {
    renderer_id: c_int,
    img_width: c_int,
    img_height: c_int,
    img_buffer: *const c_uchar,
}

#[repr(C)]
struct LUMICE_StatsResult {
    ray_seg_num: c_ulong,
    sim_ray_num: c_ulong,
    crystal_num: c_ulong,
}

#[link(name = "lumice")]
extern "C" {
    fn LUMICE_CreateServer() -> *mut c_void;
    fn LUMICE_DestroyServer(server: *mut c_void);
    fn LUMICE_SceneFromJsonFile(filename: *const c_char, out_scene: *mut *mut c_void) -> c_int;
    fn LUMICE_CommitScene(server: *mut c_void, scene: *const c_void, out_reused: *mut c_int) -> c_int;
    fn LUMICE_SceneDestroy(scene: *mut c_void);
    fn LUMICE_GetRenderResults(server: *mut c_void, out: *mut LUMICE_RenderResult, max_count: c_int) -> c_int;
    fn LUMICE_GetStatsResults(server: *mut c_void, out: *mut LUMICE_StatsResult, max_count: c_int) -> c_int;
    fn LUMICE_QueryServerState(server: *mut c_void, out: *mut c_int) -> c_int;
    fn LUMICE_StopServer(server: *mut c_void);
}
```

## FAQ

### Q1: How do I load a configuration from a file?

**A**: `LUMICE_SceneFromJsonFile()` parses the file into a scene handle; `LUMICE_CommitScene()` then submits it, and `LUMICE_SceneDestroy()` frees the handle. If you already have the document in memory, use `LUMICE_SceneFromJson()` instead — the rest is identical. Parsing and committing are separate calls on purpose: parsing never touches the server, so you can validate or edit a config without disturbing a running simulation.

### Q2: What is the image data format?

**A**: RGB format, 3 bytes per pixel (R, G, B), stored row by row. Image size = `img_width * img_height * 3` bytes.

### Q3: When does img_buffer become invalid?

**A**: The `img_buffer` pointer remains valid until the next call to `LUMICE_GetRenderResults()` or `LUMICE_CommitScene()`. If you need to retain the data long-term, you should `memcpy` it yourself.

### Q4: Can the server process multiple configurations simultaneously?

**A**: No. Each call to `LUMICE_CommitScene()` stops the current task and starts a new one. To process in parallel, create multiple server instances. (A single scene handle may be committed to several servers — commit reads it as `const` and keeps no reference.)

### Q5: How do I check if the result array is empty?

**A**: Check whether the first element is the sentinel. For render results: `renders[0].img_buffer == NULL` means no results. For stats results: `stats[0].sim_ray_num == 0` means no results.

## Related Documentation

- [Configuration Documentation](configuration.md) - Detailed configuration format reference
- [System Architecture](architecture.md) - System architecture design
- [Developer Guide](developer-guide.md) - Development guide
- [Documentation Index](README.md) - Navigation for all documentation
