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
#define LUMICE_MAX_RENDER_RESULTS 16  // Maximum capacity of the render result array
#define LUMICE_MAX_STATS_RESULTS 1    // Maximum capacity of the stats result array
```

### Data Types

#### LUMICE_Server

Server handle, an opaque pointer type.

```c
typedef struct LUMICE_Server_ LUMICE_Server;
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
- `img_buffer` remains valid until the next call to `LUMICE_GetRenderResults()` or `LUMICE_CommitConfig()`
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

#### LUMICE_CommitConfig

Submits a configuration as a JSON string.

```c
LUMICE_ErrorCode LUMICE_CommitConfig(LUMICE_Server* server, const char* config_str);
```

**Parameters**:
- `server`: server handle pointer
- `config_str`: configuration string in JSON format

**Return value**:
- `LUMICE_OK`: success
- `LUMICE_ERR_NULL_ARG`: `server` or `config_str` is `NULL`
- `LUMICE_ERR_INVALID_JSON`: invalid JSON format
- `LUMICE_ERR_INVALID_CONFIG` / `LUMICE_ERR_MISSING_FIELD` / `LUMICE_ERR_INVALID_VALUE`: invalid configuration content

**Notes**:
- After submitting the configuration, the server begins processing immediately
- See the [Configuration Documentation](configuration.md) for the configuration format

#### LUMICE_CommitConfigFromFile

Loads and submits a configuration from a file.

```c
LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename);
```

**Parameters**:
- `server`: server handle pointer
- `filename`: path to the configuration file

**Return value**:
- `LUMICE_OK`: success
- `LUMICE_ERR_NULL_ARG`: `server` or `filename` is `NULL`
- `LUMICE_ERR_FILE_NOT_FOUND`: file does not exist or cannot be opened
- Other error codes: invalid configuration content

#### Per-raypath color classes (`raypath_color`) and `LUMICE_SetRaypathColors`

`LUMICE_Config` carries an optional `raypath_color[]` (Design 2, task-342.2):
each *color class* has an RGB color plus a set of placement-scoped *match*
predicates `{layer, crystal, predicate}` that decide which surviving rays get
color-tagged. `raypath_color_mode` selects the display-time composite mode
(`LUMICE_COLOR_MODE_DOMINANT` / `_ADDITIVE` / `_PAINTER`). `raypath_color_count
== 0` disables color entirely — the mono `LUMICE_GetRenderResults` output and
the emitted JSON are byte-identical to a config without the field.

##### Lifetime — `raypath_color` is a heap pointer (BREAKING v4.8)

Since v4.8 (task-344), `LUMICE_Config.raypath_color` is a heap-allocated
`LUMICE_ColorClass*`, **not** an inline fixed-size array. The change shrinks
`sizeof(LUMICE_Config)` from ~467 KB to ~113 KB (fixed a stack overflow in
the GUI test harness when two `LUMICE_Config` locals shared a single stack
frame) and hands the caller explicit ownership of the color-class allocation.

Two APIs manage the allocation:

```c
LUMICE_ColorClass* LUMICE_ConfigCreateColorClasses(LUMICE_Config* cfg, int count);
void               LUMICE_ConfigReleaseColorClasses(LUMICE_Config* cfg);
```

- `Create` zero-initializes `count` `LUMICE_ColorClass` entries, writes them
  into `cfg->raypath_color` (with `cfg->raypath_color_count = count`), and
  returns the array pointer for the caller to fill. `count == 0` sets
  `raypath_color = NULL / raypath_color_count = 0` (equivalent to "no color
  classes"). Rejects `cfg == NULL`, `count < 0`, or
  `count > LUMICE_MAX_CONFIG_COLOR_CLASSES` by returning `NULL`.
  **Idempotent / create-or-replace**: if `cfg->raypath_color` is already
  non-NULL (previous `Create` or implicit-alloc path), the old allocation
  is freed first, so calling `Create` twice on the same `cfg` is safe.
- `Release` frees the allocation and resets `raypath_color / raypath_color_count`
  to zero. Null-safe and idempotent — safe to call on an already-released or
  never-allocated `cfg`.

**Implicit-alloc paths** (caller does NOT pre-call `Create`; the function
allocates internally): `LUMICE_ParseConfigString` / `LUMICE_ParseConfigFile`
and the GUI's `FillLumiceConfig` allocate `raypath_color` based on data they
parse (JSON `classes` length or `GuiState.raypath_color.size()`). The caller
still owns the resulting allocation and **must call `Release`** (directly or
via the RAII guard below) once done, regardless of who triggered the alloc.

**Do not copy `LUMICE_Config` by value.** Because `raypath_color` is an
owning heap pointer, `LUMICE_Config b = a;` / by-value function parameters /
`std::vector<LUMICE_Config>` alias the same allocation and double-free at
second `Release`. Route through `LUMICE_Config*` or `const LUMICE_Config&`.
`scripts/check_policies.py` (rule `no-config-by-value-copy`) enforces this
statically for `src/` and `test/`.

**RAII helper (C++ only)** — `src/include/lumice_config_scope.hpp` provides
`lumice::ConfigColorGuard`, a non-copyable / non-movable scope guard that
calls `Release` on the referenced `LUMICE_Config` at end of scope, covering
all early-return paths (assertions, `IM_CHECK`, exceptions):

```cpp
LUMICE_Config cfg{};
lumice::ConfigColorGuard _color_guard(cfg);   // frees raypath_color on scope exit
LUMICE_ParseConfigString(json_str, strlen(json_str), &cfg);
// ... use cfg ...
```


Two disjoint change paths — pick by whether the *members* change:

- **Structure change** (`match[]` refs / `combine`) → **re-simulation**. Edit
  `LUMICE_Config.raypath_color[]` and re-submit via `LUMICE_CommitConfig` (JSON
  string) or `LUMICE_CommitConfigStruct` (C struct); both produce identical
  composites. Read the composite images via `LUMICE_GetCompositeResults` (one
  sRGB `LUMICE_RenderResult` per colored renderer).
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

    // 2. Load configuration from file
    if (LUMICE_CommitConfigFromFile(server, "config.json") != LUMICE_OK) {
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

    // 3. Load configuration
    LUMICE_ErrorCode err = LUMICE_CommitConfigFromFile(server, argv[1]);
    if (err != LUMICE_OK) {
        fprintf(stderr, "Error: Failed to load config (error code: %d)\n", err);
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
LUMICE_ErrorCode err = LUMICE_CommitConfigFromFile(server, "config.json");
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
- `LUMICE_CommitConfig()` / `LUMICE_CommitConfigFromFile()`: configuration submission
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
lib.LUMICE_CommitConfigFromFile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
lib.LUMICE_CommitConfigFromFile.restype = ctypes.c_int
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
        err = lib.LUMICE_CommitConfigFromFile(server, config_file.encode('utf-8'))
        if err != 0:
            raise RuntimeError(f"Failed to load config (error: {err})")

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
    fn LUMICE_CommitConfigFromFile(server: *mut c_void, filename: *const c_char) -> c_int;
    fn LUMICE_GetRenderResults(server: *mut c_void, out: *mut LUMICE_RenderResult, max_count: c_int) -> c_int;
    fn LUMICE_GetStatsResults(server: *mut c_void, out: *mut LUMICE_StatsResult, max_count: c_int) -> c_int;
    fn LUMICE_QueryServerState(server: *mut c_void, out: *mut c_int) -> c_int;
    fn LUMICE_StopServer(server: *mut c_void);
}
```

## FAQ

### Q1: How do I load a configuration from a file?

**A**: Use `LUMICE_CommitConfigFromFile()` and pass the file path directly. Alternatively, you can read the file contents into a string and pass it to `LUMICE_CommitConfig()`.

### Q2: What is the image data format?

**A**: RGB format, 3 bytes per pixel (R, G, B), stored row by row. Image size = `img_width * img_height * 3` bytes.

### Q3: When does img_buffer become invalid?

**A**: The `img_buffer` pointer remains valid until the next call to `LUMICE_GetRenderResults()` or `LUMICE_CommitConfig()`. If you need to retain the data long-term, you should `memcpy` it yourself.

### Q4: Can the server process multiple configurations simultaneously?

**A**: No. Each call to `LUMICE_CommitConfig()` stops the current task and starts a new one. To process in parallel, create multiple server instances.

### Q5: How do I check if the result array is empty?

**A**: Check whether the first element is the sentinel. For render results: `renders[0].img_buffer == NULL` means no results. For stats results: `stats[0].sim_ray_num == 0` means no results.

## Related Documentation

- [Configuration Documentation](configuration.md) - Detailed configuration format reference
- [System Architecture](architecture.md) - System architecture design
- [Developer Guide](developer-guide.md) - Development guide
- [Documentation Index](README.md) - Navigation for all documentation
