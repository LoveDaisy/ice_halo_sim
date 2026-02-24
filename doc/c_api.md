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

#### LUMICE_InitLogger

Initializes the logging system.

```c
void LUMICE_InitLogger(LUMICE_Server* server);
```

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

    // 2. Initialize logging
    LUMICE_InitLogger(server);

    // 3. Load configuration from file
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

    // 2. Initialize logging
    LUMICE_InitLogger(server);
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
