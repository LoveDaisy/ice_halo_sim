[中文版](architecture_zh.md)

# System Architecture

This document describes the system architecture design of the Lumice project.

## Overview

The project adopts a Server-Consumer architecture pattern, supporting multi-threaded parallel ray tracing and batch processing.

### Design Goals

1. **High Performance**: Improve simulation speed through multi-threaded parallel processing
2. **Scalability**: Support multiple renderers running in parallel
3. **Modularity**: Clear module boundaries for easier maintenance and extension
4. **Flexibility**: A flexible configuration system that supports complex simulation scenarios

### Architecture Highlights

- **Server-Consumer Pattern**: Uses a producer-consumer pattern to decouple simulation from rendering
- **Multi-threaded Parallelism**: Uses 4 simulator threads in parallel by default
- **Queue System**: Uses thread-safe queues for data transfer
- **Single-Project Mode**: One project configuration (containing one scene and multiple renderers) is submitted at a time

## Overall Architecture

The system follows a three-layer architecture:

```
┌─────────────────────────────────────────┐
│           Config Layer (Config)         │
│  ConfigManager, various Config classes  │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│          Server Layer (Server)          │
│  Server, ServerImpl, Queue System      │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│       Processing Layer (Processing)     │
│  Simulator, Consumer, Render           │
└─────────────────────────────────────────┘
```

## Core Components

### 1. Server

The `Server` class is the central entry point of the architecture. It uses the PIMPL pattern to hide the implementation inside `ServerImpl`. Its responsibilities include:

- Configuration management: parsing and managing JSON configuration files
- Task scheduling: dispatching project configurations to simulators
- Result collection: collecting processed results from consumers
- Lifecycle management: controlling server startup and shutdown

**Main Interface**:
- `CommitConfig(const std::string&)`: Submit a JSON configuration string
- `CommitConfigFromFile(const std::string&)`: Load configuration from a file
- `GetRenderResults()`: Retrieve render results (non-blocking)
- `GetStatsResult()`: Retrieve statistics results (non-blocking)
- `IsIdle()`: Check whether the server is idle
- `Stop()` / `Terminate()`: Stop the server

**Error Handling**:
```cpp
struct Error {
  ErrorCode code;   // kSuccess, kInvalidJson, kInvalidConfig, kMissingField, ...
  std::string message;
  std::string field;
};
```

### 2. Simulator

The `Simulator` is responsible for ray tracing simulation:

- Fetches scene configurations from the scene queue
- Performs ray tracing computations
- Puts simulation results into the data queue

**Characteristics**:
- Multiple parallel instances: 4 `Simulator` instances are created by default
- Independent threads: each `Simulator` runs in its own thread
- Independent RNG: each instance uses a different random seed

### 3. Consumer

The `IConsume` interface defines the consumer abstraction:

- `Consume(const SimData&)`: Consume simulation data
- `GetResult()`: Retrieve processed results

**Implementations**:
- `RenderConsumer`: Renders simulation data into images
- `StatsConsumer`: Collects statistics such as ray count and crystal count
- `ShowRayInfoConsumer`: Displays ray information for debugging

### 4. Queue System

Thread-safe `Queue<T>` templates are used for data transfer, implementing blocking retrieval based on `std::mutex` and `std::condition_variable`:

- `scene_queue_`: Scene configuration queue (shared by multiple Simulators)
- `data_queue_`: Simulation data queue (shared by multiple Simulators)

## Data Flow

### Configuration Loading Flow

```
JSON configuration file
    ↓
ConfigManager (parsing)
    ↓
Various Config objects (LightSourceConfig, CrystalConfig, etc.)
    ↓
ProjConfig (combines scene and renderers)
    ↓
CommitConfig() (creates Consumers, starts threads)
```

### Simulation Flow

```
ProjConfig (from config_manager_.project_)
    ↓
GenerateScene() (generates SceneConfig)
    ↓
scene_queue_ (scene queue, shared)
    ↓
Simulator (multiple in parallel)
    ↓
SimData (simulation data)
    ↓
data_queue_ (data queue, shared)
```

### Rendering Flow

```
SimData (from data_queue_)
    ↓
ConsumeData() (dispatches to Consumers)
    ↓
RenderConsumer / StatsConsumer
    ↓
RenderResult / StatsResult
    ↓
GetRenderResults() / GetStatsResult()
```

### Complete Flow

```
Configuration file
  ↓
Server::CommitConfig()
  ↓
ConfigManager (parse configuration)
  ↓
Create Consumers (Render/Stats)
  ↓
Start() → GenerateScene() (generate scenes)
  ↓
scene_queue_ (scene queue) ──→ Simulator × 4 (parallel)
  ↓                                    ↓
data_queue_ (data queue) ←──────────────┘
  ↓
ConsumeData() (dispatch, protected by consumer_mutex_)
  ↓
Consumer (Render/Stats)
  ↓
Result (protected by consumer_mutex_)
  ↓
Server::GetResults()
```

## Threading Model

The system uses a multi-threaded architecture:

1. **Main thread**: Runs `main()`, calls `Server::CommitConfig()` and result retrieval APIs
2. **Simulator threads**: 4 threads by default, each running a `Simulator::Run()`
3. **Scene generation thread**: The `GenerateScene()` thread generates scenes from the project configuration and enqueues them into `scene_queue_`
4. **Data consumption thread**: The `ConsumeData()` thread dequeues data and dispatches it to consumers

**Thread Synchronization**:
- `std::mutex` is used to protect shared resources
- `consumer_mutex_`: Protects `Consume()` and `GetResult()` calls on `consumers_`, preventing races between the data consumption thread and the main thread
- `std::condition_variable` is used for inter-thread communication
- Atomic variables (`std::atomic_bool`) are used to control thread state

## Module Descriptions

### config Module

**Responsibility**: Configuration parsing and management

**Main Classes**:
- `ConfigManager`: Unified configuration manager; holds mappings of all configuration objects (`std::map<IdType, *Config>`)
- `LightSourceConfig`: Light source configuration
- `CrystalConfig`: Crystal configuration
- `FilterConfig`: Filter configuration
- `RenderConfig`: Render configuration
- `SceneConfig`: Scene configuration (includes `ScatteringSetting` and multi-scattering info `MsInfo`)
- `ProjConfig`: Project configuration (combines scene and renderer references)
- `SimData`: Simulation output data structure (contains `RayBuffer` and crystal list)

**Dependencies**:
- Depends on `nlohmann/json` for JSON parsing
- Depends on the `Distribution` type from `core/math`

### core Module

**Responsibility**: Core physics algorithm implementation

**Main Classes**:
- `Crystal`: Crystal geometry representation (triangle mesh), supports creating hexagonal prisms (`CreatePrism`) and hexagonal pyramids (`CreatePyramid`)
- `Simulator`: Ray tracing simulator; fetches scenes from queue, performs ray tracing, outputs `SimData`
- `Filter`: Filter system supporting multiple filtering criteria such as ray path, entry/exit face, and direction
- `RayPath` / `RaySeg`: Ray path and ray segment data structures
- `Optics`: Optical computations (refraction, reflection, Fresnel coefficients)
- `Geo3D`: 3D geometry computations (triangle intersection, normals, etc.)
- `Math`: Math utilities (vector operations, matrices, random distributions, etc.)

### server Module

**Responsibility**: Server architecture implementation

**Main Classes**:
- `Server`: Public server interface (PIMPL pattern)
- `ServerImpl`: Internal server implementation; manages threads, queues, and consumers
- `IConsume`: Abstract consumer interface
- `RenderConsumer`: Render consumer; implements various lens projection algorithms
- `StatsConsumer`: Statistics consumer
- `ShowRayInfoConsumer`: Ray information consumer
- `c_api.cpp`: C API wrapper implementation

**Characteristics**:
- Thread-safe queue system
- Multi-threaded parallel processing
- Non-blocking result retrieval

### util Module

**Responsibility**: Utilities and infrastructure

**Main Classes**:
- `Queue<T>`: Thread-safe blocking queue template based on `std::condition_variable`
- `ThreadingPool`: Thread pool with support for range-step / range-slice task submission
- `log.hpp`: Thin wrapper around spdlog, providing `LOG_DEBUG` / `LOG_INFO` macros and `GetLogger()` named loggers
- `ArgParser`: Command-line argument parser
- `json_util.hpp`: JSON helper macros
- `color_data.hpp`: CIE color matching function data

### gui Module

**Responsibility**: Graphical user interface application

**Main Components**:
- `app.cpp/hpp`: Main application loop, panel rendering, global state management (extracted from `main.cpp` for testability)
- `gui_state.hpp`: `GuiState` struct defining the complete GUI state model (crystals, filters, renderers, scene settings)
- `panels.cpp/hpp`: ImGui panel implementations (Crystal, Render, Filter, Scene tabs)
- `crystal_renderer.cpp/hpp`: OpenGL FBO-based crystal 3D preview (wireframe, hidden-line, shaded styles)
- `preview_renderer.cpp/hpp`: Equirectangular texture preview with lens projection shaders
- `file_io.cpp/hpp`: `.lmc` project file format read/write (binary header + JSON + optional PNG texture)
- `main.cpp`: GUI entry point, GLFW/OpenGL initialization

**Key Design Decisions**:
- GUI logic lives in `lumice::gui` namespace with global state (`g_state`, `g_preview`, etc.)
- Communicates with the simulation core via `lumice.h` C API (same interface as the CLI)
- `lumice_gui_obj` OBJECT library shares compiled GUI + ImGui code between `LumiceGUI` and `LumiceGUITests`

**Dependencies**:
- Dear ImGui v1.91.8-docking (immediate-mode GUI)
- GLFW 3.4 (window management)
- nfd v1.2.1 (native file dialogs)
- OpenGL 3.2 Core Profile
- stb (PNG read/write for `.lmc` textures)

### include Module

**Responsibility**: Public C API header files

- `lumice.h`: Public C interface header using an opaque pointer pattern

## Program Entry Points

**`main.c` → `Lumice`** (CLI)
- Purpose: Command-line simulation program
- Interacts with the core library through the `lumice.h` public API
- Usage:
  ```bash
  ./build/cmake_install/Lumice -f examples/config_example.json
  ```

**`src/gui/main.cpp` → `LumiceGUI`** (GUI, requires `-g` build flag)
- Purpose: Graphical interface for interactive simulation configuration and preview
- Provides crystal 3D preview, render preview, parameter editing, and `.lmc` file management
- Usage:
  ```bash
  ./build/cmake_install/LumiceGUI
  ```

Build targets:
- `lumice`: Core static/shared library (static by default; use `-DBUILD_SHARED_LIBS=ON` to build as a shared library)
- `Lumice`: CLI executable, linked against the `lumice` library
- `LumiceGUI`: GUI executable (built when `-DBUILD_GUI=ON`), linked against `lumice_gui_obj` and `lumice`
- `LumiceGUITests`: GUI test executable (built when both `-DBUILD_GUI=ON` and `-DBUILD_TESTING=ON`)

## C API

The public API is exposed through a C interface (`lumice.h`) using an opaque pointer pattern. Most APIs return a `LUMICE_ErrorCode` error code, with actual output passed via pointer parameters:

```c
// Server lifecycle
LUMICE_Server* LUMICE_CreateServer(void);
void LUMICE_DestroyServer(LUMICE_Server* server);

// Logging
void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level);

// Configuration (returns LUMICE_ErrorCode)
LUMICE_ErrorCode LUMICE_CommitConfig(LUMICE_Server* server, const char* config_str);
LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename);

// Result retrieval (array + sentinel pattern, returns LUMICE_ErrorCode)
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);
LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count);

// State and control
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out);
void LUMICE_StopServer(LUMICE_Server* server);
```

For detailed usage, see the [C API documentation](c_api.md).

## Dependencies

All dependencies are managed automatically via [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake):

| Dependency | Version | Purpose |
|------------|---------|---------|
| [nlohmann/json](https://github.com/nlohmann/json) | v3.10.5 | JSON configuration parsing |
| [spdlog](https://github.com/gabime/spdlog) | v1.15.0 | Logging system |
| [tl-expected](https://github.com/TartanLlama/expected) | v1.1.0 | C++17 `expected<T,E>` |
| [GoogleTest](https://github.com/google/googletest) | v1.15.2 | Unit testing |
| [Dear ImGui](https://github.com/ocornut/imgui) | v1.91.8-docking | GUI (when `-g` enabled) |
| [GLFW](https://www.glfw.org/) | 3.4 | Window management (when `-g` enabled) |
| [nfd](https://github.com/btzy/nativefiledialog-extended) | v1.2.1 | Native file dialogs (when `-g` enabled) |
| [imgui_test_engine](https://github.com/ocornut/imgui_test_engine) | v1.91.8 | GUI testing (when `-g` + `-t` enabled) |

## Extension Points

### Adding a New Consumer

1. Inherit from the `IConsume` interface
2. Implement the `Consume()` method to process data
3. Implement the `GetResult()` method to return results
4. Register the consumer in `ServerImpl`

### Adding a New Configuration Type

1. Define a configuration struct
2. Implement `to_json()` and `from_json()` functions
3. Add management logic in `ConfigManager`

### Custom Simulator

1. Inherit from or modify the `Simulator` class
2. Implement custom ray tracing logic
3. Use the custom simulator in `ServerImpl`

## Important Notes

1. **Thread Safety**: All shared resources require appropriate synchronization mechanisms
2. **Memory Management**: Pay attention to the lifecycle of data in queues
3. **Error Handling**: Configuration parsing errors must be handled properly
4. **Performance Tuning**: The number of simulator threads can be adjusted based on hardware

## Related Documentation

- [README](../README.md) - User documentation
- [Configuration Guide](configuration.md) - Configuration format reference
- [Developer Guide](developer-guide.md) - Developer guide
- [C API Documentation](c_api.md) - C API usage reference
- [Documentation Index](README.md) - Navigation for all documentation
