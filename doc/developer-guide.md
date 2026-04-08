[中文版](developer-guide_zh.md)

# Developer Guide

This document provides guidance for project developers on development environment setup, code style, feature extension, testing, and debugging.

## Development Environment Setup

### System Requirements

- **Operating System**: macOS 13.0+ (Ventura), Linux (Ubuntu 24.04+), or Windows 10+
- **Compiler**: C++17-compatible compiler (GCC, Clang, or MSVC)
- **CMake**: >= 3.14
- **Ninja** (recommended): Default build generator, provides faster incremental builds with automatic parallelism

All other dependencies are automatically downloaded and managed via [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake):
- [nlohmann/json](https://github.com/nlohmann/json) v3.10.5 — JSON parsing (header-only)
- [spdlog](https://github.com/gabime/spdlog) v1.15.0 — Logging (header-only)
- [tl-expected](https://github.com/TartanLlama/expected) v1.1.0 — C++17 `expected<T,E>` (header-only)
- [GoogleTest](https://github.com/google/googletest) v1.15.2 — Unit testing (downloaded when `-t` is enabled)

> **About Ninja**: If Ninja is not installed, you can remove `-G Ninja` from `scripts/build.sh`, and CMake will fall back to the system default generator (typically Unix Makefiles).

### Installing Dependencies

#### macOS

```bash
# Using Homebrew
brew install cmake ninja
```

#### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install cmake ninja-build
```

### Build Configuration

The project uses CMake for building and provides a `scripts/build.sh` script to simplify the build process:

```bash
./scripts/build.sh -j release    # Release build (recommended)
./scripts/build.sh -j debug      # Debug build (for debugging)
```

**Build options**:
- `-t`: Compile and run tests
- `-g`: Build GUI application (Dear ImGui + GLFW + OpenGL)
- `-b`: Build benchmarks (Google Benchmark)
- `-j`: Parallel compilation
- `-k`: Clean build artifacts (keep dependency cache)
- `-x`: Clean everything including dependency cache
- `-s`: Build shared libraries (static libraries by default)
- `-h`: Show help information

**Common flag combinations**:
- `./scripts/build.sh -gj release`: Build GUI without tests
- `./scripts/build.sh -gtj release`: Build GUI + run all tests (unit + GUI)
- `./scripts/build.sh -tj release`: Build core + run unit tests only (no GUI)

### IDE Configuration

#### CLion

1. Open the project root directory
2. CLion will automatically detect the CMake configuration
3. Configure CMake options (if needed)

#### VS Code

1. Install the C/C++ extension
2. Install the CMake Tools extension
3. Open the project root directory
4. Configure CMake (as needed)

#### Other IDEs

- Use CMake to generate project files
- Configure include paths and library paths

### Debugging Environment Configuration

#### Using LLDB (macOS)

```bash
./scripts/build.sh debug
lldb ./build/cmake_build/Lumice
(lldb) run -f examples/config_example.json
```

#### Using GDB (Linux)

```bash
./scripts/build.sh debug
gdb ./build/cmake_build/Lumice
(gdb) run -f examples/config_example.json
```

## Code Style Guide

### General Principles

- Use the C++17 standard
- Follow the existing code style of the project
- Use `clang-format` to automatically format code

### Naming Conventions

#### Class Names

- Use `PascalCase`
- Examples: `Crystal`, `ServerImpl`, `ConfigManager`

```cpp
class Crystal {
  // ...
};

class ServerImpl {
  // ...
};
```

#### Function Names

- Use `PascalCase` (consistent with class methods)
- Examples: `CreatePrism()`, `GetResults()`, `CommitConfig()`

```cpp
class Server {
 public:
  void CommitConfig(const std::string& config);
  std::vector<Result> GetResults();
};
```

#### Variable Names

- Use `snake_case`
- Member variables may use a trailing `_` suffix (e.g., `server_`, `config_`)
- Examples: `ray_num`, `max_hits`, `img_buffer_`

```cpp
int ray_num = 1000000;
size_t max_hits_ = 7;
float* img_buffer_ = nullptr;
```

#### Constant Names

- Use `k` prefix + `PascalCase`
- Examples: `kDefaultSimulatorCnt`, `kMaxSceneCnt`

```cpp
static constexpr int kDefaultSimulatorCnt = 4;
static constexpr size_t kMaxSceneCnt = 128;
```

#### Namespaces

- Use `snake_case`
- All code uses the `lumice` namespace

```cpp
namespace lumice {

class Crystal {
  // ...
};

}  // namespace lumice
```

### Code Formatting

The project uses a `.clang-format` configuration file. Key rules:

- **Indentation**: 2 spaces
- **Line width**: 120 characters
- **Pointer alignment**: Left-aligned (`int* ptr`)
- **Namespaces**: Not indented
- **Brace style**: Attach style

**Format code**:
```bash
./scripts/format.sh
```

### File Organization

#### Header Files

- Use the `.hpp` extension
- Include header guards:
```cpp
#ifndef MODULE_FILE_H_
#define MODULE_FILE_H_
// ...
#endif  // MODULE_FILE_H_
```

#### Source Files

- Use the `.cpp` extension
- Place the corresponding header file in the same directory

#### Include Order

1. Corresponding header file
2. C++ standard library
3. Third-party libraries
4. Other project header files

```cpp
#include "config/config_manager.hpp"  // 1. Corresponding header file

#include <vector>                     // 2. C++ standard library
#include <memory>

#include <nlohmann/json.hpp>           // 3. Third-party libraries

#include "core/crystal.hpp"           // 4. Other project header files
#include "util/log.hpp"
```

### Commenting Conventions

#### File Header Comments (Optional)

```cpp
/**
 * @file crystal.hpp
 * @brief Crystal geometry and operations
 */
```

#### Class Comments

```cpp
/**
 * @brief Crystal geometry representation
 * @details This class represents a crystal with triangular mesh.
 *          It provides methods for creating crystals and querying geometry data.
 */
class Crystal {
  // ...
};
```

#### Function Comments

```cpp
/**
 * @brief Create a hexagonal prism crystal
 * @param h Height-to-diameter ratio (h/a)
 * @param fd Face distance array [d1, d2, d3, d4, d5, d6], optional
 * @return A Crystal object representing the prism
 * @note Default face distances are [1,1,1,1,1,1] for regular hexagon
 */
static Crystal CreatePrism(float h, const float* fd = nullptr);
```

#### Inline Comments

- Use English for comments
- Explain "why" rather than "what"
- Complex logic must be commented

```cpp
// Multiply by sqrt(3)/4 to convert face distance to actual distance
x.mean *= math::kSqrt3_4;
```

## How to Add New Features

### Adding a New Crystal Type

#### Steps

1. **Add a parameter struct in `crystal_config.hpp`**:
```cpp
struct NewCrystalParam {
  Distribution param1_;
  Distribution param2_[6];
};
```

2. **Update the `CrystalParam` variant**:
```cpp
using CrystalParam = std::variant<PrismCrystalParam, PyramidCrystalParam, NewCrystalParam>;
```

3. **Implement JSON parsing in `crystal_config.cpp`**:
```cpp
void from_json(const nlohmann::json& j, NewCrystalParam& p) {
  // Parsing logic
}
```

4. **Add a creation method in `crystal.hpp`**:
```cpp
static Crystal CreateNewType(float param1, const float* param2);
```

5. **Implement the creation logic in `crystal.cpp`**:
```cpp
Crystal Crystal::CreateNewType(float param1, const float* param2) {
  // Creation logic
}
```

6. **Update the configuration documentation**: Add a description of the new type in `configuration.md`

### Adding a New Filter Type

#### Steps

1. **Add a parameter struct in `filter_config.hpp`**:
```cpp
struct NewFilterParam {
  IdType target_id_;
  float threshold_;
};
```

2. **Update the `SimpleFilterParam` variant**:
```cpp
using SimpleFilterParam = std::variant<
    NoneFilterParam, RaypathFilterParam, EntryExitFilterParam,
    DirectionFilterParam, CrystalFilterParam, NewFilterParam>;
```

3. **Implement JSON parsing in `filter_config.cpp`**:
```cpp
void from_json(const nlohmann::json& j, FilterConfig& f) {
  // ...
  else if (type == "new_filter") {
    NewFilterParam p{};
    j.at("target_id").get_to(p.target_id_);
    j.at("threshold").get_to(p.threshold_);
    f.param_ = p;
  }
}
```

4. **Implement the filtering logic in `filter.cpp`**:
```cpp
bool Filter::ApplyNewFilter(const NewFilterParam& param, const RaySeg& ray) {
  // Filtering logic
}
```

5. **Update the configuration documentation**: Add a description of the new filter type in `configuration.md`

### Adding a New Consumer

#### Steps

1. **Inherit the `IConsume` interface**:
```cpp
#include "server/consumer.hpp"

namespace lumice {

class MyConsumer : public IConsume {
 public:
  void Consume(const SimData& data) override {
    // Process data
  }

  Result GetResult() const override {
    // Return result
    return MyResult{};
  }
};

}  // namespace lumice
```

2. **Register the consumer in `ServerImpl`**:
```cpp
// In ServerImpl::Start() or a similar location
consumers_.emplace_back(std::make_unique<MyConsumer>());
```

3. **Update the documentation**: Add consumer description in `architecture.md`

### Adding a New Configuration Type

#### Steps

1. **Define the configuration struct**:
```cpp
struct MyConfig {
  IdType id_;
  // Other fields
};
```

2. **Implement JSON serialization**:
```cpp
void to_json(nlohmann::json& j, const MyConfig& c);
void from_json(const nlohmann::json& j, MyConfig& c);
```

3. **Add management in `ConfigManager`**:
```cpp
struct ConfigManager {
  std::map<IdType, MyConfig> my_configs_;
  // ...
};
```

4. **Update the configuration documentation**: Add a description of the new configuration type in `configuration.md`

## Testing Guide

### Unit Testing

#### Testing Framework

The project uses the GoogleTest framework for unit testing.

#### Writing Tests

1. **Create test files**:
   - Location: `test/` directory
   - Naming: `test_*.cpp`

2. **Test structure**:
```cpp
#include <gtest/gtest.h>
#include "core/crystal.hpp"

using namespace lumice;

TEST(CrystalTest, CreatePrism) {
  auto crystal = Crystal::CreatePrism(1.3);
  EXPECT_GT(crystal.TotalTriangles(), 0);
  EXPECT_GT(crystal.TotalVertices(), 0);
}

TEST(CrystalTest, CreatePyramid) {
  auto crystal = Crystal::CreatePyramid(0.1, 1.2, 0.5);
  EXPECT_GT(crystal.TotalTriangles(), 0);
}
```

3. **Test Fixtures**:
```cpp
class CrystalTest : public ::testing::Test {
 protected:
  void SetUp() override {
    crystal_ = Crystal::CreatePrism(1.3);
  }

  Crystal crystal_;
};

TEST_F(CrystalTest, GetTriangleVtx) {
  const auto* vtx = crystal_.GetTriangleVtx();
  ASSERT_NE(vtx, nullptr);
}
```

#### Running Tests

```bash
# Using the build script
./scripts/build.sh -t release

# Or run manually
cd build/cmake_build
ctest
```

### GUI Testing

The project uses [ImGui Test Engine](https://github.com/ocornut/imgui_test_engine) for GUI automated testing. Tests run in a hidden-window mode and cover widget interactions and visual regression.

#### Running GUI Tests

```bash
# Build and run all tests (unit + GUI)
./scripts/build.sh -gtj release
```

#### Test Structure

GUI tests are located in `test/gui/` and organized by priority:

- **Smoke tests**: Basic application state verification
- **P0 (File operations)**: New, Save/Open roundtrip, Run/Stop UI
- **P1 (CRUD)**: Crystal/Filter add/delete with confirmation popups
- **P2 (Parameters)**: Lens switching, mark-dirty detection
- **Screenshot tests**: FBO capture and PSNR comparison
- **Visual regression tests**: Crystal preview (Prism/Pyramid, Wireframe/HiddenLine/Shaded) and render preview data correctness

#### Writing GUI Tests

GUI tests use the GuiFunc (main thread) + TestFunc (test thread) pattern:

```cpp
ImGuiTest* t = IM_REGISTER_TEST(engine, "category", "test_name");
t->GuiFunc = [](ImGuiTestContext* ctx) {
  // Runs on main thread — safe for GL calls
};
t->TestFunc = [](ImGuiTestContext* ctx) {
  // Runs on test thread — drive UI interactions
  ctx->ItemClick("##TopBar/New");
  IM_CHECK(gui::g_state.is_dirty == false);
};
```

#### Visual Regression Reference Images

Reference images are in `test/gui/references/`. To update:
1. Run `./scripts/build.sh -gtj release` to generate new screenshots
2. Copy updated images to `test/gui/references/`
3. Rebuild and verify PSNR = inf (pixel-identical)

> **Note**: Reference images are generated on macOS + Apple Silicon. Cross-platform PSNR may differ.

#### Testing Best Practices

- **Test naming**: `TestSuiteName_TestCaseName`
- **Test independence**: Each test should be independent and not rely on other tests
- **Test data**: Use fixed data or random seeds to ensure reproducibility
- **Assertion choice**:
  - `EXPECT_*`: Test continues execution on failure
  - `ASSERT_*`: Test stops immediately on failure

### E2E Testing

End-to-end tests verify the complete CLI simulation pipeline, from configuration parsing to image output.

#### Framework

- **Python**: pytest + Pillow (for image comparison)
- **Location**: `test/e2e/`

#### Test Structure

```
test/e2e/
├── conftest.py              # pytest fixtures (binary path, temp dirs)
├── test_smoke.py            # Smoke tests with PSNR image verification
├── test_error_handling.py   # Error scenario tests (exit code verification)
├── configs/                 # Test configuration JSON files
└── references/              # Reference output images (*.jpg)
```

#### Running E2E Tests

```bash
# Requires a built binary at build/cmake_install/Lumice
pytest test/e2e/ -v
```

#### Image Verification

E2E smoke tests compare output images against reference images using PSNR (Peak Signal-to-Noise Ratio):
- Reference images are stored in `test/e2e/references/*.jpg`
- PSNR threshold: tests pass if PSNR exceeds a defined minimum (typically 40 dB)
- To update reference images: run the simulation with the test config and replace the reference file

#### Adding New E2E Tests

1. Create a configuration JSON in `test/e2e/configs/`
2. Generate the reference image by running the CLI with the new config
3. Save the reference to `test/e2e/references/`
4. Add a test case in `test_smoke.py` or a new test file

## Debugging Tips

### Logging System

The project uses [spdlog](https://github.com/gabime/spdlog) as the logging backend, with convenient macros provided through `util/log.hpp`.

#### Log Levels

- `TRACE` / `VERBOSE`: Most detailed trace information
- `DEBUG`: Debug information
- `INFO`: General information (default level)
- `WARNING`: Warning messages
- `ERROR`: Error messages
- `CRITICAL` / `FATAL`: Critical error messages

#### Using Logging

```cpp
#include "util/log.hpp"

LOG_DEBUG("Debug message: {}", value);
LOG_VERBOSE("Verbose message");
LOG_INFO("Info message");
LOG_WARNING("Warning message");
LOG_ERROR("Error message: {}", error_str);
```

Logging uses [fmt](https://fmt.dev/) formatting syntax (`{}` placeholders), not `printf`-style `%d`/`%s`.

#### Class-Level Logging

For logs that need to distinguish their source, use a named logger:

```cpp
#include "util/log.hpp"

class MyClass {
  static auto logger = lumice::GetLogger("MyClass");

  void DoWork() {
    logger->info("Working on task {}", task_id);
    logger->debug("Detail: {}", detail);
  }
};
```

#### Setting the Log Level

The log level can be set programmatically via `lumice::SetLogLevel()`:

```cpp
#include "util/logger.hpp"

// Global logger auto-initializes at kInfo level on first use; no explicit init needed.
lumice::GetGlobalLogger().SetLevel(lumice::LogLevel::kDebug);  // Set to debug level
```

### Debugging Tools

#### GDB/LLDB Basic Commands

```bash
# Start debugging
lldb ./build/cmake_build/Lumice

# Set breakpoints
(lldb) breakpoint set --file crystal.cpp --line 100
(lldb) b crystal.cpp:100

# Run the program
(lldb) run -f examples/config_example.json

# Inspect variables
(lldb) print variable_name
(lldb) p variable_name

# View the call stack
(lldb) bt

# Step execution
(lldb) step      # Step into function
(lldb) next      # Next line
(lldb) continue  # Continue execution
```

#### Valgrind Memory Check (Linux)

```bash
valgrind --leak-check=full ./build/cmake_build/Lumice -f examples/config_example.json
```

#### Performance Profiling

```bash
# Using perf (Linux)
perf record ./build/cmake_install/Lumice -f examples/config_example.json
perf report

# Using Instruments (macOS)
instruments -t "Time Profiler" ./build/cmake_install/Lumice -f examples/config_example.json
```

### Common Troubleshooting

#### Compilation Errors

1. **Header file not found**:
   - Check include paths in `CMakeLists.txt`
   - Verify the header file path is correct

2. **Linking errors**:
   - Check that libraries are linked correctly
   - Verify the library paths are correct

3. **C++17 features not supported**:
   - Check the compiler version
   - Verify the C++ standard setting in the CMake configuration

#### Runtime Errors

1. **Segmentation Fault**:
   - Debug with GDB/LLDB
   - Check for null pointer dereferences
   - Check for array out-of-bounds access

2. **Configuration parsing errors**:
   - Verify the JSON format is correct
   - Enable verbose logging to view error messages
   - Refer to the configuration documentation to validate configuration entries

3. **Memory leaks**:
   - Check with Valgrind
   - Verify that resources are properly released

#### Performance Issues

1. **Slow execution**:
   - Check whether you are using a Debug build (use Release build instead)
   - Use profiling tools to locate bottlenecks

2. **High memory usage**:
   - Check for memory leaks
   - Review whether data structures are appropriate
   - Consider using object pools to reduce allocations

## Contribution Guide

### Code Submission Workflow

1. **Create a branch**:
   ```bash
   git checkout -b feature/my-feature
   ```

2. **Write code**:
   - Follow the code style guide
   - Add necessary tests
   - Update relevant documentation

3. **Commit code**:
   ```bash
   git add .
   git commit -m "feat: add new feature"
   ```

4. **Push and create a PR**:
   ```bash
   git push origin feature/my-feature
   ```

### Pull Request Guidelines

- **Title**: Concisely describe the changes
- **Description**: Explain the reason for and impact of the changes in detail
- **Testing**: Describe how to test the changes
- **Documentation**: Update relevant documentation (if needed)

### Code Review Requirements

- Code style conforms to project standards
- Adequate test coverage is provided
- Documentation is updated (if needed)
- No new warnings or errors are introduced

## Related Documentation

- [System Architecture](architecture.md) - Understand the system design
- [Configuration Documentation](configuration.md) - Configuration format reference
- [C API Documentation](c_api.md) - C interface usage guide
- [Documentation Index](README.md) - Navigation for all documents
