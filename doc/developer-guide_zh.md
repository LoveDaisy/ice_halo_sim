[English version](developer-guide.md)

# 开发指南

本文档为项目开发者提供开发环境设置、代码风格、功能扩展、测试和调试等方面的指导。

## 开发环境设置

### 系统要求

- **操作系统**: macOS 13.0+（Ventura）、Linux（Ubuntu 24.04+）或 Windows 10+
- **编译器**: 支持 C++17 的编译器（GCC、Clang 或 MSVC）
- **CMake**: >= 3.14
- **Ninja**（推荐）: 默认构建生成器，增量构建更快且自动并行

所有其他依赖通过 [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake) 自动下载和管理：
- [nlohmann/json](https://github.com/nlohmann/json) v3.10.5 — JSON 解析（header-only）
- [spdlog](https://github.com/gabime/spdlog) v1.15.0 — 日志（header-only）
- [tl-expected](https://github.com/TartanLlama/expected) v1.1.0 — C++17 `expected<T,E>`（header-only）
- [GoogleTest](https://github.com/google/googletest) v1.15.2 — 单元测试（启用 `-t` 时下载）

> **关于 Ninja**: 若未安装 Ninja，可将 `scripts/build.sh` 中的 `-G Ninja` 删除，CMake 将回退到系统默认生成器（通常为 Unix Makefiles）。

### 依赖安装

#### macOS

```bash
# 使用 Homebrew
brew install cmake ninja
```

#### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install cmake ninja-build
```

### 构建配置

项目使用CMake构建，提供了 `scripts/build.sh` 脚本简化构建过程：

```bash
./scripts/build.sh -j release    # Release版本（推荐）
./scripts/build.sh -j debug      # Debug版本（用于调试）
```

**构建选项**：
- `-t`: 编译并运行测试
- `-g`: 构建 GUI 应用（Dear ImGui + GLFW + OpenGL）
- `-b`: 构建基准测试（Google Benchmark）
- `-j`: 并行编译
- `-k`: 清理构建产物（保留依赖缓存）
- `-x`: 清理全部（含依赖缓存）
- `-s`: 构建共享库（默认为静态库）
- `-h`: 显示帮助信息

**常用选项组合**：
- `./scripts/build.sh -gj release`：构建 GUI（不运行测试）
- `./scripts/build.sh -gtj release`：构建 GUI + 运行所有测试（单元测试 + GUI 测试）
- `./scripts/build.sh -tj release`：仅构建核心 + 运行单元测试（不含 GUI）

### IDE配置

#### CLion

1. 打开项目根目录
2. CLion会自动识别CMake配置
3. 配置CMake选项（如需要）

#### VS Code

1. 安装C/C++扩展
2. 安装CMake Tools扩展
3. 打开项目根目录
4. 配置CMake（按需）

#### 其他IDE

- 使用CMake生成项目文件
- 配置包含路径和库路径

### 调试环境配置

#### 使用LLDB (macOS)

```bash
./scripts/build.sh debug
lldb ./build/cmake_build/Lumice
(lldb) run -f examples/config_example.json
```

#### 使用GDB (Linux)

```bash
./scripts/build.sh debug
gdb ./build/cmake_build/Lumice
(gdb) run -f examples/config_example.json
```

## 代码风格指南

### 总体原则

- 使用C++17标准
- 遵循项目现有的代码风格
- 使用 `clang-format` 自动格式化代码

### 命名规范

#### 类名

- 使用 `PascalCase`（大驼峰）
- 示例：`Crystal`, `ServerImpl`, `ConfigManager`

```cpp
class Crystal {
  // ...
};

class ServerImpl {
  // ...
};
```

#### 函数名

- 使用 `PascalCase`（与类方法一致）
- 示例：`CreatePrism()`, `GetResults()`, `CommitConfig()`

```cpp
class Server {
 public:
  void CommitConfig(const std::string& config);
  std::vector<Result> GetResults();
};
```

#### 变量名

- 使用 `snake_case`（小写下划线）
- 成员变量可以加后缀 `_`（如 `server_`, `config_`）
- 示例：`ray_num`, `max_hits`, `img_buffer_`

```cpp
int ray_num = 1000000;
size_t max_hits_ = 7;
float* img_buffer_ = nullptr;
```

#### 常量名

- 使用 `k` 前缀 + `PascalCase`
- 示例：`kDefaultSimulatorCnt`, `kMaxSceneCnt`

```cpp
static constexpr int kDefaultSimulatorCnt = 4;
static constexpr size_t kMaxSceneCnt = 128;
```

#### 命名空间

- 使用 `snake_case`
- 所有代码使用 `lumice` 命名空间

```cpp
namespace lumice {

class Crystal {
  // ...
};

}  // namespace lumice
```

### 代码格式

项目使用 `.clang-format` 配置文件，主要规则：

- **缩进**: 2个空格
- **行宽**: 120字符
- **指针对齐**: 左对齐 (`int* ptr`)
- **命名空间**: 不缩进
- **括号风格**: Attach风格

**格式化代码**：
```bash
./scripts/format.sh
```

### 文件组织

#### 头文件

- 使用 `.hpp` 扩展名
- 包含头文件保护：
```cpp
#ifndef MODULE_FILE_H_
#define MODULE_FILE_H_
// ...
#endif  // MODULE_FILE_H_
```

#### 源文件

- 使用 `.cpp` 扩展名
- 对应的头文件放在同一目录

#### Include顺序

1. 对应的头文件
2. C++标准库
3. 第三方库
4. 项目其他头文件

```cpp
#include "config/config_manager.hpp"  // 1. 对应头文件

#include <vector>                     // 2. C++标准库
#include <memory>

#include <nlohmann/json.hpp>           // 3. 第三方库

#include "core/crystal.hpp"           // 4. 项目其他头文件
#include "util/log.hpp"
```

### 注释规范

#### 文件头注释（可选）

```cpp
/**
 * @file crystal.hpp
 * @brief Crystal geometry and operations
 */
```

#### 类注释

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

#### 函数注释

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

#### 行内注释

- 使用英文注释
- 解释"为什么"而不是"是什么"
- 复杂逻辑必须注释

```cpp
// Multiply by sqrt(3)/4 to convert face distance to actual distance
x.mean *= math::kSqrt3_4;
```

## 如何添加新功能

### 添加新的晶体类型

#### 步骤

1. **在 `crystal_config.hpp` 中添加参数结构**：
```cpp
struct NewCrystalParam {
  Distribution param1_;
  Distribution param2_[6];
};
```

2. **更新 `CrystalParam` variant**：
```cpp
using CrystalParam = std::variant<PrismCrystalParam, PyramidCrystalParam, NewCrystalParam>;
```

3. **在 `crystal_config.cpp` 中实现JSON解析**：
```cpp
void from_json(const nlohmann::json& j, NewCrystalParam& p) {
  // 解析逻辑
}
```

4. **在 `crystal.hpp` 中添加创建方法**：
```cpp
static Crystal CreateNewType(float param1, const float* param2);
```

5. **在 `crystal.cpp` 中实现创建逻辑**：
```cpp
Crystal Crystal::CreateNewType(float param1, const float* param2) {
  // 创建逻辑
}
```

6. **更新配置文档**：在 `configuration.md` 中添加新类型的说明

### 添加新的过滤器类型

#### 步骤

1. **在 `filter_config.hpp` 中添加参数结构**：
```cpp
struct NewFilterParam {
  IdType target_id_;
  float threshold_;
};
```

2. **更新 `SimpleFilterParam` variant**：
```cpp
using SimpleFilterParam = std::variant<
    NoneFilterParam, RaypathFilterParam, EntryExitFilterParam,
    DirectionFilterParam, CrystalFilterParam, NewFilterParam>;
```

3. **在 `filter_config.cpp` 中实现JSON解析**：
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

4. **在 `filter.cpp` 中实现过滤逻辑**：
```cpp
bool Filter::ApplyNewFilter(const NewFilterParam& param, const RaySeg& ray) {
  // 过滤逻辑
}
```

5. **更新配置文档**：在 `configuration.md` 中添加新过滤器类型的说明

### 添加新的消费者

#### 步骤

1. **继承 `IConsume` 接口**：
```cpp
#include "server/consumer.hpp"

namespace lumice {

class MyConsumer : public IConsume {
 public:
  void Consume(const SimData& data) override {
    // 处理数据
  }

  Result GetResult() const override {
    // 返回结果
    return MyResult{};
  }
};

}  // namespace lumice
```

2. **在 `ServerImpl` 中注册消费者**：
```cpp
// 在 ServerImpl::Start() 或类似位置
consumers_.emplace_back(std::make_unique<MyConsumer>());
```

3. **更新文档**：在 `architecture.md` 中添加消费者说明

### 添加新的配置类型

#### 步骤

1. **定义配置结构体**：
```cpp
struct MyConfig {
  IdType id_;
  // 其他字段
};
```

2. **实现JSON序列化**：
```cpp
void to_json(nlohmann::json& j, const MyConfig& c);
void from_json(const nlohmann::json& j, MyConfig& c);
```

3. **在 `ConfigManager` 中添加管理**：
```cpp
struct ConfigManager {
  std::map<IdType, MyConfig> my_configs_;
  // ...
};
```

4. **更新配置文档**：在 `configuration.md` 中添加新配置类型的说明

## 测试指南

### 单元测试

#### 测试框架

项目使用GoogleTest框架进行单元测试。

#### 编写测试

1. **创建测试文件**：
   - 位置：`test/` 目录
   - 命名：`test_*.cpp`

2. **测试结构**：
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

3. **测试Fixture**：
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

#### 运行测试

```bash
# 使用构建脚本
./scripts/build.sh -t release

# 或手动运行
cd build/cmake_build
ctest
```

### GUI 测试

项目使用 [ImGui Test Engine](https://github.com/ocornut/imgui_test_engine) 进行 GUI 自动化测试。测试在隐藏窗口模式下运行，覆盖控件交互和视觉回归。

#### 运行 GUI 测试

```bash
# 构建并运行所有测试（单元测试 + GUI 测试）
./scripts/build.sh -gtj release
```

#### 测试结构

GUI 测试位于 `test/gui/`，按优先级组织：

- **Smoke 测试**：基本应用状态验证
- **P0（文件操作）**：New、Save/Open 往返、Run/Stop UI
- **P1（CRUD）**：晶体/过滤器增删（含确认弹窗）
- **P2（参数）**：镜头切换、dirty 状态检测
- **截图测试**：FBO 截图捕获 + PSNR 比对
- **视觉回归测试**：Crystal preview（Prism/Pyramid、线框/隐藏线/着色）和 Render preview 数据正确性

#### 编写 GUI 测试

GUI 测试采用 GuiFunc（主线程）+ TestFunc（测试线程）模式：

```cpp
ImGuiTest* t = IM_REGISTER_TEST(engine, "category", "test_name");
t->GuiFunc = [](ImGuiTestContext* ctx) {
  // 在主线程运行 — 可安全执行 GL 调用
};
t->TestFunc = [](ImGuiTestContext* ctx) {
  // 在测试线程运行 — 驱动 UI 交互
  ctx->ItemClick("##TopBar/New");
  IM_CHECK(gui::g_state.is_dirty == false);
};
```

#### 视觉回归参考图片

参考图片位于 `test/gui/references/`。更新流程：
1. 运行 `./scripts/build.sh -gtj release` 生成新截图
2. 将更新后的图片拷贝到 `test/gui/references/`
3. 重新构建并验证 PSNR = inf（像素一致）

> **注意**：参考图片在 macOS + Apple Silicon 上生成，跨平台 PSNR 可能有差异。

#### 测试最佳实践

- **测试命名**：`TestSuiteName_TestCaseName`
- **测试独立性**：每个测试应该独立，不依赖其他测试
- **测试数据**：使用固定数据或随机种子确保可复现
- **断言选择**：
  - `EXPECT_*`: 测试失败继续执行
  - `ASSERT_*`: 测试失败立即停止

### E2E 测试

端到端测试验证完整的 CLI 模拟管线，从配置解析到图像输出。

#### 框架

- **Python**：pytest + Pillow（图像比较）
- **位置**：`test/e2e/`

#### 测试结构

```
test/e2e/
├── test_smoke.py            # 冒烟测试（PSNR 图像验证）
├── test_errors.py           # 错误场景测试（退出码验证）
├── configs/                 # 测试配置 JSON 文件
└── references/              # 参考输出图片（*.jpg）
```

#### 运行 E2E 测试

```bash
# 需要已构建的二进制文件 build/cmake_install/Lumice
pytest test/e2e/ -v
```

#### 图像验证

E2E 冒烟测试使用 PSNR（峰值信噪比）将输出图像与参考图像进行比较：
- 参考图片存放在 `test/e2e/references/*.jpg`
- PSNR 阈值：通常为 40 dB，超过即通过
- 更新参考图片：使用测试配置运行 CLI 模拟，替换参考文件

#### 添加新的 E2E 测试

1. 在 `test/e2e/configs/` 中创建配置 JSON
2. 使用新配置运行 CLI 生成参考图像
3. 将参考图像保存到 `test/e2e/references/`
4. 在 `test_smoke.py` 或新测试文件中添加测试用例

## 调试技巧

### 日志系统

项目使用 [spdlog](https://github.com/gabime/spdlog) 作为日志后端，通过 `util/log.hpp` 提供便捷宏。

#### 日志级别

- `TRACE` / `VERBOSE`: 最详细的跟踪信息
- `DEBUG`: 调试信息
- `INFO`: 一般信息（默认级别）
- `WARNING`: 警告信息
- `ERROR`: 错误信息
- `CRITICAL` / `FATAL`: 严重错误信息

#### 使用日志

```cpp
#include "util/log.hpp"

LOG_DEBUG("Debug message: {}", value);
LOG_VERBOSE("Verbose message");
LOG_INFO("Info message");
LOG_WARNING("Warning message");
LOG_ERROR("Error message: {}", error_str);
```

日志使用 [fmt](https://fmt.dev/) 格式化语法（`{}` 占位符），而非 `printf` 风格的 `%d`/`%s`。

#### 类级别日志

对于需要区分来源的日志，可使用命名 logger：

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

#### 设置日志级别

日志级别可通过 `lumice::SetLogLevel()` 在程序中设置：

```cpp
#include "util/logger.hpp"

// 全局 logger 首次访问时自动以 kInfo 级别初始化，无需显式调用。
lumice::GetGlobalLogger().SetLevel(lumice::LogLevel::kDebug);  // 设置为 debug 级别
```

### 调试工具

#### GDB/LLDB基本命令

```bash
# 启动调试
lldb ./build/cmake_build/Lumice

# 设置断点
(lldb) breakpoint set --file crystal.cpp --line 100
(lldb) b crystal.cpp:100

# 运行程序
(lldb) run -f examples/config_example.json

# 查看变量
(lldb) print variable_name
(lldb) p variable_name

# 查看调用栈
(lldb) bt

# 单步执行
(lldb) step      # 进入函数
(lldb) next      # 下一行
(lldb) continue  # 继续执行
```

#### Valgrind内存检查（Linux）

```bash
valgrind --leak-check=full ./build/cmake_build/Lumice -f examples/config_example.json
```

#### 性能分析

```bash
# 使用 perf (Linux)
perf record ./build/cmake_install/Lumice -f examples/config_example.json
perf report

# 使用 Instruments (macOS)
instruments -t "Time Profiler" ./build/cmake_install/Lumice -f examples/config_example.json
```

### 常见问题排查

#### 编译错误

1. **找不到头文件**：
   - 检查 `CMakeLists.txt` 中的包含路径
   - 检查头文件路径是否正确

2. **链接错误**：
   - 检查库是否正确链接
   - 检查库路径是否正确

3. **C++17特性不支持**：
   - 检查编译器版本
   - 检查CMake配置中的C++标准设置

#### 运行时错误

1. **段错误（Segmentation Fault）**：
   - 使用GDB/LLDB调试
   - 检查空指针访问
   - 检查数组越界

2. **配置解析错误**：
   - 检查JSON格式是否正确
   - 启用详细日志查看错误信息
   - 参考配置文档验证配置项

3. **内存泄漏**：
   - 使用Valgrind检查
   - 检查资源是否正确释放

#### 性能问题

1. **运行缓慢**：
   - 检查是否使用了Debug版本（应使用Release版本）
   - 使用性能分析工具定位瓶颈

2. **内存占用高**：
   - 检查是否有内存泄漏
   - 检查数据结构是否合理
   - 考虑使用对象池减少分配

## 贡献指南

### 代码提交流程

1. **创建分支**：
   ```bash
   git checkout -b feature/my-feature
   ```

2. **编写代码**：
   - 遵循代码风格指南
   - 添加必要的测试
   - 更新相关文档

3. **提交代码**：
   ```bash
   git add .
   git commit -m "feat: add new feature"
   ```

4. **推送并创建PR**：
   ```bash
   git push origin feature/my-feature
   ```

### Pull Request规范

- **标题**：简洁描述变更内容
- **描述**：详细说明变更原因和影响
- **测试**：说明如何测试变更
- **文档**：更新相关文档（如需要）

### 代码审查要求

- 代码风格符合项目规范
- 有适当的测试覆盖
- 文档已更新（如需要）
- 没有引入新的警告或错误

## 相关文档

- [系统架构文档](architecture_zh.md) - 了解系统设计
- [配置文档](configuration_zh.md) - 配置格式说明
- [C接口文档](c_api_zh.md) - C接口使用说明
- [文档索引](README_zh.md) - 所有文档的导航
