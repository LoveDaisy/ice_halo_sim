# 开发指南

**版本**: V3  
**最后更新**: 2025-12-19

本文档为项目开发者提供开发环境设置、代码风格、功能扩展、测试和调试等方面的指导。

## 开发环境设置

### 系统要求

- **操作系统**: macOS 10.14+ 或 Linux (Ubuntu 14.04+)
- **编译器**: 支持C++17的编译器（GCC 7+, Clang 5+）
- **CMake**: >= 3.14
- **依赖库**:
  - OpenCV >= 3.3 (用于图像处理)
  - Boost >= 1.63 (filesystem, system)
  - nlohmann/json (已包含在 thirdparty 目录)
  - GoogleTest (用于单元测试，CMake会自动下载)

### 依赖安装

#### macOS

```bash
# 使用 Homebrew
brew install opencv boost cmake
```

#### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install libopencv-dev libboost-filesystem-dev libboost-system-dev cmake
```

### 构建配置

项目使用CMake构建，提供了 `build.sh` 脚本简化构建过程：

```bash
cd cpp
./build.sh release    # Release版本（推荐）
./build.sh debug      # Debug版本（用于调试）
```

**构建选项**：
- `-j`: 并行编译
- `-t`: 编译并运行测试
- `-k`: 清理构建文件
- `-v`: 启用详细日志
- `-b`: 运行性能基准测试

### IDE配置

#### CLion

1. 打开项目根目录
2. CLion会自动识别CMake配置
3. 配置CMake选项（如需要）

#### VS Code

1. 安装C/C++扩展
2. 安装CMake Tools扩展
3. 打开 `cpp` 目录
4. 配置CMake（按需）

#### 其他IDE

- 使用CMake生成项目文件
- 配置包含路径和库路径

### 调试环境配置

#### 使用LLDB (macOS)

```bash
cd cpp
./build.sh debug
lldb ./build/cmake_build/IceHaloV3
(lldb) run -f v3_config_example.json
```

#### 使用GDB (Linux)

```bash
cd cpp
./build.sh debug
gdb ./build/cmake_build/IceHaloV3
(gdb) run -f v3_config_example.json
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
- V3代码使用 `icehalo::v3` 命名空间
- 旧代码使用 `icehalo` 命名空间

```cpp
namespace icehalo {
namespace v3 {

class Crystal {
  // ...
};

}  // namespace v3
}  // namespace icehalo
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
cd cpp
./format.sh  # 如果有格式化脚本
# 或使用 clang-format
clang-format -i src/**/*.{hpp,cpp}
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

#include "json.hpp"                   // 3. 第三方库

#include "core/crystal.hpp"           // 4. 项目其他头文件
#include "include/log.hpp"
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

namespace icehalo {
namespace v3 {

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

}  // namespace v3
}  // namespace icehalo
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
   - 命名：`test_*.cpp` 或 `test_v3_*.cpp`

2. **测试结构**：
```cpp
#include <gtest/gtest.h>
#include "core/crystal.hpp"

using namespace icehalo;

TEST(CrystalTest, CreatePrism) {
  auto crystal = v3::Crystal::CreatePrism(1.3);
  EXPECT_GT(crystal.TotalTriangles(), 0);
  EXPECT_GT(crystal.TotalVertices(), 0);
}

TEST(CrystalTest, CreatePyramid) {
  auto crystal = v3::Crystal::CreatePyramid(0.1, 1.2, 0.5);
  EXPECT_GT(crystal.TotalTriangles(), 0);
}
```

3. **测试Fixture**：
```cpp
class CrystalTest : public ::testing::Test {
 protected:
  void SetUp() override {
    crystal_ = v3::Crystal::CreatePrism(1.3);
  }
  
  v3::Crystal crystal_;
};

TEST_F(CrystalTest, GetTriangleVtx) {
  const auto* vtx = crystal_.GetTriangleVtx();
  ASSERT_NE(vtx, nullptr);
}
```

#### 运行测试

```bash
# 使用构建脚本
cd cpp
./build.sh -t release

# 或手动运行
cd build/cmake_build
ctest
```

#### 测试最佳实践

- **测试命名**：`TestSuiteName_TestCaseName`
- **测试独立性**：每个测试应该独立，不依赖其他测试
- **测试数据**：使用固定数据或随机种子确保可复现
- **断言选择**：
  - `EXPECT_*`: 测试失败继续执行
  - `ASSERT_*`: 测试失败立即停止

### 基准测试

#### 测试框架

项目使用Google Benchmark框架进行性能基准测试。

#### 编写基准测试

1. **创建基准测试文件**：
   - 位置：`test/` 目录
   - 命名：`bench_*.cpp`

2. **测试结构**：
```cpp
#include <benchmark/benchmark.h>
#include "core/math.hpp"

static void BM_Dot3(benchmark::State& state) {
  float a[3] = {1.0f, 2.0f, 3.0f};
  float b[3] = {4.0f, 5.0f, 6.0f};
  
  for (auto _ : state) {
    float result = icehalo::Dot3(a, b);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(BM_Dot3);

BENCHMARK_MAIN();
```

3. **使用Fixture**：
```cpp
class MathBenchmark : public ::benchmark::Fixture {
 public:
  void SetUp(::benchmark::State& state) override {
    // 初始化测试数据
  }
  
 protected:
  float* data_;
};

BENCHMARK_DEFINE_F(MathBenchmark, Dot3)(::benchmark::State& state) {
  for (auto _ : state) {
    // 基准测试代码
  }
}
BENCHMARK_REGISTER_F(MathBenchmark, Dot3);
```

#### 运行基准测试

```bash
# 使用构建脚本
cd cpp
./build.sh -b release

# 或手动运行
cd build/cmake_build
./bench_math  # 或相应的基准测试可执行文件
```

#### 基准测试最佳实践

- **避免优化**：使用 `benchmark::DoNotOptimize()` 防止编译器优化
- **多次运行**：基准测试会自动多次运行取平均值
- **参数化测试**：使用 `state.range(0)` 进行参数化测试
- **结果解读**：关注时间复杂度和实际性能

## 调试技巧

### 日志系统

#### 日志级别

项目提供了日志系统，支持以下级别：
- `DEBUG`: 调试信息
- `VERBOSE`: 详细信息
- `INFO`: 一般信息
- `WARNING`: 警告信息
- `ERROR`: 错误信息

#### 使用日志

```cpp
#include "include/log.hpp"

LOG_DEBUG("Debug message: %d", value);
LOG_VERBOSE("Verbose message");
LOG_INFO("Info message");
LOG_WARNING("Warning message");
LOG_ERROR("Error message: %s", error_str);
```

#### 启用日志

```bash
# 启用详细输出
./build/cmake_install/IceHaloV3 -f config.json -v

# 启用调试输出
./build/cmake_install/IceHaloV3 -f config.json -d
```

### 调试工具

#### GDB/LLDB基本命令

```bash
# 启动调试
lldb ./build/cmake_build/IceHaloV3

# 设置断点
(lldb) breakpoint set --file crystal.cpp --line 100
(lldb) b crystal.cpp:100

# 运行程序
(lldb) run -f config.json

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
valgrind --leak-check=full ./build/cmake_build/IceHaloV3 -f config.json
```

#### 性能分析

```bash
# 使用 perf (Linux)
perf record ./build/cmake_install/IceHaloV3 -f config.json
perf report

# 使用 Instruments (macOS)
instruments -t "Time Profiler" ./build/cmake_install/IceHaloV3 -f config.json
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
   - 使用基准测试定位瓶颈
   - 检查是否使用了Debug版本（应使用Release版本）
   - 使用性能分析工具

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

- [系统架构文档](architecture.md) - 了解系统设计
- [配置文档](configuration.md) - 配置格式说明
- [C接口文档](c_api.md) - C接口使用说明
- [文档索引](README.md) - 所有文档的导航
