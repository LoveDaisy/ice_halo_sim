[English version](architecture.md)

# 系统架构文档

本文档描述 Lumice 项目的系统架构设计。

## 概述

项目采用服务器-消费者（Server-Consumer）架构模式，支持多线程并行光线追踪和批量处理。

### 设计目标

1. **高性能**：通过多线程并行处理提升模拟速度
2. **可扩展性**：支持多渲染器并行处理
3. **模块化**：清晰的模块划分，便于维护和扩展
4. **灵活性**：灵活的配置系统，支持复杂的模拟场景

### 架构特点

- **服务器-消费者模式**：采用生产者-消费者模式，实现模拟和渲染的解耦
- **多线程并行**：默认使用 4 个模拟器线程并行处理
- **队列系统**：使用线程安全的队列进行数据传递
- **单项目模式**：每次提交一个项目配置（包含一个场景和多个渲染器）

## 整体架构

采用三层架构：

```
┌─────────────────────────────────────────┐
│           配置层 (Config)                │
│  ConfigManager, 各种Config类            │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│         服务器层 (Server)                │
│  Server, ServerImpl, 队列系统            │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│       处理层 (Processing)                │
│  Simulator, Consumer, Render            │
└─────────────────────────────────────────┘
```

## 核心组件

### 1. Server（服务器）

`Server` 类是架构的核心入口，采用 PIMPL 模式将实现隐藏在 `ServerImpl` 中。负责：

- 配置管理：解析和管理 JSON 配置文件
- 任务调度：将项目配置分发到模拟器
- 结果收集：从消费者收集处理结果
- 生命周期管理：控制服务器启动和停止

**主要接口**：
- `CommitConfig(const std::string&)`: 提交 JSON 配置字符串
- `CommitConfigFromFile(const std::string&)`: 从文件加载配置
- `GetRenderResults()`: 获取渲染结果（非阻塞）
- `GetStatsResult()`: 获取统计结果（非阻塞）
- `IsIdle()`: 检查是否空闲
- `Stop()` / `Terminate()`: 停止服务器

**错误处理**：
```cpp
struct Error {
  ErrorCode code;   // kSuccess, kInvalidJson, kInvalidConfig, kMissingField, ...
  std::string message;
  std::string field;
};
```

### 2. Simulator（模拟器）

`Simulator` 负责光线追踪模拟：

- 从场景队列获取场景配置
- 执行光线追踪计算
- 将模拟结果放入数据队列

**特点**：
- 多实例并行：默认创建 4 个 Simulator 实例
- 独立线程：每个 Simulator 运行在独立线程中
- 独立随机数生成器：每个实例使用不同的种子

### 3. Consumer（消费者）

`IConsume` 接口定义了消费者的抽象：

- `Consume(const SimData&)`: 消费模拟数据
- `GetResult()`: 获取处理结果

**实现类**：
- `RenderConsumer`: 渲染消费者，将模拟数据渲染为图像
- `StatsConsumer`: 统计消费者，收集光线数量、晶体数量等统计信息
- `ShowRayInfoConsumer`: 光线信息消费者，用于调试

### 4. 队列系统

使用线程安全的 `Queue<T>` 模板进行数据传递，基于 `std::mutex` 和 `std::condition_variable` 实现阻塞式获取：

- `scene_queue_`: 场景配置队列（多个 Simulator 共享）
- `data_queue_`: 模拟数据队列（多个 Simulator 共享）

## 数据流

### 配置加载流程

```
JSON配置文件
    ↓
ConfigManager (解析)
    ↓
各种Config对象 (LightSourceConfig, CrystalConfig, etc.)
    ↓
ProjConfig (组合场景和渲染器)
    ↓
CommitConfig() (创建Consumer, 启动线程)
```

### 模拟流程

```
ProjConfig (从config_manager_.project_)
    ↓
GenerateScene() (生成SceneConfig)
    ↓
scene_queue_ (场景队列，共享)
    ↓
Simulator (多个并行)
    ↓
SimData (模拟数据)
    ↓
data_queue_ (数据队列，共享)
```

### 渲染流程

```
SimData (从data_queue_)
    ↓
ConsumeData() (分发到Consumer)
    ↓
RenderConsumer / StatsConsumer
    ↓
RenderResult / StatsResult
    ↓
GetRenderResults() / GetStatsResult()
```

### 完整流程

```
配置文件
  ↓
Server::CommitConfig()
  ↓
ConfigManager (解析配置)
  ↓
创建 Consumer (Render/Stats)
  ↓
Start() → GenerateScene() (生成场景)
  ↓
scene_queue_ (场景队列) ──→ Simulator × 4 (并行)
  ↓                                    ↓
data_queue_ (数据队列) ←──────────────┘
  ↓
ConsumeData() (分发, consumer_mutex_ 保护)
  ↓
Consumer (Render/Stats)
  ↓
Result (结果, consumer_mutex_ 保护)
  ↓
Server::GetResults()
```

## 线程模型

使用多线程架构：

1. **主线程**：运行 `main()` 函数，调用 `Server::CommitConfig()` 和结果获取接口
2. **模拟器线程**：默认 4 个线程，每个运行一个 `Simulator::Run()`
3. **场景生成线程**：`GenerateScene()` 线程，从项目配置生成场景并放入 `scene_queue_`
4. **数据消费线程**：`ConsumeData()` 线程，从数据队列分发数据到消费者

**线程同步**：
- 使用 `std::mutex` 保护共享资源
- `consumer_mutex_`：保护 `consumers_` 的 `Consume()` 和 `GetResult()` 调用，防止数据消费线程与主线程之间的竞争
- 使用 `std::condition_variable` 进行线程间通信
- 使用原子变量（`std::atomic_bool`）控制线程状态

## 模块说明

### config 模块

**职责**：配置解析和管理

**主要类**：
- `ConfigManager`: 统一配置管理器，持有所有配置对象的映射（`std::map<IdType, *Config>`）
- `LightSourceConfig`: 光源配置
- `CrystalConfig`: 晶体配置
- `FilterConfig`: 过滤器配置
- `RenderConfig`: 渲染配置
- `SceneConfig`: 场景配置（包含散射设置 `ScatteringSetting`、多次散射信息 `MsInfo`）
- `ProjConfig`: 项目配置（组合场景和渲染器引用）
- `SimData`: 模拟输出数据结构（包含 `RayBuffer` 和晶体列表）

**依赖**：
- 依赖 `nlohmann/json` 进行 JSON 解析
- 依赖 `core/math` 中的 `Distribution` 类型

### core 模块

**职责**：核心物理算法实现

**主要类**：
- `Crystal`: 晶体几何表示（三角面片网格），支持创建六棱柱（`CreatePrism`）和六棱锥（`CreatePyramid`）
- `Simulator`: 光线追踪模拟器，从队列获取场景、执行光线追踪、输出 `SimData`
- `Filter`: 过滤器系统，支持光线路径、入射/出射面、方向等多种过滤方式
- `RayPath` / `RaySeg`: 光线路径和光线段数据结构
- `Optics`: 光学计算（折射、反射、菲涅尔系数）
- `Geo3D`: 3D 几何计算（三角形相交、法向量等）
- `Math`: 数学工具（向量运算、矩阵、随机数分布等）

### server 模块

**职责**：服务器架构实现

**主要类**：
- `Server`: 服务器公开接口（PIMPL 模式）
- `ServerImpl`: 服务器内部实现，管理线程、队列和消费者
- `IConsume`: 消费者抽象接口
- `RenderConsumer`: 渲染消费者，实现各种镜头投影算法
- `StatsConsumer`: 统计消费者
- `ShowRayInfoConsumer`: 光线信息消费者
- `c_api.cpp`: C API 封装实现

**特点**：
- 线程安全的队列系统
- 多线程并行处理
- 非阻塞的结果获取

### util 模块

**职责**：工具和基础设施

**主要类**：
- `Queue<T>`: 线程安全的阻塞队列模板，基于 `std::condition_variable`
- `ThreadingPool`: 线程池，支持范围分步（range-step）/ 范围分片（range-slice）任务提交
- `log.hpp`: spdlog 薄封装，提供 `LOG_DEBUG` / `LOG_INFO` 等宏和 `GetLogger()` 命名 logger
- `ArgParser`: 命令行参数解析器
- `json_util.hpp`: JSON 辅助宏
- `color_data.hpp`: CIE 色彩匹配函数数据

### gui 模块

**职责**：图形用户界面应用

**主要组件**：
- `app.cpp/hpp`：主应用循环、面板渲染、全局状态管理（从 `main.cpp` 提取以支持测试）
- `gui_state.hpp`：`GuiState` 结构体，定义完整的 GUI 状态模型（晶体、过滤器、渲染器、场景设置）
- `panels.cpp/hpp`：ImGui 面板实现（Crystal、Render、Filter、Scene 四个标签页）
- `crystal_renderer.cpp/hpp`：基于 OpenGL FBO 的晶体 3D 预览（线框、隐藏线、着色三种风格）
- `preview_renderer.cpp/hpp`：等距圆柱投影纹理预览，带镜头投影 shader
- `file_io.cpp/hpp`：`.lmc` 项目文件格式读写（二进制头 + JSON + 可选 PNG 纹理）
- `main.cpp`：GUI 入口，GLFW/OpenGL 初始化

**关键设计决策**：
- GUI 逻辑位于 `lumice::gui` 命名空间，使用全局状态（`g_state`、`g_preview` 等）
- 通过 `lumice.h` C API 与模拟核心通信（与 CLI 使用相同接口）
- `lumice_gui_obj` OBJECT library 在 `LumiceGUI` 和 `LumiceGUITests` 间共享编译产物

**依赖**：
- Dear ImGui v1.91.8-docking（即时模式 GUI）
- GLFW 3.4（窗口管理）
- nfd v1.2.1（原生文件对话框）
- OpenGL 3.2 Core Profile
- stb（`.lmc` 纹理的 PNG 读写）

### include 模块

**职责**：公共 C API 头文件

- `lumice.h`: 对外暴露的 C 接口头文件，使用不透明指针（opaque pointer）模式

## 程序入口

**`main.c` → `Lumice`**（CLI）
- 功能：命令行模拟程序
- 通过 `lumice.h` 公共 API 与核心库交互
- 使用方式：
  ```bash
  ./build/cmake_install/Lumice -f examples/config_example.json
  ```

**`src/gui/main.cpp` → `LumiceGUI`**（GUI，需要 `-g` 构建选项）
- 功能：图形界面，用于交互式模拟配置和预览
- 提供晶体 3D 预览、渲染预览、参数编辑和 `.lmc` 文件管理
- 使用方式：
  ```bash
  ./build/cmake_install/LumiceGUI
  ```

构建目标：
- `lumice`: 核心静态/共享库（默认静态，`-DBUILD_SHARED_LIBS=ON` 编译为共享库）
- `Lumice`: CLI 可执行文件，链接 `lumice` 库
- `LumiceGUI`: GUI 可执行文件（`-DBUILD_GUI=ON` 时构建），链接 `lumice_gui_obj` 和 `lumice`
- `LumiceGUITests`: GUI 测试可执行文件（同时 `-DBUILD_GUI=ON` 和 `-DBUILD_TESTING=ON` 时构建）

## C API

公共 API 通过 C 接口（`lumice.h`）暴露，使用不透明指针模式。大多数 API 返回 `LUMICE_ErrorCode` 错误码，实际输出通过指针参数传递：

```c
// 服务器生命周期
LUMICE_Server* LUMICE_CreateServer(void);
void LUMICE_DestroyServer(LUMICE_Server* server);

// 日志
void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level);

// 配置（返回 LUMICE_ErrorCode）
LUMICE_ErrorCode LUMICE_CommitConfig(LUMICE_Server* server, const char* config_str);
LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename);

// 结果获取（数组+哨兵模式，返回 LUMICE_ErrorCode）
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);
LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count);

// 状态与控制
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out);
void LUMICE_StopServer(LUMICE_Server* server);
```

详细用法参见 [C 接口使用文档](c_api_zh.md)。

## 依赖关系

所有依赖通过 [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake) 自动管理：

| 依赖 | 版本 | 用途 |
|------|------|------|
| [nlohmann/json](https://github.com/nlohmann/json) | v3.10.5 | JSON 配置解析 |
| [spdlog](https://github.com/gabime/spdlog) | v1.15.0 | 日志系统 |
| [tl-expected](https://github.com/TartanLlama/expected) | v1.1.0 | C++17 `expected<T,E>` |
| [GoogleTest](https://github.com/google/googletest) | v1.15.2 | 单元测试 |
| [Dear ImGui](https://github.com/ocornut/imgui) | v1.91.8-docking | GUI（启用 `-g` 时）|
| [GLFW](https://www.glfw.org/) | 3.4 | 窗口管理（启用 `-g` 时）|
| [nfd](https://github.com/btzy/nativefiledialog-extended) | v1.2.1 | 原生文件对话框（启用 `-g` 时）|
| [imgui_test_engine](https://github.com/ocornut/imgui_test_engine) | v1.91.8 | GUI 测试（同时启用 `-g` + `-t` 时）|

## 扩展点

### 添加新的消费者

1. 继承 `IConsume` 接口
2. 实现 `Consume()` 方法处理数据
3. 实现 `GetResult()` 方法返回结果
4. 在 `ServerImpl` 中注册消费者

### 添加新的配置类型

1. 定义配置结构体
2. 实现 `to_json()` 和 `from_json()` 函数
3. 在 `ConfigManager` 中添加管理逻辑

### 自定义模拟器

1. 继承或修改 `Simulator` 类
2. 实现自定义的光线追踪逻辑
3. 在 `ServerImpl` 中使用自定义模拟器

## 注意事项

1. **线程安全**：所有共享资源都需要适当的同步机制
2. **内存管理**：注意队列中数据的生命周期
3. **错误处理**：配置解析错误需要适当处理
4. **性能调优**：模拟器线程数可以根据硬件调整

## 相关文档

- [README](../README_zh.md) - 用户文档
- [配置文档](configuration_zh.md) - 配置格式说明
- [开发指南](developer-guide_zh.md) - 开发指南
- [C接口文档](c_api_zh.md) - C接口使用说明
- [文档索引](README_zh.md) - 所有文档的导航
