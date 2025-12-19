# V3 系统架构文档

**版本**: V3  
**最后更新**: 2025-12-19

本文档描述 Ice Halo Simulation 项目 V3 版本的系统架构设计。

## 概述

V3 版本是当前主要开发的版本，采用了服务器-消费者（Server-Consumer）架构模式，支持多线程并行处理和批量处理能力。

### 设计目标

1. **高性能**：通过多线程并行处理提升模拟速度
2. **可扩展性**：支持多场景、多渲染器的批量处理
3. **模块化**：清晰的模块划分，便于维护和扩展
4. **灵活性**：灵活的配置系统，支持复杂的模拟场景

### 架构特点

- **服务器-消费者模式**：采用生产者-消费者模式，实现模拟和渲染的解耦
- **多线程并行**：默认使用4个模拟器线程并行处理
- **队列系统**：使用线程安全的队列进行数据传递
- **批量处理**：支持在一个配置文件中定义多个场景和渲染器

## V3 架构设计

### 整体架构

V3 版本采用三层架构：

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

### 核心组件

#### 1. Server（服务器）

`Server` 类是 V3 架构的核心，负责：

- 配置管理：解析和管理 JSON 配置文件
- 任务调度：将项目配置分发到模拟器
- 结果收集：从消费者收集处理结果
- 生命周期管理：控制服务器启动和停止

**主要接口**：
- `CommitConfig()`: 提交配置
- `GetResults()`: 获取处理结果（非阻塞）
- `IsIdle()`: 检查是否空闲
- `Stop()` / `Terminate()`: 停止服务器

#### 2. Simulator（模拟器）

`Simulator` 负责光线追踪模拟：

- 从场景队列获取场景配置
- 执行光线追踪计算
- 将模拟结果放入数据队列

**特点**：
- 多实例并行：默认创建4个 Simulator 实例
- 独立线程：每个 Simulator 运行在独立线程中
- 批量处理：可以处理多个场景

#### 3. Consumer（消费者）

`IConsume` 接口定义了消费者的抽象：

- `Consume()`: 消费模拟数据
- `GetResult()`: 获取处理结果

**实现类**：
- `RenderConsumer`: 渲染消费者，将模拟数据渲染为图像
- `StatsConsumer`: 统计消费者，收集统计信息

#### 4. 队列系统

使用线程安全的队列进行数据传递：

- `proj_queue_`: 项目配置队列
- `scene_queue_`: 场景配置队列（共享）
- `data_queue_`: 模拟数据队列（共享）

### 数据流

#### 配置加载流程

```
JSON配置文件
    ↓
ConfigManager (解析)
    ↓
各种Config对象 (LightSourceConfig, CrystalConfig, etc.)
    ↓
ProjConfig (组合场景和渲染器)
    ↓
proj_queue_ (项目队列)
```

#### 模拟流程

```
ProjConfig (从proj_queue_)
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

#### 渲染流程

```
SimData (从data_queue_)
    ↓
ConsumeData() (分发到Consumer)
    ↓
RenderConsumer (渲染)
    ↓
RenderResult (图像数据)
    ↓
GetResults() (返回结果)
```

#### 完整流程

```
配置文件
  ↓
Server::CommitConfig()
  ↓
ConfigManager (解析配置)
  ↓
proj_queue_ (项目队列)
  ↓
GenerateScene() (生成场景)
  ↓
scene_queue_ (场景队列) ──→ Simulator × 4 (并行)
  ↓                                    ↓
data_queue_ (数据队列) ←──────────────┘
  ↓
ConsumeData() (分发)
  ↓
Consumer (Render/Stats)
  ↓
Result (结果)
  ↓
Server::GetResults()
```

### 线程模型

V3 版本使用多线程架构：

1. **主线程**：运行 `main()` 函数，调用 `Server::CommitConfig()` 和 `Server::GetResults()`
2. **模拟器线程**：默认4个线程，每个运行一个 `Simulator::Run()`
3. **场景生成线程**：`GenerateScene()` 线程，从项目队列生成场景
4. **数据消费线程**：`ConsumeData()` 线程，从数据队列分发数据到消费者

**线程同步**：
- 使用 `std::mutex` 保护共享资源
- 使用 `std::condition_variable` 进行线程间通信
- 使用原子变量（`std::atomic_bool`）控制线程状态

## 模块说明

### config 模块

**职责**：配置解析和管理

**主要类**：
- `ConfigManager`: 统一配置管理器，管理所有配置对象
- `LightSourceConfig`: 光源配置
- `CrystalConfig`: 晶体配置
- `FilterConfig`: 过滤器配置
- `RenderConfig`: 渲染配置
- `SceneConfig`: 场景配置
- `ProjConfig`: 项目配置

**依赖关系**：
- 依赖 `nlohmann/json` 进行 JSON 解析
- 依赖 `core/math` 中的 `Distribution` 类型

### core 模块

**职责**：核心算法实现

**主要类**：
- `Crystal`: 晶体几何和操作
- `Simulator`: 光线追踪模拟器
- `Filter`: 过滤器系统
- `RayPath`: 光线路径管理
- `Optics`: 光学计算（折射、反射）
- `Geo3D`: 3D 几何计算
- `Math`: 数学工具（向量、矩阵等）

**关键算法**：
- 光线与晶体相交算法
- 折射/反射计算
- 光线路径追踪
- 多散射处理

### server 模块

**职责**：服务器架构实现

**主要类**：
- `Server`: 服务器公开接口
- `ServerImpl`: 服务器实现
- `IConsume`: 消费者接口
- `RenderConsumer`: 渲染消费者
- `StatsConsumer`: 统计消费者

**特点**：
- 线程安全的队列系统
- 多线程并行处理
- 非阻塞的结果获取

### process 模块

**职责**：处理流程编排

**主要类**：
- `Simulation`: 模拟流程（旧版，V3中已整合到Simulator）
- `Render`: 渲染流程（旧版，V3中已整合到Consumer）

**注意**：V3 版本中，这些流程已整合到 server 模块中。

### context 模块

**职责**：上下文管理

**主要类**：
- `CameraContext`: 相机上下文
- `CrystalContext`: 晶体上下文
- `FilterContext`: 过滤器上下文
- `RenderContext`: 渲染上下文
- `SunContext`: 太阳光源上下文

**用途**：为旧版代码提供上下文管理，V3 版本中部分功能已整合到配置系统。

### io 模块

**职责**：文件 I/O 和序列化

**主要类**：
- `File`: 文件操作
- `Serialize`: 序列化（二进制格式）

### util 模块

**职责**：工具类

**主要类**：
- `ThreadingPool`: 线程池（旧版使用）
- `ObjectPool`: 对象池
- `Logger`: 日志系统
- `Queue`: 线程安全队列

## 程序入口说明

### 旧版本入口（计划废弃）

以下入口计划废弃，不建议新项目使用：

1. **`trace_main.cpp` → `IceHaloTrace`**
   - 功能：仅执行光线追踪
   - 状态：计划废弃
   - 不推荐使用的原因：功能单一，不支持批量处理

2. **`render_main.cpp` → `IceHaloRender`**
   - 功能：仅渲染已有数据
   - 状态：计划废弃
   - 不推荐使用的原因：需要先运行追踪程序，流程繁琐

3. **`endless_main.cpp` → `IceHaloEndless`**
   - 功能：循环执行追踪-渲染
   - 状态：计划废弃
   - 不推荐使用的原因：不支持多线程并行，性能较差

### V3 入口（推荐使用）

以下入口是 V3 版本推荐的入口：

1. **`main_v3.cpp` → `IceHaloV3`**
   - 功能：C++ 接口的主程序
   - 使用场景：C++ 项目集成，命令行使用
   - 推荐度：⭐⭐⭐⭐⭐
   - 示例：
     ```bash
     ./build/cmake_install/IceHaloV3 -f v3_config_example.json
     ```

2. **`main_v3_c.c` → `IceHaloV3C`**
   - 功能：C 接口的封装程序
   - 使用场景：需要 C 接口的项目
   - 推荐度：⭐⭐⭐⭐
   - 示例：
     ```bash
     ./build/cmake_install/IceHaloV3C -f v3_config_example.json
     ```

3. **`server/cserver.cpp` → `IceHaloLibV3`**
   - 功能：静态库，供 C 接口使用
   - 使用场景：作为库集成到其他项目
   - 推荐度：⭐⭐⭐⭐⭐
   - API：参见 `include/cserver.h`

## 与旧版本对比

### 架构变化

| 方面 | 旧版本 | V3 版本 |
|------|--------|---------|
| 架构模式 | 单线程顺序处理 | 服务器-消费者模式，多线程并行 |
| 配置系统 | 单一配置结构 | 模块化配置系统 |
| 批量处理 | 不支持 | 支持多场景、多渲染器 |
| 程序入口 | 3个独立入口 | 统一的 Server 接口 |

### 命名空间隔离

- **旧代码**：`icehalo` 命名空间
- **V3 代码**：`icehalo::v3` 命名空间
- **共存策略**：两者可以共存，便于逐步迁移

### 配置格式变化

详见 `README_zh.md` 中的"V3 版本说明"章节。

### 性能改进

- **多线程并行**：默认4个模拟器线程，显著提升处理速度
- **批量处理**：一次配置可以处理多个场景，减少配置开销
- **队列系统**：高效的线程安全队列，减少锁竞争

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

- [README_zh.md](../README_zh.md): 用户文档
- [configuration.md](configuration.md): 配置文档（待完成）
