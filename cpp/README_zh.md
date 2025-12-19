# C++ 代码

**版本**: V3  
**最后更新**: 2025-12-19

[English version](README.md)

## 文档导航

- [文档索引](doc/README.md) - 所有文档的导航和索引
- [配置文档](doc/configuration.md) - V3配置文件的完整说明
- [系统架构文档](doc/architecture.md) - V3系统架构设计
- [开发指南](doc/developer-guide.md) - 开发指南
- [C接口文档](doc/c_api.md) - C接口使用说明
- [API文档](doc/api/html/) - 自动生成的API文档（需要本地生成）

这是一个冰晕模拟程序，速度快且高效。

## 快速开始

克隆项目后，可以运行构建脚本来构建和安装：

~~~bash
cd ice_halo_sim/cpp
./build.sh -rj release
~~~

如果一切顺利，可执行文件将安装到 `cpp/build/cmake_install`。然后可以这样运行：

~~~bash
./build/cmake_install/IceHaloV3 -f v3_config_example.json
~~~

程序将输出一些信息，以及几张渲染的图片文件。

如果你需要更多详细信息，请继续阅读下面的章节。

## 开始使用

### 构建项目

本项目使用 [CMake](https://cmake.org/) 构建。它依赖于 [OpenCV](https://opencv.org/) 和 [boost](https://www.boost.org/)。在开始构建之前，请确保已安装这些依赖。（实际上它们对核心功能并不关键，我计划移除这些依赖）。

我提供了一个构建脚本来简化操作。
使用 `-h` 可以看到帮助信息：

~~~bash
./build.sh -h
Usage:
  ./build.sh [-tjkrh1] <debug|release|minsizerel>
    Executables will be installed at build/cmake_install
OPTIONS:
  -t:          Build test cases and run test on them.
  -j:          Build in parallel, i.e. use make -j option.
  -k:          Clean temporary building files.
  -b:          Run a benchmarking. It tells how fast the program runs on your computer.
  -v:          Enable verbose log.
  -r:          Use random seed for random number generator. Without this option,
               the program will use a constant value. Thus generate a repeatable result
               (usually together with -1).
  -1:          Use single thread.
  -h:          Show this message.
~~~

注意，debug 版本的可执行文件不会被安装，它们位于 `build/cmake_build`。

我使用 [GoogleTest](https://github.com/google/googletest) 框架进行单元测试。
如果设置了 `-t` 选项，测试用例将被构建并运行。
这在 CI/CD 管道中很有用。

## 配置文件

配置文件包含所有配置。它使用 JSON 格式编写。
我使用 [nlohmann's json](https://github.com/nlohmann/json) 来解析 JSON 文件。

我提供了一个示例配置文件 `v3_config_example.json`。

### 光源

以下是一个元素的示例：

~~~json
"id": 2,
"type": "sun",
"altitude": 20.0,
"azimuth": 0,
"diameter": 0.5,
"wavelength": [ 420, 460, 500, 540, 580, 620 ],
"wl_weight": [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ]
~~~

`light_source` 节描述光源的属性。它可以包含多个元素，对应多个光源。它们通过 `id` 引用。
ID 应该是大于 0 的唯一数字。ID 不必连续递增。

字段 `azimuth`、`altitude` 描述太阳的位置。它们以度为单位，`diameter` 也是如此。

`wavelength` 和 `wl_weight` 描述光源的光谱。
它们是数组，包含你想要在模拟中使用的所有波长。波长决定折射率，其数据来自
[Refractive Index of Crystals](https://refractiveindex.info/?shelf=3d&book=crystals&page=ice)。

**注意**：
- `azimuth` 和 `diameter` 是可选的（有默认值）
- `wavelength` 和 `wl_weight` 数组长度必须相等

### 晶体

以下是一个元素的示例：

~~~json
"id": 3,
"type": "prism",
"shape": {
  "height": 1.3,
  "face_distance": [1, 1, 1, 1, 1, 1]
},
"axis": {
  "zenith": {
    "type": "gauss",
    "mean": 90,
    "std": 0.3
  },
  "roll": {
    "type": "uniform",
    "mean": 0,
    "std": 360
  },
  "azimuth": {
    "type": "uniform",
    "mean": 0,
    "std": 360
  }
}
~~~

`crystal` 节存储模拟中使用的所有晶体。它可以包含多个元素（不同的晶体）。它们通过 `id` 引用。

`zenith`、`roll` 和 `azimuth`（可选）：
这些字段定义晶体的姿态。`zenith` 定义 c 轴方向，`roll` 定义绕 c 轴的旋转。
它们是*分布类型*，可以是标量（表示确定性分布），也可以是元组 (`type`, `mean`, `std`) 描述均匀分布或高斯分布。所有角度都以度为单位。

**默认值**：如果 `axis` 字段不存在，则使用以下默认值：
- `zenith`: 90度（水平方向）
- `azimuth`: 0度
- `roll`: 0度

`type` 和 `shape`：它们描述晶体的形状。
目前有两种晶体类型：`prism` 和 `pyramid`。
每种类型都有自己的形状参数。

  * `prism`（六棱柱）：
    参数 `height`，定义为 `h / a`，其中 `h` 是棱柱高度，`a` 是沿 a 轴（也是程序中的 x 轴）的直径。它是*分布类型*。
    `face_distance` 描述不规则六边形面（稍后描述）。
    **默认值**：
    - `height`: 1.0（如果未指定）
    - `face_distance`: [1, 1, 1, 1, 1, 1]（如果未指定，表示正六边形）
    <img src="doc/figs/hex_prism_01.png" width="400">.

  * `pyramid`（六棱锥）：
    `{upper|lower|prism}_h` 描述各段的高度，见下图。`{upper|lower}_h` 分别表示 `h1 / H1` 和 `h3 / H3`，其中
    `H1` 表示上锥段的最大可能高度，`H3` 类似。`prism_h` 是柱体段的高度比 h/a。
    <img src="doc/figs/hex_pyramid_01.png" width="400">.
    `{upper|lower}_indices` 是
    [Miller index](https://en.wikipedia.org/wiki/Miller_index) 描述
    锥面的方向。
    例如，`[a, b, c]` 表示 Miller index `(a, 0, -a, b)`，其中第三个值 `c` 对应 `-30°`、`90°`、`-150°` 方向。
    对于典型的冰晶面（面编号 13），其 Miller index 是 `(1, 0, -1, 1)`，因此这里参数写成 `[1, 0, 1]`。
    它也可以有 `face_distance` 参数。
    **默认值**：
    - `prism_h`: 必填
    - `upper_h`: 0.0（如果未指定）
    - `lower_h`: 0.0（如果未指定）
    - `upper_indices`: [1, 0, 1]（如果未指定）
    - `lower_indices`: [1, 0, 1]（如果未指定）
    - `face_distance`: [1, 1, 1, 1, 1, 1]（如果未指定）

  * `face_distance`：
    这里的距离表示实际面距离与正六边形距离的比值。正六边形的距离为 `[1, 1, 1, 1, 1, 1]`。
    下图显示了一个不规则六边形，距离为 `[1.1, 0.9, 1.5, 0.9, 1.7, 1.2]`
    <img src="doc/figs/irr_hex_01.png" width="400">.

### 过滤器

以下是两个常见示例：

~~~json
[
  {
    "id": 3,
    "type": "raypath",
    "raypath": [3, 5],
    "symmetry": "P"
  },
  {
    "id": 4,
    "type": "entry_exit",
    "entry": 3,
    "exit": 5,
    "action": "filter_in"
  }
]
~~~

`type`：可以是以下类型之一：`raypath`、`entry_exit`、`direction`、`crystal`、`complex`、`none`。

### 场景

以下是一个示例：

~~~json
"id": 3,
"light_source": 2,
"ray_num": 1000000,
"max_hits": 7,
"scattering": [
  {
    "crystal": [1, 2, 3],
    "prob": 0.2
  },
  {
    "crystal": [2, 3],
    "proportion": [20, 100],
    "filter": [2, 1]
  }
]
~~~

`scattering` 数组定义了多晶散射配置。每个元素可以包含：
- `crystal`: 晶体ID数组（必填）
- `proportion`: 比例数组，长度必须等于 `crystal` 数组长度（可选）
- `prob`: 概率值，用于多散射（可选）
- `filter`: 过滤器ID数组，长度必须等于 `crystal` 数组长度（可选，使用 -1 表示无过滤器）

**注意**：
- `ray_num` 可以为 -1，表示自动计算光线数量
- `max_hits` 定义光线与晶体表面的最大碰撞次数

### 渲染

以下是一个示例：

~~~json
"id": 3,
"lens": {
  "type": "linear",
  "fov": 40
},
"resolution": [1920, 1080],
"view": {
  "azimuth": -50,
  "elevation": 30,
  "roll": 0,
  "distance": 8
},
"visible": "upper",
"background": [0, 0, 0],
"ray": [1, 1, 1],
"opacity": 0.8,
"grid": {
  "central": [
    {
      "value": 22,
      "color": [1, 1, 1],
      "opacity": 0.4,
      "width": 1.2
    }
  ],
  "elevation": [],
  "outline": true
}
~~~

`view`：描述相机姿态。

`lens`：镜头类型，可以是以下值之一：`fisheye`、`linear`、`dual_fisheye_equidistant`、`dual_fisheye_equiarea`、`rectangular`。
可以使用 `fov`（视场角，度）或 `f`（焦距，mm）来指定。如果使用 `f`，程序会自动计算对应的 `fov`。

**默认值**：
- `view` 的各个字段（`azimuth`, `elevation`, `roll`, `distance`）都有默认值，如果未指定则使用默认值
- `visible`: "upper"（如果未指定）
- `background`: [0, 0, 0]（如果未指定）
- `ray`: [1, 1, 1]（如果未指定）
- `opacity`: 1.0（如果未指定）
- `lens_shift`: [0, 0]（如果未指定）

### 项目

没什么复杂的。它只是保持对场景和渲染器的引用。

## V3 版本说明

V3 是当前主要开发的版本，相对旧版本有较大重构：

### 架构改进

- **服务器-消费者模式**：采用服务器-消费者架构，支持多线程并行处理
- **模块化配置系统**：配置系统更加模块化和灵活
- **批量处理**：支持多场景、多渲染器的批量处理

### 程序入口

**旧版本入口**（计划废弃）：
- `trace_main.cpp` → `IceHaloTrace`: 仅执行光线追踪
- `render_main.cpp` → `IceHaloRender`: 仅渲染已有数据
- `endless_main.cpp` → `IceHaloEndless`: 循环执行追踪-渲染

**V3 入口**（推荐使用）：
- `main_v3.cpp` → `IceHaloV3`: C++ 接口的主程序
- `main_v3_c.c` → `IceHaloV3C`: C 接口的封装程序
- `server/cserver.cpp` → `IceHaloLibV3`: 静态库，供 C 接口使用

### 配置格式变化

V3 版本的配置格式与旧版本不同：

- **光源配置**：从 `sun` 改为 `light_source` 数组
- **晶体配置**：从 `type: "HexPrism"` 改为 `type: "prism"`，参数结构从 `parameter` 改为 `shape`
- **晶体方向**：从 `axis`（天顶角）改为 `zenith`、`azimuth`、`roll`
- **多散射配置**：从 `multi_scatter` 改为 `scene.scattering` 数组
- **新增配置节**：`filter`（过滤器）、`scene`（场景）、`project`（项目）

### 命名空间隔离

- V3 代码主要在 `icehalo::v3` 命名空间
- 旧代码在 `icehalo` 命名空间
- 两者可以共存，便于逐步迁移

## TODO 列表

* 为这些代码编写一个（Web）GUI。
