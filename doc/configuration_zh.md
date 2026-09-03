[English version](configuration.md)

# 配置文档

本文档详细说明配置文件格式、各配置项的含义、默认值以及配置验证规则。

## 配置概述

使用 JSON 格式的配置文件，配置文件包含以下主要部分：

- `crystal`: 晶体定义数组
- `filter`: 过滤器定义数组
- `scene`: 场景定义（单个对象），内含光源配置
- `render`: 渲染器定义数组

**重要提示**：
- 样例配置文件（`examples/config_example.json`）并未穷举所有合法的配置写法
- 大量配置参数具有默认值，如果不写会使用默认值
- **必须参考代码实现**来了解完整的配置逻辑和默认值
- 本文档基于代码实现提取了所有默认值信息

## 配置项详细说明

### crystal（晶体配置）

晶体配置定义模拟中使用的晶体形状和朝向分布。

> **坐标系与旋转约定**：世界帧、局部帧、方位角符号约定，以及 `axis.{zenith, azimuth, roll}` 背后
> 的旋转链语义，详见 [`coordinate-convention_zh.md`](coordinate-convention_zh.md)。早期版本的配置在
> 当前 chain 下渲染朝向**会发生确定性变化**（除非 `axis` 为全向均匀采样）。

#### 基本结构

```json
{
  "id": <唯一标识符>,
  "type": "prism" | "pyramid",
  "shape": { ... },
  "axis": { ... }
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `id` | 整数 | 是 | - | 唯一标识符，必须大于0 |
| `type` | 字符串 | 是 | - | 晶体类型："prism" 或 "pyramid" |
| `shape` | 对象 | 是 | - | 形状参数，见下方说明 |
| `axis` | 对象 | 否 | 见下方 | 晶体方向分布 |

#### axis（方向分布）默认值

这里有两种不同的"缺失"，行为并不相同——不要把其中一种当成另一种的简写。

**`axis` 整个字段不存在** ⇒ 晶体取一个固定朝向，完全不随机：

```json
{
  "zenith": 0.0,
  "azimuth": 0.0,
  "roll": 0.0
}
```

**`axis` 存在但省略了 `azimuth` / `roll`** ⇒ 被省略的那个轴在 0–360° 内均匀自由旋转。
例如只写 `"axis": {"zenith": 30}` 等价于：

```json
{
  "zenith": 30.0,
  "azimuth": { "type": "uniform", "mean": 180.0, "std": 360.0 },
  "roll": { "type": "uniform", "mean": 180.0, "std": 360.0 }
}
```

（`mean`/`std` 是所有分布类型统一的落盘键名；对 `uniform` 而言，它们分别是区间中点和
**完整**宽度，不是统计学意义上的均值/标准差。）

`zenith` 没有这种兜底：只要写了 `axis`，`zenith` 就是必填的——见下方
[必填字段验证](#必填字段验证)。

#### prism（六棱柱）类型

**shape 结构**：

```json
{
  "height": <数值或分布>,
  "face_distance": [<6个数值或分布>],
  "sync_group": { "height": <int>, "face_distance": [<6个int>] }
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `height` | 数值/分布 | 否 | 1.0 | 高度比 h/a，h是棱柱高度，a是底面直径 |
| `face_distance` | 数组 | 否 | [1,1,1,1,1,1] | 6个面的距离比，正六边形为[1,1,1,1,1,1] |
| `sync_group` | 对象 | 否 | 全部独立 | 形状标量 sync group，详见下方[形状标量 Sync Group](#形状标量-sync-group) |

`face_distance` 允许负值：负值会被接受并按其符号参与几何构造（不再被静默折叠为
正值）。构造后的网格会做实体性校验——若不满足闭合流形条件，该晶体会被拒绝
（丢弃，不贡献能量），而不是被静默接受。

**示例**：

```json
{
  "id": 1,
  "type": "prism",
  "shape": {
    "height": 1.3,
    "face_distance": [1, 1, 1, 1, 1, 1]
  }
}
```

#### pyramid（六棱锥）类型

**shape 结构**：

```json
{
  "prism_h": <数值或分布>,
  "upper_h": <数值或分布>,
  "lower_h": <数值或分布>,
  "upper_indices": [<3个整数>],
  "lower_indices": [<3个整数>],
  "face_distance": [<6个数值或分布>],
  "sync_group": {
    "prism_h": <int>, "upper_h": <int>, "lower_h": <int>, "face_distance": [<6个int>]
  }
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `prism_h` | 数值/分布 | 是 | - | 棱柱段高度比 |
| `upper_h` | 数值/分布 | 否 | 0.0 | 上锥段相对高度——详见下方 [pyramid 形状合法性](#pyramid-形状合法性) |
| `lower_h` | 数值/分布 | 否 | 0.0 | 下锥段相对高度——详见下方 [pyramid 形状合法性](#pyramid-形状合法性) |
| `upper_indices` | 整数数组 | 否 | [1,0,1] | 上锥段Miller指数 |
| `lower_indices` | 整数数组 | 否 | [1,0,1] | 下锥段Miller指数 |
| `face_distance` | 数组 | 否 | [1,1,1,1,1,1] | 6个面的距离比 |
| `sync_group` | 对象 | 否 | 全部独立 | 形状标量 sync group，详见下方[形状标量 Sync Group](#形状标量-sync-group) |

`face_distance` 允许负值：构造与拒绝语义与上方 prism 类型一致。

**示例**：

```json
{
  "id": 5,
  "type": "pyramid",
  "shape": {
    "prism_h": 1.2,
    "upper_h": 0.1,
    "lower_h": 0.5,
    "upper_indices": [2, 0, 3]
  }
}
```

#### 形状标量 Sync Group

默认情况下，晶体的每个可随机化形状标量——`height`（prism）、`prism_h` /
`upper_h` / `lower_h`（pyramid），以及两种类型都有的 6 个 `face_distance`——
各自独立抽样。即使把 6 个 `face_distance` 的均值都配成相等，每次抽样仍会各自
被扰动成互不相等的不等边六边形：均值对称不代表单次抽样对称。

`sync_group` 把一个晶体的若干形状标量划入**同一组**，使它们在该晶体实例上
**共享同一次随机抽样**，而不是各自独立抽样。这是表达"每次抽样都严格对称"的
habit（而不仅是均值对称）的唯一方式——最典型的场景是三方（C3）六棱柱：面
0/2/4 须逐样本相等，面 1/3/5 须逐样本相等。

**schema**——一个可选对象，键名与 shape 对象里该标量自身的 JSON 字段名相同；
`face_distance` 取 6 元素整数数组（与 `face_distance` 本身的写法一致），而不是
标量：

```json
"sync_group": {
  "height": 1,                        // 仅 prism
  "prism_h": 1, "upper_h": 2, "lower_h": 2,   // 仅 pyramid
  "face_distance": [1, 2, 1, 2, 1, 2]         // 两种类型都有
}
```

- **`0` = 独立**（默认值——缺省 `sync_group` 键，或缺省某个标量的条目，均等价于
  全部独立，与 sync group 出现之前的行为完全一致）。**`1..N` = 组号**：组号相同
  的标量共享一次抽样。组号只需要表达一个*划分*，不要求连续或已排序——
  `{1,2,1,2,1,2}` 与 `{2,1,2,1,2,1}` 描述的是同一个划分，加载时会被规范化为同一
  个标准形（按固定标量顺序——即 RNG 抽取顺序 `height`/`upper_h`/`prism_h`/
  `lower_h`/`face_distance[0..5]`，而非上面结构体的字段声明顺序——首次出现即
  重编号）。只剩一个成员的组会退化回独立（`0`）。
- **共享的是同一个值，不是同一个随机变量**：一个组每个晶体实例只抽一次，组内
  全部成员拿到同一个原始值；不是统计意义上"相关"，而是逐实例位级相等。
- **组的分布取自其 leader**：leader 是该组在 RNG 抽取顺序上的第一个成员（对
  `face_distance` 而言即组内下标最小的面）。其余成员各自声明的分布会在加载时
  被 leader 的分布覆盖；若某个非 leader 成员声明了不同的分布，该声明会被丢弃
  并记一条 WARN 日志——既不静默丢弃也不报错拒绝。
- **跨类组（高度标量与面标量同组）机制上合法，但存在一个已记录的不对称**：
  高度标量在使用前会取 `abs()`（负高度没有独立于朝向之外的物理含义），而
  `face_distance` 保持带符号（负的面距离是合法的、跨越原点的平面偏移，见上文）。
  这类组共享同一个*原始*抽样值，但高度成员取其绝对值而面成员保留符号——两者
  并非数值相等，只是绝对值相等。机制不阻止这类组，也不做量纲可通约性校验。
- **物理边界，如实说明**：`height` / `face_distance` 随机化只沿法向平移面，
  从不旋转面的法向方向（背景见
  [`geometry-randomization-value-and-measurement.md`](geometry-randomization-value-and-measurement.md)）。
  sync group 继承这一边界：它本身不会在新的角度产生光晕特征。它买到的是
  **habit 保真度**——一个逐个晶体都真正对称的 ensemble（例如真正的三方 C3
  群体，而不只是均值 C3），这在 roll **不是**均匀随机时才真正显形（Parry /
  Lowitz / 固定 roll 场景；roll 若做完整 360° 均匀随机，ensemble 平均本就会
  统计性恢复六重对称，与单个晶体是否对称无关）；此外，把对面面对（`i` 与
  `i+3`）设为同组还有一个副作用——两者之和的符号结构随之固定，可在强随机化下
  降低几何合法性拒绝率。

**C3 三方晶体示例**（已实测验证：全部面用相同均值、相同标准差的高斯分布，
配两个交替的 sync group——与单元测试
`SyncGroupPreview.C3GroupingCollapsesRandomDrawsToTwo` 覆盖的场景完全一致）：

```json
{
  "id": 1,
  "type": "prism",
  "shape": {
    "height": 1.3,
    "face_distance": [
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 }
    ],
    "sync_group": {
      "face_distance": [1, 2, 1, 2, 1, 2]
    }
  }
}
```

每个采样出的晶体现在只有两个不同的面距离（面 0/2/4 共享一次抽样，面 1/3/5
共享另一次），而不是六个各自独立的值——每次抽样都是严格的三方 habit，而不是
仅在均值上如此。

#### 分布类型（Distribution）

许多参数支持分布类型，可以是：

1. **标量值**：确定性值
   ```json
   "height": 1.3
   ```

2. **分布对象**：均匀分布或高斯分布
   ```json
   "height": {
     "type": "gauss",
     "mean": 1.3,
     "std": 0.2
   }
   ```
   或
   ```json
   "height": {
     "type": "uniform",
     "mean": 0.5,
     "std": 0.4
   }
   ```

**分布类型说明**：

| 类型 | `mean` | `std` | 说明 |
|------|--------|-------|------|
| `gauss` | 中心值（度） | 标准差（度） | 高斯分布，用于稳定晶体取向 |
| `uniform` | 中心值（度） | 全范围宽度（度） | 均匀分布，用于随机取向或自转角 |
| `zigzag` | 倾斜偏移（度） | 幅度（度） | 折叠反正弦分布，用于大晶体之字形振荡 |
| `laplacian` | 中心值（度） | 尺度参数（度） | 拉普拉斯分布，用于尺寸聚合倾斜 |
| `gauss_legacy` | 中心值（度） | 标准差（度） | 无 Jacobian 修正的高斯分布（用于复现旧版模拟结果） |

**说明：**
- `gauss` 和 `uniform` 是冰晕模拟中最常用的类型
- `zigzag` 模拟高雷诺数下大晶体的振荡运动
- `laplacian` 简化混合尺寸晶体群体的配置（物理背景见 [crystal-orientation-sampling_zh.md](crystal-orientation-sampling_zh.md)）
- `gauss_legacy` 复现早期程序版本的采样行为（未包含球面面积元 Jacobian 修正），仅在对比旧版模拟输出时使用

**示例：**
```json
"zenith": { "type": "zigzag", "mean": 5, "std": 30 }
```
```json
"zenith": { "type": "laplacian", "mean": 90, "std": 2.0 }
```

### filter（过滤器配置）

过滤器用于过滤光线路径或方向。

#### 基本结构

```json
{
  "id": <唯一标识符>,
  "type": "none" | "raypath" | "entry_exit" | "direction" | "crystal" | "complex",
  "symmetry": "P" | "B" | "D" | "PBD" | ...,
  "action": "filter_in" | "filter_out",
  ...
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `id` | 整数 | 是 | - | 唯一标识符 |
| `type` | 字符串 | 是 | - | 过滤器类型 |
| `symmetry` | 字符串 | 否 | "" | 对称性开关：P/B/D 的任意组合——"P"（C6 旋转）、"B"（水平镜面）、"D"（竖直镜面）。启用条件与精确语义见 [Raypath 对称性](raypath-symmetry.zh.md)。 |
| `action` | 字符串 | 否 | "filter_in" | 动作："filter_in" 或 "filter_out" |

#### 各类型特定参数

**1. none（无过滤器）**
```json
{
  "id": 1,
  "type": "none"
}
```

**2. raypath（光线路径）**
```json
{
  "id": 2,
  "type": "raypath",
  "raypath": [3, 5],
  "symmetry": "P"
}
```
- `raypath`: 整数数组，光线路径面编号。面编号遵循六角晶体约定：
  底面 = 1/2，棱柱面 = 3–8，上锥面 = 13–18，下锥面 = 23–28。
  Crystal 编辑弹窗的 3D 预览会在每个可见面上叠加对应编号。

**3. entry_exit（入口出口）**
```json
{
  "id": 3,
  "type": "entry_exit",
  "entry": 3,
  "exit": 5,
  "action": "filter_in"
}
```
- `entry`: 入口面编号
- `exit`: 出口面编号

**4. direction（方向）**
```json
{
  "id": 4,
  "type": "direction",
  "az": 180,
  "el": 25,
  "radii": 0.5,
  "action": "filter_out"
}
```
- `az`: 方位角（度）
- `el`: 仰角（度）
- `radii`: 半径（度）

**5. crystal（晶体）**
```json
{
  "id": 5,
  "type": "crystal",
  "crystal_id": 3
}
```
- `crystal_id`: 晶体ID

**6. complex（复合）**
```json
{
  "id": 6,
  "type": "complex",
  "composition": [1, [2, 6], 5]
}
```
- `composition`: 过滤器组合表达式

### scene（场景配置）

场景配置定义模拟场景，包括光源、晶体组合和光线数量。场景是单个对象（非数组），不含 `id` 字段。

#### 基本结构

```json
{
  "light_source": { ... },
  "ray_num": <整数或"infinite">,
  "max_hits": <整数>,
  "scattering": [ ... ]
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `light_source` | 对象 | 是 | - | 内联光源配置（见下方） |
| `ray_num` | 整数或字符串 | 是 | - | **总光线数**（跨所有光谱波长），或 `"infinite"` 持续模拟 |
| `max_hits` | 整数 | 是 | - | 最大碰撞次数 |
| `scattering` | 数组 | 是 | - | 散射配置数组 |

> **`ray_num` 的语义（task-323 起变更）**：`ray_num` 是**总光线数**，服务端内部按
> `per_wl = ⌈ray_num / N_波长⌉` 分配到每个离散波长，实际追踪总数 =
> `per_wl × N_波长 ≥ ray_num`（少量上舍入余量）。连续/预设光谱视作 N_波长=1，语义不变。
>
> **迁移说明**：外部手写的离散谱配置若沿用旧的"每波长"语义，需要将 `ray_num` 乘以离散
> 波长数。仓库自带的两个受影响配置（`examples/config_example.json`、
> `test/e2e/configs/color.json`）已在 task-323 中同步迁移，PSNR 参考图保持不变。

#### light_source（光源配置）

光源配置以内联对象的方式直接嵌入 `scene` 中，不再是顶层数组，也不含 `id` 字段。仅支持 `"sun"` 类型。

```json
{
  "type": "sun",
  "altitude": <角度>,
  "azimuth": <角度>,
  "diameter": <角度>,
  "spectrum": <光谱配置>
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `type` | 字符串 | 是 | - | 光源类型，仅支持 "sun" |
| `altitude` | 浮点数 | 是 | - | 地平高度（度） |
| `azimuth` | 浮点数 | 否 | 0.0 | 方位角（度） |
| `diameter` | 浮点数 | 否 | 0.5 | 直径（度），真实太阳通常为0.5 |
| `spectrum` | 字符串或对象数组 | 是 | - | 光谱配置，见下方说明 |

##### spectrum（光谱配置）

`spectrum` 支持两种格式：

**1. 标准光源模式**（字符串）——使用 CIE 标准光源的光谱功率分布（SPD）：
```json
"spectrum": "D65"
```
支持的标准光源：`"D50"`、`"D55"`、`"D65"`、`"D75"`、`"A"`、`"E"`

**2. 离散波长模式**（对象数组）——手动指定波长和权重：
```json
"spectrum": [
  {"wavelength": 420, "weight": 1.0},
  {"wavelength": 550, "weight": 1.0}
]
```

##### 光源注意事项

- 波长决定折射率，数据来自 [Refractive Index of Crystals](https://refractiveindex.info/?shelf=3d&book=crystals&page=ice)
- `azimuth` 和 `diameter` 是可选的，如果不指定会使用默认值
- 标准光源模式下，模拟器从 [380, 780] nm 范围内均匀采样波长，按 SPD 加权

##### 光源示例

```json
"light_source": {
  "type": "sun",
  "altitude": 20.0,
  "azimuth": 0,
  "diameter": 0.5,
  "spectrum": "D65"
}
```

```json
"light_source": {
  "type": "sun",
  "altitude": 20.0,
  "diameter": 0.5,
  "spectrum": [
    {"wavelength": 420, "weight": 1.0},
    {"wavelength": 460, "weight": 1.0},
    {"wavelength": 500, "weight": 1.0},
    {"wavelength": 540, "weight": 1.0},
    {"wavelength": 580, "weight": 1.0},
    {"wavelength": 620, "weight": 1.0}
  ]
}
```

#### scattering（散射配置）

每个散射配置项包含一个 `entries` 结构化对象数组，替代了原来的并行数组（`crystal[]`、`proportion[]`、`filter[]`）。

```json
{
  "prob": <概率>,
  "entries": [
    {
      "crystal": <晶体ID>,
      "proportion": <比例>,
      "filter": <过滤器ID>
    }
  ]
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `prob` | 浮点数 | 是 | - | 多散射概率。此前为可选、缺省时静默取 `0.0`；现在省略该字段会报错。若要保持旧行为，请显式写 `"prob": 0`。 |
| `entries` | 对象数组 | 是 | - | 散射条目数组 |

**entries 条目字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `crystal` | 整数 | 是 | - | 晶体ID引用 |
| `proportion` | 浮点数 | 否 | 100.0 | 比例值 |
| `filter` | 整数 | 否 | （不使用过滤器） | 过滤器ID引用，省略则不使用过滤器 |

#### 场景示例

```json
"scene": {
  "light_source": {
    "type": "sun",
    "altitude": 20.0,
    "spectrum": "D65"
  },
  "ray_num": 1000000,
  "max_hits": 7,
  "scattering": [
    {
      "entries": [
        {"crystal": 1, "proportion": 100},
        {"crystal": 2, "proportion": 30},
        {"crystal": 3, "proportion": 50}
      ],
      "prob": 0.2
    },
    {
      "entries": [
        {"crystal": 2, "proportion": 20, "filter": 2},
        {"crystal": 3, "proportion": 100, "filter": 1}
      ]
    }
  ]
}
```

使用 `"infinite"` 作为 `ray_num` 表示持续模拟：

```json
"scene": {
  "light_source": {
    "type": "sun",
    "altitude": 15.0,
    "spectrum": "D65"
  },
  "ray_num": "infinite",
  "max_hits": 7,
  "scattering": [
    {
      "prob": 0,
      "entries": [
        {"crystal": 1}
      ]
    }
  ]
}
```

### render（渲染配置）

渲染配置定义渲染器的参数。

#### 基本结构

```json
{
  "id": <唯一标识符>,
  "lens": { ... },
  "resolution": [<宽度>, <高度>],
  "lens_shift": [<x偏移>, <y偏移>],
  "view": { ... },
  "visible": "upper" | "lower" | "full",
  "front": <bool>,
  "background": [<r>, <g>, <b>],
  "ray_color": [<r>, <g>, <b>],
  "intensity_factor": <浮点数>,
  "ev_mode": "relative" | "absolute",
  "grid": { ... },
  "filter": [<过滤器ID数组>]
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `id` | 整数 | 是 | - | 唯一标识符 |
| `lens` | 对象 | 否 | 见下方 | 镜头配置 |
| `resolution` | 整数数组 | 是 | - | 分辨率 [宽度, 高度] |
| `lens_shift` | 整数数组 | 否 | [0, 0] | 镜头偏移 [x, y] |
| `view` | 对象 | 否 | 见下方 | 视角配置 |
| `visible` | 字符串 | 否 | "upper" | 可见半球："upper"、"lower"、"full"。它是**显示层裁剪**而非能量层剔除，四个镜头族（单镜头 `linear`/`fisheye_*`、`rectangular`、双鱼眼、`globe`）语义一致：光线始终按全天累积落盘，被排除的区域在出像素时才丢弃——最终既不含背景也不含光线能量。⛔ 别把它当吞吐旋钮：它不减少每条光线的计算量。曝光测光发生在裁剪**之前**、因而不受本键影响；这对既有单镜头场景有一个亮度后果，见 [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §2.7。另注意与投影自身的结构性裁剪区分（例如正交鱼眼背后的方向根本没有像素可落），那属于投影的定义域，不是本键管的事。 |
| `front` | 布尔 | 否 | false | 前半球裁剪：只保留相机朝向的那半边。它是**独立于 `visible` 的第二个裁剪维度**，两者相与，而不是 `visible` 的第四个取值。（写成 `"visible": "front"` 会被静默当作 `"upper"`，务必用这个独立键。） |
| `background` | 浮点数组 | 否 | [0, 0, 0] | 背景颜色 RGB，**sRGB** 空间（即取色器上显示的那组数） |
| `ray_color` | 浮点数组 | 否 | [-1, -1, -1] | 光线颜色 RGB，-1表示使用真实颜色 |
| `intensity_factor` | 浮点数 | 否 | 1.0 | 强度因子（`2^EV`） |
| `ev_mode` | 字符串 | 否 | "relative" | 曝光锚点：`"relative"` 锚到**场景**的天空亮度——在一块固定全天缓冲上量到的 P99 辐亮度，因此 lens / FOV / 相机朝向 / `visible` / 输出分辨率都不再改变曝光，同一份 config 无论怎么看都渲染出同样的亮度（画面外观同样随 `ray_num` 增长保持稳定，即 `ray_num` 仍与亮度相关——那正是 `"absolute"` 存在的理由）；`"absolute"` 锚到光源发射的能量，使不同 config 在同一 `intensity_factor` 下直接可比（仅限同一 lens/FOV/分辨率——见 [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §7）。缺该键或值无法识别都视为 `"relative"`。⚠️ 换锚时 `"relative"` 的输出亮度**发生了位移**：所有 config 都会动，实测语料上为 −2.02…+2.55 stop，方向取决于场景；迁移律与实测表见 [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §2.8，若某张图需要恢复旧观感，`intensity_factor` 可精确抵消。另见 [`doc/adaptive-brightness.zh.md`](adaptive-brightness.zh.md) §3。 |
| `grid` | 对象 | 否 | 见下方 | 网格配置 |
| `filter` | 整数数组 | 否 | [] | 多散射过滤器ID数组 |

#### lens（镜头配置）

```json
{
  "type": "linear" | "fisheye_equal_area" | "fisheye_equidistant" | "fisheye_stereographic" | "dual_fisheye_equal_area" | "dual_fisheye_equidistant" | "dual_fisheye_stereographic" | "rectangular" | "fisheye_orthographic" | "dual_fisheye_orthographic" | "globe",
  "fov": <角度>  // 或 "f": <焦距>
}
```

**默认值**：
- `type`: "linear"
- `fov`: 90.0（度）；`globe` 默认 30.0

**注意**：
- `fov` 为**全对角线视场角**（度）。`rectangular` 和 `dual_*` 类型会忽略 `fov`（始终为全天投影）。
- `fisheye_orthographic` 和 `dual_fisheye_orthographic` 的 FOV 上限为 **180°**（投影公式 `r = f·sin(θ)` 在 θ > 90° 时回折），超出值会被拒绝。
- `dual_fisheye_orthographic` 不支持 `overlap` 参数（会被静默忽略并输出一条 VERBOSE 日志）。
- 可以使用 `f`（焦距，mm，基于 35mm 胶片）代替 `fov`，程序会根据投影模型使用正确公式换算：
  - Linear: `fov = 2·atan(d/f)`
  - Equal area: `fov = 4·arcsin(d/(2f))`
  - Equidistant: `fov = 2d/f`（弧度 → 度）
  - Stereographic: `fov = 4·arctan(d/(2f))`
  - Orthographic: `fov = 2·arcsin(d/f)`（`f ≥ 12mm` 时 fov=180）
  - Rectangular: `f` 被忽略（始终全天投影）
  - Globe: 不支持 `f`，请直接使用 `fov`

**`globe` 镜头（外部观察天球的透视视角）**：
- 投影模型：相机位于距单位球心 `D = 4.0`（单位球半径为单位）处，朝球心方向看；shader 对单位球做光线—球面求交，命中点对应的样本被着色。
- `fov` 范围：`(0°, 90°]`，默认 `30°`。默认 fov 下球面占视口短边约 96%。
- `view.roll`：作为常规 `view` 字段保存，但渲染时**强制为 0**（仅 `globe` 适用）；右侧面板 Roll slider 在选中 `globe` 时置灰；切回非 Globe lens 时恢复原值（字段保留，仅渲染时屏蔽）。
- `view.azimuth` / `view.elevation` 语义：在 `globe` 下表示**观察者绕球公转的视角**，而非相机自身姿态。view 矩阵与 inside-out lens 完全相同，但用户心智反转（Az/El 指向球面上某个点，而非天空中某个方向）。
- `view.elevation` 在 `globe` 下被 clamp 到 `[-89°, +89°]`，避免 ±90° 的 view-matrix 退化；trackball drag 与右侧 slider 共享此约束。
- `.lmc` 兼容性：未引入新字段，旧 `.lmc` 加载行为不变。

#### view（视角配置）

```json
{
  "azimuth": <角度>,
  "elevation": <角度>,
  "roll": <角度>
}
```

**默认值**：
- `azimuth`: 0.0
- `elevation`: 0.0
- `roll`: 0.0

#### grid（网格配置）

```json
{
  "angular_dist": [ ... ],
  "elevation": [ ... ],
  "longitude": [ ... ],
  "horizon": <布尔值>,
  "zenith_nadir": { ... }
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `angular_dist` | 对象数组 | 否 | [] | 距太阳角距恒定的圈——最常见的就是 22° 与 46° 晕。**会被绘制。** `value`、`opacity`、`color` 均生效，`width` 不生效（见下）。本键缺失时会回退读取旧键 `central`。 |
| `elevation` | 对象数组 | 否 | [] | 等高线——仰角恒定的线，单位为度。**会被绘制**（自 v4.18 起；此前版本只解析、不画）。规则与 `angular_dist` 相同：`value`、`opacity`、`color` 生效，`width` 不生效。 |
| `longitude` | 对象数组 | 否 | [] | 经度线——方位角恒定的线，单位为度。**会被绘制。** 规则与 `elevation` 相同。v4.18 新增；文件里没有这个键时得到空列表，即什么都不画。 |
| `horizon` | 布尔值 | 否 | false | 沿天球地平线（仰角 0）画一条线，只画在可见半球内。默认关闭，需显式设为 `true` 才绘制。 |
| `zenith_nadir` | 对象 | 否 | 见下 | 在天顶与天底绘制的像素空间圆环标记。两个标记共用一个对象，而不是线数组：这两个方向是固定的，没有需要逐条命名的东西；GUI 也只对这一对标记给一个开关、一个颜色、一个半径。v4.19 新增。 |

**`zenith_nadir` 对象**：

```json
{
  "enabled": <布尔值>,
  "radius_px": <浮点数>,
  "opacity": <浮点数>,
  "color": [<r>, <g>, <b>]
}
```

**默认值**：
- `enabled`: false
- `radius_px`: 8.0
- `opacity`: 0.6
- `color`: [0.8, 0.2, 0.2]（sRGB）

**注意**：每个键都是可选的，包括 `enabled`——对象里出现但省略的字段沿用上面的结构体默认值。
`enabled` 默认为 false，所以完全不写 `zenith_nadir` 的配置什么都不画。

**`central` 是 `angular_dist` 的旧名**

改名的理由是 `central` 从未说清这个数是什么：它就是**距太阳的角距**。两个解析器都把 `central`
当别名读；万一某个文件两个键都有，以 `angular_dist` 为准。写出端只写 `angular_dist`，所以旧文件
读进来再存出去就会换成新名。旧文件可以一直用下去——这里没有版本开关，因为两种拼写从来没有表示过
不同的含义。

C API 字段同步改名（`LUMICE_RenderParam.angular_dist` / `angular_dist_count`，原
`central_grid` / `central_grid_count`）。这是**源码级**兼容性破坏，内存布局不变；详见
`src/include/lumice.h` 中 `LUMICE_API_VERSION` 处的 BREAKING 说明。

**三族线各自画什么、忽略什么**

`value`、`opacity`、`color` 对三者全部生效：每条线各自成曲线、各自上色、各自按自己的不透明度
混合。`width` 会被读取、校验并原样写回，但**不影响任何像素**——线的粗细来自它所属那个场的局部
梯度，这样一条曲线穿过投影被拉伸的区域时仍保持一条线的宽度，而这个算法没有任何宽度入口。设置了
`width` 的配置不算错误，只是画出来一样。

**为什么叫 `longitude` 而不是 `azimuth`**

经度线沿用标注层公共符号里已经在用的那个词（`LUMICE_ANNOTATION_LONGITUDE`、
`LUMICE_AnnotationRequest::longitude_deg`）。持久化键若改叫 `azimuth`，就等于给同一个概念造出
两套词汇，而这换不来任何东西。

**GUI 导出时如何填 `elevation` 与 `longitude`**

GUI 没有逐条网格线的控件：它由当前 FOV 推出**单一步长**（≥120° 时 30°，一路到 <2° 时 0.5°），
并对整个网格共用一种颜色和一个不透明度。导出时把这个步长**展开**成本 schema 依赖的显式列表——
等高线取 ±80° 内每隔一个步长一条、排除 0°（0° 由 horizon 拥有并自带颜色），经度线取半开区间
(-180°, 180°] 内每隔一个步长一条（这样 180° 只出现一次）——并把那份共用外观逐条重复写上。

所以**自适应步长是显示层的便利，不是模型的一部分**：模型就是这份列表，配置可以任意指定等高线与
经度线的位置、间距，甚至给每条线不同的颜色。

有一个后果值得在放大视角前知道：窄 FOV 会选到很细的步长，而细步长展开后可能超过
`LUMICE_MAX_CONFIG_GRID_LINES`（64）——20° FOV 就是 72 条经度线。此时 GUI **拒绝导出**，并给出
一条写明是哪一族、多少条的提示，而不是写出一份被截断的网格：截断意味着 CLI 渲染出来的图与屏幕上
的图悄悄不同。把 FOV 调大，或者关掉网格线，再导出即可。

**网格线配置**：

```json
{
  "value": <角度>,
  "color": [<r>, <g>, <b>],
  "opacity": <浮点数>,
  "width": <浮点数>
}
```

**默认值**：
- `color`: [1.0, 1.0, 1.0]（白色）
- `opacity`: 1.0
- `width`: 1.0

## 配置验证规则

### ID 唯一性验证

- `crystal`、`filter`、`render` 数组中的 `id` 必须在各自类型内唯一
- `id` 必须大于 0

### ID 引用有效性验证

- `scene.scattering[].entries[].crystal` 引用的晶体ID必须存在于 `crystal` 数组中
- `scene.scattering[].entries[].filter` 引用的过滤器ID必须存在于 `filter` 数组中（如果指定）
**注意**：`scene` 是单个对象，不是数组。`light_source` 是 `scene` 内的内联对象，不再需要 ID 引用。所有定义在 `render` 数组中的渲染器均自动生效。

### 数组长度匹配验证

- `scene.light_source.spectrum` 格式为字符串（标准光源名称）或对象数组（每个对象含 `wavelength` 和 `weight`）
- `crystal[].shape.face_distance` 数组长度必须为 6（如果指定）
- `crystal[].shape.upper_indices` 数组长度必须为 3（如果指定）
- `crystal[].shape.lower_indices` 数组长度必须为 3（如果指定）
- `crystal[].shape.sync_group.face_distance` 数组长度必须为 6（如果指定）；比 6 短会将剩余槽位
  按独立（0）处理，比 6 长会被截断，而不是报错拒绝——详见[形状标量 Sync
  Group](#形状标量-sync-group)
- `render[].resolution` 数组长度必须为 2

### 数值范围验证

- 角度值通常在 -180 到 180 度之间（某些情况下可能超出）
- `crystal[].shape.upper_h` 和 `lower_h` 并不被钳制在 `[0.0, 1.0]`：负值会先取绝对值
  再参与构造，任何 `>= 1.0` 的值都得到与恰好 `1.0` 相同的满顶结果——这不是错误。
  各区间实际产出的形状见下方 [pyramid 形状合法性](#pyramid-形状合法性)。
- `render[].background` 和 `ray_color` 颜色值应在 0.0 到 1.0 之间
- `render[].background` 是 **sRGB**：在没有晕能量的像素上，渲染出来的颜色就精确等于这里写的三元组。
  读取配置时会转换为线性空间，因为背景是加到晕的辐亮度上的，这个加法只在线性空间里才有意义；
  写出配置时再转换回 sRGB。（`ray_color` 则是线性的——它是施加在辐亮度上的着色，不是观察者直接看到的颜色。
  `LUMICE_RenderParam` C 结构体的这两个字段都是线性的，只有 JSON 键的语义不同。）

### pyramid 形状合法性

- **`upper_h` / `lower_h` 的边界语义**：
  - `0.0`（或折叠后为 `0.0` 的负值）：该侧完全没有锥，改由一个基面封口（基面如何
    挂接见下方 `prism_h == 0.0` 的三种组合）。
  - `(0.0, 1.0)`：锥在未到顶点前被截断（一个平顶棱台），该侧在截断面处有一个基面。
  - `>= 1.0`：该侧锥到达完整顶点——该侧没有基面。大于 `1.0` 的值不是错误，会得到
    与恰好 `1.0` 相同的结果。
  - 紧贴 `1.0` 下方的一段极窄区间的 `upper_h`/`lower_h`（量级约 `1e-4` 宽）会被
    直接吸附为精确的满顶点，而不是按请求的分数高度产出。这是刻意设计：它替代了
    历史上落在这段区间的晶体会静默丢失整个锥的缺陷。该吸附在什么情况下会打日志，
    见下方[常见配置错误](#常见配置错误)一节。
- **wedge 角合法性**：一侧锥的 wedge 角——直接给定的 `upper_alpha`/`lower_alpha`，
  或由 `upper_indices`/`lower_indices`（Miller 指数）推出的等效角——必须落在
  `[0.1°, 89.9°]` 区间内，**两个端点都包含**：恰好 `0.1` 与恰好 `89.9` 仍会构建出锥。
  超出该区间时该侧锥被当作不存在处理，与 `upper_h`/
  `lower_h` 折叠为 `0.0` 效果相同——且不打印任何警告（只要晶体其余部分——另一侧的
  锥或棱柱段——仍能凑够面数，形状照常构建）。
- **`prism_h == 0.0`（没有直棱柱段）在三种锥组合下均合法**：
  - *一侧有锥、另一侧没有*：合法。无锥一侧的基面直接由有锥一侧侧壁环的顶点构成
    ——反正也没有棱柱侧壁可挂接。
  - *两侧都有锥*：合法——一个没有直段的双锥体，两个锥直接在同一圈顶点相接；两侧
    都不会有基面（基面只出现在没有锥的那一侧）。
  - *两侧都没有锥*：零体积。该晶体会被丢弃、贡献零能量，与其它退化形状一致——这
    是三种组合里唯一不合法的一种。
- **`face_distance` 不规则时，顶点可能变成一条棱而非一个点**——这是合法几何，不是
  退化输入。当 6 个 `face_distance` 并不全相等时，定位锥顶点的求解可能在一条线段
  上同时取得最大值，而非单点，此时"顶点"其实是一条两端点的棱。每个端点只属于六个
  锥面里平面确实经过它的那个子集，不是全部六个。

### 必填字段验证

- `scene.light_source`: `type`, `altitude`, `spectrum` 必填
- `crystal`: `id`, `type`, `shape` 必填
- `filter`: `id`, `type` 必填
- `scene`: `light_source`, `ray_num`, `max_hits`, `scattering` 必填
- `scene.scattering[]`: `prob` 必填（没有默认值——缺失 `prob` 会被拒绝；不存在隐式回落到 0，
  想保留旧的"不做多次散射"行为必须显式写 `"prob": 0`）
- `scene.scattering[].entries[]`: `crystal` 必填
- `render`: `id`, `resolution` 必填
- `crystal[].axis` 只要存在就要求 `zenith`；`azimuth` 与 `roll` 可以各自独立省略
  （见上方 [axis（方向分布）默认值](#axis方向分布默认值)）
- 任何写成对象的分布槽——`axis.{zenith,azimuth,roll}`，以及形状标量
  `height` / `prism_h` / `upper_h` / `lower_h` / `face_distance[]` 中的任一项——都要求 `type`。
  分布槽也可以写成一个裸数字表示固定值（如 `"zenith": 30`），此时不需要 `type`。
### 类型验证

- `crystal[].type` 必须是 "prism" 或 "pyramid"
- `scene.light_source.type` 必须是 "sun"
- `filter[].type` 必须是 "none"、"raypath"、"entry_exit"、"direction"、"crystal" 或 "complex"
- `render[].visible` 必须是 "upper"、"lower" 或 "full"
- `render[].lens.type` 必须是 "linear"、"fisheye_equal_area"、"fisheye_equidistant"、"fisheye_stereographic"、"dual_fisheye_equal_area"、"dual_fisheye_equidistant"、"dual_fisheye_stereographic"、"rectangular"、"fisheye_orthographic"、"dual_fisheye_orthographic" 或 "globe"
- `scene.ray_num` 必须是正整数或字符串 `"infinite"`

## 常见配置错误

### 1. 晶体ID未定义错误

**错误描述**：散射条目引用了不存在的晶体ID

**错误示例**：
```json
{
  "scene": {
    "scattering": [
      {
        "prob": 0,
        "entries": [
          {"crystal": 999}
        ]
      }
    ]
  }
}
```

**正确示例**：
```json
{
  "crystal": [
    { "id": 1, "type": "prism", "shape": { "height": 1.3 } }
  ],
  "scene": {
    "scattering": [
      {
        "prob": 0,
        "entries": [
          {"crystal": 1}
        ]
      }
    ]
  }
}
```

### 2. scattering 使用旧的并行数组格式

**错误描述**：使用旧的 `crystal[]`/`proportion[]`/`filter[]` 并行数组格式

**错误示例**：
```json
{
  "scattering": [
    {
      "prob": 0,
      "crystal": [1, 2, 3],
      "proportion": [10, 20, 30],
      "filter": [1, 2, -1]
    }
  ]
}
```

**正确示例**：
```json
{
  "scattering": [
    {
      "prob": 0,
      "entries": [
        {"crystal": 1, "proportion": 10, "filter": 1},
        {"crystal": 2, "proportion": 20, "filter": 2},
        {"crystal": 3, "proportion": 30}
      ]
    }
  ]
}
```

### 3. 类型错误

**错误描述**：使用了错误的晶体类型名称

**错误示例**：
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "HexPrism"  // 错误：应使用 "prism"
    }
  ]
}
```

**正确示例**：
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism"  // 正确：使用 "prism"
    }
  ]
}
```

### 4. 缺少必填字段

**错误描述**：缺少必填字段

**错误示例**：
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism"
      // 错误：缺少 "shape" 字段
    }
  ]
}
```

**正确示例**：
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": {
        "height": 1.3
      }
    }
  ]
}
```

### 5. 配置结构错误

**错误描述**：配置结构不符合要求

**错误示例**：
```json
{
  "scene": [
    {
      "id": 1,
      "light_source": 1  // 错误：scene 不再是数组，light_source 不再是ID引用
    }
  ]
}
```

**正确示例**：
```json
{
  "scene": {
    "light_source": {
      "type": "sun",
      "altitude": 20.0,
      "spectrum": "D65"
    },
    "ray_num": 1000000,
    "max_hits": 7,
    "scattering": [
      {
        "prob": 0,
        "entries": [
          {"crystal": 1}
        ]
      }
    ]
  }
}
```

### 6. ray_num 使用 -1

**错误描述**：使用 -1 表示无限光线数量（旧格式）

**错误示例**：
```json
{
  "scene": {
    "ray_num": -1
  }
}
```

**正确示例**：
```json
{
  "scene": {
    "ray_num": "infinite"
  }
}
```

### 7. 读懂"顶点吸附"警告

**错误描述**：`upper_h` 或 `lower_h` 落得足够接近 `1.0`，求解器把该侧锥直接吸附为
精确满顶点，而不是按请求的分数高度构造。产出的晶体依旧是合法实体——这不是拒绝——
但这条日志值得一读，因为它说明这次请求正好落在"截断"与"满顶"两个判据的交界上
（参见上方 [pyramid 形状合法性](#pyramid-形状合法性)）。

**日志长这样**（源自 `src/core/geo3d_closedform.cpp`）：
```
ComputeClosedFormPyramid: <upper|lower> cone has no cross-section at inset <m>
(apex inset <apex_m>), yet is not recognised as collapsed; face_distance=[...].
Degrading that cone to a single apex point. To get the truncated shape instead,
move upper_h/lower_h away from 1.0 or widen the spread between face_distance
entries.
```

**该怎么改**：按日志自带的建议——把 `upper_h`/`lower_h` 调得离 `1.0` 更远一些，或者
拉开 `face_distance` 各分量之间的差距。在当前已落地的容差绑定下，这条路径按设计
预期永远不会触发（覆盖近顶点窗口的 413,280 个采样点扫描里一次都没触发过）；如果
你真的看到它，说明你撞上了一个比已收窄窗口更极端的边缘情况，同样的两条建议依然
适用。

### 8. 读懂"面被丢弃"警告

**错误描述**：晶体上有一个面窄到几何求解器已经分辨不出来了——通常是某个侧面，
它的两个界定角点之间只差晶体自身尺寸的几倍 `1e-5`。求解器在遍历横截面时确实碰到了
这个面，却没能在它上面留下 3 个互异顶点，于是这个面被丢弃。与上面的"顶点吸附"不同，
这次的产出**不是**合法实体：被丢弃那个面旁边的各个面仍然保留着本该由它封口的边，
于是曲面上留了一个洞。

下游没有任何东西会拦下这样的晶体——合法性检查只数存在的面数，不检验它们是否构成
闭合曲面——所以这条警告是你唯一能拿到的信号。光线照样会在剩下的面上折射，这一轮
仿真也照样会给出统计结果。

**日志长这样**（源自 `src/core/geo3d_closedform.cpp`）：
```
ComputeClosedFormPyramid: face slot(s) [13] were reached by the emitter but kept
fewer than 3 distinct vertices, so they are dropped and the surface they bounded is
left open; face_distance=[...], upper_h_inset=..., lower_h_inset=... This crystal has
a face thinner than the solver can resolve. To get a well-formed solid instead, widen
the spread between the face_distance entries around the offending face, or move
upper_h/lower_h away from that face's collapse height.
```

**该怎么改**：日志里的 slot 编号就指名了出问题的面——`0`/`1` 是上下两个基面，
`2`–`7` 是六个棱面，`8`–`13` 是上锥六个面，`14`–`19` 是下锥六个面，其中第 `i` 项与
`face_distance[i]` 共用方向 `i`。把该方向附近的 `face_distance` 各分量差距拉开，
让这个面要么明确存在、要么干净地不存在，而不是卡在分辨率边界上。如果这个面不是在
肩部、而是在锥面中途才收缩掉，调整 `upper_h`/`lower_h` 同样有效。

### 9. scattering 层缺少 `prob`

**错误描述**：`scene.scattering[]` 的某一层缺少必填字段 `prob`。这里没有隐式默认值——
历史上会静默回落到 `0.0`，但这个回落已经被移除。

**报错原文**：
```text
scene.scattering[0] is missing required field "prob" (multi-scattering probability). The
historical default was 0.0; add "prob": 0 explicitly to keep that behavior.
```

**错误示例**：
```json
{
  "scattering": [
    {
      // 错误：缺少 "prob"
      "entries": [{"crystal": 1}]
    }
  ]
}
```

**正确示例**：
```json
{
  "scattering": [
    {
      "prob": 0,  // 正确：即使想保留旧的不做多次散射行为，也要显式写出来
      "entries": [{"crystal": 1}]
    }
  ]
}
```

### 10. 分布对象缺少 `type`

**错误描述**：分布槽（`axis.zenith` / `axis.azimuth` / `axis.roll`，或形状标量如 `height`、
`prism_h`、`upper_h`、`lower_h`、`face_distance[]` 中的某一项）写成了 JSON 对象却省略了
`type`。裸数字不需要 `type`；对象需要。

**报错原文**（axis 三槽会自报槽名；形状标量共用一条不点名字段的通用消息）：
```text
axis.zenith is a distribution object with no "type". Write axis.zenith either as a bare number
for a fixed angle (e.g. "zenith": 20) or as an object naming the distribution (e.g. "zenith":
{"type": "gauss", "mean": 20, "std": 5}).
```
```text
distribution object is missing required key "type". Write either a bare number (e.g. 20) or an
object naming the distribution (e.g. {"type": "gauss", "mean": 20, "std": 5}).
```

**错误示例**：
```json
{
  "axis": {
    "zenith": { "mean": 20, "std": 5 }  // 错误：缺少 "type"
  }
}
```

**正确示例**：
```json
{
  "axis": {
    "zenith": { "type": "gauss", "mean": 20, "std": 5 }  // 正确
  }
}
```

### 11. `axis` 存在但缺少 `zenith`

**错误描述**：只要写了 `axis`，就必须写 `zenith`——此时它没有默认值。
（想要默认的固定朝向，应整个省略 `axis`；见上方
[axis（方向分布）默认值](#axis方向分布默认值)。）

**报错原文**：
```text
axis is present but has no "zenith", which is required whenever `axis` is written at all (omit
`axis` entirely to get the default orientation instead). Write axis.zenith either as a bare
number for a fixed angle (e.g. "zenith": 20) or as an object naming the distribution (e.g.
"zenith": {"type": "gauss", "mean": 20, "std": 5}).
```

**错误示例**：
```json
{
  "axis": { "azimuth": 0 }  // 错误："axis" 存在但缺少 "zenith"
}
```

**正确示例**：
```json
{
  "axis": { "zenith": 30, "azimuth": 0 }  // 正确
}
```

## 配置最佳实践

### 性能优化建议

1. **光线数量设置**：
   - 测试时使用较小的 `ray_num`（如 10000）
   - 生产环境根据需求调整，通常 1000000 以上
   - 使用 `"infinite"` 进行持续模拟

2. **配置复用**：
   - 避免重复定义相同的晶体，通过 ID 引用复用

3. **过滤器使用**：
   - 合理使用过滤器可以减少不必要的计算
   - 在 `scene.scattering[].entries[]` 中使用 `filter` 字段

### 常见场景配置模板

#### 简单日晕模拟

```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": { "height": 1.2 }
    }
  ],
  "scene": {
    "light_source": {
      "type": "sun",
      "altitude": 20.0,
      "diameter": 0.5,
      "spectrum": "D65"
    },
    "ray_num": 1000000,
    "max_hits": 7,
    "scattering": [
      {
        "prob": 0,
        "entries": [
          {"crystal": 1}
        ]
      }
    ]
  },
  "render": [
    {
      "id": 1,
      "resolution": [1920, 1080],
      "lens": { "type": "linear", "fov": 40 }
    }
  ]
}
```

#### 多晶散射模拟

```json
{
  "crystal": [
    { "id": 1, "type": "prism", "shape": { "height": 1.2 } },
    { "id": 2, "type": "prism", "shape": { "height": 0.8 } },
    { "id": 3, "type": "pyramid", "shape": { "prism_h": 1.0, "upper_h": 0.3 } }
  ],
  "filter": [
    { "id": 1, "type": "raypath", "raypath": [3, 5], "symmetry": "P" },
    { "id": 2, "type": "entry_exit", "entry": 3, "exit": 5 }
  ],
  "scene": {
    "light_source": {
      "type": "sun",
      "altitude": 20.0,
      "spectrum": "D65"
    },
    "ray_num": 1000000,
    "max_hits": 7,
    "scattering": [
      {
        "entries": [
          {"crystal": 1, "proportion": 100},
          {"crystal": 2, "proportion": 50},
          {"crystal": 3, "proportion": 30}
        ],
        "prob": 0.2
      },
      {
        "entries": [
          {"crystal": 2, "proportion": 20, "filter": 2},
          {"crystal": 3, "proportion": 100, "filter": 1}
        ]
      }
    ]
  },
  "render": [
    {
      "id": 1,
      "resolution": [1920, 1080],
      "lens": { "type": "linear", "fov": 40 }
    }
  ]
}
```

### 调试配置建议

1. **小规模测试配置**：
   - `ray_num`: 100 或更小
   - 使用单一波长（`"spectrum": [{"wavelength": 550, "weight": 1.0}]`）
   - 减少晶体数量

2. **详细日志配置**：
   - 查看配置解析日志以定位问题

## GUI 个人默认值（用户覆盖文件）

这是**独立于上述场景配置的另一份文件**——它不与 `crystal`/`filter`/`scene`/`render` 放在一起，只被 GUI 读取，CLI 从不读取。它让用户把自己的习惯（常用镜头、调过的 axis 预设）固化下来，在本机每次新建文档时套用，而不是把某一个用户的习惯写死成所有人的出厂值。

#### 文件位置

每个操作系统的用户级配置目录下一份 JSON 文件，绝不放在可执行文件旁边（只读安装或多用户机器上后者可能不可写）：

| 平台 | 位置 |
|------|------|
| Windows | `%APPDATA%\Lumice\user_defaults.json` |
| macOS | `~/Library/Application Support/Lumice/user_defaults.json` |
| Linux | `$XDG_CONFIG_HOME/lumice/user_defaults.json`，未设置时回退 `~/.config/lumice/user_defaults.json` |

GUI 的日志文件原先无条件写在 `$HOME` 下，现改为与此目录同处。

#### 格式：一份残缺的 GuiState 文档 + 预设子树

文件主体**不是新格式**——它是 `.lmc` 文件序列化配置区时所用的同一份 JSON 的局部转储（同样的文档级字段：`sun`、`sim`、`renderer`，以及叠加层颜色、宽高比预设、背景图等视图类字段）。文件里缺的 key 一律回退到出厂值，因为读取它的解析器就是 GUI 到处复用的那一个，而该解析器一贯把"缺 key"当作"用出厂值"——本文件没有另开一套合并逻辑。

除此之外，一个 `presets` 子树携带内置 axis 方向预设各自的覆盖值，例如：

```json
{
  "renderer": { "lens_type": "..." },
  "presets": {
    "axis": {
      "column": { "zenith_std": 0.3 }
    }
  }
}
```

只有内置预设的 zenith std 可以被覆盖，且覆盖值被约束在该预设既有的分类判据容差域内——越界值会被 clamp 到边界而不是被拒绝，clamp 行为会被上报（见下）。没有可调面的预设（例如全均匀分布的预设）会直接拒绝任何存入值。

#### 降级输入的处置

文件本身无法解析、字段类型不符、预设值非数字或非有限、预设值被 clamp，这几种情况都归入同一个提示通道，而不是散落成若干条各写各的警告，方便用户在一处看全所有被忽略或调整的内容。字段级类型错误会丢弃整份覆盖文件的 GuiState 半区（不会出现只应用一半的结果）；`presets` 子树的解析与之独立。

**个人默认值只在全新文档上生效**——它绝不会覆盖正在打开的文件里已经写明的值，无论这份文件是 `.lmc` 工程文件还是通过 GUI 导入的 CLI JSON 配置。打开别人发来的文件，看到的永远是那份文件自己指定的值（或它省略部分的出厂值），与执行加载的这台机器上保存了什么无关。

#### 编辑：每次 Save 一次写盘，从不是文件的实时视图

顶栏 `Settings` 按钮打开的 `Settings` 面板（`src/gui/defaults_panel.cpp`）不是这份文件的实时视图。它在打开时读一次文件，面板内的一切编辑——勾选框、单元格里改的值、"Reset all"——都只改这份内存中的副本，**文件本身只在按下 Save 时被写入恰好一次**。以任何其它方式关闭面板（Close、标题栏 X、Esc）都会丢弃这份副本，磁盘上的内容不会移动。

面板表格里有两列命名了同一个 key 的两个不同值，值得在这里说清楚，因为两者极易被混淆：

- **"Current value"（当前值）**——工作副本里这个 key 的值，即此刻按下 Save 会写出的内容。它不是磁盘上这份文件现在存的值；那是另一列"Source"（"Mine" 表示面板打开时文件里已有这个 key，否则是 "Factory"）。
- **"Origin value"（出厂值）**——字面意义的出厂值，即 `GuiState{}` 序列化后的结果，没有叠加任何东西。它不是"你开始编辑前的生效默认值"——一个你曾保存过非出厂值的 key，这一列显示的出厂值与此无关，恒定不变。

面板的内部架构（一行的存在性如何从序列化文档生成、它的编辑器为何是一张独立的注册表、以及上面这套副本模型的完整形态）见 `doc/gui-state-governance.md` §8。

#### `--user-config` / `--no-user-config`

两个 CLI 开关（刻意不做成环境变量——用户可见的行为开关交给 env var 会导致逐机器静默漂移）：

- `--user-config <dir>`：用指定目录代替 OS 默认目录，用于测试或维护多份互相隔离的配置。
- `--no-user-config`：完全跳过个人默认值，每份新文档只用出厂值。

交互式 GUI 二进制在两个开关都不传时默认自动探测 OS 目录；GUI 测试二进制则默认禁用，这样一张视觉回归参考图就不会依赖拍摄它的那台机器上恰好存在的 `user_defaults.json`。

## 相关文档

- [README](../README_zh.md): 用户文档
- [系统架构文档](architecture_zh.md): 系统架构文档
- [配置示例文件](../examples/config_example.json): 配置示例文件
