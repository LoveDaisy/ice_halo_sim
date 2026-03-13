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

如果 `axis` 字段不存在，使用以下默认值：

```json
{
  "zenith": 90.0,    // 水平方向
  "azimuth": 0.0,
  "roll": 0.0
}
```

#### prism（六棱柱）类型

**shape 结构**：

```json
{
  "height": <数值或分布>,
  "face_distance": [<6个数值或分布>]
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `height` | 数值/分布 | 否 | 1.0 | 高度比 h/a，h是棱柱高度，a是底面直径 |
| `face_distance` | 数组 | 否 | [1,1,1,1,1,1] | 6个面的距离比，正六边形为[1,1,1,1,1,1] |

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
  "face_distance": [<6个数值或分布>]
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `prism_h` | 数值/分布 | 是 | - | 棱柱段高度比 |
| `upper_h` | 数值/分布 | 否 | 0.0 | 上锥段相对高度（0.0-1.0） |
| `lower_h` | 数值/分布 | 否 | 0.0 | 下锥段相对高度（0.0-1.0） |
| `upper_indices` | 整数数组 | 否 | [1,0,1] | 上锥段Miller指数 |
| `lower_indices` | 整数数组 | 否 | [1,0,1] | 下锥段Miller指数 |
| `face_distance` | 数组 | 否 | [1,1,1,1,1,1] | 6个面的距离比 |

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
- `gauss`: 高斯分布，`mean` 为均值，`std` 为标准差
- `uniform`: 均匀分布，`mean` 为中心值，`std` 为范围的一半

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
| `symmetry` | 字符串 | 否 | "" | 对称性："P"（平面）、"B"（双面）、"D"（双重） |
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
- `raypath`: 整数数组，光线路径面编号

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
| `ray_num` | 整数或字符串 | 是 | - | 光线数量，整数或 `"infinite"` |
| `max_hits` | 整数 | 是 | - | 最大碰撞次数 |
| `scattering` | 数组 | 是 | - | 散射配置数组 |

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
| `prob` | 浮点数 | 否 | 0.0 | 多散射概率 |
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
  "background": [<r>, <g>, <b>],
  "ray_color": [<r>, <g>, <b>],
  "opacity": <浮点数>,
  "intensity_factor": <浮点数>,
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
| `visible` | 字符串 | 否 | "upper" | 可见半球："upper"、"lower"、"full" |
| `background` | 浮点数组 | 否 | [0, 0, 0] | 背景颜色 RGB |
| `ray_color` | 浮点数组 | 否 | [-1, -1, -1] | 光线颜色 RGB，-1表示使用真实颜色 |
| `opacity` | 浮点数 | 否 | 1.0 | 透明度 |
| `intensity_factor` | 浮点数 | 否 | 1.0 | 强度因子 |
| `grid` | 对象 | 否 | 见下方 | 网格配置 |
| `filter` | 整数数组 | 否 | [] | 多散射过滤器ID数组 |

#### lens（镜头配置）

```json
{
  "type": "linear" | "fisheye_equal_area" | "fisheye_equidistant" | "fisheye_stereographic" | "dual_fisheye_equal_area" | "dual_fisheye_equidistant" | "dual_fisheye_stereographic" | "rectangular",
  "fov": <角度>  // 或 "f": <焦距>
}
```

**默认值**：
- `type`: "linear"
- `fov`: 90.0（度）

**注意**：
- `fov` 为**全对角线视场角**（度）。`rectangular` 和 `dual_*` 类型会忽略 `fov`（始终为全天投影）。
- 可以使用 `f`（焦距，mm，基于 35mm 胶片）代替 `fov`，程序会根据投影模型使用正确公式换算：
  - Linear: `fov = 2·atan(d/f)`
  - Equal area: `fov = 4·arcsin(d/(2f))`
  - Equidistant: `fov = 2d/f`（弧度 → 度）
  - Stereographic: `fov = 4·arctan(d/(2f))`
  - Rectangular: `f` 被忽略（始终全天投影）

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
  "central": [ ... ],
  "elevation": [ ... ],
  "outline": <布尔值>
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `central` | 对象数组 | 否 | [] | 中心网格线配置 |
| `elevation` | 对象数组 | 否 | [] | 仰角网格线配置 |
| `outline` | 布尔值 | 否 | true | 是否显示天球轮廓 |

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
- `render[].resolution` 数组长度必须为 2

### 数值范围验证

- 角度值通常在 -180 到 180 度之间（某些情况下可能超出）
- `crystal[].shape.upper_h` 和 `lower_h` 应在 0.0 到 1.0 之间
- `render[].opacity` 应在 0.0 到 1.0 之间
- `render[].background` 和 `ray_color` 颜色值应在 0.0 到 1.0 之间

### 必填字段验证

- `scene.light_source`: `type`, `altitude`, `spectrum` 必填
- `crystal`: `id`, `type`, `shape` 必填
- `filter`: `id`, `type` 必填
- `scene`: `light_source`, `ray_num`, `max_hits`, `scattering` 必填
- `scene.scattering[].entries[]`: `crystal` 必填
- `render`: `id`, `resolution` 必填
### 类型验证

- `crystal[].type` 必须是 "prism" 或 "pyramid"
- `scene.light_source.type` 必须是 "sun"
- `filter[].type` 必须是 "none"、"raypath"、"entry_exit"、"direction"、"crystal" 或 "complex"
- `render[].visible` 必须是 "upper"、"lower" 或 "full"
- `render[].lens.type` 必须是 "linear"、"fisheye_equal_area"、"fisheye_equidistant"、"fisheye_stereographic"、"dual_fisheye_equal_area"、"dual_fisheye_equidistant"、"dual_fisheye_stereographic" 或 "rectangular"
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

## 相关文档

- [README](../README_zh.md): 用户文档
- [系统架构文档](architecture_zh.md): 系统架构文档
- [配置示例文件](../examples/config_example.json): 配置示例文件
