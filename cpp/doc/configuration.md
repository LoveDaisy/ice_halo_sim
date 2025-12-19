# V3 配置文档

**版本**: V3  
**最后更新**: 2025-12-19

本文档详细说明 V3 版本的配置文件格式、各配置项的含义、默认值以及配置验证规则。

## 配置概述

V3 版本使用 JSON 格式的配置文件，配置文件包含以下主要部分：

- `light_source`: 光源定义数组
- `crystal`: 晶体定义数组
- `filter`: 过滤器定义数组
- `scene`: 场景定义数组
- `render`: 渲染器定义数组
- `project`: 项目定义数组

**重要提示**：
- 样例配置文件（`v3_config_example.json`）并未穷举所有合法的配置写法
- 大量配置参数具有默认值，如果不写会使用默认值
- **必须参考代码实现**来了解完整的配置逻辑和默认值
- 本文档基于代码实现提取了所有默认值信息

## 配置项详细说明

### light_source（光源配置）

光源配置定义模拟中使用的光源属性。

#### 基本结构

```json
{
  "id": <唯一标识符>,
  "type": "sun" | "streetlight",
  "altitude": <角度>,
  "azimuth": <角度>,
  "diameter": <角度>,
  "wavelength": [<波长数组>],
  "wl_weight": [<权重数组>]
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `id` | 整数 | 是 | - | 唯一标识符，必须大于0 |
| `type` | 字符串 | 是 | - | 光源类型："sun" 或 "streetlight" |
| `altitude` | 浮点数 | 是 | - | 地平高度（度） |
| `azimuth` | 浮点数 | 否 | 0.0 | 方位角（度） |
| `diameter` | 浮点数 | 否 | 0.5 | 直径（度），真实太阳通常为0.5 |
| `wavelength` | 浮点数组 | 是 | - | 波长数组（nm），必须与 `wl_weight` 长度相等 |
| `wl_weight` | 浮点数组 | 是 | - | 权重数组，必须与 `wavelength` 长度相等 |

#### 示例

```json
{
  "id": 1,
  "type": "sun",
  "altitude": 20.0,
  "azimuth": 0,
  "diameter": 0.5,
  "wavelength": [420, 460, 500, 540, 580, 620],
  "wl_weight": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
}
```

#### 注意事项

- `wavelength` 和 `wl_weight` 数组长度必须相等
- 波长决定折射率，数据来自 [Refractive Index of Crystals](https://refractiveindex.info/?shelf=3d&book=crystals&page=ice)
- `azimuth` 和 `diameter` 是可选的，如果不指定会使用默认值

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

场景配置定义模拟场景，包括光源、晶体组合和光线数量。

#### 基本结构

```json
{
  "id": <唯一标识符>,
  "light_source": <光源ID>,
  "ray_num": <整数>,
  "max_hits": <整数>,
  "scattering": [ ... ]
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `id` | 整数 | 是 | - | 唯一标识符 |
| `light_source` | 整数 | 是 | - | 光源ID引用 |
| `ray_num` | 整数 | 是 | - | 光线数量，-1表示自动计算 |
| `max_hits` | 整数 | 是 | - | 最大碰撞次数 |
| `scattering` | 数组 | 是 | - | 散射配置数组 |

#### scattering（散射配置）

每个散射配置项结构：

```json
{
  "crystal": [<晶体ID数组>],
  "proportion": [<比例数组>],
  "prob": <概率>,
  "filter": [<过滤器ID数组>]
}
```

**字段说明**：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `crystal` | 整数数组 | 是 | - | 晶体ID数组 |
| `proportion` | 浮点数组 | 否 | 100.0 | 比例数组，长度必须等于 `crystal` 数组长度 |
| `prob` | 浮点数 | 否 | 0.0 | 多散射概率 |
| `filter` | 整数数组 | 否 | -1 | 过滤器ID数组，长度必须等于 `crystal` 数组长度，-1表示无过滤器 |

**示例**：

```json
{
  "id": 1,
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
  "ray": [<r>, <g>, <b>],
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
| `ray` | 浮点数组 | 否 | [-1, -1, -1] | 光线颜色 RGB，-1表示使用真实颜色 |
| `opacity` | 浮点数 | 否 | 1.0 | 透明度 |
| `intensity_factor` | 浮点数 | 否 | 1.0 | 强度因子 |
| `grid` | 对象 | 否 | 见下方 | 网格配置 |
| `filter` | 整数数组 | 否 | [] | 多散射过滤器ID数组 |

#### lens（镜头配置）

```json
{
  "type": "linear" | "fisheye" | "dual_fisheye_equal_area" | "dual_fisheye_equidistant" | "rectangular",
  "fov": <角度>  // 或 "f": <焦距>
}
```

**默认值**：
- `type`: "linear"
- `fov`: 90.0（度）

**注意**：可以使用 `fov`（视场角，度）或 `f`（焦距，mm），如果使用 `f`，程序会自动计算对应的 `fov`。

#### view（视角配置）

```json
{
  "azimuth": <角度>,
  "elevation": <角度>,
  "roll": <角度>,
  "distance": <距离>
}
```

**默认值**：
- `azimuth`: 0.0
- `elevation`: 0.0
- `roll`: 0.0
- `distance`: 未指定时使用默认值

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

### project（项目配置）

项目配置组合场景和渲染器。

#### 基本结构

```json
{
  "id": <唯一标识符>,
  "scene": <场景ID>,
  "render": [<渲染器ID数组>]
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `id` | 整数 | 是 | - | 唯一标识符 |
| `scene` | 整数 | 是 | - | 场景ID引用 |
| `render` | 整数数组 | 是 | - | 渲染器ID数组 |

#### 示例

```json
{
  "id": 1,
  "scene": 2,
  "render": [1, 2, 4]
}
```

## 配置验证规则

### ID 唯一性验证

- 所有配置节中的 `id` 必须在各自类型内唯一
- `id` 必须大于 0

### ID 引用有效性验证

- `scene.light_source` 引用的光源ID必须存在于 `light_source` 数组中
- `scene.scattering[].crystal` 引用的晶体ID必须存在于 `crystal` 数组中
- `scene.scattering[].filter` 引用的过滤器ID必须存在于 `filter` 数组中（或为-1）
- `project.scene` 引用的场景ID必须存在于 `scene` 数组中
- `project.render` 引用的渲染器ID必须存在于 `render` 数组中

### 数组长度匹配验证

- `light_source[].wavelength` 和 `wl_weight` 数组长度必须相等
- `scene.scattering[].proportion` 数组长度必须等于 `crystal` 数组长度
- `scene.scattering[].filter` 数组长度必须等于 `crystal` 数组长度（如果指定）
- `crystal[].shape.face_distance` 数组长度必须为 6（如果指定）
- `crystal[].shape.upper_indices` 数组长度必须为 3（如果指定）
- `crystal[].shape.lower_indices` 数组长度必须为 3（如果指定）
- `render[].resolution` 数组长度必须为 2

### 数值范围验证

- 角度值通常在 -180 到 180 度之间（某些情况下可能超出）
- `crystal[].shape.upper_h` 和 `lower_h` 应在 0.0 到 1.0 之间
- `render[].opacity` 应在 0.0 到 1.0 之间
- `render[].background` 和 `ray` 颜色值应在 0.0 到 1.0 之间

### 必填字段验证

- `light_source`: `id`, `type`, `altitude`, `wavelength`, `wl_weight` 必填
- `crystal`: `id`, `type`, `shape` 必填
- `filter`: `id`, `type` 必填
- `scene`: `id`, `light_source`, `ray_num`, `max_hits`, `scattering` 必填
- `render`: `id`, `resolution` 必填
- `project`: `id`, `scene`, `render` 必填

### 类型验证

- `crystal[].type` 必须是 "prism" 或 "pyramid"
- `light_source[].type` 必须是 "sun" 或 "streetlight"
- `filter[].type` 必须是 "none"、"raypath"、"entry_exit"、"direction"、"crystal" 或 "complex"
- `render[].visible` 必须是 "upper"、"lower" 或 "full"
- `render[].lens.type` 必须是 "linear"、"fisheye"、"dual_fisheye_equal_area"、"dual_fisheye_equidistant" 或 "rectangular"

## 常见配置错误

### 1. ID 未定义错误

**错误描述**：引用了不存在的ID

**错误示例**：
```json
{
  "scene": [
    {
      "id": 1,
      "light_source": 999  // 错误：光源ID 999 不存在
    }
  ]
}
```

**正确示例**：
```json
{
  "light_source": [
    { "id": 1, ... }
  ],
  "scene": [
    {
      "id": 1,
      "light_source": 1  // 正确：引用已定义的光源
    }
  ]
}
```

### 2. 数组长度不匹配

**错误描述**：`proportion` 数组长度与 `crystal` 数组长度不一致

**错误示例**：
```json
{
  "scattering": [
    {
      "crystal": [1, 2, 3],
      "proportion": [10, 20]  // 错误：长度不匹配
    }
  ]
}
```

**正确示例**：
```json
{
  "scattering": [
    {
      "crystal": [1, 2, 3],
      "proportion": [10, 20, 30]  // 正确：长度匹配
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
      "type": "HexPrism"  // 错误：V3版本应使用 "prism"
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
      "type": "prism"  // 正确：V3版本使用 "prism"
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
  "crystal": {
    "id": 1,  // 错误：crystal 应该是数组
    "type": "prism"
  }
}
```

**正确示例**：
```json
{
  "crystal": [  // 正确：crystal 是数组
    {
      "id": 1,
      "type": "prism",
      "shape": { "height": 1.3 }
    }
  ]
}
```

## V3 迁移指南

### 概述

V3 版本的配置格式与旧版本有较大变化。本节说明如何从旧版配置迁移到 V3 配置。

### 主要变化

#### 1. 光源配置

**旧版**：
```json
{
  "sun": {
    "altitude": 20.0,
    "diameter": 0.5
  },
  "ray": {
    "number": 1000000,
    "wavelength": [420, 460, 500, 540, 580, 620]
  }
}
```

**V3版本**：
```json
{
  "light_source": [
    {
      "id": 1,
      "type": "sun",
      "altitude": 20.0,
      "azimuth": 0,
      "diameter": 0.5,
      "wavelength": [420, 460, 500, 540, 580, 620],
      "wl_weight": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    }
  ]
}
```

**变化说明**：
- `sun` → `light_source` 数组
- 需要添加 `id` 和 `type` 字段
- `wavelength` 从 `ray` 移动到 `light_source`
- 需要添加 `wl_weight` 数组
- `ray.number` 移动到 `scene.ray_num`

#### 2. 晶体配置

**旧版**：
```json
{
  "crystal": {
    "type": "HexPrism",
    "parameter": [1.4],
    "axis": {
      "mean": 90,
      "std": 0.3,
      "type": "Gauss"
    },
    "roll": {
      "mean": 0,
      "std": 360,
      "type": "Uniform"
    }
  }
}
```

**V3版本**：
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": {
        "height": 1.4
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
    }
  ]
}
```

**变化说明**：
- `type: "HexPrism"` → `type: "prism"`
- `parameter` → `shape.height`
- `axis` 结构变化：从单一值变为 `zenith`、`azimuth`、`roll` 三个字段
- 分布类型名称变化：`"Gauss"` → `"gauss"`，`"Uniform"` → `"uniform"`
- `crystal` 从对象变为数组

#### 3. 多散射配置

**旧版**：
```json
{
  "multi_scatter": {
    "repeat": 2,
    "probability": 0.2
  }
}
```

**V3版本**：
```json
{
  "scene": [
    {
      "id": 1,
      "light_source": 1,
      "ray_num": 1000000,
      "max_hits": 7,
      "scattering": [
        {
          "crystal": [1, 2, 3],
          "prob": 0.2
        }
      ]
    }
  ]
}
```

**变化说明**：
- `multi_scatter` → `scene.scattering` 数组
- `repeat` 不再需要，由 `max_hits` 控制
- `probability` → `prob`
- 需要明确指定晶体ID数组

#### 4. 渲染配置

**旧版**：
```json
{
  "camera": {
    "azimuth": -50,
    "elevation": 30,
    "rotation": 0,
    "fov": 40,
    "width": 1920,
    "height": 1080,
    "lens": "linear"
  },
  "render": {
    "visible_semi_sphere": "upper",
    "background_color": [0, 0, 0],
    "ray_color": "real"
  }
}
```

**V3版本**：
```json
{
  "render": [
    {
      "id": 1,
      "lens": {
        "type": "linear",
        "fov": 40
      },
      "resolution": [1920, 1080],
      "view": {
        "azimuth": -50,
        "elevation": 30,
        "roll": 0
      },
      "visible": "upper",
      "background": [0, 0, 0],
      "ray": [-1, -1, -1]
    }
  ]
}
```

**变化说明**：
- `camera` 和 `render` 合并为 `render` 数组
- `width`、`height` → `resolution` 数组
- `rotation` → `view.roll`
- `visible_semi_sphere` → `visible`
- `background_color` → `background`
- `ray_color: "real"` → `ray: [-1, -1, -1]`

### 迁移步骤

1. **创建光源配置**
   - 将 `sun` 配置转换为 `light_source` 数组
   - 添加 `id`、`type`、`wl_weight` 字段

2. **创建晶体配置**
   - 将旧版晶体配置转换为 V3 格式
   - 更新类型名称（`HexPrism` → `prism`）
   - 更新参数结构（`parameter` → `shape`）
   - 更新方向配置（`axis` → `zenith`、`azimuth`、`roll`）

3. **创建场景配置**
   - 将 `ray.number` 移动到 `scene.ray_num`
   - 将 `multi_scatter` 转换为 `scene.scattering`
   - 添加 `max_hits` 字段

4. **创建渲染配置**
   - 合并 `camera` 和 `render` 为 `render` 数组
   - 更新字段名称和结构

5. **创建项目配置**
   - 创建 `project` 数组，组合场景和渲染器

### 注意事项

1. **不兼容的变化**：
   - 配置结构完全重构，无法直接兼容
   - 需要手动迁移配置

2. **功能增强**：
   - 支持多场景、多渲染器批量处理
   - 更灵活的过滤器系统
   - 更细粒度的配置控制

3. **推荐做法**：
   - 参考 `v3_config_example.json` 作为模板
   - 逐步迁移，先迁移简单场景
   - 验证迁移后的配置是否正确

## 配置最佳实践

### 性能优化建议

1. **光线数量设置**：
   - 测试时使用较小的 `ray_num`（如 10000）
   - 生产环境根据需求调整，通常 1000000 以上

2. **多场景配置**：
   - 利用 V3 的批量处理能力，一次配置多个场景
   - 避免重复定义相同的晶体和光源

3. **过滤器使用**：
   - 合理使用过滤器可以减少不必要的计算
   - 在 `scene.scattering` 中使用 `filter` 字段

### 常见场景配置模板

#### 简单日晕模拟

```json
{
  "light_source": [
    {
      "id": 1,
      "type": "sun",
      "altitude": 20.0,
      "diameter": 0.5,
      "wavelength": [550],
      "wl_weight": [1.0]
    }
  ],
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": { "height": 1.2 }
    }
  ],
  "scene": [
    {
      "id": 1,
      "light_source": 1,
      "ray_num": 1000000,
      "max_hits": 7,
      "scattering": [
        { "crystal": [1] }
      ]
    }
  ],
  "render": [
    {
      "id": 1,
      "resolution": [1920, 1080],
      "lens": { "type": "linear", "fov": 40 }
    }
  ],
  "project": [
    {
      "id": 1,
      "scene": 1,
      "render": [1]
    }
  ]
}
```

#### 多晶散射模拟

```json
{
  "scene": [
    {
      "id": 1,
      "light_source": 1,
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
    }
  ]
}
```

### 调试配置建议

1. **小规模测试配置**：
   - `ray_num`: 100 或更小
   - 使用单一波长（`wavelength: [550]`）
   - 减少晶体数量

2. **详细日志配置**：
   - 运行程序时使用 `-v` 或 `-d` 选项
   - 查看配置解析日志

## 相关文档

- [README_zh.md](../README_zh.md): 用户文档
- [architecture.md](architecture.md): 系统架构文档
- [v3_config_example.json](../v3_config_example.json): 配置示例文件
