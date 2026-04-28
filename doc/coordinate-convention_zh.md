[English version](coordinate-convention.md)

# 坐标系与旋转约定

> **破坏性变更（v3）。** v3 版本重构了旋转链。来自旧版本、含 `crystal.axis.*` 字段的配置
> 渲染结果将出现方向偏移，可能需要按本文档下方的新约定（含 `azimuth` 符号约定与 `−180°` 链
> 偏移）重新编写。`filter.raypath` 与 `light.*` / `view.*` 不受影响。

本文档定义 Lumice 在晶体姿态、光源位置、相机视角等场景下使用的坐标系、轴约定与旋转链。所有数值示例默认以度为单位，弧度场景会显式注明。

## 1. 晶体局部坐标系

晶体局部坐标系随网格固定，跟随晶体一起旋转。约定为右手系：

- `N1`（1 号面，顶面基面）外法向 → 局部 `+z`（即 c-axis）
- `N3`（3 号面）外法向 → 局部 `+x`
- `N1 × N3` → 局部 `+y`（右手定则）

其余面由六棱柱对称性自然推出，详见 `src/core/crystal.cpp::FillHexFnMap` 中的法向表。

`Nx` 表示 `x` 号面的外法向单位向量（由晶体内部指向外部）。本文档中 c-axis 始终指 N1 方向。

## 2. 世界坐标系

世界坐标系固定且为右手系：

- `+z` 指向天顶
- `xy` 平面为水平面（halo 模拟中即为地平面）
- `+x` 是参考方位角方向（详见 §3）

世界坐标系是晶体姿态采样和相机视角共用的参考系。

## 3. 方位角约定

Lumice 使用**数学习惯**的方位角约定：

- `az = 0°` 对应世界 `+x`
- 方位角增加方向为绕 `+z` 自上而下俯视呈**逆时针**
- 与地理约定（北 = 0°、向东递增）相反

该约定统一适用于所有方位角字段：`crystal.axis.azimuth`、`scene.light_source.azimuth`、`render[].view.azimuth`。

## 4. 光源位置

光源方向由 `(altitude, azimuth)` 参数化：

- `altitude`（也称太阳高度角）∈ [0°, 90°]：0° 在地平线，90° 在天顶
- `azimuth` 遵循 §3 约定；`azimuth = 0°` 把光源置于 `+x` 方向
- `diameter` 控制日盘的角张度

`SampleRayDir`（在 `src/core/simulator.cpp` 中）发射光子方向**指向**观察者（即与太阳位置向量相反）；这是采样侧约定，对用户配置不可见。

## 5. 典型晶体姿态（世界坐标）

以下姿态是四个内置 axis preset 的特征姿态。每条描述固定**均值**朝向；实际采样朝向叠加 §7 中的 Gauss / Uniform 扰动。

### 5.1 Plate（片晶）

- `N1` 指向世界 `+z`（c-axis 垂直）
- 晶体绕 `N1`（即世界 `+z`）自由旋转

### 5.2 Column（柱晶）

- `N1` 位于 `xy` 平面（c-axis 水平）
- `N1` 绕世界 `+z` 旋转（由 `azimuth` 采样）
- 晶体绕自身 `N1` 自由旋转（由 `roll` 采样）

### 5.3 Parry

- `N3` 指向世界 `+z`
- 晶体绕 `N3`（即世界 `+z`，由 `azimuth` 采样）自由旋转；`roll` 锁定在 0° 附近以保持 `N3` 稳定向上

### 5.4 Lowitz

- `N3 × N1` 位于 `xy` 平面
- `N3 × N1` 绕世界 `+z` 旋转（由 `azimuth` 采样）
- `zenith` 带较大的 Gauss 扰动（默认 σ ≈ 40°），c-axis 围绕天顶大幅摆动——这一大 σ 是 Lowitz 视觉特征的来源，并非 chain 中的独立项

## 6. 旋转链

Lumice 对所有 preset 与自定义配置使用单一旋转链。给定一组采样 `(azimuth, zenith, roll)`（度），局部到世界的旋转矩阵为：

```
R(azimuth, zenith, roll) = Rz(azimuth − 180°) · Ry(−zenith) · Rz(roll)
```

链作用于局部向量时**自内向外**展开：

1. `Rz(roll)` 绕局部 c-axis（与世界 `+z` 重合于初始时刻）
2. `Ry(−zenith)` 绕世界 `−y`
3. `Rz(azimuth − 180°)` 绕世界 `+z`

`Rn(θ)` 是绕轴 `n` 旋转 `θ` 的标准右手旋转矩阵。

实现见 `lumice::BuildCrystalRotation(azimuth_rad, latitude_rad, roll_rad)`（`src/core/simulator.hpp`），其中 `latitude_rad = π/2 − zenith_rad`，对应内部球面采样使用的 latitude 约定。

## 7. Preset 默认采样参数

| Preset | zenith              | azimuth                | roll                    |
|--------|---------------------|------------------------|-------------------------|
| Plate  | Gauss(μ=0°, σ)      | Uniform [0°, 360°)     | Uniform [0°, 360°)      |
| Column | Gauss(μ=90°, σ)     | Uniform [0°, 360°)     | Uniform [0°, 360°)      |
| Parry  | Gauss(μ=90°, σ)     | Uniform [0°, 360°)     | Gauss(μ=0°, σ) locked   |
| Lowitz | Gauss(μ=0°, σ_L)    | Uniform [0°, 360°)     | Gauss(μ=0°, σ) locked   |

"locked" 表示 GUI 把分布类型固定为 Gauss，仅 σ 可由用户调整。默认 σ 取值参见 `src/gui/edit_modals.cpp::kAxisPresets`（当前 Plate / Column / Parry σ = 1°，Lowitz σ_L = 40°）。

各 preset 在运行时的视觉差异主要来自**分布形态**（Uniform vs locked Gauss、大 σ vs 小 σ），而非均值参数差异。Column 与 Parry 在默认下共享相同的 `(μ_az, μ_zenith, μ_roll)` 三元组，仅在 `roll` 采样方式上不同。

## 8. azimuth − 180° 偏移

azimuth 项的 `−180°` 偏移由局部坐标架选择 `N3 = +x` 决定。若不加偏移，Parry 默认 `(zenith = 90°, azimuth = 0°, roll = 0°)` 会把 `N3` 映射到世界 `−x` 而非 `+z`，与 §5.3 描述矛盾。

Parry 默认下的数值验证：

```
R = Rz(−180°) · Ry(−90°) · Rz(0°)

N1_world = R · (0, 0, 1)
         = Rz(−180°) · Ry(−90°) · (0, 0, 1)
         = Rz(−180°) · (−1, 0, 0)
         = (+1, 0, 0)             → +x

N3_world = R · (1, 0, 0)
         = Rz(−180°) · Ry(−90°) · (1, 0, 0)
         = Rz(−180°) · (0, 0, 1)
         = (0, 0, +1)             → +z   ✓
```

直观上，`−180°` 偏移把"配置中的 azimuth = 0°"对齐到"Parry 姿态下 N1 在 +x 侧"，符合用户在"`+x` 即光源方向"参考下配置晶体的自然预期。

## 9. 相机视角约定

相机坐标架独立于晶体坐标架，由 `render[].view` 下的 `(elevation, azimuth, roll)` 参数化。

### 9.1 朝向

相机在世界坐标下的 forward 方向为：

```
forward = (cos(elevation) · cos(azimuth),
           cos(elevation) · sin(azimuth),
           sin(elevation))
```

等价于：

- `elevation = 0°, azimuth = 0°` → forward = `+x`
- `elevation = 90°` → forward = `+z`（向上看）
- `elevation = 0°, azimuth = 90°` → forward = `+y`

实现见 `lumice::BuildViewMatrix`（`src/gui/preview_renderer.cpp` 及 core 中等价路径）。

### 9.2 方位与高度的符号约定

`view.azimuth` 遵循 §3 同一数学约定（自 `+z` 看 `+x` 起逆时针为正）。

`view.elevation`：正值表示向上看，`elevation = -10°` 表示稍微低于地平线方向。

### 9.3 Roll

`view.roll` 绕相机 forward 轴旋转。正值 roll 使得渲染图像在观察者眼中**逆时针**旋转（即相机局部 `+x` 朝局部 `+y` 旋转）。

### 9.4 与光源的关系

view 方位与光源方位采用**同一**方位角约定但相互独立。`light_source.azimuth = 0°`（光源在 `+x`）配合 `view.azimuth = 0°` 时，相机面向光源。

一个常见的 halo 截图约定是 `view.azimuth = 180°`，即相机背向光源、太阳在观察者身后；该约定与 §6 的 chain 约定完全独立。

### 9.5 字段语义不变

§6 chain 重构不影响 `view.*` 字段的取值与语义。已有 config / `.lmc` 文件中 `view.*` 字段保持渲染同样的相机取景。

## 10. 持久化兼容

### 10.1 语义变化字段（破坏性）

- `crystal.axis.{zenith, azimuth, roll}`：§6 chain 改动后，相同数值会渲染出不同朝向。旧 `.lmc` / `config.json` 文件渲染结果会改变。

### 10.2 语义保持字段

- `filter.raypath`：面编号物理位置不变。`src/core/crystal.cpp` 中 `ref_norms[]` 未修改，`raypath = [3, 5]` 仍指向同一对物理面。
- `light.*`、`view.*`、`crystal.shape.*`、`filter.symmetry`：不变。

### 10.3 迁移策略

不提供自动迁移脚本。用户对照 §5 重新撰写 `crystal.axis.*` 字段。

- 确定性 axis 值（如 `axis.zenith = 90`），通常只需翻一个符号或 180°，肉眼对照渲染结果即可调整
- 概率分布（Gauss / Uniform）下分布形态不变，只是采样产生的渲染朝向变化

#### 朝向 delta 速查（确定性 axis）

下表对比同一 `(azimuth, zenith, roll)` 输入在新旧 chain 下 N1 / N3 的世界朝向，用于重写固定朝向配置时的快速核对。

| (azimuth°, zenith°, roll°) | 旧 N1 → world | 新 N1 → world | 旧 N3 → world | 新 N3 → world |
|----------------------------|---------------|---------------|---------------|---------------|
| (0, 0, 0)                  | +z            | +z            | +x            | −x            |
| (0, 90, 0)                 | +x            | +x            | −z            | +z            |
| (180, 90, 0)               | −x            | −x            | −z            | +z            |
| (90, 0, 0)                 | +z            | +z            | +y            | −y            |

观察规律：
- 当 `zenith = 0`（Plate 类）且 `roll = 0` 时，N3 沿 `±x` 翻转；若旧配置依赖 N3 指向 `+x`，新 chain 下需把 `azimuth` 加 180°
- 当 `zenith = 90`（Column / Parry）时，N3 在 `±z` 间翻转；如果旧配置想让"N3 朝天顶"（Parry 语义），新 chain 下在 `azimuth = 0°` 直接成立，而旧 chain 需额外调整
- 使用 `azimuth = Uniform[0°, 360°)` 全向采样的配置统计不变，不需要迁移

## 11. 验证

实现正确性由三层独立验证：

1. **数学层**：`test/test_simulator.cpp` 单元测试（`BuildCrystalRotation.CaseA_AzOffsetOnly` ... `CaseD_RollAroundCAxis`）以 4 条数学可分辨的输入分别探查 chain 各项
2. **结构层**：GUI 缩略图（`src/gui/thumbnail_cache.cpp`）展示 §5 中的典型 preset 姿态
3. **物理层**：E2E 与 GUI 参考图（`test/e2e/references/*.jpg` / `test/gui/references/*.png`）核验 halo pattern 在 chain 改动前后的稳定性。所有 axis 全方位均匀采样的配置在 chain 改动下统计不变，参考图无需重新生成

任意一层验证失败时，按以下优先级排查：

- `azimuth − 180°` 偏移符号（§8）
- 方位角符号约定（§3）
- 中层 `Ry(−zenith)` 符号（§6）

## 附录：chain 输出速查表

下表的 4 个用例数学可分辨，独立探查 chain 各项，对应 `test/test_simulator.cpp` 中的断言。

| Case | (az°, zenith°, roll°) | N1_world (= R · ê_z) | N3_world (= R · ê_x) | 验证目标             |
|------|------------------------|----------------------|----------------------|----------------------|
| A    | (0, 0, 0)              | +z                   | −x                   | az − 180° 偏移       |
| B    | (0, 90, 0)             | +x                   | +z                   | Ry(−zenith) 符号     |
| C    | (90, 90, 0)            | +y                   | +z                   | Rz(az − 180°) 非平凡 az |
| D    | (0, 0, 90)             | +z                   | −y                   | Rz(roll)             |

容差：`Dot3(N_actual, N_expected) > 1 − 1e-5`。

Preset 与 Case 的对应关系：

- Plate 默认（zenith = 0°）：与 Case A 等价（不计随机 roll / az）
- Parry 默认（zenith = 90°, roll ≈ 0°）：与 Case B 等价（不计随机 az）
- Column 默认（zenith = 90°, roll Uniform）：均值层与 Parry 相同；preset 差异在 `roll` **分布**上，不在 chain 输出上
- Lowitz 默认（zenith = 0° mean，大 σ）：均值层与 Plate 相同；c-axis 因大 σ 在天顶附近大幅摆动
