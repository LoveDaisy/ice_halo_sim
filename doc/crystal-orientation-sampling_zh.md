# 晶体取向采样

本文档描述 Lumice 在蒙特卡洛冰晕模拟中如何采样晶体 c 轴取向。

## 物理模型

大气中的冰晶在空气动力学和重力作用下具有优先取向。晶体取向分解为两个独立分量：

1. **c 轴方向**（天顶角 + 方位角）：晶体主对称轴的指向。重力约束天顶角；方位角均匀分布（无优先水平方向）。
2. **自旋角**（绕 c 轴旋转）：与轴向方向独立。

这种分解是物理上精确的，不是近似。c 轴方向分布在球面上形成**纬度带**（zonal band，纬度集中、经度均匀），而**非**旋转对称的球冠（cap）。

## 配置

JSON 配置中通过以下格式指定晶体取向：

```json
"axis": {
  "zenith": { "type": "gauss", "mean": 90, "std": 0.5 },
  "roll": { "type": "uniform", "mean": 0, "std": 360 }
}
```

- `zenith.mean`：与垂直方向的夹角（度）（0 = c 轴垂直，90 = 水平）
- `zenith.std`：展宽（度）（高斯标准差）
- 内部转换：`latitude_mean = 90 - zenith_mean`

## 采样方法

### 目标分布

对于高斯天顶角，余纬 `θ` 的目标分布为：

```
p(θ) ∝ G(θ - θ₀, σ) × sin(θ)
```

其中：
- `θ` 为余纬（= 天顶角，弧度）
- `θ₀` 为目标余纬
- `σ` 为标准差（弧度）
- `sin(θ)` 为球面面积元的 Jacobian 项

`sin(θ)` 项至关重要：缺少它时，球面坐标采样在极区产生错误的密度分布（`sin(θ) → 0` 使极点密度发散），导致样本过度集中于极点。

### 为什么不用 von Mises-Fisher（vMF）？

vMF 产生围绕目标方向旋转对称的**球冠**分布。冰晶需要的是**纬度带**分布（天顶角集中、方位角均匀）。在赤道处（`θ₀ = π/2`），vMF 会将方位角也约束在小范围内，这在物理上不正确。vMF 仅在极点处（带退化为冠）符合需求。

### 实现：混合 Rayleigh + Rejection

`math.cpp` 中的 `SampleSphericalPointsSph(AxisDistribution)` 函数使用两条路径：

**1. Rayleigh 路径**（`colatitude_center < 0.5°` 时）：

c 轴几乎在极点时，在切平面上使用二维高斯：

```
dx, dy ~ N(0, σ)
colatitude = √(dx² + dy²)
latitude = sign(mean) × (π/2 - colatitude)
```

这产生 Rayleigh 分布 `p(θ) ~ θ × exp(-θ²/2σ²)`，在小 `θ` 时等于 `sin(θ) × G(θ, σ)`（因为 `sin(θ) ≈ θ`）。在 `θ₀ = 0` 时精确，无需 rejection，每样本 O(1)。

**2. 优化 rejection**（其他情况）：

```
M = cos(max(|latitude_mean| - 3σ, 0))
do:
    φ = 从 Gaussian(latitude_mean, σ) 采样
    将 φ 钳位到 [-π/2, π/2]
while uniform() >= cos(φ) / M
```

接受率：
- 赤道附近（`zenith ≈ 90°`）：~100%（`cos(φ) ≈ 1`）
- 中纬度（`zenith ≈ 45°`）：~70-80%
- 近极区（`zenith ≈ 5°`）：~30-40%

包络 `M = cos(max(|mean| - 3σ, 0))` 由探索验证的余纬公式 `M = sin(min(θ₀ + 3σ, π/2))` 通过恒等式 `sin(π/2 - x) = cos(x)` 推导得出。

### 非高斯类型

- `kNoRandom`：直接返回固定均值（不采样）
- `kUniform`：使用原始逻辑，不做 Jacobian 修正（已知限制：`kUniform` latitude + 非 `kUniform` azimuth 组合存在同样问题，但实际中极少见）
- 完全均匀（latitude 和 azimuth 均为 `kUniform`）：走独立的均匀球面采样代码路径（`asin(u)` 方法）

## 历史背景

原始实现直接从 `Gaussian(mean, std)` 采样 latitude，缺少 `cos(φ)` Jacobian。影响随配置不同而异：

| 天顶角 | 失真系数 | 角距 std 偏差 |
|--------|----------|---------------|
| 90° | 1.0x | < 0.1% |
| 45° | 1.4x | -0.5% |
| 10° | 5.8x | -8.8% |
| 0° | ∞ | -7.9%，密度过高 8x |

该修正通过 explore-zenith-sampling（2026-04）识别，同时评估并排除了 vMF、Matrix Fisher、Bingham 等替代分布。纬度带模型和 Jacobian 修正方案被确认为数学严谨且计算高效。

## 参考

- 探索总结：`scratchpad/scrum-sim-sampling-research/explore-zenith-sampling/SUMMARY.md`
- Shoemake (1992)：SO(3) 均匀随机旋转（背景参考）
- Wood (1994)：von Mises-Fisher 分布采样（本用例已排除 vMF）
