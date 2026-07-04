# 近极面积测度采样设计（Near-Pole Area-Measure Sampling）

> 状态：设计蓝图（explore-326 GO + scrum-328 子任务 328.1 收敛，2026-07-04）。
> 改近极朝向采样 / GPU device gen sampler / Rayleigh 路 / `ComputeJacobianEnvelope` 前先读。
> 关联：`crystal-orientation-sampling.md`、`numerical-robustness.md`、`gpu-single-engine-implementation.md`。

## 1. 问题

近极纬度采样（plate 晶体 c 轴竖直，如幻日/parry；config `zenith≈0` → 内部 `latitude≈90°`）走常数包络拒绝采样（`SampleSphericalPointsSph` generic rejection）时，接受率 `cos(φ)/M` 随极点 `cos(φ)=sinθ→0` 崩溃。实测端到端吞吐税：**Gaussian −14% / Laplacian −15~17% / uniform −86%**（Metal device gen，explore-326）。这是"常数包络跟不了 `cosθ` 衰减"的**方案固有天花板**——无论 M 多优，近极 mean attempts 仍 3.7(gauss)/4.9(lap)。同时它是 task-325 光柱偏绿的同一机制层根（拒绝循环耗尽 PCG slots → 波长×朝向碰撞）。

## 2. 方案：面积测度重要性采样（消除拒绝，保持精确）

换到余纬 `θ = 90° − latitude`（近极 θ→0）。球面面积测度目标密度：

```
target(θ) ∝ base_pdf(θ) · sinθ        （sinθ = cos(latitude) 是面积元 Jacobian）
```

利用**严格上界 `sinθ ≤ θ`（∀θ ≥ 0，非近似）**：

- **提案** `q(θ) ∝ base_pdf(θ) · θ`（把线性 Jacobian `θ` 吸进提案）
- **接受概率** `sinθ / θ`（`M = 1` 是最紧常数，`sinθ/θ ≤ 1` 恒成立 → **永不 clamp**）
- accepted ∝ `base_pdf(θ)·θ · (sinθ/θ) = base_pdf(θ)·sinθ` = target → **精确（零分布误差）**，接受率 `E[sinθ/θ] ≈ 99%`（3σ 尾部）。

这即 owner 直觉「近似上界 + 拒绝 = 精确低浪费」的实现：`sinθ ≤ θ` 是那个免费的严格上界。

### 2.1 各分布提案闭式（差别仅在此）

| 分布 | `base_pdf(θ)·θ` | 提案采样 |
|------|-----------------|----------|
| **Gaussian(μ=±90, σ)** | `θ·exp(−θ²/2σ²)` = `Rayleigh(σ)` | `θ = σ·√(−2 ln u)`（= pole 处 2D 高斯幅值，即现 Rayleigh 路）|
| **Laplacian(μ=±90, b)** | `θ·exp(−θ/b)` = `Gamma(2, b)` | `θ = −b·(ln u₁ + ln u₂)` |
| **uniform（bounded 近极）** | 常数 | `M = cos(max(\|mean\|−std/2, 0)°)`（P0，非 tight-envelope）|

> ⭐ 推翻 explore-326「Laplacian 无闭式对偶」：folded Laplacian 近极余纬 = `Exp(b)`，`θ·Exp(b) = Gamma(2,b)` 有闭式采样。

### 2.2 触发（跨越点）

`mean ≈ ±90°`（colat_center ≈ 0，**任意 σ**）→ fast-path；否则 fallback generic rejection。
- 容忍 colat_center ≲ 0.5°（mean 偏离极点 <0.5°）；≥1° 即失配。
- fast-path 对任意 σ 都精确且都赢（实测 σ≤60° 都 exact，σ=1° 100% vs 26.5% / σ=60° 70% vs 59%）。
- off-pole 近极（colat_center 1~10°）走 fallback（25-30% 接受，不灾难），未来可上 P2 泛化（局部线性上界 + 拒绝）。

### 2.3 `sinθ/θ` 数值

`theta > eps ? sin(theta)/theta : 1.0`（θ→0 极限 = 1）。提案 θ>0 恒成立，仅 u≈1 时 θ≈0 触发 guard。放共享头单源。

## 3. 精确性验证纪律（⭐AC 硬约束）

proposed 采样器**有意偏离**当前采样器（更精确）：
- 修当前 Laplacian generic rejection 的 `M=cos(5b)` **尾部 clamp 偏差**（>5b 处 `sinθ/M>1` → clamp 至 1 → 尾部欠权重；实测 KS 0.006 可检测）。
- 移除当前 Rayleigh 路（`math.cpp:481-496`，无接受步骤）的 `sinθ≈θ` 偏差（实测 <500k 噪声底，但存在）。

**故 parity 绝不可 bit/KS-match 当前采样器**——须对**解析目标 `∝exp(−θ/b)·sinθ`** 或 `current(M=1)`（无 clamp 参照）验证。天真"匹配旧行为"的 parity 测试会对正确改进的代码判 FAIL。

## 4. 三后端接线矩阵

| 层 | 位置 | 改动 |
|----|------|------|
| **device sampler（单源→Metal shader+CUDA）** | `pcg_shared.h::sample_lat_lon_roll` L269-320 | Rayleigh 分支 L281-290 加 `sinθ/θ` 接受；新增 Laplacian Gamma(2,b) 分支 + `pcg_gamma2` helper；新/扩 `lat_path` 常量 |
| **CPU sampler** | `math.cpp::SampleSphericalPointsSph` L444-534 | 镜像同上 |
| **host path-selection ×3** | `math.cpp:462-475` / `metal_trace_backend.mm:1456-1484` / `cuda_trace_backend.cu:296-324` | 触发放宽（mean≈±90 任意 σ）；⭐建议下沉共享 `SelectLatPath(axis_dist)→{lat_path, rejection_m}` 消 3× 镜像 |
| **envelope ×3** | `math.cpp:422` / `metal.mm:202` / `cuda.cu:256` | uniform P0 修（M=cos min纬）；tight-envelope 路 M=1 |
| **struct 镜像** | `GenRootKernelParams`（pcg_shared.h）↔ `metal.mm:159` | 布局自动化门禁 |

> `pcg_shared.h` 被 `lumice_trace.metal`(shader) + `cuda_trace_backend.cu` + `metal_trace_backend.mm` 同编 → device sampler 一处即两 GPU 后端；但 host 触发/envelope 三后端手工镜像。

## 5. device RNG-stream 可观测性设施 co-design

近极 sampler 三后端 parity 是 backlog「device RNG-stream 一等可观测性设施」的第 2 消费者（第 1 = task-325 的 5 个 test-only hook）。设施（3 组件）：
1. **raw-gen harness**：只驱动 gen kernel + 读回（朝向 lat/lon/roll + attempt count），补齐 Metal 侧缺失的 `ReadbackGenDirsForTest`（现只有 `ReadbackRootRotForTest`），跨三后端对称。
2. **显式选流 enum 探针**：`enum RngStream{gen, gate_ms1, gate_final, transit}` + payload `{raw_uN, attempt_count, lat_lon_roll}`，取代隐式时机路由 + 裸 tid（加 `ci_start` 偏移）。
3. **`TraceBackendTestHooks` struct**：test-only inject/readback 从生产 backend public 接口剥离，消除现有 Metal/CUDA 方法不对称。
+ 债：`SelectLatPath()` / `ComputeJacobianEnvelope` 下沉共享头；struct 布局门禁；scene helper 去重。

## 6. 落地任务

scrum-328 `near-pole-area-measure-sampling`：328.2（设施）→ 328.3（统一核心 + Gaussian + uniform）→ 328.4（Laplacian）。CUDA 硬件验证后置 dev49（与 task-325 wl_stream FU 合并）。

## 附：一手实测锚点（explore 328.1，Python MC，a01 双锚对齐 explore-326 真 C++ 插桩）

- Laplacian b5：当前 20.4%（att 4.90 ≈ 真码 4.91）→ 提案 99.3%；KS(提案 vs current M=1)匹配。
- Gaussian σ5：当前 26.8%（att 3.76 ≈ 真码 3.73）→ 提案 99.8%；KS(提案 vs ground truth)=0.0026<crit。
- 复现脚本：`scratchpad/scrum-near-pole-area-measure-sampling/explore-unified-sampler-impl-details/exp{1,2,3}_*.py`。
