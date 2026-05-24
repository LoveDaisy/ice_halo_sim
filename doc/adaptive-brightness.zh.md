[English Version](adaptive-brightness.md)

# 自适应亮度（Adaptive Brightness）

本文档介绍 Lumice 的**自适应亮度**功能：算法原理、两种模式（Off / On）的精确语义、Off 模式所保证的物理可加性不变量，以及 UI tooltip 文案的设计理由。

**目标读者**：希望理解为何启用或关闭 Adaptive Brightness 会改变 filter 输出比较方式的进阶用户，以及需要理解 EV 归一化流水线的贡献者。

---

## 1. 概述

冰晕模拟的原始 XYZ 辐射度数值在不同场景配置下量级差异悬殊——数百万条 22° 晕圈光线积累出的 buffer 远亮于稀有弧的 buffer。若无归一化，比较两种配置需要手动调 EV 滑条。

**自适应亮度**自动化了此 EV 调整过程。GUI 计算 EV 偏移量（显示在 checkbox 旁，形如 `+N.NN EV`），并将其应用于后处理流水线，使一定比例的亮像素落在用户可配置的目标亮度附近。

checkbox 位于**右侧面板 → Display** 区域。

---

## 2. 算法

### 2.1 P99 锚点归一化

核心算法（`src/gui/gui_ev_auto.hpp` 中的 `ComputeP99Y` / `ComputeEvAuto`）：

1. **提取** XYZ buffer 中所有正值的 Y 通道。
2. **计算 P99 值**（`y_p99`）：上述 Y 值的第 99 百分位数。
3. **相对 `snapshot_intensity`（总积累强度）归一化**：
   ```
   p99_norm = y_p99 / snapshot_intensity
   ```
4. **将 `p99_norm` 映射到 sRGB [0, 255] 刻度上的 `target_white`**。  
   `target_white` 固定为 200（GUI 已不再暴露滑条；如需进一步调整，由用户拖动手动 EV 滑条）。  
   映射先做 sRGB 传递函数的逆变换，得到线性目标值：
   ```
   t = target_white / 255
   target_linear = (t ≤ 0.04045) ? t / 12.92 : ((t + 0.055) / 1.055)^2.4
   ```
5. **计算 EV 偏移**（单位：曝光档），结果夹到 [−6, +6]：
   ```
   ev = log2(target_linear / p99_norm)
   ```

得到的 `ev` 与手动 EV 滑条值相加后进入后处理通道。若 `p99_norm` 尚不可用（无数据），EV 贡献为 0，GUI 显示 `(no data)`。

### 2.2 数据源选择

P99 由不同 XYZ buffer 计算，取决于当前模式：

| 模式 | P99 数据来源 |
|------|-------------|
| **Off** | `anchor_p99_y`（服务端）——filter-pass + filter-fail 合并射线 P99；详见 `LUMICE_RawXyzResult.anchor_p99_y`，由 `anchor_snapshot_intensity` 归一化 |
| **On**  | `ComputeP99Y(xyz_buffer)`（GUI 端）——已过滤 XYZ buffer 的 P99，由 `snapshot_intensity` 归一化 |

---

## 3. 双模式

### Off——输出物理可比

**Off** 模式下，P99 锚点来自 **anchor lane**——一个在 simulator 内部同时累积 filter-pass 和 filter-fail 射线的发射累积器（见 §5c）。

这意味着：

- 同一次模拟运行的所有 filter 配置共享相同的 EV 刻度。
- 打开或关闭 filter，或在两个互补 filter 之间切换，不会整体偏移亮度——只是改变了显示的光线子集。
- **物理可加性成立**（见 §4）。

推荐场景：

- 并排比较多个 filter。
- 验证一组互补 filter（划分）叠加后等于无 filter 对照实验。
- 了解各 ray-path 类别的绝对贡献。

**性能说明**：Off 模式在正常模拟之外额外运行 anchor pass。开销随 filter-fail 射线量正比增长：

- `ms_prob ≈ 0`（单散射或近零多重散射）：开销可忽略（~0%）。
- `ms_prob ≈ 0.5`：约 +97% 开销（~2× 模拟时长）。

### On——自适应可见性

**On** 模式下，P99 锚点来自**已过滤** buffer：仅当前 filter 存活的光线参与锚点计算。

这意味着：

- 捕获稀有暗弱弧的 filter（存活光线少、绝对亮度低）获得独立的局部 EV 提升——该弧在全局刻度下原本不可见，现在也能清晰显现。
- 切换不同 filter 时，各 filter 的 EV 偏移量相互独立。
- **不保证可加性**（见 §4）。

推荐场景：

- 单独检查某个 filter 的视觉效果。
- 无需手动调 EV，即可让暗弱或稀有弧可见。

---

## 4. 可加性

### 4.1 线性 XYZ 空间——Off 模式

设 F₁, F₂, …, Fₙ 为一组**互补 ray-path filter**——这些 filter 构成全部出射光线的划分（每条光线恰好属于一个 Fᵢ，且没有重复计数）。设 `buf(Fᵢ)` 为激活 filter Fᵢ 时的 XYZ 累积 buffer。

在 **Off** 模式下，所有 buffer 由同一个未过滤锚点归一化，因此：

```
buf(F₁) + buf(F₂) + … + buf(Fₙ) = buf(unfiltered)
```

此等式在**线性 XYZ 色彩空间**中**精确成立**。这是累积通道线性性的直接推论：每条光线的 XYZ 贡献恰好被计入某一个 `buf(Fᵢ)`。

### 4.2 sRGB 像素层面——不可加

sRGB 色彩空间应用非线性 gamma 传递函数（亮区为 `x^(1/2.4)`）。由于 gamma 非线性：

```
sRGB(buf(F₁)) + sRGB(buf(F₂)) ≠ sRGB(buf(F₁) + buf(F₂))
```

对于典型冰晕场景，视觉差异通常较小（暗像素的 gamma 曲线近似线性，亮像素的提亮效果有限），但等式在一般情况下不成立。

**小结**：

| | XYZ 线性空间 | sRGB 像素空间 |
|---|---|---|
| Off 模式，互补 filter | ✅ 精确可加 | ❌ 不可加（gamma 非线性） |
| On 模式 | ❌ 不保证可加 | ❌ 不保证可加 |

### 4.3 On 模式

**On** 模式下，每个 filter 使用各自独立的锚点，归一化系数不同。将两个已归一化的像素 buffer 相加没有物理意义。需要比较或合并 filter 输出时，应使用 Off 模式。

---

## 5. Tooltip 文案设计理由

GUI 在 Adaptive Brightness checkbox 旁显示一个 `(?)` info icon，悬停时展示：

> ON (default): EV anchored to the current filtered image.
>   Filter switches may shift brightness. Simulation is fast.
> OFF: EV anchored to the filter-independent physical baseline.
>   Brightness is stable when switching filters.
>   Simulation is slower (~2x for high multi-scatter probability scenes).
> Switching this mode re-runs the simulation.

文案有意回避了"XYZ 线性空间"、"P99 百分位"、"sRGB gamma"等技术术语。tooltip 面向的是没有色彩科学背景的普通用户，以用户关心的结果来表达关键区别：

- **ON**：每个 filter 根据其可见光线自适应亮度刻度——适合检查原本不可见的暗弱弧。仿真速度快，无额外开销。
- **OFF**：所有 filter 配置共享相同的物理亮度刻度，输出可直接比较。因 anchor lane 而产生额外模拟开销。

需要技术规范的贡献者请参阅本文档和 `src/gui/gui_ev_auto.hpp`。

---

## 5b. 历史：行为变更说明（task-query-filter-uplift-v2）

在本次改动之前，simulator 端会把任何未通过 `scattering.entries[].filter`
的光线标记为已停止（即 T2 前的 `kStopped` 状态，现已并入 `IsTir()` 派生谓词），
这些光线无法抵达 consumer 的 unfiltered accumulator，导致 `unfiltered_xyz_buffer`
实际上是 *post-filter* 的，Off 模式 EV 因而间接受 filter 影响，违反 §4 的可加性不变量。

修复将 simulator 端 filter 降级为纯 branch gate（仅控制 multi-scatter 的
`IsContinue()` 位翻转），filter-fail 光线统一作为 `IsOutgoing()` 释放，consumer 的
Path B 现在累积真正的 unfiltered 全集。

**用户可见影响**：

- 配置了通过率很低的 filter（例如某条 raypath filter 只让极少部分光线通过）的
  场景，Off 模式 EV 会向"更暗"方向偏移，因为 anchor 现在是全光线集而非
  post-filter 子集。通过率 ~0.1% 的 filter 可能让 Off 模式 anchor 下调约 10
  stops。
- On 模式行为不变。
- filtered（On 模式）XYZ buffer 与给定 EV 下最终渲染图像不变。

若依赖旧的 Off 模式行为，请改用 On 模式或手动调整 EV slider。

---

## 5c. Off 模式重设计——F1 Anchor Lane（scrum-221）

> **注意**：本节替换了早期"task-revert（219.4）待实施状态"的描述。
> Off 模式重设计已在 scrum-221（`feat: F1 mode-toggle + anchor lane`）中落地。

task-revert（219.4）恢复 Design A（filter 回归 simulator 端）后，`unfiltered_xyz_buffer`
等同 `xyz_buffer`，Off 模式失去意义。scrum-221 通过 simulator 内部的并行 **anchor lane**
重新设计了 Off 模式。

### F1 机制

Off 模式激活时，simulator 在正常 filter-pass 路径之外运行辅助收集 pass：

```
crystal 退出
    │
    ├─ filter 通过  →  xyz_buffer  +  anchor lane
    │
    └─ filter 失败  →  anchor lane only
                       （不渲染；仅贡献到 EV 锚点）
```

Anchor lane 累积 filter-pass + filter-fail 射线。C API 通过 `LUMICE_RawXyzResult.anchor_p99_y`
和 `anchor_snapshot_intensity` 暴露此数据。GUI（`src/gui/app.cpp` 中的 `SyncFromPoller()`）
在 Off 模式激活时使用这些数据计算 EV 偏移。

这恢复了可加性不变量（§4.1）：同一次运行的所有 filter 配置共享相同的 EV 刻度，
因为锚点来自全量发射光线而非已过滤子集。

### ABI 变更

task-200 引入的旧 `unfiltered_xyz_buffer` 和 `unfiltered_snapshot_intensity` 字段已在
scrum-221 ABI swap 中移除。Off 模式 EV 锚定请使用 `anchor_p99_y` 和
`anchor_snapshot_intensity`。详见 `doc/filter-architecture.md §7`。

---

## 6. 参考

### 代码路径

| 组件 | 文件 | 功能 |
|------|------|------|
| 算法 | `src/gui/gui_ev_auto.hpp` | `ComputeP99Y`、`ComputeEvAuto` |
| 数据源切换 | `src/gui/app.cpp` — `SyncFromPoller()` | ON：`ComputeP99Y(xyz_buffer)` / `snapshot_intensity`；OFF：`anchor_p99_y` / `anchor_snapshot_intensity` |
| GUI 控件 | `src/gui/app_panels.cpp` | checkbox + `(?)` tooltip + EV 显示（`target_white` 固定 200，无滑条） |
| Anchor lane | `src/server/render.cpp` — `Consume()` | Off 模式下累积 filter-pass + filter-fail 射线到 anchor buffer |
| C API 字段 | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `anchor_p99_y` 与 `anchor_snapshot_intensity`（Off 模式）；`xyz_buffer` 与 `snapshot_intensity`（On 模式） |

### 相关文档

- `doc/configuration.md` — 完整 JSON 配置参考
- `doc/gui-guide.md` — GUI 布局与面板概述
- `doc/raypath-symmetry.md` — ray-path 对称性与 filter 语义（P、B、D 开关）
