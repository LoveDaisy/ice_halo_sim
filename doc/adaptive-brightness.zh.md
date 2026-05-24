[English Version](adaptive-brightness.md)

# 自适应亮度（Adaptive Brightness）

本文档介绍 Lumice 的**自适应亮度**功能：算法原理、支撑它的 F1 anchor lane 机制，以及由此保证的物理可加性不变量。

**目标读者**：希望了解 Lumice 如何在切换 filter 时保持 EV 刻度稳定的进阶用户，以及需要理解 EV 归一化流水线的贡献者。

---

## 1. 概述

冰晕模拟的原始 XYZ 辐射度数值在不同场景配置下量级差异悬殊——数百万条 22° 晕圈光线积累出的 buffer 远亮于稀有弧的 buffer。若无归一化，比较两种配置需要手动调 EV 滑条。

**自适应亮度**自动化了此 EV 调整过程。GUI 计算 EV 偏移量（在**右侧面板 → Display**节里手动 EV 滑条旁，形如 `+N.NN EV auto`），并将其应用于后处理流水线，使一定比例的亮像素落在用户可配置的目标亮度附近。

该功能始终启用——GUI 不再提供开关。Filter 切换不会造成 EV 跳变，因为锚点取自 filter-independent 的 F1 anchor lane。

---

## 2. 算法

### 2.1 P99.5 锚点归一化

核心算法（`src/gui/gui_ev_auto.hpp` 中的 `ComputeP995Y` / `ComputeEvAuto`）：

1. **提取** anchor XYZ buffer 中所有正值的 Y 通道。
2. **计算 P99.5 值**（`y_p995`）：上述 Y 值的第 99.5 百分位数。
3. **相对锚点强度归一化**：
   ```
   p99_norm = y_p995 / anchor_snapshot_intensity
   ```
4. **将 `p99_norm` 映射到 sRGB [0, 255] 刻度上的 `target_white`**。
   `target_white` 固定为 135（GUI 已不再暴露滑条；如需进一步调整，由用户拖动手动 EV 滑条）。
   映射先做 sRGB 传递函数的逆变换，得到线性目标值：
   ```
   t = target_white / 255
   target_linear = (t ≤ 0.04045) ? t / 12.92 : ((t + 0.055) / 1.055)^2.4
   ```
5. **计算 EV 偏移**（单位：曝光档），结果夹到 [−6, +6]：
   ```
   ev_auto = log2(target_linear / p99_norm)
   ```

`ev_auto` 与手动 EV 滑条值相加后进入后处理通道。若数据尚不可用，EV 贡献为 0，GUI 显示 `(auto: no data)`。

### 2.2 数据来源——F1 Anchor Lane

`y_p995` 和归一化强度都来自服务端的 F1 anchor lane：

| 字段 | 含义 |
|------|------|
| `anchor_p995_y` | filter-pass + filter-fail 发射合并集的 Y 通道 P99.5 |
| `anchor_snapshot_intensity` | 同一集合的每像素 landed 强度 |

二者都通过 `LUMICE_RawXyzResult` 暴露（参见 `src/include/lumice.h`）。

**退化路径（无 filter）**：当未配置 filter 时，simulator 跳过 anchor lane（不写 `IsFilterDropped`），server 返回 `anchor_p995_y = 0` 与 `anchor_snapshot_intensity = 0`。此时 `SyncFromPoller()` 回退到 filtered snapshot：

```cpp
if (data.anchor_p995_y > 0.0f && data.anchor_snapshot_intensity > 0.0f) {
    g_state.p995_raw_y = data.anchor_p995_y;
    g_state.ev_auto = ComputeEvAuto(g_state.p995_raw_y, g_state.anchor_snapshot_intensity, target_white);
} else {
    g_state.p995_raw_y = ComputeP995Y(data.xyz_data);
    g_state.ev_auto = ComputeEvAuto(g_state.p995_raw_y, g_state.snapshot_intensity, target_white);
}
```

无 filter 时 filtered snapshot 即为全量发射，两个分支结果数值等价——回退分支只是缺数据保护，并非语义切换。

---

## 3. 行为说明

同一次模拟运行的所有 filter 配置共享相同的 EV 刻度：

- 打开或关闭 filter，或在两个互补 filter 之间切换，不会整体偏移亮度——只是改变了显示的光线子集。
- 互补 ray-path filter 之间**物理可加性成立**（见 §4）。

推荐工作流：

- 并排比较多个 filter，无需重新调 EV。
- 验证一组互补 filter（划分）叠加后等于无 filter 对照实验。
- 了解各 ray-path 类别的绝对贡献。

**性能说明**：当 filter 存在时，simulator 在正常模拟之外额外运行 anchor lane 收集 pass。开销随 filter-fail 射线量正比增长：

- `ms_prob ≈ 0`（单散射或近零多重散射）：开销可忽略（~0%）。
- `ms_prob ≈ 0.5`：约 +97% 开销（~2× 模拟时长）。

未配置 filter 时 anchor lane 始终为空（simulator 跳过 `IsFilterDropped` 写入），无任何额外成本。

---

## 4. 可加性

### 4.1 线性 XYZ 空间

设 F₁, F₂, …, Fₙ 为一组**互补 ray-path filter**——这些 filter 构成全部出射光线的划分（每条光线恰好属于一个 Fᵢ，且没有重复计数）。设 `buf(Fᵢ)` 为激活 filter Fᵢ 时的 XYZ 累积 buffer。

由于每次 filter 运行都用同一个 anchor 强度归一化，因此：

```
buf(F₁) + buf(F₂) + … + buf(Fₙ) = buf(unfiltered)
```

此等式在**线性 XYZ 色彩空间**中**精确成立**。这是累积通道线性性的直接推论：每条光线的 XYZ 贡献恰好落入某一个 `buf(Fᵢ)`。

### 4.2 sRGB 像素层面——不可加

sRGB 色彩空间应用非线性 gamma 传递函数（亮区为 `x^(1/2.4)`）。由于 gamma 非线性：

```
sRGB(buf(F₁)) + sRGB(buf(F₂)) ≠ sRGB(buf(F₁) + buf(F₂))
```

对于典型冰晕场景，视觉差异通常较小（暗像素的 gamma 曲线近似线性，亮像素的提亮效果有限），但等式在一般情况下不成立。

**小结**：

| 合成方式 | XYZ 线性空间 | sRGB 像素空间 |
|---|---|---|
| 互补 filter 共享 anchor | ✅ 精确可加 | ❌ 不可加（gamma 非线性） |

---

## 5. F1 Anchor Lane 机制

当 filter 存在时，simulator 在正常 filter-pass 路径之外运行并行 anchor lane pass：

```
crystal 退出（outgoing 候选）
    │
    ├─ filter 通过  →  xyz_buffer  +  anchor lane
    │
    └─ filter 失败  →  anchor lane only
                       （不渲染；仅贡献到 EV 锚点）
```

Anchor lane 累积 filter-pass + filter-fail 射线。C API 通过 `LUMICE_RawXyzResult.anchor_p995_y` 与 `anchor_snapshot_intensity` 暴露 P99.5 与强度。GUI（`src/gui/app.cpp` 中的 `SyncFromPoller()`）使用这些数据计算 EV 偏移。

`CollectData`（参见 `src/core/simulator.cpp`）的分支表为：

```
filter-pass + prob-pass → IsContinue()       （下一次 MS 散射）
filter-pass + prob-fail → IsOutgoing()        （emit，贡献到 xyz_buffer + anchor）
filter-fail + prob-pass → IsContinue()       （anchor-lane bypass——继续传播）
filter-fail + prob-fail → IsFilterDropped() （仅 anchor lane）
```

`IsContinue()` 与 `IsFilterDropped()` 互斥。完整设计依据参见 `doc/filter-architecture.md §7`。

---

## 6. 历史说明

### 6.1 task-query-filter-uplift-v2 之前——unfiltered buffer 行为

在 task-query-filter-uplift-v2 之前，simulator 端会把任何未通过 `scattering.entries[].filter` 的光线标记为已停止（即 T2 前的 `kStopped` 状态，现已并入 `IsTir()` 派生谓词），这些光线无法抵达 consumer 的 unfiltered accumulator，导致 `unfiltered_xyz_buffer` 实际上是 *post-filter* 的，EV 锚点因而间接受 filter 影响。

修复将 simulator 端 filter 降级为纯 branch gate（仅控制 multi-scatter 的 `IsContinue()` 位翻转），filter-fail 光线统一作为 `IsOutgoing()` 释放，consumer 的 Path B 累积真正的 unfiltered 全集。

### 6.2 scrum-221——双模式 F1 Toggle

scrum-221 引入了 GUI 的 Adaptive Brightness ON/OFF 开关，在 per-frame self-anchor（ON，Design A）和 F1 anchor lane（OFF）之间切换。该开关与 Design A 路径已在 `task-remove-adaptive-brightness-on-mode`（本次改动）中移除——内测验证 F1 anchor 在可接受性能成本下提供严格更好的体验。Anchor lane 实现本身未变，被移除的只是模式分发。

---

## 7. 参考

### 代码路径

| 组件 | 文件 | 功能 |
|------|------|------|
| 算法 | `src/gui/gui_ev_auto.hpp` | `ComputeP995Y`、`ComputeEvAuto` |
| EV 来源 | `src/gui/app.cpp` — `SyncFromPoller()` | 读取 `anchor_p995_y` / `anchor_snapshot_intensity`（两者均为 0 时退化为 filtered snapshot） |
| GUI 显示 | `src/gui/app_panels.cpp` | `(+N.NN EV auto)` 文本与 `ev_total = exposure_offset + ev_auto` |
| Anchor lane | `src/server/render.cpp` — `Consume()` | filter 存在时累积 filter-pass + filter-fail 射线到 anchor buffer |
| Simulator | `src/core/simulator.cpp` — `CollectData()` | F1 分支表；将 filter-fail outgoing 路由至 anchor lane |
| C API 字段 | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `anchor_p995_y`、`anchor_snapshot_intensity`、`xyz_buffer`、`snapshot_intensity` |

### 相关文档

- `doc/filter-architecture.md` §7 — F1 anchor lane 设计与分支语义
- `doc/configuration.md` — 完整 JSON 配置参考
- `doc/gui-guide.md` — GUI 布局与面板概述
- `doc/raypath-symmetry.md` — ray-path 对称性与 filter 语义（P、B、D 开关）
