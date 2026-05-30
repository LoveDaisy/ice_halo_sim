[English Version](adaptive-brightness.md)

# 自适应亮度（Adaptive Brightness）

本文档介绍 Lumice 的**自适应亮度**功能：基于当前帧可见 framebuffer 自动计算 EV 偏移量。

**目标读者**：希望了解 Lumice 如何挑选合理 EV 起点的进阶用户，以及需要理解 EV 归一化流水线的贡献者。

---

## 1. 概述

冰晕模拟的原始 XYZ 辐射度数值在不同场景配置下量级差异悬殊——数百万条 22° 晕圈光线积累出的 buffer 远亮于稀有弧的 buffer。若无归一化，比较两种配置需要手动调 EV 滑条。

**自适应亮度**自动化了此 EV 调整过程。GUI 计算 EV 偏移量（在**右侧面板 → Display**节里手动 EV 滑条旁，形如 `+N.NN EV auto`），并将其应用于后处理流水线，使一定比例的亮像素落在用户可配置的目标亮度附近。

该功能始终启用——GUI 不提供开关。锚点取自当前帧可见 framebuffer（self-P99）。

---

## 2. 算法

### 2.1 P99 锚点归一化

核心算法（`src/gui/gui_ev_auto.hpp` 中的 `ComputeP99Y` / `ComputeEvAuto`）：

1. **提取**可见 XYZ buffer（`data.xyz_data`）中所有正值的 Y 通道。
2. **计算 P99 值**（`y_p99`）：上述 Y 值的第 99 百分位数。
3. **相对每像素 landed 强度归一化**：
   ```
   p99_norm = y_p99 / snapshot_intensity
   ```
4. **将 `p99_norm` 映射到 sRGB [0, 255] 刻度上的 `target_white`**。`target_white` 固定为 135。
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

### 2.2 数据来源

P99 在 poller 线程中由可见 XYZ buffer 计算（`PollerData::p99_y`）；`snapshot_intensity` 为 server 返回的每像素 landed 强度。两个字段无条件填充，没有按 filter 分支。

```cpp
g_state.p99_raw_y = data.p99_y;
g_state.ev_auto = ComputeEvAuto(g_state.p99_raw_y, g_state.snapshot_intensity, target_white);
```

### 2.3 Filter 交互

存在 ray-path filter 时，只有 filter-pass 的射线累积到可见 framebuffer（Design A：filter-fail 射线在 `CollectData` 中立即终止）。P99 / `snapshot_intensity` 二元组因此追踪 filter 子集；切换或开关 filter 通常会改变 EV 刻度，因为分子分母都基于新的可见集计算。

这是有意取舍：之前版本使用 anchor lane 让 EV 在 filter 切换时保持稳定，但内测表明该功能极少使用，且 multi-scattering 开销巨大（`ms_prob=0.5` 时约 2×）。在 `task-remove-anchor-lane` 中移除 anchor lane，回退到更简单的 self-P99 路径。

---

## 3. 参考

### 代码路径

| 组件 | 文件 | 用途 |
|------|------|------|
| 算法 | `src/gui/gui_ev_auto.hpp` | `ComputeP99Y`, `ComputeEvAuto` |
| P99 计算 | `src/gui/server_poller.cpp` | 从 staged XYZ 数据计算 `p99_y` |
| EV 数据源 | `src/gui/app.cpp` — `SyncFromPoller()` | 将 `p99_y` + `snapshot_intensity` 映射为 `ev_auto` |
| GUI 显示 | `src/gui/app_panels.cpp` | `(+N.NN EV auto)` 文本与 `ev_total = exposure_offset + ev_auto` |
| C API 字段 | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `xyz_buffer`, `snapshot_intensity` |

### 相关文档

- `doc/filter-architecture.md` — Design A filter 语义
- `doc/configuration.md` — 完整 JSON 配置参考
- `doc/gui-guide.md` — GUI 布局与面板概览
