[English Version](adaptive-brightness.md)

# 自适应亮度（Adaptive Brightness）

本文档介绍 Lumice 的**自适应亮度**功能：基于当前帧可见 framebuffer 自动计算 EV 偏移量。

**目标读者**：希望了解 Lumice 如何挑选合理 EV 起点的进阶用户，以及需要理解 EV 归一化流水线的贡献者。

---

## 1. 概述

冰晕模拟的原始 XYZ 辐射度数值在不同场景配置下量级差异悬殊——数百万条 22° 晕圈光线积累出的 buffer 远亮于稀有弧的 buffer。若无归一化，比较两种配置需要手动调 EV 滑条。

**自适应亮度**自动化了此 EV 调整过程。GUI 计算 EV 偏移量（在**右侧面板 → Display**节里手动 EV 滑条旁，形如 `+N.NN EV auto`），并将其应用于后处理流水线，使一定比例的亮像素落在用户可配置的目标亮度附近。

锚点本身**始终计算、始终显示**——这一点没有开关。有开关的是"计算出的曝光是否真的被采用"：
Display 分组的 **Mode** 控件（Relative / Absolute，见 §3）在两套曝光公式间切换，本文的 P99 self-anchor
只被其中一套消费。以下第 2 节描述的是 `Relative` 模式——它是默认值，且行为与本文最初记录的单模式行为一致。

---

## 2. 算法（Relative 模式）

### 2.1 P99 锚点归一化

核心算法（`ComputeP99Y` / `ComputeEvAuto`，均在 `src/core/ev_anchor.hpp` 中——经 C API
`LUMICE_ComputeP99Y` / `LUMICE_ComputeEvAuto` 到达）：

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

> ⚠️ **不要把这件事与 `Absolute` 模式的 filter 无关性（§3）混为一谈。**
> `Absolute` 模式的归一化是 filter 无关的——filter 改变了落到像素上的东西，却不再改变显示尺度，
> 因为分母是光源**发射**的能量，在 filter 生效之前就已计入。这**不是**上面被移除的 anchor lane 复活：
> anchor lane 是靠额外累积一份 filter 无关的发射统计量、以真实的 multi-scattering 代价（filter-fail
> 的光线仍需继续追迹才能喂给它）来让 *self-P99 锚点本身*在 filter 切换时保持稳定。`Absolute` 模式完全
> 不碰 P99 锚点或 `ev_auto`——该模式下它根本不被消费（§3）——而且零额外成本：发射能量在光线追迹之前、
> 发射的那一刻就已确定。这两个"filter 无关"表面相似，机制与代价却完全不同；不要把其中一个当作另一个的复活。

---

## 3. Absolute 模式

`Absolute` 是 Display 分组 **Mode** 控件的第二个取值（第一个是 `Relative`，见 §1–2，仍是默认值）。
它不改变第 2 节的算法——`ev_auto` 仍然照旧由 P99 锚点算出——它改变的是**算出的曝光是否被使用**：

- 曝光分母从每像素 landed 强度（`snapshot_intensity`）换成光源**发射**的能量
  （`snapshot_emitted_energy`，来自 `LUMICE_RawXyzResult::emitted_energy`）——由光源与光线预算固定，
  不受 filter、场景通过率或 lens 裁剪影响。这正是让不同配置的两次渲染在同一 EV 下可比较绝对亮度的
  机制，也是 `Absolute` 模式存在的目的。精确定义、实测的 `landed_fraction` 位移规律与已记录的跨 lens
  边界见 [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §7。
- `ev_auto` **不会**被加进曝光。把一个逐帧变化的自动锚点悄悄叠进一个本该读作"比物理亮/暗几档"的数字上，
  会让这个读数重新落在一个会移动的基线上，违背其本意。自动锚点仍会被计算并显示在 EV 读数旁——标注为
  未生效——因为一个 UI 不再更新的值仍然占着一个标签位，不标注反而像是它悄悄失效了。
- EV 读数的形态本身也不同：`Relative` 显示手动 + 自动 = 有效值（三个数字，因为在稀疏场景下自动锚点
  可能在手动值下方移动多档）；`Absolute` 只显示手动偏移量，相对物理量。

composite（raypath-color）路径用同样的方式选择，与 mono 路径共享同一份绝对尺度而非另行推导（见
[`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §2.6）——`Absolute` 模式下 mono 与
composite 在同一 EV 下依然可比。

`ev_mode` 是随 config/`.lmc` 保存的逐文档字段，不是全局偏好，且被排除在重仿真字段集合之外——切换它会
立即重绘当前帧而不触发重跑。config 键见 [`doc/configuration.md`](configuration.md)，完整机制见
[`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md)（§2.6、§4、§7）。

---

## 4. 参考

### 代码路径

| 组件 | 文件 | 用途 |
|------|------|------|
| 算法（单一 owner） | `src/core/ev_anchor.hpp` | `ComputeP99Y`, `ComputeEvAuto`，经 C API 到达 |
| P99 计算 | `src/gui/server_poller.cpp` | 从 staged XYZ 数据计算 `p99_y` |
| EV 数据源 | `src/gui/app.cpp` — `SyncFromPoller()` | 将 `p99_y` + `snapshot_intensity` 映射为 `ev_auto` |
| 模式感知曝光（单一 owner） | `src/gui/mono_exposure_scale.hpp` | `ComputeMonoExposure()`——按 `ev_mode` 分叉；供显示/导出/`.lmc` 缩略图共用 |
| GUI 显示 | `src/gui/app_panels.cpp` | Mode 下拉、EV 读数文本 |
| C API 字段 | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `xyz_buffer`, `snapshot_intensity`, `emitted_energy` |
| C API 字段 | `src/include/lumice.h` — `LUMICE_RenderParam` | `ev_mode`（`LUMICE_EV_MODE_RELATIVE` / `LUMICE_EV_MODE_ABSOLUTE`） |

### 相关文档

- [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) — 完整内部机制、两种模式、发射能量的定义与诚实边界（§7）
- `doc/filter-architecture.md` — Design A filter 语义
- `doc/configuration.md` — 完整 JSON 配置参考
- `doc/gui-guide.md` — GUI 布局与面板概览
