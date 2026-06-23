# Overlay Label Placement — curve-centric design

> Status: **design / blueprint**（explore-288.5 收敛，2026-06-23）。实施见 task 288.6
> (`label-placement-impl`)。本文从 scratchpad explore 提升为 tracked 设计记录，避免
> 推理丢失（信息价值 = 内在价值 × 被检索到的概率）。

## 范围

GUI 预览区 overlay 文字 **label**（坐标网格的纬度/经度、地平线、角距圈数值）的**放置**
逻辑——即 `src/gui/overlay_labels.cpp` 的 `ComputeOverlayLabels`。不涉及 overlay **线**
的绘制（那是 `preview_renderer.cpp` 片元着色器 `overlayAuxLines`，另见 GUI overlay 的
三套渲染机制说明）。

## 现状（boundary-centric 5-source）与其缺陷

现 `ComputeOverlayLabels` 是 **boundary-centric**：沿若干"边界曲线"采样，再检测网格值在
相邻采样间是否穿越，命中则放 label。共 5 个 sample source：

1. `sample_viewport_edges` — 视口矩形四边（所有 lens）。
2. `sample_hemisphere_equator` — 赤道边界（`!full_sky` 且 visible∈{upper,lower}）。
3. `sample_front_great_circle` — 前半球大圆（`!full_sky` 且 front）。
4. `sample_interior_latitudes` — 纬度环 interior label（`!full_sky` 且 lens≠linear）。
5. `sample_sun_circle_interior` — 角距圈 interior label（`!full_sky` 且 show_sun_circles）。

**实测审计（explore-288.5 实验1，512² 视口 elev=20 visible=full）暴露 4 个独立缺口**：

| projection | 现状 | 根因 |
|------------|------|------|
| globe | 13 个 label 全是纬度、沿 0°经线弧，**零经度** | disc 在视口内 → 边采样无穿越；只有纬度专属 Source 4 兜底 |
| rectangular | 40 个全是纬度（左右边）、**零经度** | full_sky 只跑 Source 1；经线本应穿上下边，但上下边=极点方位奇异，被 `abs(az1-az0)<20` 守卫全拒 |
| dual_fisheye | **0 个 label** | full_sky 只跑 Source 1，两圆盘在方形视口内未命中有效穿越 |
| linear / fisheye | 覆盖够，但**边缘掠射处 label 重复成簇**（顶边 ~15 个重复） | 每个边采样穿越各产一个 label，掠射时一条线多次穿同一边 |

**病根 = boundary-centric**：沿边采样天然漏 disc 内曲线、被极点奇异坑、掠射处成簇。
单点补丁（给 globe 补一个对偶经度 sampler）只能解 1/4，且继续在 5-source 上堆特例。

## 设计：curve-centric 统一放置

反过来，以**曲线**为中心。对每条网格 level-set 曲线（altitude=const / azimuth=const /
sun_dist=const）：

1. 在世界系参数化 walk（扫描另一坐标，如 altitude 曲线扫 azimuth 0..360）。
2. forward 投影到屏幕像素。
3. 裁剪到**可见区域** = 视口矩形 ∩ 投影 disc 有效域 ∩ 半球（visible upper/lower/full + front）。
4. 求可见弧，**二选一 dispatch**：
   - **边界模式**（可见弧触可见区域边界 = 曲线穿出视野）：label 落边界交点。
     覆盖部分环 / 掠射 / 视口边裁剪等所有"穿出"情形。
   - **内部模式**（可见弧 = 完整曲线，没穿出）：label 落曲线 canonical 锚点，
     **锚点数/位置按曲线类型参数化**：
     - 角距环 → 4 个（绕 sun_dir 90° 间隔）
     - 纬圈 / 经线 / 地平线 → 1–2 个（canonical 参考位）

**统一概念**：`label = 网格曲线 ∩ 可见区域边界`；"可见区域边界" = 视口边 ∪ 投影 disc 轮廓
∪ 半球裁剪边。三者统一为同一"可见区域边界"，对应直觉上的三类锚点（特殊经纬值线 /
裁剪边 / 视口边）。

### 为什么一个机制同解 4 缺口

- globe / dual_fisheye 经度：经线 walk 会穿 disc 轮廓 → 边界模式产 label（实验2 实测
  globe 经度 0→36/36，全有边界锚）。
- rectangular 经度：按经线**自身的 azimuth 值**在其视口边出口放 label，绕开"沿边采样
  在极点处算方位"的奇异。
- dual_fisheye：补 forward 投影器后，曲线裁剪到两圆盘 → 有锚点。
- 成簇：每条曲线只产一个（边界模式）或 N 个（内部模式按类型）label，不再是每个边采样
  穿越各产一个。

### 内部模式的一般性

"完整环在内"不止角距环：globe 正对极点（elev≈±90）时纬圈是 disc 内同心圆、地平线是
完整圈，同样触发内部模式。角距环的"4 label"只是它的内部锚点参数。现 Source 4/5 被
收编为两类曲线的内部模式，而非并列特例。

## 锚点取点规则（实施细化，task 288.6 + 复核修正）

边界模式"label 落边界交点"对**纬圈/角距环**直接取可见弧的 invis→vis 入口即可（这些
曲线的入口点彼此分离）。但**子午线（经度曲线）不能用通用入口锚点**：所有子午线都过极点，
而极点（alt=±90，az 无关 → `(0,0,±1)` 投影到同一像素）常落在可见区域边界上（globe /
dual_fisheye 的 disc 轮廓），导致每条子午线的入口都是那个极点汇聚点 → **全部经度 label
堆叠成一个像素**（draw 时 bbox 抑制后只剩 ~1 个）。这是 288.6 首版的退化缺陷，独立空间
分析（探针实测 globe 36 经度全在 (256,370)、dual_fisheye 全在 (384,256)）抓到。

**修正规则（owner 选定）**：经度 label 锚在子午线与**参考纬度环的交点**——优先地平线
alt=0（可见时），否则可见范围内最接近赤道的纬度（从 alt=0 样本向外搜首个可见样本）。
低纬度处子午线方位间距最大 → label 彼此分离。此规则**对所有 projection 统一应用**
（rectangular 等亦如此，更"在线上"），修复退化且不退化已正常的 lens。实测修复后 globe /
dual_fisheye 经度 x-span 114/256px → 227/384px，各点唯一。

## 实施要点（task 288.6）

1. **full_sky lens 需补 forward 投影器**：`overlay_labels::WorldDirToPixel` 当前对
   rect/dual_fisheye/dual_ortho 直接 `return {0,0,false}`。需补 forward 数学——
   **沿用 overlay_labels 已镜像 shader 公式的本地实现模式**（不直接 `#include` core，
   遵守 GUI↔core API 边界）；core `src/core/projection.hpp` 的 `RectangularForward` /
   `FisheyeXxxForward` 是镜像参考。dual_fisheye 需处理"哪个圆盘"分支。
2. **内部模式 per-type 锚点**：角距 4 个、纬经地平线 1–2 个。
3. **半球 / front 裁剪**：curve walk 时复用现有 `is_visible` 谓词。
4. **dedup / 遮挡**：每曲线一个/少数 label 大幅缓解；保留 draw 时 bbox 抑制作第二道。

## 验收（task 288.6）

- globe / rectangular 出现经度 label；dual_fisheye 不再零 label；角距完整环 4 锚点；
  linear/fisheye 边缘不再重复成簇。
- 现有 `overlay_labels` 单测（clamp_label_pos / project_world_dir）不回归；新增
  curve-centric 覆盖单测。
- GUI 视觉硬核验（导出各 projection label 实图）。

## 取证资产

explore-288.5 探针代码（白盒，直接调 `ComputeOverlayLabels` / 加 forward hook 跑 curve
walk）存于 `scratchpad/scrum-overlay-marker-lines/explore-overlay-label-placement/`
（`probe_labelset.cpp.txt`、`probe_curvecentric.cpp.txt`），288.6 可复用作覆盖测试基础。
