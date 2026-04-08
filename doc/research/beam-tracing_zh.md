[English](beam-tracing.md)

# 光束追踪（Beam Tracing）

本文档描述了作为蒙特卡洛（MC）光线追踪替代方案而探索的实验性光束追踪（BT）方法。
实现位于 `exp/beam_tracing` 分支，**未合并到 main**——本文档说明了方法、结果，
以及为什么成本收益权衡不支持采纳。

## 动机

标准 MC 光线追踪对每条光线采样一条随机路径。对于每个晶体朝向，需要大量光线来估计出射光
分布。光束追踪用精确的几何计算替代了逐光线的随机采样：给定晶体朝向和光照方向，解析计算
**所有**出射光束及其精确的方向和面积加权 Fresnel 强度——完全消除入射点采样方差。

理论吸引力在于方差缩减：根据全方差定律，

```
Var(f) = Var_R[E_s(f|R)] + E_R[Var_s(f|R)]
```

其中 `R` 为朝向，`s` 为入射点采样。BT 精确消除了第二项（朝向内方差）。

## 算法

### 概述

对于给定的晶体朝向和光照方向，`BeamTrace()` 在晶体内部执行 BFS：

1. **入射面检测**：识别 `dot(light_dir, face_normal) < 0` 的面。对每个入射面计算投影
   面积，创建初始光束（折射进入晶体的部分）。

2. **BFS 传播**：处理队列中的每个光束：
   - `PartitionBeam` 确定光束截面将击中哪些内部面
   - 对每个命中面：计算 Fresnel 系数，产生折射出射光束（离开晶体）和反射续行光束
     （保留在队列中）
   - 低于最小面积阈值的续行光束被剪枝

3. **终止**：队列为空或达到 `max_hits` 深度时 BFS 结束。

### PartitionBeam：面平面投影 + Sutherland-Hodgman 裁剪

核心几何运算是将光束截面分配到候选下一面：

1. **候选面选择**：找到 `dot(beam_dir, face_normal) > eps` 的内部面（面向光束方向），
   排除当前面。

2. **投影**：提取每个候选面的 3D 多边形顶点，沿光束方向做平行投影到光束的 2D 截面平面
   （`u = dot(v, basis_u)`, `v = dot(v, basis_v)`）。

3. **交集计算**：使用 Sutherland-Hodgman 裁剪（`IntersectConvexPolygons`）计算光束
   截面多边形与每个投影候选面的交集。

4. **结果**：每个非空交集成为一个分区——击中特定面的子光束，携带父光束权重的对应比例。

晶体凸性保证分区不重叠。

### API

```cpp
struct BeamTraceResult {
  std::vector<float> outgoing_d;                   // 出射方向，每个 3 个 float
  std::vector<float> outgoing_w;                   // 面积加权 Fresnel 权重
  std::vector<std::vector<int>> outgoing_raypath;  // 每个光束的面序列
  float total_entry_area = 0.0f;                   // 总投影入射面积
};

BeamTraceResult BeamTrace(const Crystal& crystal, const Rotation& rot,
                          const float* light_dir, float refractive_index,
                          size_t max_hits);
```

## 正确性验证

### 固定朝向逐光束匹配

对于平面晶面、固定朝向和点光源，每个唯一的光路（面序列）产生恰好一个出射方向。MC 和 BT
应产生相同的离散出射方向集合及匹配的权重。

`FixedOrientationMcVsBt` 测试验证了这一点：
- 500K MC 光线 vs 1 次 BT 朝向（固定 azimuth=45°, zenith=30°, roll=15°）
- 将 MC 和 BT 输出分别按方向聚类（余弦阈值 0.99999）
- 比较每个匹配聚类的归一化权重

**修复 bug 后的结果**：Pearson 相关系数 **0.999998**，所有显著聚类的相对差异在 2.1% 以内。

### 发现并修复的 Bug

1. **重投影坐标不匹配**（主因）：反射光束时，截面多边形重建为 `p = u*bu + v*bv`，丢失了
   沿光束方向的分量。这导致后续 `PartitionBeam` 调用中与绝对面坐标投影的坐标不一致。
   修复：通过面平面交点计算实际 3D 命中点后再重投影。Pearson 从 0.948 提升到 0.999998。

2. **反弹深度差一**：BT 处理了 `max_hits + 1` 次面交互，而 MC 为 `max_hits` 次。
   修复：将深度阈值从 `>= max_hits` 调整为 `>= max_hits - 1`。

3. **max_hits=0 下溢**：`max_hits` 为 0 时 `size_t` 减法下溢。修复：添加提前返回保护。

## 性能分析

### 单位成本：比 MC 慢 219 倍

| 指标 | MC | BT |
|------|----|----|
| 配置 | 1000 万光线 | 100 万朝向 |
| 墙钟时间 | 2.0s | 46.6s |
| 每单位 CPU 成本 | ~2 μs/光线 | ~437 μs/朝向 |

219 倍的比值分解为两个因素：

- **步数放大（~32 倍）**：MC 每条光线追踪约 4 次有效反弹。BT 穷举探索完整 BFS 树：
  约 3.5 个入射面 × 分支因子约 2 × 深度 6，剪枝后约 130 个 BFS 节点/朝向。

- **单步成本（~10 倍）**：每个 BFS 节点需要 `ExtractPolygonFaceVertices`（扫描 20 个
  三角面 + 去重 + 排序，约 400 次操作 × 约 3.5 个候选面）和 `IntersectConvexPolygons`
  （Sutherland-Hodgman O(V_a × V_b)，约 250 次操作 × 约 3.5 个候选面），而 MC 只需
  简单的光线-平面板交（约 200 次操作）。

### 方差：BT 并不更好

通过 5 次运行的两两 PSNR 衡量运行间一致性：

| 方法 | 配置 | 平均两两 PSNR | 时间 |
|------|------|---------------|------|
| MC | 1000 万光线 | **28.9 dB** | ~3s |
| BT | 100 万朝向 | **28.2 dB** | ~50s |

**BT 的一致性略*低于* MC**，尽管慢了 16 倍。

### 原因：方差分解

关键洞察在于：对于给定晶体朝向的平行光，出射光束在球面上形成**有限个离散 delta 分布**
（每个光路对应一个方向）。离散分布收敛极快——MC 只需几十个入射点采样就能精确估计每个
delta 的权重。BT 消除的朝向内方差本来就微不足道。

另一方面，朝向采样位于 **SO(3) 流形**（3 维、连续）上，收敛速度为 O(1/√N)。这是主要
噪声来源，而 BT 对此毫无帮助——它与 MC 采样朝向的方式完全相同，只是因为每个朝向的
计算成本高 219 倍，同等时间内的朝向数少了 10 倍。

简言之：BT 花费约 219 倍的计算量精确求解了一个 MC 用约 30 个样本就能充分解决的子问题，
而真正的瓶颈（朝向采样）没有任何改善。

## 集成状态

`exp/beam_tracing` 分支包含与模拟管线的完整集成：

- **CLI 输出**：BT 数据通过 `RenderConsumer` 流转产生图像
- **渲染级过滤器**：通过 2 节点 `RaySeg` 链变通方案（entry + outgoing）支持，
  使 `FilterRay` 链式遍历可工作。限制为 1 层渲染级过滤器。
- **配置**：场景配置 JSON 中 `"use_beam_tracing": true`
- **E2E 测试**：`test/e2e/configs/halo_22_bt.json` 含 PSNR 阈值

## 分支引用

所有光束追踪代码位于 `exp/beam_tracing`（9 个 commit，未合并到 main）：

```
0ab370f feat(beam-tracer): add beam tracing core geometry module
f62d3e8 feat(beam-tracer): integrate beam tracing into Simulator
4551953 fix(beam-tracer): fix BT sampling bias + MC vs BT distribution test
cc887fb refactor(beam-tracer): rewrite PartitionBeam with face-plane projection
1439c1f refactor(core): deduplicate InitRay_rot + HitSurface, move to geo3d
913dc50 feat(config): add use_beam_tracing JSON serialization + BT e2e config
d0fb711 fix(server): BT CLI output by replacing rays_.Empty() shutdown sentinel
440ba26 fix(beam-tracer): fix re-projection coordinate mismatch + per-beam test
3482b59 feat(beam-tracer): render-level filter via 2-node RaySeg chain
```

## 潜在优化方向（未实现）

如果未来重新考虑 BT，主要的优化机会有：

1. **缓存 `ExtractPolygonFaceVertices`**（占 BFS 成本约 40%）：对于固定晶体，8 个多边形
   面的顶点集合在所有 BFS 节点和朝向间相同。当前每个朝向重复提取约 455 次。
   预估提速 1.5-1.7 倍。

2. **候选面预计算**：对每个源面，候选下一面可从晶体几何预计算，而非每个 BFS 节点扫描
   所有面。

3. **混合方法**：仅对前 1-2 次反弹使用 BT（精确分区入射光束有价值），然后切换到 MC
   处理更深层反弹（Fresnel 衰减使精确计算的价值降低）。
