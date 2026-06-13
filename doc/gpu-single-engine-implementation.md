# GPU 单引擎实现设计（seam-design §5 落地）

> 本文是 §5 单引擎 GPU simulator **重写**的实现设计与上下文锚，供后续 scrum 引用。
> 配套（读这些获取完整推理）：
> - `doc/seam-design.md` —— §5 目标架构蓝图（§3.6 原始之罪 / §4.5 filter 出口 / §5 统一 seam 形态）。
> - `doc/gpu-route-history.md` —— GPU 迁移 #250→266 全程回顾。
> - `scratchpad/explore-gpu-single-engine/{SUMMARY,insights,experiments}.md` —— explore-266 de-risk 细节（gitignored，本文固化其 durable 结论）。

## 1. 状态与目标

- **当前**：Metal 是 GUI **opt-in**（`use_metal_backend` 复选框，默认关）；走 **12-worker + host-side MS 续传**（`metal_trace_backend.mm` kernel 写 continuation，host `CopyContSliceToRootBuf` 逐光线做 filter+prob+frame-transit）。这是 258-265 把已 de-risk 零件**焊进 legacy 结构**的产物，非 §5。
- **legacy CPU 始终是 GUI 默认实走路径 + 永久 ground truth**（perf 基线 + 正确性参照）。
- **目标**：实现 seam-design §5 —— 单引擎、三时钟解耦（几何采样 / trace 派发 / 图像回读各自频率）的 wavefront GPU simulator，替换 12-worker + host-side MS 编排。CPU 不重设计，留作可信 oracle。

## 2. explore-266 关键结论（固化）

1. **divergence 裁决 = 伪命题**。层内是**线性链**：每跳必有一支离开（收集进 buffer、不再参与本层）、一支留内继续（TIR 无人离开；weight 衰竭提前止）→ 一层出射 = **O(root × max_hits) 线性**（非递归树）→ 一线程一根光线寄存器内跑线性链 = megakernel 干净、divergence-free。跨 MS 层每条离开光线独立 prob 续传 → 几何增长（explore-257 amp 单层 4.81→两层 24.65）→ **wavefront across layers**。**§5 = 层内 megakernel + 层间 wavefront**。
2. **device 续传 filter 可行（实测 de-risk）**。raypath filter 唯一非平凡 per-ray 核 `detail::ReduceBuffer`（对称折叠）移植 MSL，对真实 host **1.44 亿检查 0 mismatch**、无寄存器压力、1.3-2.7G/s。**§4.5"对称折叠依赖晶体配置→filter 须留 consumer"是悲观误判**——它混淆了 orbit **构建**（config 依赖、host/每 session 一次）与 per-ray **匹配**（有界整数核、device 逐位可移植）。逐类型：None/Direction/Crystal=trivial；Raypath/EntryExit=ReduceBuffer+GetFn 表+memcmp+长度界；Complex=布尔组合——**全 device 可行**。
3. **吞吐（harvest 257/263/265，非重测）**。单个当前式 Metal 引擎仅 ~18K rays/s（< legacy 50K）；12-worker 把 host 串行 hop 并行到 12 核才达 225K（W8 饱和）。§5 单引擎要胜出须**双 lever 且都做满**：大 dispatch（摊薄 launch 延迟）+ device-resident 续传（消 host hop），两者正交 = 三时钟解耦。
4. **correctness gap（E5-E7，**已知、不单独修、由本重写顺带解决**）**。当前 Metal 多 MS + 读 recorder/对称的 filter 对 legacy **+16% 能量**；受控隔离矩阵定位到 host `CopyContSliceToRootBuf` 对续传光线的**解耦 filter 消费**（GetFn / orbit 构建 crystal-axis 的 hop_* slot 状态），非匹配逻辑（E4 逐位）、非单层、非多晶体、非 prob 机制。

## 3. 设计原则（硬约束）

- **legacy = ground truth；当前 Metal 代码至多作参考，不照搬**（owner 定调：可推翻重写，别过早陷入"修当前实现"的局部陷阱）。
- **统计等价验收**（seam-design §3.6），度量用 **raw-XYZ parity**（`GetRawXyzResults` block-mean corr，避开 sRGB tonemap 失真），覆盖单/多 MS × 有/无 filter，对 legacy。
- **device 续传 gate 融进 kernel 逐跳 emit**（非"把 host hop 搬上 device"）：每 emit 一条离开光线（此刻 `path[]`=路径至今在寄存器）当场 device prob+filter+frame-transit → 续传写 continuation buffer / 输出写 exit buffer / fail 丢。三重收益：①逐位对齐 legacy `CollectData`（它就是逐跳对每条离开光线用"路径至今"判 filter，simulator.cpp:426）；②**天然修掉 §2.4 的 +16% bug**（其根因正是解耦的 host 重建；inline 用寄存器路径至今，单 MS exit 已证此 path 对 → correct-by-construction）；③E4 证融入零额外代价。
- **几何单源**：晶体构造留 CPU（`MakeCrystal`），device 侧零几何代码（复杂稀少留 CPU/每 session，简单海量上 GPU/逐线程）。
- **架构为 CUDA 承接**：按离散内存语义设计，不焊统一内存假设（M2 上 host hop 廉价的结论不可外推 PCIe）。

## 4. 实现弧（2 scrum + 后续）

| 阶段 | 性格 | 交付 |
|------|------|------|
| **Scrum 1：device 续传引擎**（本文 §5） | 实现导向（设计已由 266 完成） | 正确 + device-resident 多 MS 续传（仍在现有编排内，每 worker 续传上 device）；correctness 地基 + host hop→device 部分吞吐 |
| **Scrum 2：单引擎编排**（本文 §6） | explore 先行（仍有设计不确定性） | 吞吐胜负手（大 dispatch + 单引擎 async + 释放 CPU 核）+ concern #2 解 |
| Phase 3：CUDA（dev49/RTX4060） | —— | 整个 GPU 迁移第二步，backlog 单独跟踪，**不在本 Metal 弧内** |

依赖：device gate 局部于 kernel + 续传 buffer，与"1 个还是 12 个引擎"正交 → Scrum 1 可独立先行；Scrum 2 在其上把编排塌成单引擎。

## 5. Scrum 1 范围：device 续传引擎

**做**：
- device 续传 gate 融进 trace kernel 逐跳 emit（device prob via PCG + device filter-match via ReduceBuffer 移植 + device frame-transit via device-root-gen 取向采样）。
- continuation 全程 device-resident（host 仅 counter + 重 dispatch），移除 host hop `CopyContSliceToRootBuf` 的逐光线 host 处理。
- 多 MS filter 正确性 correct-by-construction（gate 内联 = legacy 语义）。
- **raw-XYZ parity 验收**：单/多 MS × 有/无 filter（含 raypath PBD/BD + complex），对 legacy 恢复统计等价；用 owner 构造的 `ms3-multi-crystal-filter.json` 范式场景。
- golden-ray 解析锚（backlog:438）：给真 kernel 续传路径装确定性 golden 光线 + 解析期望，补"多反弹 trace loop / 面求交 / 续传"的 absolute 锚缺口（本轮 parity bug 暴露此盲区）。

**可复用零件**（266 + 现有 Metal 作参考，非照搬）：trace kernel optics（Fresnel/refr/TIR）、device root-gen（PCG，scrum-260）、exit-seam atomic compaction、富出射记录 schema（scrum-258.2 inline-only）、`ReduceBuffer` MSL 移植（E4 spike `scratchpad/explore-gpu-single-engine/spike/`）、CpuTraceBackend oracle + parity 测试网。

**验收**：raw-XYZ parity 对 legacy 在 filtered + multi-MS 场景恢复（当前 +16% 消除）；统计等价（§3.6）。

## 6. Scrum 2 范围：单引擎编排（explore 先行）

- explore 设计：单大 dispatch + async 引擎替 12-worker（12-worker 本质是延迟隐藏 hack，explore-263 证 W8 饱和）；**commit↔batch 解耦**（concern #2，backlog:565/571——`LUMICE_BATCH_RAY_NUM` 当前一身二职=GPU dispatch 粒度 + SimData 到达粒度，须解耦使 GUI 用大 GPU batch + 细 commit 响应）；GPU 大 dispatch 内部如何增量 drain/commit 给 consumer；CPU 核预算。
- **几何池 / concern #5**（晶体几何参数随机变动 → batch 内多晶体几何、per-ray 索引）在此评估或延后（§6 待定项，257 exp#5 证 memory divergence +20%@K1024 modest，control divergence 未测）。
- 再由 explore 结论动态生成实现子任务。

## 7. 验证策略（贯穿两 scrum）

- **ground truth = legacy CPU 渲染**（raw-XYZ 优先于 sRGB）。oracle 另可用 CpuTraceBackend 独立重实现（#253 范式）抓 kernel 数值/时序错。
- parity 全程统计级（累加图像 / raw-XYZ block-mean corr），非逐光线 identity（mt19937 流不可对齐，PCG 同分布）。
- perf 基线 = legacy CPU（GUI 实走路径），勿用 `LUMICE_TRACE_BACKEND=cpu_backend`（慢 2.5× 辅助产物）。
