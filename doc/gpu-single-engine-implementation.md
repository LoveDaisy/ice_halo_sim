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
- **device 续传 gate 融进 kernel 逐跳 emit**（非"把 host hop 搬上 device"）：每 emit 一条离开光线（此刻 `path[]`=路径至今在寄存器）当场 device prob+filter → 续传写 continuation buffer / 输出写 exit buffer / fail 丢。三重收益：①逐位对齐 legacy `CollectData`（它就是逐跳对每条离开光线用"路径至今"判 filter，simulator.cpp:426）；②**天然修掉 §2.4 的 +16% bug**（其根因正是解耦的 host 重建；inline 用寄存器路径至今，单 MS exit 已证此 path 对 → correct-by-construction）；③E4 证融入零额外代价。
- **frame-transit 落点（C3，2026-06-14 精化）**：emit gate 只融 **prob + filter**——它们依赖"路径至今"，必须在离开那跳、寄存器里做。**frame-transit（重采下一层晶体取向 `InitRay_rot` + `ApplyInverse` + 重采入射点/面 `InitRay_p_fid`）不依赖路径、依赖\*下一层晶体几何\*，放到\*下一层 dispatch 的 kernel 入口\***（该 dispatch 已绑定自己那块晶体）。**为什么不塞进 emit gate**：多晶体场景（Scrum 1 验收 config 即 7 晶体）下，若 frame-transit 在 emit 那跳做，gate 就需要下一层全部晶体几何池 + per-ray 选择 = 把 concern #5 几何池提前拽进 Scrum 1，与"几何池留 Scrum 2"自相矛盾。切在 ingest 则：emit gate 保持 path-local、几何池干净留 Scrum 2、per-ray 下一层晶体路由变成层间 compaction 的一步（wavefront Recombine 本就该建模）。**此边界须在 Scrum 1 设计期钉死，否则实现中途会撞"多晶体 gate 做不了 device frame-transit"。**
- **几何单源**：晶体构造留 CPU（`MakeCrystal`），device 侧零几何代码（复杂稀少留 CPU/每 session，简单海量上 GPU/逐线程）。
- **架构为 CUDA 承接**：按离散内存语义设计，不焊统一内存假设（M2 上 host hop 廉价的结论不可外推 PCIe）。

## 4. 实现弧（2 scrum + 后续）

| 阶段 | 性格 | 交付 |
|------|------|------|
| **Scrum 1：device 续传引擎**（本文 §5） | 实现导向（设计已由 266 完成） | **正确性 + device-resident 续传机制**。交付物=raw-XYZ parity 对 legacy 恢复（+16% 消除）+ device 续传 gate 可独立切换运行。**不是吞吐**（见 C2）。 |
| **Scrum 2：单引擎编排**（本文 §6） | explore 先行（仍有设计不确定性） | 吞吐胜负手（大 dispatch + 单引擎 async + 释放 CPU 核）+ concern #2 解 + 几何池（concern #5） |

> **C1 clean-engine 形态（2026-06-14 owner 拍板）**：Scrum 1 **不是在 `metal_trace_backend.mm` 上原地做减法**，而是**新建一条干净的 §5 续传引擎路径**（device emit-gate kernel + device-resident 续传核），旧 Metal 路径降为**可切换的参考实现**，对 legacy 的 raw-XYZ parity 达标后**整体删除**旧 host-hop 路径。判定"是重写非补丁"的硬纪律：续传从 §5 emit-gate 原则重建、`CopyContSliceToRootBuf` 是\*删\*不是\*改\*、唯一验收门=对 legacy 的 raw-XYZ parity。缝契约 `trace_backend.hpp` 保留不动（它就是 §5 的 host/device 契约，见复用账本）。
>
> **C2 Scrum 1 验收是 correctness，不是吞吐（2026-06-14）**：266 E2/E3 已证——Lever B（device 续传）单独留在 12-worker 里，吞吐方向不明甚至可能退化（12 worker 原靠 12 核并行 host hop 拿到 225K；hop 移上 device 后 12 worker 争抢一块 GPU、host 核空出但 GPU 成唯一瓶颈）。吞吐胜负要"双 lever 且都做满"（Lever A 大 dispatch 在 Scrum 2）。故 **Scrum 1 验收 = raw-XYZ parity；吞吐至多做"无灾难性回退" sanity check，绝不设为 perf 门**。
| Phase 3：CUDA（dev49/RTX4060） | —— | 整个 GPU 迁移第二步，backlog 单独跟踪，**不在本 Metal 弧内** |

依赖：device gate 局部于 kernel + 续传 buffer，与"1 个还是 12 个引擎"正交 → Scrum 1 可独立先行；Scrum 2 在其上把编排塌成单引擎。

## 5. Scrum 1 范围：device 续传引擎

**做**：
- **emit gate**（融进 trace kernel 逐跳 emit）= device prob（PCG）+ device filter-match（ReduceBuffer 移植）。只融这两件——它们依赖"路径至今"（C3）。
- **frame-transit 在下一层 dispatch 的 kernel 入口**（device-root-gen 取向采样 + 入射点/面重采）——不在 emit gate（C3）。
- continuation 全程 device-resident（host 仅 counter + 重 dispatch），新引擎不含 host hop 的逐光线处理；旧 `CopyContSliceToRootBuf` 路径可切换保留，parity 达标后删（C1）。
- 多 MS filter 正确性 correct-by-construction（gate 内联 = legacy 语义）。
- **raw-XYZ parity 验收**（唯一硬门，C2）：单/多 MS × 有/无 filter（含 raypath PBD/BD + complex），对 legacy 恢复统计等价；用 owner 构造的 `ms3-multi-crystal-filter.json` 范式场景。建议按 E7 隔离梯子分级：先单晶体多 MS+filter（证 device gate 修掉续传 filter bug 的机制）→ 再多晶体（验 frame-transit 在 ingest 的多晶体路由）。
- golden-ray 解析锚（backlog:438）：给真 kernel 续传路径装确定性 golden 光线 + 解析期望，补"多反弹 trace loop / 面求交 / 续传"的 absolute 锚缺口（本轮 parity bug 暴露此盲区）。

### 5.1 复用 / 舍弃账本（钉死参考边界，防边写边被旧结构同化）

> 现有"设计"分两层：**缝契约**（`trace_backend.hpp`）= §5 蓝图本身，保留；**Metal 实现**（`metal_trace_backend.mm`）= 干净核 + 罪壳混合，罪壳重写。

**保留并复用（已 de-risk，誊抄进新引擎，非"原地留着"）：**
- **缝契约 `trace_backend.hpp`** — 不动。invariant 1-6（粗粒度融合 / 续传 device-resident 不透明 handle / 只 4B counter 过缝 / 出射世界系 / 离散内存语义）逐条=§5 的 host/device 契约；`TraceLayer`/`Recombine` 逐层模型=explore-266 把 §5 修正为"层间 wavefront"后的正确形态。
- trace kernel **内层 optics**（`mm:157-382`：`GetReflectRatio`/折射反射 TIR/面求交/`path[]` 记录）= #250 寄存器驻留核，逐字搬。
- exit-seam 出射记录写出（`mm:276-294`，ms_mode==0 分支）= 规范出口。
- `gen_root_kernel`（PCG device root-gen，scrum-260，`mm:688`）→ 复用为 frame-transit 取向采样。
- `exit_seam.hpp` schema / `ReadbackExitRays` / parity harness / `CpuTraceBackend` oracle / `ReduceBuffer` MSL spike（E4 `scratchpad/explore-gpu-single-engine/spike/`）。
- 机械管线（PSO / `Ensure*` buffer / `UploadCrystal`）= backend 脚手架非罪，**共享复用，不重写**。

**重写 / 删除（原始之罪，device 续传缺口）：**
- **`CopyContSliceToRootBuf`（`mm:1571-1737`）= 删除**，不修改。其 (A)frame-transit 迁到下一层 ingest、(B)filter/prob 迁到 emit gate。+16% bug 随之 correct-by-construction 消失。
- **trace kernel ms_mode==1 分支（`mm:217-261`）= 重写**：现写"半续传"（世界系 dir + 占位 `out_p=centroid` + 给 host 的 cont metadata），改为离开那跳用寄存器 `path[]` 当场 device prob+filter。
- **cont metadata 并行 buffer**（`cont_crystal_id`/`face_seq`）= 仅为喂 host hop 而存在 → device gate 上线后删。

**推迟到 Scrum 2（本 scrum 不碰）：** 12-worker→单引擎（缝之上 server.cpp，缝已支持）；per-batch 单晶体→几何池 per-ray 索引（§6 / concern #5）；commit↔batch 解耦（concern #2）；exit 分支 inline image 投影残留（image-seam，归 P3/CUDA）。

**验收**：raw-XYZ parity 对 legacy 在 filtered + multi-MS 场景恢复（当前 +16% 消除）；统计等价（§3.6）。

## 6. Scrum 2 范围：单引擎编排（explore 先行）

- explore 设计：单大 dispatch + async 引擎替 12-worker（12-worker 本质是延迟隐藏 hack，explore-263 证 W8 饱和）；**commit↔batch 解耦**（concern #2，backlog:565/571——`LUMICE_BATCH_RAY_NUM` 当前一身二职=GPU dispatch 粒度 + SimData 到达粒度，须解耦使 GUI 用大 GPU batch + 细 commit 响应）；GPU 大 dispatch 内部如何增量 drain/commit 给 consumer；CPU 核预算。
- **几何池 / concern #5**（晶体几何参数随机变动 → batch 内多晶体几何、per-ray 索引）在此评估或延后（§6 待定项，257 exp#5 证 memory divergence +20%@K1024 modest，control divergence 未测）。
- 再由 explore 结论动态生成实现子任务。

## 7. 验证策略（贯穿两 scrum）

- **ground truth = legacy CPU 渲染**（raw-XYZ 优先于 sRGB）。oracle 另可用 CpuTraceBackend 独立重实现（#253 范式）抓 kernel 数值/时序错。
- parity 全程统计级（累加图像 / raw-XYZ block-mean corr），非逐光线 identity（mt19937 流不可对齐，PCG 同分布）。
- perf 基线 = legacy CPU（GUI 实走路径），勿用 `LUMICE_TRACE_BACKEND=cpu_backend`（慢 2.5× 辅助产物）。
