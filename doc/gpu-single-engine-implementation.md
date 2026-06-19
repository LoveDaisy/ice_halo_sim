# GPU 单引擎实现设计（seam-design §5 落地）

> 本文是 §5 单引擎 GPU simulator **重写**的实现设计与上下文锚，供后续 scrum 引用。
> 配套（读这些获取完整推理）：
> - `doc/seam-design.md` —— §5 目标架构蓝图（§3.6 原始之罪 / §4.5 filter 出口 / §5 统一 seam 形态）。
> - `doc/gpu-route-history.md` —— GPU 迁移 #250→268 全程回顾（§9 = 单引擎弧高潮）。
> - `doc/trace-backend-frame-lifecycle.md` —— Metal 帧生命周期 as-built（§8 = DR-3 波长）。
> - `doc/testing-architecture.md §4.2` —— parity metric-masks-bugs battery 方法论（cross-seed + 能量守恒 + golden 锚 + 人眼）。
> - `scratchpad/explore-gpu-single-engine/{SUMMARY,insights,experiments}.md` —— explore-266 de-risk 细节（gitignored，本文固化其 durable 结论）。

## 0. ⭐ As-Built 状态（scrum-267 + scrum-268 完成，2026-06-17）

> 本节为接手须知。两个 scrum 均已合 main；下方 §1-§7 保留设计推理原文。

> ⚠️ **吞吐数字 2026-06-19 受控重测后纠偏**（task-fix-throughput-bench-honesty）。
> 原表的 "CLI 9.5× / GUI 2.07× / 6× poller headroom" 三处均为测量假象，已替换为下方
> 实测 regime 表。纠偏详情与度量教训见本节末"度量纠偏"小节。

| 里程碑 | 数值 |
|--------|------|
| **CLI 引擎吞吐**（重场景，`--benchmark` setup-excluded，dispatch 32768，M2 Max，2026-06-19 复测） | `ms_multi_crystal_complex_filter` **8.1× legacy**；`ms_multi_crystal_filtered_bd` **10.1× legacy** |
| **GUI steady 吞吐**（真实 GUI regime，infinite + reconstruct 路径，dual_fisheye 512×256，M2 Max） | 重场景 **~9.5× legacy**（轻 1.8× / 中 2.2× / 最重棱锥 5.7×） |
| **GUI first_upload**（median，regime 相关） | 重场景 ~18ms / 中场景 ~71ms（均 < 150ms 冻结阈） |
| **引擎 vs GUI** | **同口径下 ≈ 1:1（无 headroom gap）**——引擎 8–10× ≈ GUI ~9.5×；旧"6× poller headroom"是假象 |
| **dispatch 甜点** | **32768**（backend-aware 默认；`LUMICE_DISPATCH_RAY_NUM`）。512/2048 仍饿死 GPU（0.2–0.8×），128 在大 ray_num 直接挂死 |
| **commit↔batch 解耦** | ✅ `LUMICE_DISPATCH_RAY_NUM` + `LUMICE_COMMIT_RAY_NUM` 双旋钮（concern #2 已解） |
| **parity matrix** | **10/10**（含 D65 illuminant + DR-3 波长） |
| **occupancy**（trace_layer_kernel PSO） | **640**（benign：R1 不触发，见 §6.1 最终裁决） |
| **PR** | #127（scrum-267）+ #129（scrum-268），均已合 main（2026-06-17） |

**度量纠偏（task-fix-throughput-bench-honesty，2026-06-19）**——原 §0 三个数字为何错、现值为何可信：
- **"CLI 9.5× legacy"**：旧 `--benchmark` 把一次性 setup（server alloc + scene gen + 首 dispatch 延迟）与 100ms 轮询量化计入吞吐分母，对 0.2s 量级的快后端系统性低估（实测被压到 4.95×）。修复（计时起点改首次 `sim_ray_num>0` + 5ms 轮询）后重测 = 8.1×/10.1×。旧"9.5×"作为 ratio 巧合接近真值，但出处口径不可信。
- **"GUI 2.07× legacy"**：是 **task-272 修 complex-filter 导入前**测的——GUI 静默丢 filter → 跑无 culling 的重 workload → 被压低。修复后真实重场景 = ~9.5×。
- **"6× poller headroom（引擎 9.5× 在 GUI 仅兑现 2.07×，差 poller 20ms 整幅回读）"**：不存在。同口径下引擎 ≈ GUI（甚至 GUI steady 绝对值更高，因 `--benchmark` 旧口径低估引擎）。poller 整幅回读是**每次 commit 的延迟成本**（体现在 first_upload，全部 < 150ms），不是吞吐天花板（explore-271 E3 已证 poll 间隔不影响吞吐）。
- **错误 config 名**：原表 `ms3_multi_crystal_complex_filter` 不存在；真名为 `ms_multi_crystal_complex_filter`（无 `3`）。
- **度量教训**：吞吐对比必须 ① 排除 setup（rate = 稳态 active 窗口）；② 对齐 workload（filter culling 与否）与投影（GUI 强制 dual_fisheye_equal_area，见 `file_io.cpp` `SerializeCoreConfig`）；③ 只比同方法学内的 ratio-over-legacy，跨方法学相除无意义。原始数据：`scratchpad/task-fix-throughput-bench-honesty/data/`。

**已删除的遗留结构**（§5.1 reuse/discard 账本承诺的删法）：
- `CopyContSliceToRootBuf`（host-side 续传）→ 删，由 `transit_root_kernel` 取代。
- 12-worker queue-per-Simulator 编排 → 删，`server.cpp` 重构为单引擎。
- `LUMICE_BATCH_RAY_NUM` 双重身份 → 拆为 `LUMICE_DISPATCH_RAY_NUM` + `LUMICE_COMMIT_RAY_NUM`（BATCH 保留为 COMMIT 的 backward-compat fallback，已废弃）。
- ms_mode==1 "半续传"分支（写占位 out_p=centroid） → 重写为 device emit-gate。

**额外修复（correct-by-construction）**：
- +16% 多 MS filter 能量 bug（根因=host `CopyContSliceToRootBuf` 解耦 filter，随删除自愈）。
- server 构造函数潜伏 bug（legacy CPU 巨型未切块 consume 假象，268.6 白盒证伪后修）。

## 1. 状态与目标（设计期原文）

> **接手注意**：本节描述 scrum-267/268 之前的出发状态（设计期），已是历史。
> as-built 见 §0。

- **当时**：Metal 是 GUI **opt-in**（`use_metal_backend` 复选框，默认关）；走 **12-worker + host-side MS 续传**（`metal_trace_backend.mm` kernel 写 continuation，host `CopyContSliceToRootBuf` 逐光线做 filter+prob+frame-transit）。这是 258-265 把已 de-risk 零件**焊进 legacy 结构**的产物，非 §5。
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

### 6.1 Scrum 1 结转的 Scrum 2 输入（267.2 fused-emit-gate 产出，2026-06-14）

- **⭐ R1 option B 最终裁决（scrum-268.6 实测，2026-06-17）**：267.2 的 emit gate 把 `trace_layer_kernel` PSO 的 `maxTotalThreadsPerThreadgroup` 从 1024 压到 704（再到 640，DR-3 波长后）。**Scrum 2 实测结果：R1 不触发（benign）。** 理由：重场景（`ms_multi_crystal_complex_filter` / `ms_multi_crystal_filtered_bd`）Metal 单引擎实测 CLI 吞吐 **8–10× legacy**（2026-06-19 复测，见 §0），GUI steady **~9.5× legacy**——occupancy 640 对吞吐无恶化，无需触发 option B（拆 filter-gate 为独立 wavefront dispatch）。occupancy 回归门固定在 640（当前实测基准），由 `TraceLayerKernelMaxThreadsForTest()` 守卫。**R1 option B 归 backlog（CUDA phase-2 开篇时重估，离散显存场景可能更敏感）。**
- **gate cleanup（Scrum 2 顺带）**：①`DeviceFilterCheck` 第 7 形参在 `kFilterMatchHelperSrc` 定义层仍名 `crystal_id`，正确实参是 `gate_slot`（= `ms_layer_idx*max_ci+crystal_id`）——建议改名 `orbit_slot` 使 API 自描述（当前靠调用点注释保护，不可扩展）；②`gate_seed` 在 `(ms_layer_idx=0,crystal_id=0)` 退化为 `gen_seed_`、与 gen_root PCG 种子重叠（corner case，M5 parity 未受影响）——加非零 XOR 偏置消除。

## 6.2 反漂移纪律（D1-D4，贯穿 §5 弧的实施纪律）

> 背景：seam-design 蓝图很早就讨论清楚，但 #250→265 多次翻车不是方向问题，而是**实施时贪图最小努力、被现有代码同化、走小步偏移**（§3.6 原始之罪的复发）。Scrum 1 没漂移，正因为有 **C1 硬规则**（删非改 / 干净引擎 / 唯一验收门）。把同款纪律一般化、前置固化为后续每个 scrum 的实施约束。依据 a04（系统完整性先于局部 / 举证责任在增加方 / 减法优先 / 过程信号是架构警报）+ a05（软约束必失效→固化为门禁）。

- **D1 验收门 pre-register**：每个 scrum 的数值验收门在 implementation 之前钉死进 `scrum.md`，门的测试 harness 作为头几个子任务先建。**禁止事后放宽门来迁就实现**——这正是 scrum-267 续传欠采样 bug 被"放宽 corr 阈值"掩盖的复发模式（[[feedback_gpu_parity_corr_masks_undersampling]]）。
- **D2 删非包 + 前置 reuse/discard 账本**：design explore 收敛时产出"复用/删除账本"（仿 §5.1），**点名要删除的 legacy 结构**，禁止"在旧结构外面再包一层适配器"（包不是删）。
- **D3 plan-review / code-review 必答三问（不答即 block）**：每个实现子任务的 plan 和 diff 显式回答——①推进了哪条 §5 不变量？②**删除**了什么（不只是加了什么）？③是否依赖 legacy 结构（12-worker / host-hop）？若是——是带 delete-ticket 的临时脚手架，还是漂移？挂在 scrum-drive 已强制的 review 门上。
- **D4 过程信号当架构警报（tripwire）**：**若某实现子任务的 diff 在 legacy 结构之上净增、且没删任何东西 → 停。** 净加 = 又在镜像 CPU 结构。这是停下来回看大目标的硬信号，不是继续走小步的理由。

> 各 scrum 的具体验收门数值是 scrum-specific，落在该 scrum 的 `scratchpad/scrum-*/scrum.md`，不进本 doc。

## 7. 验证策略（贯穿两 scrum）

- **ground truth = legacy CPU 渲染**（raw-XYZ 优先于 sRGB）。oracle 另可用 CpuTraceBackend 独立重实现（#253 范式）抓 kernel 数值/时序错。
- parity 全程统计级（累加图像 / raw-XYZ block-mean corr），非逐光线 identity（mt19937 流不可对齐，PCG 同分布）。
- perf 基线 = legacy CPU（GUI 实走路径），勿用 `LUMICE_TRACE_BACKEND=cpu_backend`（慢 2.5× 辅助产物）。
- **⚠️ parity metric-masks-bugs battery**：corr 单指标两次放过真 bug（267.3 欠采样 + 268.8 平坦谱静默）。所有 parity gate 必须组合：**cross-seed 自洽 + 能量守恒 + golden 绝对锚 + 人眼核查 + revert 反验**。详见 `doc/testing-architecture.md §4.2`（权威规范，不在此处重复）。

## 8. DR-3：per-ray 波长决策链（scrum-268.3/268.8）

> 波长设计经历了三轮决策（DR-1→DR-2→DR-3），最终落点与原计划不同。

**DR-1（原计划）**：per-threadgroup 波长——所有位于同一 threadgroup 的光线共享同一折射波长（MSL uniform + tg 内协作读）。

**DR-1 被推翻（owner 白盒 + runner 自阻塞双向发现）**：
- 多 MS 续传原子压缩光线数——光子的 `tg_id` 在层间变化（tg 内存活数不固定，续传打包重排），per-tg 波长使光子中途换折射率 → 路径计算错误，非方差问题。
- runner 自阻塞：尝试把 per-batch `wl` 穿过 exit seam 发现 per-batch wl 无法跟着续传存活到下一层。
- 两端独立发现合流，owner 拍板 DR-3。

**DR-2(B)（过渡）**：host 预采波长池 + 上传，device 仅读 index（零 device Sellmeier，满足 §3.7 合规）。

**DR-3（最终，as-built）**：波长是**每个光子的终生属性**（`wl_idx`，从 root-gen 到最终 consumer 贯穿全路径）：
- `gen_root_kernel` 按 `global_idx % M` 分配 `wl_idx`（均匀覆盖 WlPool）。
- trace kernel 按 `wl_idx` 从 WlPool 读折射率（寄存器驻留，零全局访问额外税）。
- emit gate 把 `wl_idx` 写进 `cont_wl_idx_out`。
- `transit_root_kernel` pass-through `cont_wl_idx_in → root_wl_idx_out`（见 `doc/trace-backend-frame-lifecycle.md §4.3/§8`）。
- consumer 用 per-ray CMF 权重积分 XYZ。

**静默失效抓捕**（硬门案例，see §2.4 正确性）：Step 9 gate 误放在 `use_backend` 分支，使 cpu_backend 路径 `curr_wl_` 保持 0（无池），产全黑图像——触发反静默回退硬门（`!per_ray_wl && !outgoing_d_.empty() && curr_wl_ < 1.0f → assert`）当场抓住。没有硬门，bug 静默产平坦谱（plausible，不崩）继续藏。

## 9. 单引擎 as-built：关键发现与弯路

> 接手：这些是 scrum-268 最值得学的教训，避免重做。

1. **"consumer-bound"误判**（268.6）：初测 GUI steady 81K vs legacy 729K → 误判 consumer 是瓶颈。实为：server bug（legacy 巨型未切块 consume）+ GUI 预算稀释。白盒证：benchmark consumer ~1.3µs/batch，健康 ~1.0µs/batch——GPU **非** consumer-bound，差距来源=poller 每 20ms 整幅图回读 + GPU 上传（非 consumer）。

2. **G2 子进程隔离 BLOCKER**（268.2）：in-process static cache 使 batch 变化无效，batch 不变性测试结论全错——改子进程隔离后才得到正确的"batch 有小幅真实依赖（cross corr 0.987-0.997 vs self-noise 0.9998，几何采样粒度效应）"。

3. **独立验证抓 runner 漏报回归**（268.7）：runner 未验证对 CLI legacy 单引擎的影响，owner 亲手发现 legacy 慢 6×（1-worker 编排=设计预期代价，非 regression）。

4. **~~吞吐天花板：poller 20ms 整幅回读~~（2026-06-19 推翻，见 §0 度量纠偏）**：原结论"引擎 9.5× 在 GUI 仅兑现 2.07×、差 poller"是测量假象——同口径下引擎（8–10×）≈ GUI（~9.5×），无 headroom gap。poller 整幅回读是 **per-commit 延迟成本**（first_upload < 150ms），不是吞吐天花板（explore-271 E3 实证 poll 间隔不影响吞吐）。partial-readback / async upload 若做，目标是降**交互延迟**（first_upload），非提吞吐。
