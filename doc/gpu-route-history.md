# GPU 路线系统回顾（#250 → #312，截至 2026-07-01）

> §一~§四 = Metal 单引擎弧（#250→268，原始回顾，截至 2026-06-13）；§五 Phase 10 = CUDA 第二步 + device-fused（#294→302）；§六 Phase 11 = CUDA 吞吐收口 + Windows 交付 + 第三时钟（#303→312）。

> 目的：把 GPU 迁移一路的决策演进、已积累数据、遗留项盘点清楚，作为重新深想的全局参照系。
> 触发：owner 反思"行动前想得不够深→地基不稳、结论来回摇摆、积累不成形"。

---

## 一、大方向决策演进（每步的决策 + 它修正了什么）

**Phase 0 — 为什么要 GPU（赌注前提）**
- beam tracing 复盘（03-29）：方差瓶颈=SO(3) 姿态采样 O(1/√N)，算法层无结构改进空间 → 提质唯一路径=加算力。
- #246 cpu-soa-refactor：量化内存墙（#248）——RaySeg 膨胀致 -29% 退化，SoA option D 落地（168→96B）。坐实"高并发 CPU 内存受限"。

**Phase 1 — GPU 赌注 de-risk（#250）**
- #250 spike：**GO**。M2 Max ~250× 全 sim、**计算受限**（28<<400 GB/s）、寄存器驻留绕开内存墙、**divergence 证伪**（无需 stream compaction）、原子累加仅 +7%。
- **06-04 关键 reframe**：把"backend 选型死结"框架松开——dev49 是 NVIDIA 不是 AMD。真实矩阵：M2(开发)=Metal、**dev49(bench)+Windows(用户)=CUDA**。覆盖三台只需 Metal+CUDA+CPU。**定两步走：Step1 Metal/M2，Step2 CUDA/dev49。**

**Phase 2 — 架构：缝（#251）**
- 决策：**薄 `TraceBackend` 缝 + N 份原生 kernel**（拒可移植 kernel 语言——会藏住寄存器驻留这个 250× 命根、又省不掉每平台 SDK/CI）。
- 设计铁律：缝粗到整条融合管线 / host 指针不穿缝（device-resident 不透明 handle，**从第 1 天按离散内存语义**）/ 层间重组=backend-local / C API 不变。
- de-risk：recorder 寄存器压力 ≤16%（唯一真风险，证伪）。

**Phase 3 — 正确性弯路（#252 取消 → #253）**
- #252 集成路线取消——根因=缝帧生命周期沉默（Metal 漏出射 `crystal_rot_.Apply` → 环塌成带）。**帧是错的。**
- #253 frame-correctness：修单/多 MS 帧、重做 parity harness（CpuTraceBackend 独立 oracle，弃 kernel-mirror 共享盲区）、1.35× 亮度裁为伪象、owner 人眼把关。corr 0.99/0.997。**任何 perf 工作前必须先有可信正确性。**

**Phase 4 — Metal 进 GUI（#254-255）**
- #254 Metal kernel + C API 开关 + GUI 复选框 + parity 0.9963。#255 日志统一。

**Phase 5 — GUI perf 转向：缝重设计（#256-259）**
- #256 metal-gui-perf：**关键发现**——GUI 慢根因=backend-seam 整幅回读税（每 batch O(W×H)），**非 Metal/GPU**。2048×1024 下 Metal 比 legacy 慢 9-57×；合批反超。当时判"GUI 是 GPU 的错误战场"。
- #257 seam 重设计：缝切在**出射光线**（buffer-egress 非 image），带宽省 266-1362×。路线图 P1 出口侧 / P2 入口侧 root-gen / P3 bulk device-fused。
- #258 exit-seam scrum：P1 转正为规范出口替代 image-seam。filter parity 真根因=BeginSession 每 SimBatch 重置 RNG（白盒命中）。
- #259：吞吐未回退。

**Phase 6 — 入口侧 GPU root-gen（#260-261）**
- #260 rootgen scrum：device root-gen 转正默认供给，单 worker 1.87×/多 ~2×，parity ≥0.99。
- #261：bench 确证 **b128 单晶体净亏 / 多晶体净赚**。← 第一个诚实的"非一致赢"信号。

**Phase 7 — Profile & 融合（#263-264）**
- #263 integrated-profile：摩擦=**GPU 命令缓冲往返延迟**（wait 89%）非计算。融合（跳 gen wait）2.41×。async 不值。多 worker=掩盖延迟非并行（单 GPU）。**gen regime 2D（大 batch device-gen 18.5M=2× 天花板）。concern #2 在此结晶。**
- #264 fusion：融合默认化。单晶体 b128 **2.40×（分母=融合前 Metal，非 legacy）**。

**Phase 8 — GUI 可行性现实核对（#265，本会话）**
- 裁决(A)：Metal 能胜 legacy 但**无全局单 batch**——轻场景要大/重场景要小（concern #2 跨 regime 坐实）。默认 b128 饿死 GPU。Metal 对重场景拖动痛点有效（4.5× 帧）。

**Phase 9 — §5 单引擎重写弧高潮（#266-268，2026-06-14/17）**
- **explore-266 de-risk**（2026-06-14）：三关键结论固化——①divergence=伪命题（层内线性链 megakernel，层间 wavefront）；②device 续传 filter 可行（E4：`ReduceBuffer` MSL，1.44M 0 mismatch，seam-design §4.5 悲观误判推翻）；③吞吐须双 lever（大 dispatch + device-resident 续传，正交）。定位 current Metal 多 MS+filter 对 legacy **+16% 能量 bug**（根因=host `CopyContSliceToRootBuf` 解耦 filter）。
- **scrum-267（Scrum 1，device 续传引擎）**：删 `CopyContSliceToRootBuf`，新建 `transit_root_kernel` device 路径；emit gate 内联 device filter-match（DeviceFilterCheck）；raw-XYZ parity 8/8 GREEN（+16% 自愈 correct-by-construction）；occupancy 704→640（DR-3 后，benign）。PR #127，合 main 2026-06-14。
- **scrum-268（Scrum 2，单引擎编排）**：核心结果（吞吐数字 2026-06-19 受控重测纠偏，见下）：
  - **CLI 引擎吞吐**（`--benchmark` setup-excluded，dispatch 32768，M2 Max，2026-06-19 复测）：`ms_multi_crystal_complex_filter` **8.1× legacy** / `ms_multi_crystal_filtered_bd` **10.1× legacy**。
  - **GUI steady 吞吐**（infinite + reconstruct，dual_fisheye）：重场景 **~9.5× legacy**（轻 1.8× / 中 2.2× / 最重 5.7×）。
  - **DR-3 波长 per-ray**：推翻 DR-1 per-tg（multi-MS 续传原子压缩使 tg_id 跨层失稳）→ host 预采 WlPool，光子终生携带 `wl_idx`（见 `doc/trace-backend-frame-lifecycle.md §8`）。
  - **concern #2 已解**：`LUMICE_BATCH_RAY_NUM` 拆为 `LUMICE_DISPATCH_RAY_NUM`（GPU dispatch 粒度）+ `LUMICE_COMMIT_RAY_NUM`（GUI commit 粒度）双旋钮（268.4）。
  - **+16% bug correct-by-construction 修复**：随 `CopyContSliceToRootBuf` 删除自愈，parity matrix 10/10。
  - PR #129，合 main 2026-06-17。
- **吞吐数字纠偏（task-fix-throughput-bench-honesty，2026-06-19）**：scrum-268 收尾时记的 "CLI 9.5× / GUI 2.07× / 6× poller headroom" 三处均为测量假象——① `--benchmark` 把 setup 计入分母低估快后端（修复后引擎 8–10×）；② GUI 2.07× 是 task-272 修 complex-filter 导入前的无 culling 数（修复后 ~9.5×）；③ 同口径下引擎 ≈ GUI，**无 poller headroom gap**（poller 整幅回读是 per-commit 延迟成本，非吞吐天花板，explore-271 E3 已证）。原始数据见 `scratchpad/task-fix-throughput-bench-honesty/data/`。

### 演进的元模式
1. **战略意图始终是"GPU 求吞吐、两步走"**，但 #256 起逐步滑向"在开发机 Mac GUI 上把 Metal 调好"。
2. **结论反复自我修正**：scrum-260"device-gen 恒默认"→#261"b128 净亏"→#263"regime 2D"→#264"2.40×(vs Metal)"→#265"Metal 输 GUI→b128 饿死→重场景 concern #2"。每步纠正前一步的过度外推，**基线/regime 不断漂移**。

---

## 二、已积累的数据（硬资产）与已探索方向

### 硬数据资产（可信、可复用）
| 数据 | 值 | 来源 | caveat |
|---|---|---|---|
| GPU trace 算力上限 | ~250× 全 sim，计算受限，register-resident | #250 | M2 统一内存 |
| divergence | 证伪（固定 max_hits，无变长路径） | #250 | — |
| 原子累加竞争 | +7%，竞争无关 | #250 | — |
| recorder 寄存器压力 | ≤16%（唯一真风险，证伪） | #251.1 | — |
| GPU-RNG gen | 171×，端到端恢复 32× | #251.5 | **统一内存，不可外推 CUDA** |
| 上传 | 非瓶颈（0.54× trace）；host-gen 才是（40.7×） | #251.5 | **统一内存 caveat** |
| 帧正确性 | 单/多 MS corr 0.99/0.997 vs CPU；1.35×=伪象 | #253 | — |
| exit-seam 带宽 | 省 266-1362× | #256-258 | — |
| device root-gen | 单 1.87×/多 ~2×，parity ≥0.99 | #260 | — |
| 摩擦真身 | GPU 命令缓冲往返**延迟**（wait 89%），非计算 | #263 | — |
| 多 worker | 掩盖延迟非并行（单 GPU，W6→8 持平） | #263 | — |
| gen regime 2D | 大 batch device-gen 18.5M=2× 天花板 | #263 | — |
| 融合 | 2.41×（vs 融合前 Metal） | #263-264 | 分母非 legacy |
| GUI 可行性 | 轻场景大 batch 2-5× 胜；重场景 concern #2 咬人 | #265 | — |
| device filter-match | 全类型（含 Raypath/Complex）1.44M 0 mismatch；seam §4.5 悲观判推翻 | #266 E4 | M2 统一内存 |
| CLI 引擎吞吐 | `--benchmark` setup-excluded + dispatch 32768 → **8.1× / 10.1× legacy**（complex_filter / filtered_bd） | #267-268, #273 复测 | M2 Max |
| GUI steady 吞吐 | infinite + reconstruct → 重场景 **~9.5× legacy**（轻 1.8× / 中 2.2× / 最重 5.7×） | #273 复测 | M2 Max |
| dispatch 甜点 | **32768**（backend-aware 默认）；512/2048 饿死 GPU（0.2–0.8×），128 大 ray_num 挂死 | #268.6, #273 | M2 Max |
| 引擎 vs GUI | 同口径 ≈ 1:1（**无 poller headroom gap**）；旧"6× headroom/2.07×/9.5×"为测量假象 | #273 | M2 Max |

### 已探索方向 & 裁决
- 可移植 kernel 语言 → **拒**（藏寄存器驻留杠杆）。
- async/双缓冲藏延迟 → **拒**（融合已 captures 大头，async 上界 3.07× 不值）。#263
- 多 worker 求 GPU 并行 → **证伪**（单 GPU）。#263
- 自适应 gen 策略 → **推迟**（依赖 commit↔batch 解耦；GUI 走大 batch 则 moot）。#263

---

## 三、遗留项盘点（现在看：仍有价值 / 已过时 / 已被取代）

| backlog 条目 | 现状判断 |
|---|---|
| **CUDA Step 2（离散显存重测 + master plan 第二步）** [425/120] | ⭐**最高战略价值，未动**。真实部署目标（Windows 用户 + dev49 bench 都是 NVIDIA）。所有 Metal"便宜"结论带统一内存 caveat，CUDA 才是真正受检处。seam 全部设计就是为它零上层改动。 |
| **commit↔batch 解耦（concern #2）** [565] | ✅**仍有价值，且已被 #265 跨 regime 证明必要**。启动条件（#264 落地）**已满足**。是 GUI 吞吐+响应的根治解。 |
| **P3 bulk device-fused** [557] | ✅仍有价值，但**属 CUDA/offline 吞吐**（非 GUI），已标 phase-2 CUDA 开篇。 |
| **golden 光线 absolute 锚测试** [438] | ✅仍有价值，尤其"CUDA backend 立项时一并做"——第二后端正需 absolute 锚分辨"kernel 错"vs"两后端一致都错"。 |
| **自适应 gen 策略** [579] | ⚠️**部分过时**。#265 显示重场景要小 batch，可能仍相关；但大半被 decouple 工作吸收，待 decouple 后重估。 |
| **Metal 收尾：默认 batch 调优** [544] | ⚠️**已被 #265 细化/取代**——不是简单"调高默认"，是 concern #2 / regime-aware 问题。 |
| Metal 收尾：sim_seed 文档 / rpath / single_ms_filter [544] | ✅小清理仍有效。 |
| **Metal device-RNG root supply** [410] | ❌**已被 scrum-260 实现/取代**，应关闭。 |
| roofline 内存天花板 [368] / fuse trace→consume [388] | ◽CPU 侧高端 perf，与 GPU 路线正交；仍有效但低优先（多数用户 8C16T 不撞墙）。 |

---

## 四、供讨论的深层张力（observation，非 prescription）

1. **战略漂移：是否在打磨 GPU 的最差战场？** GPU 结构优势在**大批量离线吞吐**（250× 计算受限、register-resident、b2048 18.5M）。而最近所有挣扎（concern #2、b128 饿死、多 worker 非并行、延迟受限）都是 GPU 在 GUI **交互 regime**里打它的**最差仗**。真正的奖品（CUDA 大批量、服务真实用户）一直没动。

2. **regime 结构早就在框架里。** backlog 120 master plan + #263 regime 2D 早把"大 batch 赢/小 batch 角落"画出来了。最近 b128输→调大赢→重场景又输的来回，本质是同一 regime 结构被反复**局部重新发现**——因为没回到全局参照系。正是 owner 说的"想得不够深"：已有全局框架没被复用。

3. **CUDA 是房间里的大象。** Metal phase-1 越打磨 GUI 细节，离 seam 真正受检点（离散显存、CUDA）越远。所有 unified-memory caveat 的结论都欠一次 CUDA 兑现。

4. **基线漂移=摇摆的机制根源。** 每个"×几"在不同分母下成立（融合前Metal/legacy CPU/CpuTraceBackend/host-gen），跨节点不一致→似是而非。已固化 `feedback_perf_baseline_is_legacy_cpu`，但历史结论需用统一基线重读。

---

## 五、Phase 10 — CUDA 第二步 + device-fused 消费(#294 → #302，2026-06-24 → 06-29)

> 接 §一 Phase 9 之后。这一段把 GPU 路线推进到「第二个真实消费者（离散显存 CUDA）」并把消费端（投影+filter+累加）从 host 搬上 device。**缝契约经 CUDA 兑现、零上层改动未返工——蓝图赌对了。**

**explore-294 → scrum-295（CUDA MVP）→ scrum-296（全量多 CI）**：CUDA backend 接入同一 `TraceBackend` 缝。单 MS 验缝 → 全量多 CI/多 MS/filter 对齐 legacy（parity 全绿）。波折：CUDA 自创 Möller-Trumbore 遍历复现了 task-275~278 已解决的绝对-ε 漏面（energy 0.735）→ 教训「几何遍历也要纳入共享核单源，别重新发明」。

**task-cuda-throughput-bench（#299，诚实吞吐基线）**：干净机实测 CUDA ≈ **0.10-0.12× legacy，flat with dispatch**；瓶颈 100% = **出射记录 per-exit PCIe 主机往返**（DrainExits D2H + host filter + host 累加，~54ns/exit）。**推翻 scrum-296 Step D「大 dispatch 1.6-2.2×」**（那是 64MiB exit 顶丢 87% exit 的假象）。结论：device 侧累加是唯一有意义的方向。

**第一性原理收敛（2026-06-28 owner 讨论）**：离散显存上唯一稀缺资源 = PCIe。数据按「是否必须跨 PCIe」分层（输入一次/在途永不/输出小图/富元数据默认永不）→ **纯融合 device 流水线**：emit gate 当场 prob+filter+固定投影+补偿累加进 device XYZ，**渲染图产物不物化出射记录**；富元数据降级为光路回溯的可选 tap。把蓝图 §4.4「两个出口」在离散显存上塌成「一条流水线、两个回读节奏」。详见 `seam-design.md` §4.8（离散显存 reframe）。

**scrum-302（device-fused accumulation）**：
- **S1（Metal）**：消费端上 device（emit gate 融投影+filter+补偿累加），砍出射 buffer，抽 option-b 共享核 `accum_shared.h`。inline 修 3 真 bug（mid-exit 漏 rectangular 分支 / server has_renderable 漏 xyz_pixel_data_ 致零图 / **最终层漏 prob draw → energy=1/(1-prob)**）。验收：parity 17/17 + e2e 15/15。意外：device-fused Metal vs 旧 Metal **2.9-4.1×**（消 host O(N) 投影）。PR 未提（合并在分支）。
- **S2（CUDA/dev49）**：CUDA 接共享核 + device 累加。inline 亲跑 dev49 修 2 bug（mid-exit 漏 device 累加 → energy=1/层数 / 测试 config 用不支持的 fisheye 投影）→ **parity 10/10 绿**。runner 三犯（被 kill / Mac 编不了 CUDA / 收窄 scope 误报 DONE）→ 巩固纪律「CUDA AC 真闭只能 scrum owner 在 dev49 亲跑」。

**⚠️ 未解的核心张力（scrum-302 S2 暴露，owner 定性）**：device-fused 去掉 roundtrip 后，CUDA 吞吐 **0.16-0.40× legacy**（16 线程 Zen5），随 max_hits 收窄不反超。owner 定性:**对一块 4060 Ti 根本不合理 = 设计/实现缺陷**（非"GPU 非天然胜"的可接受现实）。诊断：compute-bound（max_hits 反比）+ 低 max_hits 仍只 1.82M/s 疑 per-CI 串行 dispatch/launch 开销（单引擎大 dispatch 理想在 CUDA 多 CI 未兑现，§3.6 原始之罪复发）+ recorder local-mem。**#250「250×」spike 与 production 间有未追的退化**——这是下一个 explore 的核心(profiler-first，见 backlog)。

---

## 六、Phase 11 — CUDA 吞吐收口 + Windows 交付 + 第三时钟(#303 → #312，2026-06-29 → 07-01)

> 接 Phase 10。这一段把上面「未解的核心张力」（CUDA 0.16-0.40× "根本不合理"）**彻底收口**——真因大半是**测量假象**，剩下是几个真 lever；随后把 CUDA 从"能算对"推到"Windows 可交付"，最后补上蓝图 §4.8 第三时钟。**Phase 10 的悲观基线全部作废，别再引用。**

**explore-303 → scrum-304（吞吐收口第一刀）**：explore-303 一度归因"零 cudaStream + 每层 sync readback → GPU 1-2% 利用"，但 scrum-304 profiling 修正：真因 = **per-batch buffer 拆建 churn**（CUDA Reset 每 batch cudaFree+cudaFreeHost，占 mh15 wall 83%）。**buffer-persist**（`Reset(keep_persistent_buffers)`，镜像 Metal Reset）一刀 → 可比轻·单MS CUDA **35–56M/s（>25M 竞品线，>Mac Metal 28-30M）**，parity 10/10。**"0.16-0.40×" 定性为测量假象**（ad-hoc 非 idle-gate + 拿重场景 `ms_multi_crystal` 当可比口径）。

**scrum-306（dispatch + exit-cap，逼近 intrinsic）**：**async 前提被 profiling 推翻**（nsys 证 cudaEventSynchronize 仅 0.3% host time，stream deferral 值 <1%，增量 2 放弃）。真杠杆 = **默认 dispatch 32768→262144 + exit-cap**（CUDA `HasDeviceXyzAccum` 恒 true → trace kernel 从不写 `d_exit_`，这块死缓冲按 n×(2·max_hits+4) 膨到 GB→崩，capping 解锁大 dispatch）→ out-of-box **~114M/s（= 134M intrinsic 的 85%）**。几何池/persist = parity-clean 结构改进但**吞吐中性**（不得当加速源）。旁证：306.6 Metal 无便宜 kernel 杠杆（同 CUDA→算法级 backlog）；306.7 legacy 能量随 dispatch 波动证伪为 MC 方差非 bug。血泪：shared dev49 CPU 争用使 host-bound 吞吐 33M↔114M 同 binary 剧烈抖动，只信 per-run interleaved。

**explore-307 → task-308（77h 漏光，MSVC 专属正确性 bug）**：`RandomSample` 在 `curr_p==0.0`（MSVC `std::uniform_real_distribution` 可返精确 0）时 no-match→out 未写→保留调用方默认 tri_id=0→选背光面当入射→全反射异色高权重漏光。两平台插桩铁证（MSVC 2e9 抽样 115 次命中 / Mac 1e8 抽样 0 次）。与 GPU 吞吐正交，但同期落地。

**scrum-309 + scrum-310（CUDA Windows 交付）**：1070Ti/sm_61 parity 10/10 + 吞吐数据点 + GUI 跨平台"Use GPU"勾选（owner 眼验）；CI 门禁（windows-2022 pin，windows-2025=VS2026 拒 CUDA）+ 多 arch fatbin（PTX 61 floor + 75/86/89 real）+ 运行时 capability<sm_61 探测+优雅降级 + release 打包 cudart dll + CLI `--backend cuda`。**CUDA 在 Windows 真正可交付。** scrum-311 清理（CUDA 死代码 + CI Node24 + exit-seam crystal 计数）。

**scrum-312（第三时钟 readback 解耦，蓝图 §4.8 兑现）**：内测反馈"默认场景 GPU 慢"→真因 = 真实 GUI 分辨率 2048×1024 下 readback 焊死在 trace 时钟（每 SimBatch 一次 device XYZ 回读）。route B 把 readback 解耦到显示节奏第三时钟 → 2048×1024 增益：4060Ti(Ada) 28→39M(1.4×) / 1070Ti(Pascal) 12.5→33.5M(2.7×) / Metal 11→32.3M(~3×)，三机一致。**推翻"Metal 统一内存零收益"**（per-batch 全幅 24MB memset+memcpy 是真成本）。精度用 periodic-drain（float32+host Neumaier+稀 drain），**f64 淘汰**（48MB 溢 L2，0.6×）。parity CUDA 10/10×2arch + Metal 14/14。

**Phase 11 元模式**：Phase 10 的"根本不合理"张力，收口后大半是**测量方法学问题**（非 idle-gate / 错口径 / setup 计进分母 / drain 粒度失真），真机制 lever 只有三个（buffer-persist / dispatch / exit-cap / 第三时钟）。**教训固化**：GPU 吞吐 correctness 不可单一复现路径自证；shared 机只信 interleaved；profile 先于改代码（async 增量差点白建）。当前 canonical 吞吐见 `doc/performance-testing.md`；per-run 详录见 `scratchpad/perf-results-log.md`。**残余**：机制 C（in-kernel atomicAdd L2 溢出，高分辨率残税）+ kernel 算法级重构（SOL 逼近）归 backlog 远期。
