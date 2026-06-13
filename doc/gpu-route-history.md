# GPU 路线系统回顾（#250 → #265，截至 2026-06-13）

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
