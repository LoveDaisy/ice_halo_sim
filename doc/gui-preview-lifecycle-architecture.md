# GUI 预览生命周期 — 后台 worker × 前台实时显示的时钟解耦设计

> 性质：**设计蓝图（blueprint / 设计推理记录）**，不是 as-built 规范。记录"这个机制第一性原理上应该怎么设计"、为什么、以及要守的不变量。落地进度见 scrum-gui-lifecycle-clock-decouple。
> 缘起事实：内测反馈 + owner 复现的"GPU 跑完但 GUI 仍显示 Simulating / 按钮仍是 Stop / 状态栏仍 simulating"卡死（Metal + CUDA 双平台复现，与 GPU 硬件无关）。
> 关联：`seam-design.md`（GPU 侧三时钟解耦，本文是它在 GUI 侧的对偶）、`accumulator-consumer-architecture.md`（§8 poll 契约）、`capi-lifecycle-architecture.md`（server 生命周期 / zero-output completion）。

---

## 1. 缘起：一个卡死 bug 暴露的架构问题

有限光线 run 在 GPU 路线跑完、后端实际已 IDLE，但 GUI 永远停在 `kSimulating`：Run/Stop 按钮显示 Stop、状态栏显示 "Simulating..."。这是一个**间歇性竞态**——取决于前台 poll 与主线程 sync 的交错时序。

直接根因（as-was 代码，供对照，不是本文重点）：
- poller worker 用**可靠信号** `server_state==IDLE && has_valid_data` 自暂停（一旦暂停就不再 poll）；
- 主线程却用**脆弱信号** `server_state==IDLE && stats_sim_ray_num>0` 来跃迁到 `kDone`；
- 而 `stats_sim_ray_num` 只在"快照 generation 前进"时才写进交接结构，且 `TrySyncData` 每次 swap 会把 staged 清成默认值（0）。于是"最后那次 IDLE poll 常常没有新 generation" ⇒ 交给主线程的 `stats_sim_ray_num==0` ⇒ `0>0` 为假 ⇒ 永不 `kDone`；而 poller 已自暂停 ⇒ **这个边沿永久丢失**。

**但真正要修的不是这三行判据，而是它背后的架构形态。** 本文撇开现有代码，从第一性原理重推。

---

## 2. 核心诊断：不是"两套状态机"错，而是生命周期被复制 + 边沿触发 + 撕裂读

直觉上会归咎于"有两套状态机"。更精确的诊断是——三件**本质不同**的关注点被缠在了一起：

| 关注点 | 本质 | 唯一合法 owner |
|--------|------|----------------|
| **线程控制**（poller 该跑还是睡） | OS 线程调度优化 | poller 自己 |
| **仿真生命周期**（在跑 / 跑完 / 重启中） | 后端的客观事实 | **只能是后端** |
| **显示负载策略**（这一帧上不上屏、防黑闪） | 前台观感策略 | 前台 |

"两套状态机"本身**不是错**——线程控制 FSM 与显示策略是合法的独立域。错的是**"仿真生命周期"这一件事被复制成了两份**：poller 独立判"完事并自暂停"，GUI 又独立判"kDone"。两个独立判据 + 采样时序 ⇒ 必然有一天不一致。

三个可命名的病理：

1. **生命周期被复制**：同一个"是否跑完"被两个地方各自从采样里重推。→ 必须收敛到单一 owner。
2. **边沿触发**：完成被当成"一次性事件"，poller 自暂停后这个边沿永久丢失。→ 应改为**电平触发**（完成是一个持续可观测的*条件*，不是一次事件）。
3. **撕裂读**：`server_state`（每 poll 更）、`stats`（仅新 generation 更）、swap 清零，三个不同节奏的字段被当成一个一致元组消费。→ 跨线程只交接**版本化不可变快照**。

---

## 3. 第一性原理

- **P1 单一真源 / 投影视图**：仿真生命周期的权威在后端。前台永远**不推断**完成，只**读取**完成。其余各方都是后端事实在某个采样时刻的投影/缓存。
- **P2 电平触发优于边沿触发**：前台每帧把自己的显示状态朝"后端当前真相"收敛（reconcile）。只要*未来任意一次*采样观测到终态，状态就会自愈。撕裂读、丢帧、时序抖动都不再致命——因为不存在"唯一的一次机会"。
- **P3 关注点隔离**：生命周期信号与显示负载信号走**不同通道、不同门控**。任何**显示 gate 永不得压制一次生命周期跃迁**（当前 bug 的本质就是显示导向的 gating 泄漏进了生命周期决策）。
- **P4 重启是一等事件，不是待推断的状态**：给每次 commit 一个单调 `epoch`，用它消解"这是重启瞬时 IDLE 还是真完成"的歧义，而不是靠 `has_valid_data`/`stats>0` 这类侧信号反推。
- **P5 廉价可读性**：生命周期 + 光线计数是 O(1) 的，其可读性**不得依赖昂贵快照的产生**。

---

## 4. 统一原语：epoch（提交世代号）

用户随时改配置 → commit → 后端重置，这是把所有 ad-hoc 启发式串起来的那把钥匙。

- `committed_epoch`：前台每次 commit 时 `++`，单调。
- 每个后端快照都携带它被产出时的 `epoch`。
- 前台永远知道"我当前期望的是哪个 epoch"。

于是全部启发式塌缩成 epoch 的推论：

| 老问题 | 老做法（补丁） | epoch 下的推论 |
|--------|----------------|----------------|
| 重启瞬时 IDLE 误判完成 | `has_valid_data` 侧信号 | `IDLE@epoch N` 而前台已 `commit N+1` ⇒ **世代不符，直接丢弃** |
| 完成判定 | `IDLE && stats>0` | `lifecycle==Completed && epoch==committed_epoch` |
| 拖滑杆防黑闪 | intensity_locked + 门槛 | 门槛/保留上一帧策略**按 epoch 键控**：`N+1` 出够光线前继续显示 `N` 的最后一帧 |
| 永远达不到门槛的 epoch（不可能 filter） | quality gate timeout | 仍按 epoch + 超时兜底，但**终帧无条件上屏**（见 §7） |

---

## 5. 通道设计：命令 / 观测分离（CQS）

不要让前后台互相 mutate 对方的状态。只有两条单向流：

- **命令流（GUI → 后端）**：`Commit(epoch++)`、`Stop`。是*意图*。
- **观测流（后端 → GUI）**：版本化的**不可变快照** `Snapshot{ epoch, lifecycle, sim_rays, seg_rays, payload? }`。

前台显示状态 = `reconcile(最后意图, 最后一个 epoch 匹配的观测)`，**在唯一一处**用纯函数算出，任何线程不得旁路写它。

**关键**：跨线程 handoff 必须是**整体原子、单调版本戳的一个值对象**（triple-buffer / seqlock / 原子指针交换），而不是一袋各自以不同节奏更新的字段。消费方永不读"半更新"的字段组合——这直接根除 §2 的撕裂读。

后端生命周期建议是显式枚举，而非裸 IDLE 让调用方拿侧信号消歧：

```
enum class SimLifecycle { Idle, Running, Completed };  // 均带 epoch
```

`Completed` = 有限 run 跑完并 drain 干净（含"全被 filter 拒 / 全黑但已收敛"这种 zero-output 完成，见 capi-lifecycle-architecture.md）。`Idle` = 尚未 run 或重置后未产数据。二者对 epoch 的语义天然区分开重启瞬态。

---

## 6. 时钟图：三个时钟，再劈一刀

owner 的三时钟直觉正确，但"生命周期"与"像素帧"成本差几个数量级，不该共用一个时钟，故劈成四个：

| 时钟 | 由谁驱动 | 职责 | 成本 |
|------|----------|------|------|
| **① 显示 / 收敛** | imgui vsync（固有 ~60fps） | 每帧拉最新快照、reconcile UI 状态 | 廉价 |
| **② 快照物化**（poll，限流） | 帧物化预算 | 何时**花代价把后端最新 batch 状态物化成一张可显示帧** | 昂贵（O(W×H) 拷贝/转换） |
| **③ batch / 生产** | 后端（CUDA 超大 batch 时很粗） | 产出原始累加数据 | — |
| **④ 生命周期 + 计数心跳**（新） | 显示时钟或慢心跳 | 持续可读 lifecycle + sim_ray_num | **O(1)**，绝不 gate 在昂贵快照上 |

**当前 bug 的机制层根因**正是把 ④（廉价生命周期）焊死在了 ②（新快照 generation）上——只有产生新贵重快照时才顺带更新 lifecycle/stats。**把 ④ 独立出来、电平触发驱动 sim_state；② 只驱动 texture。** 这一刀就是修复的核心。

两个衍生洞察：
- 20ms poll 快于 batch 时，读到的 stats 根本不变（所以老代码才把它 gate 在 generation 上）。**数据的真实节奏是 batch 时钟**；poll 时钟合法的存在理由只是"给昂贵帧物化限流"，不是"数据刷新"。
- ④ 的廉价读取原语已经存在——task-317 的 `GetLiveSimRayCount`（O(1) 读运行计数，不触发 render-per-poll）。当前只是没把它用于生命周期跃迁。

（激进可选版：batch 边界由后端主动 push 帧进 triple-buffer，② 退化掉。本蓝图不要求，留作后续。）

---

## 7. 显示策略分层（按 epoch 键控，与生命周期正交）

质量门槛 / 防黑闪纯是*显示负载*策略。三条规则足矣：

1. 门槛**只压制运行中的中间稀疏帧**；
2. **终帧永远上屏**——跑完了没有更多帧，显示稀疏/全黑结果不叫"闪"，叫"结果"（不可能 filter 的全黑也要如实显示）；
3. epoch 前进时**不立即丢旧帧**（避免黑闪），但**超时兜底**，防止某个永远达不到门槛的 epoch 卡住显示。

一条铁律（P3 的落地）：**任何显示 gate 永不得压制一次生命周期跃迁。**

---

## 8. Stop 响应性与超大 batch（③ 很粗时）

CUDA 超大 batch 中途无法响应。第一性原理：
- 生命周期粒度 = batch 粒度，这是**诚实的**——batch 没跑完，GUI 显示 "Simulating" 是*正确*的（它确实还在算）。
- 用户中途按 Stop：后端无法打断正在派发的 batch，但前台可**立即反映 "Stopping…" 意图**（乐观 UI），待后端 drain 完当前 batch 后 reconcile 到终态。这正是 §5 把*意图*与*观测*分离的价值——一个小命令通道独立于观测通道。

> **实现修正（2026-07-03，task 1.6 cqs-optimistic-stop）**：本节最初把 Stop 终态抽象写成 "Idle"，
> 落地时经 owner 决定改为 **"Done"（`kStopEndState = kDone`）**。原因：GUI 的 Save 守卫是以 `kIdle`
> 为「无数据」判据的（`RefreshCpuTextureForSave` 提前返回 + Save 菜单 `has_server = sim_state != kIdle`），
> Stop→Idle 会 regress「Stop 后仍可保存已算结果」；且 `kIdle` 会被重载（「从未 run」vs「Stop 后有部分结果」，
> 恰是本设计要消除的那类歧义）。`kDone` = 现有终态行为，故 Save 守卫无需迁移。这是实现现实对蓝图抽象的
> 正当修正（非静默偏离），并保留本节要求的乐观 "Stopping…" 中间态 + 异步响应性（DoStop 立即返回、把既有阻塞
> 的 `poller.Stop() + LUMICE_StopServer` offload 到后台 `std::async` 线程，完成后由 `g_stop_inflight` 完成闩推进
> 意图 kStopping→kStopped）。`kStopEndState` 保留为单一 flip 点：未来若要改回 Idle，改这一行 + 迁移上述 Save 守卫即可。

---

## 9. 设计必须守的不变量（可固化成门禁 / 测试）

按 a01/a04（正交一手双确认 + 软约束须固化为自动化门禁）落成可验证断言：

1. **I1 世代单调 & 丢弃陈旧**：`committed_epoch` 单调；任何 `snapshot.epoch < committed_epoch` 的观测一律丢弃。
2. **I2 单一纯函数 owner**：`sim_state = f(最新 epoch 匹配快照, committed_epoch, dirty)`，只在一处计算，任何线程不得旁路写。
3. **I3 不停摆前提**：只要 `观测真相 ≠ 显示状态`，poll 不得停（电平触发自愈的前提）。允许 idle 时降频慢心跳，但不得彻底静默到无法自愈。
4. **I4 廉价可读**：生命周期 / 计数的可读性不依赖昂贵快照的产生。
5. **I5 原子快照**：跨线程 handoff 是单个版本化不可变值；消费方永不读半更新字段组合。
6. **I6 gate 不越权**：任何显示 gate 不得压制生命周期跃迁；终帧无条件上屏。

其中 **I3、I4 正是当前 bug 违反的两条**——回归测试应直接钉住它们。

---

## 10. 落地边界：最小核 vs 完整版

**最小落地核（先做，根治卡死，不推翻现有防闪逻辑）**：
- 引入 ④ 廉价生命周期心跳，与 ② 快照物化解耦；
- 生命周期跃迁改为**电平触发**（poller 不因自暂停而丢失终态；或 idle 慢心跳持续 reconcile）；
- 交接结构里生命周期信号与显示 payload 分离，生命周期字段每次采样都写（不 gate 在 generation 上）。
- 钉 I3/I4 的回归测试。

**完整版（后续，按需）**：
- epoch 世代号贯穿 commit / 快照 / 显示策略，替换 `has_valid_data`/`intensity_locked`/`stats>0` 一系列侧信号；
- 后端显式 `SimLifecycle` 枚举；
- 版本化不可变快照 handoff（triple-buffer/seqlock）；
- CQS 命令通道 + 乐观 "Stopping…" UI；
- （激进）后端 push 帧、退化 poll 时钟。

分期理由（a02/a03）：最小核已消除 owner 报告的卡死并守住 I3/I4；完整版是把一堆侧信号统一到 epoch 之下的结构性收敛，收益在于"未来不再长补丁"，可独立于紧急修复推进。

---

## 11. 与 GPU 路线的对偶

本文与 `seam-design.md` 是同一思想的两侧：
- seam-design：**GPU 侧**把"几何采样 / trace 派发 / 图像回读"三时钟解耦，别让一个旋钮打架另外两个。
- 本文：**GUI 侧**把"显示刷新 / 快照物化 / batch 生产 / 生命周期心跳"四时钟解耦，别让昂贵帧物化绑架廉价生命周期。

共同的元原则：**每个时钟一个独立频率；跨时钟边界只用单一版本化交接；真相有唯一 owner，其余是电平触发的投影。**
