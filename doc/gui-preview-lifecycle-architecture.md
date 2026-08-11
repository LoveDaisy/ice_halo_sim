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

> ⭐ **"drain 干净"这半句是 §9 的 I7，不是修辞**（owner 2026-08-06 定案）。它约束的是
> **谁有资格宣告终态**：终态谓词必须蕴含消费端已排空，否则宣告之后读到的累加量可能只是部分总和。
> 落地形态见 I7 条目与其下的补丁追记；机制入口是 `LUMICE_GetDrainStatus`。
> ⚠️ **这行交叉引用本身是有代价换来的**：这句定义在本文存在了数月却从未被抬进 §9 的可核验清单，
> 那次转录损失的账单是两个历史缺陷（终帧被质量闸吞掉的卡死、消费端排空前终态被观测到导致部分总和）。
> **改本节的 `Completed` 语义时必须同步改 I7，反之亦然——两处是同一条契约的两个投影。**

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
7. **I7 完成蕴含排空**：任何向消费方宣告终态的谓词（`SimLifecycle::Completed` / `kIdle`），必须蕴含**该世代的数据已被消费端完全排空**——即终态宣告之后读到的累加量即为终值；且终态之后的销毁路径不得丢弃未消费批次。

其中 **I3、I4 正是当前 bug 违反的两条**——回归测试应直接钉住它们。

> **落地补丁（2026-08-01，PR 见 git log）：I6「终帧无条件上屏」曾长期未被落实。**
> 这不是新增不变量，而是补上实现对 **I6 后半句**（§7 规则 2）的一次合规回归——I6 本身文本不变。
>
> gap（四环，每环都有生产代码位置）：①`ServerPoller::SetCalibratedThreshold` 把启动标定阈值
> （真机量级 3–4×10⁴ 光线）写进全局 poller；②`PollOnce()` 的质量闸按该阈值拒绝稀疏快照；
> ③质量闸本有 `kQualityGateTimeoutMs`(500ms) 兜底，但 **poller 的 COMPLETED 自暂停位于同一次
> `PollOnce()` 调用内、且在质量闸之后**——有限低采样跑约 80ms 就 COMPLETED，此后再无轮询去触发兜底；
> ④⇒ 纹理永不上传。**用户可见症状：把总光线数配到标定阈值以下的有限仿真，跑完了预览图也永远空白。**
> 根因形态正是本文档要治理的「显示钟 × 生命周期钟」交界——§7 规则 1 的前提（"还有更好的将来帧"）
> 在 COMPLETED 之后不成立，而实现把规则 1 无条件套到了终局帧上。
>
> 修复落点：`ServerPoller` 新增 **per-resume**（非 per-poller-lifetime）状态
> `uploaded_since_resume_`，在每个 kPaused→kRunning 边沿（`Start()` 与 `TransitionToRunning()`，
> 即 `WakeForRestart` 与 `WakeForRefresh` 两条唤醒入口）与 `last_generation_` /
> `last_quality_pass_time_` 一起复位；`PollOnce()` 在 `lifecycle==COMPLETED` 且本次 resume 尚未
> 上传过任何一帧时**绕过质量闸补投一帧**。三点值得记住：
> - 该判据**故意求值在 `has_new_snapshot` 分支之外**——§6 的生命周期心跳与快照物化是两个解耦的钟，
>   COMPLETED 完全可能落在一次不带新 generation 的 poll 上，只堵「快照与终态同一次到达」那条
>   交错会留下另一条同形缺陷。
> - 500ms 超时兜底**保留不动**，两者并列不替代：兜底救的是「运行中永远够不到门槛」，本补丁救的是
>   「已经结束且结束在门槛之下」。
> - `!uploaded_since_resume_` 是防止补投蜕化成「每次 COMPLETED 都重物化一帧」的唯一闸门（那会用稀疏帧
>   盖掉已有好帧，正好制造质量闸本要防的闪烁）；per-resume 而非 per-lifetime 也是必需的，否则只有进程内
>   **第一次**低光线完成跑被救到。`WakeForRefresh` 同样复位，是因为它的全部用途就是给已完成的跑买
>   一次额外 poll 去重新物化（改染色 / composite EV），被质量闸吞掉的话该跑的显示编辑将永远不生效。
>
> 回归测试（2026-08-11 重锚：GUI 套件重写把这三阶段从需要真实帧的 `gui_test` `gui_lifecycle` 组
> 挪进了无窗口的 `gui_unit_test`）：`ServerPoller.ATerminalFrameBelowTheQualityGateIsStillUploadedOnEveryResume`
> （`test/unit-correctness/gui/test_server_poller.cpp`），三阶段仍分别覆盖首次完成跑、经
> `WakeForRestart` 的第二次完成跑、以及经 `WakeForRefresh` 的 display-time 刷新。

> **落地补丁（2026-08-03，PR 见 git log）：I1 的前提「世代号是诚实的」曾不成立。**
> 同样不是新增不变量，而是补上实现对 I1 的一次合规回归——I1 文本不变。
>
> I1 靠**世代号**丢弃陈旧观测，`ShouldUploadPayload`（`src/gui/app.cpp`）的
> `payload_epoch > display_epoch_floor` 是它在纹理通道的落地。但这道闸只能问「号是多少」，
> 问不了「这份内容是不是真产自该号」——**前提是生产端不会给旧内容盖新号**。生产端曾会：
> ①`RenderConsumer::Reset()` 显式不清 `snapshot_xyz_`（"PrepareSnapshot will memcpy over it"），
> 而 `PrepareSnapshot()` 只在 `snapshot_dirty_` 为真时跑，`ServerImpl::Stop()` 刚把它清掉
> ⇒ 重启后、新一代首批落地前，缓冲区**原封不动**是上一代的完整图像；
> ②`RawXyzResult::epoch` 每次读取都无条件重取活的 `committed_epoch_`，`CommitConfig` 已 bump；
> ③`ServerPoller::PollOnce()` 用这对值构造 payload ⇒ 旧图 + 新号。
> 此时 `MarkStructHardDirty` 抬到**旧** `committed_epoch` 的 floor 恰好拦不住（新号 > 旧 floor），
> 于是"新一代还没出图就先把上一代的图当本代成果推上屏"，把 floor 这道围栏原地架空。
>
> 注意与「同一次读取内不会读到半更新的配对」**不是同一件事**。那条保证今天由不可变结果帧承担：
> 一帧由一趟快照整体组装后发布，故同一帧上的任意两次读天然同世代（v4.15 之前它由
> `ServerImpl::GetRawXyzAndCompositeResults` 的锁下序列化提供，那个配对 getter 已随帧模型一起删除
> ——机制换了，这里要说的区别没变）。此处两个值**各自完整自洽**，错的是配对本身——
> **无 tear ≠ 无陈旧**。
>
> 修复落点：`PollOnce()` 在质量闸的两条 bypass（超时兜底、终帧救援）**之后**追加一道内容世代闸，
> 只在「内容确属当前世代」时才允许物化 payload。判据不是裸 `has_valid_data`：它镜像
> `has_ever_consumed_`，被 `Stop()` 清除，因而为两种语义相反的原因转 false——「新一代尚未产出」
> （须抑制）与「用户按了 Stop，手上这帧仍属本代」（display-time 编辑须能重绘，是活路径）。
> 故改为比世代：`has_valid_data` 为 false 期间像素缓冲被冻结（只有 `ConsumeData` 能置
> `snapshot_dirty_`，且它在同一临界区内一并置 `has_ever_consumed_`），其真实世代即上一份已物化
> payload 的 `payload_epoch`；与活 epoch 相等 ⟺ 自像素产出以来没发生过 commit。
> 置于两条 bypass 之后是刻意的：兜底救「产出中但太稀疏」、终帧救「结束在门槛之下」，
> 都不构成发布别代像素的许可。I6 实质不受影响——COMPLETED 由 `has_ever_consumed_` 派生，
> 终帧必然满足第一个析取项。染色通道自动同覆盖：`PopulateCompositePayload` 就在同一个
> `if (quality_ok)` 块内，且合成结果同样不随 `Stop()` 清除——它与像素同住那份已发布的结果帧，
> 而 `Stop()` 不撤回该帧（见 `doc/accumulator-consumer-architecture.md` §3.1）。
>
> **主动接受的窄缺口**：某一代若产出了数据、又在 poller 从未 poll 过的间隙里被 Stop，则没有任何
> payload 记录过该代 epoch，其画面会被抑制、屏幕停在上一帧。闭掉它需要**服务端**戳出
> "`snapshot_xyz_` 是在哪一代填的"（新的 C API 字段）——poller 侧状态**结构上做不到**，因为它
> 按定义从未观测过那一代。判定其代价大于该缺口，故留白。
>
> 回归测试（2026-08-11 重锚至 GUI 套件重写后的名字，均在 `gui_unit_test` 的
> `test/unit-correctness/gui/test_server_poller.cpp`）：
> `ServerPoller.TheRestartWindowRepublishesNeitherThePriorRunsStatsNorItsPixels` 断言**像素内容
> 指纹**而非只断言世代号——只断言号会在时序恰好没撞上时以错误理由变绿；
> `ServerPoller.TheRestartWindowDoesNotRepublishThePriorRunsComposite` 是它在合成通道的同族，
> 断言窗口期内整份 payload 被原样 carry-forward（`is_composite` / `payload_epoch` /
> `texture_serial` 三者都停在上一代）而不是拿上一代的 lane 重建一份。

> **落地补丁（2026-08-06，PR 见 git log）：I3 两个分句此前都不合规。**
> 同前两条，不是新增不变量，而是补上实现对 I3 的一次合规回归——I3 文本不变。
>
> **前半句「观测真相 ≠ 显示状态则 poll 不得停」**：`ServerPoller::PollOnce()` 尾部的自暂停
> guard 只判 `lifecycle == COMPLETED`。`COMPLETED` 是**生产端**判据（无 simulator 忙、无待生成
> 场景、生成已结束），它不问消费端是否已把 `data_queue_` 排空；而 `PreviewSnapshot` 的四个统计
> 计数在本次 poll 无新产出时是从上一份 **carry-forward** 过来的。两者相乘 = 在消费端排空之前
> 观测到 `COMPLETED` 的那次 poll 会发布一个**部分总和**然后停摆，此后没有任何一次 poll 去纠正它。
> 实测形态：`orientation_num` 19616 对 20000，短缺恒为 128（dispatch 粒度）的整数倍；
> Linux + Mesa（尤其 4 核）复现，macOS/Metal 不复现。
> guard 原注释的论证（「最后一次发布携带 COMPLETED，主线程 reconcile 仍能到 kDone」）本身成立，
> 但**只覆盖 sim_state 这一路投影**，覆盖不了同一 bundle 里的计数——「我可以不看了吗」必须对
> bundle 的每个字段都意味着「现在看到的就是真相」。
>
> **后半句「允许 idle 降频慢心跳，但不得彻底静默到无法自愈」**：`WorkerLoop` 在自暂停后坐
> 无超时 `cv_.wait`，**零轮询**；5 个唤醒入口（`Start` / 两处 `WakeForRestart` /
> 两处 `WakeForRefresh`）**全部由用户动作驱动**。即第二分句完全没有落实。
>
> ⭐ **两者合起来才是这条缝的机制层定性**：实现只依赖第一分句（guard 判得准），第二分句给的
> 安全网是零；而第一分句的前提不成立 ⇒ **guard 判错 + 无网可兜**。换句话说，**自暂停把终态观测
> 变成了唯一一次机会**——恰恰是在整个设计最要紧的那一次跃迁上，把电平触发退回成了边沿触发，
> 而 §3 P2 的原话正是「撕裂读、丢帧、时序抖动都不再致命，**因为不存在"唯一的一次机会"**」。
> 这解释了此前被当成三个独立缺陷分别修过的三次故障：终帧被质量闸吞掉、计数没到终值、
> 服务端侧同构的 `kIdle` 早报——**不是三个 bug，是一个一次性结构的三种失效方式**。
> 两半必须一起修：只修 guard 得到「更准但仍是一次性」，只加心跳得到「有网但 guard 仍错」。
>
> 修复落点（均在 `src/gui/server_poller.*`，无 C API / core 改动）：
> - 自暂停判据抽成纯自由函数 `ShouldSelfPause(lc, drain)`，三个条件：`COMPLETED` +
>   `drain.drained_epoch == lc.epoch`（消费端已排空——`LUMICE_GetDrainStatus`，见
>   `doc/capi-lifecycle-architecture.md`）+ `drain.current_epoch == lc.epoch`（读 `lc` 之后没有
>   更新的代被提交）。第三项**不能被第二项蕴含**：`drained_epoch` 单调而 `lc.epoch` 是快照，
>   commit 之后 `drained == lc.epoch` 仍然成立，少了第三项就会把刚开跑的新一代判成已终态。
> - 该函数是这个决策的**唯一 owner**，调用点不得再复制其中任何一项。这不是风格要求，是实测：
>   调用点曾保留一个 `lifecycle == COMPLETED` 前置检查，红态探针把谓词改成恒真时，本该抓它的
>   端到端否定用例仍然全绿——运行中的仿真根本走不到谓词。决策被劈成两半，测试就只能看见一半。
> - 自暂停的目的地从 `kPaused` 改为新增的 `kIdleHeartbeat`：worker 在该态用
>   `cv_.wait_for(kIdleHeartbeatIntervalMs)` 等待，超时即跑一次 `PollOnce()`。心跳复用既有
>   `PollOnce()` 而不是发明第二条读取路径——无新 generation 时它是 O(1) 的
>   （`DoSnapshot` 在 `snapshot_dirty_` 为假时直接早退，不做任何 stats/纹理构造），
>   且必然是安全空操作：`uploaded_since_resume_` 已为真 ⇒ 不触发终帧救援 ⇒ 走 carry-forward ⇒
>   `texture_serial` 不变 ⇒ 消费端不会重复上传。
> - **`kPaused` 与 `kIdleHeartbeat` 必须是两个状态**。心跳引入前，「自暂停」与「不再碰
>   `server_`」是同一件事，所以 `Stop()` 可以对自暂停态直接早退。加了心跳之后这是两个不同的事实：
>   `Stop()` 的调用方下一行就销毁服务器（`MaybeReconstructServerForBackend` 里
>   `Stop()` → `LUMICE_DestroyServer`；`DoStop` 里 `Stop()` → `LUMICE_StopServer`），
>   早退会留下真实 UAF 窗口。故 `Stop()` 现在把 `kIdleHeartbeat` 也收敛到 `kPaused` 并照旧等
>   `active_`，而 `active_` 的语义相应扩大为「正在执行**任意一次** `PollOnce()`」。
>   这一条经红态探针坐实：把 `Stop()` 的早退改回只认 `kRunning`，回归用例直接段错误。
> - `kIdleHeartbeat` 态若观测到 `lifecycle != COMPLETED`，只打一条 `GUI_LOG_WARNING` **探测日志**，
>   不做任何模式切换。理由：全 GUI 树枚举「能让 lifecycle 离开 COMPLETED」的路径，每一条都紧邻
>   一次 poller 唤醒或停止，该状态到不了；而心跳**本身**已是电平触发 reconciler，显示收敛不依赖
>   任何人通知，所谓「恢复全速」只是**速率优化**而非正确性属性。给到不了的状态写恢复分支，得到的
>   是一个永远跑不到、构造上无法被测试覆盖、却要后人搞懂它在防什么的死分支；写成探测器则真不可达
>   时零成本，真发生时**能学到东西**。明示接受的代价：若漏了某条路径，症状是「未通知的重启后预览
>   刷新走心跳速率而非全速」——可见、良性、下次任何唤醒即自愈。
>
> `kIdleHeartbeatIntervalMs = 500`（`gui_constants.hpp`）是纯 UX/成本折衷，**无正确性含义**——
> I3 只要求「不得彻底静默」，没给数字；该值待后续用容器复现配方做一次能耗/自愈延迟实测校准。
>
> 回归测试（2026-08-11 重锚至 GUI 套件重写后的名字与落点）：纯真值表那条现在归
> `composition_correctness_test` 的
> `RunLifecycleChain.SelfPauseNeedsCompletionAndADrainedUnsupersededEpoch`
> （`test/composition-correctness/gui/test_run_lifecycle_chain.cpp`，仍是 6 行真值表，直接喂
> `ShouldSelfPause(lc, drain)`）；其余四条留在 `gui_unit_test` 的
> `test/unit-correctness/gui/test_server_poller.cpp`：
> `ServerPoller.APollAtTheDrainedMomentPublishesFinalTotals`（接线 + 独立交叉核对
> `LUMICE_GetDrainStatus`）、`ServerPoller.AnUnboundedRunNeverSelfPauses`（guard 恒为真方向的
> 端到端否定）、`ServerPoller.TheIdleHeartbeatKeepsTickingAndRepublishesNothing`（I3b + 心跳不
> bump `texture_serial`）、`ServerPollerShutdown.StopQuiescesTheHeartbeatBeforeReturning`
> （Stop 静默契约；`Stop()` 返回**之后**取基线——契约是「返回后不得再有 tick」，而 `Stop()`
> 允许在途 tick 跑完）；外加 `gui_test`
> `run_lifecycle/a_finite_run_reaches_done_and_the_heartbeat_holds` 里的 `--fixed-dt` 双时钟断言
> （该用例在 Apple 上把 `use_gpu_backend` 打开后再跑，旧名 `gpu_run_reaches_done` 的 GPU 特化
> 并未丢失，只是并进了这一条）。
>
> **未闭合、明示**：`COMPLETED 但尚未排空` 这个组合没有注入 seam，本机（macOS/Metal）不复现，
> 因此只由真值表覆盖其逻辑；端到端证据来自 Linux/Mesa 容器复现，不来自本机绿。

> **落地补丁（2026-08-06，PR 见 git log）：I4 两个分句，前半句已合规回归，后半句明示留白。**
> 同前三条，不是新增不变量，而是补上实现对 I4 的一次合规回归——I4 文本不变。
>
> I4 原文管两样东西：「生命周期 / 计数的可读性不依赖昂贵快照的产生」。lifecycle 心跳读取一直在
> `PollOnce()` 的 gate 之外无条件跑；四个计数字段（`sim_ray_num`/`ray_seg_num`/`crystal_num`/
> `orientation_num`）的读取此前被同一个 `if (has_new_snapshot || force_final_upload)` 分支圈住
> ——同一条不变量的两半落在 gate 的两侧，中间隔约 160 行代码，是位置的意外而非设计选择。
>
> **前半句（写入不得 gate 在昂贵快照上）**：已合规——统计读取移出该分支，与其上方的 lifecycle
> 心跳读取对齐；纹理 payload 的物化仍留在分支内（只有真正新/被抢救的世代才需要物化纹理，这部分
> 本就不属于 I4 管的范围）。移出不增加成本：`PollOnce()` 本就无条件 `AcquireResultFrame()`，帧
> 早已取到，统计只是从帧上读。
>
> **后半句（读取路径本身廉价）**：明示留白，非遗漏。廉价原语 `LUMICE_GetSimRayCount`
> （`src/server/c_api.cpp`）只暴露 `sim_ray_num` 一个；`ray_seg_num`/`crystal_num`/
> `orientation_num` 在 C API 上仍无任何廉价读取路径——读它们仍须经 `AcquireResultFrame()` 触发
> 的帧物化（`DoSnapshot()` 在 `!snapshot_dirty_` 时早退，缓解但不消除该依赖）。留白理由
> （a04：举证责任在增加的一方）：今天没有任何调用方需要在不取帧的前提下读这三个计数；一旦发布到
> 公共 C API 面即不可逆（见 `doc/api-layering-and-product-lines.md`），先加后撤的代价远大于按需
> 再加。**重开条件**：出现真实消费者——需要在不取帧的前提下读这三个计数的调用方。
>
> 回归测试（2026-08-11 重锚）：`gui_unit_test` 的
> `ServerPoller.StatsAreReadOnAFirstPollWithBothMaterializeDoorsClosed`
> （`test/unit-correctness/gui/test_server_poller.cpp`），钉住前半句唯一可达的可观测缺口——某 poller 实例第一次 poll 恰好门关闭（两个 disjunct 皆假）且
> 无 `prev` 可 carry-forward，旧代码会让统计字段停在默认构造的 0。

> **落地补丁（2026-08-06）：新增 I7「完成蕴含排空」——owner 已拍板升格，§5 已同步指向它。**
>
> 与上面两条追记不同，**这一条是新增不变量，不是补合规回归**。但它新增的只是"清单上的位置"：
> 完成契约的排空语义早就写在本文档里——**本文 §5** 定义 `Completed` =「有限 run 跑完**并 drain
> 干净**（含 zero-output 完成）」——却从未被抬进本节「可固化成门禁/测试」的清单。
>
> ⚠️ **这是一次转录损失，账单已经付过两次**：语义在设计里存在了数月，却不在任何人核对合规时会看的
> 地方。两个已知历史缺陷（终帧被质量闸吞掉的卡死、消费端排空前终态被观测到导致部分总和）
> **可以读成同一处清单缺失的两次代价**。⇒ 记住这条的教训不是"要加 I7"，而是
> **"设计文本里的语义若没被抬进可核验清单，等于没写"**。
>
> **机制（先于规格落地，owner 定案的次序）**：`LUMICE_GetDrainStatus(server, &out)` 返回
> `{drained_epoch, current_epoch}`，契约「相等 ⟺ 该世代已排空」，由**消费线程**在排空时发布
> （`ServerImpl::PublishDrainedEpochIfSettled`，见 `doc/capi-lifecycle-architecture.md`）——
> 而非由生产侧谓词推断。⭐ 刻意**不挂在 `kIdle` 上**：§8 的实现修正已记 `kIdle` 被重载
> （「从未 run」vs「Stop 后有部分结果」），往一个有书面失败记录的超载信号上挂第三种含义是明知故犯。
>
> **消费点**：`ShouldSelfPause` 已把它作为自暂停 guard 的三个条件之一（见上方 I3 补丁）；
> `test/e2e/capi_runner.py` 已从"等 stats 收敛"的启发式轮询迁移到读该信号。
>
> **⛔ 同步义务**：§5 的 `Completed` 语义与本条 I7 是同一条契约的两个投影，改一处必须改另一处。
>
> **已知未覆盖**：`Queue::Shutdown()` 仍会丢弃排队中未消费的项，故「IDLE 即拆」是真实丢数据路径。
> 本轮**明示决定不改**（Stop 契约变更会让停止延迟取决于积压深度，属独立决策），
> ⇒ I7 后半句"终态之后的销毁路径不得丢弃未消费批次"**在 `Stop()` 路径上尚未落实**。

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

**落地状态（2026-08-06 核对，逐条据 §9 各条追记）**：

| 不变量 | 状态 | 落地方式 |
|---|---|---|
| I1 世代单调 & 丢弃陈旧 | ✅ 合规 | 更早一轮改动（PR 见 git log）落地内容世代闸主体 + §9 2026-08-03 追记补齐诚实前提 |
| I2 单一纯函数 owner | ✅ 合规 | 更早一轮改动（PR 见 git log），`ReconcileSimState` 落地为唯一纯函数 owner |
| I3 不停摆前提（两分句） | ✅ 合规 | §9 2026-08-06 追记（drain-aware 自暂停 guard + `kIdleHeartbeat` 慢心跳） |
| I4 廉价可读（两分句） | 🟡 前半句合规，后半句明示留白 | §9 2026-08-06 追记；三个计数无廉价 C API 路径判「已知已接受，不修」，见该追记的重开条件 |
| I5 原子快照 | ✅ 合规 | 更早一轮改动（PR 见 git log），`published_` 原子指针交换落地 |
| I6 gate 不越权 | ✅ 合规 | §9 2026-08-01 追记（终帧无条件上屏救援） |

上表之外，「（激进）后端 push 帧、退化 poll 时钟」**未落地，明示保留为后续 stretch 项**——
它是一个吞吐/延迟优化方向，不对应任何一条 I1–I7 不变量的合规性，不受本表约束。

**分期理由的更正（2026-08-06）**：本节曾断言"最小核已消除 owner 报告的卡死并**守住 I3/I4**；
完整版……收益在于'未来不再长补丁'，可独立于紧急修复推进"。**这句话把"先做的"等同于"已达成
的"，独立核验发现方向恰好相反**：被这里标为"完整版、按需再做"的 I1/I2/I5/I6 四条，全部**先于**
I3/I4 达成合规（随更早一轮改动与两轮独立补丁，日期见上表与 §9 2026-08-01/2026-08-03 追记）；
而这里标为"最小核已守住"的 I3/I4，在写下这句话时**实际四格全不合规**——直到这一轮核验才逐条
补齐 I3a/I3b/I4a，I4b 明示不修。⛔ 不删除本段是刻意的：这条缝已经把同一机制当新问题重新发现过
两次（§9 2026-08-01/2026-08-03 追记各自记了一次「合规回归」），本段是第三次核验的记录，留着才
能防第四次。
两者不是巧合：**§3 P2「电平触发优于边沿触发」的价值主张，恰恰只在"终态那一次观测"上才吃紧**——
I1/I2/I5/I6 管的是世代号诚实、owner 唯一、交接原子、gate 不越权，这些性质在仿真运行期间随时可
核，不特别依赖某一次观测；I3/I4 管的正是终态那一次观测本身能不能看见真相。把"先做后做"的排期
顺序当成"已达成"的证据，在这两条上正好落进了 P2 想防的那个陷阱。机制层的完整定性见 §9「整族的
机制层定性」。

---

## 11. 与 GPU 路线的对偶

本文与 `seam-design.md` 是同一思想的两侧：
- seam-design：**GPU 侧**把"几何采样 / trace 派发 / 图像回读"三时钟解耦，别让一个旋钮打架另外两个。
- 本文：**GUI 侧**把"显示刷新 / 快照物化 / batch 生产 / 生命周期心跳"四时钟解耦，别让昂贵帧物化绑架廉价生命周期。

共同的元原则：**每个时钟一个独立频率；跨时钟边界只用单一版本化交接；真相有唯一 owner，其余是电平触发的投影。**
