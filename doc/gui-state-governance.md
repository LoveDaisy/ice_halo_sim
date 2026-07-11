# 设计：GUI 状态治理（染色 / 仿真前端状态的统一转换模型）

> 状态：blueprint（explore-gui-state-governance 收敛，2026-07-11）。改动 GUI 状态转换 / 染色 display 通道 / 文档切换重置 / 仿真生命周期显示联动前先读。
> 关联：`doc/gui-preview-lifecycle-architecture.md`（epoch / 四时钟 / I1–I6，本文的 sim 生命周期地基）、`doc/gui-custom-spectrum-and-raypath-color.md` §4.0/§5#25（re-sim vs display-time 分流铁律，本文形式化的对象）。

## 0. 为什么要这份文档

染色功能落地 GUI 后，人工走查陆续发现一批 bug（改色不刷新、Open 残留旧渲染连修三个错误层、show/hide 闪 Simulating、新增类全类误报 no-match……）。逐个修都能修好，但它们指向同一个系统性坏味道：**前端状态转换逻辑各自散落在多处、各自逻辑难对齐，没有统一 owner**。GUI 越复杂，这种散乱会加速拖死开发。本文是一次专项治理的收敛产物，回答 owner 定的总纲：

> **每一个「用户操作」 → ① 内部状态如何转换？ ② 所有相关显示如何更新？**

## 1. 诊断（一句话）

**`GuiState` 的数据模型是集中的（好）；散乱的是"状态转换"。而且几乎所有偏离点都不在 sim 生命周期本体，而在 display 通道与 sim 通道的交界处。**

- **sim 生命周期本体治理严整**，是正解样板：`ReconcileSimState`（app.cpp:945）是生产环境唯一写 `sim_state` 的地方（I2），每帧从 `(run_intent, committed_epoch, snapshot 观测, dirty)` 纯函数派生；业务操作只写这三个输入；epoch 栅栏（I1）弃陈旧世代；immediate 模式 filter/crystal 的 diff-gated 双档 mark 干净。
- **散乱集中在交界**：display 通道（`PushDisplayState`→`LUMICE_SetRaypathColors`+`EnsureRunning`）与 sim 通道（`FillColorClasses`→`CommitConfigStruct`）字段割裂、互不重推、且 display-time 操作会借用 re-sim 味的原语污染 sim_state。
- **根因不是缺中心数据结构，是缺"字段 → 转换档位 → 下游通道"的单一分类器**——今天每个 widget 就地决定发 `MarkDirty` / `MarkFilterDirty` / `PushDisplayState` / 直接赋值。

## 2. 两组正交的转换档位（读表前必读）

| 档位 | 语义 | 机制（现状） | 触发者示例 |
|---|---|---|---|
| **T-struct·hard** | 拓扑变，清屏 + 抬 epoch floor 栅栏旧世代纹理 | `MarkFilterDirty()`（gui_state.hpp:707）= MarkDirty + snapshot_intensity=0 + p99_raw_y=0 + display_epoch_floor=committed_epoch | 编辑谓词/combine/增删类/增删 ref、filter 结构变、staged filter commit |
| **T-struct·soft** | 配置脏但保留 carry-forward 纹理（不清屏） | `MarkDirty()`（gui_state.hpp:693）只置 dirty | 晶体几何/朝向、光谱、layer/prob、sim_resolution |
| **T-display** | 纯显示，即时下发 server，**不 dirty / 不 epoch** | `PushDisplayState`→`LUMICE_SetRaypathColors` | color rgb / visible / solo / z_order / composite mode |
| **T-view** | 纯客户端，仅 preview shader 实时重投影 | 无 server 下发（仿真投影固定全天空 dual-fisheye） | lens / fov / view / exposure / opacity / bg |
| **T-session** | 会话偏好，不持久、不 dirty、不进 ConfigSnapshot | 直接改字段 | show_composite_preview / color_window_open / trackball |

> 关键机制：**auto-commit 只在 kSimulating 下发生**（main.cpp:285-299：`dirty && sim_state==kSimulating` 每 kCommitIntervalMs 自动 DoRun）。**kModified（kDone+dirty）不会自动重跑**——用户必须手点 Run。

## 3. 全表：用户操作 × ①状态转换 × ②显示更新（spec 骨架）

完整逐操作表（~35 操作，带 file:line）见 `scratchpad/explore-gui-state-governance/`（E2 全表）。此处固化**偏离点**——即"用户做了操作，但状态转换或显示更新不对齐"的地方，按治理优先级排列：

### 偏离 A（活 bug，owner 头号症状）— display-time 颜色操作闪 Simulating / Run→Stop
- **现象**：有限仿真完成后，改颜色 / show-hide / solo / z_order / 切合成模式，Run 按钮瞬间变红 Stop、状态栏闪 "Simulating..."，随即恢复。
- **机制（白盒确认）**：
  1. 有限完成后 `run_intent` **停在 kRunning**（app.cpp 只有 New/Open/Run/Stop 写它，完成不改），`sim_state=kDone` 完全靠每帧"fresh valid COMPLETED 快照"派生——**未 latch**。
  2. display 操作 → `PushDisplayState`（color_window.cpp:84）无条件调 `EnsureRunning` → 内部 `PublishValidReset()`（server_poller.cpp:89）把已发布快照 `valid=false`（此复位本是给**真正重启**防闪烁用）。
  3. 下次 `PollOnce` 重发 valid=true 之前的若干帧，`ReconcileSimState`（app.cpp:952,969）在 kRunning intent 下 `!fresh` → base=`kSimulating`。
  4. poller 醒来重发 COMPLETED → 回 kDone，闪烁结束。
- **双重根因**：(a) display-time 操作借用 re-sim 味原语（EnsureRunning+PublishValidReset）；(b) `valid` 语义重载（"重启待定" vs "显示刷新待定"不分）+ 完成态未 latch，靠 volatile 输入每帧重算。color_window.cpp:79 注释自称 "display-time semantics preserved"，与实际相悖。

### 偏离 B — display 通道 vs commit 通道字段割裂（S1）
- commit 通道 `FillColorClasses`（file_io.cpp:1440-1461）发 color/combine/visible/solo/match/mode 但**漏 z_order**；display 通道 `PushDisplayState`（color_window.cpp:52-63）发 color/visible/solo/z_order/mode 但**漏 combine/match**。两通道各按不同字段子集下发同一份 `raypath_color`，靠"都从同一 state 读"维持一致——脆弱。
- **动态后果（偏离 B'）**：`DoRun` 走 commit 通道且不调 `PushDisplayState`，故每次 Run 重建 composite consumer 后 z_order 排序失效，dominant/painter 优先级错乱，直到用户下一次触碰任一颜色控件才恢复。

### 偏离 C — Revert 不重推 display 状态（S7）
- `DoRevert`（app.cpp:913-932）只 `ApplyTo` 恢复 GuiState（含 raypath_color）+ `dirty=false`，**无 `PushDisplayState`**。若 revert 掉的是颜色 display 编辑，server 仍持 revert 前的 display 状态，composite 预览要等下一次颜色编辑或 Run 才纠正。

### 偏离 D — 结构态与显示态挤在同一结构（S3，是 A–C 的共因）
- `ColorClassConfig`（gui_state.hpp:536-551）把 combine/match（改了要 re-sim）与 color/visible/solo/z_order（纯 display）放同一 struct。字段级路由只能在每个 widget 逐字段手判 → 是"漏推""漏发字段""发错通道"的温床。

### 其它（已固化，供裁定，非本次主线）
- **S2** `show_composite_preview` 双写路径（app.cpp:1021 vs app_panels.cpp:288）违背自声明单写者契约。
- **S4** `MarkFilterDirty` 名不副实（实为拓扑硬重置 + epoch 栅栏，被 color-class 结构编辑复用）。
- **S5** signal 缓存（color_window.cpp WindowLocalState static）无 server/epoch 键控，后端切换后最多 500ms 展示旧 server 信号。
- **偏离 E（设计问题待裁定）** Save kModified 态：`RefreshCpuTextureForSave` 存当前预览（上次 sim）+ 新 config + `dirty=false`（app.cpp:310）→ 落盘图≠配置且静默去 Modified。可能是"保存所见"的有意行为。
- **已证伪**：sim_resolution 改动**确实** MarkDirty（app_panels.cpp:726）——非偏离。

## 4. 目标模型：三支柱 + 文档重置 owner

无约束最优 = **一切下游 / server 可见态由单一 owner 从 `GuiState` 派生，而非在编辑点命令式推送**。约束（perf：whole-table server push 不能每帧做）剪枝为三支柱：

### 支柱 1 — field→tier 分类器（声明式，单源）
每个可编辑字段声明**唯一**转换档位（§2 的五档），集中为一张"字段 → 档位"声明表 + 单一 dispatch，取代今天每个 widget 就地手挑。最强的结构强制：**把 `ColorClassConfig` 拆成结构态子结构（combine/match）+ 显示态子结构（color/visible/solo/z_order/mode）**，让类型系统告诉你字段属哪条通道（消偏离 D，进而堵住 A–C 的共因）。

### 支柱 2 — 每通道单一序列化器 + 重推纪律
- commit 通道与 display 通道**都从同一个显示态子结构派生完整 payload** → 字段子集割裂（偏离 B）在结构上不可能。
- **重推纪律（新不变量 I-repush）**：任何改动 committed 显示态或重建 server-side consumer 的操作（`DoRun` / `DoRevert` / 后端切换）都必须重建完整 display 态 → 消偏离 B'（Run 后 z_order）+ 偏离 C（Revert）。
- **T1 落地方式的适用范围（code-review round-1 Minor-1）**：T1（染色域）把 `gui_state_reconcile.cpp` 直接 `#include` 具体 widget 头文件 `color_window.hpp` 来调用其 `PushDisplayState`。单域下这是可接受的最小实现，但**不是**要被 T2/T3 逐域字面复制的模式——若每条 `need_X_push` 通道都让通用 reconciler 反过来 include 对应 widget 头文件，reconciler.cpp 会随迁移域数量线性堆积对所有域 widget 头文件的依赖，违反"widget 依赖 reconciler 抽象、reconciler 不反向依赖 widget"的分层方向。T2 立项前应评估一层轻量的 push-handler 注册/分派抽象（例如按域注册回调，reconciler 只依赖注册表而非具体头文件），再决定是否值得为多域场景引入。

### 支柱 3 — latch 派生态 + display-time 操作禁碰 re-sim/lifecycle 原语
- (a) `PushDisplayState` 唤醒 poller 用**"刷新唤醒"**（不做 `PublishValidReset`，无 valid=false 窗口）——把 `EnsureRunning` 拆成 `WakeForRestart`（带 valid 复位，给真重启）vs `WakeForRefresh`（不带，给 display 刷新）。
- (b) 有限完成后 **latch COMPLETED 边沿**（run_intent kRunning→终态，或 reconcile 记住完成边沿），使单帧 valid=false 不再把 kDone 降级 kSimulating。与 lifecycle 蓝图"完成是电平可观测"一致，加边沿 latch 让 display 刷新惰性。
- 合起来消偏离 A。

### 文档重置 owner（支柱 1 应用于 New/Open/Revert = backlog #5）
单一 `ResetFrontendState(reason)` owner，按文档切换原因确定性重置**全部**前端态：预览纹理 + GL blank + poller staged（`InvalidateStagedTexture`）+ epoch floor + display 态重推 + trackball + run_intent + sim_state 输入 + signal cache。今天 New / Open(.lmc±baked) / Open(.json) / Revert 各手工重置不同子集——正是"Open 残留旧渲染连修三个错误层"（task-349.1→350→351）的根因。可顺带把文档切换视作 epoch 递增事件，统一栅栏陈旧快照。

### 与既有蓝图自洽
- §5#25 re-sim/display-time 边界 = 支柱 1 的 field→tier 分类器（形式化，不改语义）。
- I2（单一 owner 派生 sim_state）= 支柱 3 所外扩的范式；latch 是 I1（弃陈旧 / 电平触发）的精化（latch COMPLETED 边沿）。
- I5（原子快照）不受影响。

## 5. 固化为门禁 / 测试的不变量

- **I-tier**：每个可编辑字段恰属一个转换档位，由声明表单源决定（新增字段必须登记，否则编译期/门禁失败）。
- **I-serialize**：commit 与 display 两通道从同一显示态子结构派生；新增显示态字段不能只改一条通道（编译期强制）。
- **I-repush**：DoRun / DoRevert / 后端切换后，server 侧 display 态 == GuiState 显示态子结构（端到端 GL 像素回归可钉）。
- **I-display-inert**：display-time 操作不得改变 `sim_state` / `run_intent` / epoch（gui_test 断言：有限完成后 toggle 任一显示态 → sim_state 恒 kDone）。
- **I-reset-complete**：每条文档切换路径重置完整前端集（GL 像素 blank + poller staged 丢弃 + display 态），task-351 类回归栅栏。

## 6. 后续 scrum 拆解建议（无环依赖，逐 task 端到端可观察 AC）

| Task | 内容 | AC（可观察，非 proxy） | 依赖 |
|---|---|---|---|
| **T1** 活 bug（最高 ROI，独立） | 修偏离 A：display-time 唤醒不闪 Simulating（拆 refresh/restart wake 或 latch COMPLETED） | 有限完成后 toggle visible/solo/color → sim_state 恒 kDone（gui_test 断言 sim_state 本身） | 无 |
| **T2** spec+doc（地基） | 本文 promote 到 doc/ + field→tier 声明表 | doc 落地 + 索引 | 无 |
| **T3** 结构拆分（使能后续） | 拆 ColorClassConfig 结构态/显示态子结构（D）+ 两序列化器共派生显示态子结构（B） | 单序列化器；加字段不能使通道脱同步（编译期）；逐像素等价回归 | T2 |
| **T4** 重推纪律 | DoRun/DoRevert/后端切换重建完整 display 态（B'+C） | Revert 掉颜色编辑后 composite 无需再交互即匹配；Run 后 z_order 即时生效 | T3 |
| **T5** 文档重置 owner（backlog #5） | 单一 ResetFrontendState(reason) 覆盖 New/Open×3/Revert（可选 epoch 递增） | 各路重置完整前端集（GL 像素 + poller staged + display 态） | T3（受益） |
| **T6** 清理/加固（可选） | MarkFilterDirty 正名（S4）、signal cache 按 epoch/server 键控（S5）、裁定 Save（偏离 E）、删 docking 死状态 | 各自小 AC | 收尾 |

推荐序：T1→T2→T3→T4→T5→T6（T1、T2 可并行）。

## 7. 反模式警示（历史反复踩过，AC 设计须规避）
- **绿测 ≠ 真行为**：渲染/多状态类 bug 的 AC 必须验实际可观察输出（GL 像素 / sim_state 值 / FBO 抓图），不可断言 CPU 代理标志（`HasTexture()` 曾连续两次假绿到 owner 肉眼才暴露）。
- **同一 bug 连修错误层**：偏离 A/B/C 同源于交界无汇聚点；治理须建汇聚点，不可逐点打补丁。
- **carry-forward / 抗闪烁机制反成陈旧态来源**：改动 display/reset 逻辑时须验证 carry-forward 窗口不泄漏旧帧。
