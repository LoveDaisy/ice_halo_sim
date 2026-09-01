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
- **根因不是缺中心数据结构，是缺"字段 → 转换档位 → 下游通道"的单一分类器**——今天每个 widget 就地决定发 `MarkDirty` / `MarkStructHardDirty`（旧名 `MarkFilterDirty`） / `PushDisplayState` / 直接赋值。

## 2. 两组正交的转换档位（读表前必读）

| 档位 | 语义 | 机制（现状） | 触发者示例 |
|---|---|---|---|
| **T-struct·hard** | 拓扑变，清屏 + 抬 epoch floor 栅栏旧世代纹理 | `MarkStructHardDirty()`（gui_state.hpp:782；scrum-353.5 前名 `MarkFilterDirty`，被 S4 正名）= MarkDirty + snapshot_intensity=0 + p99_raw_y=0 + display_epoch_floor=committed_epoch | 编辑谓词/combine/增删类/增删 ref、filter 结构变、staged filter commit |
| **T-struct·soft** | 配置脏但保留 carry-forward 纹理（不清屏） | `MarkDirty()`（gui_state.hpp:693）只置 dirty | 晶体几何/朝向、光谱、layer/prob、sim_resolution |
| **T-display** | 纯显示，即时下发 server，**不 dirty / 不 epoch** | `PushDisplayState`→`LUMICE_SetRaypathColors` | color rgb / visible / solo / z_order / composite mode |
| **T-view** | 纯客户端，仅 preview shader 实时重投影 | 无 server 下发（仿真投影固定全天空 dual-fisheye） | lens / fov / view / exposure |
| **T-session** | 会话偏好，不持久、不 dirty、不进 ConfigSnapshot | 直接改字段 | show_composite_preview / color_window_open / trackball |

> ⚠️ `renderer.background`（画面背景色，与 `bg_show` / `bg_path` 那组**背景图 overlay** 字段是两回事）**曾**列在 T-view 行的示例里，现已移出：它不是纯客户端重投影参数，编辑它必须能被 Revert 撤销。它今天处在 T-display 的**一半**上——「不 dirty / 不 epoch」已成立（`RenderConfigResimFields` 不再含它，Revert 改由 `ConfigSnapshot::renderer_background` 这个独立槽位追踪），而「即时下发 server」那一半**尚未建立**，所以一次背景色编辑目前不会到达任何地方，要等下一次 commit。故意不写进 T-display 行的示例列：写上去会读成"通道已通"。

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
- **S4** ~~`MarkFilterDirty` 名不副实~~ **已由 scrum-353.5 落地正名为 `MarkStructHardDirty`**（gui_state.hpp:782），与档位表 T-struct·hard 对齐。
- **S5** ~~signal 缓存无 server/epoch 键控~~ **已由 scrum-353.5 落地**：`RefreshColorClassSignals`（color_window.cpp）在入口比较 `(server, committed_epoch)`，任一 mismatch 即清 `signal_flags` + `last_poll_time=-1000` 强制立即重 poll，后端切换/epoch 递增不再展示旧信号。
- **偏离 E（Save 裁定，已落地）**：owner 裁定 = kModified 态 Save 前弹提示（"Run first / Save anyway / Cancel"）——scrum-353.5 落地。Save anyway 保留"存所见"能力，但由用户显式确认；同时**不清 kModified**（保留视觉线索）。
- **偏离 F（Revert 字段范围，已落地）**：~~Revert 恢复的 `renderer` 字段集宽于「什么才算改动」判定集——`ConfigSnapshot::From`/`ApplyTo` 整体拷贝 `RenderConfig`，于是改视角/换镜头/调 FOV 这些**从未点亮过 Revert 按钮**的操作会被 Revert 连带回滚（入不算改动、出算回滚）~~ **已修复**：这组字段的清单现在只声明在 `RenderConfigResimFields`（`gui_state.hpp`）一处，**「什么才算改动」与「Revert 恢复什么」读的是同一个真源**——该类型自己拥有三个方向：`From`（捕获）/ `Matches`（判定，`gui_state_reconcile.cpp::DiffAgainstCommitBaseline` 与 `app.cpp::DoRun` 的 `expect_rebuild` 都调它）/ `ApplyTo`（恢复）。`ConfigSnapshot` 的 Revert 基线字段随之从整份 `RenderConfig` 收窄为 `RenderConfigResimFields renderer_resim`，T-view 字段（lens_type / fov / elevation / azimuth / roll / visible / front）与 `exposure_offset` **结构性地不在基线里**（不是"捕获了但恢复时跳过"，那会让 `From`/`ApplyTo` 不对称），Revert 后保持用户当前实时值。

  ⚠️ 后续补充：「不在 `renderer_resim` 里」与「不在 Revert 基线里」这个等价关系**对 `background` 不成立**。它已从 `RenderConfigResimFields` 移出（改背景色不再算改动、不再触发重跑），但仍在 Revert 基线里——走的是 `ConfigSnapshot::renderer_background` 这个与 `renderer_resim` 并列的独立槽位。`RenderConfigResimFields` 单个类型表达不了这个组合（进了它，捕获/判定/恢复三件事绑在一起），所以第三种处置是「排除出判定、但另开槽位保留 Revert 追踪」。加 `RenderConfig` 字段时的三选一，见 `gui_state.hpp` 该类型上方的注释块。
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
| **T6** 清理/加固（scrum-353.5，已落地） | ✅ MarkFilterDirty→MarkStructHardDirty 正名（S4）、✅ signal cache 按 epoch/server 键控（S5）、✅ 裁定 Save（偏离 E owner 裁定 = 提示需 Run + 用户显式确认 Save anyway）、审计确认无 docking 死状态可删（活功能，a01 优先于 a05） | 各自小 AC，全部收敛 | 收尾 |

推荐序：T1→T2→T3→T4→T5→T6（T1、T2 可并行）。

## 7. 反模式警示（历史反复踩过，AC 设计须规避）
- **绿测 ≠ 真行为**：渲染/多状态类 bug 的 AC 必须验实际可观察输出（GL 像素 / sim_state 值 / FBO 抓图），不可断言 CPU 代理标志（`HasTexture()` 曾连续两次假绿到 owner 肉眼才暴露）。
- **同一 bug 连修错误层**：偏离 A/B/C 同源于交界无汇聚点；治理须建汇聚点，不可逐点打补丁。
- **carry-forward / 抗闪烁机制反成陈旧态来源**：改动 display/reset 逻辑时须验证 carry-forward 窗口不泄漏旧帧。

## 8. 用户默认值层（建于本文 §2 的档位表之上）

> 回答另一个总纲："每个可编辑字段是否可以固化为用户个人默认？固化后如何生效、如何回出厂？"这一层不引入新的状态转换档位，而是把 §2 的档位表**复用为资格判定的单一权威**，避免第二份"哪些字段可默认"的手写清单与档位表本身漂移。承载这一层的是顶栏 `ICON_FA_GEAR " Settings"` 按钮打开的 `Settings` 面板（`src/gui/defaults_panel.cpp`，下称"面板"）——它已从一次性的"确认对话框"重构为**纯编辑器**，8.3/8.5 是这次重构落地的两条核心分工，写在这里是因为它们最容易被后人误读。

### 8.1 四个命名空间

| # | 命名空间 | 成员 | 处置 |
|---|----------|------|------|
| ① 单例文档默认 | `FieldTier::kStructSoft` 中的单例字段（`sun` / `sim` / `renderer`） | 进覆盖文件的 GuiState 半区 |
| ② 预设库 | 内置 axis 预设表每一行可调的 zenith std（该值不是 `GuiState` 的顶层字段，落地时机与运行时形态见 8.7） | 进覆盖文件独立的 `presets` 子树，不占用 GuiState 半区的键名空间 |
| ③ app 偏好 | `FieldTier::kView` 中未被 `SerializeGuiStateJson` 序列化的那批（日志级别三件套 / 日志面板展开态 / 左侧面板折叠态），以及 `FieldTier::kStructSoft` 但 `auto_diff_excluded=true` 的字段（如渲染后端选择） | 一期不提供个人默认——这批字段的共同点是"不进文档"，归属清楚，故一期排除不留半拉工程；有具体诉求再评估 |
| ④ 集合区 | `FieldTier::kStructHard` 的成员，以及 `kStructSoft` 里被 `kCollectionFields`（`user_defaults.hpp:64`）标记的容器：晶体 / layer / filter / 染色规则 | 排除。这些容器在序列化后的 key path 里携带文档局部下标（如某个数组的第几项），脱离具体文档后这个下标没有意义 |

### 8.2 资格判定：从档位表派生，不手写第二份清单

`ResolveDefaultEligibility()`（`user_defaults.hpp:105`）以字段名为输入，在 `kDerivedFieldsExcludeList`（`gui_state_tiers.hpp:128`）与 `kFieldTierTable`（`gui_state_tiers.hpp:48`）这一"治理并集"里查找，返回 `kEligible` / `kIneligible`（附一个具体原因）/ `kUnregistered`（后者只应在字段名打错或漏注册时出现，从不是合法结果）。判定完全由已有的、被 `scripts/check_policies.py` 强制"每个 `GuiState` 顶层字段恰好登记一处"的档位表推导，本层没有再引入任何手写字段名列表——新增一个 `GuiState` 字段时，它的默认值资格随档位表的登记自动确定，不需要在这里同步第二处。

### 8.3 行的存在性是生成式，行的编辑器是注册式（分工写死，否则后人会读成 D3 被推翻）

面板"这个 key 值得展示成一行"与"这一行怎么编辑"来自两条独立的权威，必须分开理解：

- **行的存在性 = 生成式。** 面板要展示的行集合，来自对"当前 `GuiState`"与"当前生效默认（工厂值 + 已保存覆盖）"两份 `SerializeGuiStateJson` 输出做 JSON 树 walk-and-diff（`BuildDefaultDiffRows`，`defaults_diff.hpp:113`——面板用的正是这个带显式 overlay 文档的重载）。没有任何手写的、按字段名枚举的表：新增一个序列化字段，零改动即可出现在面板里，这条纪律与 8.2 的资格判定共享同一个道理（单一权威推导，不手写第二份清单）。
- **行的编辑器 = 注册式。** 一行画成滑条还是取色器、定义域是什么、当前是否可编辑，来自一张显式登记表（`FieldEditorEntry`，`field_editor_registry.hpp:111`），以**序列化键路径**（如 `"overlay_grid_color"`）而非 C++ 字段名（`grid_color`）为键——42 个可编辑叶子里有 18 个两者不同名。控件的形状不能从一个 JSON 叶子反推：`"0.3"` 说不出它是 `[0,1]` 里的 alpha、一个概率，还是一个角度。查表用 `FindFieldEditor()`（`field_editor_registry.hpp:126`），未注册返回 `nullptr`——这是设计好的合法答案，面板把它渲染成**只读**并在视觉上说明原因，而不是退化成一个通用输入框（后者会接受真实控件本会拒绝的值，比什么都不显示更有害）。

  405 的 D3（"编辑器只能注册，不能从 JSON 叶子派生"）**没有被推翻**：它一直管的是编辑器这一半，这次重构只是新增了存在性那一半的生成式做法，并第一次把两者在同一份文档里并列写清楚——不写清楚的代价，是后人看见"存在性是生成的"就顺势以为"编辑器现在也可以是生成的"。

  这张注册表也是主 UI 与面板收敛到"同一份权威"的落地方式：主 UI 里所有绑定到已注册字段的滑条调用点（`app_panels.cpp` 的 View / Display / Overlay 与 `panels.cpp` 的 Sun / Simulation，共 16 处）都改读同一个 `ConstraintFor()`（`field_editor_registry.hpp:137`），不再各自持有一份边界字面量——改一处，主 UI 与面板同时移动（`main-ui-constraint-registry` / 408.8 落地；其 AC3 的红态探针证明了这一点：故意改错某个约束的边界，主 UI 与表格单元格在同一次改动、同一帧一起偏移）。

### 8.4 覆盖文件即"残缺的 GuiState 文档"

覆盖文件的 GuiState 半区不是一种新格式：它就是一份省略了大多数 key 的 `SerializeGuiStateJson` 输出，读取时复用既有的反序列化器——后者本就是"缺 key = 走工厂值"的语义。应用时先把工厂 `GuiState{}` 序列化、再用覆盖文档做 JSON merge-patch、最后整体反序列化回 `GuiState`，即"工厂值 + 用户已存的键"，而不是把用户文档合并进调用方当时手上那份可能已被污染的状态。字段级类型错误会让整份覆盖被丢弃（而不是部分应用半份），这条设计取舍见 8.6 I3。

### 8.5 副本模型：面板是编辑器，不是文件的实时视图

面板打开时只读一次覆盖文件（`OpenDefaultsPanel()`，`defaults_panel.cpp:856`），产出两份文档：

- `g_snapshot_doc`——打开那一刻磁盘上的样子，整个会话内**冻结不变**。面板对"这个 key 来自哪里"（Source 列）、"它的生效默认是什么"、"复选框的起始状态"这些判断，全部锚定在这一份上，不随会话内的编辑而移动。
- `g_copy_doc`——按下 Save 此刻会写盘的内容。只有 §1 预设的 std 编辑直接改写它；主表的复选框把意图表达成一个选中集合，只在 Save 时一次性折进这份副本（`CommitCopy()`，`defaults_panel.cpp:323`，经 `WriteActiveOverlayDoc()` 写盘，`defaults_diff.hpp:98`）。

一切编辑——勾选/取消勾选、单元格内改值、Reset all——都只改这份工作副本；覆盖文件本身在按下 Save 之前**不会被写入一个字节**；关闭面板（Close / 标题栏 X / Esc）不写盘，直接丢弃这份副本。这是**单一提交点**：405.x 时代 Revert / Reset 曾经点击即写盘、Save 另外再提交一次，导致"点了 Reset all，面板看起来什么都没变——那 Save 现在到底会不会带上这次 Reset"从屏幕上根本看不出答案；副本模型把提交点收成一个，这个问题不复存在。

对应地，判据从"当前 GUI 值 vs 生效默认"改写为**"副本 vs 磁盘"**：`RowWouldChangeOnSave()`（`defaults_diff.hpp:150`）比较的是"按当前复选框状态，Save 会写出什么"与"磁盘上现在有什么"——这与"Differs from factory"（当前值 vs 出厂值）、"Edited this session"（复选框 vs 打开时的复选框）是三个独立的问题，互不可替代，三者的区分正是这次重构要解决的混淆本身。

**§2/§3 互斥完备不变量退休。** 405 时代面板分两节——§2"待采纳的改动"、§3"其余条目"——任何"当前值 ≠ 生效默认"的字段必然落在其中恰好一节，这条完备性是当时的一条固化不变量。两节合并成一张统一列表（408.3）之后，面板只有一张表、每行一个复选框，语义统一为"这个 key 是否在我的默认里"；"这行属于哪一节"这个问题连提出的位置都没有了，所以这条不变量不是被违反，而是**随它所描述的结构一起退休**——记在这里是为了不让后人在读旧 PR 时，以为两节的完备性今天仍然是判据。

### 8.6 五条不变量

- **I1 — 个人默认只影响"新建"，绝不改写已加载文件里已有的值。** 生效路径只有一条：`MakeNewDocumentState()`（`user_defaults.cpp:554`）的返回值被真正保留下来。`DoNew()`（`app.cpp:717`）走的是这条路径。`.lmc` 二进制加载完全不调用 `MakeNewDocumentState()`，直接把文件内容读入现有状态。CLI JSON 导入（`DoOpen()` 的 `.json` 分支，`app.cpp:628`）表面上也调用了 `MakeNewDocumentState()`，但随后的 `DeserializeFromJson()` 以整体赋值 `state = GuiState{}`（`file_io.cpp:1727`）覆盖掉这次调用的返回值，个人默认值在这条路径上因此不生效——调用点因此显式丢弃该次调用产生的降级计数，避免它被误报成"这次加载丢失了个人默认"。三条路径的净效果一致：打开一份已有文件，永远只看到文件自己写的值或工厂值，看不到别人机器上的个人默认。
- **I2 — 来源可见 + 一键回出厂。** 面板的一行同时给出来源与去留：Source 列基于 `g_snapshot_doc`（打开时磁盘上是否有这个 key）显示"Mine" / "Factory"，复选框表达"这个 key 是否在我的默认里"。撤销一条 = 取消勾选；一次性回到出厂 = "Reset all my defaults"，效果是清空整个勾选集合。两者都只改工作副本，真正回出厂只在随后按下 Save 时生效（8.5 的单一提交点）——不再有 405.x 那种点击即写盘的单行 Revert 按钮，来源可见与回出厂共享同一套"勾选 + Save"机制，不再是两条并列的控件路径。
- **I3 — schema 降级不炸，且留痕。** 覆盖文件比使用者自己保存的项目文件更新周期更松散，几乎必然经历字段增删。文件整体不可解析、字段类型不符、预设覆盖值非数字或非有限、覆盖值越界被 clamp，都归入同一个降级计数 + 说明列表通道，而不是分散成若干条各写各的警告——同一类失效只在一处可查。
- **I4 — 测试必须能与开发者本机的个人默认隔离。** `--user-config <dir>` / `--no-user-config` 是显式 CLI 开关而非环境变量（用户可见行为不能只靠 env 漂移），GUI 交互式二进制与测试二进制对"什么都不传"时的默认值不同：交互式默认自动探测 OS 用户配置目录，测试二进制默认禁用，从而参考图比对与本机个人默认解耦。
- **I5 — 覆盖文件位置遵循各平台的用户级配置目录约定**（Windows `%APPDATA%`、macOS `~/Library/Application Support`、Linux `$XDG_CONFIG_HOME` 或回退 `~/.config`），而不是与可执行文件同目录（只读安装 / 多用户机器下后者不可写或跨用户共享）。GUI 原先无条件写在 `$HOME` 下的日志文件一并收编到同一目录。

### 8.7 预设库覆盖值的定案：约束在既有分类器容差域内，不引入身份机制

用户可以覆盖内置 axis 预设（如常用的那几个）的 zenith std，但覆盖值被约束在该预设分类判据既有的容差域内；越界时 clamp 到边界并显式提示，判据本身零改动。这意味着"覆盖一个预设"目前实际上只有一个可调面（zenith std 一个浮点数）——分类判据对均值与方位角分布的约束比这更紧，所以设计上不应在 UI 上暗示可调面比这更宽。这一定案的前提是：触发需求的方向是"往判据合法域内部调紧"而非"越界调宽"；若未来出现真实的越界诉求，需要重新评估是否值得为其引入更重的身份携带机制。

### 8.8 与 `ConfigSnapshot` / Revert baseline 的边界

`GuiState::ConfigSnapshot`（`gui_state.hpp:1101`）只覆盖七个配置字段（晶体 / filter / layer / `sun` / `sim` / `renderer_resim` / 染色规则），其中没有任何一个**顶层字段**登记为 `FieldTier::kView`——而本层的默认值候选恰恰以 `kView` 为主力（`bg_*` / `show_*` / 各类颜色与 alpha / 面板折叠态）。

⚠️ "没有 `kView` 顶层字段"本身不等于"没有视图性的设置"，因为档位登记在**顶层字段**这一粒度上（`renderer` 整体登记为 `kStructSoft`，`gui_state_tiers.hpp`），一个顶层字段内部的成员不会单独出现在那份七项清单里。但对 `renderer` 这一项，答案现在是明确的：快照存的是 `RenderConfigResimFields renderer_resim` **外加一个 `float renderer_background[3]` 独立槽位**，而**不是**整份 `RenderConfig`，镜头 / fov / elevation / azimuth / roll / visible / front 与 `exposure_offset` 结构性地不在快照里，Revert 不覆盖它们；`background` 则相反——不在 `renderer_resim` 里（改它不算改动）却在快照里（Revert 要撤销它），见 §3 偏离 F 的补充。「Revert 应当覆盖哪些字段」这个语义问题已有答案且只有一个真源——`RenderConfigResimFields`，见 §3 偏离 F。

默认值层复用的是 `ConfigSnapshot` 背后"两份状态结构化 diff、按需派生效果"的**模式**，但不能复用其 struct：两者覆盖的字段集合不同，把默认值的 diff 引擎强行套进 `ConfigSnapshot` 会连带改动 Revert 语义。

### 8.9 覆盖文件的版本戳：只记录，不迁移，不设闸

覆盖文件带一个根级 `defaults_schema_version`（整数，当前 1），由**唯一写入方** `WriteUserDefaultsFile`（`user_defaults.cpp`）在写盘那一刻盖上——不是 diff 行、不由面板决定、调用方自带的值会被覆盖。所以"哪条路径写的文件"这个问题不存在：`src/` 下经 `WriteActiveOverlayDoc` 的那唯一一条生产路径必然带戳，这条单点约束本身由 `scripts/check_policies.py` 的 `user-defaults-single-write-path` 机械守着。

**它记录，但不裁决。** 加载路径上没有任何分支读它做决定：戳缺失 → 静默按正常文件加载（这是版本戳出现之前的所有存量文件，给它们记一条降级警告等于让每个老用户升级后白吃一条提示，与 §8.6 I3 想留痕的那类真实失效不是一回事）；戳的形态非法（字符串 / 负数 / 0 / 小数 / null）→ 等同缺失，但记一条 notice，因为文件对自己说了一句不可能为真的话；戳比当前构建**新** → 记一条 notice，**其余键照常全部生效**。

最后这条是本节与 `.lmc` 策略的分水岭，也是这个设计最容易被"顺手加固"改错的地方。`.lmc` 的 `kLmcVersion` 是硬闸——一份读不全的整份文档不该假装能忠实显示，那是对的。覆盖文件的结构恰好相反：它是一袋各自独立、每个都已有"不存在即取工厂值"回退的提示（§8.4），因为其中一个键来自未来就拒掉整袋，比"忽略不认识的键、其余照常工作"这个**它今天已有的行为**更差。真实场景就摆在这里：用户从 release 页下载多个版本来回切换，跑一次新版再退回旧版，一道闸会让旧版拒掉他全部个人默认值。⇒ **任何情况下都不得因为版本号而拒绝或降级加载整份覆盖文件。**

那它现在有什么用？现在没有，将来才有——而这正是它必须先写进存量文件的原因。本仓库既有的迁移触发器全是**数据形态**而不是版本号（如 `MigrateLegacyRaypathCommaConnector` 每次 load 无条件按文本判别，`raypath_segments.hpp`），键存在性与值形态能覆盖绝大多数变更。它们覆盖不了的恰好是一格：键名没变、值没变、**语义**变了。`presets.axis.*` 存的是裸 zenith std，含义依赖 `ClassifyAxisPreset` 的阈值常量——那两个常量哪天挪动，存量文件里的值就会静默映射到另一个预设，任何形态判别都看不出来。判别位必须在那次变更**之前**就在文件里，所以它今天先写，等有人需要时再读。

⚠️ **与 `kGuiStateSchemaVersion`（`file_io.hpp`，`.lmc` / GuiState payload 的版本）是两个独立计数器，故意不对齐，不得假设二者同步、也不得为了"看起来整齐"把其中一个重新编号。** 两者的键空间**重叠但不相等**：`presets.*` 是覆盖文件独有，`layers` 是 `.lmc` 独有且被覆盖文件整个排除（`kDiffEngineExcludedRootKeys`）——`.lmc` 至今 v1→v4 四次变更全部发生在 `layers` 底下，即覆盖文件的键空间零变更历史。共用一个计数器意味着任何一侧的语义变更都推高另一侧从未变过的数，制造"版本号涨了但其实什么都没变"的噪音。反过来，当一个**两边都能承载**的字段（`layers` 之外的某个 GuiState 序列化键）语义变了，两个计数器都要评估。这条告诫在两处常量旁各写了一遍——只写一处等于没写，改另一处的人看不见。

两条与文件形态相关的连带约束：

- **`schema_version` 与 `defaults_schema_version` 是两个键，不是一个。** 前者是 `SerializeGuiStateJson` 的输出里本来就有的根键（因此被 `kDiffEngineExcludedRootKeys` 排除在 diff 行之外），后者是覆盖文件自己的。同名复用会让同一份合并文档里一个键承载两种含义，打开文件的人无从区分。另外，`ApplyUserDefaultsOverlay` 的 merge 是 `merge_patch`，磁盘文档里任何与工厂文档同名的根键都会覆盖合并结果里的那个（值为 `null` 时更是直接**删除**该键）——所以 `BuildMergedOverlayDocument` 在 merge 之后无条件把 `schema_version` 回写成当前构建值：喂给反序列化器的那份文档描述的是**它自己**，不是它读过的文件。今天无害（反序列化器不读它），它咬人的那天正是版本号开始被用上的那天。
- **"只含一个版本戳"必须继续读作"没有个人默认值"。** 用户把所有默认值都改回出厂值后，文件从 `{}` 变成只剩这一个键。任何回答"这个用户有没有个人默认值"的判断都不得因此翻面——今天这样的判断只有 `ApplyUserDefaultsOverlay` 的早退一处，它按键名忽略版本戳。

---

## 9. 同一份 GuiState 的两个 Scene 编码投影（`kSimCommit` vs `kJsonExport`）

§2 的档位表回答「一个字段的改动该走哪条通道」。本节回答一个正交的问题：**同一份 `GuiState`
被编码成 `LUMICE_Scene` 时，有两个消费者，而它们要的不是同一张图。**

### 9.1 机制

`BuildScene(state, intent)`（`src/gui/file_io.cpp`）是 GUI 侧**唯一**的 core-config 产出点：
仿真提交与「Export Config JSON」都经它。两个 intent：

| intent | 消费者 | 它在回答的问题 | 不变式 |
|---|---|---|---|
| `kSimCommit` | `LUMICE_CommitScene`，喂给仿真 | 「core 要产一张什么样的**纹理**，供预览 shader 重投影？」 | 恒为 dual equal-area / fov 180 / `visible=full` / 黑背景的全天纹理，**独立于每一个视图设置**。用户的 lens/fov/view/visible/background 是显示期的 shader uniform（`RefreshPreviewParams` → `preview_renderer.cpp`），推进仿真会把天空裁剪并重投影两次 |
| `kJsonExport` | `LUMICE_SceneToJson`，落盘给 CLI | 「要告诉 CLI 什么，它才能画出屏幕上这张**成品图**？」 | 逐字段描述用户当下所见 |

⚠️ **这两条臂必须分叉，这是设计不是缺陷。** 曾经它们共用同一份 `LUMICE_RenderParam`、只有
`intensity_factor` 按 intent 分叉，于是导出的 config 是「GUI 内部仿真策略」的誊本而不是画面的
描述：用户看 linear/55° 得到一份 dual equal-area 全天配置，而 `grid.horizon` 不只是被丢弃、
而是被**写反**（导出的图会画一条用户明确关掉的地平线）。

⭐ 那时 `src/gui/file_io.hpp` 的注释把共用写成优点——「export 与 simulation 因此不可能漂移，
因为两者经同一个 `LUMICE_Scene` 编码」。**那句话字面为真，而且正是它保证了导出的东西描述的是
内部策略参数、不是用户所见。** 共用一个 struct 让「`lens_type` 对两条臂是不是同一个意思」这个
歧义**无法在类型上表达**，于是它静默按其中一个意思固化了四年。⇒ 一般化的判据：
**任何「两条路共用一个编码器、只在个别字段上分叉」的结构，都要问：不分叉的那些字段，
对两条路是不是同一个意思？**

（反面对照：`raypath_color_mode` 同样是显示层字段却从不出问题，因为它走独立的
`LUMICE_SceneSetColorMode` 不经过那份共用 struct。判据指向的是**那个共用结构**，
不是「显示层」这个属性。）

### 9.2 分叉面清单（权威源在测试里，本表是它的说明）

机械权威是 `test/composition-correctness/gui/test_scene_commit_chain.cpp` 的 `kDivergingKeys`
与用例 `IntentionalDivergenceFieldsMatchDocumentedSet`：把清单内的键从两份文档里 erase 掉，
**其余必须逐字段全等**。所以它同时守两个方向——清单内允许不同，**清单外意外分叉会红**。

| 键 | 为何分叉 |
|---|---|
| `intensity_factor` | 显示期 EV vs 烤进配置（commit 臂须留 `1.0`，否则重跑时手动 EV 被叠两次；export 臂须烤 `2^exposure_offset`） |
| `lens` | 投影 + fov：固定纹理 vs 用户的取景 |
| `view` | 相机角：全零 vs 用户的 |
| `visible` | 半球裁剪：恒 `full` vs 用户的 |
| `background` | 显示期合成 vs CLI 烤进图里 |
| `resolution` | 2:1 纹理 vs 用户的画幅 |
| `grid.horizon` | 恒开 vs 用户的开关 |

⚠️ **`grid` 是按子键豁免的，不是整键。** 只有 `horizon` 分叉，其余 grid 子字段（counts、colors…）
在两条臂上应当相同；整键 erase 会连带停止检查它们是否意外漂移。

### 9.3 三处具名例外（有意不分叉，且都不是遗漏）

- **`ray_color`**：两条臂都写 core 的「用真实光谱色」哨兵 `{-1,-1,-1}`。GUI 侧没有 tint 概念
  （全树只有 `src/server/render.cpp` 的烤图路读它），哨兵**就是** GUI 显示的样子。
- **`lens_shift`**：两条臂都保持零。GUI 全无对应控件 ⇒ 没有可供 export 臂「诚实描述」的用户值。
  哪天加了控件，这条注释就该停止是注释。
- **`front`（前半球裁剪）**：core 侧**无此概念**（`VisibleRange` 只有 upper/lower/full）。
  当前处置是 `BuildExportJsonOrWarn` 在 `front == true` 时**拒绝导出并告警**，复用它对
  ABI 超限的既有 `false + *out_warning` 契约。
  ⛔ **绝不可编码成 `"visible": "front"`**：`NLOHMANN_JSON_SERIALIZE_ENUM` 把任何未登记字符串
  映射到表中**第一项**，而 `kUpper` 正是第一项（`src/config/render_config.hpp` 那段注释为
  `EvMode` 写过同一条理由）⇒ 那样写出来的 config **CLI 渲上半球、GUI 渲全天+前半球裁剪，且不报错**。

### 9.4 改动纪律

- 动分叉面（增/删/改一个键）时，**先在本节说明理由再改清单**。⛔ 不得因为那道闸红了就把键加进清单绕开。
- **辅助线相关的键：`grid.angular_dist` 已经分叉，`grid.elevation` 仍不分叉。**
  上一版这里写的是「两条臂都不填它们，所以都不分叉」，那句话对 `angular_dist` 已经过期。
  裁定结果是上面两个候选里的**第一个**：core 现在算注解几何（`LUMICE_ComputeAnnotationOverlay`），
  但 GUI 仍然自己画 overlay，因为标注属于**成品图**而不是那张全天纹理——往纹理里烤线会被重投影
  一起重采样，线宽与位置都变形。所以 export 臂填用户的角度表、commit 臂留空，两者加入分叉面
  （`grid.horizon` 与 `grid.angular_dist` 两个子键在 `kDivergingKeys` 处按子键豁免）。
  `grid.elevation` 两条臂仍都不填，仍不分叉：GUI 那份是 FOV 自适应的单一步长 + 共用颜色，
  与本 schema「逐条命名」的形态还没有对齐方案。
  （附带一条仍然成立的观察：commit 臂的 `horizon` 值是**惰性**的——GUI 消费
  `LUMICE_FrameGetRawXyz`，而 horizon 画在 `PostSnapshot` 的 mono 烤图里，GUI 从来不读它；
  `angular_dist` 在 commit 臂留空是同一个理由的另一面。）
- 两条臂**是否真的只在清单上分叉**由上面那个用例守；两条臂**各自是否正确**由跨进程的
  CLI↔GUI 出图对照守（`test/gui/parity/`，见 `doc/testing-architecture.md` §4.10）。
  两道闸问的是不同的问题，缺一不可。
