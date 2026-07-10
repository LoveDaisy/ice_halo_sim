# 设计：GUI 自定义光谱 + per-raypath 颜色标记

状态：
- **功能 1（自定义离散光谱）已由 task-323 落地**（2026-07-03，见 `scratchpad/task-gui-custom-spectrum/`；同批完成 `ray_num` 语义统一为总数）。
- **功能 2（per-raypath 颜色标记）：地基（phase-1 CPU + phase-2 GPU）已由 scrum-331 落地合入 main**（PR #175 merge `aff094ef`，2026-07-06）——per-ray uint64 component 掩码三后端（CPU/Metal/CUDA）产出 + 跨 MS 层前向累积 + 交付 consumer，parity 亲验，生产默认 gated off、渲染逐位不变。**剩 phase-3 上层**（binning / 颜色分组层 / rule editor / 合成 shader）。
- **2026-07-05 设计深化 session（owner × AI）**：把功能 2 从最初"独立全局 classifier + 单层 bitmask"深化为 **跨层 component 掩码地基 + summand 级色桶（Fork C）+ 跨层 rule 层**。本文档据此重写 §1 / §4 / §5 / §6。核心结论：**地基窄而稳**——只需"产出 + 跨层累积 + 交付 per-ray component 掩码"三件事；binning 模型、rule 层、UI 全属地基之上，可延后，且**更换它们不需要重打三后端 seam**。
- **2026-07-06 phase-3 产品模型深化 session（owner × AI）**：地基焊好后进一步收敛"用户能做什么"这层，见 §4.1 / §4.6 / §5 台账 #17–#20。要点：① **色桶粒度收敛为"UI filter = 一个颜色类"**（拍平进 core 记边界，见 §4.1）；② **放弃放宽 1:1（Fork A）**——一个复杂 filter 已能表达任意跨 type 并集，多绑定无必要；③ **事实纠正**：core 的 `ComplexFilterParam` **本就支持跨 type OR/AND**（`filter_config.hpp:40`），缺口在 GUI 编辑器；④ **component 位预算 = 去重后按需分配的谓词原子**，颜色/rule 是免费的位集组合，超预算优雅降级（§4.6）。
- **⭐2026-07-08 Design-2 重定向 session（owner × AI，3b 开工前）**：owner 灵光一现——**把"颜色类"与"晶体上绑定的物理 filter"解耦**。颜色类不再是 filter 的 summand（推翻 Fork C），而是一张**独立的 placement-scoped 颜色谓词列表**，gate 上非破坏评估。物理 filter 决定存活、颜色谓词决定被画，两者独立；被门挡掉的染色路径自然不出弧。**见新增 §4.0（数据模型以它为准；§4.1 Fork C + §4.7 schema 标 SUPERSEDED）**。

功能 1 部分的 as-built 见 `doc/configuration.md` 的 `ray_num` 语义章节与 `src/gui/edit_modals.cpp::RenderSpectrumModal`。

---

## 0. 背景与两个功能

owner 计划扩展两个相关但不同的功能：

1. **自定义光谱**：让用户在 GUI 里配置离散波长 + 权重（把 config JSON 早已支持的能力带回 GUI）。**已由 task-323 完成。**
2. **per-raypath 颜色标记**：对不同 raypath 家族用不同颜色标记（如 3-5 红、1-3 蓝）。启用时**覆盖初始光谱颜色，仅取强度信号（Y 分量）来染色**。

两者在"强度依赖光谱"这一点耦合（功能2 的 Y 亮度仍由功能1 的光谱加权），但工作量差一个数量级：功能1 是补齐一条已存在的能力管道；功能2 是新增一个横切 core + GPU seam + GUI 合成器的可视化子系统。

---

## 1. 总纲：一个物理机制，三层角色（本设计的地基）

从用户视角只有一个"filter"概念。实现上是**同一个物理机制的三层角色**——注意不是三个独立子系统，全程 **一次物理仿真、固定种群、非破坏 tag**：

| 角色 | 是什么 | 绑定 / 粒度 | 对光线 | 改动触发 |
|---|---|---|---|---|
| **物理门 gate**（不动） | filter（Design A），决定光线**是否传播** | per-crystal，1:1（`filter-architecture.md §1` 硬不变量） | **破坏性丢弃** `w_=-1` | 改谓词 → 重仿真 |
| **色桶 bucket** | filter 的 **summand**（`ComplexFilterParam` sum-of-products 的一个 OR 项） | per-summand，随 filter（**Fork C**） | 非破坏 tag | 改 summand 谓词 → 重仿真；改色 → display-time |
| **跨层 rule** | **component（带层键的 summand）的布尔组合** | 全局 rule 列表 | 非破坏 tag | 改成员 → 见 §4.3 上层 binning；改色/序 → display-time |

**关键统一**：色桶就是 filter 自己的 summand——所以"filter = 门 + 桶"是**同一对象的两个角色**，不需要独立 Color Layers 面板、不动 1:1 绑定（详见 §4.1 Fork C）。

**为什么必须是"一次物理仿真、非破坏 tag"**：Design A filter 是物理门，会改变传播到下一 MS 层的种群。用户要的"把打到屏幕的光按 raypath 家族染色"，前提是**一次仿真、固定种群、事后分区**。若用"每色跑一次 Design-A filter trace"（被否的路线A），多散射下 N 次是 N 个**不同的物理仿真**而非一个种群的分区，会重复计数/错分，还会重蹈 task-200（把物理门挪去 consumer 侧 → 三 bug + 丢多层剪枝 → revert）。

---

## 2. 当前架构现状（explore 实测 2026-07-03 + 跨层核实 2026-07-05）

### 2.1 光谱 / 颜色管道
- core + config-JSON 层**完全支持**自定义离散光谱：`SpectrumConfig = std::variant<std::vector<WlParam>, IlluminantType>`（`src/config/light_config.hpp:24`）。功能1 已把这条通到 GUI（task-323）。
- 波长→XYZ 是线性叠加 `CMF(λ)×weight`（`src/core/color_util.hpp`，`SpectrumToXyz` / `SpectrumToXyzPerRay`）。**亮度 Y = CMF-Y 加权**，故功能2 的每桶 Y 仍依赖光谱——两功能在此耦合。
- 波长粒度依后端：离散列表 → per-(batch×wl) 遍历（CPU）；illuminant → per-ray wl pool（Metal/CUDA，`wl_idx` 是 uint8，`src/core/backend/wl_pool.hpp`）。**`wl_idx` 是"有界 per-ray 字段跨三后端 seam"的现成先例**，component 掩码类比它。
- **副产品发现**：当前**无色适应/参考白解耦**，白点钉死 D65 sRGB 矩阵。→ 独立议题，本设计不含。

### 2.2 filter / accumulator / consumer
- filter = per-crystal 物理门。`FilterConfig`（`src/config/filter_config.hpp`）；谓词有 raypath / entry-exit / direction / crystal / complex。**`ComplexFilterParam` 是 sum-of-products** `(f*..)+(f*..)+...`（`filter_config.hpp`），当前**塌缩成单一布尔**——但塌缩前的**逐-summand 匹配**正是功能2 要的 component（见 §4.2）。
- **多值 OR = 已经是 Complex**：GUI 的 `3-5; 1-3` 多 raypath OR、EE 多值 OR，都由 `file_io.cpp::SerializeFilterForCore` 翻译成一个 `ComplexFilterParam`，composition `[[id0],[id1],…]`（`filter-architecture.md §多-value OR routing`）。**即：OR 项 = summand 已是数据模型里的结构单元。**
- gate 在 `CollectData()`（`src/core/simulator.cpp` 约 :466）：pass+prob-pass→continue；pass+prob-fail→emit；**fail→丢弃**（`w_=-1`）。§5.1 device filter-match 已把 raypath/EE/Complex 匹配移到 device（三后端）。
- consumer（`RenderConsumer`）= **单条 XYZ lane**（`src/server/render.hpp`），**无 per-filter channel**。多图 fan-out 轴是 `RenderConfig`（view/lens）**不是 filter**，所有 consumer 拿同一份出射流。
- 预览 = **单纹理单 RenderConfig**：poller 写死 `GetRawXyzResults(..., 1)`；shader 有 overlay（背景 blend + 线/网格/label）但**不是多色合成器**。

### 2.3 GUI 现状
- filter 编辑 `RenderFilterModal`（`src/gui/edit_modals.cpp`）：per-crystal 卡片，谓词 = raypath / entry-exit + action(In/Out) + 对称 P/B/D，raypath 子面板已支持 `;`-分隔多 OR 项。**`FilterConfig` 无颜色字段**。多 filter 已 pool + 引用 round-trip。
- filter↔crystal 是 **1:1 单键绑定**（`ScatteringSetting{ FilterConfig filter_; CrystalConfig crystal_; }`，`proj_config.hpp`），`filter-architecture.md §1` 列为硬不变量 + 配套 linked-entry/传播机制。

### 2.4 MS 跨层结构（2026-07-05 核实，功能2 地基所系）
- **完整轨迹是跨层的链**：每段有 `prev_ray_idx_`（父段）、`root_ray_idx_`（链根），续传段从上一层 `is_continue_` 父段继承（`doc/raypath-rayseg-architecture.md`，`core/raypath.hpp`）。走 `prev_ray_idx_` 可重建整条跨层轨迹。
- **但 `rp_` 只是"当前 crystal 那一段"**（doc 明写 "rp_ is fully populated for the current crystal"）。故 gate/summand 匹配**只看当前层**——这就是"单层色桶接不了跨层轨迹"的机制根因。
- **⚠️ 更正（task-331.1 实测）**：早先设计假设"`is_prior_filter_failed_` 是唯一的跨-MS-层持久 per-ray bool、加宽它即可"——**该字段已被 scrum-237（remove-anchor-lane, 2026-05-28）连同 anchor lane 一起删除**（`grep is_prior_filter_failed_ src/` 零命中；现状 filter-fail → `w_=-1` 直接终止，无跨层"曾失败"标志；真源见 `filter-architecture.md §7`）。故 component 掩码**不是"加宽"某字段，而是全新建**一个 `RayBuffer` 并行数组，沿用同一套**前向传播模式**：只在 `InitRayFirstMs` reset、经 `TraceRayBasicInfo` fan-out 复制到子段、`CollectData` 交接进 `init_data[1]` 带入下一 MS 层、`InitRayOtherMs` 不 reset 故跨层存活。（`raypath-rayseg-architecture.md` 的旧字段表同属这片 stale 债，task-331.1 已顺手修正。）

---

## 3. 功能1 设计：自定义光谱（小）—— 已由 task-323 落地

保留供参考：core 就绪，缺口在 GUI state / C API（扩 struct 加离散数组载体，非走 JSON 侧信道）/ file_io 两条 load/save 路径。as-built 见 `RenderSpectrumModal`。注意波长数直接乘仿真量，UI 需给上界。

---

## 4. 功能2 设计：per-raypath 颜色标记（大，横切子系统）

### 4.0 ⭐Design-2 重定向：色 tag 与物理 filter **解耦**（2026-07-08 定案；SUPERSEDES §4.1 Fork C + §4.7 `{filter,summand}` schema）

> **状态**：owner 在 3b（GUI 编辑器）开工前的设计 session（2026-07-08）灵光一现并压测收敛。**推翻 Fork C**（§5 台账 #2「色桶 = filter 的 summand」）与 §4.7 的 `{layer,crystal,filter,summand}` id-ref schema。§4.1 / §4.6 / §4.7 保留作历史推理（含仍有效的思想：§1 三层角色、去重、rule-lane binning、compositor），但**数据模型以本节为准**。引擎（scrum-339）刚合入 main、**尚无 GUI / 无用户依赖**，故现在改 schema 是最便宜的窗口（a14 pre-release 纠正）。

**一句话**：颜色类不再"寄生"在物理 filter 的 summand 上，而是一张**独立的、placement-scoped 的颜色谓词列表**，在 gate 上作为**非破坏 pass** 评估。**物理 filter 决定光线是否存活；颜色谓词决定存活光线怎么被画。两者解耦。**

**为什么更对——回到 §1 根 principle**：§1 早就说物理门（破坏性）与 色 tag（非破坏）是同一次仿真的**两个不同角色**。Fork C 当初为了"复用 §5.1 gate 已算的 summand 匹配、省一次评估"，把 tag 又粘回了 gate 的 summand——把两个本该分开的角色重新耦合了。Design 2 拆开它们，回到根 principle。**故本次不是"违反蓝图"，而是纠正一个中层妥协。**

**决定性论据（Fork C 表达不了的常见诉求）**："保留全部光线，只把其中 3-5 那一支染蓝"。
- Fork C 下颜色 = filter summand：要染 3-5，晶体上就得有个含 3-5 的物理 filter，**而那会把非 3-5 挡掉**（改了物理）；或者只能染"整颗晶体"（339.1 none-filter 整晶体位），染不了子集。
- Design 2 下：物理 filter 决定谁存活，颜色谓词决定谁被画，独立。3-5 染蓝，其余照常出射。
- owner 原话"染色路径被 filter 挡掉就没有那条弧"——这是**物理正确**行为：染色是对**存活种群**的事后划分（§1「一次仿真、固定种群、事后分区」）。

**变（管线前段） / 不变（地基后段，那些贵且难对的）**：

| | 内容 |
|---|---|
| **变** | ① config schema：`match` 从 `{layer,crystal,filter,summand}` id-ref → **placement-scoped 独立谓词** `{layer, crystal, <raypath/EE 谓词>}`（脱离该晶体的物理 filter）；② `BuildComponentTable` 位来源：从"扫物理 filter 的 summand" → "扫颜色配置的谓词列表"（**恰好 = on-demand 分配**，§4.6 收敛点，二者收敛为同一实现）；③ CPU gate：加一遍**非破坏颜色谓词匹配**（同 §5.1 一样用 `rp_`，只是不动 `w_`） |
| **不变** | 跨层掩码前向携带、consumer rule-lane（§4.7 定案 1）、compositor（dominant/additive/painter + z-order + 可见性）、三后端掩码机制、landmine 审计——**全不碰**。GPU 侧的颜色谓词评估折进 scrum-3c |

**placement-scoped 不变量（= 承诺 #2 的延续，不可退让）**：谓词**必须带 (layer, crystal)**。物理事实：**同一 raypath 在不同晶体上是不同的弧**（几何不同→偏折不同→落屏位置不同）。全局谓词 `3-5→红` 会把两颗晶体的 3-5 **灾难性塌成一色**（three-arcs fixture 正是证伪此塌缩）。"到处染同一色" = **一个类 + combine:any + 多条 placement-scoped 原子** `[3-5@C1, 3-5@C2, …]`，UI 给"应用到所有晶体"便捷按钮展开。跨层 AND 也靠这个键：`3-5@(L1,A) ∧ 1-3@(L2,C)`。

**Design 2 顺手塌掉的特例（a12 统一原理的信号）**：
- **none-filter 整晶体位（339.1）** → 退化成 "match-all 谓词 @ (layer,crystal)"，不再特判。
- **linked-group / filter 共享 / per-placement 归属歧义**（3b 设计时暴露的坑）→ **蒸发**：颜色谓词根本不挂在 filter placement 上。
- **UI 大幅简化**（见下）。

**UI 形态定论——Tier 1 内联色块 ❌ 否决，专用颜色窗 ✅**（2026-07-08 owner）：
- **否决 Tier 1**（filter 卡上内联点色）：① 点了一个 filter 的色，其他 filter 怎么办（自动分配？保留光谱色？当不存在？）——无好答案；② 拖 z-order / 改配色 / 临时开关某类是**高频**操作，仍得回专用界面。故内联色块无必要。
- **✅ 专用颜色窗 = 一张颜色谓词列表**：每条 = placement-scoped 谓词 + 色 + combine(any/all) + 可见性 + z-order。谓词编辑**复用 333/334 已建的 H5 SoP 编辑器**（同一 widget）。跨层 AND = combine:all 的多原子条目。
- **Design 1（从已有 filter 自动生成类 + 只能组合已有条目）的价值保留为非绑定便捷**："从 filter 导入"预填一个颜色谓词模板，不是结构耦合。

**空弧反馈（footgun 可见化，v1 纳入）**：颜色谓词可引用被物理门挡掉的路径 → 空弧。每个颜色类显示"匹配 N 条光线"，N=0 标警告（具体形式 = 全量计数 vs 仅空警告，落地时定）。

**preview 合成 v1（as-built 白捡）**：core 已合成且已由 `LUMICE_GetCompositeResults()`（`lumice.h:436`）暴露（有 `raypath_color` 才非空）。v1 = poller 切 `GetCompositeResults` + 复用**已存在**的 sRGB 上传路径（`preview_renderer.cpp:617` `UploadTexture`，`u_xyz_mode=0`），**零 shader 改、零新读 API**。多 lane shader（host 侧瞬时切模式 / per-class 透明度）延后到证明需要（a05/a14）。

**API 按 re-sim 边界拆（seam 上焊死契约，a09）**：
- 改**成员/谓词**（哪些 component、any↔all） → config commit（dirty → 预览重累积，同改物理谓词）。
- 改**颜色 rgb / 合成模式 / 可见性 / z-order** → **独立 display-time setter**（`LUMICE_SetRaypathColors` 之类），只重合成、**绝不重启仿真累积**（不能因换个色丢掉收敛好的图）。
- C API 承载："成员" 进 `LUMICE_Config`（镜像 `compositions[]` 先例，`lumice.h:336-338`）；颜色/模式走独立 setter。

**UX 契约总表（rule-lane binning 的内在结果，非本次新增）**：

| 操作 | 边界 |
|---|---|
| 改颜色 rgb / 可见性 / z-order / 合成模式 | display-time（重合成，不重启累积） |
| 改色类成员 / any↔all | re-sim（预览重累积，同改谓词；GUI live preview 本就持续重仿真，非模态等待） |

**scrum-3b 自然分段（Design 2 下）**：① 引擎重定向到 Design 2（新 schema + `BuildComponentTable` 改源 + gate 非破坏颜色 pass，CPU，带回归）= a03 正确性地基先行；② C API 面（`raypath_color` 进 `LUMICE_Config` + 独立 display-time setter）；③ GUI 颜色窗（复用 H5 编辑器 + 色/combine/可见/z-order + 空弧反馈）；④ preview 合成 v1（host 侧 `GetCompositeResults`）。GPU device-side lane 仍是 scrum-3c。

**⭐as-built（②C API 面，task-capi-color-surface / task-342.2）**：上面"API 按 re-sim 边界拆"已落地并焊死：
- **成员/谓词路径**：`LUMICE_Config.raypath_color[]` / `raypath_color_count` / `raypath_color_mode`（**BREAKING v4.8** — 见下方 as-built #344；原始 342.2 以 v4.7 内联定长数组落地，344 转为堆指针）走既有 struct→JSON→`CommitConfig` 管线，**无独立 struct→core 分叉**——`ConfigToJson` 忠实 emit 与 `RaypathColorConfig::from_json` 同形状的 JSON，故 JSON-commit 与 struct-commit 逐像素等价（AC1，`RaypathColorApi.JsonAndStructCommitPixelEquivalent` 一手验）。谓词类型是 `LUMICE_FilterParam` 的**窄化复用**（raypath/entry_exit/direction/crystal/none，无 id/action/symmetry/complex）；`LUMICE_ColorPredicate.type==UNSET` 刻意 = match-all（与物理 filter 的 UNSET-reject 反向，对齐 core `NoneFilterParam` 默认）。
- **display-time 路径**：`LUMICE_SetRaypathColors(server, classes[], class_count, z_order, mode)` 只在 `consumer_mutex_` 下原地改 `active_class_table_` 的 color/visible/solo/z_order + composite mode，置 `snapshot_dirty_`，**绝不碰 epoch / 累积器 / Stop/Start**（AC2 一手验：epoch 前后一致 + `sim_ray_count` 不减 + 新色确实生效）。`class_count` 须等于已提交的 `raypath_color_count`（不等 = 成员结构变了 → `INVALID_CONFIG`，逼重 commit）；`z_order` 非 NULL 时须是 `[0,class_count)` 的排列（AC3）。
- **⭐z-order 与 Y-lane 物理下标解耦（关键正确性决策，白盒发现的陷阱）**：`ColorClass` 加 `int z_order_` 显示态字段（`BuildColorClassTable` 默认 = list 位置，`NeedsRebuild` 不比较它）。compositor `GatherActiveClasses` 按 `z_order_` 排序遍历，但**始终用原始下标 i 取 `GetColorClassLaneY(i)`**——遍历顺序（可被 setter 改）与 lane 物理绑定（构造时永久固定）彻底解耦。**天真地重排 `classes_` vector 会造成静默错误染色**（某条弧被涂错色），回归钉在 `test_component_compositor.cpp::ZOrderReordersDrawButNotLaneBinding`。
- **pre-existing 观察（不在本任务修）**：`CommitConfig` 在 `consumer_mutex_` 锁外写 `active_class_table_`/`active_composite_mode_`，与 `DoSnapshot`/`SetRaypathColors`（锁内）间有既有竞态（单-owner 规则本就假设 `CommitConfig` 不与他调用并发）。

**⭐as-built（③GUI 颜色窗，task-gui-color-window / task-342.3）**：`src/gui/color_window.{hpp,cpp}` 落地非模态浮动窗 `RenderColorWindow`，顶栏按钮开关，只经 `include/lumice.h` 访问 core（GUI API 边界门禁）：
- **外壳 + 类行**：按 `z_order` 升序展示，每行 = 上/下箭头 / 色块 `ColorEdit3` / 可见性（眼睛图标，普通点击 toggle `visible`，**Alt+click = 唯一 solo**，再次 Alt+click 复原全可见）/ 空弧警告 ⚠️ / delete。窗头下拉切合成模式（`dominant/additive/painter`）。**独立的 solo 列已删除**（task-list-row-ergonomics ③，owner 拍板）：`ColorClassConfig.solo` 字段与 compositor 的"有 solo 则取 solo 集"合成语义都不动，只是 UI 保证同一时刻至多一个类进入该集合；从旧配置文件加载的多 solo 状态不做启动时归一化，等用户下次 Alt+click 时才收敛为互斥（AC4 owner 复验清单需知悉）。
- **z-order/物理下标解耦在 GUI 层的复现**：`SwapZOrder(state, phys_a, phys_b)` 只交换两个 class 的 `z_order` 标量字段，`state.raypath_color[]` vector 物理顺序永不变；这是 §4.0 上方"z-order 与 Y-lane 物理下标解耦"契约在 GUI 层的镜像，回归钉在 `test_gui_color_window.cpp`（`swap_zorder_*` 系列用例）。删除类后 `CompactZOrder` 把留空的 `z_order` 压缩回 `[0,N)` 排列（`LUMICE_SetRaypathColors` 的 `z_order` 参数硬要求）。
- **ref 编辑器**：per-class 展开区含 combine（any/all）+ per-ref layer/crystal 下拉（从当前 layer 已有 placement 反查去重，从源头消除孤儿引用）+ whole-crystal checkbox + 谓词文本框，复用 333/334 已建的 H5 SoP 单-atom 校验（`ValidateSingleAtomText`：空文本=whole-crystal 合法、单 raypath 合法、多 factor 或 `;`-OR 拒绝——一条 ref 只能是单谓词原子，跨谓词的 AND 走 combine:all 的多条 ref，不进单条文本框）。**whole checkbox 勾选时谓词文本框冻结**（`BeginDisabled` 灰化而非隐藏）而非清空/消失（task-list-row-ergonomics ④），取消勾选后恢复可用且保留原文本；`SetRefMatchAll` 只改 `match_all` 一位，`file_io.cpp:1337` `FillColorPredicate` 优先判 `match_all` 后再读 `predicate_text`，冻结态陈留文本不进 commit（序列化安全性已白盒核验）。
- **从 filter 导入**：`BuildClassFromFilter` 把一个物理 filter 的 SoP 逐行转 ref，单-factor 行转 ref，多-factor 行跳过并计入 `skipped_rows`（复用既有 Import Warning 弹窗提示）；`combine` 默认 `any`（§4.0"Design 1 的价值保留为非绑定便捷"——只是预填模板，非结构耦合）。
- **AC4 空弧警告**：轮询 `LUMICE_GetColorClassSignal`，500ms 节流（`kSignalPollIntervalSec`，对齐 lumice.h 头注释"debounce cadence, not per-frame"契约），`match_count>0 && signal==0` 时显示 ⚠️。
- **写路径分流**（§4.0"API 按 re-sim 边界拆"在 GUI 侧的落地）：色/可见/solo/z_order/合成模式 → 立即调 `LUMICE_SetRaypathColors`（display-time，不重仿真）；谓词/combine/加删类/加删 ref → `MarkFilterDirty()`（走既有 debounce commit 管线）。
- **测试**：`test/gui/functional/test_gui_color_window.cpp`（10 例，直接调 4 个纯 helper，无需驱动 ImGui 交互）+ Step 3/4 的 21 例 struct/JSON 结构等价 + `test_raypath_color.py` 端到端 PSNR 回归。AC6（owner on-screen 手感验收：拖 z-order、编谓词、导入、切模式）留给 owner 亲验，非 CI 可判定。

**⭐as-built（④preview 合成 v1，task-preview-composite-v1 / task-342.4）**：GUI 预览按帧感知 `raypath_color` 激活状态，激活时用 host 侧合成图取代 XYZ 上传，未激活逐字节回落到 §4.0 上方设想的"零 shader 改、零新读 API"路径：
- **Poller 双源采集**（`src/gui/server_poller.cpp` `PollOnce()`）：`LUMICE_GetRawXyzResults` 之后紧接一次 `LUMICE_GetCompositeResults(server, out, 1)`，以 `out[0].img_buffer != nullptr` 作为 raypath_color 是否激活的哨兵（`ColoredMask==0` 的 consumer 被 DoSnapshot Phase-2 skip，正是 §4.0 上方规定的 sentinel 语义）——**不引入跨线程状态**、无需从 GUI 主线程传 `GuiState::raypath_color` 到后台 poller。
- **`TexturePayload` 双缓冲**（`src/gui/server_poller.hpp`）：新增 `rgb_data`（sRGB uint8, W×H×3）与 `is_composite`；XYZ float 缓冲 **始终**填充（auto-EV / p99 / effective_pixels / quality gate 全从 xyz_data 派生，激活与否不影响），激活时**追加**填 rgb_data；未激活时 default-constructed 的 empty/false 即正确态。约 +25% 帧内存换 auto-EV/quality gate 管线零改动。
- **上传分流**（`src/gui/app.cpp` `SyncFromPoller`）：按 `payload->is_composite` 分派到既有 `UploadTexture`（`preview_renderer.cpp:617`，sRGB 上传 + `u_xyz_mode=0`）或 `UploadXyzTexture`（原路，`u_xyz_mode=1`）。**snapshot_intensity/effective_pixels/texture_upload_count/last_uploaded_texture_serial/p99_raw_y/ev_auto 一字未改**，继续统一来自 xyz_data 派生字段。
- **代际漂移检测 + 丢帧回退**（`ServerPoller::PopulateCompositePayload`，`server_poller.cpp`）：`PollOnce()` 里 `LUMICE_GetRawXyzResults` 捕获 `xyz_generation` 与之后的 `LUMICE_GetCompositeResults` 调用之间存在窗口——若窗口内消费了更新的 dirty 事件，两者会分属不同物化世代，直接配对会静默把两个物化事件混进同一 payload。`PopulateCompositePayload` 在拷贝 composite 字节后立即用一次廉价的 `LUMICE_GetRawXyzResults` recheck 比对 `snapshot_generation`：一致则 `is_composite=true`；不一致则清空 `rgb_data`（`clear()+shrink_to_fit()`）、`is_composite` 保持 false，payload 退回纯 xyz 态（stats/auto-EV/quality-gate 仍按 xyz 侧正常推进），下一次 poll 再重试配对。`LUMICE_RenderResult` 不带世代字段（扩它超出本任务范围），故用 host 侧 recheck 而非结构体字段判定。
- **⭐正确性地基（Step 1）——`DoSnapshot()` 收敛四类 consumer 共享单一物化入口**：白盒发现 `GetRawXyzResults` 与 `DoSnapshot`（`GetRenderResults`/`GetCompositeResults`/`GetStatsResult` 共享）**各自持有独立的 Phase-1 dirty 消费逻辑**，两套逻辑竞争同一 `snapshot_dirty_` 标志。本任务第一次在同一次 `PollOnce()` 里让 RawXyz + Composite 两个消费者同 tick 读取，两种朴素调用顺序都会破坏可观察行为——先 RawXyz 则 Composite 见 `snapshot_dirty_==false` → `cached_composite_results_` 永久陈旧（AC1/AC3 失效）；先 Composite 则 `snapshot_generation_` 从此不再增长（poller `has_new_snapshot` 恒假，纹理再也不更新）。修复：把 `snapshot_generation_++` 从 `GetRawXyzResults` 挪进 `DoSnapshot()` Phase-1 临界区、后者签名 `void`→`bool`；`GetRawXyzResults` 改为调用 `DoSnapshot()` 后再读结果（同 `GetRenderResults`/`GetCompositeResults`/`GetStatsResult`），删除末尾与 Phase-2 重复的非-RenderConsumer StatsResult 缓存分支。修复后不变量：**`snapshot_dirty_` 只被"第一个到达的 Get\* 调用"通过 `DoSnapshot` 消费一次，`PrepareSnapshot`/`PostSnapshot`/合成/`snapshot_generation_++` 作为同一原子事件发生**——正是 `gui-preview-lifecycle-architecture.md` 反复强调的"单一物化事件 + 多个只读视图"在第 4 个消费者身上的补课。回归钉在 `RaypathColorApi.RawXyzThenCompositeSeesFreshGeneration` / `.CompositeThenRawXyzGenerationStillAdvances`（`test/unit-correctness/server/test_c_api.cpp`）。**未来任何要求在 `PollOnce()` 里加第三/第四个消费者的改动，都必须走 `DoSnapshot()`，别再手搓 Phase-1**。
- **测试**：
  - unit-correctness `RaypathColorApi.RawXyzThenCompositeSeesFreshGeneration`/`.CompositeThenRawXyzGenerationStillAdvances`——同 tick 内两种调用顺序的正确性回归（AC1/AC3 前置）。
  - gui_test `gui_composite_preview.raypath_color_active_populates_rgb_payload`——AC1 headless 锚点：payload `rgb_data` 与 `LUMICE_GetCompositeResults` 字节相等且非零。
  - gui_test `gui_composite_preview.no_raypath_color_stays_on_xyz_path`——AC2 逐字节零回归锚点：`xyz_data` 与 `LUMICE_GetRawXyzResults` 相等、`is_composite==false`、composite 哨兵 NULL。
  - gui_test `gui_composite_preview.generation_drift_drop_then_recover`——真正执行到 `PopulateCompositePayload` 内部的判断分支本身（区别于 `test_c_api.cpp` 只锚定"漂移可被 recheck 观测到"的 C API 前置条件）：借 `ServerPoller::PopulateCompositePayloadForTest`（测试专用直调 seam）先用 `LUMICE_SetRaypathColors` 制造真实漂移并断言丢帧分支（`is_composite==false` 且 `rgb_data.empty()`），再用正确配对的一对结果断言下一次配对恢复成功（`is_composite==true` 且字节与 `LUMICE_GetCompositeResults` 一致）。
- **AC4（切换激活/未激活无闪烁/撕裂 + 观感 + overlay 叠加）**：owner on-screen 人工闸，同 342.3 先例；本任务的自动化只覆盖机制层，视觉体验由 owner 亲验。
- **未做**：多 lane shader（host 侧瞬时切模式 / per-class 透明度）延后到证明需要（a05/a14）；GPU device-side 颜色谓词评估仍归 scrum-3c。

**⭐as-built（⑤染色 / 全光谱显示态切换 + 持久标记，task-colored-mode-affordance / task-345.4）**：④之上加一层"用户偏好"的 display-time 开关，用户不必删除颜色类就能临时看回全光谱视图，且激活状态下界面持久可见"当前处于染色模式"标记（关闭 Colors 窗仍可见）。零 C API / core 改动，纯 `src/gui/` 内。
- **两个 GuiState 显示态字段**（`src/gui/gui_state.hpp`，均不参与 dirty/Revert 生命周期，均不进 `ConfigSnapshot`）：
  - `show_composite_preview`（用户偏好，默认 true）——单一写点是顶栏 checkbox（346.3 起；345.4 首版位于状态栏）；
  - `last_uploaded_as_composite`（ground truth，`SyncFromPoller` 上传成功后写入）——供 UI 读作"屏幕上实际显示的是什么"。绑定 ground truth 而非用户偏好，结构性消除偏好翻转到下一次上传之间的一帧"标记撒谎"竞态。
  - backend swap（CPU↔GPU）时随其他显示态字段一起清 `last_uploaded_as_composite = false`，`show_composite_preview` 刻意保留（用户偏好，不由 backend 变更代做选择）。
- **纯谓词单源**（`src/gui/app.{hpp,cpp}`）：新增两条 pure predicate 与既有 `ShouldUploadPayload` 同一测试契约：
  - `ShouldUseCompositeUpload(payload_is_composite, show_composite_preview)` = 两者 AND；
  - `ShouldFireCompositeUpload(snap, last_serial, floor, show_pref, last_uploaded_as_composite)` = 既有 `ShouldUploadPayload(...)` **OR** mode_changed（`effective_composite != last_uploaded_as_composite`）。`SyncFromPoller` 换用该谓词作为唯一 fire-gate，测试与生产共享同一决策。
- **为什么需要 OR-branch**：`ShouldUploadPayload` 是 serial-dedup 门禁；仿真已完成、poller 自暂停（322/345.1 定性的常态）时，本地翻转 `show_composite_preview` **不会**产生新快照——如果只靠 serial-dedup，点击开关"看起来毫无反应"（与 345.1 ③/④ 同一类"显示态改动被消费门禁吞掉"）。mode_changed 分支专治此症。
- **AC4 零回归**：无颜色类 ⇒ `payload->is_composite = false` ⇒ `effective_composite = false` ⇒ `mode_changed = false` ⇒ 决策与 pre-345.4 逐位相等（布尔代数自然退化，无需额外分支）。
- **顶栏"切换 + 持久标记"合一控件**（`src/gui/app_panels.cpp::RenderTopBar`，task-colored-toggle-to-topbar / 346.3 起）：`!g_state.raypath_color.empty()` 时在顶栏 Colors 按钮右侧插入 `ImGui::Checkbox`，label `ICON_FA_PALETTE " Colored"/" Full Spectrum"` 与 `##CompositePreviewToggle` ID 后缀均沿用 345.4，绑定局部 `bool checked = last_uploaded_as_composite`（读 ground truth）、点击翻转 `show_composite_preview`（写 user preference）。345.4 首版控件位于状态栏 `SmallButton`，346.3 owner on-screen 反馈"Colors 弹窗按钮在顶栏、启用染色开关在底栏"的错位感后搬迁；同步把 345.4 的 accent style token 从 `ImGuiCol_Button`/`ImGuiCol_ButtonHovered` 换成 `ImGuiCol_FrameBg`/`ImGuiCol_FrameBgHovered`/`ImGuiCol_CheckMark`（Checkbox 实际渲染槽位）。**"标记语义"由结构性事实保证**：`##TopBar` 与 Colors 窗（`##ColorsWindow`）是两个独立渲染的窗口，关闭 Colors 窗不触及 TopBar，AC3 因此在移动后自动满足，无需额外的独立标记 UI 元素（issue.md 给出的两个选项中"随 checkbox 一起呈现"更符合 a05）。零颜色类时 checkbox 不渲染（AC4）。
- **测试**：
  - gui_test `gui_composite_preview.should_use_composite_upload_truth_table`——4 组合真值表，headless（AC1 决策正确性）。
  - gui_test `gui_composite_preview.mode_flip_forces_refire_at_same_serial`——用真实 kColorConfig snapshot 直调 `ShouldFireCompositeUpload`，钉住 T0 首帧 fire、T1 idempotent、T2 mode-off 翻转触发 fire、T3 稳态 idempotent、T4 mode-on 恢复 fire、T5 再稳态 idempotent（AC1 机制 + AC2 无损恢复的决策证据）。
  - gui_test `gui_composite_preview.mode_toggle_hidden_when_no_color_classes`——AC4 render-gate condition 断言 + fire-gate 与 `ShouldUploadPayload` 等价证明（mode_changed 结构性不可能触发）。
  - 端到端 `SyncFromPoller()` GL 上传路径**不**从此 functional 测试驱动（coroutine 无 GL context，`g_preview.UploadTexture` 会 SIGILL；实测过），实际 GL 与视觉体验交给 AC5 owner on-screen 与既有 visual 测试。
- **AC5**：owner on-screen 人工闸——切换手感 / 标记辨识度。若纯色高亮不够醒目，低成本备选是让图标随状态变（而非仅换色/文案），plan §risk 2 已记录。
- **未做**：Colors 窗内镜像同一开关（单一顶栏控件已同时满足 AC1+AC3，属 a05）；.lmc 序列化该开关（会话内状态，与 `color_window_open` 同类）；server 侧是否节流合成计算（issue 只要求显示态切换）。

**⭐as-built（`raypath_color` 堆指针化，task-raypath-color-dynamic-abi / task-344，BREAKING v4.8）**：342.2 以 `LUMICE_ColorClass raypath_color[LUMICE_MAX_CONFIG_COLOR_CLASSES]` 内联定长数组落地，每类 ~5.6 KB × 64 类 → `sizeof(LUMICE_Config)` 膨胀到 **467 KB**。全量 `gui_test`（不带 `--filter`）里 `filter_expand_struct_vs_json` 用例在同一函数体声明两个局部 `LUMICE_Config`（934 KB），在 ImGui 测试引擎线程 ~512 KB 栈上确定性 `EXC_BAD_ACCESS` in `___chkstk_darwin`（回归被 342.3 用 targeted `--filter` 掩过一次）。**344 的解**：
- **ABI 改**：`LUMICE_ColorClass raypath_color[64]` → `LUMICE_ColorClass* raypath_color;`（`raypath_color_count` / `_mode` 保留；`match[32]` 继续内联，owner 明确不缩容）。`sizeof(LUMICE_Config)` 从 467 KB 降至 **~113 KB**（`static_assert` 上限 160 KB，保 ~40% headroom）。C/C++ 侧的 `cfg.raypath_color[i]` 下标语法保持源码兼容——指针支持相同下标；只有 value-init/copy 语义变化（零初始化后 `raypath_color == NULL`，而非 64 个零初始化 `LUMICE_ColorClass`）。
- **两个新 C API + RAII guard**：`LUMICE_ConfigCreateColorClasses(cfg, count)` / `LUMICE_ConfigReleaseColorClasses(cfg)`（`calloc`/`free`，跨 C ABI 边界故刻意不用 `new[]`/`delete[]`；Create 幂等 create-or-replace；Release null-safe / 对已释放的 cfg 幂等）。C++ 侧 `src/include/lumice_config_scope.hpp::lumice::ConfigColorGuard` 是非拷贝/非移动 RAII scope guard，构造引用宿主 `LUMICE_Config`、析构调 `Release`，覆盖 `IM_CHECK`/`ASSERT_EQ` 早退路径（同一模式在 `learnings/code-quality/imgui-modal-test.md` 已记录）。GUI/tests 全数改为"声明 `LUMICE_Config` 后紧跟一行 `lumice::ConfigColorGuard`"。
- **隐式分配契约**（本任务白盒审计后补齐的语义）：`LUMICE_ParseConfigString` / `LUMICE_ParseConfigFile` / GUI 的 `FillLumiceConfig`（以及测试工厂 `MakeOneColorClassConfig`）**内部自动**调 `Create`（大小由被解析的 JSON `classes` 长度或 `GuiState.raypath_color.size()` 决定）——调用方无需预先知道 count，但**必须**在使用完 `out` 后 `Release`（推荐经 `ConfigColorGuard`）。这打破了"调用方总是主动 Create"的单一叙事，是"先解析才知道大小"这一约束下唯一可行的形状。
- **memset-before-Release 潜伏泄漏**（白盒新发现，非 342 遗留 bug）：`c_api.cpp::JsonToConfig` 和 `file_io.cpp::FillLumiceConfig` 开头都有 `std::memset(out, 0, sizeof(LUMICE_Config))`。若 `out` 已持有先前分配的 `raypath_color` 堆块，`memset` 会清指针字段而不释放，造成孤儿堆块。当前代码无调用点会喂同一 `LUMICE_Config` 两次而不重新声明（已逐一审计），故属**潜伏但尚未触发**的隐患。修复：两处 `memset` 前各插一行 `LUMICE_ConfigReleaseColorClasses(out)`（Release 对空指针是 no-op，首次调用路径无副作用）；code-review 阶段跑一次强制 valgrind 交叉验证并把结果写入 SUMMARY。
- **按值拷贝的结构性门禁**：`raypath_color` 指针化后，`LUMICE_Config` 从"数组内联、按值拷贝安全"变为"raw pointer 拥有堆内存、按值拷贝即别名"——任何拷贝构造、按值传参、赋值都会在两份副本各自 Release 时触发 double-free。落实 a09（软约束须固化为自动化门禁），`scripts/check_policies.py` 新增静态规则 `no-config-by-value-copy`，覆盖 5 类语法（copy-init `a = b`、direct-init `a(b)`、list-init `a{b}`、by-value 参数、以及两个同类型本地变量之间的后置赋值 `b = a`），扫描 `src/` + `test/`。已知局限：不覆盖 `std::vector<LUMICE_Config>` 等容器持有该 struct 的场景（代码库当前无此用法，留待后续任务）。
- **回归口径**：AC2 硬约束"全量 `-gtj` 不带 `--filter`"——342.3 的 targeted 跑法曾把这个栈溢出掩过一次，本任务的验收闸不再接受任何 `--filter` 快跑。

### 4.1 三层模型如何组合（门 / summand 色桶 / 跨层 rule）

> ⚠️ **SUPERSEDED（2026-07-08，见 §4.0）**：本节 Fork C（色桶 = filter 的 summand）已被 Design-2 重定向推翻——颜色类现为**独立于物理 filter 的 placement-scoped 谓词列表**。以下保留作历史推理（§1 三层角色、去重、按需分配等思想仍有效），但**数据模型以 §4.0 为准**。

**Fork C —— 色桶 = filter 的 summand（位），颜色类 = summand 位的布尔组合（2026-07-06 收敛）**：
- crystal A 的 `1-3;3-5` filter 在数据里**已经是** `(1-3)+(3-5)` 两个 summand；地基（`component_table.cpp`）**已按 summand 逐个发 component 位**。所以"给光路家族染色"的原子就是 summand 位，**颜色类 = 对这些位的一个布尔谓词**（单层 OR = "命中任一位"；跨层 AND = "这些位全命中"，见下"跨层 rule"）。
- **默认心智：UI filter = 一个颜色类**（owner 拍板，低心智负担）。用户在界面上给一颗晶体建的每个 filter 就对应一种颜色；想细分就展开成 per-summand 色，想合并就把多个 summand 位并成一个色桶。

**"同一晶体建多个 filter，各挂一色"怎么落地 —— 拍平进 core，边界记在 display 侧**：
- 用户在 crystal C 上建 F1=`1-3`(红)、F2=`3-5`(蓝)、F3=`1-3;3-5`(绿)。**进 core 前拼成一个 `ComplexFilterParam`**（summand 顺序拼接），物理门 = 这些 summand 的 OR。**core 仍是一颗晶体一个 filter，§1 的 1:1 绑定原封不动、零核改。**
- **染色配置存一张小表** `(layer, crystal) → [(UI filter, summand 块, 颜色)]`：把拍平后的 summand 块切回带色的 UI filter。这就是"UI filter = 色"这个心智的落点——core 那边拍平了没关系，边界记在 display 侧。
- F3 与 F1/F2 **重叠**：一条 `1-3` 光线同时落进 F1(红) 和 F3(绿) → 桶重叠 → z-order/painter 解（见 §4.3）。模型天然支持"色桶可重叠、并非划分"。
- **去重是硬需求，不是优化**（见 §4.6）：拍平时对结构相同的 summand 塌成同一个位（`config_compare.hpp` 已有 summand `operator==`），否则 component 位（uint64 也稀缺）很快用光、界面上建不了几个颜色。去重后 F3 = 位0∪位1 是**纯 display-time 重组、免重仿真**。

**放弃 Fork A（放宽 1:1、同 crystal 挂多个物理门）—— 2026-07-06 定论**：
- 上面的"多 filter 各挂一色"是 **display 侧的颜色分组**，不是核心的多门绑定。一个复杂 filter 已能表达**任意跨 type 的谓词并集**（含 raypath + entry-exit 混合，见"事实纠正"），所以为了染色**没有理由**去动 `ScatteringSetting{filter_, crystal_}` 的 1:1 绑定。
- 真正需要放宽 1:1 的只有 exotic 场景："同 crystal 不同色桶要不同 `action_`/`symmetry_`（不同物理门）"——那才是"多个独立物理门"，**YAGNI，现不做**。染色语义永远是 **OR（并集）**；真正的 AND 门（交集，"既是 A 又是 B 才放行"）属于复杂 filter 自身的能力，**别让"多绑定"这扇门去做 AND**（否则两个机制干同一件事）。

**⚠️ 事实纠正（2026-07-06 核对代码）**：core 的 `ComplexFilterParam`（`filter_config.hpp:40`，`vector<vector<pair<IdType, SimpleFilterParam>>>` = sum-of-products）**本就支持跨 type 的 OR 与 AND**——一个 summand 可 AND 混任意 `SimpleFilterParam`（raypath / entry-exit / direction / crystal），不同 summand 也可不同类型。backlog「Complex Filter 完整能力」讲的"跨 type 未支持"指的是 **GUI 编辑器**目前只能产出 OR-of-single-raypath，**不是 core 表达力**。故"同一晶体 raypath 类 + EE 类各一色"只欠 GUI 编辑器，core 与地基已就绪。

**跨层 rule —— 色桶 = 跨层 component 的布尔组合**：
- 用户几乎必然会要"整条跨层轨迹染色"：如"L1 crystal A 的 3-5 **且** L2 crystal C 的 1-3 → 蓝"。单层 summand（Fork C）粒度接不了这种合取。
- 解：把 summand 抬升为 **component**（带层键的 summand），跨层前向累积成 per-ray 掩码；**一个颜色 = component 的布尔组合（rule）**，通常是跨层 AND。
- 向下兼容：单层时 rule = 单 component，退化成 Fork C 的"给 summand 染色"。

### 4.2 地基（本设计的核心，必须一次焊对；回填极贵）

**地基只有一件事**：产出 + 跨层累积 + 交付「per-ray component 掩码」。

```
每层 gate (CollectData):
   从 §5.1 已算的 per-summand Complex 匹配取位  ──►  当前层 component 位
   OR 进 per-ray 累积掩码，跨 MS 层前向携带 (RayBuffer 并行数组，reset@InitRayFirstMs + fan-out + init_data 交接；见 §2.4 更正)
   │
   ▼ 任一层 emit：随出射 SimData 交给 consumer（类比 wl_idx）
consumer 收到 (pixel, Y, component 掩码)  ──►  地基到此为止；如何 binning 属上层（§4.3）
```

**三条硬承诺（回填极贵，一次焊对）**：
1. **携带的是原始 component 掩码，不是预先算好的 rule 状态**——保持 rule-无关，上层 rule 怎么演进都不改携带格式。
   - *为什么不能携带 rule 状态*：cross-layer rule 是 component 的 **AND（跨层合取）**，而**边缘和无法重建联合分布**——从"命中 A:3-5 的总 Y"和"命中 C:1-3 的总 Y"推不出"两者都命中的那批光线"。掩码是可行的、rule-无关的联合载体（"留全部光线"内存不可行，已否）。
2. **component 必须带「层」键** `(layer, crystal, summand)`，不是只 `(crystal, summand)`。最隐蔽的 must-have：前向 OR 会丢顺序，但只要**位本身带层身份**，"A:3-5 @L1 ∧ C:1-3 @L2"这种**有序跨层 rule** 就天然可表达。不带层键，以后想区分顺序就得重打 seam。
3. **宽度 uint64**（64 个 component，8 字节/ray，在本就几十字节的 SimData 上可忽略）。**只有被某颜色类/rule 引用到的、去重后的谓词原子才占位**（本意即按需分配，见 §4.6）——颜色类与 rule 是位集的免费布尔组合、不额外占位，故 64 对"不同谓词数"很宽裕；真超了 soft-cap 优雅降级。⚠️ **as-built 走样**：scrum-331 phase-1 的 `BuildComponentTable` 目前是**急切分配**（每 summand 都发位，不看有没有颜色引用），phase-3 须收敛回按需（§4.6）。

**产出点复用 §5.1**：Complex 匹配 = 逐 summand 匹配再 OR 塌缩成门布尔；component 掩码就是"塌缩前那批位"。**不是新增评估，是别把中间结果扔掉**。component 表跟现有 filter 表一起上传，无新上传机制。

**⚠️ 跨层携带的实现陷阱（T3/T4 血泪，通用规则）**：掩码作为**并行 per-ray 数组**跨层携带时，**所有重排/swap/gather 光线的点都必须一并搬运掩码**，否则静默去相关、单元测试测不到。CPU 侧 T3/T4 就抓出：续传去相关 shuffle 原本 `std::swap(buf[i],buf[j])` 只换 `rays_`、落下 `components_` → 掩码与光线跨层错位；修 = `RayBuffer::SwapRay` 同换。**GPU 同构风险**：Metal/CUDA 的 device 续传 shuffle（`cont_d`/`cont_w`/`cont_wl_idx`…）在 T5/T6 加 device 掩码时必须把掩码也加进 swap 列表，否则 device 上重现同 bug。规则：**新增任何"随光线跨层携带"的并行状态，先审计全部 ray-reorder 点**。

**隔离契约（本设计的定心丸）**：consumer 如何 binning、以及"改 rule 要不要重仿真"，**全是地基之上的 consumer 层选择**——因为不管上层怎么选，穿过 seam、交到 consumer 手里的都是同一个 component 掩码。**更换上层不动 seam。** 所以"轻量边界"这个 owner 最在意的问题**可以延后**（见 §4.3）。

### 4.3 地基之上（上层；⭐显示模型已由 spike 收敛，binning 分桶仍可延后）

> **⭐ 显示模型定案（2026-07-07 CPU spike 端到端实测，`three-arcs.json`：单 MS 层双晶体，晶体1 挂 raypath `3-5`、晶体2 挂 complex(`3` OR `3-5`)，两晶体都含 `3-5`）**。spike = 一次性 `ComponentBinConsumer` 旁挂在正常 consumer 上、复用生产 `ProjectExitToPixel`、按 component 位分 Y-lane 合成写图，**core 零改**。四条结论已 tracked，真开工按此走：
>
> 1. **地基正确（无 bug）**：per-ray mask histogram 证实三 component 全 **popcount=1、无跨 crystal 掩码污染**（无 `bit0|bit2` 之类）；且**承诺 #2（层键）在真实渲染被验证**——两晶体的 raypath `3-5` 空间上大幅重叠（实测 blue 的 86% 像素与 red 同址）却被正确分成两色。若只按 raypath 序列 keying（无 crystal 键）会把这两条不同物理弧灾难性塌成一色——层键正是防这个。
> 2. **⭐共享曝光/归一化 = 硬不变量（不是可选项）**：所有 component **必须**用同一个曝光（= 本节下方 EV anchor：Σ桶Y = 未分类图Y）。**反例（spike 首版真踩的坑，务必写进警示）**：对每个 lane 按其**自身** p99 归一化 → 把物理暗的 component 抬到亮的量级、制造**假洋红**。实测 cup 处 raw blue=12092 vs red=4560（blue 物理 2.65× red），per-lane 归一化把两者都压到 ~0.19 → 假洋红淹掉整条蓝弧；换共享曝光后 blue 正确主导、蓝弧复现。**实现即复用现有 `RenderConsumer` 的 EV 管线、component lane 挂其下；切勿自建归一化**（spike 的唯一错误就是自造了一套坏归一化，这是整段调试的根因）。
> 3. **合成方式 = display-time 用户可选，默认 dominant**（三者皆 display-time、同一批 lane、不重仿真、不动 seam）：
>    - **dominant（每像素 argmax component 取其色，默认）**：最易读、无假色；实测 three-arcs 下三弧各归其色、零假洋红。
>    - **additive（共享曝光之上）**：诚实叠加，仅 tie 处混色；可选。
>    - **painter / z-order（顶层盖住）**：需组件排序 UI，进阶可选。
> 4. **可见性（show/hide/solo）与合成方式正交、且更日常**：单独 lane 图始终是最诚实的检查视图。故 display 控制 = **两正交轴**（可见性 / 合成方式）。
> 5. **overlap / z-order 降级为小 tiebreaker**：曝光共享后重叠区多由物理强度自然定色；早先把 z-order 当载重决策是**高估**（源于上面归一化 bug 的误判，已纠正）。下方"z-order/桶重叠"条据此降级。
>
> spike 代码已 revert（价值提取完毕），证据图留在 `scratchpad/three-arcs-*.png`（components=per-lane 假洋红 / shared=共享曝光 / dominant=默认）。

- **consumer binning（延后定，二者同一地基）**：
  - **按 rule 分桶**（丢掉掩码）：改 rule 成员 → 重仿真；lane 数 = 用户 rule 数（少）。
  - **按实际出现的轨迹掩码值分桶**（留住掩码，稀疏）：改 rule → display-time 重组（免重仿真）；lane 数 = 实际出现的轨迹类（可能多，高分辨率显存吃紧）。
  - 混合 / top-K 掩码值 + "other" lane 可作折中。
- **re-sim 边界**：由上面 binning 决定，故延后。不变的是：**改颜色 = display-time；改"哪些光线进桶"（谓词/component 组合）= 至多重仿真**，与既有 filter dirty 语义一致。
- **z-order / 桶重叠**：列表序 = z-order，可拖排。**仅在 painter 式"顶层盖住"合成下有意义**；纯加性光叠加与序无关。per-ray 重叠（同 crystal 多 summand，或多 rule 命中同一光线）才是 z-order 真正需要定优先级的场景。
- **none-filter（无 filter 的 crystal = 绑了 NoneFilterParam，放行全部）**：✅ **对称处理**——给它一个颜色，当普通桶。心智 + 开发成本最小，且"显示 = 命中并集"下不匹配任何谓词的光线自然不显示（旧"未匹配光线怎么显示"问题消解）。方案(b)"none-filter 用自然光谱色"**不免费**（自然色需 XYZ，破坏"每桶标量 Y"的统一，要单开 XYZ lane）→ 以后做成"任意桶可选自然色"opt-in，现不做。
- **lane 成本**：每桶 W×H 标量 Y（非全 XYZ），但 N×W×H 与**分辨率耦合**，且在 GPU 上喂 `atomicAdd`——正是 backlog"高分辨率 atomicAdd L2 溢出税"。半分辨率桶 lane（÷4 显存）是可选优化。
- **EV anchor** = Σ 桶 Y = 未分类图的 Y → 开/关染色 EV 都稳。
- **GUI 合成**：poller `max_count=1` 放开读多条；preview 加多色合成 shader pass（今天无合成器）。

### 4.4 GUI 编辑器（上层，可延后；形态已勾方向）

两视图，同一 filter pool：
- **crystal card（不变）**：per-crystal 授权 + 物理门 + summand 谓词编辑。**改谓词 → 标 dirty → 重仿真。**
- **全局 filter 列表面板（新）**：一屏看全所有 filter + 各自绑定哪个 crystal（**绑定 = 门禁作用域的标识，不是染色 scope**）+ 各 summand 颜色 + z-order + rule editor。建议做成**非模态 side panel** 而非阻塞 modal——因为改色/拖序是 display-time 即时生效，side panel 才能边拖边看预览实时重着色。
- UX 原则：**视觉上区分 trace-time 编辑（谓词，要重跑）vs display-time 编辑（色/序/开关，即时）**，与既有 filter dirty 语义一致。

### 4.5 验证（phase-1 CPU 白盒，脱离 rule/UI）

地基正确性可**完全脱离 rule/UI** 验：CPU + 最简 consumer，出 per-component（或 per-掩码值）各一张图，逐条光线核对——多层 emit 处掩码累积对不对、层键对不对、跨层 AND 能否还原。把最大风险（**三后端跨层掩码一致性**）在 CPU 白盒钉死，再上 Metal/CUDA parity。

### 4.6 component 位预算（2026-07-06 深化；决定"用户能建多少颜色"）

**核心区分：位只花在"不同的谓词原子"上，颜色类是原子的免费布尔组合。** 所以被 64 卡住的是"**一共探测了多少个不同谓词**"，**不是颜色/rule 的数量**（后者无上限）。三条杠杆把预算撑住：

1. **去重（硬需求）**：拍平一颗晶体上的多个 UI filter 时，对**结构相同的 summand** 塌成同一个位（`config_compare.hpp` 已有 summand `operator==`），dedup 域 = 一个 `(layer, crystal)` 内。F1=`1-3`、F3=`1-3;3-5` 的 `1-3` 共享一个位；F3 = 位0∪位1 是免费组合。**不去重 → uint64 也很快用光、界面上建不了几个颜色**（owner 强调这不是优化是硬需求）。
2. **按需分配（更大的杠杆，也是蓝图 §4.2 承诺 #3 的本意）**：**只有被某颜色类/rule 引用到的谓词原子才占位**。纯物理门、用户不想染色的晶体 → 不占位，其光线进共享的"背景/other"桶。这样**场景复杂度与位预算解耦**：10 颗晶体只给 2 颗分色，就只有那 2 颗的谓词吃位。⚠️ **as-built 走样**：`component_table.cpp:BuildComponentTable` 目前**急切分配**（扫全场景每 summand 都发位，不看颜色引用），phase-3 须把分配改为**由颜色配置驱动**、收敛回按需。
3. **颜色 / 跨层 rule = 免费组合**：颜色类 = 位集布尔谓词，rule（含跨层 AND）= 已有原子位的组合，**都不吃新位**。

**与 re-sim 边界自洽**：引用**新谓词** → 原子表变 → 掩码变 → 重仿真；**复用已有原子的合并/重组/改色** → 表不变 → 免重仿真（正是 §4.3 的"改色 display-time / 改分类逻辑 re-sim"）。

**⚠️ filter 编辑器输入语法 ↔ 位粒度（scrum-334 收敛，2026-07-06 白盒查证）**：GUI filter 编辑器里 `1-3;3-5` 这类"单行/多行多 raypath"**天生就是 summand 级 OR = 多个位**——因为 core `RaypathFilterParam{vector<IdType>}`（`filter_config.hpp:16`）只装一条序列、无 alternatives 字段，`file_io.cpp:FactorAlternatives`+`ExpandSopToClauses` 把每个 `;`-分段 Cartesian 摊成独立 clause=独立 summand=独立位。**曾考虑让 `1-3;3-5` 塌成"一个 summand=一个位"（更省预算、语义"这团光路不可分"）→ rejected**：需改 `RaypathFilterParam` 为多-alternative + 匹配器 + `BuildComponentTable`（动已验 filter+染色地基），而它的收益"这团光路一个颜色"**§4.1（filter=色）+ 本节去重/并集（`位0∪位1` 免费）已无核改地给出**，且拆成两位**保留了以后分两色的可拆分性**。**故定论：`;` 保持 summand-OR（多位），"一个颜色"永远交给 filter 级/display-time 位集并集，不为省位去焊死不可分原子。**（窄例外：per-summand 染色 + 明确要不可分 + 位预算真紧，三条同时成立才重议，现均不成立。）

**超预算优雅降级**：地基已留 `ComponentTable::kNoBit` 软上限（超的记 `entries_` 但不进掩码、构造 warn 一次，不 throw）。phase-3 接到 UI：**超预算谓词 → 落共享 other 桶 + 界面提示**。

---

### 4.7 色类配置 schema + rule-lane binning（⭐2026-07-07 定案，`scrum-raypath-color-engine` 的规格）

> ⚠️ **SUPERSEDED（2026-07-08，见 §4.0）**：本节的 `{layer,crystal,filter,summand}` id-ref schema 已被 Design-2 重定向替换为 placement-scoped 独立谓词 `{layer,crystal,<谓词>}`。**consumer rule-lane binning（定案 1）、compositor、可见性、案例覆盖表等下游机制不变、仍有效**；仅"位来源 / schema / none-filter 整晶体位"随 §4.0 改。以下保留作 rule-lane / 案例覆盖的推理参考。

> **背景**：2026-07-06 `scrum-336`（raypath-color-display）落地了 CLI 侧 **per-bit lane** 显示(单 component + 不相交 OR-union，见 §4.3 spike 定案 + 台账 as-built)。2026-07-07 owner×AI session 把剩余架构一次性收敛,目标 = 把染色**引擎**在 CLI/core 层(CPU)做到蓝图规划的**全部**能力。本节是 `scrum-raypath-color-engine` 的权威规格,冷启动读这一节即可。

**定案 1 —— binning = rule-lanes（per-color-class lane），解掉 §4.3 悬案**：
- 机制:consumer 对每条出射光线,拿它的 per-ray component 掩码(带层键)**逐个 color class 求谓词**(`any` = 命中任一成员位 / `all` = 成员位全命中),满足就把该光线的 Y 累进**那个 class 的 lane**(共享曝光,复用 §4.3 的 `ExposureScale()`)。compositor 对 **class lane** 做 dominant/additive/painter + z-order + 可见性。
- **一个机制统一覆盖全部情况**:单 component(单成员 class)/ OR-union(含**相交**——一条光线可满足多个 class → z-order 解)/ 跨层 **AND**(class 谓词=成员位全命中,掩码带层键正好够)/ none-filter(见定案 3)。
- **超越 per-bit**:per-bit lane 是"单成员 class"的退化特例。本 scrum 把 336 的 per-bit(336.2 consumer / 336.3 compositor)**升级为 per-class**。
- **re-sim 边界**:改 class **成员**(谓词/引用哪些 component) → 重仿真(不保留全掩码流,§4.2 已否"留全部光线");改**颜色/序/可见** → display-time。lane 数 = class 数(少)。
- **为何 rule-lanes 而非掩码值-lanes**:掩码值分桶(按实际出现的轨迹掩码值)允许改 rule 免重仿真,但 lane 数 = 实际出现的轨迹类(无界、高分辨率显存吃紧),且要留掩码;rule-lanes lane 数可控、无需留掩码、re-sim 边界与既有 filter dirty 语义一致。故 rule-lanes 作默认(spike + 2026-07-07 分析)。

**定案 2 —— color-class 配置 schema（config-facing 契约，替换 336.1 的 flat `crystal_slot` 形态）**:
```json
"raypath_color": [
  { "color": [r,g,b], "combine": "any" | "all", "visible": true, "solo": false,
    "match": [ {"layer": L, "crystal": Cid, "filter": Fid, "summand": S}, ... ] }
]
```
- 每条 entry = 一个**色类** = {颜色 + 对 component 的布尔谓词 + visible/solo}。
- `match` = component 引用列表,两种 ref 形态:
  - `{layer, crystal, filter, summand}` —— 具体 summand(`summand` 单项 filter 默认 0 可省)。
  - `{layer, crystal}` —— **整晶体**(none-filter/all-pass,见定案 3)。
- ⭐`crystal`/`filter` 用 **id 引用**(与 `scene.scattering.entries[].crystal/filter` 同风格,一眼可读);解析:`(layer, crystal, filter)` → `scene.scattering[layer].entries[]` 里匹配的散射条目(ci) → component 位。**替换 336.1 误导性的 positional `crystal_slot`**——config 别处都用 id 引用,这里统一。(注:同层不可能有两个完全相同的 (crystal,filter) 配对,故唯一;退化重复 → builder 报错。)
- `combine`: `"any"`(OR,默认) / `"all"`(跨层 AND)。**flat,不是完整布尔**——蓝图具体案例只需 any/all,**刻意不做 NOT/嵌套**(a05)。⭐**可演进**:将来若真要,`match` 可长成嵌套表达式树(叶=ref,节点=and/or/not),flat 形是单层树的糖,**加性、不破坏已有 config**。
- `visible`/`solo`:display-time 可见性轴(与 combine 正交,336.3 已定)。
- **向后兼容**:缺 `raypath_color` → 不染色、mono 逐字节不变(纯加性)。

**定案 3 —— none-filter 对称配色（§4.3/§9 现落地）**:NoneFilterParam 晶体(放行全部、0 summand)获得一个**"整晶体"component 位**——地基:`BuildComponentTable` 给每个 none-filter setting 发 1 位;CPU emit gate 对穿过该晶体的**每条光线**置该位。`match` 用 `{layer, crystal}`(省 filter/summand)引用它。**本 scrum CPU-only**;GPU parity 归 GPU scrum。

**案例覆盖（2026-07-07 逐条压测,对照蓝图 §1/§4.1 每一种设计情况 → schema 具体写法)**:

| # | 蓝图情况(出处) | schema 写法 | 判定 |
|---|---|---|---|
| A | 单 summand → 一色(§1 色桶 / §4.1 per-summand) | `{color, match:[{layer,crystal,filter,summand}]}` | ✅ |
| B | UI filter=一色,该 filter 多 summand(`1-3;3-5`)→并集(§4.1"多 summand 并成一个色桶"/"F3=位0∪位1") | `{color, combine:"any", match:[s0,s1]}` | ✅ OR |
| C | 同晶体多 filter 各挂色 + **重叠**(§4.1/§17,F1红/F2蓝/F3绿共享 `1-3`) | 三条 entry 各自 `match`+`color`;`1-3` 光线同满足 F1/F3 → per-ray 多类 → **z-order/painter**(合成方式 + entry 列序) | ✅ |
| D | 跨层 **AND**("L1 A:3-5 ∧ L2 C:1-3→蓝",§4.1/§3) | `{color, combine:"all", match:[A@L1, C@L2]}` | ✅ AND（e2e 回归锚点：`test_raypath_color.py::TestRaypathColorMultiLayer::test_multi_layer_all_semantics`，task-339.5 落地，fixture 用**三 L0 entries 分别 gate 不同 filter**让 MAGENTA/RED/GREEN 的 ray-set 互斥——否则 subset-of-superset + dominant argmax 会吞掉 AND 类，见任务 progress.md v0→v7 迭代记录） |
| E | **none-filter 对称配色**(§4.3/§9,无 filter 晶体放行全部) | `{color, match:[{layer,crystal}]}`(整晶体 ref,定案 3) | ✅ |
| F | 未匹配光线=显示即命中并集(黑)(§8) | 无 entry 匹配 → 不显示 | ✅(无需 schema 特性) |
| G | 可见性 show/hide/solo | per-entry `visible`/`solo` | ✅ |
| H | 桶重叠 z-order=列表序 painter(§4.3/§10) | entry 列表序 = z-order；**painter 方向 = 列表首项 = 顶层**（`ascending scan → first ey>0`；dominant 严格 `>` + ascending → tie 也归列表首项，保持方向一致）。回归锚点：`test_component_compositor.cpp::PainterVsDominantDivergeWhenTopClassDimmer` + `OverlapDominantPicksBrighterAdditiveMixes`（task-339.4 落地时把这条方向定案从 plan 附注沉淀到本节，避免下次翻旧计划考古）。 | ✅ |
| — | 通用布尔 **NOT / 混合嵌套**("(A OR B) AND C"、"A AND NOT B") | ❌ **刻意延后**——蓝图无具体案例需要;`match` 可加性演进为嵌套表达式树(叶=ref,节点=and/or/not) | ⏸ 可演进 |

> **为何 `crystal_slot`(336.1)不用、改 id 引用**:owner 指出 config 别处都用 id 引用 item(`scattering.entries[].crystal:1` 引 crystal 段 id、`filter:4` 引 filter 段 id),而 `crystal_slot` 是散射条目的**位置下标**且名字误导。**fixture 恰好证伪 crystal-id 当键**:`raypath_color_three_arcs.json` 两个 entry 都是 `crystal:1`(同晶体、filter 1 和 4),真正区分它们的是**散射条目**(crystal+filter 配对)——故 ref 用 `(layer,crystal,filter)` id 三元组唯一定位条目。
>
> **为何 rule-lanes 是被这些案例逼出来的**:owner 举案例 D(跨层组合光路一色)时揭示——flat 的"一 summand 一色"写不出 AND,且 336 的 **per-bit lane pipeline 结构上只到并集为止**(per-bit lane 累积"带该位"的所有光线,分不出"两位同时命中");案例 C(相交)同理需按类而非按位合成。这两条把 binning 从 per-bit 逼到 **per-class rule-lane**(定案 1)。

**⭐范围划定（2026-07-07 owner 拍板)**:
- **`scrum-raypath-color-engine`（本节规格,本次立项)** = 引擎在 CLI/core 做完(CPU 全语义):color-class schema + none-filter 位 + rule-lane consumer + per-class compositor(3 模式+z-order+可见性)+ 跨层 AND + 相交 class + CLI 全回归。做完后染色语义 100% 齐活。在未 PR 的 `feat/color-components` 分支上直接演进 336 的 per-bit → per-class。
- **GUI 编辑器(§4.4) = 另立 `scrum-3b`**:非模态 filter 面板 + 颜色/模式/可见性/rule editor UI + live 多色 shader。最大一块,要 owner on-screen 逐个手感验收,建在完备引擎上(a03 引擎正确性先行)。
- **GPU device-side lane = 另立 `scrum-3c`**:opt-in 性能 handoff,三后端 parity。

## 5. 决策台账（owner 已同意；2026-07-05 深化 / 2026-07-06 phase-3 产品模型 / 2026-07-07 引擎定案 / 2026-07-08 Design-2 重定向）

> ⚠️ **2026-07-08 Design-2 重定向修订（见 §4.0）**：#2（Fork C 色桶=summand）改为**色 tag 独立于 filter 的 placement-scoped 谓词**；#7 binning 的 rule-lane 机制**保留**，仅位来源改；#17（拍平进 core 记边界）、#18（拍平去重）作废（无 filter 耦合即无"拍平"）；#19（按需分配）与新模型**收敛为同一实现**。新增 #21–#25 见表尾。

| # | 岔路 | 决策 | 备注 |
|---|---|---|---|
| 1 | 三角色是否一个物理机制 | ✅ **一次仿真 + 非破坏 tag**；绝不 route A | task-200 覆辙 |
| 2 | 色桶粒度 | ✅ **Fork C**：原子 = filter 的 summand（位）；**颜色类 = summand 位的布尔组合**，默认 **UI filter = 一个颜色类**（2026-07-06 收敛，见 §4.1） | 保 §1 的 1:1 绑定不动；"filter=门+桶"统一成立 |
| 3 | 跨层轨迹染色 | ✅ 抬升到 **rule** = component 布尔组合 | "L1 A:3-5 ∧ L2 C:1-3→蓝"场景；Fork C 单层不够 |
| 4 | 地基携带什么 | ✅ 原始 **component 掩码**（非 rule 状态）、**带层键**、**uint64** | 三点回填极贵，一次焊对（§4.2） |
| 5 | 掩码怎么跨层 | ✅ **全新** `RayBuffer` 并行数组，前向传播模式（reset@InitRayFirstMs + fan-out + `init_data` 交接） | ⚠️更正：`is_prior_filter_failed_` 已被 scrum-237 删除，非"加宽"而是"新增"（task-331.1 实测，见 §2.4） |
| 6 | component 产出点 | ✅ 复用 §5.1 per-summand Complex 匹配（别扔塌缩前的位） | 非新增评估 |
| 7 | binning + re-sim 边界 | ✅ **定案 rule-lanes**（per-color-class lane，2026-07-07,§4.7）：consumer 对每 ray 掩码逐 class 求谓词(any/all)累进 class lane;改 class 成员→重仿真,改色/序/可见→display-time | 否决掩码值-lanes(lane 无界+要留掩码);per-bit(336)= 单成员 class 特例,本 scrum 升级为 per-class |
| 8 | 未匹配光线（旧#2） | ✅ **消解**：显示=命中并集，每显示光线必有色 | none-filter 对称配色兜底 |
| 9 | none-filter | ✅ **对称配色**(a)；自然色(b) 需 XYZ lane 更贵，延后 | |
| 10 | 桶重叠 z-order（旧#3） | ✅ 列表序=z-order 可拖；仅 painter 合成下有意义 | 上层 UI，延后 |
| 11 | 谓词范围 | ✅ 复用 raypath / entry-exit（与 Design-A 同源） | |
| 12 | N / lane 上限 | ✅ component **uint64（64）**；预算花在**去重后按需分配的谓词原子**上，颜色/rule 是免费组合（§4.6）；lane 数由分辨率显存**软约束** | 旧"MVP 8"是 bitmask 字节产物，**废弃**；as-built 急切分配待 phase-3 收敛回按需 |
| 13 | 光谱 C API（功能1） | ✅ 扩 struct | 已由 task-323 落地 |
| 14 | 参考白 / 色适应 | ✅ 本轮不做，单独排 | 见 §2.1 |
| 15 | Fork A（放宽 1:1 绑定） | ✅ **不做**（YAGNI，2026-07-06 复核确认） | "多 filter 各挂一色"是 display 侧颜色分组、非核心多门；一个复杂 filter 已能表达任意跨 type 并集，无需动 1:1。仅 exotic"不同色桶要不同 action/symmetry"才需要 |
| 16 | 编辑器形态 | ⏸ 两视图（crystal card 不变 + 全局 filter 列表**非模态**面板）；延后细化 | §4.4 |
| 17 | 同 crystal 多颜色怎么落地 | ✅ **UI filter = 一个颜色类**：多 UI filter **拍平进 core** 成一个 `ComplexFilterParam`，**边界记在 display 侧**小表 `(layer,crystal)→[(UI filter, summand 块, 色)]` | core 零改、1:1 不动；重叠靠 z-order（§4.1） |
| 18 | 拍平去重 | ✅ **硬需求**（非优化）：同 `(layer,crystal)` 内结构相同 summand 塌一个位 | uint64 也稀缺；去重后合并类=免费 display-time 重组（§4.6） |
| 19 | 位按需分配 | ✅ **只有被颜色/rule 引用的原子占位**（§4.2 承诺 #3 本意）；as-built 急切分配待 phase-3 收敛 | 场景复杂度与位预算解耦；纯物理门不占位（§4.6） |
| 20 | 跨 type 组合能力 | ✅ **事实纠正**：core `ComplexFilterParam` **本就支持**跨 type OR/AND；缺口在 GUI 编辑器 | `filter_config.hpp:40`；backlog「Complex Filter 完整能力」指的是编辑器（§4.1） |
| 21 | 色 tag ↔ 物理 filter | ✅ **解耦**（Design 2，2026-07-08）：颜色类 = 独立 placement-scoped 谓词列表，gate 非破坏 pass 评估 | 推翻 #2 Fork C；回 §1 根 principle；闭合"染子集不改物理"缺口（§4.0） |
| 22 | 谓词 scope | ✅ **placement-scoped**（带 layer+crystal）；"到处染" = 多原子 combine:any + UI 展开 | 守承诺 #2；全局谓词会塌缩不同晶体的同一光路（§4.0） |
| 23 | GUI 形态 | ✅ **专用颜色窗**（谓词列表，复用 H5 编辑器）；**否决 Tier 1 内联色块** | Tier 1 有"其他 filter 怎么办"两难 + 高频操作仍需专用界面（§4.0） |
| 24 | preview 合成 v1 | ✅ host 侧 `GetCompositeResults` + 现有 sRGB 上传；多 lane shader 延后 | as-built 白捡（§4.0） |
| 25 | API 按 re-sim 边界拆 | ✅ 成员/谓词→config commit（dirty）；色/模式/可见/z-order→独立 display-time setter | 契约焊在 seam（a09，§4.0） |

---

## 6. 推进节奏（phase-1/2 地基 ✅ 已由 scrum-331 落地；phase-3 上层待立项）

1. **功能1** —— 已完成（task-323）。
2. **功能2 分三阶段，先解耦**：
   - **phase-1 地基（core + CPU）—— ✅ 已完成**（scrum-331，2026-07-06 合 main PR #175）：component 掩码产出（复用 §5.1）+ 跨层前向累积（全新 `RayBuffer` 并行数组，见 §2.4 更正）+ 交付 consumer；CPU 白盒（§4.5）钉死三后端跨层掩码一致性语义。
   - **phase-2 GPU parity —— ✅ 已完成**（scrum-331 T5/T6）：Metal（`70ea738c`）+ CUDA（`2ed31715`，dev49 亲验）gate 产同一 component 掩码，parity 锚定；device 续传 shuffle 的掩码携带陷阱（§4.2）已在 gather kernel + host swap 两处处理。
   - **phase-3 上层（待做，可另立 scrum）**，三条相对独立的线程：
     - **(a) 颜色分组层 + binning**（display 侧，core 不动）：定 consumer binning（§4.3，rule-lanes 倾向）+ 位**按需分配**（§4.6，收敛掉 `BuildComponentTable` 的急切分配）+ 去重（§4.6）+ none-filter 对称色 + 超预算降级。
     - **(b) GUI 编辑器**（§4.4）：**复杂 filter 编辑器**（补 backlog #178 缺口，让用户能编跨 type / 多 summand filter——core 已就绪）+ "UI filter = 色"分组编辑（§4.1，含拍平记边界）+ 全局 filter 列表**非模态**面板 + rule editor + display 多色合成 shader。
     - **(c) GPU handoff 解耦**：T5/T6 把"产位"与 test-only host capture ring 共用了 `capture_component` 一个 flag；须解耦——产位 → device 侧 component 累加（per-component atomicAdd lanes），不走 host readback ring。
   - **顺序理由**：地基的跨层 seam 最难回填，已钉死；上层三线程换之不动 seam。(a) 是最小可视化闭环的关键路径，(b) 的编辑器可增量铺，(c) 是 GPU 转正前提。

---

## 7. 顺带要修的债

- `doc/accumulator-consumer-architecture.md` 的 anchor-lane 章节（§6.1/§7.2 及 ON/OFF anchor 引用）是 **stale**——该 lane 早被 scrum-237 删除，`render.hpp` 无 anchor 成员。`raypath-rayseg-architecture.md` 里"filter 失败 route 到 anchor lane"的措辞属同一片 stale 债（`is_prior_filter_failed_` 的下游路由目标）。碰这块时顺手修，免误导后来人。

---

## 8. 关联

- **代码事实**：filter↔crystal 1:1（`proj_config.hpp`、`filter-architecture.md §1`）；多值 OR 已是 `ComplexFilterParam`（`file_io.cpp::SerializeFilterForCore`）；跨层链 `prev_ray_idx_`/`root_ray_idx_` + `rp_` per-crystal（`core/raypath.hpp`、`simulator.cpp`）。⚠️ 早先说的"唯一跨层 per-ray bool `is_prior_filter_failed_`"**已过时**——该字段被 scrum-237 删除（task-331.1 核实，见 §2.4）；跨层持久 per-ray 状态现由 task-331.1 新建的 component 掩码承担。
- **doc**：`filter-architecture.md`（§1 绑定不变量、§5.1 device filter-match、§多值 OR routing、§6 task-200 revert 史）、`raypath-rayseg-architecture.md`（跨层链、`is_prior_filter_failed_`）、`ev-pipeline-architecture.md`（单 lane、EV anchor）、`accumulator-consumer-architecture.md`（consumer buffer 模型 + §7 stale 债）。
- **相关 memory**：`project_design_gui_spectrum_filter_color`（本设计索引，需据本次深化更新）；filter 物理门 vs 渲染分类的区分即本设计核心。
