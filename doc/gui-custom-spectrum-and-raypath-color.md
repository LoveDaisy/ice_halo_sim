# 设计：GUI 自定义光谱 + per-raypath 颜色标记

状态：
- **功能 1（自定义离散光谱）已由 task-323 落地**（2026-07-03，见 `scratchpad/task-gui-custom-spectrum/`；同批完成 `ray_num` 语义统一为总数）。
- **功能 2（per-raypath 颜色标记）仍在 backlog，未立项**，但设计已深化到"地基可开工"。
- **2026-07-05 设计深化 session（owner × AI）**：把功能 2 从最初"独立全局 classifier + 单层 bitmask"深化为 **跨层 component 掩码地基 + summand 级色桶（Fork C）+ 跨层 rule 层**。本文档据此重写 §1 / §4 / §5 / §6。核心结论：**地基窄而稳**——只需"产出 + 跨层累积 + 交付 per-ray component 掩码"三件事；binning 模型、rule 层、UI 全属地基之上，可延后，且**更换它们不需要重打三后端 seam**。

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

### 4.1 三层模型如何组合（门 / summand 色桶 / 跨层 rule）

**Fork C —— 色桶 = 单个 filter 的 summand**（不动 §1 的 1:1 绑定）：
- crystal A 的 `1-3;3-5` filter 在数据里**已经是** `(1-3)+(3-5)` 两个 summand。给**每个 summand 挂一个颜色** → 1-3 红、3-5 蓝。
- 满足"区分同一 crystal 内 OR 光路"而**不需要**"同 crystal 挂多 filter"（那才要放宽 §1）。owner"filter = 门 + 桶 同一对象"的统一模型原封成立。
- 单 filter 内 summand 相交 → 一条光线中多个 → 桶重叠 → z-order 按 summand 序（见 §4.3）。
- **被否 Fork A**（放宽 1:1、同 crystal 挂多个 filter）：多个门只会 AND/OR 成一个 admission 集合，第二个门对"放不放进来"无独立意义，其真正的活儿只有染色 = 本质是 classifier 不是 gate。仅在"同 crystal 不同色桶要不同 `action_`/`symmetry_`"这种 exotic 需求下才需要 Fork A，**YAGNI，现不做**。

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
3. **宽度 uint64**（64 个 component，8 字节/ray，在本就几十字节的 SimData 上可忽略）。**只有被 rule 引用的 summand 才占位**，故 64 很宽裕；真超了再 soft-cap。

**产出点复用 §5.1**：Complex 匹配 = 逐 summand 匹配再 OR 塌缩成门布尔；component 掩码就是"塌缩前那批位"。**不是新增评估，是别把中间结果扔掉**。component 表跟现有 filter 表一起上传，无新上传机制。

**隔离契约（本设计的定心丸）**：consumer 如何 binning、以及"改 rule 要不要重仿真"，**全是地基之上的 consumer 层选择**——因为不管上层怎么选，穿过 seam、交到 consumer 手里的都是同一个 component 掩码。**更换上层不动 seam。** 所以"轻量边界"这个 owner 最在意的问题**可以延后**（见 §4.3）。

### 4.3 地基之上（上层，可延后；换之不动 seam）

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

---

## 5. 决策台账（owner 已同意；2026-07-05 深化）

| # | 岔路 | 决策 | 备注 |
|---|---|---|---|
| 1 | 三角色是否一个物理机制 | ✅ **一次仿真 + 非破坏 tag**；绝不 route A | task-200 覆辙 |
| 2 | 色桶粒度 | ✅ **Fork C**：filter 的 summand（sum-of-products OR 项） | 保 §1 的 1:1 绑定不动；"filter=门+桶"统一成立 |
| 3 | 跨层轨迹染色 | ✅ 抬升到 **rule** = component 布尔组合 | "L1 A:3-5 ∧ L2 C:1-3→蓝"场景；Fork C 单层不够 |
| 4 | 地基携带什么 | ✅ 原始 **component 掩码**（非 rule 状态）、**带层键**、**uint64** | 三点回填极贵，一次焊对（§4.2） |
| 5 | 掩码怎么跨层 | ✅ **全新** `RayBuffer` 并行数组，前向传播模式（reset@InitRayFirstMs + fan-out + `init_data` 交接） | ⚠️更正：`is_prior_filter_failed_` 已被 scrum-237 删除，非"加宽"而是"新增"（task-331.1 实测，见 §2.4） |
| 6 | component 产出点 | ✅ 复用 §5.1 per-summand Complex 匹配（别扔塌缩前的位） | 非新增评估 |
| 7 | binning + re-sim 边界 | ⏸ **上层决策，延后**；换之不动 seam | rule-lanes vs 掩码值-lanes（§4.3） |
| 8 | 未匹配光线（旧#2） | ✅ **消解**：显示=命中并集，每显示光线必有色 | none-filter 对称配色兜底 |
| 9 | none-filter | ✅ **对称配色**(a)；自然色(b) 需 XYZ lane 更贵，延后 | |
| 10 | 桶重叠 z-order（旧#3） | ✅ 列表序=z-order 可拖；仅 painter 合成下有意义 | 上层 UI，延后 |
| 11 | 谓词范围 | ✅ 复用 raypath / entry-exit（与 Design-A 同源） | |
| 12 | N / lane 上限 | ✅ component **uint64（64）**；lane 数由分辨率显存**软约束** | 旧"MVP 8"是 bitmask 字节产物，**废弃** |
| 13 | 光谱 C API（功能1） | ✅ 扩 struct | 已由 task-323 落地 |
| 14 | 参考白 / 色适应 | ✅ 本轮不做，单独排 | 见 §2.1 |
| 15 | Fork A（放宽 1:1 绑定） | ✅ **不做**（YAGNI） | 仅 exotic"同 crystal 不同色桶要不同 action/symmetry"才需要 |
| 16 | 编辑器形态 | ⏸ 两视图（crystal card 不变 + 全局 filter 列表**非模态**面板）；延后细化 | §4.4 |

---

## 6. 推进节奏（建议，未立项）

1. **功能1** —— 已完成（task-323）。
2. **功能2 分三阶段，先解耦**：
   - **phase-1 地基（core + CPU）**：component 掩码产出（复用 §5.1）+ 跨层前向累积（全新 `RayBuffer` 并行数组，见 §2.4 更正）+ 交付 consumer。**T1 已完成**（task-331.1，commit `fb24c9e5`：传输管道全通、值恒 0、17 单测绿）。**CPU 白盒**（§4.5）钉死三后端前需验的**跨层掩码一致性语义**——per-component/per-掩码值 出图逐光线核对。这是最大正确性风险，最省成本地先钉死。
   - **phase-2 GPU parity**：Metal/CUDA gate 产同一 component 掩码，parity 锚定（对解析/CPU 目标，非旧采样器）。
   - **phase-3 上层**：定 consumer binning（§4.3）+ rule editor + 全局 filter 列表面板（§4.4）+ display 多色合成 shader + none-filter 对称色。
   - **顺序理由**：地基的跨层 seam 最难回填，先钉；上层（binning/rule/UI）换之不动 seam，后铺。UI/UX 里不碰地基的部分随 phase-3 再细化。

---

## 7. 顺带要修的债

- `doc/accumulator-consumer-architecture.md` 的 anchor-lane 章节（§6.1/§7.2 及 ON/OFF anchor 引用）是 **stale**——该 lane 早被 scrum-237 删除，`render.hpp` 无 anchor 成员。`raypath-rayseg-architecture.md` 里"filter 失败 route 到 anchor lane"的措辞属同一片 stale 债（`is_prior_filter_failed_` 的下游路由目标）。碰这块时顺手修，免误导后来人。

---

## 8. 关联

- **代码事实**：filter↔crystal 1:1（`proj_config.hpp`、`filter-architecture.md §1`）；多值 OR 已是 `ComplexFilterParam`（`file_io.cpp::SerializeFilterForCore`）；跨层链 `prev_ray_idx_`/`root_ray_idx_` + `rp_` per-crystal（`core/raypath.hpp`、`simulator.cpp`）。⚠️ 早先说的"唯一跨层 per-ray bool `is_prior_filter_failed_`"**已过时**——该字段被 scrum-237 删除（task-331.1 核实，见 §2.4）；跨层持久 per-ray 状态现由 task-331.1 新建的 component 掩码承担。
- **doc**：`filter-architecture.md`（§1 绑定不变量、§5.1 device filter-match、§多值 OR routing、§6 task-200 revert 史）、`raypath-rayseg-architecture.md`（跨层链、`is_prior_filter_failed_`）、`ev-pipeline-architecture.md`（单 lane、EV anchor）、`accumulator-consumer-architecture.md`（consumer buffer 模型 + §7 stale 债）。
- **相关 memory**：`project_design_gui_spectrum_filter_color`（本设计索引，需据本次深化更新）；filter 物理门 vs 渲染分类的区分即本设计核心。
