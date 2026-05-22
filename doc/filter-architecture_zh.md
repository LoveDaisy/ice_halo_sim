[English version](filter-architecture.md)

# Filter 子系统架构

本文档记录 Lumice filter 子系统的核心设计约束。
本文档是**规格说明**（spec）：所有实现必须与这里陈述的不变式对齐。
若实现细节与本文某条断言冲突，先修 doc（若 doc 有误）或修 code（若 code 偏移），两者不得静默分歧。

**目标读者**：负责实现涉及 filter 路由、晶体配置、多重散射传播或 C API 功能的贡献者，以及对照 Design A 做 code-review 的评审者。

---

## §1 绑定模型：Filter ↔ 晶体（单键绑定）

Filter 绑定到**晶体**（crystal），而非 layer、stage 或 scattering entry。
映射关系为一对一：

```
crystal_id  ──────────────  filter_id   （单键对）
```

### JSON / 配置表示

JSON 配置中的 `scattering[].entries[].filter` 是**序列化细节**。
语义上等同于"该 entry 所引用晶体的 filter"。
同一 `crystal_id` 在所有引用它的 scattering entries 中预期持有相同的 filter——
GUI 在 UI 层强制此不变式（见 §5）；JSON 加载器可在解析时断言。

### GUI 链接条目（Linked Entries）

当且仅当两个 entry 同时共享 `crystal_id` 和 `filter_id`，它们才是**链接的**。
GUI 在 ≥ 2 个 entry 构成此配对时显示 link 徽章。
链接组是**原子共享单元**：对任意成员的编辑必须在所有成员上可观察到。

编辑传播规则——自动传播 vs 需显式传播：

1. **晶体内容编辑**（在共享 pool slot 原地覆写）：自动传播。
   所有共享该 `crystal_id` 的 entry 通过 pool 间接引用自动感知更新。
2. **Filter 内容编辑**（在共享 pool slot 原地覆写）：自动传播。
   通过共享 `filter_id` 的 pool 机制，同 §1.1。
3. **Filter 添加**（`entry.filter_id: None → Some N`）：需显式传播。
   新 pool slot 默认只绑定到发起编辑的 entry；此前与该 entry 同处 `(cid, None)` 的链接兄弟条目也需将 `filter_id` 翻转为 N，否则链接组解体。
4. **Filter 移除**（`entry.filter_id: Some → None`）：需显式传播。
   链接兄弟条目也需清空 `filter_id`。

参见 `src/gui/gui_state.hpp:294-328` 的权威注释块和传播所有者（`ApplyBuffersToEntry` 中的 `propagate_filter_id_to_linked` lambda）。

### "相同参数晶体，不同 filter"

若需要两个几何参数相同但 filter 不同的晶体，应创建**两个独立的 `crystal_id` 实例**。
GUI 提供 link/unlink 操作：
"Link to…" 原子地采用目标的 `(crystal_id, filter_id)` 对；"Unlink" 拆分出独立克隆。

---

## §2 Filter 在 Simulation 中的角色（Design A）

Filter **参与 simulation 传播**。它们在 simulator 内部每个 ray-exit-crystal 决策点充当门控。

### 决策规则（每条 ray，每次 crystal 退出）

```
ray 退出晶体
    │
    ├─ filter 检查通过  ──►  ray 进入 outgoing / MS 传播
    │
    └─ filter 检查失败  ──►  ray 被 DROP（丢弃）
                             （既不作为 outgoing 发射，
                              也不传递到下一 MS 层）
```

`FilterSpec::Check(ray)` 在每个 outgoing-ray 候选点被调用一次，绑定到刚发射该 ray 的晶体的 filter。
失败的 ray 不会出现在任何下游 buffer 中。

### 设计理由

将门控置于 simulator 端（而非 consumer / render 端）有两点好处：

1. **正确性**：filter 选择性在多重散射链的每一层都生效，
   物理正确的 ray 子集贯穿完整的晶体序列传播。
2. **性能**：高选择性 filter 在早期剪枝 ray 群体；剪枝效果在各层上叠乘（见 §3）。

---

## §3 多重散射（Multi-Scatter）语义

在多重散射（MS）配置中，每一层是一个 `(prob, crystal, filter)` 三元组。

```
第 0 层：prob₀ × crystal₀ × filter₀
               │
         仅 filter₀ 通过的 ray
               │
第 1 层：prob₁ × crystal₁ × filter₁
               │
         仅 filter₁ 通过的 ray
               │
         ……
```

Filter 在 simulator 内部的**每个 crystal 退出点**进行评估。
第 N 层的 filter 决定哪些 ray 有资格进入第 N+1 层。
在任意层 filter 失败的 ray 被丢弃，不再传播，也不出现在输出 buffer 中。

Filter 选择性越高 → simulation 越快：幸存 ray 比例以 `∏ᵢ (pass_rateᵢ × probᵢ)` 的方式跨层叠乘剪枝。

---

## §4 Render 阶段：无 Filter 逻辑

Render / consumer 阶段**不执行任何 filter 操作**。

`RenderConsumer` 接收的所有 ray 在 simulator 内部已完成过滤。
Render 通道是纯投影和 XYZ 累积：

```
simulator 输出  →  RenderConsumer  →  投影  →  XYZ 累积
                                      （无 filter）
```

`RenderConfig` 不持有 filter 字段。先前存在的 `ms_filter_` 字段将在 task-revert（Design A 目标，219.4 删除）中移除。
在 consumer 层应用 filter 是设计违规。

---

## §5 数据流不变式

### 配置表示

Filter 数据存储于 `SceneConfig.ms_.setting_[].filter_`（`ScatteringSetting.filter_`，
per-entry，位于 `src/config/proj_config.hpp`）。

**底层假设**：同一 `crystal_id` 在所有引用它的 scattering entries 中持有相同 filter。
GUI `linked-crystal` 不变式在 UI 层强制此约束。
JSON 加载器可在解析时断言。

### Filter 检查入口

Simulator 中每个 outgoing-ray 候选均调用一次 `FilterSpec::Check(ray)`。
`FilterSpec::Create(config, crystal, axis_dist)` 根据绑定到该晶体的 filter config 构建 spec。

调用点位于 `src/core/simulator.cpp` 中的 `CollectData()` 函数——决策规则见 §2。

### RenderConfig 不变式

`RenderConfig` 不得持有 filter 字段。Filter 是纯 simulation 端关注点。

---

## §6 历史脉络

### task-200（query-filter-uplift-v2，2026-05-18）——路由决策：已回退

**目标**：将 filter 从 simulator 端迁移到 consumer 端，使 filter 实时编辑无需重跑 simulation。

**实现方式**：`config_manager.cpp:194-225`（已在 task-revert-filter-to-simulator-side / 219.4 中删除）
新增了 post-parse 自动绑定，将 `scattering.entries[].filter` 复制到 `renderer.ms_filter_`，
激活了 `render.cpp` 中此前休眠的 `FilterRay` 路径。
在 simulator 中，filter 被降级为纯 MS-branch gate（filter 失败的 ray 仍作为 outgoing 发射）。

**发现的问题**（scrum-prob-zero-leak / #219，2026-05-22）：

1. 预期收益——"filter 编辑跳过 simulation"——在 GUI 中从未实现。
   修改 filter 仍会触发 `MarkFilterDirty → 全量 sim 重跑`。
2. task-200 代码路径发现三个实现 bug：
   - Post-parse 平铺操作在多层 MS 配置中丢失了 per-entry filter 绑定。
   - 晶体上下文在配置解析时被冻结，导致 live 晶体更新无法到达 consumer 端 filter。
   - 配置绑定器中的 chain-walk 边界条件在某些 MS 配置下导致索引越界。
3. Consumer 端过滤效率更低：filter 选择性不再在 simulation 期间剪枝 ray 群体，
   失去了多层叠乘剪枝的性能收益（§3）。

**决策**：回退到 Design A（task-revert-filter-to-simulator-side，219.4）。

### scrum-210（filter-architecture-refactor，2026-05-19）——保留

scrum-210 引入了 `FilterSpec` 多态接口以及基于 canonical-form / orbit 的 raypath 匹配方案。
这是**纯算法层改进**，与路由决策正交。
task-revert（219.4）迁移 filter 调用点（consumer → simulator），但不触动 `FilterSpec` 算法接口。

保留组件：`src/core/filter_spec.{hpp,cpp}`、simulator 中的 per-batch `specs_table` 调用、scrum-210 测试套件。

### scrum-prob-zero-leak / #219（2026-05-22）——权威基线

本 scrum 审计了 task-200 的各 bug，评估了回退方案，将 Design A 确定为权威路由模型。
`unfiltered_*` C API 字段以降级语义保留（§7）；自适应亮度 Off 模式暂时禁用（§7）。
后续工作项（Design A 基线上重设计自适应亮度 Off 模式、可加性测试、unfiltered baseline）已追踪到项目 backlog。

---

## §7 C API 字段 DEPRECATION 与 GUI Off 模式临时禁用

### `LUMICE_RawXyzResult`——已废弃字段

以下字段由 task-200 引入，在 task-revert 后**保留以兼容 ABI**，但语义降级：

| 字段 | task-200 语义 | task-revert 后语义 |
|------|--------------|-------------------|
| `unfiltered_xyz_buffer` | consumer 端 filter 应用前的全部 outgoing ray | **等同** `xyz_buffer`（filter 回归 simulator 端后，filtered = unfiltered） |
| `unfiltered_snapshot_intensity` | unfiltered ray 的 per-pixel 强度 | **等同** `snapshot_intensity` |

**状态**：`DEPRECATED`。调用方应使用 `xyz_buffer` 和 `snapshot_intensity`。
不得依赖 `unfiltered_*` 与 `filtered_*` 存在差异的语义。

"Unfiltered baseline"和"可加性测试"在 Design A 基线上的真正重设计，已作为 backlog 条目跟踪：
*"Adaptive Brightness Off mode + 可加性测试：在 Design A 基线上重设计"*。

参见 `src/include/lumice.h` 中这些字段上的 DEPRECATED 注释。

### GUI 自适应亮度 Off 模式——暂时禁用

task-revert 后，GUI **强制禁用**自适应亮度 Off 模式开关
（开关置灰，tooltip 说明原因）。

**原因**：Design A 中 `unfiltered_xyz_buffer` 等同 `xyz_buffer`，
Off 模式与 On 模式产生相同的 EV 锚点——两者没有实质差异。
更重要的是，若场景配置了高选择性 filter，Off 模式的 EV 锚点将从与 On 模式相同的过滤集合中计算，
失去了 Off 模式设计时旨在防止的 ~10 stops 亮度跳跃的保护作用。
在 Design A 基线上完成 Off 模式重设计之前，禁用优于暴露静默错误的 UX。

参见 `doc/adaptive-brightness.md` §5b 与 §6 了解代码路径引用及 pre-revert 行为变更说明。

---

## §8 不变式自动化（未来工作——占位）

以下自动化检查尚未实现，此处作为未来贡献者的锚点：

- **Simulator 输出断言**：单元测试验证 simulator 的 outgoing buffer 中无任何 ray 未通过所配置的 filter。
  防止 task-200 consumer 端 filter 路由的回归引入。
- **Linked-crystal 不变式断言**：JSON 加载器断言，确保所有在多个 `scattering.entries` 中出现的 `crystal_id`
  引用相同的 `filter` 配置。

---

## 交叉引用

| 关注点 | 位置 |
|--------|------|
| 绑定模型、GUI linked-group 不变式 | `src/gui/gui_state.hpp:294-328` |
| Filter ↔ 晶体 per-entry 配置 | `src/config/proj_config.hpp` — `ScatteringSetting` |
| Simulator 端 filter 检查（Design A 门控） | `src/core/simulator.cpp` — `CollectData()` |
| FilterSpec 算法接口 | `src/core/filter_spec.hpp` |
| C API `unfiltered_*` 已废弃字段 | `src/include/lumice.h` — `LUMICE_RawXyzResult` |
| Filter JSON schema | `doc/configuration.md` |
| Raypath 语义、P/B/D filter 开关 | `doc/raypath-symmetry.md` |
| 自适应亮度 Off 模式、可加性 | `doc/adaptive-brightness.md` |
