[English](testing-architecture.md)

# 测试架构

本文档是 Lumice 测试如何组织、以及如何新增一个测试的**权威源**。它是一份**规范**：当你写一个
测试时，本文告诉你它属于哪一层、放在哪里、必须遵循什么 oracle 与阈值纪律、带什么 marker/label。
若某个实现细节与本文冲突，先修文档（当文档错时）或修正测试布局（当它漂移时），但**绝不允许**默默背离。

**目标读者**：新增或迁移测试的贡献者；核查新测试是否落在正确层、用对 oracle 的评审者；任何重排
测试套件的人。

> **状态声明（先读）。** 本文定义的是**目标态**架构（purpose 主轴，层 × subsystem）。撰写时，磁盘
> 上的测试树仍是旧扁平形态（`unit_test` / `integration_test` gtest target、`test/e2e/` pytest、
> `LumiceGUITests`）。物理迁移到 `test/<layer>/<subsystem>/` 在 milestone-cleanup 的 270.3–270.7
> 子任务完成。§6 同时给出**现状**与**目标态**两栏，使你今天能定位一个测试、并知道它将去往何处。
> 迁移落地前：新测试按*层*（本文）归类，但暂放进其 harness 对应的现有物理位置。

---

## §0 为何 purpose 是主轴

历史上套件按**机制 / harness**组织：扁平的 `unit_test` gtest 二进制（约 30 文件）、一个
`integration_test` 二进制、一棵 `test/e2e/` pytest 树、以及 imgui-engine 的 `LumiceGUITests`
二进制。每个桶内文件平铺。

人们真正用来思考的类别——"单元""性能""正确性""GUI""scrum-268 验收门"——其实是**验证目的
（purpose）**，不是机制。purpose 与 mechanism 正交，于是同一 purpose 被切碎到多处：Metal 正确性
散在约 4 处，性能散在约 5 处，G1–G4 验收门散在约 4 处。"我该怎么加一个新测试？"没有答案，于是新
测试落在任何方便的地方——熵增。

purpose 必须为主轴的更深层原因：**不同 purpose 遵循不同规则。** 每个 purpose 有自己的 oracle、
自己的阈值纪律、自己的运行节奏、自己的 false-green 风险。机制桶**不编码**这些规则。一个验收门
（带预登记硬阈值与反漂移纪律）和一句随手的单元断言都是"一个 gtest `TEST()`"，但它们不是同*种*
测试，绝不能用同一套规则维护。

**决定**：主轴 = 验证目的（七层，§1）。二维 = **subsystem**（层内 tag，§2）。harness/语言是每层
的实现细节，永不作主轴。

一层是一个**规则同质单元**：层内每个测试共享同一族 oracle、阈值纪律与节奏。这是判定层归属的判据
——见 §1 中 `test_gui_perf` 的案例（performance）与横切规则 §4.4。

---

## §1 七层

每层由六个字段定义：**目的 / oracle / 阈值约定 / 运行节奏 / 命名约定 / 物理位置**。节奏取值：
`CI-fast`（每次 push 的快速腿）、`PR`（拉取请求）、`nightly`（定时 / 本地 heavy）、`local`（开发者本地跑）。

### §1.1 `unit-correctness`

- **目的**：孤立组件正确且自洽（数学、几何、optics kernel、解析、config 快照、RNG、队列、线程原语）。
- **oracle**：测试内手算的期望值、不变量、round-trip 恒等式。不依赖跨后端或整管线。
- **阈值约定**：精确或紧容差（`EXPECT_EQ` / `EXPECT_NEAR` 小 epsilon）。无统计阈值。
- **节奏**：`CI-fast`——每 commit。
- **命名**：`test_<component>.{cpp}`；gtest `TEST(<Component>, <behavior>)`。
- **物理位置**：目标态 `test/unit-correctness/<subsystem>/`；现状 `unit_test` target。

### §1.2 `golden-analytic`

- **目的**：管线（或其某阶段）复现**闭式物理真值**——独立于模拟器导出的解析值（如投影公式、
  有已知解析答案的法向入射续传结果）。
- **oracle**：闭式 / 解析导出的值——*绝对*真值，而非另一条代码路径。这是它区别于
  `parity-cross-backend`（oracle 是另一后端）与 `unit-correctness`（范围是单一组件）之处。
- **阈值约定**：对解析值的绝对容差；容差由物理而非 run-to-run 噪声决定。
- **节奏**：`CI-fast`——每 commit（确定性）。
- **命名**：`<Phenomenon>AnalyticTruth` / `...NormalIncidence` 式，使解析锚显式。
- **物理位置**：目标态 `test/golden-analytic/<subsystem>/`；现状嵌在 `unit_test`（如
  `test_metal_trace_parity.cpp` 内的解析锚），可能也在 `test_projection` / `test_optics`。

### §1.3 `parity-cross-backend`

- **目的**：非 legacy 后端（Metal；未来 CUDA）在同一场景下与 **legacy CPU** 参考统计等价。
- **oracle**：**legacy CPU 是 ground truth。** 等价**不可仅凭相关性**断言——相关性已两次掩盖真
  bug（scrum-267）。必须配齐 metric-masks-bugs 全套：跨 seed 自洽 + 总能量守恒 + golden/解析锚 +
  人眼核查 + revert 反验（见 §4.2）。
- **阈值约定**：统计型（相关性地板 + 能量守恒界 + 跨 seed 一致）。绝不用裸相关性门。
- **节奏**：`PR` 与 `nightly`（heavy 变体）。
- **命名**：`test_<backend>_<aspect>_parity.{cpp,py}`；`...Parity` gtest suite。
- **物理位置**：目标态 `test/parity-cross-backend/<subsystem>/`；现状 `unit_test`（`.cpp`/`.mm`）
  + `test/e2e/`（pytest parity 测试）。

### §1.4 `e2e-correctness`

- **目的**：**整条 CLI 管线**端到端运行，对真实 config 产出正确图像 / 输出。
- **oracle**：tracked 参考图按 **PSNR** 对比；或非图像场景的 CLI 退出码 + 输出形状断言。
- **阈值约定**：每场景 PSNR 地板（钉在测试里）；冒烟级用退出码 / 文件非空检查。
- **节奏**：`PR`（快速子集跑 `-m "not slow"`；shared-lib 变体跑 `-m slow`）。
- **命名**：`test/e2e/` 下 `test_<feature>.py`。
- **物理位置**：目标态 `test/e2e-correctness/`；现状 `test/e2e/`。

### §1.5 `performance`

- **目的**：某后端**吞吐**达到或超过基线——GPU/单引擎路线必须打赢 legacy CPU，而非仅仅能跑。
- **oracle**：**分母永远是 legacy CPU**（GUI 实走的路径）。`CpuTraceBackend` 仅作 GPU 验证参考，
  绝不可作 perf 基线（见 §4.1）。报告跨重复的 `median` + `CoV`。
- **阈值约定**：对 legacy CPU 的比值地板；统计以 `median` 与变异系数报告；有 committed bench
  harness 提供 `CoV` 后收紧地板。
- **节奏**：`PR`（廉价哨兵）与 `nightly`（完整 bench 扫描）。committed bench harness 是任务 270.6。
- **命名**：`test_<backend>_throughput.py`；committed harness 用 `scripts/bench_*`。
- **物理位置**：目标态 `test/performance/`；现状 `test/e2e/`（吞吐哨兵）+ CI `Benchmark` 步骤
  （`--benchmark -f examples/bench_config.json`）。
- **边界注**：GUI 帧延迟 / 响应性测试（`test_gui_perf`）**不**在本层——其 oracle 是绝对帧预算，
  而非对 legacy CPU 的比值。它属于 `gui`（响应 tag）。见 §4.4。

### §1.6 `gui`

- **目的**：GUI 功能正确、视觉正确、响应及时。层内三个 tag：**功能**（控件行为、文件操作、交互）、
  **视觉**（渲染输出对参考）、**响应**（实时循环的交互交付——帧间隔、commit→首次 upload 延迟、
  **以及 steady-state / slider-drag 场景下 rays/restart、upload_rays 等光线交付量**）。光线交付量
  *反映*吞吐，但其 oracle 是 GUI 交互循环、而非对 legacy CPU 的比值——故归此层而非 `performance`
  （见 §4.4）。
- **oracle**：imgui test engine 驱动 app；**视觉**对 tracked 参考图断言（PSNR，每场景阈值在
  `_thresholds.json`）；**响应**对绝对帧延迟预算断言；**功能**断言控件/状态结果。
- **阈值约定**：视觉 = 每场景 PSNR（N 次随机渲染的 mean−3σ，见 AGENTS.md auto_ev 再生）；响应 =
  绝对延迟预算；功能 = 精确。
- **节奏**：`PR`。需要显示服务器，除非用 `LUMICE_SKIP_GUI_TESTS=1` 跳过。
- **命名**：`test_gui_<aspect>.cpp`；参考图在 `test/gui/references/`。
- **物理位置**：目标态 `test/gui/<tag>/`；现状 `test/gui/`（+ 由 pytest 驱动的
  `test_metal_gui_acceptance.py`，一个恰好走 e2e harness 的 gui 层测试——正是 purpose 主轴要处理的
  "层 ≠ 目录"典型）。

### §1.7 `regression-sentinel`

- **目的**：某个特定历史 bug 不再复发。
- **oracle**：**issue 的复现场景**——哨兵必须复现原始失败，而非人造替身（这是硬规则：回归测试用
  真实 issue 场景，绝不自造）。
- **阈值约定**：当初能抓到该 bug 的断言（精确，或被违反的那条具体不变量）。
- **节奏**：依 harness 取 `CI-fast` / `PR`——廉价处每 commit。
- **命名**：`test_<bug-symptom>.py/.cpp`，注释链到修复 commit / issue。
- **物理位置**：目标态 `test/regression-sentinel/`；现状 `test/e2e/`
  （`test_capi_sentinel_overflow.py`、`test_ms_filter_leak.py`、`test_errors.py`）。

---

## §2 subsystem 维度（层内 tag）

subsystem 是**二级**轴：层*内*的 tag，绝非顶层桶（纯 subsystem 轴会把 unit 与 perf 重新混在一起，
正是我们要离开的状态）。

| tag | 边界 |
|-----|------|
| `core` | 数学、optics、几何、simulator、ray path、filter、buffer |
| `backend` | `TraceBackend` 实现：Metal device 引擎、`CpuTraceBackend`、host/device seam |
| `server` | server render loop、consumer、stats、C API 桥 |
| `gui` | imgui app、面板、preview、文件 IO、poller |
| `config` | 配置解析与仿真配置数据 |
| `util` | logger、threading、queue、arguments、color data |

> **同名警示**：此处的 `gui` *tag* 仅是层内 subsystem 标签——例如 GUI 组件的单元测试落在
> `test/unit-correctness/gui/`。它与 `gui` *层*（§1.6，物理 `test/gui/`）**不是一回事**：后者按
> purpose 跨 功能/视觉/响应，按目的而非 subsystem 归类。词相同，但 tag ≠ 层。

tag 如何编码取决于该层的物理形态（§6）：对有自然 subsystem 划分的层用子目录
（`test/<layer>/<subsystem>/`），对保持扁平的层用 CTest `LABELS` / pytest marker。

---

## §3 决策树——"我要给 X 加一个测试"

路由到**层 + subsystem tag**。具体 target 名与物理路径由 §6 物理蓝图解析——本树决定*归属*，§6
决定*落点*。

```
1. X 是不是我要防止复发的历史 bug？
   → 是 → regression-sentinel。原样用 issue 复现场景。(§1.7)
   → 否 → 继续。

2. X 是否需要整条 CLI 管线运行（产出图像 / CLI 输出）？
   （由 GUI app harness 驱动整管线的测试——如经 pytest 的 test_metal_gui_acceptance——
    不算 CLI 管线；这里答"否"，走第 3 步。）
   → 是 → oracle 是参考图/输出，还是吞吐数字？
            • 图像/输出正确性 → e2e-correctness (§1.4)
            • 对 legacy CPU 的吞吐   → performance (§1.5)
   → 否 → 继续。

3. X 是否关于 GUI（控件行为、渲染视图、响应性）？
   → 是 → gui，选 tag：功能 / 视觉 / 响应。(§1.6)
            （响应性/帧延迟留在这里，不是 performance——§4.4。）
   → 否 → 继续。

4. X 是否把非 legacy 后端（Metal/CUDA）对 legacy CPU 比较？
   → 是 → parity-cross-backend。oracle = legacy CPU + §4.2 全套
            （仅相关性不足）。(§1.3)
   → 否 → 继续。

5. X 是否对闭式 / 解析物理真值断言？
   → 是 → golden-analytic。(§1.2)
   → 否 → unit-correctness。按 subsystem 打 tag（core/backend/server/gui/config/util）。(§1.1)
```

然后：选 subsystem tag（§2），按 §6 落点。*具体 target / 路径 / marker 见 §6。*

---

## §4 横切规则

以下规则跨所有层成立，编码的是用血泪换来的教训。

### §4.1 性能分母 = legacy CPU

任何性能声明的分母**必须是 legacy CPU**——GUI 实走的路径。`CpuTraceBackend` 仅用于在非 Metal
机器上验证 GPU seam；它**不是**性能基线，绝不可顶替分母。每条 perf 断言、每份 benchmark 报告都
显式声明分母，并指明测的是哪条线。

### §4.2 parity：metric-masks-bugs 全套

某后端与 legacy CPU 的相关性可以很高，而后端却是**错的**——这已发生两次（欠采样 bug 藏在健康
相关性背后，scrum-267）。因此 `parity-cross-backend` 测试**不可**仅靠相关性，必须组合：

1. **跨 seed 自洽**——后端跨 RNG seed 与自身一致（抓相关性抹平掉的欠采样）。
2. **总能量守恒**——发射能量在 MS 各层被核算。
3. **golden / 解析锚**——至少一个有闭式答案的配置（§1.2）。
4. **人眼核查**——一份人真正看过的渲染对比。
5. **revert 反验**——确认把修复 revert 后测试*会失败*（证明测试有牙）。

相关性是*烟雾信号*，不是裁决。跨 seed 自洽 + 能量守恒双门是 scrum-267.3 的刻意补强，不可删除。
这套全套也是未来 CUDA 后端区分"kernel 错"与"两后端一致但都错"的依据。

### §4.3 config 与 reference 的归属

- 每张参考图**恰好属于一层**：`e2e-correctness` 拥有 `test/e2e/references/*.jpg`；`gui` 拥有
  `test/gui/references/*.jpg` + `_thresholds.json`。
- 参考图在 `.gitignore` 中显式 un-ignore 并正常 tracked；config 与多数生成产物 git-ignored。移动
  参考图路径需同步更新 un-ignore 规则、读取它的测试、以及任何 CI 路径假设——三者一起。
- 随机参考图的再生遵循文档化流程（GUI `auto_ev`：`scripts/regen_gui_test_refs.py`，见 AGENTS.md）。
  参考图永不手工编辑。

### §4.4 `performance` 与 `gui`-响应性的边界

判据是 **oracle**，而非某指标是否"反映速度"。`performance` 层的定义 oracle 是**对 legacy CPU
基线的吞吐比值**（`median` + `CoV`）——它永远带 legacy-CPU 分母。一个 oracle 是**经 imgui engine
对 GUI 交互循环、按绝对预算测量**的测试**不**共享该 oracle，因此按规则同质原则（§0）它属于
`gui`（响应 tag），而非 `performance`。

这**显式包含吞吐味的** GUI 指标，不止延迟类：`test_gui_perf` 测帧间隔与 commit→首次 upload 延迟
（延迟），**也**测 steady-state / slider-drag 场景的 rays/restart、upload_rays（光线交付量）。
光线交付量*看起来像*吞吐，但其 oracle 是"实时约束(poller 节奏、commit interval、texture hold)下
实时预览交付了多少"——一个绝对交互预算，**无 legacy-CPU 分母**（历史上以 GUI regime 绝对增量记，
如"rays 18K→79K"，从不是对 legacy 的比值）。放进 `performance` 会强迫它套 perf 的 legacy-CPU 分母
纪律，而它不满足——成规则异质成员。故归 **gui（响应）**。

真正属于 `performance` 的吞吐是带 legacy-CPU 分母的那种：committed bench harness（270.6）与
`test_metal_throughput`。**"反映性能"不是判据；"oracle = 对 legacy CPU 的比值"才是。**

---

## §5 物理布局命名约定

迁移（270.3–270.5）时三套命名系统必须保持对齐：

- **CMake target**：目标态引入按 purpose 命名的 target 取代扁平 `unit_test`。**命名模式：
  `<layer-snake>_test`**（snake_case，沿用现有 `unit_test` / `integration_test` 约定）——如
  `unit_correctness_test`、`parity_test`、`golden_analytic_test`；GUI 层的 target 为 `gui_test`
  （取代 `LumiceGUITests`）。是否再按 subsystem 进一步拆 target（单个 `unit_correctness_test` vs
  `unit_correctness_core_test`+…）由 **270.3 裁定**，但上述*模式*在此固定，防命名漂移。迁移落地前，
  `unit_test` / `integration_test` / `LumiceGUITests` 保留。
- **CTest LABELS**：目标态为每层加 purpose 轴 label：`unit-correctness`、`golden-analytic`、
  `parity`（该 LABEL 是 `parity-cross-backend` 层名的缩写——全名对 CMake 过长；这是**唯一**缩写
  label，其余层名**不可**同样截断——`unit-correctness` 不可缩成 `unit`，会与旧机制轴 label 冲突）、
  `performance`、`gui`、`regression-sentinel`。现状 label 是机制轴（`unit` / `integration` / `gui`）。
- **pytest marker**：`slow`（需 shared-lib 构建；排除出 CI 快路径）与 `heavy`（slow + 冗余 parity
  变体；按 PR 用 `not heavy` 取消选择）是运行节奏 marker，保留。层/subsystem 在目标态用目录 +
  marker 表达。

**迁移锚 checklist（每次 270.3–270.7 移动强制）。** CI 硬编码了以下锚点；任何 rename/move/marker
改动漏掉一条都会让 CI 变红：

- [ ] `.github/workflows/ci.yml` 中 `ctest -R LumiceUnitTest`（及任何 `-R`/`-L` selector）在
      target 改名后仍能解析。
- [ ] `ci.yml` 中 pytest 路径参数仍能解析——E2E-Slow matrix 按文件名引用具体文件
      （parity 腿 `test_metal_exit_seam_parity.py`；rest 腿 `--ignore=...`）。
- [ ] marker selector `-m "not slow"` / `-m "slow and not heavy"` 仍选中预期集合。
- [ ] 参考图路径（`test/e2e/references/`、`test/gui/references/`）及其 `.gitignore` un-ignore 规则
      随读取它们的测试一起移动。
- [ ] `release.yml` 不受影响（它不跑测试）——确认，别假设。

---

## §6 现存测试 → 七层（exhaustiveness 映射）

下表证明七层覆盖整个现存套件、**无孤儿**，并作为 270.3–270.7 的迁移源。**迁移约束**列标出不可
随意移动/删除的健康项。

> 目标态目录规则（为 270.3 解决"子目录还是平铺"的歧义）：有**自然 subsystem 划分**的层
> （`unit-correctness`、`parity-cross-backend`、`golden-analytic`）用 `test/<layer>/<subsystem>/`；
> subsystem 边界模糊的层（`e2e-correctness`、`performance`、`regression-sentinel`）**平铺**为
> `test/<layer>/`，subsystem 用 marker/label 编码。`gui` 层用 `test/gui/<tag>/`（功能/视觉/响应）。

| 层 | 目标态路径 | 现状 C++（unit/integration） | 现状 e2e（pytest） | 现状 gui | 迁移约束 |
|----|-----------|-------------------------------|---------------------|----------|----------|
| **unit-correctness** | `test/unit-correctness/<subsystem>/` | `test_math`、`test_geo3d`、`test_optics`†、`test_crystal`、`test_rng`、`test_queue`、`test_threading_pool`、`test_color_space`、`test_json`、`test_filter`、`test_filter_spec`、`test_config_snapshot`、`test_render_config`、`test_sim_data`、`test_simulator`、`test_cpu_info`、`test_axis_presets`、`test_slider_mapping`、`test_window_sizing`、`test_raypath_segments`、`test_reduce_raypath_audit`、`test_c_api`、`test_exit_records`、`test_ev_auto`、`test_proj`(integration)、`test_integration_main` | — | — | — |
| **golden-analytic** | `test/golden-analytic/<subsystem>/` | `test_projection`†、`test_optics` 内闭式段†、`MultiMsContinuationNormalIncidence`（在 `test_metal_trace_parity.cpp`，2-MS 解析锚） | — | — | †逐文件确认"解析真值 vs unit-correctness"边界后才拆出 |
| **parity-cross-backend** | `test/parity-cross-backend/<subsystem>/` | `test_metal_trace_parity`、`test_metal_root_gen`、`test_metal_trace_backend`、`test_metal_filter_match_parity`(.mm)、`test_cpu_trace_backend` | `test_metal_exit_seam_parity`、`test_metal_batch_invariance`、`test_device_gen_default_path`、`test_cpu_backend_route` | — | `_parity_metrics.py` 是 parity 指标单一真源——**DO_NOT_MIGRATE_INDEPENDENTLY**（与其依赖者一起移）。能量守恒 + 跨 seed 双门是 267.3 补强——**勿删**。`test_metal_batch_invariance` 的能量守恒 `xfail` 是**合法的**（worst-case drain 未落地）——勿当 bug "修"掉。 |
| **e2e-correctness** | `test/e2e-correctness/`（平铺） | — | `test_smoke`、`test_cli`、`test_raypath_equivalence` | — | — |
| **performance** | `test/performance/`（平铺） | （无独立 C++ perf target；CI `Benchmark` 步骤跑 `--benchmark`） | `test_metal_throughput` | — | — |
| **gui** | `test/gui/<tag>/`（功能/视觉/响应） | — | `test_metal_gui_acceptance`（G4；gui 层，走 pytest harness） | `test_gui_auto_ev`、`test_gui_visual`、`test_gui_render`、`test_gui_bg`、`test_gui_crystal_renderer`、`test_gui_export`、`test_gui_import_export`、`test_gui_interaction`、`test_gui_face_number_overlay`、`test_gui_overlay_labels`、`test_gui_smoke`、`test_project_world_dir`、**`test_gui_perf`（响应 tag）**、`test_gui_main`/`test_screenshot`/`test_gui_shared`（harness） | `test_gui_perf` oracle = 绝对帧预算（§4.4），非吞吐对 legacy。 |
| **regression-sentinel** | `test/regression-sentinel/`（平铺） | — | `test_capi_sentinel_overflow`、`test_ms_filter_leak`、`test_errors` | — | `test_capi_sentinel_overflow` / `test_ms_filter_leak` 用 issue 复现守真 bug——**勿改场景**。`test_ms_filter_leak` 也与 parity 相关；其**主** purpose 是 sentinel（多 purpose → 按主 purpose 归类）。 |

**多 purpose 裁决规则**：一个测试服务多个 purpose 时，按其**主** purpose 归类（最直接守护其回归的
那个 bug/属性），并在注释记次要 purpose。例：`test_ms_filter_leak` → `regression-sentinel`（主），
parity 相关（次）。

**健康项——勿过度清理（合并"勿动"清单）**：`_parity_metrics.py`（单一真源）、能量守恒 + 跨 seed
自洽双门（267.3 corr-blind 补强）、`test_capi_sentinel_overflow.py` 与 `test_ms_filter_leak.py`
（issue 复现哨兵）、以及 `test_metal_batch_invariance.py` 中合法的 `xfail`。

> **legacy CPU 红线**：legacy CPU 是 parity ground truth（§1.3、§4.2）与 perf 分母（§1.5、§4.1）。
> 它及其测试在任何层都**绝不**是清理目标。
