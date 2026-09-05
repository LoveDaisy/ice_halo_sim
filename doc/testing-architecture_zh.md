[English](testing-architecture.md)

# 测试架构

本文档是 Lumice 测试如何组织、以及如何新增一个测试的**权威源**。它是一份**规范**：当你写一个
测试时，本文告诉你它属于哪一层、放在哪里、必须遵循什么 oracle 与阈值纪律、带什么 marker/label。
若某个实现细节与本文冲突，先修文档（当文档错时）或修正测试布局（当它漂移时），但**绝不允许**默默背离。

**目标读者**：新增或迁移测试的贡献者；核查新测试是否落在正确层、用对 oracle 的评审者；任何重排
测试套件的人。

> **状态声明（先读）。** 本文定义的是**目标态**架构（purpose 主轴，层 × subsystem）。物理迁移
> 在 milestone-cleanup 的 270.3–270.7 子任务中陆续落地：270.3 将单元 gtest 层拆分为
> `unit_correctness_test` / `golden_analytic_test` / `parity_test`，270.4 将 pytest 树重组到
> `test/e2e-correctness/`、`test/parity-cross-backend/`、`test/performance/`、`test/gui/`、
> `test/regression-sentinel/`，270.5（本层）将 GUI 测试迁入
> `test/gui/{functional,visual,responsiveness}/` 并把 target 重命名为 `gui_test`。§6 同时给出
> **历史**与**目标态**两栏，便于追溯测试来源。

---

## §0 为何 purpose 是主轴

历史上套件按**机制 / harness**组织：扁平的 `unit_test` gtest 二进制（约 30 文件）、一个
`integration_test` 二进制、一棵 `test/e2e/` pytest 树、以及 imgui-engine 的 `gui_test`
（原名 `LumiceGUITests`）二进制。每个桶内文件平铺。

人们真正用来思考的类别——"单元""性能""正确性""GUI""scrum-268 验收门"——其实是**验证目的
（purpose）**，不是机制。purpose 与 mechanism 正交，于是同一 purpose 被切碎到多处：Metal 正确性
散在约 4 处，性能散在约 5 处，G1–G4 验收门散在约 4 处。"我该怎么加一个新测试？"没有答案，于是新
测试落在任何方便的地方——熵增。

purpose 必须为主轴的更深层原因：**不同 purpose 遵循不同规则。** 每个 purpose 有自己的 oracle、
自己的阈值纪律、自己的运行节奏、自己的 false-green 风险。机制桶**不编码**这些规则。一个验收门
（带预登记硬阈值与反漂移纪律）和一句随手的单元断言都是"一个 gtest `TEST()`"，但它们不是同*种*
测试，绝不能用同一套规则维护。

**决定**：主轴 = 验证目的（八层，§1）。二维 = **subsystem**（层内 tag，§2）。harness/语言是每层
的实现细节，永不作主轴。

一层是一个**规则同质单元**：层内每个测试共享同一族 oracle、阈值纪律与节奏。这是判定层归属的判据
——见 §1 中 `test_gui_perf` 的案例（performance）与横切规则 §4.4。

purpose 本身不足以完全分开八层中的两层：`unit-correctness`、`composition-correctness`，与
`gui` 里功能/交互那部分，都可能被要求证明关于 `src/gui/` 同一类命题。真正把它们分开的是另外
两条轴，彼此正交，也与上面的层/subsystem 划分正交（完整判据见 §1.7 与 §3）：

- **链条长度**——要证伪该命题，需要几个 `src/` 单元协作：一个（`unit-correctness`）、多个但
  不到整个应用（`composition-correctness`）、或整个应用（`e2e-correctness`）。
- **机械需求**——证伪该命题需要什么：什么都不需要（一次纯函数调用）、一帧真实渲染、或一次
  合成的输入事件（点击/拖拽/按键）。只有后两者才需要 `gui` 的 harness。

这两条轴曾长期被塌缩成一条：所有触及 `src/gui/` 的东西都只按机械需求路由——这正是为什么
`composition-correctness` 直到「不需要任何机械、却跨多个单元」的命题被发现已经无名地散落在
`unit-correctness` 与 `gui` 两处（§1.2、§1.7）之前，从未作为一个命名层存在过。

---

## §1 八层

每层由六个字段定义：**目的 / oracle / 阈值约定 / 运行节奏 / 命名约定 / 物理位置**。节奏取值：
`CI-fast`（每次 push 的快速腿）、`PR`（拉取请求）、`nightly`（定时 / 本地 heavy）、`local`（开发者本地跑）。

### §1.1 `unit-correctness`

- **目的**：孤立组件正确且自洽（数学、几何、optics kernel、解析、config 快照、RNG、队列、线程原语）。
- **oracle**：测试内手算的期望值、不变量、round-trip 恒等式。不依赖跨后端或整管线。
- **阈值约定**：精确或紧容差（`EXPECT_EQ` / `EXPECT_NEAR` 小 epsilon）。无统计阈值。
- **节奏**：`CI-fast`——每 commit。
- **命名**：`test_<component>.{cpp}`；gtest `TEST(<Component>, <behavior>)`。
- **物理位置**：目标态 `test/unit-correctness/<subsystem>/`；现状 `unit_test` target。

### §1.2 `composition-correctness`

- **目的**：若干组件各自独立正确，但要证明它们沿着一条单一单元测试看不见的调用链**协作**正确——
  文档往返、跨通道一致性检查、多步生命周期。这是 `unit-correctness` 的 oracle 纪律应用在一个
  *关于*多个协作单元的命题上，不是一层更弱或更松的层。
- **oracle**：与 `unit-correctness` 同一族——测试内手算的期望值、不变量、round-trip 恒等式。
  无统计阈值、无跨后端依赖、不需要真实渲染。
- **阈值约定**：精确或紧容差，与 `unit-correctness` 相同。
- **节奏**：`CI-fast`——每 commit。
- **命名**：`test_<chain-topic>_chain.cpp`；gtest `TEST(<ChainTopic>Chain, <behavior>)`，其中
  `<chain-topic>` 命名的是这次协作本身（如文档往返、运行生命周期），绝不是单一 `src/` 文件名——
  一条跨多个单元的链条压不进一个单元的名字里。
- **物理位置**：`test/composition-correctness/<subsystem>/`；今天只有 `test/composition-correctness/gui/`
  一个 subsystem 被填充。
- **归属判据（把这层与两个邻层分开的东西）**：证伪该命题是否需要**两个或更多协作的 `src/`
  单元**（不是一个只为给另一个搭夹具而调用的单元），**且**是否**既不需要真实帧也不需要合成的
  输入事件**？两条都必须成立。链条长度本身不足以把一个用例送到这里——一条同时还需要渲染帧或
  点击/拖拽/按键的链条，无论跨多少单元都属于 `gui`（§1.7）；一条需要整个运行中应用的链条属于
  `e2e-correctness`（§1.5）。完整判定流程见 §3；§1.7 给出这两条轴彼此独立的实测证据——曾发现
  21 个用例 / 621 行跨 2–4 个 `src/gui/` 单元、却完全不需要任何运行时机械，正是这一发现最先让
  这一层的存在变得不可否认。

### §1.3 `golden-analytic`

- **目的**：管线（或其某阶段）复现**闭式物理真值**——独立于模拟器导出的解析值（如投影公式、
  有已知解析答案的法向入射续传结果）。
- **oracle**：闭式 / 解析导出的值——*绝对*真值，而非另一条代码路径。这是它区别于
  `parity-cross-backend`（oracle 是另一后端）与 `unit-correctness`（范围是单一组件）之处。
- **阈值约定**：对解析值的绝对容差；容差由物理而非 run-to-run 噪声决定。
- **节奏**：`CI-fast`——每 commit（确定性）。
- **命名**：`<Phenomenon>AnalyticTruth` / `...NormalIncidence` 式，使解析锚显式。
- **物理位置**：目标态 `test/golden-analytic/<subsystem>/`；现状嵌在 `unit_test`（如
  `test_metal_trace_parity.cpp` 内的解析锚），可能也在 `test_projection` / `test_optics`。

### §1.4 `parity-cross-backend`

- **目的**：非 legacy 后端（Metal；未来 CUDA）在同一场景下与 **legacy CPU** 参考统计等价。
- **oracle**：**legacy CPU 是 ground truth。** 等价**不可仅凭相关性**断言——相关性已两次掩盖真
  bug（scrum-267）。必须配齐 metric-masks-bugs 全套：跨 seed 自洽 + 总能量守恒 + golden/解析锚 +
  人眼核查 + revert 反验（见 §4.2）。
- **阈值约定**：统计型（相关性地板 + 能量守恒界 + 跨 seed 一致）。绝不用裸相关性门。
- **节奏**：`PR`。（本行原先列的 `nightly` heavy 变体已于 2026-08-11 随 `heavy` marker 一并删除
  ——没有任何 workflow 带 `schedule:` 触发器，所以它们从来没跑过。）
- **命名**：`test_<backend>_<aspect>_parity.{cpp,py}`；`...Parity` gtest suite。
- **物理位置**：目标态 `test/parity-cross-backend/<subsystem>/`；现状 `unit_test`（`.cpp`/`.mm`）
  + `test/e2e/`（pytest parity 测试）。
- **projection 子系统（315.5）**：`test/parity-cross-backend/backend/test_{metal,cuda}_projection_parity.py`
  对每种 LensType 做一次 legacy-oracle 对照，使**全部 11 种投影**（现经共享
  `src/core/shared/projection_shared.h::ProjectExitToPixel` 在 Metal + CUDA 上渲染）保持 cross-backend
  等价，并确认每种类型真走 GPU、不再静默回落 legacy CPU。二者共用共享 battery
  `test/e2e/_projection_battery.py`。

### §1.5 `e2e-correctness`

- **目的**：**整条 CLI 管线**端到端运行，对真实 config 产出正确图像 / 输出。
- **oracle**：tracked 参考图按 **PSNR** 对比；或非图像场景的 CLI 退出码 + 输出形状断言。
- **阈值约定**：每场景 PSNR 地板（钉在测试里）；冒烟级用退出码 / 文件非空检查。
- **节奏**：`PR`（快速子集跑 `-m "not slow"`；shared-lib 变体跑 `-m slow`）。
- **命名**：`test/e2e/` 下 `test_<feature>.py`。
- **物理位置**：目标态 `test/e2e-correctness/`；现状 `test/e2e/`。

### §1.6 `performance`

- **目的**：某后端**吞吐**达到或超过基线——GPU/单引擎路线必须打赢 legacy CPU，而非仅仅能跑。
- **oracle**：**分母永远是 legacy CPU**（GUI 实走的路径）。`CpuTraceBackend` 仅作 GPU 验证参考，
  绝不可作 perf 基线（见 §4.1）。报告跨重复的 `median` + `CoV`。
- **阈值约定**：对 legacy CPU 的比值地板；统计以 `median` 与变异系数报告；有 committed bench
  harness 提供 `CoV` 后收紧地板。
- **节奏**：`PR`（廉价哨兵——如 `test_metal_throughput`）与 `nightly`（完整 bench 扫描，跑
  `scripts/bench_throughput.py`，task-270.6 落地）。该 committed harness 是仓内所有
  `--benchmark` 类吞吐声明的标准工具；`scratchpad/bench/` 下的临时脚本不具权威性。
- **命名**：`test_<backend>_throughput.py`；`scripts/bench_throughput.py` 是 committed harness
  （矩阵 = {legacy, cpu_backend, metal} × heavy config × Metal dispatch sweep，median+CoV，
  比值分母锁 legacy CPU）。
- **物理位置**：目标态 `test/performance/`；现状 `test/e2e/`（吞吐哨兵）+ CI `Benchmark` 步骤
  （`--benchmark -f examples/bench_config.json`）。
- **边界注**：GUI 帧延迟 / 响应性测试（`test_gui_perf`）**不**在本层——其 oracle 是绝对帧预算，
  而非对 legacy CPU 的比值。它属于 `gui`（响应 tag）。见 §4.4。

### §1.7 `gui`

- **目的**：**一份正定义，不是残差定义**——一个命题属于这里，仅当证伪它需要经 imgui test engine
  产出一帧真实渲染、或一次合成的输入事件（点击/拖拽/按键/窗口操作）。这比"任何触及 `src/gui/`
  的东西"窄：一个关于 `src/gui/` 代码、但两者都不需要的命题，应按链条长度归入 `unit-correctness`
  （§1.1）或 `composition-correctness`（§1.2）。层内两个 tag：**功能**（控件行为与交互——控件是否
  读写了正确的状态，或对输入事件作出了正确反应）与**视觉**（渲染输出对参考）。第三个 tag
  **响应**，覆盖实时循环的交互交付——帧间隔、commit→首次 upload 延迟、**以及 steady-state /
  slider-drag 场景下 rays/restart、upload_rays 等光线交付量**。光线交付量*反映*吞吐，但其 oracle
  是 GUI 交互循环、而非对 legacy CPU 的比值——故归此层而非 `performance`（见 §4.4）。
  这一层更早的版本还有一个**功能**子桶专放文件 I/O 命题（"file ops"）；它们并不共享这一层的
  oracle（既不需要帧也不需要输入事件），现已按链条长度移到了 `unit-correctness` 或
  `composition-correctness`，与任何其他非机械命题一样——见 §2 的同名警示与 §3。
- **oracle**：imgui test engine 驱动 app；**视觉**对 tracked 参考图断言（PSNR，每场景阈值在
  `_thresholds.json`）；**响应**对绝对帧延迟预算断言；**功能**断言控件/状态结果。
- **阈值约定**：视觉 = 每场景 PSNR（N 次随机渲染的 mean−4σ，见 AGENTS.md lens_proj 再生）；响应 =
  绝对延迟预算；功能 = 精确。
- **节奏**：`PR`，外加 CI 每次 push 在 `xvfb-run` + llvmpipe 下跑的那两个参考图组（§4.6）。
  需要显示服务器，除非用 `LUMICE_SKIP_GUI_TESTS=1` 跳过。
- **命名**：`test_<aspect>.cpp`（功能/视觉）；历史命名早于这条约定的仍是 `test_gui_<aspect>.cpp`；
  参考图在 `test/gui/references/`。
- **物理位置**：`test/gui/<tag>/`（`functional/`、`visual/`、`responsiveness/`）+ 由 pytest 驱动的
  `test_metal_gui_acceptance.py`，一个恰好走 e2e harness 的 gui 层测试——正是 purpose 主轴要处理的
  "层 ≠ 目录"典型。

### §1.8 `regression-sentinel`

- **目的**：某个特定历史 bug 不再复发。
- **oracle**：**issue 的复现场景**——哨兵必须复现原始失败，而非人造替身（这是硬规则：回归测试用
  真实 issue 场景，绝不自造）。
- **阈值约定**：当初能抓到该 bug 的断言（精确，或被违反的那条具体不变量）。
- **节奏**：依 harness 取 `CI-fast` / `PR`——廉价处每 commit。
- **命名**：`test_<bug-symptom>.py/.cpp`，注释链到修复 commit / issue。
- **物理位置**：目标态 `test/regression-sentinel/`；现状 `test/e2e/`
  （`test_capi_sentinel_overflow.py`、`test_ms_filter_leak.py`、`test_errors.py`）。

#### §1.8.1 探测力：哪一类 oracle 会无声腐坏

哨兵会活得比它守的缺陷更久，而树里没有任何东西会告知它已经看不见那个缺陷了。这种衰减在两个方向上
都是无声的：测试照跑、照绿，在评审里照样读作"有覆盖"。决定一条哨兵会不会暴露在这件事里的，是
**它的 oracle 形态**——具体说，是它所比较的那个量究竟被构造钉死，还是可以随基线一起漂移。

| oracle 形态 | 会不会无声失去探测力 | 理由 |
|---|---|---|
| 二值事件——崩溃、退出码、`fell_back`、某个颜色类有没有出现 | 不会 | 事件要么发生要么不发生，没有余量可供漂移的基线蚕食。 |
| 两个活量互相比较——相等、相对差、健康时按构造恒为 1.0 或 0 的比值 | 不会 | 两侧一起漂移，比较本身对那份漂移免疫。 |
| 测量值 vs 硬编码常数，且健康值**不**被构造钉死 | **会** | 基线移动、余量被吃掉，且没有任何东西报告这件事。断言会一直绿下去，直到绿得空洞。 |

需要写下要求的只有第三行，而且**判别依据是被测量有没有被构造锚定，不是断言的语法形态**。
`test_benchmark_rate_not_impossible.py` 拿一个测量值比硬编码常数（`work_ratio <= _MAX_WORK_RATIO`），
却仍属第二行：它测的量——声称的每秒光线数 × 整轮墙钟 ÷ 该轮真正追踪的光线数——在估计器诚实时按构造
恒为 1.0，缺陷态下则是 ~20。换更快的机器或重写后端都不会移动这个数。反过来，
`test_full_sphere_roll_flip.py` 里的后半球能量占比（`_REAR_FRACTION_RANGE`）在语法上完全相同，性质却
相反：那是一个没有任何东西按着的绝对物理量，场景、采样器或某个晶体默认值都能让它自己朝着或离开那条
带走。要读的是那个量，不是那个运算符。（此处的测试名只是撰写时的形态例子；分类本身不依赖这几个文件
继续存在。）

**这条判据是拿什么换来的。** commit `f717a8f5` 在实测探测力为零之后，整文件删除了一条 benchmark 挂起
哨兵。它的断言是一个 30 秒墙钟上限，而那**一个**数字同时承担着两份要求相反的活：挂起兜底要宽、回归
判据要紧。把缺陷放回去之后，那份开销落进了 poll 线程本就存在的 `sleep_for` 空闲里——CPU 墙钟只动了
1.01×（3.54 → 3.59 s）、Metal 0.87×（0.78 → 0.68 s），即该闸盯着的那口钟根本看不见它；与此同时，同一个
数字的"宽"那一端在 `pytest -n auto` 下一天内造了两次假红。一个常数被要求同时又宽又紧，是值得及早认出
的前兆：第三行的 oracle 正是这样走到"余量已经不再约束任何东西"的。

**起疑时怎么确认。** 仓库里已有三个做两臂红态验证的脚本，该找的是它们而不是重新发明一遍：
`scripts/verify_crash_sentinel_detection_power.sh` 与
`scripts/verify_pyramid_crash_sentinel_detection_power.sh`（两者针对本层的哨兵），以及
`scripts/verify_apex_rescue_warning_detection_power.sh`（针对
`test/golden-analytic/core/test_closed_form_pyramid.cpp` 的近顶点守卫与 `src/core/geo3d_closedform.cpp`
里的降级 warning——不同的层，同一套方法）。三者都在独立的 `git worktree` 里回退修复，绝不碰调用者的
工作树，也都从头构建、而不是把增量构建的 "up to date" 当成"确实重建过"的证据——两个崩溃脚本强制
`rm -rf build/` 并在两臂二进制 md5 相同时拒绝继续，第三个则把那一臂建在自己的构建目录里、且在找不到
待打补丁的字符串时硬报错，因此没有哪一个会无声地去测一棵没被改动的树。两个崩溃脚本还会报出红的次数
及其统计功效（N=15、基线崩溃率 ~20% 时 P(0/15) ≈ 3.5%）。它们**不是**
CI 闸，也不该被升格成 CI 闸：每个脚本的文件头都自述了这一点，它们描述的调用方式是手动的——在对某条
具体哨兵起疑时跑（例如某次重构动了修复 commit，或会话级环境发生变化）。定期跑它们买不到任何东西，却
每次都要付一次干净重建。

**新增一条第三行哨兵时欠什么。** 当断言是"测量值 vs 硬编码常数"、且健康值不被构造钉死时，须在测试内
记录健康值实测是多少、这个常数留了多大余量——而不是只留一个裸数字。本层已有几条正是这么写的，值得照
抄：`test_relative_ev_breaks_additivity.py` 同时写下实测 spread 与它绝不能吞掉的效应量，再把两者之间的
间隙本身作为断言；`test_full_sphere_roll_flip.py` 与 `test_absolute_energy_additivity.py` 记录了跨若干
seed 的实测 spread 及其与缺陷态取值的对照。这条要求被**刻意**限定在第三行。另外两类形态不加任何新要求
——因为足以支撑一条要求的真实代价，至今只被证实过一次，就是上面那个 commit；为没有失效过的形态发明规则，
等于为想象中的伤害付费。

**这一节没有主张什么。** 上面那张表是一种"读断言即可判断一条哨兵"的方法，不是"本层哨兵已被盘点过"的
报告。本层只有一条哨兵的探测力做过红态重建的实证，另有两条有上述脚本；其余都是按 oracle 形态做的推断
——那是更弱的一类主张，也应当照更弱的读。

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
> `test/unit-correctness/gui/`。它与 `gui` *层*（§1.7，物理 `test/gui/`）**不是一回事**：后者按
> purpose 跨 功能/视觉/响应，按目的而非 subsystem 归类。词相同，但 tag ≠ 层。
>
> 这**一个**目录由**两个** CMake target 编译，而这条分界是**链接边界**，不是层或 subsystem 边界：
> `unit_correctness_test` 只链 `lumice_obj`，装 header-only 的那一半；`gui_unit_test` 额外链
> `lumice_gui_obj`，装会调用 `file_io.cpp` / `user_defaults.cpp` 的那一半（如
> `test_defaults_diff.cpp`）。同目录、同 `unit-correctness` LABEL，只有链接行不同，所以
> `ctest -L unit-correctness` 两个都选中。`gui_unit_test` 不开窗口、不建 GL 上下文；需要真实出帧的
> 用例应去 `gui` 层（`test/gui/`，target `gui_test`）。
>
> `gui` *tag* 也不是 `src/gui/` 命题唯一还能落脚的另一处：一个跨 ≥2 协作单元、却不需要真实帧或
> 输入事件的用例是 `composition-correctness`（§1.2），不是 `gui`——它自己的 `gui` subsystem 目录
> 是 `test/composition-correctness/gui/`。与 `unit-correctness` 的 `gui` tag 不同，
> `composition-correctness` 的**不**拆成两个 CMake target：`composition_correctness_test` 是唯一
> 编译它的 target，因为这里的每个用例本来就需要 `lumice_gui_obj`（不存在"链条里只需要 `lumice_obj`
> 的那一半"这种东西，那是单一单元测试才有的自由度）。

tag 如何编码取决于该层的物理形态（§6）：对有自然 subsystem 划分的层用子目录
（`test/<layer>/<subsystem>/`），对保持扁平的层用 CTest `LABELS` / pytest marker。

---

## §3 决策树——"我要给 X 加一个测试"

路由到**层 + subsystem tag**。具体 target 名与物理路径由 §6 物理蓝图解析——本树决定*归属*，§6
决定*落点*。

```
1. X 是不是我要防止复发的历史 bug？
   → 是 → regression-sentinel。原样用 issue 复现场景。(§1.8)
   → 否 → 继续。

2. X 是否需要整条 CLI 管线运行（产出图像 / CLI 输出）？
   （由 GUI app harness 驱动整管线的测试——如经 pytest 的 test_metal_gui_acceptance——
    不算 CLI 管线；这里答"否"，走第 3 步。）
   → 是 → oracle 是参考图/输出，还是吞吐数字？
            • 图像/输出正确性 → e2e-correctness (§1.5)
            • 对 legacy CPU 的吞吐   → performance (§1.6)
   → 否 → 继续。

3. 证伪 X 是否需要经 imgui test engine 产出一帧真实渲染、或一次合成的输入事件
   （点击/拖拽/按键/窗口操作）？
   （问的是这条**命题**本身，不是它今天恰好写成了什么样——一条本可以不碰 `ctx` 就断言的链条，
    即便某个现存用例经 GUI 驱动它，这里仍答"否"。§1.2 的诚实边界适用。）
   → 是 → gui，选 tag：功能（控件行为、交互）/ 视觉 / 响应。(§1.7)
            （响应性/帧延迟留在这里，不是 performance——§4.4。）
   → 否 → 继续。

4. 证伪 X 是否需要 ≥2 个协作的 `src/` 单元一起动作——一次往返、一次跨通道一致性检查、
   一次多步生命周期？（一个只为给真正被测的那个单元搭夹具而调用的单元不算——命题必须
   是*关于*这次协作本身，而不只是恰好调用了另一个单元。）
   → 是 → composition-correctness。(§1.2)
   → 否 → 继续。

5. X 是否把非 legacy 后端（Metal/CUDA）对 legacy CPU 比较？
   → 是 → parity-cross-backend。oracle = legacy CPU + §4.2 全套
            （仅相关性不足）。(§1.4)
   → 否 → 继续。

6. X 是否对闭式 / 解析物理真值断言？
   → 是 → golden-analytic。(§1.3)
   → 否 → unit-correctness。按 subsystem 打 tag（core/backend/server/gui/config/util）。(§1.1)
```

第 3、4 步分别是 §1 开篇提到的机械需求轴与链条长度轴，按照与旧的、purpose 被塌缩掉的路由
曾经失效的方式相反的顺序发问：机械需求先问，因为这正是 `composition-correctness` 出现之前
那棵树已经在问的问题（§1.7 的"目的"现在给出的是**正面**答案，不再靠排除法）；链条长度后问，
这样一个第 3 步答"否"的命题就不会像过去那样直接落进 `unit-correctness`。现在要落进
`unit-correctness`，还必须先过第 4 步的"否"——不能只靠"跟 GUI 无关"。

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
3. **golden / 解析锚**——至少一个有闭式答案的配置（§1.3）。
4. **人眼核查**——一份人真正看过的渲染对比。
5. **revert 反验**——确认把修复 revert 后测试*会失败*（证明测试有牙）。

相关性是*烟雾信号*，不是裁决。跨 seed 自洽 + 能量守恒双门是 scrum-267.3 的刻意补强，不可删除。
这套全套也是未来 CUDA 后端区分"kernel 错"与"两后端一致但都错"的依据。

#### §4.2.1 差分测试对「两侧共享的同步漂移」存在结构性盲区

上面那套全套防的是**指标**掩盖缺陷；还有第二个盲区，它属于**差分这个形态本身**而非指标：
一个「跑两条路径、断言两者一致」的测试，只能看见**不一致**。当两条路径同出一个权威时，
改动该权威会让两侧**一起移动**，测试照绿——无论比较写得多仔细。

样例是 `JsonParserParity`（`test/unit-correctness/server/test_json_parser_parity.cpp`）：
它把一份 config 语料分别喂给 core 的解析器直读、以及 C API（解析 → 回编码 → 再用 core 解析），
然后断言两侧一致。**两侧读的是 core 的同一张键名表**。若把某个键名**在权威表与其消费点一起改掉**，
两侧仍完全自洽 ⇒ 差分测试**不会变红**，尽管此时产出的 JSON 已经与磁盘上所有既有配置文件不兼容。
这是实测而非推演：PR #230 建立红态判据时，一次突变实际就是这个结果。

真正能抓住它的，是**期望值写成裸字符串字面量、且完全不调用被测权威函数**的钉子测试——
`test/unit-correctness/config/test_json.cpp` 里的 `CrystalSchemaKeyNames.*`（覆盖 `shape` 与
`axis` 两个对象），以及 `test/unit-correctness/config/test_crystal_sync_group.cpp` 里的
sync_group 子表钉子。若把这些期望改写成调用 `ShapeScalarSyncKeyName(...)`，权威表与它自己的
测试就会一起漂移、永远通过。

**规则**：当某个域被改造成「名称 / wire format 单一权威」时（render、filter、light source
都可能做同款改造），差分 parity 测试**不能**充当它的红态判据。至少要有一个测试，
拿被测代码**之外**的东西来断言字面的 wire format。

### §4.3 config 与 reference 的归属

- 每张参考图**恰好属于一层**：`e2e-correctness` 拥有 `test/e2e/references/*.jpg`；`gui` 拥有
  `test/gui/references/*.jpg` + `_thresholds.json`。
- 参考图在 `.gitignore` 中显式 un-ignore 并正常 tracked；config 与多数生成产物 git-ignored。移动
  参考图路径需同步更新 un-ignore 规则、读取它的测试、以及任何 CI 路径假设——三者一起。
- 随机参考图的再生遵循文档化流程（GUI `lens_proj`：`scripts/regen_gui_test_refs.py`，见 AGENTS.md）。
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

### §4.5 门禁化：`gui_test` / `gui_unit_test` 的分层是闸门，不是约定

§1.7 与 §5 按"是否需要活的帧"来放置一个 `src/gui/` 测试。这个放置决定了该测试**在 CI 里跑不跑**：
`gui_test` 需要显示器，只有一条 runner 腿供得上，且只跑 §4.6 点名的那几个参考图组；
`gui_unit_test` 链接同一个对象库，但无窗口、
无 GL context、无 ImGui 测试引擎，因而在每个能构建 GUI 的平台上都真跑。仅作为约定，这条分层活不下来——
阻力最小的路径指向错误方向：往 `gui_test` 里加是肌肉记忆，而且它一直能编译过。**一条打不过默认路径的
约定不是边界，只是偏好。**

因此该边界对**改动新增的部分**做门禁，由 `scripts/check_new_gui_tests.py` 执行——继
`check_policies.py`（全树）、`check_new_refs.py`（散文）之后的第 3 个 diff-scoped 入口。它在 CI 的
`new-refs` job 里对 PR 相对 merge-base 运行，在 pre-commit hook 里对暂存 diff 运行。它拒绝**由本次改动
带来的**、其 `TestFunc` lambda 自己声明了"不驱动 GUI"的用例：要么参数匿名，要么把参数标记
`IM_UNUSED` 之后再不提及。两者都是**显式表态**（一个来自编译器，一个来自作者本人）；规则建立在表态而非
"从函数体推断意图"之上，这正是它在本仓历史上假阳性为零的原因。

完整判据、**刻意不拦什么**、以及"无行内豁免"原则写在 `AGENTS.md` 的 "Testing and Platform Notes"
一节——单一权威，此处只引用不复述，以免两处漂移。有两条性质值得在本文档层面知道：

- **"是否新注册"是身份问题，不是行号问题。** 一个用例算新，当且仅当其 `category/name` 此前不存在于
  `test/gui/` 下。改读行 diff 会在**两个方向上同时失效**，且都是实测出来的：被拒的签名是模板化文本，
  于是"删掉若干用例 + 新增一个"的改动会让 git 把新写的签名行与被删的同文本行配对，门禁漏拦；而改锚
  `IM_REGISTER_TEST` 行——用例里唯一**不可能**别名的行，因为它带名字——又会在邻近用例被删时把仅仅
  位置平移的既有用例误判为新增。身份判据对两者都免疫。
- **门禁做成 diff-scoped，是因为存量清不了零。** 当前 `test/gui/` 下有 26 个用例符合被拒形态
  （8 个匿名、18 个标了 `IM_UNUSED`），全树门禁无法开局即绿；替代方案是冻结基线，而那是一份需要永远
  维护正确的资产。与其散文侧的姊妹门禁同一取舍、同一理由。注意：**形态本身不足以断定这 26 个放错了
  地方**——一个用例可以既需要帧又不碰 `ctx`，这正是门禁只对"新增"发言、且提示语是建议而非断言的原因。

### §4.6 GUI 视觉回归层红了：它意味着什么、你必须做什么

这一层的检出力是两个因子的乘积——P(破坏发生时它变红) × P(变红时它被当真)——而历来只有第一项被
工程化过。第二项被直接量过：描述里提到 GUI 视觉失败的 5 个已合入 PR（#90、#119、#129、#168、#180），
**无一例外走了同一个动作**——重跑一次、和 base commit 对比、判为已知的随机 flake、合入。#180 的原话有
代表性：「the one miss is the known `overlay_ea` stochastic flake, 3/3 on standalone re-run」。一旦这
成为默认响应，**一次真命中就会以与假警报完全相同的方式被处理掉**，第一项也就什么都买不到了。压低噪声
地板修不好一个等于零的第二项。

这份处置习惯之上还有一个更基础的问题：这一层在它的全部历史里，真的抓到过什么吗？两条独立检索——
本仓库全部历史的 commit message，以及 29 个描述里提到 GUI 视觉回归结果的已合入 PR 正文——均未找到
任何"因这一层变红而发现产品缺陷"的记录。这里的措辞是**未找到**，不是**从未发生过**：两个检索面
都是合入者留下的文字，一次从未被写进文字的命中，对两者都是不可见的。这不是留退路的模糊说法——它与
本节其余部分依赖的诚实态度是同一件事。

本节是取代那个默认动作的东西。三部分：CI 里真跑了什么，红了之后你有什么义务，以及——写出来而不是留在
沉默里——这一层还有哪些部分除了你没有任何闸门。

**CI 里真跑什么、不跑什么。** `build` job 的 `Ubuntu x86_64` 腿装 Xvfb + Mesa llvmpipe 软件光栅化器，
在其下跑 `gui_test` 的 `modal_layout` 与 `defaults_panel_layout` 两个参考图组。这条腿本来就是 required
status check，所以不需要新增 branch-protection context：这十个场景现在与 `ctest` 一样能挡住合入。
覆盖面是按实测选的，不是按方便选的：

| 参考图组 | Linux + llvmpipe 对 macOS 拍摄的参考图 | 进 CI？ |
|---|---|---|
| `defaults_panel_layout`（6 景） | **60.56–61.99 dB**（真实 amd64 runner），地板 40 dB；arm64 下 5 次重复逐位相同 | 是 |
| `modal_layout`（4 景） | **46.88–47.96 dB**（真实 amd64 runner），地板 40 dB；arm64 下 5 次重复逐位相同 | 是 |
| `lens_proj`（6 景） | 19.49–27.72 dB——每一景都**高于**按 macOS 标定的阈值 0.88–1.49 dB | 否，见下 |
| `screenshot`/`visual` 的晶体场景（3 景） | 34.78–35.94 dB，地板 40 dB | 否——不可移植 |
| `capture_harness` 的 `fullframe` | 21.92 dB，地板 40 dB | 否——不可移植 |

**amd64 确认，以及它改变了什么。** 上表两个进 CI 的组现在填的是真实 `ubuntu-24.04` runner 的数字
（取自 CI 步骤自身日志，12/12 景通过）。与它们替换掉的 arm64 数字的对照值得留下，因为这是**唯一一次
实测那个代理环境到底外推了多远**：

| 组 | arm64 容器（代理） | amd64 runner（真实） | 差 |
|---|---|---|---|
| `defaults_panel_layout` | 60.56–61.99 dB | 60.56–61.99 dB | 无——完全相同 |
| `modal_layout` | 47.12–48.23 dB | 46.88–47.96 dB | ≈ −0.27 dB |

即：代理对一个组精确命中，对另一个组差约四分之一 dB。两个结果都不动摇结论——最紧的真实余量是
46.88 dB 对 40 dB 地板，余 6.88 dB——但第二行正说明下面那条告诫值得写，而不是想当然地假定可移植。
其余各行仍然只有 arm64 数字：没有任何东西在 amd64 上跑它们。

**这张表没法沉默带过的一条告诫。** 下面每个数字、以及上表里的 arm64 数字，都测自一个 **arm64**
Docker 容器（`ubuntu:24.04` +
Xvfb + Mesa llvmpipe，跑在 Apple Silicon 宿主机上），用来代替这个步骤实际跑在其上的 **amd64**
`ubuntu-24.04` GitHub Actions runner——两者差的不只是"一台带软件光栅化器的 Linux 机器"，是 CPU 架构
本身。这个替代是一次有意的、有界的选择（当时没有可用的 x86_64 机器），不是疏漏，它稳妥地回答了本节
依赖的那些架构无关的问题——Xvfb+llvmpipe 下能否建立 GL 3.3 core 上下文、`gui_test` 能否跑完、PSNR
量级是否对——但给不出**已确认的 amd64 余量**：上表的 dB 数字换到真 runner 上跑，可能会变。缓解这一点
的是：这个 CI 步骤已经真的在跑，所以这个缺口会被持续闭合而不需要一次性确认——每次触及 `Ubuntu x86_64`
腿的 push，都会在这一步自己的日志里报出真实的 amd64 数字（`ci.yml` 里 `Upload GUI visual-regression
log` 步骤会把它保留 30 天），也就是说合入后的第一次真实运行本身就是这张表目前缺的那份确认。在真正跑过
一次 push 之前，"10/10 高于确定性地板"应读作 arm64 代理证据，不是 amd64 目标证据——这个区分很重要，
本节存在的意义正是不让数字被凭空采信。

第一个意外是参考图有多能跨平台。本仓所有参考图都拍摄于 macOS 的 Metal GL 栈；拿到 Linux 上用软件光栅
化器、在不同 CPU 架构上比对，按统计式阈值判定的 `lens_proj` 组仍然全部越过在拍摄机上标定的阈值。
「分布式比较不依赖随机流，这正是跨平台可移植性的来源」在此之前只是设计意图，现在有了证据。

第二个意外是可移植性到哪里为止。晶体预览场景走 `crystal_renderer.cpp` 的着色与线框光栅化，`fullframe`
则整帧读回默认帧缓冲、里面就含着那个预览。GPU 驱动与 llvmpipe 在光栅化与抗锯齿上的差异值 4–18 dB——
远远越过一个本就为断言"近乎逐位相同"而存在的 40 dB 地板。这些场景**只在拍摄它们的那一类渲染器上有
意义**；把它们排除在 CI 之外不是为了躲红而放松阈值。

**为什么 `lens_proj` 不在 CI 里，以及什么能把它放进去。** 不是渲染原因——它的像素是可移植的，见上表。
挡路的是 `test/gui/visual/test_gui_lens_projection.cpp:276`：一条对"跑完的仿真报告了多少光线"的精确
相等断言，其自身注释宣称这个数字「is identical on every machine and every rerun」。在软件渲染下这条
前提是假的：

| 环境 | 重复 | 失败 | 亏空 |
|---|---|---|---|
| macOS，GPU | 5 | 0 | — |
| Linux + llvmpipe，12 核 | 5 | 2 | 128 条光线 |
| Linux + llvmpipe，4 核（贴近 runner） | 5 | 3 | 128 / 896 / 1024 条光线 |

亏空恒为 128 条光线一批的整数倍。上表的实测早于一次修复（追踪在
`doc/gui-preview-lifecycle-architecture.md` §9，不变量 I3）——那次修复动的正是本节最初诊断的机制：
过去只有当这一轮 poll 携带了新的 snapshot generation、或者走了终帧救援分支时才会重读统计，而观察到
`LUMICE_LIFECYCLE_COMPLETED` 的那一轮 poll 会在自己末尾把 poller 自暂停、此后再无任何一轮去纠正它
——于是亏空恰好落在最后一轮时永远不被吸收。这个结构现在已改为电平触发（drain-aware 自暂停 guard +
持续 reconcile 的慢心跳），修复后的一次容器内白盒探针证实了 poller 这一侧：无论跑到一半还是跑完之后
取样，显示端数字都与后端活计数逐位相等，后端的 `Completed`/排空信号也与 poller 侧一致——**poller 不
是那个变陈旧的一方**。

**但同一份探针显示 128 整数倍的亏空依然存在**——就藏在后端自己的活计数里，不只是 poller 显示出来的
数字。这把缺陷的根因整个挪到了显示层的上游：一次已完成、已完全排空的运行，其累计计数仍可能比配置
预算少整数个 128 光线一批的派发粒度，且这发生在 poller 读取它**之前**、生产/入队侧。任何显示层的
修复——包括这一次——都不可能补上一个显示数字本身就已经短缺的口子。这个独立发现的亏空是否要单独立
一条缺陷记录、以及上表对原始 `lens_proj` 实测的归因是否需要因这同一种数字形状被证实另有上游成因而
修订，两者都还悬而未决，等待本文档的 owner 拍板——不是本节能自行了断的。

**净效果：本节最初点名的那个机制已经修好，但 `lens_proj` 之所以仍未从 CI 解锁，是因为它的挡路者换了
一个，而不是被清掉了。** 今天把它接进 required check，仍会在同样两个 5,000,000 光线的场景上人为制造
假红——也就是说，本节原先"那个缺陷修好之后，把 `lens_proj` 加进 CI 步骤的 `--filter` 就是全部改动"
这句话不再成立；挡路的是第二个、位于上游的缺陷。harness、LFS 拉取、"它真的跑了"断言依然已经就位，
所以只要那第二个缺陷被关闭，机制层面剩下的仍然只是一行 `--filter` 改动。

**这个闸真拦得住吗？** 部分拦得住，而诚实的形状比让人安心的版本更重要。该步骤跑在 `Ubuntu x86_64` 里，
它**确实**在 `required_status_checks.contexts` 中，所以红了会挡住合入按钮。但本仓
`enforce_admins: false`、`required_approving_review_count: 0`，而 owner 是管理员——所以 owner 本人
可以越过这里的任何红，包括这一条。这不是本步骤引入的弱点：它是本仓**每一条** required check 的既有
性质，而 `policy` 与 `new-refs` 两个 job 更弱——它们压根不在 required 列表里，尽管别处的散文把它们
描述为"已门禁"。因此这一步买到的东西是有界的，值得精确命名：它把这一层从**从未执行、零信号**，
搬到了**每次 push 都执行、红是可见的、越过它需要一个主动动作**。这就是全部主张。CI 线以下的部分是
没有任何机器会替你抓的部分——这正是它被写下来的原因。

**处置判据清单——红了之后你有什么义务。** 适用于这一层的每一次红，无论在 CI 还是在开发者本机。
「已知 flake」不是一种处置；它留下的记录正是上面那次审计所发现、且无从据以行动的东西。

1. **diff 碰没碰渲染路径？** `src/gui/preview_renderer.cpp`、`export_fbo_renderer.cpp`、
   `crystal_renderer.cpp`、`edit_modals.cpp`、`defaults_panel.cpp`、任何 shader，或参考图捕获 harness。
   若碰了，在被证否之前这次红就是回归——举证责任在改动一方，不在测试一方。
2. **在同一个 base commit 上、用同一种运行模式复现它。** 不是"重跑看看"：在 base 上用与变红那次相同的
   filter 和相同的池（`--fixed-dt` 与否）跑。红在 base 上依然存在 = 既有状况；不存在 = 是你的。
3. **记数字，不是记结论。** 失败场景的 PSNR、它的阈值、重复几次中失败几次。每一次比较都会往 stderr 打
   `[<group>] <scene>: PSNR=... (threshold=...)`，把它贴上。一份不含数字的处置记录不可复核。
4. **在说"环境问题"之前先把机制说出来。** 一次确实不是渲染回归的红也仍然有成因，而且几乎总能定位——
   上面那条 `test_gui_lens_projection.cpp:276` 在没人去看的岁月里一直读作"随机 flake"，结果是 poller
   终态边沿的一条竞态。机制说不出来，红就成立。
5. **重生成参考图是另一个主张。** 若这次红是被同一个 PR 里的参考图重生成解释掉的，就写明重生成了哪个
   组、用的什么命令、之后的 PSNR 余量是多少。在一个"红被归咎于它"的 PR 里做重生成，会切断后来者据以
   区分"重生成残留漂移"与"新缺陷"的那条审计链——这一点当时由实施方自陈于 PR #93，也是本条规则存在的
   原因。

**显式降级——哪些平台与哪些组没有任何闸门覆盖。** 写出来，使它成为一个选择而不是一段沉默：

- **macOS ARM64 与 Windows MSVC x86_64** 构建 `gui_test` 但从不运行它。macOS 没有 Xvfb 的等价物，
  Windows 的软件 GL 路径未经验证——两者都未在此研究过，也都不应假定容易。在这两个平台上这一层是
  **本机 gate**：红不会被任何东西挡住，上面那份清单就是执行力的全部。
- **`lens_proj`** 在上述 poller 缺陷修好之前，在所有平台上都是本机 gate——包括在 Linux 上，尽管它的
  像素被实测证明是可行的。
- **晶体预览与整帧场景**永久性地只是"拍摄它们的那类机器"上的本机 gate。没有让它们可移植的计划，
  它们在软件光栅化器上的红完全不说明任何问题。

**为什么是这个形状，而不是四个单选答案之一。** 桌面上原有四条路线，而没有一条能单独作答的原因是：
它们其实不是互斥项——每条覆盖这一层的不同切片，而切片是由实测划出来的，不是由路线自己假设的。

- *把 `gui_test` 接进 CI。* 采纳，但只在被证明可行的那个切片上——十个场景，在贴近 runner 的配置下重复
  逐位一致。它当初被记为 build-only 的理由是"CI runner 没有显示服务器"，这条前提一直成立到有人真去试
  Xvfb 为止；这个方向多年前就被识别过并被主动推迟，从未被验证过。它**不**被采纳于另外三个切片，而且
  每次都是因为实测理由而非谨慎：`lens_proj` 是 poller 竞态，晶体与整帧场景是渲染器不可移植。
- *做成门禁。* 在门禁能存在的地方采纳，而那个范围比听上去窄。门禁作用于**代码形态**；这里要修的失败
  是一种**处置行为**——人看到红之后做了什么。没有任何检查器能读它。近似它的尝试（比如禁止 PR 描述里
  出现"known flake"字样）门禁的是措辞而不是行为，换个说法即可绕过。所以机械的那部分恰好是可机械化的
  那部分——检查真跑、且是 required——其余部分作为纪律陈述，而不是打扮成执行力。
- *把纪律写明、并让红可归因。* 采纳，而且它是 CI 够不着的所有部分的承重件：上面那份清单，用一份可复核
  的记录取代了「重跑一次、对比 base、合入」。它是软约束，也会以软约束的方式失效。它依然值得写，因为
  在那些平台上，替代方案不是一条更强的规则——而是没有规则。
- *接受现状、降级这一层。* **只**作为显式声明采纳，绝不作为默认态。旧状态站不住脚的地方不是"有些场景
  没人管"，而是这个仓库为一个它并不据以行动的信号支付了标定与维护成本，却从未就此做过决定。上面那份
  降级清单就是这个决定，按平台、按组，出声地做出来。

被直接否决的是第五个选项——没人提议、但一切都会朝它漂移的那个：把阈值放松到这一层不再变红。那是把
"不被相信"换成"不再报告"，检出力同样是零，只是连证据一起抹掉了。

### §4.7 为什么 `auto_ev` 被退役了，而 `lens_proj` 没有

两者都曾是一组来自随机渲染的参考图，但它们变的是相反的轴。`lens_proj` 的场景钉死仿真——六景全共用
`halo_22.json`——变的是显示轴，每个逆投影分支一景。已退役的 `auto_ev` 组反过来：全部十景钉死显示轴
（`fisheye_equal_area`、elevation 20），变的是仿真轴（光谱、晶体 type、天顶分布、filter、多散射层数）。

这对一个显示回归层来说是选错了要变的轴，而且这条轴早已别处有主。`auto_ev` 十个场景背后是九个不同
config（`overlay_ea` 复用 `halo_22.json`），其中七个
（`halo_22`、`color`、`cza`、`filters`、`multi_scatter`、`parhelion`、`pyramid`）同时也是
`test/e2e-correctness/test_smoke.py` 的 config，在那里断言的阈值平均严 8.7 dB——而且 `test_smoke.py`
经 CLI 在每个 PR 上真跑，`gui_test` 的 `auto_ev` 组当时却只编译不跑。两者不是同一起跑线（CLI 那组
config 的光线预算也比 GUI 这边高），但 `auto_ev` 把十分之九的场景花在变一条本仓库早已在别处、每次
push 都测得更严的轴上。

`auto_ev` 唯一独占的那条轴——显示路径——结果也不是它独占的。`test_gui_lens_projection.cpp` 读的是
同一份 `snapshot_intensity` / `ev_auto` 状态，走的是已退役的 `test_gui_auto_ev.cpp` 当年那条一模一样
的代码路径来设 exposure，而 `EvAuto` 套件（`test/unit-correctness/gui/test_gui_widget_rules.cpp`，
`unit_correctness_test` target）用确定性断言直接验证 auto-EV 的计算本身，且仍在三平台 CI 上跑。把
仿真轴场景记到 `test_smoke.py` 名下、把 auto-EV 管线场景记到 `EvAuto` 套件名下之后，`auto_ev` 唯一
独占的领地只剩 `overlayAuxLines()` 画的 zenith/nadir
标记与坐标网格——一景的量。那一景，`overlay_ea`，被保留下来并迁入了 `lens_proj`，没有跟着其余场景
一起退役。

---

### §4.8 GUI 套件为何长成这个形状：规则不是数据，测试就只能是实例

**历史诊断，留其机制，不留其户口普查。** 紧接下方的数字，量的是 §4.8.2 记录的分层拆分与重写
**之前**的套件状态——重写新增了整整一层（`composition-correctness`，§1.2），这里数到的两个
GUI target 也早已不再持有量数时的那批用例。本节论证的机制——一条规则若停留在控制流而非成为
数据，就会迫使它的测试长成实例形态——正是促成那次重写的原因，且不因这个计数陈旧而失效；
§4.8.1 与 §4.8.2 记录了由此带来的实际变化。下方出现的**文件名**同理：`test_gui_interaction.cpp`
正是那次重写删掉的杂物间，本节提到它是在记录「作诊断时那些用例住在哪」，不是今天的去处
（当前地图见 §6 的布局表）。

`gui_test` 与 `gui_unit_test` 合计 721 个注册用例中，**只有 23 个断言的是一个对「生产代码派生的
集合」全称量化的命题**——字段注册表、枚举、能力表——因而新增一个字段或一种镜头类型时，它们不必
改动就自动覆盖到。其余 698 个各断言一个具体个例。这个 3% 对 97% 的比例是关于本套件形状最承重的
事实，而它的成因不在测试里。

**同一个文件里，两种形态相隔十五行并存。** `src/gui/app_panels.cpp` 中，FOV 滑块的禁用状态取自
字段编辑器注册表——`ConstraintFor("renderer.fov", g_state)`，其注释写明 domain、format 与
disabled-when "all come from the field editor registry rather than being written here"。测试可以
调同一个取值函数去对拍，`test_gui_interaction.cpp` 里有四个用例正是这么做的。而下方几行的可见性
单选按钮，把同一类问题写成了内联判断：`if (full_sky) { ImGui::BeginDisabled(); } …
ImGui::BeginDisabled(is_globe);`。没有 `IsVisibilityWidgetEnabled(lens, widget)` 可调，测试**根本
无从与规则对拍**，只能设一种镜头、戳一个 widget。于是产出九个近乎相同的用例，(镜头 × widget) 网格
一格一个；其中两个在归一化掉镜头常量与期望值之后**逐字相同**。

这个对应关系是全树成立的：GUI 目标里每一个不变量形态的用例，都落在注册表或表驱动的子系统之上
（字段 tier 表、用户默认值资格解析器、默认值 diff 的键集合、字段编辑器注册表）。**规则进了数据，
测试就长成不变量；规则留在控制流里，测试就停在实例。**

**实例形态的代价付两遍。**

*体量。* 把每个用例体里的字符串、数字与 `k…` 常量归一化后，用 TF-IDF 加权余弦比较——经校准使
人工核实过的「同网格」对得分 0.66–1.00、「不同机制」对得分 0.01–0.18——**54% 的用例处于一个成员两两
互为近拷贝的团中，37% 是团内第一个之外的成员**。该数字对阈值敏感（合理区间内约 30–50%），且
「可压缩」不等于「可删」：把网格收拢要求不变量形态可被表达，而那正是上文的生产侧问题。最大的几个
团无一例外都是双参数网格被一格一个写出来——边界值 × 预设类型、镜头 × trackball 手势、文档入口
路径 × 清空、投影 × 镜头类型。

*看起来被覆盖了的洞。* `src/gui/file_io.cpp` 有 99 处 `.value(key, default)` 回退，其中 12 处的
默认值派生自其所属结构体（如 `RenderConfig{}.azimuth`），**87 处硬编码字面量**，其中 44 处是非平凡
取值（`altitude` 20.0f、`diameter` 0.5f、`max_hits` 8、`sim_resolution` 1024、`lens_type` "linear"）。
commit `00fb12fc` 修的正是其中**一处**——一个已经与真实默认值漂移的 `ray_num_millions` 回退——
同形态的其余站点没有被一并扫掉。该区域的两种测试形态**在结构上都够不到它们**：完整往返永远会写入
该键，于是 `js.value(key, fallback)` 永远命中，回退分支永不求值；手写旧格式文档的用例省略了一些键，
因而**确实执行了**一部分回退，但只断言作者当时想到的那两三个字段。**行覆盖率工具会把这些回退标记
为已覆盖。** 把 `00fb12fc` 的缺陷重新注入，两个 GUI 目标中没有任何一个用例变红。

**由此固化的工作规则。** 这些重新发现一次都很贵，每条都至少付过一次学费。

1. **廉价信号只能收窄候选，永远不能判决。** 名字相似、词法分类器、共享符号交集、孤儿 JSON 键——
   在本仓各自产出过看起来很确定的候选集，而其成员一经读码都被判为非冗余。这个失败是结构性的、
   不是调参能修的：冗余是**语义**性质（「不存在只有其中一个能抓到的破坏」），而所有廉价信号测的
   都是表面共现。两个共享 61% 生产符号的测试可以有零断言重叠——「吞吐为正」与「像素有光」是关于
   同一条管线的两个不同问题。
2. **在采信任何「没找到」之前，先用已知阳性校准方法。** 一次全树审计给出的干净结论，其可信度不
   超过它已被证明的灵敏度；已退役的 `auto_ev` 组（§4.7）是本仓做套件形状类工作时的校准样本，可从
   历史中取回。把它归入「干净」档的分类器，测的不是它声称在测的东西。
3. **判断形态一律读体，绝不看名字。** 用例名记录的是作者的**意图**，而这里讨论的性质是**代码结构**
   的性质，两者之间没有可靠映射。
4. **提出任何删除之前，先把冗余与三角测量分开。** 两个在同一破坏下同时变红的用例，只有在「谁都没
   有对方所缺的归因信息」时才是冗余——也就是说，只有当**不存在**只被其中一个抓到的破坏时才是。
   若这样的破坏存在，它们就是三角测量，必须都留；§4.2 的 metric-masks-bugs 全套是长期实例，而在
   本仓，仅凭相关性已经两次掩盖过真实缺陷。用针对真实代码的红态探针去确定你面对的是哪一种，
   而不是靠观察。

**红态探针的基线必须在同一条件下、取多次。** 单次全套件绿不构成基线：真实时序类用例对负载敏感，
机器上另一处并发构建就足以让其中一个变红。⭐**负载只制造假红，不制造假绿**——所以「没有任何用例
抓到它」这类结论在嘈杂机器上依然成立，而「这个套件不稳定」这类结论不成立。

#### §4.8.1 落地状态（2026-08-07）

上文诊断点名了三处生产侧站点，各自把一条规则留在控制流而非数据里。落地结果，逐条：

| 站点 | 结果 |
|---|---|
| `src/gui/app_panels.cpp` 可见性单选按钮（`renderer.visible`、`renderer.front`） | 已落地。两个字段其实**早已注册了谓词**（`NotUnderFullSky` / `NotUnderFullSkyOrGlobe`，`src/gui/field_editor_registry.cpp:432-433`）——正是上方十五行的 FOV 滑块已在用的那套；单选按钮只是没接上。`app_panels.cpp:697` 与 `:708` 现在按同样方式读 `ConstraintFor(...)`。一条 `gui_unit_test` 语义不变量 + 一条 `gui_test` 调用点矩阵（11 镜头 × 4 widget）替代了 9 个实例用例。 |
| `src/gui/file_io.cpp` 的硬编码 `.value(key, default)` 回退 | 已落地。硬编码字面量占比 88% → 21.2%；一条全称「缺键→结构体默认值」不变量现覆盖 80 个字段，能重现 `00fb12fc` 那类漂移——此前 721 个用例对它零捕获。附带发现：legacy core-JSON 加载路径的对照物其实是 **core** 结构体的默认值，不是 GUI 结构体的——core 的解析器对每个字段一律走 `if (contains) get_to`，缺键落在 core 的 seed 上，不是 GUI 侧的字面量。据此原本看似 6 处的 GUI/core 默认值真分歧收窄为 2 处，均留作 core 侧的政策裁定。 |
| 投影数学抽 GLSL/C++ 共享头 | **未立项——是对理由的 NO-GO，不是对可行性的 NO-GO。** 见下。 |

**投影共享头的裁决。** 一次真实的双编译已经跑过：同一段候选文本被 clang（`-std=c++17 -Wall
-Wextra`，exit 0）与驱动自身在 `gui_test` 启动时用的 GLSL 编译器双双接受，场景也正常渲出——头文件
是可构建的。真正否决提案的是它原本要解锁的两条性质测试：`inverse(forward(d)) ≈ d` 对**任何**
自洽约定都恒闭合，测不出手性翻转（`test/unit-correctness/gui/test_render_handedness_guard.cpp`
的头部注释已论证过这一点，该套件也正因此选了绝对屏侧判据）；「GUI 逆投影与 CLI 前向投影手性一致」
则**已经存在**——同一文件的 8 个用例已跨 3 份前向实现加 1 次逆向读回覆盖了它。仓内这套投影数学
共有五份实现，其中四份早已有共享头（`src/core/shared/projection_shared.h`，被 CLI/Metal/CUDA 共用），
唯独片段着色器那一份没并进去。对五个 GLSL 逆投影函数（`linearInverse`、`fisheyeInverse`、
`dualFisheyeInverse`、`rectangularInverse`、`globeInverse`）跑 `git log -L`，合计 14 次改动，最近一次
在裁决前三个多月，而同期该文件本身有 54 次提交——这部分逻辑几乎不动，手工维护两份同步副本的代价
因此偏低。**触发条件，不是待办：** 那 14 次改动里有几次修的正是「GLSL 与 C++ 对不上」本身
（commit `458a6785`：`BuildViewMatrix` 的旋转链、fisheye `img_radius`、dual-fisheye 符号）——两侧
**历史上确实发散过**，只是发散期集中在投影数学被主动调整的那段窗口。若这部分数学再次进入活跃期
（docking 迁移改动视口语义、新增镜头类型等），须重新打开这条裁决——它所依赖的低变动率前提届时
不再成立。

**套件范围的裁决（owner，2026-08-07）。** 原先 N=20/50/100 的预算提案被两个正交轴取代——层次
（T1 必保 / T2 其次 / T3 更全面）× 可写性（今天可写 / 需先动生产侧 / 已满足 / 已证伪）——因为三份
名单本来就是累加的（N=50 是「N=20 再加 30」，N=100 是「N=50 再加 50，不引入新的不变量形态」），
分层要去掉的只是把一个数字钉死在层边界上这件事（「为什么是 20 不是 25」）。**范围：写 T1 + T2。**
T1 原本 20 条候选不变量中：1 条（上述往返恒等式）因结构性不可测被划掉；9 条已满足，含刚发现的
手性一致那条；2 条被一个属于预览生命周期线、而非本线的可观测 seam 卡住；剩下 8 条今天确实还没写——
这 8 条动手前都须重新对着现有套件核实一遍，因为往返恒等式那条已经证明本次分析给出的「未写」标签
本身可能是陈旧的。**退役不是一个单独立项的任务，也从不整层一次性发生。** 它附着在每一条真正落地
并被证明覆盖了某个实例用例的不变量上：可见性谓词落地时退役 8 个 `visibility_*` 用例，是因为每一个
都过了红态探针；回退不变量落地时退役对应的逐字段默认值断言，同理。不存在「721 → N」这种一次性
切换——删除某个实例用例的授权只能来自指向那个具体用例的红态探针或等价性证明，从不来自层次标签。
三条边界原样延续：上文的压缩数字（37%，区间 30–50%，白盒实证 28%）只支持补洞维度，不支持覆盖面
维度——覆盖面维度没有证据，且几乎肯定不会因此变好；T1 单独放弃了六类失效模式（overlay label 放置、
具体 widget 交互、filter SOP 语法、颜色/合成、非法输入退化、多显示器/窗口尺寸），本身不构成可上线
的套件。

#### §4.8.2 composition-correctness 拆分及其行数结果（2026-08-10）

`composition-correctness`（§1.2）与收窄后的 `gui`（§1.7）都是把上文诊断贯彻到底得到的结构性
结论：`functional` 那个三分残差定义（控件行为、文件操作、交互——三件只靠"不是视觉、不是响应"
拴在一起的不相关的事）被追到了根，`gui` 被重写成正定义，而那些早已在做跨单元断言、却被归档在
一个单单元层名下的用例，得到了属于自己的一层，而不是一段更长的告诫段落。

与这次拆分同时进行的是一个更硬的目标：把套件的去注释代码行数从 21,336 行的基线压低 30%
（≤14,900 行），冲刺目标 40%（≤12,800 行）。重写落在了 **19,485 行 / 64 个文件 / 556 个用例，
分布在现覆盖 `src/gui/` 的三层之上**——降幅 8.7%，两个数字都未达到。这被如实记为**未达成**，
由 owner 裁决接受，理由有两条，而不是靠重新定义目标或放宽计入口径：

1. **这次重写付出的覆盖扩张成本是一次性的、不可谈判的。** 按面板而非按历史文件重写，暴露出
   四个 `src/gui/` 单元——合计约占 `src/gui/` 自身代码行数的四分之一——在这次重写之前**零**
   单元级覆盖；补上这个洞正是套件变大的原因，不是冗余的实例测试，且不会再复发。
2. **剩下的缺口是样板密度，不是重复断言。** 重写之后对 `test/gui/` 的实测审计发现，大约三分之
   二的行是场景搭建与夹具准备而非断言，且这些准备工作里有相当一部分跨用例近乎相同——这种形状
   要继续压缩，得靠提取共享夹具，而不是靠删覆盖。owner 把这项提取工作与一项可读性代价放在一起
   权衡（过度提取夹具会让单个用例更难独立读懂），并把它划出了本次重写的范围，而不是在行数目标
   压力下强推。

这次重写的另一个结构性产出——一道关闭了本次工作中反复出现的同一缺陷形状的门禁——记在 §4.9。

### §4.9 第四道门禁：致命断言不得直接坐在可重复执行的作用域里

`scripts/check_loop_fatal_asserts.py` 是继 `check_policies.py`（全树）、`check_new_refs.py`
（散文）、`check_new_gui_tests.py`（`AGENTS.md`"Testing and Platform Notes"）之后的**第四个**
diff-scoped 入口。与另外三个同一条纪律：**检查器就是规则，不是规则的近似**——0 命中是改动干净
的证据，不是对手写评审清单的补充。它在 CI 的 `new-refs` job 里对 PR 相对 merge-base 运行，在
pre-commit hook 里对暂存 diff 运行，扫描 `test/` 下新增的行（不限于 `test/gui/`——这个缺陷形状
不是 GUI 专属，只是在 GUI 套件里格外密集）。

**它守的规则**：在一个会重复执行的作用域里，一次非中断式的错误报告之后，不得再有代码继续驱动
同一个测试上下文。具体说：一个 `for` 循环体里若直接调用致命断言（gtest 的 `ASSERT_*`，或本仓
ImGuiTestEngine 的 `IM_CHECK*`），会在第一行失败时就中止**整个外层函数**——把它后面每一行的
结果都悄悄藏起来，这是测试本身的正确性缺口，不是风格问题。`IM_ERRORF`（本套件的非致命报告）
有一个镜像失效：因为它不返回，循环若在它之后继续跑，就会继续驱动一个已经无效的 UI 状态，把
假红归咎到错误的那一行。

这条规则在引入它的这次重写过程中泛化了四次，每一次关闭的都是同一条规则的下一个自由度，而不是
一个新的、不相关的缺陷：句法形状（致命断言直接写在循环体里）→ 致命性（`IM_CHECK*` 是致命的，
`IM_ERRORF` 看起来安全，实则通过不返回而非提前返回共享同一种失效）→ 顺序（循环在第 N 次报错
后仍在第 N+1 次继续驱动，与同一次报错内继续驱动一样会级联——循环会绕回来）→ 绑定（一个重写过
的 lambda 把它的 `ImGuiTestContext*` 参数改了名，脱离了扫描器最初匹配的字面标识符，但它依然在
驱动这个上下文；扫描器现在绑定的是宿主 lambda 实际的参数名，绝不是硬编码的标识符）。一个停在
这些泛化之一的检查器，比没有检查器更危险：一次干净的扫描结果会被读作"这里不存在这种缺陷形状"，
而一条已知有误的阴性对照，恰好就在它本该抓住的输入上，制造了这种虚假的信心。

---

## §5 物理布局命名约定

迁移（270.3–270.5）时三套命名系统必须保持对齐：

- **CMake target**：目标态引入按 purpose 命名的 target 取代扁平 `unit_test`。**命名模式：
  `<layer-snake>_test`**（snake_case，沿用现有 `unit_test` / `integration_test` 约定）——如
  `unit_correctness_test`、`parity_test`、`golden_analytic_test`、`composition_correctness_test`；
  GUI 层的 target 为 `gui_test`（270.5 由 `LumiceGUITests` 重命名而来）。是否再按 subsystem 进一步
  拆 target（单个 `unit_correctness_test` vs `unit_correctness_core_test`+…）由 **270.3 裁定**，
  但上述*模式*在此固定，防命名漂移。
  **例外——写成一般化规则，而非逐个具名的特例清单**：当一个 target 的存在理由是**链接边界**
  而非层/subsystem 边界时，它按链接依赖命名，不套 `<layer-snake>_test`。`gui_test` 是第一个
  实例（它是 `gui` 层的 target，但名字说的是"链 GUI 的那个"）；`gui_unit_test` 是第二个——
  严格套模式应叫 `unit_correctness_gui_test`，那个名字说的是它与 `unit_correctness_test`
  **共有**的层，反而藏起了唯一真正不同的东西：`lumice_gui_obj` 依赖。层身份由 CTest LABEL
  承载（选择器本来就从那里读），所以 target 名可以腾出来表达链接事实。
- **CTest LABELS**：目标态为每层加 purpose 轴 label：`unit-correctness`、`composition-correctness`、
  `golden-analytic`、`parity`（该 LABEL 是 `parity-cross-backend` 层名的缩写——全名对 CMake 过长；
  这是**唯一**缩写 label，其余层名**不可**同样截断——`unit-correctness` 不可缩成 `unit`，会与旧
  机制轴 label 冲突，`composition-correctness` 同理不可缩成 `composition`）、`performance`、`gui`、
  `regression-sentinel`。现状 label 是机制轴（`unit` / `integration` / `gui`）。
  `ctest -L "unit-correctness|composition-correctness|parity|golden-analytic"` 是
  `scripts/test.sh` 的 `quick`/`full`/`pr` 模式用来一次选中这四个非平铺层的选择器，不需要任何
  per-target 知识。
- **pytest marker**：`slow`（需 shared-lib 构建；排除出 CI 快路径）是**唯一**的运行节奏 marker，
  保留。另一个 marker `heavy` 注册到 2026-08-11 为止：它命名的是「本地/nightly 跑」这一从未有任何
  workflow 提供的节奏，因此它那三条测试哪里都没跑过；测试与注册已一并删除。层/subsystem 在目标态
  用目录 + marker 表达。
- **`addopts = ["-m", "not slow"]`（`pyproject.toml`）**：裸 `pytest` 被钳定到 fast 子集，让上文
  "裸 pytest = e2e fast 子集" 的说法从需要调用方记住的约定变成结构性事实。加之前，裸 `pytest`
  实测 collected 全集 166 个（而非 `-m "not slow"` 选出的 81 个 fast 子集），尽管 `AGENTS.md`
  一直把它文档化为 fast-only；runner 历史样本显示调用方**一贯以为**自己在跑 fast 路径、实际却
  跑了全集——这正是选择「门禁」而非「改文档」的原因（提醒型约束抓不住调用方本来就不知道自己
  错了的失效模式）。命令行 `-m` 会整体覆盖 addopts（不是 AND 组合），所以 `-m ''` 仍是取全集的
  逃生口，`-m slow` 也仍不受影响地选中 CI 慢腿。任何依赖旧「无 `-m`
  = 跑该路径下全部」默认语义的 pytest 调用点都需要补上显式 `-m`——见
  `doc/gpu-remote-cuda-build-testing.md` 里 CUDA parity recipe 的修正案例。

**迁移锚 checklist（每次 270.3–270.7 移动强制）。** CI 硬编码了以下锚点；任何 rename/move/marker
改动漏掉一条都会让 CI 变红：

- [ ] `.github/workflows/ci.yml` 中 `ctest -R LumiceUnitTest`（及任何 `-R`/`-L` selector）在
      target 改名后仍能解析。
- [ ] `ci.yml` 中 pytest 路径参数仍能解析——E2E-Slow matrix 按文件名引用具体文件
      （parity 腿 `test_metal_exit_seam_parity.py`；rest 腿 `--ignore=...`）。
- [ ] marker selector `-m "not slow"` / `-m slow` 仍选中预期集合。
- [ ] 参考图路径（`test/e2e/references/`、`test/gui/references/`）及其 `.gitignore` un-ignore 规则
      随读取它们的测试一起移动。
- [ ] `release.yml` 不受影响（它不跑测试）——确认，别假设。

---

## §6 现存测试 → 八层（exhaustiveness 映射）

下表证明八层覆盖整个现存套件、**无孤儿**，并作为 270.3–270.7 的迁移源。**迁移约束**列标出不可
随意移动/删除的健康项。

> 目标态目录规则（为 270.3 解决"子目录还是平铺"的歧义）：有**自然 subsystem 划分**的层
> （`unit-correctness`、`composition-correctness`、`parity-cross-backend`、`golden-analytic`）
> 用 `test/<layer>/<subsystem>/`；subsystem 边界模糊的层（`e2e-correctness`、`performance`、
> `regression-sentinel`）**平铺**为 `test/<layer>/`，subsystem 用 marker/label 编码。`gui` 层用
> `test/gui/<tag>/`（功能/视觉/响应）。

| 层 | 目标态路径 | 现状 C++（unit/integration） | 现状 e2e（pytest） | 现状 gui | 迁移约束 |
|----|-----------|-------------------------------|---------------------|----------|----------|
| **unit-correctness** | `test/unit-correctness/<subsystem>/` | `test_math`、`test_geo3d`、`test_optics`†、`test_crystal`、`test_rng`、`test_queue`、`test_threading_pool`、`test_color_space`、`test_json`、`test_filter`、`test_filter_spec`、`test_config_snapshot`、`test_render_config`、`test_sim_data`、`test_simulator`、`test_cpu_info`、`test_raypath_segments`、`test_reduce_raypath_audit`、`test_c_api`、`test_exit_records`、`test_proj`(integration)、`test_integration_main`；`gui` subsystem 第一个 target `unit_correctness_test`（只链 `lumice_obj`，header-only 可达）：`test_axis_presets`、`test_filter_sop_grammar`、`test_gui_widget_rules`（已吸收原先独立的 `test_slider_mapping` 与 `test_window_sizing` 两个文件——`SliderMapping` / `WindowSizingTest` 两个 suite 现住在这里）、`test_user_defaults_eligibility`；第二个 target `gui_unit_test`（见右）：`test_defaults_diff`、`test_state_reconcile`、`test_preview_renderer`、`test_export_params`、`test_crystal_renderer`、`test_render_handedness_guard`（渲染 `right=+az` 跨实现手性 guard——绝对屏侧判据，与 `test_projection` 的 golden 绝对列钉子配对；它是唯一需要 `lumice_gui_obj` 与 `lumice_obj` 一起链接的用例，因而 `gui_unit_test` 是它唯一可能的归宿）、`test_axis_absent_alignment`、`test_user_defaults`、`test_render_bg_logic`、`test_sampling_density_stats`、`test_server_poller`、`test_face_number_overlay`、`test_overlay_labels`、`test_composite_preview`、`test_color_window_logic`，外加 `gui_unit_test_env`（在任何用例跑之前安装本 target 的个人默认值隔离基线） | — | — | 本层的 `gui` subsystem 目录由**两个**按链接边界拆开的 CMake target 共用（§2）：`unit_correctness_test`（只链 `lumice_obj`）与 `gui_unit_test`（额外链 `lumice_gui_obj`，无窗口）。两者同挂 LABEL `unit-correctness`，所以用例在两者之间搬家**不需要**改任何 `-L` 选择器——但**需要**在两个 `add_executable` 的源文件列表之间搬，且 `gui_unit_test` 只在 `if(BUILD_GUI)` 门内存在。 |
| **composition-correctness** | `test/composition-correctness/<subsystem>/` | `gui` subsystem，单一 target `composition_correctness_test`（§2 的同名警示）：`test_document_roundtrip_chain`、`test_document_defaults_chain`、`test_document_switch_chain`、`test_legacy_document_chain`、`test_scene_commit_chain`、`test_filter_reconstruct_chain`、`test_raypath_color_document_chain`、`test_run_lifecycle_chain`、`test_run_warning_chain`、`test_user_defaults_chain`、`test_field_editor_chain`、`test_edit_modal_chain`、`test_preview_projection_chain` | — | — | 最新的一层（§1.2）；今天只有 `gui` subsystem 被填充。文件名是链条的主题，绝不是某个单一 `src/` 单元的名字——见 §1.2 的命名规则。 |
| **golden-analytic** | `test/golden-analytic/<subsystem>/` | `test_projection`†、`test_optics` 内闭式段†、`MultiMsContinuationNormalIncidence`（在 `test_metal_trace_parity.cpp`，2-MS 解析锚） | — | — | †逐文件确认"解析真值 vs unit-correctness"边界后才拆出 |
| **parity-cross-backend** | `test/parity-cross-backend/<subsystem>/` | `test_metal_trace_parity`、`test_metal_root_gen`、`test_metal_trace_backend`、`test_metal_filter_match_parity`(.mm)、`test_cpu_trace_backend` | `test_metal_exit_seam_parity`、`test_metal_batch_invariance`、`test_device_gen_default_path`、`test_cpu_backend_route`、**projection 子系统**（315.5）：`test_metal_projection_parity`、`test_cuda_projection_parity`（共用 `_projection_battery.py`） | — | `_parity_metrics.py` 是 parity 指标单一真源——**DO_NOT_MIGRATE_INDEPENDENTLY**（与其依赖者一起移）。能量守恒 + 跨 seed 双门是 267.3 补强——**勿删**。`test_metal_batch_invariance` 的能量守恒 `xfail` 是**合法的**（worst-case drain 未落地）——勿当 bug "修"掉。`_projection_battery.py` 是共享的 per-projection battery（oracle = legacy CPU）——与 `test_{metal,cuda}_projection_parity` 一起移。 |
| **e2e-correctness** | `test/e2e-correctness/`（平铺） | — | `test_smoke`、`test_cli`、`test_raypath_equivalence` | — | — |
| **performance** | `test/performance/`（平铺） | （无独立 C++ perf target；CI `Benchmark` 步骤跑 `--benchmark`） | `test_metal_throughput` | — | — |
| **gui** | `test/gui/<tag>/`（功能/视觉/响应） | — | `test_metal_gui_acceptance`（G4；gui 层，走 pytest harness） | `functional/`：`test_background_overlay`、`test_color_window`、`test_defaults_panel`、`test_edit_modal`、`test_entry_management`、`test_export`、`test_file_ops`、`test_filter_editor`、`test_gui_face_number_overlay`、`test_gui_overlay_labels`、`test_gui_preview_animation`、`test_gui_sim_smoke`、`test_log_panel`、`test_overlay_controls`、`test_preview_texture`、`test_preview_viewport`、`test_run_lifecycle`、`test_scene_controls`、`test_shell_chrome`、`test_status_bar`、`test_view_display_controls`；`visual/`：`test_gui_capture_smoke`、`test_gui_defaults_panel`、`test_gui_lens_projection`、`test_gui_modal_layout`、`test_preview_pixels`；`responsiveness/`：**`test_gui_perf`**；harness（平铺于 `test/gui/`）：`test_gui_main`、`test_screenshot`、`test_gui_shared` | `test_gui_perf` oracle = 绝对帧预算（§4.4），非吞吐对 legacy。`functional/` 不再有一个以 `interaction` 命名的杂物间——它原来的内容现已按上表被驱动的窗口/面板拆开，或者在用例不需要真实帧时移出到 `unit-correctness`/`composition-correctness`（§1.7）。 |
| **regression-sentinel** | `test/regression-sentinel/`（平铺） | — | `test_capi_sentinel_overflow`、`test_ms_filter_leak`、`test_errors` | — | `test_capi_sentinel_overflow` / `test_ms_filter_leak` 用 issue 复现守真 bug——**勿改场景**。`test_ms_filter_leak` 也与 parity 相关；其**主** purpose 是 sentinel（多 purpose → 按主 purpose 归类）。 |

**多 purpose 裁决规则**：一个测试服务多个 purpose 时，按其**主** purpose 归类（最直接守护其回归的
那个 bug/属性），并在注释记次要 purpose。例：`test_ms_filter_leak` → `regression-sentinel`（主），
parity 相关（次）。

**健康项——勿过度清理（合并"勿动"清单）**：`_parity_metrics.py`（单一真源）、能量守恒 + 跨 seed
自洽双门（267.3 corr-blind 补强）、`test_capi_sentinel_overflow.py` 与 `test_ms_filter_leak.py`
（issue 复现哨兵）、以及 `test_metal_batch_invariance.py` 中合法的 `xfail`。

> **legacy CPU 红线**：legacy CPU 是 parity ground truth（§1.4、§4.2）与 perf 分母（§1.6、§4.1）。
> 它及其测试在任何层都**绝不**是清理目标。
