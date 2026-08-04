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
- **projection 子系统（315.5）**：`test/parity-cross-backend/backend/test_{metal,cuda}_projection_parity.py`
  对每种 LensType 做一次 legacy-oracle 对照，使**全部 11 种投影**（现经共享
  `src/core/shared/projection_shared.h::ProjectExitToPixel` 在 Metal + CUDA 上渲染）保持 cross-backend
  等价，并确认每种类型真走 GPU、不再静默回落 legacy CPU。二者共用共享 battery
  `test/e2e/_projection_battery.py`。

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

### §1.6 `gui`

- **目的**：GUI 功能正确、视觉正确、响应及时。层内三个 tag：**功能**（控件行为、文件操作、交互）、
  **视觉**（渲染输出对参考）、**响应**（实时循环的交互交付——帧间隔、commit→首次 upload 延迟、
  **以及 steady-state / slider-drag 场景下 rays/restart、upload_rays 等光线交付量**）。光线交付量
  *反映*吞吐，但其 oracle 是 GUI 交互循环、而非对 legacy CPU 的比值——故归此层而非 `performance`
  （见 §4.4）。
- **oracle**：imgui test engine 驱动 app；**视觉**对 tracked 参考图断言（PSNR，每场景阈值在
  `_thresholds.json`）；**响应**对绝对帧延迟预算断言；**功能**断言控件/状态结果。
- **阈值约定**：视觉 = 每场景 PSNR（N 次随机渲染的 mean−4σ，见 AGENTS.md lens_proj 再生）；响应 =
  绝对延迟预算；功能 = 精确。
- **节奏**：`PR`，外加 CI 每次 push 在 `xvfb-run` + llvmpipe 下跑的那两个参考图组（§4.6）。
  需要显示服务器，除非用 `LUMICE_SKIP_GUI_TESTS=1` 跳过。
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
>
> 这**一个**目录由**两个** CMake target 编译，而这条分界是**链接边界**，不是层或 subsystem 边界：
> `unit_correctness_test` 只链 `lumice_obj`，装 header-only 的那一半；`gui_unit_test` 额外链
> `lumice_gui_obj`，装会调用 `file_io.cpp` / `user_defaults.cpp` 的那一半（如
> `test_defaults_diff.cpp`）。同目录、同 `unit-correctness` LABEL，只有链接行不同，所以
> `ctest -L unit-correctness` 两个都选中。`gui_unit_test` 不开窗口、不建 GL 上下文；需要真实出帧的
> 用例应去 `gui` 层（`test/gui/`，target `gui_test`）。

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

§1.6 与 §5 按"是否需要活的帧"来放置一个 `src/gui/` 测试。这个放置决定了该测试**在 CI 里跑不跑**：
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

亏空恒为 128 条光线一批的整数倍，机制在 `server_poller.cpp` 里看得见：只有当这一轮 poll 携带了新的
snapshot generation、或者走了终帧救援分支时才会重读统计（:269、:281），而观察到
`LUMICE_LIFECYCLE_COMPLETED` 的那一轮 poll 会在自己末尾把 poller 自暂停（:501-504）。:299 处
「统计推迟一轮由 carry-forward 吸收」的推理，在推迟恰好落在最后一轮时没有下一轮可供吸收。软件渲染让
每次 `ctx->Yield()` 都要光栅化一整帧，于是同一场景用约 169 帧而不是约 1200–2000 帧跑完，poll 序列被
压缩到那个窗口被例行命中——核越少越糟，所以在 CI runner 上比在工作站上更糟。

这是 GUI 在一次运行结束时**报告的数字**里的真缺陷，不是测试 flake；修它属于生命周期不变量所在之处
（`doc/gui-preview-lifecycle-architecture.md`），不属于这里。在它被修好之前，把 `lens_proj` 接进
required check 会人为制造约 60% 的假红率——那等于用手把本节存在的目的（终结那个习惯）重新造一遍。
**那个缺陷修好之后，把 `lens_proj` 加进 CI 步骤的 `--filter` 就是全部改动**：harness、LFS 拉取、
"它真的跑了"断言都已就位。

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
的代码路径来设 exposure，而 `test/unit-correctness/gui/test_ev_auto.cpp` 用四条确定性断言直接验证
auto-EV 的计算本身，且仍在三平台 CI 上跑。把仿真轴场景记到 `test_smoke.py` 名下、把 auto-EV 管线场景
记到 `test_ev_auto.cpp` 名下之后，`auto_ev` 唯一独占的领地只剩 `overlayAuxLines()` 画的 zenith/nadir
标记与坐标网格——一景的量。那一景，`overlay_ea`，被保留下来并迁入了 `lens_proj`，没有跟着其余场景
一起退役。

---

## §5 物理布局命名约定

迁移（270.3–270.5）时三套命名系统必须保持对齐：

- **CMake target**：目标态引入按 purpose 命名的 target 取代扁平 `unit_test`。**命名模式：
  `<layer-snake>_test`**（snake_case，沿用现有 `unit_test` / `integration_test` 约定）——如
  `unit_correctness_test`、`parity_test`、`golden_analytic_test`；GUI 层的 target 为 `gui_test`
  （270.5 由 `LumiceGUITests` 重命名而来）。是否再按 subsystem 进一步拆 target（单个
  `unit_correctness_test` vs `unit_correctness_core_test`+…）由 **270.3 裁定**，但上述*模式*在
  此固定，防命名漂移。
  **例外——写成一般化规则，而非逐个具名的特例清单**：当一个 target 的存在理由是**链接边界**
  而非层/subsystem 边界时，它按链接依赖命名，不套 `<layer-snake>_test`。`gui_test` 是第一个
  实例（它是 `gui` 层的 target，但名字说的是"链 GUI 的那个"）；`gui_unit_test` 是第二个——
  严格套模式应叫 `unit_correctness_gui_test`，那个名字说的是它与 `unit_correctness_test`
  **共有**的层，反而藏起了唯一真正不同的东西：`lumice_gui_obj` 依赖。层身份由 CTest LABEL
  承载（选择器本来就从那里读），所以 target 名可以腾出来表达链接事实。
- **CTest LABELS**：目标态为每层加 purpose 轴 label：`unit-correctness`、`golden-analytic`、
  `parity`（该 LABEL 是 `parity-cross-backend` 层名的缩写——全名对 CMake 过长；这是**唯一**缩写
  label，其余层名**不可**同样截断——`unit-correctness` 不可缩成 `unit`，会与旧机制轴 label 冲突）、
  `performance`、`gui`、`regression-sentinel`。现状 label 是机制轴（`unit` / `integration` / `gui`）。
- **pytest marker**：`slow`（需 shared-lib 构建；排除出 CI 快路径）与 `heavy`（slow + 冗余 parity
  变体；按 PR 用 `not heavy` 取消选择）是运行节奏 marker，保留。层/subsystem 在目标态用目录 +
  marker 表达。
- **`addopts = ["-m", "not slow"]`（`pyproject.toml`）**：裸 `pytest` 被钳定到 fast 子集，让上文
  "裸 pytest = e2e fast 子集" 的说法从需要调用方记住的约定变成结构性事实。加之前，裸 `pytest`
  实测 collected 全集 166 个（而非 `-m "not slow"` 选出的 81 个 fast 子集），尽管 `AGENTS.md`
  一直把它文档化为 fast-only；runner 历史样本显示调用方**一贯以为**自己在跑 fast 路径、实际却
  跑了全集——这正是选择「门禁」而非「改文档」的原因（提醒型约束抓不住调用方本来就不知道自己
  错了的失效模式）。命令行 `-m` 会整体覆盖 addopts（不是 AND 组合），所以 `-m ''` 仍是取全集的
  逃生口，`-m slow` / `-m "slow and not heavy"` 也仍不受影响地选中 CI 慢腿。任何依赖旧「无 `-m`
  = 跑该路径下全部」默认语义的 pytest 调用点都需要补上显式 `-m`——见
  `doc/gpu-remote-cuda-build-testing.md` 里 CUDA parity recipe 的修正案例。

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
| **unit-correctness** | `test/unit-correctness/<subsystem>/` | `test_math`、`test_geo3d`、`test_optics`†、`test_crystal`、`test_rng`、`test_queue`、`test_threading_pool`、`test_color_space`、`test_json`、`test_filter`、`test_filter_spec`、`test_config_snapshot`、`test_render_config`、`test_sim_data`、`test_simulator`、`test_cpu_info`、`test_axis_presets`、`test_slider_mapping`、`test_window_sizing`、`test_raypath_segments`、`test_reduce_raypath_audit`、`test_c_api`、`test_exit_records`、`test_ev_auto`、`test_proj`(integration)、`test_integration_main`、`test_defaults_diff`（在第二个 target `gui_unit_test` 里，见右） | — | — | 本层的 `gui` subsystem 目录由**两个**按链接边界拆开的 CMake target 共用（§2）：`unit_correctness_test`（只链 `lumice_obj`）与 `gui_unit_test`（额外链 `lumice_gui_obj`，无窗口）。两者同挂 LABEL `unit-correctness`，所以用例在两者之间搬家**不需要**改任何 `-L` 选择器——但**需要**在两个 `add_executable` 的源文件列表之间搬，且 `gui_unit_test` 只在 `if(BUILD_GUI)` 门内存在。 |
| **golden-analytic** | `test/golden-analytic/<subsystem>/` | `test_projection`†、`test_optics` 内闭式段†、`MultiMsContinuationNormalIncidence`（在 `test_metal_trace_parity.cpp`，2-MS 解析锚） | — | — | †逐文件确认"解析真值 vs unit-correctness"边界后才拆出 |
| **parity-cross-backend** | `test/parity-cross-backend/<subsystem>/` | `test_metal_trace_parity`、`test_metal_root_gen`、`test_metal_trace_backend`、`test_metal_filter_match_parity`(.mm)、`test_cpu_trace_backend` | `test_metal_exit_seam_parity`、`test_metal_batch_invariance`、`test_device_gen_default_path`、`test_cpu_backend_route`、**projection 子系统**（315.5）：`test_metal_projection_parity`、`test_cuda_projection_parity`（共用 `_projection_battery.py`） | — | `_parity_metrics.py` 是 parity 指标单一真源——**DO_NOT_MIGRATE_INDEPENDENTLY**（与其依赖者一起移）。能量守恒 + 跨 seed 双门是 267.3 补强——**勿删**。`test_metal_batch_invariance` 的能量守恒 `xfail` 是**合法的**（worst-case drain 未落地）——勿当 bug "修"掉。`_projection_battery.py` 是共享的 per-projection battery（oracle = legacy CPU）——与 `test_{metal,cuda}_projection_parity` 一起移。 |
| **e2e-correctness** | `test/e2e-correctness/`（平铺） | — | `test_smoke`、`test_cli`、`test_raypath_equivalence` | — | — |
| **performance** | `test/performance/`（平铺） | （无独立 C++ perf target；CI `Benchmark` 步骤跑 `--benchmark`） | `test_metal_throughput` | — | — |
| **gui** | `test/gui/<tag>/`（功能/视觉/响应） | — | `test_metal_gui_acceptance`（G4；gui 层，走 pytest harness） | `test_gui_lens_projection`、`test_gui_sim_smoke`、`test_gui_visual`、`test_gui_render`、`test_gui_bg`、`test_gui_export`、`test_gui_import_export`、`test_gui_interaction`、`test_gui_face_number_overlay`、`test_gui_overlay_labels`、`test_gui_composite_preview`、`test_gui_lifecycle`、`test_gui_sampling_density_stats`、`test_gui_defaults_panel`、**`test_gui_perf`（响应 tag）**、`test_gui_main`/`test_screenshot`/`test_gui_shared`（harness） | `test_gui_perf` oracle = 绝对帧预算（§4.4），非吞吐对 legacy。 |
| **regression-sentinel** | `test/regression-sentinel/`（平铺） | — | `test_capi_sentinel_overflow`、`test_ms_filter_leak`、`test_errors` | — | `test_capi_sentinel_overflow` / `test_ms_filter_leak` 用 issue 复现守真 bug——**勿改场景**。`test_ms_filter_leak` 也与 parity 相关；其**主** purpose 是 sentinel（多 purpose → 按主 purpose 归类）。 |

**多 purpose 裁决规则**：一个测试服务多个 purpose 时，按其**主** purpose 归类（最直接守护其回归的
那个 bug/属性），并在注释记次要 purpose。例：`test_ms_filter_leak` → `regression-sentinel`（主），
parity 相关（次）。

**健康项——勿过度清理（合并"勿动"清单）**：`_parity_metrics.py`（单一真源）、能量守恒 + 跨 seed
自洽双门（267.3 corr-blind 补强）、`test_capi_sentinel_overflow.py` 与 `test_ms_filter_leak.py`
（issue 复现哨兵）、以及 `test_metal_batch_invariance.py` 中合法的 `xfail`。

> **legacy CPU 红线**：legacy CPU 是 parity ground truth（§1.3、§4.2）与 perf 分母（§1.5、§4.1）。
> 它及其测试在任何层都**绝不**是清理目标。
