# 设计讨论：C API 的层次混杂与产品线扩张方向

> 性质：**设计讨论记录**，不是定稿规范，也不是产品路线图。记录一次 owner 与 assistant 的架构讨论：
> 起点是"要不要把 core 和 gui 拆成两个仓库"，结论是这个问题问错了层次，真正的发现是
> **今天的 C API 把三个不同高度的东西混在了一个平面上**。
> 关联：`doc/c_api.md`（如何*使用* API）、`doc/capi-lifecycle-architecture.md`（C API 的内部不变量与生命周期状态机）、
> `doc/architecture.md`（as-built 三层：Config / Server / Core）、`doc/seam-design.md`（host/device seam，另一条正交的缝）。

---

## 0. 缘起

当前代码基本分为 core 与 gui 两部分，且边界是真实的：`src/gui/` 对 `core/`、`config/` 的
`#include` 数为 **0**，产品可执行文件只链接 C API 库（`src/gui/CMakeLists.txt:125`），
并由 `scripts/check_policies.py` 的 API boundary 规则加门禁。于是自然会想到下一步：
把 core 拆成独立仓库，让它更专注仿真本身与效率，前端（GUI / CLI / 其他程序）随便换。

讨论的结论是：**这个动作解决不了它想解决的问题**，因为想要的解耦性质已经成立；
而当扩张方向被摊开之后，暴露出来的是另一个更深的问题——今天叫 "core" 的东西，
以及暴露它的 C API，其实是三个高度压在一起的产物。当前只做冰晕仿真这一条产品线时，
这个混杂完全看不出来；一旦出现第二条产品线，它会立刻变成阻塞点。

---

## 1. 核心发现（一句话）

**当前 C API 不是"光线追踪引擎 API"，而是"冰晕仿真应用 API"，中间还掺进了一批为
GUI 编辑器服务的 schema 助手函数。三个高度共用一个平面，没有任何标记区分。**

---

## 2. 证据：三个高度，一个平面

| 高度 | 性质 | C API 中的代表（`src/include/lumice.h`） |
|---|---|---|
| **L0 引擎** | 与冰晕无关的通用能力：几何、求交、折射、采样、后端调度 | *C API 中没有独立出口*——只能透过 L1 间接触及 |
| **L1 冰晕域模型** | 晶体族、光路、朝向分布、天空角空间投影、filter | `LUMICE_SceneAddCrystal`:717、`LUMICE_SceneSetLightSource`:744、`LUMICE_SceneAddScatterLayer`:730、`LUMICE_SceneAddFilter`:721 |
| **L2 编辑器支撑** | 服务于"有一个图形界面在编辑这份配置"这件事，与仿真执行无关 | `LUMICE_ShapeScalarSyncKeyName`:1136、`LUMICE_AxisScalarKeyName`:1170、`LUMICE_ShapeIndicesKeyName`:1152、`LUMICE_ShapeWedgeAngleKeyName`:1146、`LUMICE_IsLegalFace`:1113、`LUMICE_IsShapeScalarApplicable`:1123、`LUMICE_ValidateRaypathText`:1186、`LUMICE_GetCrystalMesh`:1091、`LUMICE_MaxFov`:1209 |

L2 那一组最能说明问题：其中四个函数的返回值是 **JSON 键名字符串**。它们存在的唯一理由，
是让编辑器与序列化格式共享单一权威（见 PR #230）——这是一个好设计，但它是
"编辑器与配置格式之间的契约"，不是"引擎对调用者的契约"。今天它和
`LUMICE_CommitScene` 并排躺在同一个头文件的同一个平面上。

源码侧同样是混的。`src/core/` 目录里 `geo3d` / `geo3d_closedform` / `math` / `optics`
偏 L0，而 `crystal` / `raypath` / `lat_lut` / `scatter_accum` / `projection` / `filter_spec`
是 L1；它们共处一个目录、编进同一个 object library、通过同一个 API 暴露。

### 2.1 一个具体的窄化例子：光源

`LUMICE_SceneSetLightSource(scene, sun_altitude, sun_azimuth, sun_diameter, spectrum)`
（`src/include/lumice.h:744`）——光源在 API 层就是"方向 + 角直径"，**没有位置**。
这不是疏忽，而是无限远光源假设的正确表达；但它把这个假设固化在了对外契约上，
而不是固化在 L1 内部。

顺带一个易撞名的历史包袱：`LUMICE_SceneSetSimParams(scene, int infinite, ...)`
（`:747`）里的 `infinite` 指的是**光线数量**无限（`ray_num: "infinite"`，
见 `src/server/c_api.cpp:1077-1078` 与 `src/server/c_api_internal.hpp:71`），
与光源距离无关。将来若要在 API 上表达光源距离，这个词位已被占用。

---

## 3. 三个扩张方向，各自压在哪道缝

owner 提出的三个扩张方向，压力点各不相同，**没有一条落在 core|gui 这道缝上**：

| 方向 | 真正的压力点 | 与仓库拓扑的关系 |
|---|---|---|
| **① 更多终端形态**（如纯网页 / 浏览器沙盒） | 可移植性与阻塞模型：线程模型、GPU 后端替换、文件 IO、C API 是否有隐含同步阻塞语义 | 无关。这是**构建目标**问题。现有的 accumulator-consumer + poller 拉取式取结果本来就是非阻塞形态，对沙盒环境是有利的地基 |
| **② 更多产品线**（如有限距离光源 / 专项光路分析） | **L0 与 L1 的分层**：产品线专属逻辑要长在通用引擎之上，而不是长在冰晕域模型之上 | 无关，且**方向相反**：需要的是从 L1 下面切出 L0，不是把 L0+L1+L2 整包搬到另一个仓 |
| **③ 正式发布动态库给第三方** | 契约与兼容政策：semver、ABI 稳定性承诺、符号可见性、install/export、废弃流程、契约测试 | 无关。单仓发布库是主流做法 |

---

## 4. 顺序约束（本文最有行动价值的一条）

> **方向 ② 决定 API 应该长什么样；方向 ③ 一旦发生就把 API 冻住。**

因此"先把动态库正式发布，再慢慢想产品线"是把顺序做反了：那会把
"冰晕应用 API + 编辑器键名助手"这个当前形态锁给第三方，而它恰好是方向 ② 最需要重切的部分。
这与既有纪律一致——没有正式发版就不要提前背兼容包袱。

同理，**拆仓也是一种提前冻结**：跨仓的 API 比仓内的 API 贵一个数量级去改。
在还不知道 L0/L1 该切在哪之前拆仓，等于用最贵的方式把当前这道缝（L1|L2）宣布为最终答案。

---

## 5. 为什么"拆成两个仓库"不是答案

1. **想买的性质已经成立**。GUI 侧零 core/config include、只链 C API、并有门禁保障。
   拆仓是用贵得多的机制去换一个已经成立的性质。
2. **跨树改动占比很高，且不是脏耦合**。截至 2026-08-01 的近 12 个月，已合并的代码 PR 中
   core-only 71 个、gui-only 50 个、**两边都动 71 个（192 个中占 37%）**。
   看这些 PR 的主题（自定义光谱、光路染色、filter 表达力提升、Scene 句柄化、auto EV）就知道，
   它们是"core 长出一个能力 → 必须在 UI 上有出口才算交付"的功能纵切，是产品形态决定的。
   拆仓会把这 37% 变成两仓 lockstep：发版、bump 依赖、两次 review、两次 CI，
   并且 `git bisect` 跨不过那道缝——而本项目大量依赖跨层白盒定位。
3. **发布形态是单一产品**。`.github/workflows/release.yml` 一次产出 CLI 与 GUI，同 tag 同签名，
   目前不存在需要独立节奏的 core 消费者。

### 5.1 唯一一处边界确实没守住（可独立行动）

`test/gui/CMakeLists.txt:54`：`gui_test` 链接 `lumice_obj`（core 内部对象库）而非 `lumice`；
`test/gui/functional/test_render_handedness_guard.cpp:41-46` 直接 include 了 6 个 `core/`、`config/` 头。

产品遵守 C API，测试不遵守。后果是：**今天并不知道 GUI 侧能否只靠 C API 活下来**，
测试替产品兜住了缺口。这一处收紧（把该 guard 移到跨后端一致性测试层，或给它一个有名字的显式豁免）
既是独立有价值的动作，也是任何未来分层/拆分的前置条件。

---

## 6. 建议的动作序列

### 现在就做（成本低、不可后悔）

1. **在头文件里把三个高度显式分区**：仿真执行（Server / Scene / Commit / `Get*Results`）
   与域 schema、编辑器支撑（`*KeyName`、`IsLegalFace`、`ValidateRaypathText`、
   `GetCrystalMesh`、`MaxFov`）分节标注。价值不在美观，在于**将来发布时可以选择只发布哪一层**。
2. **暂不正式发布动态库**；若要发布，明确标注 experimental、不承诺 ABI。
3. **加一个"第三方消费者"CI 小样**：只用 install 出来的头文件 + 库编译一个几十行的 C 程序跑一次仿真。
   它同时是方向 ① 与方向 ③ 的探针，成本比拆仓低两个数量级；它红了说明 API 表达力不足
   （PR #224 的句柄化过程中已两次暴露此类潜伏缺陷）。

### 值得各花一次探索（不紧急，但正是"不能等需求到眼前"的那类）

4. **产品线触点勘察**：不写产品代码，只回答"要支持一条假设不同的产品线，
   core 的触点有多少、分布在哪"。产出是一份触点清单，它会直接指出 L0/L1 的缝该切在哪。
   估算 LOC 之前先数触点——这类估算容易高估 2–3 倍。
5. **沙盒/网页环境可行性勘察**：只测 core 能否在目标工具链下编出来并跑通一次仿真，不做 UI。
   一次性暴露线程、阻塞、文件 IO 的真实边界。

### 拆仓的触发条件（写清楚，以便将来机械判断）

- 出现**真实的外部 core 消费者**，且其发布节奏与本产品不同步；
- core 与前端需要**不同的授权 / 开源策略**；
- **第二个人或团队**长期只负责其中一侧。

并且：当 L0 引擎层真的被切出来、并有了自己稳定的 API 时，**该拆的是 L0，不是 core|gui**；
到那一步，拆不拆已退化为一个无关紧要的打包决定。

---

## 7. 诚实边界

- **产品线的具体设计不在本文范围内**。owner 对新产品线（如街灯等有限距离光源场景）已有远比本文深入的构想，
  并指出 **core 侧可复用的部分比本文推断的多得多**。本文只记录 API 分层这一侧的分析，
  不对产品线的技术形态下任何结论；§2.1 的光源例子仅用于说明"假设被固化在对外契约上"这一现象，
  不构成对该产品线实现路径的判断。
- **本文的分层命名（L0/L1/L2）是分析工具，不是既有代码结构的命名**。目前代码中不存在这三层的物理边界，
  这正是本文要指出的事情。
- **§6 的动作序列是建议，未经立项评审**。其中第 1、2、3 项是低成本且可逆的；第 4、5 项是探索性质，
  其价值取决于产品线方向的确定程度，应由 owner 决定时机。
- §5 的 37% 数字来自对合并提交的机械分类（按 PR 的 diff 是否同时触及 `src/gui/` 与
  `src/{core,config,server,util,include}/`），口径为"已合并的代码 PR"，不含纯文档/测试/CI 的 PR。
