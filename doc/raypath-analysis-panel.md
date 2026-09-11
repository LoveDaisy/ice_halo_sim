# Raypath Analysis Panel — 设计记录

> Status：设计蓝图（2026-09-11 owner × assistant 讨论收敛，尚未实现）。子任务 2–5
> （core 链 id 携带 → server 直方图 consumer → C API → GUI 面板）据此展开 plan。
> 改「光路分析面板」（raypath-analysis-panel）相关的任何一层前先读本文；本文与 as-built
> 状态的回写由后续收尾任务负责，届时会在本文标注 file:line 落点。

## 1. 问题与形态

需求原本被拆成两半：(i) **识别**——在一张普通仿真图上，回答「这道弧是谁做的」，
需要每像素一个主贡献者 id；(ii) **剥离**——回答「关掉这道亮的，看下面压着什么」，
需要每个分桶（晶体 / filter 组合）的真实辐射量，成本与桶数 N 线性、且是 W×H 的一整张图。
(ii) 的五条实现路线已经挂起：内存墙不是通道装了几个分量造成的，而是「为每个桶预分配
一条全分辨率通道」这个**形态**本身——1024² 下 Y-only 就是每桶 8MB，40 个晶体已经 5GB。
挂起时留下的开放问题是产品问题，不是实现问题：用户到底要多大的 K？有了某种替代方案后，
是否还需要「实时开关」这个具体解法？

owner 在 2026-09-11 提出第三种形态，不再试图同时满足「渲染图」与「逐光路分解」：

> 一个独立的光路分析面板，带自己的 Analyze 按钮；点击发起一轮**专用运行**——这轮运行
> **不渲染**，只按「完整光路」（从光源到相机，跨 MS 层，按对称性逐段约化）聚合能量，
> 按能量降序列出。ROI 三档：用户点了一个点 → 分析该点附近的锥形邻域；没点 →
> 分析可见区域或全天。

这个形态消解内存墙的方式很具体，而且是可拆解的两处耦合各拆一处：

- **N（按光路）× W×H（按像素）→ O(不同光路数)**：耦合的根源是「同一次运行 + 按像素存」——
  N 来自把结果按光路分桶，W×H 来自把每个桶存成一张图，「同一次运行」把两者焊死在一起，
  谁都拆不掉。**专用运行**先把它与渲染运行解耦（不再要求一次运行同时产出两种产物），
  **统计量而非图**再把像素维度拆掉（分桶结果是标量累加器，不是纹理）。两处耦合各消解一处，
  内存不再随 N×W×H 增长，只随「有多少条不同的光路」增长。
- 它是已有的路线 E「不存，需要时算回来」的**泛化**：路线 E 是同一个 consumer 针对同一个桶
  逐次重跑；这里是换一个 consumer（按链 id 聚合而非按像素渲染）跑一次，一次运行就拿到所有
  桶的排序结果，不必逐桶重跑。

## 2. 四条裁决

以下四条均为 **owner 裁决**（2026-09-11），逐条附理由：

1. **后端：v1 只走 CPU 路，且必须显式声明，不是静默 fallback。**
   理由：Metal/CUDA 的生产路径是 device-fused 单引擎，per-ray 记录不回传 host——
   `src/core/backend/metal_trace_backend.mm` 里 `MetalTraceBackend::ReadbackExitRays`
   的实现直接返回空，注释写明「S1 device-fused: kernel 不再物化 per-exit records」。
   Analyze 是用户主动点击的显式动作，等几秒可以接受，而且统计量（不产图）本身收敛得比
   渲染快。「强制 CPU」要作为一个显式的会话属性写下并记日志（复用
   `LUMICE_SetPreferredBackend` / `ResolveGpuRoute` 的既有强制路径，`src/server/server.cpp`
   的 `ResolveGpuRoute` 已经是「按显式选择路由，不是按能力探测静默降级」的先例），
   使得将来给 GPU 补上这条能力时，是「换一份实现」而不是「悄悄改变了语义」。
2. **「光路」= 从光源到相机的完整链，不是单晶体内的一段。**
   例如 `crystal1(1-3-5)-crystal2(3-2)`；单晶体 MS 或单层退化为链长 1，不特判成另一种数据形状。
   理由（机制约束）：现有数据结构拿不到全链——`ExitRayRecord::path` 只装最后一层的面序列，
   `RaypathRecorder`（`rp_`）只装当前晶体那一段——要跨层拼出完整链，需要一个**随光线跨 MS 层
   前向携带的 per-ray 链 id（uint32）**，配合一张 interning 表把 `(父链 id, 本层 crystal_id,
   本层约化后的 segment)` 映射成新的链 id，天然长成一棵 trie；要打印完整链时沿父指针链回溯。
   不携带字节序列本身（太胖，跨层每次拷贝一遍字节数组）、也不携带哈希（丢失打印能力，且有
   碰撞风险）——链 id + interning 表是唯一同时满足「跨层轻量携带」与「事后可读」的形状。
3. **ROI = 方向空间里的一个锥（中心方向 + 角半径），与画面投影无关；角半径用滑杆，且滑杆必须
   display-time 生效。**
   记录阶段按到锥中心的角距离分环累加（不是只记「在/不在锥内」这一个布尔量），滑杆只决定
   显示时求和到第几环——这样调滑杆不需要重跑统计，是纯粹的显示层操作。「可见区域」定义为
   落在当前画幅内（复用 `ProjectExitToPixel` 的画幅内判定）；「全天」不做任何空间判定，
   聚合所有出射光线。
4. **总能量降序排序必有；其它排序方式可以有，但「能量 / 覆盖立体角」这种密度量 v1 不做。**
   理由：每个链 id 对应的累加器只需要是标量（Σ(Y·w)、计数），代价很低；密度排序需要给
   **每个** key 都配一张粗天球栅格才能算出「这条链覆盖了多大立体角」，存储与实现复杂度都
   上升一个量级。v1 先把「总能量降序」这个最基本、最省资源的排序做对，密度排序留作明确的
   后续升级点（见 §6 风险 6、§7）。

## 3. 机制

### 3.1 链 id 前向携带

链 id 是一个新的 per-ray side-car，携带点与既有的 `RayBuffer::components_`
（`src/config/sim_data.hpp` 里的 `std::unique_ptr<uint64_t[]> components_`，
raypath-color 基础设施新增的 per-ray component mask）**完全同一套点**——这不是巧合，
而是硬约束：任何随光线跨层携带的并行状态，必须跟着**所有**同一批 swap/gather/fan-out/续传
交接点走，漏一个就是静默错位、且现有单元测试测不到（这正是 T3/T4 阶段吃过的教训）。
需要一并搬的点：

- `ComponentFanOut`（`src/core/simulator.cpp:690` 附近）——一次打两半时子段继承父段状态。
- `EmplaceBack` 的三个重载（`src/config/sim_data.hpp:112-114`）——批量追加时状态跟着搬。
- `SwapRay`（`src/core/simulator.cpp:1454`、`src/core/backend/cpu_trace_backend.cpp:488`）——
  shuffle 时状态要跟着交换，否则会把并行数组与光线错位关联。
- 续传交接 `init_data[1].EmplaceBack(...)` 之后紧跟的 `SetComponent`
  （`src/core/simulator.cpp:819-822`）——MS 层之间续传时的状态复制点。

风险 3（见 §6）里给出的建议方向是：既然这是第二个同形状的 side-car，值得把
`components_` 与链 id 收进同一个 per-ray side-car 结构，让「漏搬一个传播点」在结构上
不可能发生，而不是把上面这份点位清单再对着链 id 复制一遍——子任务 2 的 plan 阶段裁定
是否采用。

### 3.2 interning 表与逐段对称约化

interning 表把 `(父链 id, 本层 crystal_id, 本层约化后的 segment) → 子链 id` 建成一棵 trie；
「本层约化后的 segment」复用现有的对称约化单一权威，不另写一套：

- `Crystal::ReduceRaypath(rp, symmetry)` / `Crystal::ReduceRaypath(rp, symmetry, sigma_a,
  d_applicable)`（`src/core/crystal.hpp:242` / `:252`）——按 (P, B, D) 对称标志约化一段
  face-index 序列。
- `detail::ReduceBuffer`（`src/core/filter_spec.hpp:141`）——同一套约化规则的原地版本，
  作用于原始 `uint8_t` 缓冲区，供 inline data 与 arena 拷贝共用。

链 id 只在光线穿过一层晶体、生成本层 raypath 之后，用该层约化后的 segment 去查/建表，
而不是缓存未约化的原始面序列——这样同一等价类的光路天然映射到同一条链，链数不会因为
「同一物理路径的不同镜像/旋转变体」被重复计数。打印一条完整链时，从叶子链 id 沿父指针
回溯到根，逐段还原出 `crystal1(1-3-5)-crystal2(3-2)` 这种展示形式。

### 3.3 消费者形态

新 consumer 实现现有的 `IConsume` 接口（`src/server/consumer.hpp:19`，契约细节见
`doc/accumulator-consumer-architecture.md` §2），与 `RenderConsumer` / `StatsConsumer` /
`AnchorConsumer` 一样挂在 `consumers_` 上、接收同一批次数据（组装点见
`src/server/server.cpp:914-926`）。它按链 id 聚合两个标量：Σ(Y·w) 与命中计数，
其中 Y 的计算必须复用 `RenderConsumer` 已有的权威（`SpectrumToXyz` /
`SpectrumToXyzPerRay`，`src/core/color_util.hpp:36` / `:67`），不得自建归一化——
07-07 的 `ComponentBinConsumer` spike（已 revert；结论见
`doc/gui-custom-spectrum-and-raypath-color.md` §"显示模型定案"一节）唯一踩过的坑
就是自造了一套归一化，产出假洋红。

### 3.4 ROI 三档的实现落点

- **锥形**：记录阶段按「出射方向与锥中心方向的角距离」分环累加，环数在记录时固定、
  滑杆只决定显示时求和到第几环之前——记录一次、显示时任意调半径，不重跑。
- **可见区域**：复用 `ProjectExitToPixel`（画幅内判定），落在画幅外的直接丢弃。
- **全天**：不做任何空间判定，所有出射光线都计入。

### 3.5 累加器字段

每个链 id 对应的最小状态是：能量累加（Σ(Y·w)）、命中计数、（锥形 ROI 时）按环分桶的
子累加器。这三类都是标量或定长小数组，不随分辨率或桶数线性增长——这正是本形态相对于
「每桶一张全分辨率图」的内存优势的直接体现。

## 4. 可复用地基清单

以下地基已核实存在、可以直接复用，**不必重建**：

- `ExitRayRecord{dir, weight, path, crystal_id, ms_layer_idx, wl_idx, component_mask}`
  （`src/core/exit_seam.hpp:40` 起）——单层出射记录的既有结构，链 id 作为新字段挂在
  同一份记录上。
- `ProjectExitToPixel`——画幅内判定的既有实现，供「可见区域」ROI 直接调用。
- `src/core/projection.hpp:28-95` 附近的五族反投影（linear / fisheye 四种变体 /
  dual-fisheye / rectangular / globe 的 `*Inverse`）——供 GUI 把「用户在预览上点的一个像素」
  换算成方向空间里的锥中心方向。
- `RenderConsumer` 的 Y 计算权威（`SpectrumToXyz` / `SpectrumToXyzPerRay`，
  `src/core/color_util.hpp`）——新 consumer 的能量累加必须用同一份权威，见 §3.3。
- 07-07 `ComponentBinConsumer` spike 的结论（已 revert，不是可复用代码，只引结论）：
  一次性旁挂 consumer、复用生产路径的 `ProjectExitToPixel`、core 零改的做法是可行的；
  唯一的坑是自建归一化产生假色，详见 `doc/gui-custom-spectrum-and-raypath-color.md`
  相应小节。

## 5. 顺带闭环：行 → 排除 filter → 重跑

列表里的每一行对应一条链；用户可以对某一行点「排除此光路」，生成一个 filter 并触发重跑。
这不是新机制，是现有「排除后重跑」语义的复用——零新增存储，只是把 filter 的生成源头
从「用户手写」换成「从分析结果点选」。v1 范围内，这个按钮**只对单段链**（链长 1，即
单晶体单层）可用：现有 filter 是单层 per-crystal 的物理门，表达不了跨层链的排除语义
（如「排除 `crystal1(1-3-5)-crystal2(3-2)` 这整条链，但保留 `crystal1(1-3-5)` 与其它
晶体的组合」）。这个限制在 UI 上要明确说明，不能让按钮在多段链上显示为可用却生成一个
不达意的 filter。

## 6. 诚实边界

逐条标注归属（owner 裁决 / assistant 推断）：

- **GPU 不覆盖**（owner 裁决，§2 第 1 条）：v1 只有 CPU 路能产出分析结果；Metal/CUDA
  用户点 Analyze 时的行为（报错 / 静默走 CPU 并提示更慢 / 禁用按钮）留给子任务 4 的 plan
  裁定，本文不预判。
- **多段链不可排除**（owner 认可，§5）：「排除此光路」按钮 v1 只对单段链生效，跨层链
  没有对应的 filter 表达能力。
- **能量 ≠ 视觉显著**（owner 裁决，§2 第 4 条的直接推论）：全天 / 可见区模式下，
  random-orientation 场景里弥散、覆盖立体角大的光路总能量可能高于视觉上更显眼的窄亮弧，
  导致排序把「不起眼但铺得广」的链排在「醒目但集中」的链前面。v1 接受这个结果，
  密度排序（能量 / 覆盖立体角）是明确的后续升级点，需要每 key 一张粗天球栅格，
  v1 不做（见 §2 第 4 条）。
- **逐段约化的独立性假设待对照，不当已证**（owner 提出的风险点，assistant 未独立验证）：
  §3.2 的逐段约化假设「各层朝向独立采样 ⇒ 各段等价类相互独立」，需要在子任务 2 的
  plan/实现阶段对照 `doc/raypath-symmetry.md` §2b 逐条核实，本文不代为下结论。
- **两条 CPU 路，行为待核实**（assistant 推断，标记为待核实）：生产环境下
  `LUMICE_BACKEND_CPU` 走 legacy 的 `Simulator::SimulateOneWavelength`（多 worker），
  它今天只打包 `outgoing_d_` / `outgoing_w_` / `outgoing_component_`
  （`src/core/simulator.cpp:1477-1479` 附近），**不产出** `exit_records_`；
  `CpuTraceBackend` 只在 `LUMICE_TRACE_BACKEND=cpu_backend` 环境覆盖下启用
  （`src/core/simulator.cpp:936` 附近的路由判断）。assistant 的推断是：链 id 方案下
  消费者只需要一个 per-ray uint32 + 一张表，所以 legacy 路可能只需要照着
  `outgoing_component_` 的样子多打包一条 `outgoing_chain_id_`，**不必**切换到
  `CpuTraceBackend`。**这条推断未经独立核实，子任务 2 的 plan 阶段必须自行核实**，
  不得当作已核实的地基使用。
- **多 worker 下的 interning 合并未定**（owner 提出的风险点）：每个 worker 一个
  `Simulator`、各自一张 interning 表，consumer 收到的是多张表各自的 id 空间。
  推荐方向是 per-worker 表 + `SimData` 携带本批新增的表增量，consumer 按约化后的
  链字符串（或等价键）跨 worker 合并；备选是分析运行强制 `worker_count=1`
  （固定 seed 时代码里已有先例，`src/server/server.cpp:552` 附近的
  `worker_count = 1;  // deterministic CPU contract`，但会牺牲多 worker 的吞吐）。
  由子任务 2 的 plan 裁定并写明理由，本文不预判。
- **反投影走 C API 还是 `src/util/` 未定**（未决问题，非裁决）：GUI 把点击换算成方向
  需要用到 §4 列出的五族反投影；这些函数今天在 `src/core/projection.hpp`，按
  `AGENTS.md` 的公共 API 边界，GUI 不能直接 include core 头文件。是把它们整体搬进
  `src/util/`（前提是它们是无状态纯几何函数，符合 `src/util/` 的豁免条件），还是新增
  一个 C API 入口，留给子任务 5 的 plan 阶段裁定。

## 7. 开放设计点清单（供子任务 plan 裁定，本文不替它们决定）

- 子任务 2：链 id side-car 是否与 `components_` 合并进同一个 per-ray 结构（§3.1 末尾建议）；
  legacy CPU 路是否需要切到 `CpuTraceBackend`（§6「两条 CPU 路」的待核实推断）；
  多 worker interning 表的合并策略（per-worker 增量 vs 强制单 worker，§6）。
- 子任务 3：ROI 锥形分环的分辨率（环数、角度步长）如何取舍精度与内存；快照产出格式
  与既有 `RenderConsumer` 快照的接口对齐方式。
- 子任务 4：Analyze 运行与渲染运行互斥的具体状态机接入点（`src/gui/sim_state_rules.hpp`、
  `src/gui/server_poller.*`）；GPU 会话下点 Analyze 的用户体验（报错 / 提示 / 禁用）；
  C API 的请求结构、结果帧结构、`LUMICE_API_VERSION` 递增方式。
- 子任务 5：反投影走 C API 还是 `src/util/`（§6）；ROI 圈的预览层绘制与既有 overlay
  绘制管线的关系；单段链判定在 GUI 侧如何呈现（禁用态说明文案）。

## 8. 与 (ii)（剥离）的关系

(ii)「每桶真实辐射量、随实时开关变化」这个原始需求形态，其五条实现路线**仍然挂起**，
本文不解除这个挂起。本形态改变的是重新访谈用户时要问的问题：不再是「你需要多大的 K」
这种直接对着实现细节发问的问题，而是「有了分析面板 + 排除后重跑这条闭环之后，你是否
还需要一个不必重跑、所见即所得的实时开关」——如果专用运行 + 排除重跑已经能覆盖大部分
剥离场景，(ii) 五条路线要解的可能只是一个更窄的剩余需求（比如需要频繁来回切换、
不能接受重跑等待的场景），值得先带着一个能用的东西去用户那里核实，而不是继续在
没有产品问题答案的情况下推进技术选型。
