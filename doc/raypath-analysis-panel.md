# Raypath Analysis Panel — 设计记录（as-built）

> Status：**as-built**（2026-09-11，五个子任务全部落地：子任务 1 本文 + backlog 拆分 / 子任务 2
> core 链 id 携带 / 子任务 3 server 直方图 consumer / 子任务 4 C API v4.29 / 子任务 5 GUI 面板，
> C API 追加到 v4.30）。改动尚未合并 `main`——本文按当前工作分支的代码状态回写，行号如与 `main`
> 上的最终合入版本有出入以后者为准。
> 本文档结构与 §1/§2 的裁决叙事保持设计阶段原文，§3 起的机制描述、§6 诚实边界、§7 开放设计点
> 已回写为实现后的状态；被下游子任务证伪的 assistant 推断保留原文并标注「已证伪：实际 …」。
> 改「光路分析面板」（raypath-analysis-panel）相关的任何一层前先读本文。

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

**as-built**：以上形态已按设计落地，见 §2–§5。

## 2. 四条裁决

以下四条均为 **owner 裁决**（2026-09-11），逐条附理由与落地状态：

1. **后端：v1 只走 CPU 路，且必须显式声明，不是静默 fallback。**
   理由：Metal/CUDA 的生产路径是 device-fused 单引擎，per-ray 记录不回传 host——
   `src/core/backend/metal_trace_backend.mm` 里 `MetalTraceBackend::ReadbackExitRays`
   的实现直接返回空，注释写明「S1 device-fused: kernel 不再物化 per-exit records」。
   Analyze 是用户主动点击的显式动作，等几秒可以接受，而且统计量（不产图）本身收敛得比
   渲染快。「强制 CPU」要作为一个显式的会话属性写下并记日志（复用
   `LUMICE_SetPreferredBackend` / `ResolveGpuRoute` 的既有强制路径，`src/server/server.cpp`
   的 `ResolveGpuRoute` 已经是「按显式选择路由，不是按能力探测静默降级」的先例），
   使得将来给 GPU 补上这条能力时，是「换一份实现」而不是「悄悄改变了语义」。
   **as-built**：`ServerImpl::mode_`（`SessionMode::kAnalysis`，`src/server/server.cpp:409`）
   进入分析会话；`ResolveGpuRoute(preferred_backend, logger_, force_cpu)`
   （`src/server/server.cpp:531`）与 `Simulator::CreateBackend`（`src/core/simulator.cpp:931`
   附近）都在 env override 之前短路成 CPU；`Simulator::SetAnalysisForceCpu(bool)`
   （`src/core/simulator.hpp:111`）写这条会话属性，`Simulator::ActiveBackend()`
   （`src/core/simulator.hpp:119`）发布解析后的实际后端，`LUMICE_GetActiveBackend`
   （`src/include/lumice.h:2347`，`src/server/c_api.cpp:3319`）把它读出来给调用方核对「强制是否生效」。
   GUI 侧**没有**为这条加任何可见提示或按钮禁用——Analyze 在 GPU 偏好会话下一样可点，只是内部
   静默走 CPU、可能比渲染慢；这是子任务 5 范围内的非目标（GUI 的 GPU 路径体验留给未来子任务）。
2. **「光路」= 从光源到相机的完整链，不是单晶体内的一段。**
   例如 `crystal1(1-3-5)-crystal2(3-2)`；单晶体 MS 或单层退化为链长 1，不特判成另一种数据形状。
   理由（机制约束）：现有数据结构拿不到全链——`ExitRayRecord::path` 只装最后一层的面序列，
   `RayBuffer::recorders_`（按 index 经 `RayBuffer::RecorderAt(idx)` 访问的并行数组，
   `src/core/raypath.hpp:138` 附近注释）只装当前晶体那一段——要跨层拼出完整链，需要一个**随光线跨 MS 层
   前向携带的 per-ray 链 id（uint32）**，配合一张 interning 表把 `(父链 id, 本层 crystal_id,
   本层约化后的 segment)` 映射成新的链 id，天然长成一棵 trie；要打印完整链时沿父指针链回溯。
   不携带字节序列本身（太胖，跨层每次拷贝一遍字节数组）、也不携带哈希（丢失打印能力，且有
   碰撞风险）——链 id + interning 表是唯一同时满足「跨层轻量携带」与「事后可读」的形状。
   **as-built**：见 §3.1/§3.2。
3. **ROI = 方向空间里的一个锥（中心方向 + 角半径），与画面投影无关；角半径用滑杆，且滑杆必须
   display-time 生效。**
   记录阶段按到锥中心的角距离分环累加（不是只记「在/不在锥内」这一个布尔量），滑杆只决定
   显示时求和到第几环——这样调滑杆不需要重跑统计，是纯粹的显示层操作。「可见区域」定义为
   落在当前画幅内（复用 `ProjectExitToPixel` 的画幅内判定）；「全天」不做任何空间判定，
   聚合所有出射光线。
   **as-built**：见 §3.4；GUI 侧滑杆 display-time 语义由专门测试钉住
   （`test/gui/functional/test_raypath_analysis_panel.cpp` 的 `radius_slider_is_display_time`
   用例：拖滑杆后 payload 指针/epoch/lifecycle/upload 计数均不变）。
4. **总能量降序排序必有；其它排序方式可以有，但「能量 / 覆盖立体角」这种密度量 v1 不做。**
   理由：每个链 id 对应的累加器只需要是标量（Σ(Y·w)、计数），代价很低；密度排序需要给
   **每个** key 都配一张粗天球栅格才能算出「这条链覆盖了多大立体角」，存储与实现复杂度都
   上升一个量级。v1 先把「总能量降序」这个最基本、最省资源的排序做对，密度排序留作明确的
   后续升级点（见 §8）。
   **as-built**：`RaypathHistogramResult::entries_`（`src/server/server.hpp:244`）按能量降序，
   相等时按链字符串字典序（tie-break，`test_raypath_histogram_consumer.cpp` AC5 覆盖）；
   密度排序 v1 未做，见 §8。

## 3. 机制

### 3.1 链 id 前向携带

链 id 是一个新的 per-ray side-car，携带点与既有的 `RayBuffer::components_`
（`src/config/sim_data.hpp` 里的 `std::unique_ptr<uint64_t[]> components_`，
raypath-color 基础设施新增的 per-ray component mask）**完全同一套点**——这不是巧合，
而是硬约束：任何随光线跨层携带的并行状态，必须跟着**所有**同一批 swap/gather/fan-out/续传
交接点走，漏一个就是静默错位、且现有单元测试测不到——这是同类跨层并行状态（如
`components_` 这条 side-car 自身）曾经历过的一般性教训：漏搬一个传播点不会立刻崩溃，
而是让并行数组与光线静默错位，只有专门针对该点写白盒测试才能测出来。

**as-built（已核实到字级，非设计阶段的推断清单）**：`chain_ids_` 是 `RayBuffer` 按需分配的
`std::unique_ptr<uint32_t[]>` 列（`src/config/sim_data.hpp:164`），`HasChainIds()` /
`ChainIdAt(idx)` / `SetChainId(idx, v)` 是访问入口。实际的六个搬运点（AC2 红态探针逐点验证，
去掉任一处均 ≥1 红，`test/unit-correctness/core/test_chain_id_carry.cpp` 26 例覆盖）：

- `RayBuffer::ChainIdFanOut(src, src_idx, dst0, dst1)`（`src/config/sim_data.cpp:252`，
  声明于 `src/config/sim_data.hpp:116`）——`TraceRayBasicInfo` 一次打两半时子段继承父段链 id
  （调用点 `src/core/simulator.cpp:705`）。
- `RayBuffer::SwapRay`（`src/config/sim_data.cpp:299` 附近）——shuffle 时链 id 跟着交换，
  两处调用点：`src/core/simulator.cpp:1601`（`init_data[0]` 批次内 shuffle）与
  `src/core/backend/cpu_trace_backend.cpp:488`（`CpuTraceBackend` 的续传缓冲区 shuffle）。
- 续传交接 `init_data[1].SetChainId(dst_idx, src_chain_id)`（`src/core/simulator.cpp:851`）与
  同一交接点上 `buffer_data[0].SetChainId(dst_idx, src_chain_id)`（`:841`）——续传时把上一层
  出射的 `src_chain_id`（`:827`，从 `buffer_data[1].ChainIdAt(idx)` 读出）复制给下一层的两份
  并行缓冲。
- 批量 `RayBuffer::EmplaceBack` 的三个重载（`src/config/sim_data.hpp:147-149`，
  `carry_chain_ids` 判空守卫在 `src/config/sim_data.cpp:375`）——批量追加时链 id 跟着搬。
- `InitRayFirstMs` 归根为 `ChainIdInterningTable::kRootChainId`
  （`src/core/simulator.cpp:311`）——新一批光线的起点。
- `InitRayOtherMs` **不重置**链 id（延续父链，靠上面的续传交接点持续携带，无需在
  `InitRayOtherMs` 内部单独处理）。

`Simulator` 内两个 intern 点（`ChainIdLayerContext` 由 `MakeChainIdLayerContext` 在光线穿过每层
晶体前构造，`src/core/simulator.cpp:1504`）：续传交接的 `init_data[1].SetChainId(j,
InternRayChainId(*chain_ctx, init_data[1], j))`（`:1562`）与真出射的
`outgoing_chain_id.push_back(InternRayChainId(*chain_ctx, buffer_data[1], j))`（`:1582`）。

设计阶段的建议方向——「把 `components_` 与链 id 收进同一个 per-ray side-car 结构，让『漏搬一个
传播点』在结构上不可能发生」——**未采纳**：实施期两条 side-car 仍是各自独立的列
（`components_` / `chain_ids_`），靠上面枚举的显式搬运点 + 26 例红态探针覆盖，而不是合并结构。
code-review Minor：`all_data` 对应的 `chain_ids_` 列的 `Reset()` 调用点未启用（`init_data[0]/[1]`
与 `buffer_data[0]/[1]` 已覆盖，`all_data` 未覆盖）——目前无害，因为唯一的分析出口是
`outgoing_chain_id_` / `chain_id_table_delta_`，没有消费者读 `all_data.ChainIdAt()`。

### 3.2 interning 表与逐段对称约化

interning 表把 `(父链 id, 本层 crystal_id, 本层约化后的 segment) → 子链 id` 建成一棵 trie；
「本层约化后的 segment」复用现有的对称约化单一权威，不另写一套：

- `Crystal::ReduceRaypath(rp, symmetry)` / `Crystal::ReduceRaypath(rp, symmetry, sigma_a,
  d_applicable)`（`src/core/crystal.hpp:242` / `:252`）——按 (P, B, D) 对称标志约化一段
  face-index 序列。
- `detail::ReduceBuffer`（`src/core/filter_spec.hpp:140`）——同一套约化规则的原地版本，
  作用于原始 `uint8_t` 缓冲区，供 inline data 与 arena 拷贝共用。

**as-built**：权威实现是新文件 `src/core/chain_id_table.{hpp,cpp}` 的
`ChainIdInterningTable`（类声明 `src/core/chain_id_table.hpp:53`）——`Intern(parent, crystal_id,
segment)` 建表，`Format(uint32_t id)`（`chain_id_table.hpp:79`，实现 `chain_id_table.cpp:80`）
沿父指针回溯打印，`Segments(uint32_t id)`（`:85`/`:70`）与 `PathToRoot(uint32_t id)`
（`:89`/`:62`）共享同一个私有 walk（贯彻单一权威，子任务 3 落地时把两者的重复实现合并）。

`sigma_a`/`d_applicable` 的推导逻辑在 `MakeChainIdLayerContext`
（定义于 `src/core/simulator.cpp:859-869`，声明于 `src/core/trace_ops.hpp:94`）与
`FilterSpec::Create`（`filter_spec.cpp:382-383`）两处字面重复——code-review Minor，仅靠测试断言
两者数值相等保证同步，未抽出共享函数（a56 的一个已知但未消解的小实例）。

链 id 只在光线穿过一层晶体、生成本层 raypath 之后，用该层约化后的 segment 去查/建表，
而不是缓存未约化的原始面序列——这样同一等价类的光路天然映射到同一条链，链数不会因为
「同一物理路径的不同镜像/旋转变体」被重复计数。打印一条完整链时，从叶子链 id 沿父指针
回溯到根，逐段还原出 `crystal1(1-3-5)-crystal2(3-2)` 这种展示形式。

**多 worker 合并（as-built）**：设计阶段的开放问题——per-worker 增量 vs 强制单 worker——
裁定为 **per-worker 表 + 增量合并**：`SimData` 新增 `producer_effective_seed_`
（`src/config/sim_data.hpp:254`）标记批次来源，`ChainIdMerger`（类声明
`src/core/chain_id_table.hpp:130`）的 `Absorb(producer_key, delta)`
（`:147`，实现 `chain_id_table.cpp:103`）把每个 worker 的本地 id 空间 remap 进一张合并表，
返回 `AbsorbReport{orphaned, non_monotonic}`（`:135`）作为契约破坏的可观测见证。
`test/unit-correctness/core/test_chain_id_merger.cpp`（新）与
`test/regression-sentinel/test_effective_seed_worker_count_invariant.py`（`-m slow`）
用两个不同 seed 的 `Simulator` 验证合并集合等于两集合之并、先证明本地 id 确有冲突。

### 3.3 消费者形态

新 consumer 实现现有的 `IConsume` 接口（`src/server/consumer.hpp:19`，契约细节见
`doc/accumulator-consumer-architecture.md` §2），与 `RenderConsumer` / `StatsConsumer` /
`AnchorConsumer` 一样挂在 `consumers_` 上、接收同一批次数据。它按链 id 聚合两个标量：
Σ(Y·w) 与命中计数，其中 Y 的计算必须复用 `RenderConsumer` 已有的权威（`SpectrumToXyz` /
`SpectrumToXyzPerRay`，`src/core/color_util.hpp:36` / `:67`），不得自建归一化——
07-07 的 `ComponentBinConsumer` spike（已 revert；结论见
`doc/gui-custom-spectrum-and-raypath-color.md` §"显示模型定案"一节）唯一踩过的坑
就是自造了一套归一化，产出假洋红。

**as-built**：`RaypathHistogramConsumer`（`src/server/raypath_histogram_consumer.{hpp,cpp}`，
新）。`Consume()` 先无条件 `Absorb` 整批 chain-id delta，再逐光线做 ROI 判定——顺序是刻意的：
先吸收表增量、再筛选光线，避免「delta 非空但本批光线全被 ROI 过滤掉」导致 orphaned 引用。
`server.hpp` 的 `Result` variant 新增第四 alternative `RaypathHistogramResult`
（`src/server/server.hpp:251`），是纯加法扩展（全仓非穷尽 `std::visit` 无需补分支）。
两阶段快照协议（`PrepareSnapshot` / `GetResult`）与 `StatsConsumer` 同形。

已知问题（code-review Minor，不阻塞，未在本次收尾修复）：`Consume()` 早退只判
`outgoing_chain_id_.empty()`，未联动判断 `chain_id_table_delta_` 是否非空；若某批
`outgoing_chain_id_` 为空但 delta 非空（例如该批只产生尚未对外暴露的中间链表项），会在
`Absorb()` 之前提前 return，后续批次引用其 parent_id 会被判为 orphaned（`Resolve()` 静默丢弃为
`kUnresolved`，一次性 `ILOG_ERROR`，能量数字偏小）；`logged_delta_contract_` 被两类语义不同的
失效路径共用，先触发的一类会掩盖后触发的另一类的诊断输出。

### 3.4 ROI 三档的实现落点

- **锥形**：记录阶段按「出射方向与锥中心方向的角距离」分环累加，环数在记录时固定、
  滑杆只决定显示时求和到第几环之前——记录一次、显示时任意调半径，不重跑。
  **as-built**：点积阈值做成员判定，`acos` 只用于分环，`double` 精度累加
  （`raypath_histogram_consumer.cpp`）。GUI 侧请求固定发送**整个**锥（`kAnalysisConeMaxRadiusDeg
  = 15°`，`kAnalysisConeRingCount = 30` 环即 0.5°/环，`src/gui/gui_constants.hpp:171-172`），
  滑杆只改变显示时求和到第几环——三者是纯工程取值，不是裁决，改这三个常数即可调整精度/范围
  （见该文件同名注释）。锥 ROI 有提前停止：落线数达到 `kAnalysisConeStopTarget = 200000`
  （`src/gui/gui_constants.hpp:176`）即结束，不必跑满 `ray_num` 预算。
- **可见区域**：复用 `ProjectExitToPixel`（`src/core/shared/projection_shared.h:243`）纯投影，
  落在画幅外的直接丢弃。**已证伪：实际** `ProjectExitToPixel` 只做投影，不含 `visible`/`front`
  语义——子任务 1 起草时的假设「该函数已含 visible 语义」被子任务 3 实现期修正：画幅内判定需要额外
  组合 `mask_detail::VisibleByRange(cfg.visible_, wz)` 与
  `mask_detail::FrontVisible(cfg.front_, forward_, wx, wy, wz)`
  （`src/server/raypath_histogram_consumer.cpp:65`）两个 DISPLAY CLIP 判定，`ProjectExitToPixel`
  本身对 `visible`/`front` 一无所知。
- **全天**：不做任何空间判定，所有出射光线都计入。

### 3.5 累加器字段

每个链 id 对应的最小状态是：能量累加（Σ(Y·w)）、命中计数、（锥形 ROI 时）按环分桶的
子累加器。这三类都是标量或定长小数组，不随分辨率或桶数线性增长——这正是本形态相对于
「每桶一张全分辨率图」的内存优势的直接体现。**as-built**：`RaypathHistogramEntry`
（`src/server/server.hpp:230`）与 `RaypathChainSegment`（`:225`）承载这些字段；C 结构体侧
`LUMICE_RaypathHistogramEntry`（`src/include/lumice.h`，「Raypath Analysis Run」一节）逐字节
镜像。

## 4. 可复用地基清单

以下地基已核实存在、并已在实现中直接复用：

- `ExitRayRecord{dir, weight, path, crystal_id, ms_layer_idx, wl_idx, component_mask}`
  （`src/core/exit_seam.hpp:40` 起）——单层出射记录的既有结构；**as-built**：链 id 未挂在这个
  结构体上，而是走 `SimData::outgoing_chain_id_`（`src/config/sim_data.hpp:245`，与
  `outgoing_w_` parallel）单独交付，与 `exit_records_` 是两条平行的出口，不是同一条记录多一个
  字段。
- `ProjectExitToPixel`——画幅内判定的既有实现，供「可见区域」ROI 直接调用（见 §3.4 的修正说明：
  它只是判定的一半，另一半是 `visible`/`front` 组合）。
- `src/core/projection.hpp:28-131` 附近的五族反投影（`LinearInverse:28` / fisheye 四种变体
  `Inverse:69-72` / `RectangularInverse:84` / `GlobeInverse:104` / dual-fisheye 的
  `DualFisheyeToPixel:123` 与 `PixelToDualFisheye:128`）——**as-built**：GUI 侧最终没有直接调用
  这五族函数本身；`LUMICE_UnprojectPixel` 复用的是 core 的 `PixelToWorld`
  （`mask_detail::` 命名空间），后者内部走同一套镜头反投影。见 §6「反投影走 C API 还是
  `src/util/`」的裁定。
- `RenderConsumer` 的 Y 计算权威（`SpectrumToXyz` / `SpectrumToXyzPerRay`，
  `src/core/color_util.hpp`）——新 consumer 的能量累加复用同一份权威（`test_raypath_histogram
  _consumer.cpp` AC2 用容差断言两者 Σenergy 一致）。
- 07-07 `ComponentBinConsumer` spike 的结论（已 revert，不是可复用代码，只引结论）：
  一次性旁挂 consumer、复用生产路径的 `ProjectExitToPixel`、core 零改的做法是可行的；
  唯一的坑是自建归一化产生假色，详见 `doc/gui-custom-spectrum-and-raypath-color.md`
  相应小节。

## 5. 顺带闭环：行 → 排除 filter → 重跑

列表里的每一行对应一条链；用户可以对某一行点「排除此光路」，生成一个 filter 并触发重跑。
这不是新机制，是现有「排除后重跑」语义的复用——零新增存储，只是把 filter 的生成源头
从「用户手写」换成「从分析结果点选」。v1 范围内，这个按钮**只对单段链**（链长 1，即
单晶体单层）可用：现有 filter 是单层 per-crystal 的物理门——`FilterSpec::Match`
（`src/core/filter_spec.hpp:34/43`）的签名只接受单个 `RaySeg` + 该层的 `RaypathRecorder`，
表达不了跨层链的排除语义（如「排除 `crystal1(1-3-5)-crystal2(3-2)` 这整条链，但保留
`crystal1(1-3-5)` 与其它晶体的组合」）。这个限制在 UI 上要明确说明，不能让按钮在多段链上
显示为可用却生成一个不达意的 filter。

**as-built**：GUI 侧「Exclude this raypath」按钮（`ICON_FA_BAN`，`src/gui/analysis_panel.cpp:573`）
的可用性由 `EvaluateExcludeEligibility`（`analysis_panel.cpp:277`，声明
`src/gui/analysis_panel.hpp:129`）判定，五类结果之一：

| 判据 | 结果 | 提示文案（原文） |
|---|---|---|
| 未选中任何行 | `kNoSelection` | "Select a raypath in the list first." |
| `chain_len != 1`（多段链） | `kMultiSegment` | "This chain crosses scattering layers. A filter belongs to one crystal, so the current filter model cannot express excluding a multi-layer chain." |
| 该链对应的晶体不在当前文档里 | `kCrystalNotInScene` | "The crystal this chain went through is not in the current document (the crystal list changed since the analysis). Run and analyze again." |
| 该晶体已有 entry 挂了 filter | `kEntryHasFilter` | "An entry using this crystal already has a filter. Excluding on top of an existing filter is not merged automatically; edit that filter instead." |
| 以上皆非 | `kOk`（按钮可点） | — |

点击后 `ApplyExcludeSelectedRaypath`（`analysis_panel.cpp:332`）生成一个
`action = filter_out`、`sym_p/sym_b/sym_d` 全开的 `FilterConfig`（与分析会话的 P|B|D 约化口径
一致，否则排除 `3-5` 会漏掉同一条链的其它取向等价变体），通过既有的
`WriteFilterToPool` / `PropagateFilterIdToLinked`（从 `edit_modals.cpp` 的局部 lambda 提升为
自由函数，`src/gui/edit_modals.hpp:27/32`）写入——与手工编辑弹窗共用同一条写入原语，
不是重新发明一条。文档随之标为 `Modified`；用户需按 Run 重跑才能看到排除后的画面。

`kEntryHasFilter` 这条判据比设计阶段预想得更保守：只要该晶体**任一** entry 已有 filter 就整体
禁用，而不是「已有同 action filter 时追加一行 OR」。这是实施期的范围收窄，尚待 owner 复核；
撤销成本低（只需在该分支前加一段追加逻辑）。

## 6. 诚实边界

逐条标注归属与落地状态（owner 裁决 / assistant 推断 → 已确认 / 已证伪 / 仍待办）：

- **GPU 不覆盖**（owner 裁决，§2 第 1 条）——**已确认，且已裁定 UI 处置**：v1 只有 CPU 路能
  产出分析结果，子任务 4 强制走 CPU（见 §2 第 1 条 as-built）。子任务 5 的裁定是**不禁用、不提示**：
  Analyze 按钮在 GPU 偏好会话下同样可点，静默走 CPU，只是可能更慢；没有加报错/警告/禁用态。
- **多段链不可排除**（owner 认可，§5）——**已确认**：「排除此光路」按钮 v1 只对单段链生效，
  跨层链没有对应的 filter 表达能力，UI 提示文案见 §5 表格。
- **能量 ≠ 视觉显著**（owner 裁决，§2 第 4 条的直接推论）——**未受实现影响，原样成立**：
  全天 / 可见区模式下，random-orientation 场景里弥散、覆盖立体角大的光路总能量可能高于视觉上
  更显眼的窄亮弧，导致排序把「不起眼但铺得广」的链排在「醒目但集中」的链前面。v1 接受这个
  结果，密度排序是明确的后续升级点（见 §8）。
- **逐段约化的独立性假设**（owner 提出的风险点，assistant 未独立验证）——**已确认（机制论证，
  非统计验证）**：论证落在 `MakeChainIdLayerContext` 声明处的注释（`src/core/trace_ops.hpp:80-93`）
  ——各层朝向独立重采样成立是因为 `InitRayFirstMs` 与 `InitRayOtherMs` 每层各自调用
  `InitRay_rot`，对上一层的朝向没有记忆；`doc/raypath-symmetry.md` §2b 只论证单层总体合法性、
  不谈跨层，独立性论证不依赖那份文档、只依赖这两个调用点本身。这是**代码级论证**，不是
  统计检验或形式证明。
- **两条 CPU 路，行为待核实**（assistant 推断，标记为待核实）——**已确认，推断成立**：
  生产路 `Simulator::SimulateOneWavelength`（legacy，多 worker）交付链 id；`CpuTraceBackend`
  （仅 `LUMICE_TRACE_BACKEND=cpu_backend` 环境覆盖下启用）不产 `exit_records_`，链 id 留空并
  一次性 `ILOG_WARN`，不崩溃。assistant 原推断「legacy 路不必切换到 `CpuTraceBackend`，只需
  照着 `outgoing_component_` 的样子多打包一条 `outgoing_chain_id_`」**成立**，子任务 2 按此落地。
- **多 worker 下的 interning 合并**（owner 提出的风险点）——**已裁定并落地**：per-worker 表 +
  `SimData` 携带本批新增的表增量，consumer 按 `ChainIdMerger` 合并（见 §3.2）；未采用「强制
  `worker_count=1`」的备选方案，多 worker 吞吐未被牺牲。诚实边界：这条合并逻辑只有合成数据 +
  白盒单测覆盖（`test_chain_id_merger.cpp`），真实多线程并发场景的端到端验证由子任务 4 的
  `test_server_analysis_run.cpp` 补齐（互斥/强制 CPU 场景），但**没有**一个测试是「多 worker
  并发跑分析会话，断言合并结果与已知答案一致」这种端到端形状——仍然是合成/白盒覆盖，
  没有再往上升级。
- **反投影走 C API 还是 `src/util/`**（未决问题，非裁决）——**已裁定：C API**。子任务 4 新增
  `LUMICE_UnprojectPixel(view, px, py, out_dir[3])`（`src/include/lumice.h:2064`），签名从
  plan 字面的 `float px, py` 改为 `int px, py`（整数像素坐标）——用浮点签名会在 bridge 层复制一份
  `PixelToWorld` 内部换算，违反「反投影只有一个实现」的硬约束。11 个 lens 分支的
  forward∘inverse **精确相等**（无需容差）。函数没有搬进 `src/util/`：`PixelToWorld`
  仍在 core 的 `mask_detail::` 命名空间，GUI 只经这一个 C API 入口调用它。

## 7. 结果条目的可打印字符串（唯一权威）

GUI 与 CLI 若都要打印一条链，打印的是**同一个字符串**，而不是各自从分段结构拼一遍。
权威实现只有一处：`ChainIdInterningTable::Format(uint32_t id)`（`src/core/chain_id_table.cpp:80`），
`RaypathHistogramConsumer::PrepareSnapshot` 用它给 `RaypathHistogramEntry::display_` 赋值，
C API 的 `LUMICE_RaypathHistogramEntry::display` 是这个字符串的**逐字节拷贝**（`c_api.cpp` 只做
截断，不重拼）。格式规则（由该实现定义，此处只是复述）：

- 逐层 root-first：先写光线进入的第一层，最后写出射层；
- 每层写作 `crystal<id>(<face>-<face>-…)`，`<id>` 是 config 里的晶体 id，括号内是**对称约化后**
  的面序列（`Crystal::ReduceRaypath`，约化对称由请求的 `chain_id_symmetry` 决定，默认
  `FilterConfig::kSymP | kSymB | kSymD`，`Simulator::kDefaultChainIdSymmetry`，
  `src/core/simulator.hpp:103`），面号之间用 `-` 连接；
- 多层之间也用 `-` 连接。

例：单层 22° 晕 `crystal1(3-5)`；两层 `crystal1(3-5)-crystal2(1-3)`。

C 结构体里的 `chain[]`/`segment[]` 与 `display` 描述同一条链，前者供程序判定（例如「是否单段链」
决定「排除此光路」按钮可用），后者供显示；两者不一致只可能来自截断（超过
`LUMICE_MAX_RAYPATH_CHAIN_LAYERS` / `LUMICE_MAX_RAYPATH_SEGMENT_LEN`(=64) /
`LUMICE_RAYPATH_DISPLAY_MAX`(=3200) 的病态链，每次读帧时 WARN 一次），正常场景下两者互为镜像。

**symmetry 来源：候选 B 已落地，owner 尚未显式签字**。设计阶段 §2 第 2 条留了一个开放决策
（默认候选 A：复用晶体上恰好一条 symmetry 的 filter；候选 B：会话级统一标志）。子任务 2 实施期在
owner 不在场的情况下按 auto 模式落地候选 B——`Simulator::SetAnalysisChainId(bool enabled,
uint8_t symmetry)`（`src/core/simulator.cpp:976`）接受一个会话级统一的对称标志，默认
`kDefaultChainIdSymmetry = kSymP|kSymB|kSymD`（理由：避免默认场景下 6 旋转变体拆行、避免隐式
依赖某个 filter 的配置）。**这是本次收尾时仍然开放的唯一产品级决策**：若 owner 后续裁定候选
A，改动范围仅限 `ChainIdLayerContext` 的 symmetry 取值来源一处（`MakeChainIdLayerContext` 的
调用点，`src/core/simulator.cpp:1504`）。子任务 4（C API）与子任务 5（GUI）的 symmetry 字段设计都
继承了候选 B 这条基线（GUI 侧目前也不暴露 symmetry 为可调 UI，直接用默认值）。

## 8. 开放设计点清单（as-built 后的状态）

设计阶段 §7 的开放设计点均已在子任务 plan 阶段裁定并落地（symmetry 来源见 §7 末尾，
锥形分环参数见 §3.4，反投影落点见 §6）。以下是**明确未做、留作后续升级**的项，各附触发条件：

- **GPU 直方图 kernel**（对应 §2 第 1 条的「GPU 不覆盖」）：触发条件——用户反馈 CPU-only 的
  分析等待时间在大 `ray_num` / 多晶体场景下不可接受，且 Metal/CUDA 补上 per-ray 记录回传
  （当前 `ReadbackExitRays` 直接返回空）的工程代价被证明值得投入。
- **密度排序（能量 / 覆盖立体角）**（对应 §2 第 4 条）：触发条件——用户明确反馈「总能量降序」
  把弥散但总量大的链排到了他们认为不重要的位置靠前。需要给每个链 key 配一张粗天球栅格才能算出
  覆盖立体角，存储与实现复杂度上升一个量级，不是加一个排序 comparator 就能做到。
- **列表 → 预览高亮联动**（点击列表行时在预览上高亮该链贡献的像素）：子任务 5 明确列为非目标；
  触发条件——用户需要「选中一行就能在画面上看到它具体亮在哪里」而不止是能量/计数数字。
  需要 consumer 侧额外记录「哪些像素属于这条链」，这与本形态「统计量而非图」的内存优势方向相反，
  只能做成按需/懒计算的形式（例如再跑一次「只统计这条链落在哪些像素」的窄化分析），不能常驻。
- **多段链排除**（对应 §5 的限制）：触发条件——filter 模型本身获得跨层链表达能力（例如一种
  「跨 crystal 序列」谓词），那之前这条限制是结构性的，不是 UI 措辞能绕开的。

## 9. 与 (ii)（剥离）的关系

(ii)「每桶真实辐射量、随实时开关变化」这个原始需求形态，其五条实现路线**仍然挂起**，
本文不解除这个挂起。本形态改变的是重新访谈用户时要问的问题：不再是「你需要多大的 K」
这种直接对着实现细节发问的问题，而是「有了分析面板 + 排除后重跑这条闭环之后，你是否
还需要一个不必重跑、所见即所得的实时开关」——如果专用运行 + 排除重跑已经能覆盖大部分
剥离场景，(ii) 五条路线要解的可能只是一个更窄的剩余需求（比如需要频繁来回切换、
不能接受重跑等待的场景），值得先带着一个能用的东西去用户那里核实，而不是继续在
没有产品问题答案的情况下推进技术选型。这个访谈**尚未进行**——本形态交付的是「能用的东西」本身，
不是访谈结果；(ii) 的五条路线分析与访谈问题的完整记录另存于项目的需求追踪记录，供重启时使用。
