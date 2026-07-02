# TraceBackend host/device seam 重新设计 — 设计讨论

> 本文整理 explore-256 收敛后、owner 与 assistant 的架构讨论，作为 explore-257 各假设的论证底稿。
> 性质：**设计推理记录**，不是定稿规范。记录"为什么这么想"、考虑过的替代、以及尚未拍板的待定项。
> 事实基座：`../explore-metal-gui-perf/`（exp#1/#4/#5）。

---

## 1. 缘起

explore-256 定位到：GUI 开 Metal 后比 legacy 慢 9-57×，根因不是 GPU/Metal，而是 **TraceBackend seam 的架构税**——backend 每个 128 光线的小 batch 都做一次 O(W×H) 的整幅图 `ReadbackImage`+Y-sum；legacy 每 batch 只发 O(光线) 原始光线、把投影/累加留给 consumer 增量做。

最初的"修复"建议是合批(把 backend batch 128→8192，exp#5 证明能反超 legacy 1.1-2.6×)。但 owner 指出这是**治标且破坏物理**：batch 大小与晶体几何随机性采样是耦合的——当前每 batch 只采一个晶体几何，合批=许多光线共用一个几何，几何分布被严重欠采样。理想是 batch=1(每光线独立几何)，128 是 CPU 上效率与几何随机性的折中。

于是问题被迫回到根上：**当前 seam 设计本身是否合理？** 拆成两个子问题：
- **Q1：进入 device 的点到底在哪？**
- **Q2：出 device 的点到底在哪？**

owner 立场：方向上赞成尽量多链路上 device(fusion kernel)，但当前 seam/kernel 规划不够好，值得重设；且 CPU/GPU 不必 bit-for-bit，**统计等价即可**。

---

## 2. 核心诊断：三个时钟被焊死

当前 Metal seam 把三个**本应独立**的"时钟"全部 = "每 128 光线一次 backend session"：

| 时钟 | 由谁驱动 | 理想粒度 |
|------|----------|----------|
| **几何采样** | 物理(每个光子遇到独立晶体) | 趋近 per-ray |
| **trace 派发** | GPU 占用率/效率 | 大(10⁴-10⁶) |
| **图像回读** | 显示响应性 | 显示节奏/按需 |

焊在一起 → 任一旋钮和另外两个打架：调大 batch 提效率→几何随机性崩；保几何随机性→batch 小→回读税炸。

**legacy 之所以"还行"**：它碰巧把"几何时钟"(32 光线粒度)和"回读时钟"(consumer 增量成像)解对了，只是"trace 时钟"对 GPU 太小(128)。

**redesign 的本质 = 给三个时钟各自独立的频率。**

事实校准（代码读出）：
- legacy/CpuBackend `kSmallBatchRayNum=32`：每 32 光线一个随机几何。
- Metal `ResolveLayerCrystalForCi`→`MakeCrystal` 每 ci 每 TraceLayer 一次：**每 batch 一个几何**。即 Metal 在 batch=128 时几何采样已比 legacy 粗 4×，合批 8192 则粗 256×。

---

## 3. Q1 — 进入 device 的点

### 3.1 理论最优：入口尽量上移到"场景分布参数 + RNG seed"

让几何采样 / 取向 / 根光线生成全部 per-thread 发生在 device。一举三得：
- 几何时钟与 trace 时钟解耦(每线程=自己的几何=batch-1 物理，无论派发多大)；
- 消掉 exp#1 的 host-bound root-gen 瓶颈；
- 对 CUDA 是刚需(否则每光线几何要走 PCIe)。

GPU RNG(前序 task/explore 已认可的方向)是使能器。

### 3.2 几何：三条路线

| 路线 | 机制 | 评价 |
|------|------|------|
| (a) 内核内逐线程造几何 | kernel 从分布参数采 height/face_distance + 造多面体面/法线 | 最纯粹，但工程量+正确性风险大；且**违反"几何单源"约束**(GPU 要一套几何构造代码) |
| **(b) 几何池**(推荐) | host 一次性造 **K 个**几何打包上传，每线程 RNG 随机选索引 | 满足"统计等价"；几何构造**单源留 CPU**(复用 `MakeCrystal`)；GPU 只读不造；CUDA 友好(每 session 一次上传 ~K×1KB) |
| (c) CPU-RNG-async | 几何留 CPU 采，双缓冲流水线隐藏开销 | 见 §3.3，被 (b) 支配，留作 fallback |

#### 3.3 辨析：池化 ≠ CPU-async（两个正交的轴）

讨论中澄清的关键点——它们长得像但回答不同问题：
- **池化**动的是**工作量**：几何构造次数 = K，与 N、与 batch 都无关；per-ray 指派是廉价随机下标。**直接拨动"几何时钟"**，同步也成立。
- **CPU-async**动的是**时序**：host 工作量没少，只是用并发挪出关键路径。**不碰几何时钟**——若配"每 batch 一个几何"，耦合原封不动，只藏了 host-bound 性能症状。

→ 池化修根因，async 修症状。两者 2×2 可叠加。**池化即使不 async 也解耦了物理。**

#### 3.4 标定 K：方差分解

```
总方差 ≈ 几何采样方差 (~1/K) + 光线/取向/波长方差 (~1/N)
```
取 K 使 1/K 项相对 1/N 可忽略即可。N/K 的复用只是"每个形状多打几束光"。池化是 legacy"32 光线共用一形状"的推广，把复用因子从 batch 解放、并用随机指派消掉连续 batch 的相关性。**有原则可标定，不是 hack。**

### 3.5 "线程 = 光线 + 晶体索引"模型 与 divergence 成本

把"晶体"从"光线被批在其下的实体"重构为"光线读取的那段三角形数据"：一次巨型 dispatch，几何 buffer 持多个晶体三角形，per-ray index 选晶体，每线程对自己的晶体三角形 trace。

**per-ray 不同三角形在 GPU 上不是零成本，但很便宜**：
- 内存 divergence：同 warp 读不同晶体地址 → 失 coalescing。但晶体几何极小(~1KB)，trace 是 compute-bound，几何读占比小。
- **一次加载多次复用**：一条光线在同一晶体里反复弹射，开头把几何载入寄存器/私有内存一次，之后每弹射是寄存器访问；散读代价被多次弹射摊薄。
- 控制 divergence：不同晶体类型面数不齐 → warp 等最长；纯 prism 无忧，混合类型用 **wavefront 排序**(按类型分桶)消解，v1 可缓。

这正是生产级 GPU 光追的常态(不同光线打不同三角形)；halo 还更简单(每光线由索引直知晶体，连 BVH 都不用，每弹射 O(面数))。

### 3.6 CPU 与 GPU 流水线不必结构镜像（"原始之罪"）

254/253 花大力气让 GPU **镜像** CPU 流水线(采一晶体→批光线变换到局部系→trace→反变换)，甚至为 parity 对齐逐光线坐标系。**正是这个镜像把 CPU 的"批共用晶体"结构搬上 GPU。**

CPU 批的原因是 cache/setup 摊薄("32 光线/晶体"是 CPU 决策)；GPU 靠成千线程并行摊薄，机制不同，批结构不该迁移。GPU-native 单元 = 线程 = 一条光线 + 它的晶体。两后端只需**物理上统计等价**，不需执行结构对齐——把"统计等价即可"从 RNG 推广到整条流水线。

**推论**：parity 应在累加图像层面测(统计分布)，不再对齐逐光线坐标系(253 的 corr 0.93 真正该锚定的层次)。

### 3.7 几何单源 = 不需要 GPU 几何代码

(b) 池化的副产物：几何构造**完全留在 CPU 单源**(现有 `MakeCrystal`)，GPU 侧没有任何几何构造代码。owner 担心的"CPU/GPU 两套几何"被绕开。分工浮现：**复杂但稀少(几何构造)留 CPU/每 session 一次；简单但海量(采样+trace)上 GPU 逐线程。**

---

## 4. Q2 — 出 device 的点

### 4.1 seam 应切在"出射光线"边界，不是"成品图像"边界

出射光线是**渲染器无关**的物理输出；投影/累加依赖 renderer(镜头/视角/filter)。当前 Metal 把投影焊进 trace → 烤死一个 renderer + 逼每 batch 回读整幅图。legacy 的高效正来自分离(发光线、consumer 投影)。

### 4.2 为什么 image 出口对 GUI 次优（精确版）

- **view 无关 vs view 特定**：出射光线 view 无关；image 烤死一个 view。GUI 价值在廉价迭代(改视角/镜头/filter/曝光，多数不改物理只改投影)。手里有出射光线 → 改视角是 O(光线) 重投影、不重 trace；只留 image → 只能全部重 trace。image 出口关死这扇门，buffer 出口保留 optionality(即使 v1 不实现光线缓存，seam 形态不堵死它)。
- **多 renderer**：buffer 出口 = trace 一次、consumer 投影 N 次，trace 不需知道 renderer；image 出口 = N 套 device 累加 + N 投影 pass。
- **filter**：buffer 出口让 filter 跑 consumer(灵活)；image 出口若扔光线，filter 必须上 device，改 filter≈重跑。

### 4.3 诚实的反点：纯吞吐 image 出口反而可能赢

大 batch 下 image 出口回读是固定 O(W×H)、摊到每光线趋近 0、投影在 GPU；buffer 出口导出 O(出射光线)、每光线常数成本不随 batch 摊薄。所以"视角固定熬收敛"的稳态里 image 出口更省。**→ GUI 选 buffer 出口的理由是灵活性/可迭代性，不是速度。**

### 4.4 收口：seam 永在出射光线，投影是可插拔策略

- **seam(出射记录 buffer)= 永久规范边界。**
- **投影 = 挂在 seam 下游的可配置 stage**：GUI 用 CPU consumer(灵活)；bulk 用 device-fused image(吞吐)。
- 两个出口安放成"按 workload 选策略"，不是二选一/先后取代。
- v2 级 hybrid：当前视角 device 累加快速收敛 + 出射光线 ring buffer 让视角切换瞬间重投影预览、后台重 trace 热身。

### 4.5 富出射 metadata —— 富 ≠ 重

owner 定调：**富出射是必须，不是可选**；filter 缺席的原型无法接入 GUI；至少光路 recorder 要带出来；必要时可重设计 metadata 结构。

好消息(代码读出)：想要的紧凑结构**已存在且已压过**——`src/core/raypath.hpp` 的 `RaypathRecorder` 共 18B：
```
offset 0:  uint8_t  size_                (1B, ≤ max_hits)
offset 1:  uint8_t  data_[kInlineCap=15] (15B inline 面序列，面 ID 存 uint8)
offset 16: uint16_t overflow_idx_        (2B, 超 15 跳指向 arena)
```
一条富出射记录 ≈ `{dir(fp16×3=6B) + weight(4B) + crystal_id(2B) + 路径(18B)} ≈ 30B/出射光线`。10⁶ 光线 ≈ 30MB，与一次 2048×1024 整幅图同量级、按大 batch 摊薄 → **GUI 可接受，富出射不必然是带宽瓶颈**。

#### 让富出射在 GPU 上可行的关键一刀

`RaypathRecorder` 的 **overflow arena**(>15 跳动态分配)在 GPU 上是噩梦。但所有真实配置 `max_hits` 7-8 < kInlineCap=15 → **overflow 永不触发**。所以 GPU 导出路径：
- 断言 **inline-only**，**砍掉 arena**，导出 = 定长 per-ray 记录、零动态分配。
- 这就是 owner 说的"重设计 metadata"的落点：把 CPU 那套 overflow 复杂度**设计掉**。甚至按配置实际 max_hits 裁剪 inline 面数组(常 ≤8，省一半)——但属"先量后优"，30B 已够用。

#### 对称折叠留 consumer

`RaypathOrbit`/`ReduceRaypath`/`BuildOrbit` 的对称规约依赖晶体配置 → 留 filter 侧。**GPU 只导出原始面序列，对称折叠+匹配在 consumer，kernel 保持笨。**

### 4.6 带宽与 regime 收口

富出射是 O(出射光线)。GUI(中等 batch)不在乎；bulk 巨量 N 时这个 O(N) 导出本身就不想付 → 正是切到 device 侧 filter+投影(image 出口)的触发点。**同一套出射记录结构：GUI 在 host 侧消费，bulk 在 device 侧消费。**

### 4.7 两个诚实的坑

1. **真·新功能**：当前 Metal `rec_sink[tid]=rec_csum` 只是每光线一个 float 校验和(parity 用)，未导出路径。要让 kernel 在 trace 中逐跳 append `to_face_` 到 per-thread 定长数组并写出——中等改动，从零加。
2. **MS 多层作用域**：路径是 per-(光线, MS层, 晶体)，每层换晶体、recorder 重起、不跨层累加。单 MS 干净；多 MS 下"filter 作用哪层 / 出射记录带几层路径"需在 schema 定死。

### 4.8 离散显存 reframe：两个出口塌成一条流水线（scrum-302，2026-06-28 第一性原理收敛）

> 本节修正 §4.2-§4.4 在**离散显存**上的结论。§4 原文成立于**统一内存**（Metal/M2）：host/device 边界近免费，故"投影在哪跑"是灵活性选择，host consumer 最灵活。CUDA 诚实吞吐基线（#299）暴露：统一内存上几乎免费的那一跳（出射记录回主机）在离散显存上是 **per-exit PCIe 主机往返**主导成本（~54ns/exit，线性于 exit 数 → 吞吐 flat 0.1×），**与 regime 无关**（GUI 还是 CLI 都吃）。

**第一性原理**：离散显存上唯一稀缺资源 = PCIe 带宽/同步。几何极简（几十三角形、寄存器驻留、无 BVH）→ 最优设计的唯一目标 = 最小化跨 PCIe 数据 × 频率。数据按"是否必须跨 PCIe"分层：

| 层 | 内容 | 大小 | 跨 PCIe? |
|----|------|------|----------|
| 输入 | 几何池 + filter orbit 表 + CIE/illuminant | KB | 一次/session |
| 在途光线状态 | dir/weight/path-so-far/wl_idx | O(N) | **永不**（device-resident） |
| 输出 | XYZ 累加图 | ~W·H·3 | 显示节奏/一次 |
| 富元数据（面序列） | 出射记录 | O(出射) | **默认永不**（仅光路回溯特性物化） |

owner 洞察：渲染图只需"方向+强度"；filter 作用在 trace 过程（MS 每层后），最终图不需要富元数据；filter 在 emit 那跳要的是寄存器里"路径至今"——**连持久化记录都不需要**。

**结论（取代 §4.4 的"两个出口"判断，离散显存下）**：
- **纯融合 device 消费**：emit gate 当场 prob + device filter + device 内投影（scrum-315 起为**全部 11 种 LensType**，经共享 `src/core/shared/projection_shared.h::ProjectExitToPixel` 在 device 内 dispatch；此前仅 dual_fisheye/rectangular 两种）+ 补偿累加进 device XYZ buffer，**渲染图产物连 device 出射 buffer 都不物化**（生产者消费者融合，比 §5 图的 bulk 列更彻底）。富元数据降级为光路回溯的可选 device 旁路 tap。
- **GUI 与 CLI 走同一条 device 流水线**，唯一区别是回读 XYZ 的频率（显示节奏 vs 一次）——即第三个时钟。§4.4 的"两个出口按 workload 选"塌成"一条流水线、两个回读节奏"。
- **后端投影不再固定，但仍在 device 融合**（更新：scrum-315，2026-07-02）：设计当初（owner 2026-06-28）判断"GUI 只请求 dual_fisheye、其他镜头前端重投影"故后端只需 2 种固定投影；scrum-315 把 forward 投影统一进共享 `projection_shared.h`，device 融合消费端改为对 `proj_type` 做 dispatch，**全部 11 种投影（含新增 globe）都在 GPU 内完成**（CLI 批渲染所有镜头都吃 GPU 加速，不再静默回落 legacy CPU）。这与本节结论**正交**：仍是"一条 device 流水线、不物化出射 buffer"，只是投影从固定单支变成共享 dispatch——§4.2-§4.3 的"buffer-egress 换灵活性"顾虑在离散显存上依旧不成立。GUI 显示重投影那套（inverse 重采样固定 dual-fisheye 全天图）不进 shared、不变。
- **消费端（filter+投影+累加）= device 共享核**（`accum_shared.h` host/MSL/CUDA 单源）；Metal 也收编进同一核（统一性 + 单源），host consumer 只留 legacy CPU。

**落地状态（scrum-302）**：Metal（S1）+ CUDA（S2）device-fused 均落地、parity 全绿。⚠️ **但 CUDA 吞吐仍 0.16-0.40× legacy = 缺陷信号待追**（compute-bound + 疑 per-CI 串行 dispatch；见 `gpu-route-history.md` Phase 10 + backlog）——device-fused 是吞吐的**必要非充分**条件。

---

## 5. 统一后的 seam 形态（待验证草案）

```
host 一次/session:
  采 K 个晶体几何 (MakeCrystal, 单源)  ──上传──►  device: 三角形池 buffer
  场景分布参数 + RNG seed             ──────────►

device 巨型 dispatch (batch = 纯性能旋钮):
  线程 = 一条光线:
    RNG 选晶体索引 (池, K 受方差标定)
    GPU 采样: 取向 / 入射点 / 波长
    trace (穿 MS 各层, continuation 全程不出 device)
    逐跳记录 to_face_ → 定长 inline 面序列
    写出射记录 {dir, weight, crystal_id, 面序列}  ──► device: 出射记录 buffer  ◄── 规范 seam
                                                                  │
                          ┌───────────────────────────────────────┴───────────────────┐
                  GUI 策略 (灵活)                                          bulk 策略 (吞吐)
            出射记录 ──► host consumer:                          出射记录留 device ──► device:
              filter(对称折叠) + 投影 + 增量累加                    filter + 投影 + 累加
              (导出 O(光线), 解 exp#4 痛点)                         只回读图像(显示节奏/一次)
```

三时钟独立：几何=逐线程(池索引)、trace=dispatch 尺寸纯性能、回读=下游策略各自节奏。

> ✅ **第三时钟已落地（scrum-312，2026-07-01）**：readback 从 trace 时钟解耦到显示节奏（device 持久累加 +
> GUI 按显示节奏回读 / CLI 收尾一次，精度用 periodic-drain=float32+host Neumaier）。真实 GUI 分辨率
> 2048×1024 增益三机一致：4060Ti 28→39M / 1070Ti 12.5→33.5M / Metal 11→32.3M。**蓝图赌对了**——本节
> 预言的"回读走各自节奏"正是 312 兑现的形态。数字/机制见 `doc/performance-testing.md`「当前 canonical」+
> `doc/gpu-route-history.md` Phase 11。

---

## 6. 待定项（收敛前需各下判断）

1. 池化 K 的方差标定；device 侧池 vs host 上传一次。
2. 出射记录精确 schema：dir 精度、面序列定长宽度、crystal_id 宽度、**MS 多层作用域**。
3. 混合晶体类型控制 divergence：是否需 wavefront 排序(v1 可缓)。
4. 寄存器压力：复杂晶体(pyramid)面数多压低 occupancy。
5. explore-256 插桩去向：回退 vs 转正(`LUMICE_BATCH_RAY_NUM` 可能成正式旋钮)。

## 7. 关键风险点（各需最小原型实测）

- per-ray 不同晶体的 divergence 实测开销(vs 全批共享一晶体)。
- 富出射记录导出带宽(30B/ray × N vs 整幅图)在 GUI 默认分辨率的实测。
- buffer 出口 + consumer 投影在 GUI 默认 2048×1024 是否复现 legacy 量级吞吐(正面解 exp#4)。

## 8. 建议分步实现路线（草案，待探索验证后细化）

- **P0 验证(本探索)**：三个风险点各跑最小原型，定 schema 与 K，出数据流定稿。
- **P1 出口侧先行**：backend 改 buffer 出口(富出射记录)+ consumer 投影/filter。**单独即可解 owner 的 GUI 痛点**(导出 O(光线) 非 O(W×H))，且复用现有 consumer，风险最低。
- **P2 入口侧上移**：几何池上传 + per-ray 采样/晶体索引上 GPU，解耦几何时钟、消 host-bound。
- **P3 bulk 路径**：device 侧 filter+投影(image 出口)，服务 CUDA/offline 吞吐。
- parity 全程改统计级口径(累加图像 corr)。
