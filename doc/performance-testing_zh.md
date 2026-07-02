# 性能测试指南

本指南涵盖 Lumice 三个层次的性能测试，从纯管线基准测试到真实 GUI 交互测试。

所有命令假设工作目录为项目根目录。

> **范围**：本指南保留稳定的操作手册 + **当前 canonical** 吞吐参考。历史 per-run 实测详录
> （带日期的表、原始 reps、各 effort 的方法论）放在 git-ignored 的 `scratchpad/perf-results-log.md`
> ——新 per-run 数字追加那里，只有成为新 canonical 锚时才提升进本文档。远程机器上的 CUDA build +
> parity/正确性验证是另一件事——见 [`gpu-remote-cuda-build-testing.md`](gpu-remote-cuda-build-testing.md)。

## 日志级别

CLI 基准测试和 GUI 性能测试均支持日志级别选项。
**建议在两个级别下各运行一次**以获取完整数据：

| 级别 | 用途 | 开销 | 额外输出 |
|------|------|------|----------|
| **info**（默认） | 精确吞吐量数据 | 可忽略 | PERF 摘要 + Consume 剖析 |
| **verbose** | GUI/Poller 循环细节 | 低（~2-5%） | 每周期 staging、upload、质量门控决策 |
| **debug** | Consume 每批次分解 | macOS ~8%，**Windows ~54-62%** | ConsumeData 每批次锁/消费计时 |

**注意**：Windows debug 日志开销显著（吞吐量下降 2-3 倍）。
- 使用 **info** 级别数据进行吞吐量比较
- 使用 **debug** 级别数据分析 Consume 内部（比率可靠，绝对值偏高）

## 关键指标

两个目标存在张力——通过以下指标量化：

| 目标 | 指标 | 来源 | 描述 |
|------|------|------|------|
| **响应性** | `first_upload`（ms） | GUI 性能测试 / 日志分析 | Commit → 首次纹理上传成功；越低越好 |
| **渲染质量** | `upload_rays` 值 + CV | GUI 性能测试 / 日志分析 | 每次上传的光线数；绝对值越高、CV 越低越好 |

**注意**：低 CV 不代表高质量——1K rays 配低 CV 仍然很差。绝对值和稳定性都重要。

辅助指标：

| 指标 | 来源 | 描述 |
|------|------|------|
| `rays/sec` | CLI 基准 / GUI 性能测试 | 管线吞吐量 |
| `upload_ratio` | GUI 性能测试 | uploads/restarts，纹理交付率 |
| `texture FPS` | GUI 性能测试 steady_state | 稳态纹理刷新率 |
| `Consume profile` | CLI -v / GUI --log-level debug | 每批次 filter/proj/accum 分解 |

## 1. CLI 管线基准测试

不含 GUI、VSync 或显示开销的纯管线吞吐量测试。

### 配置

使用 `examples/bench_config.json`：1 晶体，1 渲染器，D65 光谱，10M 光线，max_hits=8。

### `--benchmark` 标志

`--benchmark` 标志运行双模式基准测试：先用单 worker 测量单核效率，再用全 worker 测量
并行吞吐量。输出两行 JSON：

```
[BENCHMARK] {"mode": "single", "workers": 1, "cores": 8, "rays": 2000000, "wall_sec": 8.51, "setup_sec": 0.01, "active_sec": 8.5, "rays_per_sec": 235294.1, "rate_basis": "steady"}
[BENCHMARK] {"mode": "multi", "workers": 8, "cores": 8, "rays": 10000000, "wall_sec": 0.6, "setup_sec": 0.02, "active_sec": 0.58, "rays_per_sec": 17241379.3, "rate_basis": "steady"}
```

`rays_per_sec` 是 `active_sec`（从首条光线追踪到 IDLE 的窗口）上的**稳态追踪率**，
**不是** `rays / wall_sec`。`setup_sec`（server alloc + 场景生成 + 首 dispatch 延迟）从分母
剔除；`rate_basis` 记录产出该率的路径。这个 setup-剔除修复（task-fix-throughput-bench-honesty）
对整个 run 只 ~0.2s 的快后端很重要——折进 setup 会把它们的 rays_per_sec 压低 >30%。

**两套独立的 `rate_basis` 阶梯**（消费者/gate 按走的哪条路径判定，不按全集字符串相等判）：
- **finite `ray_num`**（legacy CPU 趟，及任何 finite-config `--benchmark`）：`steady` /
  `active_short` / `wall_fallback`（上面的 honesty-fix 阶梯）。
- **`ray_num="infinite"`**（GPU 趟，task-gpu-bench-drain-aligned-rate）：`drain_aligned`（恰好
  测了 N 个整 drain 窗口）或 `too_few_drains`（未凑满 N drain 就退出——异常/不可信）。见下方
  drain-count-driven canonical。两套阶梯不共用同一字符串空间。

**术语**："workers"指 simulator 线程（执行光线追踪的线程）。每个 server 实例还有 2 个
内部线程（场景生成 + 数据消费），总线程数 = workers + 2。

**并行扩展效率** = `multi_rps / (single_rps × workers)`。接近 1.0 表示良好扩展；
偏低说明存在锁竞争、内存带宽饱和或调度开销。**仅对 legacy CPU 路线有意义**——见下方 GPU caveat。

> **⚠️ GPU 后端是单引擎——不存在 "single" vs "multi" 并行。** GPU 路线（Metal / CUDA）无条件
> `worker_count=1`（`server.cpp:284`）；只有 legacy CPU 路线是真多 worker（`worker_count =
> PhysicalCoreCount()`）。既然 GPU 的 "single" 与 "multi" 趟都跑在同一个单引擎（只差暖机+光线数、
> 非并行），**`--benchmark` 对 GPU 路线塌成 ONE 稳态趟**（label `mode="multi"`）、跳过暖机趟；
> legacy CPU 路线保留真双趟。路线检测是 env-aware 的（`LUMICE_WillUseGpuRoute` 认 `LUMICE_TRACE_BACKEND`，
> 故 env 选的 GPU run 也塌）。读 GPU 结果时：
> - GPU 的 `[BENCHMARK]` / `bench_throughput.py` 行**只有 `multi`**（`single`/`single_rps` 缺失
>   =预期、非 INCOMPLETE）。`multi` 数是稳态代表值。
> - **`efficiency` 对 GPU 不适用**（无 single/parallel 对；`explore-306.1` E1 `worker_count=1` 铁证）。
>   CI summary 的 efficiency 列仅对 legacy CPU 有意义。
> - 对旧数据：2026-07 前带 `single` 列的 GPU 表是旧双趟（single = 2M 光线暖机 run），那些 GPU `single`
>   数从来不是"单核"指标。

benchmark 模式的行为差异：
- **双趟运行**：依次创建两个独立 server 实例，指定不同 worker 数
  （单 worker pass 为 1，多 worker pass 为 `hardware_concurrency()`）
- **单 worker pass 光线数减少**：2M（而非配置原始值），以限制 CI 耗时
- **不写图片**：跳过 `SaveRenderResults`，`wall_sec` 纯反映模拟耗时
- **5ms 轮询间隔**（默认 1s）：把 IDLE 检测量化误差压到几毫秒（曾是 100ms，那单独就能给快 run 的
  wall time 加上一整个轮询间隔）

### Benchmark 场景注册表（canonical 吞吐场景）

> **吞吐比较的单一真源。** 永远引用本表里的真实 config——绝不臆造场景名。（本表存在是因为文档
> 曾引用不存在的 `ms3_multi_crystal_complex_filter` 并对标无法复现的数字；见
> task-fix-throughput-bench-honesty。）`scripts/bench_throughput.py` 跑 Metal-可比子集；两者保持同步。

测量口径：**引擎** = `Lumice --benchmark` multi pass，setup-剔除稳态率；**GUI** =
`gui_test perf_test` steady_state，无限 budget，reconstruct 路径。基线分母永远是 **legacy CPU**
（GUI 真实路径）——绝不用 `cpu_backend`。下方 `Metal vs legacy` 比值是 M2 Max、2026-06-19 回归锚
（`scratchpad/task-fix-throughput-bench-honesty/data/`）；任何吞吐改动前后须同会话重测。

| config（真实文件） | regime | rays / MS / filter | lens / Metal 可比 | 角色 | Metal vs legacy（锚） |
|---|---|---|---|---|---|
| `bench_light_single_ms.json` | 轻·单MS · 512×256 | 10M / 1 / 无 | dual_fisheye_EA ✅ | 轻场景吞吐基准（bench 专用，勿因 e2e 改动）；**512×256 落 GPU L2，系统性高估 GPU 优势**；真实分辨率用 `--res-sweep`（见"分辨率是一等吞吐维度"） | 引擎 ~1.7× / GUI ~1.8× |
| `ms_multi_crystal.json` | 中·无filter | 2M / 2 / 无 | dual_fisheye_EA ✅ | 无 culling 中等基准；`--res-sweep` 第二代表场景（看场景依赖） | 引擎 ~2.0× / GUI ~2.2× |
| `ms_multi_crystal_complex_filter.json` | 重·标准 | 2M / 2 / complex | dual_fisheye_EA ✅ | **G1 + G4 主基准** | 引擎 ~8.1× / GUI ~9.5× |
| `ms_multi_crystal_filtered_bd.json` | 重·bd | 2M / 2 / bd | dual_fisheye_EA ✅ | G1 第二基准 | 引擎 ~10.1× |
| `ms3_mixed_pyramid_heavy.json` | 最重·棱锥 | 5M / 3 / 4×raypath | dual_fisheye_EA ✅ | register-pressure 上界 | 引擎 **~5.5×**（M2 Max，2026-06-24；GUI 互证 ~5.7×）。注：legacy single pass ~274s 超出 `bench_throughput.py` 的 `RUN_TIMEOUT_SEC`，故该场景仍从自动跑中排除——基线靠手动大 timeout 单测取得 |
| `halo_22.json` | 轻·单MS | 10M / 1 / 无 | **fisheye_EA（单）→ CLI Metal 回退** | **e2e 资产，勿改**；legacy-only 轻基准 | N/A（Metal 不兼容此投影；轻·Metal 用 `bench_light_single_ms`） |

注意：
- **上表 4 个 light/mid/heavy 主基准（bench_light / ms_multi / complex_filter / filtered_bd）都是
  512×256**（`ms3_mixed_pyramid_heavy` 例外，为 2048×1024）。512×256 的 XYZ 累加 buffer（W×H×3
  float = 1.5MB）落在典型 GPU 的 L2 内，**系统性高估 GPU 吞吐**——真实 GUI 默认渲染 2048×1024
  （16× 像素，24MB buffer，越 L2）。**跨分辨率数字不可比；报吞吐必须带分辨率**。真实分辨率吞吐用
  `bench_throughput.py --res-sweep`（分辨率轴，默认 dispatch，隔离单变量）——详见下方"分辨率是一等
  吞吐维度"。
- dispatch 甜点是**后端 + 分辨率依赖**：Metal 32768 / CUDA 262144 是 **512×256** 下的甜点；**分辨率
  升高时 CUDA 最优 dispatch 显著上移**（2048×1024 下 ~786K–2M，因 per-batch readback 摊薄）。小
  dispatch 饿死 GPU（512/2048 = 0.2–0.8× legacy）。
- `bench_throughput.py` 每 run 把 `ray_num` override 到大值（temp config，不动 committed 文件），
  使稳态窗口足够长而稳定。
- **⚠️ 第三时钟 drain 路径曾逼出 `multi_wall` workaround（scrum-312）——现对 GPU 由 drain-count-driven
  `drain_aligned` 取代**（task-gpu-bench-drain-aligned-rate，2026-07-02）。根因：`multi_med`
  （steady `rays_per_sec`）的窗口以 `sim_ray_num` 为界，而它只在第三时钟 drain 时前进（每 64×dispatch
  rays；CUDA 默认 262144 → 16.8M rays/drain）。旧 bench `ray_num=20M` 只 ~1.19 drain → `sim_ray_num`
  近 0 → 大部分 tracing 误算 setup → **5× 假低**（explore-315：wall 报 24.7M，真 plateau 130M）。旧修
  = `multi_wall = rays/wall_sec`；新修 = GPU 设 `ray_num="infinite"` 直接测整数个 *drain 窗口*
  （`drain_aligned`），既免 setup 又 drain-粒度精确。**GPU 行现重新用 `rays_per_sec`（`multi_med`）报诚实
  稳态率**；`multi_wall` 只对 finite（legacy）趟有意义，`drain_aligned` GPU 行忽略之。per-batch legacy
  不受影响（finite `steady` 阶梯）。

#### 分辨率是一等吞吐维度（device-fused XYZ 累加 → cost 随 buffer vs GPU L2）

> **报任何 GPU 吞吐数字必须带渲染分辨率；跨分辨率不可比。** 这不是二阶细节——它常主导轻场景的 GPU
> 吞吐（轻场景 trace 便宜，累加/回读占大头）。

机制：device-fused XYZ 累加（scrum-302）把每条出射光线在 **trace kernel 内** `atomicAdd` 进
W×H×3 float 图像 buffer（12 B/px）。cost 随 buffer 相对 GPU L2 变化：

- buffer ≤ L2（512×256 = 1.5MB，落多数 GPU 的 ~2MB L2）→ 累加走 cache，快。
- buffer ≫ L2（2048×1024 = 24MB）→ 每次 atomicAdd 打 DRAM，DRAM 带宽绑定，慢。膝在 L2 边界
  （~512→768），越 L2 后随分辨率平滑衰减（非二元断崖）。

CUDA 路线还有 **per-batch 同步 readback** 税（`ReadbackXyzAccum` 每 SimBatch 一次
`cudaDeviceSynchronize` + 阻塞 24MB PCIe D2H + memset），scrum-312 已把它解耦到显示节奏（"第三时钟"，
`seam-design.md` §4.8）——见下方 canonical 结果。Metal 无此税（`StorageModeShared` 统一内存 + 延迟等）。

**流程要求**：

1. GPU 吞吐用 `bench_throughput.py --res-sweep` 扫多档分辨率（默认 `256×128 … 2048×1024` 六档，2:1，
   跨 L2 膝两侧），而非只报单点。脚本每档在 temp config 里 override `render[].resolution`（committed
   文件不动，同 ray_num override 机制）。默认扫 `bench_light_single_ms`（轻，累加/回读主导）+
   `ms_multi_crystal`（稍重，看场景依赖），`--res-list` / `--res-configs` 可覆盖。至少覆盖 **512×256
   （引擎天花板 / L2-resident）+ 2048×1024（GUI 真实体验）** 两点。
2. 与竞品 / 硬件能力对标（下方 25M bar）时**对齐分辨率**——我方历史 512×256 数字是 L2-resident 上界，
   非用户体验。
3. 数字入表**必标分辨率**（现有表默认 512×256，除 `ms3_mixed_pyramid_heavy`）。sweep 的**曲线数据不入
   committed 文件**（按机器/会话变），入表的是按 N≥5 CoV 协议正式产出的 canonical 点。

#### 验收标尺：硬件能力，不是"× legacy CPU"

> **`× legacy CPU` 比值是地板，不是成功标尺。** 打过 legacy 只是必要门槛，绝非目标——GPU 后端可以
> "× legacy 赢"却只用 1–2% 利用率（scrum-304 就踩过这坑：一个报成"~1.7× legacy 16 核"的 CUDA 数其实
> 是重于可比口径的场景上饿着的 GPU）。GPU 后端真正的标尺是**这块卡的硬件能力**，锚到可比工作负载 +
> 外部参照。

**已注册硬件能力目标**（绝对、场景锚——判 GPU 吞吐时引用这个，而非"× legacy"）：

| 场景 | 可比负载 | 硬件能力目标 | 来源 |
|---|---|---|---|
| `bench_light_single_ms`（轻·单MS） | 单晶 + 单 MS + 无续传 | **4060 Ti ≥ 25M rays/s** | 竞品实测——轻·单MS 是 apples-to-apples 对比 |

- 永远拿**可比**场景对标外部目标——别拿重场景（多晶/多 MS，如 `ms_multi_crystal`）的数去对标单晶单散射。
  逐光线工作量差一个数量级。
- GPU 数远低于硬件能力目标时，**无论对 legacy 比值多少都不算成功**；查利用率（CUDA 可用 nsys active%，
  见下）+ 机制，别收尾。
- 到不了目标，交付物必须是 **profiler 落地的机制解释**（时间花哪了），不是"GPU 非天然胜"的黑盒话术。
- **⚠️ 分辨率对齐**：25M bar 的竞品渲染分辨率未知；我方 `bench_light_single_ms` 是 **512×256
  （L2-resident，上界）**。真实 GUI 默认 2048×1024 下同卡同场景吞吐掉 3.6–5×（见"分辨率是一等吞吐
  维度"）。判定是否达标前必须**对齐分辨率**——别拿 512×256 的 L2-resident 数字宣称达 bar。用
  `bench_throughput.py --res-sweep` 取真实分辨率对标点。

### 当前 canonical 吞吐结果

> 当前权威参考数字。**跨硬件数字不可比**——只在同一 host 块内读。历史 per-run 详录（带日期的表、原始
> reps、各 effort 方法论）在 `scratchpad/perf-results-log.md`。GPU 稳态率 = `drain_aligned`
> `rays_per_sec`（见下）；较旧的 scrum-312 res-sweep 块读 `multi_wall`。

#### drain-count-driven canonical · default dispatch · config 默认分辨率 · `drain_aligned` `rays_per_sec` · 2026-07-02

**背景**（task-gpu-bench-drain-aligned-rate）：GPU `--benchmark` 现设 `ray_num="infinite"`，恰好测
N=10 个整 drain 窗口（`rate_basis="drain_aligned"`），修复 drain-量化假低（explore-315：旧 finite 20M 下
CUDA 只 ~1.19 drain → 5× 假低）。下表是各 config **默认分辨率**下的诚实稳态 `rays_per_sec`（**不是**
scrum-312 分辨率 sweep——两块是不同 metric/轴，不可逐行比较）。每格 N reps，>15% CoV → N=9 escalation。

| config | dev49 4060Ti CUDA (vs legacy) | win-builder 1070Ti CUDA | Mac Metal ⚠️(近似) | dev49 legacy CPU |
|---|---|---|---|---|
| `bench_light_single_ms` | **130.4 M/s** (15.6×, CoV 0.1%) | 71.8 M/s (1.1%) | ~69 M/s (CoV 11%) | 8.34 M/s |
| `ms_multi_crystal` | 21.3 M/s (17.2×, 10.2%) | 12.6 M/s (2.5%) | ~16.7 M/s (8.3%) | 1.24 M/s |
| `ms_multi_crystal_complex_filter` | 411.3 M/s (67.6×, 1.8%) | 99.0 M/s (1.0%) | ~24.6 M/s (18% 热) | 6.09 M/s |
| `ms_multi_crystal_filtered_bd` | 660.3 M/s (110.7×, 1.4%) | 135.3 M/s (0.6%) | ~26.7 M/s (8.4%) | 5.97 M/s |

**要点**：
- **5× 假低已修**：`bench_light_single_ms` 4060Ti 读 **130.4 M/s** @0.1% CoV，命中 explore-315 独立实测
  plateau（400M-ray wall = 130.2 M/s）。这才是诚实 CUDA 稳态率；旧 finite-20M bench 读 24.7 M/s（5× 假低）。
- **锁频桌面（CUDA）是权威 canonical**：4060Ti 与 1070Ti CoV 0.1–2.5%。drain-count-driven 在 Ada/Pascal(sm_61) 一致。
- **⚠️ Mac Metal 是 phase-1 近似**，非 canonical。Mac *笔记本* Metal 吞吐由热/GPU-boost 主导，跨 run 摆动
  ~2×（CoV 8–28%）；任何 N 都稳不住（N-sweep 证 CoV 不随 N 单调降——超 ~N=10 热漂移反使方差回增）。上表 Metal
  是单 run 中位数，只作数量级参考。
- **关于旧「4060Ti@512×256 = 110 M/s」（scrum-312 res-sweep `multi_wall`）**——非矛盾：那是*固定 512×256*
  的 `multi_wall`，与此处 `drain_aligned` 默认分辨率率是不同 metric+分辨率。drain-count-driven 130.4 M/s
  是诚实稳态率、取代 finite-bench 数作为 GPU 率基准；下方 res-sweep 块保留其分辨率依赖分析价值。
- 正确性（本次纯测量改动不影响）：CUDA parity 10/10 @4060Ti + 10/10 @1070Ti/sm_61；Metal parity 完好。
  Mac G1 gate `test_metal_throughput.py` 仍绿。

#### scrum-312 第三时钟 canonical · `--res-sweep` · `multi_wall` · 2026-07-01

**背景**：readback 从 trace 时钟解耦到显示节奏第三时钟（seam-design §4.8）。真实 GUI 分辨率
2048×1024 下 readback/per-batch-copy 税被摊薄。`bench_light_single_ms`（轻·单MS，L2/readback 主导），
per-resolution `multi_wall`：

| host / backend | 256×128 | 512×256 | 1024×512 | 1536×768 | **2048×1024** |
|---|---|---|---|---|---|
| dev49 RTX 4060Ti (Ada) CUDA | 116 M/s | 110 M/s | 92 M/s | 65 M/s | **39.2 M/s** |
| win-builder GTX 1070Ti (Pascal) CUDA | 77 M/s | 69 M/s | 45 M/s | 40 M/s | **33.5 M/s** |
| Mac M-series Metal | 28.1 M/s | 30.3 M/s | 31.2 M/s | 35.1 M/s | **32.3 M/s** |
| dev49 legacy CPU (baseline) | 9.0 M/s | 8.8 M/s | 8.4 M/s | 7.7 M/s | 6.9 M/s |
| Mac legacy CPU (baseline) | 5.1 M/s | 4.8 M/s | 4.7 M/s | 4.8 M/s | 4.7 M/s |

**第三时钟在 2048×1024 的增益**（vs per-batch drain 旧值，interleaved 同 binary 隔离 drain cadence）：

| host / backend | 旧（per-batch） | 第三时钟 | 增益 | 机制 |
|---|---|---|---|---|
| 4060Ti (Ada, 32MB L2) CUDA | 28 M/s | 39 M/s | **1.4×** | Ada L2 大→旧值已 L2-resident，税主要是 per-batch D2H readback |
| 1070Ti (Pascal, 2MB L2) CUDA | 12.5 M/s | 33.5 M/s | **2.7×** | 兼有 L2 溢出 + readback 税；第三时钟消 readback（L2 残留） |
| M-series Metal（统一内存） | 11 M/s | 32.3 M/s | **~3×** | 无 PCIe readback；per-batch 24MB memset+memcpy(×76 batch≈3.6GB) 被摊薄 |

**要点**：
- **三种硬件（CUDA-Ada / CUDA-Pascal / Metal）一致确证第三时钟在真实 GUI 分辨率显著提速**；增益随
  "旧值里 per-batch readback/copy 占比"放大。
- **Metal 曲线近平**（28→35 M/s 跨 6 档），第三时钟使 Metal 分辨率-鲁棒（旧路径每 batch 全幅
  memset+memcpy 在高分辨率是真成本，非仅 CUDA readback）。**推翻早先"Metal 统一内存零收益"判断**。
- 正确性：CUDA parity 10/10 @4060Ti + 10/10 @1070Ti/sm_61；Metal parity 14/14（含 batch-invariance
  = drain-cadence 独立性）。
- win-builder 1070Ti 未列 vs-legacy（CPU 弱，比值虚高，读绝对值）；`multi_med` 列在第三时钟下失真已弃用
  （用 `multi_wall`）。
- 重场景 `ms_multi_crystal`（trace 主导，readback 占比小）增益温和：4060Ti 2048 = 14.9 M/s、1070Ti =
  7.3 M/s、Metal ≈ 17 M/s@256（Mac 热噪大，N=9 后仍抖，需稳定机复测 canonical）。

### macOS

```bash
# 构建
./scripts/build.sh -j release

# benchmark 模式（推荐——结构化输出，不写图片）
./build/cmake_install/Lumice --benchmark -f examples/bench_config.json -o /tmp

# 手动模式（info 级别——带图片输出）
time ./build/cmake_install/Lumice -f examples/bench_config.json -o /tmp 2>&1 \
  | grep -E "Consume profile|Stats:"

# 手动模式（debug 级别——Consume 分解）
time ./build/cmake_install/Lumice -f examples/bench_config.json -v -o /tmp 2>&1 \
  | grep -E "Consume profile|Stats:"
```

关键输出：
- `--benchmark`：两行 `[BENCHMARK]` JSON（单 worker + 多 worker），含单核和并行吞吐量数据
- 手动模式：`Consume profile` 行 + `time` 墙钟时间 → 吞吐量 = 10M / 墙钟秒数

### Windows

通过 CI 构建，然后传输二进制：

```bash
# 下载 CI 产物
gh run download <RUN_ID> --name gui-test-windows-msvc --dir /tmp/ci-win

# 传输二进制和配置到 Windows 机器
scp /tmp/ci-win/bin/Lumice.exe <windows-host>:<path>/
scp examples/bench_config.json <windows-host>:<path>/

# benchmark 模式（SSH / PowerShell）
.\Lumice.exe --benchmark -f bench_config.json -o .

# 手动模式（PowerShell 墙钟时间）
Measure-Command { .\Lumice.exe -f bench_config.json -o . 2>&1 | Out-Null } | Select-Object TotalSeconds
```

### Linux / CUDA（dev49）

CUDA 吞吐在 dev49（RTX 4060 Ti，Linux，CUDA docker）测。两机完整 build + parity/正确性 recipe
（源码同步、docker/BuildTools 工具链、`LUMICE_HAS_CUDA` un-skip 闸、parity battery）见
[`gpu-remote-cuda-build-testing.md`](gpu-remote-cuda-build-testing.md)。**禁止每个 task 自己写 bench
脚本**——用 committed harness `scripts/bench_throughput.py`（已支持 CUDA env 覆盖、跑 canonical 场景集
含可比的 `bench_light_single_ms`、确认 GPU 路由、median+CoV）。

```bash
# dev49 CUDA docker 内（先 build -DLUMICE_CUDA_ENABLED=ON -DBUILD_SHARED_LIBS=ON）
LUMICE_BENCH_BIN=/work/build/Release/bin/Lumice \
LUMICE_BENCH_LIBDIR=/work/build/Release/lib \
LUMICE_BENCH_BACKENDS=legacy,cuda \
  python3 scripts/bench_throughput.py
# 只跑可比轻场景（对标 25M/s）：LUMICE_BENCH_CONFIGS=bench_light_single_ms
# dev49 共享机：严格吞吐须 idle 窗口；用完立刻清自己进程，别占机。
```

#### GPU active% 诊断（仅 CUDA——Metal/Mac 无同口径 CLI 路径）

单看吞吐数字无法判断 GPU 是被喂饱还是饿着。CUDA/dev49 上把 bench 配一个 `nsys` capture 读 GPU 利用率
——这是区分"真赢"与"GPU 空转还打过 CPU 的假赢"的诊断。profiler 已在 dev49 de-risk（nsys 2023.4.4 +
ncu 2024.1.1；安装 + 用法见 `explore-cuda-step2-derisk/TOOLCHAIN.md`）。active% **非硬性普适门**——
macOS 上 Metal 无同口径 CLI active% 路径，故跨平台验收判据仍是上方绝对硬件能力目标；active% 是 CUDA 侧
佐证"为何这个数字高或低"的诊断。

```bash
# GPU active% = (sum of cuda_gpu_kern_sum) / wall，取自一次代表性 run 的 nsys timeline。
# 低 active%（如 1–2%）=> GPU 饿着 => 吞吐数字是 host-bound，非硬件能力结果。
nsys profile --trace=cuda --stats=true -o /tmp/rep <Lumice ...>   # 见 TOOLCHAIN.md
```

### CI 自动化基准测试

每次 push 会在所有 4 个 CI 平台（Ubuntu x64/ARM、macOS ARM、Windows MSVC）上运行 CLI
benchmark。结果汇总到 workflow run 页面的 summary 表格（`$GITHUB_STEP_SUMMARY`）。
详见 `.github/workflows/ci.yml`。

summary 表包含硬件上下文和双模式结果：

| 列 | 说明 |
|----|------|
| CPU | CPU 型号（运行时自动检测） |
| Cores | 逻辑核心数（`hardware_concurrency()`） |
| Workers | 多 worker pass 使用的 simulator 线程数 |
| Single rps | 单 worker rays/sec（单核效率） |
| Multi rps | 多 worker rays/sec（并行吞吐量） |
| Efficiency | `multi_rps / (single_rps × workers)`——并行扩展效率 |

**注意**：`Single rps` 是跨平台比较最有意义的指标，因为它自然包含了 IPC、
缓存层次和内存带宽差异，无需 GHz 归一化。`Efficiency` 揭示各平台特有的扩展瓶颈。

### 历史趋势

push 到 `main` 的 benchmark 结果会通过
[github-action-benchmark](https://github.com/benchmark-action/github-action-benchmark)
自动存储到 `gh-pages` 分支。Chart.js 仪表盘地址：

**https://lovedaisy.github.io/ice_halo_sim/bench/**

仪表盘追踪 12 条时间序列（4 平台 × 3 指标）：

| 指标 | 单位 | 说明 |
|------|------|------|
| `<Platform> / Single` | rays/sec | 单 worker 吞吐量（单核效率） |
| `<Platform> / Multi` | rays/sec | 多 worker 吞吐量（并行管线） |
| `<Platform> / Efficiency` | % | `multi_rps / (single_rps × workers) × 100`——自参照并行扩展指标 |

**如何解读图表**：
- Single/Multi rps **突然下降**可能是代码回归，也可能是 CI runner 硬件变更（查看 tooltip 中的 CPU 型号）
- **Efficiency** 对 runner 硬件变化免疫——Efficiency 下降可靠地表示并行扩展回归（如锁竞争加剧）
- 告警阈值设为 200%（性能下降超过 50% 才触发 commit 评论）

**已知限制**：
- 仅存储 `main` 分支 push 的历史；功能分支 benchmark 仅在 per-run summary 表中显示
- CI runner 硬件可能不经通知就更换，导致绝对 rps 指标出现阶跃变化
- 告警阈值是全局的（所有指标共享 200%）；Efficiency 回归低于阈值时需人工巡检图表

### 运行时调优旋钮

| 环境变量 | 默认 | 说明 |
|----------|------|------|
| `LUMICE_DISPATCH_RAY_NUM` | **262144**（CUDA）/ **32768**（Metal）/ 128（CPU） | task-268.4 旋钮；GPU 每-dispatch 网格大小。scrum-268.6 定 Metal 默认 32768；scrum-306.2 定 CUDA 默认 262144（都是实测甜点）。摊薄 GPU kernel 启动 + per-batch host 开销；小 dispatch 饿死 GPU（512/2048 = 0.2–0.8× legacy，128 在大 ray_num 下 hang）。设为 2 的幂对齐。server 启动时读取，运行中改无效。与 `LUMICE_COMMIT_RAY_NUM` 独立——喂大 GPU 而不牺牲 GUI 节奏。**分辨率依赖**：262144/32768 甜点测于 **512×256**；分辨率升高最优上移（2048×1024 下 CUDA ~786K–2M，实测 262144→~1M = 2.25×），因 per-batch readback buffer 16× 大、需更大 dispatch 摊薄。分辨率变时重标（见"分辨率是一等吞吐维度"）。 |
| `LUMICE_COMMIT_RAY_NUM` | 128 | task-268.4：`ConsumeData` 内 SimData-to-consumer 提交粒度。更小提交 = 更细 GUI 快照节奏（与 dispatch 大小无关）。仅 backend exit-seam 路径（legacy CPU SimData 绕过 chunker）。**⚠️ GPU device-fused 路线（Metal/CUDA，`HasDeviceXyzAccum()`==true）上是 no-op**：该路径产出 `xyz_pixel_data_` 而非 `outgoing_d_`，commit chunker（`server.cpp:809`）被跳过；device readback 频率实由 `LUMICE_DISPATCH_RAY_NUM`（每 SimBatch 一次 `ReadbackXyzAccum`）决定，**非本旋钮**。（曾误用本旋钮做"readback 无关"判据，无效。） |
| `LUMICE_BATCH_RAY_NUM` | （已废弃） | 仅作 `LUMICE_COMMIT_RAY_NUM` 的向后兼容回退。task-268.4 前它同时兼作 dispatch 和 commit 粒度。scrum-268：DISPATCH 拆分是 GPU 吞吐的主驱动；现在设 `LUMICE_BATCH_RAY_NUM` 只控 commit 节奏、不控 GPU dispatch 大小。优先用上面两个拆分 env。 |
| `LUMICE_TRACE_BACKEND` | 未设（legacy CPU） | trace 后端选择：未设 = legacy CPU；`metal` = Metal GPU 后端；`cuda` = CUDA GPU 后端；`cpu_backend` = SoA CPU 后端。 |
| `LUMICE_DISABLE_DEVICE_GEN` | 未设（device-gen 开） | 逃生舱：强制 GPU 后端走 **host** root-ray 生成。device root-gen（GPU PCG root-ray 供给）在合格层（single-crystal-per-ci，`tri_count ≤ 64`）是**默认**。仅在镜像 host `std::mt19937` 流的严格-一致 parity 测试里设 `1`（device PCG 流无法与之对齐）。每个后端实例构造时读一次。 |

> #### ⚠️ `LUMICE_DISPATCH_RAY_NUM` 是 GPU 专属旋钮——比较中绝不施给 legacy（scrum-306.1/306.4）
>
> `LUMICE_DISPATCH_RAY_NUM` 定 GPU 引擎的 per-dispatch 网格。**CUDA 总能量 dispatch-不变**
> （实测：ΣY = 261.29 M ±0.001% 跨 dispatch ∈ {128, 8192, 32768, 131072}，`dual_fisheye_ref`）——
> 即 GPU 结果在任何 dispatch 下都正确。**legacy（CPU）总能量 dispatch-不不变**：同旋钮（override
> legacy 的 `kDefaultRayNum`=128）使 legacy ΣY 波动 **−5%..+13%**——这是 per-batch 波长采样的
> Monte-Carlo 方差，**非 bug**（默认 128 收敛正确；见 per-run log 里 scrum-306.7）。
>
> **须避免的历史误诊**：为探 CUDA 而全局设 `LUMICE_DISPATCH_RAY_NUM=131072`，使
> `test_cuda_single_ms_no_filter_parity` 报 `energy_ratio=0.8922`——曾被**误归为"CUDA
> exit-cap/cont-cap 静默丢能量"**（原 scrum-304.3 backlog 条）。这不是 CUDA bug：旋钮漏进 legacy
> 参考 run、膨胀了**分母**（legacy_Y），`261.29/292.87 = 0.892`。parity harness 现对 `legacy` 后端
> 剥掉 `LUMICE_DISPATCH_RAY_NUM`（`test/e2e/capi_runner.py`，scrum-306.4），使 oracle 停在 canonical
> 默认，`energy_ratio` 只反映 GPU 后端正确性（复验：@131072 0.8922 FAIL → 1.0063 PASS）。
> `bench_throughput.py` 已把 legacy 排除出 dispatch sweep（`DISPATCH_PLAN["legacy"]=[None]`）。
>
> **过程教训**：304.3 的正确性断言只以 backlog 一行存在、无保留脚本 → 一个错诊断（CUDA）传播且无法
> 对齐。正确性断言必须落进 tracked 文档 + 可复现 recipe。复现：容器 `pip install pytest numpy`，
> `LUMICE_HAS_CUDA=1`，然后
> `LUMICE_DISPATCH_RAY_NUM=131072 pytest -m slow test/parity-cross-backend/backend/test_cuda_exit_seam_parity.py::test_cuda_single_ms_no_filter_parity`；
> 逐 dispatch ΣY 拆分用 `bench_work/harness2.cpp`（dev49）。

**GPU device root-gen（scrum-260）**：GPU 后端上，root 光线（取向 / 方向 / 入射点）经 counter-based
PCG 流 `(gen_seed, gen_ray_base + tid)` 在 device 上生成，替代 host 预生成 + 上传。这是默认路径；对
legacy 的统计等价性由 slow-e2e parity harness 验证（`ds_corr ≥ 0.99`）。device-gen ON/OFF 吞吐刻画
（Metal，phase-1）——默认 batch 下 device-gen 对单晶单散射是净亏、对多晶多 worker 是净赚，大 batch 才
有强赢——记录在 `scratchpad/perf-results-log.md`。

## 2. GUI 性能测试（隐藏窗口，无 VSync）

由 ImGui Test Engine 驱动的自动化 GUI 测试。使用隐藏窗口和 `swapInterval(0)`，
因此**不反映真实显示/VSync 开销**。

默认应用 16ms 帧率限制（匹配真实应用的 `kTargetFrameTimeMs` 回退），以产生现实基线指标。
使用 `--no-frame-limit` 禁用以进行原始无限 FPS 比较。

两个测试场景：
- **steady_state**：2 秒累积——测量 rays/sec + 纹理 FPS
- **slider_drag**：5 秒交替参数变化——测量 rays/sec + upload ratio + first_upload 延迟 + 每次上传光线数 CV

### 诊断标志

| 标志 | 默认 | 描述 |
|------|------|------|
| `--visible` | 关 | 显示 GLFW 窗口（用于 display/DWM 测试） |
| `--vsync` | 关 | 启用 VSync（隐含 `--visible`） |
| `--frame-limit` | 开 | 应用 16ms sleep 帧率限制 |
| `--no-frame-limit` | — | 禁用帧率限制以测试原始吞吐量 |
| `--main-loop-commit` | 关 | 在主循环中调用 `CommitConfig`（忠实模拟真实应用） |
| `--log-panel` | 关 | 测试期间显示日志面板 |
| `--dorun-delay <ms>` | 0 | 为 `DoRun` 添加人为延迟（模拟慢环境） |
| `--skip-calibration` | 关 | 跳过启动时质量阈值校准 |

### macOS

```bash
# 构建（需要 -gt 生成 GUI 测试目标）
./scripts/build.sh -gtj release

# 仅运行性能测试（PERF 输出在 stderr，服务器日志在 stdout）
./build/Release/bin/gui_test --filter perf_test \
  > /tmp/perf_stdout.txt 2>/tmp/perf_stderr.txt
grep "\[PERF\]" /tmp/perf_stderr.txt

# debug 级别（增加 ConsumeData 每批次 + Consume 剖析）
./build/Release/bin/gui_test --filter perf_test --log-level debug \
  > /tmp/perf_stdout_debug.txt 2>/tmp/perf_stderr_debug.txt
grep "Consume profile" /tmp/perf_stdout_debug.txt
```

**注意**：PERF 结果在 **stderr**，服务器日志在 **stdout**——需要分别重定向。

### Windows

```bash
# 从 CI 产物传输 GUI 测试二进制
scp /tmp/ci-win/bin/gui_test.exe <windows-host>:<path>/

# 运行（PowerShell，必须使用 --filter 避免非性能测试崩溃）
$proc = Start-Process -FilePath ".\gui_test.exe" `
  -ArgumentList "-nopause","--filter","perf_test" `
  -NoNewWindow -Wait `
  -RedirectStandardOutput "perf_stdout.txt" `
  -RedirectStandardError "perf_stderr.txt" `
  -PassThru
Write-Host "Exit code: $($proc.ExitCode)"

# 查看结果
Get-Content perf_stderr.txt
```

**已知问题**：在 Windows SSH 下运行完整 GUI 测试套件会导致 ACCESS_VIOLATION。
使用 `--filter perf_test` 仅运行性能测试。

### 完整流程

每个平台运行两次：
1. 不设 `--log-level`（默认 info）——用于精确吞吐量数据
2. 设置 `--log-level debug`——用于 Consume 剖析分解

## 3. GUI 手动测试（可见窗口，VSync）

反映真实用户体验。需要带可见窗口的显示环境。

### 步骤

1. 启动 GUI 应用（macOS：双击 .app，Windows：直接运行 .exe）
2. 加载配置，点击 Run
3. **稳态测试**：等待约 5 秒累积，观察 rays/sec
4. **拖拽测试**：持续拖拽晶体高度滑动条约 10 秒
5. 启用文件日志（GUI Log 面板 → Enable File Log）
6. 使用分析脚本分析日志文件

### Windows 远程测试

无需物理接触即可测试 Windows，使用 watcher 远程工作流。
参见 [Windows 远程测试指南](windows-remote-testing_zh.md) 了解设置说明。

```bash
# 在 Windows 上运行带 VSync 的 GUI 性能测试
./scripts/win_remote_test.sh /tmp/ci-win/bin/gui_test.exe \
  --filter perf_test --vsync --log-level verbose
```

### 对比

| 条件 | GUI 性能测试 | 手动测试 |
|------|-------------|---------|
| 窗口 | 隐藏（默认）/ 可见（`--visible`） | 可见 |
| VSync | 关（默认）/ 开（`--vsync`） | 开（系统默认） |
| 帧率限制 | 开（默认）/ 关（`--no-frame-limit`） | 开 |
| 输入 | 自动化（ImGui Test Engine） | 手动（滑动条拖拽） |
| 可重复性 | 高 | 低（人为差异） |
| 反映真实体验 | 部分（配合 `--visible --vsync`） | 是 |

## 4. 日志分析脚本

`scripts/analyze_perf_log.py` 解析 GUI/Poller 日志文件，检测操作阶段
（STEADY / DRAG / PAUSE），并报告每阶段性能指标。

```bash
# 文本输出（比较多个文件）
python scripts/analyze_perf_log.py log1.log log2.log

# 带可视化（PNG 图表）
python scripts/analyze_perf_log.py -p log1.log log2.log

# 过滤时间范围（从日志开始的秒数）
python scripts/analyze_perf_log.py --from 2.0 --to 8.0 log.log

# 输出图表到指定目录
python scripts/analyze_perf_log.py -p -o /tmp/perf_plots log.log
```

主要功能：
- 阶段检测：STEADY（硬件吞吐量）、DRAG（交互响应性）、PAUSE（恢复）
- 每周期指标：首次上传延迟、上传光线数、commit 延迟
- 首次上传分解：commit 延迟 + 门控等待
- 多文件时的对比表格和 CDF 图表
