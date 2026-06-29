# 性能测试指南

本指南涵盖 Lumice 三个层次的性能测试，从纯管线基准测试到真实 GUI 交互测试。

所有命令假设工作目录为项目根目录。

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
[BENCHMARK] {"mode": "single", "workers": 1, "cores": 8, "rays": 2000000, "wall_sec": 8.5, "rays_per_sec": 235294.1}
[BENCHMARK] {"mode": "multi", "workers": 8, "cores": 8, "rays": 10000000, "wall_sec": 2.4, "rays_per_sec": 4166666.7}
```

**术语**："workers"指 simulator 线程（执行光线追踪的线程）。每个 server 实例还有 2 个
内部线程（场景生成 + 数据消费），总线程数 = workers + 2。

**并行扩展效率** = `multi_rps / (single_rps × workers)`。接近 1.0 表示良好扩展；
偏低说明存在锁竞争、内存带宽饱和或调度开销。

benchmark 模式的行为差异：
- **双趟运行**：依次创建两个独立 server 实例，指定不同 worker 数
  （单 worker pass 为 1，多 worker pass 为 `hardware_concurrency()`）
- **单 worker pass 光线数减少**：2M（而非配置原始值），以限制 CI 耗时
- **不写图片**：跳过 `SaveRenderResults`，`wall_sec` 纯反映模拟耗时
- **100ms 轮询间隔**（默认 1s）：将计时量化误差降至 ~0.1s

### 最新实测结果（每次更新追加）

> 每次重跑 bench 把最新数字追加这里。记**绝对 rays/sec**(不止倍数)、各 config 都有、附硬件摘要。**跨硬件数字不可比**——只在同一 host 块内读,别拿 Mac 行比 Linux 行。来源:`scripts/bench_throughput.py`(默认 dispatch,`ray_num`=20M,N≥5,CoV>15% 重跑 N=9)。

#### Mac — Apple M2 Max(12 核 8P+4E,32GB,macOS 14.7)· legacy vs Metal · 2026-06-28

| config | legacy single | legacy multi | Metal single | Metal multi | Metal/legacy(multi) |
|---|---|---|---|---|---|
| `bench_light_single_ms`（轻·单MS） | 573 K/s | 4.62 M/s | **27.8 M/s** | **30.6 M/s** | 6.62× |
| `ms_multi_crystal`（中·无filter） | 94 K/s | 759 K/s | 7.08 M/s | 7.66 M/s | 10.08× |
| `ms_multi_crystal_complex_filter`（重·标准） | 333 K/s | 1.54 M/s | 12.4 M/s | 12.8 M/s | 8.33× |
| `ms_multi_crystal_filtered_bd`（重·bd） | 368 K/s | 1.62 M/s | 13.7 M/s | 14.0 M/s | 8.59× |

- **Metal 在可比的轻·单MS 场景已 ~28–30 M/s,达到/超过 25 M/s 硬件能力目标。** 证明这个标尺在本 codebase 够得着,给 CUDA 一个同引擎目标(4060 Ti 与 M2 Max GPU 算力同级);CUDA 剩余差距是执行模型,不是 kernel 根本极限。

#### Linux — dev49 RTX 4060 Ti(AMD Zen5 9950X 16C/32T host）· legacy vs CUDA · 2026-06-29

scrum-304.2 buffer-persist 后,GPU idle-gate(跑前 `nvidia-smi` 实测 0%),`scripts/bench_throughput.py` 默认 dispatch(=32768),ray_num=20M,N≥5(CoV>15% 重跑 N=9)。

| config | legacy single | legacy multi | CUDA single | CUDA multi | CUDA/legacy(multi) |
|---|---|---|---|---|---|
| `bench_light_single_ms`（轻·单MS） | 744 K/s | 8.98 M/s | **35.2 M/s** | **55.9 M/s** | 6.23× |
| `ms_multi_crystal`（中·无filter） | 126 K/s | 1.44 M/s | 9.67 M/s | 13.7 M/s | 9.55× |
| `ms_multi_crystal_complex_filter`（重·标准） | 471 K/s | 6.26 M/s | 38.0 M/s | 60.3 M/s | 9.63× |
| `ms_multi_crystal_filtered_bd`（重·bd） | 510 K/s | 6.73 M/s | 39.1 M/s | 61.9 M/s | 9.19× |

- **竞品线 25M 已达到/超过**:可比轻·单MS 场景 **35–56 M/s**,≥ 25M 竞品目标,也 > Mac Metal 参照(28–30M)。仅 buffer-persist(304.2)就到这了;之前"0.16–0.40× / 1.5M"的悲观是**测量假象**——ad-hoc 非 idle-gate 在重场景 `ms_multi_crystal` 上测、用错标尺。务必 idle-gate + 用 canonical harness + 可比场景。
- **GPU 尚未满载**:nsys 轻场景(50M plain run)GPU active 17.6%(含 JIT/setup);benchmark steady 窗口 active ≈ 43%。trace kernel 占 GPU 95%。满载天花板 ≈ 129M rays/s(50M / 386ms kernel),现在 56M ≈ 43% → **还剩 ~2.3× headroom**,锁在每-dispatch host 串行(默认流 + per-layer 同步;explore-303 找到,304.2 未动)。要拿这部分=async-engine 工作,**现由 active% 数据支撑,非猜测**。
- 注:竞品确切配置(光谱/max_hits)未知,我们场景是合理的单晶单MS 代理(prism,D65,max_hits 7)。需精确 apples-to-apples 时对齐。

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

CUDA 吞吐在 dev49（RTX 4060 Ti，Linux，CUDA docker）测。**禁止每个 task 自己写 bench 脚本**——用 committed harness `scripts/bench_throughput.py`（已支持 CUDA env 覆盖、跑 canonical 场景集含可比的 `bench_light_single_ms`、确认 GPU 路由、median+CoV）。详见英文版 `performance-testing.md` 同名节 + `explore-cuda-step2-derisk/TOOLCHAIN.md`。

```bash
# dev49 CUDA docker 内（先 build -DLUMICE_CUDA_ENABLED=ON -DBUILD_SHARED_LIBS=ON）
LUMICE_BENCH_BIN=/work/build/Release/bin/Lumice \
LUMICE_BENCH_LIBDIR=/work/build/Release/lib \
LUMICE_BENCH_BACKENDS=legacy,cuda \
  python3 scripts/bench_throughput.py
# 只跑可比轻场景（对标 25M/s）：LUMICE_BENCH_CONFIGS=bench_light_single_ms
# dev49 共享机：严格吞吐须 idle 窗口；用完立刻清自己进程，别占机。
```

**⭐ 验收标尺 = 硬件能力,不是"× legacy CPU"。** 打过 legacy 只是必要门槛（floor），不是成功——GPU 可以"× legacy 赢"却只用 1–2% 利用率（scrum-304 踩过这坑）。GPU 后端真正的标尺是**这块卡的硬件能力**，锚到**可比工作负载 + 外部参照**：

| 场景 | 可比负载 | 硬件能力目标 | 来源 |
|---|---|---|---|
| `bench_light_single_ms`（轻·单MS） | 单晶 + 单 MS + 无续传 | **4060 Ti ≥ 25M rays/s** | 竞品实测 |

- 别拿重场景（多晶/多 MS）的数对标单晶单散射——逐光线工作量差一个数量级。
- GPU 数远低于硬件能力目标时,**无论对 legacy 比值多少都不算成功**;查利用率（CUDA 可用 nsys active%；Mac/Metal 无同口径 CLI 路径,故 active% 非硬性普适门,跨平台判据仍是绝对能力目标）+ 机制,别收尾。
- 到不了目标,交付物必须是 **profiler 落地的机制解释**（时间花哪了），不是"GPU 非天然胜"的黑盒话术。

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
| Multi rps | 多 worker rays/sec（并行吞吐���） |
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
