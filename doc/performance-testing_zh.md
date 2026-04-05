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
gh run download <RUN_ID> --name LumiceGUITests-windows-msvc --dir /tmp/ci-win

# 传输二进制和配置到 Windows 机器
scp /tmp/ci-win/bin/Lumice.exe <windows-host>:<path>/
scp examples/bench_config.json <windows-host>:<path>/

# benchmark 模式（SSH / PowerShell）
.\Lumice.exe --benchmark -f bench_config.json -o .

# 手动模式（PowerShell 墙钟时间）
Measure-Command { .\Lumice.exe -f bench_config.json -o . 2>&1 | Out-Null } | Select-Object TotalSeconds
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
| Multi rps | 多 worker rays/sec（并行吞吐���） |
| Efficiency | `multi_rps / (single_rps × workers)`——并行扩展效率 |

**注意**：`Single rps` 是跨平台比较最有意义的指标，因为它自然包含了 IPC、
缓存层次和内存带��差异，无需 GHz 归一化。`Efficiency` 揭示各平台特有的扩展瓶颈。

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
| `--perf-bench` | 关 | 运行 `--perf-bench` 模式：独立吞吐量测量 |

### macOS

```bash
# 构建（需要 -gt 生成 GUI 测试目标）
./scripts/build.sh -gtj release

# 仅运行性能测试（PERF 输出在 stderr，服务器日志在 stdout）
./build/Release/bin/LumiceGUITests --filter perf_test \
  > /tmp/perf_stdout.txt 2>/tmp/perf_stderr.txt
grep "\[PERF\]" /tmp/perf_stderr.txt

# debug 级别（增加 ConsumeData 每批次 + Consume 剖析）
./build/Release/bin/LumiceGUITests --filter perf_test --log-level debug \
  > /tmp/perf_stdout_debug.txt 2>/tmp/perf_stderr_debug.txt
grep "Consume profile" /tmp/perf_stdout_debug.txt
```

**注意**：PERF 结果在 **stderr**，服务器日志在 **stdout**——需要分别重定向。

### Windows

```bash
# 从 CI 产物传输 GUI 测试二进制
scp /tmp/ci-win/bin/LumiceGUITests.exe <windows-host>:<path>/

# 运行（PowerShell，必须使用 --filter 避免非性能测试崩溃）
$proc = Start-Process -FilePath ".\LumiceGUITests.exe" `
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

### 自动化 --perf-bench 模式

`--perf-bench` 标志提供独立吞吐量测量，运行固定时长后报告平均 rays/sec。
适用于不同构建或平台之间的 apple-to-apple 比较，无需 GUI 交互。

```bash
# macOS
./build/cmake_install/LumiceGUI --perf-bench

# Windows（通过远程 watcher，参见 doc/windows-remote-testing_zh.md）
./scripts/win_remote_test.sh ./LumiceGUI.exe --perf-bench
```

### Windows 远程测试

无需物理接触即可测试 Windows，使用 watcher 远程工作流。
参见 [Windows 远程测试指南](windows-remote-testing_zh.md) 了解设置说明。

```bash
# 在 Windows 上运行带 VSync 的 GUI 性能测试
./scripts/win_remote_test.sh /tmp/ci-win/bin/LumiceGUITests.exe \
  --filter perf_test --vsync --log-level verbose
```

### 对比

| 条件 | GUI 性能测试 | 手动测试 | --perf-bench |
|------|-------------|---------|--------------|
| 窗口 | 隐藏（默认）/ 可见（`--visible`） | 可见 | 可见 |
| VSync | 关（默认）/ 开（`--vsync`） | 开（系统默认） | 开（系统默认） |
| 帧率限制 | 开（默认）/ 关（`--no-frame-limit`） | 开 | 开 |
| 输入 | 自动化（ImGui Test Engine） | 手动（滑动条拖拽） | 无（仅稳态） |
| 可重复性 | 高 | 低（人为差异） | 高 |
| 反映真实体验 | 部分（配合 `--visible --vsync`） | 是 | 部分 |

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
