# Performance Testing Guide

This guide covers three levels of performance testing for Lumice, from pure pipeline
benchmarking to real-world GUI interaction testing.

All commands assume the working directory is the project root.

## Log Level

Both CLI benchmark and GUI perf test support log level options.
**Run at both levels** for complete data:

| Level | Purpose | Overhead | Extra output |
|-------|---------|----------|-------------|
| **info** (default) | Accurate throughput numbers | Negligible | PERF summary + Consume profile |
| **verbose** | GUI/Poller cycle details | Low (~2-5%) | Per-cycle staging, upload, quality gate decisions |
| **debug** | Consume per-batch breakdown | macOS ~8%, **Windows ~54-62%** | ConsumeData per-batch lock/consume timing |

**Important**: Windows debug logging overhead is significant (2-3x throughput drop).
- Use **info** level data for throughput comparisons
- Use **debug** level data for Consume internal breakdown (ratios are reliable, absolute values are inflated)

## Key Metrics

Two goals are in tension — quantified by these metrics:

| Goal | Metric | Source | Description |
|------|--------|--------|-------------|
| **Responsiveness** | `first_upload` (ms) | GUI perf test / log analysis | Commit → first successful texture upload; lower is better |
| **Render quality** | `upload_rays` value + CV | GUI perf test / log analysis | Rays per upload; higher absolute value and lower CV is better |

**Note**: Low CV alone doesn't mean good quality — 1K rays with low CV is still poor.
Both absolute value and stability matter.

Supporting metrics:

| Metric | Source | Description |
|--------|--------|-------------|
| `rays/sec` | CLI benchmark / GUI perf test | Pipeline throughput |
| `upload_ratio` | GUI perf test | uploads/restarts, texture delivery rate |
| `texture FPS` | GUI perf test steady_state | Steady-state texture refresh rate |
| `Consume profile` | CLI -v / GUI --log-level debug | Per-batch filter/proj/accum breakdown |

## 1. CLI Pipeline Benchmark

Pure pipeline throughput test without GUI, VSync, or display overhead.

### Configuration

Use `examples/bench_config.json`: 1 crystal, 1 render, D65 spectrum, 10M rays, max_hits=8.

### macOS

```bash
# Build
./scripts/build.sh -j release

# Run (info level — accurate throughput)
time ./build/cmake_install/Lumice -f examples/bench_config.json -o /tmp 2>&1 \
  | grep -E "Consume profile|Stats:"

# Run (debug level — Consume breakdown)
time ./build/cmake_install/Lumice -f examples/bench_config.json -v -o /tmp 2>&1 \
  | grep -E "Consume profile|Stats:"
```

Key output:
- `Consume profile` line: per-batch average with filter/proj/accum breakdown
- `time` wall time → throughput = 10M / wall_seconds

### Windows

Build via CI, then transfer the binary:

```bash
# Download CI artifact
gh run download <RUN_ID> --name LumiceGUITests-windows --dir /tmp/ci-win

# Transfer binary + config to Windows machine
scp /tmp/ci-win/bin/Lumice.exe <windows-host>:<path>/
scp examples/bench_config.json <windows-host>:<path>/

# Run (via SSH / PowerShell)
.\Lumice.exe -f bench_config.json -o .

# Measure wall time (PowerShell)
Measure-Command { .\Lumice.exe -f bench_config.json -o . 2>&1 | Out-Null } | Select-Object TotalSeconds
```

## 2. GUI Perf Test (Hidden Window, No VSync)

Automated GUI test driven by ImGui Test Engine. Uses a hidden window with `swapInterval(0)`,
so it **does not reflect real display/VSync overhead**.

By default, the perf test applies a 16ms frame rate limit (matching the real app's
`kTargetFrameTimeMs` fallback) to produce realistic baseline metrics. Use `--no-frame-limit`
to disable this for raw unlimited-FPS comparison.

Two scenarios:
- **steady_state**: 2 seconds of accumulation — measures rays/sec + texture FPS
- **slider_drag**: 5 seconds of alternating parameter changes — measures rays/sec + upload ratio +
  first_upload delay + per-upload rays CV

### Diagnostic Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--visible` | off | Show the GLFW window (for display/DWM testing) |
| `--vsync` | off | Enable VSync (implies `--visible`) |
| `--frame-limit` | on | Apply 16ms sleep-based frame rate limit |
| `--no-frame-limit` | — | Disable frame rate limit for raw throughput |
| `--main-loop-commit` | off | Call `CommitConfig` in the main loop (faithful to real app) |
| `--log-panel` | off | Show the log panel during test |
| `--dorun-delay <ms>` | 0 | Add artificial delay to `DoRun` (simulate slow environment) |
| `--skip-calibration` | off | Skip startup quality threshold calibration |
| `--perf-bench` | off | Run `--perf-bench` mode: standalone throughput measurement |

### macOS

```bash
# Build (needs -gt for GUI test target)
./scripts/build.sh -gtj release

# Run perf tests only (PERF output on stderr, server logs on stdout)
./build/Release/bin/LumiceGUITests --filter perf_test \
  > /tmp/perf_stdout.txt 2>/tmp/perf_stderr.txt
grep "\[PERF\]" /tmp/perf_stderr.txt

# With debug level (adds ConsumeData per-batch + Consume profile)
./build/Release/bin/LumiceGUITests --filter perf_test --log-level debug \
  > /tmp/perf_stdout_debug.txt 2>/tmp/perf_stderr_debug.txt
grep "Consume profile" /tmp/perf_stdout_debug.txt
```

**Note**: PERF results are on **stderr**, server logs on **stdout** — redirect separately.

### Windows

```bash
# Transfer GUI test binary from CI artifact
scp /tmp/ci-win/bin/LumiceGUITests.exe <windows-host>:<path>/

# Run (PowerShell, must use --filter to avoid crash on non-perf tests)
$proc = Start-Process -FilePath ".\LumiceGUITests.exe" `
  -ArgumentList "-nopause","--filter","perf_test" `
  -NoNewWindow -Wait `
  -RedirectStandardOutput "perf_stdout.txt" `
  -RedirectStandardError "perf_stderr.txt" `
  -PassThru
Write-Host "Exit code: $($proc.ExitCode)"

# View results
Get-Content perf_stderr.txt
```

**Known issue**: Running the full GUI test suite on Windows via SSH causes an ACCESS_VIOLATION.
Use `--filter perf_test` to run only the performance tests, which work correctly.

### Complete procedure

Run twice per platform:
1. Without `--log-level` (info default) — for accurate throughput numbers
2. With `--log-level debug` — for Consume profile breakdown

## 3. GUI Manual Test (Visible Window, VSync)

Reflects real user experience. Requires a display environment with a visible window.

### Procedure

1. Launch the GUI application (macOS: double-click .app, Windows: run .exe directly)
2. Load configuration, click Run
3. **Steady-state test**: Wait ~5 seconds for accumulation, observe rays/sec
4. **Drag test**: Drag crystal height slider continuously for ~10 seconds
5. Enable file logging (GUI Log panel → Enable File Log)
6. Analyze the log file with the analysis script

### Automated --perf-bench Mode

The `--perf-bench` flag provides a standalone throughput measurement that runs for a
fixed duration and reports average rays/sec. Useful for apples-to-apples comparisons
between builds or platforms without GUI interaction.

```bash
# macOS
./build/cmake_install/LumiceGUI --perf-bench

# Windows (via remote watcher, see doc/windows-remote-testing.md)
./scripts/win_remote_test.sh ./LumiceGUI.exe --perf-bench
```

### Remote Windows Testing

For Windows testing without physical access, use the watcher-based remote workflow.
See [Windows Remote Testing Guide](windows-remote-testing.md) for setup instructions.

```bash
# Run GUI perf test on Windows with VSync
./scripts/win_remote_test.sh /tmp/ci-win/bin/LumiceGUITests.exe \
  --filter perf_test --vsync --log-level verbose
```

### Comparison

| Condition | GUI Perf Test | Manual Test | --perf-bench |
|-----------|--------------|-------------|--------------|
| Window | hidden (default) / visible (`--visible`) | visible | visible |
| VSync | off (default) / on (`--vsync`) | on (system default) | on (system default) |
| Frame limit | on (default) / off (`--no-frame-limit`) | on | on |
| Input | automated (ImGui Test Engine) | manual (slider drag) | none (steady-state only) |
| Reproducibility | high | low (human variance) | high |
| Reflects real UX | partially (with `--visible --vsync`) | yes | partially |

## 4. Log Analysis Script

`scripts/analyze_perf_log.py` parses GUI/Poller log files, detects operational phases
(STEADY / DRAG / PAUSE), and reports per-phase performance metrics.

```bash
# Text output (compare multiple files)
python scripts/analyze_perf_log.py log1.log log2.log

# With visualization (PNG plots)
python scripts/analyze_perf_log.py -p log1.log log2.log

# Filter time range (seconds from log start)
python scripts/analyze_perf_log.py --from 2.0 --to 8.0 log.log

# Output plots to specific directory
python scripts/analyze_perf_log.py -p -o /tmp/perf_plots log.log
```

Key features:
- Phase detection: STEADY (hardware throughput), DRAG (interaction responsiveness), PAUSE (recovery)
- Per-cycle metrics: first upload delay, upload rays, commit latency
- First upload split: commit latency + gate wait decomposition
- Comparison table and CDF plots when multiple files are provided
