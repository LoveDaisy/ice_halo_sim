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

### `--benchmark` flag

The `--benchmark` flag runs a dual-mode benchmark: a single-worker pass for per-core
efficiency, followed by a multi-worker pass for parallel throughput. Two JSON lines are
output:

```
[BENCHMARK] {"mode": "single", "workers": 1, "cores": 8, "rays": 2000000, "wall_sec": 8.51, "setup_sec": 0.01, "active_sec": 8.5, "rays_per_sec": 235294.1, "rate_basis": "steady"}
[BENCHMARK] {"mode": "multi", "workers": 8, "cores": 8, "rays": 10000000, "wall_sec": 0.6, "setup_sec": 0.02, "active_sec": 0.58, "rays_per_sec": 17241379.3, "rate_basis": "steady"}
```

`rays_per_sec` is the **steady trace rate** over `active_sec` (the window from
first traced ray to IDLE), NOT `rays / wall_sec`. `setup_sec` (server alloc +
scene gen + first-dispatch latency) is excluded from the denominator;
`rate_basis` records which path produced the rate (`steady` / `active_short` /
`wall_fallback`). This setup-exclusion fix (task-fix-throughput-bench-honesty)
matters for fast backends whose whole run is ~0.2s — folding setup in deflated
their rays_per_sec by >30%.

**Terminology**: "workers" refers to simulator threads (the threads performing ray tracing).
Each server instance also has 2 internal threads (scene generation + data consumption), so
total thread count = workers + 2.

**Parallel efficiency** = `multi_rps / (single_rps × workers)`. A value near 1.0 indicates
good scaling; lower values suggest lock contention, memory bandwidth saturation, or
scheduling overhead.

Behavior differences in benchmark mode:
- **Dual-pass**: Two independent server instances are created sequentially, each with a
  specific worker count (1 for single, `hardware_concurrency()` for multi)
- **Reduced rays for single pass**: 2M rays (vs the config's original value) to limit CI
  runtime, since single-worker execution is slower
- **No image I/O**: `SaveRenderResults` is skipped, so `wall_sec` reflects pure simulation time
- **5ms poll interval** (vs default 1s): caps the IDLE-detection quantization at a few ms
  (was 100ms; that alone added up to a full poll interval to a fast run's wall time)

### Benchmark Scene Registry (canonical throughput scenes)

> **Single source of truth for throughput comparisons.** Always cite a real
> config from this table — never invent a scene name. (This table exists because
> the docs once cited a nonexistent `ms3_multi_crystal_complex_filter` and
> compared against numbers that could not be reproduced; see
> task-fix-throughput-bench-honesty.) `scripts/bench_throughput.py` runs the
> Metal-comparable subset; keep the two in sync.

Measurement basis: **engine** = `Lumice --benchmark` multi pass, setup-excluded
steady rate; **GUI** = `gui_test perf_test` steady_state, infinite budget,
reconstruct path. Baseline denominator is always **legacy CPU** (the GUI's real
path) — never `cpu_backend`. All ratios below measured on **M2 Max, 2026-06-19**
(`scratchpad/task-fix-throughput-bench-honesty/data/`); treat as the regression
anchor, re-measure same-session before/after any throughput change.

> **Why two measurement paths (engine vs GUI) coexist — do not collapse them.**
> `Lumice --benchmark` (engine ceiling) and `gui_test perf_test` (GUI-fidelity)
> answer orthogonal questions: the former is the *perf denominator / engine
> headroom* reference (explore-271 used it to establish the ~10.7M rays/s
> sustained engine rate as the GUI's headroom anchor); the latter measures the
> user-experienced reconstruct path. `--benchmark` is **load-bearing** — the G1
> gate (`test/performance/test_metal_throughput.py`) and the committed harness
> (`scripts/bench_throughput.py`) both invoke it — and is **not** redundant with
> the GUI path, so it is not a retire candidate. (Evaluated & settled 2026-06-24;
> the now-removed `LumiceGUI --perf-bench` flag *was* a genuine orphan, retired in
> #292.) Revisit only if a GUI-regime gate ever becomes the sole throughput门.

| config（真实文件） | regime | rays / MS / filter | lens / Metal 可比 | 角色 | Metal vs legacy（实测基线） |
|---|---|---|---|---|---|
| `bench_light_single_ms.json` | 轻·单MS | 10M / 1 / 无 | dual_fisheye_EA ✅ | 轻场景吞吐基准（bench 专用，勿因 e2e 改动） | 引擎 ~1.7× / GUI ~1.8× |
| `ms_multi_crystal.json` | 中·无filter | 2M / 2 / 无 | dual_fisheye_EA ✅ | 无 culling 中等基准 | 引擎 ~2.0× / GUI ~2.2× |
| `ms_multi_crystal_complex_filter.json` | 重·标准 | 2M / 2 / complex | dual_fisheye_EA ✅ | **G1 + G4 主基准** | 引擎 ~8.1× / GUI ~9.5× |
| `ms_multi_crystal_filtered_bd.json` | 重·bd | 2M / 2 / bd | dual_fisheye_EA ✅ | G1 第二基准 | 引擎 ~10.1× |
| `ms3_mixed_pyramid_heavy.json` | 最重·棱锥 | 5M / 3 / 4×raypath | dual_fisheye_EA ✅ | register-pressure 上界 | 引擎 **~5.5×**（2026-06-24 同会话 M2 Max：legacy multi 46.1K/s、single 7.3K/s；Metal multi 255K/s、single 245K/s）；GUI legacy ~48.7K → Metal ~5.7×（互证）。**注**：legacy single pass ~274s，超出 `bench_throughput.py` 的 `RUN_TIMEOUT_SEC`，故该场景仍从自动跑中排除——基线靠手动大 timeout 单测取得 |
| `halo_22.json` | 轻·单MS | 10M / 1 / 无 | **fisheye_EA（单）→ CLI Metal 回退** | **e2e 资产，勿改**；legacy-only 轻基准 | N/A（Metal 不兼容此投影；轻·Metal 用 `bench_light_single_ms`） |

Notes:
- The dispatch sweet spot is 32768 (`LUMICE_DISPATCH_RAY_NUM`); small dispatches
  starve the GPU (512/2048 = 0.2–0.8× legacy). See the Runtime Tuning Knobs table.
- `bench_throughput.py` overrides `ray_num` to a large value per run (temp config,
  committed files untouched) so the steady window is long enough to be stable.

#### Acceptance yardstick: hardware capability, NOT just "× legacy CPU"

> **The `× legacy CPU` ratio is a FLOOR, not the success bar.** Beating the
> legacy CPU is a necessary gate, never the goal — a GPU backend can "win × legacy"
> while running the device at 1–2% utilization (this exact trap cost a day in
> scrum-304: a CUDA number reported as "~1.7× legacy 16-core" was actually a
> starved GPU on a heavier-than-comparable scene). For GPU backends the real bar
> is the **device's hardware capability**, anchored to a comparable workload and
> a known external reference.

**Registered hardware-capability targets** (absolute, scene-anchored — cite these,
not "× legacy", when judging GPU throughput):

| scene | comparable workload | hardware-capability target | source |
|---|---|---|---|
| `bench_light_single_ms` (轻·单MS) | single crystal, single MS, no continuation | **≥ 25M rays/s on RTX 4060 Ti** | competitor product (measured) — the light single-MS scene is the apples-to-apples comparison |

- Always benchmark the **comparable** scene against an external target — do NOT
  cite a heavy multi-crystal / multi-MS number (e.g. `ms_multi_crystal`) when
  the reference is single-crystal single-MS. They differ by an order of magnitude
  in per-ray work.
- When a GPU backend lands far below its hardware-capability target, the throughput
  number is **not** a success regardless of the legacy ratio — investigate
  utilization (see the CUDA active% diagnostic below) and the mechanism, do not
  close out.
- If a target genuinely cannot be reached, the acceptance artifact must be a
  **profiler-grounded mechanism explanation** (where the time goes), not a black-box
  "GPU doesn't naturally win" claim.

### Latest measured results (per-run log)

> Append the newest numbers here each time the bench is re-run. Record **absolute
> rays/sec** (not just × legacy), per config, with a hardware summary. **Cross-hardware
> numbers are NOT comparable** — always read within one host block; never compare a
> Mac row against a Linux row. Source: `scripts/bench_throughput.py` (default dispatch,
> `ray_num`=20M, N≥5 reps, N=9 re-run on CoV>15%).

#### Mac — Apple M2 Max (12-core: 8P+4E, 32 GB, macOS 14.7) · legacy vs Metal · 2026-06-28

| config | legacy single | legacy multi | Metal single | Metal multi | Metal/legacy (multi) |
|---|---|---|---|---|---|
| `bench_light_single_ms` (轻·单MS) | 573 K/s | 4.62 M/s | **27.8 M/s** | **30.6 M/s** | 6.62× |
| `ms_multi_crystal` (中·无filter) | 94 K/s | 759 K/s | 7.08 M/s | 7.66 M/s | 10.08× |
| `ms_multi_crystal_complex_filter` (重·标准) | 333 K/s | 1.54 M/s | 12.4 M/s | 12.8 M/s | 8.33× |
| `ms_multi_crystal_filtered_bd` (重·bd) | 368 K/s | 1.62 M/s | 13.7 M/s | 14.0 M/s | 8.59× |

- **Metal on the comparable light single-MS scene already hits ~28–30 M/s — at/above
  the 25 M/s hardware-capability target.** This proves the bar is reachable on this
  codebase and gives CUDA a same-engine target (the RTX 4060 Ti is comparable raw
  compute to the M2 Max GPU); the remaining CUDA gap is execution-model, not a
  fundamental kernel limit.

#### Linux — dev49 RTX 4060 Ti (AMD Zen5 9950X 16C/32T host) · legacy vs CUDA · 2026-06-29

After scrum-304.2 buffer-persist, GPU-idle-gated (`nvidia-smi` 0% verified before run),
`scripts/bench_throughput.py` default dispatch (=32768), ray_num=20M, N≥5 (N=9 on CoV>15%).

| config | legacy single | legacy multi | CUDA single | CUDA multi | CUDA/legacy (multi) |
|---|---|---|---|---|---|
| `bench_light_single_ms` (轻·单MS) | 744 K/s | 8.98 M/s | **35.2 M/s** | **55.9 M/s** | 6.23× |
| `ms_multi_crystal` (中·无filter) | 126 K/s | 1.44 M/s | 9.67 M/s | 13.7 M/s | 9.55× |
| `ms_multi_crystal_complex_filter` (重·标准) | 471 K/s | 6.26 M/s | 38.0 M/s | 60.3 M/s | 9.63× |
| `ms_multi_crystal_filtered_bd` (重·bd) | 510 K/s | 6.73 M/s | 39.1 M/s | 61.9 M/s | 9.19× |

- **Competitive bar MET**: the comparable light single-MS scene is **35–56 M/s**, at/above
  the 25 M/s competitor target AND above the Mac Metal reference (28–30 M/s). buffer-persist
  alone (scrum-304.2) got CUDA here; the earlier "0.16–0.40× / 1.5M" pessimism was a
  measurement artifact — ad-hoc non-idle-gated runs on the heavier `ms_multi_crystal`, the
  wrong yardstick. Always idle-gate + use the canonical harness + the comparable scene.
- **GPU not yet saturated**: nsys on the light scene (50M-ray plain run) = GPU active 17.6%
  (JIT/setup-included); benchmark steady-window active ≈ 43%. trace_single_ms_kernel is 95%
  of GPU time. The fully-fed kernel ceiling is ~129 M rays/s (50M / 386 ms kernel), so the
  current 56 M is ~43% of ceiling — **~2.3× headroom remains**, locked behind per-dispatch
  host serialization (default stream + per-layer sync; explore-303, untouched by 304.2).
  Pursuing it is the async-engine work — now justified by the active% data, not speculation.
- Caveat: the competitor's exact config (spectrum / max_hits) is unknown; our scene is a
  reasonable single-crystal single-MS proxy (prism, D65, max_hits 7). Align if a precise
  apples-to-apples is needed.

### macOS

```bash
# Build
./scripts/build.sh -j release

# Benchmark mode (recommended — structured output, no image I/O)
./build/cmake_install/Lumice --benchmark -f examples/bench_config.json -o /tmp

# Manual mode (info level — with image output)
time ./build/cmake_install/Lumice -f examples/bench_config.json -o /tmp 2>&1 \
  | grep -E "Consume profile|Stats:"

# Manual mode (debug level — Consume breakdown)
time ./build/cmake_install/Lumice -f examples/bench_config.json -v -o /tmp 2>&1 \
  | grep -E "Consume profile|Stats:"
```

Key output:
- `--benchmark`: Two `[BENCHMARK]` JSON lines (single-worker + multi-worker) with per-core
  and parallel throughput data
- Manual mode: `Consume profile` line + `time` wall time → throughput = 10M / wall_seconds

### Windows

Build via CI, then transfer the binary:

```bash
# Download CI artifact
gh run download <RUN_ID> --name gui-test-windows-msvc --dir /tmp/ci-win

# Transfer binary + config to Windows machine
scp /tmp/ci-win/bin/Lumice.exe <windows-host>:<path>/
scp examples/bench_config.json <windows-host>:<path>/

# Benchmark mode (via SSH / PowerShell)
.\Lumice.exe --benchmark -f bench_config.json -o .

# Manual mode (PowerShell wall time)
Measure-Command { .\Lumice.exe -f bench_config.json -o . 2>&1 | Out-Null } | Select-Object TotalSeconds
```

### Linux / CUDA (dev49)

CUDA throughput is measured on the dev49 bench box (NVIDIA RTX 4060 Ti, Linux,
CUDA docker — see `reference_dev49_linux_bench` / `explore-cuda-step2-derisk/TOOLCHAIN.md`).
**Do NOT improvise per-task bench scripts.** Use the committed harness
`scripts/bench_throughput.py` — it already supports CUDA via env overrides, runs
the canonical scene set (including the comparable `bench_light_single_ms`),
confirms GPU routing (a silent fallback to legacy is flagged, not reported as a
GPU number), and reports median + CoV with thermal re-runs.

```bash
# On dev49, inside the CUDA docker (build first: -DLUMICE_CUDA_ENABLED=ON
# -DBUILD_SHARED_LIBS=ON -> build/Release/{bin/Lumice, lib/liblumice.so}).
# Canonical CUDA throughput run (legacy baseline + CUDA, full canonical scene set):
LUMICE_BENCH_BIN=/work/build/Release/bin/Lumice \
LUMICE_BENCH_LIBDIR=/work/build/Release/lib \
LUMICE_BENCH_BACKENDS=legacy,cuda \
  python3 scripts/bench_throughput.py
# Narrow to the comparable light scene only (vs the 25M/s hardware-capability target):
#   LUMICE_BENCH_CONFIGS=bench_light_single_ms
# Idle-gate: dev49 is shared (load often high); take strict throughput numbers in
# an idle window only. Clean up your processes when done (do not leave Lumice /
# detached bench running on the shared box).
```

#### GPU active% diagnostic (CUDA only — Metal/Mac has no equivalent CLI path)

A throughput number alone cannot tell you whether the GPU is fed or starved.
On CUDA/dev49, pair the bench with an `nsys` capture to read GPU utilization —
this is the diagnostic that distinguishes a real win from a "beat the CPU while
the GPU idles" false win. Profiler is already de-risked on dev49 (nsys 2023.4.4 +
ncu 2024.1.1; install + usage recipe in `explore-cuda-step2-derisk/TOOLCHAIN.md`
§"Profiler 可用性"). active% is **not a hard universal gate** — Metal on macOS has
no comparable CLI active% path, so the cross-platform acceptance criterion stays
the absolute hardware-capability target above; active% is a CUDA-side corroborating
diagnostic for *why* a number is high or low.

```bash
# GPU active% = (sum of cuda_gpu_kern_sum) / wall, from an nsys timeline of one
# representative run. Low active% (e.g. 1–2%) => GPU starved => the throughput
# number is host-bound, not a hardware-capability result.
nsys profile --trace=cuda --stats=true -o /tmp/rep <Lumice ...>   # see TOOLCHAIN.md
```

### CI Automated Benchmark

Every push triggers a CLI benchmark on all 4 CI platforms (Ubuntu x64/ARM, macOS ARM,
Windows MSVC). Results are collected into a summary table visible on the workflow run page
(`$GITHUB_STEP_SUMMARY`). See `.github/workflows/ci.yml` for details.

The summary table includes hardware context and dual-mode results:

| Column | Description |
|--------|-------------|
| CPU | CPU model (auto-detected at runtime) |
| Cores | Logical core count (`hardware_concurrency()`) |
| Workers | Simulator worker threads used for multi-worker pass |
| Single rps | Single-worker rays/sec (per-core efficiency) |
| Multi rps | Multi-worker rays/sec (parallel throughput) |
| Efficiency | `multi_rps / (single_rps × workers)` — parallel scaling efficiency |

**Note**: `Single rps` is the most meaningful metric for cross-platform comparison, as it
naturally factors in IPC, cache hierarchy, and memory bandwidth without requiring GHz
normalization. `Efficiency` reveals scaling bottlenecks specific to each platform.

### Historical Trend

Benchmark results from pushes to `main` are automatically stored to the `gh-pages` branch
using [github-action-benchmark](https://github.com/benchmark-action/github-action-benchmark).
A Chart.js dashboard is available at:

**https://lovedaisy.github.io/ice_halo_sim/bench/**

The dashboard tracks 12 time-series (4 platforms × 3 metrics):

| Metric | Unit | Description |
|--------|------|-------------|
| `<Platform> / Single` | rays/sec | Single-worker throughput (per-core efficiency) |
| `<Platform> / Multi` | rays/sec | Multi-worker throughput (parallel pipeline) |
| `<Platform> / Efficiency` | % | `multi_rps / (single_rps × workers) × 100` — self-referential parallel scaling |

**How to read the charts**:
- **Sudden drops** in Single/Multi rps may indicate code regression OR CI runner hardware change
  (check the tooltip for CPU model)
- **Efficiency** is immune to runner hardware changes — a drop in Efficiency reliably indicates
  a parallel scaling regression (e.g., increased lock contention)
- Alert threshold is set at 200% (performance must drop by >50% to trigger a commit comment)

**Known limitations**:
- History is only stored for `main` branch pushes; feature branch benchmarks appear in the
  per-run summary table but are not tracked historically
- CI runner hardware may change without notice, causing step changes in absolute rps metrics
- The alert threshold is global (same 200% for all metrics); Efficiency regressions below the
  threshold require manual inspection of the chart

### Runtime Tuning Knobs

| Environment variable | Default | Description |
|----------------------|---------|-------------|
| `LUMICE_DISPATCH_RAY_NUM` | **32768** (Metal) / 128 (CPU) | task-268.4 knob; scrum-268.6 set the Metal backend-aware default to **32768** (empirical sweet spot). Re-measured 2026-06-19 (task-fix-throughput-bench-honesty, setup-excluded `--benchmark`): heavy-scene engine **8–10× legacy** at 32768, GUI steady **~9.5× legacy** on M2 Max. Amortizes Metal kernel launch overhead; small dispatches starve the GPU (512/2048 = 0.2–0.8× legacy, 128 hangs at large ray_num) — the full win requires ≥32768. Set to a power of two for alignment. Applies at server startup; changing mid-run has no effect. Independent of `LUMICE_COMMIT_RAY_NUM` (see next row) — pick "feed GPU big" without sacrificing GUI cadence. |
| `LUMICE_COMMIT_RAY_NUM` | 128 | task-268.4: SimData-to-consumer commit granularity inside `ConsumeData`. Smaller commits = finer GUI snapshot cadence regardless of dispatch size. Backend exit-seam path only (legacy CPU SimData bypass the chunker). |
| `LUMICE_BATCH_RAY_NUM` | (deprecated) | Backward-compat fallback for `LUMICE_COMMIT_RAY_NUM` only. Pre-task-268.4 this knob doubled as both dispatch and commit granularity. **scrum-268: the DISPATCH split is the primary driver for Metal throughput; setting `LUMICE_BATCH_RAY_NUM` now only controls commit cadence, not GPU dispatch size.** Prefer the two split env vars above. |
| `LUMICE_TRACE_BACKEND` | unset (legacy CPU) | Trace backend selection: unset = legacy CPU; `metal` = Metal GPU backend (exit-seam + device root-gen); `cpu_backend` = SoA CPU backend. |
| `LUMICE_DISABLE_DEVICE_GEN` | unset (device-gen ON) | Escape hatch to force **host** root-ray generation on the Metal backend. Device root-gen (GPU PCG root-ray supply) is the **default** for the Metal backend on eligible layers (single-crystal-per-ci, `tri_count ≤ 64`); it activates on both the deterministic single-worker path and the default multi-worker random path (each worker gets a non-zero derived `effective_seed_`). Set this to `1` only for strict-identity parity tests that mirror the host `std::mt19937` stream (which cannot align with the device PCG stream). Read once per backend instance at construction. |

**GPU device root-gen (scrum-260)**: on the Metal backend, root rays (orientation / direction / entry point) are generated on-device via a counter-based PCG stream keyed by `(gen_seed, gen_ray_base + tid)`, replacing host pre-generation + upload. This is the default path (single- and multi-crystal per-ci); statistical equivalence vs legacy is validated by the slow-e2e parity harness (`ds_corr ≥ 0.99`). Throughput uplift is hardware-dependent — quantify on a frequency-locked bench machine.

**Phase-1 confirmed throughput (task-metal-rootgen-throughput-confirm, Mac M2 Max, 2026-06-11)**: device-gen ON vs OFF measured with `scratchpad/bench/device_gen_throughput_bench.py` (`LUMICE_TRACE_BACKEND=metal`; ON = `LUMICE_DISABLE_DEVICE_GEN` unset, OFF = `=1`). 5 reps per cell, median; CoV reported. Activation that ON==GPU PCG vs OFF==host mt19937 is independently asserted by the slow-e2e test `test_device_gen_activation_proof_fixed_seed` (`dual_fisheye_ref` sim_seed=42, rel_err=4.4e-5 ≫ 1e-5 floor — host-gen fallback would give rel_err=0). (Note: the bench script lives under the git-ignored `scratchpad/` tree by design — consistent with `seam_exit_bench.py` — and is not committed; re-running requires the author's local copy. The numbers below are the durable record.)

| Config | Batch | Single rps ON | Single rps OFF | ON/OFF (single) | Multi rps ON | Multi rps OFF | ON/OFF (multi) |
|--------|-------|---------------|----------------|-----------------|--------------|---------------|----------------|
| `dual_fisheye_ref` (single-MS, 1 crystal) | 128  |   343 k |   476 k | **0.72×** |  2.45 M |  4.03 M | **0.61×** |
| `dual_fisheye_ref`                        | 512  | 1.22 M | 1.18 M | 1.04× |  9.19 M |  9.34 M | 0.98× |
| `dual_fisheye_ref`                        | 2048 | 3.06 M | 1.23 M | 2.47× | 23.37 M | 13.27 M | 1.76× |
| `ms_multi_crystal` (multi-MS, 2 crystals) | 128  |   107 k |   111 k | 0.97× |   979 k |   793 k | **1.23×** |
| `ms_multi_crystal`                        | 512  |   311 k |   242 k | 1.28× |  2.63 M |  1.17 M | 2.24× |
| `ms_multi_crystal`                        | 2048 |   447 k |   318 k | 1.41× |  4.77 M |  1.61 M | **2.95×** |

All cells `routed_ok=True` (no fallback); CoV ≤ 5% for 21 of 24 measurements, max CoV 15.0% (`single_ms` b2048 multi ON — kernel saturation, not thermal). The 15.0% cell sits exactly at the CoV>15% retry threshold and was not re-run because the threshold is strict-greater (per the pre-specified CoV>15% → N=9 retry rule; ≤15% is in-spec, not a license to skip warranted reruns).

**Verdicts**:

- **`dual_fisheye_ref` (single-MS, 1 crystal) at default batch=128 is a NET LOSS for device-gen** (single 0.72×, multi 0.61×). This **contradicts the scrum-260 probe** which reported ~1.87× at the same config — the probe was a single throwaway invocation and did not reflect steady-state behavior. Net gain only kicks in at `batch ≥ ~512` and reaches 2.47× / 1.76× at `batch=2048`. At default batch GPU device-gen pays per-dispatch overhead that the small per-batch ray count cannot amortize.
- **`ms_multi_crystal` (multi-MS, 2 crystals) at default batch=128 is roughly neutral for single-worker (0.97×) and a net gain (1.23×) for multi-worker.** This is the opposite of the plan's pre-bench hypothesis (which feared multi-crystal per-ci dispatch overhead would dominate); in fact multi-MS workloads amortize device-gen better than single-MS because the multi-MS layer count multiplies the per-batch work GPU-side. At larger batches the multi-MS multi-worker gain grows to 2.24× (b512) and 2.95× (b2048).
- **Activation cross-check for multi-crystal**: `ms_multi_crystal` multi-worker ON/OFF goes 1.23× → 2.24× → 2.95× as batch grows. The monotone ON-favored ratio with batch (the b2048 crossover predicted by plan §3 Step 3 Minor 1 for "device-gen active but small-dispatch overhead-dominated") confirms device-gen IS activating on multi-crystal per-ci paths; if it had silently fallen back, ON ≡ OFF and the ratio would sit at 1.0 across all batches. The activation-proof test only covers single-crystal; the batch-scaling pattern here is the indirect activation evidence for multi-crystal.

**Implication for default config**: users running the Metal backend at the default `LUMICE_BATCH_RAY_NUM=128` see device-gen **hurt** single-crystal throughput and **help** multi-crystal multi-worker throughput modestly. The strong wins (>2×) require `LUMICE_BATCH_RAY_NUM=2048`. A future task may revisit (a) raising the default batch when device-gen is active, (b) adaptive per-ci batch coalescing for the small-dispatch regime, or (c) selectively disabling device-gen for single-crystal/single-MS scenes at small batches. Until then the escape hatch `LUMICE_DISABLE_DEVICE_GEN=1` is the workaround for users hitting the small-batch single-crystal loss.

Example: measure throughput at a higher batch size to characterize the Metal dispatch amortization curve:

```bash
# Default batch (128 rays/batch)
./build/cmake_install/Lumice --benchmark -f examples/bench_config.json -o /tmp

# Larger batch (512 rays/batch) — Metal backend crosses over here
LUMICE_BATCH_RAY_NUM=512 ./build/cmake_install/Lumice --benchmark -f examples/bench_config.json -o /tmp
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

### macOS

```bash
# Build (needs -gt for GUI test target)
./scripts/build.sh -gtj release

# Run perf tests only (PERF output on stderr, server logs on stdout)
./build/Release/bin/gui_test --filter perf_test \
  > /tmp/perf_stdout.txt 2>/tmp/perf_stderr.txt
grep "\[PERF\]" /tmp/perf_stderr.txt

# With debug level (adds ConsumeData per-batch + Consume profile)
./build/Release/bin/gui_test --filter perf_test --log-level debug \
  > /tmp/perf_stdout_debug.txt 2>/tmp/perf_stderr_debug.txt
grep "Consume profile" /tmp/perf_stdout_debug.txt
```

**Note**: PERF results are on **stderr**, server logs on **stdout** — redirect separately.

### Windows

```bash
# Transfer GUI test binary from CI artifact
scp /tmp/ci-win/bin/gui_test.exe <windows-host>:<path>/

# Run (PowerShell, must use --filter to avoid crash on non-perf tests)
$proc = Start-Process -FilePath ".\gui_test.exe" `
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

### Remote Windows Testing

For Windows testing without physical access, use the watcher-based remote workflow.
See [Windows Remote Testing Guide](windows-remote-testing.md) for setup instructions.

```bash
# Run GUI perf test on Windows with VSync
./scripts/win_remote_test.sh /tmp/ci-win/bin/gui_test.exe \
  --filter perf_test --vsync --log-level verbose
```

### Comparison

| Condition | GUI Perf Test | Manual Test |
|-----------|--------------|-------------|
| Window | hidden (default) / visible (`--visible`) | visible |
| VSync | off (default) / on (`--vsync`) | on (system default) |
| Frame limit | on (default) / off (`--no-frame-limit`) | on |
| Input | automated (ImGui Test Engine) | manual (slider drag) |
| Reproducibility | high | low (human variance) |
| Reflects real UX | partially (with `--visible --vsync`) | yes |

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
