# Performance Testing Guide

This guide covers three levels of performance testing for Lumice, from pure pipeline
benchmarking to real-world GUI interaction testing.

All commands assume the working directory is the project root.

> **Scope**: this guide holds the stable how-to + the **current canonical** throughput
> reference. Historical per-run measurement dumps (dated tables, raw reps, per-effort
> methodology notes) live in the git-ignored `scratchpad/perf-results-log.md` — append new
> per-run numbers there, and promote a number here only when it becomes the new canonical
> anchor. CUDA build + parity/correctness on the remote boxes is a separate concern — see
> [`gpu-remote-cuda-build-testing.md`](gpu-remote-cuda-build-testing.md).

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

## ⚠️ Two ways `--benchmark` silently reports a number that answers a different question

Both were hit in one session while calibrating the geometry clock; each produced a confident,
self-consistent, wrong conclusion that survived until something else contradicted it.

### A. `--benchmark` emits TWO `[BENCHMARK]` lines on the legacy CPU route — pick the right one

The legacy CPU route runs a dual pass and prints one JSON per pass:
`"mode":"single","workers":1` and `"mode":"multi","workers":N`. A GPU route is a single engine and
prints one line only (`workers:1`).

**The legacy CPU product path is `worker_count = PhysicalCoreCount()`** (`ServerImpl::ServerImpl`,
`server.cpp:444`; only a fixed seed or a GPU route forces 1). So `mode:single` is a
per-core/parallel-efficiency diagnostic — **it is not the shipping configuration**, and a
`grep '"single"'` that looks right will quietly measure a config nobody runs.

This is not a small correction — **effects can invert between the two passes**. Measured, tracing
identical work (`face_distance` fixed, so zero construction), 20M rays:

| geometry clock | `mode:single` | `mode:multi` (12 workers, = product) |
|---|---|---|
| 1 | 691 395 rays/s | 3 225 102 rays/s |
| 16 | 493 789 rays/s | 4 545 052 rays/s |
| | small clock **40% faster** | small clock **29% slower** |

Parallel efficiency is itself clock-dependent (4.7x vs 9.2x speedup on 12 cores), so the two passes
do not differ by a constant factor and **neither one can be rescaled into the other**. Cache effects
in particular are invisible in `mode:single` — one thread owns the whole shared L2/L3, so a
working-set effect that costs 11% in production measures as zero alone.

Rule: **quote `mode:multi` for legacy CPU throughput.** Use `mode:single` only when per-core cost
or parallel efficiency IS the question, and say so.

### B. A GPU throughput run shorter than ~7s measures the clock-boost transient, not steady state

`rays_per_sec` on the `steady` basis is `(rays_end - rays_at_active_start) / active_sec`, and the
window OPENS at the first poll that observes `rays > 0`. If throughput is not constant across that
window, the reported rate depends on how much of the startup transient the window happened to
swallow.

Measured on Metal, same binary, same config, interleaved runs:

| ray_num | wall | spread over 10 runs |
|---|---|---|
| 20M | ~0.6s | **2.4x** (24.97M – 51.59M rays/s) |
| 200M | ~7s | **1.22x** (26.05M – 31.88M rays/s), CV ~7% |

The fast outliers (42M, 51M) vanish entirely at 200M: the 2.4x is **a short-run artifact, not GPU
noise**. A 20M run only occupies ~0.6s, so the GPU clock-boost ramp dominates the window.

Rule: **GPU throughput needs >= 200M rays (~7s)**; at that length CV ~7%, which repeats can average
down. Below it the number is not a throughput measurement. Corollary: **"that backend is just noisy"
is not a property of a backend** — an unexplained variance with no mechanism is an unpaid debt, and
it silently downgrades every later measurement into "it can't be measured anyway".

### C. GPU cold-context init is folded into `rays_per_sec` when `active_sec` rounds to 0

The first GPU call in a process triggers backend-specific one-time lazy init: on CUDA it is the
context creation (~1.3s cold, ~62–67ms warm); on Metal it is PSO compile. That cost is paid
exactly once per process. When the timed pass is short enough (small finite `ray_num`, or a
too-few-drains exit on the infinite path) for `active_sec` to round to zero, the `rate_basis`
ladder falls through to `wall_fallback` and `rays_per_sec = rays / wall_sec` — silently
including that one-time init in the denominator. So a "cold" run (paying init) compared against
a "warm" run (init already paid by an earlier process) can measure a many-x cold/warm gap that
is a **process-order artifact, not a property of the trace**. This mechanism was the source of
the "stoch vs det 15.7×" cold/warm measurement artifact.

Four defensive rules — the first is now enforced by the tool, the other three remain usage
discipline:

1. **`--benchmark` runs one silent warm-up pass on the GPU route before the timed steady pass**
   (`src/main.cpp` `main()`'s `benchmark_mode` branch, `gpu_route == true`; the warm-up uses
   `kBenchmarkGpuWarmupRays` = 100k rays and the same config as the steady pass, only with
   `scene.ray_num` overridden). It absorbs the one-time init so the reported `[BENCHMARK]` line
   measures steady trace only. The legacy CPU route is unchanged (no lazy GPU init to strip).
2. **When comparing two arms (e.g. stoch vs det, K=1 vs K=64), interleave A/B/A/B independent
   process invocations** — do not run all-A back-to-back then all-B. Even with the warm-up
   pass in (1), OS-level, driver-level, or thermal state can drift across a long same-arm
   burst; interleaving averages that drift equally across both arms.
3. **`ray_num` must be large enough that trace itself dominates any residual fixed overhead.**
   The GPU throughput cross-check in §B already puts this at ≥ 200M rays / ~7s for a stable
   number; legacy CPU numbers on short configs likewise inherit this lower bound.
4. **Only trust `rate_basis` in `{steady, active, drain_aligned}`.** A `wall_fallback` or
   `too_few_drains` result is by definition not a steady trace rate. `--benchmark` now emits
   an explicit stderr warning when the reported pass falls through to `wall_fallback`
   (`src/main.cpp` `RunBenchmarkPass`, next to the `[BENCHMARK]` JSON emit) so this cannot be
   missed silently, but the discipline is the same: discard such numbers, do not use them for
   perf comparison.

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
`rate_basis` records which path produced the rate. This setup-exclusion fix
(task-fix-throughput-bench-honesty) matters for fast backends whose whole run is
~0.2s — folding setup in deflated their rays_per_sec by >30%.

**Two independent `rate_basis` ladders** (a reader/gate parses by which path, NOT
by string-equality across the whole set):
- **finite `ray_num`** (legacy CPU pass, and any finite-config `--benchmark`):
  `steady` / `active_short` / `wall_fallback` (the honesty-fix ladder above).
- **`ray_num="infinite"`** (GPU passes, task-gpu-bench-drain-aligned-rate):
  `drain_aligned` (measured exactly N drain windows) or `too_few_drains`
  (exited before N drains — unexpected/untrustworthy). See the drain-count-driven
  canonical below. The two ladders never share the same string space.

**Terminology**: "workers" refers to simulator threads (the threads performing ray tracing).
Each server instance also has 2 internal threads (scene generation + data consumption), so
total thread count = workers + 2.

**Parallel efficiency** = `multi_rps / (single_rps × workers)`. A value near 1.0 indicates
good scaling; lower values suggest lock contention, memory bandwidth saturation, or
scheduling overhead. **Meaningful for the legacy CPU route only** — see the GPU caveat next.

> **⚠️ GPU backends are single-engine — there is no "single" vs "multi" parallelism.** The GPU
> route (Metal / CUDA) runs `worker_count=1` unconditionally (`server.cpp:284`); only the legacy
> CPU route is genuinely multi-worker (`worker_count = PhysicalCoreCount()`). Because a GPU
> "single" and "multi" pass would both run on the same one engine (differing only by warmup +
> ray-count, not parallelism), **`--benchmark` collapses the GPU route to ONE steady pass**
> (labelled `mode="multi"`) and skips the warmup pass; the legacy CPU route keeps the genuine
> dual-pass. Route detection is env-aware (`LUMICE_WillUseGpuRoute` honors `LUMICE_TRACE_BACKEND`,
> so an env-selected GPU run collapses too). Consequences when reading GPU results:
> - A GPU `[BENCHMARK]`/`bench_throughput.py` row has **`multi` only** (`single`/`single_rps` are
>   absent — this is expected, not INCOMPLETE). The `multi` number is the representative steady rate.
> - **`efficiency` does not apply to GPU** (there is no single/parallel pair; `explore-306.1` E1
>   `worker_count=1` is the proof). The CI summary's efficiency column is legacy-CPU-only.
> - Cross-checking old data: pre-2026-07 GPU tables that show a `single` column were the old
>   dual-pass (single = 2M-ray warmup run); those `single` GPU numbers were never a per-core figure.

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
path) — never `cpu_backend`. The `Metal vs legacy` ratios below are the M2 Max,
2026-06-19 regression anchor (`scratchpad/task-fix-throughput-bench-honesty/data/`);
re-measure same-session before/after any throughput change.

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
> #292.)

| config (real file) | regime | rays / MS / filter | lens / Metal-comparable | role | Metal vs legacy (anchor) |
|---|---|---|---|---|---|
| `bench_light_single_ms.json` | light·single-MS · 512×256 | 10M / 1 / none | dual_fisheye_EA ✅ | light-scene throughput baseline (bench-only, do NOT change for e2e); **512×256 fits GPU L2, systematically overstates the GPU advantage**; use `--res-sweep` for real resolution (see "resolution is a first-class throughput dimension") | engine ~1.7× / GUI ~1.8× |
| `ms_multi_crystal.json` | mid·no-filter | 2M / 2 / none | dual_fisheye_EA ✅ | no-culling mid baseline; second `--res-sweep` scene (scene-dependence check) | engine ~2.0× / GUI ~2.2× |
| `ms_multi_crystal_complex_filter.json` | heavy·standard | 2M / 2 / complex | dual_fisheye_EA ✅ | **G1 + G4 primary baseline** | engine ~8.1× / GUI ~9.5× |
| `ms_multi_crystal_filtered_bd.json` | heavy·bd | 2M / 2 / bd | dual_fisheye_EA ✅ | G1 second baseline | engine ~10.1× |
| `ms3_mixed_pyramid_heavy.json` | heaviest·pyramid | 5M / 3 / 4×raypath | dual_fisheye_EA ✅ | register-pressure upper bound | engine **~5.5×** (M2 Max, 2026-06-24; GUI corroborated ~5.7×). Note: legacy single pass ~274s exceeds `bench_throughput.py`'s `RUN_TIMEOUT_SEC`, so this scene stays excluded from the auto run — its baseline is taken via a manual large-timeout unit test |
| `halo_22.json` | light·single-MS | 10M / 1 / none | **fisheye_EA (single) → CLI Metal falls back** | **e2e asset, do NOT change**; legacy-only light baseline | N/A (Metal incompatible with this projection; use `bench_light_single_ms` for light·Metal) |

Notes:
- **The 4 primary light/mid/heavy baselines (bench_light / ms_multi / complex_filter /
  filtered_bd) are all 512×256** (`ms3_mixed_pyramid_heavy` is the exception, at 2048×1024).
  A 512×256 XYZ accumulation buffer (W×H×3 float = 1.5 MB) fits in a typical GPU's L2 and
  **systematically overstates GPU throughput** — the real GUI default renders 2048×1024 (16×
  pixels, 24 MB buffer, exceeds L2). **Cross-resolution numbers are not comparable; always
  report throughput with its resolution.** For real-resolution throughput use
  `bench_throughput.py --res-sweep` (resolution axis, default dispatch, single variable
  isolated) — see "resolution is a first-class throughput dimension" below.
- The dispatch sweet spot is **backend- and resolution-dependent**: Metal 32768 / CUDA 262144
  are the **512×256** sweet spots; **as resolution rises the CUDA optimum shifts up markedly**
  (~786K–2M at 2048×1024, because it amortizes the per-batch readback — see the resolution
  subsection + Runtime Tuning Knobs). Small dispatches starve the GPU (512/2048 = 0.2–0.8×
  legacy).
- `bench_throughput.py` overrides `ray_num` to a large value per run (temp config,
  committed files untouched) so the steady window is long enough to be stable.
- **⚠️ The third-clock drain path once forced a `multi_wall` workaround (scrum-312) — now
  SUPERSEDED for GPU by drain-count-driven `drain_aligned`** (task-gpu-bench-drain-aligned-rate,
  2026-07-02). Root cause: `multi_med` (steady `rays_per_sec`) divides by a window bounded by
  `sim_ray_num`, which only advances on a third-clock drain (every 64×dispatch rays; CUDA
  default 262144 → 16.8M rays/drain). At the old bench `ray_num=20M` that is ~1.19 drains →
  `sim_ray_num` stays ~0 → most tracing mis-counted as setup → **5× under-report** (explore-315:
  wall read 24.7M, true plateau 130M). The old fix was `multi_wall = rays/wall_sec`; the new fix
  sets GPU `ray_num="infinite"` and measures an integer number of *drain windows* directly
  (`drain_aligned`), which is both setup-free and drain-granularity-exact. **GPU rows now report
  the honest steady rate in `rays_per_sec` (`multi_med`) again**; `multi_wall` is only meaningful
  for finite (legacy) passes and is ignored on `drain_aligned` GPU rows. Per-batch legacy is
  unaffected (finite `steady` ladder).

#### Resolution is a first-class throughput dimension (device-fused XYZ accumulation → cost scales with buffer vs GPU L2)

> **Every GPU throughput number MUST carry its render resolution; cross-resolution numbers are
> not comparable.** This is not a second-order detail — it often dominates GPU throughput on
> light scenes (light scenes trace cheaply, so accumulation/readback dominate).

Mechanism: device-fused XYZ accumulation (scrum-302) has every exit ray `atomicAdd` into a
W×H×3 float image buffer (12 B/px) **inside the trace kernel**. Cost scales with the buffer
relative to GPU L2:

- buffer ≤ L2 (512×256 = 1.5 MB, fits most GPUs' ~2 MB L2) → accumulation is cached, fast.
- buffer ≫ L2 (2048×1024 = 24 MB) → each atomicAdd hits DRAM, DRAM-bandwidth-bound, slow. The
  knee is at the L2 boundary (~512→768), decaying smoothly with resolution past it (not a
  binary cliff).

The CUDA route also carries a **per-batch synchronous readback** tax (`ReadbackXyzAccum`: one
`cudaDeviceSynchronize` + blocking 24 MB PCIe D2H + memset per SimBatch), which was decoupled to
the display cadence (the "third clock", `seam-design.md` §4.8) in scrum-312 — see the canonical
results below. Metal has no such tax (`StorageModeShared` unified memory + deferred wait).

**Process requirements**:

1. Measure GPU throughput with `bench_throughput.py --res-sweep` across resolutions (default six
   steps `256×128 … 2048×1024`, 2:1, straddling the L2 knee), not a single point. The script
   overrides `render[].resolution` in a temp config (committed files untouched, same mechanism as
   the `ray_num` override). Default sweeps `bench_light_single_ms` (light, accumulation/readback
   dominated) + `ms_multi_crystal` (heavier, scene-dependence check); `--res-list` / `--res-configs`
   override. Cover at least **512×256 (engine ceiling / L2-resident) + 2048×1024 (real GUI
   experience)**.
2. When comparing against a competitor / hardware capability (the 25M bar below), **align
   resolution** — our historical 512×256 numbers are an L2-resident upper bound, not the user
   experience.
3. Numbers entering a table **must state their resolution** (existing tables default to 512×256,
   except `ms3_mixed_pyramid_heavy`). Sweep curve data does NOT enter committed files (varies by
   machine/session); what enters a table is a canonical point produced under the N≥5 CoV protocol.

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
| `bench_light_single_ms` (light·single-MS) | single crystal, single MS, no continuation | **≥ 25M rays/s on RTX 4060 Ti** | competitor product (measured) — the light single-MS scene is the apples-to-apples comparison |

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
- **⚠️ Align resolution**: the 25M bar's competitor render resolution is unknown; our
  `bench_light_single_ms` is **512×256 (L2-resident, upper bound)**. On the same card / scene the
  real GUI default 2048×1024 drops throughput 3.6–5× (see "resolution is a first-class throughput
  dimension"). Align resolution before judging the bar — don't claim the bar is met from a 512×256
  L2-resident number. Use `bench_throughput.py --res-sweep` for the real-resolution comparison point.

### Canonical throughput results (current)

> The current authoritative reference numbers. **Cross-hardware numbers are NOT comparable** —
> read within one host block. Historical per-run dumps (dated tables, raw reps, per-effort
> methodology) live in `scratchpad/perf-results-log.md`. GPU steady rate = `drain_aligned`
> `rays_per_sec` (see below); the older scrum-312 res-sweep block reads `multi_wall`.

#### drain-count-driven canonical · default dispatch · config-default resolution · `drain_aligned` `rays_per_sec` · 2026-07-02

**Context** (task-gpu-bench-drain-aligned-rate + **task-317 render-per-poll fix**): GPU
`--benchmark` sets `ray_num="infinite"` and measures exactly N=10 integer drain windows
(`rate_basis="drain_aligned"`), fixing the drain-quantization under-report (explore-315: at the
old finite 20M, CUDA read ~1.19 drains → 5× deflated). Numbers below are the honest steady
`rays_per_sec` at each config's **default resolution** (NOT the scrum-312 resolution sweep — the
two blocks are different metrics/axes and must not be compared row-to-row). N reps per cell with
the ≥15% CoV → N=9 escalation.

> **task-317 re-canonicalization**: the dev49 CUDA + legacy columns below were **regenerated with
> the render-per-poll fix** (commit `ee98065a`). Before it, the `--benchmark` poll loop triggered a
> full sRGB render every iteration, which starved drain-window closure and (on CUDA) let the
> unbounded session trip the 32-bit device PCG ray-index cap → legacy fallback → the pass never
> terminated. Post-fix, CUDA infinite closes in ~1.6 s (8 reps, CoV 0.1–1.8 %). These clean dev49 values
> supersede the pre-fix "首次 run" numbers (which were noisier / partly contended). The
> **win-builder column was re-run with the fixed binary** and came back essentially identical to
> the pre-fix orphan-free run (≤1.5 % delta, within CoV) — confirming the fix does not alter the
> measurement on a machine that did not catastrophically hang (1070Ti's render tax was
> non-catastrophic), and that the ~10 % dev49 delta was old-run noise, not a systematic effect.

| config | dev49 4060Ti CUDA (vs legacy) | win-builder 1070Ti CUDA | Mac Metal ⚠️(approx) | dev49 legacy CPU (5M) |
|---|---|---|---|---|
| `bench_light_single_ms` | **130.5 M/s** (12.5×, CoV 0.2%) | 80.7 M/s (0.4%) | ~69 M/s (CoV 11%) | 10.45 M/s |
| `ms_multi_crystal` | 22.2 M/s (12.7×, 0.1%) | 13.2 M/s (0.4%) | ~16.7 M/s (8.3%) | 1.74 M/s |
| `ms_multi_crystal_complex_filter` | 371.6 M/s (56.9×, 0.8%) | 112.4 M/s (0.6%) | ~24.6 M/s (18% thermal) | 6.53 M/s |
| `ms_multi_crystal_filtered_bd` | 591.2 M/s (89.6×, 1.8%) | 158.8 M/s (0.8%) | ~26.7 M/s (8.4%) | 6.60 M/s |

**Key points**:
- **The 5× under-report is fixed.** `bench_light_single_ms` on 4060Ti reads **130.5 M/s** at
  0.2% CoV — matching explore-315's independently-measured plateau (400M-ray wall = 130.2 M/s).
  This *is* the honest steady CUDA rate; the old finite-20M bench read 24.7 M/s (deflated 5×).
- **The infinite `--benchmark` no longer hangs** (task-317): reading `sim_ray_num` via the cheap
  O(1) `LUMICE_GetSimRayCount` instead of the render-triggering `LUMICE_GetStatsResults` removed
  the per-poll sRGB render that starved window closure. dev49 CUDA infinite: 3/3 reps RC=0 ~1.6 s.
- **Locked-clock desktops (CUDA) are the authoritative canonical**: CoV 0.1–2.5% on both the
  4060Ti and 1070Ti. drain-count-driven works identically on Ada and Pascal (sm_61).
- **⚠️ Mac Metal is phase-1 approximate**, NOT canonical. On a Mac *laptop* Metal throughput is
  thermal/GPU-boost dominated and swings ~2× run-to-run (CoV 8–28%); NO N stabilizes it (an
  N-sweep confirmed CoV does not drop monotonically with N — beyond ~N=10 thermal drift over the
  longer measurement *regrows* variance). The Metal figures above are one run's medians; treat
  them as order-of-magnitude only.
- **Re: the old "4060Ti@512×256 = 110 M/s" (scrum-312 res-sweep `multi_wall`)** — not a
  contradiction: that was `multi_wall` at a *fixed 512×256* resolution, a different metric and
  resolution than the `drain_aligned` default-resolution rate here. The drain-count-driven
  130.4 M/s is the honest steady rate and supersedes the finite-bench numbers as the GPU rate
  basis; the res-sweep block below is retained for its resolution-dependence analysis.
- Correctness (unchanged by this measurement-only change): CUDA parity 10/10 @4060Ti + 10/10
  @1070Ti/sm_61; Metal parity intact. Mac G1 gate `test_metal_throughput.py` still green.

#### scrum-312 third-clock canonical · `--res-sweep` · `multi_wall` · 2026-07-01

**Context**: readback decoupled from the trace clock to the display-cadence third clock
(seam-design §4.8). At the real GUI resolution 2048×1024 the readback / per-batch-copy tax is
amortized. `bench_light_single_ms` (light·single-MS, L2/readback dominated), per-resolution
`multi_wall`:

| host / backend | 256×128 | 512×256 | 1024×512 | 1536×768 | **2048×1024** |
|---|---|---|---|---|---|
| dev49 RTX 4060Ti (Ada) CUDA | 116 M/s | 110 M/s | 92 M/s | 65 M/s | **39.2 M/s** |
| win-builder GTX 1070Ti (Pascal) CUDA | 77 M/s | 69 M/s | 45 M/s | 40 M/s | **33.5 M/s** |
| Mac M-series Metal | 28.1 M/s | 30.3 M/s | 31.2 M/s | 35.1 M/s | **32.3 M/s** |
| dev49 legacy CPU (baseline) | 9.0 M/s | 8.8 M/s | 8.4 M/s | 7.7 M/s | 6.9 M/s |
| Mac legacy CPU (baseline) | 5.1 M/s | 4.8 M/s | 4.7 M/s | 4.8 M/s | 4.7 M/s |

**Third-clock gain at 2048×1024** (vs the old per-batch drain, interleaved same-binary to isolate
the drain cadence):

| host / backend | old (per-batch) | third clock | gain | mechanism |
|---|---|---|---|---|
| 4060Ti (Ada, 32MB L2) CUDA | 28 M/s | 39 M/s | **1.4×** | large Ada L2 → old value already L2-resident; the tax is mainly per-batch D2H readback |
| 1070Ti (Pascal, 2MB L2) CUDA | 12.5 M/s | 33.5 M/s | **2.7×** | both L2 overflow + readback tax; third clock removes the readback (L2 residual) |
| M-series Metal (unified memory) | 11 M/s | 32.3 M/s | **~3×** | no PCIe readback; per-batch 24MB memset+memcpy (×76 batch ≈ 3.6GB) amortized |

**Key points**:
- **All three hardware classes (CUDA-Ada / CUDA-Pascal / Metal) confirm the third clock is a
  significant speedup at the real GUI resolution**; the gain scales with how much per-batch
  readback/copy the old value contained.
- **The Metal curve is nearly flat** (28→35 M/s across 6 steps) — the third clock makes Metal
  resolution-robust (the old path's full-frame per-batch memset+memcpy is a real cost at high
  resolution, not just CUDA readback). **Refutes the earlier "Metal unified memory = zero benefit"
  judgment.**
- Correctness: CUDA parity 10/10 @4060Ti + 10/10 @1070Ti/sm_61; Metal parity 14/14 (incl.
  batch-invariance = drain-cadence independence).
- win-builder 1070Ti omits vs-legacy (weak CPU inflates the ratio, read absolute values); the
  `multi_med` column is distorted under the third clock and is deprecated (use `multi_wall`).
- Heavy scene `ms_multi_crystal` (trace-dominated, small readback share) has a modest gain: 4060Ti
  2048 = 14.9 M/s, 1070Ti = 7.3 M/s, Metal ≈ 17 M/s@256 (Mac thermal noise large, still jittery at
  N=9 — needs a stable box to re-measure the canonical).

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
CUDA docker). For the full two-machine build + parity/correctness recipe (source sync,
docker/BuildTools toolchain, `LUMICE_HAS_CUDA` un-skip gate, parity battery), see
[`gpu-remote-cuda-build-testing.md`](gpu-remote-cuda-build-testing.md).
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
ncu 2024.1.1; install + usage recipe in `explore-cuda-step2-derisk/TOOLCHAIN.md`).
active% is **not a hard universal gate** — Metal on macOS has no comparable CLI
active% path, so the cross-platform acceptance criterion stays the absolute
hardware-capability target above; active% is a CUDA-side corroborating diagnostic
for *why* a number is high or low.

```bash
# GPU active% = (sum of cuda_gpu_kern_sum) / wall, from an nsys timeline of one
# representative run. Low active% (e.g. 1–2%) => GPU starved => the throughput
# number is host-bound, not a hardware-capability result.
nsys profile --trace=cuda --stats=true -o /tmp/rep <Lumice ...>   # see TOOLCHAIN.md
```

### Filter OR-summand scaling — no throughput cliff (scrum-filter-form-big-or de-risk, 2026-07-16)

> **De-risk sanity result, NOT a canonical throughput point** (uses synthetic N-summand
> filter configs, not the canonical scene set). Records why raising the device filter
> OR-clause cap from 8 to 4096 is throughput-safe.

Concern: a Complex filter with N OR-summands walks up to N `complex_sub_desc_buf` global
reads per exit (device match). Would large N (256) cause a GPU memory-pressure cliff?

Measured `--benchmark` (`mode:multi` rays_per_sec) across N=8/64/256 summands, filter-only
(no color), per-backend baselines (Metal+CPU on Mac, CUDA+CPU on dev49 4060Ti):

| backend | N=8 | N=64 | N=256 | 256/8 |
|---|---|---|---|---|
| CUDA (dev49 4060Ti) | 419.7M | 91.2M | **754.3M** | 1.80× |
| Metal (Mac) | ~48.7M | 14.8M | **68.5M** | 1.41× |
| CPU (Mac / dev49) | 5.0M / 10.0M | 3.2M / 6.6M | 5.6M / 10.2M | 1.1× / 1.0× |

**No cliff.** On both GPUs N=256 is the throughput *peak* (256 never slower than 8). The
curve is a non-monotonic N=64-trough U-shape on **all four backends including CPU** — since
CPU has no GPU global-memory notion, the variation is raypath-geometry/survival-driven, not
summand-count or GPU-memory. Cross-backend consistency decisively rules out a memory cliff.
Confound (honest): summand count can't be isolated from raypath geometry (more summands ⇒ more
distinct raypaths); the CPU cross-check is what disambiguates. Owner spot-verified CUDA N=8=425M
/ N=256=772M independently. → OR-clause cap 8→4096 is throughput-safe.

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
| `LUMICE_DISPATCH_RAY_NUM` | **262144** (CUDA) / **32768** (Metal) / 128 (CPU) | task-268.4 knob; the GPU per-dispatch grid size. scrum-268.6 set the Metal default to 32768; scrum-306.2 set the CUDA default to 262144 (both empirical sweet spots). Amortizes GPU kernel launch + per-batch host overhead; small dispatches starve the GPU (512/2048 = 0.2–0.8× legacy, 128 hangs at large ray_num). Set to a power of two for alignment. Applies at server startup; changing mid-run has no effect. Independent of `LUMICE_COMMIT_RAY_NUM` — feed the GPU big without sacrificing GUI cadence. **Resolution-dependent**: the 262144/32768 sweet spots were measured at **512×256**; the optimum shifts up as resolution rises (2048×1024: CUDA ~786K–2M, measured 262144→~1M = 2.25×), because the per-batch readback buffer is 16× larger and needs a bigger dispatch to amortize. Re-calibrate when resolution changes (see "resolution is a first-class throughput dimension"). |
| `LUMICE_COMMIT_RAY_NUM` | 128 | task-268.4: SimData-to-consumer commit granularity inside `ConsumeData`. Smaller commits = finer GUI snapshot cadence regardless of dispatch size. Backend exit-seam path only (legacy CPU SimData bypass the chunker). **⚠️ No-op on the GPU device-fused route (Metal/CUDA, `SupportsDeviceXyzAccum()`==true)**: that path produces `xyz_pixel_data_`, not `outgoing_d_`, so the commit chunker (`server.cpp:809`) is skipped; device readback frequency is set by `LUMICE_DISPATCH_RAY_NUM` (one `ReadbackXyzAccum` per SimBatch), **not this knob**. (Was once mis-used as a "readback-independent" probe — ineffective.) |
| `LUMICE_BATCH_RAY_NUM` | (deprecated) | Backward-compat fallback for `LUMICE_COMMIT_RAY_NUM` only. Pre-task-268.4 this knob doubled as both dispatch and commit granularity. scrum-268: the DISPATCH split is the primary driver for GPU throughput; setting `LUMICE_BATCH_RAY_NUM` now only controls commit cadence, not GPU dispatch size. Prefer the two split env vars above. |
| `LUMICE_TRACE_BACKEND` | unset (legacy CPU) | Trace backend selection: unset = legacy CPU; `metal` = Metal GPU backend; `cuda` = CUDA GPU backend; `cpu_backend` = SoA CPU backend. |
| `LUMICE_DISABLE_DEVICE_GEN` | unset (device-gen ON) | Escape hatch to force **host** root-ray generation on the GPU backends. Device root-gen (GPU PCG root-ray supply) is the **default** on eligible layers (single-crystal-per-ci, `tri_count ≤ 64`). Set this to `1` only for strict-identity parity tests that mirror the host `std::mt19937` stream (which cannot align with the device PCG stream). Read once per backend instance at construction. |

> #### ⚠️ `LUMICE_DISPATCH_RAY_NUM` is a GPU-only knob — never apply it to legacy in a comparison (scrum-306.1/306.4)
>
> `LUMICE_DISPATCH_RAY_NUM` sizes the GPU engine's per-dispatch grid. **CUDA total
> energy is dispatch-invariant** (verified: ΣY = 261.29 M ±0.001% across dispatch
> ∈ {128, 8192, 32768, 131072} on `dual_fisheye_ref`) — i.e. the GPU result is
> correct at any dispatch. **Legacy (CPU) total energy is NOT dispatch-invariant**:
> the same knob (which overrides legacy's `kDefaultRayNum`=128) swings legacy ΣY
> **−5%..+13%** — this is Monte-Carlo variance from per-batch wavelength sampling,
> not a bug (converged at the default 128; see scrum-306.7 in the per-run log).
>
> **Consequence / historical misdiagnosis to NOT repeat**: setting
> `LUMICE_DISPATCH_RAY_NUM=131072` globally to probe CUDA made
> `test_cuda_single_ms_no_filter_parity` report `energy_ratio=0.8922` — this was
> **mis-attributed to a "CUDA exit-cap/cont-cap silent energy loss"** (the original
> scrum-304.3 backlog entry). It is NOT a CUDA bug: the knob leaked into the legacy
> reference run and inflated the **denominator** (legacy_Y), `261.29/292.87 = 0.892`.
> The parity harness now strips `LUMICE_DISPATCH_RAY_NUM` for the `legacy` backend
> (`test/e2e/capi_runner.py`, scrum-306.4) so the oracle stays at its canonical
> default and `energy_ratio` reflects the GPU backend's correctness alone
> (re-verified: @131072 0.8922 FAIL → 1.0063 PASS). `bench_throughput.py` already
> excludes legacy from the dispatch sweep (`DISPATCH_PLAN["legacy"]=[None]`).
>
> **Process lesson**: the 304.3 correctness claim lived only as a backlog one-liner
> with no preserved script → a wrong diagnosis (CUDA) propagated and could not be
> re-aligned. Correctness assertions must land in a tracked doc with a reproducible
> recipe. Reproduce: container `pip install pytest numpy`, `LUMICE_HAS_CUDA=1`, then
> `LUMICE_DISPATCH_RAY_NUM=131072 pytest -m slow test/parity-cross-backend/backend/test_cuda_exit_seam_parity.py::test_cuda_single_ms_no_filter_parity`;
> per-dispatch ΣY split via `bench_work/harness2.cpp` (dev49).

**GPU device root-gen (scrum-260)**: on the GPU backends, root rays (orientation / direction /
entry point) are generated on-device via a counter-based PCG stream keyed by
`(gen_seed, gen_ray_base + tid)`, replacing host pre-generation + upload. This is the default
path; statistical equivalence vs legacy is validated by the slow-e2e parity harness
(`ds_corr ≥ 0.99`). The device-gen ON/OFF throughput characterization (Metal, phase-1) — device-gen
is a net loss for single-crystal single-MS at the default batch and a net gain for multi-crystal
multi-worker, with strong wins only at large batch — is recorded in `scratchpad/perf-results-log.md`.

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
