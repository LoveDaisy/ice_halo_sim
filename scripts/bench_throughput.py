#!/usr/bin/env python3
"""Committed throughput bench harness (task-270.6 / performance layer §1.5).

Runs `Lumice --benchmark` across a 4-backend × heavy-config × GPU-dispatch
matrix, reports median rays/s + CoV per cell, ratios against the legacy CPU
baseline. Backends: legacy CPU (baseline), cpu_backend (verify-only), Metal
(Apple host only), CUDA (CUDA host only, e.g. dev49 — scrum-296 Step D). The
host-incompatible GPU rows print as N/A. Replaces the gitignored `scratchpad/bench/seam_exit_bench.py` (whose
LUMICE_BATCH_RAY_NUM knob was removed in scrum-268.4).

Baseline policy: the denominator is always legacy CPU (env unset = the GUI's
real path). `CpuTraceBackend` is a GPU-validation reference, NOT a perf
baseline — see feedback_perf_baseline_is_legacy_cpu / doc/testing-architecture.md §4.1.

Metal pass note: the server is single-engine, so the benchmark's "single" pass
and "multi" pass both run on 1 Metal engine. The legacy CPU "multi" pass runs
on N=PhysicalCoreCount workers. The G1 gate compares Metal(1-engine, large
dispatch) vs legacy(N-workers, small dispatch); that asymmetry is intentional.

Lineage: framework (run / measure_cell / _cov / _fmt_*) was lifted from
`scratchpad/bench/device_gen_throughput_bench.py`; if the [BENCHMARK] CLI
format or CoV decision tree changes there, mirror the change here.

Usage:
    ./scripts/build.sh -j release   # ensure build/cmake_install/Lumice exists
    python scripts/bench_throughput.py
"""
from __future__ import annotations

import argparse
import atexit
import json
import os
import platform
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
# BIN/lib are env-overridable so the same harness runs on dev49 (CUDA bench, Linux
# docker: binary at /work/build/Release/bin/Lumice) as on a local Mac. Default is
# the local install path.
_BIN_ENV = os.environ.get("LUMICE_BENCH_BIN")
BIN = Path(_BIN_ENV) if _BIN_ENV else PROJECT_ROOT / "build" / "cmake_install" / "Lumice"
# rpath workaround for -sj shared-lib builds (backlog #345). Harmless for -j.
# LUMICE_BENCH_LIBDIR prepends a dir (dev49: /work/build/Release/lib).
_LIBDIR_ENV = os.environ.get("LUMICE_BENCH_LIBDIR")
LIB_DIRS = ([Path(_LIBDIR_ENV)] if _LIBDIR_ENV else []) + [
    PROJECT_ROOT / "build" / "Release" / "lib",
    PROJECT_ROOT / "build" / "cmake_install" / "lib",
    PROJECT_ROOT / "build" / "RelWithDebInfo" / "lib",
]
# Shared-lib search-path env var differs by platform: macOS=DYLD_, Linux=LD_.
LIB_PATH_VAR = "DYLD_LIBRARY_PATH" if platform.system() == "Darwin" else "LD_LIBRARY_PATH"

BENCH_RE = re.compile(r"\[BENCHMARK\]\s*(\{.*\})")
ROUTE_METAL = "routing via MetalTraceBackend"
ROUTE_CUDA = "routing via CudaTraceBackend"
FALLBACK = "falling back"
# GPU backends whose routing must be confirmed in the log (else the run silently
# fell back to legacy CPU and the number is meaningless). legacy/cpu_backend are
# not route-gated (cpu_backend is verify-only; legacy is the baseline path).
GPU_ROUTE_STRINGS = {"metal": ROUTE_METAL, "cuda": ROUTE_CUDA}

# Canonical throughput-regime scene set (light -> heavy). Mirrors the benchmark
# scene registry in doc/performance-testing.md ("基准场景注册表") — keep the two
# in sync. All entries are dual_fisheye_equal_area (Metal-comparable, no CLI
# fallback) AND completable by legacy within RUN_TIMEOUT_SEC, so every row yields
# a real ratio. The heaviest scene `ms3_mixed_pyramid_heavy` is in the registry
# table but NOT auto-run here: legacy --benchmark's single pass is hardwired to
# 2M rays (kBenchmarkSingleRays) and times out on that 3-MS pyramid scene, so it
# has no legacy baseline — it is Metal-only viable (see the registry note).
CONFIGS = {
    "bench_light_single_ms": PROJECT_ROOT
    / "test" / "e2e" / "configs" / "bench_light_single_ms.json",
    "ms_multi_crystal": PROJECT_ROOT
    / "test" / "e2e" / "configs" / "ms_multi_crystal.json",
    "ms_multi_crystal_complex_filter": PROJECT_ROOT
    / "test" / "e2e" / "configs" / "ms_multi_crystal_complex_filter.json",
    "ms_multi_crystal_filtered_bd": PROJECT_ROOT
    / "test" / "e2e" / "configs" / "ms_multi_crystal_filtered_bd.json",
}

# Per-run ray_num override (task-fix-throughput-bench-honesty). The committed
# heavy configs ship ray_num=2M; on Metal that completes in ~0.1s — too short a
# steady window to resolve cleanly even after the setup-exclusion fix (thermal
# noise dominates). Bump to 2e7 so the active window is ~1s+ on Metal while
# legacy (slower) still finishes well within RUN_TIMEOUT_SEC. The committed
# config files are NOT mutated; each run writes a temp config with this ray_num.
RAY_NUM_OVERRIDE = 20_000_000

# --- Resolution sweep (scrum-312.1) ------------------------------------------
# GPU throughput is NOT resolution-invariant: the device-fused XYZ accumulation
# (scrum-302) atomicAdds each exit ray into a W*H*3 float image buffer (12 B/px).
# When that buffer fits GPU L2 (512x256 = 1.5MB) accumulation is cache-fast; once
# it spills L2 (2048x1024 = 24MB) every atomicAdd hits DRAM and throughput falls
# 3.6-5x. The knee sits at the L2 boundary and MOVES with the GPU's L2 size, so a
# handful of fixed points can't locate it — a sweep can. `--res-sweep` overrides
# render[].resolution per run (temp config, committed files untouched — same
# mechanism as the ray_num override) at DEFAULT dispatch, isolating resolution as
# the single variable. (Dispatch x resolution interaction is a separate explore;
# see doc/performance-testing.md "分辨率是一等吞吐维度".)
# 2:1 aspect (dual_fisheye full-globe layout); spans the L2 knee both sides.
RES_SWEEP_DEFAULT = [
    (256, 128), (512, 256), (768, 384), (1024, 512), (1536, 768), (2048, 1024),
]
# Two representative scenes: a light one (accumulation/readback dominates, cleanest
# L2 signal) and a heavier one (throughput also varies with exit count / scatter
# locality — see the backlog readback-tax entry).
RES_SWEEP_CONFIGS_DEFAULT = ["bench_light_single_ms", "ms_multi_crystal"]

# (label, LUMICE_TRACE_BACKEND value).  None => env unset (legacy CPU).
# IMPORTANT: "legacy" must be first — subsequent backends use legacy_multi_median
# as their ratio denominator, which is only set once the legacy row runs.
BACKENDS = [
    ("legacy", None),
    ("cpu_backend", "cpu_backend"),
    ("metal", "metal"),
    ("cuda", "cuda"),
]

# Dispatch sweep applies to GPU backends (Metal, CUDA) — CPU paths ignore
# LUMICE_DISPATCH_RAY_NUM for parity with how the GUI runs. `None` means "do NOT
# set the env var" — the server picks its own backend-aware default (Metal 32768,
# CUDA 262144 since scrum-306.2, kDefaultRayNum for CPU; server.cpp ResolveGpuRoute).
# Keeping `None` distinct from the explicit value lets the table verify "default ==
# the resolved per-backend value" without conflating intents. The small-batch points
# (128, 512) characterize the "small dispatch starves the GPU" curve; CUDA adds 262144
# (the scrum-306.2 plateau ≈ 85% of the 134M intrinsic kernel rate, after capping the
# dead d_exit_ buffer) so the sweep brackets its real operating point.
DISPATCH_PLAN = {
    "legacy": [None],
    "cpu_backend": [None],
    "metal": [None, 128, 512, 2048, 32768],
    "cuda": [None, 128, 512, 2048, 32768, 262144],
}

N_REPS = 5
N_REPS_HIGH_COV = 9
COV_THRESHOLD = 0.15  # >15% triggers N=9 re-run; still >15% -> HIGH_COV_THERMAL
RUN_TIMEOUT_SEC = 240

BASELINE_BACKEND = "legacy"
BASELINE_LABEL = "[BASELINE]"
CPU_BACKEND_LABEL = "[verify-only]"  # not a baseline; see feedback_perf_baseline_is_legacy_cpu


# --- Optional env-driven matrix narrowing (for focused / robust remote runs) ---
# The committed defaults above run the full matrix. For a focused dev49 run (e.g.
# scrum-296 Step D: legacy vs cuda only, fewer reps, larger dispatch probe) these
# env knobs trim the matrix WITHOUT editing the committed defaults. Unset = full.
#   LUMICE_BENCH_BACKENDS=legacy,cuda      # subset of backend labels (order kept)
#   LUMICE_BENCH_CONFIGS=ms_multi_crystal  # subset of config labels
#   LUMICE_BENCH_DISPATCH=default,32768,65536,131072  # GPU dispatch list ("default"->None)
#   LUMICE_BENCH_NREPS=5                    # override N_REPS
def _csv_env(name: str) -> list[str] | None:
    raw = os.environ.get(name)
    if not raw:
        return None
    return [tok.strip() for tok in raw.split(",") if tok.strip()]


_sel_backends = _csv_env("LUMICE_BENCH_BACKENDS")
if _sel_backends is not None:
    BACKENDS = [b for b in BACKENDS if b[0] in _sel_backends]

_sel_configs = _csv_env("LUMICE_BENCH_CONFIGS")
if _sel_configs is not None:
    CONFIGS = {k: v for k, v in CONFIGS.items() if k in _sel_configs}

_sel_dispatch = _csv_env("LUMICE_BENCH_DISPATCH")
if _sel_dispatch is not None:
    _parsed = [None if tok == "default" else int(tok) for tok in _sel_dispatch]
    for _gpu in ("metal", "cuda"):
        DISPATCH_PLAN[_gpu] = _parsed

_nreps_env = os.environ.get("LUMICE_BENCH_NREPS")
if _nreps_env:
    N_REPS = int(_nreps_env)


def run(config_path: Path, backend_env: str | None, dispatch_num: int | None) -> dict:
    """Execute one --benchmark invocation. Returns parsed result dict."""
    env = dict(os.environ)
    env[LIB_PATH_VAR] = ":".join(str(d) for d in LIB_DIRS)
    if backend_env is None:
        env.pop("LUMICE_TRACE_BACKEND", None)
    else:
        env["LUMICE_TRACE_BACKEND"] = backend_env
    if dispatch_num is None:
        env.pop("LUMICE_DISPATCH_RAY_NUM", None)
    else:
        env["LUMICE_DISPATCH_RAY_NUM"] = str(dispatch_num)
    env.pop("LUMICE_COMMIT_RAY_NUM", None)  # keep commit granularity at default

    cmd = [str(BIN), "--benchmark", "-f", str(config_path), "-v"]
    proc = subprocess.run(
        cmd, capture_output=True, text=True, env=env, timeout=RUN_TIMEOUT_SEC
    )
    out = proc.stdout + proc.stderr
    benches = []
    for m in BENCH_RE.finditer(out):
        try:
            benches.append(json.loads(m.group(1)))
        except json.JSONDecodeError:
            pass  # malformed [BENCHMARK] line → INCOMPLETE note below
    by_mode = {b["mode"]: b["rays_per_sec"] for b in benches}

    expected_route = GPU_ROUTE_STRINGS.get(backend_env)  # None for legacy/cpu_backend
    routed_gpu = (expected_route in out) if expected_route else True
    fell_back = FALLBACK in out
    note = ""
    if expected_route is not None:  # metal or cuda — must confirm GPU routing
        if fell_back:
            note = "FELL_BACK"
        elif not routed_gpu:
            note = f"NO_{backend_env.upper()}_ROUTE(rc={proc.returncode})"
        elif len(benches) < 2:
            note = f"INCOMPLETE(n={len(benches)},rc={proc.returncode})"
    else:
        if len(benches) < 2:
            note = f"INCOMPLETE(n={len(benches)},rc={proc.returncode})"

    routed_ok = (expected_route is None) or (routed_gpu and not fell_back)
    return {
        "single_rps": by_mode.get("single"),
        "multi_rps": by_mode.get("multi"),
        "routed_ok": routed_ok,
        "note": note,
        "rc": proc.returncode,
    }


def _cov(values):
    """Coefficient of variation (stdev / mean). None for n<2 or zero mean."""
    vals = [v for v in values if v is not None]
    if len(vals) < 2:
        return None
    mean = statistics.mean(vals)
    if mean == 0:
        return None
    return statistics.stdev(vals) / mean


def measure_cell(config_path: Path, backend_env: str | None, dispatch_num: int | None,
                 n_reps: int) -> dict:
    single_vals, multi_vals, all_ok, notes = [], [], True, []
    for i in range(n_reps):
        try:
            r = run(config_path, backend_env, dispatch_num)
        except subprocess.TimeoutExpired:
            single_vals.append(None)
            multi_vals.append(None)
            all_ok = False
            notes.append(f"rep{i}:TIMEOUT")
            continue
        single_vals.append(r["single_rps"])
        multi_vals.append(r["multi_rps"])
        if not r["routed_ok"]:
            all_ok = False
        if r["note"]:
            notes.append(f"rep{i}:{r['note']}")
    s_clean = [v for v in single_vals if v is not None]
    m_clean = [v for v in multi_vals if v is not None]
    return {
        "single_vals": single_vals,
        "multi_vals": multi_vals,
        "single_median": statistics.median(s_clean) if s_clean else None,
        "multi_median": statistics.median(m_clean) if m_clean else None,
        "single_cov": _cov(single_vals),
        "multi_cov": _cov(multi_vals),
        "routed_ok": all_ok,
        "notes": notes,
    }


def _fmt_rps(v):
    return f"{v:>12,.0f}/s" if v else "             - "


def _fmt_cov(c):
    return f"{100 * c:4.1f}%" if c is not None else "  -  "


def _fmt_ratio(num, den):
    if num and den:
        return f"{num / den:5.2f}x"
    return "    - "


def _fmt_dispatch(d):
    return "default" if d is None else str(d)


def _backend_na_reason(backend_label: str, is_darwin: bool) -> str | None:
    """Which backends are not runnable on this host (returns reason, else None).

    Metal needs an Apple host; CUDA needs a CUDA-enabled build + NVIDIA device
    (the dev49 Linux docker image), which is never the Mac dev host.
    """
    if backend_label == "metal" and not is_darwin:
        return "Darwin only"
    if backend_label == "cuda" and is_darwin:
        return "CUDA host only"
    return None


def _preflight() -> list[str]:
    """Validate prerequisites; returns list of error strings (empty == OK)."""
    errs: list[str] = []
    if not BIN.is_file() or not os.access(BIN, os.X_OK):
        errs.append(
            f"Binary missing or not executable: {BIN}\n"
            f"  Run: ./scripts/build.sh -j release"
        )
    for label, path in CONFIGS.items():
        if not path.is_file():
            errs.append(f"Config missing: {label} -> {path}")
    return errs


def _override_config(configs: dict, tmp_dir: str,
                     resolution: tuple[int, int] | None = None) -> dict:
    """Write temp copies of each config with scene.ray_num = RAY_NUM_OVERRIDE and,
    when `resolution` is given, every render[].resolution set to [w, h].

    Committed config files are never mutated; returns {label: temp_path}.
    """
    out = {}
    for label, src in configs.items():
        cfg = json.loads(Path(src).read_text())
        if "scene" not in cfg or "ray_num" not in cfg["scene"]:
            raise KeyError(f"{label}: config missing scene.ray_num (cannot apply override)")
        cfg["scene"]["ray_num"] = RAY_NUM_OVERRIDE
        suffix = ""
        if resolution is not None:
            renders = cfg.get("render")
            if not isinstance(renders, list) or not renders:
                raise KeyError(f"{label}: config missing render[] (cannot set resolution)")
            for r in renders:
                r["resolution"] = [resolution[0], resolution[1]]
            suffix = f"_{resolution[0]}x{resolution[1]}"
        dst = Path(tmp_dir) / f"{label}{suffix}.json"
        dst.write_text(json.dumps(cfg))
        out[label] = dst
    return out


def _measure_with_cov_escalation(cfg_path: Path, backend_env: str | None,
                                 dispatch_num: int | None, label: str,
                                 cov_log: list[str]) -> dict:
    """measure_cell + the >15% CoV -> N=9 re-run -> HIGH_COV_THERMAL decision tree.
    Shared by the default matrix and the resolution sweep so they can't diverge."""
    cell = measure_cell(cfg_path, backend_env, dispatch_num, N_REPS)
    worst_cov = max(
        (c for c in (cell["single_cov"], cell["multi_cov"]) if c is not None),
        default=None,
    )
    if worst_cov is not None and worst_cov > COV_THRESHOLD:
        msg = (
            f"[{label}] CoV={100 * worst_cov:.1f}% > 15% @ N={N_REPS} — "
            f"re-running at N={N_REPS_HIGH_COV}..."
        )
        print(msg, flush=True)
        cov_log.append(msg)
        cell = measure_cell(cfg_path, backend_env, dispatch_num, N_REPS_HIGH_COV)
        worst_cov2 = max(
            (c for c in (cell["single_cov"], cell["multi_cov"]) if c is not None),
            default=None,
        )
        if worst_cov2 is not None and worst_cov2 > COV_THRESHOLD:
            cell["notes"].append("HIGH_COV_THERMAL")
            cov_log.append(
                f"  ↳ still {100 * worst_cov2:.1f}% after N=9 — HIGH_COV_THERMAL"
            )
    return cell


def run_res_sweep(resolutions: list[tuple[int, int]],
                  sweep_config_labels: list[str]) -> int:
    """Sweep render resolution at DEFAULT dispatch to characterize the GPU L2 knee.

    One curve per (config, backend); legacy CPU is the per-(config, resolution)
    denominator. Committed configs are never mutated (temp override). See the
    RES_SWEEP_DEFAULT comment for the mechanism.
    """
    # Honor the LUMICE_BENCH_CONFIGS narrowing already applied to CONFIGS.
    sweep_configs = {k: v for k, v in CONFIGS.items() if k in sweep_config_labels}
    if not sweep_configs:
        sys.stderr.write(
            "[bench_throughput] --res-sweep: no matching configs "
            f"(requested {sweep_config_labels}, available {list(CONFIGS)})\n"
        )
        return 2

    is_darwin = platform.system() == "Darwin"
    tmp_dir = tempfile.mkdtemp(prefix="bench_res_sweep_")
    atexit.register(shutil.rmtree, tmp_dir, ignore_errors=True)

    print(
        "# bench_throughput --res-sweep — resolution axis, DEFAULT dispatch.\n"
        "# denominator = legacy CPU multi_rps per (config, resolution).\n"
        f"# ray_num overridden to {RAY_NUM_OVERRIDE:,}; W*H*3*4 B = XYZ buffer size.\n"
        "# Watch for the throughput knee where buffer_MB crosses the GPU L2 size.\n",
        flush=True,
    )
    header = (
        f"{'backend':<12s} {'config':<26s} {'res':>11s} {'buf_MB':>7s} "
        f"{'single_med':>15s} {'multi_med':>15s} "
        f"{'cov_s':>6s} {'cov_m':>6s} {'vs_legacy':>10s} note"
    )
    print(header, flush=True)
    print("-" * len(header), flush=True)

    rows: list[dict] = []
    cov_log: list[str] = []
    for cfg_label, cfg_src in sweep_configs.items():
        for (w, h) in resolutions:
            cfg_path = _override_config({cfg_label: cfg_src}, tmp_dir, (w, h))[cfg_label]
            buf_mb = w * h * 3 * 4 / (1024 * 1024)
            legacy_multi_median = None
            for backend_label, backend_env in BACKENDS:
                na_reason = _backend_na_reason(backend_label, is_darwin)
                if na_reason is not None:
                    continue  # skip N/A rows to keep the curve readable
                label = f"{backend_label} {cfg_label} {w}x{h}"
                cell = _measure_with_cov_escalation(cfg_path, backend_env, None,
                                                    label, cov_log)
                if backend_label == BASELINE_BACKEND:
                    legacy_multi_median = cell["multi_median"]
                    ratio_str = "1.00x" if cell["multi_median"] else "    - "
                    suffix = BASELINE_LABEL
                elif backend_label == "cpu_backend":
                    ratio_str = _fmt_ratio(cell["multi_median"], legacy_multi_median)
                    suffix = CPU_BACKEND_LABEL
                else:
                    ratio_str = _fmt_ratio(cell["multi_median"], legacy_multi_median)
                    suffix = ""
                flags = list(cell["notes"])
                if not cell["routed_ok"]:
                    flags.insert(0, "ROUTE?")
                note_str = f"<-- {','.join(flags)} {suffix}".rstrip() if flags else suffix
                print(
                    f"{backend_label:<12s} {cfg_label:<26s} {f'{w}x{h}':>11s} "
                    f"{buf_mb:>7.1f} "
                    f"{_fmt_rps(cell['single_median']):>15s} "
                    f"{_fmt_rps(cell['multi_median']):>15s} "
                    f"{_fmt_cov(cell['single_cov']):>6s} "
                    f"{_fmt_cov(cell['multi_cov']):>6s} "
                    f"{ratio_str:>10s} {note_str}",
                    flush=True,
                )
                rows.append({
                    "backend": backend_label, "config": cfg_label,
                    "resolution": [w, h], "buffer_mb": round(buf_mb, 2),
                    "single_median_rps": cell["single_median"],
                    "multi_median_rps": cell["multi_median"],
                    "single_cov": cell["single_cov"], "multi_cov": cell["multi_cov"],
                    "vs_legacy_multi": (
                        cell["multi_median"] / legacy_multi_median
                        if cell["multi_median"] and legacy_multi_median else None
                    ),
                    "routed_ok": cell["routed_ok"], "notes": cell["notes"],
                })
            print("", flush=True)

    if cov_log:
        print("# CoV escalation log:", flush=True)
        for line in cov_log:
            print(f"  {line}", flush=True)
        print("", flush=True)
    print("# raw rows (json):", flush=True)
    print(json.dumps(rows, indent=2, default=str), flush=True)
    return 0


def _parse_res_list(s: str) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    for tok in s.split(","):
        tok = tok.strip().lower()
        if not tok:
            continue
        w, h = tok.split("x")
        out.append((int(w), int(h)))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Committed throughput bench harness (matrix + resolution sweep)."
    )
    parser.add_argument(
        "--res-sweep", action="store_true",
        help="sweep render resolution at default dispatch (characterize the GPU L2 knee)",
    )
    parser.add_argument(
        "--res-list", default=None, metavar="WxH,...",
        help="comma-separated resolutions for --res-sweep (default: 256x128..2048x1024)",
    )
    parser.add_argument(
        "--res-configs", default=None, metavar="label,...",
        help="config labels to sweep (default: bench_light_single_ms,ms_multi_crystal)",
    )
    args = parser.parse_args()

    errs = _preflight()
    if errs:
        sys.stderr.write("[bench_throughput] preflight failed:\n")
        for e in errs:
            sys.stderr.write(f"  - {e}\n")
        return 2

    if args.res_sweep:
        resolutions = _parse_res_list(args.res_list) if args.res_list else RES_SWEEP_DEFAULT
        sweep_labels = (
            [s.strip() for s in args.res_configs.split(",") if s.strip()]
            if args.res_configs else RES_SWEEP_CONFIGS_DEFAULT
        )
        return run_res_sweep(resolutions, sweep_labels)

    tmp_dir = tempfile.mkdtemp(prefix="bench_throughput_")
    atexit.register(shutil.rmtree, tmp_dir, ignore_errors=True)
    configs = _override_config(CONFIGS, tmp_dir)

    is_darwin = platform.system() == "Darwin"
    if not is_darwin:
        print("[bench_throughput] non-Darwin host: Metal rows -> N/A; CUDA is the GPU backend.",
              flush=True)
    else:
        print("[bench_throughput] Darwin host: CUDA rows -> N/A; Metal is the GPU backend.",
              flush=True)

    print(
        "# bench_throughput — denominator = legacy CPU multi_rps (per config).\n"
        "# Metal: single-engine; single/multi passes both run 1 engine.\n"
        "# CpuTraceBackend rows are verify-only (NOT a baseline).\n"
        f"# rays_per_sec = steady trace rate (setup excluded, --benchmark fix);\n"
        f"# ray_num overridden to {RAY_NUM_OVERRIDE:,} for a stable steady window.\n",
        flush=True,
    )

    rows = []
    cov_log: list[str] = []

    header = (
        f"{'backend':<12s} {'config':<34s} {'dispatch':>9s} "
        f"{'single_med':>15s} {'multi_med':>15s} "
        f"{'cov_s':>6s} {'cov_m':>6s} {'vs_legacy':>10s} note"
    )
    print(header, flush=True)
    print("-" * len(header), flush=True)

    for cfg_label, cfg_path in configs.items():
        legacy_multi_median = None

        for backend_label, backend_env in BACKENDS:
            na_reason = _backend_na_reason(backend_label, is_darwin)
            if na_reason is not None:
                print(
                    f"{backend_label:<12s} {cfg_label:<34s} {'-':>9s} "
                    f"{'N/A':>15s} {'N/A':>15s} {'-':>6s} {'-':>6s} {'-':>10s} "
                    f"<-- N/A ({na_reason})",
                    flush=True,
                )
                continue

            for dispatch_num in DISPATCH_PLAN[backend_label]:
                # CoV decision tree: >15% @ N=5 -> rerun at N=9; still >15% -> flag thermal.
                label = f"{backend_label} {cfg_label} disp={_fmt_dispatch(dispatch_num)}"
                cell = _measure_with_cov_escalation(
                    cfg_path, backend_env, dispatch_num, label, cov_log
                )

                if backend_label == BASELINE_BACKEND:
                    legacy_multi_median = cell["multi_median"]
                    ratio_str = "1.00x" if cell["multi_median"] else "    - "
                    suffix = BASELINE_LABEL
                elif backend_label == "cpu_backend":
                    ratio_str = _fmt_ratio(cell["multi_median"], legacy_multi_median)
                    suffix = CPU_BACKEND_LABEL
                else:
                    ratio_str = _fmt_ratio(cell["multi_median"], legacy_multi_median)
                    suffix = ""

                flags = list(cell["notes"])
                if not cell["routed_ok"]:
                    flags.insert(0, "ROUTE?")
                note_str = f"<-- {','.join(flags)} {suffix}".rstrip() if flags else suffix

                print(
                    f"{backend_label:<12s} {cfg_label:<34s} "
                    f"{_fmt_dispatch(dispatch_num):>9s} "
                    f"{_fmt_rps(cell['single_median']):>15s} "
                    f"{_fmt_rps(cell['multi_median']):>15s} "
                    f"{_fmt_cov(cell['single_cov']):>6s} "
                    f"{_fmt_cov(cell['multi_cov']):>6s} "
                    f"{ratio_str:>10s} {note_str}",
                    flush=True,
                )

                rows.append({
                    "backend": backend_label,
                    "config": cfg_label,
                    "dispatch": _fmt_dispatch(dispatch_num),
                    "single_median_rps": cell["single_median"],
                    "multi_median_rps": cell["multi_median"],
                    "single_cov": cell["single_cov"],
                    "multi_cov": cell["multi_cov"],
                    "vs_legacy_multi": (
                        cell["multi_median"] / legacy_multi_median
                        if cell["multi_median"] and legacy_multi_median else None
                    ),
                    "routed_ok": cell["routed_ok"],
                    "notes": cell["notes"],
                })
        print("", flush=True)

    if cov_log:
        print("# CoV escalation log:", flush=True)
        for line in cov_log:
            print(f"  {line}", flush=True)
        print("", flush=True)

    print("# raw rows (json):", flush=True)
    print(json.dumps(rows, indent=2, default=str), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
