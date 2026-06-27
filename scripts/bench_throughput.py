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
# set the env var" — the server picks its own backend-aware default (32768 for any
# GPU route, kDefaultRayNum for CPU; server.cpp ResolveGpuRoute). Keeping `None`
# distinct from `32768` lets the table verify "default == 32768 today" without
# conflating intents if the default later moves. The small-batch points (128, 512)
# characterize the "small dispatch starves the GPU" curve that motivates the
# single-engine large-dispatch route (doc/seam-design.md §5).
DISPATCH_PLAN = {
    "legacy": [None],
    "cpu_backend": [None],
    "metal": [None, 128, 512, 2048, 32768],
    "cuda": [None, 128, 512, 2048, 32768],
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


def _override_ray_num(configs: dict, tmp_dir: str) -> dict:
    """Write temp copies of each config with scene.ray_num = RAY_NUM_OVERRIDE.

    Committed config files are never mutated; returns {label: temp_path}.
    """
    out = {}
    for label, src in configs.items():
        cfg = json.loads(Path(src).read_text())
        if "scene" not in cfg or "ray_num" not in cfg["scene"]:
            raise KeyError(f"{label}: config missing scene.ray_num (cannot apply override)")
        cfg["scene"]["ray_num"] = RAY_NUM_OVERRIDE
        dst = Path(tmp_dir) / f"{label}.json"
        dst.write_text(json.dumps(cfg))
        out[label] = dst
    return out


def main() -> int:
    errs = _preflight()
    if errs:
        sys.stderr.write("[bench_throughput] preflight failed:\n")
        for e in errs:
            sys.stderr.write(f"  - {e}\n")
        return 2

    tmp_dir = tempfile.mkdtemp(prefix="bench_throughput_")
    atexit.register(shutil.rmtree, tmp_dir, ignore_errors=True)
    configs = _override_ray_num(CONFIGS, tmp_dir)

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
                cell = measure_cell(cfg_path, backend_env, dispatch_num, N_REPS)

                # CoV decision tree: >15% @ N=5 -> rerun at N=9; still >15% -> flag thermal.
                worst_cov = max(
                    (c for c in (cell["single_cov"], cell["multi_cov"]) if c is not None),
                    default=None,
                )
                if worst_cov is not None and worst_cov > COV_THRESHOLD:
                    msg = (
                        f"[{backend_label} {cfg_label} disp={_fmt_dispatch(dispatch_num)}] "
                        f"CoV={100 * worst_cov:.1f}% > 15% @ N={N_REPS} — "
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
