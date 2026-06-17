#!/usr/bin/env python3
"""Committed throughput bench harness (task-270.6 / performance layer §1.5).

Runs `Lumice --benchmark` across a 3-backend × heavy-config × Metal-dispatch
matrix, reports median rays/s + CoV per cell, ratios against the legacy CPU
baseline. Replaces the gitignored `scratchpad/bench/seam_exit_bench.py` (whose
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

import json
import os
import platform
import re
import statistics
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
BIN = PROJECT_ROOT / "build" / "cmake_install" / "Lumice"
# rpath workaround for -sj shared-lib builds (backlog #345). Harmless for -j.
DYLIB_DIRS = [
    PROJECT_ROOT / "build" / "Release" / "lib",
    PROJECT_ROOT / "build" / "cmake_install" / "lib",
    PROJECT_ROOT / "build" / "RelWithDebInfo" / "lib",
]

BENCH_RE = re.compile(r"\[BENCHMARK\]\s*(\{.*\})")
ROUTE_METAL = "routing via MetalTraceBackend"
FALLBACK = "falling back"

CONFIGS = {
    "ms_multi_crystal_complex_filter": PROJECT_ROOT
    / "test" / "e2e" / "configs" / "ms_multi_crystal_complex_filter.json",
    "ms_multi_crystal_filtered_bd": PROJECT_ROOT
    / "test" / "e2e" / "configs" / "ms_multi_crystal_filtered_bd.json",
}

# (label, LUMICE_TRACE_BACKEND value).  None => env unset (legacy CPU).
BACKENDS = [
    ("legacy", None),
    ("cpu_backend", "cpu_backend"),
    ("metal", "metal"),
]

# Dispatch sweep applies to Metal only — CPU paths ignore LUMICE_DISPATCH_RAY_NUM
# for parity with how the GUI runs. `None` means "do NOT set the env var" — the
# server picks its own backend-aware default (32768 for Metal, kDefaultRayNum for
# CPU). Keeping `None` distinct from `32768` lets the table verify "default ==
# 32768 today" without conflating intents if the default later moves.
DISPATCH_PLAN = {
    "legacy": [None],
    "cpu_backend": [None],
    "metal": [None, 128, 512, 2048, 32768],
}

N_REPS = 5
N_REPS_HIGH_COV = 9
COV_THRESHOLD = 0.15  # >15% triggers N=9 re-run; still >15% -> HIGH_COV_THERMAL
RUN_TIMEOUT_SEC = 240

BASELINE_BACKEND = "legacy"
BASELINE_LABEL = "[BASELINE]"
CPU_BACKEND_LABEL = "[verify-only]"  # not a baseline; see feedback_perf_baseline_is_legacy_cpu


def run(config_path: Path, backend_env: str | None, dispatch_num: int | None) -> dict:
    """Execute one --benchmark invocation. Returns parsed result dict."""
    env = dict(os.environ)
    env["DYLD_LIBRARY_PATH"] = ":".join(str(d) for d in DYLIB_DIRS)
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
    benches = [json.loads(m.group(1)) for m in BENCH_RE.finditer(out)]
    by_mode = {b["mode"]: b["rays_per_sec"] for b in benches}

    routed_metal = ROUTE_METAL in out
    fell_back = FALLBACK in out
    note = ""
    if backend_env == "metal":
        if fell_back:
            note = "FELL_BACK"
        elif not routed_metal:
            note = f"NO_METAL_ROUTE(rc={proc.returncode})"
        elif len(benches) < 2:
            note = f"INCOMPLETE(n={len(benches)},rc={proc.returncode})"
    else:
        if len(benches) < 2:
            note = f"INCOMPLETE(n={len(benches)},rc={proc.returncode})"

    routed_ok = (backend_env != "metal") or (routed_metal and not fell_back)
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


def main() -> int:
    errs = _preflight()
    if errs:
        sys.stderr.write("[bench_throughput] preflight failed:\n")
        for e in errs:
            sys.stderr.write(f"  - {e}\n")
        return 2

    is_darwin = platform.system() == "Darwin"
    if not is_darwin:
        print("[bench_throughput] non-Darwin host: Metal rows will be reported as 'N/A'.",
              flush=True)

    print(
        "# bench_throughput — denominator = legacy CPU multi_rps (per config).\n"
        "# Metal: single-engine; single/multi passes both run 1 engine.\n"
        "# CpuTraceBackend rows are verify-only (NOT a baseline).\n",
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

    for cfg_label, cfg_path in CONFIGS.items():
        legacy_multi_median = None

        for backend_label, backend_env in BACKENDS:
            if backend_label == "metal" and not is_darwin:
                print(
                    f"{backend_label:<12s} {cfg_label:<34s} {'-':>9s} "
                    f"{'N/A':>15s} {'N/A':>15s} {'-':>6s} {'-':>6s} {'-':>10s} "
                    f"<-- N/A (Darwin only)",
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
                    ratio_str = "1.00x"
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
