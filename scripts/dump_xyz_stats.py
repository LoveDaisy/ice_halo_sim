#!/usr/bin/env python3
"""Batch XYZ percentile statistics tool for Lumice auto-EV algorithm exploration.

Runs 9 e2e scenes via C API (ctypes), extracts unfiltered/filtered XYZ buffers,
computes normalized linear-Y percentile + distribution stats, outputs per-scene JSON
and a consolidated summary.

Usage:
    python scripts/dump_xyz_stats.py [--scenes <s1,s2,...>] [--output <dir>] [--no-rp46]

Prerequisites:
    Build release: ./scripts/build.sh -j release
    Library:       build/Release/lib/liblumice.dylib  (override via LUMICE_LIB env)
"""

import argparse
import ctypes
import datetime
import json
import os
import sys
import time

import numpy as np

# ─── Scene table ──────────────────────────────────────────────────────────────

ALL_SCENES = {
    "halo_22":      "test/e2e/configs/halo_22.json",
    "multi_scatter": "test/e2e/configs/multi_scatter.json",
    "color":        "test/e2e/configs/color.json",
    "pyramid":      "test/e2e/configs/pyramid.json",
    "cza":          "test/e2e/configs/cza.json",
    "parhelion":    "test/e2e/configs/parhelion.json",
    "filters":      "test/e2e/configs/filters.json",
    "rp46":         "test/e2e/configs/raypath_symmetry_4_6.json",
    "rp46_nof":     "test/e2e/configs/raypath_symmetry_4_6_nofilter.json",
}
RP46_SCENES = {"rp46", "rp46_nof"}

TIMEOUT_DEFAULT_S = 60
TIMEOUT_RP46_S = 90

PERCENTILE_KEYS = ["p50", "p70", "p90", "p95", "p99", "p99_3", "p99_5", "p99_7", "p99_9", "p99_95", "p99_99"]
PERCENTILE_VALUES = [50.0, 70.0, 90.0, 95.0, 99.0, 99.3, 99.5, 99.7, 99.9, 99.95, 99.99]

# ─── ctypes structs ───────────────────────────────────────────────────────────

# Mirrors LUMICE_RawXyzResult from src/include/lumice.h:63-77
# Layout verified by manual offset calculation (72 bytes on 64-bit macOS).
class LUMICE_RawXyzResult(ctypes.Structure):
    _fields_ = [
        ("renderer_id",                   ctypes.c_int),
        ("img_width",                     ctypes.c_int),
        ("img_height",                    ctypes.c_int),
        # implicit pad 4B (pointer alignment)
        ("xyz_buffer",                    ctypes.POINTER(ctypes.c_float)),
        ("snapshot_intensity",            ctypes.c_float),
        ("intensity_factor",              ctypes.c_float),
        ("has_valid_data",                ctypes.c_int),
        # implicit pad 4B (uint64 alignment)
        ("snapshot_generation",           ctypes.c_uint64),
        ("effective_pixels",              ctypes.c_int),
        # implicit pad 4B (pointer alignment)
        ("unfiltered_xyz_buffer",         ctypes.POINTER(ctypes.c_float)),
        ("unfiltered_snapshot_intensity", ctypes.c_float),
        # implicit pad 4B (struct tail alignment to 8)
    ]


assert ctypes.sizeof(LUMICE_RawXyzResult) == 72, (
    f"LUMICE_RawXyzResult size mismatch: {ctypes.sizeof(LUMICE_RawXyzResult)} != 72 — "
    "check lumice.h and re-verify field layout"
)

# LUMICE_ServerState constants (lumice.h:47-51)
LUMICE_SERVER_IDLE      = 0
LUMICE_SERVER_RUNNING   = 1
LUMICE_SERVER_NOT_READY = 2

# ─── Library loading ──────────────────────────────────────────────────────────

def load_lib():
    lib_path = os.environ.get("LUMICE_LIB", "build/Release/lib/liblumice.dylib")
    if not os.path.exists(lib_path):
        print(f"[ERROR] Library not found: {lib_path}", file=sys.stderr)
        print("[ERROR] Build first: ./scripts/build.sh -j release", file=sys.stderr)
        sys.exit(1)

    try:
        lib = ctypes.CDLL(lib_path)
    except OSError as e:
        print(f"[ERROR] Failed to load {lib_path}: {e}", file=sys.stderr)
        sys.exit(1)

    lib.LUMICE_CreateServer.restype  = ctypes.c_void_p
    lib.LUMICE_CreateServer.argtypes = []

    lib.LUMICE_DestroyServer.restype  = None
    lib.LUMICE_DestroyServer.argtypes = [ctypes.c_void_p]

    lib.LUMICE_CommitConfigFromFile.restype  = ctypes.c_int
    lib.LUMICE_CommitConfigFromFile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]

    lib.LUMICE_QueryServerState.restype  = ctypes.c_int
    lib.LUMICE_QueryServerState.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]

    lib.LUMICE_GetRawXyzResults.restype  = ctypes.c_int
    lib.LUMICE_GetRawXyzResults.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(LUMICE_RawXyzResult),
        ctypes.c_int,
    ]

    return lib

# ─── Statistics computation ───────────────────────────────────────────────────

def _zero_stats(pixel_count):
    return {
        "pixel_count":    pixel_count,
        "non_zero_count": 0,
        "percentiles":    {k: 0.0 for k in PERCENTILE_KEYS},
        "mean":           0.0,
        "std":            0.0,
        "cv":             0.0,
        "max":            0.0,
        "saturation_rate": 0.0,
    }


def compute_stats(ptr, pixel_count, intensity):
    """Compute normalized linear-Y statistics from an XYZ float buffer pointer."""
    if not ptr or pixel_count <= 0 or intensity <= 0.0:
        return _zero_stats(pixel_count)

    addr = ctypes.cast(ptr, ctypes.c_void_p).value
    if not addr:
        return _zero_stats(pixel_count)

    n = pixel_count * 3
    float_arr = (ctypes.c_float * n).from_address(addr)
    raw = np.frombuffer(float_arr, dtype=np.float32).copy()

    # Y channel: index 1 of each XYZ triplet
    y_all = raw[1::3]
    nonzero_mask = y_all > 0.0
    nonzero_count = int(nonzero_mask.sum())

    if nonzero_count == 0:
        return _zero_stats(pixel_count)

    norm_y = y_all[nonzero_mask] / intensity
    pct_vals = np.percentile(norm_y, PERCENTILE_VALUES)

    mean_v = float(np.mean(norm_y))
    std_v  = float(np.std(norm_y))
    max_v  = float(np.max(norm_y))

    return {
        "pixel_count":    pixel_count,
        "non_zero_count": nonzero_count,
        "percentiles":    {k: float(v) for k, v in zip(PERCENTILE_KEYS, pct_vals)},
        "mean":           mean_v,
        "std":            std_v,
        "cv":             std_v / mean_v if mean_v > 0.0 else 0.0,
        "max":            max_v,
        "saturation_rate": float(np.sum(norm_y > 1.0) / nonzero_count),
    }

# ─── Per-scene runner ─────────────────────────────────────────────────────────

def run_scene(lib, scene_name, config_path, timeout_s):
    """Run one scene; returns result dict. Raises RuntimeError on fatal failure."""
    print(f"  [{scene_name}] Starting (timeout={timeout_s}s)...", flush=True)
    t_start = time.time()

    server = lib.LUMICE_CreateServer()
    if not server:
        raise RuntimeError(f"LUMICE_CreateServer returned NULL")

    truncated = False
    try:
        err = lib.LUMICE_CommitConfigFromFile(server, config_path.encode("utf-8"))
        if err != 0:
            raise RuntimeError(f"CommitConfigFromFile failed: err={err}, config={config_path}")

        # result array: 1 renderer + 1 sentinel slot
        results = (LUMICE_RawXyzResult * 2)()
        state_out = ctypes.c_int(0)

        while True:
            elapsed = time.time() - t_start

            if elapsed > timeout_s:
                err = lib.LUMICE_GetRawXyzResults(server, results, 1)
                if err == 0 and results[0].has_valid_data and results[0].xyz_buffer:
                    print(f"  [{scene_name}] Timeout ({elapsed:.1f}s) — using current data (truncated)", flush=True)
                    truncated = True
                    break
                raise RuntimeError(f"Timeout after {elapsed:.1f}s with no valid data")

            err = lib.LUMICE_GetRawXyzResults(server, results, 1)
            if err != 0:
                raise RuntimeError(f"GetRawXyzResults failed: err={err}")

            err2 = lib.LUMICE_QueryServerState(server, ctypes.byref(state_out))
            if err2 != 0:
                raise RuntimeError(f"QueryServerState failed: err={err2}")

            state = state_out.value
            if state == LUMICE_SERVER_NOT_READY:
                raise RuntimeError("Server NOT_READY — failed to initialize")

            if results[0].has_valid_data and state == LUMICE_SERVER_IDLE:
                elapsed = time.time() - t_start
                print(f"  [{scene_name}] Done in {elapsed:.1f}s", flush=True)
                break

            time.sleep(0.5)

        r = results[0]
        pixel_count = r.img_width * r.img_height

        if r.xyz_buffer is None:
            print(f"  [{scene_name}] WARNING: xyz_buffer is NULL despite has_valid_data", file=sys.stderr)

        filtered_stats   = compute_stats(r.xyz_buffer,            pixel_count, r.snapshot_intensity)
        unfiltered_stats = compute_stats(r.unfiltered_xyz_buffer, pixel_count, r.unfiltered_snapshot_intensity)

        return {
            "scene":         scene_name,
            "config":        config_path,
            "timestamp":     datetime.datetime.now().isoformat(),
            "run_duration_s": round(time.time() - t_start, 2),
            "truncated":     truncated,
            "unfiltered":    unfiltered_stats,
            "filtered":      filtered_stats,
        }

    finally:
        lib.LUMICE_DestroyServer(server)

# ─── Summary Markdown ─────────────────────────────────────────────────────────

_SUMMARY_PCTS = ["p95", "p99", "p99_5", "p99_9"]


def build_summary_md(all_results, total_elapsed_s):
    lines = []
    lines.append("# XYZ Stats Summary\n")
    lines.append(f"Generated: {datetime.datetime.now().isoformat()}  ")
    lines.append(f"Total wall-time: {total_elapsed_s:.1f}s\n")

    cols = ["scene", "dur_s", "trunc"]
    for prefix in ("unf", "flt"):
        for p in _SUMMARY_PCTS:
            cols.append(f"{prefix}.{p}")
        cols += [f"{prefix}.max", f"{prefix}.sat_rate"]
    lines.append("| " + " | ".join(cols) + " |")
    lines.append("| " + " | ".join(["---"] * len(cols)) + " |")

    for r in all_results:
        if r.get("error"):
            row = [r["scene"], "ERR", "-"] + ["ERR"] * (len(cols) - 3)
            lines.append("| " + " | ".join(row) + " |")
            continue
        row = [r["scene"], str(r["run_duration_s"]), "Y" if r["truncated"] else "N"]
        for side in ("unfiltered", "filtered"):
            d = r[side]
            for p in _SUMMARY_PCTS:
                val = d["percentiles"].get(p, float("nan"))
                row.append(f"{val:.5f}")
            row.append(f"{d['max']:.5f}")
            row.append(f"{d['saturation_rate']:.5f}")
        lines.append("| " + " | ".join(row) + " |")

    return "\n".join(lines) + "\n"

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Dump XYZ percentile stats for Lumice scenes")
    parser.add_argument("--scenes", type=str, default=None,
                        help="Comma-separated scene names (default: all 9)")
    parser.add_argument("--output", type=str, default="xyz_stats_output",
                        help="Output directory (default: xyz_stats_output/)")
    parser.add_argument("--no-rp46", action="store_true",
                        help="Skip rp46 and rp46_nof scenes")
    args = parser.parse_args()

    if args.scenes:
        scene_names = [s.strip() for s in args.scenes.split(",")]
        unknown = [s for s in scene_names if s not in ALL_SCENES]
        if unknown:
            print(f"[ERROR] Unknown scene(s): {', '.join(unknown)}", file=sys.stderr)
            print(f"[ERROR] Valid scenes: {', '.join(ALL_SCENES)}", file=sys.stderr)
            sys.exit(1)
    else:
        scene_names = list(ALL_SCENES.keys())

    if args.no_rp46:
        scene_names = [s for s in scene_names if s not in RP46_SCENES]
        print("[INFO] --no-rp46: skipping rp46/rp46_nof", flush=True)

    os.makedirs(args.output, exist_ok=True)
    lib = load_lib()
    lib_path = os.environ.get("LUMICE_LIB", "build/Release/lib/liblumice.dylib")
    print(f"[INFO] Loaded: {lib_path}", flush=True)
    print(f"[INFO] Running {len(scene_names)} scene(s): {', '.join(scene_names)}", flush=True)

    all_results = []
    t_total = time.time()

    for scene_name in scene_names:
        config_path = ALL_SCENES[scene_name]
        timeout_s = TIMEOUT_RP46_S if scene_name in RP46_SCENES else TIMEOUT_DEFAULT_S

        try:
            result = run_scene(lib, scene_name, config_path, timeout_s)
        except Exception as e:
            print(f"  [{scene_name}] ERROR: {e}", file=sys.stderr, flush=True)
            result = {
                "scene":         scene_name,
                "config":        config_path,
                "timestamp":     datetime.datetime.now().isoformat(),
                "run_duration_s": 0.0,
                "truncated":     False,
                "error":         str(e),
                "unfiltered":    None,
                "filtered":      None,
            }

        all_results.append(result)
        out_path = os.path.join(args.output, f"{scene_name}.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        print(f"  [{scene_name}] -> {out_path}", flush=True)

    total_elapsed = time.time() - t_total
    print(f"\n[INFO] Total wall-time: {total_elapsed:.1f}s", flush=True)

    summary_json = os.path.join(args.output, "summary.json")
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(all_results, f, indent=2)
    print(f"[INFO] Summary JSON:     {summary_json}", flush=True)

    summary_md = os.path.join(args.output, "summary.md")
    with open(summary_md, "w", encoding="utf-8") as f:
        f.write(build_summary_md(all_results, total_elapsed))
    print(f"[INFO] Summary Markdown: {summary_md}", flush=True)


if __name__ == "__main__":
    main()
