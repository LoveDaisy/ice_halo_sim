#!/usr/bin/env python3
"""
Reference generation and threshold calibration driver for auto_ev GUI tests.

Phase A: Run gui_test N times with --keep-export-png, pixel-average the
  captures per (scene, mode), and save the mean image as the new reference.
Phase B: Run gui_test N_calib times, parse PSNR output from stderr, and
  compute per-scene threshold recommendations (mean - 3sigma, floor to 0.5 dB).

See AGENTS.md "GUI Test Reference Regeneration" for usage.
"""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime

try:
    import numpy as np
    from PIL import Image
except ImportError:
    print("ERROR: numpy and Pillow are required. Install with: pip install numpy Pillow", file=sys.stderr)
    sys.exit(1)

# Auto-EV scene names — must match kScenes[] order in test_gui_auto_ev.cpp
SCENE_NAMES = [
    "halo_22",
    "multi_scat",
    "color",
    "pyramid",
    "cza",
    "parhelion",
    "filters",
    "rp46",
    "rp46_nof",
]

MODES = ["off", "on"]
STAGING_DIR = "/tmp/gui_refs_build"

# PSNR output pattern from CheckAgainstReference in test_gui_auto_ev.cpp
_PSNR_RE = re.compile(r"\[auto_ev\]\s+(\S+):\s+PSNR=([0-9.]+)\s+dB")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _run(binary: str, extra_args: list[str], capture_stderr: bool = False) -> tuple[int, str]:
    cmd = [binary, "--filter", "auto_ev"] + extra_args
    result = subprocess.run(cmd, capture_output=capture_stderr, text=True)
    return result.returncode, result.stderr if capture_stderr else ""


def _collect_pngs(run_dir: str) -> None:
    """Move /tmp/lumice_auto_ev_*.png → run_dir/<key>.png for every (scene, mode)."""
    os.makedirs(run_dir, exist_ok=True)
    for scene in SCENE_NAMES:
        for mode in MODES:
            key = f"{scene}_{mode}"
            src = f"/tmp/lumice_auto_ev_{key}.png"
            dst = os.path.join(run_dir, f"{key}.png")
            if os.path.exists(src):
                shutil.move(src, dst)
            else:
                print(f"  WARNING: expected PNG not found: {src}", file=sys.stderr)


def _load_as_float32(path: str) -> np.ndarray:
    return np.array(Image.open(path).convert("RGB"), dtype=np.float32) / 255.0


def _to_uint8(arr: np.ndarray) -> np.ndarray:
    return np.clip(arr * 255.0 + 0.5, 0, 255).astype(np.uint8)


def _save_jpeg(arr: np.ndarray, path: str, quality: int) -> int:
    Image.fromarray(_to_uint8(arr)).save(path, "JPEG", quality=quality)
    return os.path.getsize(path)


def _save_png(arr: np.ndarray, path: str) -> int:
    Image.fromarray(_to_uint8(arr)).save(path, "PNG")
    return os.path.getsize(path)


def _rms(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2)))


# ---------------------------------------------------------------------------
# Phase A — mean-ref generation
# ---------------------------------------------------------------------------


def phase_a(args: argparse.Namespace) -> None:
    binary = args.binary
    n = args.n
    refs_dir = args.refs_dir
    quality = args.quality

    print(f"[Phase A] Mean-ref generation: N={n} runs, JPEG quality={quality}")
    print(f"[Phase A] Binary : {binary}")
    print(f"[Phase A] Refs   : {refs_dir}")

    # Clear staging dir for idempotent reruns
    if os.path.exists(STAGING_DIR):
        shutil.rmtree(STAGING_DIR)
    os.makedirs(STAGING_DIR)

    # Collect N independent runs
    for i in range(n):
        run_dir = os.path.join(STAGING_DIR, f"run_{i}")
        print(f"[Phase A] Run {i + 1}/{n}...", flush=True)
        rc, _ = _run(binary, ["--keep-export-png"])
        if rc != 0:
            print(f"  WARNING: run {i} exited {rc}", file=sys.stderr)
        _collect_pngs(run_dir)

    # Per (scene, mode): pixel-average → apply format silence rule → save reference
    print()
    updated = 0
    for scene in SCENE_NAMES:
        for mode in MODES:
            key = f"{scene}_{mode}"

            frames: list[np.ndarray] = []
            for i in range(n):
                p = os.path.join(STAGING_DIR, f"run_{i}", f"{key}.png")
                if os.path.exists(p):
                    frames.append(_load_as_float32(p))
                else:
                    print(f"  WARNING: missing {p}", file=sys.stderr)

            if not frames:
                print(f"  ERROR: no frames for {key} — skipping", file=sys.stderr)
                continue

            stack = np.stack(frames, axis=0)   # (N, H, W, C)
            mean_img = stack.mean(axis=0)       # (H, W, C)
            original_rms = float(np.sqrt(np.mean(np.var(stack, axis=0))))

            # Evaluate both formats via temporary files
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
                tmp_jpg = f.name
            with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
                tmp_png = f.name
            try:
                jpg_size = _save_jpeg(mean_img, tmp_jpg, quality)
                png_size = _save_png(mean_img, tmp_png)
                jpg_decoded = _load_as_float32(tmp_jpg)
                jpg_extra_rms = _rms(mean_img, jpg_decoded)

                # Silence rule: prefer JPEG unless size or noise budget exceeded
                use_jpg = jpg_size <= png_size and jpg_extra_rms <= original_rms * 1.5
                chosen_fmt = "jpg" if use_jpg else "png"

                print(
                    f"  {key}: orig_rms={original_rms:.6f} jpg_noise={jpg_extra_rms:.6f} "
                    f"jpg={jpg_size}B png={png_size}B → {chosen_fmt}"
                )

                ref_base = os.path.join(refs_dir, f"auto_ev_{key}")
                if use_jpg:
                    shutil.copy(tmp_jpg, ref_base + ".jpg")
                else:
                    shutil.copy(tmp_png, ref_base + ".png")
                    # Remove old .jpg so the stale file does not mislead future runs
                    old_jpg = ref_base + ".jpg"
                    if os.path.exists(old_jpg):
                        os.remove(old_jpg)
                    print(
                        f"  WARNING: {key} saved as PNG — update ref path in "
                        f"test/gui/test_gui_auto_ev.cpp from .jpg to .png",
                        file=sys.stderr,
                    )
            finally:
                os.unlink(tmp_jpg)
                os.unlink(tmp_png)
            updated += 1

    print(f"\n[Phase A] Done — {updated} references updated in {refs_dir}")


# ---------------------------------------------------------------------------
# Phase B — threshold calibration
# ---------------------------------------------------------------------------


def phase_b(args: argparse.Namespace) -> None:
    binary = args.binary
    n_calib = args.n_calib
    refs_dir = args.refs_dir

    print(f"[Phase B] Threshold calibration: N_calib={n_calib} runs")
    print(f"[Phase B] Binary : {binary}")

    psnr_data: dict[str, list[float]] = {}
    for i in range(n_calib):
        print(f"[Phase B] Calibration run {i + 1}/{n_calib}...", flush=True)
        _, stderr = _run(binary, [], capture_stderr=True)
        for m in _PSNR_RE.finditer(stderr):
            tag, val = m.group(1), float(m.group(2))
            psnr_data.setdefault(tag, []).append(val)

    if not psnr_data:
        print("ERROR: no PSNR output parsed — check that references exist and tests pass", file=sys.stderr)
        sys.exit(1)

    print("\n[Phase B] Recommendations (mean − 3σ, floor to 0.5 dB precision):")
    scenes_out: dict[str, dict] = {}
    for tag in sorted(psnr_data):
        vals = np.array(psnr_data[tag])
        mean = float(vals.mean())
        std = float(vals.std(ddof=0))
        threshold = math.floor((mean - 3 * std) * 2) / 2
        print(f"  {tag}: mean={mean:.2f} dB  std={std:.4f} dB  threshold={threshold:.1f} dB")
        scenes_out[tag] = {
            "psnr_mean": round(mean, 4),
            "psnr_std": round(std, 4),
            "threshold": threshold,
        }

    out = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "n_ref_runs": getattr(args, "n", 10),
        "n_calib_runs": n_calib,
        "scenes": scenes_out,
    }
    json_path = os.path.join(refs_dir, "_thresholds.json")
    with open(json_path, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"\n[Phase B] Thresholds written to {json_path}")
    print("[Phase B] Copy 'threshold' values into kScenes[] in test/gui/test_gui_auto_ev.cpp")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Regenerate auto_ev mean-ref images and calibrate PSNR thresholds."
    )
    parser.add_argument(
        "--binary",
        default="build/Release/bin/gui_test",
        help="Path to gui_test binary (default: build/Release/bin/gui_test)",
    )
    parser.add_argument(
        "--n", type=int, default=10, help="Phase A: number of reference runs (default: 10)"
    )
    parser.add_argument(
        "--n-calib", type=int, default=10, help="Phase B: number of calibration runs (default: 10)"
    )
    parser.add_argument(
        "--refs-dir",
        default="test/gui/references",
        help="Reference image directory (default: test/gui/references)",
    )
    parser.add_argument("--quality", type=int, default=85, help="JPEG quality (default: 85)")
    parser.add_argument("--phase-a-only", action="store_true", help="Only run Phase A")
    parser.add_argument("--phase-b-only", action="store_true", help="Only run Phase B")

    args = parser.parse_args()

    if args.phase_a_only and args.phase_b_only:
        print("ERROR: --phase-a-only and --phase-b-only are mutually exclusive", file=sys.stderr)
        sys.exit(1)
    if not os.path.isfile(args.binary):
        print(f"ERROR: binary not found: {args.binary}", file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(args.refs_dir):
        print(f"ERROR: refs-dir not found: {args.refs_dir}", file=sys.stderr)
        sys.exit(1)

    if args.phase_b_only:
        phase_b(args)
    elif args.phase_a_only:
        phase_a(args)
    else:
        phase_a(args)
        phase_b(args)


if __name__ == "__main__":
    main()
