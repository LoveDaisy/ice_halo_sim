#!/usr/bin/env python3
"""
Reference generation and threshold calibration driver for GUI visual-regression tests.

Phase A: Run gui_test N times with --keep-export-png, pixel-average the
  captures per (scene, mode), and save the mean image as the new reference.
Phase B: Run gui_test N_calib times, parse PSNR output from stderr, and
  compute per-scene threshold recommendations (mean - 3sigma, floor to 0.5 dB).

Both phases are driven by the GROUPS registry below: a reference group names the
gui_test category to filter on, its scenes/modes, and the tmp/reference filename
prefixes. Adding a visual-regression suite means adding a GROUPS entry — the Phase
A/B algorithms themselves stay untouched.

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
from dataclasses import dataclass
from datetime import datetime

try:
    import numpy as np
    from PIL import Image
except ImportError:
    print("ERROR: numpy and Pillow are required. Install with: pip install numpy Pillow", file=sys.stderr)
    sys.exit(1)


@dataclass(frozen=True)
class ReferenceGroup:
    """One registered visual-regression suite.

    key         gui_test category AND the "[<key>]" stderr tag its comparisons print;
                also the _thresholds.json group name. Must not be a substring of any
                other gui_test category — --filter is substring-matched, so a colliding
                key would pull unrelated tests into this group's PSNR sampling.
    scenes      scene names, matching the test registration order in the group's source.
    modes       per-scene variant suffixes; [None] for groups without variants.
    tmp_prefix  the test writes its capture to /tmp/<tmp_prefix><key>.png.
    ref_prefix  the reference lives at <refs_dir>/<ref_prefix><key>.jpg.
    source      the group's test source, quoted in the "copy thresholds back" hint.
    """

    key: str
    scenes: list[str]
    modes: list[str | None]
    tmp_prefix: str
    ref_prefix: str
    source: str


# Auto-EV scene names — must match kScenes[] order in test_gui_auto_ev.cpp.
# Only the auto-EV-applied ("on") capture is exported/compared. The legacy "off"
# (intensity_factor=1.0) mode was dropped in chore-auto-ev-regression-drop-off — the GUI
# has no auto-EV toggle, so off was a degenerate non-default state with no unique coverage.
GROUPS: dict[str, ReferenceGroup] = {
    "auto_ev": ReferenceGroup(
        key="auto_ev",
        scenes=[
            "halo_22",
            "multi_scat",
            "color",
            "pyramid",
            "cza",
            "parhelion",
            "filters",
            "rp46",
            "rp46_nof",
            "overlay_ea",
        ],
        modes=["on"],
        tmp_prefix="lumice_auto_ev_",
        ref_prefix="auto_ev_",
        source="test/gui/visual/test_gui_auto_ev.cpp",
    ),
    "capture_harness": ReferenceGroup(
        key="capture_harness",
        scenes=["fullframe"],
        modes=[None],
        tmp_prefix="lumice_capture_harness_",
        ref_prefix="smoke_",
        source="test/gui/visual/test_gui_capture_smoke.cpp",
    ),
    # Lens-projection scene names — must match kScenes[] order in test_gui_lens_projection.cpp.
    # One scene per projection branch of the preview fragment shader; all four share the same
    # simulated frame, so a PSNR drop localizes to the projection math.
    "lens_proj": ReferenceGroup(
        key="lens_proj",
        scenes=[
            "fisheye_equal_area_120",
            "fisheye_orthographic_180",
            "dual_fisheye_equal_area_full",
            "rectangular",
        ],
        modes=[None],
        tmp_prefix="lumice_lens_proj_",
        ref_prefix="lens_proj_",
        source="test/gui/visual/test_gui_lens_projection.cpp",
    ),
}

STAGING_DIR = "/tmp/gui_refs_build"

# PSNR output pattern from lumice::test::CheckAgainstReference (test_screenshot.cpp).
# Group 1 is the reference-group tag, so Phase B can drop lines belonging to other groups
# when several of them print into the same stderr. "inf" is matched too: ComputePsnr
# returns infinity for a pixel-identical capture, which is the normal case for a
# deterministic scene (no simulation, no RNG) — dropping those lines would look like
# "no PSNR output parsed" instead.
_PSNR_RE = re.compile(r"\[(\w+)\]\s+(\S+):\s+PSNR=(inf|[0-9.]+)\s+dB")

# Threshold recorded when every calibration run was pixel-identical to the reference, so
# mean − 3σ has nothing finite to work with. Bit-exactness cannot be demanded — references
# are committed and compared on other machines — so this falls back to the value the repo's
# other deterministic GL comparisons already use (screenshot/left_panel_psnr,
# screenshot/crystal_psnr): high enough that a moved widget or changed color fails, loose
# enough to absorb encoder/driver noise.
DETERMINISTIC_FLOOR_DB = 40.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _selected_groups(args: argparse.Namespace) -> list[ReferenceGroup]:
    """[GROUPS[args.group]] when --group is given, else every registered group."""
    if getattr(args, "group", None):
        return [GROUPS[args.group]]
    return list(GROUPS.values())


def _scene_list(group: ReferenceGroup, args: argparse.Namespace) -> list[str]:
    """group.scenes, or [args.scene] when --scene is given."""
    return [args.scene] if getattr(args, "scene", None) else group.scenes


def _scene_key(scene: str, mode: str | None) -> str:
    """Scene identifier used in filenames, stderr tags and _thresholds.json."""
    return f"{scene}_{mode}" if mode else scene


def _gtest_filter(group: ReferenceGroup, args: argparse.Namespace) -> str:
    """Build the gui_test --filter value. Filter is substring-matched against
    each test's Name OR Category (see imgui_te_engine.cpp:PassFilter — not a
    glob, not gtest syntax). With --scene, use ^name$ to anchor an exact match
    on the scene name so 'rp46' does not also pick 'rp46_nof'."""
    if getattr(args, "scene", None):
        return f"^{args.scene}$"
    return group.key


def _run(binary: str, gtest_filter: str, extra_args: list[str], capture_stderr: bool = False) -> tuple[int, str]:
    cmd = [binary, "--filter", gtest_filter] + extra_args
    result = subprocess.run(cmd, capture_output=capture_stderr, text=True)
    return result.returncode, result.stderr if capture_stderr else ""


def _collect_pngs(group: ReferenceGroup, run_dir: str, scenes: list[str]) -> None:
    """Move /tmp/<tmp_prefix>*.png → run_dir/<key>.png for every (scene, mode)."""
    os.makedirs(run_dir, exist_ok=True)
    for scene in scenes:
        for mode in group.modes:
            key = _scene_key(scene, mode)
            src = f"/tmp/{group.tmp_prefix}{key}.png"
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


def _thresholds_path(refs_dir: str) -> str:
    return os.path.join(refs_dir, "_thresholds.json")


def _read_thresholds(json_path: str) -> dict:
    """Read _thresholds.json, rejecting the pre-group flat schema rather than
    silently treating its thresholds as absent (which would drop them on write).

    Called once up front from main() so a schema problem fails before the
    calibration runs, and again at write time to merge into the current contents.
    """
    if not os.path.exists(json_path):
        return {}
    try:
        with open(json_path) as fh:
            data = json.load(fh)
    except (json.JSONDecodeError, OSError) as exc:
        print(f"  WARNING: existing {json_path} unreadable ({exc}); starting fresh", file=sys.stderr)
        return {}
    if "scenes" in data and "groups" not in data:
        print(
            f"ERROR: {json_path} uses the pre-group flat schema (top-level 'scenes'). "
            "Nest it under groups.auto_ev before running this script, or its thresholds "
            "would be dropped by the next write.",
            file=sys.stderr,
        )
        sys.exit(1)
    return data


# ---------------------------------------------------------------------------
# Phase A — mean-ref generation
# ---------------------------------------------------------------------------


def phase_a_group(group: ReferenceGroup, args: argparse.Namespace) -> None:
    binary = args.binary
    n = args.n
    refs_dir = args.refs_dir
    quality = args.quality
    scenes = _scene_list(group, args)
    gtest_filter = _gtest_filter(group, args)

    print(f"[Phase A][{group.key}] Mean-ref generation: N={n} runs, JPEG quality={quality}")
    print(f"[Phase A][{group.key}] Binary : {binary}")
    print(f"[Phase A][{group.key}] Refs   : {refs_dir}")
    print(f"[Phase A][{group.key}] Scenes : {scenes} (gtest filter='{gtest_filter}')")

    # Clear staging dir for idempotent reruns
    staging = os.path.join(STAGING_DIR, group.key)
    if os.path.exists(staging):
        shutil.rmtree(staging)
    os.makedirs(staging)

    # Collect N independent runs
    for i in range(n):
        run_dir = os.path.join(staging, f"run_{i}")
        print(f"[Phase A][{group.key}] Run {i + 1}/{n}...", flush=True)
        rc, _ = _run(binary, gtest_filter, ["--keep-export-png"])
        if rc != 0:
            print(f"  WARNING: run {i} exited {rc}", file=sys.stderr)
        _collect_pngs(group, run_dir, scenes)

    # Per (scene, mode): pixel-average → apply format silence rule → save reference
    print()
    updated = 0
    for scene in scenes:
        for mode in group.modes:
            key = _scene_key(scene, mode)

            frames: list[np.ndarray] = []
            for i in range(n):
                p = os.path.join(staging, f"run_{i}", f"{key}.png")
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

                ref_base = os.path.join(refs_dir, f"{group.ref_prefix}{key}")
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
                        f"{group.source} from .jpg to .png",
                        file=sys.stderr,
                    )
            finally:
                os.unlink(tmp_jpg)
                os.unlink(tmp_png)
            updated += 1

    print(f"\n[Phase A][{group.key}] Done — {updated} references updated in {refs_dir}")


def phase_a(args: argparse.Namespace) -> None:
    for group in _selected_groups(args):
        phase_a_group(group, args)


# ---------------------------------------------------------------------------
# Phase B — threshold calibration
# ---------------------------------------------------------------------------


def phase_b_group(group: ReferenceGroup, args: argparse.Namespace) -> None:
    binary = args.binary
    n_calib = args.n_calib
    refs_dir = args.refs_dir
    gtest_filter = _gtest_filter(group, args)

    print(f"[Phase B][{group.key}] Threshold calibration: N_calib={n_calib} runs")
    print(f"[Phase B][{group.key}] Binary : {binary}")
    print(f"[Phase B][{group.key}] Filter : {gtest_filter}")

    psnr_data: dict[str, list[float]] = {}
    for i in range(n_calib):
        print(f"[Phase B][{group.key}] Calibration run {i + 1}/{n_calib}...", flush=True)
        _, stderr = _run(binary, gtest_filter, [], capture_stderr=True)
        for m in _PSNR_RE.finditer(stderr):
            # Drop PSNR lines emitted by other reference groups: a substring --filter can
            # legitimately select more than this group's tests.
            if m.group(1) != group.key:
                continue
            tag, val = m.group(2), float(m.group(3))
            psnr_data.setdefault(tag, []).append(val)

    if not psnr_data:
        print(
            f"ERROR: no PSNR output parsed for group '{group.key}' — check that references "
            "exist and tests pass",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"\n[Phase B][{group.key}] Recommendations (mean − 3σ, floor to 0.5 dB precision):")
    scenes_out: dict[str, dict] = {}
    for tag in sorted(psnr_data):
        samples = psnr_data[tag]
        identical = sum(1 for v in samples if math.isinf(v))
        finite = [v for v in samples if math.isfinite(v)]
        if not finite:
            # Every run matched the reference exactly. Report the deterministic floor rather
            # than a statistic computed from nothing.
            print(f"  {tag}: {identical}/{len(samples)} runs pixel-identical → "
                  f"threshold={DETERMINISTIC_FLOOR_DB:.1f} dB (deterministic floor)")
            scenes_out[tag] = {
                "psnr_mean": None,
                "psnr_std": None,
                "identical_runs": identical,
                "n_samples": len(samples),
                "threshold": DETERMINISTIC_FLOOR_DB,
            }
            continue
        # Identical runs are excluded from the statistics: an infinite sample cannot enter a
        # mean, and leaving it out can only lower the threshold, which is the safe direction.
        vals = np.array(finite)
        mean = float(vals.mean())
        std = float(vals.std(ddof=0))
        threshold = math.floor((mean - 3 * std) * 2) / 2
        suffix = f"  ({identical}/{len(samples)} runs pixel-identical, excluded)" if identical else ""
        print(f"  {tag}: mean={mean:.2f} dB  std={std:.4f} dB  threshold={threshold:.1f} dB{suffix}")
        scenes_out[tag] = {
            "psnr_mean": round(mean, 4),
            "psnr_std": round(std, 4),
            "identical_runs": identical,
            "n_samples": len(samples),
            "threshold": threshold,
        }

    # Merge into existing thresholds.json at two levels: scenes this run did not touch keep
    # their audit history (e.g. --scene overlay_ea must not wipe the 9 other auto_ev scenes),
    # and groups this run did not touch are left untouched entirely — including their
    # generated_at, which is per group precisely so a capture_harness run cannot restamp
    # auto_ev's calibration date.
    json_path = _thresholds_path(refs_dir)
    merged = _read_thresholds(json_path)
    groups_out = merged.get("groups") if isinstance(merged.get("groups"), dict) else {}
    group_entry = groups_out.get(group.key) if isinstance(groups_out.get(group.key), dict) else {}
    existing_scenes = group_entry.get("scenes") if isinstance(group_entry.get("scenes"), dict) else {}
    existing_scenes.update(scenes_out)
    groups_out[group.key] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "n_ref_runs": getattr(args, "n", 10),
        "n_calib_runs": n_calib,
        "scenes": existing_scenes,
    }
    merged["groups"] = groups_out
    with open(json_path, "w") as fh:
        json.dump(merged, fh, indent=2)
        fh.write("\n")
    print(f"\n[Phase B][{group.key}] Thresholds written to {json_path} (scenes={len(existing_scenes)})")
    print(f"[Phase B][{group.key}] Copy 'threshold' values into {group.source}")


def phase_b(args: argparse.Namespace) -> None:
    for group in _selected_groups(args):
        phase_b_group(group, args)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Regenerate GUI mean-ref images and calibrate PSNR thresholds."
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
    parser.add_argument(
        "--group",
        default=None,
        choices=list(GROUPS),
        help=(
            "Reference group to regenerate/calibrate. Default: every registered group "
            f"({', '.join(GROUPS)}), each with its own filter, references and "
            "_thresholds.json entry."
        ),
    )
    parser.add_argument(
        "--scene",
        default=None,
        help=(
            "If set, only regenerate refs / calibrate thresholds for this single scene of "
            "--group (which is then required, since scene names are only unique per group). "
            "Phase A overwrites only this scene's reference image; Phase B updates only "
            "this scene's entry in _thresholds.json (other scenes preserved via merge-write). "
            "Default: run all scenes of every selected group."
        ),
    )

    args = parser.parse_args()

    if args.phase_a_only and args.phase_b_only:
        print("ERROR: --phase-a-only and --phase-b-only are mutually exclusive", file=sys.stderr)
        sys.exit(1)
    if args.scene and not args.group:
        print("ERROR: --scene requires --group (scene names are only unique within a group)", file=sys.stderr)
        sys.exit(1)
    if args.scene and args.scene not in GROUPS[args.group].scenes:
        print(
            f"ERROR: unknown scene '{args.scene}' for group '{args.group}'; "
            f"choose from {GROUPS[args.group].scenes}",
            file=sys.stderr,
        )
        sys.exit(1)
    if not os.path.isfile(args.binary):
        print(f"ERROR: binary not found: {args.binary}", file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(args.refs_dir):
        print(f"ERROR: refs-dir not found: {args.refs_dir}", file=sys.stderr)
        sys.exit(1)
    # Fail on an unusable thresholds file now rather than after the calibration runs.
    _read_thresholds(_thresholds_path(args.refs_dir))

    if args.phase_b_only:
        phase_b(args)
    elif args.phase_a_only:
        phase_a(args)
    else:
        phase_a(args)
        phase_b(args)


if __name__ == "__main__":
    main()
