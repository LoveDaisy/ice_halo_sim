#!/usr/bin/env python3
"""
Reference generation and threshold calibration driver for GUI visual-regression tests.

Phase A: Run gui_test N times with --keep-export-png, pixel-average the
  captures per (scene, mode), and save the mean image as the new reference.
Phase B: Run gui_test N_calib times, parse PSNR output from stderr, and
  compute per-scene threshold recommendations
  (mean - max(SIGMA_MARGIN*sigma, MIN_MARGIN_DB), floor to 0.5 dB).

Both phases are driven by the GROUPS registry below: a reference group names the
gui_test category it tags its output with, its scenes/modes, and the tmp/reference
filename prefixes. Adding a visual-regression suite means adding a GROUPS entry — the
Phase A/B algorithms themselves stay untouched.

Every run of both phases is a full gui_test run under the same invocation
scripts/build.sh uses for its correctness pool (see SUITE_ARGS), because that is the
condition the committed thresholds have to hold under; groups are separated by the
"[<group>]" tag their comparisons print, not by narrowing what runs.

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
                also the _thresholds.json group name. Must be unique across groups:
                PSNR lines are attributed by an exact match on this tag, so two groups
                sharing one would have their samples pooled into a single threshold.
    scenes      scene names, matching the test registration order in the group's source.
    modes       per-scene variant suffixes; [None] for groups without variants.
    tmp_prefix  the test writes its capture as <tmp_prefix><key>.png. The DIRECTORY is not
                part of this contract: the driver passes gui_test --export-dir and collects
                from there, so only the filename is shared with the test source.
    ref_prefix  the reference lives at <refs_dir>/<ref_prefix><key>.jpg.
    source      the group's test source, quoted in the "copy thresholds back" hint.
    """

    key: str
    scenes: list[str]
    modes: list[str | None]
    tmp_prefix: str
    ref_prefix: str
    source: str


GROUPS: dict[str, ReferenceGroup] = {
    "capture_harness": ReferenceGroup(
        key="capture_harness",
        scenes=["fullframe"],
        modes=[None],
        tmp_prefix="lumice_capture_harness_",
        ref_prefix="smoke_",
        source="test/gui/visual/test_gui_capture_smoke.cpp",
    ),
    # Lens-projection scene names — must match kScenes[] order in test_gui_lens_projection.cpp.
    # One scene per projection branch of the preview fragment shader, plus overlay_ea, which
    # reuses the equal-area branch to cover the marker/grid overlay stage instead. All share
    # the same simulated frame, so a PSNR drop localizes to the projection math.
    "lens_proj": ReferenceGroup(
        key="lens_proj",
        scenes=[
            "fisheye_equal_area_120",
            "fisheye_orthographic_180",
            "linear",
            "dual_fisheye_equal_area_full",
            "rectangular",
            "overlay_ea",
        ],
        modes=[None],
        tmp_prefix="lumice_lens_proj_",
        ref_prefix="lens_proj_",
        source="test/gui/visual/test_gui_lens_projection.cpp",
    ),
    # Defaults-panel layout scene names — must match kScenes[] order in
    # test/gui/visual/test_gui_defaults_panel.cpp. Each scene is one state of the "Save Current as
    # Defaults" modal (pending changes / expanded read-only section / filtered / nothing to adopt),
    # captured as the modal's own on-screen rectangle.
    "defaults_panel_layout": ReferenceGroup(
        key="defaults_panel_layout",
        scenes=[
            "pending_changes",
            "other_expanded",
            "filtered",
            "no_changes",
            "presets_expanded",
            "presets_warning",
        ],
        modes=[None],
        tmp_prefix="lumice_defaults_panel_",
        ref_prefix="defaults_panel_",
        source="test/gui/visual/test_gui_defaults_panel.cpp",
    ),
    # Edit-modal layout scene names — must match kScenes[] order in test_gui_modal_layout.cpp.
    # Each scene is one (tab, crystal type, H/V layout) combination of the unified edit popup,
    # captured as the modal's own on-screen rectangle.
    "modal_layout": ReferenceGroup(
        key="modal_layout",
        scenes=[
            "crystal_prism",
            "crystal_pyramid",
            "filter_raypath",
            "filter_ee",
        ],
        modes=[None],
        tmp_prefix="lumice_modal_layout_",
        ref_prefix="modal_layout_",
        source="test/gui/visual/test_gui_modal_layout.cpp",
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
# mean − kσ has nothing finite to work with. Bit-exactness cannot be demanded — references
# are committed and compared on other machines — so this falls back to the value the repo's
# other deterministic GL comparisons already use (screenshot/left_panel_psnr,
# screenshot/crystal_psnr): high enough that a moved widget or changed color fails, loose
# enough to absorb encoder/driver noise.
DETERMINISTIC_FLOOR_DB = 40.0

# Sigma margin behind each recommended threshold. 4, not 3: every threshold this repo
# actually ships is mean − 4σ (reproduce it from the psnr_mean/psnr_std recorded in
# _thresholds.json — six of the ten scenes in the since-retired auto_ev group discriminated
# between 3σ and 4σ, and all six matched 4σ), so 3σ here would have been a recommendation
# nobody adopted. It is also the margin the observed tails need: three sigma left the
# lens_proj dual-fisheye threshold 0.08 dB under the lowest sample of an independent 20-run
# batch.
SIGMA_MARGIN = 4.0

# Floor on how tight a threshold may get, in dB below the sampled mean. Four sigma alone is
# only a defence against run-to-run noise on THIS machine; a scene whose sigma nearly
# vanishes (the ray-gated lens_proj scenes calibrate at sigma ~0.06 dB) would otherwise get a
# threshold 0.2 dB under its own mean, which a different GPU or driver could cross on its
# own. The size comes from this repo's own history: regenerating auto_ev against a changed
# orientation sampler moved its PSNRs 0.3-0.8 dB, so a margin under 1 dB does not survive
# legitimate upstream change. This is the same reasoning as DETERMINISTIC_FLOOR_DB above,
# applied to scenes that are merely quiet rather than pixel-identical. Since the auto_ev group
# (whose 4σ ran 0.84 dB and up) was retired, it is no longer the tie-breaker it was: every
# remaining stochastic scene calibrates at 4σ well under 1 dB, so this floor — not the sigma
# margin — is what actually sets all six lens_proj thresholds. Read a shipped threshold as
# "mean − 1.0 dB, rounded down to 0.5 dB" and check SIGMA_MARGIN only if a sigma ever exceeds
# 0.25 dB.
MIN_MARGIN_DB = 1.0


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


# Every run — reference generation AND calibration — invokes gui_test exactly the way
# scripts/build.sh runs its correctness pool: the whole suite minus the real-timing tests,
# with --fixed-dt. Sampling a group in isolation (`--filter <key>`) is measurably optimistic,
# because each test then gets the machine to itself and its Monte-Carlo frame accumulates
# less noise: across the lens_proj group, isolated runs came out 0.34–0.62 dB above
# full-suite runs, enough that an isolated threshold sat ABOVE full-suite samples
# actually observed. auto_ev learned the same lesson the expensive way (its references
# predate this and had to be regenerated from full-suite runs after a 31% run-level flake).
# So the condition is not a knob: thresholds must be calibrated under the condition the
# assertions run in, and there is no way to ask this script for anything else.
# A standalone named constant (rather than embedding the string as the 3rd element of
# SUITE_ARGS) so scripts/check_policies.py's gui-test-suite-args-sync check can extract it by
# name via a plain text/regex read, matching how it reads scripts/build.sh — instead of having
# to import this module (and its numpy/PIL dependency graph) just to reach one string.
SUITE_FILTER_EXPR = (
    "-perf_test,-save_open_visual_consistency,-revert_repushes_server_display_state,"
    "-zorder_priority_persists_across_rerun,-gpu_color_class_overflow"
)
# --no-user-config trails SUITE_FILTER_EXPR, matching scripts/build.sh. A reference image is the
# one artifact where reading the generating machine's personal defaults would be permanent: the
# contamination ships in the committed .jpg and every other machine inherits it as a threshold miss.
SUITE_ARGS = ["--fixed-dt", "--filter", SUITE_FILTER_EXPR, "--no-user-config"]


def _run(binary: str, extra_args: list[str], capture_stderr: bool = False) -> tuple[int, str]:
    cmd = [binary] + SUITE_ARGS + extra_args
    result = subprocess.run(cmd, capture_output=capture_stderr, text=True)
    return result.returncode, result.stderr if capture_stderr else ""


def _collect_pngs(group: ReferenceGroup, run_dir: str, scenes: list[str], export_dir: str) -> None:
    """Move export_dir/<tmp_prefix>*.png → run_dir/<key>.png for every (scene, mode).

    export_dir is the directory this driver told gui_test to write to (--export-dir), not a
    location reconstructed here. gui_test's own default is a per-process temp subdirectory it
    picks itself, which a collector could not predict — and should not have to.
    """
    os.makedirs(run_dir, exist_ok=True)
    for scene in scenes:
        for mode in group.modes:
            key = _scene_key(scene, mode)
            src = os.path.join(export_dir, f"{group.tmp_prefix}{key}.png")
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
            "Nest it under groups.<key> for the group it belonged to before running this "
            "script, or its thresholds "
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

    print(f"[Phase A][{group.key}] Mean-ref generation: N={n} runs, JPEG quality={quality}")
    print(f"[Phase A][{group.key}] Binary : {binary}")
    print(f"[Phase A][{group.key}] Refs   : {refs_dir}")
    print(f"[Phase A][{group.key}] Scenes : {scenes} (full-suite runs; --scene selects which to average)")

    # Clear staging dir for idempotent reruns
    staging = os.path.join(STAGING_DIR, group.key)
    if os.path.exists(staging):
        shutil.rmtree(staging)
    os.makedirs(staging)

    # Collect N independent runs
    for i in range(n):
        run_dir = os.path.join(staging, f"run_{i}")
        print(f"[Phase A][{group.key}] Run {i + 1}/{n}...", flush=True)
        export_dir = os.path.join(staging, f"export_{i}")
        os.makedirs(export_dir, exist_ok=True)
        rc, _ = _run(binary, ["--keep-export-png", "--export-dir", export_dir])
        if rc != 0:
            print(f"  WARNING: run {i} exited {rc}", file=sys.stderr)
        _collect_pngs(group, run_dir, scenes, export_dir)

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
    # Every group prints into the same full-suite stderr, and --scene must leave the group's
    # other scenes' entries untouched, so sampling is restricted to the selected keys.
    wanted = {_scene_key(scene, mode) for scene in _scene_list(group, args) for mode in group.modes}

    print(f"[Phase B][{group.key}] Threshold calibration: N_calib={n_calib} runs")
    print(f"[Phase B][{group.key}] Binary : {binary}")
    print(f"[Phase B][{group.key}] Scenes : {sorted(wanted)}")

    psnr_data: dict[str, list[float]] = {}
    for i in range(n_calib):
        print(f"[Phase B][{group.key}] Calibration run {i + 1}/{n_calib}...", flush=True)
        _, stderr = _run(binary, [], capture_stderr=True)
        for m in _PSNR_RE.finditer(stderr):
            # Drop PSNR lines belonging to another group, or to a scene this run did not select.
            if m.group(1) != group.key or m.group(2) not in wanted:
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

    # A tag short of n_calib samples means some calibration run's stderr never printed its PSNR
    # line (e.g. a timing/watchdog early-exit) — surface that immediately rather than leaving it
    # discoverable only by cross-checking n_samples in _thresholds.json after the fact.
    for tag in sorted(wanted - psnr_data.keys()):
        print(
            f"WARNING: [Phase B][{group.key}] tag '{tag}' produced 0/{n_calib} PSNR samples "
            "(no calibration run printed it)",
            file=sys.stderr,
        )

    print(
        f"\n[Phase B][{group.key}] Recommendations "
        f"(mean − max({SIGMA_MARGIN:.0f}σ, {MIN_MARGIN_DB:.1f} dB), floor to 0.5 dB precision):"
    )
    scenes_out: dict[str, dict] = {}
    for tag in sorted(psnr_data):
        samples = psnr_data[tag]
        if len(samples) < n_calib:
            print(
                f"WARNING: [Phase B][{group.key}] tag '{tag}' produced only {len(samples)}/{n_calib} "
                "PSNR samples — the resulting mean/std/threshold are computed from fewer runs than "
                "declared in n_calib_runs",
                file=sys.stderr,
            )
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
        threshold = math.floor((mean - max(SIGMA_MARGIN * std, MIN_MARGIN_DB)) * 2) / 2
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
    # their audit history (e.g. --scene overlay_ea must not wipe the other lens_proj scenes),
    # and groups this run did not touch are left untouched entirely — including their
    # generated_at, which is per group precisely so a capture_harness run cannot restamp
    # lens_proj's calibration date.
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
        default="build/Release/static/bin/gui_test",
        help="Path to gui_test binary (default: build/Release/static/bin/gui_test)",
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
            "Selects what is averaged/calibrated, not what gui_test runs — every run is a "
            "full-suite run (see SUITE_ARGS). Default: all scenes of every selected group."
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
