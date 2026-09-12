#!/usr/bin/env python3
"""
Reference generation and threshold calibration driver for GUI visual-regression tests.

Phase A: Run gui_test N times with --keep-export-png, pixel-average the
  captures per (scene, mode), and save the mean image as the new reference.
Phase B: Run gui_test N_calib times, parse PSNR output from stderr, and
  compute per-scene threshold recommendations
  (mean - max(SIGMA_MARGIN*sigma, MIN_MARGIN_DB), floor to 0.5 dB).
  For a scene the binary compares under the pixel ruler (its stderr carries an
  "n_diff=... maxcc=..." line), Phase B also records that ruler's tau and the largest
  maxcc it saw on THIS machine (maxcc_tau / maxcc_local_max) — an audit that the
  reference machine still matches its own references byte-for-byte, not a threshold
  recommendation; the cross-machine K lives in the test source (see _MAXCC_RE).

Both phases are driven by the GROUPS registry below: a reference group names the
gui_test category it tags its output with, its scenes/modes, the tmp/reference
filename prefixes, and whether it is deterministic. Adding a visual-regression suite
means adding a GROUPS entry — the Phase A/B algorithms themselves stay untouched.

Every run of both phases is a full gui_test run under the same invocation
scripts/build.sh uses for its correctness pool (see SUITE_ARGS), because that is the
condition the committed thresholds have to hold under; groups are separated by the
"[<group>]" tag their comparisons print, not by narrowing what runs. Because every such
run exports every group's captures and prints every group's PSNR lines, one invocation
covering several groups runs max(n) full suites, not sum(n), and deals each run out to
every group that still wants a sample. N itself is per group: a deterministic group
(no simulation, no RNG in any scene) defaults to a single run per phase, since a second
run of the same frame is no information; a stochastic one to STOCHASTIC_RUNS.

Phase A is also gated on origin/main: if main has already reshot a reference this run
would write, the run refuses (that reference would be discarded at rebase time) unless
--allow-stale-base is passed. See check_reference_base.

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
    deterministic
                True when every scene's frame carries no simulation and no RNG, so one run
                is the whole population: the reference is that single capture, Phase B's
                statistics have nothing to estimate, and the group is held to the pixel
                ruler in its test source rather than to a PSNR floor. This flag sets the
                run-count DEFAULT (1/1 instead of STOCHASTIC_RUNS) — nothing else. Repeating
                a full-suite run on a deterministic group is zero information at ~90 s per
                run, which is how one two-group reshoot came to cost 64 minutes; a
                registration must declare it, since the alternative is the driver guessing
                from _thresholds.json's identical_runs after the fact, on a group that may
                have no entry yet. It is not a correctness claim the driver enforces:
                Phase B's maxcc_local_max audit is what catches a "deterministic" scene
                that has stopped being one.
    """

    key: str
    scenes: list[str]
    modes: list[str | None]
    tmp_prefix: str
    ref_prefix: str
    source: str
    deterministic: bool


# Run counts a stochastic group defaults to, for Phase A (mean of N frames) and Phase B
# (N_calib PSNR samples behind mean − max(4σ, 1 dB)). A deterministic group defaults to 1/1
# and takes these only when --n / --n-calib are passed explicitly, which applies the same count
# to every selected group.
STOCHASTIC_RUNS = 10

GROUPS: dict[str, ReferenceGroup] = {
    "capture_harness": ReferenceGroup(
        key="capture_harness",
        scenes=["fullframe"],
        modes=[None],
        tmp_prefix="lumice_capture_harness_",
        ref_prefix="smoke_",
        source="test/gui/visual/test_gui_capture_smoke.cpp",
        deterministic=True,
    ),
    # Lens-projection scene names — must match kScenes[] order in test_gui_lens_projection.cpp.
    # One scene per projection branch of the preview fragment shader, plus overlay_ea, which
    # reuses the equal-area branch to cover the marker/grid overlay stage instead, and the two
    # *_border scenes, which reuse a projection branch to cover the lens-border overlay stage. All
    # share the same simulated frame, so a PSNR drop localizes to the projection math.
    #
    # A scene added to kScenes[] but NOT listed here is silently left without a reference: the
    # driver's Phase A averages only the scenes it is told about, so the new scene's own gui_test
    # case keeps failing against a file that was never written, with the regen run reporting
    # success. Adding a scene means editing both lists.
    "lens_proj": ReferenceGroup(
        key="lens_proj",
        scenes=[
            "fisheye_equal_area_120",
            "fisheye_orthographic_180",
            "linear",
            "dual_fisheye_equal_area_full",
            "rectangular",
            "overlay_ea",
            "fisheye_equal_area_120_border",
            "dual_fisheye_equal_area_full_border",
            "sky_colour_ea_180",
        ],
        modes=[None],
        tmp_prefix="lumice_lens_proj_",
        ref_prefix="lens_proj_",
        source="test/gui/visual/test_gui_lens_projection.cpp",
        deterministic=False,
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
            "wedge_presets",
        ],
        modes=[None],
        tmp_prefix="lumice_defaults_panel_",
        ref_prefix="defaults_panel_",
        source="test/gui/visual/test_gui_defaults_panel.cpp",
        deterministic=True,
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
        deterministic=True,
    ),
}

# NOT registered here, and won't be reshot by any command in this file: the "visual" category in
# test/gui/visual/test_preview_pixels.cpp (crystal_preview_prism/pyramid/wireframe/shaded,
# left_panel). It compares under the pixel ruler at tau=0, K=0 (a compile-time constant, not a
# Phase B statistic), and its reference filenames (crystal_prism_default.png,
# left_panel_default.png, ...) do not follow the <ref_prefix><scene> convention every
# ReferenceGroup above assumes — registering it would mean renaming those files and editing the
# hardcoded ref-path strings in test_preview_pixels.cpp for a Phase A/B calibration this category
# has no use for (deterministic scenes have nothing to average or calibrate). After a theme/layout
# change, reshoot these by hand: one full-suite gui_test run (SUITE_ARGS below) with
# --keep-export-png --export-dir, then copy the changed lumice_<scene>.png exports over the
# matching test/gui/references/*.png as-is (PNG, never JPEG: the ruler demands byte-identity, and
# JPEG quantisation alone reads as thousands of differing pixels).
STAGING_DIR = "/tmp/gui_refs_build"

# PSNR output pattern from lumice::test::CheckAgainstReference (test_screenshot.cpp).
# Group 1 is the reference-group tag, so Phase B can drop lines belonging to other groups
# when several of them print into the same stderr. "inf" is matched too: ComputePsnr
# returns infinity for a pixel-identical capture, which is the normal case for a
# deterministic scene (no simulation, no RNG) — dropping those lines would look like
# "no PSNR output parsed" instead.
_PSNR_RE = re.compile(r"\[(\w+)\]\s+(\S+):\s+PSNR=(inf|[0-9.]+)\s+dB")

# Pixel-ruler output pattern from the same function, printed right after the PSNR line for a
# scene compared under lumice::test::MaxCcRuler (test/support/pixel_diff_metrics.hpp). Phase B
# records tau and the largest maxcc seen per scene as an audit trail only. It does NOT derive a
# threshold from them: every deterministic group's K is a cross-machine number, measured from the
# CI llvmpipe leg's captures against Metal-shot references (doc/testing-architecture.md §4.6), and
# this driver only ever runs on the reference machine, where the honest reading is maxcc = 0.
# A non-zero maxcc_local_max here therefore means the scene is no longer deterministic on the
# machine that shot its reference, which is the thing to investigate before re-shooting.
_MAXCC_RE = re.compile(r"\[(\w+)\]\s+(\S+):\s+n_diff=(\d+)\s+maxcc=(\d+)\s+dmax=(\d+)\s+\(tau=(\d+),\s+K=(\d+)\)")

# Threshold recorded when every calibration run was pixel-identical to the reference, so
# mean − kσ has nothing finite to work with. Kept as the historical PSNR figure for such a
# scene's `threshold` field, but it is no longer what any deterministic group is held to: those
# groups compare under the pixel ruler (see _MAXCC_RE), because this floor measurably passed a
# 127-px text-row drift at 42.6 dB and a 90-px scrollbar-thumb drift at 53.45 dB. A stochastic
# scene that happens to come out identical on every calibration run would still receive it.
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


def _resolve_runs(group: ReferenceGroup, explicit: int | None) -> int:
    """Run count for one group: the explicit --n / --n-calib value when given, else the group's
    default — 1 for a deterministic group, STOCHASTIC_RUNS otherwise. An explicit value is the
    caller's statement that they want that many samples from every selected group, so it is
    not clamped for deterministic ones (that is how a suspected non-determinism gets measured).
    """
    if explicit is not None:
        if explicit < 1:
            print(f"ERROR: run count must be >= 1, got {explicit}", file=sys.stderr)
            sys.exit(1)
        return explicit
    return 1 if group.deterministic else STOCHASTIC_RUNS


def _run_plan(groups: list[ReferenceGroup], explicit: int | None) -> dict[str, int]:
    """Per-group run counts for one phase. The phase runs max() of these full-suite runs and
    hands every run's output to each group that still wants a sample, so several groups in
    one invocation cost max(n), not sum(n): each full-suite run already exports every group's
    captures and prints every group's PSNR lines, and it was only ever the driver that threw
    the other groups' share of each run away.
    """
    return {g.key: _resolve_runs(g, explicit) for g in groups}


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
    "-zorder_priority_persists_across_rerun,-gpu_color_class_overflow,"
    "-run_after_analysis_renders_gpu"
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
# Stale-base gate — refuse to reshoot references origin/main has already reshot
# ---------------------------------------------------------------------------


def _git(args: list[str]) -> tuple[int, str, str]:
    result = subprocess.run(["git"] + args, capture_output=True, text=True)
    return result.returncode, result.stdout, result.stderr


def _planned_reference_paths(groups: list[ReferenceGroup], args: argparse.Namespace) -> set[str]:
    """Every reference file Phase A may write in this invocation, as paths relative to the
    current directory (both extensions: the format silence rule decides per scene)."""
    out: set[str] = set()
    for group in groups:
        for scene in _scene_list(group, args):
            for mode in group.modes:
                base = os.path.join(args.refs_dir, f"{group.ref_prefix}{_scene_key(scene, mode)}")
                out.add(os.path.normpath(base + ".jpg"))
                out.add(os.path.normpath(base + ".png"))
    return out


def check_reference_base(groups: list[ReferenceGroup], args: argparse.Namespace) -> None:
    """Exit non-zero if origin/main has changed any reference this run is about to write.

    A reference reshot on a branch whose base is behind origin/main on that same file is work
    that gets thrown away: at rebase time the branch's copy and main's copy conflict, and the
    only sane resolution is to take main's and reshoot again on top of it. That is also the
    order the gate asks for — rebase first, then shoot. The comparison is HEAD...origin/main
    restricted to the reference directory, i.e. what main changed since the merge base; the
    branch's own reference changes do not matter here (they are the thing being redone).

    Scoped to the planned image files, deliberately not to _thresholds.json: main rewrites that
    file's group entry on every calibration of any group, so including it would fire this gate
    for a lens_proj reshoot because main recalibrated modal_layout.

    Compares against the LOCAL origin/main ref and does not fetch: a fetch is a network side
    effect this script should not make on the caller's behalf, so the caller runs `git fetch`
    when currency matters. No origin/main at all (no remote, exported tree) skips the gate with
    a note; any other git failure stops the run, since a gate that fails open on an error is
    not a gate. --allow-stale-base is the explicit override for both.
    """
    if args.allow_stale_base:
        print("[base] --allow-stale-base: skipping the origin/main reference check")
        return
    rc, _, _ = _git(["rev-parse", "--verify", "-q", "origin/main"])
    if rc != 0:
        print("[base] origin/main not found (no remote?) — skipping the stale-base check")
        return
    rc, out, err = _git(["diff", "--name-only", "--relative", "HEAD...origin/main", "--", args.refs_dir])
    if rc != 0:
        print(
            f"ERROR: git diff HEAD...origin/main -- {args.refs_dir} failed: {err.strip()}\n"
            "Resolve that (a refs dir outside this checkout, or no merge base), or pass "
            "--allow-stale-base to shoot anyway.",
            file=sys.stderr,
        )
        sys.exit(1)
    changed_on_main = {os.path.normpath(line) for line in out.splitlines() if line.strip()}
    hits = sorted(_planned_reference_paths(groups, args) & changed_on_main)
    if not hits:
        print("[base] origin/main has not touched any reference this run would write")
        return
    print(
        "ERROR: origin/main has already reshot reference(s) this run would write:\n"
        + "".join(f"  {path}\n" for path in hits)
        + "A reference shot here would be discarded at rebase time in favour of main's copy.\n"
        "Rebase this branch onto origin/main first (git fetch && git rebase origin/main),\n"
        "then reshoot on top of it. Pass --allow-stale-base to override.",
        file=sys.stderr,
    )
    sys.exit(1)


# ---------------------------------------------------------------------------
# Phase A — mean-ref generation
# ---------------------------------------------------------------------------


def _phase_a_average_group(group: ReferenceGroup, n: int, args: argparse.Namespace) -> None:
    """Per (scene, mode): pixel-average the group's n collected frames, apply the format
    silence rule, save the reference. n == 1 is the deterministic case and is not special-cased:
    the mean of one frame is that frame, and the rms terms below evaluate to 0 for it."""
    refs_dir = args.refs_dir
    quality = args.quality
    scenes = _scene_list(group, args)
    staging = os.path.join(STAGING_DIR, group.key)

    print(f"\n[Phase A][{group.key}] Averaging N={n} run(s), JPEG quality={quality}")
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

    print(f"[Phase A][{group.key}] Done — {updated} references updated in {refs_dir}")


def phase_a(args: argparse.Namespace) -> None:
    groups = _selected_groups(args)
    plan = _run_plan(groups, args.n)
    n_max = max(plan.values())
    binary = args.binary

    print(f"[Phase A] Mean-ref generation: {n_max} full-suite run(s) shared by {len(groups)} group(s)")
    print(f"[Phase A] Binary : {binary}")
    print(f"[Phase A] Refs   : {args.refs_dir}")
    for group in groups:
        kind = "deterministic" if group.deterministic else "stochastic"
        print(f"[Phase A][{group.key}] N={plan[group.key]} ({kind}); "
              f"scenes {_scene_list(group, args)} (--scene selects which to average)")

    # Clear staging for idempotent reruns: each selected group's collected frames, and the
    # shared export directories every run writes into before the frames are dealt out.
    for group in groups:
        staging = os.path.join(STAGING_DIR, group.key)
        if os.path.exists(staging):
            shutil.rmtree(staging)
        os.makedirs(staging)
    for i in range(n_max):
        export_dir = os.path.join(STAGING_DIR, f"export_{i}")
        if os.path.exists(export_dir):
            shutil.rmtree(export_dir)
        os.makedirs(export_dir)

    # n_max independent runs; run i is dealt to every group whose plan still wants sample i.
    for i in range(n_max):
        print(f"[Phase A] Run {i + 1}/{n_max}...", flush=True)
        export_dir = os.path.join(STAGING_DIR, f"export_{i}")
        rc, _ = _run(binary, ["--keep-export-png", "--export-dir", export_dir])
        if rc != 0:
            print(f"  WARNING: run {i} exited {rc}", file=sys.stderr)
        for group in groups:
            if i < plan[group.key]:
                run_dir = os.path.join(STAGING_DIR, group.key, f"run_{i}")
                _collect_pngs(group, run_dir, _scene_list(group, args), export_dir)

    for group in groups:
        _phase_a_average_group(group, plan[group.key], args)


# ---------------------------------------------------------------------------
# Phase B — threshold calibration
# ---------------------------------------------------------------------------


@dataclass
class _GroupSamples:
    """What one group harvested from the calibration runs dealt to it."""

    wanted: set[str]
    psnr: dict[str, list[float]]
    # tag -> (tau, [maxcc per run]); only scenes the binary compares under the pixel ruler.
    maxcc: dict[str, tuple[int, list[int]]]


def _phase_b_ingest(group: ReferenceGroup, samples: _GroupSamples, stderr: str) -> None:
    """Pull this group's PSNR / pixel-ruler lines out of one full-suite stderr."""
    for m in _PSNR_RE.finditer(stderr):
        # Drop PSNR lines belonging to another group, or to a scene this run did not select.
        if m.group(1) != group.key or m.group(2) not in samples.wanted:
            continue
        tag, val = m.group(2), float(m.group(3))
        samples.psnr.setdefault(tag, []).append(val)
    for m in _MAXCC_RE.finditer(stderr):
        if m.group(1) != group.key or m.group(2) not in samples.wanted:
            continue
        tag, maxcc, tau = m.group(2), int(m.group(4)), int(m.group(6))
        prev_tau, per_run = samples.maxcc.setdefault(tag, (tau, []))
        if prev_tau != tau:
            print(
                f"ERROR: [Phase B][{group.key}] tag '{tag}' printed tau={tau} after tau={prev_tau} "
                "— one scene, one ruler",
                file=sys.stderr,
            )
            sys.exit(1)
        per_run.append(maxcc)


def _phase_b_write_group(
    group: ReferenceGroup, samples: _GroupSamples, n_calib: int, n_ref: int, args: argparse.Namespace
) -> None:
    refs_dir = args.refs_dir
    wanted = samples.wanted
    psnr_data = samples.psnr
    maxcc_data = samples.maxcc

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
        vals_all = psnr_data[tag]
        if len(vals_all) < n_calib:
            print(
                f"WARNING: [Phase B][{group.key}] tag '{tag}' produced only {len(vals_all)}/{n_calib} "
                "PSNR samples — the resulting mean/std/threshold are computed from fewer runs than "
                "declared in n_calib_runs",
                file=sys.stderr,
            )
        identical = sum(1 for v in vals_all if math.isinf(v))
        finite = [v for v in vals_all if math.isfinite(v)]
        if not finite:
            # Every run matched the reference exactly. Report the deterministic floor rather
            # than a statistic computed from nothing.
            print(f"  {tag}: {identical}/{len(vals_all)} runs pixel-identical → "
                  f"threshold={DETERMINISTIC_FLOOR_DB:.1f} dB (deterministic floor)")
            scenes_out[tag] = {
                "psnr_mean": None,
                "psnr_std": None,
                "identical_runs": identical,
                "n_samples": len(vals_all),
                "threshold": DETERMINISTIC_FLOOR_DB,
            }
            continue
        # Identical runs are excluded from the statistics: an infinite sample cannot enter a
        # mean, and leaving it out can only lower the threshold, which is the safe direction.
        vals = np.array(finite)
        mean = float(vals.mean())
        std = float(vals.std(ddof=0))
        threshold = math.floor((mean - max(SIGMA_MARGIN * std, MIN_MARGIN_DB)) * 2) / 2
        suffix = f"  ({identical}/{len(vals_all)} runs pixel-identical, excluded)" if identical else ""
        print(f"  {tag}: mean={mean:.2f} dB  std={std:.4f} dB  threshold={threshold:.1f} dB{suffix}")
        scenes_out[tag] = {
            "psnr_mean": round(mean, 4),
            "psnr_std": round(std, 4),
            "identical_runs": identical,
            "n_samples": len(vals_all),
            "threshold": threshold,
        }

    # Pixel-ruler audit fields. Attached to the scene entry beside the PSNR statistics rather
    # than replacing them: `threshold` keeps its meaning for the stochastic groups, and for a
    # deterministic group it is now the historical PSNR figure, not the ruler.
    for tag, (tau, per_run) in sorted(maxcc_data.items()):
        entry = scenes_out.setdefault(tag, {})
        entry["maxcc_tau"] = tau
        entry["maxcc_local_max"] = max(per_run)
        entry["maxcc_samples"] = len(per_run)
        note = "" if max(per_run) == 0 else "  ← NOT byte-identical on this machine; investigate before re-shooting"
        print(f"  {tag}: pixel ruler tau={tau}, local maxcc max={max(per_run)} over {len(per_run)} runs{note}")

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
    # The three fields beside "scenes" are group-scoped and are REWRITTEN whole on every run,
    # including a --scene run that recalibrated exactly one member. So they describe THIS run,
    # not the provenance of every scene in the group: after `--scene foo --n-calib 10` inside a
    # group whose other members were calibrated at 30, the file will read n_calib_runs=10 while
    # those members keep their original psnr_mean/psnr_std untouched. Do not read these three as
    # per-scene metadata and do not infer from them that existing scenes were re-calibrated —
    # the per-scene audit trail is n_samples / identical_runs inside each scene entry.
    # n_ref_runs is the Phase A count this invocation's flags resolve to for the group, whether
    # or not Phase A ran (--phase-b-only), matching what the same flags would have shot.
    groups_out[group.key] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "n_ref_runs": n_ref,
        "n_calib_runs": n_calib,
        "scenes": existing_scenes,
    }
    merged["groups"] = groups_out
    with open(json_path, "w") as fh:
        json.dump(merged, fh, indent=2)
        fh.write("\n")
    print(f"\n[Phase B][{group.key}] Thresholds written to {json_path} (scenes={len(existing_scenes)})")
    if maxcc_data and all(tag in maxcc_data for tag in scenes_out):
        print(f"[Phase B][{group.key}] Every scene compares under the pixel ruler; its K lives in "
              f"{group.source} and is not derived here — nothing to copy back.")
    else:
        print(f"[Phase B][{group.key}] Copy 'threshold' values into {group.source}")


def phase_b(args: argparse.Namespace) -> None:
    groups = _selected_groups(args)
    plan = _run_plan(groups, args.n_calib)
    ref_plan = _run_plan(groups, args.n)
    n_max = max(plan.values())
    binary = args.binary

    print(f"\n[Phase B] Threshold calibration: {n_max} full-suite run(s) shared by {len(groups)} group(s)")
    print(f"[Phase B] Binary : {binary}")
    samples: dict[str, _GroupSamples] = {}
    for group in groups:
        # Every group prints into the same full-suite stderr, and --scene must leave the group's
        # other scenes' entries untouched, so sampling is restricted to the selected keys.
        wanted = {_scene_key(scene, mode) for scene in _scene_list(group, args) for mode in group.modes}
        samples[group.key] = _GroupSamples(wanted=wanted, psnr={}, maxcc={})
        kind = "deterministic" if group.deterministic else "stochastic"
        print(f"[Phase B][{group.key}] N_calib={plan[group.key]} ({kind}); scenes {sorted(wanted)}")

    for i in range(n_max):
        print(f"[Phase B] Calibration run {i + 1}/{n_max}...", flush=True)
        _, stderr = _run(binary, [], capture_stderr=True)
        for group in groups:
            if i < plan[group.key]:
                _phase_b_ingest(group, samples[group.key], stderr)

    for group in groups:
        _phase_b_write_group(group, samples[group.key], plan[group.key], ref_plan[group.key], args)


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
        "--n",
        type=int,
        default=None,
        help=(
            "Phase A: number of reference runs, applied to every selected group. Default: "
            f"per group — 1 for a deterministic group, {STOCHASTIC_RUNS} for a stochastic one "
            "(the `deterministic` flag in GROUPS). Pass it for a deterministic group only when "
            "you are measuring something, e.g. whether it is still deterministic."
        ),
    )
    parser.add_argument(
        "--n-calib",
        type=int,
        default=None,
        help="Phase B: number of calibration runs, applied to every selected group. Default: same rule as --n.",
    )
    parser.add_argument(
        "--allow-stale-base",
        action="store_true",
        help=(
            "Shoot even if origin/main has already changed a reference this run would write "
            "(the default refuses, because that reference would be discarded at rebase time; "
            "rebase first instead)."
        ),
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
        return
    # Phase A is the only phase that writes reference images, so it is the only one gated.
    check_reference_base(_selected_groups(args), args)
    if args.phase_a_only:
        phase_a(args)
    else:
        phase_a(args)
        phase_b(args)


if __name__ == "__main__":
    main()
