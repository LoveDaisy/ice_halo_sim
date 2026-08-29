#!/usr/bin/env python3
"""Locate GUI colours that do not follow the palette.

The question this answers is the one the theme cannot answer about itself: given a second,
deliberately distant palette, WHICH PIXELS COME OUT THE SAME? A pixel that is identical under both
palettes was drawn with a colour some call site chose on its own — either a leak to fix, or an
exemption that needs a name and a reason (doc/gui-visual-language.md, section 7).

It only ever produces CANDIDATES. A pixel can legitimately be palette-independent (a user-chosen
data colour, a mark drawn on top of the rendered image), so nothing here is a verdict; the
disposition is done by a person, per candidate, with a written reason.

Two layers, kept separate on purpose because they have different callers:

  Layer A -- compare_export_dirs(): a general per-pixel comparison of two directories of PNGs,
             paired by filename. It knows nothing about gui_test, palettes or this task. Both
             modes below call it, and it is the only place a pixel is ever compared.

  Layer B -- the default CLI mode: drive gui_test twice over the `theme_scan` cases (once with the
             production palette, once with --theme-palette contrast) and hand the two export
             directories to layer A.

  Layer A is also reachable directly: `--compare-dirs A B --tolerance T` skips layer B's
  orchestration entirely and compares two directories the caller already prepared. That is how a
  before/after check of an unrelated reference group is done (same palette on both sides, so
  --tolerance 0 asks for exact equality).

Usage:
    # Layer B: find candidates.
    python scripts/scan_theme_leaks.py --binary build/Release/static/bin/gui_test

    # Layer A: did this code change move any pixel of an already-exported group?
    python scripts/scan_theme_leaks.py --compare-dirs before/ after/ --tolerance 0
"""

import argparse
import os
import subprocess
import sys
import tempfile

try:
    import numpy as np
    from PIL import Image
except ImportError:
    print("ERROR: numpy and Pillow are required. Install with: pip install numpy Pillow", file=sys.stderr)
    sys.exit(1)

# The gui_test category these scenes register under. Also the filename prefix they export with,
# so the two runs' PNGs pair up by name.
SCAN_FILTER = "theme_scan"

# Default tolerance for layer B, in 0..255 RGB Euclidean distance.
#
# Calibrated against the two populations it has to separate, both sampled from real exports of
# these scenes: a themed pixel moves by hundreds (the contrast palette is >=0.54 away per field in
# 0..1 units, i.e. >=138 here), while a pixel that is genuinely the same colour in both runs moves
# by 0 -- except along an anti-aliased glyph or rounded-rect edge, where the SAME literal colour
# composites against a DIFFERENT background and can shift by a few counts. 8 sits far above that
# edge noise and two orders below a real palette response, so the choice is not delicate.
DEFAULT_TOLERANCE = 8.0

# A pixel that survives the tolerance test is painted this, so the highlight PNG reads as "here are
# the candidates" at a glance; everything else is dimmed rather than blanked, to keep the context
# needed to tell WHICH widget a candidate belongs to.
HIGHLIGHT_RGB = (255, 0, 0)
CONTEXT_DIM = 0.28


def _load_rgb(path: str) -> np.ndarray:
    """PNG -> float32 HxWx3 in 0..255. Alpha, if present, is dropped rather than composited:
    these captures come from glReadPixels on the default framebuffer, where alpha carries no
    information the comparison wants."""
    with Image.open(path) as im:
        return np.asarray(im.convert("RGB"), dtype=np.float32)


def compare_export_dirs(dir_a: str, dir_b: str, tolerance: float, out_dir: str | None = None) -> list[dict]:
    """Layer A. Pair PNGs by filename across two directories and report, per pair, how much of the
    image did NOT change by more than `tolerance`.

    Returns one dict per pair: name, total pixels, unchanged pixels, unchanged fraction, and the
    path of the highlight image if one was written. A file present in only one directory is
    reported with an `error` field instead of being skipped silently -- a missing scene is a hole
    in the scan, not a pair that happens to match.
    """
    names_a = {f for f in os.listdir(dir_a) if f.endswith(".png")}
    names_b = {f for f in os.listdir(dir_b) if f.endswith(".png")}
    results: list[dict] = []

    for name in sorted(names_a | names_b):
        if name not in names_a or name not in names_b:
            missing = dir_b if name not in names_b else dir_a
            results.append({"name": name, "error": f"missing in {missing}"})
            continue

        a = _load_rgb(os.path.join(dir_a, name))
        b = _load_rgb(os.path.join(dir_b, name))
        if a.shape != b.shape:
            results.append({"name": name, "error": f"shape {a.shape} vs {b.shape}"})
            continue

        dist = np.sqrt(np.sum((a - b) ** 2, axis=2))
        unchanged = dist <= tolerance
        total = int(unchanged.size)
        n_unchanged = int(np.count_nonzero(unchanged))

        entry = {
            "name": name,
            "total": total,
            "unchanged": n_unchanged,
            "fraction": n_unchanged / total if total else 0.0,
            "highlight": None,
        }

        if out_dir is not None:
            os.makedirs(out_dir, exist_ok=True)
            vis = (a * CONTEXT_DIM).astype(np.uint8)
            vis[unchanged] = HIGHLIGHT_RGB
            highlight = os.path.join(out_dir, f"highlight_{name}")
            Image.fromarray(vis).save(highlight)
            entry["highlight"] = highlight

        results.append(entry)

    return results


def _run_scan(binary: str, export_dir: str, contrast: bool) -> None:
    """One layer-B gui_test run. --keep-export-png so the PNGs survive for the comparison."""
    cmd = [binary, "--filter", SCAN_FILTER, "--export-dir", export_dir, "--keep-export-png", "--fixed-dt",
           "--no-user-config"]
    if contrast:
        cmd += ["--theme-palette", "contrast"]
    print("  $ " + " ".join(cmd))
    result = subprocess.run(cmd)
    # Not fatal: a scene may fail its own assertions while still having exported a usable PNG, and
    # a partial scan is more useful than none. It is printed loudly because a non-zero exit here is
    # a reason to distrust whichever scene is missing below.
    if result.returncode != 0:
        print(f"  WARNING: gui_test exited {result.returncode}", file=sys.stderr)


def _report(results: list[dict], tolerance: float) -> int:
    print(f"\n{'scene':44s} {'unchanged':>12s} {'of total':>12s} {'pct':>7s}")
    print("-" * 80)
    errors = 0
    for r in results:
        if "error" in r:
            print(f"{r['name']:44s}   ERROR: {r['error']}")
            errors += 1
            continue
        print(f"{r['name']:44s} {r['unchanged']:12d} {r['total']:12d} {100 * r['fraction']:6.2f}%")
        if r["highlight"]:
            print(f"{'':44s}   -> {r['highlight']}")
    print(f"\ntolerance = {tolerance} (RGB Euclidean, 0..255)")
    print("Non-zero 'unchanged' is a CANDIDATE LIST, not a verdict: dispose of each one by hand.")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary", default="build/Release/static/bin/gui_test",
                    help="gui_test binary (layer B only)")
    ap.add_argument("--compare-dirs", nargs=2, metavar=("DIR_A", "DIR_B"),
                    help="layer A direct call: compare two prepared export directories, no gui_test run")
    ap.add_argument("--tolerance", type=float, default=None,
                    help=f"RGB Euclidean distance below which a pixel counts as unchanged "
                         f"(default {DEFAULT_TOLERANCE} for layer B; required to be given explicitly "
                         f"is not enforced, but --compare-dirs callers should state it)")
    ap.add_argument("--out-dir", default=None,
                    help="where to write highlight PNGs (default: a directory next to the exports)")
    args = ap.parse_args()

    tolerance = args.tolerance if args.tolerance is not None else DEFAULT_TOLERANCE

    if args.compare_dirs:
        dir_a, dir_b = args.compare_dirs
        results = compare_export_dirs(dir_a, dir_b, tolerance, args.out_dir)
        return 1 if _report(results, tolerance) else 0

    if not os.path.exists(args.binary):
        print(f"ERROR: gui_test not found at {args.binary}. Build with: ./scripts/build.sh -gtj release",
              file=sys.stderr)
        return 1

    work = tempfile.mkdtemp(prefix="theme_scan_")
    dir_default = os.path.join(work, "palette_default")
    dir_contrast = os.path.join(work, "palette_contrast")
    os.makedirs(dir_default)
    os.makedirs(dir_contrast)

    print(f"Scratch: {work}")
    print("Run 1/2 - production palette")
    _run_scan(args.binary, dir_default, contrast=False)
    print("Run 2/2 - contrast palette")
    _run_scan(args.binary, dir_contrast, contrast=True)

    out_dir = args.out_dir or os.path.join(work, "highlights")
    results = compare_export_dirs(dir_default, dir_contrast, tolerance, out_dir)
    return 1 if _report(results, tolerance) else 0


if __name__ == "__main__":
    sys.exit(main())
