"""End-to-end coverage for painter as the *default* raypath_color composite mode
(task-painter-default-e2e-coverage, follow-up to task-367 PR#199).

Motivation
----------
task-367 flipped the default composite mode from ``dominant`` to ``painter`` at
`src/config/raypath_color_config.hpp::kDefaultCompositeMode`. During that task
the two existing raypath_color e2e fixtures (`raypath_color_three_arcs.json` /
`raypath_color_multi_layer.json`) were **explicitly pinned to ``"mode":
"dominant"``** because their structural probes (argmax pixel-count floor,
phantom-hue cap) were calibrated against dominant-specific physics. As a
side-effect no e2e test drove the full pipeline "bare-array config -> parses as
painter default -> renders end-to-end". The painter default was covered only by
config-layer unit tests (`test_raypath_color_config.cpp::JsonRoundTripDefaultModeBareArray`)
plus compositor pixel-math unit tests (`test_component_compositor.cpp`). This
file closes that gap.

Fixture design (see plan.md §3 "Key design point 1")
----------------------------------------------------
``painter_default_overlap.json`` is a minimal single-crystal single-filter
scene with TWO color classes that share the **exact same** match predicate
(``{"layer": 0, "crystal": 1}``, match-all). Every ray hits both lanes with
identical Y-lane values (bit-for-bit equal accumulation). This turns the
painter/dominant behavioural difference into a **structural certainty** rather
than a statistical hope:

  * painter mode (bare array, default): alpha-over blends the second class on
    top of the first at every overlap pixel -> the underlying class (RED) is
    visible through the top class (BLUE) so long as the top-lane alpha is not
    saturated -> nearly all lit pixels have BOTH R and B nonzero.

  * dominant mode (same config wrapped in ``{"mode": "dominant", "classes": [...]}``):
    ``CompositeDominantPixel`` uses ``if (ey > best_ey)`` (STRICT greater-than)
    walking classes in list order -> equal Y-lane values keep the list-first
    (RED) class as winner -> B channel is exactly zero on every lit pixel.

Why the two lanes are bit-for-bit equal (white-box, platform-independent)
-------------------------------------------------------------------------
``RenderConsumer::AccumulateColorClassLanes`` walks rays in an outer loop and,
per ray, adds the SAME scalar ``y`` to every class lane whose predicate is
satisfied (``lane_y_[c][pidx] += y``). RED and BLUE carry the IDENTICAL match
predicate, so for every ray ``satisfied`` is identical for both classes and
each lane receives the exact same sequence of float additions in the same
order. The two lanes are therefore bit-identical regardless of worker-thread
scheduling or reduction order -- any float non-associativity affects both lanes
symmetrically and cancels. Hence ``ey_RED == ey_BLUE`` exactly, on any
platform, and the strict-`>` tie-break deterministically picks RED, so the
dominant B channel is exactly 0. This is why the dominant tolerance below is a
hard zero rather than an epsilon (plan §7 risk 2 considered an epsilon; the
accumulation-symmetry proof makes it unnecessary -- a nonzero B channel would
be a genuine tie-break / accumulation regression worth failing on, not float
noise). The PR CI run validates this across Ubuntu/macOS/Windows before merge.

Structural assertion (see plan.md §3 "Key design point 2")
----------------------------------------------------------
On the "lit pixel" set (``max(R,G,B) > 8``) we count how many pixels have both
R and B nonzero. Painter must be near 100% of lit; dominant must be exactly 0
(the RED-wins tie-break is deterministic given bit-equal lanes; there is no
MC-noise tail to tolerate).

Threshold calibration (see plan.md §3 "Key design point 2" / Step 3)
--------------------------------------------------------------------
Measured on macOS Release build, 3 fresh CLI runs each (2026-07-15):
  Run 1 painter lit=6462 both_nz=6461 frac=0.9998  dominant lit=6470 both_nz=0
  Run 2 painter lit=6468 both_nz=6467 frac=0.9998  dominant lit=6456 both_nz=0
  Run 3 painter lit=6481 both_nz=6480 frac=0.9998  dominant lit=6469 both_nz=0
Given the dominant path is bit-deterministic (RED always wins the list-first
tie-break on bit-equal lanes) and painter is essentially saturated (only the
single edge pixel per run is not both-nonzero, likely a floating-point
boundary where BLUE's alpha rounds RED contribution to 0), the thresholds
are set with wide safety margin:
  painter: both_nz/lit >= 0.90 (real regression to a non-blending compositor
    would drop this to ~0.0)
  dominant: both_nz == 0 (strict; any nonzero would signal a tie-break rule
    change in ``CompositeDominantPixel``, e.g. `>` -> `>=` or reverse walk
    order)

Design notes
------------
  * The output format is ``--format png`` (not the default jpg): JPEG chroma
    subsampling smears small second-channel signals and could mask the
    painter blend. PNG is byte-exact against the compositor uint8 output.
  * We do NOT reuse ``test.e2e.image_utils.classify_pixels_by_color_direction``
    -- its docstring still describes the pre-task-367 painter semantics (a
    single winning class per pixel) which is stale under alpha-over blend.
    The tiny local helper below is scoped to this file; if a second consumer
    appears later, lift it to a shared helper then (deferred registration:
    SUMMARY.md).
  * The dominant control run does NOT get its own fixture file. We read the
    bare-array JSON at test time, wrap ``classes`` in the explicit ``{"mode":
    "dominant", "classes": [...]}`` form, write to ``self.output_dir``, and
    invoke the CLI a second time. Two fixture files would drift.
"""

import json
from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from PIL import Image
    from test.e2e.image_utils import get_dimensions

CONFIG = get_project_root() / "test" / "e2e" / "configs" / "painter_default_overlap.json"

LUMINANCE_FLOOR = 8

PAINTER_BOTH_NZ_FRAC_MIN = 0.90

DOMINANT_BOTH_NZ_MAX = 0


def _both_nonzero_fraction(png_path):
    """Return (lit_count, both_nz_count, both_nz_fraction) for the PNG at path.

    A "lit" pixel is one where max(R,G,B) > LUMINANCE_FLOOR. "both_nz" means
    both the R and B channels are strictly > 0. The fraction is
    both_nz_count / lit_count (0.0 when lit_count == 0).
    """
    with Image.open(png_path) as img:
        rgb = img.convert("RGB")
        pixels = list(rgb.getdata())

    lit = [(r, g, b) for r, g, b in pixels if max(r, g, b) > LUMINANCE_FLOOR]
    both_nz = sum(1 for r, _, b in lit if r > 0 and b > 0)
    frac = (both_nz / len(lit)) if lit else 0.0
    return len(lit), both_nz, frac


class TestPainterDefaultComposite(LumiceTestCase):
    """Painter is the default composite mode + painter blends where dominant occludes."""

    def test_bare_array_renders_default_painter(self):
        """A bare-array ``raypath_color`` config (no ``mode`` field) renders
        end-to-end without error and produces both the mono and composite PNGs
        at the configured resolution. Existence + non-empty size + dims is
        enough here; the blend-vs-occlude structural assertion lives in the
        next test.
        """
        self.assertTrue(
            CONFIG.exists(),
            f"fixture {CONFIG} missing -- this regression guard must FAIL "
            f"(not silently skip) if the painter-default fixture is deleted/moved",
        )

        result = self.run_lumice(
            ["-f", str(CONFIG), "-o", self.output_dir, "--format", "png"]
        )
        self.assertEqual(
            result.returncode, 0, f"Lumice failed:\n{result.stderr}"
        )

        composite = Path(self.output_dir) / "img_01_components.png"
        mono = Path(self.output_dir) / "img_01.png"

        self.assertTrue(
            composite.exists(), f"composite image not produced: {composite}"
        )
        self.assertGreater(
            composite.stat().st_size, 0, f"{composite} is empty"
        )
        self.assertTrue(mono.exists(), f"mono image not produced: {mono}")
        self.assertGreater(mono.stat().st_size, 0, f"{mono} is empty")

        if HAS_PILLOW:
            with open(CONFIG) as f:
                res = json.load(f)["render"][0]["resolution"]
            expected_dims = (res[0], res[1])
            self.assertEqual(
                get_dimensions(str(composite)),
                expected_dims,
                "composite resolution must match config render[0].resolution",
            )

    def test_painter_blends_dominant_occludes(self):
        """Painter (default, via bare array) blends both classes at overlap
        pixels; the same fixture wrapped as ``{"mode": "dominant", ...}``
        occludes the second class via list-first tie-break. See module
        docstring for the mechanism and threshold calibration.
        """
        if not HAS_PILLOW:
            self.skipTest("Pillow not available")
        self.assertTrue(
            CONFIG.exists(),
            f"fixture {CONFIG} missing -- this regression guard must FAIL "
            f"(not silently skip) if the painter-default fixture is deleted/moved",
        )

        painter_out = Path(self.output_dir) / "painter"
        painter_out.mkdir()
        result_p = self.run_lumice(
            ["-f", str(CONFIG), "-o", str(painter_out), "--format", "png"]
        )
        self.assertEqual(
            result_p.returncode, 0,
            f"Lumice (painter) failed:\n{result_p.stderr}"
        )
        painter_composite = painter_out / "img_01_components.png"
        self.assertTrue(
            painter_composite.exists(),
            f"painter composite not produced: {painter_composite}",
        )

        with open(CONFIG) as f:
            bare_cfg = json.load(f)
        dominant_cfg = dict(bare_cfg)
        dominant_cfg["raypath_color"] = {
            "mode": "dominant",
            "classes": bare_cfg["raypath_color"],
        }
        dominant_out = Path(self.output_dir) / "dominant"
        dominant_out.mkdir()
        dominant_cfg_path = dominant_out / "painter_default_overlap_dominant.json"
        with open(dominant_cfg_path, "w") as f:
            json.dump(dominant_cfg, f)
        result_d = self.run_lumice(
            ["-f", str(dominant_cfg_path), "-o", str(dominant_out), "--format", "png"]
        )
        self.assertEqual(
            result_d.returncode, 0,
            f"Lumice (dominant) failed:\n{result_d.stderr}"
        )
        dominant_composite = dominant_out / "img_01_components.png"
        self.assertTrue(
            dominant_composite.exists(),
            f"dominant composite not produced: {dominant_composite}",
        )

        p_lit, p_both, p_frac = _both_nonzero_fraction(painter_composite)
        d_lit, d_both, d_frac = _both_nonzero_fraction(dominant_composite)

        self.assertGreater(
            p_lit, 100,
            f"painter has too few lit pixels ({p_lit}); fixture may be broken",
        )
        self.assertGreater(
            d_lit, 100,
            f"dominant has too few lit pixels ({d_lit}); fixture may be broken",
        )

        self.assertGreaterEqual(
            p_frac, PAINTER_BOTH_NZ_FRAC_MIN,
            f"painter both-nonzero fraction {p_frac:.4f} < "
            f"{PAINTER_BOTH_NZ_FRAC_MIN} (lit={p_lit}, both_nz={p_both}); "
            f"painter default appears NOT to blend -- did the compositor "
            f"default silently revert to dominant?",
        )

        self.assertLessEqual(
            d_both, DOMINANT_BOTH_NZ_MAX,
            f"dominant both-nonzero pixel count {d_both} > "
            f"{DOMINANT_BOTH_NZ_MAX} (lit={d_lit}); dominant tie-break in "
            f"CompositeDominantPixel appears to have changed from strict `>` "
            f"or list-first walk order.",
        )
