"""End-to-end regression for the per-raypath color display chain (scrum-336.4).

Pins the full CLI delivery path: a config with a `raypath_color` section must
produce an additional composited image `img_XX_components.<fmt>` (via the
LUMICE_GetCompositeResults C-API + SaveCompositeResults in main.cpp), while the
mono `img_XX.<fmt>` output is still produced (additive, not a replacement).

The fixture `raypath_color_three_arcs.json` is a single-MS scene under the
Design-2 placement-scoped color model (task-engine-redirect-design2,
doc/gui-custom-spectrum-and-raypath-color.md §4.0): two geometrically-identical
crystals hold disjoint physical filters — crystal 1 (filter 1, pass-all) and
crystal 2 (filter 4, len in {2,3}). The RED class is a match-all predicate on
crystal 1; GREEN / BLUE are entry_exit length predicates (len==2 / len>=3) on
crystal 2. All three classes get non-zero hits (verified via the color-direction
classifier: RED/GREEN/BLUE ~= 52937 / 2780 / 35663).

PSNR threshold calibration (e2e methodology, matches test_smoke.py):
  Reference regenerated on macOS against the participating-P99 self-anchor
  compositor (task-fix-composite-participating-exposure-anchor). Three fresh
  CLI runs vs the reference measured PSNR = 20.90 / 20.86 / 20.91 dB (stable —
  MC orientation noise only). The absolute PSNR floor is lower than the
  pre-anchor version (~24 dB) because the new anchor produces a significantly
  brighter composite — every MC-varied pixel now sits at a higher byte value,
  amplifying MC noise error rms even though the underlying orientation seeds
  are equivalently stable. threshold = min - 3 dB, floored to 0.5 = 17.5, with
  extra cross-platform margin (reference generated on macOS, CI runs on Linux)
  -> 16.5 dB. A structural regression (composite chain broken -> wrong/black
  image, or mono leaking into the composite slot) still drops PSNR into the
  single-digit range, so the gate still catches it.
"""

from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from test.e2e.image_utils import (
        classify_pixels_by_color_direction,
        compute_mse,
        compute_psnr,
        get_dimensions,
    )

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
REFERENCES_DIR = get_project_root() / "test" / "e2e-correctness" / "references"

CONFIG = CONFIGS_DIR / "raypath_color_three_arcs.json"
REFERENCE = REFERENCES_DIR / "raypath_color_three_arcs_components.jpg"

# Config resolution [width, height] (must match the fixture's render.resolution).
EXPECTED_DIMS = (512, 256)

# See module docstring for calibration.
PSNR_THRESHOLD = 16.5

# ------ task-339.5 multi-layer full-semantics fixture (see class below) ------
MULTI_LAYER_CONFIG = CONFIGS_DIR / "raypath_color_multi_layer.json"
MULTI_LAYER_REFERENCE = REFERENCES_DIR / "raypath_color_multi_layer_components.jpg"

# PSNR calibration (macOS host, reference regenerated against the
# participating-P99 self-anchor compositor,
# task-fix-composite-participating-exposure-anchor):
#   Run 1/2/3 = 20.011 / 19.987 / 19.986 dB (stable, MC noise only).
#   Absolute PSNR floor is lower than the pre-anchor version (~26 dB) for the
#   same reason as three-arcs — brighter composite amplifies MC noise error rms.
#   min - 3 dB, floored to 0.5 dB -> 16.5 dB. Extra cross-platform margin
#   (reference on macOS, CI on Linux) matching the ~1 dB safety used above
#   -> 15.5 dB. A structural regression (wrong dominant winner, per-class lane
#   wiring broken) drops PSNR far below 15.5 dB and the gate still fires.
MULTI_LAYER_PSNR_THRESHOLD = 15.5

# task-339.5 color-class list order in raypath_color_multi_layer.json.
# Order matters: dominant-mode tie-break is list-first, and the classifier
# receives colors in the same order so per_class[i] refers to class i.
MULTI_LAYER_CLASS_COLORS = [
    (1.0, 0.0, 1.0),   # 0: MAGENTA — cross-layer AND {L0,C1}∧{L1,C2} (case D)
    (1.0, 0.55, 0.0),  # 1: ORANGE  — L1 crystal 3 match-all (case E)
    (1.0, 0.0, 0.0),   # 2: RED     — L0 C4 match-all single class (case A)
    (0.0, 1.0, 0.0),   # 3: GREEN   — L0 C5 OR-union of two length predicates (case B)
]

# task-339.5 per-class minimum pixel count. Calibration below matches
# `classify_pixels_by_color_direction` with cos_similarity_tol=0.90.
# Observed per-class counts on the reference run (macOS, Design-2 renderer):
#   MAGENTA/ORANGE/RED/GREEN = 10528 / 56360 / 11155 / 20341.
# Threshold set to ~1/4 of the observed minimum (~10500 -> 2500) so each
# class is comfortably above zero (no wiring regression) and above MC noise
# floor, while tolerating cross-platform / MC-seed variation.
MULTI_LAYER_MIN_PIXELS_PER_CLASS = 2500

# task-339.5 unclassified-pixel cap. Observed unc=2173 (2% of total) at
# tol=0.90. Cap at 8% of total pixels (~10500 for 512x256) — a real
# phantom-hue regression (compositor mode leaking to additive-like mix,
# or class colors accidentally quantized off-direction) would blow past
# this floor by an order of magnitude.
MULTI_LAYER_MAX_UNCLASSIFIED_FRAC = 0.08

# task-339.5 classifier cosine tolerance. See
# `classify_pixels_by_color_direction` docstring — 0.90 tolerates the
# chroma bleed introduced by JPEG encoding at halo boundaries. Above 0.95
# the unclassified count grows into the 20%+ range (compression noise,
# not a real defect).
MULTI_LAYER_COS_TOL = 0.90


class TestRaypathColor(LumiceTestCase):
    """CLI end-to-end for per-raypath color composite output."""

    def test_composite_output_and_psnr(self):
        """A raypath_color config produces img_01_components.jpg matching the reference."""
        if not CONFIG.exists():
            self.skipTest(f"{CONFIG} not found")

        result = self.run_lumice(["-f", str(CONFIG), "-o", self.output_dir])
        self.assertEqual(
            result.returncode, 0, f"Lumice failed:\n{result.stderr}"
        )

        composite = Path(self.output_dir) / "img_01_components.jpg"
        mono = Path(self.output_dir) / "img_01.jpg"

        # (a) composite produced and non-empty.
        self.assertTrue(
            composite.exists(), f"composite image not produced: {composite}"
        )
        self.assertGreater(
            composite.stat().st_size, 0, f"{composite} is empty"
        )

        # (c) mono image still produced (composite is additive, not a replacement).
        self.assertTrue(mono.exists(), f"mono image not produced: {mono}")
        self.assertGreater(mono.stat().st_size, 0, f"{mono} is empty")

        if HAS_PILLOW:
            # (a) dims == config resolution.
            self.assertEqual(
                get_dimensions(str(composite)),
                EXPECTED_DIMS,
                "composite resolution mismatch",
            )

            # (b) PSNR(composite, reference) >= threshold.
            self.assertTrue(
                REFERENCE.exists(), f"reference missing: {REFERENCE}"
            )
            mse = compute_mse(str(composite), str(REFERENCE))
            psnr = compute_psnr(mse)
            self.assertGreaterEqual(
                psnr,
                PSNR_THRESHOLD,
                f"composite PSNR {psnr:.1f} dB < threshold {PSNR_THRESHOLD} dB",
            )


class TestRaypathColorMultiLayer(LumiceTestCase):
    """CLI end-to-end for the 2-MS-layer full-semantics composite (task-339.5).

    Fixture `raypath_color_multi_layer.json` is the raypath-color-engine
    capstone regression, migrated to the Design-2 placement-scoped model
    (task-engine-redirect-design2, §4.0): a 2-MS-layer scene with 4 color
    classes covering the case rows the engine supports today —
      A single class (RED, match-all `{L0,C4}`)
      B OR-union   (GREEN, `{L0,C5}` len==2 predicate ∨ len>=3 predicate)
      C overlap    (ORANGE overlaps RED / GREEN via shared L1-C3 rays;
                    dominant argmax + list-first tie-break decides)
      D cross-layer AND (MAGENTA, match-all `{L0,C1}` ∧ `{L1,C2}`)
      E whole crystal (ORANGE, match-all `{L1,C3}`)

    Design constraint: under Design-2 the placement key is the crystal id
    (not a filter id), so the three L0 physical sub-populations that were
    previously one crystal + three filters (F2/F3/F4) are now three
    geometrically-identical crystals (C1/C4/C5) each holding one filter.
    This keeps MAGENTA / RED / GREEN ray-sets pairwise disjoint (a
    subset-of-superset relation would let the dominant `>` argmax with
    list-first tie-break swallow the more specific class). Crystal 2 is a
    plate (zenith gauss 0/1) and Crystal 3 a rotating column so C2 and C3
    halo footprints are spatially distinct, letting ORANGE win the 22° halo
    cleanly.
    """

    def test_multi_layer_all_semantics(self):
        """Multi-layer raypath_color config produces a well-formed composite
        with all four classes visibly present and no phantom-hue regression.
        """
        if not MULTI_LAYER_CONFIG.exists():
            self.skipTest(f"{MULTI_LAYER_CONFIG} not found")

        result = self.run_lumice(["-f", str(MULTI_LAYER_CONFIG), "-o", self.output_dir])
        self.assertEqual(
            result.returncode, 0, f"Lumice failed:\n{result.stderr}"
        )

        composite = Path(self.output_dir) / "img_01_components.jpg"
        mono = Path(self.output_dir) / "img_01.jpg"

        # (a) Composite + mono produced (additive delivery, not replacement).
        self.assertTrue(
            composite.exists(), f"composite image not produced: {composite}"
        )
        self.assertGreater(
            composite.stat().st_size, 0, f"{composite} is empty"
        )
        self.assertTrue(mono.exists(), f"mono image not produced: {mono}")
        self.assertGreater(mono.stat().st_size, 0, f"{mono} is empty")

        if HAS_PILLOW:
            # (b) Composite resolution matches config.
            self.assertEqual(
                get_dimensions(str(composite)),
                EXPECTED_DIMS,
                "multi-layer composite resolution mismatch",
            )

            # (c) PSNR(composite, reference) >= threshold.
            self.assertTrue(
                MULTI_LAYER_REFERENCE.exists(),
                f"multi-layer reference missing: {MULTI_LAYER_REFERENCE}",
            )
            mse = compute_mse(str(composite), str(MULTI_LAYER_REFERENCE))
            psnr = compute_psnr(mse)
            self.assertGreaterEqual(
                psnr,
                MULTI_LAYER_PSNR_THRESHOLD,
                f"multi-layer composite PSNR {psnr:.1f} dB < "
                f"threshold {MULTI_LAYER_PSNR_THRESHOLD} dB",
            )

            # (d) All four classes present in the composite output.
            # Any class dropping to zero flags a per-class lane wiring
            # regression (e.g. none-filter bit not carrying forward, AND
            # predicate rejecting all rays, entry-gate mapping wrong).
            result = classify_pixels_by_color_direction(
                str(composite),
                MULTI_LAYER_CLASS_COLORS,
                cos_similarity_tol=MULTI_LAYER_COS_TOL,
            )
            class_labels = ["MAGENTA", "ORANGE", "RED", "GREEN"]
            for idx, (label, count) in enumerate(
                zip(class_labels, result["per_class"])
            ):
                self.assertGreaterEqual(
                    count,
                    MULTI_LAYER_MIN_PIXELS_PER_CLASS,
                    f"class {idx} ({label}) has only {count} lit pixels, "
                    f"below floor {MULTI_LAYER_MIN_PIXELS_PER_CLASS}; "
                    f"predicate wiring likely regressed",
                )

            # (e) Phantom-hue floor: unclassified lit pixels must stay
            # under the cap. Blowing past it signals the compositor
            # switched modes silently (e.g. dominant -> additive) or a
            # class color got quantized off-direction.
            unclassified_cap = int(
                result["total"] * MULTI_LAYER_MAX_UNCLASSIFIED_FRAC
            )
            self.assertLessEqual(
                result["unclassified"],
                unclassified_cap,
                f"{result['unclassified']} unclassified lit pixels exceed "
                f"cap {unclassified_cap} ({MULTI_LAYER_MAX_UNCLASSIFIED_FRAC:.0%} "
                f"of {result['total']}); possible phantom-hue regression",
            )
