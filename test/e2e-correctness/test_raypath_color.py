"""End-to-end regression for the per-raypath color display chain (scrum-336.4).

Pins the full CLI delivery path: a config with a `raypath_color` section must
produce an additional composited image `img_XX_components.<fmt>` (via the
LUMICE_GetCompositeResults C-API + SaveCompositeResults in main.cpp), while the
mono `img_XX.<fmt>` output is still produced (additive, not a replacement).

The fixture `raypath_color_three_arcs.json` is a two-crystal single-MS scene
mirroring the topology of MakeTwoCrystalColoredScene
(test/unit-correctness/server/test_component_compositor.cpp): crystal slot 0
produces component bit 0 (red), slot 1's 2-summand complex filter produces bits
1/2 (green/blue). All three bits get non-zero hits (verified during authoring).

PSNR threshold calibration (e2e methodology, matches test_smoke.py):
  Reference generated on macOS. Three fresh CLI runs vs the reference measured
  PSNR = 24.47 / 24.46 / 24.41 dB (stable — MC orientation noise only).
  threshold = min - 3 dB, floored to 0.5, with extra cross-platform margin
  (reference generated on macOS, CI runs on Linux) -> 20.0 dB. A structural
  regression (composite chain broken -> wrong/black image, or mono leaking into
  the composite slot) drops PSNR far below 20 dB, so the gate still catches it.
"""

from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from test.e2e.image_utils import compute_mse, compute_psnr, get_dimensions

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
REFERENCES_DIR = get_project_root() / "test" / "e2e-correctness" / "references"

CONFIG = CONFIGS_DIR / "raypath_color_three_arcs.json"
REFERENCE = REFERENCES_DIR / "raypath_color_three_arcs_components.jpg"

# Config resolution [width, height] (must match the fixture's render.resolution).
EXPECTED_DIMS = (512, 256)

# See module docstring for calibration.
PSNR_THRESHOLD = 20.0


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
