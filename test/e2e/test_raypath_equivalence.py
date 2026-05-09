"""E2E test: raypath 4-6 and 7-3 produce equivalent renderings.

Validates the symmetry fix from feat/raypath_symmetry:
SampleSphericalPointsSph fold + roll += π coupling ensures that
raypath [4,6] and [7,3] produce identical halo images under an
isotropic crystal orientation (zenith = Gauss(mean=0, std=180)).

PSNR threshold: calibrated by running both configs 3 times
(different random seeds) and taking min_psnr - 3 dB.
"""

import glob
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW

if HAS_PILLOW:
    from test.e2e.image_utils import compute_mse, compute_psnr

CONFIGS_DIR = Path(__file__).resolve().parent / "configs"

# Threshold (dB) calibrated via 3 runs of min(PSNR(4-6, 7-3)) − 3 dB.
# Calibration on 2026-05-09: runs = [24.62, 24.52, 24.66] dB, min = 24.52 dB.
EQUIVALENCE_PSNR_THRESHOLD = 21.5


class TestRaypathEquivalence(LumiceTestCase):
    """Verify that raypath [4,6] and [7,3] produce equivalent renderings."""

    def _run_config_and_get_image(self, config_name: str) -> str:
        """Run Lumice with a config and return the path to the output image."""
        cfg_path = CONFIGS_DIR / f"{config_name}.json"
        if not cfg_path.exists():
            self.skipTest(f"Config not found: {cfg_path}")

        result = self.run_lumice(["-f", str(cfg_path), "-o", self.output_dir])
        self.assertEqual(
            result.returncode, 0,
            f"{config_name} failed:\nstdout: {result.stdout}\nstderr: {result.stderr}",
        )
        images = sorted(glob.glob(os.path.join(self.output_dir, "img_*.jpg")))
        self.assertTrue(len(images) > 0, f"No output images for {config_name}")
        return images[0]

    def test_raypath_4_6_runs_successfully(self):
        """raypath_symmetry_4_6 should exit 0 and produce a non-empty image."""
        img = self._run_config_and_get_image("raypath_symmetry_4_6")
        self.assertGreater(os.path.getsize(img), 0)

    def test_raypath_7_3_runs_successfully(self):
        """raypath_symmetry_7_3 should exit 0 and produce a non-empty image."""
        img = self._run_config_and_get_image("raypath_symmetry_7_3")
        self.assertGreater(os.path.getsize(img), 0)

    @unittest.skipUnless(HAS_PILLOW, "Pillow not installed")
    def test_raypath_4_6_equals_7_3_psnr(self):
        """PSNR between raypath [4,6] and [7,3] outputs must exceed threshold.

        Both configs use zenith=Gauss(0,180°) (isotropic orientation),
        under which 4-6 and 7-3 are geometrically equivalent (P-symmetry,
        C6 rotation by 3 steps / 180°).
        """
        dir_46 = tempfile.mkdtemp(prefix="lumice_46_")
        dir_73 = tempfile.mkdtemp(prefix="lumice_73_")
        orig_dir = self.output_dir
        try:
            self.output_dir = dir_46
            img_46 = self._run_config_and_get_image("raypath_symmetry_4_6")

            self.output_dir = dir_73
            img_73 = self._run_config_and_get_image("raypath_symmetry_7_3")

            mse = compute_mse(img_46, img_73)
            psnr = compute_psnr(mse)
            self.assertGreaterEqual(
                psnr,
                EQUIVALENCE_PSNR_THRESHOLD,
                f"Raypath equivalence failed: PSNR(4-6, 7-3) = {psnr:.1f} dB "
                f"< threshold {EQUIVALENCE_PSNR_THRESHOLD} dB. "
                f"This may indicate a regression in SampleSphericalPointsSph "
                f"fold+roll coupling.",
            )
        finally:
            self.output_dir = orig_dir
            shutil.rmtree(dir_46, ignore_errors=True)
            shutil.rmtree(dir_73, ignore_errors=True)
