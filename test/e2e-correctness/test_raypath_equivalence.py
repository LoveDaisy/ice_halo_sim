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
import unittest

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from test.e2e.image_utils import compute_mse, compute_psnr

# TODO: relocate configs when follow-up task completes
CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# Threshold (dB) calibrated via 3 runs of min(PSNR(4-6, 7-3)) − 3 dB.
# Calibration on 2026-05-10: runs = [26.73, 26.80, 26.69] dB, min = 26.69 dB.
EQUIVALENCE_PSNR_THRESHOLD = 23.5


CONFIG_NAMES = ("raypath_symmetry_4_6", "raypath_symmetry_7_3")


class TestRaypathEquivalence(LumiceTestCase):
    """Verify that raypath [4,6] and [7,3] produce equivalent renderings."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        # Each config is rendered once for the whole class. The two "runs
        # successfully" tests and the PSNR comparison ask three questions of the
        # same two frames, so rendering per test bought the same pixels twice.
        for name in CONFIG_NAMES:
            cfg_path = CONFIGS_DIR / f"{name}.json"
            if not cfg_path.exists():
                raise unittest.SkipTest(f"Config not found: {cfg_path}")
        cls.renders = {
            name: cls.render_once(CONFIGS_DIR / f"{name}.json") for name in CONFIG_NAMES
        }

    def _image_for(self, config_name: str) -> str:
        """Return the image path of this class's shared render of `config_name`."""
        result = self.renders[config_name]
        self.assertEqual(
            result.returncode, 0,
            f"{config_name} failed:\nstdout: {result.stdout}\nstderr: {result.stderr}",
        )
        images = sorted(glob.glob(os.path.join(result.output_dir, "img_*.jpg")))
        self.assertTrue(len(images) > 0, f"No output images for {config_name}")
        return images[0]

    def test_raypath_4_6_runs_successfully(self):
        """raypath_symmetry_4_6 should exit 0 and produce a non-empty image."""
        img = self._image_for("raypath_symmetry_4_6")
        self.assertGreater(os.path.getsize(img), 0)

    def test_raypath_7_3_runs_successfully(self):
        """raypath_symmetry_7_3 should exit 0 and produce a non-empty image."""
        img = self._image_for("raypath_symmetry_7_3")
        self.assertGreater(os.path.getsize(img), 0)

    @unittest.skipUnless(HAS_PILLOW, "Pillow not installed")
    def test_raypath_4_6_equals_7_3_psnr(self):
        """PSNR between raypath [4,6] and [7,3] outputs must exceed threshold.

        Both configs use zenith=Gauss(0,180°) (isotropic orientation),
        under which 4-6 and 7-3 are geometrically equivalent (P-symmetry,
        C6 rotation by 3 steps / 180°).
        """
        img_46 = self._image_for("raypath_symmetry_4_6")
        img_73 = self._image_for("raypath_symmetry_7_3")

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
