"""Smoke tests for Lumice — run all configs and verify outputs."""

import glob
import os
from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW

if HAS_PILLOW:
    from test.e2e.image_utils import compute_mse, compute_psnr, get_dimensions

CONFIGS_DIR = Path(__file__).resolve().parent / "configs"
REFERENCES_DIR = Path(__file__).resolve().parent / "references"

# PSNR thresholds per reference image (dB).
# Calibrated by running each config 3 times and taking min_psnr - 3dB.
# Set to None to skip PSNR check for that output.
PSNR_THRESHOLDS = {
    "color_01": 18.5,
    "cza_01": 35.1,
    "filters_01": 16.8,
    "halo_22_01": 17.9,
    "multi_lens_01": 25.5,
    "multi_lens_02": 26.3,
    "multi_lens_03": 38.5,
    "multi_scatter_01": 17.8,
    "parhelion_01": 25.7,
    "pyramid_01": 18.9,
    "render_opts_01": 22.6,
}


def _discover_configs():
    """Return sorted list of config JSON paths."""
    return sorted(CONFIGS_DIR.glob("*.json"))


class TestSmoke(LumiceTestCase):
    """Smoke tests: run every config and verify basic outputs."""

    def test_all_configs_run_successfully(self):
        """Every config should exit 0, produce non-empty images of correct size and PSNR."""
        configs = _discover_configs()
        self.assertTrue(len(configs) > 0, "No configs found in test/e2e/configs/")

        for cfg_path in configs:
            config_name = cfg_path.stem
            with self.subTest(config=config_name):
                result = self.run_lumice(
                    ["-f", str(cfg_path), "-o", self.output_dir]
                )
                self.assertEqual(
                    result.returncode,
                    0,
                    f"{config_name} failed:\n{result.stderr}",
                )

                # Check output files exist and are non-empty
                output_imgs = sorted(
                    glob.glob(os.path.join(self.output_dir, "img_*.jpg"))
                )
                self.assertTrue(
                    len(output_imgs) > 0,
                    f"{config_name}: no output images in {self.output_dir}",
                )

                for img_path in output_imgs:
                    size = os.path.getsize(img_path)
                    self.assertGreater(
                        size, 0, f"{img_path} is empty"
                    )

                    if HAS_PILLOW:
                        # Check dimensions
                        w, h = get_dimensions(img_path)
                        self.assertEqual(
                            (w, h), (256, 256),
                            f"{img_path} dimensions {w}x{h} != 256x256",
                        )

                        # Check PSNR against reference image
                        renderer_id = Path(img_path).stem.split("_")[-1]
                        ref_name = f"{config_name}_{renderer_id}.jpg"
                        ref_path = REFERENCES_DIR / ref_name
                        threshold_key = f"{config_name}_{renderer_id}"

                        if ref_path.exists() and threshold_key in PSNR_THRESHOLDS:
                            threshold = PSNR_THRESHOLDS[threshold_key]
                            if threshold is not None:
                                mse = compute_mse(img_path, str(ref_path))
                                psnr = compute_psnr(mse)
                                self.assertGreaterEqual(
                                    psnr,
                                    threshold,
                                    f"{ref_name}: PSNR {psnr:.1f} dB < threshold {threshold} dB",
                                )

                # Clean output_dir for next config
                for f in glob.glob(os.path.join(self.output_dir, "*")):
                    os.remove(f)

    def test_stdout_contains_stats(self):
        """Lumice stdout should contain Stats: and Saved: lines."""
        cfg = CONFIGS_DIR / "halo_22.json"
        if not cfg.exists():
            self.skipTest("halo_22.json not found")

        result = self.run_lumice(
            ["-f", str(cfg), "-o", self.output_dir]
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("Stats:", result.stdout)
        self.assertIn("Saved:", result.stdout)
