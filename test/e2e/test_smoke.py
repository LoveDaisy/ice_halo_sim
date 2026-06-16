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
    "color_01": 27.1,
    "cza_01": 41.5,
    "filters_01": 26.7,
    "halo_22_01": 26.7,
    # ms_multi_crystal: MS multi-crystal-per-layer + dual_fisheye + D65 + 2M rays
    # is noisier than the single-wavelength configs. Measured run-to-run PSNR
    # ≈ 23.4 dB (stable: 23.41/23.42/23.44); threshold = min - 3dB floored to
    # 20.0 for cross-platform margin (reference generated on macOS, CI on Linux).
    # A structural regression (e.g. the frame band-vs-ring bug) drops PSNR far
    # below 20, so the gate still catches gross regressions.
    "ms_multi_crystal_01": 20.0,
    "multi_lens_01": 34.3,
    "multi_lens_02": 35.4,
    "multi_lens_03": 40.3,
    "multi_scatter_01": 26.6,
    "parhelion_01": 35.0,
    "pyramid_01": 28.8,
    "render_opts_01": 30.3,
    "dual_fisheye_ref_01": 25.8,
}


def _discover_configs():
    """Return sorted showcase config JSON paths (those with a reference image).

    The smoke leg runs on the fast (`-m "not slow"`) CI path with a 10-minute step
    budget. test/e2e/configs/ also holds heavy gate fixtures added for the Metal
    parity/throughput suites (ms3_*, ms_multi_crystal_complex_filter, parity_*, etc.)
    at 2-5M rays — and since task-268.7 the CPU/CLI route is single-worker (~12x
    slower), running those here blows the budget (the smoke leg timed out on
    ms3_mixed_pyramid_heavy). Those fixtures are validated by their own dedicated
    tests (test_metal_*, test_raypath_*, test_ms_filter_leak, test_cpu_backend_route),
    so the smoke test scopes to the showcase configs — identified by having a
    reference image under references/ (exactly the PSNR_THRESHOLDS set).
    """
    return sorted(
        cfg for cfg in CONFIGS_DIR.glob("*.json")
        if list(REFERENCES_DIR.glob(f"{cfg.stem}_*.jpg"))
    )


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
