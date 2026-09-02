"""Smoke tests for Lumice — run all configs and verify outputs."""

import glob
import os
from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from test.e2e.image_utils import compute_mse, compute_psnr, get_dimensions

# TODO: relocate configs when follow-up task completes
CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
REFERENCES_DIR = get_project_root() / "test" / "e2e-correctness" / "references"

# PSNR thresholds per reference image (dB).
# Calibrated by running each config 3 times and taking min_psnr - 3dB.
# Set to None to skip PSNR check for that output.
PSNR_THRESHOLDS = {
    # Thresholds for the 11 single-lens-family references (linear + 4 single-
    # fisheye) below were recalibrated by scrum-azimuth-handedness-alignment /
    # task-fix-render-projection-handedness (2026-07-03) after `xy.x` was
    # negated in ProjectExitToPixel's single-lens branch to align screen
    # handedness (right=+az) with the GUI. Method: 3 fresh CLI runs per config
    # against run_1 → threshold = min(PSNR_2vs1, PSNR_3vs1) - 3 dB, floored to
    # 0.5 dB precision. The 3 control-set thresholds (multi_lens_03,
    # ms_multi_crystal_01, dual_fisheye_ref_01 — dual-fisheye family, unaffected
    # by the flip) are preserved from their prior calibration.
    "color_01": 35.0,
    "cza_01": 41.0,
    "filters_01": 29.0,
    "halo_22_01": 26.5,
    # ms_multi_crystal: MS multi-crystal-per-layer + dual_fisheye + D65 + 2M rays
    # is noisier than the single-wavelength configs. Measured run-to-run PSNR
    # ≈ 23.4 dB (stable: 23.41/23.42/23.44); threshold = min - 3dB floored to
    # 20.0 for cross-platform margin (reference generated on macOS, CI on Linux).
    # A structural regression (e.g. the frame band-vs-ring bug) drops PSNR far
    # below 20, so the gate still catches gross regressions.
    "ms_multi_crystal_01": 20.0,
    "multi_lens_01": 33.5,
    "multi_lens_02": 34.5,
    "multi_lens_03": 40.3,
    "multi_scatter_01": 26.5,
    # orthographic_180: D65 + uniform full-random orientation + 1M rays. Per-run
    # PSNR (measured on macOS): run-to-run 22.74/22.76 dB (3 runs). Threshold
    # = min - 3 dB ≈ 19.7 → set 19.5 dB (rounded down to 0.5 dB precision) to
    # tolerate cross-platform sampling noise (reference generated on macOS,
    # CI runs on Linux). Re-introduces orthographic-projection e2e coverage
    # lost when scrum-268.6 scoped smoke to "configs with reference images"
    # (task-270.7 / explore-269 P0). A structural regression (frame bug,
    # wrong projection) drops PSNR far below this floor.
    "orthographic_180_01": 19.5,
    "parhelion_01": 34.5,
    "pyramid_01": 28.5,
    # render_opts: re-shot for 469.7, when `grid.outline` stopped being a no-op and started
    # drawing the celestial horizon — this config sets it, so the reference now carries a two-pixel
    # red line across rows 182-183 that the previous reference (taken while the key did nothing)
    # does not. The old reference scores 29.9 dB against the new render, i.e. the change is real
    # and this key is no longer structurally blind to the render option in its own name. Same
    # calibration method as the rest of this table: 3 fresh CLI runs, threshold =
    # min(PSNR_2vs1, PSNR_3vs1) - 3 dB floored to 0.5 dB precision.
    # Re-shot once more when the absolute-exposure branch landed on top of that change: the two
    # touch the same picture (one draws the horizon, the other sets the scale it is drawn at), so
    # neither side's reference is right for the merged result and the image had to come from a
    # binary carrying both. Run-to-run measured 35.92 / 36.02 dB, i.e. tighter than the 32.78 /
    # 33.06 dB of the previous shoot, so the threshold rises rather than relaxes.
    "render_opts_01": 32.5,
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

    def _check_one_config(self, cfg_path):
        """Run one config and verify exit code, outputs and PSNR."""
        config_name = cfg_path.stem
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


def _make_config_test(cfg_path):
    def _test(self):
        self._check_one_config(cfg_path)

    _test.__name__ = f"test_config_{cfg_path.stem}"
    _test.__doc__ = f"{cfg_path.stem}: exits 0, produces non-empty images, PSNR >= threshold."
    return _test


# One pytest item per config, rather than one item looping over all of them: xdist
# distributes by item, so a single looping item pins every config to one worker and
# becomes the whole set's wall clock. `unittest.TestCase` methods cannot be driven by
# @pytest.mark.parametrize (pytest does not support it there), hence the setattr
# generation below. The assert makes "discovered nothing" an import-time hard failure
# instead of a silent collection of zero cases.
_CONFIGS = _discover_configs()
assert _CONFIGS, "No configs with reference images found in test/e2e/configs/"
for _cfg in _CONFIGS:
    setattr(TestSmoke, f"test_config_{_cfg.stem}", _make_config_test(_cfg))
