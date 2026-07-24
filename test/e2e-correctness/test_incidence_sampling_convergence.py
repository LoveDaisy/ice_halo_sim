"""E2E convergence sanity (AC3) for crystal entry-point incidence sampling.

Third acceptance criterion of the 口径-B (distribution-invariance) harness for the
entry-point sampler. AC1 (per-face projected-area distribution) and AC2 (in-face
uniformity) are the analytic gates — see
test/golden-analytic/core/test_incidence_sampling_polygon_oracle.cpp. AC3 is the
end-to-end sanity leg: a full CLI render over showcase profiles (22° halo,
plate parhelion). It is deliberately NOT the primary judge — a compensating bug
can pass a PSNR gate — so it runs *alongside* AC1+AC2, never instead of them.

What this leg establishes (and what it does NOT):
  * SELF-CONSISTENCY FLOOR: two statistically-independent renders of the SAME
    config (the CLI seeds its RNG from the wall clock, so separate processes draw
    independent sequences) differ only by Monte-Carlo noise. The PSNR between them
    is the noise floor of "the same distribution rendered twice". T2/T3, after
    changing the sampler, judge "did the new sampler converge to the SAME
    distribution as the old one" against exactly this floor — a new-vs-old PSNR at
    or above it means the distributions are indistinguishable at this ray count.
  * FROZEN BASELINE: the current (triangle) sampler's output is already frozen for
    these two configs by the smoke references (references/halo_22_01.jpg,
    references/parhelion_01.jpg, gated by test_smoke.py). This leg reuses those
    references as the 口径-B baseline rather than duplicating the JPEGs, and asserts
    a fresh render still matches them — so a T2/T3 regression that shifts the
    overall distribution shows up here too.

In T1 this leg has NO power to distinguish old vs new sampler (there is only one
sampler yet). Its T1 deliverable is the methodology + the calibrated floor; the
discriminating power switches on in T2/T3.
"""

import glob
import os
import shutil
import tempfile
import unittest

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from test.e2e.image_utils import compute_mse, compute_psnr

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
REFERENCES_DIR = get_project_root() / "test" / "e2e-correctness" / "references"

# Self-consistency PSNR floor (dB) per config. Calibrated 2026-07-23 by rendering
# each config 3 times (independent wall-clock-seeded processes) and taking the min
# pairwise PSNR − 3 dB, floored to 0.5 dB (repo convention, cf. test_smoke.py /
# test_raypath_equivalence.py):
#   halo_22:   pairwise 29.68 / 29.73 / 29.71 → min 29.68 → 26.5 dB
#   parhelion: pairwise 37.99 / 38.01 / 38.04 → min 37.99 → 35.0 dB
SELF_CONSISTENCY_FLOOR = {
    "halo_22": 26.5,
    "parhelion": 35.0,
}


class TestIncidenceSamplingConvergence(LumiceTestCase):
    """AC3: end-to-end convergence sanity for the entry-point sampler."""

    def _render(self, config_name: str, out_dir: str) -> str:
        """Render one config into out_dir; return the first output image path."""
        cfg_path = CONFIGS_DIR / f"{config_name}.json"
        if not cfg_path.exists():
            self.skipTest(f"Config not found: {cfg_path}")
        result = self.run_lumice(["-f", str(cfg_path), "-o", out_dir])
        self.assertEqual(
            result.returncode, 0,
            f"{config_name} failed:\nstdout: {result.stdout}\nstderr: {result.stderr}",
        )
        images = sorted(glob.glob(os.path.join(out_dir, "img_*.jpg")))
        self.assertTrue(len(images) > 0, f"No output image for {config_name}")
        self.assertGreater(os.path.getsize(images[0]), 0, f"{config_name}: empty image")
        return images[0]

    @unittest.skipUnless(HAS_PILLOW, "Pillow not installed")
    def test_self_consistency_and_frozen_baseline(self):
        """Two independent renders agree within the noise floor, and match baseline.

        For each config: render twice (independent seeds). run1-vs-run2 PSNR gives
        the self-consistency noise floor (must clear the calibrated threshold), and
        run1-vs-frozen-reference PSNR ties the current sampler to the 口径-B baseline
        the smoke references already freeze.
        """
        for config_name, floor in SELF_CONSISTENCY_FLOOR.items():
            with self.subTest(config=config_name):
                dir_a = tempfile.mkdtemp(prefix=f"lumice_conv_{config_name}_a_")
                dir_b = tempfile.mkdtemp(prefix=f"lumice_conv_{config_name}_b_")
                try:
                    img_a = self._render(config_name, dir_a)
                    img_b = self._render(config_name, dir_b)

                    # Self-consistency: two independent renders of the same config.
                    psnr_self = compute_psnr(compute_mse(img_a, img_b))
                    self.assertGreaterEqual(
                        psnr_self, floor,
                        f"{config_name}: self-consistency PSNR {psnr_self:.1f} dB "
                        f"< floor {floor} dB — renders are noisier than the "
                        f"calibrated Monte-Carlo floor (rendering regression?).",
                    )

                    # Frozen baseline: current sampler vs the smoke reference.
                    ref_path = REFERENCES_DIR / f"{config_name}_01.jpg"
                    self.assertTrue(
                        ref_path.exists(),
                        f"Baseline reference missing: {ref_path}",
                    )
                    psnr_ref = compute_psnr(compute_mse(img_a, str(ref_path)))
                    self.assertGreaterEqual(
                        psnr_ref, floor,
                        f"{config_name}: PSNR vs frozen baseline {psnr_ref:.1f} dB "
                        f"< floor {floor} dB — the entry-point distribution drifted "
                        f"from the 口径-B baseline.",
                    )
                finally:
                    shutil.rmtree(dir_a, ignore_errors=True)
                    shutil.rmtree(dir_b, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
