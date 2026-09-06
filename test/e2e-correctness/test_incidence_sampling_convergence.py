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
  * FROZEN BASELINE — retired, deliberately not asserted here. Both configs are
    already frozen against the very same JPEGs (references/halo_22_01.jpg,
    references/parhelion_01.jpg) by test_smoke.py, at a threshold that is equal
    (halo_22: 26.5 vs 26.5) or stricter (parhelion: 37.5 vs 35.0). Re-rendering
    them here bought a second, strictly weaker copy of an oracle smoke already
    owns — same CLI, same references, hence the same failure modes, so it was
    never independent corroboration either. A T2/T3 distribution shift still
    shows up: it shows up in test_smoke.py.

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

    def _check_self_consistency(self, config_name: str, floor: float):
        """Two independent renders of one config agree within the noise floor.

        The CLI seeds its RNG from the wall clock, so two separate processes draw
        independent sequences; their PSNR is the noise floor of "the same
        distribution rendered twice". T2/T3 judge a new sampler against exactly
        this floor.
        """
        dir_a = tempfile.mkdtemp(prefix=f"lumice_conv_{config_name}_a_")
        dir_b = tempfile.mkdtemp(prefix=f"lumice_conv_{config_name}_b_")
        try:
            img_a = self._render(config_name, dir_a)
            img_b = self._render(config_name, dir_b)

            psnr_self = compute_psnr(compute_mse(img_a, img_b))
            self.assertGreaterEqual(
                psnr_self, floor,
                f"{config_name}: self-consistency PSNR {psnr_self:.1f} dB "
                f"< floor {floor} dB — renders are noisier than the "
                f"calibrated Monte-Carlo floor (rendering regression?).",
            )
        finally:
            shutil.rmtree(dir_a, ignore_errors=True)
            shutil.rmtree(dir_b, ignore_errors=True)


def _make_self_consistency_test(config_name: str, floor: float):
    @unittest.skipUnless(HAS_PILLOW, "Pillow not installed")
    def _test(self):
        self._check_self_consistency(config_name, floor)

    _test.__name__ = f"test_self_consistency_{config_name}"
    _test.__doc__ = (
        f"{config_name}: two independent renders agree within the "
        f"{floor} dB self-consistency floor."
    )
    return _test


# One pytest item per config, rather than one item looping over both: xdist
# distributes by item, so a single looping item pins every config to one worker
# and becomes the whole set's wall clock (doc/testing-architecture.md §7.0).
# `unittest.TestCase` methods cannot be driven by @pytest.mark.parametrize, hence
# the setattr generation below (same shape as test_smoke.py).
for _config_name, _floor in SELF_CONSISTENCY_FLOOR.items():
    setattr(
        TestIncidenceSamplingConvergence,
        f"test_self_consistency_{_config_name}",
        _make_self_consistency_test(_config_name, _floor),
    )


if __name__ == "__main__":
    unittest.main()
