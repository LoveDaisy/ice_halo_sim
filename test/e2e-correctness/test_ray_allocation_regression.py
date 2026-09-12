"""E2E test: scene.ray_allocation = "adaptive" is a variance change, not an image change.

Two propositions, each on its own scene, because they pull in opposite
directions and one scene cannot serve both (see the module constants):

1. Zero expected-image regression (the PSNR pair). A three-crystal, no-filter
   scene with proportions 100/50/10 -- "ordinary" per-ray energy statistics, so
   the Neyman deal q_i ∝ p_i·√E[e²] stays close to p_i -- rendered once under
   ``"proportional"`` and once under ``"adaptive"``. The two frames must agree to
   at least the PSNR a proportional frame agrees with another proportional frame:
   the only thing adaptive is allowed to move is the noise distribution.

2. The mechanism actually ran (the q ≠ p assertion). A PSNR floor alone cannot
   tell "adaptive worked" from "adaptive silently fell back to proportional" --
   the fallback passes it *more* easily -- and this tree has had exactly that
   failure shape (an analysis session's adaptive layers deal by p by design; a
   render commit's must not). So a second, deliberately skewed scene -- a
   filtered high-proportion entry whose rays are individually dim next to an
   unfiltered ``proportion: 1`` entry whose rays each carry far more energy --
   is rendered under ``"adaptive"`` and its stdout (the CLI's logger sink writes
   there, not to stderr) parsed for the pilot's
   ``RayAllocationPilot: layer L entry E: p=... q=...`` lines, which must show a
   sampling share moved well away from the energy share. This is the only signal
   of the pilot that crosses the process boundary: the pilot's numbers have
   direct C++ coverage (test_ray_allocation_pilot_commit.cpp,
   test_ray_allocation_backends.cpp), but the CLI exposes no programmatic way to
   read q, so a log-text contract is the price of testing it end to end.

The same mechanism scene is not used for (1): on it q is nowhere near p, so a
PSNR pairing would measure a large, legitimate change in noise distribution and
say nothing about expected-image regression.

PSNR threshold: calibrated by rendering each arm 3 times (the CLI has no seed
argument, so runs differ naturally), taking min over the 3×3 cross-mode pairs,
and placing the threshold about 1 dB under it, floored to 0.5 dB -- the
`mean − max(4σ, 1.0 dB)` shape the GUI reference groups use, not
test_raypath_equivalence.py's 3 dB margin: at 3 dB this ruler was measured to
let a halving of the main crystal's share through (34.0 dB against a 33.0 dB
floor at 5,000,000 rays), i.e. it could not see the regression it is for.
Calibration on 2026-09-12 (128×128, 10,000,000 rays, legacy CPU path):
  proportional × adaptive : [39.82, 39.73, 39.81, 39.97, 39.77, 39.95, 39.89, 39.82, 39.82] dB
  proportional × proportional (noise floor): [39.73, 39.76, 39.90] dB
  adaptive × adaptive (noise floor):         [39.85, 39.86, 39.78] dB
  min cross-mode = 39.73 dB → threshold 38.5 dB.
The cross-mode pairs sit inside the within-mode band, which is the claim.
Red / green samples for this ruler, same day, deliberately broken configs
compared against the 3 proportional frames:
  main crystal's proportion 100 → 50 (a 2× share error): 35.88–35.95 dB → RED.
  the 10 % crystal's proportion 10 → 0 (dropped outright): 39.22–39.31 dB → still
  GREEN -- an error confined to a 10 % share sits under this ruler's resolution at
  this ray budget (doubling the rays was measured to buy ~0.5 dB more, not enough
  to change that). That is why the mechanism test below exists: this PSNR pair
  answers "is the expected image unchanged to within MC noise", not "did adaptive
  do anything".
Pilot output on that scene (deterministic -- the pilot is seeded; measured at
5,000,000 rays, and the pilot's own budget does not depend on ray_num):
  p = 100/50/10 → q = 0.6354/0.3019/0.06276 (shares 0.6250/0.3125/0.0625 → at most
  3.4 % relative movement), i.e. the "q ≈ p" regime the scene was built for.

Mechanism threshold: the skewed scene's pilot reads p=100 q=0.02011 (share
0.990 → 0.020) and p=1 q=0.9799 (share 0.0099 → 0.980) on 2026-09-12, identical
across 3 runs; the largest relative share movement is ~98×. The assertion asks
for 0.25 (25 %): far below what the scene delivers, far above the 3.4 % the
q ≈ p control scene shows, so it cannot be satisfied by noise on a pilot that
did nothing.
"""

import glob
import os
import re
import unittest

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from test.e2e.image_utils import compute_mse, compute_psnr

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# See the module docstring for the calibration behind both numbers.
ZERO_REGRESSION_PSNR_THRESHOLD = 38.5
MECHANISM_MIN_RELATIVE_SHARE_MOVE = 0.25

PROPORTIONAL = "ray_allocation_no_filter_proportional"
ADAPTIVE = "ray_allocation_no_filter_adaptive"
SKEWED_ADAPTIVE = "ray_allocation_skewed_adaptive"

# The pilot's per-entry report line (Simulator::RunRayAllocationPilot, ILOG_INFO):
#   RayAllocationPilot: layer 0 entry 1: p=1 q=0.9799 rays=1980 exits=11344
PILOT_ENTRY_RE = re.compile(
    r"RayAllocationPilot: layer (?P<layer>\d+) entry (?P<entry>\d+): "
    r"p=(?P<p>[0-9.eE+-]+) q=(?P<q>[0-9.eE+-]+) rays=(?P<rays>\d+) exits=(?P<exits>\d+)"
)
PILOT_RAN_MARKER = "RayAllocationPilot: ran"


def parse_pilot_entries(log_text: str):
    """Return {(layer, entry): (p, q)} for every pilot report line in `log_text`."""
    out = {}
    for m in PILOT_ENTRY_RE.finditer(log_text):
        key = (int(m.group("layer")), int(m.group("entry")))
        out[key] = (float(m.group("p")), float(m.group("q")))
    return out


class TestRayAllocationRegression(LumiceTestCase):
    """adaptive vs proportional: same expected image; and adaptive really dealt by q."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        for name in (PROPORTIONAL, ADAPTIVE, SKEWED_ADAPTIVE):
            cfg_path = CONFIGS_DIR / f"{name}.json"
            if not cfg_path.exists():
                raise unittest.SkipTest(f"Config not found: {cfg_path}")
        cls.renders = {
            name: cls.render_once(CONFIGS_DIR / f"{name}.json")
            for name in (PROPORTIONAL, ADAPTIVE, SKEWED_ADAPTIVE)
        }

    def _image_for(self, config_name: str) -> str:
        result = self.renders[config_name]
        self.assertEqual(
            result.returncode, 0,
            f"{config_name} failed:\nstdout: {result.stdout}\nstderr: {result.stderr}",
        )
        images = sorted(glob.glob(os.path.join(result.output_dir, "img_*.jpg")))
        self.assertTrue(len(images) > 0, f"No output images for {config_name}")
        return images[0]

    def test_proportional_runs_successfully(self):
        img = self._image_for(PROPORTIONAL)
        self.assertGreater(os.path.getsize(img), 0)

    def test_adaptive_runs_successfully(self):
        img = self._image_for(ADAPTIVE)
        self.assertGreater(os.path.getsize(img), 0)

    @unittest.skipUnless(HAS_PILLOW, "Pillow not installed")
    def test_adaptive_matches_proportional_psnr(self):
        """No-filter mixed scene: adaptive and proportional frames agree above threshold."""
        img_p = self._image_for(PROPORTIONAL)
        img_a = self._image_for(ADAPTIVE)
        psnr = compute_psnr(compute_mse(img_p, img_a))
        self.assertGreaterEqual(
            psnr,
            ZERO_REGRESSION_PSNR_THRESHOLD,
            f"ray_allocation zero-regression failed: PSNR(proportional, adaptive) = "
            f"{psnr:.2f} dB < threshold {ZERO_REGRESSION_PSNR_THRESHOLD} dB. adaptive "
            f"must change only the noise distribution, never the expected image -- "
            f"suspect the p_i/q_i weight correction (ComputeRayAllocationCorrection) "
            f"or the first-layer emitted_energy_ charge.",
        )

    def test_proportional_never_runs_the_pilot(self):
        """Negative control for the mechanism test: proportional emits no pilot line."""
        result = self.renders[PROPORTIONAL]
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn(
            PILOT_RAN_MARKER, result.stdout,
            "the proportional arm ran the ray-allocation pilot; the default mode must "
            "stay the pre-pilot path bit for bit",
        )

    def test_adaptive_pilot_moves_a_share_on_the_skewed_scene(self):
        """The pilot ran on a render commit and dealt a share visibly away from p.

        Guards the silent-fallback shape: an adaptive layer whose weights were
        never delivered deals by p with every correction 1.0f and passes the
        PSNR test above with room to spare.
        """
        result = self.renders[SKEWED_ADAPTIVE]
        self.assertEqual(
            result.returncode, 0,
            f"{SKEWED_ADAPTIVE} failed:\nstdout: {result.stdout}\nstderr: {result.stderr}",
        )
        self.assertIn(
            PILOT_RAN_MARKER, result.stdout,
            "no RayAllocationPilot summary line in stdout: the pilot did not run on a "
            "render commit with scene.ray_allocation = adaptive (or its ILOG_INFO "
            "wording changed -- update PILOT_ENTRY_RE together with the source)",
        )
        entries = parse_pilot_entries(result.stdout)
        layer0 = {k: v for k, v in entries.items() if k[0] == 0}
        self.assertEqual(
            len(layer0), 2,
            f"expected 2 pilot entry lines for layer 0, parsed {entries} from:\n{result.stdout}",
        )
        total_p = sum(p for p, _ in layer0.values())
        total_q = sum(q for _, q in layer0.values())
        self.assertGreater(total_p, 0.0)
        self.assertGreater(total_q, 0.0)
        moves = {
            k: abs(q / total_q - p / total_p) / (p / total_p) for k, (p, q) in layer0.items()
        }
        self.assertGreaterEqual(
            max(moves.values()),
            MECHANISM_MIN_RELATIVE_SHARE_MOVE,
            f"the pilot delivered q ≈ p on a scene built to skew it (relative share "
            f"moves {moves}); adaptive dealing is not taking effect -- see "
            f"ComputeAdaptiveRayAllocationWeights and the CommitConfig pilot call",
        )
