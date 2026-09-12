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
   there, not to stderr) parsed for the online allocation's final report,
   ``RayAllocationOnline(final): layer L entry E: p=... q=... rays=...`` (one
   line per entry, written by the server's Stop once no worker can move the
   statistic), which must show a sampling share moved well away from the energy
   share. This is the only signal of the online q that crosses the process
   boundary: the loop's numbers have direct C++ coverage
   (test_ray_allocation_online_commit.cpp, test_ray_allocation_backends.cpp,
   test_simulator.cpp), but the CLI exposes no programmatic way to read q, so a
   log-text contract is the price of testing it end to end. The workers also
   write ``RayAllocationOnline: layer ...`` milestone lines while the run goes
   (each doubling of the first layer's dealt count); the final line is the one
   parsed because it is the converged value, not a snapshot on the way.

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
Online q on that scene at the end of the run (measured by the render itself, so
it carries the run's own sampling noise; 10,000,000 rays, legacy CPU path, 3 runs
on 2026-09-12): p = 100/50/10 -> q = 0.6353/0.302/0.0627 on every run (shares
0.6250/0.3125/0.0625 -> at most 3.4 % relative movement), i.e. the "q ≈ p"
regime the scene was built for -- and the same values, to the third digit, that
the pilot it replaced reported (0.6354/0.3019/0.06276).

Mechanism threshold: the skewed scene's final lines read p=100 q=0.016-0.020
(share 0.990 -> 0.02) and p=1 q=0.980-0.984 (share 0.0099 -> 0.98) over 3 runs
on 2026-09-12, at its 10,000 rays; the largest relative share movement is ~98×,
and it is already there after the first few thousand rays (the cold start deals
the two entries 50/50, the first batch's tally moves it). The assertion asks for
0.25 (25 %): far below what the scene delivers, far above the 3.4 % the q ≈ p
control scene shows, so it cannot be satisfied by noise on a loop that did
nothing.
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

# The online allocation's final per-entry report line (LogRayAllocationState,
# core/simulator.cpp, written by ServerImpl::Stop; q is the layer's share):
#   RayAllocationOnline(final): layer 0 entry 1: p=1 q=0.9799 rays=1980 sum_w=... sum_w2=...
FINAL_ENTRY_RE = re.compile(
    r"RayAllocationOnline\(final\): layer (?P<layer>\d+) entry (?P<entry>\d+): "
    r"p=(?P<p>[0-9.eE+-]+) q=(?P<q>[0-9.eE+-]+) rays=(?P<rays>\d+)"
)
# Any line of the online loop at all -- milestone or final.
ONLINE_MARKER = "RayAllocationOnline"


def parse_final_entries(log_text: str):
    """Return {(layer, entry): (p, q)} for every final report line in `log_text`."""
    out = {}
    for m in FINAL_ENTRY_RE.finditer(log_text):
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

    def test_proportional_keeps_no_online_tally(self):
        """Negative control for the mechanism test: proportional emits no online line."""
        result = self.renders[PROPORTIONAL]
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn(
            ONLINE_MARKER, result.stdout,
            "the proportional arm kept an online ray-allocation tally; the default mode "
            "must stay the pre-allocation path bit for bit",
        )

    def test_adaptive_online_q_moves_a_share_on_the_skewed_scene(self):
        """The online loop ran on a render commit and dealt a share visibly away from p.

        Guards the silent-fallback shape: an adaptive layer given no snapshot
        deals by p with every correction 1.0f and passes the PSNR test above
        with room to spare.
        """
        result = self.renders[SKEWED_ADAPTIVE]
        self.assertEqual(
            result.returncode, 0,
            f"{SKEWED_ADAPTIVE} failed:\nstdout: {result.stdout}\nstderr: {result.stderr}",
        )
        self.assertIn(
            "RayAllocationOnline(final)", result.stdout,
            "no RayAllocationOnline(final) line in stdout: the online loop did not run on "
            "a render commit with scene.ray_allocation = adaptive (or its ILOG_INFO "
            "wording changed -- update FINAL_ENTRY_RE together with the source)",
        )
        entries = parse_final_entries(result.stdout)
        layer0 = {k: v for k, v in entries.items() if k[0] == 0}
        self.assertEqual(
            len(layer0), 2,
            f"expected 2 final entry lines for layer 0, parsed {entries} from:\n{result.stdout}",
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
            f"the online loop left q ≈ p on a scene built to skew it (relative share "
            f"moves {moves}); adaptive dealing is not taking effect -- see "
            f"RayAllocationOnline / ComputeAdaptiveRayAllocationWeights and the "
            f"CommitConfig binding of active_ray_alloc_",
        )
