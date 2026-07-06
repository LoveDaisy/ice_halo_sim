"""E2E test (AC1, task-composition-editor-ui / 333.4): the GUI filter editor's
sum-of-products expansion is semantically equivalent to expressing the same
compound filter directly in core.

The GUI editor lets a user build a cross-type, AND-bearing filter as a
sum-of-products (SoP): (raypath 3-5) OR (entry:2 & exit:4) OR (3-5 & entry:2).
On save/export, src/gui/file_io.cpp's ExpandSopToClauses + SerializeFilterForCore
lower that SoP to a core `complex` filter: one simple filter per (clause, term)
with NO dedup, wrapped by a complex whose composition is [[1],[2],[3,4]].

What THIS test mechanically proves: the exact lowered form the GUI emits for that
SoP — [[1],[2],[3,4]] over 4 simple filters, no dedup (fixture A) — drives the
simulator to the *same halo* as the identical predicate authored idiomatically by
hand in core — [[1],[2],[1,3]] over 3 simple filters, deduped (fixture B). The two
configs share every other field byte-for-byte, so a render-compare isolates the
filter encoding: it confirms the no-dedup expansion the GUI produces is a
gate-equivalent (not merely structurally-similar) way to express the compound
predicate. Fixture A is a hand-checked transcription of ExpandSopToClauses'
output — this test does not itself re-run the GUI lowering.

Layered fidelity (the full AC1 chain, transitively): that the GUI editor actually
lowers the three rows to fixture A's structure, and round-trips the SoP losslessly
through .lmc, is guarded by the GUI tests p2_filter_type/multi_row_commits_sop and
p2_filter_type/sop_roundtrip_via_gui_editor (same three-row scenario), the
struct-vs-JSON expansion by the unit test filter_expand_struct_vs_json, and the
gate-equivalence of the emitted encoding by THIS test. Together: GUI edit → save/
reload → the simulator applies the compound predicate correctly.

PSNR threshold: calibrated 2026-07-06 over 18 back-to-back A/B render pairs
(2M rays, 512x256) — 17/18 pixel-identical (PSNR=inf), worst finite = 53.31 dB.
Threshold 40.0 dB leaves a wide margin below the worst observation while still
asserting the images are visually identical (mean per-channel error < ~2.5/255).
Because A and B are the *same* predicate, any residual gap is pure Monte-Carlo /
thread-scheduling noise, not a semantic difference.
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

# See module docstring for the calibration record.
EQUIVALENCE_PSNR_THRESHOLD = 40.0

SOP_EXPORT_CONFIG = "composition_sop_gui_export.json"
CORE_DIRECT_CONFIG = "composition_core_direct.json"


class TestCompositionEquivalence(LumiceTestCase):
    """GUI SoP export ≡ hand-authored core complex filter (AC1)."""

    def _run_config_and_get_image(self, config_name: str) -> str:
        cfg_path = CONFIGS_DIR / config_name
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

    def test_sop_gui_export_runs_successfully(self):
        """The GUI-export SoP config must render (exit 0, non-empty image)."""
        img = self._run_config_and_get_image(SOP_EXPORT_CONFIG)
        self.assertGreater(os.path.getsize(img), 0)

    def test_core_direct_runs_successfully(self):
        """The hand-authored core-direct config must render too."""
        img = self._run_config_and_get_image(CORE_DIRECT_CONFIG)
        self.assertGreater(os.path.getsize(img), 0)

    @unittest.skipUnless(HAS_PILLOW, "Pillow not installed")
    def test_gui_sop_equals_core_direct_psnr(self):
        """The GUI SoP export and the idiomatic core filter must render alike.

        (raypath 3-5) OR (entry:2 & exit:4) OR (3-5 & entry:2), lowered by the GUI
        as [[1],[2],[3,4]] (4 simple filters, no dedup) vs authored directly in core
        as [[1],[2],[1,3]] (3 simple filters, deduped). Same compound gate → same
        halo. A failure here means the GUI's SoP expansion diverged semantically
        from the core predicate (e.g. an ExpandSopToClauses regression).
        """
        dir_a = tempfile.mkdtemp(prefix="lumice_sop_")
        dir_b = tempfile.mkdtemp(prefix="lumice_core_")
        orig_dir = self.output_dir
        try:
            self.output_dir = dir_a
            img_a = self._run_config_and_get_image(SOP_EXPORT_CONFIG)

            self.output_dir = dir_b
            img_b = self._run_config_and_get_image(CORE_DIRECT_CONFIG)

            mse = compute_mse(img_a, img_b)
            psnr = compute_psnr(mse)
            self.assertGreaterEqual(
                psnr,
                EQUIVALENCE_PSNR_THRESHOLD,
                f"GUI SoP export vs core-direct filter diverged: "
                f"PSNR = {psnr:.1f} dB < threshold {EQUIVALENCE_PSNR_THRESHOLD} dB. "
                f"The GUI editor's sum-of-products expansion "
                f"(ExpandSopToClauses/SerializeFilterForCore) is no longer "
                f"semantically equivalent to the same filter authored directly in "
                f"core's ComplexFilterParam — a compound-predicate regression.",
            )
        finally:
            self.output_dir = orig_dir
            shutil.rmtree(dir_a, ignore_errors=True)
            shutil.rmtree(dir_b, ignore_errors=True)
