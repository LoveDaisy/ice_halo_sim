"""CLI behavior tests for Lumice."""

import glob
import os
from pathlib import Path

from e2e.base import LumiceTestCase

CONFIGS_DIR = Path(__file__).resolve().parent / "configs"


class TestCli(LumiceTestCase):
    def test_help_flag(self):
        """Lumice -h should exit 0 and print usage."""
        result = self.run_lumice(["-h"])
        self.assertEqual(result.returncode, 0)
        self.assertIn("Usage:", result.stdout)

    def test_no_args(self):
        """Lumice with no arguments should exit non-zero."""
        result = self.run_lumice([])
        self.assertNotEqual(result.returncode, 0)

    def test_output_directory(self):
        """Lumice -o should write images to the specified directory."""
        cfg = CONFIGS_DIR / "halo_22.json"
        if not cfg.exists():
            self.skipTest("halo_22.json not found")

        result = self.run_lumice(["-f", str(cfg), "-o", self.output_dir])
        self.assertEqual(result.returncode, 0)

        output_imgs = glob.glob(os.path.join(self.output_dir, "img_*.jpg"))
        self.assertTrue(
            len(output_imgs) > 0,
            f"No images found in {self.output_dir}",
        )
