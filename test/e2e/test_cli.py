"""CLI behavior tests for Lumice."""

import glob
import os
from pathlib import Path

from test.e2e.base import LumiceTestCase

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


class TestOutputFormat(LumiceTestCase):
    """Tests for --format and --quality CLI options."""

    def _get_config(self):
        cfg = CONFIGS_DIR / "halo_22.json"
        if not cfg.exists():
            self.skipTest("halo_22.json not found")
        return cfg

    def test_output_format_png(self):
        """--format png should produce .png files."""
        cfg = self._get_config()
        result = self.run_lumice(
            ["-f", str(cfg), "-o", self.output_dir, "--format", "png"]
        )
        self.assertEqual(result.returncode, 0)

        output_imgs = glob.glob(os.path.join(self.output_dir, "img_*.png"))
        self.assertTrue(
            len(output_imgs) > 0,
            f"No PNG images found in {self.output_dir}",
        )

    def test_output_format_jpg_explicit(self):
        """--format jpg should produce .jpg files."""
        cfg = self._get_config()
        result = self.run_lumice(
            ["-f", str(cfg), "-o", self.output_dir, "--format", "jpg"]
        )
        self.assertEqual(result.returncode, 0)

        output_imgs = glob.glob(os.path.join(self.output_dir, "img_*.jpg"))
        self.assertTrue(
            len(output_imgs) > 0,
            f"No JPEG images found in {self.output_dir}",
        )

    def test_jpeg_quality(self):
        """--quality should affect JPEG file size (quality 1 < quality 95)."""
        cfg = self._get_config()

        # Run with quality 95 (default)
        dir_q95 = os.path.join(self.output_dir, "q95")
        os.makedirs(dir_q95)
        result = self.run_lumice(
            ["-f", str(cfg), "-o", dir_q95, "--quality", "95"]
        )
        self.assertEqual(result.returncode, 0)

        # Run with quality 1
        dir_q1 = os.path.join(self.output_dir, "q1")
        os.makedirs(dir_q1)
        result = self.run_lumice(
            ["-f", str(cfg), "-o", dir_q1, "--quality", "1"]
        )
        self.assertEqual(result.returncode, 0)

        imgs_q95 = sorted(glob.glob(os.path.join(dir_q95, "img_*.jpg")))
        imgs_q1 = sorted(glob.glob(os.path.join(dir_q1, "img_*.jpg")))
        self.assertTrue(len(imgs_q95) > 0, "No quality-95 images")
        self.assertEqual(len(imgs_q95), len(imgs_q1))

        for f95, f1 in zip(imgs_q95, imgs_q1):
            size_q95 = os.path.getsize(f95)
            size_q1 = os.path.getsize(f1)
            self.assertGreater(
                size_q95, size_q1,
                f"Expected quality 95 ({size_q95}B) > quality 1 ({size_q1}B)",
            )

    def test_invalid_format(self):
        """--format with unsupported value should exit non-zero."""
        result = self.run_lumice(
            ["-f", "dummy.json", "--format", "bmp"]
        )
        self.assertNotEqual(result.returncode, 0)

    def test_invalid_quality_range(self):
        """--quality outside [1, 100] should exit non-zero."""
        for val in ["0", "101"]:
            result = self.run_lumice(
                ["-f", "dummy.json", "--quality", val]
            )
            self.assertNotEqual(
                result.returncode, 0,
                f"--quality {val} should be rejected",
            )

    def test_invalid_quality_non_numeric(self):
        """--quality with non-numeric value should exit non-zero."""
        result = self.run_lumice(
            ["-f", "dummy.json", "--quality", "abc"]
        )
        self.assertNotEqual(result.returncode, 0)

    def test_quality_missing_value(self):
        """--quality as last argument (missing value) should exit non-zero."""
        result = self.run_lumice(
            ["-f", "dummy.json", "--quality"]
        )
        self.assertNotEqual(result.returncode, 0)
