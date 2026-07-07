"""Self-check for image_utils.classify_pixels_by_color_direction.

The classifier is a white-box tool that relies on the compositor's
dominant/painter mode direction invariant (`out = color_c * ey`). Before
we trust its verdicts on real Lumice output (task-cli-regression-full
Step 3–4), the tool itself must pass a known-answer synthetic test:
compose a small image with hand-placed color blocks + noise, run the
classifier, and assert per-class pixel counts match the ground truth.

Rationale: a04 — a self-blessing classifier is worthless. If this test
fails, do NOT touch the caller (test_raypath_color) until the tool is
re-verified.
"""

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from test.e2e.image_utils import (
    HAS_PILLOW,
    classify_pixels_by_color_direction,
)

if HAS_PILLOW:
    from PIL import Image


@unittest.skipUnless(HAS_PILLOW, "Pillow not installed")
class TestClassifyPixelsByColorDirection(unittest.TestCase):
    """Ground-truth counts on synthetic images."""

    CLASS_COLORS = [
        (1.0, 0.0, 0.0),      # red
        (0.0, 1.0, 0.0),      # green
        (1.0, 0.0, 1.0),      # magenta (AND class)
        (1.0, 0.55, 0.0),     # orange (none-filter class)
    ]

    def _make_synthetic(self, path, w=40, h=20, block_counts=None,
                        exposure_scale=None, background=(0, 0, 0)):
        """Paint a WxH image with `block_counts[i]` pixels of class i (raw color).

        Optional `exposure_scale[i]` applies a per-class scalar (0<s<=1) to
        emulate `out = color_c * ey`; the direction is unchanged so the
        classifier must still recover the right class.
        """
        block_counts = block_counts or [0] * len(self.CLASS_COLORS)
        exposure_scale = exposure_scale or [1.0] * len(self.CLASS_COLORS)
        img = Image.new("RGB", (w, h), background)
        px = img.load()
        cursor = 0
        for idx, count in enumerate(block_counts):
            r, g, b = self.CLASS_COLORS[idx]
            s = exposure_scale[idx]
            rgb = (int(round(255 * r * s)), int(round(255 * g * s)), int(round(255 * b * s)))
            for _ in range(count):
                y = cursor // w
                x = cursor % w
                if y >= h:
                    raise ValueError("block_counts exceed image size")
                px[x, y] = rgb
                cursor += 1
        img.save(path, "PNG")
        return cursor

    def test_perfect_saturated_blocks(self):
        """Fully saturated (raw class color) blocks classify exactly."""
        with TemporaryDirectory() as td:
            path = Path(td) / "perfect.png"
            counts = [50, 40, 30, 20]  # total 140 lit; 40*20=800 bg
            written = self._make_synthetic(path, block_counts=counts)
            self.assertEqual(written, sum(counts))
            result = classify_pixels_by_color_direction(
                str(path), self.CLASS_COLORS
            )
            self.assertEqual(result["total"], 40 * 20)
            self.assertEqual(result["per_class"], counts)
            self.assertEqual(result["unclassified"], 0)
            self.assertEqual(result["background"], 40 * 20 - sum(counts))

    def test_exposure_scaled_blocks_classify_correctly(self):
        """Direction invariant: `out = color * ey` still classifies as color."""
        with TemporaryDirectory() as td:
            path = Path(td) / "exposed.png"
            counts = [50, 40, 30, 20]
            # Each class rendered at a different exposure — direction unchanged.
            scales = [0.9, 0.5, 0.3, 0.7]
            self._make_synthetic(path, block_counts=counts, exposure_scale=scales)
            result = classify_pixels_by_color_direction(
                str(path), self.CLASS_COLORS
            )
            self.assertEqual(result["per_class"], counts)
            self.assertEqual(result["unclassified"], 0)

    def test_dim_pixels_below_floor_are_background(self):
        """Pixels dimmer than luminance_floor count as background, not unclassified."""
        with TemporaryDirectory() as td:
            path = Path(td) / "dim.png"
            # Class-0 (red) rendered at very low exposure -> dominant channel < floor.
            self._make_synthetic(
                path, w=20, h=10,
                block_counts=[10, 0, 0, 0],
                exposure_scale=[0.02, 1.0, 1.0, 1.0],  # 0.02*255 ~ 5, below floor=8
            )
            result = classify_pixels_by_color_direction(
                str(path), self.CLASS_COLORS, luminance_floor=8.0
            )
            self.assertEqual(result["per_class"][0], 0)
            self.assertEqual(result["background"], 20 * 10)

    def test_phantom_hue_counts_as_unclassified(self):
        """A cyan pixel (equidistant from red/green/magenta) is unclassified."""
        with TemporaryDirectory() as td:
            path = Path(td) / "phantom.png"
            img = Image.new("RGB", (10, 10), (0, 0, 0))
            px = img.load()
            for i in range(5):
                px[i, 0] = (0, 200, 200)  # cyan — not any class direction
            img.save(path, "PNG")
            result = classify_pixels_by_color_direction(
                str(path), self.CLASS_COLORS
            )
            self.assertEqual(result["per_class"], [0, 0, 0, 0])
            self.assertEqual(result["unclassified"], 5)
            self.assertEqual(result["background"], 100 - 5)

    def test_zero_class_color_rejected(self):
        with TemporaryDirectory() as td:
            path = Path(td) / "img.png"
            Image.new("RGB", (2, 2), (0, 0, 0)).save(path, "PNG")
            with self.assertRaises(ValueError):
                classify_pixels_by_color_direction(str(path), [(0.0, 0.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
