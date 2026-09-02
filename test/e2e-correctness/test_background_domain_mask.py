"""End-to-end pin for the core-side projection-domain mask on the background fill.

`render[].background` is added to every pixel of the rendered frame. "Every pixel" is only
correct where the lens actually images something: a 180 deg equal-area fisheye on a
non-square canvas leaves the frame corners outside the image circle, and a `visible`
setting other than `full` leaves the far hemisphere unimaged. Those pixels carry no sky,
so they must stay black rather than being painted the sky background — otherwise a 180 deg
fisheye render is a solid rectangle of background with an invisible circle inside it, and
the lens boundary the GUI draws has no counterpart in the CLI image.

The config renders a 180 deg equal-area fisheye at 400x300 pointing at the zenith. Radius
normalises by `min(400, 300) / 2 = 150` px, and the lens images out to its rim at
r = sqrt(2), so the imaged disc has radius 212.1 px centred on the frame; the frame corners
sit at ~246 px from the centre, i.e. outside it (r = 1.64 against a rim of 1.414, not a
boundary coin flip). Pointing at the zenith with `visible: upper` makes the imaged disc and
the visible hemisphere the same set, so this pin isolates the *domain* half of the mask from
the *visibility* half.

The 150 px in that first sentence used to be the whole story: before 474.1 core stopped at
the equator (r = 1) for every single-lens fisheye. Widening the cull per lens type moved the
disc out to r = sqrt(2) without moving this fixture's verdict, because its corner probes were
already outside the wider disc too.

Both halves are asserted, and the second is what keeps the first honest: the corners must
be black AND the pixels inside the circle must still carry the authored background, so a
mask that simply painted nothing would fail too.
"""

import json
import os
from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from PIL import Image

CONFIG = (
    get_project_root() / "test" / "e2e" / "configs" / "background_domain_mask_probe.json"
)

WIDTH, HEIGHT = 400, 300
# Imaged-disc radius in pixels: an equal-area fisheye at fov=180 maps theta=90 deg onto
# r = short_edge / 2 (see ComputeLensScale's kFisheyeEqualArea branch) and reaches its rim,
# theta=180 deg, at sqrt(2) times that.
IMAGE_RADIUS = min(WIDTH, HEIGHT) / 2.0 * 2 ** 0.5  # 212.13 px

# Pixels sampled well outside the image circle (distance from centre, in units of
# IMAGE_RADIUS, in parentheses).
OUTSIDE_PIXELS = [(2, 2), (WIDTH - 3, 2), (2, HEIGHT - 3), (WIDTH - 3, HEIGHT - 3)]
# Pixels sampled well inside it — dead centre plus four points at ~2/3 radius.
INSIDE_PIXELS = [(200, 150), (200, 50), (200, 250), (100, 150), (300, 150)]

# Same truncation allowance as test_background_color_contract.py: the uint8 write is
# `static_cast<uint8_t>(rgb * 255)`, a truncation, so a channel whose float32 round trip
# lands a hair low loses a whole byte (0.2 -> 50 rather than 51).
TOLERANCE_LSB = 1


class TestBackgroundDomainMask(LumiceTestCase):
    def setUp(self):
        super().setUp()
        if not HAS_PILLOW:
            self.skipTest("Pillow not installed")
        if not CONFIG.exists():
            self.skipTest(f"{CONFIG} not found")

    def _render(self):
        result = self.run_lumice(
            ["-f", str(CONFIG), "-o", self.output_dir, "--format", "png"]
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        img_path = os.path.join(self.output_dir, "img_01.png")
        self.assertTrue(os.path.exists(img_path), f"no img_01.png in {self.output_dir}")
        with Image.open(img_path) as im:
            im = im.convert("RGB")
            self.assertEqual(im.size, (WIDTH, HEIGHT))
            return im.load()

    def test_pixels_outside_the_image_circle_stay_black(self):
        px = self._render()
        for x, y in OUTSIDE_PIXELS:
            dist = ((x + 0.5 - WIDTH / 2.0) ** 2 + (y + 0.5 - HEIGHT / 2.0) ** 2) ** 0.5
            self.assertGreater(dist, IMAGE_RADIUS, f"sample ({x},{y}) is not outside the circle")
            self.assertEqual(
                px[x, y],
                (0, 0, 0),
                f"pixel ({x},{y}) is {dist / IMAGE_RADIUS:.2f} image-circle radii from the "
                f"centre — outside the lens's domain, so nothing is imaged there — yet it "
                f"carries {px[x, y]}. The background must only be added inside the imaged "
                f"region.",
            )

    def test_pixels_inside_the_image_circle_still_carry_the_background(self):
        px = self._render()
        expected = tuple(
            round(c * 255)
            for c in json.loads(CONFIG.read_text())["render"][0]["background"]
        )
        for x, y in INSIDE_PIXELS:
            dist = ((x + 0.5 - WIDTH / 2.0) ** 2 + (y + 0.5 - HEIGHT / 2.0) ** 2) ** 0.5
            self.assertLess(dist, IMAGE_RADIUS, f"sample ({x},{y}) is not inside the circle")
            got = px[x, y]
            for c in range(3):
                # >= rather than ==: halo energy may have landed on this pixel, and it only
                # ever adds. The floor is what the background contributes.
                self.assertGreaterEqual(
                    got[c] + TOLERANCE_LSB,
                    expected[c],
                    f"pixel ({x},{y}) channel {c} is {got[c]}, below the authored background "
                    f"floor {expected[c]} (whole pixel {got}, expected floor {expected}). "
                    f"This pixel is inside the image circle and must be painted.",
                )
