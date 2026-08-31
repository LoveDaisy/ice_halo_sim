"""End-to-end pin for the background color-space contract.

The `render[].background` JSON key is authored in sRGB — the numbers a color picker shows — while
the struct field it lands in is linear, because the background is added to the halo's radiance
before the sRGB transfer curve is applied. Both JSON parsers convert at their boundary.

What that buys the user is one identity, and this is the test of it: on a pixel carrying no halo
energy, the rendered byte is the sRGB triple written in the config, because
`LinearToSrgb(SrgbToLinear(x)) == x`. Before the conversion existed the same config rendered
`LinearToSrgb(0.2) = 0.4845` → byte 123 where it now renders 51 — so this pin separates the two
readings by ~70 LSB, far outside the 1 LSB tolerance explained below.

The parser-level unit tests (test_json_parser_parity.cpp) pin the conversion at each boundary; this
one pins the property those conversions exist for, through the real CLI and the real render path.

The "on a pixel carrying no halo energy" reading depends on the background reaching every pixel of
the frame, and it no longer does everywhere: the render path adds it only inside the region the
lens images and the `visible` setting admits (RenderConsumer::PostSnapshot, gated on
core/lens_proj_build.hpp's BuildVisibleMask). The fixture is therefore chosen so the whole 64x64
frame is inside that region — `"visible": "full"` removes the horizon cut, and fov=90 puts the
equal-area image circle at 1.31 frame half-diagonals, leaving the far corners at 75% of the
boundary rather than the coin flip fov=120 produced (its corner pixels land at r = 0.98). Both
premises are asserted below rather than assumed, because if either lapsed the frame minimum would
quietly become a masked black pixel and this test would keep passing while measuring nothing.
"""

import json
import os
from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from PIL import Image

CONFIG = get_project_root() / "test" / "e2e" / "configs" / "background_srgb_contract.json"

# The uint8 write is a TRUNCATION, not a rounding: render.cpp does
# `static_cast<uint8_t>(rgb[j] * 255)`. A channel whose float32 round trip lands a hair below its
# authored value therefore loses a whole byte — measured here, 0.2 comes back as 0.19999999, so
# 0.19999999 * 255 = 50.999996 truncates to 50 where round(0.2 * 255) is 51. The other two channels
# (0.35 → 89, 0.6 → 153) land exactly. This is the pipeline's pre-existing truncation behaviour,
# the same one already documented for LinearToSrgb(1.0f) = 0.99999994 → 254; it is not something
# this contract introduces, so the pin allows it rather than asserting a value that would encode
# one platform's float32 rounding as the contract.
TOLERANCE_LSB = 1


class TestBackgroundColorContract(LumiceTestCase):
    def setUp(self):
        super().setUp()
        if not HAS_PILLOW:
            self.skipTest("Pillow not installed")
        if not CONFIG.exists():
            self.skipTest(f"{CONFIG} not found")

    def test_background_pixels_match_the_authored_srgb_triple(self):
        result = self.run_lumice(
            ["-f", str(CONFIG), "-o", self.output_dir, "--format", "png"]
        )
        self.assertEqual(result.returncode, 0, result.stderr)

        img_path = os.path.join(self.output_dir, "img_01.png")
        self.assertTrue(os.path.exists(img_path), f"no img_01.png in {self.output_dir}")

        with Image.open(img_path) as im:
            pixels = list(im.convert("RGB").getdata())

        expected = [
            round(c * 255)
            for c in json.loads(CONFIG.read_text())["render"][0]["background"]
        ]

        # Premise of the "whole frame floor" reading below: nothing here is outside the imaged
        # region. A masked pixel is pure black, so counting black pixels tests it directly.
        black = sum(1 for p in pixels if p == (0, 0, 0))
        self.assertEqual(
            black,
            0,
            f"{black} of {len(pixels)} pixels are pure black, i.e. outside the lens's imaged region or the "
            f"visible hemisphere. The frame-minimum reading below only means 'the background floor' while "
            f"the background reaches every pixel; re-pick the fixture rather than relaxing this.",
        )

        # The per-channel minimum over the frame IS the background floor: the background is added
        # to every pixel and halo energy only ever adds more, so the darkest value a channel takes
        # is the one where no ray landed. Taking the minimum rather than a fixed corner pixel keeps
        # the pin independent of where the (stochastic) halo happens to fall.
        floor = [min(p[c] for p in pixels) for c in range(3)]
        for c in range(3):
            self.assertLessEqual(
                abs(floor[c] - expected[c]),
                TOLERANCE_LSB,
                f"channel {c}: background floor {floor[c]} is not the authored sRGB byte "
                f"{expected[c]} (whole frame floor {floor}, expected {expected}). "
                f"A floor near {round(((expected[c] / 255) ** (1 / 2.2)) * 255)} would mean the "
                f"config value was taken as linear and gamma-encoded a second time.",
            )

        # Guard against the pin passing on a frame that is nothing but background: if the render
        # produced no halo at all, the floor above would still match and the test would be vacuous.
        peak = max(max(p) for p in pixels)
        self.assertGreater(
            peak,
            max(floor) + 32,
            f"frame looks empty (peak {peak}, floor {floor}); the pin needs a real render",
        )
