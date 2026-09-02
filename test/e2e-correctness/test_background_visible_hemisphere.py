"""End-to-end pin for the `visible` half of the background mask.

`render[].background` is only added where the lens images a piece of sky the `visible` setting
admits. That gate has two independent halves, and the sibling fixture next door
(test_background_domain_mask.py) deliberately exercises only one of them: it points the camera at
the zenith, which makes the image circle and the visible hemisphere the SAME set, so it isolates
the *domain* half — "outside the lens's image circle" — and says nothing about the *visibility*
half. Nothing at this layer covered the other one; the per-lens comparison of the two predicates is
a unit-level test (test/unit-correctness/gui/test_visible_mask_gui_parity.cpp), and this is the one
config in the e2e tree that renders anything other than `"visible": "full"`.

This fixture puts the camera ON the horizon instead. A 180 deg equal-area fisheye then splits the
frame into three regions that are all present in one image:

  A. outside the image circle          -> nothing is imaged      -> black
  B. inside it, below the horizon      -> `visible: upper` cuts it -> black
  C. inside it, above the horizon      -> sky                    -> the authored background

Geometry, all of it derived rather than observed. `ComputeLensScale`'s equal-area branch gives
scale = (short_edge / 2) / (sqrt(2) * sin(fov/4)), which at fov=180 on a 400x300 canvas is exactly
150. The core mask's domain guard (`FisheyeEqualAreaInverse`) rejects normalised r beyond the
lens's rim, and for equal-area that rim is sqrt(2) -- theta = 180 deg -- so the imaged disc reaches
212.1 px and the frame corners at 250 px are outside it. Region A is sampled beyond 230 px and
region B/C inside 140 px, both with margin, so no probe sits on the boundary.

That rim used to be 150 px, not 212.1: core stopped at the equator for every single-lens fisheye
while the GUI preview kept inverting out to theta = 180 deg, and the 150..212 px annulus was a
pinned CLI/GUI divergence. 474.1 closed it by taking the cull per lens type, which moved region A's
inner bound from 170 px to 230 px -- the numbers here are the visible consequence of that change,
not a relaxation of the assertion.

Two renders come out of ONE run, differing only in `visible`, so region B has a paired positive
control from the same rays: it must be background under `full` and black under `upper`. Without
that pair, a render that had simply stopped painting a background anywhere would pass.

Region B is read twice, and the two readings answer different questions. The MINIMUM says the
background constant was never added there: since the background is one constant added to every
masked pixel, and energy only ever raises individual pixels, a floor at the authored triple is
the unambiguous signature of a painted background. The MAXIMUM says the display clip actually
discarded the halo energy that landed there. That second reading is new with 478.2: `visible` is a
display clip for all four lens families now, so the excluded region is dropped on the way to
pixels rather than never accumulated -- energy DOES land below the horizon (the raw XYZ buffer is
identical under `upper` and `full`, which is what test_render_consumer_visible_mask.cpp asserts at
the unit layer), and this fixture is where the two halves are seen composing end to end. Before
478.2 the maximum would have read 0 here for the wrong reason: this config's lens is a single
fisheye, the one family whose rays were culled outright.
"""

import json
import math
import os

from test.e2e.base import LumiceTestCase
from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import get_project_root

if HAS_PILLOW:
    from PIL import Image

CONFIG = (
    get_project_root()
    / "test"
    / "e2e"
    / "configs"
    / "background_visible_horizon_probe.json"
)

WIDTH, HEIGHT = 400, 300
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
# Equal-area fisheye at fov=180: the scale that normalises radius is short_edge / 2, and the core
# mask's inverse accepts out to the lens's rim, r = sqrt(2) (theta = 180 deg). See the module
# docstring's derivation, including what this number was before 474.1 and why it moved.
IMAGE_RADIUS = min(WIDTH, HEIGHT) / 2.0 * 2 ** 0.5  # 212.13 px

# Region bounds, in pixels. The two radii straddle IMAGE_RADIUS with margin either way (18 px
# outside, 72 px inside; the outer one is bounded by the 250 px frame corner, and region A still
# holds 1740 pixels), and the horizon rows are 15 px clear of CY, so nothing here is a boundary
# coin flip.
OUTSIDE_RADIUS = 230.0
INSIDE_RADIUS = 140.0
BELOW_FIRST_ROW = 165
ABOVE_LAST_ROW = 135

# Same truncation allowance, and the same reason, as test_background_color_contract.py: the uint8
# write is `static_cast<uint8_t>(rgb * 255)`, so 0.2 comes back as byte 50 rather than 51.
TOLERANCE_LSB = 1


def _regions():
    """The three regions as pixel lists. Pure geometry — no image is read here."""
    outside, below, above = [], [], []
    for y in range(HEIGHT):
        for x in range(WIDTH):
            r = math.hypot(x + 0.5 - CX, y + 0.5 - CY)
            if r > OUTSIDE_RADIUS:
                outside.append((x, y))
            elif r < INSIDE_RADIUS:
                if y >= BELOW_FIRST_ROW:
                    below.append((x, y))
                elif y <= ABOVE_LAST_ROW:
                    above.append((x, y))
    return outside, below, above


class TestBackgroundVisibleHemisphere(LumiceTestCase):
    def setUp(self):
        super().setUp()
        if not HAS_PILLOW:
            self.skipTest("Pillow not installed")
        if not CONFIG.exists():
            self.skipTest(f"{CONFIG} not found")
        self.outside, self.below, self.above = _regions()
        for name, region in (
            ("outside", self.outside),
            ("below", self.below),
            ("above", self.above),
        ):
            self.assertGreater(len(region), 1000, f"region {name} is too small to mean anything")

    def _render(self):
        """Render both entries of the config and return their pixel accessors, in order."""
        result = self.run_lumice(
            ["-f", str(CONFIG), "-o", self.output_dir, "--format", "png"]
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        out = []
        for n in (1, 2):
            path = os.path.join(self.output_dir, f"img_{n:02d}.png")
            self.assertTrue(os.path.exists(path), f"no img_{n:02d}.png in {self.output_dir}")
            with Image.open(path) as im:
                im = im.convert("RGB")
                self.assertEqual(im.size, (WIDTH, HEIGHT))
                out.append(im.load())
        return out

    @staticmethod
    def _floor(px, region):
        """Per-channel minimum over a region — what the background contributes, since halo energy
        only ever adds."""
        return [min(px[p][c] for p in region) for c in range(3)]

    @staticmethod
    def _expected():
        render = json.loads(CONFIG.read_text())["render"]
        assert render[0]["visible"] == "upper" and render[1]["visible"] == "full"
        assert render[0]["background"] == render[1]["background"]
        return [round(c * 255) for c in render[0]["background"]]

    def test_the_background_stops_at_the_horizon_and_at_the_image_circle(self):
        upper, full = self._render()
        expected = self._expected()

        # Region A, both renders: nothing is imaged there, so nothing may be written there —
        # neither background nor energy. This is the domain half, restated on a camera pose the
        # sibling fixture does not use.
        for tag, px in (("visible=upper", upper), ("visible=full", full)):
            worst = max(max(px[p]) for p in self.outside)
            self.assertEqual(
                worst,
                0,
                f"{tag}: the brightest channel outside the image circle "
                f"(r > {OUTSIDE_RADIUS} px, image circle {IMAGE_RADIUS} px) is {worst}, not 0. "
                f"Nothing is imaged there.",
            )

        # Region C, both renders: above the horizon is sky under either setting, and its floor is
        # the authored triple exactly — the same identity the color-space contract pins, read here
        # on a frame where two thirds of the pixels are masked.
        for tag, px in (("visible=upper", upper), ("visible=full", full)):
            floor = self._floor(px, self.above)
            for c in range(3):
                self.assertLessEqual(
                    abs(floor[c] - expected[c]),
                    TOLERANCE_LSB,
                    f"{tag}: channel {c} of the above-horizon floor is {floor[c]}, not the "
                    f"authored sRGB byte {expected[c]} (floor {floor}, expected {expected}).",
                )

        # Region B — the proposition this file exists for. Under `upper` the constant was never
        # added, so the floor is 0; under `full`, from the same rays and the same geometry, it is
        # the authored triple. The pair is what makes the first half attributable to `visible`
        # rather than to a background that stopped being painted at all.
        below_upper = self._floor(upper, self.below)
        self.assertEqual(
            below_upper,
            [0, 0, 0],
            f"visible=upper: the below-horizon floor is {below_upper}, not black. A floor at "
            f"{expected} means the background was painted into the hemisphere `visible` excludes; "
            f"the region is {len(self.below)} px, all of it at least "
            f"{BELOW_FIRST_ROW - CY:.0f} px below the horizon and inside the image circle.",
        )
        # The other half of the same proposition, and the one 478.2 made assertable: the energy
        # that DOES land below the horizon must be discarded on the way to pixels, not merely left
        # unlit. `_floor` cannot see this -- a single leaked pixel never moves a minimum.
        below_upper_peak = max(max(upper[p]) for p in self.below)
        self.assertEqual(
            below_upper_peak,
            0,
            f"visible=upper: the brightest channel below the horizon is {below_upper_peak}, not 0. "
            f"`visible` is a display clip: the rays still land there (the raw XYZ buffer is the "
            f"same under `upper` and `full`), so every one of those {len(self.below)} px must be "
            f"discarded when the image is composed.",
        )

        below_full = self._floor(full, self.below)
        for c in range(3):
            self.assertLessEqual(
                abs(below_full[c] - expected[c]),
                TOLERANCE_LSB,
                f"visible=full: channel {c} of the below-horizon floor is {below_full[c]}, not "
                f"the authored sRGB byte {expected[c]} (floor {below_full}, expected {expected}). "
                f"Without this control the assertion above would also pass on a render that had "
                f"stopped painting a background anywhere.",
            )

    def test_the_kept_hemisphere_is_the_one_the_sun_is_in(self):
        """Which half survives is checked against the physics, not against the mask's own
        arithmetic. The light source sits at altitude +20 deg — above the horizon — so the halo it
        makes is above the horizon too, and the half that keeps its background must therefore be
        the half the brightest pixel is in. Halo energy is not masked (see the module docstring),
        so the two are measured independently and only then compared: an upper/lower swap moves the
        background to the other side of the frame while leaving the halo exactly where it was.
        """
        upper, _ = self._render()
        expected = self._expected()
        altitude = json.loads(CONFIG.read_text())["scene"]["light_source"]["altitude"]
        self.assertGreater(altitude, 0.0, "the fixture's sun must be above the horizon")

        def carries_background(region):
            floor = self._floor(upper, region)
            return all(abs(floor[c] - expected[c]) <= TOLERANCE_LSB for c in range(3))

        painted_above = carries_background(self.above)
        painted_below = carries_background(self.below)
        self.assertNotEqual(
            painted_above,
            painted_below,
            f"under `visible: upper` exactly one hemisphere may carry the background, but "
            f"above={painted_above} below={painted_below}",
        )

        peak_sum, peak_row = max(
            ((sum(upper[x, y]), y) for y in range(HEIGHT) for x in range(WIDTH))
        )
        self.assertGreater(peak_sum, 3 * max(expected), "frame looks empty; no halo to locate")
        peak_is_above = peak_row < CY

        self.assertEqual(
            painted_above,
            peak_is_above,
            f"the background is in the {'upper' if painted_above else 'lower'} half while the "
            f"brightest pixel (row {peak_row}, horizon row {CY:.0f}) is in the "
            f"{'upper' if peak_is_above else 'lower'} one, and the sun is {altitude} deg ABOVE the "
            f"horizon. The kept hemisphere and the lit one have come apart — `visible: upper` is "
            f"keeping the wrong half.",
        )
