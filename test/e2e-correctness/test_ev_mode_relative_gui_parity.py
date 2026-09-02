"""``ev_mode: relative`` renders what the GUI has always displayed.

The CLI and the GUI have never applied the same exposure. The CLI baked one on the server
(``RenderConsumer::PostSnapshot``); the GUI ignored that image entirely, took the RAW XYZ and
applied its own anchor in the preview shader. Making the two agree is the whole point of the
``relative`` mode -- and "agree" has to mean the pixels, not the intent, because the two are
separate implementations that were only ever reasoned to be equivalent:

  GUI  : ev   = ComputeEvAuto(P99, snapshot_intensity, 135)      [clamped to +/-6 stops]
         scale = 2^(exposure_offset + ev) / snapshot_intensity
  CLI  : scale = intensity_factor * TargetWhiteToLinear(135) / P99

They are the same number because ``snapshot_intensity`` cancels between ``ComputeEvAuto``'s
numerator and the shader's divisor, and because an exported config's ``intensity_factor`` IS
``2^exposure_offset``. That cancellation is an algebraic claim about code in two files, which
is exactly the kind of claim that holds until someone edits one of them.

No window is needed to check it. The GUI's mono display path is three C API calls over the raw
buffer -- ``LUMICE_ComputeP99Y`` / ``LUMICE_ComputeEvAuto`` / ``LUMICE_XyzToSrgbUint8`` -- and
running those three IS running the GUI's algorithm, on the same shared library the app links.
So this compares the server's baked image against the GUI's own formula, over one simulation.

Why the scene is deliberately plain. A background colour, a ray_color tint, a grid or the
celestial outline all enter the CLI's bake and none enter ``LUMICE_XyzToSrgbUint8``, so any of
them would show up as a difference that has nothing to do with exposure. The config is stripped
to exactly that: black background, default (spectral) ray colour, no grid, no outline. If this
test ever goes red, check the scene before the formula.

That list used to have four members and now it has five. The fifth is the RENDER-DOMAIN MASK, and it
is the one the scene cannot be stripped of: ``RenderConsumer::PostSnapshot`` zeroes every pixel
whose centre inverse-projects outside the lens domain (and outside ``visible``), while
``LUMICE_XyzToSrgbUint8`` is a per-pixel transfer function that masks nothing. On this config's
dual-fisheye frame that is a sub-pixel ring just outside each image circle -- 893 pixels of
1_572_864 lit on the GUI side and black on the CLI's -- because the forward binning is an AREA
predicate (any direction landing in a pixel deposits energy there, so a pixel straddling the rim
collects some) while the mask is a POINT predicate on the pixel centre. Both are correct and the
product agrees with itself: the preview shader discards that ring too. What did not carry the
mask was this fixture's simplified GUI arm, so the mask is applied here, read from core's own
answer -- ``LUMICE_ComputeAnnotationOverlay``'s ``drawable``, which is built from the same
``mask_detail::PixelToWorld`` + ``VisibleByRange`` + ``FrontVisible`` predicate as the
``BuildVisibleMask`` the renderer bakes with. Mirroring the geometry in Python instead would put a
second authority on which pixels the lens images, which is the class of defect this suite exists
to catch rather than to add.

Not necessarily byte-identical, and the residual is understood rather than tolerated: the GUI
path routes its scale through ``2^log2(x)`` while the CLI computes the ratio directly, so the two
scales differ in the last float bits. Measured, that moved 7 channels out of 1.5 million by one
8-bit step; it currently moves none, because which pixel a rounding lands on is a property of
where the energy fell and the forward binning has since shifted by half a pixel. The count is
incidental, the mechanism is not, so the bands below stay where that measurement put them rather
than being tightened onto a zero this machine happens to read. They are set against it and
against the negative control, which says what a real disagreement looks like on the same
calipers.
"""

from __future__ import annotations

import ctypes
import json
import re
import shutil
import tempfile
from pathlib import Path

import numpy as np
import pytest

# _find_lib rather than a local search: it is the single owner of the library-resolution rules,
# including the LUMICE_LIB override that scripts/test.sh relies on. A second copy of that search
# would go stale silently and in the worst direction -- deciding the library is missing.
from test.e2e.capi_runner import _find_lib, run_scene_capi_buffered
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
PARITY_CONFIG = CONFIGS_DIR / "ev_mode_relative_gui_parity.json"

_SEED = 42

# The GUI's own two constants, restated as literals rather than read from the source under test:
# f=8 is the mono path's downsample factor (gui_constants.hpp::kEvAutoDownsampleFactor, mirrored
# in core as kMonoAnchorDownsampleFactor) and 135 is the target white (GuiState::target_white /
# kAnchorTargetWhite). Reading them from the thing being checked would make a changed constant
# agree with itself.
_GUI_DOWNSAMPLE_FACTOR = 8
_GUI_TARGET_WHITE = 135.0

# Measured on the parity config: 7 differing channels out of 1_572_864, all off by one step.
# The band is ~20x that and ~5 orders of magnitude under the negative control's 0.407.
_MAX_DIFFERING_FRACTION = 1e-4
# One 8-bit step is the float-rounding residual described above. Two would mean something else.
_MAX_CHANNEL_DIFF = 1

# Measured on the parity config: the mask turns off 893 lit pixels of 524_288 (0.170%), the
# sub-pixel ring described in the module docstring. See test_the_domain_mask_only_takes_the_rim.
_MAX_MASKED_LIT_FRACTION = 5e-3

# What the negative control has to clear. Absolute mode measures 0.407 of channels differing,
# with a max step of 7 -- so this floor is 40x under the effect and 100x over the band above.
_CONTROL_MIN_DIFFERING_FRACTION = 1e-2


# ---------------------------------------------------------------------------------------------
# The render-domain mask, read from core rather than re-derived. See the fifth item in the module
# docstring for why the GUI arm needs it at all.
#
# These four ctypes structs mirror lumice.h field for field. The mirror matters more here than for
# a read-only result struct: the CALLER allocates LUMICE_AnnotationOverlay and core fills it, so a
# mirror short by one field is not a wrong number but a write past the end of Python's storage.
# _assert_mirrors_match_header() below reads the header and checks the field NAMES, which is the
# same guard capi_runner.py puts on LUMICE_StatsResult and for the same reason: a size assertion
# compares this file to a number typed in this file.
# ---------------------------------------------------------------------------------------------


class _AnnotationView(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("lens_type", ctypes.c_int),
        ("lens_fov", ctypes.c_float),
        ("lens_shift", ctypes.c_int * 2),
        ("overlap", ctypes.c_float),
        ("view_azimuth", ctypes.c_float),
        ("view_elevation", ctypes.c_float),
        ("view_roll", ctypes.c_float),
        ("visible", ctypes.c_int),
        ("front", ctypes.c_int),
    ]


class _AnnotationRequest(ctypes.Structure):
    _fields_ = [
        ("view", _AnnotationView),
        ("horizon", ctypes.c_int),
        ("elevation_deg", ctypes.POINTER(ctypes.c_float)),
        ("elevation_count", ctypes.c_int),
        ("longitude_deg", ctypes.POINTER(ctypes.c_float)),
        ("longitude_count", ctypes.c_int),
        ("angular_dist_deg", ctypes.POINTER(ctypes.c_float)),
        ("angular_dist_count", ctypes.c_int),
        ("reference_dir", ctypes.c_float * 3),
        ("zenith_nadir", ctypes.c_int),
        ("want_labels", ctypes.c_int),
    ]


class _AnnotationOverlay(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("drawable", ctypes.POINTER(ctypes.c_ubyte)),
        ("horizon", ctypes.POINTER(ctypes.c_ubyte)),
        ("elevation", ctypes.POINTER(ctypes.c_ubyte)),
        ("longitude", ctypes.POINTER(ctypes.c_ubyte)),
        ("angular_dist", ctypes.POINTER(ctypes.c_ubyte)),
        ("zenith_px", ctypes.c_float),
        ("zenith_py", ctypes.c_float),
        ("zenith_valid", ctypes.c_int),
        ("nadir_px", ctypes.c_float),
        ("nadir_py", ctypes.c_float),
        ("nadir_valid", ctypes.c_int),
        ("labels", ctypes.c_void_p),
        ("label_count", ctypes.c_int),
        ("storage", ctypes.c_void_p),
    ]


_HEADER = get_project_root() / "src" / "include" / "lumice.h"


def _header_struct_fields(tag: str) -> list[str] | None:
    """Field names of one `typedef struct <tag>_ { ... } <tag>;` in lumice.h, in order."""
    if not _HEADER.is_file():  # source tree not available (e.g. installed wheel)
        return None
    body = re.search(
        rf"typedef struct {tag}_\s*\{{(.*?)\}}\s*{tag};",
        _HEADER.read_text(encoding="utf-8"),
        re.DOTALL,
    )
    assert body is not None, f"could not locate {tag} in lumice.h"
    decls = re.sub(r"//.*", "", body.group(1))
    names = []
    for decl in decls.split(";"):
        match = re.search(r"([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*$", decl.strip())
        if match:
            names.append(match.group(1))
    return names


def _assert_mirrors_match_header() -> None:
    for tag, mirror in (
        ("LUMICE_AnnotationView", _AnnotationView),
        ("LUMICE_AnnotationRequest", _AnnotationRequest),
        ("LUMICE_AnnotationOverlay", _AnnotationOverlay),
    ):
        header_fields = _header_struct_fields(tag)
        if header_fields is None:
            return
        mirror_fields = [name for name, *_ in mirror._fields_]
        assert header_fields == mirror_fields, (
            f"{tag} drift -- lumice.h has {header_fields}, this mirror has {mirror_fields}. "
            "Core writes this struct through a pointer Python sized from the mirror, so fix the "
            "mirror before running anything else"
        )


_assert_mirrors_match_header()


def _header_enum_map(prefix: str) -> dict[str, int]:
    """`#define <prefix><NAME> <n>` -> {"<name>": n}, lowercased.

    The config spells a lens type and a visible range exactly as the lowercase of these suffixes
    (render_config.hpp's NLOHMANN_JSON_SERIALIZE_ENUM tables), so reading the header is what keeps
    this file from carrying a hand-written second copy of either enum.

    Unlike `_header_struct_fields`, this has no "source tree absent" no-op: its caller needs the
    actual enum values to compute the mask, not just a drift check to skip, so there is nothing
    correct to return in that case -- this parity test requires the source tree to run.
    """
    text = _HEADER.read_text(encoding="utf-8")
    return {
        name.lower(): int(value)
        for name, value in re.findall(rf"^#define {prefix}(\w+)\s+(\d+)$", text, re.MULTILINE)
    }


def _render_domain_mask(doc: dict) -> np.ndarray:
    """The pixels the renderer bakes, as core answers it: True where the lens images sky.

    One `LUMICE_ComputeAnnotationOverlay` call with no annotation family requested, which makes it
    the `drawable` sweep and nothing else (the header's own note: "The masks alone are several
    times cheaper than masks plus anchors").
    """
    render = doc["render"][0]
    width, height = (int(v) for v in render["resolution"])
    view = _AnnotationView(
        width=width,
        height=height,
        lens_type=_header_enum_map("LUMICE_LENS_TYPE_")[render["lens"]["type"]],
        lens_fov=float(render["lens"]["fov"]),
        lens_shift=(ctypes.c_int * 2)(*(int(v) for v in render.get("lens_shift", (0, 0)))),
        overlap=float(render.get("overlap", 0.0)),
        view_azimuth=float(render["view"]["azimuth"]),
        view_elevation=float(render["view"]["elevation"]),
        view_roll=float(render["view"]["roll"]),
        visible=_header_enum_map("LUMICE_VISIBLE_")[render.get("visible", "full")],
        front=1 if render.get("front", False) else 0,
    )
    request = _AnnotationRequest(view=view)
    overlay = _AnnotationOverlay()

    lib = ctypes.CDLL(str(_find_lib()))
    lib.LUMICE_ComputeAnnotationOverlay.restype = ctypes.c_int
    lib.LUMICE_ComputeAnnotationOverlay.argtypes = [
        ctypes.POINTER(_AnnotationRequest),
        ctypes.POINTER(_AnnotationOverlay),
    ]
    lib.LUMICE_ReleaseAnnotationOverlay.restype = None
    lib.LUMICE_ReleaseAnnotationOverlay.argtypes = [ctypes.POINTER(_AnnotationOverlay)]

    err = lib.LUMICE_ComputeAnnotationOverlay(ctypes.byref(request), ctypes.byref(overlay))
    assert err == 0, f"LUMICE_ComputeAnnotationOverlay failed err={err}"
    try:
        assert (overlay.width, overlay.height) == (width, height), (
            f"overlay canvas {overlay.width}x{overlay.height} != config {width}x{height}"
        )
        assert overlay.drawable, "core returned a NULL drawable mask for a non-degenerate view"
        # Copied, not viewed: the buffer belongs to core and dies at Release below.
        flat = np.ctypeslib.as_array(overlay.drawable, shape=(height * width,)).copy()
    finally:
        lib.LUMICE_ReleaseAnnotationOverlay(ctypes.byref(overlay))
    return flat.reshape(height, width) != 0


def _psnr(a: np.ndarray, b: np.ndarray) -> float:
    mse = float(((a.astype(np.float64) - b.astype(np.float64)) ** 2).mean())
    return float("inf") if mse == 0.0 else float(10.0 * np.log10(255.0**2 / mse))


class _GuiMonoPath:
    """The GUI's mono display formula, as the three C API calls the app makes."""

    def __init__(self) -> None:
        self._lib = ctypes.CDLL(str(_find_lib()))
        self._lib.LUMICE_ComputeP99Y.restype = ctypes.c_float
        self._lib.LUMICE_ComputeP99Y.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
        ]
        self._lib.LUMICE_ComputeEvAuto.restype = ctypes.c_float
        self._lib.LUMICE_ComputeEvAuto.argtypes = [ctypes.c_float] * 3
        self._lib.LUMICE_XyzToSrgbUint8.restype = ctypes.c_int
        self._lib.LUMICE_XyzToSrgbUint8.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_int,
            ctypes.c_float,
        ]

    def render(self, result) -> tuple[np.ndarray, float]:
        """Returns (sRGB image the GUI would show, the auto-EV it chose in stops)."""
        w, h = int(result.img_width), int(result.img_height)
        xyz = np.ascontiguousarray(result.flt_buf.reshape(-1), dtype=np.float32)
        p_xyz = xyz.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

        p99 = self._lib.LUMICE_ComputeP99Y(p_xyz, w, h, _GUI_DOWNSAMPLE_FACTOR)
        ev_auto = self._lib.LUMICE_ComputeEvAuto(
            ctypes.c_float(p99),
            ctypes.c_float(result.snapshot_intensity),
            ctypes.c_float(_GUI_TARGET_WHITE),
        )
        # exposure_offset = 0 for this document, so intensity_factor is 2^ev_auto alone. The
        # divisor is app.cpp's `intensity_scale = intensity_factor / snapshot_intensity`.
        intensity_scale = (2.0**ev_auto) / result.snapshot_intensity

        out = np.zeros(w * h * 3, dtype=np.uint8)
        err = self._lib.LUMICE_XyzToSrgbUint8(
            p_xyz,
            out.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
            w * h,
            ctypes.c_float(intensity_scale),
        )
        assert err == 0, f"LUMICE_XyzToSrgbUint8 failed err={err}"
        return out.reshape(h, w, 3), float(ev_auto)


def _compare(config_path: Path, doc: dict) -> dict:
    result = run_scene_capi_buffered(str(config_path), sim_seed=_SEED, backend="legacy", timeout_sec=600)
    assert result.has_valid_data, f"{config_path.name}: simulation produced no data"
    gui_image, ev_auto = _GuiMonoPath().render(result)
    # The fifth CLI-bake-only item, applied to the GUI arm exactly as the renderer applies it:
    # PostSnapshot writes 0 into a masked-out pixel BEFORE the background term and before any
    # annotation, so on this scene a masked pixel is black in linear and black in the PNG.
    drawable = _render_domain_mask(doc)
    assert drawable.shape == gui_image.shape[:2], (
        f"mask {drawable.shape} does not cover the rendered image {gui_image.shape[:2]}"
    )
    masked_lit = int(np.count_nonzero(np.any(gui_image != 0, axis=2) & ~drawable))
    gui_image = np.where(drawable[:, :, None], gui_image, np.uint8(0))
    diff = np.abs(gui_image.astype(np.int32) - result.rgb_buf.astype(np.int32))
    return {
        "ev_auto": ev_auto,
        "max_channel_diff": int(diff.max()),
        "differing_fraction": float((diff > 0).sum()) / float(diff.size),
        # What the mask CHANGED, not what it covers: most of a dual-fisheye canvas is outside
        # the two image circles and is black on both arms anyway, so "21.5% of pixels are masked"
        # says nothing about whether this mask is doing something it should not.
        "masked_lit_fraction": float(masked_lit) / float(drawable.size),
        "psnr": _psnr(gui_image, result.rgb_buf),
    }


@pytest.fixture(scope="module")
def relative_run():
    with open(PARITY_CONFIG, encoding="utf-8") as fp:
        return _compare(PARITY_CONFIG, json.load(fp))


@pytest.fixture(scope="module")
def absolute_control():
    """The same document with only ev_mode flipped -- the control for the comparison itself."""
    with open(PARITY_CONFIG, encoding="utf-8") as fp:
        doc = json.load(fp)
    doc["render"][0]["ev_mode"] = "absolute"
    tmp_dir = Path(tempfile.mkdtemp(prefix="lumice_ev_parity_"))
    try:
        # Written beside a copy of nothing else: the document is self-contained, and the config's
        # own directory must not gain a file the suite would then have to explain.
        control_path = tmp_dir / "ev_mode_absolute_control.json"
        control_path.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        yield _compare(control_path, doc)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


@pytest.mark.slow
def test_the_auto_ev_does_not_hit_the_guis_clamp(relative_run):
    """Premise. ComputeEvAuto clamps to +/-6 stops and the CLI has no such clamp.

    That asymmetry is deliberate -- the clamp guards a GUI slider's range, and the CLI has no
    slider -- so at the clamp the two paths are SUPPOSED to diverge, and the parity assertion
    below would be measuring the wrong thing. This case pins that the chosen scene sits well
    inside the band, so a red there is about the formula rather than about the fixture drifting
    into a corner nobody meant to test.
    """
    assert abs(relative_run["ev_auto"]) < 5.0, (
        f"the fixture's auto-EV is {relative_run['ev_auto']:+.3f} stops, close enough to "
        "ComputeEvAuto's +/-6 clamp that the GUI and CLI paths are allowed to disagree -- pick a "
        "scene of ordinary brightness rather than relaxing the parity band below"
    )


@pytest.mark.slow
def test_the_domain_mask_only_takes_the_rim(relative_run):
    """Premise. The mask applied to the GUI arm must be a rim, not a lid.

    Masking the GUI arm is how the fifth CLI-bake-only item stops being counted as an exposure
    difference (see the module docstring). It is also, applied carelessly, a way to make the
    assertion below pass by deleting the pixels it disagrees about -- and nothing in that
    assertion could tell the two apart. This one can: it counts the pixels the mask actually
    turns off, which on this config is a sub-pixel ring around each image circle where the
    forward binning's area predicate and the mask's point predicate disagree. Measured: 893
    pixels of 524_288, 0.170%. The band is ~3x that and still under one 1-pixel-wide ring around
    both circles (2 * 2*pi*256 = 3217 px, 0.61%), so a mask that started eating lit sky rather
    than a rim cannot pass it.
    """
    assert relative_run["masked_lit_fraction"] <= _MAX_MASKED_LIT_FRACTION, (
        f"the render-domain mask turns off {relative_run['masked_lit_fraction'] * 100:.4f}% of "
        f"the GUI arm's lit pixels (band {_MAX_MASKED_LIT_FRACTION * 100:.4f}%). That is more "
        "than the rim where the two predicates disagree, so the parity assertion below would be "
        "measuring a frame the mask had already edited -- fix the mask or the scene, not the band"
    )


@pytest.mark.slow
def test_relative_mode_reproduces_the_gui_image(relative_run):
    """The CLI's baked image is the image the GUI would display, pixel for pixel."""
    assert relative_run["max_channel_diff"] <= _MAX_CHANNEL_DIFF, (
        f"a channel differs by {relative_run['max_channel_diff']} steps (allowed "
        f"{_MAX_CHANNEL_DIFF}, which is the float-rounding residual). A larger step is a "
        f"different formula, not rounding. PSNR={relative_run['psnr']:.2f} dB"
    )
    assert relative_run["differing_fraction"] <= _MAX_DIFFERING_FRACTION, (
        f"{relative_run['differing_fraction'] * 100:.4f}% of channels differ (band "
        f"{_MAX_DIFFERING_FRACTION * 100:.4f}%). Before touching the exposure formula, check the "
        "scene: a background colour, a ray_color tint, a grid or the celestial outline enters the "
        f"CLI bake and not LUMICE_XyzToSrgbUint8. PSNR={relative_run['psnr']:.2f} dB"
    )


@pytest.mark.slow
def test_the_comparison_can_tell_the_two_modes_apart(absolute_control):
    """Negative control: the same comparison on the same scene under the absolute anchor.

    Without it, the case above would pass just as well for a build that ignored ev_mode
    entirely, or for a comparison too blunt to resolve an exposure difference at all -- and
    there would be nothing in the suite saying which. Here the criterion is shown failing, on
    the one input it must fail on.
    """
    assert absolute_control["differing_fraction"] >= _CONTROL_MIN_DIFFERING_FRACTION, (
        f"under ev_mode=absolute only {absolute_control['differing_fraction'] * 100:.4f}% of "
        f"channels differ from the GUI's formula (needs >="
        f"{_CONTROL_MIN_DIFFERING_FRACTION * 100:.2f}%). Either the mode is not reaching the "
        "renderer, or this comparison cannot resolve an exposure difference -- in both cases the "
        "parity result above means nothing"
    )
