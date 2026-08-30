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

Not byte-identical, and the residual is understood rather than tolerated: the GUI path routes
its scale through ``2^log2(x)`` while the CLI computes the ratio directly, so the two scales
differ in the last float bits. Measured, that moves 7 channels out of 1.5 million by one 8-bit
step. The bands below are set against that measurement and against the negative control, which
says what a real disagreement looks like on the same calipers.
"""

from __future__ import annotations

import ctypes
import json
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

# What the negative control has to clear. Absolute mode measures 0.407 of channels differing,
# with a max step of 7 -- so this floor is 40x under the effect and 100x over the band above.
_CONTROL_MIN_DIFFERING_FRACTION = 1e-2


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


def _compare(config_path: Path) -> dict:
    result = run_scene_capi_buffered(str(config_path), sim_seed=_SEED, backend="legacy", timeout_sec=600)
    assert result.has_valid_data, f"{config_path.name}: simulation produced no data"
    gui_image, ev_auto = _GuiMonoPath().render(result)
    diff = np.abs(gui_image.astype(np.int32) - result.rgb_buf.astype(np.int32))
    return {
        "ev_auto": ev_auto,
        "max_channel_diff": int(diff.max()),
        "differing_fraction": float((diff > 0).sum()) / float(diff.size),
        "psnr": _psnr(gui_image, result.rgb_buf),
    }


@pytest.fixture(scope="module")
def relative_run():
    return _compare(PARITY_CONFIG)


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
        yield _compare(control_path)
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
