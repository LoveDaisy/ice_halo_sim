"""The other half of the additivity oracle: what the RELATIVE anchor does to it.

``test_absolute_energy_additivity.py`` next door checks the absolute anchor's defining
property -- the denominator is the energy the source EMITTED, so it does not know the
filter exists, and two complementary halves of one scene add back up to the whole. That
file reads RAW (unexposed) XYZ, which is the same buffer whichever anchor is selected, so
running it under ``ev_mode: relative`` reproduces it verbatim rather than contradicting it.
Its greenness is therefore not evidence that the mode switch does anything.

This file supplies the evidence, by moving the same oracle onto the quantity the switch
actually controls: the exposure scale the server applies when it bakes the image. The two
modes are opposites there, and the opposition is the point --

  absolute: scale = intensity_factor * kNormScale * total_pix / emitted_energy
            The three configs emit identically, so once each one's own intensity_factor is
            divided out they must land on ONE number. The anchor is a property of the light
            source and the ray budget.

  relative: scale = intensity_factor * target_linear / P99(this frame)
            Each split renormalizes to whatever survived its own filter, so the anchor is a
            property of the picture. The half that keeps the dim rays is re-brightened by
            roughly the amount its filter removed -- the same re-brightening the retired
            landed-weight denominator performed, which is why additivity breaks in the same
            direction here.

Neither mode is being called wrong. Relative is the DEFAULT and is what the GUI has always
displayed; a self-anchored image is supposed to move with its content. What must not happen
is the two modes behaving alike, because then the field is inert -- and a field that is
inert while a config, an ABI slot and a document key all claim it exists is the failure this
file is here to catch.

How the applied scale is measured. It is recovered from the product's own output rather than
recomputed from the formula: the rendered sRGB image is linearized, its luminance divided by
the raw luminance of the same pixel, and the median taken over pixels in a band that avoids
both uint8 quantization at the bottom and the [0,1] clamp at the top. Recomputing the formula
would make the test agree with render.cpp by construction and go green on any pair of modes
whose formulas were both copied here correctly -- including a pair that never reached the
renderer at all. The recovery lives in ``test/e2e/_display_scale.py``, shared with the
ray-count suite, and is accurate to about 1%: it puts kNormScale at 0.0793 against its declared
0.08, which is the quantization bias and is 450x smaller than the effect below.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from test.e2e._display_scale import recover_applied_scale
from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# The same seed the absolute sibling pins, for the same reason: it makes the runs
# reproducible against each other and converges the server onto a single worker.
_SEED = 42

# Measured spread of the absolute anchor across the three splits: 1.0006x, on two seeds
# (42 and 7). The band is ~16x that, and still ~450x under the relative effect, so it is not
# wide enough to swallow what it exists to detect.
_ABS_ANCHOR_SPREAD_MAX = 1.01

# Measured relative spread: 4.53x (seed 42) and 4.56x (seed 7). The floor is well below both
# and well above the absolute band -- the gap between the two numbers is the whole assertion,
# so neither is chosen to sit near the other.
_REL_ANCHOR_SPREAD_MIN = 2.0

# Additivity of displayed energy. Absolute measures +0.19% / +0.99% on the two seeds; the
# band is the sibling file's _SUM_Y_TOL, restated here rather than imported because
# `test/regression-sentinel` is not an importable package name (the hyphen).
_DISPLAY_ADD_TOL = 0.06

# Relative measures +219% / +221%. Same floor the sibling's negative control uses, and for
# the same reason: it has to be a margin no Monte-Carlo noise could produce.
_DISPLAY_ADD_MIN_BREAK = 0.50


def _config_path(split: str, mode: str) -> Path:
    stem = f"absolute_additivity_{split}" + ("_relative" if mode == "relative" else "")
    return CONFIGS_DIR / f"{stem}.json"


def _measure(split: str, mode: str) -> dict:
    path = _config_path(split, mode)
    with open(path, encoding="utf-8") as fp:
        doc = json.load(fp)
    render = doc["render"][0]
    assert render["ev_mode"] == mode, f"{path.name} declares ev_mode={render['ev_mode']!r}, expected {mode!r}"
    intensity_factor = float(render["intensity_factor"])

    result = run_scene_capi_buffered(str(path), sim_seed=_SEED, backend="legacy", timeout_sec=600)
    assert result.has_valid_data, f"{path.name}: simulation produced no data"

    scale = recover_applied_scale(result, label=path.name)
    return {
        "sum_y": float(result.flt_buf[:, :, 1].astype(np.float64).sum()),
        # The applied scale with the config's own EV knob divided out. What is left is the
        # ANCHOR -- the part the mode selects. Leaving intensity_factor in would make the two
        # halves differ by their deliberate half-stop in both modes and hide the effect.
        "anchor": scale / intensity_factor,
        "emitted": float(result.emitted_energy),
    }


@pytest.fixture(scope="module")
def anchors():
    """Six runs: the whole scene and its two halves, under each anchor."""
    return {mode: {split: _measure(split, mode) for split in ("all", "in", "out")} for mode in ("absolute", "relative")}


def _spread(parts: dict) -> float:
    values = [p["anchor"] for p in parts.values()]
    return max(values) / min(values)


def _display_excess(parts: dict) -> float:
    """(in + out - all) / all on displayed energy -- 0 when the halves partition."""
    energy = {name: p["sum_y"] * p["anchor"] for name, p in parts.items()}
    return (energy["in"] + energy["out"] - energy["all"]) / energy["all"]


@pytest.mark.slow
def test_the_absolute_anchor_does_not_know_which_filter_ran(anchors):
    """Absolute: one anchor for all three splits, because they emit the same energy.

    Stated on the anchor rather than on the image because this is the claim the mode makes.
    The premise is checked alongside it: if the three configs did NOT emit identically the
    anchors could agree for a reason that has nothing to do with the mode.
    """
    parts = anchors["absolute"]
    emitted = parts["all"]["emitted"]
    assert emitted > 0.0, "emitted energy was never charged"
    for split in ("in", "out"):
        assert parts[split]["emitted"] == pytest.approx(emitted, rel=1e-6), (
            f"premise broken: {split} emitted {parts[split]['emitted']!r} against {emitted!r}, "
            "so the three splits are not the same scene and the anchors below prove nothing"
        )

    spread = _spread(parts)
    assert spread <= _ABS_ANCHOR_SPREAD_MAX, (
        f"the absolute anchor spans {spread:.4f}x across the three filter splits "
        f"(band {_ABS_ANCHOR_SPREAD_MAX}x) -- it is tracking the filter, which is the one "
        "thing an absolute anchor must not do: "
        + ", ".join(f"{k}={v['anchor']:.8g}" for k, v in parts.items())
    )


@pytest.mark.slow
def test_the_relative_anchor_moves_with_the_split(anchors):
    """Relative: the anchor is a property of the picture, so it must NOT hold still.

    This is the assertion that gives the mode switch observable effect. Without it, an
    ev_mode that never reached RenderConsumer::ExposureScale would leave every other test in
    this file and its sibling green -- they would all be measuring the absolute path twice.
    """
    spread = _spread(anchors["relative"])
    assert spread >= _REL_ANCHOR_SPREAD_MIN, (
        f"the relative anchor spans only {spread:.4f}x across the three splits (needs "
        f">={_REL_ANCHOR_SPREAD_MIN}x to be distinguishable from the absolute anchor's "
        f"{_ABS_ANCHOR_SPREAD_MAX}x band) -- the two modes are behaving alike, so the field "
        "is not reaching the exposure scale: "
        + ", ".join(f"{k}={v['anchor']:.8g}" for k, v in anchors["relative"].items())
    )


@pytest.mark.slow
def test_displayed_energy_adds_up_under_absolute_and_does_not_under_relative(anchors):
    """The additivity oracle itself, run on both anchors in one case.

    Both halves of the asymmetry are asserted here rather than in two files, because the
    proposition is the CONTRAST: "absolute is additive" is worth little on its own (the raw
    buffer is additive whatever the mode), and "relative is not additive" alone reads as a
    defect report. Together they say the mode selects between two different behaviors, which
    is what the field is for.
    """
    absolute = _display_excess(anchors["absolute"])
    assert abs(absolute) <= _DISPLAY_ADD_TOL, (
        f"under the absolute anchor the two halves' displayed energy misses the whole by "
        f"{absolute * 100:+.3f}% (band +/-{_DISPLAY_ADD_TOL * 100:.1f}%) -- two renders at "
        "one EV are no longer comparable, which is the property the mode exists to provide"
    )

    relative = _display_excess(anchors["relative"])
    assert abs(relative) >= _DISPLAY_ADD_MIN_BREAK, (
        f"under the relative anchor the halves are off by only {relative * 100:+.2f}%, under "
        f"the {_DISPLAY_ADD_MIN_BREAK * 100:.0f}% that separates a self-anchored image from an "
        "absolute one. Do NOT widen this by loosening the absolute band above: the two "
        "assertions are a pair, and the gap between them is the measurement"
    )
