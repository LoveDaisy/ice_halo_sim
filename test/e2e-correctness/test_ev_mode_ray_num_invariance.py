"""What each ``ev_mode`` keeps still when ``ray_num`` changes -- and they keep still different things.

Raising the ray budget is supposed to reduce noise, not change the picture. But "not change the
picture" means two incompatible things, and the mode is the choice between them:

  absolute: the BRIGHTNESS holds. The scale is ``k / emitted_energy`` and emitted energy is a
            deterministic function of the ray budget, so the scale is exactly inversely
            proportional to it: 30x the rays, 1/30 the scale, the same displayed image. The
            config alone determines how bright the output is.

  relative: the APPEARANCE holds. The scale is ``k / P99(this frame)``, which follows the picture
            rather than the budget, so displayed brightness is NOT determined by the config --
            the ray count co-determines it -- while the tonal distribution stays put.

Both are asserted here, on one scene, and so is the fact that they are different measurements:
the absolute anchor's ``scale x ray_num`` is constant to 0.13% while the relative anchor's is off
by 13-23%. Without that third case, the first two could be one property measured twice.

Honest limit, stated so a later reader does not over-read case 3. On this fixture lit-p50 is
stable in BOTH modes (absolute -0.08/-0.17 stop, relative -0.23/-0.22 stop over 30x rays), so the
appearance caliber does NOT by itself separate them. Every measurement in this repo that HAS
separated them on the ray-count axis needed a far sparser scene than this one, where the f=8
coarse anchor sits ~64x under the fine anchor. What separates the modes here is the anchor case,
and the parity/additivity suites elsewhere. Case 3 says relative keeps its own promise; it does
not say absolute breaks it.

The scene. ``ev_mode_ray_num_invariance.json`` is deliberately narrow-orientation (zenith and roll
std 0.3 deg) at 2048x1024, which spreads few rays over many pixels -- the regime where a
self-anchored scale has room to drift off 1/N at all. Its ``ray_num`` is the low end of the sweep;
the test writes both ends, and both modes, as temporary documents.
"""

from __future__ import annotations

import json
import math
import shutil
import tempfile
from pathlib import Path

import pytest

from test.e2e._display_scale import lit_pixel_percentile, recover_applied_scale
from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

BASE_CONFIG = get_project_root() / "test" / "e2e" / "configs" / "ev_mode_ray_num_invariance.json"

_SEED = 42
_RAY_NUM_LOW = 100_000
_RAY_NUM_HIGH = 3_000_000

# scale x ray_num, as a deviation from constant. Absolute measures 0.13% on both seeds tried
# (42 and 7) -- the residual of the recovery, not of the renderer, since the relation is exact.
# The band is ~15x that and ~10x under what relative does.
_ABS_ANCHOR_INVARIANCE_MAX = 0.02

# The same quantity for relative: 12.9% (seed 42) and 23.4% (seed 7). The floor sits between the
# two bands rather than beside either, so neither number is doing the separating on its own.
_REL_ANCHOR_DRIFT_MIN = 0.05

# Displayed total energy, in stops. Absolute measures +0.008 / -0.029 -- the residual is the
# Monte-Carlo wobble in what fraction of emitted energy lands, which is a property of the
# simulation and not of the anchor.
_ABS_BRIGHTNESS_STOPS = 0.15

# lit-p50, in stops. Relative measures -0.228 / -0.218 here. The band is ~2x that and inside the
# -0.84 stop this repo has measured on a harsher scene, so it is a claim about THIS fixture.
_REL_APPEARANCE_STOPS = 0.5


def _stops(ratio: float) -> float:
    return math.log2(ratio)


@pytest.fixture(scope="module")
def sweep():
    """Four runs: {absolute, relative} x {low, high} ray count, on one scene."""
    with open(BASE_CONFIG, encoding="utf-8") as fp:
        base = json.load(fp)

    tmp_dir = Path(tempfile.mkdtemp(prefix="lumice_ev_raynum_"))
    out: dict = {}
    try:
        for mode in ("absolute", "relative"):
            out[mode] = {}
            for ray_num in (_RAY_NUM_LOW, _RAY_NUM_HIGH):
                doc = json.loads(json.dumps(base))
                doc["render"][0]["ev_mode"] = mode
                doc["scene"]["ray_num"] = ray_num
                path = tmp_dir / f"{mode}_{ray_num}.json"
                path.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n", encoding="utf-8")

                result = run_scene_capi_buffered(str(path), sim_seed=_SEED, backend="legacy", timeout_sec=900)
                assert result.has_valid_data, f"{path.name}: simulation produced no data"
                scale = recover_applied_scale(result, label=path.name)
                out[mode][ray_num] = {
                    "scale": scale,
                    "raw_total": float(result.flt_buf[:, :, 1].astype("float64").sum()),
                    "emitted": float(result.emitted_energy),
                    "lit_p50": lit_pixel_percentile(result.rgb_buf, 50.0),
                }
        yield out
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def _scale_times_budget(parts: dict) -> float:
    """(scale x ray_num) at the high end over the low end -- 1.0 when scale is exactly 1/N.

    The budget is read as emitted energy rather than as the ray_num written in the config: emitted
    energy is what the scale's denominator actually is, so this compares the two things the
    absolute claim relates, with nothing assumed about how they were configured.
    """
    low, high = parts[_RAY_NUM_LOW], parts[_RAY_NUM_HIGH]
    return (high["scale"] / low["scale"]) * (high["emitted"] / low["emitted"])


@pytest.mark.slow
def test_the_absolute_scale_is_exactly_inverse_to_the_ray_budget(sweep):
    """Absolute: 30x the rays, 1/30 the scale. The relation is exact, not statistical."""
    parts = sweep["absolute"]
    budget_ratio = parts[_RAY_NUM_HIGH]["emitted"] / parts[_RAY_NUM_LOW]["emitted"]
    assert budget_ratio == pytest.approx(_RAY_NUM_HIGH / _RAY_NUM_LOW, rel=0.02), (
        f"premise: emitted energy grew {budget_ratio:.4f}x for a {_RAY_NUM_HIGH / _RAY_NUM_LOW:.0f}x "
        "ray budget, so the denominator is not tracking the budget and the claim below is about "
        "something else"
    )

    deviation = abs(_scale_times_budget(parts) - 1.0)
    assert deviation <= _ABS_ANCHOR_INVARIANCE_MAX, (
        f"scale x budget moved by {deviation * 100:.2f}% across the sweep (band "
        f"{_ABS_ANCHOR_INVARIANCE_MAX * 100:.0f}%) -- under the absolute anchor this product is "
        "fixed by construction, so any real movement means the scale is reading something other "
        "than emitted energy"
    )


@pytest.mark.slow
def test_absolute_display_brightness_does_not_move_with_the_ray_budget(sweep):
    """The product-level half of the same claim: the picture comes out as bright either way.

    Total displayed energy rather than a percentile, because brightness is an integral quantity
    and a percentile of it moves when the set of lit pixels grows -- which it does, with more
    rays, for reasons that have nothing to do with exposure.
    """
    parts = sweep["absolute"]
    low, high = parts[_RAY_NUM_LOW], parts[_RAY_NUM_HIGH]
    ratio = (high["scale"] * high["raw_total"]) / (low["scale"] * low["raw_total"])
    assert abs(_stops(ratio)) <= _ABS_BRIGHTNESS_STOPS, (
        f"displayed brightness moved {_stops(ratio):+.3f} stops across a "
        f"{_RAY_NUM_HIGH / _RAY_NUM_LOW:.0f}x ray budget (band +/-{_ABS_BRIGHTNESS_STOPS} stop) -- "
        "under the absolute anchor the config alone is supposed to determine output brightness"
    )


@pytest.mark.slow
def test_relative_appearance_does_not_move_with_the_ray_budget(sweep):
    """Relative: the tonal distribution holds, measured at lit-p50.

    p50, and NOT p90/p99. Under a self-anchored exposure on a sparse scene the top percentiles
    saturate at 255 for every ray count -- a statistic that is constant because it is clipped,
    which reads as perfect stability and is worth nothing. That is measured, not feared: the f=8
    coarse anchor was found to sit ~64x under the fine one, blowing a large share of pixels to
    white. Do not "strengthen" this case by adding an upper percentile.
    """
    parts = sweep["relative"]
    ratio = parts[_RAY_NUM_HIGH]["lit_p50"] / parts[_RAY_NUM_LOW]["lit_p50"]
    assert abs(_stops(ratio)) <= _REL_APPEARANCE_STOPS, (
        f"lit-p50 moved {_stops(ratio):+.3f} stops across a {_RAY_NUM_HIGH / _RAY_NUM_LOW:.0f}x "
        f"ray budget (band +/-{_REL_APPEARANCE_STOPS} stop) -- under the relative anchor the "
        "image is supposed to keep its look as the noise comes down"
    )


@pytest.mark.slow
def test_the_two_kinds_of_stability_are_not_the_same_measurement(sweep):
    """The relative scale is NOT 1/N, which is what makes the two cases above two claims.

    If ev_mode never reached ExposureScale, both modes would run the absolute branch and every
    other case in this file would still pass -- they would be measuring one anchor twice. This is
    the case that fails in that world.
    """
    absolute_dev = abs(_scale_times_budget(sweep["absolute"]) - 1.0)
    relative_dev = abs(_scale_times_budget(sweep["relative"]) - 1.0)
    assert relative_dev >= _REL_ANCHOR_DRIFT_MIN, (
        f"the relative scale x budget product moved only {relative_dev * 100:.2f}% (needs "
        f">={_REL_ANCHOR_DRIFT_MIN * 100:.0f}%), against absolute's {absolute_dev * 100:.2f}%. "
        "The two anchors are behaving alike, so either the mode is not reaching the renderer or "
        "this scene is too dense to tell them apart -- check the scene's orientation spread "
        "before touching the band"
    )
