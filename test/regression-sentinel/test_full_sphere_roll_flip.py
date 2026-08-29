"""End-to-end guard: an axis whose roll is fixed must not take the full-sphere fast path.

The reported symptom was that one config rendered two different halos depending on
whether the user had touched the Range slider. The mechanism, in three steps:

1. The GUI's sqrt-scaled Range slider stored ``sqrtf(360)**2 == 359.999969`` when dragged
   to its stop -- 3.05e-5 short of 360, and invisible behind a "%.1f" readout.
2. ``AxisDistribution::IsFullSphereUniform`` compares that range with an absolute
   epsilon of 1e-5, so it flipped, and ``lat_path::SelectLatPath`` rerouted the whole
   orientation sampler from ``kFullSphere`` to ``kLutInverseCdf``.
3. Those two paths were not equivalent. The general path applies the pole-crossing
   ``roll += pi`` that ``detail::NormalizeLatitude`` reports; the fast path never did.
   With a fixed roll, half the crystals came out rotated a further 180 degrees about
   their own c axis, which shifts the prism face numbering by three -- and this config
   filters on raypath [3,5], so it selected an entirely different set of physical paths.

The fix makes ``IsFullSphereUniform`` require roll to be rotationally symmetric, so the
fast path is only ever taken where dropping the correction is invisible in distribution.
The three configs below are the user's own, differing only in how they do (or do not)
approach the full-sphere condition.
"""

from __future__ import annotations

import numpy as np
import pytest

from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# One seed, shared by every arm. The first two arms are meant to be the same simulation
# down to the random stream, so they must be handed the same one to be compared at the
# precision below.
_SEED = 20260829

# The exact/drifted pair differs only by float rounding inside the accumulator: measured
# relative L1 over the XYZ image ran 4.6e-8 to 2.8e-5 across six seeds, against 0.63-0.65
# in the defect state. This sits ~40x above the largest agreeing value and ~650x below the
# smallest disagreeing one.
_MAX_REL_L1 = 1e-3

# The third arm resamples rather than reproduces (see the docstring on the second test), so
# it is compared on aggregates, not pixels. Measured spread of the total-energy difference
# across six seeds: 0 to 1.3%. In the defect state the fast-path arm carried 1.8x the energy.
_MAX_REL_TOTAL = 0.08

# Rear-hemisphere energy share on the general path, measured across six seeds: 0.0092 to
# 0.0116 on either arm. On the fast path it was exactly 0.00000000 -- the sharpest single
# fingerprint of the defect, and the reason the lower bound here is what matters.
_REAR_FRACTION_RANGE = (0.004, 0.025)


def _arm(config_name: str) -> np.ndarray:
    """Luminance channel of one arm's accumulated XYZ image."""
    result = run_scene_capi_buffered(str(CONFIGS_DIR / config_name), sim_seed=_SEED, timeout_sec=600)
    assert result.has_valid_data, f"{config_name}: server returned no valid data"
    luminance = result.flt_buf[..., 1].astype(np.float64)
    assert luminance.sum() > 0.0, f"{config_name}: rendered nothing at all"
    return luminance


def _rel_l1(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.abs(a - b).sum() / a.sum())


def _rear_fraction(luminance: np.ndarray) -> float:
    """Share of the energy in the right half of the dual-fisheye image, i.e. the rear hemisphere."""
    return float(luminance[:, luminance.shape[1] // 2 :].sum() / luminance.sum())


@pytest.mark.slow
def test_a_slider_sized_range_drift_does_not_change_the_halo():
    """The user's reproduction: zenith range 360 against 359.9999694824219.

    A difference of 3.05e-5 degrees in a range must not be able to select a different
    physical answer. It could, and did.
    """
    exact = _arm("full_sphere_roll_flip_exact.json")
    drifted = _arm("full_sphere_roll_flip_drifted.json")
    difference = _rel_l1(exact, drifted)
    assert difference < _MAX_REL_L1, f"a 3.05e-5 deg range difference moved the image: relative L1 {difference:.3g}"


@pytest.mark.slow
def test_both_arms_land_on_the_pole_crossing_answer_not_the_fast_path_one():
    """Agreement alone is not the property under test; agreeing on the RIGHT answer is.

    Two arms would also agree if a later change sent both down the fast path again -- and
    that is the defect, not the fix. So this pins the answer itself, from two directions.

    The first is a third arm that reaches the general path through a condition this task
    did not touch: a zenith centered at 0.001 deg rather than 0, which fails
    IsFullSphereUniform's center check. It is on the general path both before and after the
    fix, so it says independently what the correct image is. It is compared on aggregates
    rather than pixel by pixel, deliberately: shifting the center by 0.001 deg re-maps the
    random stream onto slightly different angles, and against a filter as sparse as
    raypath [3,5] that changes WHICH rays survive, not just where they land. Measured
    per-pixel L1 against the main arm was bimodal across seeds (0 on three of six, 0.14 to
    0.52 on the rest) while total energy stayed within 1.3% -- ordinary Monte-Carlo
    resampling on a sparse image, not instability in the physics.

    The second is the rear hemisphere, which must carry energy. Under the fast path it
    carried exactly none (0.00000000 against ~0.010, with 1.8x the total energy), and no
    run that applies the pole-crossing correction can reproduce that zero.
    """
    exact = _arm("full_sphere_roll_flip_exact.json")
    mean_shifted = _arm("full_sphere_roll_flip_mean_shifted.json")

    total_difference = abs(float(exact.sum() - mean_shifted.sum())) / float(exact.sum())
    assert total_difference < _MAX_REL_TOTAL, (
        "the full-360 arm carries a different amount of energy from an arm that reaches the general "
        f"path by another route: relative difference {total_difference:.3g}"
    )

    low, high = _REAR_FRACTION_RANGE
    for name, luminance in (("exact", exact), ("mean_shifted", mean_shifted)):
        rear = _rear_fraction(luminance)
        assert low < rear < high, (
            f"{name}: rear-hemisphere energy share {rear:.8f} is outside [{low}, {high}] — "
            "exactly 0 is the fast-path signature this test exists to catch"
        )
