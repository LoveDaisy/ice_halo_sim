"""Energy additivity on the absolute display scale.

The renderer normalizes by the energy the light source EMITTED, not by the energy
that landed on a pixel. The point of that substitution is comparability: the
denominator stops depending on what the scene happens to do with its rays, so two
scenes at the same EV differ in brightness by their real physical difference.

Additivity is the sharpest statement of that property that can be checked from
outside. Split one scene's rays into two complementary halves with a filter --
``filter_in`` on a raypath and ``filter_out`` on the same one -- and, with no
multi-scattering to redistribute anything (``prob = 0``), the two halves must add
back up to the whole ON THE NORMALIZED SCALE. Under the old landed-weight
denominator they did not, and could not: each half was divided by its own smaller
landed weight, i.e. re-brightened by exactly the amount its filter had removed, so
the two halves summed to roughly twice the whole. That factor of two is measured
here as a negative control rather than described, so the pass is known to mean
something.

Caliber. Two independent readings of the same claim are checked, because they have
different noise: the summed Y over the image (``sumY``) and the scalar landed
weight the server reports (``snapshot_intensity``). The scalar reading is roughly
5x tighter than the image one.

Not a per-pixel criterion, deliberately. Filter-fail rays terminate early
(``doc/filter-architecture.md``, Design A), which changes how fast the shared RNG
stream is consumed, so the three runs do not light the same pixels -- the per-pixel
relative difference has a median near 1.0 while the surplus and the deficit cancel
almost exactly in the sum. Integrated calibers are the ones that survive that, and
it has nothing to do with multi-scattering.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# Fixed so the three runs are reproducible against each other. A non-zero
# sim_seed also converges the server onto a single worker, which is what makes
# the run bit-reproducible at all; it does NOT make different configs comparable,
# and nothing here assumes it does.
_SEED = 42

# Tolerance for the image caliber. Seed-to-seed spread of a single config
# measures ~1.5% here (four seeds), so this is a little over 3 sigma. The
# negative control below lands near 105%, so the band is not chosen to be
# generous enough to swallow the effect it exists to catch.
_SUM_Y_TOL = 0.06

# Same, for the scalar caliber, whose measured spread is ~0.35%.
_SNAP_TOL = 0.02

# What the negative control must clear to count as a control at all: the old
# denominator has to fail this criterion by a margin no amount of Monte-Carlo
# noise could produce. Measured deviation is ~105%.
_CONTROL_MIN = 0.50


def _run(name: str):
    return run_scene_capi_buffered(
        str(CONFIGS_DIR / f"absolute_additivity_{name}.json"),
        sim_seed=_SEED,
        backend="legacy",
        timeout_sec=600,
    )


def _sum_y(result) -> float:
    y = result.flt_buf.reshape(result.img_height, result.img_width, 3)[:, :, 1]
    return float(y.astype(np.float64).sum())


@pytest.fixture(scope="module")
def halves():
    """The three runs: the whole scene and its two complementary halves."""
    out = {}
    for name in ("all", "in", "out"):
        r = _run(name)
        assert r.has_valid_data, f"{name}: simulation produced no data"
        out[name] = {
            "sum_y": _sum_y(r),
            "snap": float(r.snapshot_intensity),
            "emitted": float(r.emitted_energy),
        }
    return out


def _excess(parts, key: str) -> float:
    """(in + out - all) / all for one caliber -- 0 when the halves partition."""
    return (parts["in"][key] + parts["out"][key] - parts["all"][key]) / parts["all"][key]


@pytest.mark.slow
def test_denominator_is_the_same_for_every_filter_split(halves):
    """The absolute denominator must not know the filter exists.

    All three configs emit the same rays from the same source; only what is kept
    afterwards differs. So the emitted energy is not merely close across them, it
    is the same number -- which is the whole claim of the change, stated without
    reference to any image.
    """
    all_emitted = halves["all"]["emitted"]
    assert all_emitted > 0.0, "emitted energy was never charged"
    for name in ("in", "out"):
        assert halves[name]["emitted"] == pytest.approx(all_emitted, rel=1e-6), (
            f"{name} config reports emitted energy {halves[name]['emitted']!r} against "
            f"{all_emitted!r} for the unfiltered scene -- the denominator is tracking "
            "the filter, which is exactly what it must not do"
        )


@pytest.mark.slow
def test_the_two_halves_add_back_to_the_whole(halves):
    """E_in + E_out == E_all, on both calibers."""
    sum_y_excess = _excess(halves, "sum_y")
    assert abs(sum_y_excess) <= _SUM_Y_TOL, (
        f"summed-Y additivity off by {sum_y_excess * 100:+.3f}% "
        f"(band +/-{_SUM_Y_TOL * 100:.1f}%)"
    )

    snap_excess = _excess(halves, "snap")
    assert abs(snap_excess) <= _SNAP_TOL, (
        f"landed-weight additivity off by {snap_excess * 100:+.3f}% "
        f"(band +/-{_SNAP_TOL * 100:.1f}%)"
    )


@pytest.mark.slow
def test_the_old_denominator_would_fail_this_criterion(halves):
    """Negative control, computed from the same three runs -- no extra simulation.

    Re-derive each run's brightness the way it was derived before this change, by
    dividing by that run's own landed weight instead of by the energy it emitted,
    and re-run the additivity check on the result. It must fail, and fail hugely.

    Without this, the two assertions above would also pass for a denominator that
    ignored its input entirely, and there would be no evidence in the suite that
    the criterion can tell the two normalizations apart.
    """
    relative = {name: {"v": parts["sum_y"] / parts["snap"]} for name, parts in halves.items()}
    control = _excess(relative, "v")
    assert abs(control) >= _CONTROL_MIN, (
        f"the old landed-weight denominator deviates by only {control * 100:+.2f}%, "
        f"under the {_CONTROL_MIN * 100:.0f}% this control needs to demonstrate that the "
        "additivity criterion can distinguish the two normalizations -- the criterion "
        "above is passing for some reason other than the one claimed"
    )
