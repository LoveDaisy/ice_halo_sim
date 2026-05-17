"""End-to-end test: ``unfiltered_snapshot_intensity`` is filter-independent.

Background (task-query-filter-uplift-v2): before the fix, the simulator
marked filter-fail rays as ``kStopped``, so the consumer's "unfiltered"
buffer was actually post-simulator-filter. For a 0.1%-pass-rate filter
like raypath ``[4,6]``, the reported ``unfiltered_snapshot_intensity``
collapsed by ~95% when the filter was enabled. After the fix the
simulator emit-gates every non-continue ray, and the consumer's Path B
sees the full pre-filter set — so the unfiltered intensity must match
across ``filter_in`` / ``filter_out`` / no-filter runs of an otherwise
identical scene.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from test.e2e.capi_runner import run_scene_capi

CONFIGS_DIR = Path(__file__).parent / "configs"

# Tolerance budget: with 200k rays and prob=0, the MC stddev of a per-pixel
# average intensity is well under 1%. 5% gives ample headroom while still
# catching the pre-fix ~95% deviation.
_REL_TOL = 0.05


@pytest.fixture(scope="module")
def additivity_runs():
    """Run three independent simulations, each with a fresh server."""
    return {
        "filter_in":  run_scene_capi(str(CONFIGS_DIR / "rp46_additivity_in.json")),
        "filter_out": run_scene_capi(str(CONFIGS_DIR / "rp46_additivity_out.json")),
        "no_filter":  run_scene_capi(str(CONFIGS_DIR / "rp46_additivity_nof.json")),
    }


@pytest.mark.slow
def test_unfiltered_intensity_consistent(additivity_runs):
    a = additivity_runs["filter_in"]
    b = additivity_runs["filter_out"]
    n = additivity_runs["no_filter"]

    assert n.unfiltered_intensity > 0, (
        f"baseline (no_filter) unfiltered intensity is zero: {n}"
    )

    rel_a = abs(a.unfiltered_intensity - n.unfiltered_intensity) / n.unfiltered_intensity
    rel_b = abs(b.unfiltered_intensity - n.unfiltered_intensity) / n.unfiltered_intensity

    assert rel_a < _REL_TOL, (
        f"filter_in unfiltered intensity deviates from no_filter by {rel_a:.1%}"
        f" (filter_in={a.unfiltered_intensity:.6g}, no_filter={n.unfiltered_intensity:.6g})"
    )
    assert rel_b < _REL_TOL, (
        f"filter_out unfiltered intensity deviates from no_filter by {rel_b:.1%}"
        f" (filter_out={b.unfiltered_intensity:.6g}, no_filter={n.unfiltered_intensity:.6g})"
    )
