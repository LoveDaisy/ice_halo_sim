"""Regression guard: degenerate-prism plane misclassified as internal next-face.

Fix commits:
- b7317619 fix(core): outward-ray short-circuit in PropagateSlab
- c3aea20f fix(metal): outward-ray short-circuit in trace kernel + mirror in parity oracle

Root cause (white-box proven, see scratchpad/task-degenerate-prism-plane-exit-misclassify/issue.md):
``Propagate`` (``src/core/optics.cpp``) and the Metal trace kernel selected
next-face as "the closest half-space plane the ray is exiting (denom > eps) at
t > 0", *without* a polygon-membership check. On a convex crystal that is
correct for rays travelling strictly *inside* the body — but for first-bounce
external reflections at the surface, the infinite plane of a degenerate
zero-height prism wall (``prism_h=0``) can still be hit at positive t even
though its polygon is empty. The ray was then mis-classified as internal
transit instead of outgoing, producing zero output at ``max_hits=1`` for
extremely flat double-pyramids around wedge 87.4-88°.

The fix adds a convex-body outward short-circuit: if a child ray leaves the
source face with ``d · n_src > 0`` it cannot hit any other polygon face, so
``next_face`` stays ``kInvalidId`` and the ray exits cleanly.

This test pins the canonical B-ring scenario (``pyramid``, ``prism_h=0``,
``upper/lower_wedge_angle=88°``, ``max_hits=1``, ``dual_fisheye_equal_area``
lens — Metal-compatible) and asserts that the legacy CPU and Metal backends
both produce a non-zero, broadly consistent image. Pre-fix legacy snapshot
intensity was 0 at this wedge and Metal hung; post-fix both should match
within a loose tolerance.

Requires shared-lib build: ./scripts/build.sh -sj release
Run: pytest test/regression-sentinel/test_degenerate_prism_b_ring.py -v -m slow
"""

from __future__ import annotations

import sys

import pytest

from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

_CONFIG = get_project_root() / "test" / "e2e" / "configs" / "degenerate_prism_b_ring.json"

_TEST_SEED = 42

# Pre-fix legacy snapshot_intensity at this config is exactly 0 (all first-
# bounce external reflections are mis-classified as internal and dropped at
# max_hits=1). Post-fix both legacy and Metal land at ~23 with seed=42; a
# floor of 5.0 leaves a wide margin while cleanly separating "fix works"
# (>>0) from "fix regressed" (==0). Tighten only if the post-fix value
# drifts up.
_MIN_INTENSITY = 5.0


@pytest.mark.slow
def test_degenerate_prism_legacy_emits_nonzero_at_mh1():
    """Legacy CPU produces non-zero output at the degenerate B-ring scenario.

    Pre-fix: snapshot_intensity == 0 (issue.md "100% → 0% hard flip" at wedge
    87.5° max_hits=1). Post-fix: outward-ray short-circuit lets first-bounce
    external reflections exit, restoring non-zero output.
    """
    result = run_scene_capi_buffered(str(_CONFIG), sim_seed=_TEST_SEED, backend="legacy")
    assert result.snapshot_intensity > _MIN_INTENSITY, (
        "Legacy CPU produced near-zero output on the degenerate B-ring scenario "
        f"(snapshot_intensity={result.snapshot_intensity:.3f}). The outward-ray "
        "short-circuit may have regressed — first-bounce external reflections are "
        "being mis-classified as internal transit at prism_h=0 + wedge=88°."
    )


@pytest.mark.slow
@pytest.mark.skipif(sys.platform != "darwin", reason="Metal backend is macOS-only")
def test_degenerate_prism_metal_does_not_hang_and_matches_legacy():
    """Metal completes within the timeout and broadly matches legacy at this scene.

    Pre-fix Metal hung at ``max_hits=1`` on this geometry (issue.md). Post-fix
    Metal must (a) return without timing out and (b) produce a non-zero output
    of the same order of magnitude as legacy.
    """
    metal = run_scene_capi_buffered(str(_CONFIG), sim_seed=_TEST_SEED, backend="metal", timeout_sec=60)
    if metal.routed_backend != "metal" or metal.fell_back:
        pytest.skip(
            f"Metal backend not active for this scene (routed={metal.routed_backend!r}, "
            f"fell_back={metal.fell_back}); skipping cross-backend check."
        )

    legacy = run_scene_capi_buffered(str(_CONFIG), sim_seed=_TEST_SEED, backend="legacy")

    assert metal.snapshot_intensity > _MIN_INTENSITY, (
        f"Metal snapshot_intensity={metal.snapshot_intensity:.3f} too low — "
        "outward-ray short-circuit in the Metal kernel may have regressed."
    )
    # Loose order-of-magnitude check: legacy and Metal differ in RNG / face
    # selection details and we only need to catch a real divergence, not enforce
    # per-pixel parity.
    ratio = metal.snapshot_intensity / max(legacy.snapshot_intensity, 1e-6)
    assert 0.3 <= ratio <= 3.0, (
        f"Metal vs legacy snapshot_intensity ratio {ratio:.3f} out of [0.3, 3.0] "
        f"(metal={metal.snapshot_intensity:.3f}, legacy={legacy.snapshot_intensity:.3f}). "
        "Backends diverging on the degenerate B-ring scene — check whether the "
        "Metal-side outward short-circuit matches the CPU side."
    )
