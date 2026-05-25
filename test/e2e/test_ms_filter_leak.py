"""End-to-end regression tests for the MS filter leak bug.

The bug: in multi-scattering configs, outgoing candidates that fail a per-crystal
filter in layer N but pass the probability check (anchor-lane bypass) carry no
persistent flag. In layer N+1, these rays can reach IsOutgoing() and leak into the
main output buffer, producing extra arcs/halos.

Fix: ``RaySeg::is_prior_filter_failed_`` persists filter failure across MS layers.
Rays carrying this flag route to the anchor lane even if they pass the current
layer's filter.

Two tests:
1. ``test_impossible_filter_produces_zero_intensity`` — layer 1 uses an impossible
   raypath filter [1,1] (cannot occur in a convex hexagonal prism); all rays fail
   the filter. With the fix, no ray reaches the main output (snapshot_intensity == 0).

2. ``test_repro_scenario_matches_single_layer`` — the issue's exact reproduction
   config (lowitz + transparent-plate layer 2) compared against the single-layer
   baseline. With the fix, the normalized images are visually identical (high PSNR).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import run_scene_capi, run_scene_capi_buffered

CONFIGS_DIR = Path(__file__).parent / "configs"

_K_NORM_SCALE = 0.08

_REPRO_TEST_SEED = 42

_REPRO_PSNR_THRESHOLD = 25.0


def _display_xyz(buf: np.ndarray, intensity: float, total_pix: int) -> np.ndarray:
    if intensity <= 0.0:
        return np.zeros_like(buf)
    return buf * (_K_NORM_SCALE * total_pix / intensity)


def _psnr(a: np.ndarray, b: np.ndarray) -> float:
    mse = float(np.mean((a - b) ** 2))
    if mse < 1e-12:
        return float("inf")
    peak = max(float(a.max()), float(b.max()), 1e-12)
    return 10.0 * np.log10(peak**2 / mse)


@pytest.mark.slow
def test_impossible_filter_produces_zero_intensity():
    """Layer 1 impossible filter [1,1] + layer 2 no filter → snapshot_intensity == 0.

    Raypath [1,1] (face 1 to face 1 consecutively) is geometrically impossible in a
    convex hexagonal prism: after hitting face 1, convexity guarantees the next face
    hit is always different. All rays fail the filter, so with the fix none reach the
    main output buffer.
    """
    result = run_scene_capi(str(CONFIGS_DIR / "ms_filter_leak_impossible.json"))
    assert result.snapshot_intensity == pytest.approx(0.0, abs=1e-6), (
        f"Expected zero snapshot_intensity (impossible filter blocks all rays), "
        f"got {result.snapshot_intensity:.6g}"
    )


@pytest.mark.slow
def test_repro_scenario_matches_single_layer():
    """2-layer MS (lowitz + transparent plate) matches single-layer baseline.

    Layer 2's filter [1,2] PBD selects basal-to-basal paths (parallel-surface
    transmission — no refraction). With the fix, filter-failed rays from layer 1
    do not leak through layer 2. The normalized output should closely match the
    single-layer baseline.
    """
    repro = run_scene_capi_buffered(str(CONFIGS_DIR / "ms_filter_leak_repro.json"), sim_seed=_REPRO_TEST_SEED)
    baseline = run_scene_capi_buffered(str(CONFIGS_DIR / "ms_filter_leak_baseline.json"), sim_seed=_REPRO_TEST_SEED)

    assert baseline.snapshot_intensity > 0, "baseline snapshot_intensity is zero"
    assert baseline.anchor_snapshot_intensity > 0, "baseline anchor is zero"

    pix = baseline.img_width * baseline.img_height
    norm_baseline = baseline.anchor_snapshot_intensity
    norm_repro = repro.anchor_snapshot_intensity if repro.anchor_snapshot_intensity > 0 else repro.snapshot_intensity

    disp_baseline = _display_xyz(baseline.flt_buf, norm_baseline, pix)
    disp_repro = _display_xyz(repro.flt_buf, norm_repro, pix)

    psnr_val = _psnr(disp_baseline, disp_repro)
    assert psnr_val >= _REPRO_PSNR_THRESHOLD, (
        f"Repro scenario PSNR too low: {psnr_val:.1f} dB < {_REPRO_PSNR_THRESHOLD} dB. "
        f"Layer 2 transparent-plate filter may be leaking extra light paths. "
        f"baseline_snap={baseline.snapshot_intensity:.6g}, "
        f"repro_snap={repro.snapshot_intensity:.6g}"
    )
