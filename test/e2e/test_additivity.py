"""End-to-end tests: partition additivity and unfiltered-intensity invariants.

Three companion tests share a single ``additivity_runs`` fixture (one
fresh-server simulation per config — ``filter_in`` / ``filter_out`` /
``no_filter``):

1. ``test_unfiltered_intensity_consistent`` — scalar ``unfiltered_snapshot_intensity``
   is filter-independent (regression guard from task-query-filter-uplift-v2).
2. ``test_partition_buffer_additivity`` — ``A.flt + B.flt ≈ N.unf`` in raw XYZ
   space, asserted on two layers:
     - **scalar primary**: ``|A.flt_intensity + B.flt_intensity - N.unf_intensity|
       / N.unf_intensity < _PARTITION_SCALAR_TOL``
     - **bright-pixel secondary**: per-pixel ``mean_rel`` over the bright mask
       (``N.unf > peak * 0.001``) ``< _PARTITION_BRIGHT_TOL``
   Replaces the buffer-wide ``mean_rel`` metric removed during scrum-193
   rollback — that metric was an MC noise floor (~36 %) in this sparse render
   regime and had no discriminatory power (see
   ``scratchpad/explore-partition-buffer-additivity-root-cause/SUMMARY.md``).
3. ``test_xyz_addition_beats_srgb`` — sRGB gamma non-linearity guard;
   XYZ-space addition must produce a lower display error than sRGB-space
   addition (the encode order matters for halo composition pipelines).

All three carry ``@pytest.mark.slow`` — they run on PR / main, not in the
default CI pytest invocation.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import run_scene_capi_buffered

CONFIGS_DIR = Path(__file__).parent / "configs"

# sRGB D65 XYZ → linear-RGB matrix. Verified against
# ``src/util/color_data.hpp:kXyzToRgb`` (constexpr float[9]) — values match
# element-wise as of 2026-05-18.
_XYZ_TO_RGB = np.array(
    [
        [3.2404542, -1.5371385, -0.4985314],
        [-0.9692660, 1.8760108, 0.0415560],
        [0.0556434, -0.2040259, 1.0572252],
    ],
    dtype=np.float64,
)

# Display normalization scale; matches ``src/server/render.cpp:31``
# ``constexpr float kNormScale = 0.08f``.
_K_NORM_SCALE = 0.08

# ── thresholds ───────────────────────────────────────────────────────────────

# Tolerance for the scalar ``unfiltered_snapshot_intensity`` consistency check
# (test_unfiltered_intensity_consistent). With 200k rays the MC stddev of the
# scalar is well below 1 %; 5 % gives ample headroom while still catching the
# pre-uplift ~95 % deviation. Inherited from task-query-filter-uplift-v2.
_REL_TOL = 0.05

# Primary scalar tolerance for partition additivity
# ``|A.snapshot + B.snapshot - N.unfiltered| / N.unfiltered``.
#
# Calibration: 5 fresh-build rounds via
# ``scratchpad/explore-partition-buffer-additivity-root-cause/probe_e1_baseline.py``
# (e1_results.json): 0.533 %, 0.550 %, 0.437 %, 0.229 %, 0.536 %.
# mean = 0.457 %, σ = 0.135 %, mean + 3σ = 0.862 % → ceiling to 0.5 % grid → 1.0 %.
_PARTITION_SCALAR_TOL = 0.01

# Secondary bright-pixel tolerance for partition additivity. PLACEHOLDER until
# Step 3 of task-partition-additivity-test-redesign replaces it with an
# empirically calibrated value (5 fresh-build rounds × mean + 3σ → 0.5 % grid).
# The placeholder 15 % is set conservatively above the expected single-run
# value (~6.7 % per sqrt(N) extrapolation from N=400 averaged 0.338 %).
_PARTITION_BRIGHT_TOL = 0.15  # PLACEHOLDER — calibrated in Step 3

# Bright-mask threshold: pixels whose any-channel XYZ value exceeds
# ``peak * _BRIGHT_MASK_FRAC`` are considered bright. Chosen to capture the
# halo signal while excluding dark-region Poisson noise.
_BRIGHT_MASK_FRAC = 0.001

# sRGB gamma non-linearity guard floor. From the f409e83 5M-ray experiment
# (XYZ error ~1 %, sRGB error ~12 %); 5 % gap is the minimum margin we require
# to call the linear-vs-encoded difference statistically meaningful.
#
# Single-run baseline (200k rays, bright mask N.unf > peak * 0.001):
# xyz_error ≈ 1.6 %, srgb_error ≈ 27.6 %, gap ≈ 26.0 %. The 5 % floor
# is therefore deeply within margin while still rejecting any accidental
# regression to sRGB-space composition.
_SRGB_NONLINEARITY_FLOOR = 0.05


# ── helpers ──────────────────────────────────────────────────────────────────


def _mean_rel(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.abs(a - b).mean() / (np.abs(b).mean() + 1e-12))


def _srgb_gamma(linear: np.ndarray) -> np.ndarray:
    c = np.clip(linear, 0.0, 1.0)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1.0 / 2.4) - 0.055)


def _xyz_to_srgb(xyz: np.ndarray) -> np.ndarray:
    return _srgb_gamma(np.tensordot(xyz, _XYZ_TO_RGB.T, axes=1))


def _display_xyz(buf: np.ndarray, intensity: float, total_pix: int) -> np.ndarray:
    if intensity <= 0.0:
        return np.zeros_like(buf)
    return buf * (_K_NORM_SCALE * total_pix / intensity)


# ── fixture ──────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def additivity_runs():
    """Run three independent simulations, each with a fresh server."""
    return {
        "filter_in":  run_scene_capi_buffered(str(CONFIGS_DIR / "rp46_additivity_in.json")),
        "filter_out": run_scene_capi_buffered(str(CONFIGS_DIR / "rp46_additivity_out.json")),
        "no_filter":  run_scene_capi_buffered(str(CONFIGS_DIR / "rp46_additivity_nof.json")),
    }


# ── tests ────────────────────────────────────────────────────────────────────


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


@pytest.mark.slow
def test_partition_buffer_additivity(additivity_runs):
    """``A.flt + B.flt ≈ N.unf`` — scalar primary + bright-pixel secondary.

    The old buffer-wide ``np.abs(diff).mean() / np.abs(ref).mean()`` was an MC
    noise floor (~36 %) in this sparse render regime — two independent N=1 runs
    on identical configs produced the same number whether or not partition
    additivity actually held (explore-202 decisive evidence: 35.78 % vs 35.99 %,
    diff < 1σ). Replaced here with a scalar primary + bright-region secondary;
    each carries an empirically calibrated threshold.
    """
    A = additivity_runs["filter_in"]
    B = additivity_runs["filter_out"]
    N = additivity_runs["no_filter"]

    assert N.unfiltered_intensity > 0, (
        f"baseline (no_filter) unfiltered intensity is zero: {N}"
    )

    # Primary: scalar partition additivity. Tight (1 %) because the scalar
    # noise floor at 200k rays is ~0.2-0.6 % (5 fresh-build rounds, mean+3σ).
    scalar_diff = abs(A.snapshot_intensity + B.snapshot_intensity - N.unfiltered_intensity)
    scalar_rel = scalar_diff / N.unfiltered_intensity
    assert scalar_rel < _PARTITION_SCALAR_TOL, (
        f"scalar partition additivity deviates by {scalar_rel:.3%}"
        f" (A.flt={A.snapshot_intensity:.6g}, B.flt={B.snapshot_intensity:.6g},"
        f" N.unf={N.unfiltered_intensity:.6g}, threshold {_PARTITION_SCALAR_TOL:.1%})"
    )

    # Secondary: bright-pixel buffer-wise mean_rel — restricts the metric to
    # pixels carrying actual halo signal so the dark-region Poisson noise does
    # not dominate the numerator.
    peak = float(N.unf_buf.max())
    bright_mask = (N.unf_buf > peak * _BRIGHT_MASK_FRAC).any(axis=-1)
    assert bright_mask.any(), (
        f"no bright pixels detected (peak={peak:.6g}, threshold={peak * _BRIGHT_MASK_FRAC:.6g})"
    )
    diff = A.flt_buf + B.flt_buf - N.unf_buf
    bright_mean_rel = (
        np.abs(diff[bright_mask]).mean() / (np.abs(N.unf_buf[bright_mask]).mean() + 1e-12)
    )
    assert bright_mean_rel < _PARTITION_BRIGHT_TOL, (
        f"bright-pixel partition additivity deviates by {bright_mean_rel:.3%}"
        f" (n_bright={int(bright_mask.sum())},"
        f" threshold {_PARTITION_BRIGHT_TOL:.1%})"
    )


@pytest.mark.slow
def test_xyz_addition_beats_srgb(additivity_runs):
    """Addition in linear XYZ must beat sRGB-space addition by ≥ 5 %.

    sRGB gamma encoding is non-linear: adding two encoded images systematically
    over-estimates bright areas. Composing halo layers must be done in linear
    XYZ followed by a single encode; this test guards against accidentally
    re-introducing the sRGB-space addition path.

    Both errors are evaluated over the bright mask (same one used in
    ``test_partition_buffer_additivity``). The historical f409e83 version
    ran at 5M rays where dark-pixel noise was negligible; at 200k rays the
    full-frame ``mean_rel`` is dominated by a ~46 % dark-region Poisson floor
    that shrinks the structural sRGB-vs-XYZ gap to ~4 % (sub-threshold).
    Restricting to bright pixels recovers the structural difference (gap
    ~26 % at single run).
    """
    A = additivity_runs["filter_in"]
    B = additivity_runs["filter_out"]
    N = additivity_runs["no_filter"]
    pix = A.img_width * A.img_height

    A_disp = _display_xyz(A.flt_buf, A.unfiltered_intensity, pix)
    B_disp = _display_xyz(B.flt_buf, B.unfiltered_intensity, pix)
    N_disp = _display_xyz(N.unf_buf, N.unfiltered_intensity, pix)
    N_srgb = _xyz_to_srgb(N_disp)

    peak = float(N.unf_buf.max())
    bright_mask = (N.unf_buf > peak * _BRIGHT_MASK_FRAC).any(axis=-1)
    assert bright_mask.any(), (
        f"no bright pixels detected (peak={peak:.6g}, threshold={peak * _BRIGHT_MASK_FRAC:.6g})"
    )

    xyz_diff = _xyz_to_srgb(A_disp + B_disp) - N_srgb
    srgb_diff = _xyz_to_srgb(A_disp) + _xyz_to_srgb(B_disp) - N_srgb
    ref = np.abs(N_srgb[bright_mask]).mean() + 1e-12
    xyz_error = float(np.abs(xyz_diff[bright_mask]).mean() / ref)
    srgb_error = float(np.abs(srgb_diff[bright_mask]).mean() / ref)

    assert srgb_error - xyz_error >= _SRGB_NONLINEARITY_FLOOR, (
        f"sRGB non-linearity guard: XYZ-space error ({xyz_error:.1%}) vs"
        f" sRGB-space error ({srgb_error:.1%}),"
        f" gap = {srgb_error - xyz_error:.1%} < floor {_SRGB_NONLINEARITY_FLOOR:.0%}"
    )
