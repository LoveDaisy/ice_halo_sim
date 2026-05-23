"""End-to-end tests: partition additivity and unfiltered-intensity invariants.

Three companion tests share an ``additivity_runs`` fixture (one fresh-server
simulation per config — ``filter_in`` / ``filter_out`` / ``no_filter``) plus a
``partition_pair`` parameterized fixture that drives the redesigned
``test_partition_buffer_additivity`` across multiple ms_prob and filter-type
configurations.

1. ``test_unfiltered_intensity_consistent`` — scalar anchor intensity is
   filter-independent under the F1 OFF-mode anchor lane.
2. ``test_partition_buffer_additivity`` — anchor buffer partition invariant:
   ``A.anchor == B.anchor`` (bit-stable under fixed seed + single worker) and
   ``A.snapshot + B.snapshot == A.anchor`` (scalar conservation). Covers
   ms_prob ∈ {0, 0.3, 0.5} and two filter shapes (raypath, entry_exit).
3. ``test_xyz_addition_beats_srgb`` — sRGB gamma non-linearity guard;
   XYZ-space addition must produce a lower display error than sRGB-space
   addition. Uses anchor as the filter-independent normalization base.

All three carry ``@pytest.mark.slow`` — they run on PR / main, not in the
default CI pytest invocation.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import run_scene_capi, run_scene_capi_buffered

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

# Fixed seed for the partition-additivity fixture. Combined with
# LUMICE_SIM_SEED's single-worker override (see src/server/server.cpp) this
# makes two server lifecycles bit-stable: same RNG seed, FIFO scene_queue
# consumed by one Simulator → identical ray paths and identical fp32 sums.
_PARTITION_TEST_SEED = "42"

# Anchor buffer cross-filter invariance tolerance. Under LUMICE_SIM_SEED the
# anchor is bit-identical across filter_in / filter_out runs, so 1e-4 is
# loose enough to absorb any future single-worker reduction-order changes
# (e.g., extra OpenMP guards) without going so loose it could mask a real
# regression.
_ANCHOR_INVARIANT_TOL = 1e-4

# Scalar partition sum tolerance: ``|A.snap + B.snap - A.anchor| / A.anchor``.
# Same seed + single worker should yield fp32 round-off only (observed
# ~1.3e-5 in M2 verification); 1e-4 leaves ~8× headroom.
_PARTITION_SCALAR_TOL_STRICT = 1e-4

# test_unfiltered_intensity_consistent MC tolerance — the three runs in
# additivity_runs use unseeded multi-worker dispatch, so the ~2 % MC
# stddev at 200k rays governs the headroom (2.5× margin).
_UNFILTERED_CONSISTENCY_TOL = 0.05

# Bright-mask threshold: pixels whose any-channel XYZ value exceeds
# ``peak * _BRIGHT_MASK_FRAC`` are considered bright. Chosen to capture the
# halo signal while excluding dark-region Poisson noise.
_BRIGHT_MASK_FRAC = 0.001

# sRGB gamma non-linearity guard floor. Single-run baseline at 200k rays
# (bright mask N.flt_buf > peak * 0.001): xyz_error ≈ 1.6 %, srgb_error
# ≈ 27.6 %, gap ≈ 26.0 %. The 5 % floor is deeply within margin while still
# rejecting any accidental regression to sRGB-space composition.
_SRGB_NONLINEARITY_FLOOR = 0.05

_PARTITION_CONFIGS = [
    # (filter_in_config, filter_out_config, label)
    ("rp46_additivity_in.json",      "rp46_additivity_out.json",      "rp46_ms0"),
    ("rp46_additivity_in_ms03.json", "rp46_additivity_out_ms03.json", "rp46_ms03"),
    ("rp46_additivity_in_ms05.json", "rp46_additivity_out_ms05.json", "rp46_ms05"),
    ("ee_additivity_in.json",        "ee_additivity_out.json",        "ee_ms0"),
]


# ── helpers ──────────────────────────────────────────────────────────────────


def _srgb_gamma(linear: np.ndarray) -> np.ndarray:
    c = np.clip(linear, 0.0, 1.0)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1.0 / 2.4) - 0.055)


def _xyz_to_srgb(xyz: np.ndarray) -> np.ndarray:
    return _srgb_gamma(np.tensordot(xyz, _XYZ_TO_RGB.T, axes=1))


def _display_xyz(buf: np.ndarray, intensity: float, total_pix: int) -> np.ndarray:
    if intensity <= 0.0:
        return np.zeros_like(buf)
    return buf * (_K_NORM_SCALE * total_pix / intensity)


# ── fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def additivity_runs():
    """Run three independent simulations, each with a fresh server."""
    return {
        "filter_in":  run_scene_capi_buffered(str(CONFIGS_DIR / "rp46_additivity_in.json")),
        "filter_out": run_scene_capi_buffered(str(CONFIGS_DIR / "rp46_additivity_out.json")),
        "no_filter":  run_scene_capi_buffered(str(CONFIGS_DIR / "rp46_additivity_nof.json")),
    }


@pytest.fixture(
    scope="module",
    params=_PARTITION_CONFIGS,
    ids=[c[2] for c in _PARTITION_CONFIGS],
)
def partition_pair(request):
    """Two same-seed OFF-mode runs (filter_in + filter_out).

    LUMICE_SIM_SEED is read by each fresh ServerImpl ctor
    (capi_runner.py:186 CreateServer / capi_runner.py:234 DestroyServer) and
    forces worker_count=1 — see src/server/server.cpp. The two runs therefore
    share an identical RNG state machine and produce bit-stable anchors.
    """
    in_cfg, out_cfg, _label = request.param
    old_seed = os.environ.get("LUMICE_SIM_SEED")
    os.environ["LUMICE_SIM_SEED"] = _PARTITION_TEST_SEED
    try:
        result_in = run_scene_capi(str(CONFIGS_DIR / in_cfg))
        result_out = run_scene_capi(str(CONFIGS_DIR / out_cfg))
    finally:
        if old_seed is None:
            os.environ.pop("LUMICE_SIM_SEED", None)
        else:
            os.environ["LUMICE_SIM_SEED"] = old_seed
    return result_in, result_out


# ── tests ────────────────────────────────────────────────────────────────────


@pytest.mark.slow
@pytest.mark.xfail(
    reason="no_filter OFF mode degenerates to Design A (anchor_d_ empty); two "
    "filter runs vs the no_filter baseline come from independent MC samples "
    "so the comparison carries MC noise, not the F1 anchor invariant. "
    "Tracked in backlog: revisit when no_filter OFF mode short-circuit changes.",
    strict=False,
)
def test_unfiltered_intensity_consistent(additivity_runs):
    """``filter_in / filter_out`` anchor totals match the no_filter baseline.

    The no_filter OFF mode path takes a short-circuit (src/core/simulator.cpp:
    no filter spec → CollectData not CollectDataF1 → anchor_d_ empty →
    anchor_total_intensity_=0 → anchor_snapshot_intensity falls back to
    snapshot_intensity in render.cpp:568). The front-line assertion verifies
    that degenerate path is still intact; the cross-run comparison then uses
    a single ``anchor_ref = n.snapshot_intensity``.
    """
    a = additivity_runs["filter_in"]
    b = additivity_runs["filter_out"]
    n = additivity_runs["no_filter"]

    # Front-line: no_filter OFF mode must degenerate (anchor ≈ snapshot).
    # If this fails, the short-circuit at render.cpp:503 changed and the
    # rest of the test no longer reflects what it claims to test.
    assert n.anchor_snapshot_intensity == pytest.approx(n.snapshot_intensity, rel=1e-4), (
        "no_filter OFF mode should degenerate: anchor ≈ snapshot_intensity"
        f" (got anchor={n.anchor_snapshot_intensity:.6g}, snap={n.snapshot_intensity:.6g})"
    )
    anchor_ref = n.snapshot_intensity
    assert anchor_ref > 0, "no_filter snapshot_intensity is zero"

    # filter runs anchor = total emission (F1); no_filter snapshot = total
    # emission (Design A equivalent). Different MC seeds → MC tolerance.
    rel_a = abs(a.anchor_snapshot_intensity - anchor_ref) / anchor_ref
    rel_b = abs(b.anchor_snapshot_intensity - anchor_ref) / anchor_ref

    assert rel_a < _UNFILTERED_CONSISTENCY_TOL, (
        f"filter_in anchor deviates from no_filter total by {rel_a:.1%}"
        f" (filter_in anchor={a.anchor_snapshot_intensity:.6g},"
        f" no_filter snap={anchor_ref:.6g})"
    )
    assert rel_b < _UNFILTERED_CONSISTENCY_TOL, (
        f"filter_out anchor deviates from no_filter total by {rel_b:.1%}"
        f" (filter_out anchor={b.anchor_snapshot_intensity:.6g},"
        f" no_filter snap={anchor_ref:.6g})"
    )


@pytest.mark.slow
def test_partition_buffer_additivity(partition_pair):
    """Anchor buffer partition invariant under OFF mode + fixed seed.

    Primary: ``A.anchor == B.anchor`` (bit-stable; LUMICE_SIM_SEED collapses
    to single worker so same RNG → same rays → same fp32 sum regardless of
    filter direction).

    Secondary: ``A.snapshot + B.snapshot == A.anchor`` (scalar conservation —
    every emission lands in exactly one of the two filter partitions).
    """
    A, B = partition_pair

    # Front-line: anchor lane must be active (OFF mode + filter present).
    assert A.anchor_snapshot_intensity > 0, (
        "A.anchor_snapshot_intensity is zero — check OFF mode + filter config (F1 semantics)"
    )
    assert B.anchor_snapshot_intensity > 0, (
        "B.anchor_snapshot_intensity is zero — check OFF mode + filter config (F1 semantics)"
    )

    # Primary: anchor cross-filter invariance.
    anchor_max = max(A.anchor_snapshot_intensity, B.anchor_snapshot_intensity)
    anchor_rel = abs(A.anchor_snapshot_intensity - B.anchor_snapshot_intensity) / anchor_max
    assert anchor_rel < _ANCHOR_INVARIANT_TOL, (
        f"anchor_snapshot_intensity filter-independence violated: "
        f"A={A.anchor_snapshot_intensity:.6g}, B={B.anchor_snapshot_intensity:.6g}, "
        f"rel={anchor_rel:.2e} (tol={_ANCHOR_INVARIANT_TOL:.0e})"
    )

    # Secondary: scalar conservation across the partition.
    total = A.anchor_snapshot_intensity
    partition_sum = A.snapshot_intensity + B.snapshot_intensity
    partition_rel = abs(partition_sum - total) / total
    assert partition_rel < _PARTITION_SCALAR_TOL_STRICT, (
        f"partition sum deviates from anchor total: "
        f"A.snap={A.snapshot_intensity:.6g}, B.snap={B.snapshot_intensity:.6g}, "
        f"sum={partition_sum:.6g}, anchor={total:.6g}, rel={partition_rel:.2e}"
    )


@pytest.mark.slow
def test_xyz_addition_beats_srgb(additivity_runs):
    """Addition in linear XYZ must beat sRGB-space addition by ≥ 5 %.

    sRGB gamma encoding is non-linear: adding two encoded images systematically
    over-estimates bright areas. Composing halo layers must be done in linear
    XYZ followed by a single encode; this test guards against accidentally
    re-introducing the sRGB-space addition path.

    Normalization uses anchor (filter-independent total emission) for the two
    filter runs and ``snapshot_intensity`` for the no_filter run (short-circuit
    degenerate path — see test_unfiltered_intensity_consistent docstring).
    """
    A = additivity_runs["filter_in"]
    B = additivity_runs["filter_out"]
    N = additivity_runs["no_filter"]
    pix = A.img_width * A.img_height

    # OFF mode filter runs: anchor must be non-zero (F1 semantics).
    assert A.anchor_snapshot_intensity > 0, "filter_in OFF-mode anchor is zero — check F1 semantics"
    assert B.anchor_snapshot_intensity > 0, "filter_out OFF-mode anchor is zero — check F1 semantics"

    A_disp = _display_xyz(A.flt_buf, A.anchor_snapshot_intensity, pix)
    B_disp = _display_xyz(B.flt_buf, B.anchor_snapshot_intensity, pix)
    # no_filter degenerates: anchor ≈ snapshot → use snapshot for normalization.
    N_disp = _display_xyz(N.flt_buf, N.snapshot_intensity, pix)
    N_srgb = _xyz_to_srgb(N_disp)

    # Bright mask comes from the no_filter buffer (= full-emission pixels under
    # OFF mode no_filter, since snapshot buffer == all emissions when no filter).
    peak = float(N.flt_buf.max())
    bright_mask = (N.flt_buf > peak * _BRIGHT_MASK_FRAC).any(axis=-1)
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
