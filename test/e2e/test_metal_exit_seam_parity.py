"""Metal exit-seam parity tests (258.6).

Replaces the 258.4 scaffolding that never actually exercised Metal (capi_runner
defaulted to the legacy CPU path; Metal-incompatible configs silently fell back).
This rewrite pins ground truth to **legacy CPU** and validates two new backends
against it on the same scenes, with two complementary metrics:

  - Axis A: Metal exit-seam        vs legacy CPU
  - Axis B: CpuTraceBackend        vs legacy CPU
  - Metric 1: raw XYZ Pearson corr (accumulated layer, GetRawXyzResults)
  - Metric 2: render-image PSNR    (final sRGB image, GetRenderResults)

Every test asserts Metal/Cpu actually ran (routed_backend matches AND no
"falling back" warning). The negative smoke test confirms the assertion
mechanism detects fallback. All tests are @pytest.mark.slow (shared-lib build:
``./scripts/build.sh -sj release``).

Concurrency contract: capi_runner.py mutates os.environ + uses a process-global
log callback. This suite MUST run serially (no pytest-xdist). Adding parallelism
requires subprocess isolation.

Thresholds: raw corr / render PSNR per scene are calibrated from baseline.md
(Step 5 measurement). Until baseline is finalized, placeholders (corr ≥ 0.90,
PSNR ≥ 30 dB) gate the obvious regressions; tighter per-scene values land with
baseline.md.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered

CONFIGS_DIR = Path(__file__).parent / "configs"
_SEED = 42
_TIMEOUT = 180


# --------------------------------------------------------------------------- #
# Metrics
# --------------------------------------------------------------------------- #

def _raw_corr(a: BufferedSimResult, b: BufferedSimResult) -> float:
    """Pearson correlation of raw XYZ buffers (linear; scale-invariant).

    Pearson is invariant to per-image scalar normalization, so any
    snapshot_intensity divergence between backends does NOT affect this
    metric — what's compared is the spatial structure of accumulated XYZ.
    """
    x = a.flt_buf.ravel().astype(np.float64)
    y = b.flt_buf.ravel().astype(np.float64)
    if x.std() == 0.0 or y.std() == 0.0:
        # Constant input → corrcoef returns NaN. Treat as definitive failure.
        return 0.0
    corr = float(np.corrcoef(x, y)[0, 1])
    if math.isnan(corr):
        return 0.0
    return corr


def _render_psnr(a: BufferedSimResult, b: BufferedSimResult) -> float:
    """PSNR between two rendered sRGB uint8 buffers (8-bit, 255 peak).

    Returns +inf if buffers are byte-identical.
    """
    if a.rgb_buf.shape != b.rgb_buf.shape:
        raise ValueError(f"render shape mismatch: {a.rgb_buf.shape} vs {b.rgb_buf.shape}")
    diff = a.rgb_buf.astype(np.float64) - b.rgb_buf.astype(np.float64)
    mse = float(np.mean(diff * diff))
    if mse == 0.0:
        return float("inf")
    return 10.0 * math.log10((255.0 * 255.0) / mse)


# --------------------------------------------------------------------------- #
# Backend execution helper
# --------------------------------------------------------------------------- #

def _run(config_name: str, backend: str) -> BufferedSimResult:
    cfg = str(CONFIGS_DIR / f"{config_name}.json")
    return run_scene_capi_buffered(cfg, sim_seed=_SEED, backend=backend, timeout_sec=_TIMEOUT)


def _assert_routed(r: BufferedSimResult, expected_backend: str, config_name: str) -> None:
    """Decision Point 1 (a): assert Metal/Cpu actually executed (no fallback).

    For legacy: no "routing via" log is emitted; _summarize_backend defaults to
    routed="legacy", so this assertion's real job is catching env pollution
    (e.g. a leaked LUMICE_TRACE_BACKEND from a prior test run), not positively
    confirming legacy executed.
    """
    assert r.routed_backend == expected_backend, (
        f"{config_name}/{expected_backend}: routed={r.routed_backend!r} "
        f"(expected {expected_backend!r}); core log did not emit the expected "
        f"routing line — see log_lines for the actual path."
    )
    assert not r.fell_back, (
        f"{config_name}/{expected_backend}: fell back to legacy (lens/view "
        f"likely incompatible). Backend was REQUESTED but did not run."
    )


# --------------------------------------------------------------------------- #
# Negative smoke — verify the fallback-detector itself works
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_metal_fallback_detector():
    """halo_22 uses fisheye_equal_area (lens_type=1) — Metal-incompatible.

    Forcing metal MUST trip the "falling back to legacy" log. This is the
    insurance that all other Axis-A (metal) assertions can meaningfully fail
    rather than silently pass on a degraded path.
    """
    r = _run("halo_22", "metal")
    assert r.fell_back, (
        "Expected fallback warning for Metal on fisheye_equal_area (lens_type=1) "
        f"but got fell_back=False. log_lines tail: {r.log_lines[-5:]}"
    )


# --------------------------------------------------------------------------- #
# Parity scenes — each test reports BOTH metrics on BOTH axes.
# Skips (not xfail) are used for crash-prone configs to prevent SIGABRT from
# blowing up the test runner — see 258.4 code-review C1, [[feedback_check_exit_code]].
# --------------------------------------------------------------------------- #

# Per-scene thresholds calibrated from baseline.md (Step 5, 2026-06-10).
# Raw corr threshold = floor(measured_corr * 100) / 100 − 0.02 (plan §4).
# Render PSNR threshold = 13.0 dB uniform — single-seed baseline lacks σ for
# mean-3σ calibration; 13 dB acts as a catastrophic-regression detector given
# the architectural raw vs render divergence documented in baseline.md F2.
#
# Filter scenes are xfailed (raw corr collapsed to ≤ 0.21 across both Metal
# and Cpu backends, matching 258.4 finding; root cause moved to 258.7/258.8).
# Non-filter scenes: floor(measured*100)/100 − 0.02 per plan §4.
# Filter scenes:     0.80 target (matches non-filter ceiling). Current
# baseline raw_corr ≤ 0.21 → assertion fails → xfail-strict=False marks the
# test as XFAIL. When 258.7/258.8 fix filter parity, the tests XPASS,
# signaling the xfail decorator should be removed.
_RAW_THRESHOLDS = {
    # config:                         (metal, cpu_backend)
    "dual_fisheye_ref":             (0.96, 0.95),
    "ms_multi_crystal":             (0.89, 0.81),
    "parity_ms_prob05":             (0.94, 0.92),
    "parity_single_ms_filter":      (0.80, 0.80),
    "ms_multi_crystal_filtered":    (0.80, 0.80),
    "parity_ms_prob05_filter":      (0.80, None),  # cpu_backend crashes (skip-marked)
}
_T_PSNR_DB = 13.0  # uniform render-PSNR floor; see baseline.md threshold section.


def _parity_axes(config_name: str) -> tuple[float, float, float, float]:
    """Return (corr_metal_vs_legacy, psnr_metal_vs_legacy,
                corr_cpu_vs_legacy,   psnr_cpu_vs_legacy)."""
    legacy = _run(config_name, "legacy")
    metal = _run(config_name, "metal")
    cpu = _run(config_name, "cpu_backend")

    _assert_routed(legacy, "legacy", config_name)
    _assert_routed(metal, "metal", config_name)
    _assert_routed(cpu, "cpu_backend", config_name)

    return (
        _raw_corr(metal, legacy),
        _render_psnr(metal, legacy),
        _raw_corr(cpu, legacy),
        _render_psnr(cpu, legacy),
    )


def _assert_parity(config_name: str, cm: float, pm: float, cc: float, pc: float) -> None:
    t_metal, t_cpu = _RAW_THRESHOLDS[config_name]
    assert cm >= t_metal, f"{config_name} Axis A raw_corr {cm:.4f} < {t_metal}"
    assert pm >= _T_PSNR_DB, f"{config_name} Axis A render PSNR {pm:.2f} dB < {_T_PSNR_DB}"
    if t_cpu is None:
        return  # cpu_backend skip-marked at suite level
    assert cc >= t_cpu, f"{config_name} Axis B raw_corr {cc:.4f} < {t_cpu}"
    assert pc >= _T_PSNR_DB, f"{config_name} Axis B render PSNR {pc:.2f} dB < {_T_PSNR_DB}"


# --- Single MS, no filter -------------------------------------------------- #

@pytest.mark.slow
def test_parity_single_ms_no_filter():
    cm, pm, cc, pc = _parity_axes("dual_fisheye_ref")
    print(f"[parity] dual_fisheye_ref: metal raw={cm:.4f} psnr={pm:.2f}dB | cpu_backend raw={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("dual_fisheye_ref", cm, pm, cc, pc)


# --- Single MS + filter ---------------------------------------------------- #

@pytest.mark.slow
@pytest.mark.xfail(
    reason="filter parity divergence under investigation — baseline raw_corr "
           "≤ 0.21 on both backends (see baseline.md F3); root cause moved to "
           "258.7 (cpu) / 258.8 (metal). Remove xfail after their fix lands.",
    strict=False,
)
def test_parity_single_ms_filter():
    cm, pm, cc, pc = _parity_axes("parity_single_ms_filter")
    print(f"[parity] parity_single_ms_filter: metal raw={cm:.4f} psnr={pm:.2f}dB | cpu_backend raw={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("parity_single_ms_filter", cm, pm, cc, pc)


# --- Multi MS prob=0.8, no filter ----------------------------------------- #

@pytest.mark.slow
def test_parity_multi_ms_prob08():
    cm, pm, cc, pc = _parity_axes("ms_multi_crystal")
    print(f"[parity] ms_multi_crystal: metal raw={cm:.4f} psnr={pm:.2f}dB | cpu_backend raw={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("ms_multi_crystal", cm, pm, cc, pc)


# --- Multi MS prob=0.8 + filter ------------------------------------------- #

@pytest.mark.slow
@pytest.mark.xfail(
    reason="filter parity divergence — see baseline.md F3; moved to 258.7/258.8.",
    strict=False,
)
def test_parity_multi_ms_prob08_filter():
    cm, pm, cc, pc = _parity_axes("ms_multi_crystal_filtered")
    print(f"[parity] ms_multi_crystal_filtered: metal raw={cm:.4f} psnr={pm:.2f}dB | cpu_backend raw={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("ms_multi_crystal_filtered", cm, pm, cc, pc)


# --- Multi MS prob=0.5, no filter ----------------------------------------- #

@pytest.mark.slow
def test_parity_multi_ms_prob05():
    cm, pm, cc, pc = _parity_axes("parity_ms_prob05")
    print(f"[parity] parity_ms_prob05: metal raw={cm:.4f} psnr={pm:.2f}dB | cpu_backend raw={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("parity_ms_prob05", cm, pm, cc, pc)


# --- Multi MS prob=0.5 + filter ------------------------------------------- #

@pytest.mark.slow
@pytest.mark.skip(
    reason="ms_prob05_filtered triggers SIGABRT on cpu_backend (258.4 finding, "
           "reproduced in baseline.md F4); skip (not xfail) to keep the runner "
           "alive — 258.4 code-review C1 + feedback_check_exit_code. Re-enable "
           "after 258.7 fix.",
)
def test_parity_multi_ms_prob05_filter():
    cm, pm, cc, pc = _parity_axes("parity_ms_prob05_filter")
    print(f"[parity] parity_ms_prob05_filter: metal raw={cm:.4f} psnr={pm:.2f}dB | cpu_backend raw={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("parity_ms_prob05_filter", cm, pm, cc, pc)
