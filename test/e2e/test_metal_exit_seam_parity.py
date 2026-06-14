"""Metal exit-seam parity tests (258.6).

Replaces the 258.4 scaffolding that never actually exercised Metal (capi_runner
defaulted to the legacy CPU path; Metal-incompatible configs silently fell back).
This rewrite pins ground truth to **legacy CPU** and validates two new backends
against it on the same scenes, with two complementary metrics:

  - Axis A: Metal exit-seam        vs legacy CPU
  - Axis B: CpuTraceBackend        vs legacy CPU
  - Metric 1: raw XYZ Pearson corr (accumulated layer, GetRawXyzResults)
                — 258.6.2 switched to block-mean (4×4) downsampled Pearson
                  (`_raw_corr_ds`) to suppress Monte-Carlo speckle floor on
                  sparse filter scenes. See baseline.md threshold section.
  - Metric 2: render-image PSNR    (final sRGB image, GetRenderResults)

Every test asserts Metal/Cpu actually ran (routed_backend matches AND no
"falling back" warning). The negative smoke test confirms the assertion
mechanism detects fallback. All tests are @pytest.mark.slow (shared-lib build:
``./scripts/build.sh -sj release``).

Concurrency contract: capi_runner.py mutates os.environ + uses a process-global
log callback. This suite MUST run serially (no pytest-xdist). Adding parallelism
requires subprocess isolation.

Thresholds: raw corr (block-mean ds) / render PSNR per scene calibrated from
baseline.md (258.6.2 measurement on 2026-06-10). See `_RAW_THRESHOLDS` below.
"""
import math
import platform
from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered

# Block-mean downsample metric (258.6.2) — single source of truth hoisted into
# a tracked module (see _parity_metrics.py header for why it no longer lives in
# the gitignored parity-harness scratchpad).
from test.e2e._parity_metrics import (
    _block_mean,
    _raw_corr_ds as _raw_corr_ds_impl,
    _DS_BH,
    _DS_BW,
)

CONFIGS_DIR = Path(__file__).parent / "configs"
_SEED = 42
# Secondary seed for the metal self-consistency cross-seed assertion (see
# `_assert_metal_self_consistency`). Different stream from _SEED is required;
# the exact value doesn't matter as long as it's not _SEED.
_SEED_B = 7
_TIMEOUT = 180

# --- Multi-MS test hardening (post Task 3 device-resident transit bugfix) --- #
# The transit-PCG stream-reuse bug (gp.gen_ray_base=0 across SimBatches) showed
# that the raw-corr-ds metric alone is corr-blind to orientation under-sampling:
# the bug produced metal_self ≈ 0.954 vs legacy_self ≈ 0.999 but corr_ds vs
# legacy still came out 0.9795 / 0.8930 — high enough that bumping thresholds
# masked it. These two assertions guard the cross-seed and global-energy axes:
#
#   - SELF_MARGIN (0.02): metal_self must be within legacy_self - 0.02. Under-
#     sampling collapses metal_self toward 0.95 while legacy stays at 0.999, so
#     a 0.02 budget catches it cleanly without flagging the natural
#     PCG-vs-mt19937 noise floor gap (~0.0001).
#   - ENERGY_TOL (0.05): |sum(metal_Y)/sum(legacy_Y) - 1| ≤ 0.05. Total energy
#     averages over all pixels so noise floors out; 5% catches gross global
#     imbalance bugs (e.g. the +16% multi-MS+filter bug that scrum-267 found
#     during fused-emit-gate) without flagging Monte-Carlo variance.
#
# Applied to multi-MS configs where device-resident continuation is exercised
# (ms_multi_crystal, ms_multi_crystal_filtered). Per-brightness-zone energy
# ratios are intentionally NOT added (selection bias on sparse filter scenes,
# cross-config inconsistent — owner decision 2026-06-14).
_SELF_MARGIN = 0.02
_ENERGY_TOL = 0.05

# Every test exercises the Metal backend (Apple-only) and asserts it routed
# without fallback, so skip on non-Darwin CI runners (the Ubuntu e2e-slow
# matrix leg) where metal falls back to legacy.
pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin", reason="Metal backend is only available on macOS"
)


# --------------------------------------------------------------------------- #
# Metrics
# --------------------------------------------------------------------------- #

def _raw_corr(a: BufferedSimResult, b: BufferedSimResult) -> float:
    """Full-resolution Pearson correlation of raw XYZ buffers.

    Retained as a diagnostic alongside the primary block-mean metric
    `_raw_corr_ds`. Suppressed by sparsity speckle floor on filter scenes —
    not used for assertions; see baseline.md F3.
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


def _raw_corr_ds(a: BufferedSimResult, b: BufferedSimResult) -> float:
    """Block-mean (4×4) downsampled Pearson correlation of raw XYZ buffers.

    Primary metric for parity assertions (258.6.2). Each output bin
    aggregates ``_DS_BH * _DS_BW`` rays so per-pixel Monte-Carlo speckle
    decays by ``sqrt(rays/bin)``; spatial structure of the bright
    population is preserved. Implementation imported from
    ``test/e2e/_parity_metrics.py`` (single source — never re-define).
    """
    return _raw_corr_ds_impl(a, b, _DS_BH, _DS_BW)


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

def _run(config_name: str, backend: str, seed: int = _SEED) -> BufferedSimResult:
    cfg = str(CONFIGS_DIR / f"{config_name}.json")
    return run_scene_capi_buffered(cfg, sim_seed=seed, backend=backend, timeout_sec=_TIMEOUT)


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

# Per-scene thresholds calibrated from baseline.md (258.6.2 re-measure,
# 2026-06-10). Primary metric is `_raw_corr_ds` (block-mean 4×4); thresholds
# below are `floor(ds_corr * 100) / 100 − 0.02` per plan §4.
# Render PSNR threshold = 13.0 dB uniform — single-seed baseline lacks σ for
# mean-3σ calibration; 13 dB acts as a catastrophic-regression detector given
# the architectural raw vs render divergence documented in baseline.md F2.
#
# Filter scenes fixed in 258.10 (BeginSession RNG幂等化): ds_corr lifted from
# 0.25–0.47 to 0.9963–0.9983 on both Metal and cpu_backend. xfail removed.
#
# scrum-267 task-device-resident-continuation bugfix (2026-06-14): when Task 3
# moved multi-MS Metal frame-transit from host to device, an earlier revision
# of this comment claimed the resulting parity drop (ms_multi_crystal 0.9795,
# ms_multi_crystal_filtered 0.8930) was the unavoidable PCG-vs-mt19937
# statistical gap — and the thresholds were widened to 0.95 / 0.87 accordingly.
# That diagnosis was WRONG. Owner re-inspected the rendered panels (scratchpad/
# debug-metal-continuation-correctness/findings.md) and traced it to a
# transit-PCG STREAM REUSE bug: BuildTransitRootParams set gp.gen_ray_base = 0
# on every SimBatch, so every batch's transit dispatch on (layer, ci) keyed PCG
# at (transit_seed, 0..ci_n-1), collapsing all batches' tid=k onto the same
# crystal orientation → severe orientation under-sampling across batches. The
# corr-only metric hid it (~15625 batches collapsing to ~ci_n discrete
# orientations still produced a smooth-ish image); a layer-decomposition
# diagnostic exposed Metal's 2-MS self-noise floor at 0.954 vs legacy 0.9999.
# Fix: add Impl::transit_ray_count_ (mirrors root_ray_count's monotone-across-
# SimBatches contract); TraceLayer fills gp.gen_ray_base from it before
# EncodeTransitRoot and advances after each successful cb. Post-fix measurement
# (visual_parity_check.py, seed 42 vs noise floor seed 7):
#   ms_multi_crystal:           0.9998 (noise floor 0.9999, gap −0.0001)
#   ms_multi_crystal_filtered:  0.9986 (noise floor 0.9988, gap −0.0002)
# Both within noise floor → metal is statistically indistinguishable from
# legacy. Thresholds restored to floor−0.02 of the post-fix measurement
# (0.97 / 0.97) — the prior 0.95 / 0.87 were bug-compensation, not legitimate
# statistical headroom.
_RAW_THRESHOLDS = {
    # config:                         (metal, cpu_backend)  ds_corr → floor−0.02
    "dual_fisheye_ref":             (0.95, 0.94),
    # ms_multi_crystal: post-bugfix measurement 0.9998 ≈ noise floor 0.9999.
    # Threshold = floor(0.9998 × 100) / 100 − 0.02 = 0.97. cpu_backend unchanged.
    "ms_multi_crystal":             (0.97, 0.81),
    "parity_ms_prob05":             (0.93, 0.90),
    # Filter scenes — single-crystal+filter (parity_ms_prob05_filter) unchanged
    # at 0.97. Multi-crystal+filter (ms_multi_crystal_filtered) post-bugfix
    # measurement 0.9986 ≈ noise floor 0.9988; threshold = 0.97.
    "ms_multi_crystal_filtered":    (0.97, 0.97),
    "parity_ms_prob05_filter":      (0.97, 0.97),
    # parity_single_ms_filter dropped: legacy returns all-zero buffer after
    # the prob=0.0→1.0 fix (commit 0d03388); metal/cpu_backend raise PY
    # exceptions on it. Test is `@pytest.mark.skip`-marked — no threshold.
}
_T_PSNR_DB = 13.0  # uniform render-PSNR floor; see baseline.md threshold section.


def _parity_axes(config_name: str) -> tuple[float, float, float, float]:
    """Return (corr_metal_vs_legacy, psnr_metal_vs_legacy,
                corr_cpu_vs_legacy,   psnr_cpu_vs_legacy).

    Corr metric is the 258.6.2 block-mean ds variant. Full-resolution
    `_raw_corr` is computed only for diagnostic prints.
    """
    _, metrics = _run_parity(config_name)
    return metrics


def _run_parity(
    config_name: str,
) -> tuple[
    tuple[BufferedSimResult, BufferedSimResult, BufferedSimResult],
    tuple[float, float, float, float],
]:
    """Run legacy/metal/cpu_backend on `config_name`, assert routing, and
    return both the raw BufferedSimResults and the (cm, pm, cc, pc) metric
    tuple. Used by tests that need the raw buffers for additional assertions
    (self-consistency, energy conservation) beyond the four parity axes.
    """
    legacy = _run(config_name, "legacy")
    metal = _run(config_name, "metal")
    cpu = _run(config_name, "cpu_backend")

    _assert_routed(legacy, "legacy", config_name)
    _assert_routed(metal, "metal", config_name)
    _assert_routed(cpu, "cpu_backend", config_name)

    metrics = (
        _raw_corr_ds(metal, legacy),
        _render_psnr(metal, legacy),
        _raw_corr_ds(cpu, legacy),
        _render_psnr(cpu, legacy),
    )
    return (legacy, metal, cpu), metrics


def _assert_metal_self_consistency(
    config_name: str,
    metal_s1: BufferedSimResult,
    legacy_s1: BufferedSimResult,
) -> None:
    """Cross-seed self-consistency: metal_self must be within legacy_self -
    SELF_MARGIN. Catches orientation under-sampling (e.g. transit-PCG stream
    reuse) that the legacy-vs-metal corr metric is blind to.

    Re-runs metal + legacy with `_SEED_B` and compares to the seed-A buffers
    passed in. Costs 2 extra sims per protected test.
    """
    metal_s2 = _run(config_name, "metal", seed=_SEED_B)
    legacy_s2 = _run(config_name, "legacy", seed=_SEED_B)
    _assert_routed(metal_s2, "metal", config_name)
    _assert_routed(legacy_s2, "legacy", config_name)
    metal_self = _raw_corr_ds(metal_s1, metal_s2)
    legacy_self = _raw_corr_ds(legacy_s1, legacy_s2)
    print(
        f"[self] {config_name}: metal_self={metal_self:.4f} "
        f"legacy_self={legacy_self:.4f} margin={_SELF_MARGIN}"
    )
    assert metal_self >= legacy_self - _SELF_MARGIN, (
        f"{config_name}: metal cross-seed self-consistency {metal_self:.4f} < "
        f"legacy_self {legacy_self:.4f} − {_SELF_MARGIN}. Suspect transit/gen "
        f"PCG stream collapse — metal orientations are less self-similar across "
        f"seeds than legacy, signalling under-sampling."
    )


def _assert_energy_conservation(
    config_name: str,
    metal: BufferedSimResult,
    legacy: BufferedSimResult,
) -> None:
    """|sum(metal_Y) / sum(legacy_Y) - 1| ≤ ENERGY_TOL. Global energy
    averages over all pixels so per-pixel Monte-Carlo speckle floors out;
    catches gross global imbalance (e.g. the +16% multi-MS+filter bug from
    fused-emit-gate scrum-267) that per-pixel corr can hide.
    """
    metal_Y = float(metal.flt_buf[..., 1].sum())
    legacy_Y = float(legacy.flt_buf[..., 1].sum())
    assert legacy_Y > 0.0, (
        f"{config_name}: legacy total Y == 0; cannot form energy ratio"
    )
    ratio = metal_Y / legacy_Y
    print(
        f"[energy] {config_name}: metal/legacy Y ratio={ratio:.4f} "
        f"tol=±{_ENERGY_TOL}"
    )
    assert abs(ratio - 1.0) <= _ENERGY_TOL, (
        f"{config_name}: metal/legacy total-Y ratio {ratio:.4f} outside "
        f"[1 ± {_ENERGY_TOL}]. Suspect global energy imbalance in continuation/"
        f"emit-gate path (cf. fused-emit-gate +16% regression class)."
    )


def _assert_parity(config_name: str, cm: float, pm: float, cc: float, pc: float) -> None:
    t_metal, t_cpu = _RAW_THRESHOLDS[config_name]
    assert cm >= t_metal, f"{config_name} Axis A ds_corr {cm:.4f} < {t_metal}"
    assert pm >= _T_PSNR_DB, f"{config_name} Axis A render PSNR {pm:.2f} dB < {_T_PSNR_DB}"
    if t_cpu is None:
        return  # cpu_backend skip-marked at suite level
    assert cc >= t_cpu, f"{config_name} Axis B ds_corr {cc:.4f} < {t_cpu}"
    assert pc >= _T_PSNR_DB, f"{config_name} Axis B render PSNR {pc:.2f} dB < {_T_PSNR_DB}"


# --- Single MS, no filter -------------------------------------------------- #

@pytest.mark.slow
@pytest.mark.heavy  # single-MS parity covered per-PR by device_gen metal; demote to local/nightly
def test_parity_single_ms_no_filter():
    cm, pm, cc, pc = _parity_axes("dual_fisheye_ref")
    print(f"[parity] dual_fisheye_ref: metal ds={cm:.4f} psnr={pm:.2f}dB | cpu_backend ds={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("dual_fisheye_ref", cm, pm, cc, pc)


# --- Single MS + filter ---------------------------------------------------- #

@pytest.mark.slow
@pytest.mark.skip(
    reason="parity_single_ms_filter became zero-signal after the 258.6 "
           "code-review prob=0.0→1.0 fix (commit 0d03388): single-MS + "
           "raypath filter [3,5] survival rate collapsed to near-zero rays "
           "(eff_px=1, snap=0.0 in legacy). Metal/cpu_backend raise "
           "PY_EXCEPTION (exit_code=2). Config-level issue, out of scope "
           "for 258.6.2. Skip until the scene is repaired or replaced.",
)
def test_parity_single_ms_filter():
    cm, pm, cc, pc = _parity_axes("parity_single_ms_filter")
    print(f"[parity] parity_single_ms_filter: metal ds={cm:.4f} psnr={pm:.2f}dB | cpu_backend ds={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("parity_single_ms_filter", cm, pm, cc, pc)


# --- Multi MS prob=0.8, no filter ----------------------------------------- #

@pytest.mark.slow
def test_parity_multi_ms_prob08():
    (legacy, metal, _cpu), (cm, pm, cc, pc) = _run_parity("ms_multi_crystal")
    print(f"[parity] ms_multi_crystal: metal ds={cm:.4f} psnr={pm:.2f}dB | cpu_backend ds={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("ms_multi_crystal", cm, pm, cc, pc)
    # Task 3 device-resident continuation hardening: cross-seed self-consistency
    # + total-Y energy conservation cover corr-blind under-sampling / energy bugs.
    _assert_energy_conservation("ms_multi_crystal", metal, legacy)
    _assert_metal_self_consistency("ms_multi_crystal", metal, legacy)


# --- Multi MS prob=0.8 + filter ------------------------------------------- #

@pytest.mark.slow
def test_parity_multi_ms_prob08_filter():
    (legacy, metal, _cpu), (cm, pm, cc, pc) = _run_parity("ms_multi_crystal_filtered")
    print(f"[parity] ms_multi_crystal_filtered: metal ds={cm:.4f} psnr={pm:.2f}dB | cpu_backend ds={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("ms_multi_crystal_filtered", cm, pm, cc, pc)
    # Task 3 device-resident continuation hardening (see _assert_* docstrings).
    _assert_energy_conservation("ms_multi_crystal_filtered", metal, legacy)
    _assert_metal_self_consistency("ms_multi_crystal_filtered", metal, legacy)


# --- Multi MS prob=0.5, no filter ----------------------------------------- #

@pytest.mark.slow
@pytest.mark.heavy  # prob-value variant of prob08 (same code path); demote to local/nightly
def test_parity_multi_ms_prob05():
    cm, pm, cc, pc = _parity_axes("parity_ms_prob05")
    print(f"[parity] parity_ms_prob05: metal ds={cm:.4f} psnr={pm:.2f}dB | cpu_backend ds={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("parity_ms_prob05", cm, pm, cc, pc)


# --- Multi MS prob=0.5 + filter ------------------------------------------- #

@pytest.mark.slow
@pytest.mark.heavy  # prob-value variant of prob08_filter (same code path); demote to local/nightly
def test_parity_multi_ms_prob05_filter():
    cm, pm, cc, pc = _parity_axes("parity_ms_prob05_filter")
    print(f"[parity] parity_ms_prob05_filter: metal ds={cm:.4f} psnr={pm:.2f}dB | cpu_backend ds={cc:.4f} psnr={pc:.2f}dB")
    _assert_parity("parity_ms_prob05_filter", cm, pm, cc, pc)
