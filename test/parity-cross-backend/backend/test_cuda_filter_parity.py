"""CUDA filter parity tests (scrum-cuda-backend-complete 296.5).

End-to-end image parity for the CUDA device-resident filter gate
(``DeviceFilterCheck`` inside ``trace_single_ms_kernel`` on ms_mode==1, plus the
``DrainExits`` final-layer host ``FilterSpec::Check`` + prob draw). Structure
mirrors ``test_cuda_multi_ms_parity.py``: corr / energy / cross-seed under the
same parity-metric-masks-bugs battery (issue.md AC1/AC2a/AC2b/AC2c).

Pre-registered acceptance bar:
  AC1   — Design A termination: filter-fail rays do not reach the cumulative
          image. ``ms_filter_leak_impossible.json`` produces zero snapshot
          intensity on CUDA, same contract as ``test_ms_filter_leak.py`` on
          legacy.
  AC2a  — block-mean (4×4 ds) Pearson corr vs legacy ≥ 0.95 for both
          single-MS-with-filter and multi-MS-with-filter configs.
  AC2c  — |sum(cuda_Y) / sum(legacy_Y) − 1| ≤ 0.05 on the same configs.
  AC2b  — cross-seed self-corr of CUDA stays within 0.02 of legacy
          (continuation undersampling is the standing risk; see
          [[feedback_gpu_parity_corr_masks_undersampling]]).

Requires:
  - ``LUMICE_CUDA_ENABLED=ON`` build with the CUDA toolchain.
  - NVIDIA device visible to the runtime.
  - Shared-lib build with the CUDA-enabled Release configuration (dev49
    docker recipe in ``dev49_parity.sh``).

All tests are @pytest.mark.slow.
"""
from __future__ import annotations

import math
import os
import platform

import numpy as np
import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e._parity_metrics import (
    _raw_corr_ds as _raw_corr_ds_impl,
    _DS_BH,
    _DS_BW,
)
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_SEED = 42
_SEED_B = 7
_TIMEOUT = 600

# Pre-registered thresholds (issue.md AC2a/b/c).
_T_RAW_CORR_DS = 0.95
_T_ENERGY_TOL = 0.05
_T_SELF_MARGIN = 0.02

_CUDA_AVAILABLE = (
    platform.system() == "Linux" and os.environ.get("LUMICE_HAS_CUDA") == "1"
)

pytestmark = pytest.mark.skipif(
    not _CUDA_AVAILABLE,
    reason=(
        "CUDA backend requires Linux + LUMICE_HAS_CUDA=1 + LUMICE_CUDA_ENABLED=ON "
        "build with NVIDIA device (dev49 docker). Skipping on this host."
    ),
)


def _raw_corr_ds(a: BufferedSimResult, b: BufferedSimResult) -> float:
    return _raw_corr_ds_impl(a, b, _DS_BH, _DS_BW)


def _render_psnr(a: BufferedSimResult, b: BufferedSimResult) -> float:
    if a.rgb_buf.shape != b.rgb_buf.shape:
        raise ValueError(f"render shape mismatch: {a.rgb_buf.shape} vs {b.rgb_buf.shape}")
    diff = a.rgb_buf.astype(np.float64) - b.rgb_buf.astype(np.float64)
    mse = float(np.mean(diff * diff))
    if mse == 0.0:
        return float("inf")
    return 10.0 * math.log10((255.0 * 255.0) / mse)


def _run(config_name: str, backend: str, seed: int = _SEED) -> BufferedSimResult:
    cfg = str(CONFIGS_DIR / f"{config_name}.json")
    return run_scene_capi_buffered(cfg, sim_seed=seed, backend=backend, timeout_sec=_TIMEOUT)


def _assert_routed(r: BufferedSimResult, expected: str, config_name: str) -> None:
    assert r.routed_backend == expected, (
        f"{config_name}/{expected}: routed={r.routed_backend!r} (expected {expected!r}); "
        f"see log_lines for the actual path."
    )
    assert not r.fell_back, (
        f"{config_name}/{expected}: fell back to legacy. Backend was REQUESTED but did not run."
    )


# --------------------------------------------------------------------------- #
# AC1 — Design A termination: filter-fail rays do not propagate.
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_impossible_filter_produces_zero_intensity():
    """AC1: ``ms_filter_leak_impossible.json`` on CUDA → snapshot_intensity == 0.

    Layer 1 uses raypath filter [1,1] (geometrically impossible on a convex
    hexagonal prism — face 1 cannot be hit twice consecutively). With the
    CUDA emit gate honouring the filter, no ray survives layer 1 → no exit
    appears in the cumulative image.

    Failure mode = filter gate logic bug: if ``DeviceFilterCheck`` returns true
    on the impossible path, prob-pass continuations transit to layer 2 + emit
    legitimate exits → snapshot_intensity > 0. The legacy regression sentinel
    ``test_ms_filter_leak.py::test_impossible_filter_produces_zero_intensity``
    already locks the legacy side of this contract.
    """
    cfg_path = str(CONFIGS_DIR / "ms_filter_leak_impossible.json")
    # run_scene_capi has no backend routing (it would silently run legacy and set
    # no LUMICE_TRACE_BACKEND); use the buffered runner, which routes to cuda via
    # env and still exposes snapshot_intensity. (296.5 test wrote the wrong runner.)
    result = run_scene_capi_buffered(cfg_path, sim_seed=_SEED, backend="cuda", timeout_sec=_TIMEOUT)
    assert result.routed_backend == "cuda", (
        f"impossible filter test: routed={result.routed_backend!r} (expected 'cuda'); "
        f"environment may have forced fallback"
    )
    assert not result.fell_back, "impossible filter test: CUDA backend fell back to legacy"
    assert result.snapshot_intensity == pytest.approx(0.0, abs=1e-6), (
        f"CUDA filter gate did NOT terminate filter-fail rays — Design A violation: "
        f"snapshot_intensity={result.snapshot_intensity:.6f} (expected ~0). "
        "Suspect kernel emit gate skipping DeviceFilterCheck, or the dummy desc "
        "fallback being used when a real filter desc was needed."
    )


# --------------------------------------------------------------------------- #
# AC2a + AC2c — single-MS filter parity (final-layer host filter path).
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_single_ms_filter_image_parity_vs_legacy():
    """Single-MS raypath filter: every exit lands on the final layer ⇒ exclusively
    exercises ``DrainExits``'s ``FilterSpec::Check`` + GetFn remap + prob path.

    Suspects on failure:
      - GetFn remap missing/wrong → corr drops while energy ratio stays at 1.
      - FilterSpec::Create on per-DrainExits call with stale Crystal/axis_dist →
        sporadic drops, energy ratio low but not zero.
      - drain_rng_ seed wrong → cross-seed independence fails (caught by AC2b).
    """
    cfg = "parity_single_ms_filter"
    legacy = _run(cfg, "legacy", seed=_SEED)
    cuda = _run(cfg, "cuda", seed=_SEED)

    _assert_routed(legacy, "legacy", cfg)
    _assert_routed(cuda, "cuda", cfg)

    corr = _raw_corr_ds(cuda, legacy)
    psnr = _render_psnr(cuda, legacy)
    cuda_Y = float(cuda.flt_buf[..., 1].sum())
    legacy_Y = float(legacy.flt_buf[..., 1].sum())
    assert legacy_Y > 0.0, f"{cfg}: legacy total Y == 0; cannot form energy ratio"
    energy_ratio = cuda_Y / legacy_Y

    print(
        f"[parity] {cfg}: cuda ds_corr={corr:.4f} psnr={psnr:.2f}dB "
        f"energy_ratio={energy_ratio:.4f} (tol ±{_T_ENERGY_TOL})"
    )

    assert corr >= _T_RAW_CORR_DS, (
        f"{cfg}: ds_corr {corr:.4f} < {_T_RAW_CORR_DS}. Suspect DrainExits "
        "GetFn remap or FilterSpec::Check argument shape — the CPU unit test "
        "test_device_filter_check_host.cpp already validated the device matcher."
    )
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{cfg}: cuda/legacy total-Y ratio {energy_ratio:.4f} outside "
        f"[1 ± {_T_ENERGY_TOL}]. Suspect final_ms_prob_ wired wrong or drain_rng_ "
        "seeded with the device-gate counter (would oversubtract)."
    )


# --------------------------------------------------------------------------- #
# AC2a + AC2c — multi-MS filter parity (device emit gate + drain).
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_multi_ms_filter_image_parity_vs_legacy():
    """2-MS prism with raypath filter on each layer: covers (i) device-side
    ``DeviceFilterCheck`` inside the kernel emit gate, (ii) Design A
    termination + mid-exit emission, (iii) final-layer host ``FilterSpec``.

    Suspects on failure:
      - DeviceFilterDesc per-slot indexing wrong (gate_slot off by max_ci) →
        wrong filter applied at each layer → corr tanks.
      - getfn_offsets prefix-sum wrong → ApplyGetFn_dev reads neighbour
        stripes → predicate scrambles → both corr + energy fail.
      - mid-exit ms_layer_idx tag wrong → DrainExits routes a mid-exit through
        host filter and drops it → energy drops while corr stays plausible.
    """
    cfg = "parity_ms_prob05_filter"
    legacy = _run(cfg, "legacy", seed=_SEED)
    cuda = _run(cfg, "cuda", seed=_SEED)

    _assert_routed(legacy, "legacy", cfg)
    _assert_routed(cuda, "cuda", cfg)

    corr = _raw_corr_ds(cuda, legacy)
    psnr = _render_psnr(cuda, legacy)
    cuda_Y = float(cuda.flt_buf[..., 1].sum())
    legacy_Y = float(legacy.flt_buf[..., 1].sum())
    assert legacy_Y > 0.0, f"{cfg}: legacy total Y == 0; cannot form energy ratio"
    energy_ratio = cuda_Y / legacy_Y

    print(
        f"[parity] {cfg}: cuda ds_corr={corr:.4f} psnr={psnr:.2f}dB "
        f"energy_ratio={energy_ratio:.4f} (tol ±{_T_ENERGY_TOL})"
    )

    assert corr >= _T_RAW_CORR_DS, (
        f"{cfg}: ds_corr {corr:.4f} < {_T_RAW_CORR_DS}. Cross-check filter "
        "desc upload (EnsureFilterBuffers per-slot index) + path_rec content "
        "at the emit gate (Metal+CPU mid-layer exits write poly-index path "
        "verbatim — CUDA must mirror)."
    )
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{cfg}: cuda/legacy total-Y ratio {energy_ratio:.4f} outside "
        f"[1 ± {_T_ENERGY_TOL}]. Audit mid-exit routing (filter-fail must drop, "
        "filter-pass + prob-fail must mid-exit) and DrainExits' branch on "
        "final_ms_layer_idx_."
    )


# --------------------------------------------------------------------------- #
# AC2b — multi-MS filter cross-seed self-consistency.
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_multi_ms_filter_cross_seed_self_consistency():
    """The hidden third rail: filter parity vs legacy can stay >= 0.95 while
    cross-seed self-corr collapses (under-sampling caused by continuation
    overflow or filter-fail Design A termination shrinking the surviving
    population). scrum-267.3's lesson — corr-vs-legacy masks the bug.
    """
    cfg = "parity_ms_prob05_filter"
    cuda_a = _run(cfg, "cuda", seed=_SEED)
    cuda_b = _run(cfg, "cuda", seed=_SEED_B)
    legacy_a = _run(cfg, "legacy", seed=_SEED)
    legacy_b = _run(cfg, "legacy", seed=_SEED_B)

    _assert_routed(cuda_a, "cuda", cfg)
    _assert_routed(cuda_b, "cuda", cfg)
    _assert_routed(legacy_a, "legacy", cfg)
    _assert_routed(legacy_b, "legacy", cfg)

    corr_cuda = _raw_corr_ds(cuda_a, cuda_b)
    corr_legacy = _raw_corr_ds(legacy_a, legacy_b)
    print(
        f"[self] {cfg}: cuda_self={corr_cuda:.4f} legacy_self={corr_legacy:.4f} "
        f"margin={_T_SELF_MARGIN}"
    )
    assert corr_cuda >= corr_legacy - _T_SELF_MARGIN, (
        f"{cfg}: cuda cross-seed self-consistency {corr_cuda:.4f} < "
        f"legacy_self {corr_legacy:.4f} − {_T_SELF_MARGIN}. Suspect drain_rng_ "
        "or transit_ray_count_ collapse under the filter-drop survival rate; "
        "verify (transit_seed, transit_ray_count + tid) tuple stays disjoint "
        "across seed-{42, 7} runs."
    )
