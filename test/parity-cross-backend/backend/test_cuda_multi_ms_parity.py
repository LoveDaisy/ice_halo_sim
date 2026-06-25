"""CUDA multi-MS parity tests (scrum-cuda-backend-complete 296.4).

End-to-end image parity for the CUDA device-resident continuation engine
(``transit_multi_ms_kernel`` + ms_mode-aware emit gate in
``trace_single_ms_kernel``). Mirrors ``test_cuda_exit_seam_parity.py``'s G1+G2+G3
oracle (block-mean ds corr / energy conservation / cross-seed self-consistency)
but exercises a ≥2 MS-layer config so the device transit path is on the hot
loop.

Pre-registered acceptance bar (issue.md AC2a/b/c):
  AC2a — block-mean (4×4 ds) Pearson corr vs legacy ≥ 0.95
  AC2c — |sum(cuda_Y) / sum(legacy_Y) − 1| ≤ 0.05
  AC2b — corr_self_cuda(seed=42, seed=7) ≥ corr_self_legacy − 0.02
          (multi-MS is the most dangerous case for cross-seed: scrum-267.3
          showed that under-sampled continuation streams collapse this margin
          while corr-vs-legacy stays in spec — see
          [[feedback_gpu_parity_corr_masks_undersampling]])

The config (``ms_prob05.json``) has 2 MS layers and a single CrystalParam,
matching the MVP single-CI assumption (multi-CI per-layer crystal pool is
296.6). The first layer uses prob=0.5 so roughly half of the emitted
continuations transit to layer 2; the second layer is the final exit (prob=0.0,
ignored). Cumulative XYZ image equals legacy ± the parity thresholds.

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
_TIMEOUT = 600  # multi-MS takes ~2x single-MS; dev49 first-launch JIT included

# Pre-registered thresholds (issue.md AC2a/b/c).
_T_RAW_CORR_DS = 0.95
_T_ENERGY_TOL = 0.05
_T_SELF_MARGIN = 0.02

# Same gate as test_cuda_exit_seam_parity.py — keep the two harnesses in sync.
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
# AC2a + AC2c — multi-MS image parity vs legacy (corr + energy conservation)
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_multi_ms_image_parity_vs_legacy():
    """AC2a (corr ≥ 0.95) + AC2c (|energy − 1| ≤ 5%) on a 2-MS prism config.

    ``ms_prob05`` has 2 scattering layers (prob 0.5 / 0.0, single crystal):
    layer 0 traces all root rays in ms_mode==1 → the emit gate routes
    survivors to the device continuation ring while prob-fail rays go to the
    layer-0 exit pool; ``Recombine`` dispatches ``transit_multi_ms_kernel`` to
    sample new orientations + entry points; layer 1 traces the continuation
    in ms_mode==0 → all exits land in the layer-1 exit pool. Cumulative XYZ
    image equals legacy ± thresholds.

    Diagnostic suspects on failure (priority order):
      - frame invariant 6 broken in transit kernel (cont_d_in→crystal-local rot
        wrong) → halo scatters into a band, corr tanks first.
      - PCG stream collapse in transit (transit_seed / transit_ray_count_
        reuse) → ds_corr stays plausible but AC2b drops; check that first.
      - emit-gate prob filter wrong sign → energy ratio drifts off ±5%.
      - mid-exit (filter_pass + prob_fail) writing to wrong buffer → energy
        ratio drops by exactly the prob_fail fraction (≈50% for prob=0.5).
    """
    cfg = "ms_prob05"
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

    # AC2a
    assert corr >= _T_RAW_CORR_DS, (
        f"{cfg}: ds_corr {corr:.4f} < {_T_RAW_CORR_DS} (AC2a floor). "
        "Suspect frame invariant 6 in transit_multi_ms_kernel (crystal-local "
        "rotation wrong) or ms_mode==1 emit gate mid-exit write path."
    )
    # AC2c
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{cfg}: cuda/legacy total-Y ratio {energy_ratio:.4f} outside "
        f"[1 ± {_T_ENERGY_TOL}]. Suspect prob_fail dropped (should be mid-exit), "
        "cont buffer overflow (warn log), or transit kernel zeroing rays via "
        "tri_to_poly kInvalidId fallback."
    )


# --------------------------------------------------------------------------- #
# AC2b — cross-seed self-consistency (the under-sampling guard)
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_multi_ms_cross_seed_self_consistency():
    """AC2b: cuda cross-seed self-corr ≥ legacy cross-seed self-corr − margin.

    This is the parity-metric-masks-bugs gate for the continuation engine.
    scrum-267.3 demonstrated that under-sampled multi-MS transit streams can
    keep AC2a (vs-legacy corr) within spec while collapsing the cross-seed
    self-corr — the cross-seed test catches what corr-vs-legacy hides. The
    margin (-0.02) tracks observed jitter in the legacy oracle so a clean
    CUDA backend reproduces or beats the legacy noise floor.
    """
    cfg = "ms_prob05"
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
        f"legacy_self {corr_legacy:.4f} − {_T_SELF_MARGIN}. Suspect PCG seed "
        "collapse in transit_multi_ms_kernel (transit_seed_ / transit_ray_count_ "
        "not advancing per dispatch) — this is the scrum-267.3 failure mode."
    )
