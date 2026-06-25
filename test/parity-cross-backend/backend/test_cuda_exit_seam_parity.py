"""CUDA exit-seam parity tests (scrum-cuda-backend-mvp 295.4).

Backend-equivalence oracle for the CUDA single-MS megakernel. Mirrors
``test_metal_exit_seam_parity.py``'s G1+G2 axes (block-mean ds_corr + total-Y
energy conservation) and adds G3 cross-seed self-consistency + G6 env-revert
sanity gate. G4 (golden-analytic) is verified separately via ``ctest -L
"unit-correctness|golden-analytic"`` in the dev49 CUDA build (kicked off by
the same harness; not part of this file).

Pre-registered acceptance bar (plan §0.5):
  G1 — block-mean (4×4 ds) Pearson corr vs legacy ≥ 0.95
  G2 — |sum(cuda_Y) / sum(legacy_Y) − 1| ≤ 0.05
  G3 — corr_self_cuda(seed=42, seed=7) ≥ corr_self_legacy(seed=42, seed=7) − 0.02
  G6 — relinquishing LUMICE_TRACE_BACKEND env reverts to legacy (asserted via
       the `routed_backend` field after a no-env run inside the same suite).

Requires:
  - ``LUMICE_CUDA_ENABLED=ON`` build with the CUDA toolchain.
  - NVIDIA device visible to the runtime (compute-sanitizer clean per M2).
  - Shared-lib build: ``./scripts/build.sh -sj release`` is not sufficient
    on its own — the CUDA path needs the CUDA-enabled Release build. The
    dev49 docker recipe in scrum-cuda-backend-mvp/.../plan.md §6 covers it.

All tests are @pytest.mark.slow (long-running parity loop + shared-lib).
"""
from __future__ import annotations

import math
import os
import platform
from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e._parity_metrics import (
    _block_mean,
    _raw_corr_ds as _raw_corr_ds_impl,
    _DS_BH,
    _DS_BW,
)
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_SEED = 42
_SEED_B = 7
_TIMEOUT = 300  # CUDA dev49 docker first-launch JIT can take a minute

# Per plan §0.5: G1=0.95, G2=0.05, G3 margin=0.02. Fixed pre-registration.
_T_RAW_CORR_DS = 0.95
_T_ENERGY_TOL = 0.05
_T_SELF_MARGIN = 0.02

# CUDA backend is only available when LUMICE_CUDA_ENABLED=ON at build time.
# Mac / Windows hosts and Linux builds without the gate fall back to legacy;
# routed_backend would come out "legacy" and the parity assertions would be
# false positives (legacy-vs-legacy). Two cheap probes:
#   1. Platform != Linux: dev49 toolchain is Linux-only — skip non-Linux to
#      keep the harness honest. (If a future macOS CUDA path lands, lift this.)
#   2. Env knob LUMICE_HAS_CUDA != "1": gate the slow runs behind an explicit
#      opt-in so default `pytest -v -m slow` on a CPU-only Linux box does not
#      light up these tests as failures. Set LUMICE_HAS_CUDA=1 in the dev49
#      docker run command (plan §6).
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


# --------------------------------------------------------------------------- #
# Metrics — block-mean ds corr + render PSNR (single-source impl from
# _parity_metrics.py, same as Metal harness).
# --------------------------------------------------------------------------- #

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
# G1 + G2 + G6 — single-MS no-filter parity
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_single_ms_no_filter_parity():
    """G1 (corr ≥ 0.95), G2 (energy ≤ 5%), G6 (env-revert) on dual_fisheye_ref.

    G6 is asserted by routing legacy with no env and confirming
    routed_backend == "legacy"; the run_scene_capi_buffered("legacy", ...)
    helper actively strips LUMICE_TRACE_BACKEND from os.environ to test that
    code path.
    """
    cfg = "dual_fisheye_ref"
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

    # G1
    assert corr >= _T_RAW_CORR_DS, (
        f"{cfg}: ds_corr {corr:.4f} < {_T_RAW_CORR_DS} (G1 floor). "
        "Suspect frame invariant 6 (rot_c2w missing), from_face guard bug, "
        "or Möller-Trumbore precision issue."
    )
    # G2
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{cfg}: cuda/legacy total-Y ratio {energy_ratio:.4f} outside "
        f"[1 ± {_T_ENERGY_TOL}]. Suspect Fresnel sampling drift / exit-cap "
        "overflow / TIR mishandling."
    )


# --------------------------------------------------------------------------- #
# G3 — cross-seed self-consistency (catches orientation under-sampling)
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_cross_seed_self_consistency():
    """G3: corr_self_cuda ≥ corr_self_legacy − margin.

    Two runs each (seed _SEED and _SEED_B) per backend. Single test function
    (not parametrize) because corr_self is a *cross-seed* relation — each
    parametrize case can only hold its own buffer.
    """
    cfg = "dual_fisheye_ref"
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
        "collapse / per-thread stream reuse in trace_single_ms_kernel."
    )
