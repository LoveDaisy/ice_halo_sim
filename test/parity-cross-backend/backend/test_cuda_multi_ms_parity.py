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
# Exit-buffer overflow regression (scrum-296 Step D / task-cuda-throughput-bench)
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_cuda_high_continuation_no_exit_overflow_corruption():
    """Guard: a HIGH-continuation config must not corrupt the image via the
    CUDA exit-buffer cap.

    ``ms_prob05`` (the parity config above) has low continuation and never fills
    the exit pool, so it could not catch this: ``ComputeExitCap`` formerly capped
    the per-layer exit buffer at 64 MiB (~762600 records). ``ms_multi_crystal``
    emits ~954k layer-1 exits at the DEFAULT dispatch — over the old cap — and the
    kernel silently dropped the tail. Total energy (~1.02) AND cross-seed
    self-corr (~0.9998) both stayed clean and masked it; only corr-vs-legacy
    exposed the spatial damage (ds_corr fell to 0.913 < the 0.95 floor). The fix
    sizes the exit/cont buffers to the analytic upper bound `n*(max_hits*2+4)` so
    no exit is ever dropped. This test pins ds_corr back at/above the floor on
    exactly the config that overflowed.
    """
    cfg = "ms_multi_crystal"
    legacy = _run(cfg, "legacy", seed=_SEED)
    cuda = _run(cfg, "cuda", seed=_SEED)

    _assert_routed(legacy, "legacy", cfg)
    _assert_routed(cuda, "cuda", cfg)

    corr = _raw_corr_ds(cuda, legacy)
    cuda_Y = float(cuda.flt_buf[..., 1].sum())
    legacy_Y = float(legacy.flt_buf[..., 1].sum())
    assert legacy_Y > 0.0, f"{cfg}: legacy total Y == 0; cannot form energy ratio"
    energy_ratio = cuda_Y / legacy_Y

    print(
        f"[parity] {cfg}: cuda ds_corr={corr:.4f} energy_ratio={energy_ratio:.4f} "
        f"(floors corr≥{_T_RAW_CORR_DS}, |energy−1|≤{_T_ENERGY_TOL})"
    )

    # ds_corr is the discriminating metric: the dropped-tail damage is spatial,
    # not energetic, so this is what regressed (0.913) under the old cap.
    assert corr >= _T_RAW_CORR_DS, (
        f"{cfg}: ds_corr {corr:.4f} < {_T_RAW_CORR_DS}. Exit/cont buffer cap "
        "may have reintroduced silent exit-record dropping (ComputeExitCap / "
        "ComputeContCap must size to the analytic upper bound, not a fixed cap)."
    )
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{cfg}: cuda/legacy total-Y ratio {energy_ratio:.4f} outside "
        f"[1 ± {_T_ENERGY_TOL}]."
    )


@pytest.mark.slow
def test_cuda_three_layer_multi_ci_parity_vs_legacy():
    """Full multi-CI guard: a 3-MS config with multiple crystals on EVERY layer
    (incl. the final layer) must match legacy.

    ``ms3_multi_crystal`` has 3 scattering layers with per-layer crystal counts
    [4, 3, 2]. It exercises the parts that 2-layer single-CI-final configs cannot:
      - per-ci transit on CONTINUATION layers (layers 1,2 multi-CI),
      - EnsureContCapacity growth across 3 layers (cont fan-out > layer-0 cap),
      - multi-CI FINAL-layer DrainExits (crystal_id-indexed FilterSpec / GetFn).
    A single-CI-only CUDA backend traces every ray through one crystal → ds_corr
    collapses; this pins the full multi-CI path against legacy.
    """
    import json
    import tempfile
    cfg = "ms3_multi_crystal"
    # The committed ms3_multi_crystal ships ray_num=5M (an e2e PSNR config); the
    # legacy oracle runs single-worker under a fixed seed and times out at 5M ×
    # 3-MS × max_hits=12. Reduce to 300K via a temp config — block-mean ds_corr is
    # robust to the lower sample count, and the multi-CI structure (3 layers, CI
    # [4,3,2]) is unchanged. Both backends run the SAME temp config.
    src = json.loads((CONFIGS_DIR / f"{cfg}.json").read_text())
    src["scene"]["ray_num"] = 300_000
    tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    json.dump(src, tmp)
    tmp.flush()
    tmp.close()

    legacy = run_scene_capi_buffered(tmp.name, sim_seed=_SEED, backend="legacy", timeout_sec=_TIMEOUT)
    cuda = run_scene_capi_buffered(tmp.name, sim_seed=_SEED, backend="cuda", timeout_sec=_TIMEOUT)

    _assert_routed(legacy, "legacy", cfg)
    _assert_routed(cuda, "cuda", cfg)

    corr = _raw_corr_ds(cuda, legacy)
    cuda_Y = float(cuda.flt_buf[..., 1].sum())
    legacy_Y = float(legacy.flt_buf[..., 1].sum())
    assert legacy_Y > 0.0, f"{cfg}: legacy total Y == 0; cannot form energy ratio"
    energy_ratio = cuda_Y / legacy_Y

    print(
        f"[parity] {cfg}(300K): cuda ds_corr={corr:.4f} energy_ratio={energy_ratio:.4f} "
        f"(floors corr≥{_T_RAW_CORR_DS}, |energy−1|≤{_T_ENERGY_TOL})"
    )
    assert corr >= _T_RAW_CORR_DS, (
        f"{cfg}: ds_corr {corr:.4f} < {_T_RAW_CORR_DS}. Multi-CI path broken on a "
        "continuation/final layer (per-ci transit / EnsureContCapacity / final-"
        "layer crystal_id-indexed DrainExits)."
    )
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{cfg}: cuda/legacy total-Y ratio {energy_ratio:.4f} outside [1 ± {_T_ENERGY_TOL}]."
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
