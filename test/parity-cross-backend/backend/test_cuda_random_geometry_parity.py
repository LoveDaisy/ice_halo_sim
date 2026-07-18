"""CUDA random-geometry parity tests.

End-to-end cross-backend regression that specifically exercises the CUDA
device-gen path when ``PrismCrystalParam`` carries STOCHASTIC shape
distributions (``h_`` and every ``d_[i]`` gaussian). The two invariants
exercised here are, in ``src/core/backend/cuda_trace_backend.cu``:

  * ``Impl::rng_`` is seeded once per Impl lifetime (see the ``rng_seeded_``
    field and its BeginSession gate). Any per-BeginSession reseed collapses
    the per-batch ``MakeCrystal`` RNG stream to a constant prefix and freezes
    crystal-shape randomization end-to-end.
  * The device-gen geometry pool is force-rebuilt every BeginSession when
    ``Impl::pool_stochastic_`` is true (see the ``geom_pool_built_``
    invalidation just below the scene-change block in BeginSession). A
    persistent pool under a stochastic scene silently traces the SAME shape
    drawn on the first BeginSession forever.

The existing CUDA parity battery (``test_cuda_exit_seam_parity.py`` /
``_filter_parity.py`` / ``_multi_ms_parity.py`` / ``_projection_parity.py``)
only exercises deterministic geometry and would keep passing under either
regression; this file plugs that hole.

Pre-registered acceptance bar:
  R1 — block-mean (4x4 ds) Pearson corr(CUDA, legacy) >= 0.90 on the
       stochastic-geometry config. The threshold is looser than the
       deterministic parity battery (0.95) because random per-batch shapes
       increase the per-pixel noise floor; the point of the test is not to
       chase the deterministic threshold but to prove correlated structure
       is preserved end-to-end.
  R2 — |sum(cuda_Y) / sum(legacy_Y) - 1| <= 0.10. Larger than the
       deterministic battery's 0.05 for the same random-noise reason.
  R3 — cross-seed self-consistency of CUDA >= legacy_self - 0.05. Guards the
       gpu_parity_corr_masks_undersampling failure mode: parity vs legacy can
       stay in spec while cross-seed self-corr collapses (both sides drop by
       the same amount), which would indicate the random shapes ended up
       collapsed to a single sample per seed.

Requires:
  - ``LUMICE_CUDA_ENABLED=ON`` build with the CUDA toolchain.
  - NVIDIA device visible to the runtime.
  - Shared-lib build with the CUDA-enabled Release configuration (dev49
    docker recipe in ``dev49_parity.sh``).

All tests are ``@pytest.mark.slow``.
"""
from __future__ import annotations

import os
import platform

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e._parity_metrics import (
    _raw_corr_ds as _raw_corr_ds_impl,
    _DS_BH,
    _DS_BW,
)
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_SEED_A = 42
_SEED_B = 7
_TIMEOUT = 600

# Random-geometry thresholds: looser than the deterministic battery on purpose
# (see module docstring). The point is to detect a broken RNG plumbing / frozen
# shape pool, not to chase deterministic parity.
_T_RAW_CORR_DS = 0.90
_T_ENERGY_TOL = 0.10
_T_SELF_MARGIN = 0.05

_CUDA_AVAILABLE = (
    platform.system() in ("Linux", "Windows") and os.environ.get("LUMICE_HAS_CUDA") == "1"
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


def _run(config_name: str, backend: str, seed: int) -> BufferedSimResult:
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


CFG = "parity_random_geometry"


# --------------------------------------------------------------------------- #
# R1 + R2 — CUDA vs legacy on a stochastic-geometry scene.
# --------------------------------------------------------------------------- #
@pytest.mark.slow
def test_cuda_random_geometry_image_parity_vs_legacy():
    """Stochastic ``h_`` + ``d_[6]`` prism config: the CUDA device-gen path
    must rebuild the geometry pool every SimBatch (Layer 2 fix) and draw a
    fresh shape each cycle from an advancing ``rng_`` (Layer 1 fix).

    Failure modes this test catches:
      - Frozen-shape regression: if ``rng_`` were re-seeded per BeginSession
        every batch would draw the same shape prefix → CUDA image would
        deterministic-lock and diverge from legacy's true random average
        both structurally (corr drops) and in total energy.
      - Pool persistence regression: if the pool remained persistent under a
        stochastic scene, every batch would trace the SAME shape drawn on the
        first BeginSession → same as the frozen-shape symptom.
    """
    legacy = _run(CFG, "legacy", seed=_SEED_A)
    cuda = _run(CFG, "cuda", seed=_SEED_A)

    _assert_routed(legacy, "legacy", CFG)
    _assert_routed(cuda, "cuda", CFG)

    corr = _raw_corr_ds(cuda, legacy)
    cuda_Y = float(cuda.flt_buf[..., 1].sum())
    legacy_Y = float(legacy.flt_buf[..., 1].sum())
    assert legacy_Y > 0.0, f"{CFG}: legacy total Y == 0; cannot form energy ratio"
    energy_ratio = cuda_Y / legacy_Y

    print(
        f"[parity] {CFG}: cuda ds_corr={corr:.4f} "
        f"energy_ratio={energy_ratio:.4f} (tol +/-{_T_ENERGY_TOL})"
    )

    assert corr >= _T_RAW_CORR_DS, (
        f"{CFG}: ds_corr {corr:.4f} < {_T_RAW_CORR_DS}. Suspect CUDA geometry-pool "
        "rebuild gate broken (pool persisted across batches under a stochastic "
        "scene → single frozen shape) or rng_ seed-once gate broken (rng_ reset "
        "every BeginSession → every batch draws the same shape prefix)."
    )
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{CFG}: cuda/legacy total-Y ratio {energy_ratio:.4f} outside "
        f"[1 +/- {_T_ENERGY_TOL}]. A large deviation on random geometry usually "
        "means one side collapsed to a single shape sample (Y flattens or clusters)."
    )


# --------------------------------------------------------------------------- #
# R3 — cross-seed self-consistency on the stochastic config.
# --------------------------------------------------------------------------- #
@pytest.mark.slow
def test_cuda_random_geometry_cross_seed_self_consistency():
    """Guards ``gpu_parity_corr_masks_undersampling``-style bugs: parity vs
    legacy could stay in spec while both sides silently collapse to a single
    shape sample (yielding identical images between the seeds regardless of
    what the RNG intended). A working randomization pipeline puts CUDA's
    cross-seed corr close to legacy's cross-seed corr on the same config.
    """
    cuda_a = _run(CFG, "cuda", seed=_SEED_A)
    cuda_b = _run(CFG, "cuda", seed=_SEED_B)
    legacy_a = _run(CFG, "legacy", seed=_SEED_A)
    legacy_b = _run(CFG, "legacy", seed=_SEED_B)

    _assert_routed(cuda_a, "cuda", CFG)
    _assert_routed(cuda_b, "cuda", CFG)
    _assert_routed(legacy_a, "legacy", CFG)
    _assert_routed(legacy_b, "legacy", CFG)

    corr_cuda = _raw_corr_ds(cuda_a, cuda_b)
    corr_legacy = _raw_corr_ds(legacy_a, legacy_b)
    print(
        f"[self] {CFG}: cuda_self={corr_cuda:.4f} legacy_self={corr_legacy:.4f} "
        f"margin={_T_SELF_MARGIN}"
    )
    # Two-sided: CUDA's cross-seed self-consistency must be CLOSE to legacy's,
    # not merely "no lower". The frozen-shape collapse this test names manifests
    # as corr_cuda ABNORMALLY HIGH (both seeds redraw the identical shape →
    # near-identical images → corr → 1.0), which a one-sided lower bound would
    # silently pass. The upper bound is the guard that actually catches the
    # collapse; the white-box StochasticScene_ShapesDifferAcrossBatches test is
    # the deterministic sibling defense for the same failure mode.
    assert corr_cuda <= corr_legacy + _T_SELF_MARGIN, (
        f"{CFG}: cuda cross-seed self-consistency {corr_cuda:.4f} > "
        f"legacy_self {corr_legacy:.4f} + {_T_SELF_MARGIN}. Suspect frozen-shape "
        "regression: CUDA is far MORE self-similar across seeds than legacy, "
        "i.e. the shape pool collapsed to a single draw regardless of seed."
    )
    assert corr_cuda >= corr_legacy - _T_SELF_MARGIN, (
        f"{CFG}: cuda cross-seed self-consistency {corr_cuda:.4f} < "
        f"legacy_self {corr_legacy:.4f} - {_T_SELF_MARGIN}. Suspect undersampling "
        "or a decorrelation regression (CUDA less self-consistent than legacy)."
    )
