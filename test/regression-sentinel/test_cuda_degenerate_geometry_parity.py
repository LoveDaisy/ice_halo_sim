"""Regression guard: CUDA must stay isomorphic with CPU/Metal on degenerate
K-shape pool geometry (no backend fallback, no per-ray log storm, no frame loss).

Root cause (the defect this guards against):
  With the per-ray K-shape geometry pool active (SceneConfig geom_clock > 0) and a
  face_distance distribution wide enough to occasionally produce a zero-triangle
  Crystal (a ring0-legal "contributes nothing" product of the closed-form validity
  gate), the CUDA BuildGeomPool used to throw BackendUnavailableError on the FIRST
  degenerate slot of a ~512-shape pool. That aborted the whole GPU backend for the
  rest of the Run() (the CPU-fallback path), and the legacy CPU InitRayFirstMs then
  emitted one unthrottled PolygonFaceOfTri WARNING per ray of every degenerate ci —
  the observed log storm. Metal's UploadCrystalPool already tolerated degenerate
  slots, so CUDA was the sole backend violating the contract. Additionally, all four
  root-gen/transit kernels (both GPU backends) shared a latent neighbor-aliasing read:
  a tri_cnt==0 pool slot's tri_off points at the NEXT slot's triangles (or the pool
  tail), so sampling it silently pulled a neighbor shape's geometry.

Fix: BuildGeomPool records a zero-length pool slot and continues instead of throwing;
all four kernels route a tri_cnt==0 pick into the existing kInvalidId zero-weight drop.

Reproduction config (test/e2e/configs/repro_cuda_geom_pool_degenerate.json): prism with
6 gauss(1.0, 0.50) face_distance draws + geom_clock=64. At std=0.50 the closed-form
gate rejects ~0.8% of shapes; with 512 shapes/batch that is ~98% of batches carrying
at least one degenerate slot, so the pre-fix throw fired on essentially every batch.
Baseline (pre-fix) on dev49: 1 fallback + ~3100 PolygonFaceOfTri warnings, crystals
count jumps from 6250 (GPU K-pool) to 12500 (CPU legacy) as the drop settles. Fixed:
0 fallback, 0 storm, all 400000 rays complete on the GPU K-pool.

Cross-backend energy differs ~7-8% here (CUDA K-pool and CPU per-ray draw statistically
distinct crystal populations from the same wide distribution — sampling variance, not a
parity bug), so this file does NOT assert a tight cross-backend energy bound; the milder
random-geometry parity contract is covered by CudaRandomGeometryParity and the parity
battery. Here we assert the isomorphism invariants that were actually broken (no
fallback / no storm / real output) plus CUDA cross-seed self-consistency (the halo is
seed-stable, proving the output is genuine geometry rather than garbage neighbor reads).

@pytest.mark.slow: needs the CUDA-enabled shared-lib build (LUMICE_LIB) + an NVIDIA
device. CUDA-gated, so it is inert on non-CUDA hosts (CI, macOS).
"""
from __future__ import annotations

import math
import os
import platform

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e._parity_metrics import _raw_corr_ds as _raw_corr_ds_impl, _DS_BH, _DS_BW
from test.e2e.runner import get_project_root

_CONFIG_PATH = str(
    get_project_root() / "test" / "e2e" / "configs" / "repro_cuda_geom_pool_degenerate.json"
)
_TIMEOUT = 600
_SEEDS = (42, 7, 123)

# CUDA cross-seed block-mean corr floor. The halo is seed-stable, so two CUDA
# runs at different sim_seed must correlate strongly even on this noisy geometry.
# Measured on dev49 (RTX 4060Ti): ~0.98; floor set with margin below that.
_T_SELF_CORR = 0.90

_CUDA_AVAILABLE = (
    platform.system() in ("Linux", "Windows") and os.environ.get("LUMICE_HAS_CUDA") == "1"
)

pytestmark = [
    pytest.mark.slow,
    pytest.mark.skipif(
        not _CUDA_AVAILABLE,
        reason=(
            "CUDA backend requires Linux/Windows + LUMICE_HAS_CUDA=1 + "
            "LUMICE_CUDA_ENABLED=ON build with an NVIDIA device. Skipping on this host."
        ),
    ),
]


def _run_cuda(seed: int) -> BufferedSimResult:
    return run_scene_capi_buffered(
        _CONFIG_PATH, sim_seed=seed, backend="cuda", timeout_sec=_TIMEOUT
    )


def _storm_count(result: BufferedSimResult) -> int:
    """Count the per-ray PolygonFaceOfTri WARNING lines (the fallback-driven storm)."""
    return sum("PolygonFaceOfTri" in line for line in result.log_lines)


@pytest.mark.parametrize("seed", _SEEDS)
def test_cuda_degenerate_no_fallback_no_storm(seed: int) -> None:
    """Degenerate K-shape pool must run on CUDA end-to-end: no fallback, no storm."""
    r = _run_cuda(seed)

    # The core isomorphism invariant that was broken: the backend must NOT drop
    # to legacy CPU on encountering a degenerate pool slot.
    assert r.routed_backend == "cuda", (
        f"seed={seed}: routed_backend={r.routed_backend!r} (expected 'cuda'). "
        f"The CUDA backend dropped mid-Run — the degenerate-slot fallback regression."
    )
    assert not r.fell_back, (
        f"seed={seed}: CUDA fell back to legacy (fell_back=True). BuildGeomPool must "
        f"tolerate a degenerate K-shape pool slot, not throw BackendUnavailableError."
    )

    # The fallback's downstream symptom: legacy InitRayFirstMs emits one
    # PolygonFaceOfTri WARNING per ray of a degenerate ci. Zero on the fixed path.
    storm = _storm_count(r)
    assert storm == 0, (
        f"seed={seed}: {storm} PolygonFaceOfTri WARNING(s) observed. This per-ray "
        f"log storm is emitted only on the legacy CPU fallback path — its presence "
        f"means the CUDA backend dropped."
    )

    # Real output, not a frame-collapsed / all-rays-dropped image.
    assert math.isfinite(r.snapshot_intensity) and r.snapshot_intensity > 0.0, (
        f"seed={seed}: snapshot_intensity={r.snapshot_intensity} — expected a finite "
        f"positive image (degenerate rays are dropped, but non-degenerate ones must render)."
    )


def test_cuda_degenerate_cross_seed_self_consistency() -> None:
    """Two CUDA seeds must produce the same halo structure (seed-stable), proving the
    tolerated-degenerate path renders genuine geometry, not neighbor-aliased garbage."""
    a = _run_cuda(_SEEDS[0])
    b = _run_cuda(_SEEDS[1])
    for label, r in (("A", a), ("B", b)):
        assert r.routed_backend == "cuda" and not r.fell_back, (
            f"cross-seed run {label}: routed={r.routed_backend!r} fell_back={r.fell_back} "
            f"— backend dropped, self-consistency check is meaningless."
        )
    corr = _raw_corr_ds_impl(a, b, _DS_BH, _DS_BW)
    assert corr >= _T_SELF_CORR, (
        f"CUDA cross-seed block-mean corr={corr:.4f} < {_T_SELF_CORR}. The degenerate "
        f"geometry halo should be seed-stable; a low corr suggests the tolerated "
        f"degenerate slots are still corrupting output."
    )
