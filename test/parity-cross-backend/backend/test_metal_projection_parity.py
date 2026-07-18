"""Per-projection Metal-vs-legacy-CPU parity battery (scrum-gpu-projection-parity 315.5).

Formalises cross-backend parity coverage for ALL 11 forward projections. After
315.2–315.4 both GPU backends project every exit through the shared
``lm_proj::ProjectExitToPixel`` (the SAME single source as the CPU oracle), and
``MetalTraceBackend::IsCompatible`` was relaxed to accept every lens type. This
suite pins two guarantees per type:

  (a) Metal-vs-legacy image parity — legacy CPU is the oracle. Three
      complementary metrics (mirrors the exit-seam battery, NOT loosened):
        - block-mean (4×4 ds) Pearson corr ≥ T_RAW_CORR_DS (0.95)
        - |sum(metal_Y)/sum(legacy_Y) − 1| ≤ T_ENERGY_TOL (0.05)
        - cross-seed self-consistency: metal_self ≥ legacy_self − T_SELF_MARGIN
      corr alone can mask undersampling ([[feedback_gpu_parity_corr_masks_undersampling]]),
      hence the energy + cross-seed axes.
  (b) NO silent fallback — the type routed to Metal (``routed_backend=="metal"``
      AND not ``fell_back``). This is the scrum's success criterion: the
      acceleration gap for the previously-CPU-only projections is closed.

``test_metal_projection_no_fallback_detector`` is the teeth check: a
multi-renderer config (projection-independent fallback trigger) MUST trip the
"falling back to legacy" log, proving the no-fallback assertions above can
meaningfully fail rather than silently pass on a degraded path.

The exit-seam battery already covers ``rectangular`` + ``dual_fisheye_equal_area``
via ``dual_fisheye_ref``; this file extends coverage to the full 11-type set with
a per-projection oracle. All tests are @pytest.mark.slow (shared-lib build:
``./scripts/build.sh -sj release``) and Darwin-only (Metal). The suite runs
serially (capi_runner mutates os.environ + a process-global log callback).
"""

from __future__ import annotations

import math
import platform

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e._parity_metrics import _raw_corr_ds as _raw_corr_ds_impl, _DS_BH, _DS_BW
from test.e2e._projection_battery import (
    PROJECTION_TYPES,
    T_ENERGY_TOL,
    T_PSNR_DB,
    T_RAW_CORR_DS,
    T_SELF_MARGIN,
    write_projection_config,
)
from test.e2e.runner import get_project_root

_SEED = 42
_SEED_B = 7
_TIMEOUT = 180

# Metal is Apple-only; on non-Darwin CI it falls back to legacy → the routing
# assertions would be false positives. Skip the whole module off macOS.
pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin", reason="Metal backend is only available on macOS"
)


# --------------------------------------------------------------------------- #
# Fixtures / helpers
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="module")
def _proj_configs(tmp_path_factory) -> dict:
    """Generate one single-MS config per projection type (module-scoped)."""
    out_dir = tmp_path_factory.mktemp("proj_parity_metal")
    return {t: write_projection_config(t, out_dir) for t in PROJECTION_TYPES}


def _raw_corr_ds(a: BufferedSimResult, b: BufferedSimResult) -> float:
    return _raw_corr_ds_impl(a, b, _DS_BH, _DS_BW)


def _render_psnr(a: BufferedSimResult, b: BufferedSimResult) -> float:
    if a.rgb_buf.shape != b.rgb_buf.shape:
        raise ValueError(f"render shape mismatch: {a.rgb_buf.shape} vs {b.rgb_buf.shape}")
    diff = a.rgb_buf.astype(float) - b.rgb_buf.astype(float)
    mse = float((diff * diff).mean())
    if mse == 0.0:
        return float("inf")
    return 10.0 * math.log10((255.0 * 255.0) / mse)


def _run(cfg_path, backend: str, seed: int = _SEED) -> BufferedSimResult:
    return run_scene_capi_buffered(str(cfg_path), sim_seed=seed, backend=backend, timeout_sec=_TIMEOUT)


def _assert_routed_metal(r: BufferedSimResult, lens_type: str) -> None:
    """No-fallback assertion (scrum success criterion): the type routed to Metal."""
    assert r.routed_backend == "metal", (
        f"{lens_type}: routed={r.routed_backend!r} (expected 'metal'); the core "
        f"log did not emit the MetalTraceBackend routing line. log tail: {r.log_lines[-5:]}"
    )
    assert not r.fell_back, (
        f"{lens_type}: Metal was requested but fell back to legacy CPU - the "
        f"projection is NOT actually accelerated. log tail: {r.log_lines[-5:]}"
    )


# --------------------------------------------------------------------------- #
# Per-projection parity — parametrized over all 11 LensParam::LensType values.
# --------------------------------------------------------------------------- #

@pytest.mark.slow
@pytest.mark.parametrize("lens_type", PROJECTION_TYPES)
def test_metal_projection_parity(lens_type: str, _proj_configs):
    """Metal-vs-legacy parity + no-fallback for one projection type."""
    cfg = _proj_configs[lens_type]

    legacy_a = _run(cfg, "legacy", seed=_SEED)
    metal_a = _run(cfg, "metal", seed=_SEED)

    # (b) No silent fallback — both the oracle and the accelerated run must route
    # as requested (legacy assertion also catches leaked env pollution).
    assert legacy_a.routed_backend == "legacy" and not legacy_a.fell_back, (
        f"{lens_type}: legacy oracle routed={legacy_a.routed_backend!r} "
        f"fell_back={legacy_a.fell_back} - env pollution suspected."
    )
    _assert_routed_metal(metal_a, lens_type)

    # (a) Parity metric 1: block-mean ds Pearson corr.
    corr = _raw_corr_ds(metal_a, legacy_a)
    psnr = _render_psnr(metal_a, legacy_a)

    # (a) Parity metric 2: total-Y energy conservation.
    metal_y = float(metal_a.flt_buf[..., 1].sum())
    legacy_y = float(legacy_a.flt_buf[..., 1].sum())
    assert legacy_y > 0.0, f"{lens_type}: legacy total Y == 0 - scene carries no signal"
    energy_ratio = metal_y / legacy_y

    # (a) Parity metric 3: cross-seed self-consistency (corr-blind undersampling).
    metal_b = _run(cfg, "metal", seed=_SEED_B)
    legacy_b = _run(cfg, "legacy", seed=_SEED_B)
    _assert_routed_metal(metal_b, lens_type)
    metal_self = _raw_corr_ds(metal_a, metal_b)
    legacy_self = _raw_corr_ds(legacy_a, legacy_b)

    print(
        f"[proj-parity] {lens_type}: ds_corr={corr:.4f} psnr={psnr:.2f}dB "
        f"energy_ratio={energy_ratio:.4f} metal_self={metal_self:.4f} "
        f"legacy_self={legacy_self:.4f}"
    )

    assert corr >= T_RAW_CORR_DS, (
        f"{lens_type}: ds_corr {corr:.4f} < {T_RAW_CORR_DS}. The shared "
        f"ProjectExitToPixel path diverged from the CPU oracle for this projection."
    )
    assert psnr >= T_PSNR_DB, f"{lens_type}: render PSNR {psnr:.2f} dB < {T_PSNR_DB}"
    assert abs(energy_ratio - 1.0) <= T_ENERGY_TOL, (
        f"{lens_type}: metal/legacy total-Y ratio {energy_ratio:.4f} outside "
        f"[1 +/- {T_ENERGY_TOL}] - global energy imbalance in the exit/emit path."
    )
    assert metal_self >= legacy_self - T_SELF_MARGIN, (
        f"{lens_type}: metal cross-seed self-consistency {metal_self:.4f} < "
        f"legacy_self {legacy_self:.4f} - {T_SELF_MARGIN} - suspect orientation "
        f"undersampling (corr-blind) in the Metal path for this projection."
    )


# --------------------------------------------------------------------------- #
# Negative smoke — verify the no-fallback detector has teeth.
# --------------------------------------------------------------------------- #

@pytest.mark.slow
def test_metal_projection_no_fallback_detector():
    """A multi-renderer config MUST trip the fallback log on Metal.

    Since 315.3/315.4 relaxed IsCompatible to accept ALL lens types, lens type
    is no longer a fallback trigger. The multi-renderer path (CanUseBackend:
    ``renders_->size() != 1``) is a projection-INDEPENDENT fallback that still
    fires — this is the insurance that the per-projection ``_assert_routed_metal``
    checks above can meaningfully fail rather than silently pass on a degraded
    path.
    """
    cfg = get_project_root() / "test" / "e2e" / "configs" / "multi_lens.json"
    r = _run(cfg, "metal")
    assert r.fell_back, (
        "Expected Metal fallback on a multi-renderer config (multi_lens.json) but "
        f"got fell_back=False. The no-fallback detector may be broken. "
        f"log tail: {r.log_lines[-5:]}"
    )
