"""CUDA energy-accounting parity: the two ledgers of one backend must agree.

Every backend keeps two independent tallies of the energy that landed on the
image:

  * the per-pixel XYZ buffer (``flt_buf``), accumulated pixel by pixel from
    ``kCmfY[wl] * w_exit`` — this is what the image is made of; and
  * ``snapshot_intensity``, a single scalar summed from the same exit weights
    (``landed_weight`` → ``total_intensity_``) and then divided by the
    normalization ``kNormScale * total_pix`` in the renderer — this is what
    the renderer exposes through the C API and what exposure math reads.

For a single-wavelength scene the two are tied by a constant that does not
depend on the rays at all:

    R = sum(flt_buf[..., Y]) / snapshot_intensity = kCmfY[wl] * kNormScale * total_pix

so ``R`` is deterministic up to float accumulation order, and ``R_cuda /
R_legacy`` measures whether the CUDA backend's *own* two ledgers stay in step
with each other the way legacy's do. This is a different invariant from the
``energy_ratio`` the other three CUDA parity files assert: ``energy_ratio``
compares one ledger across the two backends and tolerates the ±5% Monte-Carlo
divergence two different samplers legitimately produce; ``R`` compares the two
ledgers *within* each backend, where there is no sampling term to hide behind,
which is why its tolerance is two orders of magnitude tighter.

What this guards. The CUDA backend once accumulated ``landed_weight`` as a
single fp32 device scalar with one ``atomicAdd`` per exit, alive across the
whole drain window (64 batches × 262144 rays). Once the running sum reached
~2e6 its ulp was 0.125–0.25, and every exit weight below half an ulp was
dropped whole — a systematic under-count that the per-pixel buffer (each pixel
summing to ~15) never suffered. The two ledgers diverged by a deterministic
+1.74% on ``R`` while every image-side parity metric stayed green, because the
image ledger was the correct one; only the scalar the exposure pipeline reads
was wrong. The fix reduces per warp in registers and folds per layer into a
host ``double``. No parity test read ``snapshot_intensity`` at the time, so
the defect had no machine signal for as long as it lived; this file is that
signal. Revert the fix and the ``cpu_backend_route`` rows below read ~+1.7%
against a 0.1% tolerance.

Tolerance. After the fix the cross-backend spread of ``R`` measured ≤0.005%
across every single-wavelength scene used here (worst row −0.004%). The
tolerance is 0.1%: ~24× above the measured spread, ~17× below the defect it
exists to catch. It is deliberately not tighter — legacy's own ``R`` sits
+0.019% above the closed-form constant because its ``total_intensity_`` is
also an fp32 running sum (over a much shorter window), and that is accepted
behaviour, not a defect.

Scene selection. Only single-wavelength configs: under a D65 spectrum ``R``
becomes ``sum(cmf_y * w) / sum(w)``, which varies ~1% between seeds on legacy
alone (it draws one wavelength per batch), so the same ratio measured there
cannot resolve a sub-percent accounting error. The four configs cover the
four session shapes the CUDA backend has (single-MS no filter; fisheye 120°
view where most exits fall outside the frame; single-MS with a filter and
random geometry; two-layer MS with five crystals) — the per-layer fold is
only exercised by the last one.

Requires:
  - ``LUMICE_CUDA_ENABLED=ON`` build with the CUDA toolchain.
  - NVIDIA device visible to the runtime.
  - Shared-lib build produced by the CUDA-enabled Release configuration
    (a plain ``./scripts/build.sh -sj release`` is not enough on its own).

All tests are @pytest.mark.slow.
"""
from __future__ import annotations

import math
import os
import platform

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_TIMEOUT = 900  # parhelion.json is 10M rays; the legacy arm is the slow one

# |R_cuda / R_legacy - 1| bound. See the module docstring for the derivation
# (measured post-fix spread ≤ 0.005%, defect signature +1.74%).
_T_R_RATIO_TOL = 0.001

_CUDA_AVAILABLE = (
    platform.system() in ("Linux", "Windows") and os.environ.get("LUMICE_HAS_CUDA") == "1"
)

pytestmark = pytest.mark.skipif(
    not _CUDA_AVAILABLE,
    reason=(
        "CUDA backend requires Linux + LUMICE_HAS_CUDA=1 + LUMICE_CUDA_ENABLED=ON "
        "build with an NVIDIA device. Skipping on this host."
    ),
)

# (config, seeds). Every entry is a single-wavelength scene (see docstring);
# adding a D65 config here would make the tolerance meaningless, not stricter.
_R_RATIO_CASES = (
    ("cpu_backend_route", (42, 43, 44)),            # 555 nm, rectangular 180°, 2M rays
    ("parhelion", (42, 43, 44)),                    # 550 nm, fisheye_equal_area 120°, 10M rays
    ("parity_random_geometry", (42, 43)),           # 550 nm, filter + random geometry
    ("orientation_sample_count_random", (42, 43)),  # 540 nm, 5 crystals, two MS layers
)
_R_RATIO_PARAMS = [(cfg, seed) for cfg, seeds in _R_RATIO_CASES for seed in seeds]
_R_RATIO_IDS = [f"{cfg}-seed{seed}" for cfg, seed in _R_RATIO_PARAMS]


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


def _y_sum(r: BufferedSimResult) -> float:
    return float(r.flt_buf[..., 1].sum())


def _r_ratio(r: BufferedSimResult) -> float:
    """``sum(flt_buf Y) / snapshot_intensity``; nan when the scalar ledger is empty.

    Returning nan rather than raising keeps the diagnosis in the assertion
    message below, next to the other side's numbers, instead of in a traceback
    that shows only one backend.
    """
    if r.snapshot_intensity == 0.0:
        return float("nan")
    return _y_sum(r) / float(r.snapshot_intensity)


@pytest.mark.slow
@pytest.mark.parametrize(("config", "seed"), _R_RATIO_PARAMS, ids=_R_RATIO_IDS)
def test_cuda_energy_ledger_matches_legacy(config: str, seed: int):
    """|R_cuda / R_legacy − 1| ≤ 0.1% on a single-wavelength scene.

    Both arms run the same config and seed; the ratio is formed from each
    backend's own two ledgers, so a red here means one backend's scalar and
    per-pixel accounting have come apart — not that the two backends sampled
    different rays.
    """
    legacy = _run(config, "legacy", seed)
    cuda = _run(config, "cuda", seed)

    _assert_routed(legacy, "legacy", config)
    _assert_routed(cuda, "cuda", config)

    r_legacy = _r_ratio(legacy)
    r_cuda = _r_ratio(cuda)

    detail = (
        f"legacy: Ysum={_y_sum(legacy):.6g} snapshot_intensity={legacy.snapshot_intensity:.6g} "
        f"R={r_legacy:.6f}; "
        f"cuda: Ysum={_y_sum(cuda):.6g} snapshot_intensity={cuda.snapshot_intensity:.6g} "
        f"R={r_cuda:.6f}"
    )

    assert not math.isnan(r_legacy) and r_legacy > 0.0, (
        f"{config}/seed{seed}: legacy R is not a positive number ({detail}); "
        "the reference arm landed nothing, so no ratio can be formed."
    )
    assert not math.isnan(r_cuda) and r_cuda > 0.0, (
        f"{config}/seed{seed}: cuda R is not a positive number ({detail}); "
        "the cuda scalar ledger is empty while its image ledger is being compared."
    )

    ratio = r_cuda / r_legacy
    print(
        f"[energy-ledger] {config}/seed{seed}: R_cuda/R_legacy={ratio:.6f} "
        f"({(ratio - 1.0) * 100:+.4f}%, tol +/-{_T_R_RATIO_TOL * 100:.2f}%) — {detail}"
    )
    assert abs(ratio - 1.0) <= _T_R_RATIO_TOL, (
        f"{config}/seed{seed}: R_cuda/R_legacy = {ratio:.6f} "
        f"({(ratio - 1.0) * 100:+.4f}%) outside [1 +/- {_T_R_RATIO_TOL}]. {detail}. "
        "The cuda backend's scalar ledger (snapshot_intensity) and per-pixel ledger "
        "(flt_buf Y) have come apart. Suspect landed_weight accumulation: a single fp32 "
        "device scalar summed across the whole drain window drops sub-ulp exit weights "
        "and reads ~+1.7% here."
    )
