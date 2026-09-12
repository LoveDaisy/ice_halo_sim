"""CUDA host root-gen fallback parity: ``LUMICE_DISABLE_DEVICE_GEN=1`` must draw the same picture.

The CUDA backend generates first-layer rays on the device by default
(``gen_root_kernel``). ``LUMICE_DISABLE_DEVICE_GEN=1`` is the escape hatch that
moves root generation back to the host (``InitRayFirstMs``) and uploads the
roots per batch; the trace kernels downstream are the same either way, so the
two arms are two production paths to one image and must agree to within the
battery's usual sampling tolerance.

What this guards. The host-roots branch once uploaded every per-ray carrier the
trace kernel reads — position, direction, weight, wavelength, crystal index —
except the per-ray K-shape carrier (``d_root_pool_shape_``), which only the
device-gen kernel wrote. On the fallback path that buffer held whatever
``cudaMalloc`` returned, the polygon traversal read a face count of zero for
every ray, and the image came out **all black** — no crash, no warning, no
red anywhere: the gtest that covers this fallback skipped itself on "landed
no energy", and no parity test ran the fallback arm at all. The fix writes the
carrier on the host path too. This file is the missing parity row: revert the
fix and both assertions below fail at once (corr collapses to 0, energy ratio
to 0), which is also how the row was calibrated.

Thresholds are the battery's own (``_T_RAW_CORR_DS`` / ``_T_ENERGY_TOL`` as in
``test_cuda_exit_seam_parity.py``): the two arms sample different roots, so
the comparison is subject to the same Monte-Carlo divergence as any
cuda-vs-legacy row, not to a byte-exact bar. Measured on the fixed code
through this harness: ds_corr 1.0000 (rounded), energy ratio 1.0000 (host_Y
2.61237e8 vs dev_Y 2.61242e8); the CLI probe that found the defect read
0.9954 / 1.0003 on the same config through 8-bit JPEGs. Under the revert both
read 0.

Requires:
  - ``LUMICE_CUDA_ENABLED=ON`` build with the CUDA toolchain.
  - NVIDIA device visible to the runtime.
  - Shared-lib build produced by the CUDA-enabled Release configuration
    (a plain ``./scripts/build.sh -sj release`` is not enough on its own).

All tests are @pytest.mark.slow.
"""
from __future__ import annotations

import contextlib
import os
import platform
from typing import Iterator

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e._parity_metrics import (
    _raw_corr_ds as _raw_corr_ds_impl,
    _DS_BH,
    _DS_BW,
)
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_HOSTGEN_CONFIG = "dual_fisheye_ref"  # 10M rays, single crystal, D65
_SEED = 42
_TIMEOUT = 900

# Same bar as the exit-seam battery (G1 / G2). The two arms are two samplers of
# one scene, so the row is held to sampling tolerance, not to identity.
_T_RAW_CORR_DS = 0.95
_T_ENERGY_TOL = 0.05

# The knob's read site logs this once per process (std::call_once) the first
# time a session sees it set. Asserting it on the host-gen arm is what keeps
# the row from passing vacuously as device-gen vs device-gen should the env
# plumbing ever stop reaching BeginSession. It holds because this file's one
# case is the only host-gen session in the battery; a second host-gen case in
# the same process would see the line only on whichever ran first.
_HOSTGEN_LOG_MARK = "env override: LUMICE_DISABLE_DEVICE_GEN=1"

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


@contextlib.contextmanager
def _disable_device_gen() -> Iterator[None]:
    """Set ``LUMICE_DISABLE_DEVICE_GEN=1`` for one run, restoring the prior value after.

    Same set-then-restore shape ``run_scene_capi_buffered`` uses for
    ``LUMICE_TRACE_BACKEND``. The knob is read in ``BeginSession`` of a fresh
    backend instance, and every ``run_scene_capi_buffered`` call creates and
    destroys its own server, so setting it around one call scopes it to that
    call's session.
    """
    was_set = "LUMICE_DISABLE_DEVICE_GEN" in os.environ
    old = os.environ.get("LUMICE_DISABLE_DEVICE_GEN")
    os.environ["LUMICE_DISABLE_DEVICE_GEN"] = "1"
    try:
        yield
    finally:
        if was_set and old is not None:
            os.environ["LUMICE_DISABLE_DEVICE_GEN"] = old
        else:
            os.environ.pop("LUMICE_DISABLE_DEVICE_GEN", None)


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


@pytest.mark.slow
def test_cuda_hostgen_fallback_matches_devicegen():
    """host-gen (``LUMICE_DISABLE_DEVICE_GEN=1``) vs device-gen, both routed to cuda.

    The knob changes where roots are generated, not which backend runs, so
    both arms must still route to ``CudaTraceBackend``; the host-gen arm must
    additionally show the knob's one-shot log line, or the comparison is
    device-gen against itself.
    """
    cfg = _HOSTGEN_CONFIG
    with _disable_device_gen():
        hostgen = _run(cfg, "cuda", _SEED)
    devicegen = _run(cfg, "cuda", _SEED)

    _assert_routed(hostgen, "cuda", cfg)
    _assert_routed(devicegen, "cuda", cfg)
    assert any(_HOSTGEN_LOG_MARK in ln for ln in hostgen.log_lines), (
        f"{cfg}/host-gen: no {_HOSTGEN_LOG_MARK!r} line in the session log; "
        "the knob did not reach BeginSession and this arm is device-gen too."
    )
    assert not any(_HOSTGEN_LOG_MARK in ln for ln in devicegen.log_lines), (
        f"{cfg}/device-gen: {_HOSTGEN_LOG_MARK!r} leaked into the device-gen arm's session; "
        "the env restore in _disable_device_gen did not take."
    )

    corr = _raw_corr_ds(hostgen, devicegen)
    host_Y = float(hostgen.flt_buf[..., 1].sum())
    dev_Y = float(devicegen.flt_buf[..., 1].sum())
    assert dev_Y > 0.0, f"{cfg}: device-gen total Y == 0; cannot form energy ratio"
    energy_ratio = host_Y / dev_Y

    print(
        f"[hostgen-parity] {cfg}: ds_corr={corr:.4f} energy_ratio(host/dev)={energy_ratio:.4f} "
        f"host_Y={host_Y:.6g} dev_Y={dev_Y:.6g} (corr floor {_T_RAW_CORR_DS}, "
        f"energy tol +/-{_T_ENERGY_TOL})"
    )

    assert corr >= _T_RAW_CORR_DS, (
        f"{cfg}: host-gen vs device-gen ds_corr {corr:.4f} < {_T_RAW_CORR_DS}. "
        f"host_Y={host_Y:.6g} dev_Y={dev_Y:.6g}. Before the host-roots branch wrote the "
        "per-ray K-shape carrier this arm rendered all black and failed here and on the "
        "energy ratio together; suspect a carrier the fallback path uploads and the "
        "device-gen kernel writes having diverged again."
    )
    assert abs(energy_ratio - 1.0) <= _T_ENERGY_TOL, (
        f"{cfg}: host-gen/device-gen total-Y ratio {energy_ratio:.4f} outside "
        f"[1 +/- {_T_ENERGY_TOL}] (host_Y={host_Y:.6g} dev_Y={dev_Y:.6g}). "
        "Same suspect as the correlation assertion: the host-roots upload set is "
        "missing a carrier the trace kernel reads."
    )
