"""Default-path device-gen sanity test (task 260.6).

Confirms that the Metal device-gen path activates and produces statistically
equivalent output in the **default multi-worker random mode** (sim_seed=0),
which is what GUI/CLI users hit by default.

Before task 260.6 the Metal backend's device-gen path was gated on
`gen_seed_ != 0`, and `sim_seed=0` made the Simulator hand the backend
`SessionSpec.seed = 0` → device-gen was silently disabled for every default
render. Task 260.6 derives a non-zero `effective_seed_` per Simulator instance
from a global atomic counter so device-gen activates on the default path while
preserving the deterministic semantics of `sim_seed != 0`.

The test runs the metal backend twice with `sim_seed=0`:
  - device-gen ON  (LUMICE_DISABLE_DEVICE_GEN unset)
  - device-gen OFF (LUMICE_DISABLE_DEVICE_GEN=1, forces host root-gen)
and asserts the block-mean (4×4) Pearson correlation between the two raw XYZ
buffers stays above a conservative floor — both runs are Monte-Carlo with
different RNG keys, so the metric measures "no systematic divergence" rather
than bit equality.

@pytest.mark.slow: requires the shared-lib build (`./scripts/build.sh -sj release`).
"""
import os
import sys
from pathlib import Path

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered

CONFIGS_DIR = Path(__file__).parent / "configs"
_TIMEOUT = 180

# Pull the block-mean ds-corr metric from the same scratchpad source the
# 258.6 parity suite uses — single source of truth.
_HARNESS_DIR = (
    Path(__file__).resolve().parents[2]
    / "scratchpad" / "scrum-metal-exit-seam" / "task-parity-harness"
)
sys.path.insert(0, str(_HARNESS_DIR))
from _measure_baseline import (  # noqa: E402  (sys.path mutated above)
    _raw_corr_ds as _raw_corr_ds_impl,
    _DS_BH,
    _DS_BW,
)


def _raw_corr_ds(a: BufferedSimResult, b: BufferedSimResult) -> float:
    return _raw_corr_ds_impl(a, b, _DS_BH, _DS_BW)


def _run_metal(config_name: str, disable_device_gen: bool) -> BufferedSimResult:
    """Run the metal backend with sim_seed=0 (default multi-worker random mode).

    Toggles `LUMICE_DISABLE_DEVICE_GEN` via os.environ around the call. The
    runner only manages `LUMICE_TRACE_BACKEND`, so this wrapper owns the
    device-gen env var lifecycle.
    """
    env_was_set = "LUMICE_DISABLE_DEVICE_GEN" in os.environ
    env_old = os.environ.get("LUMICE_DISABLE_DEVICE_GEN")
    if disable_device_gen:
        os.environ["LUMICE_DISABLE_DEVICE_GEN"] = "1"
    else:
        os.environ.pop("LUMICE_DISABLE_DEVICE_GEN", None)
    try:
        cfg = str(CONFIGS_DIR / f"{config_name}.json")
        return run_scene_capi_buffered(
            cfg, sim_seed=0, backend="metal", timeout_sec=_TIMEOUT,
        )
    finally:
        if env_was_set:
            os.environ["LUMICE_DISABLE_DEVICE_GEN"] = env_old
        else:
            os.environ.pop("LUMICE_DISABLE_DEVICE_GEN", None)


# Initial threshold: 0.85 (plan §6). sim_seed=0 means each run uses a different
# RNG key, so the metric measures distributional equivalence, not bit equality.
# Recalibrate if the first run shows the headroom is much larger than 0.05.
_DS_CORR_FLOOR = 0.85


@pytest.mark.slow
def test_default_path_device_gen_vs_host_gen_single_ms():
    """sim_seed=0, single-MS scene: device-gen ON vs OFF must agree statistically.

    Uses dual_fisheye_ref (the 258.6 single-MS no-filter parity baseline) — a
    single-crystal, ≤64-face scene that matches the device-gen activation
    envelope (single MS layer, host fallback gated on geometry-pool 260.3 not
    yet landed).
    """
    on = _run_metal("dual_fisheye_ref", disable_device_gen=False)
    off = _run_metal("dual_fisheye_ref", disable_device_gen=True)

    assert on.routed_backend == "metal" and not on.fell_back, (
        f"device-gen ON run did not route through metal (routed={on.routed_backend!r}, "
        f"fell_back={on.fell_back}); tail: {on.log_lines[-5:]}"
    )
    assert off.routed_backend == "metal" and not off.fell_back, (
        f"device-gen OFF run did not route through metal (routed={off.routed_backend!r}, "
        f"fell_back={off.fell_back}); tail: {off.log_lines[-5:]}"
    )

    ds = _raw_corr_ds(on, off)
    print(
        f"[default-path] dual_fisheye_ref sim_seed=0 metal: "
        f"device-gen ON vs OFF ds_corr={ds:.4f} (floor={_DS_CORR_FLOOR})"
    )
    assert ds >= _DS_CORR_FLOOR, (
        f"dual_fisheye_ref default path: device-gen ON vs OFF ds_corr={ds:.4f} "
        f"< {_DS_CORR_FLOOR} — systematic divergence between device-gen and "
        f"host-gen on the default render path."
    )
