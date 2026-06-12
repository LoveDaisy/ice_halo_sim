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
import platform
from pathlib import Path

import numpy as np
import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered

# Block-mean ds-corr metric — single source of truth shared with the 258.6
# parity suite, hoisted into a tracked module (see _parity_metrics.py header).
from test.e2e._parity_metrics import (
    _raw_corr_ds as _raw_corr_ds_impl,
    _DS_BH,
    _DS_BW,
)

CONFIGS_DIR = Path(__file__).parent / "configs"
_TIMEOUT = 180

# Metal backend is Apple-only. Skip on non-Darwin CI runners (the Ubuntu
# e2e-slow matrix leg) where forcing the metal backend falls back to legacy and
# the routed-backend assertions below would fail.
pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin", reason="Metal backend is only available on macOS"
)


def _raw_corr_ds(a: BufferedSimResult, b: BufferedSimResult) -> float:
    return _raw_corr_ds_impl(a, b, _DS_BH, _DS_BW)


def _run_metal(
    config_name: str,
    disable_device_gen: bool,
    sim_seed: int = 0,
) -> BufferedSimResult:
    """Run the metal backend with the given sim_seed.

    Toggles `LUMICE_DISABLE_DEVICE_GEN` via os.environ around the call. The
    runner only manages `LUMICE_TRACE_BACKEND`, so this wrapper owns the
    device-gen env var lifecycle.

    sim_seed=0 (default) preserves the original multi-worker random-mode path.
    Non-zero sim_seed is passed through to `DeriveEffectiveSeed`
    (src/core/simulator.cpp:481-486) and returned verbatim, collapsing to
    single-worker — used by the fixed-seed activation proof test to compare
    PCG(seed) vs mt19937(seed).
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
            cfg, sim_seed=sim_seed, backend="metal", timeout_sec=_TIMEOUT,
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


# Activation-proof RelErr floor. PCG vs mt19937 with the same effective seed
# yield distinct sample streams; on dual_fisheye_ref the sum-of-XYZ relative
# difference measures ~4e-5 empirically (single-worker sim_seed=42, 2026-06-11).
# Per-pixel variance is larger but the global sum has strong MC cancellation —
# total-sum RelErr is the conservative end of the differentiation envelope.
# Floor 1e-5 is below the ~4e-5 active-path measurement (~4× headroom) and far
# above the byte-equal fallback regime (rel_err == 0 if device-gen silently
# degrades to host-gen), so the test cleanly distinguishes the two cases.
# Both runs are single-worker deterministic at sim_seed=42, so the gap has no
# run-to-run variance and the 4× headroom is not flaky.
_ACTIVATION_RELERR_FLOOR = 1e-5


@pytest.mark.slow
def test_device_gen_activation_proof_fixed_seed():
    """device-gen ON (GPU PCG) vs OFF (host mt19937) at fixed seed must diverge.

    Uses sim_seed=42. `DeriveEffectiveSeed` (src/core/simulator.cpp:481-486) is
    `if (seed != 0) return seed;` — non-zero seeds bypass the global atomic
    counter and are returned verbatim, so ON and OFF both see
    `effective_seed_=42` deterministically (unit guard:
    test/test_simulator.cpp:656 SimulatorEffectiveSeed.FixedSeedPreserved).
    The Metal device-gen gate (src/core/metal_trace_backend.mm:2157-2161)
    requires `gen_seed_!=0` ∧ `tri_count≤64`; dual_fisheye_ref is a single
    prism crystal (~20 triangles ≤ 64) and seed=42 ≠ 0, so the activation
    envelope is satisfied.

    With the same seed but different RNG algorithms (GPU PCG vs host mt19937)
    the two runs produce different XYZ buffers — the total-sum relative error
    is ~4e-5 empirically (actual: rel_err=4.4e-5, single-worker sim_seed=42,
    2026-06-11), ~4× above the 1e-5 floor. If device-gen silently fell back to
    host-gen (same RNG on both sides), ON ≡ OFF byte-for-byte and RelErr ≈ 0,
    failing the assertion. This closes the "assert-may-pass" hole that
    sim_seed=0 leaves open (each side draws different atomic-counter seeds,
    so ds_corr cannot distinguish active vs fallback).
    """
    on = _run_metal("dual_fisheye_ref", disable_device_gen=False, sim_seed=42)
    off = _run_metal("dual_fisheye_ref", disable_device_gen=True, sim_seed=42)

    assert on.routed_backend == "metal" and not on.fell_back, (
        f"device-gen ON run did not route through metal (routed={on.routed_backend!r}, "
        f"fell_back={on.fell_back}); tail: {on.log_lines[-5:]}"
    )
    assert off.routed_backend == "metal" and not off.fell_back, (
        f"device-gen OFF run did not route through metal (routed={off.routed_backend!r}, "
        f"fell_back={off.fell_back}); tail: {off.log_lines[-5:]}"
    )

    on_sum = float(np.sum(on.flt_buf))
    off_sum = float(np.sum(off.flt_buf))
    # Symmetric denominator: more conservative than the single-sided form in
    # plan §3 pseudo-code; 4× headroom over the floor confirmed empirically.
    rel_err = abs(on_sum - off_sum) / (abs(on_sum) + abs(off_sum) + 1e-30)
    print(
        f"[activation-proof] dual_fisheye_ref sim_seed=42 metal: "
        f"on_sum={on_sum:.6e} off_sum={off_sum:.6e} rel_err={rel_err:.3e} "
        f"(floor={_ACTIVATION_RELERR_FLOOR:.0e})"
    )
    assert rel_err > _ACTIVATION_RELERR_FLOOR, (
        f"dual_fisheye_ref sim_seed=42: device-gen ON vs OFF rel_err={rel_err:.3e} "
        f"<= {_ACTIVATION_RELERR_FLOOR:.0e} — GPU PCG and host mt19937 produced "
        f"near-identical output at the same seed. device-gen likely fell back "
        f"to host-gen; the default-path coverage in this file is then assert-may-pass."
    )
