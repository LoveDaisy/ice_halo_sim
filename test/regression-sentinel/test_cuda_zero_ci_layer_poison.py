"""Regression guard: a layer that traces ZERO rays must not poison the CUDA backend.

Defect this guards against
--------------------------
Three independent faults in `src/core/backend/cuda_trace_backend.cu` chained into one
symptom — the whole GPU backend silently dropping to legacy CPU:

1. `CudaTraceBackend::TraceLayer()` records `ev_end_h2d_` / `ev_end_kernel_` INSIDE the
   per-crystal-instance loop, and that loop `continue`s on `ci_n == 0`. When every
   crystal in the layer has `crystal_proportion_ == 0`, `PartitionCrystalRayNum` returns
   an all-zero split (`simulator.cpp` short-circuits on `total_prop <= 0`), so the loop
   body never runs and neither event is recorded. The events are created once per backend
   INSTANCE (`events_created_`) while a backend is built per `Run()`, so if the FIRST
   `TraceLayer` of a Run is all-zero the pair has never been recorded at all.
2. The three `cudaEventElapsedTime` calls after the loop were unchecked. On an unrecorded
   event the call returns `cudaErrorInvalidResourceHandle` and leaves it on CUDA's
   thread-local sticky error slot — which nothing then consumed.
3. The kernel-launch checks read that slot with `cudaPeekAtLastError()`, which does NOT
   clear it. So the NEXT Run's first `gen_root_kernel` launch check read an error left
   behind by a previous Run's *timer read* and blamed the launch:
   `TraceBackend unavailable (CudaTraceBackend::TraceLayer: gen_root_kernel launch:
   invalid resource handle); dropping backend and falling back to legacy CPU`.

The user-visible face of this was a Windows/CUDA report: run once with every population
weight at 0, then give one population a weight, and the first seconds come back garishly
over-saturated — the fallback happens while the dispatch grain stays at the GPU's, so each
legacy batch carries a single wavelength.

Why this test commits TWICE
---------------------------
The poisoning and the reading of the poison happen in DIFFERENT runs, and that is not an
incidental detail of how it was found — it is the only shape in which the defect is
reachable. Within one Run, a layer whose partition is all-zero produces zero
continuations, so the next layer's `TraceLayer` early-returns on `n == 0` before reaching
any launch check, and the sticky error is never read. The error survives because it lives
on the CUDA runtime's per-HOST-THREAD state, which outlives the backend object; only a
subsequent Run on that same simulator thread walks into it. A single static config
therefore cannot express this, and a future simplification to one commit would silently
lose all reproducing power while still passing. `run_scene_sequence_capi_buffered` exists
for that reason: it keeps one server, one simulator thread, and one log capture across
both commits.

Phase A/B differ in EXACTLY one line (verified: `proportion: 0.0` -> `100.0` on one entry),
so "the zero-weight layer" is the only variable between them.

Cross-backend audit (Metal / cpu_backend): NOT affected, and not for want of a similar
loop
--------------------------------------------------------------------------------------
`metal_trace_backend.mm` has the same `if (ci_n == 0) { continue; }` in its per-crystal
loop, but none of the three faults has a counterpart there: it does no GPU-side segment
timing (no `cudaEvent*` analogue, no `GPUStartTime`/`GPUEndTime`/`addCompletedHandler`
readings around the loop), so there is no unrecorded-handle read to fail; and Metal
reports errors through `MTLCommandBuffer.error`, a per-command-buffer property, not a
thread-local sticky slot, so there is nothing that could be carried into a later Run and
mis-attributed. `cpu_trace_backend.cpp` has neither device timing nor a device error
model. So this is "no same-shaped code", not "same-shaped code that happens not to fire".
Confirmed by inspection only; those two backends are deliberately left untouched here.

NO CI JOB RUNS THIS TEST
------------------------
Stated plainly because a green pipeline is not evidence about it. The main matrix's
Windows job builds with `BUILD_TEST` but without CUDA; the CUDA jobs build with CUDA but
without `BUILD_TEST`; the Linux CUDA leg likewise has no test job. The two halves both
exist and their intersection is empty, so this file is evaluated on a CUDA reference
machine, by hand, and nowhere else:

    ./scripts/build.sh -sj release        # shared lib, CUDA-enabled build
    LUMICE_HAS_CUDA=1 pytest -v -m slow test/regression-sentinel/test_cuda_zero_ci_layer_poison.py

Do not cite CI for this guard. See `doc/gpu-remote-cuda-build-testing.md` for the
reference-machine protocol and for why `LUMICE_HAS_CUDA` (this un-skip gate) and
`LUMICE_CUDA_ENABLED` (the build switch) must both be set and are not the same thing.

@pytest.mark.slow: needs the CUDA-enabled shared-lib build (LUMICE_LIB) + an NVIDIA
device. CUDA-gated, so it is inert on non-CUDA hosts (CI, macOS).
"""
from __future__ import annotations

import math
import os
import platform

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_sequence_capi_buffered
from test.e2e.runner import get_project_root

_CONFIG_DIR = get_project_root() / "test" / "e2e" / "configs"
_PHASE_A = str(_CONFIG_DIR / "repro_cuda_zero_ci_layer_phase_a.json")
_PHASE_B = str(_CONFIG_DIR / "repro_cuda_zero_ci_layer_phase_b.json")

_TIMEOUT = 600
_SEED = 42

# The exact diagnostic the defect emitted. Asserted on top of the coarse `fell_back`
# flag so a failure distinguishes THIS defect from any other reason a backend might be
# dropped — the flag alone would report the two identically.
_POISON_MARKERS = ("TraceBackend unavailable", "dropping backend")

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


def _poison_lines(result: BufferedSimResult) -> list:
    return [ln for ln in result.log_lines if any(m in ln for m in _POISON_MARKERS)]


def test_cuda_survives_a_zero_ray_layer() -> None:
    """An all-zero-weight run must leave the CUDA backend usable for the NEXT run."""
    r = run_scene_sequence_capi_buffered(
        [_PHASE_A, _PHASE_B], sim_seed=_SEED, backend="cuda", timeout_sec=_TIMEOUT
    )

    # The defect's own signature, checked first because it names the cause.
    poisoned = _poison_lines(r)
    assert not poisoned, (
        "the CUDA backend was dropped after a zero-ray layer — the sticky-error "
        "poisoning regression. Offending log line(s):\n  " + "\n  ".join(poisoned)
    )

    # The coarse invariants: phase B really ran on CUDA, start to finish.
    assert r.routed_backend == "cuda", (
        f"routed_backend={r.routed_backend!r} (expected 'cuda'). The second commit did "
        f"not run on the GPU backend at all."
    )
    assert not r.fell_back, (
        "a fallback warning was observed during the sequence. If no line above matched "
        "the poisoning signature, this is a DIFFERENT fallback — read the log:\n  "
        + "\n  ".join(ln for ln in r.log_lines if "falling back" in ln)
    )

    # Real output: a weighted population must render, i.e. the run that follows the
    # zero-ray one is not merely "not fallen back" but actually productive.
    assert r.has_valid_data, "phase B produced no valid data"
    assert math.isfinite(r.snapshot_intensity) and r.snapshot_intensity > 0.0, (
        f"snapshot_intensity={r.snapshot_intensity} — phase B weights one population, so "
        f"it must render a finite positive image."
    )
