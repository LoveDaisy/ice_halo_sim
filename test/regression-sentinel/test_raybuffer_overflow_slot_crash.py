"""Regression guard: small LUMICE_DISPATCH_RAY_NUM must not corrupt the hit-loop buffers.

Fix commit: the hit-loop buffer pair (`buffer_data[0]` / `buffer_data[1]` in
`Simulator::SimulateOneWavelength`) is now sized by `ResetHitLoopBuffers`, which
gives `buffer_data[1]` twice `buffer_data[0]`'s capacity.

Root cause: both buffers were reset to `ray_num * 2`, which sizes
`buffer_data[1]` for the NOMINAL fan-out and leaves zero margin. Each hit turns
every ray in `buffer_data[0]` into two children in `buffer_data[1]`, and the
nominal case assumes at most one of the two survives as "normal" (the reflected
child stays inside the crystal, the refracted one exits). Nothing enforced that
assumption. One grazing/near-edge ray whose refracted child still found a next
face pushed `buffer_data[0].size_` to `curr_ray_num + 1`, and the next hit's
fan-out then wrote past the end of `buffer_data[1]`'s `rays_` / `recorders_` /
`components_` arrays — corrupting the adjacent heap. The corrupted bytes landed
in a neighbouring `RaypathRecorder::overflow_idx_`, so `HasOverflow()` went true
on a recorder that owned no arena slot and `RayBuffer::DupOverflowSlot` memcpy'd
from a wild pointer. The visible crash was therefore three frames away from its
cause.

Why the batch size selects the bug: the per-crystal batch is
`curr_ray_num = min(geom_clock_, ...)` and `geom_clock_` defaults to
`Simulator::kSmallBatchRayNum == 32`. For `LUMICE_DISPATCH_RAY_NUM <= 32` the
batch equals the dispatch size, so the fan-out needs exactly the whole capacity
and a single extra normal ray overflows. Larger dispatch sizes clamp the batch
at 32 while the capacity keeps growing, which buys `2 * ray_num - 64` slots of
accidental slack — that slack, not correctness, is why the defect was invisible
at the shipped default.

The parametrisation therefore straddles the cliff on purpose: {8, 16, 24, 32}
are the zero-margin sizes that crashed, and 48 is a control above the clamp that
passed both before and after the fix. A sentinel that only covered the crashing
sizes could not tell "always crashes" apart from "crashes only in the
zero-margin regime", which is the property being guarded.

Scenario: `test/e2e/configs/repro_raybuffer_overflow_slot_crash.json` is the
single-MS-layer scene from the original report (8 crystal definitions, 7 filter
definitions, one scattering layer at prob 0, max_hits=8), reduced only in ray
budget so the case stays in the fast pool. Note max_hits=8 is BELOW
`RaypathRecorder::kInlineCap == 15`, so this scene legitimately never needs an
overflow slot at all — every `HasOverflow()` seen here is corrupted state, which
is what separates this defect from the max_hits>15 one guarded by
test_max_hits_crash.py.

CPU-only by design: the crash lives in `Simulator::SimulateOneWavelength`, the
legacy CPU trace loop. The GPU backends do not run that loop (they dispatch
through `SimulateOneWavelengthWithBackend`) and have no `buffer_data[]` pair, so
there is no device-side analogue of this buffer-pair contract to cover.

Runs fast (~1s per case) so it stays in the default `pytest -v` PR gate.
"""

from __future__ import annotations

import os
import subprocess

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root

# Dispatch sizes at or below Simulator::kSmallBatchRayNum (32) — the zero-margin
# regime where the defect fired — plus one control above it.
CRASHING_DISPATCH_SIZES = [8, 16, 24, 32]
CONTROL_DISPATCH_SIZE = 48


def _run_with_dispatch_ray_num(dispatch_ray_num: int) -> subprocess.CompletedProcess:
    binary = find_lumice_binary()
    cfg = get_project_root() / "test" / "e2e" / "configs" / "repro_raybuffer_overflow_slot_crash.json"
    env = os.environ.copy()
    env["LUMICE_DISPATCH_RAY_NUM"] = str(dispatch_ray_num)
    return subprocess.run(
        [str(binary), "-f", str(cfg)],
        capture_output=True,
        text=True,
        timeout=300,
        env=env,
    )


@pytest.mark.parametrize("dispatch_ray_num", CRASHING_DISPATCH_SIZES + [CONTROL_DISPATCH_SIZE])
def test_small_dispatch_ray_num_no_crash(dispatch_ray_num: int) -> None:
    """A small dispatch batch must not overflow the hit-loop buffers.

    Pre-fix these exited by signal (SIGSEGV / SIGABRT / SIGTRAP, i.e. a negative
    or >128 return code depending on platform and shell); the control size
    exited 0 both before and after.
    """
    result = _run_with_dispatch_ray_num(dispatch_ray_num)
    assert result.returncode == 0, (
        f"Lumice exited {result.returncode} for LUMICE_DISPATCH_RAY_NUM={dispatch_ray_num} "
        f"(non-zero here means the hit-loop buffer-pair capacity contract regressed and the "
        f"process died by signal)\nstderr:\n{result.stderr}"
    )
    # The FatalAbort gates added with the fix (Simulator's TraceRayBasicInfo
    # fan-out gate and RayBuffer::DupOverflowSlot's ownership gate) are the
    # Release-judgable replacements for what used to be comment-only
    # preconditions. A clean exit code already implies neither fired, but assert
    # on the text too so a future change that downgrades either gate to a
    # non-fatal warning still fails here.
    assert "FATAL:" not in result.stderr, (
        f"A fatal invariant gate fired for LUMICE_DISPATCH_RAY_NUM={dispatch_ray_num}"
        f"\nstderr:\n{result.stderr}"
    )
