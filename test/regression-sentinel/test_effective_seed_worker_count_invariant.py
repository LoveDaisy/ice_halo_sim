"""Gate: a fixed `sim_seed` collapses the CPU route to ONE worker.

`ServerImpl::ServerImpl` (src/server/server.cpp) clamps `worker_count` to 1
whenever `sim_seed != 0`. That rule exists for the deterministic-CPU contract,
and the raypath histogram consumer leans on it for something else: every
`SimData` is tagged with its Simulator's `effective_seed_`
(`SimData::producer_effective_seed_`), and `ChainIdMerger` keys each worker's
chain-id remap on that tag. A fixed seed is returned as the effective seed
verbatim (test_chain_id_merger.cpp pins that), so two live workers under one
fixed seed would share a tag and the merger would silently fuse their chains
— not a crash, wrong numbers. The clamp is what makes that state unreachable.

This file is the witness that the clamp still holds, from the outside: request
several workers AND a fixed seed, read the pool size the server logs, assert
it is 1. Relaxing the clamp is legitimate, but then every worker's seed has to
be made distinct in the same change — this test going red is the reminder.

Marked `@pytest.mark.slow`: drives the C API through the shared library
(`./scripts/build.sh -sj release`), like every other capi_runner-based test.
Run it alone with `pytest <this file> -v -m ''` — pyproject.toml's `addopts`
excludes `slow` from a bare `pytest`.
"""

from __future__ import annotations

import re

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e.runner import get_project_root


_CFG = get_project_root() / "test" / "e2e" / "configs" / "crystal_sample_count_deterministic.json"
_TIMEOUT = 240
_FIXED_SEED = 20260911
_WORKERS_REQUESTED = 4

# Same line test_crystal_count_dispatch_invariance.py reads; a private copy
# because that file is a test module, not a helper package.
_RE_WORKER_COUNT = re.compile(r"ServerImpl: gpu_route=\w+ worker_count=(\d+)")


def _worker_count(result: BufferedSimResult) -> int:
    """Workers the server actually spawned (0 if the line never logged)."""
    counts = [int(m.group(1)) for ln in result.log_lines
              for m in [_RE_WORKER_COUNT.search(ln)] if m]
    return max(counts) if counts else 0


def _run(sim_seed: int, num_workers: int) -> BufferedSimResult:
    result = run_scene_capi_buffered(
        str(_CFG),
        sim_seed=sim_seed,
        backend="legacy",
        timeout_sec=_TIMEOUT,
        num_workers=num_workers,
    )
    assert result.has_valid_data, f"{_CFG.name}: no valid data"
    assert result.routed_backend == "legacy" and not result.fell_back, (
        f"asked for 'legacy' but ran {result.routed_backend!r} "
        f"(fell_back={result.fell_back})"
    )
    return result


@pytest.mark.slow
def test_fixed_seed_collapses_the_cpu_route_to_one_worker() -> None:
    # Positive control first: the same request WITHOUT a seed must actually
    # spawn the pool asked for, otherwise "1 worker" below would say nothing
    # about the seed clamp (it could be the request being ignored).
    free = _run(sim_seed=0, num_workers=_WORKERS_REQUESTED)
    w_free = _worker_count(free)
    assert w_free == _WORKERS_REQUESTED, (
        f"asked for {_WORKERS_REQUESTED} workers at sim_seed=0 but the server "
        f"spawned {w_free} (from the 'ServerImpl: ... worker_count=' log line); "
        f"the clamp assertion below would be vacuous"
    )

    pinned = _run(sim_seed=_FIXED_SEED, num_workers=_WORKERS_REQUESTED)
    w_pinned = _worker_count(pinned)
    assert w_pinned == 1, (
        f"sim_seed={_FIXED_SEED} with {_WORKERS_REQUESTED} workers requested "
        f"spawned {w_pinned} workers. ServerImpl no longer clamps a fixed seed "
        f"to one worker — every worker now shares one effective seed, so "
        f"SimData::producer_effective_seed_ no longer tells workers apart and "
        f"ChainIdMerger fuses their chain ids. Either restore the clamp or "
        f"give each worker a distinct seed in the same change"
    )
