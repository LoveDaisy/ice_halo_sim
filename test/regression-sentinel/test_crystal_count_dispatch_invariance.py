"""Gate: `LUMICE_StatsResult.crystal_num` is a scene property, not a schedule artifact.

`crystal_num` (CLI `Stats: crystals=N`) is defined as **how many distinct crystal
geometries this run actually sampled**. Two consequences follow directly from
that definition, and this file gates both:

1. **Dispatch invariance.** `LUMICE_DISPATCH_RAY_NUM` is a per-SimBatch
   scheduling grain — a performance knob. Sweeping it must not move
   `crystal_num`, because it does not change which geometries the scene
   contains. Historically it did: the counter was a per-batch quantity summed
   by StatsConsumer, so it reported "how many crystal objects were materialised
   across all batches" and swung by two orders of magnitude with the knob alone.

2. **Randomization observability.** A deterministic-shape scene samples each
   (layer, ci) geometry exactly once, so `crystal_num` must equal the scene's
   (layer, ci) count. A scene with stochastic shape distributions draws a fresh
   geometry per ray-group, so it must report substantially more. Historically
   the deterministic and stochastic twins of the same scene reported the SAME
   number on the `cpu_backend` arm — the stat was blind to the one question it
   is meant to answer.

3. **Worker invariance.** The CPU route runs `PhysicalCoreCount()` workers by
   default, each owning its own Simulator and therefore its own geometry
   sampling. A deterministic scene's geometry set does not depend on how many
   threads redundantly derive it, so `crystal_num` must not scale with the pool
   size. Historically it did — exactly `(layer, ci) count x workers` — which is
   the same defect as (1) with a different knob, and it hid from the first two
   tests because pinning `sim_seed` collapses the CPU route to one worker.

Both arms covered here are pure-CPU (`legacy` = the default in-simulator trace
path, `cpu_backend` = the `CpuTraceBackend` seam implementation), so they run
anywhere pytest runs. Metal/CUDA carry the same contract but are gated by their
own backend-level unit tests (`GetLastBatchStochasticCrystalSampleCount*` in
test/parity-cross-backend/backend/), which assert the per-batch half directly —
"a reuse batch reports 0" — without needing a GPU device in this file.

Why the first two tests pin `sim_seed != 0` and the third must NOT: a non-zero
seed collapses the CPU route to a single worker (server.cpp ServerImpl), which
removes one source of variance from tests (1) and (2) so their expected values
are exact. But it also removes the very axis test (3) exists to sweep, and the
shipped CLI never sets a seed — so test (3) runs at `sim_seed == 0` and pins
`num_workers` explicitly instead.

Marked `@pytest.mark.slow`: drives the C API through the shared library
(`./scripts/build.sh -sj release`), like every other capi_runner-based test.
"""

from __future__ import annotations

import contextlib
import json
import os
import re
from typing import Iterator, Optional

import pytest

from test.e2e.capi_runner import BufferedSimResult, run_scene_capi_buffered
from test.e2e.runner import get_project_root


_CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_DETERMINISTIC_CFG = _CONFIGS_DIR / "crystal_sample_count_deterministic.json"
_ZERO_PROPORTION_CFG = _CONFIGS_DIR / "crystal_sample_count_zero_proportion.json"
_RANDOM_CFG = _CONFIGS_DIR / "crystal_sample_count_random.json"

# Non-zero → single worker on the CPU route (see the module docstring).
_SIM_SEED = 20260727
_TIMEOUT = 240

# The two fixtures carry ray_num=20000. 128 is the shipped CPU default grain
# (157 batches); 20000 puts the whole run in ONE batch. If the counter were
# still per-batch-accumulated, these two would differ by ~157x — the widest
# lever available without changing the scene.
_DISPATCH_SMALL = 128
_DISPATCH_WHOLE_RUN = 20000

# Positive control: the batch count actually observed inside the run must track
# the requested grain, otherwise an env knob that silently stopped taking effect
# would leave this file green for the wrong reason (both arms equal because
# nothing varied). 4x is far below the ~157x the grains imply, and far above any
# incidental jitter.
_MIN_BATCH_COUNT_RATIO = 4.0

# Worker counts swept by the worker-invariance test. 1 is the reference; 4 is
# well above it yet available on any CI box (the default pool is
# PhysicalCoreCount(), so a hardcoded larger value would be untestable on small
# runners). Pre-fix these reported expected*1 vs expected*4.
_WORKERS_ONE = 1
_WORKERS_MANY = 4

_RE_CONSUME_PROFILE = re.compile(r"Consume profile: (\d+) batches")
_RE_WORKER_COUNT = re.compile(r"ServerImpl: gpu_route=\w+ worker_count=(\d+)")


def _layer_ci_total(config_path) -> int:
    """Σ over MS layers of that layer's entry count — the (layer, ci) pair count.

    This is the number of distinct crystal geometries a deterministic-shape
    scene samples: one per (layer, ci), reused for every ray-group afterwards.
    Derived from the fixture instead of hardcoded so editing the fixture cannot
    silently decouple the expectation from the scene.
    """
    with open(config_path) as f:
        cfg = json.load(f)
    return sum(len(layer["entries"]) for layer in cfg["scene"]["scattering"])


@contextlib.contextmanager
def _dispatch_grain(n: Optional[int]) -> Iterator[None]:
    """Pin LUMICE_DISPATCH_RAY_NUM (+ COMMIT, for the batch-count witness).

    COMMIT is pinned alongside DISPATCH so the "Consume profile: N batches"
    positive control stays a clean witness on the exit-seam arm too: leaving
    commit at its default would clamp the chunk count to ~exits/128 regardless
    of the dispatch grain and erase the ratio signal. `n is None` leaves both
    knobs untouched (shipped defaults).
    """
    keys = ("LUMICE_DISPATCH_RAY_NUM", "LUMICE_COMMIT_RAY_NUM")
    saved = {k: os.environ.get(k) for k in keys}
    try:
        if n is not None:
            for k in keys:
                os.environ[k] = str(n)
        yield
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def _consumed_batches(result: BufferedSimResult) -> int:
    """Batch count the RenderConsumer actually saw (0 if the line never logged)."""
    counts = [int(m.group(1)) for ln in result.log_lines
              for m in [_RE_CONSUME_PROFILE.search(ln)] if m]
    return max(counts) if counts else 0


def _worker_count(result: BufferedSimResult) -> int:
    """Workers the server actually spawned (0 if the line never logged)."""
    counts = [int(m.group(1)) for ln in result.log_lines
              for m in [_RE_WORKER_COUNT.search(ln)] if m]
    return max(counts) if counts else 0


def _run(config_path, backend: str, dispatch: Optional[int],
         sim_seed: int = _SIM_SEED, num_workers: int = 0) -> BufferedSimResult:
    with _dispatch_grain(dispatch):
        result = run_scene_capi_buffered(
            str(config_path),
            sim_seed=sim_seed,
            backend=backend,
            timeout_sec=_TIMEOUT,
            num_workers=num_workers,
            # The legacy arm normally has this knob stripped (it protects parity
            # oracles whose energy is not dispatch-invariant); here sweeping it
            # IS the measurement.
            preserve_dispatch_env=True,
        )
    assert result.has_valid_data, f"{config_path.name} @ {backend}: no valid data"
    assert result.routed_backend == backend and not result.fell_back, (
        f"{config_path.name}: asked for {backend!r} but ran "
        f"{result.routed_backend!r} (fell_back={result.fell_back}); the "
        f"measurement would be about the wrong backend"
    )
    return result


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_crystal_num_is_dispatch_invariant(backend: str) -> None:
    """Same scene, two dispatch grains → identical crystal_num."""
    small = _run(_DETERMINISTIC_CFG, backend, _DISPATCH_SMALL)
    whole = _run(_DETERMINISTIC_CFG, backend, _DISPATCH_WHOLE_RUN)

    # Positive control first: a no-op env knob would make the invariance
    # assertion below trivially true.
    n_small, n_whole = _consumed_batches(small), _consumed_batches(whole)
    assert n_small > 0 and n_whole > 0, (
        f"{backend}: no 'Consume profile: N batches' line captured "
        f"({n_small} / {n_whole}) — cannot confirm the dispatch grain took "
        f"effect, so the invariance assertion below would be unwitnessed"
    )
    assert n_small >= n_whole * _MIN_BATCH_COUNT_RATIO, (
        f"{backend}: dispatch grain did not take effect — batches consumed "
        f"{n_small} @ grain {_DISPATCH_SMALL} vs {n_whole} @ grain "
        f"{_DISPATCH_WHOLE_RUN}; expected at least {_MIN_BATCH_COUNT_RATIO}x. "
        f"Both runs traced the same schedule, so crystal_num equality proves "
        f"nothing."
    )

    assert small.crystal_num == whole.crystal_num, (
        f"{backend}: crystal_num moved with the dispatch grain — "
        f"{small.crystal_num} @ LUMICE_DISPATCH_RAY_NUM={_DISPATCH_SMALL} vs "
        f"{whole.crystal_num} @ {_DISPATCH_WHOLE_RUN}. crystal_num counts the "
        f"distinct crystal geometries the scene sampled; a per-SimBatch "
        f"scheduling knob cannot change that. A ratio close to "
        f"{n_small // max(n_whole, 1)}x means the counter is again summing a "
        f"per-batch quantity instead of counting new samples."
    )
    assert small.crystal_num > 0, (
        f"{backend}: crystal_num == 0 for a scene that traces rays through "
        f"{_layer_ci_total(_DETERMINISTIC_CFG)} crystal populations — equality "
        f"above would also hold for a counter stuck at zero"
    )


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_crystal_num_observes_shape_randomization(backend: str) -> None:
    """Deterministic scene → exactly the (layer, ci) count; stochastic twin → more.

    The two fixtures are structurally identical (same layers, same entries, same
    proportions, same ray_num); they differ only in whether `shape.height` is a
    scalar or a gaussian distribution. So any difference in crystal_num is
    attributable to shape randomization alone.
    """
    expected = _layer_ci_total(_DETERMINISTIC_CFG)

    fixed = _run(_DETERMINISTIC_CFG, backend, None)
    assert fixed.crystal_num == expected, (
        f"{backend}: deterministic scene reported crystal_num="
        f"{fixed.crystal_num}, expected {expected} = Σ over MS layers of that "
        f"layer's entry count. A deterministic shape is sampled once per "
        f"(layer, ci) and reused for every ray-group after that, so anything "
        f"larger means reuse is being counted as fresh sampling"
    )

    stochastic = _run(_RANDOM_CFG, backend, None)
    assert stochastic.crystal_num > fixed.crystal_num, (
        f"{backend}: stochastic-shape scene reported crystal_num="
        f"{stochastic.crystal_num}, no more than its deterministic twin's "
        f"{fixed.crystal_num}. Every ray-group draws a fresh geometry when the "
        f"shape carries a distribution, so this stat is blind to the one "
        f"question it exists to answer: did my randomization take effect?"
    )
    # Separation, not just strict inequality: the fixtures run 20000 rays, so a
    # working counter reports hundreds of samples, not expected+1.
    assert stochastic.crystal_num >= expected * 10, (
        f"{backend}: stochastic-shape scene reported only "
        f"{stochastic.crystal_num} distinct samples for 20000 rays across "
        f"{expected} crystal populations — expected at least {expected * 10}. "
        f"Too close to the deterministic value {fixed.crystal_num} to "
        f"distinguish 'randomization observed' from measurement noise"
    )


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_crystal_num_is_worker_invariant(backend: str) -> None:
    """Same scene, 1 vs N workers → identical crystal_num.

    Runs at `sim_seed == 0` on purpose: a pinned seed clamps the CPU route to a
    single worker, so a seeded version of this test would sweep nothing and pass
    against a counter that scales linearly with the pool. That is precisely how
    this defect survived the first two tests in this file.
    """
    expected = _layer_ci_total(_DETERMINISTIC_CFG)

    one = _run(_DETERMINISTIC_CFG, backend, None, sim_seed=0, num_workers=_WORKERS_ONE)
    many = _run(_DETERMINISTIC_CFG, backend, None, sim_seed=0, num_workers=_WORKERS_MANY)

    # Positive control: `num_workers` is a request, and the server is free to
    # clamp it (GPU route pins 1; a non-zero seed pins 1). If both runs ended up
    # with the same pool, equality below would be vacuous.
    w_one, w_many = _worker_count(one), _worker_count(many)
    assert w_one == _WORKERS_ONE and w_many == _WORKERS_MANY, (
        f"{backend}: asked for {_WORKERS_ONE} / {_WORKERS_MANY} workers but the "
        f"server spawned {w_one} / {w_many} (from the 'ServerImpl: ... "
        f"worker_count=' log line) — the pool size did not vary, so the "
        f"invariance assertion below would prove nothing"
    )

    assert one.crystal_num == many.crystal_num, (
        f"{backend}: crystal_num scaled with the worker pool — "
        f"{one.crystal_num} @ {_WORKERS_ONE} worker vs {many.crystal_num} @ "
        f"{_WORKERS_MANY}. Each worker owns its own Simulator and redundantly "
        f"derives the same deterministic geometries; crystal_num counts the "
        f"geometries the SCENE has, so how many threads re-derive them is not "
        f"its business. A ratio of ~{_WORKERS_MANY}x means the deterministic "
        f"population is being summed per worker instead of carried as the "
        f"config constant it is"
    )
    assert many.crystal_num == expected, (
        f"{backend}: {_WORKERS_MANY}-worker run reported crystal_num="
        f"{many.crystal_num}, expected {expected} = Σ over MS layers of that "
        f"layer's entry count. Equality with the 1-worker run above would also "
        f"hold if both were wrong by the same factor"
    )


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_crystal_num_counts_scene_slots_not_traced_slots(backend: str) -> None:
    """A crystal_proportion_ == 0 slot still counts toward the deterministic half.

    This pins the property that BUYS the immunity asserted by the two tests above,
    and it is the one a well-meaning change is most likely to break: counting only
    the (layer, ci) slots a run actually reached looks strictly more accurate, and
    it silently reintroduces exactly the defect this whole gate exists to catch.
    Which slots get traced is a function of the ray partition, so it moves with the
    dispatch grain and the worker pool — the deterministic half must be derived
    from the committed config alone, never from what a particular run touched.

    The fixture is the deterministic scene with its last entry's proportion set to
    0. That slot never reaches MakeCrystal on any backend, yet the reported count
    must still be the full slot total, identical to the un-zeroed scene.
    """
    expected = _layer_ci_total(_ZERO_PROPORTION_CFG)
    zeroed = _run(_ZERO_PROPORTION_CFG, backend, None)

    assert zeroed.crystal_num == expected, (
        f"{backend}: scene with a proportion=0 slot reported crystal_num="
        f"{zeroed.crystal_num}, expected {expected} = every (layer, ci) slot in "
        f"the committed config. Counting only slots the run actually traced makes "
        f"the value a function of the ray partition again, which is what the "
        f"dispatch- and worker-invariance tests above forbid"
    )
    assert zeroed.crystal_num == _layer_ci_total(_DETERMINISTIC_CFG), (
        f"{backend}: zeroing a slot's proportion changed crystal_num; the "
        f"deterministic half is a property of the committed config, and setting a "
        f"population's share to 0 does not remove it from the config"
    )
