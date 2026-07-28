"""Gate: `LUMICE_StatsResult.orientation_num` is a scene property, not a schedule artifact.

`orientation_num` (CLI `Stats: orientations=N`) is defined as **how many crystal
orientations this run actually sampled**. It is the sibling of `crystal_num` and
carries the same two-term contract — a config-constant deterministic half plus an
accumulated stochastic half — so it inherits the same three obligations, gated
here against the same two pure-CPU arms as
`test_crystal_count_dispatch_invariance.py`:

1. **Dispatch invariance.** `LUMICE_DISPATCH_RAY_NUM` is a scheduling grain.
   Sweeping it must not move `orientation_num`.

2. **Randomization observability.** An all-deterministic-axis scene samples no
   orientation at all, so `orientation_num` must equal the scene's (layer, ci)
   count. Its random-axis twin redraws per ray and must report vastly more.

3. **Worker invariance.** The deterministic half must not scale with the pool.

What this file gates that its crystal-side sibling structurally CANNOT: the two
statistics come apart. Every fixture here holds `shape.height` at a fixed scalar,
so on the shape axis all three scenes are identical and `crystal_num` is pinned at
the (layer, ci) count throughout. Any variation observed below is attributable to
the axis alone. `test_orientation_and_crystal_counts_are_independent` makes that
explicit, because it is the whole reason this statistic exists: the predicate
behind `crystal_num` (`IsDeterministic(CrystalParam)`) looks only at shape params,
so the commonest halo setup — a fixed shape under a random axis — reports one
geometry no matter how richly the orientation was sampled. Wiring the orientation
count to the shape predicate would leave every other test in this file green.

Note the counts are per RAY, not per ray-group: orientations are redrawn for every
ray even when the geometry it is paired with came from cache, so the stochastic
figures here are ~ray_num-scale rather than the ~ray_num/32 the crystal side sees.
That asymmetry is the reuse difference, reported honestly.

One consequence shapes the fixture choices below and is easy to rediscover the
hard way: on a MULTI-layer scene the stochastic total is itself a random
variable. How many rays survive a layer's `prob` roll depends on the RNG, and
re-partitioning the run into a different number of batches changes the stream
partitioning, so the layer-1 population — and with it the total — moves by ~1%
across dispatch grains (measured: 69154 vs 69787). That is the scene being
stochastic, not the counter tracking the schedule. Exact equality is therefore
only assertable on the single-layer fixture, which is what the dispatch-invariance
test uses; the multi-layer fixture carries the ordering and scale claims instead.

Seed discipline is inherited deliberately and must NOT be "tidied": the first two
tests pin `sim_seed != 0`, which collapses the CPU route to one worker and makes
their expectations exact; the worker-invariance test must run at `sim_seed == 0`,
because a pinned seed removes the very axis it exists to sweep. That is how the
crystal-side defect survived its own first two tests.

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
_DETERMINISTIC_CFG = _CONFIGS_DIR / "orientation_sample_count_deterministic.json"
_ZERO_PROPORTION_CFG = _CONFIGS_DIR / "orientation_sample_count_zero_proportion.json"
_RANDOM_CFG = _CONFIGS_DIR / "orientation_sample_count_random.json"
# Single-MS-layer twin of _RANDOM_CFG. Needed by the dispatch-invariance test —
# see its docstring for why the multi-layer scene cannot carry that assertion.
_SINGLE_LAYER_RANDOM_CFG = _CONFIGS_DIR / "orientation_sample_count_single_layer_random.json"

# Non-zero → single worker on the CPU route (see the module docstring).
_SIM_SEED = 20260728
_TIMEOUT = 240

# The fixtures carry ray_num=20000. 128 is the shipped CPU default grain;
# 20000 puts the whole run in ONE batch.
_DISPATCH_SMALL = 128
_DISPATCH_WHOLE_RUN = 20000

# Positive control on the dispatch knob actually taking effect.
_MIN_BATCH_COUNT_RATIO = 4.0

_WORKERS_ONE = 1
_WORKERS_MANY = 4

_RE_CONSUME_PROFILE = re.compile(r"Consume profile: (\d+) batches")
_RE_WORKER_COUNT = re.compile(r"ServerImpl: gpu_route=\w+ worker_count=(\d+)")


def _layer_ci_total(config_path) -> int:
    """Σ over MS layers of that layer's entry count — the (layer, ci) pair count.

    For an all-deterministic-axis scene this is the whole reported orientation
    count: no slot draws, so the config-constant half is the entire answer.
    Derived from the fixture rather than hardcoded so editing the fixture cannot
    silently decouple the expectation from the scene.
    """
    with open(config_path) as f:
        cfg = json.load(f)
    return sum(len(layer["entries"]) for layer in cfg["scene"]["scattering"])


def _ray_num(config_path) -> int:
    with open(config_path) as f:
        return int(json.load(f)["scene"]["ray_num"])


@contextlib.contextmanager
def _dispatch_grain(n: Optional[int]) -> Iterator[None]:
    """Pin LUMICE_DISPATCH_RAY_NUM (+ COMMIT, for the batch-count witness).

    Same rationale as the crystal-count gate: leaving commit at its default
    clamps the chunk count regardless of the dispatch grain and erases the ratio
    signal the positive control depends on.
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
def test_orientation_num_is_dispatch_invariant(backend: str) -> None:
    """Same scene, two dispatch grains → orientation_num == ray_num exactly, both times.

    Three fixture choices here are load-bearing, and each rules out a way this
    test could look right while proving nothing:

    * RANDOM axes, not deterministic ones. On a deterministic-axis scene the
      stochastic half is identically zero and the whole reported value is a
      config constant, so dispatch invariance would hold even for a counter that
      summed per batch — there would be nothing to sum.

    * SINGLE MS layer. On the multi-layer fixture the count is genuinely NOT
      grain-invariant, and that is a property of the scene rather than a defect
      in the counter: how many rays survive layer 0's `prob` roll is a random
      variable, and re-partitioning 20000 rays into 157 batches instead of 1
      changes the RNG stream partitioning, hence the layer-1 population. Measured
      spread across grains is ~1% (e.g. 69154 vs 69787) — the honest invariant
      there is distributional, and asserting equality would produce a test that
      fails for a true reason. With one layer and `prob: 0.0` there is no
      survivor term, so the count is exact.

    * An EXACT expected value (ray_num), not just equality between the two runs.
      Every ray draws exactly one orientation and the run traces ray_num rays at
      any grain, so the answer is pinned on both sides independently. Equality
      alone would also hold if both runs were wrong by the same factor — which is
      exactly what the pre-fix per-batch-summed counter did to `crystal_num`.
    """
    expected = _ray_num(_SINGLE_LAYER_RANDOM_CFG)

    small = _run(_SINGLE_LAYER_RANDOM_CFG, backend, _DISPATCH_SMALL)
    whole = _run(_SINGLE_LAYER_RANDOM_CFG, backend, _DISPATCH_WHOLE_RUN)

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
        f"Both runs traced the same schedule, so orientation_num equality "
        f"proves nothing."
    )

    assert small.orientation_num == whole.orientation_num, (
        f"{backend}: orientation_num moved with the dispatch grain — "
        f"{small.orientation_num} @ LUMICE_DISPATCH_RAY_NUM={_DISPATCH_SMALL} "
        f"vs {whole.orientation_num} @ {_DISPATCH_WHOLE_RUN}. Both runs trace "
        f"the same {expected} rays and each ray draws one orientation; only the "
        f"grain they are grouped into changed."
    )
    assert small.orientation_num == expected, (
        f"{backend}: single-layer random-axis run over {expected} rays reported "
        f"orientation_num={small.orientation_num}. Every ray draws exactly one "
        f"orientation and none are reused, so the answer is pinned at "
        f"{expected}. A multiple of it means a per-batch quantity is being "
        f"summed; a fraction means some population is going uncounted"
    )


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_orientation_num_observes_axis_randomization(backend: str) -> None:
    """Deterministic-axis scene → exactly the (layer, ci) count; random twin → far more.

    The two fixtures are structurally identical (same layers, entries,
    proportions, ray_num, and the same FIXED shape on every crystal); they differ
    only in whether the axis distributions are scalars or distributions. So any
    difference in orientation_num is attributable to axis randomization alone —
    the fixtures' generator asserts that field-level equality directly.
    """
    expected = _layer_ci_total(_DETERMINISTIC_CFG)
    ray_num = _ray_num(_RANDOM_CFG)

    fixed = _run(_DETERMINISTIC_CFG, backend, None)
    assert fixed.orientation_num == expected, (
        f"{backend}: deterministic-axis scene reported orientation_num="
        f"{fixed.orientation_num}, expected {expected} = Σ over MS layers of "
        f"that layer's entry count. An all-kNoRandom axis consumes no RNG and "
        f"yields one fixed rotation for every ray, so anything larger means "
        f"rays that reused a single orientation are being counted as draws"
    )

    stochastic = _run(_RANDOM_CFG, backend, None)
    assert stochastic.orientation_num > fixed.orientation_num, (
        f"{backend}: random-axis scene reported orientation_num="
        f"{stochastic.orientation_num}, no more than its deterministic twin's "
        f"{fixed.orientation_num}. This stat exists to answer 'did my "
        f"orientation randomization take effect?' and would be blind to it"
    )
    # Scale, not just ordering. Orientations are per-ray with NO reuse, so the
    # first MS layer alone must account for ray_num draws; anything materially
    # below that means whole populations are going uncounted (e.g. only the
    # first MS layer wired up, or a per-ray-group count sneaking in).
    assert stochastic.orientation_num >= ray_num, (
        f"{backend}: random-axis scene reported only "
        f"{stochastic.orientation_num} orientation draws for a {ray_num}-ray "
        f"run whose every slot randomizes its axis. Orientation is resampled "
        f"per ray with no ray-group reuse, so the first MS layer alone owes "
        f"{ray_num}; a smaller total means some population or some MS layer is "
        f"not being counted"
    )


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_orientation_num_is_worker_invariant(backend: str) -> None:
    """Same scene, 1 vs N workers → identical orientation_num.

    Runs at `sim_seed == 0` on purpose: a pinned seed clamps the CPU route to a
    single worker, so a seeded version of this test would sweep nothing and pass
    against a counter that scales linearly with the pool. Do not "stabilise"
    this test by pinning the seed — that reintroduces the blind spot the
    crystal-side gate was written to close.
    """
    expected = _layer_ci_total(_DETERMINISTIC_CFG)

    one = _run(_DETERMINISTIC_CFG, backend, None, sim_seed=0, num_workers=_WORKERS_ONE)
    many = _run(_DETERMINISTIC_CFG, backend, None, sim_seed=0, num_workers=_WORKERS_MANY)

    # Positive control: `num_workers` is a request, and the server is free to
    # clamp it. If both runs ended up with the same pool, equality is vacuous.
    w_one, w_many = _worker_count(one), _worker_count(many)
    assert w_one == _WORKERS_ONE and w_many == _WORKERS_MANY, (
        f"{backend}: asked for {_WORKERS_ONE} / {_WORKERS_MANY} workers but the "
        f"server spawned {w_one} / {w_many} (from the 'ServerImpl: ... "
        f"worker_count=' log line) — the pool size did not vary, so the "
        f"invariance assertion below would prove nothing"
    )

    assert one.orientation_num == many.orientation_num, (
        f"{backend}: orientation_num scaled with the worker pool — "
        f"{one.orientation_num} @ {_WORKERS_ONE} worker vs "
        f"{many.orientation_num} @ {_WORKERS_MANY}. Every worker redundantly "
        f"derives the same deterministic axes; a ratio of ~{_WORKERS_MANY}x "
        f"means the deterministic half is being summed per worker instead of "
        f"carried as the config constant it is"
    )
    assert many.orientation_num == expected, (
        f"{backend}: {_WORKERS_MANY}-worker run reported orientation_num="
        f"{many.orientation_num}, expected {expected}. Equality with the "
        f"1-worker run above would also hold if both were wrong by the same "
        f"factor"
    )


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_orientation_num_counts_scene_slots_not_traced_slots(backend: str) -> None:
    """A crystal_proportion_ == 0 slot still counts toward the deterministic half.

    This pins the property that BUYS the immunity asserted above, and it is the
    one a well-meaning change is most likely to break: counting only the
    (layer, ci) slots a run actually reached looks strictly more accurate, and it
    silently reintroduces the defect the whole gate exists to catch. Which slots
    get traced is a function of the ray partition, so it moves with the dispatch
    grain and the worker pool.
    """
    expected = _layer_ci_total(_ZERO_PROPORTION_CFG)
    zeroed = _run(_ZERO_PROPORTION_CFG, backend, None)

    assert zeroed.orientation_num == expected, (
        f"{backend}: scene with a proportion=0 slot reported orientation_num="
        f"{zeroed.orientation_num}, expected {expected} = every (layer, ci) slot "
        f"in the committed config. Counting only slots the run actually traced "
        f"makes the value a function of the ray partition again"
    )
    assert zeroed.orientation_num == _layer_ci_total(_DETERMINISTIC_CFG), (
        f"{backend}: zeroing a slot's proportion changed orientation_num; the "
        f"deterministic half is a property of the committed config, and setting "
        f"a population's share to 0 does not remove it from the config"
    )


@pytest.mark.slow
@pytest.mark.parametrize("backend", ["legacy", "cpu_backend"])
def test_orientation_and_crystal_counts_are_independent(backend: str) -> None:
    """The two statistics come apart on the scene that motivated this one.

    Every fixture in this file holds `shape.height` at a fixed scalar, so on the
    shape axis all of them are the same deterministic scene and `crystal_num` is
    pinned at the (layer, ci) count. Swapping the axes from scalars to
    distributions must therefore move `orientation_num` by orders of magnitude
    while leaving `crystal_num` untouched.

    This is the assertion that fails if anyone routes the orientation count
    through `IsDeterministic(CrystalParam)` — the shape predicate — instead of
    `AxisDistribution::IsAxisDeterministic`. Every other test in this file would
    stay green under that mistake: they compare orientation counts to each other
    and to the slot total, and the shape predicate reports "deterministic" for
    both fixtures, collapsing the random one's stochastic half to zero in a way
    that only shows up beside the crystal count.
    """
    slots = _layer_ci_total(_DETERMINISTIC_CFG)

    fixed = _run(_DETERMINISTIC_CFG, backend, None)
    random = _run(_RANDOM_CFG, backend, None)

    assert fixed.crystal_num == slots and random.crystal_num == slots, (
        f"{backend}: crystal_num was expected to stay at the slot total {slots} "
        f"across both fixtures (every shape is a fixed scalar in both), but got "
        f"{fixed.crystal_num} / {random.crystal_num}. The fixtures have drifted "
        f"on the shape axis, so the independence claim below is confounded"
    )
    assert random.orientation_num > random.crystal_num * 100, (
        f"{backend}: fixed shapes under random axes reported crystal_num="
        f"{random.crystal_num} and orientation_num={random.orientation_num}. "
        f"The orientation count is supposed to dwarf the geometry count on this "
        f"scene — that gap IS the sampling richness the geometry count cannot "
        f"see. A value in the same range as crystal_num means the orientation "
        f"count is being judged by the shape predicate"
    )
