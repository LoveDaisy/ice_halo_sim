"""Regression guard: `--benchmark` must never report a rate the run had no time for.

Fix: `RunBenchmarkPass` (`src/main.cpp`) — the `rate_basis=active_short` branch
used to compute `rays_per_sec = r_end / active_sec`.

Root cause (diagnosed white-box by tracing `(t, sim_ray_num, server_state)` at
every poll): on a GPU backend `sim_ray_num` advances in whole drain quanta
(`kDefaultXyzDrainBatches` * dispatch_size = 64 * 32768 = 2,097,152 rays with the
Metal default). A config whose entire `ray_num` is below one quantum reads 0 at
every poll and then publishes once, at the very end, so the run yields exactly ONE
observation and there is no window to measure. `active_short` is precisely the
branch that detects this (`r_end == rays_at_active_start`), yet it divided by
`active_sec` — which in that state is the gap between the poll that saw the
publish and the poll that saw IDLE, i.e. IDLE-detection latency. The published
rate was therefore `ray_num / (k * poll_interval)`, unbounded above as k -> 1:
measured 266-390 M rays/s against the same scene's true 13.9 M rays/s.

Why it needed a guard rather than just a fix: the error is one-directional
(always upward) and the only single-sample consumer, `test_metal_throughput.py`,
compares the Metal rate against a FLOOR. A noise source that can only turn a red
green produces no complaints, so nothing else in the tree would report it.

The invariant asserted here is deliberately basis-agnostic and physical rather
than a re-statement of the formula: **the rays the estimator claims per second,
multiplied by the whole run's wall clock, cannot exceed the rays the run actually
traced by more than a small factor.** Every honest basis lands near 1.0 (measured:
`steady` 0.99-1.0 on legacy CPU and on Metal at 20M/200M rays; `wall_fallback`
and `active_short` are exactly 1.0 by construction), while the defect scored ~20.
`_MAX_WORK_RATIO` sits between them with room on both sides.

Scope, stated rather than implied: this is macOS + Metal only, because the defect
needs a drain quantum coarse enough to swallow a whole run — legacy CPU publishes
per batch, and at ray_num=200,000 it still reports `steady`, so it cannot reach
the branch. CUDA is untested here (no local device); its quantum differs.

Detection is probabilistic in REACHING the branch, not in judging it: which basis
a run lands on is a race, measured at 4-8% `active_short` over 220 runs, so
`_RUNS` is sized to make a miss unlikely rather than impossible. The assertion
itself is evaluated on every sample and cannot false-red — the test prints how
many `active_short` samples it actually saw so a vacuous pass is visible.

@pytest.mark.slow — needs the release binary and ~25 repeated benchmark runs;
sits beside the other perf-measurement gate (test/performance/) rather than in
the fast leg.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root
from test.performance.test_metal_throughput import _HEAVY_CONFIGS

pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin", reason="the coarse-drain-quantum path needs Metal (macOS)"
)

_CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# Below the 2,097,152-ray Metal drain quantum, which is what makes the window
# degenerate. Reuses the throughput gate's own `_HEAVY_CONFIGS` (rather than a
# second hardcoded list) so the two tests cannot silently drift apart on what
# they are measuring.
_CONFIGS = _HEAVY_CONFIGS

# P(no active_short sample in _RUNS) = 0.96**25 ~ 36% at the low end of the
# measured 4-8% rate, per config; with two configs a total miss is ~13%. Each run
# is ~0.25s, so the whole test is ~13s.
_RUNS = 25

# Honest bases score ~1.0; the defect scored ~20. See the module docstring.
_MAX_WORK_RATIO = 3.0

_TIMEOUT = 120


def _benchmark_rows(config_name: str, out_dir: str) -> list[dict]:
    """One `--benchmark` invocation on Metal; return its parsed [BENCHMARK] rows."""
    env = dict(os.environ)
    env["LUMICE_TRACE_BACKEND"] = "metal"
    proc = subprocess.run(
        [
            str(find_lumice_binary()),
            "--benchmark",
            "-f",
            str(_CONFIGS_DIR / f"{config_name}.json"),
            "-o",
            out_dir,
        ],
        capture_output=True,
        text=True,
        timeout=_TIMEOUT,
        env=env,
    )
    assert proc.returncode == 0, f"{config_name}: benchmark exited {proc.returncode}\n{proc.stderr}"
    return [
        json.loads(line.split("[BENCHMARK]", 1)[1].strip())
        for line in proc.stdout.splitlines()
        if "[BENCHMARK]" in line
    ]


@pytest.mark.slow
@pytest.mark.parametrize("config_name", _CONFIGS)
def test_reported_rate_is_physically_possible(config_name: str, tmp_path) -> None:
    seen_basis: dict[str, int] = {}
    worst = None  # (work_ratio, row) over every sample, reported either way.

    for _ in range(_RUNS):
        for row in _benchmark_rows(config_name, str(tmp_path)):
            basis = str(row.get("rate_basis", "?"))
            seen_basis[basis] = seen_basis.get(basis, 0) + 1
            wall_sec = float(row["wall_sec"])
            if wall_sec <= 0.0:
                continue  # rounded-away wall clock carries no bound; not a failure
            work_ratio = float(row["rays_per_sec"]) * wall_sec / float(row["rays"])
            if worst is None or work_ratio > worst[0]:
                worst = (work_ratio, row)
            # Reported inside the loop so the failing SAMPLE is in the message,
            # and so a red stops here instead of driving 24 more runs against an
            # already-established failure.
            assert work_ratio <= _MAX_WORK_RATIO, (
                f"{config_name}: [BENCHMARK] claims {row['rays_per_sec']:,.0f} rays/s over a "
                f"{wall_sec}s run that traced only {row['rays']:,} rays — that is "
                f"{work_ratio:.1f}x more work than the run had time for "
                f"(limit {_MAX_WORK_RATIO}). rate_basis={basis}, active_sec={row.get('active_sec')}. "
                f"The estimator is dividing by something that is not a trace duration."
            )

    assert worst is not None, f"{config_name}: no usable [BENCHMARK] sample in {_RUNS} runs"
    # Vacuity is visible rather than silent: if `active_short` never came up, the
    # branch the guard exists for was not exercised this time.
    print(
        f"[rate-sanity] {config_name}: {_RUNS} runs, bases={seen_basis}, "
        f"worst work_ratio={worst[0]:.2f} (limit {_MAX_WORK_RATIO})"
    )
