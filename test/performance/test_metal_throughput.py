"""Metal-vs-legacy throughput regression gate (scrum-268 / G1).

The §5 single-engine arc's whole payoff is throughput: the landed Scrum 1
device-resident continuation engine still runs inside the 12-worker structure
and measures ~0.58x legacy on heavy multi-MS+filter scenes (C2 sanity at PR#128
merge). Scrum 2 (single big dispatch + async + commit<->batch decoupling) must
flip that to >= 1x (target 2-5x, explore-265).

This is the **D1 pre-registered throughput gate** (scrum.md §3/§4): the gate
threshold (Metal multi-worker rays/s >= legacy) is committed NOW, before the
implementation, so it cannot be quietly weakened later. Two assertion layers
keep CI honest without faking green:

  - **C2 sentinel (active, must pass today):** Metal actually runs (no fallback),
    does not hang, and is not catastrophically slower than legacy
    (ratio >= _SANITY_FLOOR). Guards against regressions/hangs on the landed
    engine right now.
  - **Throughput gate (pre-registered, xfail until Scrum 2 lands):** Metal multi
    >= legacy (ratio >= _GATE). Currently ~0.58x -> imperative xfail with the
    measured ratio in the reason. When Scrum 2 delivers, ratio crosses 1.0, the
    test stops xfailing and the gate is GREEN — visible in CI, not buried.

The printed ratio is also the explore-3 R1-occupancy input (scrum.md §6.1): if
Scrum 2's single engine regresses throughput > ~10% vs legacy, R1 option B
(split filter-gate into its own wavefront dispatch) triggers.

Baseline is legacy CPU (the GUI's real path); NEVER cpu_backend (a 2.5x-slow
auxiliary, see feedback_perf_baseline_is_legacy_cpu).

@pytest.mark.slow — needs the release binary; Darwin-only (Metal).
"""
import json
import os
import platform
import re
import subprocess

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root

pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin", reason="Metal backend is only available on macOS"
)

_CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# Heavy multi-MS + filter + multi-crystal scenes (MS2, complete fast, do NOT
# overflow the exit buffer — unlike ms3_mixed_pyramid_heavy which is a Scrum 2
# worst-case TARGET, not a gate input; see scrum.md §5).
_HEAVY_CONFIGS = [
    "ms_multi_crystal_complex_filter",
    "ms_multi_crystal_filtered_bd",
]

# C2 sentinel floor: ratio below this = catastrophic regression / near-hang.
# `rays_per_sec` is a SINGLE sample per run (no warm-up/median).
#
# WHICH `rate_basis` THIS GATE ACTUALLY CONSUMES, measured rather than assumed:
# the legacy arm reports `steady` on both configs, but the **Metal arm never
# does**. `_HEAVY_CONFIGS` carry ray_num=2,000,000, which is below the Metal
# drain quantum (2,097,152 = 64 batches * 32768 dispatch), so sim_ray_num is
# published exactly once and no steady window exists — 240 runs gave
# `wall_fallback` 96% / `active_short` 4%, `steady` zero times. The numerator is
# therefore a wall-clock rate that still contains whatever setup the warm-up pass
# did not absorb; measured ~13.0-13.5M vs the same scene's true steady rate of
# ~13.9M at ray_num=20M, i.e. ~5% conservative. That is a ratio DEflation, so it
# cannot manufacture a false green here — but the earlier `active_short` defect
# could and did: that basis used to divide by IDLE-detection latency and reported
# 266-390M rays/s on 4% of Metal runs, multiplying this ratio by ~20x with nothing
# wrong. Fixed in `RunBenchmarkPass` (main.cpp), guarded by
# test/regression-sentinel/test_benchmark_rate_not_impossible.py.
#
# THIS RATIO HAS A MOVING DENOMINATOR, and that has now bitten once. The floor
# was 3.0, derived from a measured nominal of ~8-10x (bench_throughput.py
# 2026-06-19: complex_filter 8.14x, filtered_bd 10.09x) — i.e. it sat at roughly
# 0.30-0.37x of nominal, leaving ~3x of headroom for the ~25% thermal CoV a
# single heavy-scene sample carries. Those nominals are now HISTORY: recycling
# the legacy CPU path's per-batch `all_data` buffer (SimWorkspace) made the
# DENOMINATOR ~2.2-2.5x faster on the same box, with the Metal numerator
# unchanged. Measured A/B on one machine, 3-5 reps per arm, arms interleaved:
#
#   scene                         legacy multi rays/s      Metal multi rays/s
#   ms_multi_crystal_complex_filter   1.93M -> 4.27M    12.8-19.5M (both arms)
#   ms_multi_crystal_filtered_bd      2.00M -> 4.99M    14.1-19.5M (both arms)
#
# so nominal fell to ~2.9-4.0x and the old 3.0 floor landed ON TOP of the
# distribution — it produced a red at ratio=2.889 with nothing wrong. Re-derived
# on the same reasoning as the original: floor = ~0.45x of the new nominal
# (~3.2x), which is 1.9x below the lowest sample actually observed (2.85), so a
# 25% thermal excursion cannot reach it, while still firing well before Metal
# degrades to "no faster than legacy" (_GATE = 1.0 below). The pre-registered D1
# gate itself is NOT touched. For a systematic median+CoV evaluation, use
# `scripts/bench_throughput.py`.
_SANITY_FLOOR = 1.5
_GATE = 1.0           # D1 pre-registered gate: Metal multi >= legacy (xfail until Scrum 2)
_TIMEOUT = 240        # --benchmark is bounded (poll-until-IDLE); guard against hangs

# Fallback detection tripwire: a Metal run on an incompatible lens/view logs
# "falling back" via ILOG_WARN (simulator.cpp:550/579/586/595) → spdlog `err`
# level → the shared STDOUT color sink (logger.hpp). At the benchmark's default
# INFO level the `err` message is still emitted. We scan BOTH stdout and stderr
# so a future sink/stream refactor can't silently disable the tripwire (which
# would let a degraded Metal run report a meaningless ratio). See code-review MINOR-1.
_RE_FALLBACK = re.compile(r"falling back", re.IGNORECASE)


def _run_benchmark(config_name: str, metal: bool) -> dict:
    """Run `Lumice --benchmark` on a config; return parsed multi-pass result.

    Returns {"multi_rps": float, "single_rps": float, "fell_back": bool}.
    """
    cfg = str(_CONFIGS_DIR / f"{config_name}.json")
    env = dict(os.environ)
    if metal:
        env["LUMICE_TRACE_BACKEND"] = "metal"
    else:
        env.pop("LUMICE_TRACE_BACKEND", None)  # legacy = unset (NOT cpu_backend)

    proc = subprocess.run(
        [str(find_lumice_binary()), "--benchmark", "-f", cfg, "-o", "/tmp"],
        capture_output=True, text=True, timeout=_TIMEOUT, env=env,
    )
    fell_back = bool(_RE_FALLBACK.search(proc.stderr) or _RE_FALLBACK.search(proc.stdout))
    multi_rps = single_rps = 0.0
    multi_basis = "?"
    for line in proc.stdout.splitlines():
        if "[BENCHMARK]" in line:
            data = json.loads(line.split("[BENCHMARK]", 1)[1].strip())
            if data.get("mode") == "multi":
                multi_rps = float(data["rays_per_sec"])
                # Reported, not asserted on: which basis each arm lands on is a
                # property of the config's ray_num vs the backend's drain quantum
                # (see the _SANITY_FLOOR comment), so pinning it would be pinning
                # the gate's inputs. Printing it makes a future drift visible in
                # the CI log instead of silent.
                multi_basis = str(data.get("rate_basis", "?"))
            elif data.get("mode") == "single":
                single_rps = float(data["rays_per_sec"])
    return {
        "multi_rps": multi_rps,
        "single_rps": single_rps,
        "fell_back": fell_back,
        "multi_basis": multi_basis,
    }


@pytest.mark.slow
@pytest.mark.parametrize("config_name", _HEAVY_CONFIGS)
def test_metal_throughput_gate(config_name):
    legacy = _run_benchmark(config_name, metal=False)
    metal = _run_benchmark(config_name, metal=True)

    assert not metal["fell_back"], (
        f"{config_name}: Metal fell back to legacy — backend requested but did not run; "
        f"throughput comparison is meaningless."
    )
    assert legacy["multi_rps"] > 0.0, f"{config_name}: legacy multi rps == 0 (benchmark parse failed?)"
    assert metal["multi_rps"] > 0.0, f"{config_name}: metal multi rps == 0 (benchmark parse failed?)"

    ratio = metal["multi_rps"] / legacy["multi_rps"]
    print(
        f"[throughput] {config_name}: legacy_multi={legacy['multi_rps']:.0f} "
        f"metal_multi={metal['multi_rps']:.0f} ratio={ratio:.3f} "
        f"(legacy_basis={legacy['multi_basis']}, metal_basis={metal['multi_basis']}; "
        f"sanity>={_SANITY_FLOOR}, gate>={_GATE})"
    )

    # --- C2 sentinel: active gate, must pass on the landed engine today --------
    assert ratio >= _SANITY_FLOOR, (
        f"{config_name}: catastrophic throughput regression ratio={ratio:.3f} < "
        f"{_SANITY_FLOOR} — Metal far slower than legacy or near-hung."
    )

    # --- D1 pre-registered throughput gate — FLIPPED GREEN at scrum-268.6 -------
    # Metal single-engine + backend-aware large default dispatch (32768) now beats
    # legacy N-worker on the heavy multi-MS+filter scenes (steady-rate re-measure
    # 2026-06-19: ~8-10x; the earlier "3.5-5.4x" was the setup-inflated --benchmark
    # reading, corrected in task-fix-throughput-bench-honesty). The §5 throughput
    # thesis is landed; this assertion keeps the pre-registered gate ACTIVE so any
    # future drop below parity is caught.
    assert ratio >= _GATE, (
        f"{config_name}: throughput gate regression ratio={ratio:.3f} < {_GATE} — "
        f"Metal single-engine no longer beats legacy N-worker (scrum-268 §5 thesis). "
        f"Check the backend-aware dispatch default (server.cpp kDefaultMetalDispatchRayNum)."
    )
