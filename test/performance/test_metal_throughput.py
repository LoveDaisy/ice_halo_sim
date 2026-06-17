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
# `rays_per_sec` is a SINGLE wall-clock sample per run (main.cpp:116-134, no
# warm-up/median), so the ratio is a noisy product of two single samples.
# After scrum-268's single-engine large-dispatch landing, Metal(default) is
# consistently 3.5-5.5x legacy on the heavy multi-MS+filter scenes (scrum-268.6
# sweep + task-270.6 smoke test at 5.52x). A floor of 2.0x is conservative but
# meaningful: it trips on a true engine collapse or near-hang while tolerating
# per-run thermal noise (3.5x floor × thermal variance still >> 2.0x). For a
# systematic median+CoV evaluation, use `scripts/bench_throughput.py`.
_SANITY_FLOOR = 2.0
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
    for line in proc.stdout.splitlines():
        if "[BENCHMARK]" in line:
            data = json.loads(line.split("[BENCHMARK]", 1)[1].strip())
            if data.get("mode") == "multi":
                multi_rps = float(data["rays_per_sec"])
            elif data.get("mode") == "single":
                single_rps = float(data["rays_per_sec"])
    return {"multi_rps": multi_rps, "single_rps": single_rps, "fell_back": fell_back}


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
        f"(sanity>={_SANITY_FLOOR}, gate>={_GATE})"
    )

    # --- C2 sentinel: active gate, must pass on the landed engine today --------
    assert ratio >= _SANITY_FLOOR, (
        f"{config_name}: catastrophic throughput regression ratio={ratio:.3f} < "
        f"{_SANITY_FLOOR} — Metal far slower than legacy or near-hung."
    )

    # --- D1 pre-registered throughput gate — FLIPPED GREEN at scrum-268.6 -------
    # Metal single-engine + backend-aware large default dispatch (32768) now beats
    # legacy N-worker on the heavy multi-MS+filter scenes (sweep 2026-06-16:
    # ~3.5–5.4x). The §5 throughput thesis is landed; this assertion makes the
    # pre-registered gate ACTIVE so any future drop below parity is caught in CI.
    assert ratio >= _GATE, (
        f"{config_name}: throughput gate regression ratio={ratio:.3f} < {_GATE} — "
        f"Metal single-engine no longer beats legacy N-worker (scrum-268 §5 thesis). "
        f"Check the backend-aware dispatch default (server.cpp kDefaultMetalDispatchRayNum)."
    )
