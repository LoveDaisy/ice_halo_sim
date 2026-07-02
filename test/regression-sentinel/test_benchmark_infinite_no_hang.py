"""Regression guard: `--benchmark` on an infinite-ray_num config must TERMINATE.

Fix commit: task-317 (benchmark reads sim_ray_num via the cheap O(1)
LUMICE_GetSimRayCount instead of the render-triggering LUMICE_GetStatsResults).

Root cause (diagnosed white-box on dev49 via live gdb stacks): the drain-count
`--benchmark` poll loop called LUMICE_GetStatsResults every iteration just to
read sim_ray_num, but that unconditionally triggers a full DoSnapshot ->
RenderConsumer sRGB (LinearToSrgbBatch, powf/pixel) render the benchmark never
uses. For the drain-count path (which runs many polls) the per-poll render tax
dominated wall-time and starved drain-window closure, so a pass that should
finish in <2s instead spun for tens of seconds to minutes (GPU 0%, one core at
100%). On CUDA the delayed closure let the unbounded infinite session run past
the 32-bit device PCG ray-index cap (~4.29e9) -> silent legacy fallback ->
effectively never terminated.

Trigger conditions:
- scene.ray_num == "infinite" (the GPU-bench drain-count-driven measurement path)
- `--benchmark` mode (the poll loop that read sim_ray_num every iteration)

Detective power / where the catastrophe lives: DoSnapshot is gated by
snapshot_dirty_, so the stray render fired ~once per drain (~N times), not per
poll. The render tax is therefore N * render_cost — CATASTROPHIC in the GPU
large-drain regime (CUDA drains are ~16.8M rays: the window spans ~1.3s of
trace during which the render-starved poll loop lets the unbounded session
cross the 32-bit device PCG cap -> legacy fallback -> never terminates; verified
pre-fix on dev49 4060Ti: 150s timeout, vs 1.6s post-fix). On the LEGACY CPU path
drains are tiny (window closes in ~N polls) so pre-fix is merely slower, not a
hard hang — there this test is a lighter smoke guard (terminates cleanly +
produces a drain_aligned result), and the strong regression detection is on the
GPU backends below. The poll loop itself is backend-agnostic, so all three
variants exercise the same fixed code path.

Runs fast (~1s when healthy) so it stays in the default `pytest -v` PR gate.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import tempfile
from pathlib import Path

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root

# Generous vs the healthy ~0.1-1.5s completion, tiny vs the pre-fix hang
# (tens of seconds to minutes / never). A TimeoutExpired here IS the regression.
_HANG_TIMEOUT_SEC = 30


def _infinite_config_path() -> str:
    cfg = json.loads(
        (get_project_root() / "test" / "e2e" / "configs" / "bench_light_single_ms.json").read_text()
    )
    cfg["scene"]["ray_num"] = "infinite"
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(cfg, f)
        return f.name


def _run_infinite_benchmark(backend_env: str | None = None) -> subprocess.CompletedProcess:
    binary = find_lumice_binary()
    cfg_path = _infinite_config_path()
    try:
        env = os.environ.copy()
        if backend_env is not None:
            env["LUMICE_TRACE_BACKEND"] = backend_env
        return subprocess.run(
            [str(binary), "--benchmark", "-f", cfg_path],
            capture_output=True,
            text=True,
            timeout=_HANG_TIMEOUT_SEC,
            env=env,
        )
    finally:
        Path(cfg_path).unlink(missing_ok=True)


def test_benchmark_infinite_terminates_cpu() -> None:
    """Legacy CPU: `--benchmark` on ray_num=infinite must terminate quickly with
    a drain_aligned result — not spin in the per-poll render (task-317)."""
    try:
        result = _run_infinite_benchmark()
    except subprocess.TimeoutExpired:
        pytest.fail(
            f"Lumice --benchmark (ray_num=infinite) did not terminate within "
            f"{_HANG_TIMEOUT_SEC}s — render-per-poll hang regression (task-317)."
        )

    # Distinguish crash from hang: a clean exit is required (feedback_check_exit_code).
    assert result.returncode == 0, (
        f"Lumice --benchmark exited {result.returncode} (non-zero = crash, not hang)\n"
        f"stderr:\n{result.stderr}"
    )
    # The drain-count-driven path must have actually run AND closed its window
    # (printed the result) — guards against a degenerate early exit masking a hang.
    assert "drain_aligned" in result.stdout, (
        f"Expected a drain_aligned [BENCHMARK] result for ray_num=infinite; "
        f"the drain-count window may not have closed.\nstdout:\n{result.stdout}"
    )


def test_benchmark_infinite_terminates_metal() -> None:
    """Native Metal: same guard on the device path (macOS only)."""
    if os.uname().sysname != "Darwin":
        pytest.skip("Metal backend is macOS-only")
    try:
        result = _run_infinite_benchmark(backend_env="metal")
    except subprocess.TimeoutExpired:
        pytest.fail(
            f"Lumice --benchmark (ray_num=infinite, Metal) did not terminate "
            f"within {_HANG_TIMEOUT_SEC}s — render-per-poll hang regression (task-317)."
        )
    assert result.returncode == 0, (
        f"Lumice --benchmark (Metal) exited {result.returncode}\nstderr:\n{result.stderr}"
    )
    assert "drain_aligned" in result.stdout, (
        f"Expected a drain_aligned [BENCHMARK] result (Metal).\nstdout:\n{result.stdout}"
    )


def test_benchmark_infinite_terminates_cuda() -> None:
    """Native CUDA: the strong regression detector. CUDA's large drains (~16.8M
    rays) make the render-per-poll bug catastrophic — pre-fix this never
    terminates (verified dev49 4060Ti: 150s timeout; the unbounded session trips
    the 32-bit device PCG ray-index cap and falls back to legacy). Gated on
    LUMICE_HAS_CUDA so it only runs where a CUDA device is actually present
    (matches the parity-cross-backend CUDA gate) rather than silently falling
    back to legacy CPU and voiding coverage."""
    # Double gate (matches test/parity-cross-backend/backend/ CUDA convention):
    # CUDA only exists on Linux/Windows AND requires an explicit device opt-in.
    if platform.system() not in ("Linux", "Windows") or os.environ.get("LUMICE_HAS_CUDA") != "1":
        pytest.skip("CUDA device not available (set LUMICE_HAS_CUDA=1 on a Linux/Windows CUDA host)")
    try:
        result = _run_infinite_benchmark(backend_env="cuda")
    except subprocess.TimeoutExpired:
        pytest.fail(
            f"Lumice --benchmark (ray_num=infinite, CUDA) did not terminate "
            f"within {_HANG_TIMEOUT_SEC}s — render-per-poll hang regression (task-317)."
        )
    assert result.returncode == 0, (
        f"Lumice --benchmark (CUDA) exited {result.returncode}\nstderr:\n{result.stderr}"
    )
    assert "drain_aligned" in result.stdout, (
        f"Expected a drain_aligned [BENCHMARK] result (CUDA).\nstdout:\n{result.stdout}"
    )
    # Detective guard: match the SPECIFIC pre-fix failure cause (the 32-bit
    # device PCG ray-index cap tripping -> fallback), not the generic "falling
    # back to legacy CPU" WARN — simulator.cpp emits that substring for several
    # unrelated reasons (Metal unavailable, incompatible renderer, etc.), which
    # would misattribute an unrelated warning to this regression.
    assert "NarrowPcgRayBase" not in result.stderr, (
        f"CUDA hit the 32-bit device PCG ray-index cap (NarrowPcgRayBase) -> "
        f"legacy fallback — the render-per-poll / cap hang regression may have "
        f"resurfaced (task-317 root#2).\nstderr:\n{result.stderr}"
    )
