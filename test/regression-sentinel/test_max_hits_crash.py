"""Regression guard: max_hits > kInlineCap must not crash.

Fix commits: task-284 (single-ray EmplaceBack arena dup + ExitFaceSeq::kCap
extension to kMaxHits=64).

Root cause: CollectData routed continuation rays through the 2-arg
RayBuffer::EmplaceBack(RaySeg, RaypathRecorder), which copies overflow_idx_ but
never calls DupOverflowSlot — so the dst buffer's overflow_arena_ stayed null
while overflow_idx_ pointed into the src arena. The next
TraceRayBasicInfo → RecorderFanOut → DupOverflowSlot then dereferenced
src.overflow_arena_(null) + slot*64 (with slot=0 landing at 0x0) → SIGSEGV.

Trigger conditions:
- max_hits > RaypathRecorder::kInlineCap (= 15)
- Any config with ray_num large enough to populate the continuation path.

This test runs a small ray budget at max_hits=32 — the exact reproduction used
to diagnose the bug — and asserts the process exits cleanly (not by signal).
Repeats on Metal backend when LUMICE_TRACE_BACKEND=metal is exported.

Runs fast (~1s) so it stays in the default `pytest -v` PR gate (no @pytest.mark.slow).
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from pathlib import Path

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root


def _build_config(max_hits: int, ray_num: int = 20000) -> dict:
    base = json.loads((get_project_root() / "test" / "e2e" / "configs" / "halo_22.json").read_text())
    base["scene"]["ray_num"] = ray_num
    base["scene"]["max_hits"] = max_hits
    return base


def _run_with_max_hits(max_hits: int, backend_env: str | None = None) -> subprocess.CompletedProcess:
    binary = find_lumice_binary()
    cfg = _build_config(max_hits)
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(cfg, f)
        cfg_path = f.name
    try:
        env = os.environ.copy()
        if backend_env is not None:
            env["LUMICE_TRACE_BACKEND"] = backend_env
        return subprocess.run(
            [str(binary), "-f", cfg_path],
            capture_output=True,
            text=True,
            timeout=120,
            env=env,
        )
    finally:
        Path(cfg_path).unlink(missing_ok=True)


@pytest.mark.parametrize("max_hits", [16, 24, 32, 64])
def test_max_hits_overflow_cpu_no_crash(max_hits: int) -> None:
    """CPU path: max_hits > kInlineCap must not SIGSEGV (exit 139)."""
    result = _run_with_max_hits(max_hits)
    assert result.returncode == 0, (
        f"Lumice exited {result.returncode} (signal death indicates SIGSEGV "
        f"regression) for max_hits={max_hits}\nstderr:\n{result.stderr}"
    )


@pytest.mark.parametrize("max_hits", [16, 24, 32, 64])
def test_max_hits_overflow_metal_no_crash(max_hits: int) -> None:
    """Metal path: max_hits > kInlineCap must not crash via the device readback path."""
    if os.uname().sysname != "Darwin":
        pytest.skip("Metal backend is macOS-only")
    result = _run_with_max_hits(max_hits, backend_env="metal")
    assert result.returncode == 0, (
        f"Lumice (Metal) exited {result.returncode} for max_hits={max_hits}"
        f"\nstderr:\n{result.stderr}"
    )
