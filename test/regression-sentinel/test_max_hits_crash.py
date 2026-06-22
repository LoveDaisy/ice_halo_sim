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


def _build_config(max_hits: int, ray_num: int = 20000, metal_compatible: bool = False) -> dict:
    # halo_22's crystal produces rays that bounce >15 times (confirmed: pre-fix
    # this config SIGSEGVs at max_hits=16/32 on the legacy CPU path), so it
    # genuinely exercises the overflow-arena path — the detective power this
    # sentinel needs.
    base = json.loads((get_project_root() / "test" / "e2e" / "configs" / "halo_22.json").read_text())
    base["scene"]["ray_num"] = ray_num
    base["scene"]["max_hits"] = max_hits
    if metal_compatible:
        # halo_22 ships a fisheye_equal_area lens, which MetalTraceBackend rejects
        # (IsCompatible → false) so LUMICE_TRACE_BACKEND=metal silently falls back
        # to legacy CPU — that would make the "metal" case a disguised CPU re-run
        # with zero device coverage. Swap to a dual-fisheye fov=180 renderer (the
        # exact projection the GUI hardcodes) so the native Metal trace + device
        # exit-seq readback path actually runs. The crystal/scene (long-path
        # producer) is untouched.
        base["render"] = [
            {
                "id": 1,
                "lens": {"type": "dual_fisheye_equal_area", "fov": 180},
                "resolution": [512, 256],
                "view": {"elevation": 0},
                "visible": "full",
            }
        ]
    return base


def _run_with_max_hits(max_hits: int, backend_env: str | None = None) -> subprocess.CompletedProcess:
    binary = find_lumice_binary()
    cfg = _build_config(max_hits, metal_compatible=(backend_env == "metal"))
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
    """Native Metal path: max_hits > kInlineCap must run cleanly through the
    device trace + exit-seq readback (ExitFaceSeq::kCap == kMaxHits, no
    truncation). Uses a dual-fisheye fov=180 renderer so the backend does NOT
    fall back to legacy CPU — otherwise this test would silently re-run the CPU
    path and have zero device coverage."""
    if os.uname().sysname != "Darwin":
        pytest.skip("Metal backend is macOS-only")
    result = _run_with_max_hits(max_hits, backend_env="metal")
    assert result.returncode == 0, (
        f"Lumice (Metal) exited {result.returncode} for max_hits={max_hits}"
        f"\nstderr:\n{result.stderr}"
    )
    # Detective-power guard: fail if the run silently fell back to legacy CPU,
    # which would void the device-path coverage this test is meant to provide.
    assert "falling back to legacy CPU" not in result.stderr, (
        f"Metal backend fell back to legacy CPU for max_hits={max_hits}; this "
        f"test no longer exercises the native Metal path.\nstderr:\n{result.stderr}"
    )
