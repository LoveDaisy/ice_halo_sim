"""End-to-end regression for the LUMICE_TRACE_BACKEND=cpu_backend production route.

Regression for scrum-253.4: the cpu_backend production route used to SIGSEGV
(exit 139) before ever writing an image — the CpuTraceBackend trace workspace
was sized per small batch (curr_ray_num*2) instead of to the layer total, so
cross-hit fan-out overflowed it (root cause in
scratchpad/scrum-metal-frame-correctness/task-cpu-backend-route-segfault/INVESTIGATION.md).

The CpuTraceBackend unit tests (test_cpu_trace_backend.cpp) cover the direct
TraceLayer call path; this test covers the full CLI/server/ConsumeData plumbing
that only the production route exercises. Marked slow (excluded from the CI fast
subset) because it drives a 1M-ray simulation.
"""

from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root

# TODO: relocate configs when follow-up task completes
CONFIG = get_project_root() / "test" / "e2e" / "configs" / "cpu_backend_route.json"


@pytest.mark.slow
def test_cpu_backend_route_runs_end_to_end():
    """LUMICE_TRACE_BACKEND=cpu_backend must render end-to-end (exit 0, non-empty PNG).

    Pre-fix this exited 139 (SIGSEGV) with no output.
    """
    assert CONFIG.exists(), f"config not found: {CONFIG}"

    try:
        binary = find_lumice_binary()
    except FileNotFoundError as e:
        pytest.skip(str(e))

    with tempfile.TemporaryDirectory(prefix="lumice_cpu_backend_") as out_dir:
        env = dict(os.environ, LUMICE_TRACE_BACKEND="cpu_backend")
        result = subprocess.run(
            [str(binary), "-f", str(CONFIG), "-o", out_dir, "--format", "png"],
            capture_output=True,
            text=True,
            timeout=300,
            env=env,
        )

        assert result.returncode == 0, (
            f"cpu_backend route exited {result.returncode} "
            f"(pre-fix segfault was 139)\nstderr:\n{result.stderr}"
        )

        pngs = list(Path(out_dir).glob("*.png"))
        assert pngs, "cpu_backend route produced no PNG output"
        assert any(p.stat().st_size > 0 for p in pngs), "cpu_backend route PNG is empty"
