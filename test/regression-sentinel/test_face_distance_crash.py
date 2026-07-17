"""Regression guard: random face_distance must not SIGSEGV.

Fix commits:
- 23a63386 (fix(core): make convex-polyhedron vertex dedup scale-relative)
- 730d54a9 (feat(core): accept signed face_distance + reject non-manifold Crystal meshes at factory boundary)
- a97fedb5 (fix(core): bounds-check the inner tri_id index in Crystal::GetFn(IdType))

Root cause: SolveConvexPolyhedronVtxD's vertex dedup used a fixed absolute
tolerance `2 * kIncidenceEpsD = 2e-5`. On hex-prism corners where 4+ planes
converge near-coincidentally — the typical outcome when face_distance is drawn
from gauss(1.0, 0.5) — per-triple numerical residuals reach exactly ~2e-5,
right at the threshold. Multiple candidate positions for one geometric corner
survive as distinct vertices (V=14 for a hex prism whose V should be 12), the
downstream BuildPolygonFaceData leaves poly_face_tri_id_ slots uninitialized,
and the polygon-indexed Crystal::GetFn wild-reads fn_map_ through the garbage
tri id → SIGSEGV in Simulator::SimulateOneWavelength.

Baseline reproduction rate: 4/20 = 20% (stochastic — depends on memory layout
of the malformed-mesh path). This sentinel replays the tracked reproducer
config (test/e2e/configs/repro_crash_face_distance.json, a verbatim copy of
the original diagnosis fixture — same gauss(1.0, 0.5) face_distance, same
filter/ray_num/max_hits — so the baseline crash-rate measurement transfers
without re-derivation) N times and requires every run to exit cleanly.

Runs fast (~1s each × N=15 ≈ 15s) so it stays in the default `pytest -v` PR
gate (no @pytest.mark.slow).
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from pathlib import Path

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root


_CONFIG_PATH = get_project_root() / "test" / "e2e" / "configs" / "repro_crash_face_distance.json"


def _run_once() -> subprocess.CompletedProcess:
    binary = find_lumice_binary()
    cfg = json.loads(_CONFIG_PATH.read_text())
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(cfg, f)
        cfg_path = f.name
    try:
        return subprocess.run(
            [str(binary), "-f", cfg_path],
            capture_output=True,
            text=True,
            timeout=60,
            env=os.environ.copy(),
        )
    finally:
        Path(cfg_path).unlink(missing_ok=True)


# 15 runs at pre-fix crash rate 20% → detection power ~96% (1 - 0.8^15).
_N_RUNS = 15


@pytest.mark.parametrize("run_idx", list(range(_N_RUNS)))
def test_random_face_distance_no_crash(run_idx: int) -> None:
    """gauss(1.0, 0.5) face_distance run must exit 0. Pre-fix rate: 4/20."""
    result = _run_once()
    assert result.returncode == 0, (
        f"Lumice exited {result.returncode} (signal death indicates SIGSEGV "
        f"regression on random face_distance) run_idx={run_idx}\n"
        f"stderr:\n{result.stderr}"
    )
