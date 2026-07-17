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
import signal
import subprocess
import tempfile
from pathlib import Path

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root


_CONFIG_PATH = get_project_root() / "test" / "e2e" / "configs" / "repro_crash_face_distance.json"

# A trivially-runnable config used only by the module-level smoke check. The
# ms_filter_leak_impossible fixture is a good pick: it uses a stable prism
# geometry (no random face_distance), always exits cleanly, and lives beside
# the reproducer fixture so it fails for the same infrastructure reasons.
_SMOKE_CONFIG_PATH = get_project_root() / "test" / "e2e" / "configs" / "ms_filter_leak_impossible.json"


def _run_config(config_path: Path, timeout: float = 60.0) -> subprocess.CompletedProcess:
    binary = find_lumice_binary()
    cfg = json.loads(config_path.read_text())
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(cfg, f)
        cfg_path = f.name
    try:
        return subprocess.run(
            [str(binary), "-f", cfg_path],
            capture_output=True,
            text=True,
            timeout=timeout,
            env=os.environ.copy(),
        )
    finally:
        Path(cfg_path).unlink(missing_ok=True)


def _run_once() -> subprocess.CompletedProcess:
    return _run_config(_CONFIG_PATH, timeout=60.0)


def _smoke_check_binary() -> None:
    """Fail fast before N runs if the binary can't even start on a stable config.

    Prior sessions hit false positives when `find_lumice_binary()` picked up a
    stale build artifact that returned a non-zero exit for pure infrastructure
    reasons (GLIBC mismatch, stale linkage), and the parametrized loop below
    reported all N runs as "SIGSEGV regression" — which was misleading because
    those failures were not signal deaths. This smoke check runs a
    fixed-face-distance config first: if the binary is broken at the
    infrastructure level, the assertion fires with a clear message before the
    parametrized loop starts.
    """
    result = _run_config(_SMOKE_CONFIG_PATH, timeout=30.0)
    assert result.returncode == 0, (
        f"Lumice binary infrastructure check failed (returncode={result.returncode}) — "
        f"this is not a SIGSEGV regression; the binary itself cannot run a known-good "
        f"config. Check LUMICE_BIN, the shared-library build (`./scripts/build.sh -j "
        f"release`), or the linker. This sentinel will now skip the parametrized "
        f"runs to avoid misattributing infrastructure failure to the random-face_distance "
        f"crash it is written to catch.\n"
        f"stderr:\n{result.stderr}"
    )


@pytest.fixture(scope="module", autouse=True)
def _binary_smoke_check() -> None:
    _smoke_check_binary()


# 15 runs at pre-fix crash rate 20% → detection power ~96% (1 - 0.8^15).
# Overridable via SENTINEL_N so scripts/verify_crash_sentinel_detection_power.sh
# can actually increase N when its own AMBIGUOUS branch recommends a rerun —
# without this, the env var only changed the script's own log text while the
# parametrized loop below silently kept running 15 regardless.
_N_RUNS = int(os.environ.get("SENTINEL_N", "15"))


def _classify_exit(returncode: int) -> str:
    """Distinguish POSIX signal death from a clean non-zero exit for reporting.

    POSIX returns a negative value from subprocess.run when the child died from
    a signal (returncode = -signum), and a positive value for a clean non-zero
    process exit. Only the signal path is what this sentinel is written to
    catch — a clean non-zero exit is a config-rejection / infrastructure / API
    issue and gets a different label so operators do not misdiagnose it as the
    SIGSEGV regression.

    Anchor: scripts/verify_crash_sentinel_detection_power.sh greps this
    function's output for the literal substrings "signal-death" and "clean
    non-zero exit" to classify the reverted-arm test run. Changing either
    literal here without updating that script's grep patterns will silently
    make it report AMBIGUOUS/0-detections regardless of what actually
    happened — the exact failure mode this coupling exists to prevent.
    """
    if returncode < 0:
        try:
            name = signal.Signals(-returncode).name
        except ValueError:
            name = f"SIG{-returncode}"
        return f"signal-death ({name}) — SIGSEGV-class regression"
    if returncode > 0:
        return f"clean non-zero exit (returncode={returncode}) — NOT a signal death; likely config rejection, infrastructure, or API issue"
    return "clean exit (returncode=0)"


@pytest.mark.parametrize("run_idx", list(range(_N_RUNS)))
def test_random_face_distance_no_crash(run_idx: int) -> None:
    """gauss(1.0, 0.5) face_distance run must exit 0. Pre-fix rate: 4/20."""
    result = _run_once()
    assert result.returncode == 0, (
        f"Lumice exited: {_classify_exit(result.returncode)}\n"
        f"run_idx={run_idx}\n"
        f"stderr:\n{result.stderr}"
    )
