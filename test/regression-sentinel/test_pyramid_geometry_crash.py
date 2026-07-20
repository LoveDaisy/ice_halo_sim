"""Regression guard: pyramid + random face_distance must not SIGSEGV on Metal.

Fix commit: 3c3bc3fb (fix(core): decouple polygon-face count/stride in
Crystal::BuildPolygonFaceData)

Root cause: BuildPolygonFaceData used a two-phase sizing where the initial
poly_face_cnt_ came from valid_cnt (planes that won at least one triangle) and
the allocation was laid out at stride=valid_cnt. The fill loop then applied a
second filter (representative-triangle area < 1e-6 * max) and rewrote
poly_face_cnt_ = idx to the final written count. The allocated buffer stayed
laid out at the larger stride while poly_face_cnt_ shrank. Crystal's copy/move
ctors re-derived the poly_face_d_ / poly_face_tri_id_ offsets from the shrunk
count and pointed them at the wrong regions of poly_face_data_:

  - CPU: GetFn(IdType)'s inner bound-check turned the wild tri_id read into a
    silent kInvalidId return; polygon-face numbers went missing, downstream
    raypath/filter behavior degraded silently.
  - Metal: UploadCrystal's centroid loop dereferenced the wild tri_id into
    the triangle-vertex buffer, walking past the mapped allocation on some
    inputs → SIGSEGV in the Simulator hot path.

The trigger requires a mesh that (a) passes the RejectMalformed factory
Euler-check gate at the boundary, yet (b) still has at least one polygon plane
whose argmax-representative triangle has near-zero area (below 1e-6 * max_area).
Thin-wedge pyramid + gauss(1, 0.5) face_distance surfaces this shrink path
reliably: this fixture hits ~6 "degenerate rep triangle" branches per 2M-ray
run on the current calibrated configuration.

The fix rewrites BuildPolygonFaceData to collect the surviving planes first and
then size the allocation off `accepted.size()`, so poly_face_cnt_ equals the
actual allocation stride from first assignment onward. Copy/move ctors then
re-derive correct offsets; UploadCrystal's centroid read stays in bounds.

Layered defenses (belt-and-suspenders, all in the same commit — the guards
below only turn any future upstream stride drift into a detectable "no fn" /
"zero centroid + WARN" symptom rather than a wild read):
  - CPU: Crystal::GetFn(IdType)'s inner tri bound-check (pre-existing).
  - Metal: MetalTraceBackend::Impl::UploadCrystal's centroid tri_id bound-check
    (added symmetrically in the same fix commit).

Assertions:
  1. `returncode == 0` — the primary signal the SIGSEGV is caught.

Retired anti-vacuous WARN guard (commit 14953369, "delete BuildPolygonFaceData
+ RejectMalformed + shrink counter"): `Crystal::BuildPolygonFaceData` — the
only emitter of the "degenerate rep triangle" WARN this assertion used to
require — no longer exists (`grep -r "degenerate rep triangle" src/` is empty
tree-wide). The crystal representation this sentinel was fixed under generated
polygon-face grouping numerically, by triangulating the mesh and picking an
argmax "representative triangle" per face; a locally near-zero-area pick was
the shrink-path trigger. The closed-form crystal representation knows face
membership parametrically from construction, so there is no per-face
triangle-argmax step left to produce a degenerate pick — the bug class is
structurally unreachable, not silently unexercised. Re-adding an equivalent
anti-vacuous guard would mean asserting on some property of the closed-form
path, which is a different sentinel for a different (currently nonexistent)
bug, not a retune of this one. `returncode == 0` remains meaningful: it still
confirms this thin-wedge pyramid + gauss(1, 0.5) face_distance config runs
Metal to completion without crashing under the current representation.

Runs fast (~2s per iteration × N=15 ≈ 30s) so it stays in the default `pytest
-v` PR gate (no @pytest.mark.slow). The two-arm detection-power verification
(scripts/verify_pyramid_crash_sentinel_detection_power.sh) still exercises the
*old* (pre-closed-form) tree via its own merge-base revert arm, so it is
unaffected by this retirement — it never runs against the current HEAD's
closed-form path.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

from test.e2e.runner import find_lumice_binary, get_project_root


# The SIGSEGV this sentinel guards lives in the Metal backend
# (UploadCrystal's centroid loop dereferencing a wild tri_id). On the CPU
# backend the same corrupted crystal is caught by Crystal::GetFn's bound-check
# (PR #206), so a CPU run neither crashes pre-fix NOR post-fix — running the
# sentinel on CPU has ZERO crash-detection power (it would pass a reverted,
# still-broken binary). We therefore force Metal below and skip entirely where
# Metal does not exist.
pytestmark = pytest.mark.skipif(
    sys.platform != "darwin",
    reason="pyramid UploadCrystal SIGSEGV is Metal-specific; no Metal backend off macOS",
)


_CONFIG_PATH = get_project_root() / "test" / "e2e" / "configs" / "repro_crash_pyramid_face_distance.json"

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
    env = os.environ.copy()
    # Bias the fixture toward per-crystal shrink-path traffic within a short
    # wallclock: smaller dispatch batches → more resample points at the same
    # ray_num. LUMICE_DISPATCH_RAY_NUM is a documented A-class experimental
    # knob (see doc/env-var-policy.md); it never bypasses any user-facing
    # switch — it only changes dispatch granularity, so setting it here is
    # equivalent to what a manual invocation would pass through the CLI.
    env.setdefault("LUMICE_DISPATCH_RAY_NUM", "1024")
    # Force the Metal backend: the guarded crash is Metal-only, and the CLI's
    # default backend is CPU on this platform, which would make every run pass
    # regardless of the fix (see module-level note). Verified in the smoke check
    # that Metal actually engaged (gpu_route=true) rather than silently falling
    # back to CPU.
    env.setdefault("LUMICE_TRACE_BACKEND", "metal")
    try:
        return subprocess.run(
            [str(binary), "-f", cfg_path],
            capture_output=True,
            text=True,
            timeout=timeout,
            env=env,
        )
    finally:
        Path(cfg_path).unlink(missing_ok=True)


def _run_once() -> subprocess.CompletedProcess:
    return _run_config(_CONFIG_PATH, timeout=60.0)


def _smoke_check_binary() -> None:
    """Fail fast before N runs if the binary can't even start on a stable config."""
    result = _run_config(_SMOKE_CONFIG_PATH, timeout=30.0)
    assert result.returncode == 0, (
        f"Lumice binary infrastructure check failed (returncode={result.returncode}) — "
        f"this is not a SIGSEGV regression; the binary itself cannot run a known-good "
        f"config. Check LUMICE_BIN, the shared-library build (`./scripts/build.sh -j "
        f"release`), or the linker. This sentinel will now skip the parametrized "
        f"runs to avoid misattributing infrastructure failure to the pyramid+random "
        f"face_distance crash it is written to catch.\n"
        f"stderr:\n{result.stderr}"
    )
    # The crash is Metal-only. If the forced backend did not actually engage
    # (e.g. Metal unavailable → CPU fallback), the parametrized runs would pass
    # vacuously on a still-broken binary. Skip rather than give false assurance.
    combined = result.stdout + result.stderr
    if "gpu_route=true" not in combined:
        pytest.skip(
            "Metal backend did not engage (no 'gpu_route=true' in output) — the "
            "pyramid crash is Metal-specific and a CPU run has no crash-detection "
            "power, so this sentinel is skipped rather than passing vacuously."
        )


@pytest.fixture(scope="module", autouse=True)
def _binary_smoke_check() -> None:
    _smoke_check_binary()


# 15 runs matches the sister prism-face_distance sentinel (~30s wallclock at
# ~2s/run). Overridable via SENTINEL_N so
# scripts/verify_pyramid_crash_sentinel_detection_power.sh can raise N when
# hunting for signal deaths without editing this file.
_N_RUNS = int(os.environ.get("SENTINEL_N", "15"))


def _classify_exit(returncode: int) -> str:
    """Distinguish POSIX signal death from a clean non-zero exit for reporting.

    Anchor: scripts/verify_pyramid_crash_sentinel_detection_power.sh greps
    this function's output for "signal-death" and "clean non-zero exit" to
    classify the reverted-arm run. Changing either literal here without
    updating the script's grep patterns will silently make it report
    AMBIGUOUS/0-detections regardless of what actually happened.
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
def test_pyramid_random_face_distance_no_crash(run_idx: int) -> None:
    """Pyramid + gauss(1, 0.5) face_distance must exit 0 (no SIGSEGV) under the
    closed-form crystal representation on Metal.

    No anti-vacuous WARN guard here (see module docstring "Retired anti-vacuous
    WARN guard"): the degenerate-representative-triangle branch this sentinel
    was originally written to force no longer exists in any code path, so a
    fixed string to assert on would not exist either.
    """
    result = _run_once()
    assert result.returncode == 0, (
        f"Lumice exited: {_classify_exit(result.returncode)}\n"
        f"run_idx={run_idx}\n"
        f"stderr:\n{result.stderr}"
    )
