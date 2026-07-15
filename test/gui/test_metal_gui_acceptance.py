"""Metal-in-GUI acceptance gate (scrum-268 / G4 — the §5 GUI-first north-star).

G1 (test_metal_throughput.py) proves the *engine* throughput thesis via
`--benchmark` (no GUI consumer / no poller readback in the loop). G4 proves the
thing the whole §5 arc actually exists for: that the single-engine Metal backend
**beats legacy CPU in the real GUI regime** (live preview = frequent commit +
intermediate-result feedback), end-to-end through the same path a user drives —
server reconstruct-on-toggle → poller readback → consumer projection.

Why this is a *separate* gate from G1 (scrum-268.6 + task-fix-throughput-bench-honesty):
  - G1 measures the bare engine via `--benchmark` (no GUI consumer / poller in the
    loop); G4 drives the real end-to-end GUI path (reconstruct → poller readback →
    consumer projection). They now agree in magnitude — both ~8-10x legacy on the
    heavy scene (the earlier "engine 9.5x but GUI only 2x → 6x poller headroom"
    was a measurement artifact: a setup-inflated --benchmark reading + a pre-
    task-272 GUI run that silently dropped the complex filter). G4 stays a
    distinct gate because it is the only one exercising the GUI-specific failure
    modes below (ctor backend propagation, finite-budget dilution), not because
    the GUI lags the engine.
  - Two real bugs were flushed out reaching this gate and are guarded here:
      1. ServerImpl ctor did not propagate preferred_backend to its simulators, so
         CreateServerEx(METAL) sized dispatches for Metal yet traced legacy CPU
         (fixed server.cpp). A regression would drop the ratio toward the stall
         (~0.1x) — caught by _SANITY_FLOOR.
      2. steady_state over a fixed window dilutes a *finite* ray budget (a fast
         backend finishes early -> reports rays/window). This gate uses an
         INFINITE-budget config so the legacy-vs-Metal comparison is fair.

Drives the REAL product path: LUMICE_PERF_METAL=1 makes the perf harness toggle
use_metal_backend so DoRun reconstructs into the single-engine Metal topology
(MaybeReconstructServerForBackend) — NOT a SetPreferredBackend hack on an
N-worker server. Baseline is legacy CPU (the GUI's real path); NEVER cpu_backend.

Correctness is NOT re-tested here — it is already gated by the raw-XYZ parity
matrix (test_metal_exit_seam_parity, 10/10) and the auto_ev PSNR references. G4
owns the GUI-regime throughput + responsiveness legs.

@pytest.mark.slow — needs the GUI test binary + a display/GL context; Darwin-only
(Metal). Skips gracefully when the binary is absent or no display is available.
"""
import json
import os
import platform
import re
import subprocess
import tempfile

import pytest

from test.e2e.runner import get_project_root

pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin", reason="Metal backend is only available on macOS"
)

_PROJECT_ROOT = get_project_root()
_GUI_BIN = _PROJECT_ROOT / "build" / "Release" / "bin" / "gui_test"
_LIB_DIR = _PROJECT_ROOT / "build" / "Release" / "lib"
_CONFIGS_DIR = _PROJECT_ROOT / "test" / "e2e" / "configs"

# Heavy multi-MS + filter + multi-crystal scene — the regime where legacy struggles
# and the GUI win matters most (same family as the G1 throughput gate).
_HEAVY_CONFIG = "ms_multi_crystal_complex_filter"

_TIMEOUT = 120  # seconds per GUI perf run (steady 2s + slider 5s + Metal warm-up)

# --- North-star throughput gate (GUI regime) -------------------------------------
# Re-measured 2026-06-19 (infinite-budget complex_filter, reconstruct path, post
# task-272 filter fix): Metal GUI steady ~9.5x legacy on the heavy scene. Gate at
# parity (>=1.0); the win has wide margin.
_GATE = 1.0
# Sanity floor: a drop far below parity means Metal fell back / the reconstructed
# server stalled (the ctor-propagation bug regressed to ~0.1x). 0.8 keeps margin
# against GUI-loop noise while tripping hard on a real regression.
_SANITY_FLOOR = 0.8

# --- Responsiveness gate: freeze detection, NOT "must beat legacy" ----------------
# Large dispatch buys ~2x throughput at the cost of a higher first-frame latency
# (an honest tradeoff, NOT a regression to hide). explore-265's failure mode was a
# single dispatch >100ms freezing the UI (5s drag -> 2 frames, i.e. per-frame
# stalls in the seconds range). This gate catches that pathological freeze regime.
#
# Calibration note (task-364, 2026-07-15): this test runs the HEAVY config
# (_HEAVY_CONFIG, multi-MS + complex filter + multi-crystal, ray_num=infinite),
# whose healthy Metal first_upload median measures ~200ms on Mac — this is the
# physical floor of the first batch's trace + XYZ readback, which the O2 PSO/device
# process-level cache (task-364) deliberately does NOT cover (it eliminates the
# per-commit PSO-rebuild cost, not the GPU work of the first batch itself). The
# prior 150ms value was calibrated against a lighter config (~71ms median) and only
# surfaced as a failure once task-364 fixed the rays>0 assertion that masked it.
# 250ms keeps headroom against run-to-run noise (measured 196-202ms, tight) while
# still tripping hard on a genuine >100ms/dispatch freeze (which lands in the
# seconds range). Whether the heavy-scene first batch can be compressed further,
# or its UX softened, is deferred to a backlog explore (not a task-364 regression).
_FIRST_UPLOAD_FREEZE_MS = 250.0

_RE_STEADY = re.compile(r"steady_state:\s+([\d.]+)\s+rays/sec")
_RE_FIRST_UPLOAD = re.compile(r"first_upload\s+avg=\d+ms\s+median=(\d+)ms")
_RE_FALLBACK = re.compile(r"falling back to legacy CPU|incompatible with render", re.IGNORECASE)
_RE_NO_DISPLAY = re.compile(r"Failed to create GLFW window|Failed to initialize", re.IGNORECASE)


def _make_infinite_config(tmp_dir: str) -> str:
    """Copy the heavy config with ray_num='infinite'.

    The GUI's live-preview regime IS infinite-budget; using it here also removes
    the finite-budget steady_state dilution that would unfairly penalise the fast
    (Metal) backend in a fixed measurement window.
    """
    src = _CONFIGS_DIR / f"{_HEAVY_CONFIG}.json"
    cfg = json.loads(src.read_text())
    cfg["scene"]["ray_num"] = "infinite"
    out = os.path.join(tmp_dir, "gui_regime_heavy_infinite.json")
    with open(out, "w") as f:
        json.dump(cfg, f)
    return out


def _run_gui_perf(config_path: str, metal: bool) -> dict:
    """Run gui_test perf scenarios; return parsed steady rps + first_upload.

    Returns {"steady_rps": float, "first_upload_ms": float, "fell_back": bool,
             "no_display": bool, "raw": str}.
    """
    env = dict(os.environ)
    env["LUMICE_PERF_CONFIG"] = config_path
    env["DYLD_LIBRARY_PATH"] = f"{_LIB_DIR}:{env.get('DYLD_LIBRARY_PATH', '')}"
    env["LUMICE_PERF_CORELOG"] = "1"  # surface the backend-route / fallback decision
    if metal:
        env["LUMICE_PERF_METAL"] = "1"  # real product path: reconstruct into single-engine Metal
    else:
        env.pop("LUMICE_PERF_METAL", None)  # legacy = unset (NOT cpu_backend)
    env.pop("LUMICE_TRACE_BACKEND", None)  # never let an env override skew the comparison

    proc = subprocess.run(
        [str(_GUI_BIN), "--filter", "perf_test"],
        capture_output=True, text=True, timeout=_TIMEOUT, env=env,
    )
    out = proc.stdout + proc.stderr
    steady = 0.0
    m = _RE_STEADY.search(out)
    if m:
        steady = float(m.group(1))
    first_upload = 0.0
    mu = _RE_FIRST_UPLOAD.search(out)
    if mu:
        first_upload = float(mu.group(1))
    return {
        "steady_rps": steady,
        "first_upload_ms": first_upload,
        "fell_back": bool(metal and _RE_FALLBACK.search(out)),
        "no_display": bool(_RE_NO_DISPLAY.search(out)),
        "raw": out,
    }


@pytest.mark.slow
def test_metal_gui_north_star():
    """Metal single-engine beats legacy in the real GUI regime + stays responsive."""
    if not (_GUI_BIN.is_file() and os.access(_GUI_BIN, os.X_OK)):
        pytest.skip(f"GUI test binary not found at {_GUI_BIN}; build with ./scripts/build.sh -gtj release")

    with tempfile.TemporaryDirectory() as tmp_dir:
        cfg = _make_infinite_config(tmp_dir)
        legacy = _run_gui_perf(cfg, metal=False)
        if legacy["no_display"]:
            pytest.skip("No display / GL context available for the GUI test binary")
        metal = _run_gui_perf(cfg, metal=True)
        if metal["no_display"]:
            pytest.skip("No display / GL context available for the GUI test binary")

    assert not metal["fell_back"], (
        "Metal fell back to legacy CPU in the GUI path — backend requested but did "
        f"not run; the comparison is meaningless.\n--- metal output tail ---\n{metal['raw'][-2000:]}"
    )
    assert legacy["steady_rps"] > 0.0, f"legacy steady rps parse failed.\n{legacy['raw'][-2000:]}"
    assert metal["steady_rps"] > 0.0, f"metal steady rps parse failed.\n{metal['raw'][-2000:]}"

    ratio = metal["steady_rps"] / legacy["steady_rps"]
    print(
        f"[gui-north-star] {_HEAVY_CONFIG}: legacy_steady={legacy['steady_rps']:.0f} "
        f"metal_steady={metal['steady_rps']:.0f} ratio={ratio:.3f} "
        f"(sanity>={_SANITY_FLOOR}, gate>={_GATE}); "
        f"metal first_upload median={metal['first_upload_ms']:.0f}ms "
        f"(freeze threshold {_FIRST_UPLOAD_FREEZE_MS:.0f}ms)"
    )

    # Sanity: catches the reconstruct-stall regression (ctor backend propagation)
    # and silent fallback — either crashes the ratio far below parity.
    assert ratio >= _SANITY_FLOOR, (
        f"GUI throughput catastrophic regression ratio={ratio:.3f} < {_SANITY_FLOOR} — "
        f"Metal reconstructed server stalled or fell back (check ServerImpl ctor "
        f"preferred_backend propagation to simulators)."
    )

    # North-star: Metal single-engine beats legacy in the real GUI regime.
    assert ratio >= _GATE, (
        f"GUI north-star regression ratio={ratio:.3f} < {_GATE} — Metal no longer "
        f"beats legacy CPU end-to-end in the GUI live-preview regime (scrum-268 §5)."
    )

    # Responsiveness: large dispatch must not regress into the UI-freeze regime.
    # (Metal first_upload is honestly higher than legacy — the throughput/latency
    # tradeoff — but must stay well below the freeze threshold.)
    assert 0.0 < metal["first_upload_ms"] < _FIRST_UPLOAD_FREEZE_MS, (
        f"GUI responsiveness regression: Metal first_upload median="
        f"{metal['first_upload_ms']:.0f}ms >= {_FIRST_UPLOAD_FREEZE_MS:.0f}ms — large "
        f"dispatch is freezing the UI (single dispatch too long; explore-265 failure mode)."
    )
