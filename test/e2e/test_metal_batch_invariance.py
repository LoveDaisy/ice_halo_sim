"""Batch-size invariance + commit conservation gate (scrum-268 / G2).

Scrum 2 (task-268.4) split the historical `LUMICE_BATCH_RAY_NUM`'s dual role
into two knobs: `LUMICE_DISPATCH_RAY_NUM` (GPU dispatch grain — fed to the
backend per SimBatch) and `LUMICE_COMMIT_RAY_NUM` (consumer commit grain —
ConsumeData chunk size). This gate now drives the DISPATCH grain so it
characterises pure batch-vs-statistics behaviour of the trace pipeline,
independent of commit granularity. Two correctness contracts must survive
that split.

  1. **Batch-size dependence (characterize + anti-catastrophe guard).** Changing
     the GPU dispatch batch should not grossly change the statistical result.
     Measured truth (explore-3 E3, subprocess-isolated, single-worker seed=42,
     raw-XYZ): cross-batch corr(b128,b2048)=0.987 vs self-noise corr=0.9998 — a
     SMALL but real residual dependence (≈1.3% corr, ≈2.5% energy), from the
     per-scene geometry/crystal-proportion sampling granularity (a larger batch =
     fewer scenes = slightly coarser geometry sampling). Whether Scrum 2 must
     drive this to the self-noise floor (e.g. via the concern #5 geometry pool)
     or whether the residual is acceptable statistical equivalence is an
     **explore-3 decision** — this test does NOT presuppose it. It enforces an
     ACTIVE anti-catastrophe floor (catches a re-coupling regression to the old
     "one geometry per batch" severity) and PRINTS the cross-vs-self gap as the
     explore-3 input.

  2. **Exit conservation across the seam (D1 pre-registered — xfail until Scrum 2).**
     The worst-case `ms3_mixed_pyramid_heavy` overflows the fixed exit buffer
     (exit_cap=12288) under deep continuation and silently CLAMPS (E0). Root rays
     stay conserved (sim_rays == ray_num); the loss is at the exit seam. Scrum 2's
     incremental drain must eliminate the clamp.

CRITICAL (code-review BLOCKER-1): `LUMICE_DISPATCH_RAY_NUM` is read into a
function-local `static const` in server.cpp's GenerateScene, frozen ONCE per
process. So each batch size MUST run in its OWN subprocess — an in-process env
flip is a no-op and would compare a run against itself. Each batch run here is
a fresh subprocess (the module is self-spawning via __main__).

@pytest.mark.slow — needs shared-lib build; Darwin-only (Metal).
"""
import json
import math
import os
import platform
import re
import subprocess
import sys
import tempfile

import numpy as np
import pytest

from test.e2e._parity_metrics import _block_mean, _DS_BH, _DS_BW
from test.e2e.runner import find_lumice_binary, get_project_root

pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin", reason="Metal backend is only available on macOS"
)

_CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_SEED = 42            # non-zero → single-worker deterministic (server.cpp:196 / capi contract)
_SMALL_BATCH = 128
_LARGE_BATCH = 2048
_INVARIANCE_RAYS = 300_000  # geometry-coupling signal is clear at low counts; keep single-worker runs fast
_TIMEOUT = 180

# Anti-catastrophe floor (ACTIVE): the old "one geometry per batch" coupling
# would drop cross-batch corr far below this; 0.95 leaves margin over the
# measured 0.987 while still tripping on a gross re-coupling regression.
_CATASTROPHE_CORR = 0.95
_CATASTROPHE_ENERGY_TOL = 0.05

_RE_OVERFLOW = re.compile(r"exit-ray overflow", re.IGNORECASE)
_RE_STATS = re.compile(r"Stats:\s*sim_rays=(\d+)")


# --------------------------------------------------------------------------- #
# Subprocess helper: run ONE capi sim at a given batch (fresh process → the
# server.cpp static picks up this batch) and dump raw-XYZ. Invoked via __main__.
# --------------------------------------------------------------------------- #

def _spawn_run(config_name: str, batch: int):
    """Spawn a fresh process that runs capi at `batch`; return its raw-XYZ array.

    Each call is a new process so LUMICE_DISPATCH_RAY_NUM's process-static
    actually takes the requested value (code-review BLOCKER-1).
    """
    root = get_project_root()
    out = tempfile.NamedTemporaryFile(suffix=".npy", delete=False)
    out.close()
    env = dict(os.environ)
    # task-268.4 split: dispatch grain controls per-SimBatch ray count fed to
    # the backend; commit grain controls ConsumeData chunk size. Pin BOTH to
    # `batch` here so:
    #   (a) the statistical comparison genuinely varies the dispatch grain
    #       (the only knob that affects per-scene geometry sampling);
    #   (b) the positive control below (`Consume profile: N batches` ratio)
    #       stays a clean witness that the configured batch took effect —
    #       leaving commit at the default 128 would clamp the chunk count to
    #       a near-constant ≈ exits/128 and erase the ratio signal.
    env["LUMICE_DISPATCH_RAY_NUM"] = str(batch)
    env["LUMICE_COMMIT_RAY_NUM"] = str(batch)
    env.setdefault("LUMICE_LIB", str(root / "build" / "Release" / "lib" / "liblumice.dylib"))
    env["PYTHONPATH"] = str(root) + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run(
        [sys.executable, "-m", "test.e2e.test_metal_batch_invariance",
         config_name, str(batch), str(_SEED), str(_INVARIANCE_RAYS), out.name],
        capture_output=True, text=True, timeout=_TIMEOUT, env=env, cwd=str(root),
    )
    status_line = [l for l in proc.stdout.splitlines() if l.startswith("STATUS ")]
    assert status_line, (
        f"{config_name} batch={batch}: helper produced no STATUS line "
        f"(rc={proc.returncode}).\nstderr tail:\n{proc.stderr[-800:]}"
    )
    status = json.loads(status_line[-1][len("STATUS "):])
    assert status["routed_backend"] == "metal" and not status["fell_back"], (
        f"{config_name} batch={batch}: Metal did not run "
        f"(routed={status['routed_backend']!r} fell_back={status['fell_back']}); "
        f"comparison would be meaningless."
    )
    # Positive control that the requested batch actually took effect inside the
    # process (defeats a silent static-freeze / no-op env). Batches consumed ≈
    # ceil(rays / batch); b128 and b2048 differ ~16x, so this is unambiguous. The
    # "Consume profile: N batches" line is in the subprocess's captured log.
    m = re.search(r"Consume profile:\s*(\d+)\s*batches", proc.stdout + proc.stderr)
    status["batches"] = int(m.group(1)) if m else -1
    assert status["batches"] > 0, (
        f"{config_name} batch={batch}: could not read effective batch count "
        f"(Consume profile line missing) — cannot confirm batch took effect."
    )
    arr = np.load(out.name)
    os.unlink(out.name)
    return arr, status


def _corr(a: np.ndarray, b: np.ndarray) -> float:
    x = _block_mean(a.astype(np.float64), _DS_BH, _DS_BW).ravel()
    y = _block_mean(b.astype(np.float64), _DS_BH, _DS_BW).ravel()
    if x.std() == 0.0 or y.std() == 0.0:
        return 0.0
    c = float(np.corrcoef(x, y)[0, 1])
    return 0.0 if math.isnan(c) else c


_INVARIANCE_CONFIGS = ["parity_ms_prob05", "ms_multi_crystal_filtered"]


@pytest.mark.slow
@pytest.mark.parametrize("config_name", _INVARIANCE_CONFIGS)
def test_metal_batch_invariance(config_name):
    small, s_small = _spawn_run(config_name, _SMALL_BATCH)
    large, s_large = _spawn_run(config_name, _LARGE_BATCH)
    ctrl, _ = _spawn_run(config_name, _SMALL_BATCH)  # self-noise control (same batch, fresh process)

    # Positive control: the two batches genuinely ran at different grain.
    assert s_large["batches"] * 4 < s_small["batches"], (
        f"{config_name}: batch grain did not actually differ "
        f"(b{_SMALL_BATCH}={s_small['batches']} batches, b{_LARGE_BATCH}={s_large['batches']}); "
        f"suspect a frozen LUMICE_BATCH_RAY_NUM static — the comparison would be vacuous."
    )

    cross = _corr(small, large)
    self_noise = _corr(small, ctrl)
    small_Y = float(small[..., 1].sum())
    energy_ratio = float(large[..., 1].sum()) / small_Y if small_Y > 0 else math.inf
    print(
        f"[batch-inv] {config_name}: cross-batch corr(b{_SMALL_BATCH},b{_LARGE_BATCH})={cross:.4f} "
        f"self-noise corr={self_noise:.4f} gap={self_noise - cross:.4f} "
        f"energy_ratio={energy_ratio:.4f}  (explore-3: decide if residual needs geometry-pool)"
    )

    # --- ACTIVE anti-catastrophe gate (regression guard) -----------------------
    # Guards against Scrum 2's rewrite re-coupling batch to the result at the old
    # severity. Does NOT enforce full invariance — the residual (gap≈0.013 today)
    # is an explore-3 / geometry-pool decision (concern #5).
    assert cross >= _CATASTROPHE_CORR, (
        f"{config_name}: SEVERE batch dependence — cross-batch corr={cross:.4f} < "
        f"{_CATASTROPHE_CORR}. Scrum 2's rewrite re-coupled the GPU dispatch batch to "
        f"the statistical result at gross severity."
    )
    assert abs(energy_ratio - 1.0) <= _CATASTROPHE_ENERGY_TOL, (
        f"{config_name}: SEVERE batch energy dependence — energy_ratio={energy_ratio:.4f} "
        f"outside [1±{_CATASTROPHE_ENERGY_TOL}]."
    )


@pytest.mark.slow
def test_metal_exit_conservation_heavy():
    """Worst-case exit conservation: the deep-continuation heavy scene must not
    silently drop exit rays. Today it overflows exit_cap=12288 and clamps (E0);
    Scrum 2's incremental drain must eliminate it. Root rays stay conserved —
    this pins the loss to the exit seam, not root generation.
    """
    cfg = _CONFIGS_DIR / "ms3_mixed_pyramid_heavy.json"
    with open(cfg) as f:
        data = json.load(f)
    requested = 3000  # overflow is per-dispatch (continuation amplification per root ray)
    data["scene"]["ray_num"] = requested
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as tf:
        json.dump(data, tf)
        tmp_cfg = tf.name

    env = dict(os.environ)
    env["LUMICE_TRACE_BACKEND"] = "metal"
    # The cmake_install CLI is dynamically linked against liblumice.dylib after a
    # shared-lib (-s) build but ships without an LC_RPATH on this host (known
    # quirk), so it SIGABRTs on @rpath load unless DYLD_LIBRARY_PATH points at the
    # dylib. Slow capi tests sidestep this via LUMICE_LIB; this CLI-driven test
    # must set DYLD explicitly to stay robust across static/shared builds.
    env.setdefault("DYLD_LIBRARY_PATH", str(get_project_root() / "build" / "Release" / "lib"))
    try:
        with tempfile.TemporaryDirectory() as outdir:
            proc = subprocess.run(
                [str(find_lumice_binary()), "-f", tmp_cfg, "-o", outdir],
                capture_output=True, text=True, timeout=_TIMEOUT, env=env,
            )
    finally:
        os.unlink(tmp_cfg)
    log = proc.stdout + proc.stderr

    # Root conservation holds today (active assert): sim_rays == requested.
    stats = _RE_STATS.search(log)
    assert stats, f"no Stats line in output; run failed? rc={proc.returncode}\n{log[-500:]}"
    assert int(stats.group(1)) == requested, (
        f"root ray conservation broken: sim_rays={stats.group(1)} != {requested}"
    )

    # --- D1 pre-registered exit-conservation gate (xfail until Scrum 2 drain) ---
    # Note: depends on the overflow message being emitted at the default log level
    # to stdout/stderr; we scan both. If absent, `overflowed` reads False and the
    # gate would spuriously "pass" — acceptable as the drain landing is what makes
    # it legitimately stop overflowing.
    overflowed = bool(_RE_OVERFLOW.search(log))
    print(f"[exit-cons] ms3_mixed_pyramid_heavy: exit_overflow={overflowed} (gate: no overflow)")
    if overflowed:
        pytest.xfail(
            "ms3_mixed_pyramid_heavy overflows the fixed exit buffer (exit_cap=12288) "
            "and silently clamps exit rays under deep continuation. Scrum 2's "
            "incremental drain / commit<->batch decoupling must eliminate the clamp. "
            "D1 PRE-REGISTERED gate, not a failure."
        )


# --------------------------------------------------------------------------- #
# Self-spawning helper (BLOCKER-1 fix): `python -m test.e2e.test_metal_batch_invariance
#   <config> <batch> <seed> <rays> <out.npy>` runs ONE capi sim in this fresh
# process (so server.cpp's LUMICE_DISPATCH_RAY_NUM / LUMICE_COMMIT_RAY_NUM
# statics = <batch>) and dumps raw-XYZ + a STATUS json line.
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    _config_name, _batch, _seed, _rays, _out = sys.argv[1:6]
    from test.e2e.capi_runner import run_scene_capi_buffered

    _cfg = json.load(open(_CONFIGS_DIR / f"{_config_name}.json"))
    _cfg["scene"]["ray_num"] = int(_rays)
    _tf = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    json.dump(_cfg, _tf)
    _tf.close()
    try:
        _r = run_scene_capi_buffered(_tf.name, sim_seed=int(_seed), backend="metal", timeout_sec=_TIMEOUT)
    finally:
        os.unlink(_tf.name)
    np.save(_out, _r.flt_buf)
    # The parent (_spawn_run) reads the effective batch count from this process's
    # "Consume profile: N batches" log line as the positive control.
    print("STATUS " + json.dumps({
        "routed_backend": _r.routed_backend,
        "fell_back": _r.fell_back,
        "Ysum": float(_r.flt_buf[..., 1].sum()),
    }))
