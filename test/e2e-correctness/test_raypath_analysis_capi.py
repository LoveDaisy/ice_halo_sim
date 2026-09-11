"""The analysis run, end to end through the C API (lumice.h "Raypath Analysis Run").

The scene is the 22° halo of ``test/e2e/configs/halo_22.json`` — same crystal, same sun —
with a 200k-ray budget (``raypath_analysis_halo_22.json``), so the top chain the C++ white-box
test already pins on the consumer (``test_raypath_histogram_consumer.cpp``) is read here
through the full stack: JSON commit, the forced-CPU session, the frame getters, the ctypes
mirrors.

AC3, three assertions on one scene:
  * full sky: the top chain is the 22° raypath, ``3-5``;
  * a 2° cone centred on the 22° ring (straight above the sun): the same top chain;
  * the same cone pointed at halo-free sky (altitude 85°, above both the 22° and the 46°
    rings): the top chain's energy is far below the on-ring cone's — the measured ratio and
    the threshold derived from it are at the assertion.
AC1 through ctypes: a render in progress refuses the analysis with LUMICE_ERR_SERVER.
AC2 through ctypes: under a Metal preference the run reports the CPU and logs the forcing.
The read-time symmetry (v4.33): the same run read under none / P / P|B / P|B|D conserves the
sums, never gains rows as bits are added, and the 22° path's ORBIT — the whole of it at the
finest, one row under P|B|D — carries the same energy at every one of them.

Requires the shared-lib build (``./scripts/build.sh -sj release``); run with
``pytest -v -m slow``.
"""

from __future__ import annotations

import ctypes
import math

import pytest

from test.e2e import capi_runner as cr
from test.e2e.runner import get_project_root

_CONFIG = str(get_project_root() / "test" / "e2e" / "configs" / "raypath_analysis_halo_22.json")

# The direction light travels for a sun at (altitude, azimuth) is the antipode of where the
# sun sits — the convention every direction in the C API uses (lumice.h, the marker family).
_SUN_ALTITUDE_DEG = 20.0
# Where the 22° halo's energy sits, straight above the sun: the ring's minimum deviation is
# 21.8°, its energy piles up just outside it, so +23° lands on the bright edge.
_ON_RING_ALTITUDE_DEG = _SUN_ALTITUDE_DEG + 23.0
# Above the 46° ring's top edge (20 + 46 = 66°) and every other arc of a randomly oriented
# prism: nothing but scattered residue lands here.
_OFF_RING_ALTITUDE_DEG = 85.0

_HALO_22 = "3-5"

# A fixed seed makes every run below a single-worker deterministic run (the seed contract),
# so the three energies are comparable and the ratio is a number, not a distribution.
_SEED = 20260911


def _sunlight_dir(altitude_deg: float, azimuth_deg: float = 0.0):
    lon = math.radians(azimuth_deg + 180.0)
    lat = math.radians(-altitude_deg)
    return (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))


def _full_sky_request() -> cr.LUMICE_RaypathAnalysisRequest:
    req = cr.LUMICE_RaypathAnalysisRequest()
    req.roi_mode = cr.LUMICE_RAYPATH_ROI_FULL_SKY
    # The scene's own ray budget, as every request here asked for before v4.32 gave the request
    # one of its own: a zero-initialized `infinite` would be a budget of zero rays.
    req.infinite = cr.LUMICE_RAYPATH_RAY_BUDGET_SCENE_DEFAULT
    return req


def _cone_request(altitude_deg: float, radius_deg: float = 2.0, rings: int = 4) -> cr.LUMICE_RaypathAnalysisRequest:
    req = _full_sky_request()
    req.roi_mode = cr.LUMICE_RAYPATH_ROI_CONE
    x, y, z = _sunlight_dir(altitude_deg)
    req.cone_center[0] = x
    req.cone_center[1] = y
    req.cone_center[2] = z
    req.cone_radius_rad = math.radians(radius_deg)
    req.cone_ring_count = rings
    req.cone_stop_target = 0  # the scene's ray budget alone, so the three runs are comparable
    return req


@pytest.mark.slow
def test_full_sky_top_chain_is_the_22_degree_path():
    r = cr.run_raypath_analysis_capi(_CONFIG, _full_sky_request(), sim_seed=_SEED)
    assert r.roi_mode == cr.LUMICE_RAYPATH_ROI_FULL_SKY
    assert r.sim_ray_num == 200000
    assert r.active_backend == cr.LUMICE_BACKEND_CPU
    assert len(r.entries) >= 2
    top = r.entries[0]
    assert top.display == _HALO_22, [e.display for e in r.entries[:5]]
    assert top.chain == [(1, [3, 5])]
    assert top.ring_energy == []
    # Sorted, and every counted ray is one the run traced (max_hits 7 bounds the fan-out).
    energies = [e.energy for e in r.entries]
    assert energies == sorted(energies, reverse=True)
    assert sum(e.count for e in r.entries) <= r.sim_ray_num * 8
    # The undeviated pass through opposite prism faces is #2, at roughly 80% of the halo:
    # measured 1.25x / 1.27x / 1.24x over the same three seeds (the C++ white-box test reads
    # 1.24x on its own run). A 1.1x floor is far from the ~52k-ray count's noise and still
    # catches a reordering.
    assert r.entries[1].display == "3-6"
    assert top.energy > 1.1 * r.entries[1].energy


def _is_22_degree_path(chain) -> bool:
    """A single-layer chain through two prism faces two apart (60° prism angle): the 22° halo.

    A physical statement about the hexagonal prism (faces 3..8 around it), NOT a re-derivation of
    core's reduction rule: every member of the orbit core folds into "3-5" under P/B/D is such a
    pair, and so is the mirror-image family "3-7" that only D folds in.
    """
    if len(chain) != 1:
        return False
    _crystal, faces = chain[0]
    if len(faces) != 2 or not all(3 <= f <= 8 for f in faces):
        return False
    return (faces[0] - faces[1]) % 6 in (2, 4)


def _is_undeviated_pass(chain) -> bool:
    """A single-layer chain through two OPPOSITE prism faces: the sun's own image, undeviated."""
    if len(chain) != 1:
        return False
    _crystal, faces = chain[0]
    if len(faces) != 2 or not all(3 <= f <= 8 for f in faces):
        return False
    return (faces[0] - faces[1]) % 6 == 3


@pytest.mark.slow
def test_read_time_symmetry_conserves_sums_and_the_22_degree_orbit_leads():
    """One run per symmetry (the seed makes them the same run), read under that symmetry.

    Expected row counts of the 22° orbit, from the prism's geometry: at the finest the orbit
    is 6 rotations x 2 mirror images = 12 rows; P folds the rotations (2 rows: "3-5" and its
    mirror "3-7"); B touches no prism-only path (still 2); D folds the mirror (1 row).

    The claim is about the orbit's UNION, not about the top row: the undeviated pass "3-6" is
    its own mirror image, so its orbit has 6 finest members to the halo's 12, and at the finest
    (and under P, where the halo is still split in two) a single undeviated row outweighs a
    single halo row. Only under D does "3-5" lead on its own — the C++ white-box test says the
    same. What holds at every symmetry is that the halo's rows together carry more than the
    undeviated pass's rows together, which is what the P|B|D order says.
    """
    by_sym = {}
    for sym in (0, cr.LUMICE_RAYPATH_SYMMETRY_P,
                cr.LUMICE_RAYPATH_SYMMETRY_P | cr.LUMICE_RAYPATH_SYMMETRY_B, cr.LUMICE_RAYPATH_SYMMETRY_ALL):
        # Every row: the unreduced read of this run has thousands, past the runner's default cap.
        by_sym[sym] = cr.run_raypath_analysis_capi(_CONFIG, _full_sky_request(), sim_seed=_SEED,
                                                   chain_id_symmetry=sym, max_entries=None)
    pbd = by_sym[cr.LUMICE_RAYPATH_SYMMETRY_ALL]
    total_count = sum(e.count for e in pbd.entries)
    total_energy = sum(e.energy for e in pbd.entries)
    prev_rows = None
    for sym, r in by_sym.items():
        assert r.sim_ray_num == pbd.sim_ray_num, sym
        assert sum(e.count for e in r.entries) == total_count, sym
        assert math.isclose(sum(e.energy for e in r.entries), total_energy, rel_tol=1e-9), sym
        if prev_rows is not None:
            assert len(r.entries) <= prev_rows, f"symmetry {sym}: rows grew as bits were added"
        prev_rows = len(r.entries)
        # The orbit's rows, at this symmetry: the union carries what the one P|B|D row carries,
        # and it is the leading orbit — ahead of the undeviated pass's rows together.
        orbit = [e for e in r.entries if _is_22_degree_path(e.chain)]
        assert sum(e.count for e in orbit) == pbd.entries[0].count, (sym, [e.display for e in orbit])
        assert math.isclose(sum(e.energy for e in orbit), pbd.entries[0].energy, rel_tol=1e-9), sym
        undeviated = [e for e in r.entries if _is_undeviated_pass(e.chain)]
        assert sum(e.energy for e in orbit) > 1.1 * sum(e.energy for e in undeviated), sym
        # And the top row is a member of one of those two orbits at every symmetry.
        assert _is_22_degree_path(r.entries[0].chain) or _is_undeviated_pass(r.entries[0].chain), (
            sym, r.entries[0].display)
        expected_orbit_rows = {0: 12, cr.LUMICE_RAYPATH_SYMMETRY_P: 2,
                               cr.LUMICE_RAYPATH_SYMMETRY_P | cr.LUMICE_RAYPATH_SYMMETRY_B: 2,
                               cr.LUMICE_RAYPATH_SYMMETRY_ALL: 1}[sym]
        assert len(orbit) == expected_orbit_rows, (sym, [e.display for e in orbit])
    # The display text follows the read: the single-crystal single-layer shape, no crystal prefix.
    assert pbd.entries[0].display == _HALO_22
    finest = by_sym[0]
    assert {e.display for e in finest.entries if _is_22_degree_path(e.chain)} >= {"3-5", "4-6", "3-7"}
    assert {e.display for e in by_sym[cr.LUMICE_RAYPATH_SYMMETRY_P].entries if _is_22_degree_path(e.chain)} == {
        "3-5", "3-7"}


@pytest.mark.slow
def test_cone_on_and_off_the_22_degree_ring():
    on_ring = cr.run_raypath_analysis_capi(_CONFIG, _cone_request(_ON_RING_ALTITUDE_DEG), sim_seed=_SEED)
    assert on_ring.roi_mode == cr.LUMICE_RAYPATH_ROI_CONE
    assert on_ring.entries, "the cone on the ring must count rays"
    top = on_ring.entries[0]
    assert top.display == _HALO_22, [e.display for e in on_ring.entries[:5]]
    assert len(top.ring_energy) == 4
    assert math.isclose(sum(top.ring_energy), top.energy, rel_tol=1e-9)
    assert all(e.display != "3-6" for e in on_ring.entries), "the sun's image cannot land 23° from the sun"

    off_ring = cr.run_raypath_analysis_capi(_CONFIG, _cone_request(_OFF_RING_ALTITUDE_DEG), sim_seed=_SEED)
    off_top_energy = off_ring.entries[0].energy if off_ring.entries else 0.0
    off_display = off_ring.entries[0].display if off_ring.entries else "(nothing)"
    # Measured at this budget over three seeds (20260911 / 1 / 7): the on-ring cone's top
    # chain carries 67x / 41x / 44x the energy of whatever leads the halo-free cone (a
    # scattered 1-3-2 residue, 11-16 rays) — the same solid angle on both sides, so
    # this is a per-steradian comparison too. 10x sits 4x under the weakest of those, far
    # from the 11-ray count's own noise, and a wrong centre convention or a broken membership
    # test would miss it by more than an order of magnitude in the other direction.
    assert top.energy > 10.0 * off_top_energy, (
        f"on-ring {top.display} energy {top.energy:.4g} vs off-ring {off_display} energy {off_top_energy:.4g}"
    )


@pytest.mark.slow
def test_render_in_progress_refuses_the_analysis():
    lib = cr._load_lib()
    server = lib.LUMICE_CreateServer()
    assert server
    try:
        cr._commit_config(lib, server, _CONFIG)  # starts a render run; 200k rays is in progress for a while
        req = _full_sky_request()
        err = lib.LUMICE_StartRaypathAnalysis(server, ctypes.byref(req))
        assert err == 7, f"expected LUMICE_ERR_SERVER (7) while the render runs, got {err}"
        lib.LUMICE_StopServer(server)
        assert lib.LUMICE_StartRaypathAnalysis(server, ctypes.byref(req)) == 0
        # And the other direction: a commit over the running analysis.
        scene = ctypes.c_void_p()
        assert lib.LUMICE_SceneFromJsonFile(_CONFIG.encode("utf-8"), ctypes.byref(scene)) == 0
        try:
            assert lib.LUMICE_CommitScene(server, scene, None) == 7
        finally:
            lib.LUMICE_SceneDestroy(scene)
    finally:
        lib.LUMICE_DestroyServer(server)


@pytest.mark.slow
def test_analysis_forces_cpu_under_a_metal_preference():
    r = cr.run_raypath_analysis_capi(
        _CONFIG, _full_sky_request(), sim_seed=_SEED, preferred_backend=cr.LUMICE_BACKEND_METAL
    )
    assert r.active_backend == cr.LUMICE_BACKEND_CPU
    assert any("forcing CPU route" in line for line in r.log_lines), r.log_lines[-10:]
    # And it really ran on the CPU: only that path carries the chain ids the histogram needs.
    assert r.entries and r.entries[0].display == _HALO_22


@pytest.mark.slow
def test_unproject_pixel_round_trip_through_ctypes():
    """The ctypes binding of LUMICE_UnprojectPixel (the exact inverse is pinned in C++)."""
    lib = cr._load_lib()
    view = cr.LUMICE_AnnotationView(width=256, height=192, lens_type=1, lens_fov=120.0,
                                    view_elevation=20.0, visible=2)
    out = (ctypes.c_float * 3)()
    valid = ctypes.c_int(-1)
    assert lib.LUMICE_UnprojectPixel(ctypes.byref(view), 128, 96, out, ctypes.byref(valid)) == 0
    assert valid.value == 1
    x, y, z = out[0], out[1], out[2]
    assert math.isclose(math.sqrt(x * x + y * y + z * z), 1.0, abs_tol=1e-5)
    # Looking up at elevation 20: the direction light travels points down (z < 0), and the
    # centre pixel is the view axis, 20° above the horizon.
    assert math.isclose(math.degrees(math.asin(-z)), 20.0, abs_tol=0.5)
    # At fov 150 the corner (160 px from the centre) is past the lens's image circle
    # (2f·sin(90°) ≈ 158 px); at fov 120 it would still be inside, since the lens images
    # up to 180° whatever fov the frame is cut at.
    view.lens_fov = 150.0
    assert lib.LUMICE_UnprojectPixel(ctypes.byref(view), 0, 0, out, ctypes.byref(valid)) == 0
    assert valid.value == 0, "the corner of a 150° fisheye canvas is outside the image circle"
