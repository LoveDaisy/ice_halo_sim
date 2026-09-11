"""ctypes-based runner for scalar-intensity e2e tests.

The standard subprocess-based runner in :mod:`test.e2e.runner` only exposes
return code / stdout / stderr; it has no way to read scalar fields like
``snapshot_intensity`` from ``LUMICE_RawXyzResult``. Tests that
need those values drive Lumice through the C API directly via ``ctypes``.

Each call to :func:`run_scene_capi` creates a fresh ``LUMICE_Server``,
commits the requested config, polls until the server returns to IDLE with
valid data (or the timeout fires), reads the scalar result, and destroys
the server.

Which library. This runner loads ``liblumice_testapi``, not ``liblumice``. The
two are built from the same ``lumice_obj`` objects, so every ``LUMICE_*`` call
behaves identically; the test library additionally exports the ``LUMICE_TEST_*``
hooks declared in ``test/support/lumice_test_api.h`` -- test-only entry points
that the product ABI (``src/include/lumice.h``) must never carry. It is a
superset stand-in rather than a companion: ``-fvisibility=hidden`` leaves a side
library nothing to link against, so the hooks ship with their own copy of the
engine, and a test process loads exactly ONE of the two (two would be two engines
with two sets of statics). Nothing in this module calls a hook itself; the tests
that need one reach it through the same handle.

Library lookup order:
    1. ``LUMICE_LIB`` environment variable (full path to the shared library).
    2. ``build/Release/shared/lib/liblumice_testapi.{dylib,so}``
    3. ``build/cmake_install/shared/{liblumice_testapi.{dylib,so}, lib/liblumice_testapi.{dylib,so}}``
    4. ``build/cmake_build/shared/liblumice_testapi.{dylib,so}``

The library must be built with ``BUILD_SHARED_LIBS=ON`` (``./scripts/build.sh -s``),
which produces both shared libraries side by side. On Windows the file is
``lumice_testapi.dll`` (no ``lib`` prefix); there is no automatic candidate for
it, as there was none for ``lumice.dll`` -- point ``LUMICE_LIB`` at it. If lookup
fails, raises :class:`FileNotFoundError`.

**Test-only module**: the first call to :func:`run_scene_capi_buffered` installs
a process-level log callback into the C library (``LUMICE_SetLogCallback``).
Do not import this module from non-test contexts (bench scripts, REPLs) as the
hook intercepts all subsequent server log output without any visible indication.
"""

from __future__ import annotations

import contextlib
import ctypes
import os
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np


# Mirrors LUMICE_RawXyzResult in src/include/lumice.h. Anchor fields removed
# in task-remove-anchor-lane (64 → 56 bytes); the trailing uint64 `epoch` field
# (backend-lifecycle-epoch, 1.3) grew it back to 64 (48-byte effective_pixels +
# 4 pad + 8-byte epoch, 8-aligned). `emitted_energy` then went into that 4-byte
# pad rather than onto the end, so the size is still 64 and `epoch` still sits
# at offset 56. The exposure anchor `anchor_l99_sky` found no pad left, so it
# appended after `epoch` at offset 64 and GREW the struct to 72 (68 rounded up
# by the 8-byte alignment). `axis_solid_angle` then went into the 4 bytes of tail
# padding that rounding created, so the size is still 72. Matches the C++
# static_assert in test/unit-correctness/server/test_c_api.cpp.
class LUMICE_RawXyzResult(ctypes.Structure):
    _fields_ = [
        ("renderer_id",                ctypes.c_int),
        ("img_width",                  ctypes.c_int),
        ("img_height",                 ctypes.c_int),
        ("xyz_buffer",                 ctypes.POINTER(ctypes.c_float)),
        ("snapshot_intensity",         ctypes.c_float),
        ("intensity_factor",           ctypes.c_float),
        ("has_valid_data",             ctypes.c_int),
        ("snapshot_generation",        ctypes.c_uint64),
        ("effective_pixels",           ctypes.c_int),
        ("emitted_energy",             ctypes.c_float),
        ("epoch",                      ctypes.c_uint64),
        ("anchor_l99_sky",             ctypes.c_float),
        ("axis_solid_angle",           ctypes.c_float),
    ]


assert ctypes.sizeof(LUMICE_RawXyzResult) == 72, (
    "LUMICE_RawXyzResult size mismatch — verify lumice.h field layout"
)

# Field OFFSETS, not just the total size. A field inserted at the wrong index
# keeps the size identical and shifts everything after it, so every later field
# silently reads a neighbour's bytes — no exception, just wrong numbers. These
# mirror the offsetof static_asserts in
# test/unit-correctness/server/test_c_api.cpp; the pair is what makes "the
# mirror agrees with the header" a checked claim on both sides.
for _name, _offset in (
    ("snapshot_intensity", 24),
    ("snapshot_generation", 40),
    ("effective_pixels", 48),
    ("emitted_energy", 52),
    ("epoch", 56),
    ("anchor_l99_sky", 64),
    ("axis_solid_angle", 68),
):
    _actual = getattr(LUMICE_RawXyzResult, _name).offset
    assert _actual == _offset, (
        f"LUMICE_RawXyzResult.{_name} at offset {_actual}, expected {_offset} — "
        "the ctypes mirror and lumice.h disagree on field order"
    )


# Mirrors LUMICE_RenderResult in src/include/lumice.h. task-345.3 grew this
# struct by adding composite_p99_y (float at offset 24, 8-byte aligned = 32
# bytes total); the ctypes mirror must include it or LUMICE_FrameGetRender
# will overflow the out array by 8 bytes and corrupt the Python heap
# (task-cuda-ctypes-teardown-crash root cause). C++-side static_assert lives
# in test/unit-correctness/server/test_c_api.cpp.
class LUMICE_RenderResult(ctypes.Structure):
    _fields_ = [
        ("renderer_id",       ctypes.c_int),
        ("img_width",         ctypes.c_int),
        ("img_height",        ctypes.c_int),
        ("img_buffer",        ctypes.POINTER(ctypes.c_ubyte)),
        ("composite_p99_y",   ctypes.c_float),
    ]

assert ctypes.sizeof(LUMICE_RenderResult) == 32, (
    "LUMICE_RenderResult size mismatch — verify lumice.h field layout"
)


# Mirrors LUMICE_StatsResult in src/include/lumice.h. All four fields are
# LUMICE_RayCount = `unsigned long long` (64-bit on every platform, unlike
# `unsigned long` on Windows — see the static_assert next to the typedef).
class LUMICE_StatsResult(ctypes.Structure):
    _fields_ = [
        ("ray_seg_num",      ctypes.c_ulonglong),
        ("sim_ray_num",      ctypes.c_ulonglong),
        ("crystal_num",      ctypes.c_ulonglong),
        ("orientation_num",  ctypes.c_ulonglong),
    ]


def _assert_stats_mirror_matches_header() -> None:
    """Cross-check this mirror against the field list in lumice.h.

    A plain ``assert ctypes.sizeof(...) == N`` — which is what guarded this
    struct until orientation_num was added — compares the mirror to a number
    typed next to it, so it cannot notice the C struct growing underneath: both
    sides of the comparison live in this file. It stayed green while the C side
    went to four fields, and the failure it let through is not a wrong assertion
    but a heap overflow: LUMICE_FrameGetStats writes sizeof(C struct) into storage
    Python sized from the mirror, so a stale mirror means the library writes past the
    end of the buffer. Read the header instead, so the next added field turns this red
    at import time rather than corrupting memory in whichever test runs first.
    """
    header = Path(__file__).resolve().parents[2] / "src" / "include" / "lumice.h"
    if not header.is_file():  # source tree not available (e.g. installed wheel)
        return
    body = re.search(
        r"typedef struct LUMICE_StatsResult_\s*\{(.*?)\}\s*LUMICE_StatsResult;",
        header.read_text(encoding="utf-8"),
        re.DOTALL,
    )
    assert body is not None, "could not locate LUMICE_StatsResult in lumice.h"
    # Field declarations only: strip // comments, then take `<type> <name>;`.
    decls = re.sub(r"//.*", "", body.group(1))
    header_fields = re.findall(r"LUMICE_RayCount\s+(\w+)\s*;", decls)
    mirror_fields = [name for name, _ in LUMICE_StatsResult._fields_]
    assert header_fields == mirror_fields, (
        f"LUMICE_StatsResult drift — lumice.h has {header_fields}, "
        f"this mirror has {mirror_fields}. Update the mirror (and any code "
        f"reading the struct) before the C API writes past the Python buffer."
    )


_assert_stats_mirror_matches_header()


# Mirrors LUMICE_DrainResult in src/include/lumice.h. Both fields are
# `unsigned long long`; the current epoch is fully drained iff they are equal.
class LUMICE_DrainResult(ctypes.Structure):
    _fields_ = [
        ("drained_epoch",  ctypes.c_ulonglong),
        ("current_epoch",  ctypes.c_ulonglong),
    ]


assert ctypes.sizeof(LUMICE_DrainResult) == 16, (
    "LUMICE_DrainResult size mismatch — verify lumice.h field layout"
)

# Backend constants (lumice.h:391-392).
LUMICE_BACKEND_CPU = 0
LUMICE_BACKEND_METAL = 1
LUMICE_BACKEND_CUDA = 2


# Mirrors LUMICE_ServerConfig in src/include/lumice.h. Must include
# preferred_backend or LUMICE_CreateServerEx will read 4 bytes past the
# ctypes-allocated struct (undefined behavior; contributed to the ctypes
# teardown crash root cause).
class LUMICE_ServerConfig(ctypes.Structure):
    _fields_ = [
        ("num_workers",       ctypes.c_int),
        ("sim_seed",          ctypes.c_uint),
        ("preferred_backend", ctypes.c_int),
    ]


assert ctypes.sizeof(LUMICE_ServerConfig) == 12, (
    "LUMICE_ServerConfig size mismatch — verify lumice.h field layout"
)


# ---- Raypath analysis run (lumice.h "Raypath Analysis Run", v4.29) ----
# Sizes and offsets below are pinned to the C side twice: here against numbers measured
# from the header, and in test/unit-correctness/server/test_c_api_raypath_analysis.cpp
# by static_assert on the same numbers, so a field added on either side turns one of the
# two red before the C library writes past a Python buffer.
LUMICE_RAYPATH_ROI_FULL_SKY = 0
LUMICE_RAYPATH_ROI_IN_FRAME = 1
LUMICE_RAYPATH_ROI_CONE = 2
LUMICE_RAYPATH_SYMMETRY_SESSION_DEFAULT = 0xFF
LUMICE_MAX_RAYPATH_CHAIN_LAYERS = 8
LUMICE_MAX_RAYPATH_SEGMENT_LEN = 64
LUMICE_MAX_RAYPATH_CONE_RINGS = 32
LUMICE_RAYPATH_DISPLAY_MAX = 3200


# Mirrors LUMICE_AnnotationView (the IN_FRAME request's frame, and LUMICE_UnprojectPixel's view).
class LUMICE_AnnotationView(ctypes.Structure):
    _fields_ = [
        ("width",          ctypes.c_int),
        ("height",         ctypes.c_int),
        ("lens_type",      ctypes.c_int),
        ("lens_fov",       ctypes.c_float),
        ("lens_shift",     ctypes.c_int * 2),
        ("overlap",        ctypes.c_float),
        ("view_azimuth",   ctypes.c_float),
        ("view_elevation", ctypes.c_float),
        ("view_roll",      ctypes.c_float),
        ("visible",        ctypes.c_int),
        ("front",          ctypes.c_int),
    ]


assert ctypes.sizeof(LUMICE_AnnotationView) == 48, (
    "LUMICE_AnnotationView size mismatch — verify lumice.h field layout"
)


class LUMICE_RaypathAnalysisRequest(ctypes.Structure):
    _fields_ = [
        ("roi_mode",          ctypes.c_int),
        ("frame_view",        LUMICE_AnnotationView),
        ("cone_center",       ctypes.c_float * 3),
        ("cone_radius_rad",   ctypes.c_float),
        ("cone_ring_count",   ctypes.c_int),
        ("cone_stop_target",  ctypes.c_ulonglong),
        ("chain_id_symmetry", ctypes.c_int),
    ]


assert ctypes.sizeof(LUMICE_RaypathAnalysisRequest) == 88, (
    "LUMICE_RaypathAnalysisRequest size mismatch — verify lumice.h field layout"
)
for _name, _offset in (("frame_view", 4), ("cone_center", 52), ("cone_stop_target", 72), ("chain_id_symmetry", 80)):
    assert getattr(LUMICE_RaypathAnalysisRequest, _name).offset == _offset, (
        f"LUMICE_RaypathAnalysisRequest.{_name} offset drift — the mirror and lumice.h disagree"
    )


class LUMICE_RaypathChainSegment(ctypes.Structure):
    _fields_ = [
        ("crystal_id",  ctypes.c_int),
        ("segment",     ctypes.c_int * LUMICE_MAX_RAYPATH_SEGMENT_LEN),
        ("segment_len", ctypes.c_int),
    ]


class LUMICE_RaypathHistogramEntry(ctypes.Structure):
    _fields_ = [
        ("chain",       LUMICE_RaypathChainSegment * LUMICE_MAX_RAYPATH_CHAIN_LAYERS),
        ("chain_len",   ctypes.c_int),
        ("display",     ctypes.c_char * LUMICE_RAYPATH_DISPLAY_MAX),
        ("energy",      ctypes.c_double),
        ("count",       ctypes.c_ulonglong),
        ("ring_energy", ctypes.c_double * LUMICE_MAX_RAYPATH_CONE_RINGS),
        ("ring_count",  ctypes.c_int),
    ]


assert ctypes.sizeof(LUMICE_RaypathChainSegment) == 264
assert ctypes.sizeof(LUMICE_RaypathHistogramEntry) == 5600, (
    "LUMICE_RaypathHistogramEntry size mismatch — verify lumice.h field layout"
)
for _name, _offset in (("chain_len", 2112), ("display", 2116), ("energy", 5320), ("count", 5328),
                       ("ring_energy", 5336), ("ring_count", 5592)):
    assert getattr(LUMICE_RaypathHistogramEntry, _name).offset == _offset, (
        f"LUMICE_RaypathHistogramEntry.{_name} offset drift — the mirror and lumice.h disagree"
    )


class LUMICE_RaypathAnalysisInfo(ctypes.Structure):
    _fields_ = [
        ("present",         ctypes.c_int),
        ("roi_mode",        ctypes.c_int),
        ("entry_count",     ctypes.c_int),
        ("cone_ring_count", ctypes.c_int),
        ("cone_radius_rad", ctypes.c_float),
    ]


assert ctypes.sizeof(LUMICE_RaypathAnalysisInfo) == 20


@dataclass
class RaypathHistogramEntry:
    """One chain of an analysis result, copied out of the C entry (no C memory referenced)."""

    display: str
    chain: List[tuple]  # [(crystal_id, [faces...]), ...] root first
    energy: float
    count: int
    ring_energy: List[float]


@dataclass
class RaypathAnalysisResult:
    roi_mode: int
    entries: List[RaypathHistogramEntry]  # energy descending, as the C API orders them
    active_backend: int                   # LUMICE_GetActiveBackend during the run
    sim_ray_num: int                      # LUMICE_FrameGetStats on the same frame
    log_lines: List[str] = field(default_factory=list)


# LUMICE_ServerState constants (lumice.h)
# Drain-wait bounds for _read_sample_counts: how long to wait for the server's drain
# signal after it reports IDLE (see the comment there). Timeout FAILS the read rather
# than returning a partial total.
_DRAIN_POLL_SEC = 0.01
_DRAIN_TIMEOUT_SEC = 30.0

_LUMICE_SERVER_IDLE = 0
_LUMICE_SERVER_RUNNING = 1
_LUMICE_SERVER_NOT_READY = 2


@dataclass
class SimResult:
    """Subset of LUMICE_RawXyzResult fields exposed to test code.

    `crystal_num` and `orientation_num` come from LUMICE_StatsResult (a
    different C API call), read once after the run reached
    IDLE-with-valid-data: how many distinct crystal geometries, and how many
    crystal orientations, the run actually drew. Each has two halves that
    aggregate differently — the deterministic population is a config constant
    carried by OVERWRITE, the stochastic draws accumulate per batch and per
    worker — so both are only meaningful once the simulation finished. The two
    are independent quantities, not a rescaling of each other: a scene of fixed
    shapes under random axes reports a tiny crystal_num and a huge
    orientation_num. See doc/c_api.md for the contract.
    """

    snapshot_intensity: float
    has_valid_data: bool
    effective_pixels: int
    # Raw total energy the light source emitted into this snapshot — the
    # denominator the renderer normalizes by. Not a rescaling of
    # snapshot_intensity above: that one measures what landed on a pixel, this
    # one what went in, and they differ by everything that removes a ray.
    emitted_energy: float = 0.0
    # The session's exposure anchor: P99 sky radiance per steradian, measured on
    # a fixed full-sky buffer rather than on this renderer's output. A property
    # of the scene, so it is the same on every row of one frame — see the field's
    # contract in lumice.h.
    anchor_l99_sky: float = 0.0
    # On-axis per-pixel solid angle of THIS renderer's view, steradians. The unit bridge
    # between anchor_l99_sky (a radiance) and the pixel buffer (a radiance times a pixel's
    # solid angle); see LUMICE_RawXyzResult in lumice.h.
    axis_solid_angle: float = 0.0
    crystal_num: int = 0
    orientation_num: int = 0


@dataclass
class BufferedSimResult:
    """SimResult plus copied XYZ + rendered RGB buffers and backend routing.

    `routed_backend` is parsed from the C-core log stream (captured via
    LUMICE_SetLogCallback). Values: "metal" / "cpu_backend" / "legacy" / "" if
    no routing line was emitted (legacy default path is silent).

    `fell_back` is True if any "falling back" warning was observed while
    running this server — this is how the test asserts Metal/Cpu didn't
    silently degrade to legacy.
    """

    snapshot_intensity: float
    has_valid_data: bool
    effective_pixels: int
    img_width: int
    img_height: int
    flt_buf: np.ndarray
    rgb_buf: np.ndarray  # (H, W, 3) uint8 sRGB rendered image
    routed_backend: str = ""
    fell_back: bool = False
    log_lines: List[str] = field(default_factory=list)
    # See SimResult.emitted_energy — same field, same contract.
    emitted_energy: float = 0.0
    # See SimResult.anchor_l99_sky — same field, same contract.
    anchor_l99_sky: float = 0.0
    # See SimResult.axis_solid_angle — same field, same contract.
    axis_solid_angle: float = 0.0
    crystal_num: int = 0
    orientation_num: int = 0


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def lib_candidates(root: Path, build_type: str = "Release") -> List[Path]:
    """The paths `_find_lib` searches, in load order.

    Split out of `_find_lib` and made public so `scripts/test.sh` can *read* this
    list instead of keeping a hand-maintained copy of it. The pr scope's
    shared-library freshness check has to look in the same places this loader
    does; a copy is a second source of truth that goes stale silently, and a
    stale copy fails in the worst direction — it decides the library is missing
    or fresh by looking somewhere the tests never load from, and reports success
    either way. Keep this function importable with no side effects: the shell
    reads it through `python3 -c`, so an import that builds, loads or logs
    anything would run on every pr-scope invocation.

    `build_type` is a parameter rather than a constant because the shell knows
    which build type its static tree was configured with, while `_find_lib`
    itself has only the Release default to go on.
    """
    return [
        # Every candidate is under the "shared" flavor: this runner loads the
        # dylib through ctypes, which only exists in a BUILD_SHARED_LIBS=ON build.
        # A static build writes to .../static/ and is correctly not found here.
        # And every candidate is the TEST library, never liblumice -- see the
        # module docstring for why the two are not interchangeable in a test
        # process even though every product call behaves the same in both.
        root / "build" / build_type / "shared" / "lib" / "liblumice_testapi.dylib",
        root / "build" / build_type / "shared" / "lib" / "liblumice_testapi.so",
        root / "build" / "cmake_install" / "shared" / "liblumice_testapi.dylib",
        root / "build" / "cmake_install" / "shared" / "liblumice_testapi.so",
        root / "build" / "cmake_install" / "shared" / "lib" / "liblumice_testapi.dylib",
        root / "build" / "cmake_install" / "shared" / "lib" / "liblumice_testapi.so",
        root / "build" / "cmake_build" / "shared" / "liblumice_testapi.dylib",
        root / "build" / "cmake_build" / "shared" / "liblumice_testapi.so",
    ]


def _announce_chosen_lib(path: Path) -> None:
    """Print which shared library was picked, and how old it is.

    `scripts/build.sh -k` deletes `build/cmake_build/<flavor>` and
    `build/cmake_install/<flavor>` but NOT the compiler output tree
    `build/<BUILD_TYPE>/<flavor>/` — and `build/Release/shared/lib/liblumice_testapi.dylib`
    is the FIRST candidate `lib_candidates` returns. So "I cleaned" can be followed
    by ctypes loading a dylib from before the clean, with nothing on screen saying
    so: a stale library produces a coherent-looking pass or a failure blamed on the
    source you are editing.

    This is a read-out, not a guard: it does not decide anything, it just puts the
    path and mtime where a human comparing them against their last build can see
    them. Deliberately NOT a freshness check that fails the run — the loader has no
    reliable "should be newer than X" reference to check against (`scripts/test.sh
    pr` owns that comparison, and it has the build tree's configured type to go on).
    Changing `-k` to clean the artifact tree would be the real fix; it is a separate
    change with its own semantics to settle (it has to guess across
    `Debug|Release|MinSizeRel`), and `scripts/build.sh --help` now says outright that
    `-k` leaves this tree alone.
    """
    try:
        mtime = datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")
    except OSError:
        mtime = "unknown"
    print(f"[capi_runner] loading {path} (built {mtime})")


def _find_lib() -> Path:
    env_lib = os.environ.get("LUMICE_LIB")
    if env_lib:
        p = Path(env_lib)
        if not p.exists():
            raise FileNotFoundError(f"LUMICE_LIB={env_lib} does not exist")
        _announce_chosen_lib(p)
        return p

    for c in lib_candidates(_project_root()):
        if c.exists():
            _announce_chosen_lib(c)
            return c
    raise FileNotFoundError(
        "liblumice_testapi shared library not found (the test-only superset of liblumice; "
        "liblumice itself is deliberately not a candidate). Build the shared flavor with "
        "./scripts/build.sh -sj release, or set LUMICE_LIB to the absolute path "
        "(lumice_testapi.dll on Windows)."
    )


# Module-level singleton; safe for single-process sequential or fork-parallel execution;
# not thread-safe on first load (double-checked load pattern has a race window).
_LIB_CACHE: Optional[ctypes.CDLL] = None


# Log callback prototype matches LUMICE_LogCallback in lumice.h:105.
# Signature: void(level, logger_name, message). Defined here (not inside
# _load_lib) so the type object is stable across calls — the C-core retains
# the function-pointer cast and a per-call rebind would re-trigger the cast.
_LogCallbackProto = ctypes.CFUNCTYPE(
    None,
    ctypes.c_int,
    ctypes.c_char_p,
    ctypes.c_char_p,
)


def _load_lib() -> ctypes.CDLL:
    global _LIB_CACHE
    if _LIB_CACHE is not None:
        return _LIB_CACHE

    lib = ctypes.CDLL(str(_find_lib()))

    lib.LUMICE_CreateServer.restype = ctypes.c_void_p
    lib.LUMICE_CreateServer.argtypes = []

    lib.LUMICE_CreateServerEx.restype = ctypes.c_void_p
    lib.LUMICE_CreateServerEx.argtypes = [ctypes.POINTER(LUMICE_ServerConfig)]

    lib.LUMICE_DestroyServer.restype = None
    lib.LUMICE_DestroyServer.argtypes = [ctypes.c_void_p]

    # Scene (opaque handle) config path — the only C API surface that commits a config.
    lib.LUMICE_SceneFromJsonFile.restype = ctypes.c_int
    lib.LUMICE_SceneFromJsonFile.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]

    lib.LUMICE_CommitScene.restype = ctypes.c_int
    lib.LUMICE_CommitScene.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]

    lib.LUMICE_SceneDestroy.restype = None
    lib.LUMICE_SceneDestroy.argtypes = [ctypes.c_void_p]

    lib.LUMICE_QueryServerState.restype = ctypes.c_int
    lib.LUMICE_QueryServerState.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]

    lib.LUMICE_GetDrainStatus.restype = ctypes.c_int
    lib.LUMICE_GetDrainStatus.argtypes = [ctypes.c_void_p, ctypes.POINTER(LUMICE_DrainResult)]

    # Result frame: an opaque handle, so it maps to a bare c_void_p and there is no
    # layout to mirror. Only the three value structs the FrameGet* functions fill still
    # have Python mirrors, and their fields are unchanged by the frame API.
    lib.LUMICE_AcquireResultFrame.restype = ctypes.c_int
    lib.LUMICE_AcquireResultFrame.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]

    lib.LUMICE_ReleaseResultFrame.restype = None
    lib.LUMICE_ReleaseResultFrame.argtypes = [ctypes.c_void_p]

    lib.LUMICE_FrameGetRawXyz.restype = ctypes.c_int
    lib.LUMICE_FrameGetRawXyz.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(LUMICE_RawXyzResult),
        ctypes.c_int,
    ]

    lib.LUMICE_FrameGetStats.restype = ctypes.c_int
    lib.LUMICE_FrameGetStats.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(LUMICE_StatsResult),
    ]

    lib.LUMICE_FrameGetRender.restype = ctypes.c_int
    lib.LUMICE_FrameGetRender.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(LUMICE_RenderResult),
        ctypes.c_int,
    ]

    lib.LUMICE_SetPreferredBackend.restype = None
    lib.LUMICE_SetPreferredBackend.argtypes = [ctypes.c_void_p, ctypes.c_int]

    lib.LUMICE_SetLogCallback.restype = None
    lib.LUMICE_SetLogCallback.argtypes = [_LogCallbackProto]

    lib.LUMICE_StopServer.restype = None
    lib.LUMICE_StopServer.argtypes = [ctypes.c_void_p]

    # Raypath analysis run (v4.29).
    lib.LUMICE_StartRaypathAnalysis.restype = ctypes.c_int
    lib.LUMICE_StartRaypathAnalysis.argtypes = [ctypes.c_void_p, ctypes.POINTER(LUMICE_RaypathAnalysisRequest)]

    lib.LUMICE_FrameGetRaypathAnalysisInfo.restype = ctypes.c_int
    lib.LUMICE_FrameGetRaypathAnalysisInfo.argtypes = [ctypes.c_void_p, ctypes.POINTER(LUMICE_RaypathAnalysisInfo)]

    lib.LUMICE_FrameGetRaypathAnalysis.restype = ctypes.c_int
    lib.LUMICE_FrameGetRaypathAnalysis.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(LUMICE_RaypathHistogramEntry),
        ctypes.c_int,
    ]

    lib.LUMICE_GetActiveBackend.restype = ctypes.c_int
    lib.LUMICE_GetActiveBackend.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]

    lib.LUMICE_UnprojectPixel.restype = ctypes.c_int
    lib.LUMICE_UnprojectPixel.argtypes = [
        ctypes.POINTER(LUMICE_AnnotationView),
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_float * 3,
        ctypes.POINTER(ctypes.c_int),
    ]

    _LIB_CACHE = lib
    return lib


# Module-level callback bookkeeping. The C-core retains the function pointer
# globally (lumice.h:107-109), so we register exactly once and route messages
# through a thread-safe dispatcher to the currently-active capture (or None).
_LOG_LOCK = threading.Lock()
_ACTIVE_LOG_SINK: Optional[List[str]] = None
_LOG_CB_PTR = None  # type: ignore[var-annotated]


def _log_dispatch(level: int, logger_name: bytes, message: bytes) -> None:
    """C log callback — appends decoded "logger: message" lines to the active sink."""
    try:
        name = logger_name.decode("utf-8", "replace") if logger_name else ""
        msg = message.decode("utf-8", "replace") if message else ""
        line = f"{name}: {msg}"
    except Exception:
        return
    with _LOG_LOCK:
        sink = _ACTIVE_LOG_SINK
        if sink is not None:
            sink.append(line)


def _ensure_log_callback_registered(lib: ctypes.CDLL) -> None:
    """Register the log dispatch callback once. The C-core retains the pointer."""
    global _LOG_CB_PTR
    if _LOG_CB_PTR is None:
        _LOG_CB_PTR = _LogCallbackProto(_log_dispatch)
        lib.LUMICE_SetLogCallback(_LOG_CB_PTR)


class _LogCapture:
    """Context manager that routes core log lines into a per-call list."""

    def __init__(self) -> None:
        self.lines: List[str] = []

    def __enter__(self) -> List[str]:
        global _ACTIVE_LOG_SINK
        with _LOG_LOCK:
            # Pytest runs serially by contract for this suite — nested capture
            # is a programming error.
            if _ACTIVE_LOG_SINK is not None:
                raise RuntimeError("nested LogCapture is not supported")
            _ACTIVE_LOG_SINK = self.lines
        return self.lines

    def __exit__(self, *_) -> None:
        global _ACTIVE_LOG_SINK
        with _LOG_LOCK:
            _ACTIVE_LOG_SINK = None


# Patterns matching the routing log lines in simulator.cpp:520-537.
_RE_ROUTED_METAL = re.compile(r"routing via MetalTraceBackend")
_RE_ROUTED_CPU_BACKEND = re.compile(r"routing via CpuTraceBackend")
_RE_ROUTED_CUDA = re.compile(r"routing via CudaTraceBackend")
_RE_FALLBACK = re.compile(r"falling back", re.IGNORECASE)


def _summarize_backend(lines: List[str]) -> tuple[str, bool]:
    """Return (routed_backend, fell_back) parsed from captured log lines.

    routed_backend ∈ {"metal", "cpu_backend", "cuda", "legacy"}; "legacy"
    means no routing line was seen (legacy path is silent in CreateBackend).
    """
    routed = "legacy"
    fell_back = False
    for ln in lines:
        if _RE_ROUTED_METAL.search(ln):
            routed = "metal"
        elif _RE_ROUTED_CPU_BACKEND.search(ln):
            routed = "cpu_backend"
        elif _RE_ROUTED_CUDA.search(ln):
            routed = "cuda"
        if _RE_FALLBACK.search(ln):
            fell_back = True
    return routed, fell_back


@contextlib.contextmanager
def _result_frame(lib, server):
    """Acquire a result frame, yield the handle, release it on the way out.

    The C contract is a plain acquire/release pair (lumice.h). Python gets a context
    manager for the same reason the C++ tests get a scoped holder: an exception raised
    between the two calls would otherwise skip the release. Every field the FrameGet*
    functions hand back points into the frame, so any reading of those fields belongs
    INSIDE the `with` — the pointers are only guaranteed while the frame is held.
    """
    frame = ctypes.c_void_p()
    err = lib.LUMICE_AcquireResultFrame(server, ctypes.byref(frame))
    if err != 0:
        raise RuntimeError(f"AcquireResultFrame failed err={err}")
    try:
        yield frame
    finally:
        lib.LUMICE_ReleaseResultFrame(frame)


def _read_sample_counts(lib, server) -> tuple:
    """Read (crystal_num, orientation_num) from LUMICE_StatsResult.

    One call rather than two: both come from the same stats struct on one frame, and
    acquiring a frame materializes a snapshot, so reading them separately would take
    two snapshots of a run that is supposed to be over.

    Call only after the polling loop observed has_valid_data AND IDLE. The value
    is the deterministic population (a config constant, OVERWRITTEN on the way
    through StatsConsumer) plus the stochastic draws (accumulated across batches
    and workers), so reading it mid-run returns a partial total: the stochastic
    half is still growing. Not a plain sum — the deterministic half deliberately
    does NOT scale with the batch count or the worker pool, which is the whole
    point of the split.
    Returns 0 when no stats row is available (no StatsConsumer output yet).
    """
    # Wait for the CONSUMER to report this epoch drained before reading anything.
    #
    # WHY THIS IS NEEDED: the polling loop above waits for LUMICE_SERVER_IDLE, but that
    # verdict is entirely PRODUCER-side — no simulator busy, no scenes pending, scene
    # generation done. None of those says the consumer has drained its queue. Meanwhile
    # crystal_num/orientation_num are running totals frozen at snapshot time, so a read
    # taken while batches are still queued returns a partial total — exactly what this
    # function's docstring warns about. It surfaced as an intermittent CI failure on
    # Linux (orientation_num 19616 vs 20000, a whole number of dispatch grains short).
    #
    # LUMICE_GetDrainStatus is the server's own answer to that question, published by the
    # consumer thread once every batch this epoch will ever produce has been consumed
    # (see doc/c_api.md). It replaces the "poll until two stats reads agree" heuristic
    # this function used to carry: that heuristic could only ever guess from the outside
    # whether more data was coming, and a slow enough producer would have satisfied it
    # mid-run. Waiting on the signal itself is what makes the read below correct rather
    # than probably-correct.
    deadline = time.time() + _DRAIN_TIMEOUT_SEC
    drain = LUMICE_DrainResult()
    while True:
        err = lib.LUMICE_GetDrainStatus(server, ctypes.byref(drain))
        if err != 0:
            raise RuntimeError(f"GetDrainStatus failed err={err}")
        if drain.drained_epoch == drain.current_epoch:
            break
        if time.time() > deadline:
            # Fail loudly. Reading anyway would turn "the epoch never drained" into a
            # green test carrying a partial total — strictly worse than the failure this
            # wait exists to prevent.
            raise RuntimeError(
                f"epoch {int(drain.current_epoch)} did not drain within "
                f"{_DRAIN_TIMEOUT_SEC}s after the server reported IDLE "
                f"(drained_epoch={int(drain.drained_epoch)}). Either the consumer is not "
                f"draining or the idle predicate fired while production was still running."
            )
        time.sleep(_DRAIN_POLL_SEC)

    stats = LUMICE_StatsResult()
    with _result_frame(lib, server) as frame:
        err = lib.LUMICE_FrameGetStats(frame, ctypes.byref(stats))
    if err != 0:
        raise RuntimeError(f"FrameGetStats failed err={err}")

    # A server holds at most one stats struct, so this is a single value rather than a
    # row array; sim_ray_num == 0 still means "nothing produced yet".
    if stats.sim_ray_num == 0:
        return 0, 0
    return int(stats.crystal_num), int(stats.orientation_num)


def _commit_config(lib, server, config_path: str) -> None:
    """Parse `config_path` into a LUMICE_Scene handle and commit it, then free the handle.

    Since v4.12 this is the whole story: LUMICE_SceneFromJsonFile + LUMICE_CommitScene is the
    only C API surface that carries a config to the server, so there is no path selector.
    """
    scene = ctypes.c_void_p()
    err = lib.LUMICE_SceneFromJsonFile(str(config_path).encode("utf-8"), ctypes.byref(scene))
    if err != 0:
        raise RuntimeError(f"SceneFromJsonFile failed err={err} config={config_path}")
    if not scene:
        raise RuntimeError(f"SceneFromJsonFile returned a NULL handle for {config_path}")
    try:
        # CommitScene deep-copies what it needs; the handle stays caller-owned.
        err = lib.LUMICE_CommitScene(server, scene, None)
        if err != 0:
            raise RuntimeError(f"CommitScene failed err={err} config={config_path}")
    finally:
        # NULL-safe no-op by contract, but the handle is non-NULL here by the check above; the
        # finally covers the commit-failure path, where the handle exists and must still be freed.
        lib.LUMICE_SceneDestroy(scene)


def _commit_and_wait_drained(lib, server, config_path: str, prev_epoch: int,
                             timeout_sec: int) -> int:
    """Commit `config_path` on an ALREADY-RUNNING server and block until the epoch it
    mints has been fully consumed. Returns that epoch.

    Only used for the NON-FINAL stages of a multi-config sequence — the final stage keeps
    the has_valid_data/IDLE predicate the single-config path always used, because that is
    the stage whose buffers get copied out.

    Why the drain signal and not IDLE: IDLE is a producer-side verdict (see
    _read_sample_counts' long comment for the measured consequence). Committing the next
    config while the previous epoch's batches are still queued would make the sequence
    stop saying what it claims to say — the point of a sequence fixture is that stage N+1
    starts after stage N really ran. drained_epoch == current_epoch is the server's own
    answer to exactly that question.

    Why the epoch is required to advance: the whole wait is keyed on it. A commit that did
    NOT mint a new epoch would leave the previous epoch's already-satisfied
    `drained == current` standing, and this function would return immediately having
    waited for nothing — a silently degenerate sequence. Fail loudly instead; a caller
    whose consecutive configs are not reset-causing has a fixture bug.

    An infinite `ray_num` never drains (production never ends), so it cannot be used for a
    non-final stage; the timeout message says so.
    """
    _commit_config(lib, server, str(config_path))

    drain = LUMICE_DrainResult()
    err = lib.LUMICE_GetDrainStatus(server, ctypes.byref(drain))
    if err != 0:
        raise RuntimeError(f"GetDrainStatus failed err={err}")
    minted = int(drain.current_epoch)
    if minted <= prev_epoch:
        raise RuntimeError(
            f"committing {config_path} did not mint a new epoch (epoch stayed at "
            f"{minted}; the previous stage was {prev_epoch}). Every sequence stage must "
            f"be a reset-causing commit, otherwise the drain wait below is a no-op."
        )

    deadline = time.time() + timeout_sec
    while True:
        err = lib.LUMICE_GetDrainStatus(server, ctypes.byref(drain))
        if err != 0:
            raise RuntimeError(f"GetDrainStatus failed err={err}")
        if int(drain.current_epoch) != minted:
            raise RuntimeError(
                f"epoch moved from {minted} to {int(drain.current_epoch)} while waiting "
                f"for {config_path} to drain — something else committed to this server."
            )
        if int(drain.drained_epoch) == minted:
            return minted
        if time.time() > deadline:
            raise RuntimeError(
                f"Timeout {timeout_sec}s waiting for {config_path} (epoch {minted}) to "
                f"drain (drained_epoch={int(drain.drained_epoch)}). A non-final sequence "
                f"stage must be a FINITE run: an infinite ray_num never drains."
            )
        time.sleep(_DRAIN_POLL_SEC)


def run_scene_capi(config_path: str, sim_seed: int = 0, timeout_sec: int = 180) -> SimResult:
    """Run a single Lumice simulation via the C API and return scalar intensity.

    Spawns a fresh server, commits ``config_path``, polls until valid data is
    available, copies scalar fields out, and destroys the server. The
    returned object does not reference any memory owned by the server.

    Args:
        config_path: absolute or repo-relative path to a JSON config.
        sim_seed: deterministic RNG seed (0 = random). Non-zero collapses to 1 worker.
        timeout_sec: maximum wall time to wait for the simulation.

    Raises:
        FileNotFoundError: if the shared library can't be located.
        RuntimeError: on C API errors or timeout without valid data.
    """
    lib = _load_lib()

    if sim_seed != 0:
        cfg = LUMICE_ServerConfig(num_workers=0, sim_seed=sim_seed)
        server = lib.LUMICE_CreateServerEx(ctypes.byref(cfg))
    else:
        server = lib.LUMICE_CreateServer()
    if not server:
        raise RuntimeError("LUMICE_CreateServer returned NULL")

    try:
        _commit_config(lib, server, str(config_path))

        results = (LUMICE_RawXyzResult * 1)()
        state_out = ctypes.c_int(0)
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start
            if elapsed > timeout_sec:
                raise RuntimeError(
                    f"Timeout {elapsed:.1f}s waiting for {config_path}"
                )

            with _result_frame(lib, server) as frame:
                err = lib.LUMICE_FrameGetRawXyz(frame, results, 1)
            if err != 0:
                raise RuntimeError(f"FrameGetRawXyz failed err={err}")

            err2 = lib.LUMICE_QueryServerState(server, ctypes.byref(state_out))
            if err2 != 0:
                raise RuntimeError(f"QueryServerState failed err={err2}")

            state = state_out.value
            if state == _LUMICE_SERVER_NOT_READY:
                raise RuntimeError("Server NOT_READY")

            if results[0].has_valid_data and state == _LUMICE_SERVER_IDLE:
                break

            time.sleep(0.2)

        r = results[0]
        crystal_num, orientation_num = _read_sample_counts(lib, server)
        return SimResult(
            snapshot_intensity=float(r.snapshot_intensity),
            has_valid_data=bool(r.has_valid_data),
            effective_pixels=int(r.effective_pixels),
            emitted_energy=float(r.emitted_energy),
            anchor_l99_sky=float(r.anchor_l99_sky),
            axis_solid_angle=float(r.axis_solid_angle),
            crystal_num=crystal_num,
            orientation_num=orientation_num,
        )

    finally:
        lib.LUMICE_DestroyServer(server)


def _wait_drained(lib, server, timeout_sec: float) -> None:
    """Block until the current epoch reports drained — the server's own "totals are final"
    signal (see _read_sample_counts for why IDLE alone is not it). Raises on timeout rather
    than returning a partial result."""
    deadline = time.time() + timeout_sec
    drain = LUMICE_DrainResult()
    while True:
        err = lib.LUMICE_GetDrainStatus(server, ctypes.byref(drain))
        if err != 0:
            raise RuntimeError(f"GetDrainStatus failed err={err}")
        if drain.drained_epoch == drain.current_epoch:
            return
        if time.time() > deadline:
            raise RuntimeError(
                f"epoch {int(drain.current_epoch)} did not drain within {timeout_sec}s "
                f"(drained_epoch={int(drain.drained_epoch)})"
            )
        time.sleep(_DRAIN_POLL_SEC)


def run_raypath_analysis_capi(
    config_path: str,
    request: LUMICE_RaypathAnalysisRequest,
    sim_seed: int = 0,
    num_workers: int = 0,
    preferred_backend: int = LUMICE_BACKEND_CPU,
    timeout_sec: int = 180,
    max_entries: int = 1024,
) -> RaypathAnalysisResult:
    """Run one ANALYSIS run via the C API on a fresh server and copy the histogram out.

    The lifecycle lumice.h describes, verbatim: create → commit `config_path` (which starts
    the render run every commit starts) → LUMICE_StopServer → LUMICE_StartRaypathAnalysis →
    wait for the drain signal → read one frame → destroy. The scene's ray_num is the run's
    budget (an "infinite" config only ends through a cone stop target, else this times out).

    `preferred_backend` goes to LUMICE_CreateServerEx; the analysis run is expected to
    ignore it (CPU is a session property), and `active_backend` in the result is what
    LUMICE_GetActiveBackend reported while the run was in progress, for the caller to assert on.
    Log lines are captured (the forced-CPU INFO line lives there).
    """
    lib = _load_lib()
    _ensure_log_callback_registered(lib)

    cfg = LUMICE_ServerConfig(num_workers=num_workers, sim_seed=sim_seed, preferred_backend=preferred_backend)
    with _LogCapture() as lines:
        server = lib.LUMICE_CreateServerEx(ctypes.byref(cfg))
        if not server:
            raise RuntimeError("LUMICE_CreateServerEx returned NULL")
        try:
            _commit_config(lib, server, str(config_path))
            lib.LUMICE_StopServer(server)
            err = lib.LUMICE_StartRaypathAnalysis(server, ctypes.byref(request))
            if err != 0:
                raise RuntimeError(f"StartRaypathAnalysis failed err={err}")
            active = ctypes.c_int(-1)
            err = lib.LUMICE_GetActiveBackend(server, ctypes.byref(active))
            if err != 0:
                raise RuntimeError(f"GetActiveBackend failed err={err}")
            _wait_drained(lib, server, timeout_sec)

            info = LUMICE_RaypathAnalysisInfo()
            stats = LUMICE_StatsResult()
            entries_c = (LUMICE_RaypathHistogramEntry * (max_entries + 1))()
            with _result_frame(lib, server) as frame:
                err = lib.LUMICE_FrameGetRaypathAnalysisInfo(frame, ctypes.byref(info))
                if err != 0:
                    raise RuntimeError(f"FrameGetRaypathAnalysisInfo failed err={err}")
                if not info.present:
                    raise RuntimeError("the frame carries no analysis result")
                err = lib.LUMICE_FrameGetRaypathAnalysis(frame, entries_c, max_entries)
                if err != 0:
                    raise RuntimeError(f"FrameGetRaypathAnalysis failed err={err}")
                err = lib.LUMICE_FrameGetStats(frame, ctypes.byref(stats))
                if err != 0:
                    raise RuntimeError(f"FrameGetStats failed err={err}")
            # Copied out INSIDE the frame's lifetime by contract; the entries are value
            # copies already, but reading them here keeps the rule uniform for every getter.
            n = min(int(info.entry_count), max_entries)
            entries = []
            for i in range(n):
                e = entries_c[i]
                chain = [
                    (int(e.chain[l].crystal_id), [int(e.chain[l].segment[f]) for f in range(e.chain[l].segment_len)])
                    for l in range(e.chain_len)
                ]
                entries.append(RaypathHistogramEntry(
                    display=e.display.decode("utf-8", "replace"),
                    chain=chain,
                    energy=float(e.energy),
                    count=int(e.count),
                    ring_energy=[float(e.ring_energy[r]) for r in range(e.ring_count)],
                ))
            return RaypathAnalysisResult(
                roi_mode=int(info.roi_mode),
                entries=entries,
                active_backend=int(active.value),
                sim_ray_num=int(stats.sim_ray_num),
                log_lines=list(lines),
            )
        finally:
            lib.LUMICE_DestroyServer(server)


_BACKEND_MODES = ("legacy", "metal", "cpu_backend", "cuda")


def run_scene_sequence_capi_buffered(
    config_paths: Sequence[str],
    sim_seed: int = 0,
    timeout_sec: int = 180,
    backend: str = "legacy",
    preserve_dispatch_env: bool = False,
    num_workers: int = 0,
) -> BufferedSimResult:
    """Commit `config_paths` in order on ONE server, and copy out the LAST one's buffers.

    This is the authoritative implementation; `run_scene_capi_buffered` is the
    single-config spelling of it and does nothing this function does not.

    A one-element sequence behaves exactly as a single run always did: create server,
    commit, poll to has_valid_data + IDLE (twice consecutively), copy out, destroy.

    With two or more, every stage but the last is committed and then waited on until the
    epoch it minted is fully drained (`_commit_and_wait_drained`), and only then is the
    next config committed. Two properties that only a sequence has, and that are the
    reason it exists rather than N separate calls:
      - the SERVER (and therefore the simulator thread, its backend lifetime, and any
        thread-local device state that outlives one Run) is shared across the stages, so
        a defect that leaks state from one Run into the next is reachable here and
        structurally unreachable from N single-config runs;
      - the log capture spans the WHOLE sequence, so `fell_back` / `routed_backend`
        answer for the entire session, not just for its final stage.

    Every non-final stage must be a FINITE run (an infinite `ray_num` never drains) and
    must differ from its predecessor enough to be a reset-causing commit; both are
    enforced, loudly, by `_commit_and_wait_drained`.

    `backend` selects the trace path:
      - "legacy"     : no env, preferred_backend = LUMICE_BACKEND_CPU. The C-API
                       server default and the ground-truth in 258.6.
      - "metal"      : no env, preferred_backend = LUMICE_BACKEND_METAL. Must NOT
                       set LUMICE_TRACE_BACKEND (env has higher priority — see
                       simulator.cpp:513 CreateBackend).
      - "cpu_backend": env LUMICE_TRACE_BACKEND=cpu_backend (env overrides
                       SetPreferredBackend).
      - "cuda"       : env LUMICE_TRACE_BACKEND=cuda (env overrides
                       SetPreferredBackend). Requires LUMICE_CUDA_ENABLED=ON
                       build + NVIDIA device on the host.

    Concurrency contract: this suite runs serially under pytest (no xdist).
    os.environ writes + LogCapture are not safe for parallel workers — adding
    parallelism here requires moving to subprocess isolation.

    Polling exits only after `has_valid_data AND IDLE` is observed on two
    consecutive samples. Buffers are copied into owned numpy arrays before
    destroying the server; the returned object holds no server-memory refs.

    `routed_backend` and `fell_back` are parsed from the captured core log;
    callers asserting "Metal really ran" must check both
    (routed_backend == "metal" and not fell_back).

    `preserve_dispatch_env` opts out of the LUMICE_DISPATCH_RAY_NUM strip that
    the legacy arm normally gets (rationale in the comment below). Pass True
    only when varying the dispatch grain on the legacy arm IS the measurement —
    the strip protects callers whose observable (energy) is not dispatch-
    invariant on legacy, which does not apply to a caller asserting invariance
    of a different observable.

    `num_workers` pins the CPU-route worker pool (0 = the shipped default: the
    physical core count, capped — see kMaxDefaultWorkerCount in server.cpp). It is honoured independently of `sim_seed`: a caller
    that pins a seed already gets one worker (server.cpp clamps the
    deterministic CPU contract to a single simulator), so sweeping this knob is
    only meaningful at `sim_seed == 0`. The GPU route ignores it (single
    engine).
    """
    if backend not in _BACKEND_MODES:
        raise ValueError(f"backend must be one of {_BACKEND_MODES}, got {backend!r}")
    config_paths = [str(c) for c in config_paths]
    if not config_paths:
        raise ValueError("config_paths must contain at least one config")
    # Every scalar/buffer this function returns comes from the last stage; the messages
    # below name it so a failure points at the config that was actually being polled.
    final_config = config_paths[-1]

    lib = _load_lib()
    _ensure_log_callback_registered(lib)

    # cpu_backend uses env; legacy/metal must not have env set (env overrides
    # SetPreferredBackend in CreateBackend, simulator.cpp:516-532).
    env_was_set = "LUMICE_TRACE_BACKEND" in os.environ
    env_old = os.environ.get("LUMICE_TRACE_BACKEND")
    if backend == "cpu_backend":
        os.environ["LUMICE_TRACE_BACKEND"] = "cpu_backend"
    elif backend == "cuda":
        os.environ["LUMICE_TRACE_BACKEND"] = "cuda"
    elif env_was_set:
        # Caller's env would override our SetPreferredBackend — strip it.
        del os.environ["LUMICE_TRACE_BACKEND"]

    # scrum-306.4: LUMICE_DISPATCH_RAY_NUM is a GPU-engine dispatch-sizing knob.
    # The legacy (CPU) parity oracle's total energy is NOT invariant to it
    # (explore-306.1: legacy Y swings −5%..+13% across dispatch sizes; a separate
    # legacy bug tracked in scrum-306.7). When a dev sets LUMICE_DISPATCH_RAY_NUM
    # globally to probe a GPU backend at a large dispatch, it leaks into the legacy
    # reference run and inflates legacy_Y → a FALSE energy_ratio failure that was
    # historically misattributed to a CUDA "silent energy loss". Pin the oracle to
    # its canonical default by stripping the knob for the legacy run so the ratio
    # reflects the GPU backend's correctness alone.
    disp_was_set = "LUMICE_DISPATCH_RAY_NUM" in os.environ
    disp_old = os.environ.get("LUMICE_DISPATCH_RAY_NUM")
    if backend == "legacy" and disp_was_set and not preserve_dispatch_env:
        del os.environ["LUMICE_DISPATCH_RAY_NUM"]

    capture = _LogCapture()

    try:
        with capture as log_lines:
            if sim_seed != 0 or num_workers != 0:
                cfg = LUMICE_ServerConfig(num_workers=num_workers, sim_seed=sim_seed)
                server = lib.LUMICE_CreateServerEx(ctypes.byref(cfg))
            else:
                server = lib.LUMICE_CreateServer()
            if not server:
                raise RuntimeError("LUMICE_CreateServer returned NULL")

            try:
                if backend == "metal":
                    lib.LUMICE_SetPreferredBackend(server, LUMICE_BACKEND_METAL)
                elif backend == "legacy":
                    lib.LUMICE_SetPreferredBackend(server, LUMICE_BACKEND_CPU)
                # cpu_backend: env handles routing; preferred is ignored.

                # Non-final stages: commit, wait for that epoch to drain, move on. The
                # final stage falls through to the poll loop below, which is the
                # unchanged single-config predicate.
                stage_epoch = 0
                for stage_cfg in config_paths[:-1]:
                    stage_epoch = _commit_and_wait_drained(
                        lib, server, stage_cfg, stage_epoch, timeout_sec
                    )
                _commit_config(lib, server, final_config)

                results = (LUMICE_RawXyzResult * 1)()
                renders = (LUMICE_RenderResult * 1)()
                state_out = ctypes.c_int(0)
                t_start = time.time()
                consecutive_ok = 0

                # Render and xyz are read off ONE frame. The call order between them
                # used to matter — the xyz getter cleared snapshot_dirty_ without running
                # PostSnapshot, so a render read afterwards found nothing prepared — but a
                # frame is materialized once, by the acquire, and carries both.
                while True:
                    elapsed = time.time() - t_start
                    if elapsed > timeout_sec:
                        raise RuntimeError(
                            f"Timeout {elapsed:.1f}s waiting for {final_config} (backend={backend})"
                        )

                    # LUMICE_FrameGet* always returns LUMICE_OK (0) when args are
                    # non-null. The err checks are a safety net for future API additions.
                    with _result_frame(lib, server) as frame:
                        err = lib.LUMICE_FrameGetRender(frame, renders, 1)
                        if err != 0:
                            raise RuntimeError(f"FrameGetRender failed err={err}")

                        err = lib.LUMICE_FrameGetRawXyz(frame, results, 1)
                        if err != 0:
                            raise RuntimeError(f"FrameGetRawXyz failed err={err}")

                    err2 = lib.LUMICE_QueryServerState(server, ctypes.byref(state_out))
                    if err2 != 0:
                        raise RuntimeError(f"QueryServerState failed err={err2}")

                    state = state_out.value
                    if state == _LUMICE_SERVER_NOT_READY:
                        raise RuntimeError("Server NOT_READY")

                    if results[0].has_valid_data and state == _LUMICE_SERVER_IDLE:
                        consecutive_ok += 1
                        if consecutive_ok >= 2:
                            break
                    else:
                        consecutive_ok = 0

                    time.sleep(0.2)

                # Re-read under a freshly held frame: the poll loop released each
                # frame as it went (the `with _result_frame` block above exits every
                # iteration), so `results`/`renders` are stale by the time the loop
                # breaks — same fix as scripts/dump_xyz_stats.py::run_scene.
                with _result_frame(lib, server) as frame:
                    err = lib.LUMICE_FrameGetRender(frame, renders, 1)
                    if err != 0:
                        raise RuntimeError(f"FrameGetRender failed err={err}")
                    err = lib.LUMICE_FrameGetRawXyz(frame, results, 1)
                    if err != 0:
                        raise RuntimeError(f"FrameGetRawXyz failed err={err}")

                    r = results[0]
                    r_w = int(r.img_width)
                    r_h = int(r.img_height)
                    r_xyz_addr = ctypes.cast(r.xyz_buffer, ctypes.c_void_p).value
                    r_snap = float(r.snapshot_intensity)
                    r_valid = bool(r.has_valid_data)
                    r_eff = int(r.effective_pixels)
                    r_emitted = float(r.emitted_energy)
                    r_anchor = float(r.anchor_l99_sky)
                    r_axis_omega = float(r.axis_solid_angle)
                    if r_xyz_addr is None:
                        raise RuntimeError(
                            f"{final_config}: race — xyz pointer became NULL after IDLE check"
                        )

                    n_xyz = r_w * r_h * 3
                    flt_buf = (
                        np.frombuffer(
                            (ctypes.c_float * n_xyz).from_address(r_xyz_addr),
                            dtype=np.float32,
                        )
                        .copy()
                        .reshape(r_h, r_w, 3)
                        .astype(np.float64)
                    )

                    rr = renders[0]
                    rr_w = int(rr.img_width)
                    rr_h = int(rr.img_height)
                    rr_addr = ctypes.cast(rr.img_buffer, ctypes.c_void_p).value
                    if rr_addr is None or rr_w == 0 or rr_h == 0:
                        raise RuntimeError(
                            f"{final_config}: LUMICE_FrameGetRender returned empty buffer"
                        )
                    # img_buffer is packed RGB uint8 (3 bytes/pixel, sRGB); per lumice.h:262.
                    n_rgb = rr_w * rr_h * 3
                    rgb_buf = (
                        np.frombuffer(
                            (ctypes.c_ubyte * n_rgb).from_address(rr_addr),
                            dtype=np.uint8,
                        )
                        .copy()
                        .reshape(rr_h, rr_w, 3)
                    )

                crystal_num, orientation_num = _read_sample_counts(lib, server)

            finally:
                lib.LUMICE_DestroyServer(server)

            # Log parsing happens AFTER teardown on purpose: ServerImpl::Stop()
            # (driven by DestroyServer) is what emits the RenderConsumer
            # "Consume profile: N batches" line, so a caller using the batch
            # count as a positive control would never see it if the snapshot
            # were taken before. Every buffer/scalar above was already copied
            # out of server memory, so nothing here touches the dead server —
            # and on the exception path this block is skipped entirely (the
            # finally re-raises), which is why it sits outside the try.
            routed, fell_back = _summarize_backend(log_lines)
            return BufferedSimResult(
                snapshot_intensity=r_snap,
                has_valid_data=r_valid,
                effective_pixels=r_eff,
                emitted_energy=r_emitted,
                anchor_l99_sky=r_anchor,
                axis_solid_angle=r_axis_omega,
                img_width=r_w,
                img_height=r_h,
                flt_buf=flt_buf,
                rgb_buf=rgb_buf,
                routed_backend=routed,
                fell_back=fell_back,
                log_lines=list(log_lines),
                crystal_num=crystal_num,
                orientation_num=orientation_num,
            )
    finally:
        # Restore env state regardless of success/failure.
        if backend in ("cpu_backend", "cuda"):
            if env_was_set:
                os.environ["LUMICE_TRACE_BACKEND"] = env_old
            else:
                os.environ.pop("LUMICE_TRACE_BACKEND", None)
        else:
            if env_was_set:
                os.environ["LUMICE_TRACE_BACKEND"] = env_old
        # scrum-306.4: restore LUMICE_DISPATCH_RAY_NUM (only the legacy branch
        # strips it; restore symmetrically regardless of backend).
        if disp_was_set:
            os.environ["LUMICE_DISPATCH_RAY_NUM"] = disp_old
        else:
            os.environ.pop("LUMICE_DISPATCH_RAY_NUM", None)


def run_scene_capi_buffered(
    config_path: str,
    sim_seed: int = 0,
    timeout_sec: int = 180,
    backend: str = "legacy",
    preserve_dispatch_env: bool = False,
    num_workers: int = 0,
) -> BufferedSimResult:
    """Run ONE config via the C API and copy out XYZ + RGB buffers.

    A one-element `run_scene_sequence_capi_buffered`, and nothing else — see that
    function for every argument's meaning and for the routed_backend / fell_back
    contract. Kept as a named entry point because the overwhelming majority of callers
    run one config and should not have to spell a list to say so; kept as a delegation
    rather than a copy because the alternative is two implementations of one semantics
    that drift apart on the first fix that lands in only one of them.
    """
    return run_scene_sequence_capi_buffered(
        [config_path],
        sim_seed=sim_seed,
        timeout_sec=timeout_sec,
        backend=backend,
        preserve_dispatch_env=preserve_dispatch_env,
        num_workers=num_workers,
    )
