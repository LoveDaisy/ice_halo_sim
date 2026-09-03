"""ctypes-based runner for scalar-intensity e2e tests.

The standard subprocess-based runner in :mod:`test.e2e.runner` only exposes
return code / stdout / stderr; it has no way to read scalar fields like
``snapshot_intensity`` from ``LUMICE_RawXyzResult``. Tests that
need those values drive Lumice through the C API directly via ``ctypes``.

Each call to :func:`run_scene_capi` creates a fresh ``LUMICE_Server``,
commits the requested config, polls until the server returns to IDLE with
valid data (or the timeout fires), reads the scalar result, and destroys
the server.

Library lookup order:
    1. ``LUMICE_LIB`` environment variable (full path to the shared library).
    2. ``build/Release/shared/lib/liblumice.{dylib,so}``
    3. ``build/cmake_install/shared/{liblumice.{dylib,so}, lib/liblumice.{dylib,so}}``
    4. ``build/cmake_build/shared/liblumice.{dylib,so}``

The library must be built with ``BUILD_SHARED_LIBS=ON`` (the default release
recipe). If lookup fails, raises :class:`FileNotFoundError`.

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
from typing import List, Optional

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
        root / "build" / build_type / "shared" / "lib" / "liblumice.dylib",
        root / "build" / build_type / "shared" / "lib" / "liblumice.so",
        root / "build" / "cmake_install" / "shared" / "liblumice.dylib",
        root / "build" / "cmake_install" / "shared" / "liblumice.so",
        root / "build" / "cmake_install" / "shared" / "lib" / "liblumice.dylib",
        root / "build" / "cmake_install" / "shared" / "lib" / "liblumice.so",
        root / "build" / "cmake_build" / "shared" / "liblumice.dylib",
        root / "build" / "cmake_build" / "shared" / "liblumice.so",
    ]


def _announce_chosen_lib(path: Path) -> None:
    """Print which shared library was picked, and how old it is.

    `scripts/build.sh -k` deletes `build/cmake_build/<flavor>` and
    `build/cmake_install/<flavor>` but NOT the compiler output tree
    `build/<BUILD_TYPE>/<flavor>/` — and `build/Release/shared/lib/liblumice.dylib`
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
        "liblumice shared library not found. Build with BUILD_SHARED_LIBS=ON "
        "(./scripts/build.sh -j release), or set LUMICE_LIB to the absolute path."
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


_BACKEND_MODES = ("legacy", "metal", "cpu_backend", "cuda")


def run_scene_capi_buffered(
    config_path: str,
    sim_seed: int = 0,
    timeout_sec: int = 180,
    backend: str = "legacy",
    preserve_dispatch_env: bool = False,
    num_workers: int = 0,
) -> BufferedSimResult:
    """Run a Lumice sim via the C API and copy out XYZ + RGB buffers.

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

    `num_workers` pins the CPU-route worker pool (0 = the shipped default,
    PhysicalCoreCount()). It is honoured independently of `sim_seed`: a caller
    that pins a seed already gets one worker (server.cpp clamps the
    deterministic CPU contract to a single simulator), so sweeping this knob is
    only meaningful at `sim_seed == 0`. The GPU route ignores it (single
    engine).
    """
    if backend not in _BACKEND_MODES:
        raise ValueError(f"backend must be one of {_BACKEND_MODES}, got {backend!r}")

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

                _commit_config(lib, server, str(config_path))

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
                            f"Timeout {elapsed:.1f}s waiting for {config_path} (backend={backend})"
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
                            f"{config_path}: race — xyz pointer became NULL after IDLE check"
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
                            f"{config_path}: LUMICE_FrameGetRender returned empty buffer"
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
