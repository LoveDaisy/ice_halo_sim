"""ctypes-based runner for scalar-intensity e2e tests.

The standard subprocess-based runner in :mod:`test.e2e.runner` only exposes
return code / stdout / stderr; it has no way to read scalar fields like
``unfiltered_snapshot_intensity`` from ``LUMICE_RawXyzResult``. Tests that
need those values (notably the unfiltered intensity additivity tests added
for task-query-filter-uplift-v2) drive Lumice through the C API directly via
``ctypes``.

Each call to :func:`run_scene_capi` creates a fresh ``LUMICE_Server``,
commits the requested config, polls until the server returns to IDLE with
valid data (or the timeout fires), reads the scalar result, and destroys
the server. The 3 additivity simulations therefore share no state.

Library lookup order:
    1. ``LUMICE_LIB`` environment variable (full path to the shared library).
    2. ``build/Release/lib/liblumice.{dylib,so}``
    3. ``build/cmake_install/{liblumice.{dylib,so}, lib/liblumice.{dylib,so}}``
    4. ``build/cmake_build/liblumice.{dylib,so}``

The library must be built with ``BUILD_SHARED_LIBS=ON`` (the default release
recipe). If lookup fails, raises :class:`FileNotFoundError`.
"""

from __future__ import annotations

import ctypes
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# Mirrors LUMICE_RawXyzResult in src/include/lumice.h (verified 72 bytes
# on 64-bit macOS/Linux). Keep in sync with scripts/dump_xyz_stats.py:51.
class LUMICE_RawXyzResult(ctypes.Structure):
    _fields_ = [
        ("renderer_id",                   ctypes.c_int),
        ("img_width",                     ctypes.c_int),
        ("img_height",                    ctypes.c_int),
        ("xyz_buffer",                    ctypes.POINTER(ctypes.c_float)),
        ("snapshot_intensity",            ctypes.c_float),
        ("intensity_factor",              ctypes.c_float),
        ("has_valid_data",                ctypes.c_int),
        ("snapshot_generation",           ctypes.c_uint64),
        ("effective_pixels",              ctypes.c_int),
        ("unfiltered_xyz_buffer",         ctypes.POINTER(ctypes.c_float)),
        ("unfiltered_snapshot_intensity", ctypes.c_float),
    ]


assert ctypes.sizeof(LUMICE_RawXyzResult) == 72, (
    "LUMICE_RawXyzResult size mismatch — verify lumice.h field layout"
)


# LUMICE_ServerState constants (lumice.h)
_LUMICE_SERVER_IDLE = 0
_LUMICE_SERVER_RUNNING = 1
_LUMICE_SERVER_NOT_READY = 2


@dataclass
class SimResult:
    """Subset of LUMICE_RawXyzResult fields exposed to test code."""

    unfiltered_intensity: float
    snapshot_intensity: float
    has_valid_data: bool
    effective_pixels: int


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _find_lib() -> Path:
    env_lib = os.environ.get("LUMICE_LIB")
    if env_lib:
        p = Path(env_lib)
        if not p.exists():
            raise FileNotFoundError(f"LUMICE_LIB={env_lib} does not exist")
        return p

    root = _project_root()
    candidates = [
        root / "build" / "Release" / "lib" / "liblumice.dylib",
        root / "build" / "Release" / "lib" / "liblumice.so",
        root / "build" / "cmake_install" / "liblumice.dylib",
        root / "build" / "cmake_install" / "liblumice.so",
        root / "build" / "cmake_install" / "lib" / "liblumice.dylib",
        root / "build" / "cmake_install" / "lib" / "liblumice.so",
        root / "build" / "cmake_build" / "liblumice.dylib",
        root / "build" / "cmake_build" / "liblumice.so",
    ]
    for c in candidates:
        if c.exists():
            return c
    raise FileNotFoundError(
        "liblumice shared library not found. Build with BUILD_SHARED_LIBS=ON "
        "(./scripts/build.sh -j release), or set LUMICE_LIB to the absolute path."
    )


# Module-level singleton; safe for single-process sequential or fork-parallel execution;
# not thread-safe on first load (double-checked load pattern has a race window).
_LIB_CACHE: Optional[ctypes.CDLL] = None


def _load_lib() -> ctypes.CDLL:
    global _LIB_CACHE
    if _LIB_CACHE is not None:
        return _LIB_CACHE

    lib = ctypes.CDLL(str(_find_lib()))

    lib.LUMICE_CreateServer.restype = ctypes.c_void_p
    lib.LUMICE_CreateServer.argtypes = []

    lib.LUMICE_DestroyServer.restype = None
    lib.LUMICE_DestroyServer.argtypes = [ctypes.c_void_p]

    lib.LUMICE_CommitConfigFromFile.restype = ctypes.c_int
    lib.LUMICE_CommitConfigFromFile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]

    lib.LUMICE_QueryServerState.restype = ctypes.c_int
    lib.LUMICE_QueryServerState.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]

    lib.LUMICE_GetRawXyzResults.restype = ctypes.c_int
    lib.LUMICE_GetRawXyzResults.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(LUMICE_RawXyzResult),
        ctypes.c_int,
    ]

    _LIB_CACHE = lib
    return lib


def run_scene_capi(config_path: str, timeout_sec: int = 180) -> SimResult:
    """Run a single Lumice simulation via the C API and return scalar intensity.

    Spawns a fresh server, commits ``config_path``, polls until valid data is
    available, copies scalar fields out, and destroys the server. The
    returned object does not reference any memory owned by the server.

    Args:
        config_path: absolute or repo-relative path to a JSON config.
        timeout_sec: maximum wall time to wait for the simulation.

    Raises:
        FileNotFoundError: if the shared library can't be located.
        RuntimeError: on C API errors or timeout without valid data.
    """
    lib = _load_lib()

    server = lib.LUMICE_CreateServer()
    if not server:
        raise RuntimeError("LUMICE_CreateServer returned NULL")

    try:
        err = lib.LUMICE_CommitConfigFromFile(server, str(config_path).encode("utf-8"))
        if err != 0:
            raise RuntimeError(f"CommitConfigFromFile failed err={err} config={config_path}")

        results = (LUMICE_RawXyzResult * 1)()
        state_out = ctypes.c_int(0)
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start
            if elapsed > timeout_sec:
                raise RuntimeError(
                    f"Timeout {elapsed:.1f}s waiting for {config_path}"
                )

            err = lib.LUMICE_GetRawXyzResults(server, results, 1)
            if err != 0:
                raise RuntimeError(f"GetRawXyzResults failed err={err}")

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
        return SimResult(
            unfiltered_intensity=float(r.unfiltered_snapshot_intensity),
            snapshot_intensity=float(r.snapshot_intensity),
            has_valid_data=bool(r.has_valid_data),
            effective_pixels=int(r.effective_pixels),
        )

    finally:
        lib.LUMICE_DestroyServer(server)
