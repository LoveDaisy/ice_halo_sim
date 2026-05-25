"""ctypes-based runner for scalar-intensity e2e tests.

The standard subprocess-based runner in :mod:`test.e2e.runner` only exposes
return code / stdout / stderr; it has no way to read scalar fields like
``anchor_snapshot_intensity`` from ``LUMICE_RawXyzResult``. Tests that
need those values drive Lumice through the C API directly via ``ctypes``.

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

import numpy as np


# Mirrors LUMICE_RawXyzResult in src/include/lumice.h (verified 64 bytes
# on 64-bit macOS/Linux after the F1 mode-toggle ABI break: the two
# unfiltered_* fields were replaced by scalar anchor_p995_y + anchor_snapshot_intensity,
# dropping a pointer slot). Keep in sync with scripts/dump_xyz_stats.py.
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
        ("anchor_p995_y",              ctypes.c_float),
        ("anchor_snapshot_intensity",  ctypes.c_float),
    ]


assert ctypes.sizeof(LUMICE_RawXyzResult) == 64, (
    "LUMICE_RawXyzResult size mismatch — verify lumice.h field layout"
)


class LUMICE_ServerConfig(ctypes.Structure):
    _fields_ = [
        ("num_workers", ctypes.c_int),
        ("sim_seed",    ctypes.c_uint),
    ]


assert ctypes.sizeof(LUMICE_ServerConfig) == 8, (
    "LUMICE_ServerConfig size mismatch — verify lumice.h field layout"
)


# LUMICE_ServerState constants (lumice.h)
_LUMICE_SERVER_IDLE = 0
_LUMICE_SERVER_RUNNING = 1
_LUMICE_SERVER_NOT_READY = 2


@dataclass
class SimResult:
    """Subset of LUMICE_RawXyzResult fields exposed to test code.

    ``anchor_p995_y`` and ``anchor_snapshot_intensity`` are non-zero only when the
    server is running in Adaptive Brightness OFF mode (F1) with a filter spec.
    The legacy ``unfiltered_*`` C API fields were removed in scrum-adaptive-
    additivity-redesign / task-f1-simulator-mode-toggle.
    """

    snapshot_intensity: float
    anchor_p995_y: float
    anchor_snapshot_intensity: float
    has_valid_data: bool
    effective_pixels: int


@dataclass
class BufferedSimResult:
    """SimResult plus a copied XYZ buffer.

    The historical ``unf_buf`` (unfiltered XYZ image) was removed alongside the
    ``unfiltered_xyz_buffer`` C API field. Tests that previously relied on it
    (the partition_buffer_additivity family in test_additivity.py) are scheduled
    for redesign on the OFF-mode anchor baseline in a follow-up task.
    """

    snapshot_intensity: float
    anchor_p995_y: float
    anchor_snapshot_intensity: float
    has_valid_data: bool
    effective_pixels: int
    img_width: int
    img_height: int
    flt_buf: np.ndarray


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

    lib.LUMICE_CreateServerEx.restype = ctypes.c_void_p
    lib.LUMICE_CreateServerEx.argtypes = [ctypes.POINTER(LUMICE_ServerConfig)]

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
            snapshot_intensity=float(r.snapshot_intensity),
            anchor_p995_y=float(r.anchor_p995_y),
            anchor_snapshot_intensity=float(r.anchor_snapshot_intensity),
            has_valid_data=bool(r.has_valid_data),
            effective_pixels=int(r.effective_pixels),
        )

    finally:
        lib.LUMICE_DestroyServer(server)


def run_scene_capi_buffered(config_path: str, sim_seed: int = 0, timeout_sec: int = 180) -> BufferedSimResult:
    """Run a Lumice sim via the C API and copy out both XYZ buffers.

    Polling exits only after `has_valid_data AND IDLE` is observed on two
    consecutive samples. The two-sample check defends against the narrow window
    where the server reports IDLE before the latest snapshot is fully visible
    to the caller; it is independent of the c_api.cpp sentinel-overflow fix.

    Buffer contents are copied into owned numpy arrays before destroying the
    server; the returned object holds no references to server memory.
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
        err = lib.LUMICE_CommitConfigFromFile(server, str(config_path).encode("utf-8"))
        if err != 0:
            raise RuntimeError(f"CommitConfigFromFile failed err={err} config={config_path}")

        results = (LUMICE_RawXyzResult * 1)()
        state_out = ctypes.c_int(0)
        t_start = time.time()
        consecutive_ok = 0

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
                consecutive_ok += 1
                if consecutive_ok >= 2:
                    break
            else:
                consecutive_ok = 0

            time.sleep(0.2)

        r = results[0]
        # Snapshot scalars + pointer addresses before any further API call.
        r_w = int(r.img_width)
        r_h = int(r.img_height)
        r_xyz_addr = ctypes.cast(r.xyz_buffer, ctypes.c_void_p).value
        r_snap = float(r.snapshot_intensity)
        r_anchor_p995 = float(r.anchor_p995_y)
        r_anchor_int = float(r.anchor_snapshot_intensity)
        r_valid = bool(r.has_valid_data)
        r_eff = int(r.effective_pixels)
        if r_xyz_addr is None:
            raise RuntimeError(
                f"{config_path}: race — xyz pointer became NULL after IDLE check"
            )

        n = r_w * r_h * 3

        def _copy_addr(addr: int) -> np.ndarray:
            return (
                np.frombuffer(
                    (ctypes.c_float * n).from_address(addr),
                    dtype=np.float32,
                )
                .copy()
                .reshape(r_h, r_w, 3)
                .astype(np.float64)
            )

        flt_buf = _copy_addr(r_xyz_addr)

        return BufferedSimResult(
            snapshot_intensity=r_snap,
            anchor_p995_y=r_anchor_p995,
            anchor_snapshot_intensity=r_anchor_int,
            has_valid_data=r_valid,
            effective_pixels=r_eff,
            img_width=r_w,
            img_height=r_h,
            flt_buf=flt_buf,
        )

    finally:
        lib.LUMICE_DestroyServer(server)
