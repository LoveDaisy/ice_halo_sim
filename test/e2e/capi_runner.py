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
    2. ``build/Release/lib/liblumice.{dylib,so}``
    3. ``build/cmake_install/{liblumice.{dylib,so}, lib/liblumice.{dylib,so}}``
    4. ``build/cmake_build/liblumice.{dylib,so}``

The library must be built with ``BUILD_SHARED_LIBS=ON`` (the default release
recipe). If lookup fails, raises :class:`FileNotFoundError`.
"""

from __future__ import annotations

import ctypes
import os
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np


# Mirrors LUMICE_RawXyzResult in src/include/lumice.h. Anchor fields removed
# in task-remove-anchor-lane; struct size shrunk from 64 → 56 bytes.
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
    ]


assert ctypes.sizeof(LUMICE_RawXyzResult) == 56, (
    "LUMICE_RawXyzResult size mismatch — verify lumice.h field layout"
)


# Mirrors LUMICE_RenderResult in src/include/lumice.h (lumice.h:57-64).
class LUMICE_RenderResult(ctypes.Structure):
    _fields_ = [
        ("renderer_id",  ctypes.c_int),
        ("img_width",    ctypes.c_int),
        ("img_height",   ctypes.c_int),
        ("img_buffer",   ctypes.POINTER(ctypes.c_ubyte)),
    ]


# Backend constants (lumice.h:391-392).
LUMICE_BACKEND_CPU = 0
LUMICE_BACKEND_METAL = 1


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
    """Subset of LUMICE_RawXyzResult fields exposed to test code."""

    snapshot_intensity: float
    has_valid_data: bool
    effective_pixels: int


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

    lib.LUMICE_GetRenderResults.restype = ctypes.c_int
    lib.LUMICE_GetRenderResults.argtypes = [
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
            assert _ACTIVE_LOG_SINK is None, "nested LogCapture is not supported"
            _ACTIVE_LOG_SINK = self.lines
        return self.lines

    def __exit__(self, *_) -> None:
        global _ACTIVE_LOG_SINK
        with _LOG_LOCK:
            _ACTIVE_LOG_SINK = None


# Patterns matching the routing log lines in simulator.cpp:520-537.
_RE_ROUTED_METAL = re.compile(r"routing via MetalTraceBackend")
_RE_ROUTED_CPU_BACKEND = re.compile(r"routing via CpuTraceBackend")
_RE_FALLBACK = re.compile(r"falling back", re.IGNORECASE)


def _summarize_backend(lines: List[str]) -> tuple[str, bool]:
    """Return (routed_backend, fell_back) parsed from captured log lines.

    routed_backend ∈ {"metal", "cpu_backend", "legacy"}; "legacy" means no
    routing line was seen (legacy path is silent in CreateBackend).
    """
    routed = "legacy"
    fell_back = False
    for ln in lines:
        if _RE_ROUTED_METAL.search(ln):
            routed = "metal"
        elif _RE_ROUTED_CPU_BACKEND.search(ln):
            routed = "cpu_backend"
        if _RE_FALLBACK.search(ln):
            fell_back = True
    return routed, fell_back


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
            has_valid_data=bool(r.has_valid_data),
            effective_pixels=int(r.effective_pixels),
        )

    finally:
        lib.LUMICE_DestroyServer(server)


_BACKEND_MODES = ("legacy", "metal", "cpu_backend")


def run_scene_capi_buffered(
    config_path: str,
    sim_seed: int = 0,
    timeout_sec: int = 180,
    backend: str = "legacy",
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

    Concurrency contract: this suite runs serially under pytest (no xdist).
    os.environ writes + LogCapture are not safe for parallel workers — adding
    parallelism here requires moving to subprocess isolation.

    Polling exits only after `has_valid_data AND IDLE` is observed on two
    consecutive samples. Buffers are copied into owned numpy arrays before
    destroying the server; the returned object holds no server-memory refs.

    `routed_backend` and `fell_back` are parsed from the captured core log;
    callers asserting "Metal really ran" must check both
    (routed_backend == "metal" and not fell_back).
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
    elif env_was_set:
        # Caller's env would override our SetPreferredBackend — strip it.
        del os.environ["LUMICE_TRACE_BACKEND"]

    capture = _LogCapture()

    try:
        with capture as log_lines:
            if sim_seed != 0:
                cfg = LUMICE_ServerConfig(num_workers=0, sim_seed=sim_seed)
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

                err = lib.LUMICE_CommitConfigFromFile(server, str(config_path).encode("utf-8"))
                if err != 0:
                    raise RuntimeError(f"CommitConfigFromFile failed err={err} config={config_path}")

                results = (LUMICE_RawXyzResult * 1)()
                renders = (LUMICE_RenderResult * 1)()
                state_out = ctypes.c_int(0)
                t_start = time.time()
                consecutive_ok = 0

                # GetRawXyzResults clears snapshot_dirty_ without invoking
                # PostSnapshot (server.cpp:421), so a later GetRenderResults
                # would observe snapshot_dirty_=false and skip DoSnapshot,
                # leaving cached_render_results_ empty. Call GetRenderResults
                # first inside the loop — it runs PrepareSnapshot+PostSnapshot
                # together when dirty, and GetRawXyzResults then reads the
                # same prepared snapshot.
                while True:
                    elapsed = time.time() - t_start
                    if elapsed > timeout_sec:
                        raise RuntimeError(
                            f"Timeout {elapsed:.1f}s waiting for {config_path} (backend={backend})"
                        )

                    err = lib.LUMICE_GetRenderResults(server, renders, 1)
                    if err != 0:
                        raise RuntimeError(f"GetRenderResults failed err={err}")

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
                r_w = int(r.img_width)
                r_h = int(r.img_height)
                r_xyz_addr = ctypes.cast(r.xyz_buffer, ctypes.c_void_p).value
                r_snap = float(r.snapshot_intensity)
                r_valid = bool(r.has_valid_data)
                r_eff = int(r.effective_pixels)
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

                # Render buffer already populated by the last GetRenderResults
                # call inside the polling loop (same snapshot as raw XYZ).
                rr = renders[0]
                rr_w = int(rr.img_width)
                rr_h = int(rr.img_height)
                rr_addr = ctypes.cast(rr.img_buffer, ctypes.c_void_p).value
                if rr_addr is None or rr_w == 0 or rr_h == 0:
                    raise RuntimeError(
                        f"{config_path}: GetRenderResults returned empty buffer"
                    )
                n_rgb = rr_w * rr_h * 3
                rgb_buf = (
                    np.frombuffer(
                        (ctypes.c_ubyte * n_rgb).from_address(rr_addr),
                        dtype=np.uint8,
                    )
                    .copy()
                    .reshape(rr_h, rr_w, 3)
                )

                routed, fell_back = _summarize_backend(log_lines)
                return BufferedSimResult(
                    snapshot_intensity=r_snap,
                    has_valid_data=r_valid,
                    effective_pixels=r_eff,
                    img_width=r_w,
                    img_height=r_h,
                    flt_buf=flt_buf,
                    rgb_buf=rgb_buf,
                    routed_backend=routed,
                    fell_back=fell_back,
                    log_lines=list(log_lines),
                )

            finally:
                lib.LUMICE_DestroyServer(server)
    finally:
        # Restore env state regardless of success/failure.
        if backend == "cpu_backend":
            if env_was_set and env_old is not None:
                os.environ["LUMICE_TRACE_BACKEND"] = env_old
            else:
                os.environ.pop("LUMICE_TRACE_BACKEND", None)
        else:
            if env_was_set and env_old is not None:
                os.environ["LUMICE_TRACE_BACKEND"] = env_old
