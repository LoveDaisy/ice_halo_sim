"""AC2 sentinel for task-#283 (metal-build-time-metallib).

Locks in the invariant that the Metal backend works end-to-end on a dev
machine WITHOUT the runtime MSL source frontend — i.e. the build-time
metallib path (newLibraryWithData on the embedded bytes) is self-sufficient.

This is the regression guard against silently reverting to the pre-task-#283
"runtime source compile only" model, which is the failure mode that broke on
macOS 26.5 (newLibraryWithSource returned a degenerate library with
functionNames=[]; see task-282 + issue.md).

Switch: env LUMICE_DISABLE_METAL_SOURCE_COMPILE=1 makes LoadMetalLibrary
refuse to fall back to newLibraryWithSource. On the dev machine this
simulates macOS 26.5 (where the source frontend is broken in practice). If
the metallib path is intact, Metal still routes successfully; if anyone
later removes the metallib path or wires the bytes incorrectly, this test
fails fast.

Positive marker: the success path emits, at INFO level (server.cpp callback sink
captures it), either
    [I] MetalTraceBackend: loaded embedded metallib (N functions)
        (one-time real load inside LoadMetalLibrary), or
    [I] MetalTraceBackend: using cached embedded metallib (N functions)
        (per-Impl marker from EnsurePso when PSOs come from the task-364 process
         cache and the one-time load fell outside this session's log window).
Asserting on either string POSITIVELY confirms the embedded-metallib route ran;
asserting only on returncode==0 / no-exception would not distinguish "metallib
ran" from "silently fell back via a different path" if a future change
weakened the env-switch guard.

Marked @pytest.mark.slow: requires the shared-lib build
(`./scripts/build.sh -sj release`) loaded via ctypes, plus a real Metal
device. CI Ubuntu/Windows legs are not Darwin → skip; CI macOS leg runs the
fast suite only (this test is in the slow pool).
"""

from __future__ import annotations

import os
import re
import sys

import pytest

from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

pytestmark = [
    pytest.mark.slow,
    pytest.mark.skipif(
        not sys.platform.startswith("darwin"),
        reason="Metal backend is Darwin-only — AC2 sentinel covers macOS 26.5 simulation",
    ),
]


# Marker(s) confirming the embedded-metallib route is in effect. Two phrasings,
# both emitted from src/core/backend/metal_trace_backend.mm (task-364 O2 cache):
#   - "MetalTraceBackend: loaded embedded metallib (N functions)"        — the
#     one-time real load inside LoadMetalLibrary (per process).
#   - "MetalTraceBackend: using cached embedded metallib (N functions)"  — a
#     per-Impl marker echoed by EnsurePso when PSOs come from the process cache
#     (the load already happened, possibly in an earlier session's log window).
# EITHER phrasing positively confirms the embedded-metallib route ran (vs the
# source-compile fallback, which emits "loaded source-compiled library" and
# matches neither). Asserting the kernel marker AND the exact "4 functions" count
# (trace_layer / gen_root / transit_root / shuffle_cont) gives early warning on:
#   (a) silent regression to source-compile-only (neither marker present), or
#   (b) accidental drop of an entry point (count != 4).
_MARKER_RE = re.compile(
    r"MetalTraceBackend:\s+(?:loaded|using\s+cached)\s+embedded\s+metallib\s+\((\d+)\s+functions\)"
)
# trace_layer_kernel + gen_root_kernel + transit_root_kernel + shuffle_cont_kernel.
# shuffle_cont_kernel added in task-gpu-backend-recombine-shuffle (continuation-
# pool device-side Feistel shuffle).
# Recount source of truth:  grep -c 'kernel void' src/core/metal/lumice_trace.metal
_EXPECTED_FUNCTION_COUNT = 4


def test_metal_runs_without_source_compile():
    """AC2: metallib path is self-sufficient under simulated macOS 26.5 conditions."""
    config = str(get_project_root() / "test" / "e2e" / "configs" / "dual_fisheye_ref.json")

    # Force LoadMetalLibrary to refuse the source-compile fallback. If anyone
    # later removes the embedded metallib path, LoadMetalLibrary returns nil
    # → EnsurePso throws BackendUnavailableError → Simulator falls back to
    # legacy CPU → routed_backend != "metal" → this test fails.
    env_was_set = "LUMICE_DISABLE_METAL_SOURCE_COMPILE" in os.environ
    env_old = os.environ.get("LUMICE_DISABLE_METAL_SOURCE_COMPILE")
    os.environ["LUMICE_DISABLE_METAL_SOURCE_COMPILE"] = "1"
    try:
        result = run_scene_capi_buffered(
            config_path=config,
            backend="metal",
            timeout_sec=180,
        )
    finally:
        if env_was_set:
            os.environ["LUMICE_DISABLE_METAL_SOURCE_COMPILE"] = env_old
        else:
            os.environ.pop("LUMICE_DISABLE_METAL_SOURCE_COMPILE", None)

    # 1. Metal must actually have run end-to-end (no silent legacy fallback).
    assert result.routed_backend == "metal", (
        f"Expected routed_backend='metal', got {result.routed_backend!r}. "
        "If this fails, the metallib path likely regressed (LoadMetalLibrary "
        "returned nil and EnsurePso threw → Simulator fell back to legacy CPU)."
    )
    assert not result.fell_back, (
        "Metal was selected but Simulator fell back. Check captured log_lines "
        "for the BackendUnavailableError message."
    )
    assert result.has_valid_data, "Metal session produced no valid render output"

    # 2. POSITIVELY confirm the embedded-metallib route ran. The "loaded" marker
    #    is emitted once per process by LoadMetalLibrary; the "using cached"
    #    marker is emitted per MetalTraceBackend instance by EnsurePso when PSOs
    #    come from the task-364 process cache. Either confirms the route; absence
    #    of both means we silently took some other route (e.g. a future refactor
    #    that bypasses LoadMetalLibrary/EnsurePso or downgrades the log level
    #    below INFO).
    matches = [m for ln in result.log_lines for m in (_MARKER_RE.search(ln),) if m]
    assert matches, (
        "Metal ran but the '(loaded|using cached) embedded metallib (N functions)' "
        "marker is absent from captured log_lines. Either LoadMetalLibrary/EnsurePso "
        "was bypassed, the marker was downgraded below INFO, or the log capture is "
        "broken.\n"
        f"Captured {len(result.log_lines)} log lines."
    )

    # 3. All four production entry points must resolve from the embedded
    #    metallib. If a future change adds a kernel, update the expected
    #    count in lock-step with the .metal source.
    counts = {int(m.group(1)) for m in matches}
    assert counts == {_EXPECTED_FUNCTION_COUNT}, (
        f"Expected exactly {_EXPECTED_FUNCTION_COUNT} entry points in the "
        f"embedded metallib (trace_layer_kernel / gen_root_kernel / "
        f"transit_root_kernel / shuffle_cont_kernel); marker reported {counts}. "
        "If you added a new MSL kernel, update _EXPECTED_FUNCTION_COUNT."
    )
