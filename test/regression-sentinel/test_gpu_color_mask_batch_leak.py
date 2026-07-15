"""End-to-end regression test for the GPU per-raypath color-mask batch leak
(fix/gpu-color-mask-batch-leak / explore-365).

Bug (before fix)
    Metal + CUDA gated the layer-0 `root_component` per-batch memset inside
    the test-only `capture_ray_mask_` branch in TraceLayer. In production
    (capture off), the layer-0 carried color-class mask buffer was never
    reset while `transit_root_kernel` (layer ≥ 1) kept writing into it, so
    batch N's layer-0 rays inherited batch N-1's carried color bits and
    OR-merged them with their own crystal's color bit. The visible symptom
    was cross-backend divergence on multi-crystal-L0 + multi-batch scenes:
    per-class total CPU/Metal ratios of 7.03× (RED) and 3.09× (GREEN) on
    `raypath_color_multi_layer.json`, with the dominant argmax winner
    shifting between backends (CPU orange/magenta dominant, Metal green
    dominant).

Regression contract
    On the multi-layer color config (5 L0 crystal entries + L1 crystal +
    ray_num=4M >> dispatch grain), each color class's dominant-pixel-count
    ratio between CPU and Metal must stay within a tight symmetric band
    around 1.0. A resurrection of the batch-leak bug decorrelates the
    per-class marginals across backends and drops the min/max ratio well
    below the band (issue.md measured ~0.14–0.32 on broken code).

Test protocol (issue-scenario contract, `feedback_test_must_use_issue_scenario`)
    - Reuses the exact explore-365 fixture
      (`test/e2e/configs/raypath_color_multi_layer.json`) — same knobs
      that triggered the bug in prod.
    - Runs a REAL server via `run_lumice` CLI (not a raw backend harness).
      The bug only reproduces when the backend is reused across per-batch
      BeginSession cycles, which the CLI does but the raw-backend +
      fresh-backend-per-batch parity harness does not.
    - `capture_ray_mask_` is off (CLI default).
    - Compares CPU vs Metal dominant-pixel counts per color class in the
      composite output.

Slow marker
    Marked `@pytest.mark.slow` because the fixture uses ray_num=4,000,000
    to guarantee `ray_num > dispatch` across dispatch grains. Not part of
    the fast CI suite; run before opening a PR that touches the GPU
    backend or trace-layer color path.

Darwin-only skip
    Metal is only available on macOS. On Linux the CLI's `--backend metal`
    silently falls back to CPU, which would degenerate this into a
    CPU-vs-CPU tautology — skip instead.
"""

from __future__ import annotations

import platform
import tempfile
from pathlib import Path

import pytest

from test.e2e.image_utils import HAS_PILLOW
from test.e2e.runner import find_lumice_binary, get_project_root, run_lumice

if HAS_PILLOW:
    from test.e2e.image_utils import classify_pixels_by_color_direction


CONFIG_PATH = get_project_root() / "test" / "e2e" / "configs" / "raypath_color_multi_layer.json"


# task-339.5 color-class list order (must match raypath_color_multi_layer.json).
# Order is significant: the classifier maps returned per_class[i] to class i.
CLASS_COLORS = [
    (1.0, 0.0, 1.0),   # 0: MAGENTA — cross-layer AND {L0,C1}∧{L1,C2}
    (1.0, 0.55, 0.0),  # 1: ORANGE  — L1 crystal 3 match-all
    (1.0, 0.0, 0.0),   # 2: RED     — L0 C4 match-all
    (0.0, 1.0, 0.0),   # 3: GREEN   — L0 C5 OR-union of two length predicates
]

CLASS_LABELS = ["MAGENTA", "ORANGE", "RED", "GREEN"]

# Cosine tolerance for direction-based pixel classification. Matches
# MULTI_LAYER_COS_TOL in test_raypath_color.py (0.90 tolerates JPEG chroma
# bleed near halo boundaries).
COS_TOL = 0.90

# Per-class dominant-pixel min/max ratio floor. On the fix each class should
# be ~1.0 up to MC noise; on the leaky code min/max collapses to ~0.14
# (RED 7.03×) or ~0.32 (GREEN 3.09×) per issue.md. 0.65 leaves headroom for
# statistical noise + JPEG quantization at 4M rays while still catching a
# structural regression by a wide margin.
PER_CLASS_MIN_RATIO = 0.65

# Sanity floor: every class must have at least this many dominant pixels on
# BOTH backends — the ratio contract is only meaningful when both backends
# actually produce that class. If a class has zero pixels on one backend,
# ratio is undefined; treat it as a hard failure separately.
CLASS_MIN_PIXELS = 100


pytestmark = pytest.mark.skipif(
    platform.system() != "Darwin",
    reason="Metal backend only available on macOS; CPU-vs-CPU is a tautology",
)


def _run_backend_composite(backend_args, out_dir: str) -> Path:
    result = run_lumice(
        ["-f", str(CONFIG_PATH), "-o", out_dir, *backend_args],
    )
    assert result.returncode == 0, (
        f"Lumice failed (backend_args={backend_args}):\n"
        f"stdout:\n{result.stdout}\n---\nstderr:\n{result.stderr}"
    )
    composite = Path(out_dir) / "img_01_components.jpg"
    assert composite.exists() and composite.stat().st_size > 0, (
        f"composite missing/empty (backend_args={backend_args}): {composite}"
    )
    return composite


@pytest.mark.slow
def test_multi_batch_per_class_ratio_cpu_vs_metal_within_band():
    """CPU vs Metal per-class dominant-pixel counts stay within a tight band.

    Guards against reintroduction of the layer-0 carried-mask cross-batch
    leak. If the reset is misgated (or absent) again, batch N's rays inherit
    prior batches' color bits on Metal but not CPU, decorrelating the
    per-class marginals: min/max ratios plunge (measured ~0.14 for RED,
    ~0.32 for GREEN in the bug state).
    """
    if not HAS_PILLOW:
        pytest.skip("Pillow required for pixel classification")

    try:
        find_lumice_binary()
    except FileNotFoundError as e:
        pytest.skip(str(e))

    assert CONFIG_PATH.exists(), f"missing fixture: {CONFIG_PATH}"

    with tempfile.TemporaryDirectory(prefix="lumice_e2e_cpu_") as cpu_dir, \
         tempfile.TemporaryDirectory(prefix="lumice_e2e_metal_") as metal_dir:
        cpu_composite = _run_backend_composite([], cpu_dir)
        metal_composite = _run_backend_composite(["--backend", "metal"], metal_dir)

        cpu_result = classify_pixels_by_color_direction(
            str(cpu_composite), CLASS_COLORS, cos_similarity_tol=COS_TOL,
        )
        metal_result = classify_pixels_by_color_direction(
            str(metal_composite), CLASS_COLORS, cos_similarity_tol=COS_TOL,
        )

        cpu_counts = cpu_result["per_class"]
        metal_counts = metal_result["per_class"]

        # Diagnostic print — kept in the assert message on failure, but also
        # emit here so `pytest -s` surfaces the numbers on green runs for
        # trend-tracking.
        print(
            "\n[gpu-color-mask-batch-leak] per-class dominant pixels:\n"
            + "\n".join(
                f"  {CLASS_LABELS[i]:8s}: cpu={cpu_counts[i]:>6d} metal={metal_counts[i]:>6d} "
                f"min/max={min(cpu_counts[i], metal_counts[i]) / max(cpu_counts[i], metal_counts[i], 1):.3f}"
                for i in range(len(CLASS_LABELS))
            )
        )

        # Both backends must produce every class above the sanity floor —
        # a zero count would signal a different regression (class wiring
        # broken) and make the ratio metric undefined.
        for idx, label in enumerate(CLASS_LABELS):
            assert cpu_counts[idx] >= CLASS_MIN_PIXELS, (
                f"CPU produced only {cpu_counts[idx]} dominant pixels for "
                f"class {label} (< sanity floor {CLASS_MIN_PIXELS}); "
                f"per_class CPU={cpu_counts} metal={metal_counts}"
            )
            assert metal_counts[idx] >= CLASS_MIN_PIXELS, (
                f"Metal produced only {metal_counts[idx]} dominant pixels "
                f"for class {label} (< sanity floor {CLASS_MIN_PIXELS}); "
                f"per_class CPU={cpu_counts} metal={metal_counts}"
            )

        # Symmetric ratio contract — the batch-leak bug pushed some classes
        # to ~0.14–0.32 on Metal, well below the floor.
        failures = []
        for idx, label in enumerate(CLASS_LABELS):
            hi = max(cpu_counts[idx], metal_counts[idx])
            lo = min(cpu_counts[idx], metal_counts[idx])
            ratio = lo / hi
            if ratio < PER_CLASS_MIN_RATIO:
                failures.append(
                    f"  class {label}: cpu={cpu_counts[idx]} metal={metal_counts[idx]} "
                    f"min/max={ratio:.3f} < floor={PER_CLASS_MIN_RATIO}"
                )
        assert not failures, (
            "per-class dominant-pixel CPU/Metal ratio outside band — "
            "possible reintroduction of the layer-0 carried-mask cross-batch "
            "leak (explore-365):\n" + "\n".join(failures) +
            f"\n  full: CPU={cpu_counts} Metal={metal_counts}"
        )
