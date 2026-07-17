"""Regression net: degenerate-crystal filter+render pipeline non-zero energy.

The `face_distance` under-review path draws six per-crystal plane offsets from
either a bounded uniform distribution or an unbounded gaussian. On a hex prism,
the geometry stays legal (Euler-closed convex mesh with 5-8 polygon faces) even
when std pushes the sample far off (1,1,1,1,1,1) — a hex whose corners
collapse into a pentagon or "trapezoidal prism" is still a valid solid, and its
side-face indices simply reshuffle rather than disappear. That has to survive
the full CLI/C-API pipeline: filter (`entry_exit 3->5` selects light entering
face 3 and exiting face 5, with `min_len`/`max_len` unset so any raypath length
matches) + render (`dual_fisheye_equal_area` at `fov=360` accumulates the entire
sphere, avoiding the "viewport doesn't cover the signal" trap from an earlier
diagnosis session that mistook aim-error for filter mismatch).

Six fixtures span the AC1/AC2 matrix:

- `degenerate_pipeline_{uniform,gaussian}_std{015,030,050}.json`
- std=0.15 is the control (~0% degenerate crystals),
  std=0.30 puts ~13.6% of prisms in the "degenerate-but-legal" bucket,
  std=0.50 pushes that to ~49.2%.

The observable is `snapshot_intensity`, the sum of accumulated ray energy across
the full-sphere render buffer per LUMICE_RawXyzResult. Non-zero here is
equivalent to "the filter admitted at least one ray whose contribution made it
to render output" — the same signal `test_ms_filter_leak.py` uses. Zero on any
of the six fixtures means the pipeline silently dropped every ray, which is
exactly the "hide bugs in the parameter domain" failure mode this regression
net guards against.

Marked `@pytest.mark.slow` because the six 400k-ray runs need the shared-lib
build (`./scripts/build.sh -sj release`); it runs locally via `pytest -v -m
slow` before opening PRs that touch simulator core / filter / C API.
"""

from __future__ import annotations

import pytest

from test.e2e.capi_runner import run_scene_capi
from test.e2e.runner import get_project_root


_CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# Fixed sim_seed → deterministic run-to-run; if a fixture ever goes zero at this
# seed the failure is a genuine pipeline regression rather than a rare draw.
_SIM_SEED = 20250717


@pytest.mark.slow
@pytest.mark.parametrize(
    "dist_kind,std_tag",
    [
        ("uniform", "015"),
        ("uniform", "030"),
        ("uniform", "050"),
        ("gaussian", "015"),
        ("gaussian", "030"),
        ("gaussian", "050"),
    ],
)
def test_degenerate_pipeline_all_sky_energy_nonzero(dist_kind: str, std_tag: str) -> None:
    """Every std tier must produce non-zero all-sky energy through entry_exit 3->5.

    A zero here means one of three failures — all of which the regression net
    is written to catch:

    1. The random face_distance path silently drops the whole crystal
       population (rejection rate spikes above ~50%),
    2. The filter no longer matches the reshuffled face indices on degenerate
       prisms (loss of AC2 "filter matches with geometric expectation"),
    3. The render silently discards the ray energy (loss of the calibrated
       observable — the exact trap that motivated switching from window
       energy to full-sphere `dual_fisheye_equal_area` per the observable
       hard-constraint below).
    """
    cfg = _CONFIGS_DIR / f"degenerate_pipeline_{dist_kind}_std{std_tag}.json"
    result = run_scene_capi(str(cfg), sim_seed=_SIM_SEED)
    assert result.has_valid_data, f"server returned no valid data for {cfg.name}"
    assert result.snapshot_intensity > 0.0, (
        f"fixture {cfg.name} produced zero all-sky energy: "
        f"filter/render pipeline dropped every ray "
        f"(observable = snapshot_intensity over dual_fisheye_equal_area fov=360, "
        f"the calibrated all-sky-energy observable)"
    )
