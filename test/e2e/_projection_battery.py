"""Shared scaffolding for the per-projection cross-backend parity battery
(scrum-gpu-projection-parity task 315.5).

After 315.2–315.4 both GPU backends (Metal + CUDA) project every exit through
the shared ``lm_proj::ProjectExitToPixel`` (``src/core/shared/projection_shared.h``)
— the SAME single source as the legacy-CPU parity oracle
(``ScatterOutgoingToXyz``). Consequently EVERY forward projection is expected to
be byte-close to legacy CPU at any view/fov, and — because Metal/CUDA
``IsCompatible`` was relaxed to accept all lens types (315.3 for the 10 non-globe
types, 315.4 for globe) — EVERY type must route to the GPU with NO silent CPU
fallback.

This module is the single source for:
  - the canonical 11-type list (mirrors ``LensParam::LensType`` in
    ``src/config/render_config.hpp``),
  - a valid FOV per type (respecting ``MaxFov`` in ``render_config.cpp``),
  - a generator that materialises a per-type single-MS config from the shared
    ``dual_fisheye_ref`` baseline (swapping only ``lens.type`` / ``lens.fov`` /
    ``view.elevation``), and
  - the parity tolerances, which are DELIBERATELY the same as the existing
    exit-seam battery (``test_metal_exit_seam_parity.py`` /
    ``test_cuda_exit_seam_parity.py``) — do NOT loosen them per projection
    (see [[feedback_gpu_parity_corr_masks_undersampling]]: corr alone masks
    undersampling, which is why energy + cross-seed self-consistency are also
    asserted by the callers).

Both the Metal (Darwin-gated) and CUDA (LUMICE_HAS_CUDA-gated) parity test files
import from here so the 11-type coverage stays in lockstep across backends.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Dict, List

from test.e2e.runner import get_project_root

# The 11 LensParam::LensType string tokens, in the enum order declared in
# src/config/render_config.hpp (NLOHMANN_JSON_SERIALIZE_ENUM). Keep this list
# exhaustive: a new projection type MUST be added here so the battery covers it.
PROJECTION_TYPES: List[str] = [
    "linear",
    "fisheye_equal_area",
    "fisheye_equidistant",
    "fisheye_stereographic",
    "fisheye_orthographic",
    "dual_fisheye_equal_area",
    "dual_fisheye_equidistant",
    "dual_fisheye_stereographic",
    "dual_fisheye_orthographic",
    "rectangular",
    "globe",
]

# A valid FOV (degrees) per type, respecting MaxFov (render_config.cpp):
#   linear 179 / stereographic 359 / orthographic 180 / globe 90 / rest 360.
# Narrow projections (linear/rectangular/globe) use a 90° FOV centred on the sun
# (view.elevation == sun altitude) so the 22° halo is comfortably in frame and
# the image carries signal (non-zero variance is required by the corr metric).
_FOV_BY_TYPE: Dict[str, float] = {
    "linear": 90.0,
    "fisheye_equal_area": 120.0,
    "fisheye_equidistant": 120.0,
    "fisheye_stereographic": 120.0,
    "fisheye_orthographic": 120.0,
    "dual_fisheye_equal_area": 180.0,
    "dual_fisheye_equidistant": 180.0,
    "dual_fisheye_stereographic": 180.0,
    "dual_fisheye_orthographic": 180.0,
    "rectangular": 90.0,
    "globe": 90.0,
}

# View elevation (degrees) — point the camera at the sun (altitude 20 in the
# baseline scene) so the 22° halo is centred in-frame for every projection,
# including the narrow-FOV ones. Structural parity is view-independent, but a
# centred halo guarantees signal on all types.
_VIEW_ELEVATION = 20.0

# Fixed output resolution. Both axes MUST be divisible by the parity block-mean
# tile (_DS_BH=_DS_BW=4 in _parity_metrics.py); 512×256 satisfies that and
# matches the rest of the battery.
_RESOLUTION = [512, 256]

# Baseline single-MS scene reused as the template (single prism, prob=0.0 pure
# single-scatter, 10M rays, sun altitude 20, D65). Only lens/view are swapped.
_BASE_CONFIG = get_project_root() / "test" / "e2e" / "configs" / "dual_fisheye_ref.json"


# --- Parity tolerances — SAME as the exit-seam battery (do not loosen) ------ #
# ds_corr floor 0.95 == the dual_fisheye_ref single-MS-no-filter Metal threshold
# in test_metal_exit_seam_parity.py and the CUDA G1 floor; energy 0.05 == G2;
# self-consistency margin 0.02 == G3; render PSNR 13 dB == _T_PSNR_DB.
T_RAW_CORR_DS = 0.95
T_ENERGY_TOL = 0.05
T_SELF_MARGIN = 0.02
T_PSNR_DB = 13.0


def write_projection_config(lens_type: str, out_dir: Path) -> Path:
    """Materialise a single-MS parity config for ``lens_type`` under ``out_dir``.

    Derived from the shared ``dual_fisheye_ref`` baseline by swapping only
    ``render[0].lens.{type,fov}``, ``render[0].view.elevation`` and
    ``render[0].resolution``. Returns the written path.
    """
    if lens_type not in _FOV_BY_TYPE:
        raise ValueError(f"unknown lens type {lens_type!r}; not in PROJECTION_TYPES")

    with _BASE_CONFIG.open("r", encoding="utf-8") as f:
        cfg = json.load(f)

    render = cfg["render"][0]
    render["lens"] = {"type": lens_type, "fov": _FOV_BY_TYPE[lens_type]}
    render["resolution"] = list(_RESOLUTION)
    render.setdefault("view", {})["elevation"] = _VIEW_ELEVATION

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"proj_{lens_type}.json"
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    return out_path
