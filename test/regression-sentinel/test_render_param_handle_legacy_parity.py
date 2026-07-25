"""Renderer fidelity of the Scene-handle config path vs the legacy LUMICE_Config path.

Guards against reintroduction of the v4.11 renderer expression gap: ``LUMICE_RenderParam`` had
no fields for ``lens`` / ``lens_shift`` / ``view`` / ``visible`` / ``background`` / ``ray_color``
/ ``grid`` / ``celestial_outline``, so every C API entry point that re-encodes a renderer replaced
them with a hardcoded ``dual_fisheye_equal_area`` / fov 180 / view {0,0,0} / ``visible=full`` /
black-background renderer. A config parsed cleanly and was then simulated through a projection
nobody asked for.

Why this file and not only the C++ corpus parity test: the unit-level assertion proves the two
parsers produce equal ``RenderConfig`` structs. That structural equality *implies* equal pixels
only if nothing downstream of the struct differs between the two paths — a mechanism-level
inference, not an observation. These tests make the observation: identical ``sim_seed``, both
paths, compare the actual rendered output.

Both paths converge on the same ``Server::CommitConfig`` with the same seed, so the expected
result is near-bit-identical output — NOT merely "highly correlated". The thresholds are
deliberately far tighter than the cross-backend battery's (``parity-cross-backend/`` compares two
different tracing implementations; here a visible difference means a field was dropped or
mistranslated).

Coverage mirrors the AT-RISK corpus the gap was diagnosed against:
  - all 11 forward projections (via the shared ``_projection_battery`` generator, the same single
    source the cross-backend batteries use),
  - ``multi_lens.json`` — three renderers with three different lens/fov in ONE config, the
    highest-confidence probe for "coverage collapsed onto one hardcoded projection",
  - the ``degenerate_pipeline_*`` family — ``fov=360`` dual-fisheye configs, which additionally
    exercise a non-default ``visible`` and ``ray_color``.

Where the teeth are (measured, not assumed). Reintroducing the defect — hardcoding
``r.lens_fov = 180`` in ``JsonToRenderers`` — was injected and the suite re-run:
``projection/fisheye_equal_area`` failed on snapshot_intensity (6.3e-2 relative, threshold 1e-4)
and ``multi_lens`` failed outright (core rejects fov 180 for a ``linear`` lens). The
``degenerate_pipeline_*`` cases stayed bit-identical and did NOT catch it — for the four
dual-fisheye projections ``fov`` is not an input to the projection at all (``lens_proj_build.hpp``
``ComputeScaleAz0``: "Dual-fisheye types: scale unused (r_scale carries the coverage control)"),
so 360 and 180 render the same image. They are kept for their ``visible`` / ``ray_color`` coverage,
but do not mistake their green for FOV coverage — that comes from the projection battery and
``multi_lens``.

All tests are ``@pytest.mark.slow`` (shared-lib build: ``./scripts/build.sh -sj release``), like
every other ``capi_runner``-based test.
"""

from __future__ import annotations

import pytest

from test.e2e._parity_metrics import (
    _DS_BH,
    _DS_BW,
    _raw_corr_ds,
    render_psnr,
)
from test.e2e._projection_battery import PROJECTION_TYPES, write_projection_config
from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

_SEED = 42
_TIMEOUT = 300

# Same seed + same core commit path => the two arms differ only by float accumulation order
# across worker threads. These bound that residue, not a modelling difference.
_T_CORR_DS = 0.999
_T_SNAPSHOT_REL = 1e-4
_T_PSNR_DB = 40.0

_CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

# The degenerate-pipeline family: fov=360 dual-fisheye configs. Pre-v4.11 the handle path would
# have clamped every one of them to fov 180.
_DEGENERATE_CONFIGS = sorted(p.name for p in _CONFIGS_DIR.glob("degenerate_pipeline_*.json"))


def _assert_handle_matches_legacy(config_path, label: str) -> None:
    """Run `config_path` through both config paths at one seed and compare the output.

    Both arms name their ``commit_path`` explicitly. This file is the one place in the suite
    that needs the two surfaces to *differ*, so neither arm may inherit the runner's default —
    a future change to that default would otherwise turn this differential into a
    handle-vs-handle self-comparison that passes for the wrong reason.
    """
    legacy = run_scene_capi_buffered(
        str(config_path),
        sim_seed=_SEED,
        timeout_sec=_TIMEOUT,
        backend="legacy",
        commit_path="config_file",
    )
    handle = run_scene_capi_buffered(
        str(config_path),
        sim_seed=_SEED,
        timeout_sec=_TIMEOUT,
        backend="legacy",
        commit_path="scene_handle",
    )

    assert legacy.has_valid_data, f"{label}: legacy arm produced no valid data"
    assert handle.has_valid_data, f"{label}: handle arm produced no valid data"
    # A scene that renders nothing would satisfy every metric below for the wrong reason.
    assert legacy.snapshot_intensity > 0.0, f"{label}: legacy arm carries no signal"

    assert (handle.img_width, handle.img_height) == (legacy.img_width, legacy.img_height), (
        f"{label}: image geometry differs — handle "
        f"{handle.img_width}x{handle.img_height} vs legacy {legacy.img_width}x{legacy.img_height}"
    )

    rel = abs(handle.snapshot_intensity - legacy.snapshot_intensity) / legacy.snapshot_intensity
    corr = _raw_corr_ds(handle, legacy, _DS_BH, _DS_BW)
    psnr = render_psnr(handle, legacy)
    print(
        f"[handle-parity] {label}: snapshot_rel={rel:.3e} ds_corr={corr:.6f} psnr={psnr:.2f}dB "
        f"(legacy_intensity={legacy.snapshot_intensity:.6g})"
    )

    assert rel <= _T_SNAPSHOT_REL, (
        f"{label}: snapshot_intensity relative difference {rel:.3e} > {_T_SNAPSHOT_REL:.0e} "
        f"(handle={handle.snapshot_intensity:.6g} legacy={legacy.snapshot_intensity:.6g}) — the "
        f"two config paths did not deliver the same renderer to the core"
    )
    assert corr >= _T_CORR_DS, (
        f"{label}: ds_corr {corr:.6f} < {_T_CORR_DS} — the handle path rendered a structurally "
        f"different image, which is what dropping a lens/view/visible field looks like"
    )
    assert psnr >= _T_PSNR_DB, f"{label}: render PSNR {psnr:.2f} dB < {_T_PSNR_DB}"


@pytest.fixture(scope="module")
def _proj_configs(tmp_path_factory) -> dict:
    """One single-MS config per projection type, from the shared battery generator."""
    out_dir = tmp_path_factory.mktemp("handle_parity_proj")
    return {t: write_projection_config(t, out_dir) for t in PROJECTION_TYPES}


@pytest.mark.slow
@pytest.mark.parametrize("lens_type", PROJECTION_TYPES)
def test_projection_survives_the_scene_handle_path(lens_type: str, _proj_configs):
    """Each of the 11 projections renders identically through both config paths."""
    _assert_handle_matches_legacy(_proj_configs[lens_type], f"projection/{lens_type}")


@pytest.mark.slow
def test_multi_renderer_config_survives_the_scene_handle_path():
    """Three renderers, three different lens/fov, one config — the coverage-collapse probe."""
    _assert_handle_matches_legacy(_CONFIGS_DIR / "multi_lens.json", "multi_lens")


@pytest.mark.slow
@pytest.mark.parametrize("config_name", _DEGENERATE_CONFIGS)
def test_degenerate_pipeline_config_survives_the_scene_handle_path(config_name: str):
    """Non-default ``visible`` + ``ray_color`` coverage. NOT a FOV probe — see the module
    docstring: fov is not an input to the dual-fisheye projections these configs use."""
    _assert_handle_matches_legacy(_CONFIGS_DIR / config_name, config_name)
