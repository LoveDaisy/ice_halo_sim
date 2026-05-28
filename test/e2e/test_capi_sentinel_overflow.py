"""Regression guard: sentinel-overflow in LUMICE_GetRawXyzResults.

Fix commit: 5287efe (fix(capi-sentinel-overflow): guard sentinel write past max_count)

Root cause: c_api.cpp wrote a sentinel at out[count] when count == max_count,
overflowing the caller's buffer by one LUMICE_RawXyzResult (72 bytes). With
3 distinct configs in rotation, the heap layout evolved such that the overflow
hit a live allocation at approximately lifecycle 31, causing SIGSEGV.

Trigger conditions:
- max_count == 1 (caller passes a size-1 array, the common case via capi_runner)
- ≥ 3 distinct configs alternated across lifecycles (single-config loops survived
  because the heap layout remained stable across repeated same-config lifecycles)

This test runs 3 configs × _REGRESSION_ROUNDS lifecycles (> 31 crash threshold)
via the same C API path used by the bug. If the sentinel guard were removed,
the process would SIGSEGV before the assertion on lifecycle ~31 completes.

Requires shared-lib build: ./scripts/build.sh -sj release
Run: pytest test/e2e/ -v -m slow
"""

from __future__ import annotations

from pathlib import Path

import pytest

from test.e2e.capi_runner import run_scene_capi

_CONFIGS_DIR = Path(__file__).parent / "configs"

_LIFECYCLE_CONFIGS = [
    str(_CONFIGS_DIR / "raypath_symmetry_4_6.json"),
    str(_CONFIGS_DIR / "raypath_symmetry_4_6_nofilter.json"),
    str(_CONFIGS_DIR / "raypath_symmetry_7_3.json"),
]

# 12 rounds × 3 configs = 36 lifecycles > 31 crash threshold
_REGRESSION_ROUNDS = 12


@pytest.mark.slow
def test_capi_sentinel_overflow_regression():
    """3-config rotation across 36 server lifecycles must not crash or produce invalid data.

    Each lifecycle calls CreateServer / CommitConfigFromFile / poll-until-IDLE /
    DestroyServer with a size-1 LUMICE_RawXyzResult array (max_count=1). Before
    fix 5287efe the sentinel write at out[max_count] overflowed this array;
    this test would have crashed around lifecycle 31 on all platforms.
    """
    n_configs = len(_LIFECYCLE_CONFIGS)
    total = _REGRESSION_ROUNDS * n_configs
    completed = 0
    for _ in range(_REGRESSION_ROUNDS):
        for cfg in _LIFECYCLE_CONFIGS:
            result = run_scene_capi(cfg)
            completed += 1
            assert result.has_valid_data, (
                f"lifecycle {completed}/{total}: no valid data ({cfg})"
            )
    assert completed == total
