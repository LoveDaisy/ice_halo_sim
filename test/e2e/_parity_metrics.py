"""Parity metrics for the Metal/Cpu backend e2e suite.

Hoisted from the 258.6 parity harness (formerly
``scratchpad/scrum-metal-exit-seam/task-parity-harness/_measure_baseline.py``)
into a *tracked* module. The tests previously mutated ``sys.path`` to import
the metric from that scratchpad file, which works locally but breaks CI
collection: ``scratchpad/`` is gitignored and absent on fresh checkouts, so
the top-level import raised ``ModuleNotFoundError`` during pytest collection
(before marker deselection ever applies).

Single source of truth for the block-mean downsampled Pearson metric shared by
``test_metal_exit_seam_parity.py`` and ``test_device_gen_default_path.py``.
"""

from __future__ import annotations

import math

import numpy as np

# Block-mean downsample tile (258.6.2). Parity scenes are fixed 512×256, so
# both axes divide evenly.
_DS_BH = 4
_DS_BW = 4


def _block_mean(buf: np.ndarray, bh: int, bw: int) -> np.ndarray:
    """Block-mean downsample a (H, W, C) buffer by (bh, bw) tiles.

    Asserts integer divisibility — caller must pass tile sizes that divide
    H and W (parity scenes are fixed 512×256). Output shape: (H//bh, W//bw, C).
    """
    h, w, c = buf.shape
    assert h % bh == 0 and w % bw == 0, (
        f"block size ({bh}, {bw}) does not divide buffer shape ({h}, {w})"
    )
    return buf.reshape(h // bh, bh, w // bw, bw, c).mean(axis=(1, 3))


def _flt_buf_of(obj) -> np.ndarray:
    """Return a float64 (H, W, 3) buffer from either a result dict (with key
    'xyz') or a ``BufferedSimResult`` (capi_runner format with attribute
    ``flt_buf``)."""
    if hasattr(obj, "flt_buf"):
        return np.asarray(obj.flt_buf, dtype=np.float64)
    if isinstance(obj, dict) and "xyz" in obj:
        return np.asarray(obj["xyz"], dtype=np.float64)
    raise TypeError(f"cannot extract flt_buf from {type(obj).__name__}")


def _raw_corr_ds(a, b, bh: int, bw: int) -> float:
    """Block-mean-downsampled Pearson correlation between two flt_buf.

    Accepts either result dicts (with key 'xyz') or ``BufferedSimResult``
    instances (with attribute ``flt_buf``). Returns 0.0 on NaN/zero-variance.
    """
    xa = _block_mean(_flt_buf_of(a), bh, bw).ravel()
    xb = _block_mean(_flt_buf_of(b), bh, bw).ravel()
    if xa.std() == 0.0 or xb.std() == 0.0:
        return 0.0
    c = float(np.corrcoef(xa, xb)[0, 1])
    return 0.0 if math.isnan(c) else c
