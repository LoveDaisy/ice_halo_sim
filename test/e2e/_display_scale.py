"""Recovering the exposure scale the server actually applied, from the image it produced.

Shared by the two suites that need to reason about ``ev_mode``'s effect on displayed brightness
(``test_relative_ev_breaks_additivity.py`` and ``test_ev_mode_ray_num_invariance.py``). It lives
here rather than in either of them because a second copy would be a second thing to keep correct,
and because the recovery has a calibration number attached (below) that must not be re-derived
per caller.

Why recover it rather than recompute it. ``RenderConsumer::ExposureScale()`` has two branches and
either could be wrong; a test that evaluated the same formula would agree with the renderer by
construction and stay green even if the value never reached the renderer at all. Dividing the
rendered image by the raw buffer measures what was applied, which is the thing under test.

Accuracy. On the additivity fixture the recovery puts ``kNormScale`` at 0.0793 against its
declared 0.08 -- a 1% low bias from uint8 quantization, stable across seeds. Every band written
against this helper is sized well outside that, and the smallest effect any caller asserts on is
about 13%.
"""

from __future__ import annotations

import numpy as np

# The band the ratio is taken over, in linear display luminance. Below the floor the uint8 grid is
# coarse enough to bias the ratio; above the ceiling a pixel is approaching the [0,1] clamp, where
# the ratio stops being a ratio.
_BAND_LO = 0.05
_BAND_HI = 0.60

# Enough samples for the median to be a median rather than an accident. A caller whose scene is
# too dim or too blown out to reach this should change the scene, not the floor.
_MIN_BAND_PIXELS = 500


def srgb_bytes_to_linear(rgb_u8: np.ndarray) -> np.ndarray:
    """Undo the sRGB transfer function on a uint8 image, giving linear values in [0, 1]."""
    u = rgb_u8.astype(np.float64) / 255.0
    return np.where(u <= 0.04045, u / 12.92, ((u + 0.055) / 1.055) ** 2.4)


def display_luminance(rgb_u8: np.ndarray) -> np.ndarray:
    """Linear luminance (Y) of a rendered sRGB image, per pixel.

    The weights are the Y row of the sRGB->XYZ matrix, so this recovers the same Y channel the
    raw XYZ buffer carries -- which is what makes the ratio in `recover_applied_scale` a scale
    rather than a colour transform.
    """
    linear = srgb_bytes_to_linear(rgb_u8)
    return 0.2126 * linear[:, :, 0] + 0.7152 * linear[:, :, 1] + 0.0722 * linear[:, :, 2]


def recover_applied_scale(result, label: str = "") -> float:
    """The scalar the server multiplied raw XYZ by before baking `result.rgb_buf`.

    `result` is a `BufferedSimResult`: `flt_buf` is the raw (unexposed) XYZ, `rgb_buf` the baked
    sRGB image. Raises AssertionError if too few pixels land in the usable band -- a silent
    fallback would return a number computed from a handful of pixels and look like a measurement.
    """
    raw_y = result.flt_buf[:, :, 1].astype(np.float64)
    displayed_y = display_luminance(result.rgb_buf)
    band = (raw_y > 0.0) & (displayed_y > _BAND_LO) & (displayed_y < _BAND_HI)
    count = int(band.sum())
    assert count >= _MIN_BAND_PIXELS, (
        f"{label or 'scene'}: only {count} pixels fall in the recovery band "
        f"[{_BAND_LO}, {_BAND_HI}] (need {_MIN_BAND_PIXELS}) -- the scene is too dim or too "
        "blown out for the applied scale to be measured from it"
    )
    return float(np.median(displayed_y[band] / raw_y[band]))


def lit_pixel_percentile(rgb_u8: np.ndarray, percentile: float) -> float:
    """A percentile of the byte luminance over LIT pixels only.

    p50 is the appearance caliber this repo uses; p90/p99 are not usable ones. Under a
    self-anchored exposure on a sparse scene the top percentiles pin at 255 across every ray
    count (measured: the f=8 coarse anchor sits ~64x under the fine one, so a large share of
    pixels clip to white), and a statistic that is constant because it is saturated reads as
    perfect stability. Callers pass the percentile explicitly so that choice stays visible.
    """
    # Byte luminance, not linear: "appearance" is what the viewer sees, and the sRGB transfer
    # function is part of that. Linearizing first would weight the dark end differently from the
    # way the eye and the screen do.
    b = rgb_u8.astype(np.float64)
    byte_luminance = 0.2126 * b[:, :, 0] + 0.7152 * b[:, :, 1] + 0.0722 * b[:, :, 2]
    lit = byte_luminance[byte_luminance > 0.0]
    assert lit.size > 0, "the image has no lit pixel, so it has no appearance to measure"
    return float(np.percentile(lit, percentile))
