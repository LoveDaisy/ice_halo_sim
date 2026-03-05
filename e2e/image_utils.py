"""Image utility functions for E2E tests."""

import math

try:
    from PIL import Image

    HAS_PILLOW = True
except ImportError:
    HAS_PILLOW = False


def get_dimensions(path):
    """Return (width, height) of a JPEG image."""
    with Image.open(path) as img:
        return img.size


def compute_mse(path1, path2):
    """Compute Mean Squared Error between two images.

    Both images must have the same dimensions. Returns MSE across all channels.
    """
    with Image.open(path1) as img1, Image.open(path2) as img2:
        if img1.size != img2.size:
            raise ValueError(
                f"Image size mismatch: {img1.size} vs {img2.size}"
            )
        rgb1 = img1.convert("RGB")
        rgb2 = img2.convert("RGB")
        get_pixels = "get_flattened_data" if hasattr(rgb1, "get_flattened_data") else "getdata"
        pixels1 = list(getattr(rgb1, get_pixels)())
        pixels2 = list(getattr(rgb2, get_pixels)())

    total = 0.0
    n = len(pixels1) * 3  # 3 channels
    for (r1, g1, b1), (r2, g2, b2) in zip(pixels1, pixels2):
        total += (r1 - r2) ** 2 + (g1 - g2) ** 2 + (b1 - b2) ** 2
    return total / n


def compute_psnr(mse, max_val=255):
    """Compute Peak Signal-to-Noise Ratio from MSE.

    Returns float('inf') if mse == 0.
    """
    if mse == 0:
        return float("inf")
    return 10 * math.log10(max_val**2 / mse)
