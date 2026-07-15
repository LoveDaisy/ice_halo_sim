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


def classify_pixels_by_color_direction(
    image_path,
    class_colors,
    luminance_floor=8.0,
    cos_similarity_tol=0.98,
):
    """Classify pixels of a composite image by their RGB direction.

    Contract (WHITE-BOX, keep in sync with compositor implementation):
      This function is valid ONLY for the compositor's `dominant` mode, where
      the output pixel is `out = color_c * ey` for the single winning class c
      (see src/server/component_compositor.cpp CompositeDominantPixel). Under
      that contract, a lit pixel's RGB direction (after normalization) is
      EQUAL to the winning class's `color_` direction, decoupled from the
      exposure scalar `ey`.
      It is NOT valid for `painter` mode: since the alpha-over redesign
      (doc/gui-custom-spectrum-and-raypath-color.md §4.8) painter composites
      `out = over(alpha_c * color_c)`, blending several classes, so a lit
      pixel's direction is a convex combination rather than one class's color.
      It is likewise NOT valid for `additive` mode (linear class mix ->
      direction is a convex combination). If the compositor's `dominant`
      direction invariant changes, this helper must be re-verified.

    Args:
      image_path: Path to a JPEG/PNG composite image.
      class_colors: Iterable of (r,g,b) tuples in [0,1], one per class
        (same convention as `raypath_color[i].color` in config JSON).
      luminance_floor: Pixels with max(R,G,B) < floor (0..255) are
        counted as background (unlit) — not misclassified.
      cos_similarity_tol: A lit pixel's normalized RGB vector must have
        cosine similarity >= tol against SOME class direction to be
        classified; otherwise it is counted as `unclassified` (phantom
        hue signal). Default 0.98 tolerates mild JPEG chroma bleeding.

    Returns:
      dict with keys:
        - "background": int, count of pixels below luminance_floor.
        - "unclassified": int, lit pixels not matching any class direction.
        - "per_class": list[int] of length len(class_colors), pixel count
          per class (best-match by cosine similarity above tolerance).
        - "total": int, total pixel count.
    """
    if not HAS_PILLOW:
        raise RuntimeError("Pillow is required for classify_pixels_by_color_direction")

    class_dirs = []
    for c in class_colors:
        r, g, b = c[0], c[1], c[2]
        norm = math.sqrt(r * r + g * g + b * b)
        if norm <= 0.0:
            raise ValueError(f"class color {c} is zero-vector")
        class_dirs.append((r / norm, g / norm, b / norm))

    per_class = [0] * len(class_dirs)
    background = 0
    unclassified = 0

    with Image.open(image_path) as img:
        rgb = img.convert("RGB")
        pixels = list(rgb.getdata())

    for r, g, b in pixels:
        m = r if r >= g else g
        if b > m:
            m = b
        if m < luminance_floor:
            background += 1
            continue
        norm = math.sqrt(r * r + g * g + b * b)
        best_idx = -1
        best_cos = -1.0
        for i, (cr, cg, cb) in enumerate(class_dirs):
            cos = (r * cr + g * cg + b * cb) / norm
            if cos > best_cos:
                best_cos = cos
                best_idx = i
        if best_cos >= cos_similarity_tol:
            per_class[best_idx] += 1
        else:
            unclassified += 1

    return {
        "background": background,
        "unclassified": unclassified,
        "per_class": per_class,
        "total": len(pixels),
    }
