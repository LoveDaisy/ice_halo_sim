#ifndef SRC_UTIL_COLOR_SPACE_H_
#define SRC_UTIL_COLOR_SPACE_H_

namespace lumice {

// ---- Low-level primitives (three independent operations) ----

// Gamut clipping: scale XYZ toward D65 gray axis so subsequent XYZ→RGB stays in [0,1].
// Input/output are XYZ tristimulus values. Caller may skip this (e.g. PostSnapshot !use_real_color path).
void GamutClipXyz(const float xyz[3], float clipped[3]);

// XYZ→linear RGB matrix multiply (no gamut clip, no gamma). Output clamped to [0,1].
void XyzToLinearRgb(const float xyz[3], float rgb[3]);

// sRGB gamma: linear → sRGB (threshold 0.0031308).
float LinearToSrgb(float linear);

// sRGB gamma batch in-place.
void LinearToSrgbBatch(float* rgb, int channel_count);

// ---- High-level API (combines primitives) ----

// Full pipeline: gamut clip → matrix → gamma → sRGB float [0,1].
// Input: arbitrary XYZ tristimulus values (caller responsible for intensity scaling).
void XyzToSrgb(const float xyz[3], float rgb[3]);

// Batch full pipeline + uint8 output (clamp + 255 scale).
void XyzToSrgbUint8(const float* xyz_in, unsigned char* out, int pixel_count);

// Batch with intensity_scale applied per-pixel before conversion (avoids temp buffer allocation).
void XyzToSrgbUint8(const float* xyz_in, unsigned char* out, int pixel_count, float intensity_scale);

}  // namespace lumice

#endif  // SRC_UTIL_COLOR_SPACE_H_
