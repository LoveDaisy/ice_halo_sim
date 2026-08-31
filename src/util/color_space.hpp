#ifndef SRC_UTIL_COLOR_SPACE_H_
#define SRC_UTIL_COLOR_SPACE_H_

#include <cmath>

namespace lumice {

// ---- Low-level primitives (three independent operations) ----

// Gamut clipping: scale XYZ toward D65 gray axis so subsequent XYZ→RGB stays in [0,1].
// Input/output are XYZ tristimulus values. Caller may skip this (e.g. PostSnapshot !use_real_color path).
void GamutClipXyz(const float xyz[3], float clipped[3]);

// XYZ→linear RGB matrix multiply (no gamut clip, no gamma). Output clamped to [0,1].
void XyzToLinearRgb(const float xyz[3], float rgb[3]);

// The scalar sRGB transfer curve, both directions. The two thresholds are the same point on the
// curve expressed in the two spaces (0.0031308 * 12.92 = 0.04045), so the pair is an exact
// analytic inverse. Used at the JSON boundary: the "background" key is authored in sRGB (what a
// colour picker shows), while the struct field it lands in is linear (what additive blending
// needs).
//
// RULE — the SCALAR transfer curve is `inline` here; the bulk/composite helpers below stay
// out-of-line in color_space.cpp. Keep both directions on the same side of that line.
//
// Why: `src/gui/` and its test targets link `lumice` / `lumice_gui_obj`, never `lumice_obj`, and a
// Release *shared* build hides lumice_obj's non-C-API symbols (-fvisibility=hidden,
// CMakeLists.txt). So an out-of-line definition here fails to link from src/gui/ in that flavor —
// even though the #include itself is within the gui-api-boundary policy (util/ is not core/ or
// config/). `inline` compiles the body into every translation unit that includes the header, so no
// cross-library symbol is needed. Static builds link the archive directly and never notice the
// difference, which is exactly why this asymmetry can sit unnoticed: only the
// shared + GUI + test combination links it the failing way.
//
// This bit us once, in the direction of a half-fix: SrgbToLinear was made inline for src/gui/'s
// sake while LinearToSrgb was left out-of-line, and a GUI test calling the forward direction then
// failed to link in the shared flavor only. Both directions are inline now; if a third direction
// or a scalar sibling is ever added, it goes here too.
inline float LinearToSrgb(float linear) {
  if (linear < 0.0031308f) {
    return linear * 12.92f;
  }
  return 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
}

inline float SrgbToLinear(float srgb) {
  if (srgb < 0.04045f) {
    return srgb / 12.92f;
  }
  return std::pow((srgb + 0.055f) / 1.055f, 2.4f);
}

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

// Batch with intensity_scale AND an additive linear-RGB background composited before the final
// clamp. Same composition as RenderConsumer::PostSnapshot's use_real_color branch (server/render
// .cpp) and as the preview fragment shader's xyzToLinearRgb/clampAndGamma split, which is what
// makes a texture baked through here agree with the frame on screen.
//
// The order is load-bearing and is the whole point of the split: gamut clip -> matrix -> ADD
// background -> clamp -> gamma. Adding after the gamma curve would break the identity a zero-halo
// pixel is supposed to satisfy (it would come back gamma-encoded twice), so `background` is LINEAR
// RGB and the caller converts from the picker's sRGB with SrgbToLinearRgb below.
void XyzToSrgbUint8(const float* xyz_in, unsigned char* out, int pixel_count, float intensity_scale,
                    const float background[3]);

// Inverse sRGB gamma over a 3-channel colour: the fixed-size sibling of the scalar SrgbToLinear,
// shaped like XyzToLinearRgb above. Every caller that hands a picker colour to the additive
// background path goes through this rather than hand-rolling the same three-iteration loop.
// inline for the same link-boundary reason as SrgbToLinear above.
inline void SrgbToLinearRgb(const float srgb[3], float linear[3]) {
  for (int j = 0; j < 3; j++) {
    linear[j] = SrgbToLinear(srgb[j]);
  }
}

}  // namespace lumice

#endif  // SRC_UTIL_COLOR_SPACE_H_
