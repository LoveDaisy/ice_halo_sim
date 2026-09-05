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

// The scalar sRGB transfer curve, both directions. Used at the JSON boundary: the "background" key
// is authored in sRGB (what a colour picker shows), while the struct field it lands in is linear
// (what additive blending needs).
//
// ROUND TRIP — the property this pair has to have, and the two separate reasons it did not.
//
// A document that is saved and reloaded goes sRGB -> linear -> sRGB -> linear, and the two linear
// values must be BIT-IDENTICAL. RenderConfig::operator== is what decides whether a config survived
// the C API round trip (test/unit-correctness/server/test_json_parser_parity.cpp) and whether an
// edit is a change at all, and it compares floats exactly. One ULP of colour is invisible; a
// spurious "this document changed" is not.
//
//  1. The curved branch is computed in DOUBLE and narrowed to float once, on return. Computed
//     entirely in float, whether the trip closed came down to how the host libm happened to round
//     powf, and the hosts disagreed: exhaustive scan of every float32 in [0,1] against the pre-fix
//     code found 6.9M of 1.07e9 sRGB inputs failing to return to their own linear value on
//     AppleClang and 4.1M on glibc — overlapping but DIFFERENT sets, which is exactly why CI went
//     red on Ubuntu and Windows and stayed green on macOS for background 0.05 in Sep 2026. It was
//     never a platform quirk; the platforms merely disagreed about which values it hit.
//  2. The two branches select on the SAME predicate applied to the SAME float. The comment that
//     used to sit here claimed the thresholds are one point in two spaces ("0.0031308 * 12.92 =
//     0.04045") — they are not: 0.0031308 * 12.92 = 0.040449936, so sRGB inputs in
//     [0.040449936, 0.04045) took the straight branch on the way in and the curved one on the way
//     back. No amount of precision makes a pair that disagrees about its own domain split
//     invertible; that is what the residual 18 failures left over by (1) alone turned out to be.
//     SrgbToLinear therefore branches on the value it is about to RETURN, using LinearToSrgb's own
//     cutoff, rather than on a separately-spelled sRGB-space constant.
//
// With both in place the trip is bit-exact for every float32 in [0,1], in both directions, on
// AppleClang and glibc alike: 0 of 1.07e9 each way, measured. Pinned by
// ColorSpace.GammaRoundTripIsBitExact — keep the two branch predicates in step if either function
// is touched again.
//
// COST, since it is not zero and the denominator matters: double-precision pow makes
// XyzToSrgbUint8 about 48% slower (1024x1024, realistic halo content, 22.3 -> 33.0 ms measured),
// and that per-pixel gamma is on the server's per-drain snapshot render — a path whose cost has
// been diagnosed as catastrophic once already, when the `--benchmark` poll loop read sim_ray_num by
// acquiring a result frame and so paid that render on every drain (see the comment above the
// `LUMICE_GetSimRayCount` call in `src/main.cpp`). Against the end-to-end denominator that
// sensitivity is about, though, it does not surface: the same CLI run (1024x1024 equal-area, grid overlays, one
// render, image written) measured 1132.9 ms before and 1134.7 ms after — +1.8 ms, 0.15%, inside
// the 7 ms run-to-run spread, A/B'd by swapping only this header and confirming the two binaries
// differ. What that number does NOT cover, and is left standing as a known limit: the GUI's
// per-snapshot CPU texture update goes through the bulk XyzToSrgbUint8 above rather than this
// render path, so it carries the isolated +10.7 ms per snapshot, on a clock decoupled from the
// display's. It is not on the ray-throughput path, and nothing cheaper
// works: keeping this direction in float and snapping the other onto its exact inverse was tried
// and measured, and cannot close the trip (the analytic starting point is already several ULPs out
// on exactly the inputs at issue, so no bounded neighbourhood search finds the right float). If
// the snapshot render ever needs the time back, the way to get it is to give the BULK uint8 paths
// a 256-entry threshold table derived from this curve — faster than the float version was, and
// still one definition of the curve.
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
// Where the straight segment gives way to the curve, in LINEAR space. Single-sourced on purpose:
// both directions decide with this one constant, so there is no second spelling of the same
// threshold to drift out of step (see point 2 above).
constexpr float kSrgbCurveCutoffLinear = 0.0031308f;

inline float LinearToSrgb(float linear) {
  if (linear < kSrgbCurveCutoffLinear) {
    return static_cast<float>(static_cast<double>(linear) * 12.92);
  }
  return static_cast<float>(1.055 * std::pow(static_cast<double>(linear), 1.0 / 2.4) - 0.055);
}

inline float SrgbToLinear(float srgb) {
  // Branch on the straight-segment RESULT, not on an sRGB-space cutoff: this is literally the
  // predicate LinearToSrgb will apply, to literally the float it will be handed.
  const float straight = static_cast<float>(static_cast<double>(srgb) / 12.92);
  if (straight < kSrgbCurveCutoffLinear) {
    return straight;
  }
  return static_cast<float>(std::pow((static_cast<double>(srgb) + 0.055) / 1.055, 2.4));
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
