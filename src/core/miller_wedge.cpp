#include "core/miller_wedge.hpp"

#include <cmath>

#include "core/geo3d.hpp"

namespace lumice {

namespace {

// Buildable wedge-angle bounds, in degrees. These mirror kMinPyramidAlpha / kMaxPyramidAlpha in
// FillHexCrystalCoef (core/geo3d.cpp), which is the predicate that actually decides whether a
// pyramidal face is emitted: below the lower bound the face is vertical (a1 overflows), above the
// upper bound it lies flat against the basal face. Judging with looser bounds here would let this
// function answer kValid for an angle the mesh builder then silently drops.
constexpr float kMinWedgeAngleDeg = 0.1f;
constexpr float kMaxWedgeAngleDeg = 89.9f;

}  // namespace

MillerConversionResult ConvertMillerIndexToWedgeAngle(int h, int k, int l, int provided_count) {
  MillerConversionResult r;
  if (provided_count < 3) {
    r.state = MillerConversionState::kIncomplete;
    return r;
  }
  if (provided_count > 3) {
    r.state = MillerConversionState::kInvalid;
    return r;
  }
  if (k != 0) {
    r.state = MillerConversionState::kInvalid;
    r.invalid_index = 1;
    return r;
  }
  // A negative index is malformed on its own terms, so it is checked before the angle: two negative
  // indices make a positive ratio and would otherwise sail through as a perfectly ordinary angle.
  if (h < 0) {
    r.state = MillerConversionState::kInvalid;
    r.invalid_index = 0;
    return r;
  }
  if (l < 0) {
    r.state = MillerConversionState::kInvalid;
    r.invalid_index = 2;
    return r;
  }
  if (h == 0) {
    r.state = MillerConversionState::kNoCone;
    r.wedge_angle_deg = 0.0f;
    return r;
  }
  float angle = MillerIndexToWedgeAngleDeg(h, l);
  if (!std::isfinite(angle) || angle < kMinWedgeAngleDeg || angle > kMaxWedgeAngleDeg) {
    r.state = MillerConversionState::kInvalid;
    return r;
  }
  r.state = MillerConversionState::kValid;
  r.wedge_angle_deg = angle;
  return r;
}

}  // namespace lumice
