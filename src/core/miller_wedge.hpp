#ifndef SRC_CORE_MILLER_WEDGE_HPP_
#define SRC_CORE_MILLER_WEDGE_HPP_

namespace lumice {

/// Verdict on one Miller-index triple offered as a pyramidal wedge angle.
enum class MillerConversionState {
  kValid,       ///< Three indices, all well-formed, and the angle they make is buildable.
  kNoCone,      ///< h == 0: this side of the crystal has no pyramidal cap. Angle is 0.
  kIncomplete,  ///< Fewer than three indices supplied; the caller is still collecting them.
  kInvalid,     ///< Too many indices, k != 0, a negative index, or an unbuildable angle.
};

/// Result of `ConvertMillerIndexToWedgeAngle`.
///
/// `wedge_angle_deg` carries a meaningful number only for `kValid` and `kNoCone`; it is 0 for the
/// other two states, and a caller that reads it there is reading "no opinion", not "0 degrees".
///
/// `invalid_index` names which of the three slots the verdict can be blamed on -- 0 = h, 1 = k,
/// 2 = l -- and is -1 whenever no single slot is at fault. The -1 cases are deliberate, not
/// laziness: a wrong index *count* is not any one slot's doing, and an angle outside the buildable
/// range comes from the *ratio* h:l, in which two individually well-formed integers combine into
/// an unbuildable face. Pointing at either one of them would mislead a caller that highlights the
/// named field.
struct MillerConversionResult {
  MillerConversionState state = MillerConversionState::kIncomplete;
  float wedge_angle_deg = 0.0f;
  int invalid_index = -1;
};

/// The single owner of "what wedge angle do these Miller indices mean, and are they even legal".
///
/// Takes the reduced three-index wire form (h, k, l) -- the same three numbers the
/// `upper_indices` / `lower_indices` JSON arrays hold, and the same three a GUI input row would
/// collect. The redundant fourth Miller-Bravais index i = -(h + k) is not passed: it is derivable,
/// so accepting it would mean accepting a value that can disagree with the other two.
///
/// `provided_count` is how many of h/k/l the caller has actually been given, which is what lets one
/// function serve both a JSON array (pass its own `size()`, so a 2- or 4-element array is judged
/// rather than skipped) and a GUI row being typed into (pass the number of filled boxes, so a
/// half-entered triple reads as `kIncomplete` rather than as an error). Slots the count says were
/// not supplied are ignored, whatever was passed in them.
///
/// Verdicts, in the order they are decided:
///   1. `provided_count` < 3            -> kIncomplete (-1)
///   2. `provided_count` > 3            -> kInvalid (-1)
///   3. k != 0                          -> kInvalid (1): a second-order pyramidal face rotated 30
///                                        degrees off the prism edges is not a shape this crystal
///                                        model can express, so this is unrepresentable input
///                                        rather than a miscalculation.
///   4. h < 0                           -> kInvalid (0)
///   5. l < 0                           -> kInvalid (2)
///   6. h == 0                          -> kNoCone (-1), angle 0
///   7. angle outside (0, 90) degrees   -> kInvalid (-1); the bounds match the ones
///      or non-finite                     `FillHexCrystalCoef` applies when it decides whether to
///                                        emit pyramidal planes at all, so an angle this returns
///                                        as kValid is one that actually produces faces.
///   8. otherwise                       -> kValid (-1), angle in degrees
MillerConversionResult ConvertMillerIndexToWedgeAngle(int h, int k, int l, int provided_count);

}  // namespace lumice

#endif  // SRC_CORE_MILLER_WEDGE_HPP_
