#ifndef CONFIG_RAYPATH_VALIDATION_HPP_
#define CONFIG_RAYPATH_VALIDATION_HPP_

#include <string>

#include "core/crystal_kind.hpp"

namespace lumice {

/// Validation state for raypath text input.
enum class RaypathValidation {
  kValid,       ///< All tokens are non-negative integers; safe to submit.
  kIncomplete,  ///< Trailing/leading separator; user is still typing.
  kInvalid,     ///< Contains non-numeric tokens or empty interior tokens.
};

/// Richer validation result carrying an optional human-readable message.
///
/// Used by the two-argument `ValidateRaypathText` overload so the GUI can read
/// both the state (for border colour / OK-button gating) and a specific error
/// description (e.g. "Face 13 is not legal on this crystal type (Prism)") in a
/// single call. `message` is empty when `state == kValid`.
struct RaypathValidationResult {
  RaypathValidation state;
  std::string message;
};

/// Validate a raypath text string (dash- or comma-separated face indices).
///
/// Rules (evaluated in priority order):
///   - Empty string → kValid (means "no raypath filter").
///   - Consecutive separators anywhere (e.g. "3--5", "--3") → kInvalid.
///   - Any token that is not a non-negative integer → kInvalid.
///   - Trailing separator (e.g. "3-5-") → kIncomplete.
///   - Leading separator (e.g. "-3") → kIncomplete.
///   - All tokens are non-negative integers → kValid.
///
/// This function is pure-stdlib (no GUI dependency) and suitable for unit testing.
RaypathValidation ValidateRaypathText(const std::string& text);

/// Validate a raypath text string against both syntax rules and face-number
/// legality for the given crystal kind.
///
/// Semantics:
///   - Syntax is checked first by delegating to the single-argument overload.
///     If the syntax state is not kValid, the result is returned immediately
///     (message = "Invalid raypath" for kInvalid, empty for kIncomplete).
///   - Otherwise every numeric token is checked against the global union of
///     legal face numbers ({1,2,3..8,13..18,23..28}); the first token outside
///     that union yields a "Face N is outside the legal range of any crystal"
///     message.
///   - Finally each token is checked against the kind-specific legal set via
///     `IsLegalFace(kind, face)`; the first token that fails yields a
///     "Face N is not legal on this crystal type (Prism/Pyramid)" message.
///
/// The single-argument `ValidateRaypathText(text)` overload remains unchanged
/// and performs syntax-only validation (so e.g. `"51"` is still kValid).
RaypathValidationResult ValidateRaypathText(const std::string& text, CrystalKind kind);

/// Validate a single face-number text input (used by the Entry-Exit filter
/// sub-panel). Unlike `ValidateRaypathText` this rejects separators
/// outright — Entry / Exit each take exactly one face number.
///
/// Rules:
///   - Empty string → kIncomplete (user is still typing; no error message).
///   - Any non-digit character (incl. separators '-' / ',') → kInvalid
///     ("must be a single non-negative integer").
///   - Token longer than 3 digits → kInvalid ("face number out of range").
///   - Numeric value not in the global legal-face union → kInvalid
///     ("Face N is outside the legal range of any crystal").
///   - Numeric value not legal on `kind` → kInvalid ("Face N is not legal on
///     this crystal type (Prism/Pyramid)").
///   - Otherwise → kValid.
RaypathValidationResult ValidateFaceNumberText(const std::string& text, CrystalKind kind);

}  // namespace lumice

#endif  // CONFIG_RAYPATH_VALIDATION_HPP_
