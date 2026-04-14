#ifndef CONFIG_RAYPATH_VALIDATION_HPP_
#define CONFIG_RAYPATH_VALIDATION_HPP_

#include <cctype>
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
inline RaypathValidation ValidateRaypathText(const std::string& text) {
  if (text.empty()) {
    return RaypathValidation::kValid;
  }

  // Check leading separator
  bool has_leading_sep = (text.front() == '-' || text.front() == ',');
  // Check trailing separator
  bool has_trailing_sep = (text.back() == '-' || text.back() == ',');

  if (has_trailing_sep || has_leading_sep) {
    // Could still be kInvalid if there are bad tokens in the middle.
    // But first check: if the entire string is just separators, it's kIncomplete.
    bool all_separators = true;
    for (char c : text) {
      if (c != '-' && c != ',') {
        all_separators = false;
        break;
      }
    }
    if (all_separators) {
      return RaypathValidation::kIncomplete;
    }
  }

  // Tokenize: split by '-' and ','
  // Walk through the string, split on separators, validate each token.
  bool prev_was_sep = false;
  bool in_token = false;
  bool found_interior_empty = false;
  bool found_bad_token = false;
  int token_start = 0;
  int token_count = 0;

  for (int i = 0; i <= static_cast<int>(text.size()); ++i) {
    bool is_sep = (i < static_cast<int>(text.size())) && (text[i] == '-' || text[i] == ',');
    bool is_end = (i == static_cast<int>(text.size()));

    if (is_sep || is_end) {
      if (in_token) {
        // Validate the token [token_start, i)
        std::string token = text.substr(token_start, i - token_start);
        // Check: all characters must be digits.
        // Note: token is guaranteed non-empty because in_token is only set when a non-separator char is seen.
        bool all_digits = true;
        for (char c : token) {
          if (!std::isdigit(static_cast<unsigned char>(c))) {
            all_digits = false;
            break;
          }
        }
        if (!all_digits) {
          found_bad_token = true;
        }
        token_count++;
        in_token = false;
      } else if (is_sep && prev_was_sep) {
        // Consecutive separators in the middle → empty interior token
        found_interior_empty = true;
      }
      prev_was_sep = is_sep;
    } else {
      if (!in_token) {
        token_start = i;
        in_token = true;
      }
      prev_was_sep = false;
    }
  }

  if (found_bad_token || found_interior_empty) {
    return RaypathValidation::kInvalid;
  }

  if (has_trailing_sep || has_leading_sep) {
    return RaypathValidation::kIncomplete;
  }

  return RaypathValidation::kValid;
}

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

}  // namespace lumice

#endif  // CONFIG_RAYPATH_VALIDATION_HPP_
