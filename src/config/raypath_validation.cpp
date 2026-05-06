#include "config/raypath_validation.hpp"

#include <cassert>
#include <cctype>
#include <climits>
#include <string>

#include "core/crystal.hpp"

namespace lumice {

namespace {

bool IsSeparator(char c) {
  return c == '-' || c == ',';
}

bool AllSeparators(const std::string& text) {
  for (char c : text) {
    if (!IsSeparator(c)) {
      return false;
    }
  }
  return true;
}

bool TokenAllDigits(const std::string& text, int begin, int end) {
  for (int i = begin; i < end; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(text[i]))) {
      return false;
    }
  }
  return true;
}

// Walk `text`, detect bad (non-numeric) tokens and interior-empty tokens.
void ScanTokens(const std::string& text, bool& found_bad_token, bool& found_interior_empty) {
  bool prev_was_sep = false;
  bool in_token = false;
  int token_start = 0;
  int n = static_cast<int>(text.size());
  for (int i = 0; i <= n; ++i) {
    bool is_end = (i == n);
    bool is_sep = !is_end && IsSeparator(text[i]);
    if (is_sep || is_end) {
      if (in_token) {
        if (!TokenAllDigits(text, token_start, i)) {
          found_bad_token = true;
        }
        in_token = false;
      } else if (is_sep && prev_was_sep) {
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
}

}  // namespace

RaypathValidation ValidateRaypathText(const std::string& text) {
  if (text.empty()) {
    return RaypathValidation::kValid;
  }
  bool has_leading_sep = IsSeparator(text.front());
  bool has_trailing_sep = IsSeparator(text.back());
  if ((has_leading_sep || has_trailing_sep) && AllSeparators(text)) {
    return RaypathValidation::kIncomplete;
  }
  bool found_bad_token = false;
  bool found_interior_empty = false;
  ScanTokens(text, found_bad_token, found_interior_empty);
  if (found_bad_token || found_interior_empty) {
    return RaypathValidation::kInvalid;
  }
  if (has_leading_sep || has_trailing_sep) {
    return RaypathValidation::kIncomplete;
  }
  return RaypathValidation::kValid;
}

namespace {

// Global union of legal face numbers across every supported CrystalKind.
// By delegating to IsLegalFace(kPyramid, ...) we keep the canonical legal
// set definition in one place (core/crystal.cpp). This relies on the
// invariant — enforced by the IsLegalFaceTest.PyramidSetEqualsValidatorGlobalStage
// contract test — that the union of all kinds currently equals the pyramid
// legal set. If a future CrystalKind introduces faces outside that set, the
// global function must be generalised to OR-combine every kind.
bool IsLegalFaceGlobal(int face) {
  return IsLegalFace(CrystalKind::kPyramid, face);
}

const char* CrystalKindLabel(CrystalKind kind) {
  switch (kind) {
    case CrystalKind::kPrism:
      return "Prism";
    case CrystalKind::kPyramid:
      return "Pyramid";
  }
  // Unreachable — new enum values must extend the switch above.
  assert(false && "CrystalKindLabel: unhandled CrystalKind");
  return "Unknown";
}

// Maximum digits permitted in a face-number token. Legal face numbers are
// at most two digits (up to 28), so three digits is already far above any
// valid input. Capping digit counts keeps the int accumulator safe from
// overflow for pathological inputs like "9999999999" that pass the
// syntax-only validator.
constexpr int kMaxFaceDigits = 3;

// Parse the next numeric token starting at `pos`, advancing `pos` past the
// token and any following separator. Returns true and writes `face` on
// success; returns false if the token is not a non-negative integer.
// The caller has already verified via the syntax-only overload that every
// token is a well-formed non-negative integer, so this helper mainly
// recovers the integer value. If the token has more than kMaxFaceDigits
// digits, the remaining digits are consumed and `face` is set to INT_MAX,
// which guarantees IsLegalFaceGlobal returns false (avoiding int overflow
// UB while still reporting the token as illegal).
bool ExtractNextFace(const std::string& text, size_t& pos, int& face) {
  // Skip any separators.
  while (pos < text.size() && (text[pos] == '-' || text[pos] == ',')) {
    ++pos;
  }
  if (pos >= text.size()) {
    return false;
  }
  int value = 0;
  int digit_count = 0;
  bool overflow = false;
  while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) {
    if (digit_count < kMaxFaceDigits) {
      value = value * 10 + (text[pos] - '0');
    } else {
      overflow = true;
    }
    ++digit_count;
    ++pos;
  }
  if (digit_count == 0) {
    return false;
  }
  face = overflow ? INT_MAX : value;
  return true;
}

}  // namespace

RaypathValidationResult ValidateRaypathText(const std::string& text, CrystalKind kind) {
  auto syntax_state = ValidateRaypathText(text);
  if (syntax_state != RaypathValidation::kValid) {
    RaypathValidationResult r;
    r.state = syntax_state;
    r.message = (syntax_state == RaypathValidation::kInvalid) ? "Invalid raypath" : std::string{};
    return r;
  }

  // Syntax passed; walk every token and check face-number legality.
  size_t pos = 0;
  int face = 0;
  while (ExtractNextFace(text, pos, face)) {
    if (!IsLegalFaceGlobal(face)) {
      RaypathValidationResult r;
      r.state = RaypathValidation::kInvalid;
      r.message = "Face " + std::to_string(face) + " is outside the legal range of any crystal";
      return r;
    }
    if (!IsLegalFace(kind, face)) {
      RaypathValidationResult r;
      r.state = RaypathValidation::kInvalid;
      r.message =
          "Face " + std::to_string(face) + " is not legal on this crystal type (" + CrystalKindLabel(kind) + ")";
      return r;
    }
  }

  return RaypathValidationResult{ RaypathValidation::kValid, std::string{} };
}

RaypathValidationResult ValidateFaceNumberText(const std::string& text, CrystalKind kind) {
  if (text.empty()) {
    return RaypathValidationResult{ RaypathValidation::kIncomplete, std::string{} };
  }
  // Reject any non-digit character outright — separators ('-', ',') are
  // raypath syntax and carry no meaning for a single face-number field.
  for (char c : text) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return RaypathValidationResult{ RaypathValidation::kInvalid, "must be a single non-negative integer" };
    }
  }
  if (text.size() > static_cast<size_t>(kMaxFaceDigits)) {
    return RaypathValidationResult{ RaypathValidation::kInvalid, "face number out of range" };
  }
  // Safe to parse: at most kMaxFaceDigits digits, fits in int.
  int face = 0;
  for (char c : text) {
    face = face * 10 + (c - '0');
  }
  if (!IsLegalFaceGlobal(face)) {
    return RaypathValidationResult{ RaypathValidation::kInvalid,
                                    "Face " + std::to_string(face) + " is outside the legal range of any crystal" };
  }
  if (!IsLegalFace(kind, face)) {
    return RaypathValidationResult{ RaypathValidation::kInvalid, "Face " + std::to_string(face) +
                                                                     " is not legal on this crystal type (" +
                                                                     CrystalKindLabel(kind) + ")" };
  }
  return RaypathValidationResult{ RaypathValidation::kValid, std::string{} };
}

}  // namespace lumice
