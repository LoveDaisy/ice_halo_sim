#include "config/raypath_validation.hpp"

#include <cassert>
#include <cctype>
#include <climits>
#include <string>

#include "core/crystal.hpp"

namespace lumice {

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
  if (pos >= text.size())
    return false;
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
  if (digit_count == 0)
    return false;
  face = overflow ? INT_MAX : value;
  return true;
}

}  // namespace

RaypathValidationResult ValidateRaypathText(const std::string& text, CrystalKind kind) {
  const auto syntax_state = ValidateRaypathText(text);
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

}  // namespace lumice
