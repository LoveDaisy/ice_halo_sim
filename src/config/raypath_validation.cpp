#include "config/raypath_validation.hpp"

#include <cctype>
#include <string>

#include "core/crystal.hpp"

namespace lumice {

namespace {

// Global union of legal face numbers across every supported CrystalKind.
// Currently equal to the pyramid legal set; if new kinds extend the set,
// both this function and IsLegalFace (in crystal.cpp) must be updated.
bool IsLegalFaceGlobal(int face) {
  if (face == 1 || face == 2)
    return true;
  if (face >= 3 && face <= 8)
    return true;
  if (face >= 13 && face <= 18)
    return true;
  if (face >= 23 && face <= 28)
    return true;
  return false;
}

const char* CrystalKindLabel(CrystalKind kind) {
  switch (kind) {
    case CrystalKind::kPrism:
      return "Prism";
    case CrystalKind::kPyramid:
      return "Pyramid";
  }
  return "Unknown";
}

// Parse the next numeric token starting at `pos`, advancing `pos` past the
// token and any following separator. Returns true and writes `face` on
// success; returns false if the token is not a non-negative integer.
// The caller has already verified via the syntax-only overload that every
// token is a well-formed non-negative integer, so this helper mainly
// recovers the integer value.
bool ExtractNextFace(const std::string& text, size_t& pos, int& face) {
  // Skip any separators.
  while (pos < text.size() && (text[pos] == '-' || text[pos] == ',')) {
    ++pos;
  }
  if (pos >= text.size())
    return false;
  int value = 0;
  bool has_digit = false;
  while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) {
    value = value * 10 + (text[pos] - '0');
    has_digit = true;
    ++pos;
  }
  if (!has_digit)
    return false;
  face = value;
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
