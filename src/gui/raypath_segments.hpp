#ifndef LUMICE_GUI_RAYPATH_SEGMENTS_HPP
#define LUMICE_GUI_RAYPATH_SEGMENTS_HPP

#include <cctype>
#include <cstdio>
#include <string>
#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

// Validation result for GUI raypath / face-number inputs.
struct GuiValidationResult {
  LUMICE_RaypathValidationState state;
  std::string message;
};

// Trim ASCII whitespace from both ends of a string.
inline std::string TrimRaypathSegment(const std::string& s) {
  size_t b = 0;
  size_t e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

// Split a raypath text by ';' and trim each segment of surrounding whitespace.
//
// Behavior:
//   - Empty input → empty vector.
//   - No ';' → 1-element vector with the trimmed text.
//   - Multiple ';' → segments split as-is, *including* any empty-after-trim
//     segments (e.g. ";3" → ["", "3"], "3;" → ["3", ""], "3;;5" → ["3", "", "5"]).
//     Callers that need to reject empty segments should use
//     ValidateRaypathTextMultiSegment which performs that check before splitting.
inline std::vector<std::string> SplitRaypathSegments(const std::string& text) {
  std::vector<std::string> out;
  if (text.empty()) {
    return out;
  }
  std::string cur;
  for (char c : text) {
    if (c == ';') {
      out.push_back(TrimRaypathSegment(cur));
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(TrimRaypathSegment(cur));
  return out;
}

// Validate a single face-number text input. Unlike LUMICE_ValidateRaypathText,
// rejects separators outright — entry/exit fields each take exactly one face number.
// Message for kind-specific rejection contains "not legal on this crystal type"
// so that ParseFaceNumberOrZero can distinguish it from a syntax error.
inline GuiValidationResult GuiValidateFaceNumberText(const std::string& text, LUMICE_CrystalKind kind) {
  GuiValidationResult r;
  if (text.empty()) {
    r.state = LUMICE_RAYPATH_INCOMPLETE;
    return r;
  }
  if (text.size() > 3) {
    r.state = LUMICE_RAYPATH_INVALID;
    r.message = "face number out of range";
    return r;
  }
  for (char c : text) {
    if (c < '0' || c > '9') {
      r.state = LUMICE_RAYPATH_INVALID;
      r.message = "must be a single non-negative integer";
      return r;
    }
  }
  int face = std::stoi(text);
  if (!LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, face)) {
    r.state = LUMICE_RAYPATH_INVALID;
    r.message = "Face " + std::to_string(face) + " is outside the legal range of any crystal";
    return r;
  }
  if (!LUMICE_IsLegalFace(kind, face)) {
    const char* kind_name = (kind == LUMICE_CRYSTAL_PRISM) ? "Prism" : "Pyramid";
    r.state = LUMICE_RAYPATH_INVALID;
    r.message = "Face " + std::to_string(face) + " is not legal on this crystal type (" + std::string(kind_name) + ")";
    return r;
  }
  r.state = LUMICE_RAYPATH_VALID;
  return r;
}

// Validate a comma-separated face-number list (Entry-Exit multi-value OR
// input). Empty text = wildcard ("any face") → kValid. Trailing comma →
// kIncomplete. Internal empty token (e.g. "3,,4") → kInvalid. Each
// non-empty token is delegated to GuiValidateFaceNumberText so the
// kind-specific legality message stays identical to the single-value path.
inline GuiValidationResult GuiValidateFaceNumberListText(const std::string& text, LUMICE_CrystalKind kind) {
  GuiValidationResult r;
  if (text.empty()) {
    r.state = LUMICE_RAYPATH_VALID;
    return r;
  }
  if (text.back() == ',') {
    r.state = LUMICE_RAYPATH_INCOMPLETE;
    return r;
  }
  size_t pos = 0;
  while (pos < text.size()) {
    size_t end = text.find(',', pos);
    if (end == std::string::npos) {
      end = text.size();
    }
    if (end == pos) {
      r.state = LUMICE_RAYPATH_INVALID;
      r.message = "Empty face number in list";
      return r;
    }
    auto tok = text.substr(pos, end - pos);
    auto sr = GuiValidateFaceNumberText(tok, kind);
    if (sr.state != LUMICE_RAYPATH_VALID) {
      return sr;
    }
    pos = end + 1;
  }
  r.state = LUMICE_RAYPATH_VALID;
  return r;
}

// Validate raypath text supporting the multi-segment ';' syntax.
//
// Job split (per plan):
//   - SplitRaypathSegments only tokenizes; it does *not* infer "this input
//     contains an empty segment".
//   - ValidateRaypathTextMultiSegment pre-checks the raw text for leading/
//     trailing ';' or repeated ';' (with optional whitespace between) before
//     calling the splitter, so inputs like ";3" / "3;" / "3;;5" are rejected
//     up-front rather than relying on splitter output.
//   - Each non-empty segment is then validated by LUMICE_ValidateRaypathText.
//   - Empty input → kValid (means "no filter").
//   - Single-segment input (no ';') → delegates to the single-segment validator
//     so existing behavior is preserved exactly (incl. kIncomplete states for
//     trailing dashes / commas).
inline GuiValidationResult ValidateRaypathTextMultiSegment(const std::string& text, LUMICE_CrystalKind kind) {
  GuiValidationResult result;

  if (text.empty()) {
    result.state = LUMICE_RAYPATH_VALID;
    return result;
  }

  // No ';' → preserve single-segment semantics.
  if (text.find(';') == std::string::npos) {
    char msg[256] = {};
    LUMICE_RaypathValidationState seg_state = LUMICE_RAYPATH_VALID;
    LUMICE_ValidateRaypathText(text.c_str(), kind, &seg_state, msg, sizeof(msg));
    result.state = seg_state;
    result.message = msg;
    return result;
  }

  // Pre-flight: scan for leading / trailing / consecutive ';' (allowing
  // whitespace between separators). Hits → reject as empty segment, *before*
  // splitter loses that information.
  bool sep_pending = true;  // start as if previous char was ';' to catch leading ';'
  for (char c : text) {
    if (c == ';') {
      if (sep_pending) {
        result.state = LUMICE_RAYPATH_INVALID;
        result.message = "Empty raypath segment";
        return result;
      }
      sep_pending = true;
    } else if (!std::isspace(static_cast<unsigned char>(c))) {
      sep_pending = false;
    }
  }
  // Trailing ';' (sep_pending still true after loop) or pure-whitespace input.
  if (sep_pending) {
    result.state = LUMICE_RAYPATH_INVALID;
    result.message = "Empty raypath segment";
    return result;
  }

  // Split and validate each segment.
  auto segs = SplitRaypathSegments(text);
  size_t idx = 0;
  for (const auto& seg : segs) {
    ++idx;
    if (seg.empty()) {
      // Defensive — pre-flight should already have caught this.
      result.state = LUMICE_RAYPATH_INVALID;
      result.message = "Empty raypath segment " + std::to_string(idx);
      return result;
    }
    char msg[256] = {};
    LUMICE_RaypathValidationState seg_state = LUMICE_RAYPATH_VALID;
    LUMICE_ValidateRaypathText(seg.c_str(), kind, &seg_state, msg, sizeof(msg));
    if (seg_state != LUMICE_RAYPATH_VALID) {
      result.state = seg_state;
      result.message = "Segment " + std::to_string(idx) + ": " + msg;
      return result;
    }
  }

  result.state = LUMICE_RAYPATH_VALID;
  return result;
}

// Parse a single segment (no ';') into a vector of non-negative face indices.
// Tolerant: skips invalid tokens, matching the existing ParseRaypathText behavior.
inline std::vector<int> ParseRaypathSegment(const std::string& seg) {
  std::vector<int> ints;
  // Normalize ',' → '-' so both separators work.
  std::string normalized = seg;
  for (auto& c : normalized) {
    if (c == ',') {
      c = '-';
    }
  }
  std::string tok;
  auto flush = [&ints, &tok]() {
    if (tok.empty()) {
      return;
    }
    try {
      int v = std::stoi(tok);
      if (v >= 0) {
        ints.push_back(v);
      }
    } catch (...) {
    }
    tok.clear();
  };
  for (char c : normalized) {
    if (c == '-') {
      flush();
    } else {
      tok.push_back(c);
    }
  }
  flush();
  return ints;
}

// Parse multi-segment raypath text into a vector of int vectors, one per
// segment. Tolerant: skips invalid tokens within each segment. Empty /
// pure-whitespace segments are dropped from the result. Callers that need
// strict rejection of malformed input should run
// ValidateRaypathTextMultiSegment first.
inline std::vector<std::vector<int>> ParseRaypathTextMultiSegment(const std::string& text) {
  std::vector<std::vector<int>> out;
  if (text.empty()) {
    return out;
  }
  auto segs = SplitRaypathSegments(text);
  for (const auto& seg : segs) {
    if (seg.empty()) {
      continue;
    }
    auto ints = ParseRaypathSegment(seg);
    if (!ints.empty()) {
      out.push_back(std::move(ints));
    }
  }
  return out;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_RAYPATH_SEGMENTS_HPP
