#ifndef LUMICE_GUI_RAYPATH_SEGMENTS_HPP
#define LUMICE_GUI_RAYPATH_SEGMENTS_HPP

#include <cctype>
#include <string>
#include <vector>

#include "config/raypath_validation.hpp"
#include "core/crystal_kind.hpp"

namespace lumice::gui {

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

// Validate raypath text supporting the multi-segment ';' syntax.
//
// Job split (per plan):
//   - SplitRaypathSegments only tokenizes; it does *not* infer "this input
//     contains an empty segment".
//   - ValidateRaypathTextMultiSegment pre-checks the raw text for leading/
//     trailing ';' or repeated ';' (with optional whitespace between) before
//     calling the splitter, so inputs like ";3" / "3;" / "3;;5" are rejected
//     up-front rather than relying on splitter output.
//   - Each non-empty segment is then validated by the kind-aware single-segment
//     validator from core (ValidateRaypathText overload).
//   - Empty input → kValid (means "no filter").
//   - Single-segment input (no ';') → delegates to the single-segment validator
//     so existing behavior is preserved exactly (incl. kIncomplete states for
//     trailing dashes / commas).
inline RaypathValidationResult ValidateRaypathTextMultiSegment(const std::string& text, CrystalKind kind) {
  RaypathValidationResult result;

  if (text.empty()) {
    result.state = RaypathValidation::kValid;
    return result;
  }

  // No ';' → preserve single-segment semantics.
  if (text.find(';') == std::string::npos) {
    return ValidateRaypathText(text, kind);
  }

  // Pre-flight: scan for leading / trailing / consecutive ';' (allowing
  // whitespace between separators). Hits → reject as empty segment, *before*
  // splitter loses that information.
  bool sep_pending = true;  // start as if previous char was ';' to catch leading ';'
  for (char c : text) {
    if (c == ';') {
      if (sep_pending) {
        result.state = RaypathValidation::kInvalid;
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
    result.state = RaypathValidation::kInvalid;
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
      result.state = RaypathValidation::kInvalid;
      result.message = "Empty raypath segment " + std::to_string(idx);
      return result;
    }
    auto r = ValidateRaypathText(seg, kind);
    if (r.state != RaypathValidation::kValid) {
      result.state = r.state;
      result.message = "Segment " + std::to_string(idx) + ": " + r.message;
      return result;
    }
  }

  result.state = RaypathValidation::kValid;
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
