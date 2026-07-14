#ifndef LUMICE_GUI_RAYPATH_SEGMENTS_HPP
#define LUMICE_GUI_RAYPATH_SEGMENTS_HPP

#include <cctype>
#include <charconv>
#include <cstdio>
#include <string>
#include <variant>
#include <vector>

#include "gui/gui_state.hpp"  // Factor / SummandText / SumOfProducts / FormatEntryExitFactorText
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
      out.emplace_back(TrimRaypathSegment(cur));
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
//
// Logic mirrors lumice::ValidateFaceNumberListText in
// src/config/raypath_validation.cpp; keep in sync. The GUI tier cannot
// include core headers directly (AGENTS.md §"Public API boundary"), hence
// the local copy.
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

// ---------------------------------------------------------------------------
// AND-of-factors small-domain grammar (task-gui-sop-data-model).
//
// Row := factor_token ( '&' factor_token )*        // whitespace tolerated
// factor_token := raypath_token | ee_token
// raypath_token := single-segment raypath text (no ';')
// ee_token    := 'entry:' facelist | 'exit:' facelist | 'len:' lengthspec
// facelist    := comma-separated face-number list (empty → wildcard)
// lengthspec  := 'N' (strict) | '<=N' (at most) | 'N-M' (range)
//
// Same-row entry:/exit:/len: tokens merge into a single EntryExitParams factor
// (per plan §3.3). Raypath tokens produce independent RaypathParams factors
// and flush any in-flight EE builder first.
//
// ⚠️ Cross-file: the EE token *formatter* (the inverse of ee_token parsing) is
// `FormatEntryExitFactorText` in `gui_state.hpp` — kept there to avoid a reverse
// include. When extending this grammar, update BOTH sides.
// ---------------------------------------------------------------------------

namespace detail {

// Split a summand row on '&' into trimmed AND-factor tokens. Internal grammar
// helper (not part of the public grammar interface) — hence in `detail::`.
inline std::vector<std::string> SplitSummandTokens(const std::string& text) {
  std::vector<std::string> out;
  std::string cur;
  bool any = false;
  for (char c : text) {
    if (c == '&') {
      out.emplace_back(TrimRaypathSegment(cur));
      cur.clear();
      any = true;
    } else {
      cur.push_back(c);
    }
  }
  if (any || !cur.empty()) {
    out.emplace_back(TrimRaypathSegment(cur));
  }
  return out;
}

// True if `s` begins with `prefix` (case-sensitive).
inline bool StartsWith(const std::string& s, const char* prefix) {
  size_t i = 0;
  while (prefix[i] != '\0') {
    if (i >= s.size() || s[i] != prefix[i]) {
      return false;
    }
    ++i;
  }
  return true;
}

// Parse a lengthspec into (mode, min_len, max_len). Returns true iff spec is
// a legal `N` / `<=N` / `N-M` with N,M >= 1. Values are stored as-provided.
inline bool ParseLengthSpec(const std::string& spec, int& mode, int& min_len, int& max_len) {
  auto s = TrimRaypathSegment(spec);
  if (s.empty()) {
    return false;
  }
  auto is_digits = [](const std::string& t) {
    if (t.empty()) {
      return false;
    }
    for (char c : t) {
      if (c < '0' || c > '9') {
        return false;
      }
    }
    return true;
  };
  // Range-safe digit->int: returns false on overflow (all-digit strings that
  // exceed `int` would otherwise make std::stoi throw std::out_of_range, which
  // no caller catches). std::from_chars reports overflow via errc instead.
  auto to_int = [](const std::string& t, int& out) {
    auto res = std::from_chars(t.data(), t.data() + t.size(), out);
    return res.ec == std::errc{} && res.ptr == t.data() + t.size();
  };
  if (s.size() >= 2 && s[0] == '<' && s[1] == '=') {
    auto n = TrimRaypathSegment(s.substr(2));
    int v = 0;
    if (!is_digits(n) || !to_int(n, v) || v < 1) {
      return false;
    }
    mode = 2;
    min_len = 1;
    max_len = v;
    return true;
  }
  auto dash = s.find('-');
  if (dash != std::string::npos) {
    auto lhs = TrimRaypathSegment(s.substr(0, dash));
    auto rhs = TrimRaypathSegment(s.substr(dash + 1));
    int a = 0;
    int b = 0;
    if (!is_digits(lhs) || !is_digits(rhs) || !to_int(lhs, a) || !to_int(rhs, b)) {
      return false;
    }
    if (a < 1 || b < 1 || a > b) {
      return false;
    }
    mode = 3;
    min_len = a;
    max_len = b;
    return true;
  }
  int v = 0;
  if (!is_digits(s) || !to_int(s, v) || v < 1) {
    return false;
  }
  mode = 1;
  min_len = v;
  max_len = v;
  return true;
}

// Merge an EE token (entry:/exit:/len:) into the in-flight builder. Returns
// false on grammar error (unknown prefix, unparseable lengthspec, or attempt
// to overwrite an already-set field). Whitespace inside facelist is preserved
// as-is; the facelist validator handles trimming semantics.
inline bool MergeEeToken(const std::string& tok, EntryExitParams& builder, bool& entry_set, bool& exit_set,
                         bool& len_set, std::string& err) {
  if (StartsWith(tok, "entry:")) {
    if (entry_set) {
      err = "duplicate entry: token in summand";
      return false;
    }
    builder.entry_text = TrimRaypathSegment(tok.substr(6));
    entry_set = true;
    return true;
  }
  if (StartsWith(tok, "exit:")) {
    if (exit_set) {
      err = "duplicate exit: token in summand";
      return false;
    }
    builder.exit_text = TrimRaypathSegment(tok.substr(5));
    exit_set = true;
    return true;
  }
  if (StartsWith(tok, "len:")) {
    if (len_set) {
      err = "duplicate len: token in summand";
      return false;
    }
    int mode = 0;
    int min_len = 1;
    int max_len = 1;
    if (!ParseLengthSpec(tok.substr(4), mode, min_len, max_len)) {
      err = "invalid length spec '" + tok.substr(4) + "'";
      return false;
    }
    builder.length_mode = mode;
    builder.min_len = min_len;
    builder.max_len = max_len;
    len_set = true;
    return true;
  }
  err = "unknown factor prefix '" + tok + "'";
  return false;
}

// True if a token looks like an EE token (starts with a known prefix).
inline bool IsEeToken(const std::string& tok) {
  return StartsWith(tok, "entry:") || StartsWith(tok, "exit:") || StartsWith(tok, "len:");
}

// Callback return value for WalkSummandEeFlush. Uniform semantics across all
// four callbacks — kAbort stops traversal, kContinue proceeds. No polarity
// inversion between callbacks (see plan.md §3 truth table).
enum class WalkAction { kContinue, kAbort };

// Shared token traversal for ValidateSummandText (strict) and ParseSummandText
// (tolerant). The skeleton owns the EE accumulation state (ee_builder /
// entry_set / exit_set / len_set / ee_started) and drives:
//   - empty-token dispatch  → on_empty_token()
//   - EE token merge         → MergeEeToken(); on success sets ee_started, on
//                              failure calls on_ee_merge_fail (skeleton itself
//                              does NOT touch EE state on failure, preserving
//                              first-wins accumulation across a mid-run skip)
//   - raypath token          → on_flush (only if ee_started), then reset EE
//                              state, then on_raypath_token
//   - end of loop            → one trailing on_flush (only if ee_started)
// The four callbacks decide policy at each decision point; the skeleton is
// oblivious to whether the caller is a validator or a parser. See plan.md §3
// truth table for validate/parse callback semantics.
// Return value: true if the traversal completed without any kAbort; false if
// some callback requested kAbort (caller's state — result / out — has been
// mutated by the callback before it returned kAbort).
template <typename OnEmptyToken, typename OnEeMergeFail, typename OnFlush, typename OnRaypathToken>
inline bool WalkSummandEeFlush(const std::vector<std::string>& tokens,
                               OnEmptyToken&& on_empty_token,      // () -> WalkAction
                               OnEeMergeFail&& on_ee_merge_fail,   // (const std::string& tok,
                                                                   //  const std::string& err) -> WalkAction
                               OnFlush&& on_flush,                 // (const EntryExitParams& ee,
                                                                   //  bool entry_set, bool exit_set) -> WalkAction
                               OnRaypathToken&& on_raypath_token)  // (const std::string& tok) -> WalkAction
{
  EntryExitParams ee_builder;
  bool entry_set = false;
  bool exit_set = false;
  bool len_set = false;
  bool ee_started = false;
  auto reset_ee = [&]() {
    ee_builder = EntryExitParams{};
    entry_set = false;
    exit_set = false;
    len_set = false;
    ee_started = false;
  };
  for (const auto& tok : tokens) {
    if (tok.empty()) {
      if (on_empty_token() == WalkAction::kAbort) {
        return false;
      }
      continue;
    }
    if (IsEeToken(tok)) {
      std::string err;
      if (MergeEeToken(tok, ee_builder, entry_set, exit_set, len_set, err)) {
        ee_started = true;
      } else if (on_ee_merge_fail(tok, err) == WalkAction::kAbort) {
        return false;
      }
      // On merge failure with kContinue: intentionally leave EE state untouched
      // so a subsequent valid EE token can accumulate into the same factor
      // (first-wins across a mid-run skip, per plan §3).
      continue;
    }
    // Raypath token: flush any in-flight EE factor (validate its facelists or
    // emit it), then reset EE state, then hand the raypath token off.
    if (ee_started) {
      if (on_flush(ee_builder, entry_set, exit_set) == WalkAction::kAbort) {
        return false;
      }
      reset_ee();
    }
    if (on_raypath_token(tok) == WalkAction::kAbort) {
      return false;
    }
  }
  if (ee_started) {
    if (on_flush(ee_builder, entry_set, exit_set) == WalkAction::kAbort) {
      return false;
    }
  }
  return true;
}

}  // namespace detail

// Strict validation of a summand row against the AND grammar. Empty input →
// kValid (means "no filter" at the row level). Uses LUMICE_ValidateRaypathText
// for raypath tokens and GuiValidateFaceNumberListText for entry:/exit:
// facelists so the messages match the surrounding single-value paths.
// Inherent grammar state-machine (token loop + EE flush/validate); NOLINTed per
// the project convention of not fragmenting dense validators.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
inline GuiValidationResult ValidateSummandText(const std::string& text, LUMICE_CrystalKind kind) {
  GuiValidationResult result;
  auto trimmed = TrimRaypathSegment(text);
  if (trimmed.empty()) {
    result.state = LUMICE_RAYPATH_VALID;
    return result;
  }

  // Reject leading / trailing / consecutive '&' (mirrors the ';' pre-flight in
  // ValidateRaypathTextMultiSegment).
  bool sep_pending = true;
  for (char c : trimmed) {
    if (c == '&') {
      if (sep_pending) {
        result.state = LUMICE_RAYPATH_INVALID;
        result.message = "Empty AND factor";
        return result;
      }
      sep_pending = true;
    } else if (!std::isspace(static_cast<unsigned char>(c))) {
      sep_pending = false;
    }
  }
  if (sep_pending) {
    result.state = LUMICE_RAYPATH_INVALID;
    result.message = "Empty AND factor";
    return result;
  }

  auto tokens = detail::SplitSummandTokens(trimmed);
  EntryExitParams ee_builder;
  bool entry_set = false;
  bool exit_set = false;
  bool len_set = false;
  bool ee_started = false;
  // Validate the in-flight EE facelists, then reset the EE state. Mirrors the
  // flush_ee reset in ParseSummandText so the validator AGREES with the parser:
  // a raypath token ends the current EE factor, and a following entry:/exit:
  // begins a fresh EE factor (not a "duplicate" of the prior one). Without this,
  // "entry:2 & 5 & entry:3" would be parsed+round-tripped fine but rejected here.
  auto flush_ee = [&]() -> bool {
    if (ee_started) {
      if (entry_set) {
        auto sr = GuiValidateFaceNumberListText(ee_builder.entry_text, kind);
        if (sr.state != LUMICE_RAYPATH_VALID) {
          result = sr;
          return false;
        }
      }
      if (exit_set) {
        auto sr = GuiValidateFaceNumberListText(ee_builder.exit_text, kind);
        if (sr.state != LUMICE_RAYPATH_VALID) {
          result = sr;
          return false;
        }
      }
      ee_builder = EntryExitParams{};
      entry_set = false;
      exit_set = false;
      len_set = false;
      ee_started = false;
    }
    return true;
  };
  for (const auto& tok : tokens) {
    if (tok.empty()) {
      result.state = LUMICE_RAYPATH_INVALID;
      result.message = "Empty AND factor";
      return result;
    }
    if (detail::IsEeToken(tok)) {
      ee_started = true;
      std::string err;
      if (!detail::MergeEeToken(tok, ee_builder, entry_set, exit_set, len_set, err)) {
        result.state = LUMICE_RAYPATH_INVALID;
        result.message = err;
        return result;
      }
    } else {
      // Raypath token ends the current EE factor (flush + validate), then
      // validate the raypath. A raypath token MAY contain ';' as a summand-level
      // OR alternative (H-A, 334.3): `1-3;3-5 & entry:2` distributes to
      // `(1-3 & entry:2) OR (3-5 & entry:2)` in ExpandSopToClauses. We delegate
      // to ValidateRaypathTextMultiSegment (already covered by 11 unit tests)
      // which rejects leading/trailing/consecutive ';' and validates each
      // segment via LUMICE_ValidateRaypathText — no new validation logic.
      if (!flush_ee()) {
        return result;
      }
      auto seg_result = ValidateRaypathTextMultiSegment(tok, kind);
      if (seg_result.state != LUMICE_RAYPATH_VALID) {
        result = seg_result;
        return result;
      }
    }
  }

  // Validate the trailing EE factor (if the row ended mid-EE).
  if (!flush_ee()) {
    return result;
  }

  result.state = LUMICE_RAYPATH_VALID;
  return result;
}

// Tolerant parse of a summand row into a Factor vector. Mirrors the
// ParseRaypathTextMultiSegment "skip invalid tokens" philosophy. Empty input
// → empty vector.
inline std::vector<Factor> ParseSummandText(const std::string& text) {
  std::vector<Factor> out;
  auto trimmed = TrimRaypathSegment(text);
  if (trimmed.empty()) {
    return out;
  }
  auto tokens = detail::SplitSummandTokens(trimmed);
  EntryExitParams ee_builder;
  bool ee_started = false;
  bool entry_set = false;
  bool exit_set = false;
  bool len_set = false;
  auto flush_ee = [&]() {
    if (ee_started) {
      out.emplace_back(ee_builder);
      ee_builder = EntryExitParams{};
      ee_started = false;
      entry_set = false;
      exit_set = false;
      len_set = false;
    }
  };
  for (const auto& tok : tokens) {
    if (tok.empty()) {
      continue;
    }
    if (detail::IsEeToken(tok)) {
      std::string err;
      // Tolerant parse: a token that fails to merge (duplicate / bad lengthspec)
      // is SKIPPED — it must NOT fabricate a factor. Only mark the EE factor
      // "started" once a field was actually set, so an all-invalid EE run (e.g.
      // "len:abc") yields no factor rather than a match-everything wildcard EE.
      // Mirrors ValidateSummandText, which rejects such tokens (code-review-02
      // Major 1).
      if (detail::MergeEeToken(tok, ee_builder, entry_set, exit_set, len_set, err)) {
        ee_started = true;
      }
    } else {
      flush_ee();
      RaypathParams rp;
      rp.raypath_text = tok;
      out.emplace_back(std::move(rp));
    }
  }
  flush_ee();
  return out;
}

// Returns how many OR-alternatives `factor` expands to (raypath ';' multi-segment, or EE
// comma-separated entry x exit product; everything else is exactly 1). A single-atom carrier
// like LUMICE_ColorPredicate requires exactly one alternative — this is declared here (the
// shared Factor toolkit) and DEFINED in file_io.cpp (where the EE/raypath decode helpers that
// back it already live), so the commit-time gate (FillColorPredicate) and the GUI-side
// pre-checks (color_window.cpp ValidateSingleAtomText / BuildClassFromFilter) share a single
// source of truth and can never drift apart again (code-review-01 Major: the front end accepted
// "1-3;5-7" as a valid single-atom predicate, then FillColorPredicate silently dropped it at
// the next commit because it resolves to 2 alternatives).
int CountFactorAlternatives(const Factor& factor);

// Format a single Factor back to canonical text.
inline std::string FormatFactor(const Factor& f) {
  return std::visit(
      [](const auto& v) -> std::string {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, RaypathParams>) {
          return v.raypath_text;
        } else if constexpr (std::is_same_v<T, EntryExitParams>) {
          return FormatEntryExitFactorText(v);
        }
      },
      f);
}

// Join Factors into a summand row using " & " as separator.
inline std::string FormatSummandText(const std::vector<Factor>& factors) {
  std::string out;
  bool first = true;
  for (const auto& f : factors) {
    if (!first) {
      out += " & ";
    }
    out += FormatFactor(f);
    first = false;
  }
  return out;
}

// Expand a summand row's raypath factors by their ';' OR-alternatives
// (H-A, 334.3) for the live editor preview, e.g.
//   "1-3;3-5 & entry:2"  ->  "(1-3 OR 3-5) & entry:2"
// Raypath factors with a single segment are shown unparenthesized.
// EE factors (entry:/exit:/len:) are shown verbatim via FormatFactor —
// their comma-list multi-value semantics are a separate, pre-existing
// expansion (handled at Cartesian-expand time by ExpandSopToClauses in
// file_io.cpp) and are intentionally NOT re-expanded here so this preview
// helper does not become a second source of truth for expansion semantics.
inline std::string FormatSummandExpansionPreview(const std::vector<Factor>& factors) {
  std::string out;
  bool first = true;
  for (const auto& f : factors) {
    if (!first) {
      out += " & ";
    }
    first = false;
    if (const auto* rp = std::get_if<RaypathParams>(&f)) {
      auto segs = SplitRaypathSegments(rp->raypath_text);
      if (segs.size() <= 1) {
        out += rp->raypath_text;
      } else {
        out += '(';
        for (size_t i = 0; i < segs.size(); ++i) {
          if (i != 0) {
            out += " OR ";
          }
          out += segs[i];
        }
        out += ')';
      }
    } else {
      out += FormatFactor(f);
    }
  }
  return out;
}

// Build the "OR of N row(s)" live preview for a whole SoP: one expanded row
// per line. Mirrors panels.cpp's committed-filter card tooltip format so both
// call sites share a single formatting source (DRY). Empty rows render as
// "*" (match-all sentinel, same as the card tooltip convention).
inline std::string FormatSopExpansionPreview(const SumOfProducts& sop) {
  std::string out = "OR of " + std::to_string(sop.size()) + " row(s):";
  for (const auto& s : sop) {
    out += "\n  ";
    if (s.factors.empty()) {
      out += s.text.empty() ? "*" : s.text;
    } else {
      out += FormatSummandExpansionPreview(s.factors);
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Legacy → SoP converters (task-gui-sop-data-model).
//
// These are the *canonical, grammar-conformant* conversion paths — as opposed
// to FilterConfig::SetRaypath / SetEntryExit which are the transparent compat
// writers that do not split ';' multi-segment OR into rows. Test utilities and
// (future) 333.3 serialization sites should use these instead of the compat
// writers when the intent is a true SoP fan-out.
// ---------------------------------------------------------------------------

// Split RaypathParams.raypath_text on ';' into one OR-row per segment. Empty
// input → single row with empty raypath (the "no filter" degenerate state, to
// match the FilterConfig default).
inline SumOfProducts FromLegacyRaypath(const RaypathParams& rp) {
  SumOfProducts out;
  if (rp.raypath_text.empty()) {
    RaypathParams empty_rp;
    out.push_back(SummandText{ std::string{}, std::vector<Factor>{ Factor{ empty_rp } } });
    return out;
  }
  auto segs = SplitRaypathSegments(rp.raypath_text);
  for (const auto& seg : segs) {
    RaypathParams seg_rp;
    seg_rp.raypath_text = seg;
    out.push_back(SummandText{ seg, std::vector<Factor>{ Factor{ std::move(seg_rp) } } });
  }
  return out;
}

// Wrap an EntryExitParams into a single-row SoP. The comma-list multi-value OR
// semantics inside entry_text / exit_text are preserved as-is (cartesian
// expansion into multiple core simple filters is 333.3's job, not this task).
inline SumOfProducts FromLegacyEntryExit(const EntryExitParams& ep) {
  SumOfProducts out;
  std::string text = FormatEntryExitFactorText(ep);
  out.push_back(SummandText{ std::move(text), std::vector<Factor>{ Factor{ ep } } });
  return out;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_RAYPATH_SEGMENTS_HPP
