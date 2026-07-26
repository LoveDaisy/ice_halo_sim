#include "gui/file_io.hpp"

#include <nfd.h>
#include <stb_image.h>
#include <stb_image_write.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <utility>
#include <vector>

#include "gui/app.hpp"
#include "gui/export_fbo_renderer.hpp"
#include "gui/gl_capture.hpp"
#include "gui/gl_common.h"
#include "gui/gui_logger.hpp"
#include "gui/gui_state.hpp"
#include "gui/preview_renderer.hpp"
#include "gui/raypath_segments.hpp"
#include "util/path_utils.hpp"

namespace lumice::gui {

using json = nlohmann::json;

// Serialize a discrete wl/weight spectrum to a JSON array of {wavelength, weight} objects.
static nlohmann::json WlWeightArrayToJson(const std::vector<WlWeight>& spec) {
  nlohmann::json arr = nlohmann::json::array();
  for (const auto& e : spec) {
    arr.push_back({ { "wavelength", e.wavelength }, { "weight", e.weight } });
  }
  return arr;
}

// Parses a JSON array of {wavelength, weight} objects into WlWeight entries.
// Skips malformed elements. Returns the parsed vector (caller sets spectrum_index).
static std::vector<WlWeight> ParseWlWeightArray(const nlohmann::json& arr) {
  std::vector<WlWeight> out;
  for (const auto& e : arr) {
    if (e.is_object() && e.contains("wavelength") && e.contains("weight")) {
      WlWeight w;
      w.wavelength = e.at("wavelength").get<float>();
      w.weight = e.at("weight").get<float>();
      out.push_back(w);
    }
  }
  return out;
}

// Convert Miller index (i1, i4) to wedge angle in degrees. Returns default (28.0) if i1 == 0.
static float MillerToAlpha(int i1, int i4) {
  constexpr float kSqrt3_2 = 0.866025403784f;
  constexpr float kIceCrystalC = 1.629f;
  constexpr float kRadToDeg = 57.2957795131f;
  if (i1 == 0) {
    return 28.0f;
  }
  return std::atan(kSqrt3_2 * i4 / i1 / kIceCrystalC) * kRadToDeg;
}

// Lens type JSON names (shared by Core config and GuiState JSON)
static const char* kLensTypeJsonNames[] = { "linear",
                                            "fisheye_equal_area",
                                            "fisheye_equidistant",
                                            "fisheye_stereographic",
                                            "dual_fisheye_equal_area",
                                            "dual_fisheye_equidistant",
                                            "dual_fisheye_stereographic",
                                            "rectangular" };

static const char* kVisibleJsonNames[] = { "upper", "lower", "full" };
static_assert(sizeof(kVisibleJsonNames) / sizeof(kVisibleJsonNames[0]) == kVisibleCount,
              "kVisibleJsonNames must match kVisibleCount");
static const char* kAspectPresetJsonNames[] = { "free", "16:9", "3:2", "4:3", "1:1", "2:1", "match_background" };
static_assert(sizeof(kAspectPresetJsonNames) / sizeof(kAspectPresetJsonNames[0]) == kAspectPresetCount,
              "kAspectPresetJsonNames must match kAspectPresetCount");


// ========== Shared helpers ==========

std::vector<int> ParseRaypathText(const std::string& text) {
  std::vector<int> result;
  // Normalize: replace ',' with '-' so both separators are accepted
  std::string normalized = text;
  for (auto& c : normalized) {
    if (c == ',')
      c = '-';
  }
  std::istringstream iss(normalized);
  std::string token;
  while (std::getline(iss, token, '-')) {
    if (token.empty())
      continue;
    try {
      int val = std::stoi(token);
      if (val < 0)
        continue;
      result.push_back(val);
    } catch (...) {
    }
  }
  return result;
}

static const char* AxisDistTypeToString(AxisDistType t) {
  switch (t) {
    case AxisDistType::kGauss:
      return "gauss";
    case AxisDistType::kUniform:
      return "uniform";
    case AxisDistType::kZigzag:
      return "zigzag";
    case AxisDistType::kLaplacian:
      return "laplacian";
    case AxisDistType::kGaussLegacy:
      return "gauss_legacy";
    default:
      GUI_LOG_ERROR("[FileIO] Unknown AxisDistType: {}", static_cast<int>(t));
      return "gauss";
  }
}

static json SerializeAxisDist(const AxisDist& a) {
  json j;
  j["type"] = AxisDistTypeToString(a.type);
  j["mean"] = a.mean;
  j["std"] = a.std;
  return j;
}

static const char* ShapeDistTypeToString(ShapeDistType t) {
  switch (t) {
    case ShapeDistType::kNoRandom:
      return "no_random";
    case ShapeDistType::kUniform:
      return "uniform";
    case ShapeDistType::kGauss:
      return "gauss";
    case ShapeDistType::kZigzag:
      return "zigzag";
    case ShapeDistType::kLaplacian:
      return "laplacian";
    case ShapeDistType::kGaussLegacy:
      return "gauss_legacy";
    default:
      GUI_LOG_ERROR("[FileIO] Unknown ShapeDistType: {}", static_cast<int>(t));
      return "no_random";
  }
}

// Serialize a crystal shape distribution. A NO_RANDOM value collapses to a bare number (its
// center) so default (non-random) crystals produce byte-identical JSON to the pre-upgrade format
// (`"height": 1.0`) — no noise added to the common case. A randomized field writes the full
// {type, mean, std} object (keys mirror SerializeAxisDist and the core on-disk format).
static json SerializeShapeDist(const ShapeDist& d) {
  if (d.type == ShapeDistType::kNoRandom) {
    return json(d.center);
  }
  json j;
  j["type"] = ShapeDistTypeToString(d.type);
  j["mean"] = d.center;
  j["std"] = d.spread;
  return j;
}

// SYNC: these two key tables must stay byte-identical to kPrismSyncGroupKeys /
// kPyramidSyncGroupKeys in src/server/c_api.cpp and src/config/crystal_config.cpp. This is the
// third copy: the GUI may not include core/config headers (public-API boundary), and c_api.cpp's
// copy is TU-private, so the table is duplicated rather than shared. There is no compile-time
// mechanism tying the three together — a single character of drift here means a .lmc the GUI
// writes declares a group under a key core never reads, and the declaration silently degenerates
// to "all independent" with nothing to warn. These SYNC comments are the only defense; changing a
// key name means editing all three.
// "face_distance" is common to both types and handled separately (a 6-element array, mirroring how
// face_distance itself serializes).
static constexpr std::pair<const char*, int> kPrismSyncGroupKeys[] = {
  { "height", LUMICE_SHAPE_SCALAR_HEIGHT },
};
static constexpr std::pair<const char*, int> kPyramidSyncGroupKeys[] = {
  { "prism_h", LUMICE_SHAPE_SCALAR_PRISM_H },
  { "upper_h", LUMICE_SHAPE_SCALAR_UPPER_H },
  { "lower_h", LUMICE_SHAPE_SCALAR_LOWER_H },
};

// Write shape.sync_group. Emits nothing when nothing is synced, so every .lmc written before
// v4.13 (and every all-independent crystal today) keeps its byte-identical wire form — the same
// "non-default only" convention face_distance already follows here.
// Mirrors WriteSyncGroupJson in src/server/c_api.cpp / src/config/crystal_config.cpp.
static void WriteSyncGroupJson(json& shape_j, const int sync_group[LUMICE_SHAPE_SCALAR_COUNT],
                               const std::pair<const char*, int>* scalar_keys, size_t scalar_key_cnt) {
  json sg = json::object();
  for (size_t k = 0; k < scalar_key_cnt; k++) {
    if (sync_group[scalar_keys[k].second] != 0) {
      sg[scalar_keys[k].first] = sync_group[scalar_keys[k].second];
    }
  }
  bool any_face = false;
  for (int i = 0; i < 6; i++) {
    any_face = any_face || sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] != 0;
  }
  if (any_face) {
    // All six, zeros included — matching how face_distance itself serializes.
    sg["face_distance"] =
        std::vector<int>(sync_group + LUMICE_SHAPE_SCALAR_FACE_0, sync_group + LUMICE_SHAPE_SCALAR_FACE_0 + 6);
  }
  if (!sg.empty()) {
    shape_j["sync_group"] = sg;
  }
}

// Read shape.sync_group. An absent key leaves every slot 0 (all independent), which is why an
// older .lmc needs no migration. Mirrors ReadSyncGroupJson in the two files named above.
static void ReadSyncGroupJson(const json& shape_j, int sync_group[LUMICE_SHAPE_SCALAR_COUNT],
                              const std::pair<const char*, int>* scalar_keys, size_t scalar_key_cnt) {
  for (int i = 0; i < LUMICE_SHAPE_SCALAR_COUNT; i++) {
    sync_group[i] = 0;
  }
  if (!shape_j.contains("sync_group") || !shape_j.at("sync_group").is_object()) {
    return;
  }
  const auto& sg = shape_j.at("sync_group");
  for (size_t k = 0; k < scalar_key_cnt; k++) {
    if (sg.contains(scalar_keys[k].first) && sg.at(scalar_keys[k].first).is_number_integer()) {
      sync_group[scalar_keys[k].second] = sg.at(scalar_keys[k].first).get<int>();
    }
  }
  if (sg.contains("face_distance") && sg.at("face_distance").is_array()) {
    size_t i = 0;
    for (const auto& elem : sg.at("face_distance")) {
      if (i >= 6) {
        break;
      }
      if (elem.is_number_integer()) {
        sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] = elem.get<int>();
      }
      i++;
    }
  }
}

// Field-sync guard for SerializeCrystal.
// If CrystalConfig gains/loses a field, sizeof changes and this fires. Developer must then
// audit BOTH SerializeCrystal (below) and FillCrystalParam (further down this file) for
// same-file sync. Platform-gated because std::string size varies across C++ stdlib impls;
// this mirror is a "Apple Silicon + libc++ reminder", not a cross-platform contract.
//
// Audited fields (2026-07 snapshot, shape scalars promoted to ShapeDist):
//   name (serialize: yes, c-api: no), type (yes/yes),
//   height, prism_h, upper_h, lower_h (now ShapeDist; serialize + c-api yes),
//   upper_alpha, lower_alpha (bare float, pyramid-only/yes),
//   face_distance[6] (now ShapeDist[6]; conditional serialize / c-api yes),
//   ShapeDist::sync_group (v4.13; conditional serialize via WriteSyncGroupJson / c-api yes
//     via FillSyncGroupArray — one int in each of the ten ShapeDist members),
//   zenith, azimuth, roll (AxisDist, yes/yes).
// Size grew 112 → 192 when the five shape scalars became ShapeDist (12 B each: +80 B total),
// then 192 → 232 when ShapeDist gained sync_group (+4 B × 10 ShapeDist members).
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(CrystalConfig) == 232,
              "CrystalConfig size changed; audit SerializeCrystal and FillCrystalParam for new/renamed fields");
#endif
static json SerializeCrystal(const CrystalConfig& c, int id) {
  json j;
  j["id"] = id;
  if (!c.name.empty()) {
    j["name"] = c.name;
  }

  if (c.type == CrystalType::kPrism) {
    j["type"] = "prism";
    j["shape"]["height"] = SerializeShapeDist(c.height);
  } else {
    j["type"] = "pyramid";
    j["shape"]["prism_h"] = SerializeShapeDist(c.prism_h);
    j["shape"]["upper_h"] = SerializeShapeDist(c.upper_h);
    j["shape"]["lower_h"] = SerializeShapeDist(c.lower_h);
    j["shape"]["upper_wedge_angle"] = c.upper_alpha;
    j["shape"]["lower_wedge_angle"] = c.lower_alpha;
  }

  // face_distance: only write when non-default. The default is the deterministic unit distance
  // ShapeDist{kNoRandom, 1.0, 0.0}; compare all three components via ShapeDist::operator== so a
  // face that is randomized but happens to have center 1.0 is NOT mistaken for the default.
  const ShapeDist default_fd = 1.0f;  // {kNoRandom, 1.0, 0.0}
  bool is_default_fd = true;
  for (int i = 0; i < 6; i++) {
    if (c.face_distance[i] != default_fd) {
      is_default_fd = false;
      break;
    }
  }
  if (!is_default_fd) {
    j["shape"]["face_distance"] = { SerializeShapeDist(c.face_distance[0]), SerializeShapeDist(c.face_distance[1]),
                                    SerializeShapeDist(c.face_distance[2]), SerializeShapeDist(c.face_distance[3]),
                                    SerializeShapeDist(c.face_distance[4]), SerializeShapeDist(c.face_distance[5]) };
  }

  // shape.sync_group: the embedded per-ShapeDist groups gathered back into the wire form's
  // parallel array. Only the keys applicable to this crystal type are consulted, matching how the
  // shape scalars themselves are written above.
  int sg[LUMICE_SHAPE_SCALAR_COUNT];
  FillSyncGroupArray(c, sg);
  if (c.type == CrystalType::kPrism) {
    WriteSyncGroupJson(j["shape"], sg, kPrismSyncGroupKeys, std::size(kPrismSyncGroupKeys));
  } else {
    WriteSyncGroupJson(j["shape"], sg, kPyramidSyncGroupKeys, std::size(kPyramidSyncGroupKeys));
  }

  j["axis"]["zenith"] = SerializeAxisDist(c.zenith);
  j["axis"]["azimuth"] = SerializeAxisDist(c.azimuth);
  j["axis"]["roll"] = SerializeAxisDist(c.roll);

  return j;
}

// Compose the action string used by both GUI and core JSON.
static const char* FilterActionToString(int action) {
  return action == 0 ? "filter_in" : "filter_out";
}

// GUI .lmc payload: per-filter JSON object.
//
// Schema v=3 (task-serialization-bidirectional): a filter carries its full
// sum-of-products as an array of canonical summand text rows under "summands"
// (type discriminator "sop"). Each row string is the canonical AND grammar
// (see raypath_segments.hpp). We store the SummandText.text verbatim — that is
// the operator== identity (text-only), so this is lossless with respect to the
// GUI's equality contract. v=1/v=2 forms (type "raypath"/"entry_exit") are still
// READ by ParseFilterFromGuiJson for backward compatibility.
static json SerializeFilterForGui(const FilterConfig& f, int id) {
  json j;
  j["id"] = id;
  if (!f.name.empty()) {
    j["name"] = f.name;
  }
  j["action"] = FilterActionToString(f.action);
  j["sym_p"] = f.sym_p;
  j["sym_b"] = f.sym_b;
  j["sym_d"] = f.sym_d;

  j["type"] = "sop";
  json summands = json::array();
  for (const auto& s : f.param) {
    summands.push_back(s.text);
  }
  j["summands"] = std::move(summands);

  return j;
}

// Parse an EntryExit face-number text buffer to int. Empty / non-numeric
// text returns 0 (the OK-button gate stops invalid commits before reaching
// here; this fallback only ever runs for bypass paths like Immediate-mode
// half-typed input). Delegates to ValidateFaceNumberText for the syntax
// rules so the digit-cap and separator policy stay in lockstep with
// raypath_validation.cpp::kMaxFaceDigits — kind-specific legality
// (Prism vs Pyramid) is intentionally bypassed here because the GUI
// layer doesn't carry the kind context across to the serializer; the
// OK gate is the kind-aware authority.
static int ParseFaceNumberOrZero(const std::string& text) {
  if (text.empty()) {
    return 0;
  }
  // ValidateFaceNumberText with kPyramid covers the union of legal
  // faces — any text the syntax stage rejects (separators, non-digits,
  // overlong) bails to 0; only the kind-specific stage might "reject"
  // a legal-on-pyramid face we want to keep, which we tolerate here.
  const auto v = GuiValidateFaceNumberText(text, LUMICE_CRYSTAL_PYRAMID);
  if (v.state == LUMICE_RAYPATH_INVALID && v.message.find("not legal on this crystal type") == std::string::npos) {
    return 0;
  }
  // Safe to parse: ValidateFaceNumberText caps digit count.
  int value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') {
      return 0;
    }
    value = value * 10 + (c - '0');
  }
  return value;
}

// Sentinel returned by ParseFaceNumberList for the wildcard (empty input).
static constexpr int kEEWildcardSentinel = -1;

// Local mirror of lumice::kMaxHits (src/core/def.hpp). Hardcoded here to keep
// the AGENTS.md "src/gui must not include core/ headers directly" boundary
// intact. Keep in sync if core/def.hpp::kMaxHits changes.
static constexpr int kEELenAbsoluteMax = 64;

// Parse a comma-separated face-number list (EE multi-value OR input). Empty
// text returns a single-element list with the wildcard sentinel, so callers
// can uniformly take the cartesian product. Tokens are individually parsed
// via ParseFaceNumberOrZero; malformed tokens fall through to 0 there (the
// OK-button gate stops invalid commits before this is reached in normal
// use). Trailing comma is ignored — kIncomplete inputs should already have
// been rejected upstream.
static std::vector<int> ParseFaceNumberList(const std::string& text) {
  std::vector<int> out;
  if (text.empty()) {
    out.push_back(kEEWildcardSentinel);
    return out;
  }
  size_t pos = 0;
  while (pos < text.size()) {
    size_t end = text.find(',', pos);
    if (end == std::string::npos) {
      end = text.size();
    }
    if (end != pos) {
      out.push_back(ParseFaceNumberOrZero(text.substr(pos, end - pos)));
    }
    pos = end + 1;
  }
  if (out.empty()) {
    out.push_back(kEEWildcardSentinel);
  }
  return out;
}

// Decode the GUI length-mode trio into the core (min_len, max_len) pair.
// Mirrors the EntryExitParams docstring; see plan §3.3.
struct EELenBounds {
  int min_len;
  // -1 here means "no upper bound" (corresponds to std::optional<size_t> nullopt
  // on the core EntryExitFilterParam::max_len_). Distinct from
  // kEEWildcardSentinel which is the face-wildcard marker — same magic value,
  // different namespace: this one is never confused with a face id because
  // EELenBounds::max_len is only ever fed to a LUMICE_FilterParam.ee_max_len / parsed back
  // from min/max length JSON fields, not into the entry/exit face channels.
  int max_len;
};
static EELenBounds DecodeLengthMode(int mode, int min_len, int max_len) {
  switch (mode) {
    case 1:  // strict N
      return { std::max(min_len, 1), std::max(min_len, 1) };
    case 2:  // at most N
      return { 1, std::max(max_len, 1) };
    case 3:  // range [N,M]
      return { std::max(min_len, 1), std::max(max_len, std::max(min_len, 1)) };
    case 0:
    default:
      return { 1, -1 };
  }
}

// Clamp a GUI int to the C API ID range and warn on overflow.
static int ClampIdValue(int v, const char* field_name) {
  if (v < 0) {
    GUI_LOG_WARNING("[Filter] {} = {} clamped to 0 (LUMICE_MAX_ID range [0, {}])", field_name, v, LUMICE_MAX_ID);
    return 0;
  }
  constexpr int max_id = LUMICE_MAX_ID;
  if (v > max_id) {
    GUI_LOG_WARNING("[Filter] {} = {} clamped to {} (LUMICE_MAX_ID range [0, {}])", field_name, v, max_id, max_id);
    return max_id;
  }
  return v;
}

// ========== Unified sum-of-products expansion (single SYNC source) ==========
//
// Backend-agnostic intermediate representation of a GUI FilterConfig fully
// expanded to disjunctive normal form: an OR of clauses, each clause an AND of
// simple-filter term descriptors. ExpandFilterToScene is now its ONLY consumer —
// the historical second consumer (a hand-written JSON emit twin) is gone: JSON is
// produced by serializing the very LUMICE_Scene the commit path builds, so the two
// cannot drift by construction. (The prior arrangement — two hand-written expansion loops,
// one per output format — produced three separate correctness bugs before being unified.)
struct SimpleTermDesc {
  bool is_raypath = true;
  std::vector<int> raypath;         // valid when is_raypath (empty == match-all)
  int entry = kEEWildcardSentinel;  // valid when !is_raypath
  int exit = kEEWildcardSentinel;   // valid when !is_raypath
  int min_len = 1;                  // decoded EE length bounds (see EELenBounds)
  int max_len = -1;                 // -1 == unbounded
};
struct ExpandedFilter {
  std::vector<std::vector<SimpleTermDesc>> clauses;  // OR of (AND of terms)
  bool overflow = false;                             // set when expansion would exceed ABI clause/term caps
};

// Step A — one Factor → its OR-of-alternatives simple-term list (length >= 1).
// Reuses the existing type-internal multi-value atoms (ParseRaypathTextMultiSegment
// for ';' OR; ParseFaceNumberList + DecodeLengthMode for EE comma OR).
static std::vector<SimpleTermDesc> FactorAlternatives(const Factor& factor) {
  std::vector<SimpleTermDesc> alts;
  std::visit(
      [&](const auto& p) {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, RaypathParams>) {
          auto segs = ParseRaypathTextMultiSegment(p.raypath_text);
          if (segs.empty()) {
            // Empty text == match-all raypath: a single alternative, empty seq.
            alts.emplace_back();
          } else {
            for (auto& seg : segs) {
              SimpleTermDesc d;
              d.is_raypath = true;
              d.raypath = std::move(seg);
              alts.push_back(std::move(d));
            }
          }
        } else if constexpr (std::is_same_v<T, EntryExitParams>) {
          auto entries = ParseFaceNumberList(p.entry_text);
          auto exits = ParseFaceNumberList(p.exit_text);
          const auto bounds = DecodeLengthMode(p.length_mode, p.min_len, p.max_len);
          for (int e : entries) {
            for (int x : exits) {
              SimpleTermDesc d;
              d.is_raypath = false;
              d.entry = e;
              d.exit = x;
              d.min_len = bounds.min_len;
              d.max_len = bounds.max_len;
              alts.push_back(d);
            }
          }
        }
      },
      factor);
  return alts;
}

// Declared in raypath_segments.hpp — shared single-alternative gate consumed by
// color_window.cpp so its front-end pre-checks can never drift from FillColorPredicate below.
int CountFactorAlternatives(const Factor& factor) {
  return static_cast<int>(FactorAlternatives(factor).size());
}

// Steps A–C — GUI FilterConfig → OR of AND-clauses. A summand (AND-of-factors)
// distributes over each factor's internal OR via a Cartesian product (DNF):
// AND(EE{3,4}, RP) = OR(AND(EE3,RP), AND(EE4,RP)). Concatenating every summand's
// clauses gives the full clause set. THIS is the single expansion source.
static ExpandedFilter ExpandSopToClauses(const FilterConfig& f) {
  ExpandedFilter ef;
  const size_t kMaxClauses = static_cast<size_t>(LUMICE_MAX_CONFIG_CLAUSES);
  const size_t kMaxTerms = static_cast<size_t>(LUMICE_MAX_CONFIG_TERMS);
  for (const auto& summand : f.param) {
    if (summand.factors.empty()) {
      // A factor-less row (e.g. blank summand text) == match-all: one clause
      // with a single match-all raypath term (mirrors the degenerate default).
      if (ef.clauses.size() + 1 > kMaxClauses) {
        ef.overflow = true;
        break;
      }
      ef.clauses.push_back(std::vector<SimpleTermDesc>{ SimpleTermDesc{} });
      continue;
    }
    // Multi-factor AND rows produce multi-term clauses; cap term count per clause.
    if (summand.factors.size() > kMaxTerms) {
      ef.overflow = true;
      break;
    }
    // Cartesian product across the summand's factors. Per-factor alternative
    // counts are individually bounded (facelist / raypath-segment sizes), but the
    // cross-factor product (the AND-chain) is exponential — so cap the RUNNING
    // product BEFORE materializing each level, and cap the total across summands,
    // against the ABI clause limit. A crafted .lmc summands row (not input-box
    // limited) can otherwise force unbounded materialization (code-review-01
    // Major 2 / plan R4). On overflow, both emit twins degrade gracefully.
    std::vector<std::vector<SimpleTermDesc>> acc{ {} };  // seed with one empty clause
    for (const auto& factor : summand.factors) {
      auto alts = FactorAlternatives(factor);
      const size_t alt_n = alts.empty() ? 1 : alts.size();
      if (ef.clauses.size() + acc.size() * alt_n > kMaxClauses) {
        ef.overflow = true;
        break;
      }
      std::vector<std::vector<SimpleTermDesc>> next;
      next.reserve(acc.size() * alt_n);
      for (const auto& clause : acc) {
        for (const auto& alt : alts) {
          auto extended = clause;
          extended.push_back(alt);
          next.push_back(std::move(extended));
        }
      }
      acc = std::move(next);
    }
    if (ef.overflow) {
      break;
    }
    for (auto& clause : acc) {
      ef.clauses.push_back(std::move(clause));
    }
  }
  if (ef.overflow) {
    // Bounded degenerate stand-in so downstream never sees a huge tree; both
    // emit twins detect ef.overflow and handle it (struct drops, JSON warns).
    ef.clauses.clear();
    ef.clauses.push_back(std::vector<SimpleTermDesc>{ SimpleTermDesc{} });
    return ef;
  }
  // Defensive: a FilterConfig with an empty param vector is not a valid state
  // (the default is a 1-row SoP), but never emit zero clauses.
  if (ef.clauses.empty()) {
    ef.clauses.push_back(std::vector<SimpleTermDesc>{ SimpleTermDesc{} });
  }
  return ef;
}

// task-gui-feedback-affordances Step 3 (AC4) — see file_io.hpp. Cheap on-typing
// summary that delegates to ExpandSopToClauses so the live preview and the
// commit path share the same expansion arithmetic (no drift). Only the total
// clause count + overflow flag are exposed; the callers do not need to see the
// clause contents.
SopExpansionSummary SummarizeSopExpansion(const FilterConfig& f) {
  const ExpandedFilter ef = ExpandSopToClauses(f);
  SopExpansionSummary summary;
  summary.clause_count = ef.clauses.size();
  summary.overflow = ef.overflow;
  return summary;
}

// True when the expansion collapses to a single simple filter (1 clause, 1 term):
// single raypath / single EE / single-factor single-alternative row. Such filters
// emit ONE simple filter with no wrapping complex (byte-equivalent to pre-uplift).
//
// ⚠️ NOT the same as gui_state.hpp's IsDegenerateSingleFactor (code-review-05 Minor 1):
// this operates on the POST-Cartesian ExpandedFilter (clause/term level); that operates
// on the FilterConfig SoP (row/factor level). They are NOT equivalent — e.g. a single-row
// single-factor raypath with ';' multi-segments is IsDegenerateSingleFactor()==true but
// expands to multiple clauses (IsDegenerateSingleTerm()==false). Do not assume they sync.
static bool IsDegenerateSingleTerm(const ExpandedFilter& ef) {
  return ef.clauses.size() == 1 && ef.clauses[0].size() == 1;
}

static AxisDistType ParseAxisDistType(const std::string& t) {
  if (t == "gauss")
    return AxisDistType::kGauss;
  if (t == "uniform")
    return AxisDistType::kUniform;
  if (t == "zigzag")
    return AxisDistType::kZigzag;
  if (t == "laplacian")
    return AxisDistType::kLaplacian;
  if (t == "gauss_legacy")
    return AxisDistType::kGaussLegacy;
  GUI_LOG_ERROR("[FileIO] Unknown axis dist type '{}', falling back to gauss", t);
  return AxisDistType::kGauss;
}

static ShapeDistType ParseShapeDistType(const std::string& t) {
  if (t == "no_random")
    return ShapeDistType::kNoRandom;
  if (t == "uniform")
    return ShapeDistType::kUniform;
  if (t == "gauss")
    return ShapeDistType::kGauss;
  if (t == "zigzag")
    return ShapeDistType::kZigzag;
  if (t == "laplacian")
    return ShapeDistType::kLaplacian;
  if (t == "gauss_legacy")
    return ShapeDistType::kGaussLegacy;
  GUI_LOG_ERROR("[FileIO] Unknown shape dist type '{}', falling back to uniform", t);
  return ShapeDistType::kUniform;
}

// Count of shape distributions downgraded to uniform during the current parse (see ParseShapeDist).
// Consumed + reset by TakeShapeDistDowngradeCount() so the GUI open handler can surface a one-time
// "some distributions were simplified" notice. GUI file loading is single-threaded, so a TU-local
// counter is safe and avoids threading an accumulator through every ParseCrystal call site.
static int g_shape_dist_downgrade_count = 0;

int TakeShapeDistDowngradeCount() {
  int n = g_shape_dist_downgrade_count;
  g_shape_dist_downgrade_count = 0;
  return n;
}

// Parse a crystal shape distribution. A bare number → {NO_RANDOM, v, 0}; an object → {type, mean,
// std}. `default_center` supplies the center for an absent/typeless value, mirroring each field's
// historical parse fallback (height/prism_h = 1.0; upper_h/lower_h = 0.0).
//
// The GUI edits UNIFORM distributions only, so any other randomized family (gauss/zigzag/laplacian/
// gauss_legacy) is downgraded to uniform here, keeping the spread value as-is. This is a deliberate,
// user-notified capability downgrade (JSON/CLI carry the full distribution set; the GUI is the
// simplified editor), surfaced via a load-complete notice — NOT a silent loss (contrast the earlier
// bug where loading collapsed a distribution to its bare mean and dropped type/std with no notice).
// NO_RANDOM (off) and UNIFORM pass through unchanged. Downgrades bump g_shape_dist_downgrade_count.
static ShapeDist ParseShapeDist(const json& j, float default_center) {
  ShapeDist d;
  d.center = default_center;
  if (j.is_number()) {
    d.type = ShapeDistType::kNoRandom;
    d.center = j.get<float>();
    d.spread = 0.0f;
  } else if (j.is_object()) {
    d.type = ParseShapeDistType(j.value("type", "no_random"));
    d.center = j.value("mean", default_center);
    d.spread = j.value("std", 0.0f);
  }
  if (d.type != ShapeDistType::kNoRandom && d.type != ShapeDistType::kUniform) {
    d.type = ShapeDistType::kUniform;
    ++g_shape_dist_downgrade_count;
  }
  return d;
}

static AxisDist ParseAxisDist(const json& j) {
  AxisDist a;
  if (j.is_number()) {
    a.type = AxisDistType::kGauss;
    a.mean = j.get<float>();
    a.std = 0.0f;
  } else if (j.is_object()) {
    auto t = j.value("type", "gauss");
    a.type = ParseAxisDistType(t);
    a.mean = j.value("mean", 0.0f);
    a.std = j.value("std", 0.0f);
  }
  return a;
}

static CrystalConfig ParseCrystal(const json& j) {
  CrystalConfig c;
  c.name = j.value("name", std::string{});

  auto type_str = j.value("type", "prism");
  c.type = (type_str == "pyramid") ? CrystalType::kPyramid : CrystalType::kPrism;

  if (j.contains("shape")) {
    auto& s = j.at("shape");
    if (c.type == CrystalType::kPrism) {
      if (s.contains("height")) {
        c.height = ParseShapeDist(s["height"], 1.0f);
      }
    } else {
      // Historical fallbacks preserved: prism_h defaults to 1.0, upper_h/lower_h to 0.0 when absent.
      c.prism_h = s.contains("prism_h") ? ParseShapeDist(s["prism_h"], 1.0f) : ShapeDist(1.0f);
      c.upper_h = s.contains("upper_h") ? ParseShapeDist(s["upper_h"], 0.0f) : ShapeDist(0.0f);
      c.lower_h = s.contains("lower_h") ? ParseShapeDist(s["lower_h"], 0.0f) : ShapeDist(0.0f);
      // Wedge angle: prefer "upper_wedge_angle", fallback to "upper_indices" conversion
      if (s.contains("upper_wedge_angle") && s["upper_wedge_angle"].is_number()) {
        c.upper_alpha = s["upper_wedge_angle"].get<float>();
      } else if (s.contains("upper_indices") && s["upper_indices"].is_array() && s["upper_indices"].size() == 3) {
        c.upper_alpha = MillerToAlpha(s["upper_indices"][0].get<int>(), s["upper_indices"][2].get<int>());
      }
      if (s.contains("lower_wedge_angle") && s["lower_wedge_angle"].is_number()) {
        c.lower_alpha = s["lower_wedge_angle"].get<float>();
      } else if (s.contains("lower_indices") && s["lower_indices"].is_array() && s["lower_indices"].size() == 3) {
        c.lower_alpha = MillerToAlpha(s["lower_indices"][0].get<int>(), s["lower_indices"][2].get<int>());
      }
    }
    // face_distance: common to both Prism and Pyramid
    if (s.contains("face_distance") && s["face_distance"].is_array()) {
      size_t n = std::min(s["face_distance"].size(), static_cast<size_t>(6));
      for (size_t i = 0; i < n; i++) {
        c.face_distance[i] = ParseShapeDist(s["face_distance"][i], 1.0f);
      }
    }
    // shape.sync_group: read into the wire form's parallel array, then scatter back into each
    // ShapeDist. Must come AFTER the shape parses above — those assign whole ShapeDist values and
    // would overwrite an already-scattered group with the default 0.
    int sg[LUMICE_SHAPE_SCALAR_COUNT];
    if (c.type == CrystalType::kPrism) {
      ReadSyncGroupJson(s, sg, kPrismSyncGroupKeys, std::size(kPrismSyncGroupKeys));
    } else {
      ReadSyncGroupJson(s, sg, kPyramidSyncGroupKeys, std::size(kPyramidSyncGroupKeys));
    }
    ApplySyncGroupArray(sg, c);
  }

  if (j.contains("axis")) {
    auto& ax = j.at("axis");
    if (ax.contains("zenith"))
      c.zenith = ParseAxisDist(ax["zenith"]);
    if (ax.contains("azimuth"))
      c.azimuth = ParseAxisDist(ax["azimuth"]);
    if (ax.contains("roll"))
      c.roll = ParseAxisDist(ax["roll"]);
  }

  return c;
}

static int LensTypeFromString(const std::string& s) {
  for (int i = 0; i < kLensTypeCount; i++) {
    if (s == kLensTypeJsonNames[i])
      return i;
  }
  return 0;  // default: linear
}

static int VisibleFromString(const std::string& s) {
  for (int i = 0; i < kVisibleCount; i++) {
    if (s == kVisibleJsonNames[i])
      return i;
  }
  return 2;  // default: full
}

static int SpectrumFromString(const std::string& s) {
  for (int i = 0; i < kSpectrumCount; i++) {
    if (s == kSpectrumNames[i])
      return i;
  }
  return 2;  // default: D65
}

static AspectPreset AspectPresetFromString(const std::string& s) {
  for (int i = 0; i < kAspectPresetCount; i++) {
    if (s == kAspectPresetJsonNames[i]) {
      return static_cast<AspectPreset>(i);
    }
  }
  return AspectPreset::kFree;  // default: free
}

static int SimResolutionIndexFromValue(int value) {
  for (int i = 0; i < kSimResolutionCount; i++) {
    if (kSimResolutions[i] == value)
      return i;
  }
  return 1;  // default: 1024
}

// ========== GUI JSON Renderer Helpers ==========
// Shared between SerializeGuiStateJson and DeserializeGuiStateJson.

static json SerializeRendererForGui(const RenderConfig& r) {
  json jr;
  jr["lens_type"] = kLensTypeJsonNames[r.lens_type];
  jr["fov"] = r.fov;
  jr["elevation"] = r.elevation;
  jr["azimuth"] = r.azimuth;
  jr["roll"] = r.roll;
  jr["sim_resolution"] = kSimResolutions[r.sim_resolution_index];
  jr["visible"] = kVisibleJsonNames[r.visible];
  jr["front"] = r.front;
  jr["background"] = { r.background[0], r.background[1], r.background[2] };
  jr["ray_color"] = { r.ray_color[0], r.ray_color[1], r.ray_color[2] };
  jr["opacity"] = r.opacity;
  jr["exposure_offset"] = r.exposure_offset;
  return jr;
}

static RenderConfig ParseRendererFromGuiJson(const json& jr) {
  RenderConfig r;
  r.lens_type = LensTypeFromString(jr.value("lens_type", "linear"));
  r.fov = jr.value("fov", RenderConfig{}.fov);
  r.elevation = jr.value("elevation", RenderConfig{}.elevation);
  r.azimuth = jr.value("azimuth", RenderConfig{}.azimuth);
  r.roll = jr.value("roll", RenderConfig{}.roll);
  r.sim_resolution_index = SimResolutionIndexFromValue(jr.value("sim_resolution", 1024));
  std::string vis_str = jr.value("visible", "full");
  if (vis_str == "front") {
    r.visible = kVisibleFull;
    r.front = true;
  } else {
    r.visible = VisibleFromString(vis_str);
    r.front = jr.value("front", false);
  }
  if (jr.contains("background") && jr["background"].is_array() && jr["background"].size() == 3) {
    for (int i = 0; i < 3; i++)
      r.background[i] = jr["background"][i].get<float>();
  }
  if (jr.contains("ray_color") && jr["ray_color"].is_array() && jr["ray_color"].size() == 3) {
    for (int i = 0; i < 3; i++)
      r.ray_color[i] = jr["ray_color"][i].get<float>();
  }
  r.opacity = jr.value("opacity", RenderConfig{}.opacity);
  r.exposure_offset = jr.value("exposure_offset", RenderConfig{}.exposure_offset);
  // Older .lmc payloads carry an "adaptive_brightness_mode" key; nlohmann's value(...) ignores
  // unknown keys, so no migration code is needed — the field becomes a silent no-op.
  return r;
}

// ========== GUI JSON Filter Helper ==========
// Shared between new-format and old-format .lmc deserialization paths.
//
// Two on-disk shapes are read:
//   - v=3 (task-serialization-bidirectional): filter carries "summands" (array
//     of canonical AND-grammar text rows) → parsed into a full SumOfProducts.
//   - v=1/v=2 (legacy): a `type` discriminator ("raypath"/"entry_exit"; missing
//     defaults to "raypath"). Read losslessly and UPGRADED to a SoP via the
//     canonical FromLegacyRaypath / FromLegacyEntryExit converters (';' fan-out
//     for raypath, single EE row otherwise).

static FilterConfig ParseFilterFromGuiJson(const json& jf) {
  FilterConfig f;
  f.name = jf.value("name", std::string{});
  auto action_str = jf.value("action", "filter_in");
  f.action = (action_str == "filter_out") ? 1 : 0;
  f.sym_p = jf.value("sym_p", FilterConfig{}.sym_p);
  f.sym_b = jf.value("sym_b", FilterConfig{}.sym_b);
  f.sym_d = jf.value("sym_d", FilterConfig{}.sym_d);

  // New sum-of-products form: reconstruct each SummandText from its canonical
  // text (factors are a parse cache derived from text).
  if (jf.contains("summands") && jf["summands"].is_array()) {
    SumOfProducts sop;
    for (const auto& js : jf["summands"]) {
      if (!js.is_string()) {
        // Loud, not silent: a corrupt .lmc with a non-string summand row is
        // dropped, but the user is told (code-review-01 Minor 1).
        GUI_LOG_WARNING("[FileIO] Filter summands entry is not a string; skipping row.");
        continue;
      }
      std::string text = js.get<std::string>();
      sop.push_back(SummandText{ text, ParseSummandText(text) });
    }
    if (sop.empty()) {
      // Defensive: an empty summands array is not a valid state — fall back to
      // the default 1-row / 1-factor empty-raypath SoP.
      sop.push_back(SummandText{ std::string{}, std::vector<Factor>{ Factor{ RaypathParams{} } } });
    }
    f.param = std::move(sop);
    return f;
  }

  // Legacy type-discriminated form → upgrade to SoP.
  std::string type = jf.value("type", "raypath");
  if (type == "entry_exit") {
    // Prefer text fields (post-task-filter-modal-polish-v1 schema); fall
    // back to legacy v2 int "entry" / "exit" for older .lmc files. An int
    // 0 → empty string keeps the user-visible "no value typed yet" state
    // distinguishable from "user typed 0".
    EntryExitParams p;
    if (jf.contains("entry_text")) {
      p.entry_text = jf.value("entry_text", std::string{});
    } else if (jf.contains("entry")) {
      int v = jf.value("entry", 0);
      p.entry_text = (v == 0) ? std::string{} : std::to_string(v);
    }
    if (jf.contains("exit_text")) {
      p.exit_text = jf.value("exit_text", std::string{});
    } else if (jf.contains("exit")) {
      int v = jf.value("exit", 0);
      p.exit_text = (v == 0) ? std::string{} : std::to_string(v);
    }
    // Length-mode trio: missing in pre-uplift .lmc files; defaults keep the
    // old "no constraint" behaviour. length_mode is clamped to [0,3] so a
    // corrupt value does not crash the UI Combo.
    p.length_mode = jf.value("length_mode", 0);
    if (p.length_mode < 0 || p.length_mode > 3) {
      p.length_mode = 0;
    }
    p.min_len = jf.value("min_len", 1);
    p.max_len = jf.value("max_len", 1);
    if (p.min_len < 1) {
      p.min_len = 1;
    }
    if (p.max_len < 1) {
      p.max_len = 1;
    }
    f.param = FromLegacyEntryExit(p);
  } else {
    if (type != "raypath") {
      GUI_LOG_WARNING("[FileIO] Unknown filter type '{}', defaulting to raypath", type);
    }
    // FromLegacyRaypath splits ';' multi-segment sugar into canonical OR rows.
    f.param = FromLegacyRaypath(RaypathParams{ jf.value("raypath_text", std::string{}) });
  }
  return f;
}


// ========== Core Config Serialization (export path) ==========

bool BuildExportJsonOrWarn(const GuiState& state, std::string* out_json, std::string* out_warning) {
  // ONE scene serves both roles the pre-handle code needed two constructions for: it is the
  // overflow-validation object AND the export source. (The old code built a throwaway
  // LUMICE_Config `probe` purely to reuse the commit path's ABI-bounds check, then discarded it
  // and re-emitted the same config through a separately-maintained ~180-line JSON writer.)
  // Export and simulation therefore reject an over-limit filter identically by construction, not
  // by two implementations agreeing — the latter previously let export silently write a
  // semantically-opposite match-all stand-in for a filter the simulator refused.
  FilterOverflowInfo overflow;
  ColorClassOverflowInfo color_overflow;
  ScenePtr scene = BuildScene(state, SceneIntent::kJsonExport, &overflow, &color_overflow);
  if (!scene) {
    if (out_warning) {
      // color_overflow.class_index >= 0 iff BuildScene failed on the color-class walk
      // (which runs strictly after the filter walk) rather than a physical-filter overflow —
      // reusing the filter-overflow wording here would misattribute the resource and the
      // limit number (code-review-01 Major 1).
      if (color_overflow.class_index >= 0) {
        *out_warning = "This raypath color configuration exceeds its limits (" +
                       FormatColorOverflowLocator(color_overflow) +
                       ").\nNo config was exported. Simplify the color configuration and try again.";
      } else {
        const std::string locator = FormatOverflowLocator(overflow);
        *out_warning = "This filter has too many OR segments / values to export (limit " +
                       std::to_string(LUMICE_MAX_CONFIG_CLAUSES) + "; " + locator +
                       ").\nNo config was exported. Simplify the filter and try again.";
      }
    }
    return false;
  }
  if (out_json) {
    // LUMICE_SceneToJson emits compact JSON (root.dump()); the exported file is read and edited
    // by humans and fed to the CLI, so restore the 2-space indentation the GUI has always
    // written. Re-indenting here (rather than widening the C API with a pretty-print flag) keeps
    // the formatting choice where the audience is.
    size_t len = 0;
    if (LUMICE_SceneToJson(scene.get(), nullptr, 0, &len) != LUMICE_OK) {
      GUI_LOG_WARNING("[FileIO] BuildExportJsonOrWarn: LUMICE_SceneToJson failed to size the document");
      return false;
    }
    std::string buf(len + 1, '\0');
    if (LUMICE_SceneToJson(scene.get(), buf.data(), buf.size(), nullptr) != LUMICE_OK) {
      GUI_LOG_WARNING("[FileIO] BuildExportJsonOrWarn: LUMICE_SceneToJson failed to serialize the scene");
      return false;
    }
    buf.resize(len);
    *out_json = json::parse(buf).dump(2);
  }
  return true;
}

// Forward decl: FillColorPredicate is the single source of truth for
// (ColorClassRefConfig → LUMICE_ColorPredicate) translation. Since the JSON emit is now
// produced by serializing the very same LUMICE_Scene the commit path builds, struct/JSON
// pixel-equivalence is structural — there is no second predicate emitter to keep in sync.
static bool FillColorPredicate(LUMICE_ColorPredicate* dst, const ColorClassRefConfig& ref);


// ========== Build a LUMICE_Scene handle (for LUMICE_CommitScene / LUMICE_SceneToJson) ==========

static void FillAxisDist(const AxisDist& src, LUMICE_Distribution* dst) {
  static_assert(static_cast<int>(AxisDistType::kCount) == 5, "Update FillAxisDist when adding new AxisDistType");
  switch (src.type) {
    case AxisDistType::kGauss:
      dst->type = LUMICE_DIST_GAUSS;
      break;
    case AxisDistType::kUniform:
      dst->type = LUMICE_DIST_UNIFORM;
      break;
    case AxisDistType::kZigzag:
      dst->type = LUMICE_DIST_ZIGZAG;
      break;
    case AxisDistType::kLaplacian:
      dst->type = LUMICE_DIST_LAPLACIAN;
      break;
    case AxisDistType::kGaussLegacy:
      dst->type = LUMICE_DIST_GAUSS_LEGACY;
      break;
    default:
      dst->type = LUMICE_DIST_GAUSS;
      break;
  }
  dst->center = src.mean;
  dst->spread = src.std;
}

// Helper: fill a LUMICE_CrystalParam from GUI CrystalConfig.
// `dst->id` is deliberately NOT set: LUMICE_SceneAddCrystal ignores any incoming .id and
// assigns its own sequential id, returned via out_id (lumice.h "Incremental build" contract).
// Field-sync guard: see the static_assert(sizeof(CrystalConfig) == 232) near
// SerializeCrystal above. One copy guards both functions (same TU, identical
// condition); this comment keeps the pairing obvious to readers.
static void FillCrystalParam(const CrystalConfig& c, LUMICE_CrystalParam* dst) {
  dst->type = c.type == CrystalType::kPrism ? 0 : 1;
  // GUI shape fields are now first-class ShapeDist; map each straight to LUMICE_Distribution so a
  // GUI-configured randomization actually reaches the simulator (the pre-upgrade NO_RANDOM wrapper
  // silently flattened every field). ToLumiceDistribution is a value-aligned static_cast.
  dst->height = ToLumiceDistribution(c.height);
  dst->prism_h = ToLumiceDistribution(c.prism_h);
  dst->upper_h = ToLumiceDistribution(c.upper_h);
  dst->lower_h = ToLumiceDistribution(c.lower_h);
  dst->upper_wedge_angle = c.upper_alpha;
  dst->lower_wedge_angle = c.lower_alpha;
  for (int i = 0; i < 6; i++) {
    dst->face_distance[i] = ToLumiceDistribution(c.face_distance[i]);
  }
  // Sync groups (v4.13). Same shared translation the preview path uses (crystal_preview.cpp's
  // BuildCrystalMeshData) — the two must agree or the preview shows a crystal the simulator never
  // traces. Raw values; core canonicalizes.
  FillSyncGroupArray(c, dst->sync_group);
  FillAxisDist(c.zenith, &dst->zenith);
  FillAxisDist(c.azimuth, &dst->azimuth);
  FillAxisDist(c.roll, &dst->roll);
}

// Expand a GUI FilterConfig into the scene: one simple filter, or (for a multi-clause /
// multi-term SoP) N simple filters + 1 complex filter whose composition references them by the
// ids the Scene handed back. Sets *out_main_id to the id a scattering entry should reference
// (the complex id when expanded, else the sole simple id). `filter_count` is the running number
// of filters already added to `scene`; it is read for the capacity pre-check and advanced here.
//
// On overflow / any Add failure this returns false and BuildScene destroys the whole scene, so
// there is no "no partial writes" contract to maintain any more (the pre-handle struct version
// had to check every bound up front because its output buffer was the caller's and survived a
// failed call; a partially-built Scene is simply never observable).
//
// The clause / term / filter-capacity pre-checks are kept even though LUMICE_SceneAdd* validates
// too, because only THIS side knows the GUI's (layer, entry, filter name) locator — the Scene can
// only report "this one call was rejected". The pre-handle `composition_count >=
// LUMICE_MAX_CONFIG_COMPLEX` check is gone: it guarded LUMICE_Config's fixed compositions[] pool,
// and a Scene stores each composition inline in its filter object, so that pool no longer exists.
// Complex filters stay bounded by LUMICE_MAX_CONFIG_FILTERS (each consumes >= 2 filter slots).
static bool ExpandFilterToScene(const FilterConfig& f, LUMICE_Scene* scene, int* filter_count, int* out_main_id) {
  const int action = f.action;
  const int symmetry = (f.sym_p ? 1 : 0) | (f.sym_b ? 2 : 0) | (f.sym_d ? 4 : 0);
  // Build one term's LUMICE_FilterParam. `.id` is left zero on purpose — LUMICE_SceneAddFilter
  // ignores it and assigns the id it returns via out_id.
  auto make_term = [&](const SimpleTermDesc& t) {
    LUMICE_FilterParam dst{};
    dst.action = action;
    dst.symmetry = symmetry;
    if (t.is_raypath) {
      dst.type = LUMICE_FILTER_TYPE_RAYPATH;
      dst.raypath_count =
          static_cast<int>(std::min(t.raypath.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_RAYPATH_LEN)));
      for (int k = 0; k < dst.raypath_count; k++) {
        dst.raypath[k] = t.raypath[k];
      }
    } else {
      dst.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
      dst.ee_entry = (t.entry == kEEWildcardSentinel) ? -1 : ClampIdValue(t.entry, "entry");
      dst.ee_exit = (t.exit == kEEWildcardSentinel) ? -1 : ClampIdValue(t.exit, "exit");
      dst.ee_min_len = t.min_len;
      dst.ee_max_len = t.max_len;
    }
    return dst;
  };

  const ExpandedFilter ef = ExpandSopToClauses(f);
  if (ef.overflow) {
    return false;  // exceeds ABI clause/term caps — graceful drop
  }

  if (IsDegenerateSingleTerm(ef)) {
    if (*filter_count >= LUMICE_MAX_CONFIG_FILTERS) {
      return false;
    }
    const LUMICE_FilterParam param = make_term(ef.clauses[0][0]);
    if (LUMICE_SceneAddFilter(scene, &param, out_main_id) != LUMICE_OK) {
      return false;
    }
    (*filter_count)++;
    return true;
  }

  int total_terms = 0;
  for (const auto& clause : ef.clauses) {
    if (static_cast<int>(clause.size()) > LUMICE_MAX_CONFIG_TERMS) {
      return false;  // AND terms per clause (newly reachable via multi-factor rows)
    }
    total_terms += static_cast<int>(clause.size());
  }
  if (static_cast<int>(ef.clauses.size()) > LUMICE_MAX_CONFIG_CLAUSES ||
      *filter_count + total_terms + 1 > LUMICE_MAX_CONFIG_FILTERS) {
    return false;
  }

  // Add the per-term simple filters first (clause order, term order within clause), collecting
  // the ids the Scene assigned. The composition then references THOSE ids rather than ids this
  // function predicts — the pre-handle version had to compute `next_filter_id + running` because
  // it owned the id space; the Scene owns it now (lumice.h: "Cross-referencing fields the caller
  // constructs later ... MUST use these returned out_id values").
  const int clause_n = static_cast<int>(ef.clauses.size());
  std::vector<int> term_counts_vec;
  std::vector<int> term_ids_vec;
  term_counts_vec.reserve(static_cast<size_t>(clause_n));
  term_ids_vec.reserve(static_cast<size_t>(total_terms));
  for (const auto& clause : ef.clauses) {
    term_counts_vec.push_back(static_cast<int>(clause.size()));
    for (const auto& term : clause) {
      const LUMICE_FilterParam param = make_term(term);
      int term_id = -1;
      if (LUMICE_SceneAddFilter(scene, &param, &term_id) != LUMICE_OK) {
        return false;
      }
      (*filter_count)++;
      term_ids_vec.push_back(term_id);
    }
  }

  // Compose the (clause_count, term_counts, term_ids) triple, hand it to the Scene, release it
  // immediately: LUMICE_SceneAddComplexFilter deep-copies the composition before returning, so
  // it must not outlive this call (and leaks if never released).
  LUMICE_ComplexComposition comp{};
  if (LUMICE_CompositionSetClauses(&comp, clause_n, term_counts_vec.data(), term_ids_vec.data()) != LUMICE_OK) {
    return false;
  }
  LUMICE_FilterParam complex_param{};
  complex_param.action = action;
  complex_param.symmetry = symmetry;
  complex_param.type = LUMICE_FILTER_TYPE_COMPLEX;  // ignored by AddComplexFilter; set for clarity
  const LUMICE_ErrorCode err = LUMICE_SceneAddComplexFilter(scene, &complex_param, &comp, out_main_id);
  LUMICE_CompositionReleaseClauses(&comp);
  if (err != LUMICE_OK) {
    return false;
  }
  (*filter_count)++;
  return true;
}

std::string FormatOverflowLocator(const FilterOverflowInfo& overflow) {
  // 1-based Layer/Entry to match the panel header convention (panels.cpp "Layer %d").
  const std::string pos =
      "Layer " + std::to_string(overflow.layer_index + 1) + " / Entry " + std::to_string(overflow.entry_index + 1);
  if (!overflow.filter_name.empty()) {
    return "filter \"" + overflow.filter_name + "\", " + pos;
  }
  return pos;
}

std::string FormatColorOverflowLocator(const ColorClassOverflowInfo& overflow) {
  if (overflow.class_over_cap) {
    return "too many color classes (limit " + std::to_string(LUMICE_MAX_CONFIG_COLOR_CLASSES) + ")";
  }
  // 1-based class index to match the panel's user-facing numbering convention.
  return "color class " + std::to_string(overflow.class_index + 1) + " has too many match references (limit " +
         std::to_string(LUMICE_MAX_CONFIG_COLOR_REFS) + ")";
}

// task-342.3 Step 3: fill a single LUMICE_ColorPredicate from a GUI ColorClassRefConfig.
// `match_all=true` (or empty/whitespace predicate_text) → LUMICE_FILTER_TYPE_UNSET.
// Non-empty text is parsed via raypath_segments::ParseSummandText; only single-Factor and
// single-alternative results are accepted (LUMICE_ColorPredicate is a single-atom carrier per
// Design 2). If the text does not resolve to exactly one atom, returns false — caller
// (AddColorClasses below) treats it as "skip this ref" (mirrors the plan §3 decision:
// arbitrary AND/multi-alt goes through combine:all across refs, not inside one predicate).
static bool FillColorPredicate(LUMICE_ColorPredicate* dst, const ColorClassRefConfig& ref) {
  *dst = LUMICE_ColorPredicate{};
  // task-356.3 — symmetry bitmask (1=P, 2=B, 4=D). Applies uniformly to all
  // predicate types (UNSET/RAYPATH/ENTRY_EXIT) so it lives before the type
  // dispatch. Literal 1/2/4 mirrors src/server/c_api.cpp SymmetryBitsToString
  // (no named LUMICE_SYM_* constants in the public header today); keep in sync.
  dst->symmetry = (ref.sym_p ? 1 : 0) | (ref.sym_b ? 2 : 0) | (ref.sym_d ? 4 : 0);
  const std::string trimmed = TrimRaypathSegment(ref.predicate_text);
  if (ref.match_all || trimmed.empty()) {
    dst->type = LUMICE_FILTER_TYPE_UNSET;  // match-all whole-crystal
    return true;
  }
  const auto factors = ParseSummandText(trimmed);
  if (factors.size() != 1) {
    return false;
  }
  const auto alts = FactorAlternatives(factors[0]);
  if (alts.size() != 1) {
    return false;
  }
  const SimpleTermDesc& t = alts[0];
  if (t.is_raypath) {
    if (t.raypath.empty()) {
      // Match-all raypath (whole-crystal via empty raypath) — represent as UNSET to align
      // with core RaypathColorRef default (no `type` field == NoneFilterParam == match-all).
      dst->type = LUMICE_FILTER_TYPE_UNSET;
      return true;
    }
    dst->type = LUMICE_FILTER_TYPE_RAYPATH;
    dst->raypath_count =
        static_cast<int>(std::min(t.raypath.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_RAYPATH_LEN)));
    for (int k = 0; k < dst->raypath_count; k++) {
      dst->raypath[k] = t.raypath[k];
    }
    return true;
  }
  dst->type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  dst->ee_entry = (t.entry == kEEWildcardSentinel) ? -1 : ClampIdValue(t.entry, "entry");
  dst->ee_exit = (t.exit == kEEWildcardSentinel) ? -1 : ClampIdValue(t.exit, "exit");
  dst->ee_min_len = t.min_len;
  dst->ee_max_len = t.max_len;
  return true;
}

// Walk state.raypath_color and add one LUMICE_ColorClass per class to
// `scene`. Returns false on ABI-bounds overflow (class count or per-class ref count over cap)
// and reports the offending index via `color_overflow` (when non-null). Orphan refs
// (crystal_pool_id not in `crystal_pool_to_core` because the referenced crystal isn't in any
// active scattering entry) are SKIPPED at the ref level (not the class) with a warning log —
// mirrors the plan §3 graceful degradation for referenced placements the user removed after
// configuring the color class. Bad predicate text (multi-factor / multi-alt) is likewise
// skipped at the ref level.
//
// The pre-handle version had to call LUMICE_ConfigCreateColorClasses first (LUMICE_Config's
// raypath_color is an owning heap array the caller then had to Release). LUMICE_SceneAddColorClass
// deep-copies a stack temporary instead, so that whole allocate-then-fill-in-place ownership
// dance is gone.
static bool AddColorClasses(const GuiState& state, const std::map<int, int>& crystal_pool_to_core, LUMICE_Scene* scene,
                            ColorClassOverflowInfo* color_overflow) {
  const int n_classes = static_cast<int>(state.raypath_color.size());
  if (n_classes > LUMICE_MAX_CONFIG_COLOR_CLASSES) {
    if (color_overflow != nullptr) {
      color_overflow->class_index = LUMICE_MAX_CONFIG_COLOR_CLASSES;
      color_overflow->ref_index = -1;
      color_overflow->class_over_cap = true;
    }
    return false;
  }
  // ABI-bounds pre-check on per-class ref counts (COUNTING orphan/invalid refs would be
  // wrong — those will be skipped; count only the surviving refs to compare with the cap).
  // Two-pass approach: first count surviving refs per class, reject any class over cap;
  // then emit. Rejecting BEFORE the first Add keeps the overflow report (which class, which
  // ref) exact — the Scene could only tell us "this Add was rejected".
  std::vector<std::vector<int>> keep_refs(n_classes);  // per-class list of surviving ref indices
  for (int i = 0; i < n_classes; i++) {
    const auto& cls = state.raypath_color[i];
    for (int j = 0; j < static_cast<int>(cls.match.size()); j++) {
      const auto& ref = cls.match[j];
      if (crystal_pool_to_core.find(ref.crystal_pool_id) == crystal_pool_to_core.end()) {
        GUI_LOG_WARNING(
            "[FileIO] raypath_color[{}].match[{}] references crystal pool {} not present in scene; skipped.", i, j,
            ref.crystal_pool_id);
        continue;
      }
      LUMICE_ColorPredicate probe{};
      if (!FillColorPredicate(&probe, ref)) {
        GUI_LOG_WARNING("[FileIO] raypath_color[{}].match[{}] predicate not a single atom (\"{}\"); skipped.", i, j,
                        ref.predicate_text);
        continue;
      }
      keep_refs[i].push_back(j);
    }
    if (static_cast<int>(keep_refs[i].size()) > LUMICE_MAX_CONFIG_COLOR_REFS) {
      if (color_overflow != nullptr) {
        color_overflow->class_index = i;
        color_overflow->ref_index = LUMICE_MAX_CONFIG_COLOR_REFS;
        color_overflow->class_over_cap = false;
      }
      return false;
    }
  }
  if (n_classes == 0) {
    // Mono-only state: touch nothing. Neither SetColorMode nor AddColorClass may run, or the
    // scene grows a raypath_color key and stops being byte-identical to the pre-v4.7 wire form
    // (the same "count == 0 -> omit" isomorphism the shared JSON encoder keeps).
    return true;
  }
  // raypath_color_mode: pass through verbatim; the Scene validates the enum range.
  if (LUMICE_SceneSetColorMode(scene, state.raypath_color_mode) != LUMICE_OK) {
    return false;
  }
  for (int i = 0; i < n_classes; i++) {
    const auto& cls = state.raypath_color[i];
    LUMICE_ColorClass dst{};
    dst.color[0] = cls.color[0];
    dst.color[1] = cls.color[1];
    dst.color[2] = cls.color[2];
    dst.combine = cls.combine;  // 0 (any) / 1 (all) — matches LUMICE_COLOR_COMBINE_*
    dst.visible = cls.visible ? 1 : 0;
    dst.solo = cls.solo ? 1 : 0;
    int m = 0;
    for (int j : keep_refs[i]) {
      const auto& ref = cls.match[j];
      LUMICE_ColorClassRef& dref = dst.match[m++];
      dref.layer = ref.layer_idx;
      dref.crystal = crystal_pool_to_core.at(ref.crystal_pool_id);
      // FillColorPredicate already validated this ref in the pre-check; assumed to succeed.
      (void)FillColorPredicate(&dref.predicate, ref);
    }
    dst.match_count = m;
    int class_id = -1;
    if (LUMICE_SceneAddColorClass(scene, &dst, &class_id) != LUMICE_OK) {
      return false;
    }
  }
  return true;
}

ScenePtr BuildScene(const GuiState& state, SceneIntent intent, FilterOverflowInfo* overflow,
                    ColorClassOverflowInfo* color_overflow) {
  ScenePtr scene(LUMICE_SceneCreate());
  if (!scene) {
    GUI_LOG_WARNING("[FileIO] BuildScene: LUMICE_SceneCreate failed");
    return nullptr;
  }

  // ID-pool model: walk entries, dedupe by pool id, emit one scene crystal per reachable pool
  // slot. Filters map pool_id -> main_id (the id a scattering entry references; the complex id
  // for an expanded multi-segment/multi-value filter). Both maps now store the id the SCENE
  // assigned (its out_id) rather than one the GUI computed: the pre-handle code derived crystal
  // ids as `pool_id + 1` and ran its own filter-id counter purely to fill LUMICE_Config's arrays
  // consistently. The Scene owns id assignment (lumice.h: "The Scene assigns this id itself and
  // IGNORES any `.id` field on the incoming POD"), so those two counters are gone. Ids stay
  // insertion-ordered either way, so the dedupe walk below is unchanged.
  std::map<int, int> crystal_pool_to_core;  // pool_id -> scene crystal id
  std::map<int, int> filter_pool_to_core;   // pool_id -> main_id
  int filter_count = 0;                     // filters added so far (ABI capacity pre-check)

  const int layer_n =
      static_cast<int>(std::min(state.layers.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_SCATTER_LAYERS)));
  for (int i = 0; i < layer_n; i++) {
    const auto& layer = state.layers[i];
    // One layer is assembled on the stack exactly as before, then handed to the Scene in a
    // single Add (which deep-copies it) instead of being written into out->scattering[i].
    LUMICE_ScatterLayer dst_layer{};
    dst_layer.probability = layer.probability;
    dst_layer.entry_count =
        static_cast<int>(std::min(layer.entries.size(), static_cast<size_t>(LUMICE_MAX_CONFIG_SCATTER_ENTRIES)));
    for (int k = 0; k < dst_layer.entry_count; k++) {
      const auto& entry = layer.entries[k];

      int cid;
      auto it_c = crystal_pool_to_core.find(entry.crystal_id);
      if (it_c == crystal_pool_to_core.end()) {
        if (static_cast<int>(crystal_pool_to_core.size()) >= LUMICE_MAX_CONFIG_CRYSTALS) {
          // Scene crystal capacity reached — truncate entries from this point.
          dst_layer.entry_count = k;
          break;
        }
        LUMICE_CrystalParam param{};
        FillCrystalParam(state.crystals[entry.crystal_id], &param);
        if (LUMICE_SceneAddCrystal(scene.get(), &param, &cid) != LUMICE_OK) {
          GUI_LOG_WARNING("[FileIO] BuildScene: LUMICE_SceneAddCrystal rejected crystal pool {}", entry.crystal_id);
          return nullptr;
        }
        crystal_pool_to_core.emplace(entry.crystal_id, cid);
      } else {
        cid = it_c->second;
      }
      dst_layer.entries[k].crystal_id = cid;
      dst_layer.entries[k].proportion = entry.proportion;

      if (entry.filter_id.has_value()) {
        int fpool = *entry.filter_id;
        int fid = -1;
        auto it_f = filter_pool_to_core.find(fpool);
        if (it_f != filter_pool_to_core.end()) {
          fid = it_f->second;
        } else {
          int main_id = -1;
          if (ExpandFilterToScene(state.filters[fpool], scene.get(), &filter_count, &main_id)) {
            filter_pool_to_core.emplace(fpool, main_id);
            fid = main_id;
          } else {
            // Exceeded ABI bounds (clause / term / filter capacity). Bail out immediately so the
            // caller keeps the old committed state rather than commit a truncated config — the
            // half-built scene dies with the ScenePtr, so there is nothing to unwind.
            // Capture the current (layer, entry, filter name) so the caller can locate the
            // offending filter. Only the FIRST reference is reported — if the same filter pool
            // id is used at multiple sites, later ones are not surfaced (they share content).
            if (overflow != nullptr) {
              overflow->layer_index = i;
              overflow->entry_index = k;
              overflow->filter_name = state.filters[fpool].name;
            }
            return nullptr;
          }
        }
        dst_layer.entries[k].filter_id = fid;
      } else {
        dst_layer.entries[k].filter_id = -1;
      }
    }
    int layer_id = -1;
    if (LUMICE_SceneAddScatterLayer(scene.get(), &dst_layer, &layer_id) != LUMICE_OK) {
      GUI_LOG_WARNING("[FileIO] BuildScene: LUMICE_SceneAddScatterLayer rejected layer {}", i);
      return nullptr;
    }
  }

  // Renderer — single renderer always emitted.
  // NOTE: GUI enforces single renderer; if multi-renderer support is added, revisit this
  // loop-of-one structure.
  {
    const auto& r = state.renderer;
    LUMICE_RenderParam dst{};
    int res = kSimResolutions[r.sim_resolution_index];
    dst.resolution_w = res * 2;
    dst.resolution_h = res;
    dst.opacity = r.opacity;
    // doc/ev-pipeline-architecture.md §2.4/§4 — the ONE field where the two
    // SceneIntent arms differ, and the reason SceneIntent exists at all:
    //
    // kSimCommit (GUI Run): intensity_factor must stay at RenderConfig::intensity_factor_'s
    // default (config/render_config.hpp's 1.0f; bound as a literal here because the GUI API
    // boundary forbids src/gui/ from including core/config headers — if that default ever
    // changes, sync this). GUI manual + auto EV are carried entirely by the display-time path:
    // mono via a shader uniform (app.cpp RefreshPreviewParams), composite via
    // LUMICE_SetCompositeExposure (pushed every frame from RenderPreviewPanel as
    // display_exposure_scale, feeding the compositor's single shared exposure scalar
    // s = ExposureScale() × display_exposure_scale). Baking 2^exposure_offset here would
    // double-apply manual EV on any re-run with EV != 0.
    //
    // kJsonExport (Save Config JSON): must bake 2^exposure_offset, because the CLI has no
    // display-time EV concept — re-rendering the exported config has to reproduce the
    // brightness the user is looking at (the Save Config semantic promise).
    dst.intensity_factor = intent == SceneIntent::kJsonExport ? std::pow(2.0f, r.exposure_offset) : 1.0f;
    dst.overlap = kDualFisheyeOverlap;
    // v4.11: LUMICE_RenderParam carries the full renderer description, so the values the C API
    // used to hardcode while re-encoding a renderer now have to be stated here. Core always
    // produces a dual equal-area fisheye full-globe texture; the GUI shader reprojects it to the
    // user's display projection, so these are GUI policy, not user-facing settings.
    dst.lens_type = LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA;  // was hardcoded in RendererToJson
    dst.lens_fov = 180.0f;                                     // was hardcoded in RendererToJson
    dst.visible = LUMICE_VISIBLE_FULL;                         // was hardcoded in RendererToJson
    // Never encoded pre-v4.11, so there is no prior hardcoded value to reproduce: take core's
    // authoritative defaults instead. {-1,-1,-1} is RenderConfig::ray_color_'s "use the natural
    // spectral color" sentinel — a zero-initialized {0,0,0} would tint every ray black.
    dst.ray_color[0] = dst.ray_color[1] = dst.ray_color[2] = -1.0f;
    dst.celestial_outline = 1;  // core RenderConfig::celestial_outline_ default (true)
    // view / background / lens_shift / grid counts keep their zero-initialized values, which match
    // both the pre-v4.11 hardcoded encoding and core's defaults. The GUI's own screen-space
    // overlay grid (gui_state.show_grid_line) is a display-time layer and is unrelated to
    // render[].grid, which core bakes into the image.
    int renderer_id = -1;
    if (LUMICE_SceneAddRenderer(scene.get(), &dst, &renderer_id) != LUMICE_OK) {
      GUI_LOG_WARNING("[FileIO] BuildScene: LUMICE_SceneAddRenderer failed");
      return nullptr;
    }
  }

  // Scene: light source. sun_azimuth is always 0 (the GUI exposes no azimuth control).
  const char* spectrum_name = "D65";
  const bool use_custom_spectrum =
      state.sun.spectrum_index == kCustomSpectrumIndex && !state.sun.custom_spectrum.empty();
  if (!use_custom_spectrum && state.sun.spectrum_index >= 0 && state.sun.spectrum_index < kSpectrumCount) {
    spectrum_name = kSpectrumNames[state.sun.spectrum_index];
  }
  if (LUMICE_SceneSetLightSource(scene.get(), state.sun.altitude, 0.0f, state.sun.diameter, spectrum_name) !=
      LUMICE_OK) {
    GUI_LOG_WARNING("[FileIO] BuildScene: LUMICE_SceneSetLightSource failed");
    return nullptr;
  }
  if (use_custom_spectrum) {
    // Discrete custom spectrum overrides the string, matching the core config's own rule.
    // SetCustomSpectrum is only called in this branch: calling it with
    // count == 0 would CLEAR the spectrum back to "D65" and clobber the preset name above.
    int n = std::min(static_cast<int>(state.sun.custom_spectrum.size()), kSpectrumHardMax);
    if (static_cast<int>(state.sun.custom_spectrum.size()) > kSpectrumHardMax) {
      GUI_LOG_WARNING("[FileIO] custom spectrum truncated from {} to {} entries", state.sun.custom_spectrum.size(),
                      kSpectrumHardMax);
    }
    std::vector<LUMICE_SpectrumEntry> entries(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
      entries[i].wavelength = state.sun.custom_spectrum[i].wavelength;
      entries[i].weight = state.sun.custom_spectrum[i].weight;
    }
    if (LUMICE_SceneSetCustomSpectrum(scene.get(), entries.data(), n) != LUMICE_OK) {
      GUI_LOG_WARNING("[FileIO] BuildScene: LUMICE_SceneSetCustomSpectrum failed");
      return nullptr;
    }
  }

  // Scene: simulation. geom_clock is passed 0 explicitly — the GUI has never set it (the
  // pre-handle path left the memset-0 field untouched) and exposes no control for it.
  if (LUMICE_SceneSetSimParams(scene.get(), state.sim.infinite ? 1 : 0,
                               static_cast<LUMICE_RayCount>(state.sim.ray_num_millions * 1e6), state.sim.max_hits,
                               /*geom_clock=*/0) != LUMICE_OK) {
    GUI_LOG_WARNING("[FileIO] BuildScene: LUMICE_SceneSetSimParams failed");
    return nullptr;
  }

  // task-342.3 Step 3: raypath color classes. Reuses `crystal_pool_to_core` built above so
  // ref.crystal_pool_id (GUI pool index) resolves to the same scene crystal id the scattering
  // entries reference. Orphan/invalid refs are skipped at ref level; class-cap / ref-cap
  // overflow returns nullptr with color_overflow populated.
  if (!AddColorClasses(state, crystal_pool_to_core, scene.get(), color_overflow)) {
    return nullptr;
  }

  return scene;
}

// ========== Core JSON Deserialization (for JSON import) ==========
// Handles Core config JSON format: root has "crystal"/"filter"/"scene"/"render" keys.
// Crystal/filter are referenced by ID in scene.scattering entries; this function converts
// the ID references into the copy-model (EntryCard/Layer) by looking up each scatter entry's
// crystal/filter ID in the parsed maps.
// NOTE: This is NOT the .lmc GUI state format — see DeserializeGuiStateJson for that.

// Parse a core-JSON EE simple-filter entry's length fields into the GUI's
// (length_mode, min_len, max_len) triple. Shared by the simple-filter parse
// branch in DeserializeFromJson and by TryReconstructComplexFilter so both
// paths use the identical mapping of (has_min, has_max) → length_mode.
static void DecodeEELengthFromJson(const json& jf, int& length_mode, int& min_len_out, int& max_len_out) {
  const bool has_min = jf.contains("min_len") && !jf.at("min_len").is_null();
  const bool has_max = jf.contains("max_len") && !jf.at("max_len").is_null();
  const int min_v = has_min ? std::max(jf.at("min_len").get<int>(), 1) : 1;
  const int max_v = has_max ? std::max(jf.at("max_len").get<int>(), 1) : 1;
  if (!has_min && !has_max) {
    length_mode = 0;
  } else if (has_max && (min_v == max_v) && has_min) {
    length_mode = 1;  // strict
  } else if (has_max && !has_min) {
    length_mode = 2;  // upper bound only
  } else if (has_min && !has_max) {
    GUI_LOG_WARNING("[FileIO] entry_exit filter with min_len={} but no max_len; mapping to range mode with max={}",
                    min_v, kEELenAbsoluteMax);
    length_mode = 3;
  } else {
    length_mode = 3;
  }
  min_len_out = min_v;
  if (has_max) {
    max_len_out = max_v;
  } else if (has_min) {
    max_len_out = kEELenAbsoluteMax;
  } else {
    max_len_out = min_v;
  }
}

// Decode a core-JSON EE "entry"/"exit" face field into the GUI's text form.
// Wildcard is encoded by field-absence (the core emitter omits entry/exit when the
// LUMICE_FilterParam field is negative) — so absent/null → empty text. A defensive negative
// (legacy/hand-written explicit sentinel) is also treated as wildcard; any
// non-negative integer (including face 0) is a real face → its decimal text.
// Shared by the simple-filter parse branch and TryReconstructComplexFilter so
// both paths decode faces identically.
static std::string DecodeEEFaceFromJson(const json& jf, const char* key) {
  if (!jf.contains(key) || jf.at(key).is_null()) {
    return std::string{};
  }
  const int v = jf.at(key).get<int>();
  return (v < 0) ? std::string{} : std::to_string(v);
}

// Reverse of ExpandFilterToScene — reconstruct a GUI FilterConfig from a core
// "complex" filter. The sum-of-products model (task-serialization-bidirectional)
// makes the GUI able to express a general OR-of-(AND-of-terms): each composition
// product (clause) becomes one SummandText, and each term id in the product
// becomes one Factor (raypath / entry_exit). This GENERALIZES the earlier
// explore-271 shape, which could only reverse a pure OR-of-singleton-products of
// one uniform child type (true AND / mixed type / non-factorizable EE were
// loudly rejected). Those are now reconstructed directly.
//
// Rejection is now reserved for genuinely non-representable inputs (GUI `Factor`
// has only raypath / entry_exit arms):
//   - a term id whose child type is not raypath/entry_exit (e.g. direction/crystal)
//   - a term id that points to another complex filter (nested complex)
//   - a term id with no matching child in the pool (dangling reference)
//   - a malformed / empty composition or non-integer term id
// On rejection returns false with fail_reason for the caller's loud warning
// (explore-271 anti-silent-miscull contract preserved for the unsupported cases).
static bool TryReconstructComplexFilter(const json& jf, const std::map<int, json>& raw, FilterConfig& out,
                                        std::string& fail_reason) {
  const int complex_id = jf.value("id", 0);

  if (!jf.contains("composition") || !jf["composition"].is_array() || jf["composition"].empty()) {
    fail_reason = "complex filter id=" + std::to_string(complex_id) + ": composition missing/empty";
    return false;
  }

  FilterConfig f;
  f.name = jf.value("name", std::string{});
  const auto action_str = jf.value("action", "filter_in");
  f.action = (action_str == "filter_out") ? 1 : 0;
  const auto sym = jf.value("symmetry", std::string{});
  f.sym_p = (sym.find('P') != std::string::npos);
  f.sym_b = (sym.find('B') != std::string::npos);
  f.sym_d = (sym.find('D') != std::string::npos);

  SumOfProducts sop;
  for (const auto& raw_product : jf["composition"]) {  // product = clause = AND of term ids
    // The core wire form writes a SINGLE-term clause as a bare id and a multi-term clause as an
    // array of ids (core filter_config.cpp to_json / c_api.cpp CompositionArrayToJson); both are
    // legal and mean the same thing. Normalize to the array form before walking terms. Until
    // 399.5 the GUI read back only its own emitter's output, which always wrote the array form,
    // so the bare form was silently unsupported — a config written by the CLI/core (or now by
    // the GUI's own export, which goes through the core encoder) failed to reconstruct.
    const json product = raw_product.is_array() ? raw_product : json::array({ raw_product });
    if (product.empty()) {
      fail_reason = "complex filter id=" + std::to_string(complex_id) + ": composition clause is not a non-empty array";
      return false;
    }
    std::vector<Factor> factors;
    factors.reserve(product.size());
    for (const auto& term : product) {
      if (!term.is_number_integer()) {
        fail_reason = "complex filter id=" + std::to_string(complex_id) + ": composition entry is not an integer id";
        return false;
      }
      const int cid = term.get<int>();
      auto it = raw.find(cid);
      if (it == raw.end()) {
        fail_reason =
            "complex filter id=" + std::to_string(complex_id) + ": child id=" + std::to_string(cid) + " not found";
        return false;
      }
      const json& cj = it->second;
      const std::string ctype = cj.value("type", std::string{});
      // NOTE: a MISSING/empty `type` is malformed core-JSON and must fall to the loud
      // reject branch below (explore-271 anti-silent-miscull contract) — do NOT fold it
      // into the raypath branch, which would silently rewrite it into a match-all filter
      // (code-review-04 Major 1). Only an empty `raypath` ARRAY (with type=="raypath") is
      // the legitimate match-all wildcard.
      if (ctype == "raypath") {
        // Join the raypath face ids with '-'. An empty/absent array is the
        // match-all raypath (empty text) — representable as a wildcard factor.
        std::string text;
        if (cj.contains("raypath") && cj["raypath"].is_array()) {
          for (size_t k = 0; k < cj["raypath"].size(); ++k) {
            if (k > 0) {
              text += kRaypathSepStr;
            }
            text += std::to_string(cj["raypath"][k].get<int>());
          }
        }
        factors.emplace_back(RaypathParams{ text });
      } else if (ctype == "entry_exit") {
        EntryExitParams p;
        p.entry_text = DecodeEEFaceFromJson(cj, "entry");
        p.exit_text = DecodeEEFaceFromJson(cj, "exit");
        DecodeEELengthFromJson(cj, p.length_mode, p.min_len, p.max_len);
        factors.emplace_back(std::move(p));
      } else if (ctype == "complex") {
        fail_reason = "complex filter id=" + std::to_string(complex_id) + ": child id=" + std::to_string(cid) +
                      " is itself a complex filter (nested complex not representable in GUI)";
        return false;
      } else {
        fail_reason = "complex filter id=" + std::to_string(complex_id) + ": child id=" + std::to_string(cid) +
                      " has unsupported type '" + ctype + "' (GUI factors are raypath / entry_exit only)";
        return false;
      }
    }
    // Canonical text is derived from the reconstructed factors; factors is the
    // parse cache carried alongside it.
    sop.push_back(SummandText{ FormatSummandText(factors), std::move(factors) });
  }

  f.param = std::move(sop);
  out = f;
  return true;
}

bool DeserializeFromJson(const std::string& json_str, GuiState& state) {
  json root;
  try {
    root = json::parse(json_str);
  } catch (...) {
    return false;
  }

  state = GuiState{};

  // Parse crystals and filters into temporary ID-indexed maps for scatter entry lookup
  std::map<int, CrystalConfig> crystal_map;
  if (root.contains("crystal") && root["crystal"].is_array()) {
    for (auto& jc : root["crystal"]) {
      int id = jc.value("id", 0);
      crystal_map[id] = ParseCrystal(jc);
    }
  }

  std::map<int, FilterConfig> filter_map;
  // Two-pass filter parse: Pass 1 indexes every filter JSON by id so Pass 2b
  // can rebuild a `complex` filter even when its referenced children appear
  // later in the array (GUI-emitted JSON always orders children first, but
  // hand-authored JSON makes no such promise). Pass 2a then runs the
  // pre-existing simple-filter decode verbatim, and Pass 2b reverse-maps
  // degenerate complex filters via TryReconstructComplexFilter.
  std::map<int, json> raw_filter_json;
  if (root.contains("filter") && root["filter"].is_array()) {
    for (auto& jf : root["filter"]) {
      raw_filter_json[jf.value("id", 0)] = jf;
    }

    // Pass 2a: parse non-complex filters into filter_map. Iterating the
    // id-keyed map walks filters in ascending-id order rather than JSON-array
    // order; this is intentional and semantically equivalent here (filter_map
    // is keyed by unique id, no ordering dependency in downstream consumers).
    for (const auto& entry : raw_filter_json) {
      const int id = entry.first;
      const json& jf = entry.second;
      const auto type_str = jf.value("type", std::string{});
      if (type_str == "complex") {
        continue;
      }
      FilterConfig f;
      f.name = jf.value("name", std::string{});
      auto action_str = jf.value("action", std::string{ "filter_in" });
      f.action = (action_str == "filter_out") ? 1 : 0;
      auto sym = jf.value("symmetry", std::string{});
      f.sym_p = (sym.find('P') != std::string::npos);
      f.sym_b = (sym.find('B') != std::string::npos);
      f.sym_d = (sym.find('D') != std::string::npos);

      if (type_str == "raypath" || type_str.empty()) {
        std::string text;
        if (jf.contains("raypath") && jf["raypath"].is_array()) {
          for (size_t i = 0; i < jf["raypath"].size(); i++) {
            if (i > 0) {
              text += kRaypathSepStr;
            }
            text += std::to_string(jf["raypath"][i].get<int>());
          }
        }
        f.SetRaypath(RaypathParams{ text });
      } else if (type_str == "entry_exit") {
        // core JSON keeps int "entry" / "exit" — translate to GUI's text
        // representation. Absent or explicit-null fields are the wildcard
        // form (post-uplift schema) and map to an empty string. min_len /
        // max_len are decoded into the four GUI length-mode buckets via the
        // shared DecodeEELengthFromJson helper (reused by complex EE rebuild).
        EntryExitParams p;
        p.entry_text = DecodeEEFaceFromJson(jf, "entry");
        p.exit_text = DecodeEEFaceFromJson(jf, "exit");
        DecodeEELengthFromJson(jf, p.length_mode, p.min_len, p.max_len);
        f.SetEntryExit(p);
      } else {
        GUI_LOG_WARNING("[FileIO] Unknown filter type '{}' on core JSON import, defaulting to empty raypath", type_str);
        f.SetRaypath(RaypathParams{});
      }
      filter_map[id] = f;
    }

    // Pass 2b: reverse-map complex filters into GUI sum-of-products (each
    // composition clause → a SummandText, each term id → a Factor). Only
    // genuinely non-representable inputs (child type not raypath/entry_exit,
    // nested complex, dangling reference) are rejected with a loud warning so
    // the user notices the silent culling instead of a wrong image (explore-271).
    for (const auto& entry : raw_filter_json) {
      const int id = entry.first;
      const json& jf = entry.second;
      if (jf.value("type", std::string{}) != "complex") {
        continue;
      }
      FilterConfig rebuilt;
      std::string fail_reason;
      if (TryReconstructComplexFilter(jf, raw_filter_json, rebuilt, fail_reason)) {
        filter_map[id] = rebuilt;
      } else {
        GUI_LOG_WARNING("[FileIO] {}", fail_reason);
        SetImportComplexFilterWarning(fail_reason);
      }
    }
  }

  // Scene
  if (root.contains("scene")) {
    auto& js = root["scene"];

    if (js.contains("light_source")) {
      auto& jl = js["light_source"];
      state.sun.altitude = jl.value("altitude", 20.0f);
      state.sun.diameter = jl.value("diameter", 0.5f);
      if (jl.contains("spectrum")) {
        const auto& sp = jl["spectrum"];
        if (sp.is_string()) {
          state.sun.spectrum_index = SpectrumFromString(sp.get<std::string>());
          state.sun.custom_spectrum.clear();
        } else if (sp.is_array()) {
          // Prior bug: silently dropped discrete arrays (no fallback log). Now imported into
          // state.sun.custom_spectrum with the sentinel index.
          state.sun.custom_spectrum = ParseWlWeightArray(sp);
          state.sun.spectrum_index = state.sun.custom_spectrum.empty() ? 2 /* D65 */ : kCustomSpectrumIndex;
        }
      }
    }

    if (js.contains("ray_num")) {
      if (js["ray_num"].is_string() && js["ray_num"].get<std::string>() == "infinite") {
        state.sim.infinite = true;
      } else if (js["ray_num"].is_number()) {
        state.sim.ray_num_millions = static_cast<float>(js["ray_num"].get<size_t>()) / 1e6;
      }
    }
    state.sim.max_hits = js.value("max_hits", 8);

    // Convert ID-referenced scattering to copy-model layers.
    if (js.contains("scattering") && js["scattering"].is_array()) {
      const auto& jscattering = js["scattering"];

      // Import to ID-pool: dedupe by source JSON ids, so two entries with the
      // same crystal/filter reference share one pool slot (identity).
      std::map<int, int> crystal_id_to_pool;
      std::map<int, int> filter_id_to_pool;
      for (const auto& jlayer : jscattering) {
        Layer layer;
        layer.probability = jlayer.value("prob", 1.0f);
        if (jlayer.contains("entries") && jlayer["entries"].is_array()) {
          const auto& jentries = jlayer["entries"];
          for (const auto& je : jentries) {
            EntryCard entry;
            int crystal_id_flat = je.value("crystal", -1);
            if (crystal_map.count(crystal_id_flat)) {
              auto it = crystal_id_to_pool.find(crystal_id_flat);
              if (it == crystal_id_to_pool.end()) {
                entry.crystal_id = static_cast<int>(state.crystals.size());
                state.crystals.push_back(crystal_map[crystal_id_flat]);
                crystal_id_to_pool.emplace(crystal_id_flat, entry.crystal_id);
              } else {
                entry.crystal_id = it->second;
              }
            } else {
              // Unknown crystal reference — fall back to a fresh default pool
              // slot so entry.crystal_id stays valid (avoid OOB on render).
              entry.crystal_id = static_cast<int>(state.crystals.size());
              state.crystals.emplace_back();
            }
            entry.proportion = je.value("proportion", 1.0f);
            int filter_id = je.value("filter", -1);
            if (filter_id >= 0 && filter_map.count(filter_id)) {
              auto it = filter_id_to_pool.find(filter_id);
              if (it == filter_id_to_pool.end()) {
                int pool_id = static_cast<int>(state.filters.size());
                state.filters.push_back(filter_map[filter_id]);
                filter_id_to_pool.emplace(filter_id, pool_id);
                entry.filter_id = pool_id;
              } else {
                entry.filter_id = it->second;
              }
            }
            layer.entries.push_back(entry);
          }
        }
        state.layers.push_back(layer);
      }

      // task-342.3 Step 4: parse top-level raypath_color into state. Placed inside the
      // scattering block so crystal_id_to_pool (built above) is still in scope — GUI ref uses
      // pool ids (indices into state.crystals), not JSON crystal ids. Accepts both wire
      // shapes core RaypathColorConfig::from_json accepts: bare array (dominant-only) or
      // object {"mode": ..., "classes": [...]}. z_order defaults to physical index i (the
      // natural new-class placement) — the wire form has no z_order field (mirrors
      // LUMICE_ColorClass / c_api.cpp ConfigToJson), so this default is what the plan's
      // roundtrip test relies on to compare equal via ColorClassConfig::operator==.
      if (root.contains("raypath_color")) {
        const auto& jrc = root["raypath_color"];
        const json* classes_arr = nullptr;
        // Bare-array wire form / object with no "mode" field defaults to painter
        // — mirrors core RaypathColorConfig::from_json's default (doc §4.8).
        // GUI cannot include config/raypath_color_config.hpp per AGENTS.md, so
        // this literal is kept in sync by convention with kDefaultCompositeMode.
        std::string mode_str = "painter";
        if (jrc.is_array()) {
          classes_arr = &jrc;
        } else if (jrc.is_object()) {
          if (jrc.contains("mode") && jrc["mode"].is_string()) {
            mode_str = jrc["mode"].get<std::string>();
          }
          if (jrc.contains("classes") && jrc["classes"].is_array()) {
            classes_arr = &jrc["classes"];
          }
        }
        if (mode_str == "dominant") {
          state.raypath_color_mode = LUMICE_COLOR_MODE_DOMINANT;
        } else if (mode_str == "additive") {
          state.raypath_color_mode = LUMICE_COLOR_MODE_ADDITIVE;
        } else {
          // "painter" (default) + unknown fallback — matches core.
          state.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;
        }
        if (classes_arr != nullptr) {
          for (size_t ci = 0; ci < classes_arr->size(); ci++) {
            const auto& jc = (*classes_arr)[ci];
            if (!jc.is_object()) {
              continue;
            }
            ColorClassConfig cls;
            if (jc.contains("color") && jc["color"].is_array() && jc["color"].size() == 3) {
              for (int k = 0; k < 3; k++) {
                cls.color[k] = jc["color"][k].get<float>();
              }
            }
            if (jc.contains("combine") && jc["combine"].is_string() && jc["combine"].get<std::string>() == "all") {
              cls.combine = LUMICE_COLOR_COMBINE_ALL;
            } else {
              cls.combine = LUMICE_COLOR_COMBINE_ANY;
            }
            cls.visible = jc.value("visible", true);
            cls.solo = jc.value("solo", false);
            cls.z_order = static_cast<int>(ci);
            if (jc.contains("match") && jc["match"].is_array()) {
              for (const auto& jr : jc["match"]) {
                if (!jr.is_object()) {
                  continue;
                }
                ColorClassRefConfig ref;
                ref.layer_idx = jr.value("layer", 0);
                const int crystal_json = jr.value("crystal", -1);
                auto it = crystal_id_to_pool.find(crystal_json);
                if (it == crystal_id_to_pool.end()) {
                  GUI_LOG_WARNING(
                      "[FileIO] DeserializeFromJson: raypath_color class {} references crystal id {} not present "
                      "in scattering; ref skipped.",
                      ci, crystal_json);
                  continue;
                }
                ref.crystal_pool_id = it->second;
                // task-356.3 — decode per-ref symmetry ("PBD" subset; absent
                // string means kSymNone). Mirrors the filter/complex-filter
                // decode blocks above (file_io.cpp:1704-1707).
                {
                  const auto sym = jr.value("symmetry", std::string{});
                  ref.sym_p = sym.find('P') != std::string::npos;
                  ref.sym_b = sym.find('B') != std::string::npos;
                  ref.sym_d = sym.find('D') != std::string::npos;
                }
                if (!jr.contains("type")) {
                  ref.match_all = true;
                } else {
                  const auto type_str = jr["type"].get<std::string>();
                  if (type_str == "raypath") {
                    std::string text;
                    if (jr.contains("raypath") && jr["raypath"].is_array()) {
                      for (size_t k = 0; k < jr["raypath"].size(); k++) {
                        if (k > 0) {
                          text += kRaypathSepStr;
                        }
                        text += std::to_string(jr["raypath"][k].get<int>());
                      }
                    }
                    if (text.empty()) {
                      // Empty raypath == whole-crystal (mirrors FillColorPredicate).
                      ref.match_all = true;
                    } else {
                      ref.match_all = false;
                      ref.predicate_text = text;
                    }
                  } else if (type_str == "entry_exit") {
                    // Reconstruct a GUI EntryExitParams and format back to canonical text so
                    // ParseSummandText re-parses to the same Factor on the return trip. Reuses
                    // the shared DecodeEE* helpers so this branch reads identically to the
                    // simple-filter EE decode branch above (no second source of truth).
                    EntryExitParams p;
                    p.entry_text = DecodeEEFaceFromJson(jr, "entry");
                    p.exit_text = DecodeEEFaceFromJson(jr, "exit");
                    DecodeEELengthFromJson(jr, p.length_mode, p.min_len, p.max_len);
                    ref.match_all = false;
                    ref.predicate_text = FormatEntryExitFactorText(p);
                  } else {
                    // Unsupported types (crystal / direction / complex / none) are outside
                    // the GUI editor's expressible set (plan §2 Out of scope); skip loudly
                    // rather than degrade to match-all (which would silently WIDEN the class
                    // to the whole crystal).
                    GUI_LOG_WARNING(
                        "[FileIO] DeserializeFromJson: raypath_color class {} ref type '{}' unsupported in GUI; "
                        "ref skipped.",
                        ci, type_str);
                    continue;
                  }
                }
                cls.match.push_back(ref);
              }
            }
            state.raypath_color.push_back(cls);
          }
        }
      }
    }
  }

  // Render — GUI uses single renderer; ignore additional array entries and malformed keys.
  // If root["render"] is missing, not an array, or empty, keep state.renderer at defaults.
  if (root.contains("render") && root["render"].is_array() && !root["render"].empty()) {
    const auto& jr = root["render"][0];
    RenderConfig r;

    if (jr.contains("lens")) {
      r.lens_type = LensTypeFromString(jr["lens"].value("type", "linear"));
      r.fov = jr["lens"].value("fov", 90.0f);
    }

    if (jr.contains("resolution") && jr["resolution"].is_array() && jr["resolution"].size() == 2) {
      int h = jr["resolution"][1].get<int>();
      for (int i = 0; i < kSimResolutionCount; i++) {
        if (kSimResolutions[i] >= h) {
          r.sim_resolution_index = i;
          break;
        }
      }
    }

    if (jr.contains("view")) {
      r.elevation = jr["view"].value("elevation", 0.0f);
      r.azimuth = jr["view"].value("azimuth", 0.0f);
      r.roll = jr["view"].value("roll", 0.0f);
    }

    std::string vis_str = jr.value("visible", "upper");
    if (vis_str == "front") {
      r.visible = kVisibleFull;
      r.front = true;
    } else {
      r.visible = VisibleFromString(vis_str);
      r.front = jr.value("front", false);
    }

    if (jr.contains("background") && jr["background"].is_array() && jr["background"].size() == 3) {
      for (int i = 0; i < 3; i++)
        r.background[i] = jr["background"][i].get<float>();
    }
    if (jr.contains("ray_color") && jr["ray_color"].is_array() && jr["ray_color"].size() == 3) {
      for (int i = 0; i < 3; i++)
        r.ray_color[i] = jr["ray_color"][i].get<float>();
    }
    r.opacity = jr.value("opacity", 1.0f);
    float ifactor = jr.value("intensity_factor", 1.0f);
    r.exposure_offset = std::log2(std::max(ifactor, 1e-6f));

    state.renderer = r;
  } else {
    GUI_LOG_WARNING("[GUI] DeserializeFromJson: no render entries; using default renderer");
  }

  return true;
}


// ========== Full GuiState JSON Serialization (for .lmc file) ==========

std::string SerializeGuiStateJson(const GuiState& state) {
  json root;

  // Layers — on-disk format remains v2 inline: each entry embeds its crystal
  // and (optionally) filter JSON. The runtime ID-pool is decompressed back to
  // inline at save time, so .lmc files saved before and after this migration
  // are byte-equivalent.
  root["layers"] = json::array();
  int ser_crystal_id = 1;
  int ser_filter_id = 1;
  for (auto& layer : state.layers) {
    json jl;
    jl["prob"] = layer.probability;
    jl["entries"] = json::array();
    for (auto& entry : layer.entries) {
      json je;
      je["crystal"] = SerializeCrystal(state.crystals[entry.crystal_id], ser_crystal_id++);
      je["proportion"] = entry.proportion;
      if (entry.filter_id.has_value()) {
        je["filter"] = SerializeFilterForGui(state.filters[*entry.filter_id], ser_filter_id++);
      }
      jl["entries"].push_back(je);
    }
    root["layers"].push_back(jl);
  }

  // Sun
  json sun;
  sun["altitude"] = state.sun.altitude;
  sun["diameter"] = state.sun.diameter;
  if (state.sun.spectrum_index == kCustomSpectrumIndex && !state.sun.custom_spectrum.empty()) {
    // "custom" sentinel (lowercase, distinct from preset names) + parallel array field.
    sun["spectrum"] = "custom";
    sun["custom_spectrum"] = WlWeightArrayToJson(state.sun.custom_spectrum);
  } else if (state.sun.spectrum_index >= 0 && state.sun.spectrum_index < kSpectrumCount) {
    sun["spectrum"] = kSpectrumNames[state.sun.spectrum_index];
  } else {
    sun["spectrum"] = "D65";
  }
  root["sun"] = sun;

  // Sim
  json sim;
  sim["ray_num_millions"] = state.sim.ray_num_millions;
  sim["max_hits"] = state.sim.max_hits;
  sim["infinite"] = state.sim.infinite;
  root["sim"] = sim;

  // Renderer (copy model: single renderer embedded directly)
  root["renderer"] = SerializeRendererForGui(state.renderer);

  // Aspect ratio (view preference)
  auto preset_idx = static_cast<int>(state.aspect_preset);
  root["aspect_ratio"] = kAspectPresetJsonNames[preset_idx];
  root["aspect_portrait"] = state.aspect_portrait;

  // Background image overlay
  root["bg_path"] = PathToU8(state.bg_path);
  root["bg_show"] = state.bg_show;
  root["bg_alpha"] = state.bg_alpha;

  // Auxiliary line overlay (line / label visibility split since task-overlay-line-label-toggle).
  // Old key `overlay_<x>` (single visibility) is no longer written; readers fall back to it
  // when the new keys are absent.
  root["overlay_horizon_line"] = state.show_horizon_line;
  root["overlay_horizon_label"] = state.show_horizon_label;
  root["overlay_grid_line"] = state.show_grid_line;
  root["overlay_grid_label"] = state.show_grid_label;
  root["overlay_sun_circles_line"] = state.show_sun_circles_line;
  root["overlay_sun_circles_label"] = state.show_sun_circles_label;
  root["overlay_sun_circle_angles"] = state.sun_circle_angles;
  root["overlay_horizon_color"] = { state.horizon_color[0], state.horizon_color[1], state.horizon_color[2] };
  root["overlay_grid_color"] = { state.grid_color[0], state.grid_color[1], state.grid_color[2] };
  root["overlay_sun_circles_color"] = { state.sun_circles_color[0], state.sun_circles_color[1],
                                        state.sun_circles_color[2] };
  root["overlay_horizon_alpha"] = state.horizon_alpha;
  root["overlay_grid_alpha"] = state.grid_alpha;
  root["overlay_sun_circles_alpha"] = state.sun_circles_alpha;
  root["overlay_zenith_nadir_line"] = state.show_zenith_nadir_line;
  root["overlay_zenith_nadir_color"] = { state.zenith_nadir_color[0], state.zenith_nadir_color[1],
                                         state.zenith_nadir_color[2] };
  root["overlay_zenith_nadir_alpha"] = state.zenith_nadir_alpha;
  root["overlay_zenith_nadir_radius_px"] = state.zenith_nadir_radius_px;

  // Panel state
  root["right_panel_collapsed"] = state.right_panel_collapsed;
  root["modal_layout_vertical"] = state.modal_layout_vertical;

  // Schema version. v=2 added the filter `type` discriminator; v=3
  // (task-serialization-bidirectional) replaces the per-filter degenerate form
  // with the full sum-of-products "summands" array. Loader is tolerant of older
  // forms (v=1/v=2 filters upgrade to a SoP), so this is a soft signal, not a
  // load gate — the binary kLmcVersion header is the actual gate (see §2.7).
  root["schema_version"] = 3;

  return root.dump(2);
}


// ========== Full GuiState JSON Deserialization ==========

bool DeserializeGuiStateJson(const std::string& json_str, GuiState& state) {
  json root;
  try {
    root = json::parse(json_str);
  } catch (...) {
    return false;
  }

  state = GuiState{};

  // Layers — on-disk format is v2 inline (each entry embeds its crystal/filter
  // JSON). Load path: append each inline crystal/filter into the runtime
  // ID-pool and record entry.crystal_id / entry.filter_id. Append-only, no
  // dedup (simpler — pool sparsity is acceptable in a single session).
  if (root.contains("layers") && root["layers"].is_array()) {
    for (auto& jl : root["layers"]) {
      Layer layer;
      layer.probability = jl.value("prob", 0.0f);
      if (jl.contains("entries") && jl["entries"].is_array()) {
        for (auto& je : jl["entries"]) {
          EntryCard entry;
          if (je.contains("crystal")) {
            entry.crystal_id = static_cast<int>(state.crystals.size());
            state.crystals.push_back(ParseCrystal(je["crystal"]));
          } else {
            // No inline crystal — provide a default slot so entry stays valid.
            entry.crystal_id = static_cast<int>(state.crystals.size());
            state.crystals.emplace_back();
          }
          entry.proportion = je.value("proportion", 100.0f);
          if (je.contains("filter") && !je["filter"].is_null()) {
            entry.filter_id = static_cast<int>(state.filters.size());
            state.filters.push_back(ParseFilterFromGuiJson(je["filter"]));
          }
          layer.entries.push_back(entry);
        }
      }
      state.layers.push_back(layer);
    }
  } else if (root.contains("crystals") && root.contains("scattering")) {
    // Legacy v1 .lmc format: pool-shaped already (top-level crystals/filters
    // arrays, scattering entries reference by id). Dedupe by source id so
    // entries that share crystal_id collapse to the same pool slot.
    std::map<int, CrystalConfig> crystal_map;
    if (root["crystals"].is_array()) {
      for (auto& jc : root["crystals"]) {
        int id = jc.value("id", 0);
        crystal_map[id] = ParseCrystal(jc);
      }
    }
    std::map<int, FilterConfig> filter_map;
    if (root.contains("filters") && root["filters"].is_array()) {
      for (auto& jf : root["filters"]) {
        int id = jf.value("id", 0);
        filter_map[id] = ParseFilterFromGuiJson(jf);
      }
    }
    if (root["scattering"].is_array()) {
      std::map<int, int> crystal_id_to_pool;
      std::map<int, int> filter_id_to_pool;
      for (auto& jl : root["scattering"]) {
        Layer layer;
        layer.probability = jl.value("prob", 0.0f);
        if (jl.contains("entries") && jl["entries"].is_array()) {
          for (auto& je : jl["entries"]) {
            EntryCard entry;
            int crystal_id = je.value("crystal_id", -1);
            if (crystal_map.count(crystal_id)) {
              auto it = crystal_id_to_pool.find(crystal_id);
              if (it == crystal_id_to_pool.end()) {
                entry.crystal_id = static_cast<int>(state.crystals.size());
                state.crystals.push_back(crystal_map[crystal_id]);
                crystal_id_to_pool.emplace(crystal_id, entry.crystal_id);
              } else {
                entry.crystal_id = it->second;
              }
            } else {
              entry.crystal_id = static_cast<int>(state.crystals.size());
              state.crystals.emplace_back();
            }
            entry.proportion = je.value("proportion", 100.0f);
            int filter_id = je.value("filter_id", -1);
            if (filter_id >= 0 && filter_map.count(filter_id)) {
              auto it = filter_id_to_pool.find(filter_id);
              if (it == filter_id_to_pool.end()) {
                int pool_id = static_cast<int>(state.filters.size());
                state.filters.push_back(filter_map[filter_id]);
                filter_id_to_pool.emplace(filter_id, pool_id);
                entry.filter_id = pool_id;
              } else {
                entry.filter_id = it->second;
              }
            }
            layer.entries.push_back(entry);
          }
        }
        state.layers.push_back(layer);
      }
    }
  }

  // Sun
  if (root.contains("sun")) {
    auto& js = root["sun"];
    state.sun.altitude = js.value("altitude", 20.0f);
    state.sun.diameter = js.value("diameter", 0.5f);
    const auto spec_str = js.value("spectrum", std::string("D65"));
    if (spec_str == "custom" && js.contains("custom_spectrum") && js["custom_spectrum"].is_array()) {
      state.sun.custom_spectrum = ParseWlWeightArray(js["custom_spectrum"]);
      state.sun.spectrum_index = state.sun.custom_spectrum.empty() ? 2 : kCustomSpectrumIndex;
    } else {
      state.sun.spectrum_index = SpectrumFromString(spec_str);
      state.sun.custom_spectrum.clear();
    }
  }

  // Sim
  if (root.contains("sim")) {
    auto& js = root["sim"];
    state.sim.ray_num_millions = js.value("ray_num_millions", SimConfig{}.ray_num_millions);
    state.sim.max_hits = js.value("max_hits", SimConfig{}.max_hits);
    state.sim.infinite = js.value("infinite", SimConfig{}.infinite);
  }

  // Renderer (copy model).
  // New format: root["renderer"] is a single object.
  // Legacy format: root["renderers"] is an array; take first element; ignore
  // selected_renderer_id/next_renderer_id (no longer meaningful).
  if (root.contains("renderer") && root["renderer"].is_object()) {
    state.renderer = ParseRendererFromGuiJson(root["renderer"]);
  } else if (root.contains("renderers") && root["renderers"].is_array() && !root["renderers"].empty()) {
    state.renderer = ParseRendererFromGuiJson(root["renderers"][0]);
  } else {
    // Neither new-format "renderer" object nor legacy "renderers" array with data found.
    // Symmetric with DeserializeFromJson's empty-render-array branch, so that malformed or
    // legacy files leave an observable trace for debugging.
    GUI_LOG_WARNING("[GUI] DeserializeGuiStateJson: no renderer key found; using default renderer");
  }

  // Aspect ratio (view preference, defaults to Free for old files)
  state.aspect_preset = AspectPresetFromString(root.value("aspect_ratio", "free"));
  state.aspect_portrait = root.value("aspect_portrait", false);

  // Background image overlay (backward compatible: missing fields use defaults)
  state.bg_path = PathFromU8(root.value("bg_path", std::string{}));
  state.bg_show = root.value("bg_show", false);
  state.bg_alpha = root.value("bg_alpha", 1.0f);

  // Auxiliary line overlay (backward compatible: legacy `overlay_<x>` key maps to
  // both line and label = legacy_value; new keys override per-axis when present).
  bool legacy_horizon = root.value("overlay_horizon", false);
  bool legacy_grid = root.value("overlay_grid", false);
  bool legacy_sun_circles = root.value("overlay_sun_circles", false);
  state.show_horizon_line = root.value("overlay_horizon_line", legacy_horizon);
  state.show_horizon_label = root.value("overlay_horizon_label", legacy_horizon);
  state.show_grid_line = root.value("overlay_grid_line", legacy_grid);
  state.show_grid_label = root.value("overlay_grid_label", legacy_grid);
  state.show_sun_circles_line = root.value("overlay_sun_circles_line", legacy_sun_circles);
  state.show_sun_circles_label = root.value("overlay_sun_circles_label", legacy_sun_circles);
  if (root.contains("overlay_sun_circle_angles") && root["overlay_sun_circle_angles"].is_array()) {
    state.sun_circle_angles.clear();
    for (const auto& v : root["overlay_sun_circle_angles"]) {
      if (v.is_number() && static_cast<int>(state.sun_circle_angles.size()) < kMaxSunCircles) {
        float angle = std::clamp(v.get<float>(), 0.1f, 180.0f);
        state.sun_circle_angles.push_back(angle);
      }
    }
    std::sort(state.sun_circle_angles.begin(), state.sun_circle_angles.end());
  }
  auto read_color3 = [&root](const char* key, float* out) {
    if (root.contains(key) && root[key].is_array() && root[key].size() == 3) {
      for (int i = 0; i < 3; i++) {
        out[i] = root[key][i].get<float>();
      }
    }
  };
  read_color3("overlay_horizon_color", state.horizon_color);
  read_color3("overlay_grid_color", state.grid_color);
  read_color3("overlay_sun_circles_color", state.sun_circles_color);
  state.horizon_alpha = root.value("overlay_horizon_alpha", 0.6f);
  state.grid_alpha = root.value("overlay_grid_alpha", 0.3f);
  state.sun_circles_alpha = root.value("overlay_sun_circles_alpha", 0.5f);
  state.show_zenith_nadir_line = root.value("overlay_zenith_nadir_line", false);
  read_color3("overlay_zenith_nadir_color", state.zenith_nadir_color);
  state.zenith_nadir_alpha = root.value("overlay_zenith_nadir_alpha", 0.6f);
  state.zenith_nadir_radius_px = root.value("overlay_zenith_nadir_radius_px", 8.0f);

  // Panel state
  state.right_panel_collapsed = root.value("right_panel_collapsed", false);
  state.modal_layout_vertical = root.value("modal_layout_vertical", true);

  return true;
}


// ========== .lmc Binary File I/O ==========

// Header: 44 bytes, little-endian
// magic[4] = "LMC\0"
// version: uint32 = 1
// flags: uint32 (bit 0: has_texture)
// json_offset: uint64
// json_size: uint64
// tex_offset: uint64
// tex_size: uint64

static constexpr uint32_t kLmcMagic = 0x00434D4C;  // "LMC\0" as little-endian uint32
// v=1 → v=2 bump introduced the FilterConfig variant + per-filter `type` field.
// v=2 → v=3 bump (task-serialization-bidirectional) replaces the per-filter
// degenerate form with the sum-of-products "summands" array. v=1/v=2 files are
// still loadable (their filters upgrade to a SoP via FromLegacyRaypath /
// FromLegacyEntryExit); newer files are rejected by older binaries (they reject
// any version > kLmcVersion). The bump is mandatory: without it an old binary
// (gate=2) would pass a v=3 file's header and then silently load empty filters
// (the summands array carries no legacy `type`/`raypath_text`). .lmc is a
// GUI-internal git-ignored format with no external release contract.
static constexpr uint32_t kLmcVersion = 3;
static constexpr uint32_t kLmcHeaderSize = 44;
static constexpr uint32_t kLmcFlagHasTexture = 0x1;

static void WriteU32(std::ofstream& out, uint32_t val) {
  out.write(reinterpret_cast<const char*>(&val), sizeof(val));
}

static void WriteU64(std::ofstream& out, uint64_t val) {
  out.write(reinterpret_cast<const char*>(&val), sizeof(val));
}

static bool ReadU32(std::ifstream& in, uint32_t& val) {
  return static_cast<bool>(in.read(reinterpret_cast<char*>(&val), sizeof(val)));
}

static bool ReadU64(std::ifstream& in, uint64_t& val) {
  return static_cast<bool>(in.read(reinterpret_cast<char*>(&val), sizeof(val)));
}

// stb_image_write callback: appends to std::vector<unsigned char>
static void StbWriteCallback(void* context, void* data, int size) {
  auto* buf = static_cast<std::vector<unsigned char>*>(context);
  auto* bytes = static_cast<unsigned char*>(data);
  buf->insert(buf->end(), bytes, bytes + size);
}

bool SaveLmcFile(const std::filesystem::path& path, const GuiState& state, const PreviewRenderer& preview,
                 bool save_texture) {
  std::string json_payload = SerializeGuiStateJson(state);

  // Encode texture to PNG in memory if requested
  std::vector<unsigned char> png_data;
  bool has_texture = false;
  if (save_texture && preview.HasTexture() && preview.GetTextureData() != nullptr) {
    int w = preview.GetTextureWidth();
    int h = preview.GetTextureHeight();
    int result = stbi_write_png_to_func(StbWriteCallback, &png_data, w, h, 3, preview.GetTextureData(), w * 3);
    if (result != 0) {
      has_texture = true;
    }
  }

  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return false;
  }

  // Compute offsets
  uint64_t json_offset = kLmcHeaderSize;
  auto json_size = static_cast<uint64_t>(json_payload.size());
  uint64_t tex_offset = has_texture ? json_offset + json_size : 0;
  auto tex_size = static_cast<uint64_t>(png_data.size());

  // Write header
  uint32_t flags = has_texture ? kLmcFlagHasTexture : 0;
  WriteU32(out, kLmcMagic);
  WriteU32(out, kLmcVersion);
  WriteU32(out, flags);
  WriteU64(out, json_offset);
  WriteU64(out, json_size);
  WriteU64(out, tex_offset);
  WriteU64(out, tex_size);

  // Write JSON payload
  out.write(json_payload.data(), static_cast<std::streamsize>(json_payload.size()));

  // Write texture PNG
  if (has_texture) {
    out.write(reinterpret_cast<const char*>(png_data.data()), static_cast<std::streamsize>(png_data.size()));
  }

  return out.good();
}

bool LoadLmcFile(const std::filesystem::path& path, GuiState& state, std::vector<unsigned char>& tex_data, int& tex_w,
                 int& tex_h) {
  tex_data.clear();
  tex_w = 0;
  tex_h = 0;

  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    GUI_LOG_ERROR("[LMC] Cannot open file: {}", PathToU8(path));
    return false;
  }

  // Read header
  uint32_t magic = 0;
  uint32_t version = 0;
  uint32_t flags = 0;
  uint64_t json_offset = 0;
  uint64_t json_size = 0;
  uint64_t tex_offset = 0;
  uint64_t tex_size = 0;

  if (!ReadU32(in, magic) || !ReadU32(in, version) || !ReadU32(in, flags) || !ReadU64(in, json_offset) ||
      !ReadU64(in, json_size) || !ReadU64(in, tex_offset) || !ReadU64(in, tex_size)) {
    GUI_LOG_ERROR("[LMC] Failed to read header");
    return false;
  }

  if (magic != kLmcMagic) {
    GUI_LOG_ERROR("[LMC] Invalid magic: 0x{:08x}", magic);
    return false;
  }

  if (version > kLmcVersion) {
    GUI_LOG_ERROR("[LMC] File version {} is newer than supported version {}", version, kLmcVersion);
    return false;
  }
  if (version < 1) {
    GUI_LOG_ERROR("[LMC] Invalid version: {}", version);
    return false;
  }
  // v=1 files are read via the same DeserializeGuiStateJson path: the parser is
  // tolerant of missing `type` field (defaults to "raypath") and missing
  // `schema_version` (treated as v=1 fallback). No separate code path needed.

  // Read JSON
  std::string json_payload(json_size, '\0');
  in.seekg(static_cast<std::streamoff>(json_offset));
  in.read(json_payload.data(), static_cast<std::streamsize>(json_size));
  if (!in) {
    GUI_LOG_ERROR("[LMC] Failed to read JSON section");
    return false;
  }

  if (!DeserializeGuiStateJson(json_payload, state)) {
    GUI_LOG_ERROR("[LMC] Failed to parse JSON");
    return false;
  }

  // Read texture if present
  bool flag_has_tex = (flags & kLmcFlagHasTexture) != 0;
  if (flag_has_tex) {
    if (tex_size == 0) {
      GUI_LOG_ERROR("[LMC] Texture flag set but size is 0");
      return false;
    }
    std::vector<unsigned char> png_buf(tex_size);
    in.seekg(static_cast<std::streamoff>(tex_offset));
    in.read(reinterpret_cast<char*>(png_buf.data()), static_cast<std::streamsize>(tex_size));
    if (!in) {
      GUI_LOG_ERROR("[LMC] Failed to read texture section");
      return false;
    }

    int channels = 0;
    unsigned char* decoded =
        stbi_load_from_memory(png_buf.data(), static_cast<int>(png_buf.size()), &tex_w, &tex_h, &channels, 3);
    if (!decoded) {
      GUI_LOG_ERROR("[LMC] Failed to decode texture PNG");
      return false;
    }
    size_t byte_count = static_cast<size_t>(tex_w) * tex_h * 3;
    tex_data.assign(decoded, decoded + byte_count);
    stbi_image_free(decoded);
  }

  return true;
}


// ========== Export Preview ==========

// Shared PNG writer: takes an RGBA8 top-down buffer and writes it as a PNG file.
// Centralizes stbi_write_png so callers (this module, app.cpp DoExportPreviewPng)
// share one error-handling convention.
[[nodiscard]] bool WriteRgbaBufferToPng(const std::filesystem::path& path, int w, int h,
                                        const std::vector<unsigned char>& rgba) {
  if (path.empty() || w <= 0 || h <= 0 || rgba.size() != static_cast<size_t>(w) * static_cast<size_t>(h) * 4) {
    return false;
  }
  auto u8path = path.u8string();
  return stbi_write_png(u8path.c_str(), w, h, 4, rgba.data(), w * 4) != 0;
}

// Thin wrapper over RenderExportToRgba: kept for binary-compatible callers in
// test/gui/ (test_gui_export, test_gui_visual, test_gui_bg). Consolidated with
// the overlay path in DoExportPreviewPng — the FBO+renderer logic lives once, in
// export_fbo_renderer.cpp.
bool ExportPreviewPng(const std::filesystem::path& path, PreviewRenderer& renderer, const PreviewViewport& vp) {
  if (vp.vp_w <= 0 || vp.vp_h <= 0 || !renderer.HasTexture()) {
    return false;
  }
  auto rgba = RenderExportToRgba(renderer, vp.params, vp.vp_w, vp.vp_h, std::nullopt);
  if (rgba.empty()) {
    return false;
  }
  return WriteRgbaBufferToPng(path, vp.vp_w, vp.vp_h, rgba);
}


// ========== Export Config JSON ==========

bool ExportConfigJson(const std::filesystem::path& path, const std::string& json_str) {
  if (path.empty()) {
    return false;
  }
  std::ofstream out(path);
  if (!out.is_open()) {
    return false;
  }
  out << json_str;
  return out.good();
}

// ========== File Dialogs ==========

std::filesystem::path ShowOpenDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_items[2] = { { "Lumice", "lmc" }, { "JSON Config", "json" } };
  nfdresult_t result = NFD_OpenDialog(&out_path, filter_items, 2, nullptr);
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowSaveDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "Lumice", "lmc" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "project.lmc");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportPngDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "PNG Image", "png" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "preview.png");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportDualFisheyeEqualAreaDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "PNG Image", "png" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "dual_fisheye_equal_area.png");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportEquirectangularDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "PNG Image", "png" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "equirectangular.png");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowExportJsonDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_item[1] = { { "JSON Config", "json" } };
  nfdresult_t result = NFD_SaveDialog(&out_path, filter_item, 1, nullptr, "config.json");
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

std::filesystem::path ShowOpenImageDialog() {
  NFD_Init();
  nfdchar_t* out_path = nullptr;
  nfdfilteritem_t filter_items[1] = { { "Images", "png,jpg,jpeg,bmp" } };
  nfdresult_t result = NFD_OpenDialog(&out_path, filter_items, 1, nullptr);
  std::filesystem::path path;
  if (result == NFD_OKAY && out_path) {
    path = PathFromU8(out_path);
    NFD_FreePath(out_path);
  }
  NFD_Quit();
  return path;
}

}  // namespace lumice::gui
