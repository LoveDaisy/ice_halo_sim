#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "config/crystal_config.hpp"        // ns::PrismCrystalParam / PyramidCrystalParam (LUMICE_GetCrystalMesh)
#include "config/raypath_color_config.hpp"  // ns::kDefaultCompositeMode (single-source default)
#include "config/raypath_validation.hpp"
#include "config/render_config.hpp"
#include "core/annotation_overlay.hpp"  // annotation::ComputeOverlay (LUMICE_ComputeAnnotationOverlay)
#include "core/crystal.hpp"
#include "core/ev_anchor.hpp"
#include "core/geo3d.hpp"
#include "core/trace_ops.hpp"  // ns::MakeCrystal (core single-source crystal sampler)
#if defined(__APPLE__)
#include "core/backend/metal_trace_backend.hpp"
#endif
#if defined(LUMICE_CUDA_ENABLED)
#include "core/backend/cuda_trace_backend.hpp"  // CudaDeviceAvailable() for LUMICE_IsBackendAvailable
#endif
#include "include/lumice.h"
#include "server/c_api_internal.hpp"
#include "server/server.hpp"
#include "util/callback_sink.hpp"
#include "util/color_space.hpp"
#include "util/logger.hpp"
#include "util/path_utils.hpp"

namespace ns = lumice;

// Convert Miller index (i1, i4) to wedge angle in degrees. Returns 28.0 (default) if i1 == 0.
static float MillerToAlpha(int i1, int i4) {
  constexpr float kSqrt3_2 = 0.866025403784f;
  constexpr float kIceCrystalC = 1.629f;
  constexpr float kRadToDeg = 57.2957795131f;
  if (i1 == 0)
    return 28.0f;
  return std::atan(kSqrt3_2 * i4 / i1 / kIceCrystalC) * kRadToDeg;
}

// =============== Internal Helpers ===============
struct LUMICE_Server_ {
  std::unique_ptr<ns::Server> server_;
};


static LUMICE_ErrorCode MapErrorCode(ns::ErrorCode code) {
  switch (code) {
    case ns::ErrorCode::kSuccess:
      return LUMICE_OK;
    case ns::ErrorCode::kInvalidJson:
      return LUMICE_ERR_INVALID_JSON;
    case ns::ErrorCode::kInvalidConfig:
      return LUMICE_ERR_INVALID_CONFIG;
    case ns::ErrorCode::kMissingField:
      return LUMICE_ERR_MISSING_FIELD;
    case ns::ErrorCode::kInvalidValue:
      return LUMICE_ERR_INVALID_VALUE;
    case ns::ErrorCode::kServerNotReady:
    case ns::ErrorCode::kServerError:
    default:
      return LUMICE_ERR_SERVER;
  }
}


// =============== Server Lifecycle ===============
LUMICE_Server* LUMICE_CreateServer() {
  auto* s = new LUMICE_Server;
  s->server_ = std::make_unique<ns::Server>();
  return s;
}


LUMICE_Server* LUMICE_CreateServerEx(const LUMICE_ServerConfig* config) {
  auto* s = new LUMICE_Server;
  int num_workers = (config != nullptr) ? config->num_workers : 0;
  uint32_t sim_seed = (config != nullptr) ? config->sim_seed : 0;
  // C-API boundary: the public ABI stays `int`; internally we use the
  // type-safe BackendKind enum. This static_cast is the single conversion
  // point — other internal code must never cast int↔BackendKind.
  int raw_backend = (config != nullptr) ? config->preferred_backend : 0;
  auto preferred_backend = static_cast<ns::BackendKind>(raw_backend);
  s->server_ = std::make_unique<ns::Server>(num_workers, sim_seed, preferred_backend);
  return s;
}


void LUMICE_DestroyServer(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  server->server_->Terminate();
  delete server;
}


// =============== Logging ===============
void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level) {
  if (!server) {
    return;
  }
  static constexpr ns::LogLevel kLevelMap[] = {
    ns::LogLevel::kTrace,    // LUMICE_LOG_TRACE
    ns::LogLevel::kDebug,    // LUMICE_LOG_DEBUG
    ns::LogLevel::kVerbose,  // LUMICE_LOG_VERBOSE
    ns::LogLevel::kInfo,     // LUMICE_LOG_INFO
    ns::LogLevel::kWarning,  // LUMICE_LOG_WARNING
    ns::LogLevel::kError,    // LUMICE_LOG_ERROR
    ns::LogLevel::kOff,      // LUMICE_LOG_OFF
  };
  if (level >= LUMICE_LOG_TRACE && level <= LUMICE_LOG_OFF) {
    auto mapped = kLevelMap[level];
    server->server_->SetLogLevel(mapped);
    ns::GetGlobalLogger().SetLevel(mapped);
  }
}


void LUMICE_SetLogCallback(LUMICE_LogCallback callback) {
  auto& sink = ns::GetCallbackSink();
  sink->SetCallback(callback);

  // Add callback sink to shared dist_sink on first call (idempotent check via static flag).
  // Set our custom formatter since sinks added after Logger::set_formatter don't inherit it.
  static bool registered = false;
  if (!registered) {
    sink->set_formatter(ns::CreateLumiceFormatter(ns::kLogPattern));
    ns::GetSharedSink()->add_sink(sink);
    registered = true;
  }
}


// =============== ConfigScratch <-> JSON (internal; see c_api_internal.hpp) ===============
// Serialize a LUMICE_Distribution to its core-compatible JSON wire form. NO_RANDOM emits a
// bare number (center), matching core Distribution::to_json's short-circuit branch
// (math.cpp: `obj = dist.center`); the other 5 types emit {"type","mean","std"} objects. The
// on-disk keys stay "mean"/"std" (core's published config format) even though the C struct
// field names are center/spread.
static nlohmann::json DistributionToJson(const LUMICE_Distribution& d) {
  if (d.type == LUMICE_DIST_NO_RANDOM) {
    return d.center;  // bare number; aligns with core Distribution::to_json for kNoRandom
  }
  nlohmann::json j;
  switch (d.type) {
    case LUMICE_DIST_UNIFORM:
      j["type"] = "uniform";
      break;
    case LUMICE_DIST_GAUSS:
      j["type"] = "gauss";
      break;
    case LUMICE_DIST_ZIGZAG:
      j["type"] = "zigzag";
      break;
    case LUMICE_DIST_LAPLACIAN:
      j["type"] = "laplacian";
      break;
    case LUMICE_DIST_GAUSS_LEGACY:
      j["type"] = "gauss_legacy";
      break;
    default:
      LOG_ERROR("Unknown LUMICE_Distribution.type: {}", d.type);
      j["type"] = "gauss";
      break;
  }
  j["mean"] = d.center;
  j["std"] = d.spread;
  return j;
}

// Encode a P/B/D symmetry bitmask (1=P, 2=B, 4=D) as its canonical string form. Single home
// for the wire encoding shared between LUMICE_FilterParam and LUMICE_ColorPredicate — callers
// each own their own emit-condition policy (FilterParam emits unconditionally; ColorPredicate
// emits only when non-default), the helper itself is oblivious to that decision.
static std::string SymmetryBitsToString(int bits) {
  std::string sym;
  if (bits & 1) {
    sym += "P";
  }
  if (bits & 2) {
    sym += "B";
  }
  if (bits & 4) {
    sym += "D";
  }
  return sym;
}

// Decode a P/B/D symmetry string back into its bitmask. Unknown characters are ignored (same
// tolerance as the previous inline loops); paired with SymmetryBitsToString above.
static int SymmetryStringToBits(const std::string& s) {
  int bits = 0;
  for (char ch : s) {
    if (ch == 'P') {
      bits |= 1;
    } else if (ch == 'B') {
      bits |= 2;
    } else if (ch == 'D') {
      bits |= 4;
    }
  }
  return bits;
}

// Emit the arm fields of a Color Predicate (task-342.2, symmetry added task-356.2). Mirrors
// the raypath / entry_exit / direction / crystal switch in JsonToFilter/ConfigToJson but
// WITHOUT id/action/composition — Design 2 color predicates are single-atom and carry no
// filter identity. A predicate whose type is LUMICE_FILTER_TYPE_UNSET intentionally emits NO
// arm fields at all: the resulting ref JSON is just {"layer", "crystal"} (plus optional
// "symmetry" if non-default), which core RaypathColorRef::from_json interprets as match-all
// (whole-crystal color). See lumice.h LUMICE_ColorPredicate for the UNSET-as-match-all
// rationale.
static void ColorPredicateToJson(const LUMICE_ColorPredicate& p, nlohmann::json& j) {
  switch (p.type) {
    case LUMICE_FILTER_TYPE_UNSET:
      // Match-all: emit no arm fields (wire form for NoneFilterParam default).
      break;
    case LUMICE_FILTER_TYPE_NONE:
      j["type"] = "none";
      break;
    case LUMICE_FILTER_TYPE_RAYPATH: {
      j["type"] = "raypath";
      nlohmann::json rp = nlohmann::json::array();
      for (int k = 0; k < p.raypath_count; k++) {
        rp.push_back(p.raypath[k]);
      }
      j["raypath"] = rp;
      break;
    }
    case LUMICE_FILTER_TYPE_ENTRY_EXIT:
      j["type"] = "entry_exit";
      if (p.ee_entry >= 0) {
        j["entry"] = p.ee_entry;
      }
      if (p.ee_exit >= 0) {
        j["exit"] = p.ee_exit;
      }
      if (p.ee_min_len > 1) {
        j["min_len"] = p.ee_min_len;
      }
      if (p.ee_max_len >= 0) {
        j["max_len"] = p.ee_max_len;
      }
      break;
    case LUMICE_FILTER_TYPE_DIRECTION:
      j["type"] = "direction";
      j["az"] = p.dir_az;
      j["el"] = p.dir_el;
      j["radii"] = p.dir_radii;
      break;
    case LUMICE_FILTER_TYPE_CRYSTAL:
      j["type"] = "crystal";
      j["crystal_id"] = p.crystal_id;
      break;
    default:
      // COMPLEX and any out-of-range discriminant is unsupported for color predicates.
      throw std::invalid_argument("LUMICE_ColorPredicate.type is invalid for a color predicate: " +
                                  std::to_string(p.type));
  }
  // Symmetry is a common field regardless of predicate arm (including match-all). Mirrors
  // core RaypathColorRef::to_json — emitted ONLY when non-default (kSymNone omitted), which
  // is DIFFERENT from LUMICE_FilterParam's "always emit (empty when no bits)" convention.
  // Keeps legacy / no-symmetry wire form byte-identical (AC3).
  if (p.symmetry != 0) {
    j["symmetry"] = SymmetryBitsToString(p.symmetry);
  }
}

// The sync_group schema key names come from core (ns::ShapeScalarSyncKeyName), which owns them.
// This layer used to carry its own copy of the {key, slot} table; the core from_json looks the keys
// up by name, so a single character of drift meant the C API emitted a key core never reads and a
// declared sync group silently degenerated back to "all independent" with no warning anywhere.
// The face slots all answer "face_distance", whose value is one 6-element array, so they are
// handled as a block rather than per slot.

// struct -> JSON. Passes sync_group through verbatim: canonicalization (drop inapplicable slots,
// collapse singletons, renumber) and leader normalization belong to core's from_json, which every
// path that actually consumes this JSON runs exactly once. Re-implementing them here would be a
// second authority for the same semantics.
// Writes nothing at all when no scalar is synced, keeping the wire form of every pre-v4.13 config
// byte-identical. Mirrors WriteSyncGroupJson in src/config/crystal_config.cpp.
static void WriteSyncGroupJson(nlohmann::json& shape_j, const int sync_group[LUMICE_SHAPE_SCALAR_COUNT],
                               ns::CrystalKind kind) {
  nlohmann::json sg = nlohmann::json::object();
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_FACE_0; slot++) {
    // A null key means the slot does not apply to this type, so an inapplicable declaration never
    // reaches the wire in the first place.
    const char* key = ns::ShapeScalarSyncKeyName(kind, slot);
    if (key != nullptr && sync_group[slot] != 0) {
      sg[key] = sync_group[slot];
    }
  }
  bool any_face = false;
  for (int i = 0; i < 6; i++) {
    any_face = any_face || sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] != 0;
  }
  const char* face_key = ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_FACE_0);
  if (any_face && face_key != nullptr) {
    // All six, zeros included — matching how face_distance itself serializes.
    sg[face_key] =
        std::vector<int>(sync_group + LUMICE_SHAPE_SCALAR_FACE_0, sync_group + LUMICE_SHAPE_SCALAR_FACE_0 + 6);
  }
  if (!sg.empty()) {
    shape_j["sync_group"] = sg;
  }
}

// JSON -> struct. Absent "sync_group" leaves every slot 0 (all independent), which is why an
// older config file needs no edit. Mirrors ReadSyncGroupJson in src/config/crystal_config.cpp.
static void ReadSyncGroupJson(const nlohmann::json& shape_j, int sync_group[LUMICE_SHAPE_SCALAR_COUNT],
                              ns::CrystalKind kind) {
  for (int i = 0; i < LUMICE_SHAPE_SCALAR_COUNT; i++) {
    sync_group[i] = 0;
  }
  if (!shape_j.contains("sync_group")) {
    return;
  }
  const auto& sg = shape_j.at("sync_group");
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_FACE_0; slot++) {
    const char* key = ns::ShapeScalarSyncKeyName(kind, slot);
    if (key != nullptr && sg.contains(key)) {
      sync_group[slot] = sg.at(key).get<int>();
    }
  }
  const char* face_key = ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_FACE_0);
  if (face_key != nullptr && sg.contains(face_key)) {
    size_t i = 0;
    for (const auto& elem : sg.at(face_key)) {
      if (i >= 6) {
        break;
      }
      sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] = elem.get<int>();
      i++;
    }
  }
}

// Non-static (declared in server/c_api_internal.hpp) so both ConfigToJson and
// LUMICE_GetCrystalMesh consume the SAME crystal-shape translation table. Returns
// {"type","shape"} only — id/axis are the caller's responsibility (ConfigToJson adds
// them; the mesh-preview path does not want them).
nlohmann::json CrystalShapeToJson(const LUMICE_CrystalParam& cr) {
  nlohmann::json j;
  const auto kind = (cr.type == 0) ? ns::CrystalKind::kPrism : ns::CrystalKind::kPyramid;
  if (cr.type == 0) {
    j["type"] = "prism";
    j["shape"][ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_HEIGHT)] = DistributionToJson(cr.height);
  } else {
    j["type"] = "pyramid";
    j["shape"][ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_PRISM_H)] = DistributionToJson(cr.prism_h);
    j["shape"][ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_UPPER_H)] = DistributionToJson(cr.upper_h);
    j["shape"][ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_LOWER_H)] = DistributionToJson(cr.lower_h);
    j["shape"][ns::ShapeWedgeAngleKeyName(true)] = cr.upper_wedge_angle;
    j["shape"][ns::ShapeWedgeAngleKeyName(false)] = cr.lower_wedge_angle;
  }
  // face_distance: emit all 6 unconditionally (mirrors core crystal_config.cpp::to_json's
  // `j["face_distance"] = p.d_`). The "only when non-default" shortcut no longer fits now that
  // each element is a distribution (a NO_RANDOM 1.0 default vs a randomized 1.0 are different
  // wire forms), so always round-trip every element.
  nlohmann::json fd = nlohmann::json::array();
  for (int fi = 0; fi < 6; fi++) {
    fd.push_back(DistributionToJson(cr.face_distance[fi]));
  }
  j["shape"][ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_FACE_0)] = fd;
  // Only the keys applicable to this crystal type are emitted, so an inapplicable declaration
  // never reaches the wire in the first place (core's canonicalization zeroes such slots anyway,
  // as defense in depth).
  WriteSyncGroupJson(j["shape"], cr.sync_group, kind);
  return j;
}

// Per-item wire encoders — the single source of truth for each subsystem's JSON shape, shared
// by both the batch path (ConfigToJson below) and the incremental Scene path (LUMICE_SceneAdd*).
// Splitting ConfigToJson's formerly-inline loops into these keeps the two paths from drifting;
// the caller owns id assignment (ConfigToJson passes the struct's .id; the Scene passes its own
// sequential id). These throw std::invalid_argument on an invalid item — the batch caller wraps
// the whole ConfigToJson in try/catch, the Scene callers wrap each Add in try/catch.

// Full crystal object: {"type","shape",...} from CrystalShapeToJson plus id + axis distributions.
static nlohmann::json CrystalToJson(const LUMICE_CrystalParam& cr, int id) {
  nlohmann::json j = CrystalShapeToJson(cr);  // {"type","shape"} single-source translation
  j["id"] = id;
  j["axis"][ns::AxisScalarKeyName(ns::kAxisScalarZenith)] = DistributionToJson(cr.zenith);
  j["axis"][ns::AxisScalarKeyName(ns::kAxisScalarAzimuth)] = DistributionToJson(cr.azimuth);
  j["axis"][ns::AxisScalarKeyName(ns::kAxisScalarRoll)] = DistributionToJson(cr.roll);
  return j;
}

// Encode the type-specific arm fields of a SIMPLE filter (none/raypath/entry_exit/direction/
// crystal) into `j`. Does NOT write id/action/symmetry — those are common fields the caller
// writes around this. Throws for COMPLEX (needs composition context — caller handles it) and for
// UNSET/out-of-range (the zero-init guard). Mirrors core config/filter_config.cpp::to_json.
// SYNC: the per-type field list here mirrors the parse side in JsonToFilter; adding/removing a
// filter type or field requires updating both.
static void EncodeSimpleFilterBody(const LUMICE_FilterParam& f, nlohmann::json& j) {
  switch (f.type) {
    case LUMICE_FILTER_TYPE_NONE:
      j["type"] = "none";
      break;
    case LUMICE_FILTER_TYPE_RAYPATH: {
      j["type"] = "raypath";
      nlohmann::json rp = nlohmann::json::array();
      for (int k = 0; k < f.raypath_count; k++) {
        rp.push_back(f.raypath[k]);
      }
      j["raypath"] = rp;
      break;
    }
    case LUMICE_FILTER_TYPE_ENTRY_EXIT:
      j["type"] = "entry_exit";
      if (f.ee_entry >= 0) {
        j["entry"] = f.ee_entry;
      }
      if (f.ee_exit >= 0) {
        j["exit"] = f.ee_exit;
      }
      if (f.ee_min_len > 1) {
        j["min_len"] = f.ee_min_len;
      }
      if (f.ee_max_len >= 0) {
        j["max_len"] = f.ee_max_len;
      }
      break;
    case LUMICE_FILTER_TYPE_DIRECTION:
      j["type"] = "direction";
      j["az"] = f.dir_az;
      j["el"] = f.dir_el;
      j["radii"] = f.dir_radii;
      break;
    case LUMICE_FILTER_TYPE_CRYSTAL:
      j["type"] = "crystal";
      j["crystal_id"] = f.crystal_id;
      break;
    case LUMICE_FILTER_TYPE_COMPLEX:
      // A complex filter cannot be encoded from the LUMICE_FilterParam alone — it needs its
      // composition. ConfigToJson handles COMPLEX before reaching here; LUMICE_SceneAddFilter
      // rejects it (the caller must use LUMICE_SceneAddComplexFilter).
      throw std::invalid_argument("complex filter cannot be encoded as a simple filter: " + std::to_string(f.type));
    default:
      // UNSET (zero-init guard) or an out-of-range discriminant: fail fast rather than silently
      // emitting a wrong/empty filter.
      throw std::invalid_argument("LUMICE_FilterParam.type is unset or invalid: " + std::to_string(f.type));
  }
}

// Encode a complex filter's sum-of-products composition as the wire "composition" array: each
// clause is a bare id (1 term) or an array of ids (multi-term), mirroring core to_json.
static nlohmann::json CompositionArrayToJson(const LUMICE_ComplexComposition& comp) {
  nlohmann::json composition = nlohmann::json::array();
  for (int cl = 0; cl < comp.clause_count; cl++) {
    int term_n = 0;
    const int* terms_p = LUMICE_CompositionClauseTerms(&comp, cl, &term_n);
    if (term_n == 1 && terms_p != nullptr) {
      composition.push_back(terms_p[0]);
    } else {
      nlohmann::json terms = nlohmann::json::array();
      for (int t = 0; t < term_n; t++) {
        terms.push_back(terms_p[t]);
      }
      composition.push_back(terms);
    }
  }
  return composition;
}

// Map LUMICE_LENS_TYPE_* to its core enumerator. Explicit switch (not a numeric cast) so a future
// reorder of either enumeration surfaces as a compile/throw rather than a silently aliased
// projection. Throws std::invalid_argument on an unknown value.
static ns::LensParam::LensType MapLensTypeFromCApi(int lens_type) {
  switch (lens_type) {
    case LUMICE_LENS_TYPE_LINEAR:
      return ns::LensParam::kLinear;
    case LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA:
      return ns::LensParam::kFisheyeEqualArea;
    case LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT:
      return ns::LensParam::kFisheyeEquidistant;
    case LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC:
      return ns::LensParam::kFisheyeStereographic;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA:
      return ns::LensParam::kDualFisheyeEqualArea;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUIDISTANT:
      return ns::LensParam::kDualFisheyeEquidistant;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_STEREOGRAPHIC:
      return ns::LensParam::kDualFisheyeStereographic;
    case LUMICE_LENS_TYPE_RECTANGULAR:
      return ns::LensParam::kRectangular;
    case LUMICE_LENS_TYPE_FISHEYE_ORTHOGRAPHIC:
      return ns::LensParam::kFisheyeOrthographic;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_ORTHOGRAPHIC:
      return ns::LensParam::kDualFisheyeOrthographic;
    case LUMICE_LENS_TYPE_GLOBE:
      return ns::LensParam::kGlobe;
    default:
      throw std::invalid_argument("LUMICE_RenderParam.lens_type is invalid: " + std::to_string(lens_type));
  }
}

// Inverse of MapLensTypeFromCApi. Total over the core enumeration (no default arm) so adding a
// projection to core breaks the build here instead of decoding to a wrong C API constant.
static int MapLensTypeToCApi(ns::LensParam::LensType type) {
  switch (type) {
    case ns::LensParam::kLinear:
      return LUMICE_LENS_TYPE_LINEAR;
    case ns::LensParam::kFisheyeEqualArea:
      return LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA;
    case ns::LensParam::kFisheyeEquidistant:
      return LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT;
    case ns::LensParam::kFisheyeStereographic:
      return LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC;
    case ns::LensParam::kDualFisheyeEqualArea:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA;
    case ns::LensParam::kDualFisheyeEquidistant:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUIDISTANT;
    case ns::LensParam::kDualFisheyeStereographic:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_STEREOGRAPHIC;
    case ns::LensParam::kRectangular:
      return LUMICE_LENS_TYPE_RECTANGULAR;
    case ns::LensParam::kFisheyeOrthographic:
      return LUMICE_LENS_TYPE_FISHEYE_ORTHOGRAPHIC;
    case ns::LensParam::kDualFisheyeOrthographic:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_ORTHOGRAPHIC;
    case ns::LensParam::kGlobe:
      return LUMICE_LENS_TYPE_GLOBE;
  }
  throw std::invalid_argument("unmapped core LensType: " + std::to_string(static_cast<int>(type)));
}

// Map LUMICE_VISIBLE_* to its core enumerator. Throws std::invalid_argument on an unknown value.
static ns::RenderConfig::VisibleRange MapVisibleFromCApi(int visible) {
  switch (visible) {
    case LUMICE_VISIBLE_UPPER:
      return ns::RenderConfig::kUpper;
    case LUMICE_VISIBLE_LOWER:
      return ns::RenderConfig::kLower;
    case LUMICE_VISIBLE_FULL:
      return ns::RenderConfig::kFull;
    default:
      throw std::invalid_argument("LUMICE_RenderParam.visible is invalid: " + std::to_string(visible));
  }
}

static ns::RenderConfig::EvMode MapEvModeFromCApi(int ev_mode) {
  switch (ev_mode) {
    case LUMICE_EV_MODE_RELATIVE:
      return ns::RenderConfig::kRelative;
    case LUMICE_EV_MODE_ABSOLUTE:
      return ns::RenderConfig::kAbsolute;
    default:
      throw std::invalid_argument("LUMICE_RenderParam.ev_mode is invalid: " + std::to_string(ev_mode));
  }
}

// Copy `count` C grid lines into the core vector form so nlohmann's to_json(GridLineParam) owns
// the wire shape (single source for the per-field optional/default logic).
static std::vector<ns::GridLineParam> GridLinesToCore(const LUMICE_GridLine* lines, int count) {
  std::vector<ns::GridLineParam> out;
  out.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; i++) {
    ns::GridLineParam g;
    g.value_ = lines[i].value;
    g.width_ = lines[i].width;
    g.opacity_ = lines[i].opacity;
    g.color_[0] = lines[i].color[0];
    g.color_[1] = lines[i].color[1];
    g.color_[2] = lines[i].color[2];
    out.push_back(g);
  }
  return out;
}

// Full renderer object. Every field of the C struct is written verbatim; the numeric encodings
// (lens f<->fov trigonometry, grid-line defaults) come from core's to_json overloads rather than
// being re-derived here, so there is exactly one implementation of each formula.
// Throws std::invalid_argument on an invalid lens_type / visible / grid count; callers wrap.
// The grid-count check lives here (not only in the entry points) because this is the single place
// that dereferences angular_dist[]/elevation_grid[]/longitude_grid[] — ConfigToJson accepts a
// caller-assembled struct with no bounds pass of its own.
static nlohmann::json RendererToJson(const LUMICE_RenderParam& r, int id) {
  if (r.angular_dist_count < 0 || r.angular_dist_count > LUMICE_MAX_CONFIG_GRID_LINES || r.elevation_grid_count < 0 ||
      r.elevation_grid_count > LUMICE_MAX_CONFIG_GRID_LINES || r.longitude_grid_count < 0 ||
      r.longitude_grid_count > LUMICE_MAX_CONFIG_GRID_LINES) {
    throw std::invalid_argument(
        "LUMICE_RenderParam grid count out of range: angular_dist=" + std::to_string(r.angular_dist_count) +
        ", elevation=" + std::to_string(r.elevation_grid_count) +
        ", longitude=" + std::to_string(r.longitude_grid_count));
  }
  nlohmann::json jr;
  jr["id"] = id;
  jr["lens"] = ns::LensParam{ MapLensTypeFromCApi(r.lens_type), r.lens_fov };
  jr["lens_shift"] = { r.lens_shift[0], r.lens_shift[1] };
  jr["resolution"] = { r.resolution_w, r.resolution_h };
  jr["view"] = ns::ViewParam{ r.view_azimuth, r.view_elevation, r.view_roll };
  jr["visible"] = MapVisibleFromCApi(r.visible);
  // Its own top-level key, not a "visible" enumerator: the two clips are orthogonal and AND
  // together. Twin of core's to_json in render_config.cpp.
  jr["front"] = r.front != 0;
  // Back to sRGB on the way out — the struct field is linear, the JSON key is not.
  jr["background"] = { ns::LinearToSrgb(r.background[0]), ns::LinearToSrgb(r.background[1]),
                       ns::LinearToSrgb(r.background[2]) };
  jr["ray_color"] = { r.ray_color[0], r.ray_color[1], r.ray_color[2] };
  jr["intensity_factor"] = r.intensity_factor;
  jr["overlap"] = r.overlap;
  jr["ev_mode"] = MapEvModeFromCApi(r.ev_mode);
  jr["grid"]["angular_dist"] = GridLinesToCore(r.angular_dist, r.angular_dist_count);
  jr["grid"]["elevation"] = GridLinesToCore(r.elevation_grid, r.elevation_grid_count);
  jr["grid"]["longitude"] = GridLinesToCore(r.longitude_grid, r.longitude_grid_count);
  jr["grid"]["horizon"] = r.horizon != 0;
  // Through core's own to_json, like every other value here: the key names and the sRGB convention
  // then have exactly one spelling in the tree.
  ns::ZenithNadirParam zn;
  zn.enabled_ = r.zenith_nadir != 0;
  zn.radius_px_ = r.zenith_nadir_radius_px;
  zn.opacity_ = r.zenith_nadir_opacity;
  zn.color_[0] = r.zenith_nadir_color[0];
  zn.color_[1] = r.zenith_nadir_color[1];
  zn.color_[2] = r.zenith_nadir_color[2];
  jr["grid"]["zenith_nadir"] = zn;
  return jr;
}

// One scattering layer: {"prob", "entries":[{"crystal","proportion"[,"filter"]}]}.
static nlohmann::json ScatterLayerToJson(const LUMICE_ScatterLayer& layer) {
  nlohmann::json jl;
  jl["prob"] = layer.probability;
  jl["entries"] = nlohmann::json::array();
  for (int k = 0; k < layer.entry_count; k++) {
    const auto& e = layer.entries[k];
    nlohmann::json je;
    je["crystal"] = e.crystal_id >= 0 ? e.crystal_id : 1;
    je["proportion"] = e.proportion;
    if (e.filter_id >= 0) {
      je["filter"] = e.filter_id;
    }
    jl["entries"].push_back(je);
  }
  return jl;
}

// One raypath color class. combine/visible/solo emitted only when non-default (mirrors core
// to_json). Throws on an invalid combine or an invalid match predicate.
static nlohmann::json ColorClassToJson(const LUMICE_ColorClass& cls) {
  nlohmann::json jc;
  jc["color"] = { cls.color[0], cls.color[1], cls.color[2] };
  if (cls.combine == LUMICE_COLOR_COMBINE_ALL) {
    jc["combine"] = "all";
  } else if (cls.combine != LUMICE_COLOR_COMBINE_ANY) {
    throw std::invalid_argument("LUMICE_ColorClass.combine is invalid: " + std::to_string(cls.combine));
  }
  if (!cls.visible) {
    jc["visible"] = false;
  }
  if (cls.solo) {
    jc["solo"] = true;
  }
  nlohmann::json match = nlohmann::json::array();
  for (int k = 0; k < cls.match_count; k++) {
    const auto& ref = cls.match[k];
    nlohmann::json jr;
    jr["layer"] = ref.layer;
    jr["crystal"] = ref.crystal;
    ColorPredicateToJson(ref.predicate, jr);
    match.push_back(jr);
  }
  jc["match"] = match;
  return jc;
}

// Map LUMICE_COLOR_MODE_* to its wire string. Throws std::invalid_argument on an invalid mode.
static const char* ColorModeToString(int mode) {
  switch (mode) {
    case LUMICE_COLOR_MODE_DOMINANT:
      return "dominant";
    case LUMICE_COLOR_MODE_ADDITIVE:
      return "additive";
    case LUMICE_COLOR_MODE_PAINTER:
      return "painter";
    default:
      throw std::invalid_argument("raypath_color_mode is invalid: " + std::to_string(mode));
  }
}

// Non-static (declared in server/c_api_internal.hpp) so unit tests can assert the
// emitted filter JSON shape field by field. See that header for rationale.
nlohmann::json ConfigToJson(const ConfigScratch& c) {
  using json = nlohmann::json;
  json root;

  // Crystals
  root["crystal"] = json::array();
  for (int i = 0; i < c.crystal_count; i++) {
    root["crystal"].push_back(CrystalToJson(c.crystals[i], c.crystals[i].id));
  }

  // Filters
  root["filter"] = json::array();
  for (int i = 0; i < c.filter_count; i++) {
    const auto& f = c.filters[i];
    json j;
    j["id"] = f.id;
    j["action"] = f.action == 0 ? "filter_in" : "filter_out";

    // COMPLEX resolves its composition from this config's compositions[] pool (the Scene path
    // gets it directly via LUMICE_SceneAddComplexFilter). Simple arms defer to the shared
    // encoder. The zero-init guard / invalid-type throw lives in EncodeSimpleFilterBody; every
    // caller of this function wraps it and maps the throw to LUMICE_ERR_INVALID_CONFIG.
    if (f.type == LUMICE_FILTER_TYPE_COMPLEX) {
      j["type"] = "complex";
      if (f.composition_index < 0 || f.composition_index >= c.composition_count) {
        throw std::invalid_argument("LUMICE_FilterParam.composition_index out of range: " +
                                    std::to_string(f.composition_index));
      }
      j["composition"] = CompositionArrayToJson(c.compositions[f.composition_index]);
    } else {
      EncodeSimpleFilterBody(f, j);
    }

    // symmetry is a common FilterConfig field (see core filter_config.cpp::to_json, which
    // emits it for every type before the per-type fields), so it stays outside the switch
    // and applies to all arms — do not fold it into the raypath case. Emitted
    // UNCONDITIONALLY (empty string when no bits set) to stay byte-isomorphic with core,
    // whose to_json always writes j["symmetry"] = sym.
    j["symmetry"] = SymmetryBitsToString(f.symmetry);
    root["filter"].push_back(j);
  }

  // Scene
  json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = c.sun_altitude;
  scene["light_source"]["azimuth"] = c.sun_azimuth;
  scene["light_source"]["diameter"] = c.sun_diameter;
  if (c.spectrum_count > 0) {
    // Discrete custom spectrum. Shape matches core light_config.cpp::SpectrumToJson.
    json spectrum = json::array();
    for (int i = 0; i < c.spectrum_count; i++) {
      json e;
      e["wavelength"] = c.spectrum_entries[i].wavelength;
      e["weight"] = c.spectrum_entries[i].weight;
      spectrum.push_back(e);
    }
    scene["light_source"]["spectrum"] = spectrum;
  } else {
    scene["light_source"]["spectrum"] = c.spectrum ? c.spectrum : "D65";
  }

  if (c.infinite) {
    scene["ray_num"] = "infinite";
  } else {
    scene["ray_num"] = c.ray_num;
  }
  scene["max_hits"] = c.max_hits;
  // geom_clock: emit only when set (mirrors core proj_config.cpp::to_json's `if (geom_clock_ != 0)`).
  if (c.geom_clock != 0) {
    scene["geom_clock"] = c.geom_clock;
  }

  scene["scattering"] = json::array();
  for (int i = 0; i < c.scatter_count; i++) {
    scene["scattering"].push_back(ScatterLayerToJson(c.scattering[i]));
  }
  root["scene"] = scene;

  // Renderers — Core always produces dual equal-area fisheye texture (full-globe, equal-area).
  // GUI shader reprojects from this format to the user's display projection.
  root["render"] = json::array();
  for (int i = 0; i < c.renderer_count; i++) {
    root["render"].push_back(RendererToJson(c.renderers[i], c.renderers[i].id));
  }

  // Raypath color classes (task-342.2). Only emit when non-empty so the mono/no-color case
  // matches the pre-v4.7 JSON shape byte-for-byte (AC4). The wire form always uses the object
  // shape `{"mode": ..., "classes": [...]}` — core RaypathColorConfig::from_json accepts both
  // the bare-array (dominant-only) and object shapes, so emitting the object form does not
  // constrain the reader; it simplifies the emit path (no mode string comparison branch).
  if (c.raypath_color_count > 0) {
    nlohmann::json classes = nlohmann::json::array();
    for (int i = 0; i < c.raypath_color_count; i++) {
      classes.push_back(ColorClassToJson(c.raypath_color[i]));
    }
    nlohmann::json rc;
    rc["mode"] = ColorModeToString(c.raypath_color_mode);
    rc["classes"] = classes;
    root["raypath_color"] = rc;
  }

  return root;
}

// =============== Scene (opaque handle) ===============
// LUMICE_Scene_ wraps an nlohmann::json `root` that is byte-isomorphic with the config wire
// format ConfigToJson emits (root["crystal"]/["filter"]/["scene"]/["render"]/["raypath_color"]).
// The Add*/Set* family reuses the SAME per-item encoders as ConfigToJson (single source), just
// one item at a time. Lifecycle mirrors LUMICE_Server_ (new/delete); the json copy constructor
// is a deep copy, so SceneClone is a one-liner with no hand-written field copying.
struct LUMICE_Scene_ {
  nlohmann::json root;
};

// Empty-scene skeleton: the same shape ConfigToJson emits for a zero-initialized ConfigScratch,
// minus the optional raypath_color (absent until SetColorMode/AddColorClass — the same
// "count == 0 → omit the key" isomorphism the batch path keeps). Scene scalars default to a
// default config's values; Set* overwrites them.
static nlohmann::json MakeEmptySceneRoot() {
  nlohmann::json root;
  root["crystal"] = nlohmann::json::array();
  root["filter"] = nlohmann::json::array();
  nlohmann::json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = 0.0f;
  scene["light_source"]["azimuth"] = 0.0f;
  scene["light_source"]["diameter"] = 0.0f;
  scene["light_source"]["spectrum"] = "D65";
  scene["ray_num"] = static_cast<LUMICE_RayCount>(0);
  scene["max_hits"] = 0;
  scene["scattering"] = nlohmann::json::array();
  root["scene"] = scene;
  root["render"] = nlohmann::json::array();
  return root;
}

// Create root["raypath_color"] = {"mode": <core default>, "classes": []} if absent, so mode and
// classes always coexist once either SetColorMode or AddColorClass has touched the color path.
// The key stays entirely absent until then (matches ConfigToJson's "raypath_color_count == 0 →
// omit" byte-isomorphism). Uses the SAME core single-source default the batch reader falls back
// to (ns::kDefaultCompositeMode).
static void EnsureRaypathColor(nlohmann::json& root) {
  if (!root.contains("raypath_color")) {
    nlohmann::json rc;
    rc["mode"] = ns::kDefaultCompositeMode;
    rc["classes"] = nlohmann::json::array();
    root["raypath_color"] = rc;
  }
}

// ---------- Lifecycle ----------
LUMICE_Scene* LUMICE_SceneCreate(void) {
  // Build the skeleton first so a throw (bad_alloc) leaks nothing.
  return new LUMICE_Scene_{ MakeEmptySceneRoot() };
}


LUMICE_Scene* LUMICE_SceneClone(const LUMICE_Scene* scene) {
  if (!scene) {
    return nullptr;
  }
  // nlohmann::json's copy constructor deep-copies the whole tree — no aliasing with the source.
  return new LUMICE_Scene_(*scene);
}


void LUMICE_SceneDestroy(LUMICE_Scene* scene) {
  delete scene;  // delete nullptr is a no-op (NULL-safe, same as LUMICE_DestroyServer)
}


// ---------- Incremental build: Add* ----------
LUMICE_ErrorCode LUMICE_SceneAddCrystal(LUMICE_Scene* scene, const LUMICE_CrystalParam* crystal, int* out_id) {
  if (!scene || !crystal || !out_id) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto& arr = scene->root["crystal"];
  if (arr.size() >= LUMICE_MAX_CONFIG_CRYSTALS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  const int id = static_cast<int>(arr.size());
  // Encode into a temporary first: any throw leaves the scene untouched (no partial write).
  nlohmann::json j;
  try {
    j = CrystalToJson(*crystal, id);
  } catch (const std::exception& e) {
    LOG_ERROR("LUMICE_SceneAddCrystal: invalid crystal: {}", e.what());
    return LUMICE_ERR_INVALID_CONFIG;
  }
  arr.push_back(std::move(j));
  *out_id = id;
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneAddFilter(LUMICE_Scene* scene, const LUMICE_FilterParam* filter, int* out_id) {
  if (!scene || !filter || !out_id) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto& arr = scene->root["filter"];
  if (arr.size() >= LUMICE_MAX_CONFIG_FILTERS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  // Defensive bound on the fixed-size raypath[] the struct carries (ConfigToJson trusts this;
  // the Scene validates at Add-time so an out-of-range count returns an error, not an OOB read).
  if (filter->type == LUMICE_FILTER_TYPE_RAYPATH &&
      (filter->raypath_count < 0 || filter->raypath_count > LUMICE_MAX_CONFIG_RAYPATH_LEN)) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  const int id = static_cast<int>(arr.size());
  nlohmann::json j;
  j["id"] = id;
  j["action"] = filter->action == 0 ? "filter_in" : "filter_out";
  try {
    // EncodeSimpleFilterBody rejects COMPLEX (use SceneAddComplexFilter) and UNSET/invalid.
    EncodeSimpleFilterBody(*filter, j);
  } catch (const std::exception& e) {
    LOG_ERROR("LUMICE_SceneAddFilter: invalid filter: {}", e.what());
    return LUMICE_ERR_INVALID_CONFIG;
  }
  j["symmetry"] = SymmetryBitsToString(filter->symmetry);
  arr.push_back(std::move(j));
  *out_id = id;
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneAddComplexFilter(LUMICE_Scene* scene, const LUMICE_FilterParam* filter,
                                              const LUMICE_ComplexComposition* composition, int* out_id) {
  if (!scene || !filter || !composition || !out_id) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto& arr = scene->root["filter"];
  if (arr.size() >= LUMICE_MAX_CONFIG_FILTERS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  // Validate the composition storage before writing anything (clause/term bounds +
  // null-pointer defense). term_ids may legitimately be null iff every
  // clause has 0 terms (total_terms == 0) — see LUMICE_CompositionSetClauses.
  const auto& comp = *composition;
  if (comp.clause_count < 0 || comp.clause_count > LUMICE_MAX_CONFIG_CLAUSES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  if (comp.clause_count > 0 && comp.term_counts == nullptr) {
    return LUMICE_ERR_NULL_ARG;
  }
  size_t total_terms = 0;
  for (int cl = 0; cl < comp.clause_count; cl++) {
    if (comp.term_counts[cl] < 0 || comp.term_counts[cl] > LUMICE_MAX_CONFIG_TERMS) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    total_terms += static_cast<size_t>(comp.term_counts[cl]);
  }
  if (total_terms > 0 && comp.term_ids == nullptr) {
    return LUMICE_ERR_NULL_ARG;
  }
  const int id = static_cast<int>(arr.size());
  nlohmann::json j;
  j["id"] = id;
  j["action"] = filter->action == 0 ? "filter_in" : "filter_out";
  j["type"] = "complex";
  // Deep-copy the composition into the JSON now: term_ids/term_counts are read and encoded, so
  // the caller's LUMICE_ComplexComposition may be released/reused right after this returns.
  j["composition"] = CompositionArrayToJson(comp);
  j["symmetry"] = SymmetryBitsToString(filter->symmetry);
  arr.push_back(std::move(j));
  *out_id = id;
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneAddRenderer(LUMICE_Scene* scene, const LUMICE_RenderParam* renderer, int* out_id) {
  if (!scene || !renderer || !out_id) {
    return LUMICE_ERR_NULL_ARG;
  }
  // Grid counts index the fixed-capacity inline arrays RendererToJson reads; validate before the
  // encode so an out-of-range count cannot walk off the end of
  // angular_dist[]/elevation_grid[]/longitude_grid[].
  if (renderer->angular_dist_count < 0 || renderer->angular_dist_count > LUMICE_MAX_CONFIG_GRID_LINES ||
      renderer->elevation_grid_count < 0 || renderer->elevation_grid_count > LUMICE_MAX_CONFIG_GRID_LINES ||
      renderer->longitude_grid_count < 0 || renderer->longitude_grid_count > LUMICE_MAX_CONFIG_GRID_LINES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  auto& arr = scene->root["render"];
  if (arr.size() >= LUMICE_MAX_CONFIG_RENDERERS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  const int id = static_cast<int>(arr.size());
  // Encode first (throws on an invalid lens_type / visible) so a failure leaves the scene
  // untouched — mirrors LUMICE_SceneAddColorClass.
  nlohmann::json jr;
  try {
    jr = RendererToJson(*renderer, id);
  } catch (const std::exception& e) {
    LOG_ERROR("LUMICE_SceneAddRenderer: invalid renderer: {}", e.what());
    return LUMICE_ERR_INVALID_CONFIG;
  }
  arr.push_back(std::move(jr));
  *out_id = id;
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneAddScatterLayer(LUMICE_Scene* scene, const LUMICE_ScatterLayer* layer, int* out_id) {
  if (!scene || !layer || !out_id) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (layer->entry_count < 0 || layer->entry_count > LUMICE_MAX_CONFIG_SCATTER_ENTRIES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  auto& scene_j = scene->root["scene"];
  // The skeleton always seeds an empty scattering array; guard defensively in case a scene built
  // by a future FromJson path lacks it.
  if (!scene_j.contains("scattering") || !scene_j["scattering"].is_array()) {
    scene_j["scattering"] = nlohmann::json::array();
  }
  auto& arr = scene_j["scattering"];
  if (arr.size() >= LUMICE_MAX_CONFIG_SCATTER_LAYERS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  const int id = static_cast<int>(arr.size());
  arr.push_back(ScatterLayerToJson(*layer));
  *out_id = id;
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneAddColorClass(LUMICE_Scene* scene, const LUMICE_ColorClass* color_class, int* out_id) {
  if (!scene || !color_class || !out_id) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (color_class->match_count < 0 || color_class->match_count > LUMICE_MAX_CONFIG_COLOR_REFS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  // Encode first (throws on invalid combine / predicate) so a failure leaves the scene untouched
  // — do NOT create the raypath_color key before this succeeds.
  nlohmann::json jc;
  try {
    jc = ColorClassToJson(*color_class);
  } catch (const std::exception& e) {
    LOG_ERROR("LUMICE_SceneAddColorClass: invalid color class: {}", e.what());
    return LUMICE_ERR_INVALID_CONFIG;
  }
  // Soft cap check must peek at the current class count WITHOUT creating the key.
  int existing = 0;
  if (scene->root.contains("raypath_color") && scene->root["raypath_color"].contains("classes")) {
    existing = static_cast<int>(scene->root["raypath_color"]["classes"].size());
  }
  if (existing >= LUMICE_MAX_CONFIG_COLOR_CLASSES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  EnsureRaypathColor(scene->root);
  auto& arr = scene->root["raypath_color"]["classes"];
  const int id = static_cast<int>(arr.size());
  arr.push_back(std::move(jc));
  *out_id = id;
  return LUMICE_OK;
}


// ---------- Scalar / whole-group settings: Set* ----------
LUMICE_ErrorCode LUMICE_SceneSetLightSource(LUMICE_Scene* scene, float sun_altitude, float sun_azimuth,
                                            float sun_diameter, const char* spectrum) {
  if (!scene) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto& ls = scene->root["scene"]["light_source"];
  ls["type"] = "sun";
  ls["altitude"] = sun_altitude;
  ls["azimuth"] = sun_azimuth;
  ls["diameter"] = sun_diameter;
  // A discrete spectrum (an array set by SceneSetCustomSpectrum) wins over the string, mirroring
  // the ConfigScratch spectrum_count > 0 rule — do NOT clobber it here. Only write the string
  // when no discrete spectrum is currently set. This makes the two call orders converge.
  if (!ls.contains("spectrum") || !ls["spectrum"].is_array()) {
    ls["spectrum"] = spectrum ? spectrum : "D65";
  }
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneSetCustomSpectrum(LUMICE_Scene* scene, const LUMICE_SpectrumEntry* entries, int count) {
  if (!scene) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (count < 0 || count > LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  if (count > 0 && entries == nullptr) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto& ls = scene->root["scene"]["light_source"];
  if (count == 0) {
    // Clear the discrete spectrum: fall back to the default string (matches spectrum_count == 0).
    ls["spectrum"] = "D65";
  } else {
    nlohmann::json spectrum = nlohmann::json::array();
    for (int i = 0; i < count; i++) {
      nlohmann::json e;
      e["wavelength"] = entries[i].wavelength;
      e["weight"] = entries[i].weight;
      spectrum.push_back(e);
    }
    ls["spectrum"] = spectrum;
  }
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneSetSimParams(LUMICE_Scene* scene, int infinite, LUMICE_RayCount ray_num, int max_hits,
                                          int geom_clock) {
  if (!scene) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto& scene_j = scene->root["scene"];
  if (infinite) {
    scene_j["ray_num"] = "infinite";
  } else {
    scene_j["ray_num"] = ray_num;
  }
  scene_j["max_hits"] = max_hits;
  // geom_clock: emit only when set (mirrors ConfigToJson's `if (geom_clock != 0)`); erase on 0 so
  // a prior non-zero value is cleared and the "omit when 0" isomorphism holds.
  if (geom_clock != 0) {
    scene_j["geom_clock"] = geom_clock;
  } else {
    scene_j.erase("geom_clock");
  }
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneSetColorMode(LUMICE_Scene* scene, int raypath_color_mode) {
  if (!scene) {
    return LUMICE_ERR_NULL_ARG;
  }
  const char* mode_str = nullptr;
  try {
    mode_str = ColorModeToString(raypath_color_mode);
  } catch (const std::exception& e) {
    LOG_ERROR("LUMICE_SceneSetColorMode: {}", e.what());
    return LUMICE_ERR_INVALID_CONFIG;
  }
  EnsureRaypathColor(scene->root);
  scene->root["raypath_color"]["mode"] = mode_str;
  return LUMICE_OK;
}


// Test-only accessor (declared in server/c_api_internal.hpp). See that header for rationale.
const nlohmann::json& SceneRoot(const LUMICE_Scene* scene) {
  return scene->root;
}


// ---------- Serialization: decoupled from commit ----------
// SceneToJson lives in the Scene section because it only depends on scene->root (no JsonToConfig).
// Buffer contract is snprintf-style (see lumice.h). root.dump() can throw type_error if a
// Set* stored a non-UTF-8 string (SetLightSource/SetCustomSpectrum take an unvalidated const char*),
// so the dump is guarded — the exception must not cross the C ABI boundary.
LUMICE_ErrorCode LUMICE_SceneToJson(const LUMICE_Scene* scene, char* out_buf, size_t buf_size, size_t* out_len) {
  if (!scene) {
    return LUMICE_ERR_NULL_ARG;
  }
  std::string json_str;
  try {
    json_str = scene->root.dump();
  } catch (const std::exception& e) {
    LOG_ERROR("LUMICE_SceneToJson: failed to serialize scene: {}", e.what());
    return LUMICE_ERR_INVALID_CONFIG;
  }
  if (out_len) {
    *out_len = json_str.size();
  }
  if (out_buf && buf_size > 0) {
    size_t n = std::min(json_str.size(), buf_size - 1);
    std::memcpy(out_buf, json_str.data(), n);
    out_buf[n] = '\0';
  }
  return LUMICE_OK;
}


// Display-time color update: see doc/capi-lifecycle-architecture.md §4 / §6.4.
// task-342.2: does NOT restart the simulation — accumulator, epoch, and consumers are
// untouched. Only the next acquired result frame re-composites with the new appearance.
LUMICE_ErrorCode LUMICE_SetRaypathColors(LUMICE_Server* server, const LUMICE_ColorClassDisplay* classes,
                                         int class_count, const int* z_order, int mode) {
  if (!server) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (class_count < 0 || (class_count > 0 && classes == nullptr)) {
    return LUMICE_ERR_NULL_ARG;
  }
  ns::CompositeMode composite_mode = ns::CompositeMode::kDominant;
  switch (mode) {
    case LUMICE_COLOR_MODE_DOMINANT:
      composite_mode = ns::CompositeMode::kDominant;
      break;
    case LUMICE_COLOR_MODE_ADDITIVE:
      composite_mode = ns::CompositeMode::kAdditive;
      break;
    case LUMICE_COLOR_MODE_PAINTER:
      composite_mode = ns::CompositeMode::kPainter;
      break;
    default:
      return LUMICE_ERR_INVALID_VALUE;
  }
  std::vector<ns::ColorClassDisplay> internal;
  internal.reserve(static_cast<size_t>(class_count));
  for (int i = 0; i < class_count; i++) {
    ns::ColorClassDisplay d;
    d.color_[0] = classes[i].color[0];
    d.color_[1] = classes[i].color[1];
    d.color_[2] = classes[i].color[2];
    d.visible_ = classes[i].visible != 0;
    d.solo_ = classes[i].solo != 0;
    internal.push_back(d);
  }
  auto err = server->server_->SetRaypathColors(internal.data(), class_count, z_order, composite_mode);
  if (err) {
    LOG_ERROR("LUMICE_SetRaypathColors failed: {}", err.message);
    return MapErrorCode(err.code);
  }
  return LUMICE_OK;
}


// task-345.3: display-time EV multiplier for the composite path. See the
// LUMICE_SetCompositeExposure comment in include/lumice.h for the semantics
// (single scalar, mono path untouched, snapshot_dirty_ flipped so the next
// acquired result frame rebakes the composite). No ev_total validation: the GUI is the
// only in-tree caller and already clamps to [-6, 6]; server-side double-clamp
// would hide caller bugs without preventing any real hazard.
LUMICE_ErrorCode LUMICE_SetCompositeExposure(LUMICE_Server* server, float ev_total) {
  if (!server) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto err = server->server_->SetCompositeExposure(ev_total);
  if (err) {
    LOG_ERROR("LUMICE_SetCompositeExposure failed: {}", err.message);
    return MapErrorCode(err.code);
  }
  return LUMICE_OK;
}


// Display-time background colour for the composite path. See the
// LUMICE_SetCompositeBackground comment in include/lumice.h for
// the semantics (3 ADDITIVE linear floats, masked to the imaged region, mono
// path untouched, snapshot_dirty_ flipped so the next acquired result frame
// rebakes the composite). Unlike the exposure setter this one takes a pointer,
// so it needs its own null check; component values are not validated for the
// same reason ev_total is not — the caller owns the colour space conversion and
// the compositor clamps at the sRGB stage regardless.
LUMICE_ErrorCode LUMICE_SetCompositeBackground(LUMICE_Server* server, const float* background_linear) {
  if (!server || !background_linear) {
    return LUMICE_ERR_NULL_ARG;
  }
  auto err = server->server_->SetCompositeBackground(background_linear);
  if (err) {
    LOG_ERROR("LUMICE_SetCompositeBackground failed: {}", err.message);
    return MapErrorCode(err.code);
  }
  return LUMICE_OK;
}


// task-342.3 AC4: per-color-class empty-arc detector.
LUMICE_ErrorCode LUMICE_GetColorClassSignal(LUMICE_Server* server, int* out_flags, int class_count) {
  if (!server) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (class_count < 0) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  if (class_count > 0 && out_flags == nullptr) {
    return LUMICE_ERR_NULL_ARG;
  }
  std::vector<uint8_t> tmp(static_cast<size_t>(class_count), 0);
  auto err = server->server_->GetColorClassSignals(tmp.data(), class_count);
  if (err) {
    LOG_ERROR("LUMICE_GetColorClassSignal failed: {}", err.message);
    return MapErrorCode(err.code);
  }
  for (int i = 0; i < class_count; i++) {
    out_flags[i] = tmp[static_cast<size_t>(i)] ? 1 : 0;
  }
  return LUMICE_OK;
}


// =============== ConfigScratch raypath-color storage (internal) ===============
// See c_api_internal.hpp for the ownership contract. calloc/free (not new[]/delete[]) is kept
// from when this pair was public C API: the allocation shape must stay interchangeable with
// LUMICE_CompositionSetClauses's, which still crosses the C ABI and must remain releasable
// without a C++ runtime — a documented exception to the "no raw new/delete" rule (AGENTS.md).
LUMICE_ColorClass* ConfigCreateColorClasses(ConfigScratch* cfg, int count) {
  if (!cfg) {
    return nullptr;
  }
  if (count < 0 || count > LUMICE_MAX_CONFIG_COLOR_CLASSES) {
    return nullptr;
  }
  // Create-or-replace: any existing allocation must be released before we overwrite the
  // pointer. Guards the "consecutive Create with different counts" pattern from leaking
  // the previous allocation, and pairs with the memset-before-Release entry in JsonToConfig
  // (which calls Release explicitly to make the intent obvious).
  if (cfg->raypath_color) {
    std::free(cfg->raypath_color);  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary; see block comment above.
    cfg->raypath_color = nullptr;
  }
  if (count == 0) {
    // Explicitly skip calloc(0, ...) — implementation-defined behavior. Zero classes is a
    // valid state (mono-only); leave pointer nullptr and count 0.
    cfg->raypath_color_count = 0;
    return nullptr;
  }
  auto* buf = static_cast<LUMICE_ColorClass*>(
      std::calloc(  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary; see block comment above.
          static_cast<size_t>(count), sizeof(LUMICE_ColorClass)));
  if (!buf) {
    // OOM: leave cfg in the "no classes" state, callers must check the return value.
    cfg->raypath_color_count = 0;
    return nullptr;
  }
  cfg->raypath_color = buf;
  cfg->raypath_color_count = count;
  return buf;
}

void ConfigReleaseColorClasses(ConfigScratch* cfg) {
  if (!cfg) {
    return;
  }
  if (cfg->raypath_color) {
    std::free(cfg->raypath_color);  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary; see block comment above.
    cfg->raypath_color = nullptr;
  }
  cfg->raypath_color_count = 0;
}


// =============== Complex-Composition storage lifecycle (task-host-abi-cpu-caps, BREAKING v4.9) ===============
// calloc/free (not new[]/delete[]) deliberately: LUMICE_ComplexComposition crosses the C ABI,
// and non-C++ bindings must be able to release the composition storage without a C++ runtime —
// a documented exception to the "no raw new/delete" project rule (AGENTS.md).

LUMICE_ErrorCode LUMICE_CompositionSetClauses(LUMICE_ComplexComposition* comp, int clause_count, const int* term_counts,
                                              const int* term_ids) {
  if (!comp) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (clause_count < 0 || clause_count > LUMICE_MAX_CONFIG_CLAUSES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  // term_counts is required whenever clause_count > 0 (every clause needs its own count).
  // term_ids, however, must NOT be required upfront: a composition where every clause has 0
  // terms (total_terms == 0) is legitimate (LUMICE_CompositionClauseTerms's doc comment already
  // treats it as such), and a caller with
  // total_terms == 0 may reasonably pass term_ids == nullptr for that empty flat buffer — e.g.
  // JsonToComplexComposition's std::vector<int> term_ids_vec, whose .data() is nullptr when
  // never push_back'd into. Requiring non-null unconditionally rejected exactly that legitimate
  // shape at the ONLY writer, before any consumer ever saw it (code-review round 2, Major —
  // round 1's fix only patched the read-side check, not this entry point). So the term_ids
  // null-check is deferred until total_terms is known.
  if (clause_count > 0 && term_counts == nullptr) {
    return LUMICE_ERR_NULL_ARG;
  }
  // Full validation BEFORE any allocation so an invalid input leaves `comp` untouched
  // (mirrors the "no partial writes on rejection" contract other Set-style API here follow).
  // Upper bounds are enforced here (not just by callers) because this is documented as
  // the ONLY supported writer, including non-C++ bindings that cannot be trusted to
  // pre-check LUMICE_MAX_CONFIG_CLAUSES/_TERMS themselves (code-review round 1, Major).
  size_t total_terms = 0;
  for (int cl = 0; cl < clause_count; cl++) {
    if (term_counts[cl] < 0 || term_counts[cl] > LUMICE_MAX_CONFIG_TERMS) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    total_terms += static_cast<size_t>(term_counts[cl]);
  }
  if (total_terms > 0 && term_ids == nullptr) {
    return LUMICE_ERR_NULL_ARG;
  }

  // Create-or-replace: release any prior allocation before overwriting the pointers,
  // guarding "consecutive Set with different clause_count" from leaking the previous
  // allocation. Pairs with the memset-before-Release entry in JsonToConfig.
  LUMICE_CompositionReleaseClauses(comp);

  if (clause_count == 0) {
    // "OR of nothing" state — no allocation, both pointers stay nullptr.
    return LUMICE_OK;
  }

  auto* counts_buf =
      static_cast<int*>(std::calloc(  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary; see block comment above.
          static_cast<size_t>(clause_count), sizeof(int)));
  if (!counts_buf) {
    // OOM: fall back to "OR of nothing" (already released above); mirror ColorClasses OOM policy.
    return LUMICE_ERR_INVALID_CONFIG;
  }
  int* ids_buf = nullptr;
  if (total_terms > 0) {
    ids_buf =
        static_cast<int*>(std::calloc(  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary; see block comment above.
            total_terms, sizeof(int)));
    if (!ids_buf) {
      std::free(counts_buf);  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary.
      return LUMICE_ERR_INVALID_CONFIG;
    }
  }
  // Both allocations succeeded — commit.
  for (int cl = 0; cl < clause_count; cl++) {
    counts_buf[cl] = term_counts[cl];
  }
  for (size_t i = 0; i < total_terms; i++) {
    ids_buf[i] = term_ids[i];
  }
  comp->term_counts = counts_buf;
  comp->term_ids = ids_buf;
  comp->clause_count = clause_count;
  return LUMICE_OK;
}

void LUMICE_CompositionReleaseClauses(LUMICE_ComplexComposition* comp) {
  if (!comp) {
    return;
  }
  if (comp->term_counts) {
    std::free(comp->term_counts);  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary; see block comment above.
    comp->term_counts = nullptr;
  }
  if (comp->term_ids) {
    std::free(comp->term_ids);  // NOLINT(cppcoreguidelines-no-malloc): C ABI boundary; see block comment above.
    comp->term_ids = nullptr;
  }
  comp->clause_count = 0;
}

void ConfigReleaseCompositions(ConfigScratch* cfg) {
  if (!cfg) {
    return;
  }
  // Composition_count is a plain int written by callers; iterate up to whatever they set,
  // but never past the ABI ceiling (defends against a garbage / uninitialized count value).
  int n = cfg->composition_count;
  if (n < 0) {
    n = 0;
  }
  if (n > LUMICE_MAX_CONFIG_COMPLEX) {
    n = LUMICE_MAX_CONFIG_COMPLEX;
  }
  for (int i = 0; i < n; i++) {
    LUMICE_CompositionReleaseClauses(&cfg->compositions[i]);
  }
  // Leave composition_count alone — this Release only frees the intra-record heap storage;
  // the inline compositions[] array itself is part of ConfigScratch's own layout.
}

const int* LUMICE_CompositionClauseTerms(const LUMICE_ComplexComposition* comp, int clause_index, int* out_term_count) {
  if (!comp) {
    if (out_term_count) {
      *out_term_count = 0;
    }
    return nullptr;
  }
  if (clause_index < 0 || clause_index >= comp->clause_count) {
    if (out_term_count) {
      *out_term_count = 0;
    }
    return nullptr;
  }
  // Prefix-sum offset into the flat term_ids buffer.
  size_t offset = 0;
  for (int cl = 0; cl < clause_index; cl++) {
    offset += static_cast<size_t>(comp->term_counts[cl]);
  }
  if (out_term_count) {
    *out_term_count = comp->term_counts[clause_index];
  }
  return (comp->term_ids != nullptr) ? (comp->term_ids + offset) : nullptr;
}


// Single source for the "hand an already-encoded scene JSON to the core and report reuse" tail.
// LUMICE_CommitScene is its only caller today (v4.12 removed the three legacy entry points that
// shared it); it stays a separate function because the core commit call, the error log, the
// error-code mapping and the out_reused write-back are one unit that a future second producer of
// an already-encoded document must reuse verbatim rather than re-derive. Callers are responsible
// for their own NULL checks before calling. `source` only tags the error log with the
// originating entry point, so a failed commit stays attributable.
static LUMICE_ErrorCode CommitJsonToServer(LUMICE_Server* server, const nlohmann::json& root, int* out_reused,
                                           const char* source) {
  bool reused = false;
  auto err = server->server_->CommitConfig(root, &reused);
  if (err) {
    LOG_ERROR("Failed to commit configuration ({}): {}", source, err.message);
    return MapErrorCode(err.code);
  }
  if (out_reused) {
    *out_reused = reused ? 1 : 0;
  }
  return LUMICE_OK;
}


// =============== Configuration Parsing (JSON -> ConfigScratch) ===============
// Symmetric inverse of ConfigToJson. Decomposed into per-section helpers.

// Parse one distribution IN PLACE. `*out` must already hold the default this slot carries in
// core, because core's Distribution::from_json overwrites ONLY the keys present in the JSON and
// leaves the rest of the destination untouched — "type without mean" keeps the destination's
// mean, and a bare number keeps its spread. Every call site below therefore pre-fills `*out` with
// the matching core-side default before calling in, rather than relying on the zeroed struct.
static LUMICE_ErrorCode JsonToDistribution(const nlohmann::json& j, LUMICE_Distribution* out) {
  // Bare number => NO_RANDOM (deterministic value). Mirrors core Distribution::from_json's
  // is_number() branch, which sets type + center and does NOT touch spread. This is the common
  // case for shape fields with randomization off, and the whole reason no_random was previously a
  // translation gap: axis JSON always carried a type object so the missing no_random case never
  // fired until shape scalars became distributions.
  if (j.is_number()) {
    out->type = LUMICE_DIST_NO_RANDOM;
    out->center = j.get<float>();
    return LUMICE_OK;
  }
  if (!j.is_object()) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  // "type" is required, mirroring core's Distribution::from_json. This must be rejected HERE and
  // not left to core: the CLI reaches core only through this parser, whose output is re-serialized
  // by ConfigToJson — and that writer emits `type` unconditionally from the struct, so the missing
  // key would already have been filled in before core ever saw it. A check living only in core
  // would leave core's own test green while the CLI silently accepted the document.
  //
  // Reporting is by return code alone, as everywhere else in this function; the three axis call
  // sites in JsonToCrystal add the LOG_ERROR that carries the migration guidance, because only
  // they know which crystal and which slot. This one cannot: the shape scalars parse through here
  // too, and it has no way to tell them apart.
  if (!j.contains("type")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  auto type_str = j.at("type").get<std::string>();
  // Defensive: tolerate an explicit {"type":"no_random", ...} object on the READ side even though
  // the WRITE side (DistributionToJson) only ever produces a bare number for NO_RANDOM.
  if (type_str == "no_random") {
    out->type = LUMICE_DIST_NO_RANDOM;
    if (j.contains("mean")) {
      out->center = j.at("mean").get<float>();
    }
    return LUMICE_OK;
  }
  if (type_str == "uniform") {
    out->type = LUMICE_DIST_UNIFORM;
  } else if (type_str == "gauss") {
    out->type = LUMICE_DIST_GAUSS;
  } else if (type_str == "zigzag") {
    out->type = LUMICE_DIST_ZIGZAG;
  } else if (type_str == "laplacian") {
    out->type = LUMICE_DIST_LAPLACIAN;
  } else if (type_str == "gauss_legacy") {
    out->type = LUMICE_DIST_GAUSS_LEGACY;
  } else {
    // Core maps an unrecognized type string to the FIRST entry of its
    // NLOHMANN_JSON_SERIALIZE_ENUM table (a silent misread), which is not a tolerance worth
    // mirroring; rejecting is the deliberate exception to "align with core".
    return LUMICE_ERR_INVALID_VALUE;
  }
  if (j.contains("mean")) {
    out->center = j.at("mean").get<float>();
  }
  if (j.contains("std")) {
    out->spread = j.at("std").get<float>();
  }
  return LUMICE_OK;
}

static const char* MapSpectrumString(const std::string& s) {
  static const std::map<std::string, const char*> kSpectrumMap = {
    { "D65", "D65" }, { "D55", "D55" }, { "D50", "D50" }, { "D75", "D75" }, { "A", "A" }, { "E", "E" },
  };
  auto it = kSpectrumMap.find(s);
  return it != kSpectrumMap.end() ? it->second : nullptr;
}

static LUMICE_ErrorCode JsonToCrystal(const nlohmann::json& cj, LUMICE_CrystalParam* cr) {
  if (!cj.contains("id") || !cj.contains("type")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  cr->id = cj.at("id").get<int>();

  // Type is dispatched BEFORE "shape" is required: core only reaches j.at("shape") inside a
  // recognized type branch, so an unknown type is an invalid VALUE, not a missing field.
  auto type_str = cj.at("type").get<std::string>();
  if (type_str != "prism" && type_str != "pyramid") {
    return LUMICE_ERR_INVALID_VALUE;
  }
  if (!cj.contains("shape")) {
    return LUMICE_ERR_MISSING_FIELD;  // core: j.at("shape")
  }
  // One derivation for the whole function: the check above already narrowed type_str to the two
  // known spellings, so every key query below — per-type and type-independent alike — asks core's
  // one table with this same kind.
  const auto kind = (type_str == "prism") ? ns::CrystalKind::kPrism : ns::CrystalKind::kPyramid;
  if (type_str == "prism") {
    cr->type = 0;
    // height is OPTIONAL: core PrismCrystalParam::h_ defaults to a deterministic 1.0 and
    // from_json only overwrites it when the key is present.
    cr->height = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
    const char* height_key = ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_HEIGHT);
    if (cj.at("shape").contains(height_key)) {
      if (auto err = JsonToDistribution(cj.at("shape").at(height_key), &cr->height); err != LUMICE_OK) {
        return err;
      }
    }
  } else if (type_str == "pyramid") {
    cr->type = 1;
    const auto& shape = cj.at("shape");
    // prism_h is required (core: j.at(prism_h)); the two pyramidal heights are optional and
    // default to a deterministic 0.0 (PyramidCrystalParam::h_pyr_u_ / h_pyr_l_).
    const char* prism_h_key = ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_PRISM_H);
    if (!shape.contains(prism_h_key)) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    if (auto err = JsonToDistribution(shape.at(prism_h_key), &cr->prism_h); err != LUMICE_OK) {
      return err;
    }
    cr->upper_h = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 0.0f, 0.0f };
    cr->lower_h = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 0.0f, 0.0f };
    const char* upper_h_key = ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_UPPER_H);
    if (shape.contains(upper_h_key)) {
      if (auto err = JsonToDistribution(shape.at(upper_h_key), &cr->upper_h); err != LUMICE_OK) {
        return err;
      }
    }
    const char* lower_h_key = ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_LOWER_H);
    if (shape.contains(lower_h_key)) {
      if (auto err = JsonToDistribution(shape.at(lower_h_key), &cr->lower_h); err != LUMICE_OK) {
        return err;
      }
    }
    // Wedge angles default to 28° (PyramidCrystalParam::wedge_angle_u_ / _l_) when neither the
    // explicit angle nor the Miller indices are given — the zeroed struct would mean 0°.
    cr->upper_wedge_angle = 28.0f;
    cr->lower_wedge_angle = 28.0f;
    // Wedge angle: prefer the explicit angle, fallback to the Miller-index conversion
    for (bool upper : { true, false }) {
      float& wedge_angle = upper ? cr->upper_wedge_angle : cr->lower_wedge_angle;
      const char* angle_key = ns::ShapeWedgeAngleKeyName(upper);
      const char* indices_key = ns::ShapeIndicesKeyName(upper);
      if (shape.contains(angle_key) && shape.at(angle_key).is_number()) {
        wedge_angle = shape.at(angle_key).get<float>();
      } else if (shape.contains(indices_key) && shape.at(indices_key).is_array() && shape.at(indices_key).size() == 3) {
        wedge_angle = MillerToAlpha(shape.at(indices_key)[0].get<int>(), shape.at(indices_key)[2].get<int>());
      }
    }
  } else {
    return LUMICE_ERR_INVALID_VALUE;
  }

  // face_distance: every element defaults to a deterministic 1.0 (regular hexagon). A shorter
  // array leaves the remaining faces at that default and a longer one is truncated at 6 — core
  // fills element-by-element and breaks at index 6 rather than demanding exactly six entries.
  for (int k = 0; k < 6; k++) {
    cr->face_distance[k] = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
  }
  const char* fd_key = ns::ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_FACE_0);
  if (cj.at("shape").contains(fd_key)) {
    const auto& fd = cj.at("shape").at(fd_key);
    if (!fd.is_array()) {
      return LUMICE_ERR_INVALID_VALUE;
    }
    const int n = std::min(6, static_cast<int>(fd.size()));
    for (int k = 0; k < n; k++) {
      if (auto err = JsonToDistribution(fd[k], &cr->face_distance[k]); err != LUMICE_OK) {
        return err;
      }
    }
  }

  // Sync groups: read verbatim, only the keys this crystal type owns. Canonicalization and
  // leader normalization stay in core's from_json (see WriteSyncGroupJson's note); this struct
  // holds what the caller / config file declared.
  ReadSyncGroupJson(cj.at("shape"), cr->sync_group, kind);

  // Axis distributions. The two cases are deliberately NOT symmetric, mirroring core:
  //   - `axis` absent      -> AxisDistribution's default constructor: zenith/azimuth/roll all
  //                           deterministic 0 (its latitude 90 IS zenith 0).
  //   - `axis` present     -> `zenith` is required; `azimuth` / `roll` default to a full 360°
  //                           uniform sweep, NOT to 0.
  // Collapsing them into one "all three optional, default 0" rule silently changes the sampled
  // orientation for a partial `axis` object.
  cr->zenith = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 0.0f, 0.0f };
  cr->azimuth = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 0.0f, 0.0f };
  cr->roll = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 0.0f, 0.0f };
  if (cj.contains("axis")) {
    const auto& axis = cj.at("axis");
    // The three rejections below log, unlike the rest of this function: each narrows a published
    // contract, and a hand-written config outside this repo cannot be enumerated, so the message
    // IS the migration guidance its author gets. It names the crystal, the slot, and both legal
    // ways to write it. The "how to write it" half comes from core (FormatAxisSlotHint) rather
    // than being spelled again here — core's parser rejects the same documents, and two copies of
    // that guidance would drift with nothing comparing them.
    const char* zenith_key = ns::AxisScalarKeyName(ns::kAxisScalarZenith);
    if (!axis.contains(zenith_key)) {
      LOG_ERROR(
          "JsonToCrystal: crystal[id={}].axis is present but has no \"{}\", which is required "
          "whenever `axis` is written at all (omit `axis` entirely to get the default orientation "
          "instead). {}",
          cr->id, zenith_key, ns::FormatAxisSlotHint(zenith_key));
      return LUMICE_ERR_MISSING_FIELD;
    }
    // Core reads the zenith key into its latitude slot and then flips it (latitude = 90 - zenith),
    // unconditionally — so a zenith object that omits `mean` inherits the latitude default 90,
    // which is zenith 90 on this side of the flip (NOT zenith 0).
    // Turns a bare LUMICE_ERR_MISSING_FIELD from JsonToDistribution — which is only ever the
    // absent "type" — into the same guidance core throws, with this crystal and this slot named.
    auto report_slot = [cr](const char* slot_key, LUMICE_ErrorCode err) {
      if (err == LUMICE_ERR_MISSING_FIELD) {
        LOG_ERROR("JsonToCrystal: crystal[id={}].axis.{} is a distribution object with no \"type\". {}", cr->id,
                  slot_key, ns::FormatAxisSlotHint(slot_key));
      }
    };
    cr->zenith = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 90.0f, 0.0f };
    if (auto err = JsonToDistribution(axis.at(zenith_key), &cr->zenith); err != LUMICE_OK) {
      report_slot(zenith_key, err);
      return err;
    }
    cr->azimuth = LUMICE_Distribution{ LUMICE_DIST_UNIFORM, 0.0f, 360.0f };
    cr->roll = LUMICE_Distribution{ LUMICE_DIST_UNIFORM, 0.0f, 360.0f };
    const char* azimuth_key = ns::AxisScalarKeyName(ns::kAxisScalarAzimuth);
    if (axis.contains(azimuth_key)) {
      if (auto err = JsonToDistribution(axis.at(azimuth_key), &cr->azimuth); err != LUMICE_OK) {
        report_slot(azimuth_key, err);
        return err;
      }
    }
    const char* roll_key = ns::AxisScalarKeyName(ns::kAxisScalarRoll);
    if (axis.contains(roll_key)) {
      if (auto err = JsonToDistribution(axis.at(roll_key), &cr->roll); err != LUMICE_OK) {
        report_slot(roll_key, err);
        return err;
      }
    }
  }
  return LUMICE_OK;
}

static LUMICE_ErrorCode JsonToFilter(const nlohmann::json& fj, LUMICE_FilterParam* f) {
  if (!fj.contains("id") || !fj.contains("type")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  f->id = fj.at("id").get<int>();

  // action is OPTIONAL and defaults to filter_in (core FilterConfig::from_json). An unrecognized
  // action string is still rejected here: core silently keeps filter_in for one, which turns a
  // typo into a wrong-but-running filter.
  f->action = 0;
  if (fj.contains("action")) {
    auto action_str = fj.at("action").get<std::string>();
    if (action_str == "filter_in") {
      f->action = 0;
    } else if (action_str == "filter_out") {
      f->action = 1;
    } else {
      return LUMICE_ERR_INVALID_VALUE;
    }
  }

  // Type-specific fields (JSON -> struct arm). Field names/defaults mirror core
  // config/filter_config.cpp::from_json. Parse does lossless mapping ONLY: value
  // validation (e.g. entry_exit min_len >= 1, max_len <= kMaxHits) stays single-source
  // in core and fires at commit time (surfaced by LUMICE_CommitScene). -1 encodes
  // an absent optional (wildcard / no bound).
  // SYNC: the per-type field list here mirrors the emit side in ConfigToJson (above);
  // adding/removing a filter type or field requires updating both.
  auto type_str = fj.at("type").get<std::string>();
  if (type_str == "none") {
    f->type = LUMICE_FILTER_TYPE_NONE;
  } else if (type_str == "raypath") {
    f->type = LUMICE_FILTER_TYPE_RAYPATH;
    // Required (core: j.at("raypath")). Accepting it as absent used to produce an empty raypath
    // filter that matches nothing, for a config core would have refused outright.
    if (!fj.contains("raypath")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    if (!fj.at("raypath").is_array()) {
      return LUMICE_ERR_INVALID_VALUE;
    }
    const auto& rp = fj.at("raypath");
    f->raypath_count = static_cast<int>(rp.size());
    if (f->raypath_count > LUMICE_MAX_CONFIG_RAYPATH_LEN) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    for (int k = 0; k < f->raypath_count; k++) {
      f->raypath[k] = rp[k].get<int>();
    }
  } else if (type_str == "entry_exit") {
    f->type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
    f->ee_entry = (fj.contains("entry") && !fj.at("entry").is_null()) ? fj.at("entry").get<int>() : -1;
    f->ee_exit = (fj.contains("exit") && !fj.at("exit").is_null()) ? fj.at("exit").get<int>() : -1;
    f->ee_min_len = (fj.contains("min_len") && !fj.at("min_len").is_null()) ? fj.at("min_len").get<int>() : 1;
    f->ee_max_len = (fj.contains("max_len") && !fj.at("max_len").is_null()) ? fj.at("max_len").get<int>() : -1;
  } else if (type_str == "direction") {
    f->type = LUMICE_FILTER_TYPE_DIRECTION;
    f->dir_az = fj.at("az").get<float>();
    f->dir_el = fj.at("el").get<float>();
    f->dir_radii = fj.at("radii").get<float>();
  } else if (type_str == "crystal") {
    f->type = LUMICE_FILTER_TYPE_CRYSTAL;
    f->crystal_id = fj.at("crystal_id").get<int>();
  } else if (type_str == "complex") {
    // Placeholder: type is set here, but composition + composition_index are resolved by
    // the second pass in JsonToConfig (needs all simple filters parsed first so composition
    // cell ids can be validated). -1 marks "not yet resolved".
    f->type = LUMICE_FILTER_TYPE_COMPLEX;
    f->composition_index = -1;
  } else {
    return LUMICE_ERR_INVALID_VALUE;  // unknown type string
  }

  // Symmetry: common field for all types. "PBD" -> bitmask.
  f->symmetry = fj.contains("symmetry") ? SymmetryStringToBits(fj.at("symmetry").get<std::string>()) : 0;
  return LUMICE_OK;
}

// Parse the arm fields of one Color Predicate from JSON, mirroring ColorPredicateToJson.
// A ref JSON with no "type" key means match-all: sets predicate.type = UNSET, which the
// C-API commit path re-serializes as "no predicate fields on the wire", the same form
// core RaypathColorRef::from_json treats as NoneFilterParam / whole-crystal. The `symmetry`
// key is parsed at the tail and is independent of the arm type — including match-all, where
// (UNSET + non-default symmetry) is a legal state (task-356.2).
static LUMICE_ErrorCode JsonToColorPredicate(const nlohmann::json& j, LUMICE_ColorPredicate* p) {
  std::memset(p, 0, sizeof(*p));
  p->ee_entry = -1;
  p->ee_exit = -1;
  p->ee_min_len = 1;
  p->ee_max_len = -1;
  if (!j.contains("type")) {
    p->type = LUMICE_FILTER_TYPE_UNSET;  // match-all — falls through to shared symmetry tail.
  } else {
    auto type_str = j.at("type").get<std::string>();
    if (type_str == "none") {
      p->type = LUMICE_FILTER_TYPE_NONE;
    } else if (type_str == "raypath") {
      p->type = LUMICE_FILTER_TYPE_RAYPATH;
      if (j.contains("raypath") && j.at("raypath").is_array()) {
        const auto& rp = j.at("raypath");
        p->raypath_count = static_cast<int>(rp.size());
        if (p->raypath_count > LUMICE_MAX_CONFIG_RAYPATH_LEN) {
          return LUMICE_ERR_INVALID_CONFIG;
        }
        for (int k = 0; k < p->raypath_count; k++) {
          p->raypath[k] = rp[k].get<int>();
        }
      }
    } else if (type_str == "entry_exit") {
      p->type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
      p->ee_entry = (j.contains("entry") && !j.at("entry").is_null()) ? j.at("entry").get<int>() : -1;
      p->ee_exit = (j.contains("exit") && !j.at("exit").is_null()) ? j.at("exit").get<int>() : -1;
      p->ee_min_len = (j.contains("min_len") && !j.at("min_len").is_null()) ? j.at("min_len").get<int>() : 1;
      p->ee_max_len = (j.contains("max_len") && !j.at("max_len").is_null()) ? j.at("max_len").get<int>() : -1;
    } else if (type_str == "direction") {
      p->type = LUMICE_FILTER_TYPE_DIRECTION;
      p->dir_az = j.at("az").get<float>();
      p->dir_el = j.at("el").get<float>();
      p->dir_radii = j.at("radii").get<float>();
    } else if (type_str == "crystal") {
      p->type = LUMICE_FILTER_TYPE_CRYSTAL;
      p->crystal_id = j.at("crystal_id").get<int>();
    } else {
      // Anything else (including "complex") is not a legal color predicate.
      return LUMICE_ERR_INVALID_VALUE;
    }
  }
  // Symmetry: common field for all arms (including match-all). Missing key => 0 / kSymNone.
  p->symmetry = j.contains("symmetry") ? SymmetryStringToBits(j.at("symmetry").get<std::string>()) : 0;
  return LUMICE_OK;
}

// Parse a single color class from JSON (mirrors core ColorClassConfig::from_json). Missing
// fields fall back to core defaults: combine="any", visible=true, solo=false.
static LUMICE_ErrorCode JsonToColorClass(const nlohmann::json& j, LUMICE_ColorClass* cls) {
  std::memset(cls, 0, sizeof(*cls));
  cls->visible = 1;
  cls->solo = 0;
  cls->combine = LUMICE_COLOR_COMBINE_ANY;

  if (!j.contains("color") || !j.at("color").is_array() || j.at("color").size() != 3) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  const auto& jc = j.at("color");
  cls->color[0] = jc.at(0).get<float>();
  cls->color[1] = jc.at(1).get<float>();
  cls->color[2] = jc.at(2).get<float>();

  if (j.contains("combine")) {
    auto s = j.at("combine").get<std::string>();
    if (s == "any") {
      cls->combine = LUMICE_COLOR_COMBINE_ANY;
    } else if (s == "all") {
      cls->combine = LUMICE_COLOR_COMBINE_ALL;
    } else {
      return LUMICE_ERR_INVALID_VALUE;
    }
  }
  if (j.contains("visible")) {
    cls->visible = j.at("visible").get<bool>() ? 1 : 0;
  }
  if (j.contains("solo")) {
    cls->solo = j.at("solo").get<bool>() ? 1 : 0;
  }

  if (!j.contains("match") || !j.at("match").is_array()) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  const auto& match = j.at("match");
  if (static_cast<int>(match.size()) > LUMICE_MAX_CONFIG_COLOR_REFS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  cls->match_count = static_cast<int>(match.size());
  for (int k = 0; k < cls->match_count; k++) {
    const auto& jr = match[k];
    auto& ref = cls->match[k];
    if (!jr.contains("layer") || !jr.contains("crystal")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    ref.layer = jr.at("layer").get<int>();
    ref.crystal = jr.at("crystal").get<int>();
    auto err = JsonToColorPredicate(jr, &ref.predicate);
    if (err != LUMICE_OK) {
      return err;
    }
  }
  return LUMICE_OK;
}

// Parse the top-level "raypath_color" JSON (both bare-array and object forms) into
// ConfigScratch. Mirrors core RaypathColorConfig::from_json.
static LUMICE_ErrorCode JsonToRaypathColor(const nlohmann::json& j, ConfigScratch* out) {
  const nlohmann::json* classes_arr = nullptr;
  // Single-source default: track the core `kDefaultCompositeMode` (currently
  // "painter" per doc §4.8) so a bare-array config / object with no "mode"
  // field resolves to the SAME enum here as it does in core from_json.
  std::string mode_str = ns::kDefaultCompositeMode;
  if (j.is_array()) {
    classes_arr = &j;
  } else if (j.is_object()) {
    if (j.contains("mode")) {
      mode_str = j.at("mode").get<std::string>();
    }
    if (j.contains("classes") && j.at("classes").is_array()) {
      classes_arr = &j.at("classes");
    }
  } else {
    return LUMICE_ERR_INVALID_VALUE;
  }

  if (mode_str == "dominant") {
    out->raypath_color_mode = LUMICE_COLOR_MODE_DOMINANT;
  } else if (mode_str == "additive") {
    out->raypath_color_mode = LUMICE_COLOR_MODE_ADDITIVE;
  } else {
    // "painter" is the default; any other unknown mode degrades to painter here
    // to mirror core ParseCompositeMode's fallback (see doc §4.8).
    out->raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;
  }

  if (classes_arr == nullptr) {
    // Explicitly clear any prior allocation (Release is null-safe).
    ConfigReleaseColorClasses(out);
    return LUMICE_OK;
  }
  if (static_cast<int>(classes_arr->size()) > LUMICE_MAX_CONFIG_COLOR_CLASSES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  const int count = static_cast<int>(classes_arr->size());
  if (count == 0) {
    ConfigReleaseColorClasses(out);
    return LUMICE_OK;
  }
  // Implicit allocation: the caller cannot know the class count before parsing, so we allocate
  // on their behalf here. Ownership is
  // transferred to `out`; the caller must eventually call
  // ConfigReleaseColorClasses (or use a RAII guard).
  LUMICE_ColorClass* classes = ConfigCreateColorClasses(out, count);
  if (!classes) {
    // count is bounded above by LUMICE_MAX_CONFIG_COLOR_CLASSES; only OOM reaches here.
    return LUMICE_ERR_INVALID_CONFIG;
  }
  // Validate/fill each class. On mid-array failure release the allocation so the caller
  // does not observe a half-populated array (mirrors the spectrum_entries discipline).
  for (int i = 0; i < count; i++) {
    auto err = JsonToColorClass((*classes_arr)[i], &classes[i]);
    if (err != LUMICE_OK) {
      ConfigReleaseColorClasses(out);
      return err;
    }
  }
  return LUMICE_OK;
}

// Reference-integrity helpers for the scene block. Crystals and filters are parsed into `out`
// before the scene block is, so a scattering entry's ids can be checked against what was actually
// declared — the C-API-side equivalent of core's m.crystals_.at() / m.filters_.at() lookups.
static bool ConfigHasCrystal(const ConfigScratch& c, int id) {
  for (int i = 0; i < c.crystal_count; i++) {
    if (c.crystals[i].id == id) {
      return true;
    }
  }
  return false;
}

static bool ConfigHasFilter(const ConfigScratch& c, int id) {
  for (int i = 0; i < c.filter_count; i++) {
    if (c.filters[i].id == id) {
      return true;
    }
  }
  return false;
}

// Parse the JSON "scene" subsection (light source + simulation params) into a ConfigScratch.
// Named ...SceneParams (not JsonToScene) to avoid colliding, on sight, with the public opaque
// type LUMICE_Scene — this file-static parser has always been about the scene PARAMS block only.
static LUMICE_ErrorCode JsonToSceneParams(const nlohmann::json& scene, ConfigScratch* out) {
  // Light source: required, and so are its `type` / `altitude` / `spectrum` keys (core
  // LightSourceConfig::from_json reads all three with .at()). A `type` other than "sun" is
  // rejected rather than mirrored: core only logs and leaves its SunParam uninitialized.
  {
    if (!scene.contains("light_source")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    const auto& ls = scene.at("light_source");
    if (!ls.contains("type") || !ls.contains("altitude") || !ls.contains("spectrum")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    if (ls.at("type").get<std::string>() != "sun") {
      return LUMICE_ERR_INVALID_VALUE;
    }
    out->sun_altitude = ls.at("altitude").get<float>();
    if (ls.contains("azimuth")) {
      out->sun_azimuth = ls.at("azimuth").get<float>();
    }
    if (ls.contains("diameter")) {
      out->sun_diameter = ls.at("diameter").get<float>();
    }
    {
      const auto& sp = ls.at("spectrum");
      if (sp.is_string()) {
        out->spectrum = MapSpectrumString(sp.get<std::string>());
        if (!out->spectrum) {
          return LUMICE_ERR_INVALID_VALUE;
        }
        out->spectrum_count = 0;
      } else if (sp.is_array()) {
        const int count = static_cast<int>(sp.size());
        if (count > LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES) {
          return LUMICE_ERR_INVALID_CONFIG;
        }
        // Validate + fill every entry BEFORE publishing spectrum_count, so a mid-array failure returns
        // an error without leaving the struct claiming N entries with only some slots written (callers
        // that reuse a non-zeroed struct would otherwise read garbage). spectrum_count is the last write.
        out->spectrum_count = 0;
        for (int i = 0; i < count; i++) {
          const auto& e = sp[i];
          if (!e.is_object() || !e.contains("wavelength") || !e.contains("weight")) {
            return LUMICE_ERR_MISSING_FIELD;
          }
          out->spectrum_entries[i].wavelength = e.at("wavelength").get<float>();
          out->spectrum_entries[i].weight = e.at("weight").get<float>();
        }
        out->spectrum_count = count;
        out->spectrum = "D65";  // fallback string kept for downstream defaults
      } else {
        return LUMICE_ERR_INVALID_VALUE;
      }
    }
  }

  // Ray num: required (core: j_scene.at("ray_num")).
  if (!scene.contains("ray_num")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  {
    const auto& rn = scene.at("ray_num");
    if (rn.is_string() && rn.get<std::string>() == "infinite") {
      out->infinite = 1;
      out->ray_num = 0;
    } else if (rn.is_number()) {
      out->infinite = 0;
      out->ray_num = rn.get<LUMICE_RayCount>();
    } else {
      return LUMICE_ERR_INVALID_VALUE;
    }
  }

  // max_hits: required (core: j_scene.at("max_hits")). Its RANGE check stays single-source in
  // core and fires at commit, matching the geom_clock convention below.
  if (!scene.contains("max_hits")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  out->max_hits = scene.at("max_hits").get<int>();

  // geom_clock: lossless parse only, no range check (range {0}∪[1,64] is validated single-source
  // in core config_manager.cpp at commit, matching the ray_num/max_hits convention here).
  if (scene.contains("geom_clock")) {
    out->geom_clock = scene.at("geom_clock").get<int>();
  }

  // Scattering: required (core: j_scene.at("scattering")), and so is each layer's "entries"
  // array. Every entry must name an EXISTING crystal, and a referenced filter must exist too —
  // core resolves both through m.crystals_.at() / m.filters_.at(), which throw on a dangling id.
  // That check is only possible here because crystals and filters are parsed before the scene
  // block, the same order core uses.
  if (!scene.contains("scattering")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  {
    const auto& scat = scene.at("scattering");
    if (!scat.is_array()) {
      return LUMICE_ERR_INVALID_VALUE;
    }
    if (static_cast<int>(scat.size()) > LUMICE_MAX_CONFIG_SCATTER_LAYERS) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    out->scatter_count = static_cast<int>(scat.size());
    for (int i = 0; i < out->scatter_count; i++) {
      const auto& lj = scat[i];
      auto& layer = out->scattering[i];
      // "prob" is required, mirroring core's ParseScatteringInfo. This must be checked HERE and
      // not left to core: the CLI reaches core only via this function, which then re-serializes
      // the parsed struct through ConfigToJson — and that writer emits `prob` unconditionally,
      // so a missing key would be silently filled in before core ever saw it. Unlike the other
      // missing-field branches in this function (which report through the return code alone),
      // this one also logs: the message IS the migration guidance for a contract narrowing.
      if (!lj.contains("prob")) {
        LOG_ERROR(
            "JsonToSceneParams: scene.scattering[{}] is missing required field \"prob\" "
            "(multi-scattering probability). The historical default was 0.0; add \"prob\": 0 "
            "explicitly to keep that behavior.",
            i);
        return LUMICE_ERR_MISSING_FIELD;
      }
      layer.probability = lj.at("prob").get<float>();
      if (!lj.contains("entries") || !lj.at("entries").is_array()) {
        return LUMICE_ERR_MISSING_FIELD;
      }
      const auto& entries = lj.at("entries");
      if (static_cast<int>(entries.size()) > LUMICE_MAX_CONFIG_SCATTER_ENTRIES) {
        return LUMICE_ERR_INVALID_CONFIG;
      }
      layer.entry_count = static_cast<int>(entries.size());
      for (int k = 0; k < layer.entry_count; k++) {
        const auto& ej = entries[k];
        auto& e = layer.entries[k];
        if (!ej.contains("crystal")) {
          return LUMICE_ERR_MISSING_FIELD;
        }
        e.crystal_id = ej.at("crystal").get<int>();
        if (!ConfigHasCrystal(*out, e.crystal_id)) {
          return LUMICE_ERR_INVALID_CONFIG;
        }
        // proportion defaults to 100 (core ScatteringSetting's crystal_proportion_); the zeroed
        // struct would mean "this crystal never gets picked".
        e.proportion = 100.0f;
        if (ej.contains("proportion")) {
          e.proportion = ej.at("proportion").get<float>();
        }
        e.filter_id = -1;
        if (ej.contains("filter")) {
          e.filter_id = ej.at("filter").get<int>();
          if (!ConfigHasFilter(*out, e.filter_id)) {
            return LUMICE_ERR_INVALID_CONFIG;
          }
        }
      }
    }
  }
  return LUMICE_OK;
}

// Inverse of MapVisibleFromCApi. Total over the core enumeration (no default arm) so adding a
// visibility range to core breaks the build here instead of decoding to a wrong C API constant —
// same fail-loud contract as MapLensTypeToCApi.
static int MapVisibleToCApi(ns::RenderConfig::VisibleRange visible) {
  switch (visible) {
    case ns::RenderConfig::kUpper:
      return LUMICE_VISIBLE_UPPER;
    case ns::RenderConfig::kLower:
      return LUMICE_VISIBLE_LOWER;
    case ns::RenderConfig::kFull:
      return LUMICE_VISIBLE_FULL;
  }
  throw std::invalid_argument("unmapped core VisibleRange: " + std::to_string(static_cast<int>(visible)));
}

// Inverse of MapEvModeFromCApi. Total over the core enumeration (no default arm), same fail-loud
// contract as MapLensTypeToCApi / MapVisibleToCApi.
static int MapEvModeToCApi(ns::RenderConfig::EvMode ev_mode) {
  switch (ev_mode) {
    case ns::RenderConfig::kRelative:
      return LUMICE_EV_MODE_RELATIVE;
    case ns::RenderConfig::kAbsolute:
      return LUMICE_EV_MODE_ABSOLUTE;
  }
  throw std::invalid_argument("unmapped core EvMode: " + std::to_string(static_cast<int>(ev_mode)));
}

// The two enum-valued renderer fields need a string pre-check: NLOHMANN_JSON_SERIALIZE_ENUM maps
// an unrecognized value to the FIRST table entry ("linear" / "upper"), so a typo'd projection
// would be silently misread rather than reported. Same deliberate exception to "align with core"
// already taken for the distribution / crystal type strings above.
static bool IsKnownLensTypeString(const std::string& s) {
  return s == "linear" || s == "fisheye_equal_area" || s == "fisheye_equidistant" || s == "fisheye_stereographic" ||
         s == "dual_fisheye_equal_area" || s == "dual_fisheye_equidistant" || s == "dual_fisheye_stereographic" ||
         s == "rectangular" || s == "fisheye_orthographic" || s == "dual_fisheye_orthographic" || s == "globe";
}

static bool IsKnownVisibleString(const std::string& s) {
  return s == "upper" || s == "lower" || s == "full";
}

// Same reason as the two above: core's EvMode table maps an unrecognized string to its first
// entry ("relative"), so without this pre-check a typo'd mode would be silently misread as the
// default instead of reported.
static bool IsKnownEvModeString(const std::string& s) {
  return s == "relative" || s == "absolute";
}

// Decode one field with core's own from_json (single source for the f->fov trigonometry, the
// MaxFov range check and the per-field optional/default logic), mapping nlohmann's exception
// taxonomy onto C API error codes: a missing key (out_of_range id 403) is MISSING_FIELD,
// everything else (type mismatch, out-of-range fov, too-short focal length) is INVALID_VALUE.
template <typename T>
static LUMICE_ErrorCode DecodeCoreField(const nlohmann::json& j, T& dst) {
  try {
    j.get_to(dst);
  } catch (const nlohmann::json::out_of_range& e) {
    return e.id == 403 ? LUMICE_ERR_MISSING_FIELD : LUMICE_ERR_INVALID_VALUE;
  } catch (const nlohmann::json::exception&) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  return LUMICE_OK;
}

// Decode one grid-line array ("grid.angular_dist" / "grid.elevation" / "grid.longitude") into the
// fixed-capacity C array.
static LUMICE_ErrorCode JsonToGridLines(const nlohmann::json& arr_j, LUMICE_GridLine* out, int* out_count) {
  if (!arr_j.is_array()) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  if (static_cast<int>(arr_j.size()) > LUMICE_MAX_CONFIG_GRID_LINES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  *out_count = static_cast<int>(arr_j.size());
  for (int k = 0; k < *out_count; k++) {
    ns::GridLineParam g;
    const LUMICE_ErrorCode err = DecodeCoreField(arr_j[k], g);
    if (err != LUMICE_OK) {
      return err;
    }
    out[k].value = g.value_;
    out[k].width = g.width_;
    out[k].opacity = g.opacity_;
    out[k].color[0] = g.color_[0];
    out[k].color[1] = g.color_[1];
    out[k].color[2] = g.color_[2];
  }
  return LUMICE_OK;
}

static LUMICE_ErrorCode JsonToRenderers(const nlohmann::json& render_arr, ConfigScratch* out) {
  if (static_cast<int>(render_arr.size()) > LUMICE_MAX_CONFIG_RENDERERS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  out->renderer_count = static_cast<int>(render_arr.size());
  for (int i = 0; i < out->renderer_count; i++) {
    const auto& rj = render_arr[i];
    auto& r = out->renderers[i];
    // id and resolution are both required (core ParseRenderConfig: j.at("id") / j.at("resolution")).
    if (!rj.contains("id") || !rj.contains("resolution")) {
      return LUMICE_ERR_MISSING_FIELD;
    }
    r.id = rj.at("id").get<int>();
    if (!rj.at("resolution").is_array() || rj.at("resolution").size() != 2) {
      return LUMICE_ERR_INVALID_VALUE;
    }
    r.resolution_w = rj.at("resolution")[0].get<int>();
    r.resolution_h = rj.at("resolution")[1].get<int>();
    // intensity_factor defaults to 1.0 in core RenderConfig; the zeroed struct would mean a
    // zero-brightness renderer.
    r.intensity_factor = 1.0f;
    if (rj.contains("intensity_factor")) {
      r.intensity_factor = rj.at("intensity_factor").get<float>();
    }
    if (rj.contains("overlap")) {
      r.overlap = std::max(0.0f, rj.at("overlap").get<float>());
    }
    // Mirrors core RenderConfig::ev_mode_'s member initializer (kRelative); the zeroed struct
    // already holds it, but stating it keeps this decoder's defaults readable in one place.
    r.ev_mode = LUMICE_EV_MODE_RELATIVE;
    if (rj.contains("ev_mode")) {
      if (!rj.at("ev_mode").is_string() || !IsKnownEvModeString(rj.at("ev_mode").get<std::string>())) {
        return LUMICE_ERR_INVALID_VALUE;
      }
      auto ev_mode = ns::RenderConfig::kRelative;
      const LUMICE_ErrorCode err = DecodeCoreField(rj.at("ev_mode"), ev_mode);
      if (err != LUMICE_OK) {
        return err;
      }
      r.ev_mode = MapEvModeToCApi(ev_mode);
    }

    // ---- Fields the struct gained in v4.11 (previously parsed and thrown away) ----
    // Every default below mirrors the corresponding core RenderConfig member initializer
    // (config/render_config.hpp), and every present-key decode runs through core's own from_json.
    ns::LensParam lens{ ns::LensParam::kLinear, 90.0f };
    if (rj.contains("lens")) {
      const auto& lens_j = rj.at("lens");
      if (!lens_j.is_object()) {
        return LUMICE_ERR_INVALID_VALUE;
      }
      // Core's LensParam::from_json does j.at("type") — the key is required, not optional.
      if (!lens_j.contains("type")) {
        return LUMICE_ERR_MISSING_FIELD;
      }
      if (!lens_j.at("type").is_string() || !IsKnownLensTypeString(lens_j.at("type").get<std::string>())) {
        return LUMICE_ERR_INVALID_VALUE;
      }
      const LUMICE_ErrorCode err = DecodeCoreField(lens_j, lens);
      if (err != LUMICE_OK) {
        return err;
      }
    }
    r.lens_type = MapLensTypeToCApi(lens.type_);
    r.lens_fov = lens.fov_;

    r.lens_shift[0] = 0;
    r.lens_shift[1] = 0;
    if (rj.contains("lens_shift")) {
      const LUMICE_ErrorCode err = DecodeCoreField(rj.at("lens_shift"), r.lens_shift);
      if (err != LUMICE_OK) {
        return err;
      }
    }

    ns::ViewParam view{};
    if (rj.contains("view")) {
      const LUMICE_ErrorCode err = DecodeCoreField(rj.at("view"), view);
      if (err != LUMICE_OK) {
        return err;
      }
    }
    r.view_azimuth = view.az_;
    r.view_elevation = view.el_;
    r.view_roll = view.ro_;

    r.visible = LUMICE_VISIBLE_UPPER;
    if (rj.contains("visible")) {
      if (!rj.at("visible").is_string() || !IsKnownVisibleString(rj.at("visible").get<std::string>())) {
        return LUMICE_ERR_INVALID_VALUE;
      }
      auto visible = ns::RenderConfig::kUpper;
      const LUMICE_ErrorCode err = DecodeCoreField(rj.at("visible"), visible);
      if (err != LUMICE_OK) {
        return err;
      }
      r.visible = MapVisibleToCApi(visible);
    }

    // Absent key = no clip, which is what a config predating the field means. Twin of core's
    // ParseRenderConfig.
    r.front = 0;
    if (rj.contains("front")) {
      bool front = false;
      const LUMICE_ErrorCode err = DecodeCoreField(rj.at("front"), front);
      if (err != LUMICE_OK) {
        return err;
      }
      r.front = front ? 1 : 0;
    }

    // Twin of core's warning in config_manager.cpp::ParseRenderConfig; see the rationale there.
    if (rj.contains("background_color")) {
      ILOG_WARN(ns::GetGlobalLogger(),
                "render[id={}]: unknown key \"background_color\" is ignored; the background color key is "
                "\"background\" (sRGB triple)",
                r.id);
    }
    // The JSON key is sRGB (what a color picker shows); LUMICE_RenderParam::background is linear
    // (what PostSnapshot's additive blend needs) — see the field's comment in lumice.h. The default
    // needs no conversion: 0 is a fixed point of both directions. Twin of the encode side in
    // RendererToJson, and of core's own conversion in config_manager.cpp::ParseRenderConfig.
    r.background[0] = r.background[1] = r.background[2] = 0.0f;
    if (rj.contains("background")) {
      const LUMICE_ErrorCode err = DecodeCoreField(rj.at("background"), r.background);
      if (err != LUMICE_OK) {
        return err;
      }
      for (float& c : r.background) {
        c = ns::SrgbToLinear(c);
      }
    }

    // Core's default is the {-1,-1,-1} "use the natural spectral color" sentinel, NOT black.
    r.ray_color[0] = r.ray_color[1] = r.ray_color[2] = -1.0f;
    if (rj.contains("ray_color")) {
      const LUMICE_ErrorCode err = DecodeCoreField(rj.at("ray_color"), r.ray_color);
      if (err != LUMICE_OK) {
        return err;
      }
    }

    r.angular_dist_count = 0;
    r.elevation_grid_count = 0;
    r.longitude_grid_count = 0;
    r.horizon = 0;  // core RenderConfig::horizon_ defaults to false
    // The marker block's defaults come from core's struct rather than being spelled again here,
    // and they are written BEFORE the "grid" branch so a document with no key at all lands on the
    // same four values ParseRenderConfig leaves. Zeroing them instead would be a real divergence
    // between the two decoders, not a harmless one: three of the four defaults are non-zero, and
    // the parity gate compares whole RenderConfigs.
    {
      const ns::ZenithNadirParam kZenithNadirDefaults;
      r.zenith_nadir = kZenithNadirDefaults.enabled_ ? 1 : 0;
      r.zenith_nadir_radius_px = kZenithNadirDefaults.radius_px_;
      r.zenith_nadir_opacity = kZenithNadirDefaults.opacity_;
      std::copy(std::begin(kZenithNadirDefaults.color_), std::end(kZenithNadirDefaults.color_),
                std::begin(r.zenith_nadir_color));
    }
    if (rj.contains("grid")) {
      const auto& gj = rj.at("grid");
      if (!gj.is_object()) {
        return LUMICE_ERR_INVALID_VALUE;
      }
      // "central" is the pre-rename spelling of "angular_dist"; read it as an alias with the new
      // key winning, exactly as ParseRenderConfig does. The two decoders have to agree — that is
      // what test_json_parser_parity.cpp checks — so this branch and that one stay identical in
      // shape. Only the new key is ever written (RendererToJson above).
      if (gj.contains("angular_dist")) {
        const LUMICE_ErrorCode err = JsonToGridLines(gj.at("angular_dist"), r.angular_dist, &r.angular_dist_count);
        if (err != LUMICE_OK) {
          return err;
        }
      } else if (gj.contains("central")) {
        const LUMICE_ErrorCode err = JsonToGridLines(gj.at("central"), r.angular_dist, &r.angular_dist_count);
        if (err != LUMICE_OK) {
          return err;
        }
      }
      if (gj.contains("elevation")) {
        const LUMICE_ErrorCode err = JsonToGridLines(gj.at("elevation"), r.elevation_grid, &r.elevation_grid_count);
        if (err != LUMICE_OK) {
          return err;
        }
      }
      if (gj.contains("longitude")) {
        const LUMICE_ErrorCode err = JsonToGridLines(gj.at("longitude"), r.longitude_grid, &r.longitude_grid_count);
        if (err != LUMICE_OK) {
          return err;
        }
      }
      if (gj.contains("horizon")) {
        bool outline = true;
        const LUMICE_ErrorCode err = DecodeCoreField(gj.at("horizon"), outline);
        if (err != LUMICE_OK) {
          return err;
        }
        r.horizon = outline ? 1 : 0;
      }
      if (gj.contains("zenith_nadir")) {
        // Seeded with the defaults above so a PARTIAL object keeps them for the keys it omits,
        // which is what core's from_json does with the same input.
        ns::ZenithNadirParam zn;
        zn.enabled_ = r.zenith_nadir != 0;
        zn.radius_px_ = r.zenith_nadir_radius_px;
        zn.opacity_ = r.zenith_nadir_opacity;
        std::copy(std::begin(r.zenith_nadir_color), std::end(r.zenith_nadir_color), std::begin(zn.color_));
        const LUMICE_ErrorCode err = DecodeCoreField(gj.at("zenith_nadir"), zn);
        if (err != LUMICE_OK) {
          return err;
        }
        r.zenith_nadir = zn.enabled_ ? 1 : 0;
        r.zenith_nadir_radius_px = zn.radius_px_;
        r.zenith_nadir_opacity = zn.opacity_;
        std::copy(std::begin(zn.color_), std::end(zn.color_), std::begin(r.zenith_nadir_color));
      }
    }
  }
  return LUMICE_OK;
}

// Resolve a complex filter's "composition" JSON into a ConfigScratch compositions[] slot and
// set f->composition_index. Validates reference integrity: each term id must reference an
// existing SIMPLE (non-complex) filter, and clause/term/composition counts respect the ABI
// bounds. Must run after all simple filters are parsed (two-pass, mirrors config_manager.cpp).
static LUMICE_ErrorCode JsonToComplexComposition(const nlohmann::json& fj, ConfigScratch* out, LUMICE_FilterParam* f) {
  if (!fj.contains("composition") || !fj.at("composition").is_array()) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  const auto& cmp = fj.at("composition");
  // v4.9 (task-host-abi-cpu-caps): build the full (term_counts, term_ids) triple on the
  // stack first, then commit atomically via LUMICE_CompositionSetClauses AFTER all
  // validation succeeds. This also removes the pre-v4.9 "composition_count++ before
  // per-clause validation" partial-write hazard (§3.5 in the plan).
  const int clause_count = static_cast<int>(cmp.size());
  if (out->composition_count >= LUMICE_MAX_CONFIG_COMPLEX || clause_count > LUMICE_MAX_CONFIG_CLAUSES) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  std::vector<int> term_counts_vec;
  std::vector<int> term_ids_vec;
  term_counts_vec.reserve(static_cast<size_t>(clause_count));
  for (int cl = 0; cl < clause_count; cl++) {
    const auto& clause = cmp[cl];
    // A clause is either a bare id (1-term) or an array of ids (matches core to_json).
    if (clause.is_array()) {
      const int tn = static_cast<int>(clause.size());
      if (tn > LUMICE_MAX_CONFIG_TERMS) {
        return LUMICE_ERR_INVALID_CONFIG;
      }
      term_counts_vec.push_back(tn);
      for (int t = 0; t < tn; t++) {
        term_ids_vec.push_back(clause[t].get<int>());
      }
    } else {
      term_counts_vec.push_back(1);
      term_ids_vec.push_back(clause.get<int>());
    }
  }
  // Reference integrity: each term must name an existing SIMPLE filter (dangling ids and
  // references to a complex filter are rejected — the latter matches core semantics, which
  // only allow SimpleFilterParam in a composition, so cycles are impossible by construction).
  for (int ref_id : term_ids_vec) {
    bool found_simple = false;
    for (int k = 0; k < out->filter_count; k++) {
      if (out->filters[k].id == ref_id && out->filters[k].type != LUMICE_FILTER_TYPE_COMPLEX) {
        found_simple = true;
        break;
      }
    }
    if (!found_simple) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
  }
  const int comp_idx = out->composition_count;
  LUMICE_ComplexComposition* comp = &out->compositions[comp_idx];
  auto err = LUMICE_CompositionSetClauses(comp, clause_count, term_counts_vec.data(), term_ids_vec.data());
  if (err != LUMICE_OK) {
    return err;
  }
  // Publish the composition to the config only after Set succeeds — matches the "advance
  // count last" pattern already used elsewhere in this file (e.g. spectrum_entries).
  out->composition_count = comp_idx + 1;
  f->composition_index = comp_idx;
  return LUMICE_OK;
}

static LUMICE_ErrorCode JsonToConfig(const nlohmann::json& root, ConfigScratch* out) {
  // v4.8: raypath_color is an owning heap pointer. If `out` was reused across two Parse
  // calls, memset would clobber the pointer without freeing it — release first so the
  // memset that follows sees a defensibly-null field. Release is null-safe / idempotent.
  ConfigReleaseColorClasses(out);
  // v4.9 (task-host-abi-cpu-caps): compositions[i].term_ids/term_counts are also owning
  // heap pointers — same memset-would-leak hazard as raypath_color; release first.
  ConfigReleaseCompositions(out);
  std::memset(out, 0, sizeof(ConfigScratch));
  out->spectrum = "D65";  // Safe default (memset leaves nullptr)

  // Crystals (required)
  if (!root.contains("crystal") || !root.at("crystal").is_array()) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  const auto& crystals = root.at("crystal");
  if (static_cast<int>(crystals.size()) > LUMICE_MAX_CONFIG_CRYSTALS) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  out->crystal_count = static_cast<int>(crystals.size());
  for (int i = 0; i < out->crystal_count; i++) {
    auto err = JsonToCrystal(crystals[i], &out->crystals[i]);
    if (err != LUMICE_OK) {
      return err;
    }
  }

  // Filters: the key is required (core iterates j.at("filter")), though an empty array is fine.
  if (!root.contains("filter") || !root.at("filter").is_array()) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  {
    const auto& filters = root.at("filter");
    if (static_cast<int>(filters.size()) > LUMICE_MAX_CONFIG_FILTERS) {
      return LUMICE_ERR_INVALID_CONFIG;
    }
    out->filter_count = static_cast<int>(filters.size());
    // Two-pass (mirrors core config_manager.cpp): pass 1 parses simple filters and marks
    // complex ones as placeholders; pass 2 resolves complex compositions once every simple
    // filter's id/type is known (needed to validate composition references).
    for (int i = 0; i < out->filter_count; i++) {
      auto err = JsonToFilter(filters[i], &out->filters[i]);
      if (err != LUMICE_OK) {
        return err;
      }
    }
    for (int i = 0; i < out->filter_count; i++) {
      if (out->filters[i].type != LUMICE_FILTER_TYPE_COMPLEX) {
        continue;
      }
      auto err = JsonToComplexComposition(filters[i], out, &out->filters[i]);
      if (err != LUMICE_OK) {
        return err;
      }
    }
  }

  // Scene (required)
  if (!root.contains("scene")) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  auto err = JsonToSceneParams(root.at("scene"), out);
  if (err != LUMICE_OK) {
    return err;
  }

  // Renderers: required (core iterates j.at("render")).
  if (!root.contains("render") || !root.at("render").is_array()) {
    return LUMICE_ERR_MISSING_FIELD;
  }
  err = JsonToRenderers(root.at("render"), out);
  if (err != LUMICE_OK) {
    return err;
  }

  // Raypath color classes (optional, Design 2 / task-342.2)
  if (root.contains("raypath_color")) {
    err = JsonToRaypathColor(root.at("raypath_color"), out);
    if (err != LUMICE_OK) {
      return err;
    }
  }

  return LUMICE_OK;
}


// Internal JSON-string -> ConfigScratch reader. Declared in c_api_internal.hpp so the
// parser-parity / strictness tests can drive it directly; production code reaches the same
// validator through JsonToScene below.
LUMICE_ErrorCode ParseConfigString(const char* json_str, ConfigScratch* out) {
  if (!json_str || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  try {
    auto root = nlohmann::json::parse(json_str);
    return JsonToConfig(root, out);
  } catch (const nlohmann::json::parse_error&) {
    return LUMICE_ERR_INVALID_JSON;
  } catch (const std::exception&) {
    // Catches nlohmann::json::exception (malformed/out-of-range values) AND std::invalid_argument
    // from JsonToConfig's decode-direction Map*ToCApi calls (unmapped core enumerator) — both are
    // "the document decoded to something invalid", not a parse-syntax error.
    return LUMICE_ERR_INVALID_VALUE;
  }
}


// ---------- Serialization: decoupled from commit (JSON -> new Scene) ----------
// JsonToScene reuses the established JsonToConfig validator (single source of truth — no parallel
// JSON reader that could drift from it) by parsing into a temporary ConfigScratch, then re-encodes
// that struct via ConfigToJson into the new handle's root. It NEVER touches LUMICE_Server: this is
// the serialization half of the handle API, deliberately independent of commit/re-sim.
//
// This double hop (text -> ConfigScratch -> JSON root) is the known, deliberate technical debt
// recorded at ConfigScratch's declaration in c_api_internal.hpp — the alternative was rewriting a
// validated parser, a far larger risk than one re-encode on a non-hot path.
//
// The temporary ConfigScratch is heap-allocated (it is large; this scrum's history includes a real
// 512 KB stack overflow from an oversized on-stack copy of this struct). Its owning heap fields
// (raypath_color, compositions[].term_ids/term_counts) are freed on EVERY return path by the
// custom deleter — JsonToConfig may leave them partially allocated when it fails mid-parse (same
// hazard JsonToConfig itself guards against with its memset-before-Release entry).
namespace {
struct ConfigDeleter {
  void operator()(ConfigScratch* cfg) const {
    if (cfg) {
      ConfigReleaseColorClasses(cfg);
      ConfigReleaseCompositions(cfg);
      delete cfg;
    }
  }
};
}  // namespace

static LUMICE_ErrorCode JsonToScene(const nlohmann::json& root, LUMICE_Scene** out_scene) {
  *out_scene = nullptr;  // contract: *out_scene is NULL on any failure, no handle to Destroy
  std::unique_ptr<ConfigScratch, ConfigDeleter> tmp(new ConfigScratch());
  auto err = JsonToConfig(root, tmp.get());
  if (err != LUMICE_OK) {
    return err;
  }
  try {
    *out_scene = new LUMICE_Scene_{ ConfigToJson(*tmp) };
  } catch (const std::exception& e) {
    LOG_ERROR("LUMICE_SceneFromJson: failed to re-encode parsed config: {}", e.what());
    return LUMICE_ERR_INVALID_CONFIG;
  }
  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_SceneFromJson(const char* json_str, LUMICE_Scene** out_scene) {
  if (out_scene) {
    *out_scene = nullptr;
  }
  if (!json_str || !out_scene) {
    return LUMICE_ERR_NULL_ARG;
  }

  try {
    auto root = nlohmann::json::parse(json_str);
    return JsonToScene(root, out_scene);
  } catch (const nlohmann::json::parse_error&) {
    return LUMICE_ERR_INVALID_JSON;
  } catch (const std::exception&) {
    // See ParseConfigString for why this is std::exception, not nlohmann::json::exception:
    // JsonToScene's inner JsonToConfig call runs the same decode-direction Map*ToCApi throws.
    return LUMICE_ERR_INVALID_VALUE;
  }
}


LUMICE_ErrorCode LUMICE_SceneFromJsonFile(const char* filename, LUMICE_Scene** out_scene) {
  if (out_scene) {
    *out_scene = nullptr;
  }
  if (!filename || !out_scene) {
    return LUMICE_ERR_NULL_ARG;
  }

  std::ifstream file(lumice::PathFromU8(filename));
  if (!file.is_open()) {
    return LUMICE_ERR_FILE_NOT_FOUND;
  }

  try {
    auto root = nlohmann::json::parse(file);
    return JsonToScene(root, out_scene);
  } catch (const nlohmann::json::parse_error&) {
    return LUMICE_ERR_INVALID_JSON;
  } catch (const std::exception&) {
    // See ParseConfigString for why this is std::exception, not nlohmann::json::exception.
    return LUMICE_ERR_INVALID_VALUE;
  }
}


// =============== Scene commit ===============
// Handle->core commit, and since v4.12 the only commit entry point in the API. It carries no
// bounds-check prologue on purpose: the removed struct path needed one because a caller could
// hand-fill the wide C struct with arbitrary counts/pointers, a state a Scene cannot reach —
// every Add*/Set* validated its own input at call time, so re-checking here would be dead code.
// scene->root and ConfigToJson's output are the same encoding (they share the per-section encode
// helpers), which is what lets JsonToScene hand this function a document it produced by the
// ConfigScratch route without any behavioral difference.
LUMICE_ErrorCode LUMICE_CommitScene(LUMICE_Server* server, const LUMICE_Scene* scene, int* out_reused) {
  if (!server || !scene) {
    return LUMICE_ERR_NULL_ARG;
  }
  return CommitJsonToServer(server, scene->root, out_reused, "scene");
}


// =============== Results ===============
// LUMICE_ResultFrame_ wraps a share of one published, immutable server-side snapshot.
// Lifecycle mirrors LUMICE_Scene_ (new/delete); the shared_ptr inside is what makes the
// borrow safe — the buffers a caller reads out of the frame cannot be freed or rewritten
// while it holds one.
struct LUMICE_ResultFrame_ {
  std::shared_ptr<const ns::ResultFrame> frame_;
};

LUMICE_ErrorCode LUMICE_AcquireResultFrame(LUMICE_Server* server, LUMICE_ResultFrame** out_frame) {
  if (!server || !out_frame) {
    return LUMICE_ERR_NULL_ARG;
  }
  *out_frame = new LUMICE_ResultFrame_{ server->server_->AcquireResultFrame() };
  return LUMICE_OK;
}


void LUMICE_ReleaseResultFrame(LUMICE_ResultFrame* frame) {
  delete frame;  // delete nullptr is a no-op (NULL-safe, same as LUMICE_DestroyServer)
}


LUMICE_ErrorCode LUMICE_FrameGetRawXyz(const LUMICE_ResultFrame* frame, LUMICE_RawXyzResult* out, int max_count) {
  if (!frame || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  const auto& results = frame->frame_->xyz_results_;
  int count = static_cast<int>(results.size());
  if (count > max_count) {
    count = max_count;
  }

  for (int i = 0; i < count; i++) {
    out[i].renderer_id = results[i].renderer_id_;
    out[i].img_width = results[i].img_width_;
    out[i].img_height = results[i].img_height_;
    out[i].xyz_buffer = results[i].xyz_buffer_;
    out[i].snapshot_intensity = results[i].snapshot_intensity_;
    out[i].intensity_factor = results[i].intensity_factor_;
    out[i].has_valid_data = results[i].has_valid_data_ ? 1 : 0;
    out[i].snapshot_generation = results[i].snapshot_generation_;
    out[i].effective_pixels = results[i].effective_pixels_;
    out[i].emitted_energy = results[i].snapshot_emitted_energy_;
    out[i].epoch = results[i].epoch_;
  }

  // Sentinel: see doc/capi-lifecycle-architecture.md §5.2 (fix: 5287efe).
  if (count < max_count) {
    std::memset(&out[count], 0, sizeof(LUMICE_RawXyzResult));
  }

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_FrameGetComposite(const LUMICE_ResultFrame* frame, LUMICE_RenderResult* out, int max_count) {
  if (!frame || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  const auto& results = frame->frame_->composite_results_;
  int count = static_cast<int>(results.size());
  if (count > max_count) {
    count = max_count;
  }

  for (int i = 0; i < count; i++) {
    out[i].renderer_id = results[i].renderer_id_;
    out[i].img_width = results[i].w_;
    out[i].img_height = results[i].h_;
    out[i].img_buffer = results[i].rgb_ ? results[i].rgb_->data() : nullptr;
    // Composite path — participating-classes union P99 (auto-EV anchor).
    out[i].composite_p99_y = results[i].p99_y_;
  }

  // Sentinel: see doc/capi-lifecycle-architecture.md §5.2 (fix: 5287efe).
  if (count < max_count) {
    std::memset(&out[count], 0, sizeof(LUMICE_RenderResult));
  }

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_FrameGetRender(const LUMICE_ResultFrame* frame, LUMICE_RenderResult* out, int max_count) {
  if (!frame || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  const auto& results = frame->frame_->render_results_;
  int count = static_cast<int>(results.size());
  if (count > max_count) {
    count = max_count;
  }

  for (int i = 0; i < count; i++) {
    out[i].renderer_id = results[i].renderer_id_;
    out[i].img_width = results[i].img_width_;
    out[i].img_height = results[i].img_height_;
    out[i].img_buffer = results[i].img_buffer_;
    // Mono path — composite_p99_y is composite-only; leave at 0.
    out[i].composite_p99_y = 0.0f;
  }

  // Sentinel: see doc/capi-lifecycle-architecture.md §5.2 (fix: 5287efe).
  if (count < max_count) {
    std::memset(&out[count], 0, sizeof(LUMICE_RenderResult));
  }

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_FrameGetStats(const LUMICE_ResultFrame* frame, LUMICE_StatsResult* out) {
  if (!frame || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  const auto& stats = frame->frame_->stats_result_;
  if (stats.has_value()) {
    out->ray_seg_num = stats->ray_seg_num_;
    out->sim_ray_num = stats->sim_ray_num_;
    out->crystal_num = stats->crystal_num_;
    out->orientation_num = stats->orientation_num_;
  } else {
    std::memset(out, 0, sizeof(LUMICE_StatsResult));
  }

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_GetSimRayCount(LUMICE_Server* server, LUMICE_RayCount* out) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }
  // Cheap O(1) live ray-count read — no snapshot / no render. For progress polling that
  // needs sim_ray_num every iteration without paying the per-poll DoSnapshot/sRGB render
  // cost of materializing a result frame.
  *out = static_cast<LUMICE_RayCount>(server->server_->GetLiveSimRayCount());
  return LUMICE_OK;
}


// =============== State & Control ===============
// QueryServerState is a PROJECTION of the single-source lifecycle truth
// (GetSimLifecycle): RUNNING -> RUNNING; IDLE | COMPLETED -> IDLE. This keeps the
// historical running/idle binary intact for the existing call sites while
// GetSimLifecycle owns the authoritative completed-vs-idle distinction (I1).
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  if (server->server_->GetSimLifecycle() == ns::SimLifecycle::kRunning) {
    *out = LUMICE_SERVER_RUNNING;
  } else {
    *out = LUMICE_SERVER_IDLE;
  }

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_GetSimLifecycle(LUMICE_Server* server, LUMICE_SimLifecycleResult* out) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  // -Wswitch: exhaustive over SimLifecycle, no `default:` — a new enum value
  // forces this mapping to add an explicit case.
  switch (server->server_->GetSimLifecycle()) {
    case ns::SimLifecycle::kIdle:
      out->lifecycle = LUMICE_LIFECYCLE_IDLE;
      break;
    case ns::SimLifecycle::kRunning:
      out->lifecycle = LUMICE_LIFECYCLE_RUNNING;
      break;
    case ns::SimLifecycle::kCompleted:
      out->lifecycle = LUMICE_LIFECYCLE_COMPLETED;
      break;
  }
  out->epoch = static_cast<unsigned long long>(server->server_->CommittedEpoch());

  return LUMICE_OK;
}


LUMICE_ErrorCode LUMICE_GetDrainStatus(LUMICE_Server* server, LUMICE_DrainResult* out) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }

  // Order matters: read the drained epoch BEFORE the current one. Sampled the
  // other way round, a CommitConfig landing between the two reads would pair an
  // old current_epoch with a drained_epoch the commit has already outrun, and
  // the caller's equality test would report the SUPERSEDED epoch as drained.
  // This order can only ever under-report (drained trails current), which the
  // caller's poll loop resolves on its next iteration.
  out->drained_epoch = static_cast<unsigned long long>(server->server_->DrainedEpoch());
  out->current_epoch = static_cast<unsigned long long>(server->server_->CommittedEpoch());

  return LUMICE_OK;
}


// Readback of the color-degrade counters. component_overflow_count is the
// synchronous host-side count written inside CommitConfig; the three GPU-only
// caps (symmetry-group / OR-summand / color-class) are published asynchronously
// from the worker's first batch (server ConsumeData) and read atomically here,
// so a GUI poll tick picks them up after DoRun. task-color-degrade-gui-surfacing.
LUMICE_ErrorCode LUMICE_GetColorOverflowInfo(LUMICE_Server* server, LUMICE_ColorOverflowInfo* out) {
  if (!server || !out) {
    return LUMICE_ERR_NULL_ARG;
  }
  out->component_overflow_count = static_cast<int>(server->server_->GetLastColorComponentOverflowCount());
  const lumice::ColorDegradeCounts degrade = server->server_->GetLastColorDegradeCounts();
  out->symmetry_group_overflow_count = static_cast<int>(degrade.symmetry_group_overflow);
  out->or_summand_overflow_count = static_cast<int>(degrade.or_summand_overflow);
  out->color_class_overflow_count = static_cast<int>(degrade.color_class_overflow);
  return LUMICE_OK;
}


void LUMICE_StopServer(LUMICE_Server* server) {
  if (!server) {
    return;
  }

  server->server_->Stop();
}


void LUMICE_SetPreferredBackend(LUMICE_Server* server, int backend) {
  if (!server) {
    return;
  }
  // C-API boundary cast: the public ABI keeps int; everything inside uses
  // BackendKind. Unknown int values map to a kCuda-or-beyond value that
  // CreateBackend / ResolveGpuRoute handle as "fall back to CPU".
  server->server_->SetPreferredBackend(static_cast<ns::BackendKind>(backend));
}


int LUMICE_IsBackendAvailable(int backend) {
  try {
    // -Wswitch: exhaustive over BackendKind, no `default:`. Adding a new
    // BackendKind value forces this site to add an explicit case (compile-time
    // gate against silently returning 0 for a freshly-added backend).
    auto kind = static_cast<ns::BackendKind>(backend);
    switch (kind) {
      case ns::BackendKind::kCpu:
        return 1;
      case ns::BackendKind::kMetal:
#if defined(__APPLE__)
        // task-282: deepen the gate from device-presence to trial-compile +
        // entry-point lookup. MetalDeviceAvailable returned true on macOS 26.5 /
        // M1 Max where MSL compilation succeeded but newFunctionWithName
        // ("trace_layer_kernel") returned nil, letting the GUI "Use GPU"
        // checkbox light up and BeginSession abort on a Metal-framework nil-
        // computeFunction assertion. MetalPipelineAvailable runs the same
        // source + options EnsurePso uses and verifies all three kernel entry
        // points resolve.
        return lumice::MetalPipelineAvailable() ? 1 : 0;
#else
        return 0;
#endif
      case ns::BackendKind::kCuda:
#if defined(LUMICE_CUDA_ENABLED)
        // Mirror the Metal gate: report CUDA available only when the build has the
        // CUDA backend AND a usable NVIDIA device is present at runtime (probed via
        // the driver). Non-CUDA builds and GPU-less hosts return 0, so the GUI
        // "Use GPU" toggle stays hidden and BeginSession never routes to a missing
        // device (the CUDA analog of task-282's Metal nil-PSO guard).
        return lumice::CudaDeviceAvailable() ? 1 : 0;
#else
        return 0;
#endif
    }
    return 0;
  } catch (...) {
    return 0;
  }
}

int LUMICE_WillUseGpuRoute(int preferred_backend) {
  try {
    // Single source of truth: delegate to the same env-aware ResolveGpuRoute the
    // server uses to size worker_count (server.cpp). Casting an unknown int to
    // BackendKind is safe here — ResolveGpuRoute treats non-Metal/non-CUDA (or an
    // unavailable device) as the legacy CPU route (returns false), the runtime analog
    // of LUMICE_IsBackendAvailable's compile-time -Wswitch guard above.
    return ns::ResolveGpuRoute(static_cast<ns::BackendKind>(preferred_backend), ns::GetGlobalLogger()) ? 1 : 0;
  } catch (...) {
    // Fail safe to the legacy CPU route, but not silently: a swallowed device-probe
    // error here would run the benchmark's CPU dual-pass while preferred_backend is a
    // GPU, with no trace (project discipline: silent fallbacks cause undebuggable
    // per-machine drift).
    ILOG_WARN(ns::GetGlobalLogger(), "LUMICE_WillUseGpuRoute: route resolution threw; assuming legacy CPU route");
    return 0;
  }
}


// =============== Crystal Mesh ===============

// Reroutes preview-only geometry through the closed-form Crystal factories so
// the LUMICE_CrystalMesh output is built from the parametric face_number /
// face_present / face_vtx tables directly. Replaces the historical pipeline of
//   CreatePrismMesh/CreatePyramidMesh → FillPerFaceTopology (argmax reversal on
//   triangle normals to reconstruct face groups) → FillHexFnMap (argmax again
//   for per-tri face numbers) → triangle-adjacency dihedral edge filter.
// All three reversals are gone: face_numbers per triangle come from the
// Crystal's fn_map_ (parametric, populated from cf_geom_.face_number in
// PopulateFromCfGeom), and face_vtx_pool / face_normals come straight from
// cf_geom_. See doc/crystal-geometry-representation.md §1 for the wider
// "delete the reversal, read the constant" story.
// Fold a 64-bit sample seed into the 32-bit seed RandomNumberGenerator accepts.
// XOR-fold (both halves participate) rather than truncate: truncation would make any
// two seeds that differ only in the high 32 bits collide deterministically; XOR-fold
// reduces that to a ~2^-32 uniform collision probability. This is NOT a "distinct
// sample_seed => distinct mesh" guarantee, only a removal of the deterministic-collision
// class of false negatives.
static uint32_t FoldSampleSeed64(unsigned long long seed) {
  return static_cast<uint32_t>(seed) ^ static_cast<uint32_t>(seed >> 32);
}

LUMICE_ErrorCode LUMICE_GetCrystalMesh(const LUMICE_CrystalParam* crystal, unsigned long long sample_seed,
                                       LUMICE_CrystalMesh* out) {
  if (!crystal || !out) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (crystal->type != 0 && crystal->type != 1) {
    return LUMICE_ERR_INVALID_VALUE;
  }

  // Single mapping table: translate the shape into the core crystal JSON schema, then
  // let the core from_json (already validated by ConfigToJson round-trips) build the
  // CrystalParam variant. This reuses the existing translation instead of maintaining
  // a second, parallel struct-to-struct field map.
  ns::CrystalParam param;
  try {
    const nlohmann::json shape = CrystalShapeToJson(*crystal).at("shape");
    if (crystal->type == 0) {
      param = shape.get<ns::PrismCrystalParam>();
    } else {
      param = shape.get<ns::PyramidCrystalParam>();
    }
  } catch (...) {
    return LUMICE_ERR_INVALID_CONFIG;
  }

  // Sample one concrete shape through the core single-source sampler. A LOCAL RNG
  // instance (not the process singleton) is what makes the determinism contract hold:
  // identical seed + identical param => identical MakeCrystal draw sequence, with no
  // cross-call state carried in a shared generator. For a fully NO_RANDOM param,
  // MakeCrystal never touches the RNG, so sample_seed is a no-op (contract).
  ns::RandomNumberGenerator rng(FoldSampleSeed64(sample_seed));
  const ns::Crystal crystal_obj = ns::MakeCrystal(rng, param);

  // On-demand triangulation: the Crystal no longer stores a triangle mesh
  // (entry-point sampling consumes cf_geom_ corners directly). Geometry export
  // is a cold path (GUI preview, gated by a param hash) so building the mesh
  // here — instead of eagerly in every MakeCrystal — costs nothing on the hot
  // path. A degenerate sample (validation gate rejected) yields a default-constructed
  // Crystal with face_cnt==0; BuildMeshFromCfGeom and the export loops below are all
  // safe (zero-iteration) on that, producing an empty-but-valid mesh and LUMICE_OK.
  const ns::CrystalGeom& g = crystal_obj.CfGeom();
  const ns::detail::BuiltMesh built = ns::detail::BuildMeshFromCfGeom(g);
  const ns::Mesh& mesh = built.mesh;

  auto vtx_cnt = static_cast<int>(mesh.GetVtxCnt());
  if (vtx_cnt > LUMICE_MAX_CRYSTAL_VERTICES) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  out->vertex_count = vtx_cnt;
  if (vtx_cnt > 0) {
    std::memcpy(out->vertices, mesh.GetVtxPtr(0), vtx_cnt * 3 * sizeof(float));
  }

  auto tri_cnt = mesh.GetTriangleCnt();
  if (static_cast<int>(tri_cnt) > LUMICE_MAX_CRYSTAL_TRIANGLES) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  out->triangle_count = static_cast<int>(tri_cnt);
  if (tri_cnt > 0) {
    std::memcpy(out->triangles, mesh.GetTrianglePtr(0), tri_cnt * 3 * sizeof(int));
  }

  // Per-triangle face_number, read straight from cf_geom_. Each triangle's
  // originating face slot (built.tri_face_slot[i]) is, by construction, a
  // present face with >= 3 corners (an absent or sub-triangle face emits no
  // triangle), so cf_geom_.face_number[slot] is always a legal fn — there is
  // no kInvalidId / -1 sentinel case to map anymore.
  for (size_t i = 0; i < tri_cnt; ++i) {
    const int slot = built.tri_face_slot[i];
    out->face_numbers[i] = g.face_number[slot];
  }

  // Per-face polygon topology: walk present slots in cf_geom_, map each face's
  // CCW (x,y,z) vertex to its index in the deduped mesh vertex pool.
  // BuildMeshFromCfGeom used the same coordinates when it built the pool, so a
  // simple linear search with the same 1e-6f tolerance is guaranteed to hit.
  const float* vtx = (vtx_cnt > 0) ? mesh.GetVtxPtr(0) : nullptr;
  constexpr float kDedupTol = 1e-6f;
  auto find_vtx_idx = [&](float x, float y, float z) -> int {
    for (int i = 0; i < vtx_cnt; ++i) {
      float dx = vtx[i * 3 + 0] - x;
      float dy = vtx[i * 3 + 1] - y;
      float dz = vtx[i * 3 + 2] - z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < kDedupTol) {
        return i;
      }
    }
    return -1;
  };

  int fi = 0;
  int pool_offset = 0;
  // Track (v_min, v_max) edge → (slot_a, slot_b). Second slot may stay -1 for
  // boundary edges (only in degenerate geometries; well-formed prism/pyramid
  // yields a closed 2-manifold so every polygon edge is shared by exactly two
  // present slots).
  struct EdgeSlotPair {
    int slot_a;
    int slot_b;
  };
  std::map<std::pair<int, int>, EdgeSlotPair> edge_slots;
  int slot_to_fi[ns::kCrystalGeomMaxFaces];
  for (int i = 0; i < ns::kCrystalGeomMaxFaces; ++i) {
    slot_to_fi[i] = -1;
  }

  for (int slot = 0; slot < g.face_cnt; ++slot) {
    if (!g.face_present[slot]) {
      continue;
    }
    int fn = g.face_vtx_cnt[slot];
    if (fn < 3) {
      continue;
    }
    if (fi >= LUMICE_MAX_CRYSTAL_FACES) {
      break;
    }
    if (pool_offset + fn > LUMICE_MAX_CRYSTAL_FACE_VTXPOOL) {
      break;  // pool exhausted
    }

    const float* face_v = g.face_vtx + slot * ns::kCrystalGeomMaxVtxPerFace * 3;
    // Resolve pool indices for this face's CCW vertex list.
    int local_indices[ns::kCrystalGeomMaxVtxPerFace];
    for (int k = 0; k < fn; ++k) {
      int idx = find_vtx_idx(face_v[k * 3 + 0], face_v[k * 3 + 1], face_v[k * 3 + 2]);
      if (idx < 0) {
        return LUMICE_ERR_INVALID_CONFIG;  // should be impossible: BuildMeshFromCfGeom deduped these coords
      }
      local_indices[k] = idx;
      out->face_vtx_pool[pool_offset + k] = idx;
    }

    out->face_numbers_by_face[fi] = g.face_number[slot];
    out->face_vtx_offsets[fi] = pool_offset;
    out->face_vtx_counts[fi] = fn;
    // Face normal from cf_geom_ is already unit outward (populated by the
    // closed-form evaluator + AdaptClosedFormXxxToCrystalGeom).
    out->face_normals[fi * 3 + 0] = g.face_normal[slot * 3 + 0];
    out->face_normals[fi * 3 + 1] = g.face_normal[slot * 3 + 1];
    out->face_normals[fi * 3 + 2] = g.face_normal[slot * 3 + 2];
    slot_to_fi[slot] = fi;
    pool_offset += fn;
    ++fi;

    // Register polygon-boundary edges for this slot.
    for (int k = 0; k < fn; ++k) {
      int a = local_indices[k];
      int b = local_indices[(k + 1) % fn];
      auto key = std::make_pair(std::min(a, b), std::max(a, b));
      auto [it, inserted] = edge_slots.try_emplace(key, EdgeSlotPair{ slot, -1 });
      if (!inserted) {
        if (it->second.slot_b < 0 && it->second.slot_a != slot) {
          it->second.slot_b = slot;
        }
      }
    }
  }
  out->face_count = fi;

  // Emit edges as polygon boundaries (no triangle-adjacency + dihedral-angle
  // threshold — that was a numerical stand-in for "shared by two polygons of
  // different faces", which cf_geom_ tells us directly).
  int edge_cnt = 0;
  for (const auto& [edge, slots] : edge_slots) {
    if (edge_cnt >= LUMICE_MAX_CRYSTAL_EDGES) {
      break;
    }
    out->edges[edge_cnt * 2 + 0] = edge.first;
    out->edges[edge_cnt * 2 + 1] = edge.second;
    // n0 = normal of first adjacent face slot; n1 = normal of second (or same
    // as n0 for boundary edges — only possible in degenerate geometries).
    const float* n0 = g.face_normal + slots.slot_a * 3;
    const float* n1 = (slots.slot_b >= 0) ? (g.face_normal + slots.slot_b * 3) : n0;
    std::memcpy(&out->edge_face_normals[edge_cnt * 6 + 0], n0, 3 * sizeof(float));
    std::memcpy(&out->edge_face_normals[edge_cnt * 6 + 3], n1, 3 * sizeof(float));
    ++edge_cnt;
  }
  out->edge_count = edge_cnt;

  return LUMICE_OK;
}


// =============== Annotation Overlay ===============
// Bridge only: the geometry, the level-set extraction and the curve walk all live in
// core/annotation_overlay.hpp. What is here is the ABI shape — validation, the enum
// translation, and the one heap allocation the C caller releases.

namespace {

// Everything LUMICE_AnnotationOverlay's pointers point into, kept in one object so a single
// Release frees the lot. Owned through the struct's opaque `storage` handle rather than through
// the individual pointers: the caller has one thing to release, and the released state is
// expressible (all pointers NULL) so a double Release is a no-op instead of a double free.
struct AnnotationStorage {
  lumice::annotation::Overlay overlay;
  std::vector<LUMICE_AnnotationLabel> labels;
};

const unsigned char* MaskPtr(const std::vector<uint8_t>& m) {
  return m.empty() ? nullptr : m.data();
}

// A request angle list, validated and copied. Returns false with `err` set on a malformed list.
bool ReadAngleList(const float* data, int count, int cap, std::vector<float>* out, LUMICE_ErrorCode* err) {
  if (count < 0 || count > cap) {
    *err = LUMICE_ERR_INVALID_VALUE;
    return false;
  }
  if (count > 0 && data == nullptr) {
    *err = LUMICE_ERR_NULL_ARG;
    return false;
  }
  out->assign(data, data + count);
  return true;
}

}  // namespace


LUMICE_ErrorCode LUMICE_ComputeAnnotationOverlay(const LUMICE_AnnotationRequest* request,
                                                 LUMICE_AnnotationOverlay* out) {
  if (!request || !out) {
    return LUMICE_ERR_NULL_ARG;
  }
  const LUMICE_AnnotationView& v = request->view;
  if (v.lens_type < 0 || v.lens_type > LUMICE_LENS_TYPE_GLOBE) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  if (v.visible != LUMICE_VISIBLE_UPPER && v.visible != LUMICE_VISIBLE_LOWER && v.visible != LUMICE_VISIBLE_FULL) {
    return LUMICE_ERR_INVALID_VALUE;
  }

  lumice::annotation::Request req;
  req.view.width = v.width;
  req.view.height = v.height;
  req.view.lens_type = static_cast<ns::LensParam::LensType>(v.lens_type);
  req.view.fov_deg = v.lens_fov;
  req.view.lens_shift[0] = v.lens_shift[0];
  req.view.lens_shift[1] = v.lens_shift[1];
  req.view.overlap = v.overlap;
  req.view.az_deg = v.view_azimuth;
  req.view.el_deg = v.view_elevation;
  req.view.roll_deg = v.view_roll;
  req.view.visible = static_cast<ns::RenderConfig::VisibleRange>(v.visible);
  req.view.front = v.front != 0;
  req.horizon = request->horizon != 0;
  req.zenith_nadir = request->zenith_nadir != 0;
  req.labels = request->want_labels != 0;
  req.reference_dir[0] = request->reference_dir[0];
  req.reference_dir[1] = request->reference_dir[1];
  req.reference_dir[2] = request->reference_dir[2];

  LUMICE_ErrorCode err = LUMICE_OK;
  if (!ReadAngleList(request->elevation_deg, request->elevation_count, LUMICE_MAX_ANNOTATION_LINES, &req.elevation_deg,
                     &err) ||
      !ReadAngleList(request->longitude_deg, request->longitude_count, LUMICE_MAX_ANNOTATION_LINES, &req.longitude_deg,
                     &err) ||
      !ReadAngleList(request->angular_dist_deg, request->angular_dist_count, LUMICE_MAX_ANNOTATION_CIRCLES,
                     &req.angular_dist_deg, &err)) {
    return err;
  }

  std::unique_ptr<AnnotationStorage> storage;
  try {
    storage = std::make_unique<AnnotationStorage>();
    storage->overlay = lumice::annotation::ComputeOverlay(req);
  } catch (...) {
    return LUMICE_ERR_UNKNOWN;
  }

  const lumice::annotation::Overlay& o = storage->overlay;
  storage->labels.reserve(o.labels.size());
  for (const lumice::annotation::Label& l : o.labels) {
    LUMICE_AnnotationLabel dst{};
    dst.px = l.px;
    dst.py = l.py;
    dst.kind = static_cast<int>(l.kind);
    dst.index = l.index;
    dst.value_deg = l.value_deg;
    // Truncation cannot happen for any angle core formats (see LUMICE_ANNOTATION_LABEL_MAX), but
    // the copy is bounded anyway: a silently over-long text would otherwise be a buffer overrun
    // rather than a short label.
    const size_t n = std::min(l.text.size(), sizeof(dst.text) - 1);
    std::memcpy(dst.text, l.text.data(), n);
    dst.text[n] = '\0';
    storage->labels.push_back(dst);
  }

  out->width = o.width;
  out->height = o.height;
  out->drawable = MaskPtr(o.drawable);
  out->horizon = MaskPtr(o.horizon);
  out->elevation = MaskPtr(o.elevation);
  out->longitude = MaskPtr(o.longitude);
  out->angular_dist = MaskPtr(o.angular_dist);
  out->zenith_px = o.zenith.px;
  out->zenith_py = o.zenith.py;
  out->zenith_valid = o.zenith.valid ? 1 : 0;
  out->nadir_px = o.nadir.px;
  out->nadir_py = o.nadir.py;
  out->nadir_valid = o.nadir.valid ? 1 : 0;
  out->labels = storage->labels.empty() ? nullptr : storage->labels.data();
  out->label_count = static_cast<int>(storage->labels.size());
  out->storage = storage.release();
  return LUMICE_OK;
}


void LUMICE_ReleaseAnnotationOverlay(LUMICE_AnnotationOverlay* overlay) {
  if (!overlay || !overlay->storage) {
    return;  // NULL-safe, and idempotent on an already-released or zero-initialized struct
  }
  const std::unique_ptr<AnnotationStorage> owned(static_cast<AnnotationStorage*>(overlay->storage));
  overlay->storage = nullptr;
  // Leave no dangling view of freed memory behind, so a caller that keeps reading the struct after
  // Release sees "nothing here" rather than a use-after-free.
  overlay->drawable = nullptr;
  overlay->horizon = nullptr;
  overlay->elevation = nullptr;
  overlay->longitude = nullptr;
  overlay->angular_dist = nullptr;
  overlay->labels = nullptr;
  overlay->label_count = 0;
  overlay->zenith_valid = 0;
  overlay->nadir_valid = 0;
}


// =============== Crystal Kind ===============
// The three kind-taking predicates below share one shape, replacing the
// `(kind == PRISM) ? kPrism : kPyramid` ternary all three used to spell:
//
//   switch (kind) { case PRISM: …; case PYRAMID: …; }   // no `default:` label
//   <negative answer>                                    // out-of-contract value
//
// The missing `default:` is the point, not an omission. -Wswitch (in -Wall) fires at
// every one of these sites the moment LUMICE_CrystalKind gains a third enumerator, so
// an expansion cannot silently keep mapping the new kind onto Pyramid the way the
// ternary did. The trailing return then answers a value outside the enum — which no
// caller in this repo can produce (every one passes a literal or a ternary over the
// GUI's own two-valued CrystalType), and which C++ cannot even form without UB, but
// which a C caller passing an int can. Rejecting beats guessing "Pyramid" there.
int LUMICE_IsLegalFace(LUMICE_CrystalKind kind, int face) {
  switch (kind) {
    case LUMICE_CRYSTAL_PRISM:
      return ns::IsLegalFace(ns::CrystalKind::kPrism, face) ? 1 : 0;
    case LUMICE_CRYSTAL_PYRAMID:
      return ns::IsLegalFace(ns::CrystalKind::kPyramid, face) ? 1 : 0;
  }
  return 0;
}

int LUMICE_IsShapeScalarApplicable(LUMICE_CrystalKind kind, int slot) {
  switch (kind) {
    case LUMICE_CRYSTAL_PRISM:
      return ns::IsShapeScalarApplicable(ns::CrystalKind::kPrism, slot) ? 1 : 0;
    case LUMICE_CRYSTAL_PYRAMID:
      return ns::IsShapeScalarApplicable(ns::CrystalKind::kPyramid, slot) ? 1 : 0;
  }
  return 0;
}

const char* LUMICE_ShapeScalarSyncKeyName(LUMICE_CrystalKind kind, int slot) {
  switch (kind) {
    case LUMICE_CRYSTAL_PRISM:
      return ns::ShapeScalarSyncKeyName(ns::CrystalKind::kPrism, slot);
    case LUMICE_CRYSTAL_PYRAMID:
      return ns::ShapeScalarSyncKeyName(ns::CrystalKind::kPyramid, slot);
  }
  return nullptr;
}

const char* LUMICE_ShapeWedgeAngleKeyName(int upper) {
  return ns::ShapeWedgeAngleKeyName(upper != 0);
}

const char* LUMICE_ShapeIndicesKeyName(int upper) {
  return ns::ShapeIndicesKeyName(upper != 0);
}


// =============== Axis Scalars ===============
// The wire indices are passed straight through to core's, so they must BE core's. A silent
// mismatch would hand every caller a neighbouring key rather than an error.
static_assert(LUMICE_AXIS_SCALAR_ZENITH == ns::kAxisScalarZenith &&
                  LUMICE_AXIS_SCALAR_AZIMUTH == ns::kAxisScalarAzimuth &&
                  LUMICE_AXIS_SCALAR_ROLL == ns::kAxisScalarRoll && LUMICE_AXIS_SCALAR_COUNT == ns::kAxisScalarCount,
              "LUMICE_AXIS_SCALAR_* must mirror core's AxisScalar enum");

const char* LUMICE_AxisScalarKeyName(int slot) {
  return ns::AxisScalarKeyName(slot);
}

int LUMICE_IsDApplicable(int azimuth_dist_type, float azimuth_full_range_deg, float roll_anchor_deg) {
  // Spelled as a switch rather than a cast: the header promises only that LUMICE_DIST_* stays
  // self-consistent, not that it stays numerically equal to core's DistributionType. Every other
  // wire-enum translation in this file (LUMICE_DIST_* above, LUMICE_LENS_TYPE_*) goes through an
  // explicit switch for the same reason. An unrecognised value answers the negative, matching
  // LUMICE_IsLegalFace / LUMICE_IsShapeScalarApplicable.
  ns::DistributionType az_type{};
  switch (azimuth_dist_type) {
    case LUMICE_DIST_NO_RANDOM:
      az_type = ns::DistributionType::kNoRandom;
      break;
    case LUMICE_DIST_UNIFORM:
      az_type = ns::DistributionType::kUniform;
      break;
    case LUMICE_DIST_GAUSS:
      az_type = ns::DistributionType::kGaussian;
      break;
    case LUMICE_DIST_ZIGZAG:
      az_type = ns::DistributionType::kZigzag;
      break;
    case LUMICE_DIST_LAPLACIAN:
      az_type = ns::DistributionType::kLaplacian;
      break;
    case LUMICE_DIST_GAUSS_LEGACY:
      az_type = ns::DistributionType::kGaussianLegacy;
      break;
    default:
      return 0;
  }
  return ns::detail::IsDApplicableParams(az_type, azimuth_full_range_deg, roll_anchor_deg) ? 1 : 0;
}


// =============== Raypath Validation ===============
LUMICE_ErrorCode LUMICE_ValidateRaypathText(const char* text, LUMICE_CrystalKind kind,
                                            LUMICE_RaypathValidationState* out_state, char* out_msg,
                                            size_t msg_buf_size) {
  if (!text || !out_state || !out_msg) {
    return LUMICE_ERR_NULL_ARG;
  }
  // Two-value enum: extend to switch+assert when CrystalKind expands.
  auto core_kind = (kind == LUMICE_CRYSTAL_PRISM) ? ns::CrystalKind::kPrism : ns::CrystalKind::kPyramid;
  auto r = ns::ValidateRaypathText(std::string(text), core_kind);
  switch (r.state) {
    case ns::RaypathValidation::kValid:
      *out_state = LUMICE_RAYPATH_VALID;
      break;
    case ns::RaypathValidation::kIncomplete:
      *out_state = LUMICE_RAYPATH_INCOMPLETE;
      break;
    case ns::RaypathValidation::kInvalid:
      *out_state = LUMICE_RAYPATH_INVALID;
      break;
    default:
      assert(false);
      *out_state = LUMICE_RAYPATH_INVALID;
      return LUMICE_ERR_UNKNOWN;
  }
  if (msg_buf_size > 0) {
    std::snprintf(out_msg, msg_buf_size, "%s", r.message.c_str());
  }
  return LUMICE_OK;
}


// =============== Lens Type ===============
float LUMICE_MaxFov(LUMICE_LensType type) {
  return ns::MaxFov(static_cast<ns::LensParam::LensType>(type));
}

// =============== Color Conversion ===============
LUMICE_ErrorCode LUMICE_XyzToSrgbUint8(const float* xyz_in, unsigned char* out, int pixel_count,
                                       float intensity_scale) {
  if (!xyz_in || !out) {
    return LUMICE_ERR_NULL_ARG;
  }
  ns::XyzToSrgbUint8(xyz_in, out, pixel_count, intensity_scale);
  return LUMICE_OK;
}

LUMICE_ErrorCode LUMICE_XyzToSrgbUint8WithBackground(const float* xyz_in, unsigned char* out, int pixel_count,
                                                     float intensity_scale, const float* background_linear) {
  if (!xyz_in || !out || !background_linear) {
    return LUMICE_ERR_NULL_ARG;
  }
  ns::XyzToSrgbUint8(xyz_in, out, pixel_count, intensity_scale, background_linear);
  return LUMICE_OK;
}

// =============== EV Auto Anchor ===============
// Thin forwards to core/ev_anchor.hpp, the single owner of the anchor algorithm. The bare-float,
// no-NULL-check contract matches LUMICE_MaxFov and the precondition documented on the header
// declaration; it is the contract the GUI-side implementation these replaced already had.
float LUMICE_ComputeP99Y(const float* xyz_data, int img_width, int img_height, int downsample_factor) {
  return ns::ComputeP99Y(xyz_data, img_width, img_height, downsample_factor);
}

float LUMICE_ComputeEvAuto(float p99_raw_y, float snapshot_intensity, float target_white) {
  return ns::ComputeEvAuto(p99_raw_y, snapshot_intensity, target_white);
}
