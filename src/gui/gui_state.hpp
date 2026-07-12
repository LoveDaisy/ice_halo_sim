#ifndef LUMICE_GUI_STATE_HPP
#define LUMICE_GUI_STATE_HPP

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "gui/gui_constants.hpp"
#include "include/lumice.h"  // LUMICE_RayCount (64-bit ray-count type)

namespace lumice::gui {

// Crystal type
enum class CrystalType { kPrism, kPyramid };

// Last user run-intent (blueprint §5 CQS "last command" channel). This is one of the two
// inputs to ReconcileSimState (the other being the last backend observation). The GUI sets
// this on DoRun/DoStop/DoOpen; SyncFromPoller derives sim_state from it each frame.
//   kNone         — fresh session / CLI-JSON import / .lmc with no baked texture (→ kIdle)
//   kLoaded       — .lmc loaded with a baked result texture (static result, no run → kDone)
//   kRunning      — Run committed; sim_state follows the fresh backend lifecycle
//   kStopping     — Stop issued but the backend is still draining the in-flight batch (async, 1.6).
//                   Optimistic intent-latched display state (→ kStopping); advanced to kStopped by
//                   SyncFromPoller once the background stop completes (g_stop_inflight clears).
//   kStopped      — Stop drained; intent-latched terminal state (→ kStopEndState = kDone, app.cpp)
//   kRunCompleted — Run reached a natural (finite) completion. Intent-latched terminal state
//                   mirroring the kStopping→kStopped Mealy pattern: SyncFromPoller advances
//                   kRunning→kRunCompleted the first time it observes a fresh COMPLETED snapshot
//                   at the current committed_epoch. Once latched, ReconcileSimState maps this
//                   directly to kDone regardless of subsequent snapshot volatility (e.g. a
//                   transient valid=false observation) — closing off AC1's root cause (b)
//                   "completion state not latched, recomputed each frame from volatile inputs"
//                   (task-color-migration §3 D4 / doc/gui-state-governance.md §4 支柱 2).
//                   DoRun writes run_intent = kRunning unconditionally on a successful commit
//                   (app.cpp), naturally re-arming the latch for the next run.
enum class RunIntent { kNone, kLoaded, kRunning, kStopping, kStopped, kRunCompleted };

// Axis distribution type for crystal orientation.
// Values must stay contiguous 0..N-1 — ImGui RadioButton relies on static_cast<int>.
enum class AxisDistType { kGauss, kUniform, kZigzag, kLaplacian, kGaussLegacy, kCount };

// Aspect ratio presets for preview window
enum class AspectPreset { kFree, k16x9, k3x2, k4x3, k1x1, k2x1, kMatchBg };
inline const char* const kAspectPresetNames[] = { "Free", "16:9", "3:2", "4:3", "1:1", "2:1", "Match Background" };
constexpr int kAspectPresetCount = 7;
static_assert(sizeof(kAspectPresetNames) / sizeof(kAspectPresetNames[0]) == kAspectPresetCount,
              "kAspectPresetNames must match kAspectPresetCount");

struct AxisDist {
  AxisDistType type = AxisDistType::kUniform;
  float mean = 0.0f;
  float std = 0.0f;  // Gauss: standard deviation; Uniform: full range; Zigzag: amplitude; Laplacian: scale

  friend bool operator==(const AxisDist& a, const AxisDist& b) {
    return a.type == b.type && a.mean == b.mean && a.std == b.std;
  }
  friend bool operator!=(const AxisDist& a, const AxisDist& b) { return !(a == b); }
};

// GUI-only data structure: crystal geometry + axis distribution.
struct CrystalConfig {
  std::string name;
  CrystalType type = CrystalType::kPrism;

  // Prism
  float height = 1.0f;

  // Pyramid
  float prism_h = 1.0f;
  float upper_h = 0.2f;
  float lower_h = 0.2f;
  float upper_alpha = 28.0f;  // Wedge angle (degrees). Default ≈ atan(√3/2 / 1.629) * 180/π, i.e. Miller {1,0,-1,1}
  float lower_alpha = 28.0f;

  // Face distance (common to Prism and Pyramid — distance from center to each of the 6 prism faces)
  float face_distance[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

  // Axis distribution (all default to uniform full rotation)
  AxisDist zenith{ AxisDistType::kUniform, 0.0f, 360.0f };
  AxisDist azimuth{ AxisDistType::kUniform, 0.0f, 360.0f };
  AxisDist roll{ AxisDistType::kUniform, 0.0f, 360.0f };

  friend bool operator==(const CrystalConfig& a, const CrystalConfig& b) {
    return a.name == b.name && a.type == b.type && a.height == b.height && a.prism_h == b.prism_h &&
           a.upper_h == b.upper_h && a.lower_h == b.lower_h && a.upper_alpha == b.upper_alpha &&
           a.lower_alpha == b.lower_alpha &&
           std::equal(std::begin(a.face_distance), std::end(a.face_distance), std::begin(b.face_distance)) &&
           a.zenith == b.zenith && a.azimuth == b.azimuth && a.roll == b.roll;
  }
  friend bool operator!=(const CrystalConfig& a, const CrystalConfig& b) { return !(a == b); }
};

// Wavelength/weight pair for user-defined discrete spectra.
struct WlWeight {
  float wavelength = 550.0f;  // nm
  float weight = 1.0f;
  friend bool operator==(const WlWeight& a, const WlWeight& b) {
    return a.wavelength == b.wavelength && a.weight == b.weight;
  }
  friend bool operator!=(const WlWeight& a, const WlWeight& b) { return !(a == b); }
};

struct SunConfig {
  float altitude = 20.0f;
  float diameter = 0.5f;
  // Invariant: spectrum_index in [0, kSpectrumCount) selects a preset (custom_spectrum ignored);
  //            spectrum_index == kCustomSpectrumIndex means "use custom_spectrum" (custom_spectrum
  //            must be non-empty). Enforced at every write site (edit modal OK, file_io load).
  int spectrum_index = 2;  // Index into kSpectrumNames: 0=D50,1=D55,2=D65,3=D75,4=A,5=E, 6=Custom...
  std::vector<WlWeight> custom_spectrum;

  friend bool operator==(const SunConfig& a, const SunConfig& b) {
    return a.altitude == b.altitude && a.diameter == b.diameter && a.spectrum_index == b.spectrum_index &&
           a.custom_spectrum == b.custom_spectrum;
  }
  friend bool operator!=(const SunConfig& a, const SunConfig& b) { return !(a == b); }
};

struct SimConfig {
  float ray_num_millions = 5.0f;
  int max_hits = 8;
  bool infinite = false;

  friend bool operator==(const SimConfig& a, const SimConfig& b) {
    return a.ray_num_millions == b.ray_num_millions && a.max_hits == b.max_hits && a.infinite == b.infinite;
  }
  friend bool operator!=(const SimConfig& a, const SimConfig& b) { return !(a == b); }
};

// Lens type names (order must match Core's LensParam::LensType enum)
inline const char* const kLensTypeNames[] = {
  "Linear",
  "Fisheye Equal Area",
  "Fisheye Equidistant",
  "Fisheye Stereographic",
  "Dual Fisheye Equal Area",
  "Dual Fisheye Equidistant",
  "Dual Fisheye Stereographic",
  "Rectangular",
  "Fisheye Orthographic",
  "Dual Fisheye Orthographic",
  "Globe",
};
constexpr int kLensTypeCount = 11;
static_assert(sizeof(kLensTypeNames) / sizeof(*kLensTypeNames) == kLensTypeCount,
              "kLensTypeNames length must match kLensTypeCount");
static_assert(kLensTypeGlobe == kLensTypeCount - 1, "LensType enum terminal value must match kLensTypeCount - 1");

// Display order for the Lens Type combo. Decoupled from enum values so that
// orthographic variants group with their fisheye / dual-fisheye siblings
// without breaking shader and JSON serialization (which use the enum value
// directly as the wire index). The static_assert below pins the array length
// to kLensTypeCount so every new LensType is forced to declare a slot here.
// Note: the size assert does NOT check for duplicates — a value listed twice
// while another is omitted has the same length but silently hides the
// omitted lens from the combo. When adding/renaming entries, manually
// verify that every LensType appears here exactly once.
inline constexpr int kLensTypePresentationOrder[] = {
  kLensTypeLinear,
  kLensTypeFisheyeEqualArea,
  kLensTypeFisheyeEquidist,
  kLensTypeFisheyeStereographic,
  kLensTypeFisheyeOrthographic,
  kLensTypeDualFisheyeEqualArea,
  kLensTypeDualFisheyeEquidist,
  kLensTypeDualFisheyeStereographic,
  kLensTypeDualFisheyeOrthographic,
  kLensTypeRectangular,
  kLensTypeGlobe,
};
static_assert(sizeof(kLensTypePresentationOrder) / sizeof(*kLensTypePresentationOrder) == kLensTypeCount,
              "kLensTypePresentationOrder must list every LensType exactly once");

// Dual fisheye overlap: max |sky.z| for the overlap zone.
// = sin(5°) ≈ 0.0872. Each hemisphere extends 5° past the equator.
// This constant defines the GUI's internal texture format overlap. It is passed to core via
// FillLumiceConfig/SerializeCoreConfig; core reads overlap from RenderConfig (no hardcoded value).
constexpr float kDualFisheyeOverlap = 0.0872f;

// Preset illuminant labels. The combo appends "Custom..." at kCustomSpectrumIndex; that entry is
// NOT part of kSpectrumNames because it is not a downstream illuminant enum — it flips SunConfig
// into the custom_spectrum path.
inline const char* const kSpectrumNames[] = { "D50", "D55", "D65", "D75", "A", "E" };
constexpr int kSpectrumCount = 6;           // number of preset illuminants (NOT the combo item count)
constexpr int kCustomSpectrumIndex = 6;     // sentinel; == kSpectrumCount by design
constexpr int kSpectrumComboItemCount = 7;  // presets + "Custom..." tail
// Hard cap for GUI-side custom spectrum length. Mirrors LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES / core
// wl_pool.hpp::kWlPoolSizeMax. Above kSpectrumSoftWarnCount the modal shows a "noisier per-wl
// sampling" advisory but does not block.
constexpr int kSpectrumHardMax = 255;
constexpr int kSpectrumSoftWarnCount = 64;  // mirrors core wl_pool.hpp::kWlPoolSizeDefault

inline const char* const kVisibleNames[] = { "Upper", "Lower", "Full" };
constexpr int kVisibleCount = 3;  // must stay in sync with fragment shader u_visible range (0-2)
static_assert(sizeof(kVisibleNames) / sizeof(*kVisibleNames) == kVisibleCount,
              "kVisibleNames length must match kVisibleCount");

inline const int kSimResolutions[] = { 512, 1024, 2048, 4096 };
constexpr int kSimResolutionCount = 4;

struct RenderConfig {
  int lens_type = 0;  // Index into kLensTypeNames
  float fov = 90.0f;
  float elevation = 0.0f;
  float azimuth = 0.0f;
  float roll = 0.0f;
  int sim_resolution_index = 1;  // Index into kSimResolutions (default 1024)
  int visible = 2;               // Index into kVisibleNames (0=upper, 1=lower, 2=full)
  bool front = false;            // Independent front-hemisphere clip flag (AND with base)
  float background[3] = { 0.0f, 0.0f, 0.0f };
  float ray_color[3] = { 1.0f, 1.0f, 1.0f };
  float opacity = 1.0f;
  float exposure_offset = 0.0f;  // EV: intensity_factor = 2^exposure_offset

  bool operator==(const RenderConfig& o) const {
    return lens_type == o.lens_type && fov == o.fov && elevation == o.elevation && azimuth == o.azimuth &&
           roll == o.roll && sim_resolution_index == o.sim_resolution_index && visible == o.visible &&
           front == o.front && std::equal(background, background + 3, o.background) &&
           std::equal(ray_color, ray_color + 3, o.ray_color) && opacity == o.opacity &&
           exposure_offset == o.exposure_offset;
  }
  bool operator!=(const RenderConfig& o) const { return !(*this == o); }
};

// task-classic-params-migration (T2) — resim-eligibility comparator for RenderConfig.
// Compares every field EXCEPT `exposure_offset`. Rationale: EV is a pure display-time
// parameter (doc/ev-pipeline-architecture.md §6.4/§6.5 — GUI Run path never bakes EV; the
// core-side NeedsRebuild() already lists intensity_factor / opacity / background / ray_color
// as appearance-only). Among those, only `exposure_offset` has no need to travel through a
// commit at all (it is pushed every frame via RefreshPreviewParams / LUMICE_SetCompositeExposure),
// so it is the single field that should be excluded from the resim/rebuild diff.
// Callers: gui_state_reconcile.cpp::DiffAgainstCommitBaseline and app.cpp::DoRun expect_rebuild
// (single source of truth — do not fork).
inline bool RenderConfigResimEqual(const RenderConfig& a, const RenderConfig& b) {
  return a.lens_type == b.lens_type && a.fov == b.fov && a.elevation == b.elevation && a.azimuth == b.azimuth &&
         a.roll == b.roll && a.sim_resolution_index == b.sim_resolution_index && a.visible == b.visible &&
         a.front == b.front && std::equal(a.background, a.background + 3, b.background) &&
         std::equal(a.ray_color, a.ray_color + 3, b.ray_color) && a.opacity == b.opacity;
}

// Apple Silicon + libc++ only. RenderConfig layout pin: mirrors the EntryCard pattern
// (see below). If this size assertion fires, a field was added/removed from RenderConfig —
// the author must inspect `RenderConfigResimEqual` above and either include the new field
// (if it participates in resim eligibility) or explicitly exclude it (like exposure_offset).
// Linux/Windows CI still compiles the struct; this only pins the Apple main-dev platform.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(RenderConfig) == 64, "RenderConfig size changed — check RenderConfigResimEqual for new fields");
#endif

// Resettable subset of RenderConfig (the View `Reset` button targets these
// four fields). Adding a resettable field here requires adding the matching
// field to RenderConfig and updating every DefaultViewParamsFor branch.
struct ViewDefaults {
  float fov;
  float elevation;
  float azimuth;
  float roll;
};

// Default view parameters per lens type, used by the View `Reset` button.
// Uses explicit lens-type sets (kFov180LensTypes / explicit orthographic /
// kLensTypeGlobe) instead of range comparisons, so adding a new LensType
// fails the kFov180LensTypeCount static_assert rather than silently picking
// a default.
inline ViewDefaults DefaultViewParamsFor(int lens_type) {
  float fov = 90.0f;  // linear / rectangular / orthographic family / fallback
  float azimuth = 0.0f;
  if (LensIsFov180(lens_type)) {
    fov = 180.0f;
  } else if (lens_type == kLensTypeGlobe) {
    fov = 30.0f;
    // Globe is outside-in: az=0 puts the camera behind the sphere. Default az=-180
    // so View Reset shows the same forward direction as non-Globe lenses at az=0.
    // Lens-combo direction continuity is handled separately by the transform formula
    // in RenderRightPanel (app_panels.cpp), not by this default.
    azimuth = -180.0f;
  }
  // Orthographic single + dual already fall through to 90; no extra branch.
  return { fov, 0.0f, azimuth, 0.0f };
}

// Globe lens forces roll=0 at render time without writing back to the stored
// RenderConfig.roll (so switching back to a non-Globe lens preserves the
// user's prior roll value). Apply this helper at every ViewParam.roll fill
// site that feeds BuildViewMatrix or overlay label projection.
inline float EffectiveRollForLens(int lens_type, float stored_roll) {
  return (lens_type == kLensTypeGlobe) ? 0.0f : stored_roll;
}

// Filter action
inline const char* const kFilterActionNames[] = { "Filter In", "Filter Out" };
constexpr int kFilterActionCount = 2;

constexpr char kRaypathSep = '-';
constexpr const char* kRaypathSepStr = "-";

// GUI-only filter parameter sub-structs, mirroring core
// `lumice::SimpleFilterParam` alternatives. See data-model-and-serialization
// task plan §"数据形态" for the explicit core ↔ GUI field-name mapping.

struct RaypathParams {
  // Dash- or comma-separated face indices; ';'-separated multi-segment OR
  // (e.g. "3-5; 1-3"). See raypath_segments.hpp for the parser/validator.
  std::string raypath_text;

  friend bool operator==(const RaypathParams& a, const RaypathParams& b) { return a.raypath_text == b.raypath_text; }
  friend bool operator!=(const RaypathParams& a, const RaypathParams& b) { return !(a == b); }
};

struct EntryExitParams {
  // GUI text buffers — validated via raypath_validation::ValidateFaceNumberListText
  // and parsed to int(s) at the GUI→core / GUI→.lmc serialization boundary.
  // Storing as string lets the user type partial input and lets validation
  // surface "Face N not legal on Prism" messages just like the raypath
  // sub-panel.  Empty text encodes the wildcard ("any face") branch.
  // Multi-value OR is encoded as comma-separated face numbers (e.g. "3,4");
  // the API translation layer expands the cartesian product into N×M
  // EntryExit SimpleFilters wrapped by a ComplexFilter (mirrors the raypath
  // multi-segment path).
  std::string entry_text;
  std::string exit_text;

  // Length constraint UI mode and bounds. The four modes are decoded at the
  // serializer boundary into (min_len, max_len):
  //   0 = no constraint    → min=1, max=nullopt
  //   1 = strict N         → min=max=min_len
  //   2 = at most N        → min=1, max=max_len
  //   3 = range [N,M]      → min=min_len, max=max_len
  // Values are stored 1-based to match the core `size_t min_len_` semantics.
  int length_mode = 0;
  int min_len = 1;
  int max_len = 1;

  friend bool operator==(const EntryExitParams& a, const EntryExitParams& b) {
    return a.entry_text == b.entry_text && a.exit_text == b.exit_text && a.length_mode == b.length_mode &&
           a.min_len == b.min_len && a.max_len == b.max_len;
  }
  friend bool operator!=(const EntryExitParams& a, const EntryExitParams& b) { return !(a == b); }
};

// One AND-factor within a summand row. A Factor is a single simple filter
// alternative (raypath or entry-exit); AND-composition of multiple Factors is
// expressed by the containing SummandText, OR-composition of multiple summands
// is expressed by the containing SumOfProducts. Renamed from the pre-uplift
// FilterParamVariant (which conflated "the filter's sole param" with "a factor")
// to make the sum-of-products shape explicit at the type level.
using Factor = std::variant<RaypathParams, EntryExitParams>;

// Format an EntryExitParams into the small-domain AND grammar text used by
// SummandText.text. Defined near EntryExitParams so raypath_segments.hpp can
// call it without pulling in a reverse include; the grammar itself is spec'd
// in raypath_segments.hpp (ValidateSummandText / ParseSummandText).
//
// Format rule (per plan §3):
//   - Always emit `entry:<entry_text>` as the type anchor (even if entry_text
//     is empty), so a serialized SummandText row can never be confused with
//     the "empty raypath" default.
//   - Append ` & exit:<exit_text>` only when exit_text is non-empty.
//   - Append ` & len:<spec>` when length_mode != 0. Spec encoding:
//       mode 1 (strict N)     → "<min_len>"
//       mode 2 (at most N)    → "<=<max_len>"
//       mode 3 (range [N,M])  → "<min_len>-<max_len>"
inline std::string FormatEntryExitFactorText(const EntryExitParams& ep) {
  std::string out = "entry:" + ep.entry_text;
  if (!ep.exit_text.empty()) {
    out += " & exit:" + ep.exit_text;
  }
  if (ep.length_mode == 1) {
    out += " & len:" + std::to_string(ep.min_len);
  } else if (ep.length_mode == 2) {
    out += " & len:<=" + std::to_string(ep.max_len);
  } else if (ep.length_mode == 3) {
    out += " & len:" + std::to_string(ep.min_len) + "-" + std::to_string(ep.max_len);
  }
  return out;
}

// One OR-row of an AND-of-factors clause. `text` is the canonical form (small
// AND grammar defined in raypath_segments.hpp); `factors` is a parse cache
// derived from `text` and is intentionally excluded from operator== (mirrors
// the RaypathParams::raypath_text convention where the parsed form is a view).
struct SummandText {
  std::string text;
  std::vector<Factor> factors;

  friend bool operator==(const SummandText& a, const SummandText& b) { return a.text == b.text; }
  friend bool operator!=(const SummandText& a, const SummandText& b) { return !(a == b); }
};

// Sum-of-products container: an OR of AND-of-factors rows. Concrete type for
// FilterConfig::param (replaces the pre-uplift Factor alias). An empty vector
// is not a valid state — a "no filter" FilterConfig holds a single-row single-
// factor SoP with an empty RaypathParams (see the FilterConfig default below).
using SumOfProducts = std::vector<SummandText>;

// GUI-only data structure: filter configuration.
//
// Top-level fields (name/action/sym_*) apply to all filter types; per-summand
// data lives inside `param` (sum-of-products of Factors). Default-constructed
// FilterConfig holds a 1-row / 1-factor SoP containing an empty RaypathParams
// (the "no filter" state of pre-variant builds).
struct FilterConfig {
  std::string name;
  int action = 0;  // 0=filter_in, 1=filter_out
  bool sym_p = true;
  bool sym_b = true;
  bool sym_d = true;
  SumOfProducts param{ SummandText{ std::string{}, std::vector<Factor>{ Factor{ RaypathParams{} } } } };

  // Compat accessors — degenerate single-factor path.
  //
  // Current callers (file_io.cpp / edit_modals.cpp / panels.cpp) still operate
  // on the pre-uplift "one filter = one simple" mental model. These helpers
  // present the SoP as if it were a single-factor variant when it structurally
  // is one, so the existing sites can be adapted mechanically (see plan §4
  // Step 3) without touching their logic.
  //
  // ⚠️ Grammar-conformance caveat (task-gui-sop-data-model): SetRaypath /
  // SetEntryExit write a SummandText whose `text` is the tolerant legacy
  // form (may contain the ';' multi-segment OR sugar for raypath). Such
  // rows are NOT guaranteed to round-trip through ParseSummandText — the
  // canonical grammar-conformant conversion path is FromLegacyRaypath /
  // FromLegacyEntryExit in raypath_segments.hpp (they split ';' into rows
  // and emit one factor per row). SetRaypath / SetEntryExit are the "keep
  // callers working" compat layer, not the canonical writer. Do not delete
  // this comment without also retiring the compat callers.
  //   NB (task-serialization-bidirectional, 333.3, landed): the GUI→core
  //   serialization (file_io.cpp ExpandSopToClauses, the single source both
  //   emit twins share) re-splits a factor's internal ';'/comma OR at emit
  //   time, so a compat-written single-factor ';' row and a FromLegacy-split
  //   multi-row SoP expand to the SAME core filters (and are operator==-equal
  //   at the text layer). The `.lmc` writer persists SummandText.text verbatim.
  bool IsDegenerateSingleFactor() const { return param.size() == 1 && param[0].factors.size() == 1; }
  bool IsRaypath() const {
    return IsDegenerateSingleFactor() && std::holds_alternative<RaypathParams>(param[0].factors[0]);
  }
  bool IsEntryExit() const {
    return IsDegenerateSingleFactor() && std::holds_alternative<EntryExitParams>(param[0].factors[0]);
  }
  const Factor& DegenerateFactor() const {
    assert(IsDegenerateSingleFactor() && "FilterConfig::DegenerateFactor() called on non-degenerate SoP");
    return param[0].factors[0];
  }
  const std::string& RaypathText() const {
    assert(IsRaypath() && "FilterConfig::RaypathText() called on non-raypath degenerate factor");
    return std::get<RaypathParams>(param[0].factors[0]).raypath_text;
  }
  std::string& MutableRaypathText() {
    assert(IsRaypath() && "FilterConfig::MutableRaypathText() called on non-raypath degenerate factor");
    return std::get<RaypathParams>(param[0].factors[0]).raypath_text;
  }
  const EntryExitParams& EntryExitParamsValue() const {
    assert(IsEntryExit() && "FilterConfig::EntryExitParamsValue() called on non-EE degenerate factor");
    return std::get<EntryExitParams>(param[0].factors[0]);
  }
  // Compat writers — see the "grammar-conformance caveat" comment above for
  // why these transparently pass rp.raypath_text through (including any ';'
  // multi-segment sugar) instead of splitting into multiple rows.
  void SetRaypath(RaypathParams rp) {
    param.assign(1, SummandText{ rp.raypath_text, std::vector<Factor>{ Factor{ std::move(rp) } } });
  }
  void SetEntryExit(EntryExitParams ep) {
    std::string text = FormatEntryExitFactorText(ep);
    param.assign(1, SummandText{ std::move(text), std::vector<Factor>{ Factor{ std::move(ep) } } });
  }

  // Used by edit_modals.cpp to detect whether the user actually modified the
  // filter buffer (so an untouched OK on a previously-empty filter doesn't
  // silently materialize a default-constructed filter into entry.filter).
  //
  // ⚠️ When adding a new field above, also:
  //   1. Compare it here in operator== (`param` vector is compared elementwise
  //      via SummandText::operator==, which compares `text` only)
  //   2. Update file_io.cpp serialization (SerializeFilterForGui/ParseFilterFromGuiJson)
  //   3. Update edit_modals.cpp Filter tab UI to expose it
  //   4. Update the Factor alternatives (variant) if a new simple-filter
  //      factor type is added (also extend FromLegacyRaypath / FromLegacyEntryExit
  //      / ValidateSummandText / ParseSummandText / FormatFactor in
  //      raypath_segments.hpp accordingly)
  friend bool operator==(const FilterConfig& a, const FilterConfig& b) {
    return a.name == b.name && a.action == b.action && a.sym_p == b.sym_p && a.sym_b == b.sym_b && a.sym_d == b.sym_d &&
           a.param == b.param;
  }
  friend bool operator!=(const FilterConfig& a, const FilterConfig& b) { return !(a == b); }
};

// Linked group invariants (post task-gui-linked-entries):
//
// Formal definition: two entries are "linked" iff they share BOTH crystal_id
// AND filter_id. The fa-link badge shows when >= 2 entries share the same
// (crystal_id, filter_id) pair. A linked group is the atomic share unit —
// edits on any member must be observable on all members.
//
// Edit propagation rules — what stays linked automatically vs needs explicit
// propagation:
//   1. Crystal content edit (in-place CrystalConfig overwrite at the shared
//      pool slot): AUTOMATIC. All entries sharing the crystal_id see the new
//      content via the pool indirection.
//   2. Filter content edit (in-place FilterConfig overwrite at the shared
//      pool slot): AUTOMATIC. Same mechanism via shared filter_id.
//   3. Filter add (entry.filter_id: None -> Some N): NEEDS PROPAGATION.
//      The new pool slot is bound only to the editing entry by default;
//      linked siblings (previously at (cid, None) with this entry) must
//      also have their filter_id flipped to N — otherwise the group
//      decoheres and the badge disappears.
//   4. Filter remove (entry.filter_id: Some -> None): NEEDS PROPAGATION.
//      Linked siblings must also have filter_id cleared.
//
// Crystal_id never changes through normal edits (the editing entry rewrites
// its own pool slot, not the id). The only id-flipping operations are:
//   - Pick-mode "Link to..." (ApplyPickLink in panels.cpp): editing entry
//     adopts target's (crystal_id, filter_id) atomically.
//   - "Unlink" (UnlinkEntryFromPool in panels.cpp): clones pool slot(s) to
//     fork off a private copy.
//   - "Duplicate" (panels.cpp): clone-to-pool produces a fully independent
//     new entry.
//   - Filter add/remove (above): only filter_id flips.
//
// Propagation owner: ApplyBuffersToEntry in edit_modals.cpp via the local
// `propagate_filter_id_to_linked` lambda. Touch that lambda's call sites if
// you add a new filter_id-flipping path.
// See doc/filter-architecture.md §1 for the core/sim-side view of the same invariant.

// GUI-only data structure: one crystal+filter entry card in the layer model.
//
// ID-pool model: EntryCard holds indices into GuiState::crystals / GuiState::filters
// rather than owning CrystalConfig/FilterConfig inline. "Linking = identity" — N
// entries sharing the same crystal_id automatically observe pool mutations on
// the next render, no mirror logic needed.
//
// operator== compares ids + proportion only (intentional: pool contents are
// compared via the pool itself in ConfigSnapshot/round-trip tests). Adding a
// field here requires updating operator== too.
struct EntryCard {
  int crystal_id = 0;
  std::optional<int> filter_id;
  float proportion = 100.0f;

  friend bool operator==(const EntryCard& a, const EntryCard& b) {
    return a.crystal_id == b.crystal_id && a.filter_id == b.filter_id && a.proportion == b.proportion;
  }
  friend bool operator!=(const EntryCard& a, const EntryCard& b) { return !(a == b); }
};
// Apple Silicon + libc++ only. New EntryCard layout (post ID-pool migration):
// int crystal_id (4) + optional<int> filter_id (8) + float proportion (4) = 16 bytes.
// Pinning the Apple build catches accidental field additions during local dev;
// Linux/Windows CI still compiles the struct.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(EntryCard) == 16,
              "EntryCard size changed (check id fields / proportion / operator== for new fields)");
#endif

struct Layer {
  float probability = 0.0f;  // Probability of multi-scatter continuation (0 = single scatter), range [0,1]
  std::vector<EntryCard> entries;

  friend bool operator==(const Layer& a, const Layer& b) {
    return a.probability == b.probability && a.entries == b.entries;
  }
  friend bool operator!=(const Layer& a, const Layer& b) { return !(a == b); }
};

// task-342.3 (scrum-raypath-color-design2): GUI-side raypath color model.
// Mirrors LUMICE_ColorClassRef / LUMICE_ColorClass but with GUI-friendly types
// (crystal pool ids instead of LUMICE C-API ids, predicate text instead of a
// filled LUMICE_ColorPredicate). Full design in
// doc/gui-custom-spectrum-and-raypath-color.md §4.0.
struct ColorClassRefConfig {
  int layer_idx = 0;           // scattering layer index
  int crystal_pool_id = 0;     // index into GuiState::crystals (NOT the LUMICE C-API id)
  bool match_all = true;       // true → LUMICE_FILTER_TYPE_UNSET (whole crystal); false → parse predicate_text
  std::string predicate_text;  // parsed via raypath_segments.hpp; must yield exactly one Factor

  friend bool operator==(const ColorClassRefConfig& a, const ColorClassRefConfig& b) {
    return a.layer_idx == b.layer_idx && a.crystal_pool_id == b.crystal_pool_id && a.match_all == b.match_all &&
           a.predicate_text == b.predicate_text;
  }
  friend bool operator!=(const ColorClassRefConfig& a, const ColorClassRefConfig& b) { return !(a == b); }
};

// task-color-migration (T1) — ColorClassConfig split into structural vs display sub-structs so
// the reconciler can route them onto separate channels (doc/gui-state-governance.md §4 支柱 1).
// Public inheritance chosen over nested named members: keeps every existing `cls.combine` /
// `cls.color` / `cls.z_order` etc. field-access site working unchanged (base-class member lookup),
// while giving the type system the "a function signature that receives `const ColorClassStructState&`
// structurally cannot see display fields" guarantee (plan §3 D1). No aggregate initialisation
// use, no sizeof-static_assert on this type — verified by grep before choosing inheritance.
struct ColorClassStructState {
  int combine = 0;  // 0 = LUMICE_COLOR_COMBINE_ANY, 1 = LUMICE_COLOR_COMBINE_ALL
  std::vector<ColorClassRefConfig> match;

  friend bool operator==(const ColorClassStructState& a, const ColorClassStructState& b) {
    return a.combine == b.combine && a.match == b.match;
  }
  friend bool operator!=(const ColorClassStructState& a, const ColorClassStructState& b) { return !(a == b); }
};

struct ColorClassDisplayState {
  float color[3] = { 1.0f, 1.0f, 1.0f };
  bool visible = true;  // A4 footgun: LUMICE_ColorClass zero-init has visible=0 (hidden). GUI new-class must be true.
  bool solo = false;
  int z_order = 0;  // display-only; never used as vector index (task-342.2 z-order/lane-binding decoupling).

  friend bool operator==(const ColorClassDisplayState& a, const ColorClassDisplayState& b) {
    return std::equal(std::begin(a.color), std::end(a.color), std::begin(b.color)) && a.visible == b.visible &&
           a.solo == b.solo && a.z_order == b.z_order;
  }
  friend bool operator!=(const ColorClassDisplayState& a, const ColorClassDisplayState& b) { return !(a == b); }
};

struct ColorClassConfig : ColorClassStructState, ColorClassDisplayState {
  // Note: label / summary is rebuilt on the fly from `match` at render time,
  // no cache field kept (plan-review Minor #1; a05 减法优先).

  friend bool operator==(const ColorClassConfig& a, const ColorClassConfig& b) {
    return static_cast<const ColorClassStructState&>(a) == static_cast<const ColorClassStructState&>(b) &&
           static_cast<const ColorClassDisplayState&>(a) == static_cast<const ColorClassDisplayState&>(b);
  }
  friend bool operator!=(const ColorClassConfig& a, const ColorClassConfig& b) { return !(a == b); }
};

// MS layer prob helpers (single source of truth for both panels.cpp last-layer
// four-state UI and app_panels.cpp "+ Layer" continuation-prob promotion).
// Keep these in one place: if the two sites diverge, a slider-dragged near-zero
// (e.g. 1e-7) can pass the strict != 0.0f check in one site while the epsilon
// check locks/unlocks in the other, silently defeating footgun #2's guard.
// kProbZeroEps = half the SliderWithInput "%.2f" step (0.01), so any value that
// the UI displays as "0.00" is treated as zero.
// Accepted GUI/CLI asymmetry (code-review Minor-2): the CLI last-layer warning
// uses a strict `prob > 0` (hand-written JSON has no slider drift), so a value
// in (0, kProbZeroEps) — e.g. a hand-edited last-layer 0.001 — warns on the CLI
// but reads as "zero" (disabled, no icon) in the GUI. This is intentional and
// harmless: such a value is below the panel's display precision and discards
// < 0.5% of output rays; forcing the GUI to flag it would contradict the
// display-precision rationale of kProbZeroEps.
constexpr float kProbZeroEps = 0.005f;
constexpr float kDefaultContinuationProb = 0.8f;
inline bool IsProbZero(float p) {
  return p < kProbZeroEps;
}

struct GuiState {
  // ID-pool model (restored from pre-card-redesign): EntryCard holds indices
  // into these pools. Editing a pool slot is observed by every entry sharing
  // its id on the next render. Pool is append-only within a session; orphan
  // entries (no entry references them) are filtered out at save time and the
  // pool is implicitly re-compacted on load.
  std::vector<CrystalConfig> crystals;
  std::vector<FilterConfig> filters;

  // Pick-mode runtime state (eyedropper "Link to..."). Holds the source entry
  // ref while the user picks a target card to share its crystal_id+filter_id
  // with. Not serialized, not in ConfigSnapshot.
  struct EntryRef {
    int layer_idx;
    int entry_idx;
  };
  std::optional<EntryRef> pick_link_source;

  // Layers (entry cards reference crystals/filters via pool ids)
  std::vector<Layer> layers;

  // Scene
  SunConfig sun;
  SimConfig sim;

  // Renderer (copy model: GuiState owns a single renderer directly).
  // Single-renderer enforced by the GUI; if multi-renderer is ever needed, revisit.
  RenderConfig renderer;

  // Raypath color classes (task-342.3 GUI, atop task-342.2 C API surface).
  // Physical array order == LUMICE class id == lane binding. `z_order` inside
  // ColorClassConfig is a separate display-only field — the UI list may be
  // rendered in z_order but this vector's order MUST stay stable across
  // reorder-drag operations (see doc/gui-custom-spectrum-and-raypath-color.md §4.0).
  std::vector<ColorClassConfig> raypath_color;
  int raypath_color_mode = 0;  // LUMICE_COLOR_MODE_DOMINANT / _ADDITIVE / _PAINTER

  // Aspect ratio (view preference, not simulation parameter — does not call MarkDirty)
  AspectPreset aspect_preset = AspectPreset::kFree;
  bool aspect_portrait = false;

  // Runtime-derived aspect clamp info — populated by ApplyAspectRatio when an
  // aspect preset cannot be honored on the current monitor (e.g. picking 2:1
  // on a 1280×720 work area). Drives the warning text rendered next to the
  // aspect-ratio combo. NOT serialized into .lmc view-prefs (it's a derived
  // signal, recomputed on every preset change / window-size callback).
  struct AspectClampInfo {
    bool was_clamped = false;
    float requested_preview_ratio = 0.0f;
    float achieved_preview_ratio = 0.0f;
  };
  AspectClampInfo aspect_clamp{};

  // Background image overlay (view preference — does not call MarkDirty)
  std::filesystem::path bg_path;
  bool bg_show = false;
  float bg_alpha = 1.0f;

  // Auxiliary line overlay (view preference — does not call MarkDirty, not in ConfigSnapshot).
  // Each overlay has independent toggles for line and label visibility, allowing
  // line-only / label-only / both / none combinations. The line flags drive the
  // shader uniform path (see app_panels.cpp pp.overlay assignment), the label flags
  // drive the CPU label sampling path (see BuildOverlayLabelInput in app.cpp).
  //
  // Tech-debt note (task-overlay-line-label-toggle, 2026-04-29): currently 12 flat
  // overlay-related fields (3×color + 3×alpha + 6×bool). When a fourth overlay class
  // is added (e.g. face number overlay merged into this panel), evaluate collapsing
  // to a substruct (per-overlay { color, alpha, line, label }) to avoid further
  // field-name proliferation.
  bool show_horizon_line = false;
  bool show_horizon_label = false;
  bool show_grid_line = false;
  bool show_grid_label = false;
  bool show_sun_circles_line = false;
  bool show_sun_circles_label = false;
  std::vector<float> sun_circle_angles = { 22.0f, 46.0f };
  float horizon_color[3] = { 0.8f, 0.2f, 0.2f };
  float grid_color[3] = { 1.0f, 1.0f, 1.0f };
  float sun_circles_color[3] = { 1.0f, 0.9f, 0.3f };
  float horizon_alpha = 0.6f;
  float grid_alpha = 0.3f;
  float sun_circles_alpha = 0.5f;

  // Zenith / Nadir pixel-space ring marker (see task-gui-zenith-nadir-marker).
  // Single toggle controls both zenith and nadir; radius is in pixels and stays
  // visually constant across lens / FOV / zoom changes.
  bool show_zenith_nadir_line = false;
  float zenith_nadir_color[3] = { 0.8f, 0.2f, 0.2f };
  float zenith_nadir_alpha = 0.6f;
  float zenith_nadir_radius_px = 8.0f;

  // File management
  std::filesystem::path current_file_path;
  bool dirty = false;
  bool save_texture = true;  // Whether to include texture in .lmc save (UI-only, not serialized)

  // Screenshot overlay toggle — when true, Screenshot export composites overlay
  // labels onto the off-screen export FBO via RenderExportToRgba. UI-only, not
  // serialized. (Pre gui-polish-v10 this flag also queued a deferred default-framebuffer
  // readback via pending_screenshot; that mechanism was retired — see SUMMARY.)
  bool screenshot_include_overlay = false;

  // Request a GPU trace backend (Metal on Apple, CUDA on NVIDIA). Toggling this
  // reconstructs the server on the next DoRun via MaybeReconstructServerForBackend
  // (backend is a construction-time topology property: CPU N-worker vs GPU single
  // engine), so the accumulated image resets on toggle. UI-only, session-only.
  bool use_gpu_backend = false;

  // Edit modal mode (UI-only, session-only, not in ConfigSnapshot).
  // Staged mode: BeginPopupModal + OK/Cancel + dirty-mark on tabs.
  // Immediate mode (default, gui-polish-v15 round 2): ImGui::Begin + single
  // Close button + no dirty-mark; every frame commits buffer to state via
  // CommitAllBuffersImmediate (crystal/axis edits only MarkDirty — filter
  // edits still MarkStructHardDirty — so infinite-rays accumulation persists
  // while the user drags a crystal slider).
  bool modal_immediate_mode = true;

  // Mark the config dirty. sim_state is NOT touched here — it is derived once per frame by
  // the single owner ReconcileSimState (I2). A dirty edit on top of a kDone result surfaces as
  // kModified via that reconcile (base kDone + dirty), not via a direct write.
  void MarkDirty() { dirty = true; }

  // Mark a struct-tier hard change. Any struct-tier field diff (filter topology, MS layer
  // count, projection, ray_num, etc. — see gui_state_tiers.hpp T-struct·hard) requires the
  // next struct commit to rebuild the server (epoch++) rather than reuse consumers. Named
  // "struct-hard" to align with doc/gui-state-governance.md §档位表 T-struct·hard; the
  // pre-353.5 name `MarkFilterDirty` was misleading (it never was filter-only — any struct
  // hard-reset routed through here). This does two orthogonal things:
  //   (a) immediate display clear — snapshot_intensity/p99 reset so the shader renders black
  //       right away (a legitimate display action, not a lock);
  //   (b) raise display_epoch_floor to the current committed_epoch so any payload still being
  //       produced by the OLD generation (epoch <= committed_epoch) is blocked by the upload
  //       gate (payload_epoch > floor). The next commit mints epoch+1, whose payloads clear
  //       the floor and reach the screen. This replaces the old intensity_locked boolean with
  //       a monotone epoch key (blueprint §7 / I1).
  // MarkDirty (crystal/sun scrub) deliberately does NOT raise the floor, so carry-forward of
  // the previous generation's texture keeps the preview alive with no black flicker (§3.3).
  void MarkStructHardDirty() {
    MarkDirty();
    snapshot_intensity = 0;
    p99_raw_y = 0.0f;
    display_epoch_floor = committed_epoch;
  }

  // Backend swap (CPU<->GPU) destroys the live server and constructs a fresh one whose epoch
  // authority (server-side committed_epoch_) resets to 0 — the next commit mints epoch 1. The
  // display epoch fence (display_epoch_floor) and the epoch bookkeeping below belong to the OLD
  // server's epoch space; carried across the swap they would fence out the new backend's low-epoch
  // payloads (the upload gate payload_epoch > display_epoch_floor fails whenever the old floor was
  // raised to >=1 by any prior filter edit), freezing the previous backend's texture on screen. A
  // reconstructed server also has no old-generation payloads in flight (the old server is destroyed),
  // so the fence has nothing legitimate left to guard. Reset the display generation to epoch 0 and
  // clear the carried-forward texture so the new backend's first frame reaches the screen. Called by
  // MaybeReconstructServerForBackend (the sole owner of server reconstruction). The epoch fields are
  // re-established from the fresh server via DoRun's post-commit LUMICE_GetSimLifecycle readback.
  void ResetDisplayGenerationForBackendSwap() {
    committed_epoch = 0;
    display_epoch_floor = 0;
    last_uploaded_texture_serial = 0;
    snapshot_intensity = 0;  // immediate clear; the new backend's first payload fills it back in
    p99_raw_y = 0.0f;
    // task-345.4: the new backend has not uploaded anything yet — clear the ground-truth mode
    // record. show_composite_preview is deliberately NOT reset (it is a user preference; a
    // backend swap must not silently flip the user's chosen display mode).
    last_uploaded_as_composite = false;
    // task-color-migration §4 M6 (repush discipline): a backend swap constructs a fresh server
    // with no display state — reset the edge-trigger baseline so the reconciler re-pushes the
    // full display payload on the next reconcile. Routed through the single-entry
    // InvalidateEffectsBaselines() member (code-review round-1 Minor-3) so a future third
    // baseline only needs to be added there, not chased across every reset call site. A member
    // method (rather than a free function) sidesteps the forward-reference problem entirely —
    // sibling member functions may call each other regardless of declaration order within the
    // class body (code-review round-1 Minor-2), so no out-of-class definition is needed.
    InvalidateEffectsBaselines();
  }

  // Panel state (view preference — does not call MarkDirty)
  // not persisted to .lmc (unlike right_panel_collapsed)
  bool left_panel_collapsed = false;
  bool right_panel_collapsed = false;
  // Edit modal layout orientation (view preference). false = horizontal
  // (preview left + tabs right); true = vertical (preview top + tabs below,
  // default since gui-polish-v15 round 2). Persisted to .lmc alongside
  // right_panel_collapsed.
  bool modal_layout_vertical = true;

  // Log panel state (view preference — does not call MarkDirty)
  int gui_log_level = 3;   // Index into log level names: 0=trace,1=debug,2=verbose,3=info,4=warning,5=error,6=off
  int core_log_level = 3;  // Same mapping
  bool log_to_file = false;
  bool log_panel_open = false;

  // Color window (task-342.3, view preference, session-only, NOT serialized).
  bool color_window_open = false;

  // Simulation state — DERIVED, not directly written. ReconcileSimState (app.cpp) is the single
  // owner (I2): it maps (run_intent, committed_epoch, last backend observation, dirty) → sim_state
  // once per frame in SyncFromPoller. The three inputs below are what business ops (DoRun/DoStop/
  // DoOpen) write instead of poking sim_state.
  // kStopping is the optimistic "Stopping…" display state shown while the async Stop drains the
  // in-flight backend batch (1.6). It sits between kSimulating and the terminal state and is NOT
  // demoted by dirty (a draining run is not an editable completed result).
  enum class SimState { kIdle, kSimulating, kStopping, kDone, kModified };
  SimState sim_state = SimState::kIdle;

  // Reconcile inputs (I1/I2). Written by DoRun/DoStop/DoOpen; read by ReconcileSimState.
  RunIntent run_intent = RunIntent::kNone;  // last user command channel (blueprint §5)
  uint64_t committed_epoch = 0;             // epoch the GUI last committed (DoRun reads it back)
  // Epoch floor for the display upload gate (blueprint §7). A payload uploads only when
  // payload_epoch > display_epoch_floor. Raised by MarkStructHardDirty to committed_epoch to fence
  // off the old generation's textures; monotone. Replaces the old intensity_locked boolean.
  uint64_t display_epoch_floor = 0;
  // Consumer-side exact-once upload cursor (migrated from a SyncFromPoller file-scope static in
  // 1.4). SyncFromPoller uploads a payload only when its snapshot texture_serial differs.
  unsigned long long last_uploaded_texture_serial = 0;

  // Stats from last poll
  LUMICE_RayCount stats_ray_seg_num = 0;
  LUMICE_RayCount stats_sim_ray_num = 0;
  float snapshot_intensity = 0;                 // Per-pixel landed intensity for XYZ→RGB normalization
  int effective_pixels = 0;                     // Non-zero pixel count (for stats display)
  unsigned long long texture_upload_count = 0;  // Cumulative texture uploads (diagnostic counter)

  // Auto-EV runtime state (display layer only, not persisted, not in ConfigSnapshot)
  float p99_raw_y = 0.0f;       // Un-normalized P99 Y value; updated each texture upload
  float ev_auto = 0.0f;         // P99-anchored auto-EV in stops; recomputed from p99_raw_y
  float target_white = 135.0f;  // Target P99 brightness on 0-255 sRGB scale

  // task-345.4: display-time raypath-color composite vs full-spectrum toggle.
  // Neither field triggers MarkDirty/MarkStructHardDirty — this is a display-time
  // choice on which already-produced payload buffer to upload, orthogonal to
  // the sim/dirty/Revert lifecycle (blueprint §4.0).
  // Ownership contract (single-writer discipline, review Suggestion #1):
  //   - `show_composite_preview`: user preference. WRITTEN ONLY by the top-bar
  //     checkbox in RenderTopBar (src/gui/app_panels.cpp; relocated from
  //     RenderStatusBar by task-colored-toggle-to-topbar / 346.3). READ by
  //     SyncFromPoller (folded into effective_composite via
  //     ShouldUseCompositeUpload) and RenderTopBar (checkbox label/highlight
  //     computed from `last_uploaded_as_composite`, not this field directly, to
  //     avoid the transient-hallucination race described in the plan §3 keypoint 2).
  //   - `last_uploaded_as_composite`: ground truth. WRITTEN ONLY by SyncFromPoller
  //     (src/gui/app.cpp) after a successful upload. READ by RenderTopBar to
  //     drive the persistent mode indicator.
  bool show_composite_preview = true;
  bool last_uploaded_as_composite = false;

  // Last committed config snapshot (for Revert — config fields only, no runtime state).
  //
  // Field-sync scope (audited 2026-04; raypath_color added 2026-07 by task-349.2):
  //   Fields mirrored here must be the subset of GuiState classified as "configuration"
  //   (i.e. those reached by MarkDirty, contributing to the dirty/Revert lifecycle).
  //   raypath_color IS configuration: structural color-class edits go through
  //   MarkStructHardDirty, so Revert must restore them; the field was missing from the
  //   original 2026-04 audit and re-added by task-349.2 (Step 2 of plan §3.4).
  //   View preferences (aspect_preset, bg_*, horizon/grid/sun circles, log levels,
  //   left_panel_collapsed, right_panel_collapsed, show_composite_preview,
  //   raypath_color_mode — display state via PushDisplayState, not through MarkDirty),
  //   runtime state (sim_state, run_intent, committed_epoch, display_epoch_floor,
  //   last_uploaded_texture_serial, last_uploaded_as_composite, stats_*, snapshot_intensity,
  //   texture_upload_count), and file management (current_file_path, dirty, save_texture)
  //   are intentionally excluded.
  //
  // Protection model (plan.md S1):
  //   - sizeof(ConfigSnapshot) guard below fires when THIS struct's fields change,
  //     forcing the developer to update From()/ApplyTo() bodies below.
  //   - It does NOT fire when GuiState gains a new configuration field — that requires
  //     discipline (code review + the field-sync audit comment above). Stronger
  //     protection was deferred (see plan.md S5b) as disproportionate to risk.
  struct ConfigSnapshot {
    std::vector<CrystalConfig> crystals;
    std::vector<FilterConfig> filters;
    std::vector<Layer> layers;
    SunConfig sun;
    SimConfig sim;
    RenderConfig renderer;
    std::vector<ColorClassConfig> raypath_color;

    // Build a snapshot from the configuration fields of `state`. Implementation is
    // out-of-class (after GuiState is complete) because GuiState is incomplete here.
    static ConfigSnapshot From(const GuiState& state);

    // Restore configuration fields of `state` from this snapshot.
    // IMPORTANT: pure field assignment, no GUI side effects. Callers that need
    // OnLayerStructureChanged(), MarkDirty(), sim_state transitions, etc. must
    // invoke them AFTER ApplyTo() returns.
    void ApplyTo(GuiState& state) const;
  };
  static_assert(std::is_copy_constructible_v<ConfigSnapshot>, "ConfigSnapshot must be copyable");
  static_assert(std::is_copy_assignable_v<ConfigSnapshot>, "ConfigSnapshot must be copy-assignable");
  std::optional<ConfigSnapshot> last_committed_state;

  // task-color-migration (T1) — edge-trigger baseline for the display-state push (plan §3 D3).
  // The frame-tail reconciler diffs the ColorClassDisplayState sub-part of every raypath_color
  // entry + raypath_color_mode against this baseline; a change fires need_display_push,
  // ApplyGuiEffects invokes PushDisplayState, and on success the baseline is updated to the
  // current value — so an idle frame between edits produces zero pushes.
  //
  // Independent from last_committed_state so that (a) pure display edits between two commits
  // still push, (b) commits do not spuriously push if nothing display-side changed. Cleared to
  // nullopt by DoRun success / DoRevert / backend swap via InvalidateEffectsBaselines
  // (plan §4 Step 6): the next frame's reconcile then unconditionally re-pushes the full display
  // state (repush discipline; fixes 偏离 B' — z_order lost after Run). Same-cardinality gate
  // (see gui_state_reconcile.cpp) skips pushing while structural class-count edits settle so we
  // do not spam LUMICE_SetRaypathColors with reject-on-mismatch retries.
  struct DisplayStateBaseline {
    std::vector<ColorClassDisplayState> color_display;
    int raypath_color_mode = 0;

    friend bool operator==(const DisplayStateBaseline& a, const DisplayStateBaseline& b) {
      return a.color_display == b.color_display && a.raypath_color_mode == b.raypath_color_mode;
    }
    friend bool operator!=(const DisplayStateBaseline& a, const DisplayStateBaseline& b) { return !(a == b); }
  };
  std::optional<DisplayStateBaseline> last_pushed_display_state;

  // task-color-migration (T1) — collapse all edge-trigger baselines behind a single reset entry
  // (plan §4 Step 6 / Round 1 review Suggestion 2). Callers reset all baselines via this one
  // method so a future third baseline does not require chasing three DoRun/DoRevert/backend-swap
  // call sites separately (a12：统一原理优先于分类打补丁). A member method (not a free function,
  // code-review round-1 Minor-2) — sibling member functions may reference each other regardless
  // of declaration order within the class body, so this needs no forward declaration and no
  // out-of-class definition, unlike the free-function form this replaced.
  void InvalidateEffectsBaselines() {
    // last_committed_state is deliberately NOT reset here — it is Revert's snapshot, its
    // lifecycle is owned by DoRun (writes) / DoRevert (reads); commingling it with the
    // display-baseline reset would silently break Revert.
    last_pushed_display_state.reset();
  }
};

// Size guard for ConfigSnapshot. If any field changes here, From/ApplyTo below must
// be audited for matching changes. Apple Silicon + libc++ only (std::vector size varies
// across stdlib implementations).
#if defined(__APPLE__) && defined(__aarch64__)
// Size bumped from 160 → 192 by task-gui-custom-spectrum: SunConfig gained a
// std::vector<WlWeight> field (custom_spectrum). From()/ApplyTo() copy `sun`
// wholesale, so this addition is covered without further field-level audit.
// Size bumped from 192 → 216 by task-349.2: added `raypath_color` field
// (std::vector<ColorClassConfig>) so Revert restores color-class edits that
// go through MarkStructHardDirty (Revert-completeness fix, plan §3.4).
static_assert(sizeof(GuiState::ConfigSnapshot) == 216,
              "GuiState::ConfigSnapshot size changed; audit From()/ApplyTo() implementations below");
#endif

inline GuiState::ConfigSnapshot GuiState::ConfigSnapshot::From(const GuiState& state) {
  // Explicit per-field assignment (symmetric with ApplyTo). Avoids aggregate
  // initialization so that when ConfigSnapshot gains a field, the sizeof guard
  // above fires AND reviewers see the obvious gap between From and ApplyTo.
  ConfigSnapshot s;
  s.crystals = state.crystals;
  s.filters = state.filters;
  s.layers = state.layers;
  s.sun = state.sun;
  s.sim = state.sim;
  s.renderer = state.renderer;
  s.raypath_color = state.raypath_color;
  return s;
}

inline void GuiState::ConfigSnapshot::ApplyTo(GuiState& state) const {
  state.crystals = crystals;
  state.filters = filters;
  state.layers = layers;
  state.sun = sun;
  state.sim = sim;
  state.renderer = renderer;
  state.raypath_color = raypath_color;
}

// Convenience helpers (intended for tests + ad-hoc call sites). Production
// code prefers explicit `state.crystals[entry.crystal_id]` because the
// indirection should be visible at the call site.
inline CrystalConfig& CrystalOf(GuiState& s, EntryCard& e) {
  return s.crystals[e.crystal_id];
}
inline const CrystalConfig& CrystalOf(const GuiState& s, const EntryCard& e) {
  return s.crystals[e.crystal_id];
}
inline std::optional<FilterConfig> FilterOf(const GuiState& s, const EntryCard& e) {
  return e.filter_id.has_value() ? std::optional<FilterConfig>{ s.filters[*e.filter_id] } : std::nullopt;
}
// Write a filter back to the pool — reuses the existing pool slot if the
// entry already references one, otherwise appends.
inline void SetFilter(GuiState& s, EntryCard& e, const FilterConfig& f) {
  if (e.filter_id.has_value()) {
    s.filters[*e.filter_id] = f;
  } else {
    e.filter_id = static_cast<int>(s.filters.size());
    s.filters.push_back(f);
  }
}

inline GuiState InitDefaultState() {
  GuiState s;

  // Seed the crystal pool with the default-constructed CrystalConfig (prism,
  // height=1, uniform random orientation). filter pool starts empty (default
  // entry has no filter).
  s.crystals.emplace_back();

  // One default layer with one entry referencing crystal pool slot 0
  Layer layer;
  EntryCard entry;  // crystal_id = 0, filter_id = nullopt, proportion = 100
  layer.entries.push_back(entry);
  s.layers.push_back(layer);

  // Default renderer is the default-constructed GuiState::renderer; no ID or index needed.

  return s;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_STATE_HPP
