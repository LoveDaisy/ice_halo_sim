#ifndef LUMICE_GUI_STATE_HPP
#define LUMICE_GUI_STATE_HPP

#include <algorithm>
#include <array>
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

// The GUI enum above is a UI presentation order (it is what a RadioButton row indexes); the
// LUMICE_DIST_* constants are the wire encoding. They are different orders, so every place that
// hands a GUI distribution to the C API needs this translation — file_io.cpp when it builds a
// LUMICE_Scene, symmetry_ui.cpp when it asks LUMICE_IsDApplicable. One owner, so a sixth type
// cannot be added to one and forgotten in the other.
inline int AxisDistTypeToWire(AxisDistType type) {
  static_assert(static_cast<int>(AxisDistType::kCount) == 5, "Update AxisDistTypeToWire when adding new AxisDistType");
  switch (type) {
    case AxisDistType::kGauss:
      return LUMICE_DIST_GAUSS;
    case AxisDistType::kUniform:
      return LUMICE_DIST_UNIFORM;
    case AxisDistType::kZigzag:
      return LUMICE_DIST_ZIGZAG;
    case AxisDistType::kLaplacian:
      return LUMICE_DIST_LAPLACIAN;
    case AxisDistType::kGaussLegacy:
      return LUMICE_DIST_GAUSS_LEGACY;
    case AxisDistType::kCount:
      break;
  }
  return LUMICE_DIST_GAUSS;
}

// Aspect ratio presets for preview window
enum class AspectPreset { kFree, k16x9, k3x2, k4x3, k1x1, k2x1, kMatchBg };
inline const char* const kAspectPresetNames[] = { "Free", "16:9", "3:2", "4:3", "1:1", "2:1", "Match Background" };
constexpr int kAspectPresetCount = 7;
static_assert(sizeof(kAspectPresetNames) / sizeof(kAspectPresetNames[0]) == kAspectPresetCount,
              "kAspectPresetNames must match kAspectPresetCount");

// Width / height of a preset, or 0 for the two presets that do not name a ratio at all: kFree is
// whatever the user dragged the window to, and kMatchBg is whatever the loaded background image
// happens to be. Callers key off `== 0` to mean "this preset states no ratio" rather than treating
// it as a degenerate number — ApplyAspectRatio bails out of the window resize on it, and the JSON
// export falls back to the simulation texture's own shape.
//
// Lives here, next to the enum it switches over, rather than in app.cpp where it started: it is a
// pure function of the enum with no window / GL / preview dependency, and both of its callers
// (app.cpp's window sizing and file_io.cpp's export) would otherwise have to reach across a layer
// to get at it. Keeping it single-source matters more than where it sits — a second hand-written
// ratio table is how the exported picture and the window it was framed in come to disagree.
inline float GetAspectRatio(AspectPreset preset) {
  switch (preset) {
    case AspectPreset::k16x9:
      return 16.0f / 9.0f;
    case AspectPreset::k3x2:
      return 3.0f / 2.0f;
    case AspectPreset::k4x3:
      return 4.0f / 3.0f;
    case AspectPreset::k1x1:
      return 1.0f;
    case AspectPreset::k2x1:
      return 2.0f;
    case AspectPreset::kFree:
    case AspectPreset::kMatchBg:
    default:
      return 0.0f;
  }
}

struct AxisDist {
  AxisDistType type = AxisDistType::kUniform;
  float mean = 0.0f;
  float std = 0.0f;  // Gauss: standard deviation; Uniform: full range; Zigzag: amplitude; Laplacian: scale

  friend bool operator==(const AxisDist& a, const AxisDist& b) {
    return a.type == b.type && a.mean == b.mean && a.std == b.std;
  }
  friend bool operator!=(const AxisDist& a, const AxisDist& b) { return !(a == b); }
};

// Crystal shape distribution type. Parallel to AxisDist but with a NO_RANDOM alternative that
// axis distributions do not have (a crystal orientation is always a distribution; a shape scalar
// may be a fixed deterministic value). Enum values are deliberately aligned 1:1 with the
// LUMICE_DIST_* wire constants (NO_RANDOM=0 / UNIFORM=1 / GAUSS=2 / ZIGZAG=3 / LAPLACIAN=4 /
// GAUSS_LEGACY=5) so mapping to LUMICE_Distribution is a static_cast, not a hand-written switch
// (contrast FillAxisDist). The static_assert block after ShapeDist pins every value at compile
// time, so any future divergence fails the build instead of silently producing the wrong dist.
enum class ShapeDistType {
  kNoRandom = 0,
  kUniform = 1,
  kGauss = 2,
  kZigzag = 3,
  kLaplacian = 4,
  kGaussLegacy = 5,
  kCount = 6
};

struct ShapeDist {
  ShapeDistType type = ShapeDistType::kNoRandom;
  float center = 0.0f;
  float spread = 0.0f;
  // Shape-scalar sync group (v4.13). 0 = independent; equal non-zero values across scalars of the
  // same crystal mean "draw once, share the value" (core PrepareSyncGroups). The GUI keeps it
  // EMBEDDED in each ShapeDist rather than as a CrystalConfig-level parallel array: CrystalConfig
  // has exactly the ten ShapeDist members the core's kShapeScalarCount indexes, so embedding keeps
  // "a scalar and its group" one object — copies, comparisons and the modal's edit buffer carry it
  // for free. The parallel-array form the C API needs is produced at the two translation boundaries
  // only, by FillSyncGroupArray / ApplySyncGroupArray below.
  // Canonicalization / leader normalization are NOT done here — core's from_json owns them for both
  // the preview path (LUMICE_GetCrystalMesh) and the commit path (LUMICE_SceneAddCrystal), so the
  // GUI passes raw user numbering through verbatim and never becomes a second authority.
  int sync_group = 0;

  ShapeDist() = default;
  ShapeDist(ShapeDistType t, float c, float s) : type(t), center(c), spread(s) {}
  // Non-explicit: a bare scalar ⇔ a NO_RANDOM deterministic value is a convention the wire format
  // already recognises (file_io.cpp's `is_number()` branch, core JSON's no_random bare-number
  // encoding). This constructor only formalises it into C++ so the ~80 existing `c.height = 5.0f`
  // style assignments and CrystalConfig's default member initialisers compile unchanged. Only the
  // float→ShapeDist direction is provided — a reverse ShapeDist→float would silently drop
  // type/spread, exactly the data-loss defect this scrum fixes.
  // NB: providing these constructors makes ShapeDist a non-aggregate, so it is constructed as
  // `ShapeDist{type, center, spread}` (this ctor) or `ShapeDist{value}` (the float ctor) — not via
  // aggregate brace-elision.
  ShapeDist(float v) : center(v) {}  // NOLINT(google-explicit-constructor)

  friend bool operator==(const ShapeDist& a, const ShapeDist& b) {
    return a.type == b.type && a.center == b.center && a.spread == b.spread && a.sync_group == b.sync_group;
  }
  friend bool operator!=(const ShapeDist& a, const ShapeDist& b) { return !(a == b); }
};

// ShapeDist → LUMICE_Distribution. Value alignment (see ShapeDistType comment) makes this a
// static_cast rather than a switch; the static_asserts below guarantee the cast is exact.
inline LUMICE_Distribution ToLumiceDistribution(const ShapeDist& src) {
  return LUMICE_Distribution{ static_cast<int>(src.type), src.center, src.spread };
}

static_assert(static_cast<int>(ShapeDistType::kNoRandom) == LUMICE_DIST_NO_RANDOM,
              "ShapeDistType/LUMICE_DIST_* misaligned");
static_assert(static_cast<int>(ShapeDistType::kUniform) == LUMICE_DIST_UNIFORM,
              "ShapeDistType/LUMICE_DIST_* misaligned");
static_assert(static_cast<int>(ShapeDistType::kGauss) == LUMICE_DIST_GAUSS, "ShapeDistType/LUMICE_DIST_* misaligned");
static_assert(static_cast<int>(ShapeDistType::kZigzag) == LUMICE_DIST_ZIGZAG, "ShapeDistType/LUMICE_DIST_* misaligned");
static_assert(static_cast<int>(ShapeDistType::kLaplacian) == LUMICE_DIST_LAPLACIAN,
              "ShapeDistType/LUMICE_DIST_* misaligned");
static_assert(static_cast<int>(ShapeDistType::kGaussLegacy) == LUMICE_DIST_GAUSS_LEGACY,
              "ShapeDistType/LUMICE_DIST_* misaligned");

// GUI-only data structure: crystal geometry + axis distribution.
struct CrystalConfig {
  std::string name;
  CrystalType type = CrystalType::kPrism;

  // Prism
  ShapeDist height = 1.0f;

  // Pyramid
  ShapeDist prism_h = 1.0f;
  ShapeDist upper_h = 0.2f;
  ShapeDist lower_h = 0.2f;
  float upper_alpha = 28.0f;  // Wedge angle (degrees). Default ≈ atan(√3/2 / 1.629) * 180/π, i.e. Miller {1,0,-1,1}
  float lower_alpha = 28.0f;

  // Face distance (common to Prism and Pyramid — distance from center to each of the 6 prism faces).
  // Each face is an independent ShapeDist so a per-face heterogeneous distribution config round-trips
  // without collapsing (the core d_[6] is six independent Distributions).
  ShapeDist face_distance[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

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

// ---- Shape-scalar sync groups: GUI embedded form <-> C API parallel-array form ----
//
// SLOT-ORDER TRAP: the LUMICE_SHAPE_SCALAR_* index space is NOT this struct's field declaration
// order — UPPER_H is slot 1 and PRISM_H slot 2, the reverse of how CrystalConfig (and the GUI
// modal) orders them. A translation written by field position instead of by name lands prism_h's
// group on upper_h and vice versa, which JSON-round-trips cleanly and only shows up as the wrong
// pair of pyramid heights moving together. So the mapping is spelled out ONCE, by named constant,
// in ShapeScalarAt below; both directions and both call sites go through it.
//
// Single mapping authority. face_distance[0..5] occupies the six contiguous slots at the end.
static_assert(LUMICE_SHAPE_SCALAR_FACE_5 == LUMICE_SHAPE_SCALAR_FACE_0 + 5 &&
                  LUMICE_SHAPE_SCALAR_COUNT == LUMICE_SHAPE_SCALAR_FACE_0 + 6,
              "LUMICE_SHAPE_SCALAR_FACE_* must be six contiguous slots ending the index space");
inline const ShapeDist& ShapeScalarAt(const CrystalConfig& c, int slot) {
  switch (slot) {
    case LUMICE_SHAPE_SCALAR_HEIGHT:
      return c.height;
    case LUMICE_SHAPE_SCALAR_UPPER_H:
      return c.upper_h;
    case LUMICE_SHAPE_SCALAR_PRISM_H:
      return c.prism_h;
    case LUMICE_SHAPE_SCALAR_LOWER_H:
      return c.lower_h;
    default:
      assert(slot >= LUMICE_SHAPE_SCALAR_FACE_0 && slot < LUMICE_SHAPE_SCALAR_COUNT);
      return c.face_distance[slot - LUMICE_SHAPE_SCALAR_FACE_0];
  }
}
inline ShapeDist& ShapeScalarAt(CrystalConfig& c, int slot) {
  return const_cast<ShapeDist&>(ShapeScalarAt(static_cast<const CrystalConfig&>(c), slot));
}

// Embedded -> parallel array, for LUMICE_CrystalParam::sync_group[]. Both C API call sites
// (file_io.cpp's commit path and crystal_preview.cpp's preview path) MUST produce the identical
// array — a divergence would render one crystal in the preview and simulate a different one, with
// nothing to notice it. Sharing this one function is what makes that mechanical rather than a
// convention two files are expected to keep.
// Values pass through verbatim; canonicalization belongs to core (see ShapeDist::sync_group).
inline void FillSyncGroupArray(const CrystalConfig& c, int sync_group[LUMICE_SHAPE_SCALAR_COUNT]) {
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; slot++) {
    sync_group[slot] = ShapeScalarAt(c, slot).sync_group;
  }
}

// Parallel array -> embedded. The inverse of FillSyncGroupArray, used by the .lmc parse path.
inline void ApplySyncGroupArray(const int sync_group[LUMICE_SHAPE_SCALAR_COUNT], CrystalConfig& c) {
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; slot++) {
    ShapeScalarAt(c, slot).sync_group = sync_group[slot];
  }
}

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
// BuildScene (file_io.hpp); core reads overlap from RenderConfig (no hardcoded value).
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

// Display labels for RenderConfig::ev_mode, indexed by the same integer the core enum uses
// (0 = relative, 1 = absolute). A separate array from file_io.cpp's kEvModeJsonNames on purpose:
// that one is the wire format and must never move, this one is user-facing text and may. The same
// split already exists for kVisibleNames vs. the visible field's serialized integer.
inline const char* const kEvModeNames[] = { "Relative", "Absolute" };
constexpr int kEvModeCount = 2;
static_assert(sizeof(kEvModeNames) / sizeof(*kEvModeNames) == kEvModeCount,
              "kEvModeNames length must match kEvModeCount");

inline const int kSimResolutions[] = { 512, 1024, 2048, 4096 };
constexpr int kSimResolutionCount = 4;
// The same four values as the combo shows them. Written once here rather than at each control:
// the View panel's combo and the defaults panel's registered editor both offer this field, and a
// second hand-typed label array is how the two come to disagree about what "1024" selects.
inline const char* const kSimResolutionLabels[] = { "512", "1024", "2048", "4096" };
static_assert(sizeof(kSimResolutionLabels) / sizeof(*kSimResolutionLabels) == kSimResolutionCount,
              "kSimResolutionLabels must cover kSimResolutions");

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
  // Serialized only (file_io.cpp) — no editor registers it (field_editor_registry.cpp), so
  // nothing in the running GUI can change it. See the RenderConfigResimFields comment below.
  float ray_color[3] = { 1.0f, 1.0f, 1.0f };
  float exposure_offset = 0.0f;  // EV: intensity_factor = 2^exposure_offset
  // Which anchor the SERVER measures its exposure scale against, mirroring core
  // RenderConfig::EvMode (0 = relative / P99 self-anchor, 1 = absolute / emitted energy).
  // 0 matches core's default, so a document that never mentions it renders the way the GUI
  // preview always has.
  //
  // Held as a plain int and registered in field_editor_registry.cpp as "renderer.ev_mode", which
  // is what puts it in the Display group's Mode combo and in the defaults panel's field table at
  // the same time. Deliberately NOT a member of RenderConfigResimFields below: switching modes
  // only changes which anchor the client-side conversion divides by, never the simulation output
  // — see the static_assert note further down for the full argument.
  int ev_mode = 0;

  bool operator==(const RenderConfig& o) const {
    return lens_type == o.lens_type && fov == o.fov && elevation == o.elevation && azimuth == o.azimuth &&
           roll == o.roll && sim_resolution_index == o.sim_resolution_index && visible == o.visible &&
           front == o.front && std::equal(background, background + 3, o.background) &&
           std::equal(ray_color, ray_color + 3, o.ray_color) && exposure_offset == o.exposure_offset &&
           ev_mode == o.ev_mode;
  }
  bool operator!=(const RenderConfig& o) const { return !(*this == o); }
};

// The resim-eligible projection of RenderConfig: ONLY the fields whose change genuinely requires
// re-running / rebuilding the simulation. Everything else is client-side display state.
//
// This type is the single place that field list is written down. Three questions read it, and all
// three are its own members so they cannot drift apart:
//   From(live)      — capture the projection (what a commit records).
//   Matches(live)   — has the live config changed in a resim-relevant way? (the "is it dirty"
//                     predicate: gui_state_reconcile.cpp::DiffAgainstCommitBaseline and
//                     app.cpp::DoRun's expect_rebuild).
//   ApplyTo(live)   — restore the projection (what Revert puts back).
// Capture and restore covering exactly the compared field set is the point, not a coincidence:
// a Revert baseline wider than the predicate would discard edits the UI never counted as changes
// (a view drag never lights the Revert button, so Revert must not undo it).
//
// EXCLUDED — T-view fields (doc/gui-state-governance.md §2 row "T-view"): the simulation always
// renders a fixed full-sky dual-fisheye and the GUI reprojects it in the preview GL shader, so
// these are pure client-side reprojection / crop parameters uploaded as shader uniforms
// (preview_renderer.cpp: u_lens_type / u_visible / u_front) and never travel to the server:
//   lens_type, fov, elevation, azimuth, roll  — camera view angle + projection (mouse orbit /
//                                                lens combo). Including them made every drag /
//                                                lens switch falsely trigger a re-sim (auto-restart
//                                                in infinite-rays mode; "Configuration changed" in
//                                                finite mode). That was the scrum-353 T2 regression:
//                                                only exposure_offset had been carved out.
//   visible, front                            — upper/lower/full hemisphere clip (display crop).
//   exposure_offset                           — EV (doc/ev-pipeline-architecture.md §6.4/§6.5;
//                                                pushed every frame via LUMICE_SetCompositeExposure).
//
//   background                                — the colour shown behind the rays. Unlike every other
//                                                name above it is NOT structurally absent from the
//                                                Revert baseline: it keeps its own slot,
//                                                ConfigSnapshot::renderer_background, so an edit is
//                                                undoable without ever counting as a change. That is
//                                                the "not dirty / not epoch" half of T-display in
//                                                doc/gui-state-governance.md §2; the other half (a
//                                                display-time push to the server) does not exist yet,
//                                                so a background edit currently reaches nothing until
//                                                the next commit. Do not read this entry as "the push
//                                                path is built".
//
//   ray_color                                 — carries no live control at all: no row in
//                                                field_editor_registry.cpp, so nothing the running
//                                                GUI does can ever change it. A loaded document's
//                                                value round-trips through Revert/Save unedited and
//                                                can therefore never produce a resim-eligible diff.
//                                                Formerly INCLUDED here specifically to keep an
//                                                edit reaching Run — that reasoning is gone along
//                                                with the control it existed for. Still serialized
//                                                (file_io.cpp) so a document saved before the
//                                                control's removal keeps round-tripping its stored
//                                                value byte-for-byte. BuildScene hard-codes the
//                                                "use the material's own spectral colour" sentinel
//                                                and never reads it either way, unrelated to this
//                                                projection's field list.
//
// INCLUDED — sim_resolution_index: changes the sim render grid → genuine re-sim. Two other members
// this list used to carry are gone: opacity had no drawing consumer anywhere in the tree, and
// ray_color lost its only reason for inclusion above. Both were removed rather than left here
// describing a resim that changes nothing.
//
// Consumers: gui_state_reconcile.cpp::DiffAgainstCommitBaseline, app.cpp::DoRun expect_rebuild,
// and GuiState::ConfigSnapshot's Revert baseline (single source of truth — do not fork).
struct RenderConfigResimFields {
  int sim_resolution_index;

  static RenderConfigResimFields From(const RenderConfig& r) { return { r.sim_resolution_index }; }

  // Write back onto a live RenderConfig, touching nothing outside this projection.
  void ApplyTo(RenderConfig& r) const { r.sim_resolution_index = sim_resolution_index; }

  // Does `live` still agree with this captured baseline on the resim-eligible fields?
  bool Matches(const RenderConfig& live) const { return From(live) == *this; }

  // Value-type equality, kept as free friends so two captured projections can be compared
  // directly (tests and debugging do this); `Matches` above is the live-vs-baseline shorthand
  // built on top of it, not the only intended consumer.
  friend bool operator==(const RenderConfigResimFields& a, const RenderConfigResimFields& b) {
    return a.sim_resolution_index == b.sim_resolution_index;
  }
  friend bool operator!=(const RenderConfigResimFields& a, const RenderConfigResimFields& b) { return !(a == b); }
};

// Apple Silicon + libc++ only. Layout pins, mirroring the EntryCard pattern (see below).
// Linux/Windows CI still compiles both structs; this only pins the Apple main-dev platform.
#if defined(__APPLE__) && defined(__aarch64__)
// RenderConfig: if this fires, a field was added/removed — the author must decide between three
// dispositions: it belongs in RenderConfigResimFields above (participates in resim eligibility);
// it is excluded outright, captured by nothing (like exposure_offset and the T-view fields); or it
// is excluded from resim eligibility but still Revert-tracked through its own ConfigSnapshot slot
// (like background — see ConfigSnapshot::renderer_background).
//
// ev_mode (v4.16) is EXCLUDED, and it now HAS a control (the Display group's Mode combo, plus the
// defaults panel's registered editor) — so the exclusion is a decision, not the absence of one.
// The reason is the same one that excludes exposure_offset a few lines up: switching the mode
// changes which anchor the GUI divides by when it converts the already-simulated XYZ to pixels
// (mono_exposure_scale.hpp), and that conversion happens client-side every frame. The simulation
// output itself is identical under both modes. So a mode switch must NOT flip a finished run into
// kModified, and Revert must not put the old mode back as if it were a config edit awaiting a
// re-run — it is a display-time reading of the same data, exactly like EV.
//
// The composite path is the one place where the mode does reach the server (RenderConsumer picks
// its anchor from config_.ev_mode_ at CommitConfig time), and there it deliberately takes effect
// on the next Run rather than the next frame — see the note at component_compositor.cpp's
// CompositeAnchorScale. That is a slower path to the same value, not a resim dependency.
static_assert(sizeof(RenderConfig) == 64, "RenderConfig size changed — check RenderConfigResimFields for new fields");
// RenderConfigResimFields: naming the field list once does NOT by itself keep the three
// directions in step. From() aggregate-initializes, so a newly added field is silently
// value-initialized rather than rejected, and ApplyTo()/operator== would quietly keep working on
// the old subset. Pinning the size is what turns that omission into a compile error.
static_assert(sizeof(RenderConfigResimFields) == 4,
              "RenderConfigResimFields size changed — update From/ApplyTo/operator== together");
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

// The whole of "the user picked a different lens": the new lens type plus the pose fix-ups that go
// with it. Called by every control that offers the choice — the View panel's combo and the defaults
// panel's per-row editor — because these fix-ups are not decoration, they are what keeps the view
// pointing at the same thing across the switch, and a second control that only assigned lens_type
// would leave a pose the first control can never produce.
//
// Deliberately NOT applied by .lmc loading or by tests that write lens_type directly: those carry
// their own fov/pose and must keep them.
inline void ApplyLensTypeSelection(RenderConfig& r, int new_lens_type) {
  if (r.lens_type == new_lens_type) {
    return;
  }
  const bool was_globe = (r.lens_type == kLensTypeGlobe);
  const bool now_globe = (new_lens_type == kLensTypeGlobe);
  r.lens_type = new_lens_type;
  // Reset fov to the new lens' default so e.g. first-time entry to Globe uses 30° instead of
  // inheriting Linear's 90°.
  r.fov = DefaultViewParamsFor(new_lens_type).fov;
  if (was_globe != now_globe) {
    // Globe is outside-in: crossing the boundary inverts both az and el.
    // az: add 180 (mod 360) — self-inverse, same formula both directions.
    r.azimuth += 180.0f;
    if (r.azimuth > 180.0f) {
      r.azimuth -= 360.0f;
    }
    // el: negate — self-inverse, same formula both directions.
    r.elevation = -r.elevation;
    // Globe el is limited to ±89° to avoid view-matrix degeneracy.
    if (now_globe) {
      r.elevation = std::max(-89.0f, std::min(89.0f, r.elevation));
    }
  }
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
  // Dash-separated face indices (e.g. "3-5"); ';'-separated multi-segment OR
  // (e.g. "3-5; 1-3"). See raypath_segments.hpp for the parser/validator.
  // ',' is NOT a path connector and is rejected by the validator with a message naming '-' and
  // ';' (raypath_validation.cpp). A document written before that was enforced may still carry the
  // legacy ',' connector; DeserializeGuiStateJson rewrites it to '-' on load.
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
// is the canonical form of "this filter states no rule" — it expands to zero
// clauses, which the commit path reads as "no filter" (see the FilterConfig
// default below).
using SumOfProducts = std::vector<SummandText>;

// GUI-only data structure: filter configuration.
//
// Top-level fields (name/action/sym_*) apply to all filter types; per-summand
// data lives inside `param` (sum-of-products of Factors). A default-constructed
// FilterConfig states nothing: its SoP is empty.
//
// It used to default to a 1-row / 1-factor SoP holding an empty RaypathParams,
// inherited from pre-variant builds where "empty raypath" WAS how the editor
// spelled "no filter". That stopped being true once a factor with empty text
// became the editor's match-all: the same shape now commits as core's `none`,
// which under filter_out excludes every ray. The default is the empty vector so
// that the type's own zero value is the harmless state rather than a match-all
// one construction away. The editor still opens "no filter" as exactly one
// blank row — that row is supplied by SetRowsFromSop (edit_modals.cpp) as a
// display affordance, and is a separate thing from this model default.
struct FilterConfig {
  std::string name;
  int action = 0;  // 0=filter_in, 1=filter_out
  bool sym_p = true;
  bool sym_b = true;
  bool sym_d = true;
  SumOfProducts param;

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
// operator== compares ids + proportion + enabled only (intentional: pool
// contents are compared via the pool itself in ConfigSnapshot/round-trip
// tests). Adding a field here requires updating operator== too.
struct EntryCard {
  int crystal_id = 0;
  std::optional<int> filter_id;
  float proportion = 100.0f;

  // "Participates in the simulation" toggle. GUI-only concept: it never
  // crosses the C API — BuildScene translates enabled=false into the
  // proportion=0 the engine already understands, leaving `proportion` itself
  // untouched so the user's weight survives a disable/enable round trip.
  //
  // Semantics are "exclude, then re-run", NOT display-time subtraction: the
  // excluded crystal's share of the fixed total ray count is redistributed to
  // its siblings by PartitionCrystalRayNum, so the remaining crystals get more
  // samples (less noise, and auto-EV brightness moves with it). This is why the
  // card's toggle must not reuse the Colors panel's eye glyph, which does mean
  // display-time subtraction.
  bool enabled = true;

  friend bool operator==(const EntryCard& a, const EntryCard& b) {
    return a.crystal_id == b.crystal_id && a.filter_id == b.filter_id && a.proportion == b.proportion &&
           a.enabled == b.enabled;
  }
  friend bool operator!=(const EntryCard& a, const EntryCard& b) { return !(a == b); }
};
// Apple Silicon + libc++ only. New EntryCard layout (post ID-pool migration):
// int crystal_id (4) + optional<int> filter_id (8) + float proportion (4)
// + bool enabled (1, padded to 4) = 20 bytes.
// Pinning the Apple build catches accidental field additions during local dev;
// Linux/Windows CI still compiles the struct.
#if defined(__APPLE__) && defined(__aarch64__)
static_assert(sizeof(EntryCard) == 20,
              "EntryCard size changed (check id fields / proportion / enabled / operator== for new fields)");
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
  // task-356.3 — per-ref P/B/D symmetry bitmask. Defaults are all false (kSymNone) to
  // mirror core RaypathColorRef::symmetry_ (literal single-orientation match); this
  // deliberately diverges from FilterConfig's default-all-true — see
  // src/config/raypath_color_config.hpp:32 for the design rationale.
  bool sym_p = false;
  bool sym_b = false;
  bool sym_d = false;

  friend bool operator==(const ColorClassRefConfig& a, const ColorClassRefConfig& b) {
    return a.layer_idx == b.layer_idx && a.crystal_pool_id == b.crystal_pool_id && a.match_all == b.match_all &&
           a.predicate_text == b.predicate_text && a.sym_p == b.sym_p && a.sym_b == b.sym_b && a.sym_d == b.sym_d;
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

// True when the layer has entries but every one of them is toggled out of the
// simulation, i.e. the layer contributes nothing. Kept as a free function (same
// pattern as IsProbZero above) so the panel's warning predicate is testable
// without an ImGui context: the warning itself is drawn with TextColored, which
// IsItemHovered/ItemExists cannot reach from a gui_test.
//
// An empty layer is deliberately NOT "all disabled" — it has no toggles in it,
// so the "you turned everything off" message would be misleading.
inline bool AllEntriesDisabled(const Layer& layer) {
  return !layer.entries.empty() &&
         std::all_of(layer.entries.begin(), layer.entries.end(), [](const EntryCard& e) { return !e.enabled; });
}

// ---- Sky reference-point markers ----
//
// One named direction's appearance in the GUI. WHAT the direction is lives in core's
// LUMICE_ANNOTATION_MARKER_* id space, not here; this struct only carries what the GUI decides
// about it, which is exactly the three questions the panel's row asks: draw the ring, draw the
// name, and in what colour.
//
// Radius and opacity are absent for the same reason they are absent from LUMICE_MarkerStyle: they
// are family-wide (GuiState::markers_alpha / markers_radius_px), because the family is read as a
// family and colour is what tells its members apart.
struct MarkerAppearance {
  bool show = false;
  bool label = false;
  float color[3] = { 0.8f, 0.2f, 0.2f };
};

// The six factory appearances, in core-id order. A function rather than a brace initializer on the
// field itself because the GuiState field-tier gate (scripts/check_policies.py) reads ONE LINE per
// field, and a six-element aggregate initializer cannot be one line — `= MakeDefaultMarkers()` can,
// and the gate documents that call-style form as supported. The colours are chosen to be
// distinguishable from one another in a single frame, which is what the family's whole design rests
// on: telling the points apart is what colour is for here (LUMICE_MarkerStyle says the same).
inline std::array<MarkerAppearance, LUMICE_ANNOTATION_MARKER_COUNT> MakeDefaultMarkers() {
  return { {
      { false, false, { 0.85f, 0.25f, 0.25f } },  // Zenith    — the red the zenith/nadir pair had
      { false, false, { 0.35f, 0.55f, 1.00f } },  // Nadir     — blue, the zenith's opposite
      { false, false, { 1.00f, 0.85f, 0.20f } },  // Sun       — yellow
      { false, false, { 0.55f, 0.85f, 1.00f } },  // Subsun    — pale cyan, the sun's reflection
      { false, false, { 1.00f, 0.55f, 0.15f } },  // Anthelion — orange
      { false, false, { 0.65f, 0.40f, 0.95f } },  // Antisolar — violet
  } };
}

// The two name tables, both INDEXED BY THE CORE ID and pinned to it by the static_asserts below.
// Two rather than one because the two audiences are different and must be allowed to diverge: the
// display name is a user-facing string a translator or a designer may change at any time, the
// serial name is part of the .lmc key path and changing it silently orphans every saved document.
inline constexpr const char* kMarkerDisplayNames[LUMICE_ANNOTATION_MARKER_COUNT] = {
  "Zenith", "Nadir", "Sun", "Subsun", "Anthelion", "Antisolar",
};
inline constexpr const char* kMarkerSerialNames[LUMICE_ANNOTATION_MARKER_COUNT] = {
  "zenith", "nadir", "sun", "subsun", "anthelion", "antisolar",
};
static_assert(LUMICE_ANNOTATION_MARKER_ZENITH == 0 && LUMICE_ANNOTATION_MARKER_NADIR == 1 &&
                  LUMICE_ANNOTATION_MARKER_SUN == 2 && LUMICE_ANNOTATION_MARKER_SUBSUN == 3 &&
                  LUMICE_ANNOTATION_MARKER_ANTHELION == 4 && LUMICE_ANNOTATION_MARKER_ANTISOLAR == 5,
              "kMarkerDisplayNames / kMarkerSerialNames / GuiState::markers are indexed by the core "
              "marker id; reordering LUMICE_ANNOTATION_MARKER_* renames every row and every .lmc key");
static_assert(sizeof(kMarkerDisplayNames) / sizeof(kMarkerDisplayNames[0]) == LUMICE_ANNOTATION_MARKER_COUNT,
              "one display name per marker id");
static_assert(sizeof(kMarkerSerialNames) / sizeof(kMarkerSerialNames[0]) == LUMICE_ANNOTATION_MARKER_COUNT,
              "one serial name per marker id");

// Which of a marker row's three serialized leaves a key names.
enum class MarkerKeyPart { kLine, kLabel, kColor };

// THE one place a per-marker serialization key is spelled. Every consumer — the writer and the
// reader in file_io.cpp, the editor registry, the tests — calls this rather than writing
// "overlay_marker_sun_color" out by hand, so a rename is one edit and the four sides cannot drift.
// Not a constexpr table of 18 strings because the shape (family, name, part) is the thing worth
// stating; a table would be the same 18 literals with the rule left implicit.
inline std::string MarkerFieldKey(int marker_id, MarkerKeyPart part) {
  const char* suffix = part == MarkerKeyPart::kLine ? "_line" : part == MarkerKeyPart::kLabel ? "_label" : "_color";
  return std::string("overlay_marker_") + kMarkerSerialNames[marker_id] + suffix;
}

// The family-wide keys, spelled once for the same reason. These are plain constants rather than a
// function because there is no per-marker axis to vary over.
inline constexpr const char* kMarkersAlphaKey = "overlay_markers_alpha";
inline constexpr const char* kMarkersRadiusKey = "overlay_markers_radius_px";
inline constexpr const char* kMarkersSectionOpenKey = "overlay_markers_section_open";

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
  // Default = painter (mirrors core `kDefaultCompositeMode` per doc §4.8). GUI
  // cannot include config/raypath_color_config.hpp per AGENTS.md, so the enum
  // literal is duplicated here and kept in sync with the core default.
  int raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;  // _DOMINANT / _ADDITIVE / _PAINTER

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
  // 2D pan + zoom of the background image, in normalized image space. The background is pinned to
  // the viewport rectangle, NOT to the sky: it never goes through the lens inverse the simulated
  // frame does, so turning the camera / switching lens_type / changing fov leaves it where it is.
  // That independence is what makes "nudge the photo, look at the gap, turn the camera, nudge
  // again" converge, and it is why a cropped photo whose optical center is off-frame cannot be
  // aligned by elevation/azimuth instead. Identity is (0, 0, 1) = today's hard-coded centered
  // contain fit, which is what an .lmc written before these fields existed deserializes to.
  float bg_offset_x = 0.0f;
  float bg_offset_y = 0.0f;
  float bg_scale = 1.0f;

  // Auxiliary line overlay (view preference — does not call MarkDirty, not in ConfigSnapshot).
  // Each overlay has independent toggles for line and label visibility, allowing
  // line-only / label-only / both / none combinations. The line flags drive the
  // shader uniform path (see app_panels.cpp pp.overlay assignment), the label flags gate the
  // label anchors core computes (AnnotationViewInputFor -> AnnotationOverlayCache -> the
  // Build*LabelSet family) and, on the export side, the three grid.*_label keys the CLI reads.
  //
  // Tech-debt note: the flat fields below are 16 (4×color + 4×alpha + 7×bool +
  // sun_circle_angles). The original note said "when a fourth overlay class is added, evaluate
  // collapsing to a substruct (per-overlay { color, alpha, line, label })". That trigger has been
  // crossed twice without the evaluation being done, and the debt is tracked in the backlog rather
  // than resting on this counter alone.
  // What the count no longer includes is the marker family: it WAS four more flat fields
  // (show_zenith_nadir_line + colour + alpha + radius) and is now the substruct the note asks for
  // — MarkerAppearance{show,label,color} in an array indexed by the core marker id. Six reference
  // points forced the question the fourth and fifth overlay classes only raised, because six sets
  // of three flat fields is eighteen names to write out at every one of the five consumers.
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

  // The sky reference points: pixel-space ring markers at N named directions. Indexed BY THE CORE
  // ID — markers[LUMICE_ANNOTATION_MARKER_SUN] is the sun's entry — which is what lets the request,
  // the shader upload, the serializer and the panel all drive off one loop rather than six
  // hand-written blocks. kMarkerDisplayNames / kMarkerSerialNames above are parallel to it and
  // pinned to the same order by the static_asserts beside those tables.
  //
  // Radius and opacity are deliberately NOT per entry: they are family-wide (markers_alpha /
  // markers_radius_px below), the same split LUMICE_MarkerStyle draws for the same reason — a set
  // of reference points reads as a family, colour is what tells its members apart, and a ring at a
  // different size reads as a different KIND of thing rather than as a different point.
  std::array<MarkerAppearance, LUMICE_ANNOTATION_MARKER_COUNT> markers = MakeDefaultMarkers();
  // Family-wide appearance. One pair for all six — see the markers[] comment above.
  float markers_alpha = 0.6f;
  float markers_radius_px = 8.0f;
  // Whether the panel's "Reference Points" section is unfolded. Serialized like every other view
  // preference so a document can carry the section already open, which is the state AC1's
  // "non-default" arm needs to be constructible at all.
  bool markers_section_open = false;

  // Lens border. Outlines the projection's own
  // valid image circle, which is otherwise pure black and indistinguishable from the
  // background. Only the fisheye family has one — see LensHasBorder() in
  // gui_constants.hpp for the authoritative set. There is no radius field: the shader
  // derives the circle from lens type / FOV / resolution.
  bool show_lens_border_line = false;
  float lens_border_color[3] = { 0.3f, 0.7f, 1.0f };
  float lens_border_alpha = 0.6f;

  // File management
  std::filesystem::path current_file_path;
  bool dirty = false;
  bool save_texture = true;  // Whether to include texture in .lmc save (UI-only, not serialized)

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
    snapshot_emitted_energy = 0;
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
    snapshot_emitted_energy = 0;
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

  // "Save current settings as my defaults" panel (defaults_panel.cpp). Session-only for the same
  // reason as color_window_open: which panels are open is not part of the document. The panel's
  // own contents (search text, which rows are unchecked) do NOT live here — they are TU-local to
  // defaults_panel.cpp, are rebuilt every time the panel opens, and have no Revert semantics.
  bool defaults_panel_open = false;

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
  // Sampling-density counters, mirroring LUMICE_StatsResult::crystal_num / ::orientation_num. Same
  // snapshot and staleness semantics as the two above — they travel the identical poller path, epoch gate and
  // DoRun reset, so "updates as the run progresses" holds by construction rather than by a second
  // update path. NOT comparable across backends (see lumice.h: the GPU route reuses one geometry
  // per batch by design); the status-bar tooltip carries that caveat.
  LUMICE_RayCount stats_crystal_num = 0;
  LUMICE_RayCount stats_orientation_num = 0;
  float snapshot_intensity = 0;  // Per-pixel landed intensity for XYZ→RGB normalization
  // Total EMITTED spectral energy behind the displayed snapshot (LUMICE_RawXyzResult::emitted_energy).
  // Not a rescaling of snapshot_intensity above: that one measures what landed on a pixel, this
  // one measures what the source put in, and they differ by every ray a filter, an absorption or
  // a miss removed. It is the denominator absolute exposure mode divides by, and dividing by an
  // input rather than an output is what makes brightness comparable between two documents.
  float snapshot_emitted_energy = 0;
  int effective_pixels = 0;                     // Non-zero pixel count (for stats display)
  unsigned long long texture_upload_count = 0;  // Cumulative texture uploads (diagnostic counter)

  // Auto-EV runtime state (display layer only, not persisted, not in ConfigSnapshot)
  //
  // p99_raw_y holds whichever anchor the CURRENT display mode uses, and the two are different
  // quantities rather than two measurements of one: mono carries the server's
  // anchor_l99_sky (a sky RADIANCE per steradian, measured on the fixed full-sky anchor plane and
  // shared with the CLI), composite carries composite_p99_y (a P99 over the participating class
  // lanes of THIS view). The name predates the split and is kept because it is what the readout
  // and the tests already call it; SyncFromPoller is the single place that picks.
  float p99_raw_y = 0.0f;       // Anchor for the displayed mode; updated each texture upload
  float ev_auto = 0.0f;         // Anchor-derived auto-EV in stops; recomputed from p99_raw_y
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
  //   snapshot_emitted_energy, texture_upload_count), and file management (current_file_path,
  //   dirty, save_texture)
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
    // Deliberately NOT a whole RenderConfig. Two slots, and the split is the point:
    // `renderer_resim` holds exactly the fields whose change counts as a change, so Revert
    // restores that set and nothing wider. The T-view fields (lens_type / fov / elevation /
    // azimuth / roll / visible / front) and exposure_offset are absent *structurally* — not
    // captured-then-skipped-on-restore, which would leave From and ApplyTo asymmetric and the
    // omission easy to lose track of.
    RenderConfigResimFields renderer_resim;
    // background sits outside the resim projection (see RenderConfigResimFields' comment block)
    // yet inside the Revert baseline: editing it must not dirty the document or re-run a finished
    // simulation, but Revert must still put the previous colour back. RenderConfigResimFields
    // cannot express that combination — membership there binds capture, restore and compare
    // together — hence a slot of its own.
    //
    // A bare array rather than a named type, unlike the other "excluded from resim but still
    // Revert-tracked" precedent in this file (ColorClassDisplayState, the display half of
    // raypath_color): that one splits a struct with a dozen mixed-semantics fields, this one is a
    // single colour. If a second RenderConfig field ever needs the same treatment, that is the
    // point to reconsider extracting a named RenderConfigDisplayFields — not before.
    float renderer_background[3];
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

// ---- Sky reference-point marker predicates (single owner) ----
//
// "Is any marker asking for X" is answered here because three call sites ask it — the preview's
// per-frame cache Update (app_panels.cpp), the off-screen export's own refresh (file_io.cpp) and
// the label block — and each of them writing its own six-way disjunction is three chances for one
// to be updated and the others not. That is the same drift the array replaced six named fields to
// prevent, one level up.
//
// TWO predicates, not one, because the two questions are genuinely different: whether core has to
// PROJECT anything, and whether any NAME has to be drawn. A marker with only its label switched on
// needs the projection (a name is placed at a point) but draws no ring, so collapsing them would
// make one of the two call sites wrong.
//
// There is deliberately no third, `AnyMarkerShown`: nothing in the product asks it. The panel's
// per-row clipping notice reads that marker's own switch, and [All]/[None] assign rather than test.
inline bool AnyMarkerLabelShown(const GuiState& s) {
  return std::any_of(s.markers.begin(), s.markers.end(), [](const MarkerAppearance& m) { return m.label; });
}
// Whether core has to project ANY marker for this frame. Either switch counts, exactly like the
// horizon's two: a user drawing only the names still needs the positions the names are placed at.
inline bool AnyMarkerRequested(const GuiState& s) {
  return std::any_of(s.markers.begin(), s.markers.end(), [](const MarkerAppearance& m) { return m.show || m.label; });
}

// ---- Display-side identity / numbering formatters (single owner) ----
//
// These four live here rather than in each renderer because every one of them had ALREADY drifted
// across its call sites before being collected: the layer number was written 1-based in the panel
// header and file_io's overflow locator but 0-based in three places in the Colors window and in the
// link badge's tooltip, and the crystal's display name was spelled one way in the Colors window and
// not spelled at all on the card. A `+ 1` re-typed at each site is what let them disagree; the fix
// is that no site types it.
//
// Placement note for whoever adds the next one: gui_state.hpp is "state structures", and
// FormatCrystalIdentity is not a pure predicate on a struct the way IsProbZero / AllEntriesDisabled
// are — it reads GuiState and returns a display string. Four of them is still within what this file
// carries comfortably. If a fifth and sixth formatting free function want in here, that is the
// signal to split them out into a gui_formatting.{hpp,cpp} instead of letting this file become
// "state + formatting toolbox". This sentence is the search anchor for that moment; do not delete it.

// Layer / Entry numbers as the USER sees them. The stored indices are 0-based everywhere (they are
// vector subscripts); every user-visible rendering of them is 1-based. Both directions of the
// conversion belong to exactly one function each.
inline int DisplayLayerNumber(int layer_idx) {
  return layer_idx + 1;
}
inline int DisplayEntryNumber(int entry_idx) {
  return entry_idx + 1;
}

inline const char* CrystalTypeName(CrystalType type) {
  return type == CrystalType::kPrism ? "Prism" : "Pyramid";
}

// The one spelling of "which crystal is this": `#N` (pool id), then the user's name when it has
// one, then the type. Both the entry card's first row and the Colors window's crystal combos /
// class summaries render it, so a user reading `#2 · plate · Prism` in the Colors window can find
// the same string on a card.
//
// `#N` is ALWAYS present and is never replaced by the name, even when there is one: nothing stops
// two crystals from carrying the same name, and two identical entries in a combo are worse than an
// id-only label. N is the pool index (GuiState::crystals), which is the key the color refs actually
// store — it is sparse (the pool is append-only; deleting a card never reclaims its slot) but
// stable, and compacting it to make the numbers prettier would rewrite every ref's key and make the
// number move whenever an unrelated card is deleted.
inline std::string FormatCrystalIdentity(const GuiState& state, int pool_id) {
  std::string out = "#" + std::to_string(pool_id);
  if (pool_id < 0 || static_cast<size_t>(pool_id) >= state.crystals.size()) {
    // The id still leads: a dangling ref's whole diagnostic value is WHICH id dangles.
    return out + " <missing>";
  }
  const CrystalConfig& cr = state.crystals[static_cast<size_t>(pool_id)];
  if (!cr.name.empty()) {
    out += " · " + cr.name;
  }
  out += " · ";
  out += CrystalTypeName(cr.type);
  return out;
}

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
// Size then shrank when the Revert baseline narrowed from a full RenderConfig to
// RenderConfigResimFields: it now holds exactly the fields that count as a change.
// The number then held at 184 when `background` moved out of RenderConfigResimFields into its own
// `renderer_background` slot: the projection lost 12 bytes and the snapshot gained 12 back, so an
// unchanged size here is the expected outcome of that move, not evidence it was a no-op. Both
// numbers were read off the compiler rather than hand-computed.
// Size then shrank 184 → 176 when `opacity` left RenderConfigResimFields with the field itself
// (it had no drawing consumer anywhere in the tree). The projection lost only 4 bytes; the
// snapshot lost 8, because the 4 came out of what had been exactly-fitting padding ahead of
// `raypath_color`. Read off the compiler, not hand-computed — the arithmetic does not predict it.
// Size then shrank 176 → 168 when `ray_color` left RenderConfigResimFields with the field itself:
// it lost its only editor (field_editor_registry.cpp no longer registers it), so nothing can ever
// change it again and it stopped being a resim-eligible field to track. Read off the compiler.
static_assert(sizeof(GuiState::ConfigSnapshot) == 168,
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
  s.renderer_resim = RenderConfigResimFields::From(state.renderer);
  std::copy(std::begin(state.renderer.background), std::end(state.renderer.background), s.renderer_background);
  s.raypath_color = state.raypath_color;
  return s;
}

inline void GuiState::ConfigSnapshot::ApplyTo(GuiState& state) const {
  state.crystals = crystals;
  state.filters = filters;
  state.layers = layers;
  state.sun = sun;
  state.sim = sim;
  renderer_resim.ApplyTo(state.renderer);
  std::copy(std::begin(renderer_background), std::end(renderer_background), state.renderer.background);
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

// Populate the collection half of a brand-new document: one crystal in the pool, one layer
// holding one entry that references it. Factored out of InitDefaultState() because
// MakeNewDocumentState() (src/gui/user_defaults.hpp) needs the identical seed after applying
// the user's personal defaults — two hand-kept copies of these four lines would be a
// comment-maintained invariant, i.e. one that eventually breaks silently.
//
// Precondition: the collections are empty (both callers either start from a fresh GuiState or
// clear them first). This seeds, it does not reset.
inline void SeedDefaultDocumentContents(GuiState& s) {
  // Seed the crystal pool with the default-constructed CrystalConfig (prism,
  // height=1, uniform random orientation). filter pool starts empty (default
  // entry has no filter).
  s.crystals.emplace_back();

  // One default layer with one entry referencing crystal pool slot 0
  Layer layer;
  EntryCard entry;  // crystal_id = 0, filter_id = nullopt, proportion = 100
  layer.entries.push_back(entry);
  s.layers.push_back(layer);
}

inline GuiState InitDefaultState() {
  GuiState s;

  SeedDefaultDocumentContents(s);

  // Default renderer is the default-constructed GuiState::renderer; no ID or index needed.

  return s;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_STATE_HPP
