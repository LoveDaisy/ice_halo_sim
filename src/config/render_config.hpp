#ifndef CONFIG_RENDER_CONFIG_H_
#define CONFIG_RENDER_CONFIG_H_

#include <nlohmann/json.hpp>
#include <vector>

#include "core/def.hpp"

namespace lumice {

struct ViewParam {
  float az_{};  // Azimuth
  float el_{};  // Elevation
  float ro_{};  // Roll
};

void to_json(nlohmann::json& j, const ViewParam& v);
void from_json(const nlohmann::json& j, ViewParam& v);

struct GridLineParam {
  float value_{};
  float width_{ 1.0f };
  float opacity_{ 1.0f };
  float color_[3]{ 1.0f, 1.0f, 1.0f };
};

void to_json(nlohmann::json& j, const GridLineParam& l);
void from_json(const nlohmann::json& j, GridLineParam& l);

// The zenith / nadir pair, drawn as a pixel-space ring around wherever those two world directions
// land on the canvas. Appearance only: WHERE the rings go comes from annotation::ComputeOverlay's
// zenith / nadir CanvasPoints, which is why this struct carries no angle the way GridLineParam
// does — the two directions are fixed, so there is nothing per-line to name.
//
// One struct for both markers, not two: the GUI has a single switch, a single colour picker and a
// single radius slider for the pair (gui_state.hpp show_zenith_nadir_line / zenith_nadir_color /
// zenith_nadir_alpha / zenith_nadir_radius_px), and the defaults below are that control's.
//
// `color_` is sRGB, like GridLineParam::color_ and unlike RenderConfig::background_ — the consumer
// converts once at the blend, which is where the linear domain begins.
struct ZenithNadirParam {
  bool enabled_ = false;
  float radius_px_ = 8.0f;
  float opacity_ = 0.6f;
  float color_[3]{ 0.8f, 0.2f, 0.2f };
};

void to_json(nlohmann::json& j, const ZenithNadirParam& z);
void from_json(const nlohmann::json& j, ZenithNadirParam& z);

// Which named sky direction a marker entry names. Numerically one-to-one with
// annotation::MarkerId, and pinned to it by static_assert in render.cpp — the one translation unit
// that includes both headers.
//
// A separate enum rather than a reuse of annotation::MarkerId, for the same reason
// RenderConfig::VisibleRange exists next to LUMICE_VISIBLE_*: annotation_overlay.hpp already
// includes THIS header (it takes a RenderConfig to derive its view), so including it back here
// would close a cycle. The duplication is two declarations of six integers held equal by a
// compile-time check, which is the trade this file has already made once.
enum class MarkerRefId : int {
  kZenith = 0,
  kNadir = 1,
  kSun = 2,
  kSubsun = 3,
  kAnthelion = 4,
  kAntisolar = 5,
};

// One reference-point marker: WHICH direction, and what colour. Radius and opacity are absent on
// purpose — they are family-wide fields on RenderConfig, because a set of reference points reads as
// a family and colour is what tells its members apart (a ring at a different size or transparency
// reads as a different kind of annotation, not as a different point).
//
// `color_` is sRGB, like GridLineParam::color_ and ZenithNadirParam::color_.
struct MarkerStyleParam {
  MarkerRefId id_ = MarkerRefId::kZenith;
  bool enabled_ = false;
  float color_[3]{ 0.8f, 0.2f, 0.2f };
};

// Hand-written on both sides, and the from_json half deliberately does NOT go through
// NLOHMANN_JSON_SERIALIZE_ENUM for `id_`: that macro maps any unregistered string to the FIRST
// table entry without an error, so a typo would land silently on the zenith. The same failure mode
// is on record for `visible` (doc/gui-state-governance.md §9: "front" silently becoming "upper"),
// which is why an unknown id here throws instead.
void to_json(nlohmann::json& j, const MarkerStyleParam& m);
void from_json(const nlohmann::json& j, MarkerStyleParam& m);

// True when two entries name the same MarkerRefId. `dup` (optional) receives the offending id.
//
// An ARRAY-level rule, so it cannot live in MarkerStyleParam::from_json — that function sees one
// entry and has no way to look at its siblings. Shared by both JSON decoders (config_manager.cpp
// and c_api.cpp) rather than implemented twice: two independent duplicate checks are two chances
// to disagree, and test_json_parser_parity.cpp exists precisely because that has happened before.
bool HasDuplicateMarkerId(const std::vector<MarkerStyleParam>& markers, MarkerRefId* dup);

struct LensParam {
  enum LensType {
    kLinear,
    kFisheyeEqualArea,
    kFisheyeEquidistant,
    kFisheyeStereographic,
    kDualFisheyeEqualArea,
    kDualFisheyeEquidistant,
    kDualFisheyeStereographic,
    kRectangular,
    kFisheyeOrthographic,
    kDualFisheyeOrthographic,
    kGlobe,
  };

  LensType type_;
  float fov_;
};

NLOHMANN_JSON_SERIALIZE_ENUM(  // declare
    LensParam::LensType,       // type
    {
        { LensParam::kLinear, "linear" },
        { LensParam::kFisheyeEqualArea, "fisheye_equal_area" },
        { LensParam::kFisheyeEquidistant, "fisheye_equidistant" },
        { LensParam::kFisheyeStereographic, "fisheye_stereographic" },
        { LensParam::kDualFisheyeEqualArea, "dual_fisheye_equal_area" },
        { LensParam::kDualFisheyeEquidistant, "dual_fisheye_equidistant" },
        { LensParam::kDualFisheyeStereographic, "dual_fisheye_stereographic" },
        { LensParam::kRectangular, "rectangular" },
        { LensParam::kFisheyeOrthographic, "fisheye_orthographic" },
        { LensParam::kDualFisheyeOrthographic, "dual_fisheye_orthographic" },
        { LensParam::kGlobe, "globe" },
    })

void to_json(nlohmann::json& j, const LensParam& l);
void from_json(const nlohmann::json& j, LensParam& l);

// Returns the maximum valid FOV (in degrees) for the given lens type.
float MaxFov(LensParam::LensType type);

struct RenderConfig {
  enum VisibleRange {
    kUpper,
    kLower,
    kFull,
  };

  // Which anchor the mono/composite exposure scale is measured against. See
  // doc/ev-pipeline-architecture.md and RenderConsumer::ExposureScale().
  //   kRelative — anchor to the frame's own P99 (what the GUI has always displayed): the image
  //               keeps its look as ray_num grows, but the config alone does NOT determine the
  //               output brightness (ray_num co-determines it). This is the default.
  //   kAbsolute — anchor to the EMITTED energy: the scale is fixed by light source + ray budget,
  //               so two simulations at the same EV are directly comparable.
  enum EvMode {
    kRelative,
    kAbsolute,
  };

  IdType id_{};
  LensParam lens_{ LensParam::kLinear, 90.0f };
  int lens_shift_[2]{};  // dx, dy
  int resolution_[2]{};  // width, height
  ViewParam view_{};
  VisibleRange visible_ = kUpper;
  // A second clip dimension, independent of visible_ and ANDed with it: when set, a pixel is
  // drawable only if its world direction also lies in the hemisphere the camera faces
  // (forward . dir >= 0). It is deliberately NOT a VisibleRange enumerator — the two axes are
  // orthogonal, and folding them into one enum would multiply out its cases. Twin of
  // LUMICE_RenderParam::front and LUMICE_AnnotationView::front.
  bool front_ = false;

  // Linear RGB. The JSON "background" key is sRGB; to_json / ParseRenderConfig convert.
  float background_[3]{};
  float ray_color_[3]{ -1.0f, -1.0f, -1.0f };  // r, g, b
  // Brightness scaling for CLI output (PostSnapshot). GUI uses exposure_offset (EV stops) in
  // gui_state.hpp directly; the two are related by intensity_factor = 2^exposure_offset but serve
  // different paths and may differ at runtime (GUI updates EV without re-committing config).
  float intensity_factor_ = 1.0f;
  float overlap_ = 0.0f;  // Dual fisheye overlap zone |sky.z| threshold (sin value). 0 = no overlap.
  // Appearance field (like intensity_factor_): it selects WHICH exposure formula PostSnapshot()
  // and the compositor use, never the accumulation layout, so a change needs no consumer rebuild.
  EvMode ev_mode_ = kRelative;

  std::vector<GridLineParam> angular_dist_grid_;
  std::vector<GridLineParam> elevation_grid_;
  // Meridians: lines of constant azimuth. Named "longitude" rather than "azimuth" because that is
  // the word the annotation layer already uses for this concept everywhere it is public
  // (annotation::Request::longitude_deg, Overlay::longitude, LUMICE_ANNOTATION_LONGITUDE); a second
  // spelling in the persisted schema would give one concept two vocabularies.
  std::vector<GridLineParam> longitude_grid_;
  // Opt-in, not on by default. It was `true` for the four years the field parsed and drew nothing,
  // which cost nothing; now that it draws, `true` would put a horizon line into every existing
  // config that never asked for one (13 of the 14 reference renders in test/e2e-correctness/ set
  // no `grid.outline` key at all). Turning an annotation on for every render is a product decision
  // nobody has made, so the default states the one thing that is certain: draw it when asked.
  bool horizon_ = false;
  // Draw the TEXT labels — the angle each line stands for, "22\u00b0" and the like — next to the
  // three line families. One switch per family, mirroring the GUI's three
  // (show_horizon_label / show_grid_label / show_sun_circles_label, gui_state.hpp), because those
  // three are independently settable there and a config that could not express the same three
  // states would be unable to describe what the GUI is showing.
  //
  // TWO LAYERS, and they are not the same statement. Whether the label GEOMETRY is computed is
  // what these fields decide, independently of whether the family's own line is drawn: a
  // horizon_label_ with horizon_ false still produces the anchors. Whether the label is VISIBLE
  // once drawn is NOT independent — the compositor gives a label its family's own colour and
  // opacity (GridLineParam::opacity_ for the grid families, the horizon's fixed constants for the
  // horizon), so a line at opacity 0 takes its labels with it. That is the GUI's behaviour too
  // (overlay_labels.cpp gives every label its family's colour and alpha), and matching it is the
  // point: a core that let a label outlive its line would be a new GUI/CLI divergence.
  //
  // Opt-in for the same reason horizon_ is: text nobody asked for must not appear in a config
  // that predates the field.
  bool horizon_label_ = false;
  // The parallels and the meridians share one switch, as they share one appearance in the GUI.
  bool grid_label_ = false;
  bool angular_dist_label_ = false;
  // The zenith / nadir ring markers. Opt-in for the same reason horizon_ is: an annotation nobody
  // asked for must not appear in a config that predates the field.
  //
  // No label switch of its own, and not an omission: a marker carries no text at all
  // (annotation::Overlay returns the two as POINTS, not labels), so there is nothing to turn on.
  ZenithNadirParam zenith_nadir_;

  // The reference-point markers — the generalization of zenith_nadir_ above to N named directions
  // with per-entry colour. Empty by default, which is what makes it opt-in AND what makes it the
  // "absent" signal: see below.
  //
  // ARBITRATION, single-sourced in RenderConsumer: a non-empty markers_ wins outright and
  // zenith_nadir_ is ignored; zenith_nadir_ is consulted only when markers_ is empty. Not a merge,
  // because a config that lists markers is describing its whole marker set, and silently adding two
  // more rings from a legacy field it also carries would draw something nobody asked for.
  // Emptiness is the ONLY absence signal, which has one consequence worth stating: an explicit
  // "markers": [] alongside a "zenith_nadir" falls back to the legacy pair, since a vector cannot
  // tell "key absent" from "key present and empty". Recording an extra "was the key seen" bit to
  // separate those two would put a JSON-parsing detail into a struct that three other paths build
  // without any JSON at all.
  std::vector<MarkerStyleParam> markers_;
  // Family-wide, one pair for the whole list — see MarkerStyleParam for why only colour is per
  // entry. The values are ZenithNadirParam's, so a markers_ list that names zenith and nadir and
  // sets nothing else looks like the legacy pair.
  float markers_opacity_ = 0.6f;
  float markers_radius_px_ = 8.0f;
};

NLOHMANN_JSON_SERIALIZE_ENUM(    // declare
    RenderConfig::VisibleRange,  // type
    {
        { RenderConfig::kUpper, "upper" },
        { RenderConfig::kLower, "lower" },
        { RenderConfig::kFull, "full" },
    })

// kRelative FIRST on purpose: NLOHMANN_JSON_SERIALIZE_ENUM maps any unrecognized string to the
// first table entry, so a misspelled value lands on the same mode a MISSING key does.
NLOHMANN_JSON_SERIALIZE_ENUM(  // declare
    RenderConfig::EvMode,      // type
    {
        { RenderConfig::kRelative, "relative" },
        { RenderConfig::kAbsolute, "absolute" },
    })

void to_json(nlohmann::json& j, const RenderConfig& r);

// Returns true if layout-affecting fields differ (resolution, lens, view, visible, overlap, filter).
// Appearance-only changes (background, ray_color, intensity_factor, grids) return false.
bool NeedsRebuild(const RenderConfig& old_cfg, const RenderConfig& new_cfg);

}  // namespace lumice

#endif  // CONFIG_RENDER_CONFIG_H_
