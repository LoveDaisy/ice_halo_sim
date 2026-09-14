#ifndef LUMICE_SERVER_C_API_ENUM_MAP_HPP
#define LUMICE_SERVER_C_API_ENUM_MAP_HPP

// The two enum-valued renderer fields that cross the C boundary in BOTH directions — lens type
// and visible range — as one bidirectional table each, kept in one header so the two halves
// cannot drift apart. Each FromCApi has a `default:` arm that throws on an int no constant
// spells; each ToCApi is TOTAL over the core enumeration (no default arm) so adding a projection
// or a visibility range to core breaks the build here instead of decoding to a wrong constant.
// The four are moved here from c_api.cpp verbatim — function bodies, contract and all.
//
// Two consumers. c_api.cpp uses both directions (LUMICE_SceneAddRenderer decodes a
// LUMICE_RenderParam; the config-scratch route encodes one). The CLI's `analyze` subcommand
// (src/main.cpp) uses the ToCApi half only, to assemble the LUMICE_AnnotationView of a
// `--roi frame` request from a `render[]` entry it read through config/render_config.hpp — the
// one place the engine's spelling of "fisheye_equal_area" lives, which is why the CLI reads the
// core type and maps it rather than keeping a string table of its own.
//
// On the layering: this is the first internal header under src/server/ the CLI includes. Every
// other internal header main.cpp reaches for sits under src/util/ (result_frame.hpp,
// cpu_info.hpp) — a different directory, so those are not a precedent for this one, and no
// checker forbids or blesses it. The reasoning is the same as for compiling
// config/render_config.cpp into the Lumice target (src/CMakeLists.txt): the CLI is a first-party
// consumer built from the same tree, and a mapping whose one end is a LUMICE_* constant is the
// C API layer's to own, not config/'s. An external consumer that needed the same read-back would
// get a C API getter instead; none exists today.
//
// `inline` rather than a .cpp: the CLI links `lumice` (the exported surface, LUMICE_* only), and
// under the shared build's -fvisibility=hidden a definition in a .cpp of lumice_obj would not be
// reachable from it. These bodies touch nothing but the enumerators and LUMICE_* macros, so the
// header adds no link dependency to whoever includes it.

#include <stdexcept>
#include <string>

#include "config/render_config.hpp"
#include "include/lumice.h"

namespace lumice::c_api_enum_map {

// Map LUMICE_LENS_TYPE_* to its core enumerator. Explicit switch (not a numeric cast) so a future
// reorder of either enumeration surfaces as a compile/throw rather than a silently aliased
// projection. Throws std::invalid_argument on an unknown value.
inline LensParam::LensType MapLensTypeFromCApi(int lens_type) {
  switch (lens_type) {
    case LUMICE_LENS_TYPE_LINEAR:
      return LensParam::kLinear;
    case LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA:
      return LensParam::kFisheyeEqualArea;
    case LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT:
      return LensParam::kFisheyeEquidistant;
    case LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC:
      return LensParam::kFisheyeStereographic;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA:
      return LensParam::kDualFisheyeEqualArea;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUIDISTANT:
      return LensParam::kDualFisheyeEquidistant;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_STEREOGRAPHIC:
      return LensParam::kDualFisheyeStereographic;
    case LUMICE_LENS_TYPE_RECTANGULAR:
      return LensParam::kRectangular;
    case LUMICE_LENS_TYPE_FISHEYE_ORTHOGRAPHIC:
      return LensParam::kFisheyeOrthographic;
    case LUMICE_LENS_TYPE_DUAL_FISHEYE_ORTHOGRAPHIC:
      return LensParam::kDualFisheyeOrthographic;
    case LUMICE_LENS_TYPE_GLOBE:
      return LensParam::kGlobe;
    default:
      throw std::invalid_argument("LUMICE_RenderParam.lens_type is invalid: " + std::to_string(lens_type));
  }
}

// Inverse of MapLensTypeFromCApi. Total over the core enumeration (no default arm) so adding a
// projection to core breaks the build here instead of decoding to a wrong C API constant.
inline int MapLensTypeToCApi(LensParam::LensType type) {
  switch (type) {
    case LensParam::kLinear:
      return LUMICE_LENS_TYPE_LINEAR;
    case LensParam::kFisheyeEqualArea:
      return LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA;
    case LensParam::kFisheyeEquidistant:
      return LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT;
    case LensParam::kFisheyeStereographic:
      return LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC;
    case LensParam::kDualFisheyeEqualArea:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA;
    case LensParam::kDualFisheyeEquidistant:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUIDISTANT;
    case LensParam::kDualFisheyeStereographic:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_STEREOGRAPHIC;
    case LensParam::kRectangular:
      return LUMICE_LENS_TYPE_RECTANGULAR;
    case LensParam::kFisheyeOrthographic:
      return LUMICE_LENS_TYPE_FISHEYE_ORTHOGRAPHIC;
    case LensParam::kDualFisheyeOrthographic:
      return LUMICE_LENS_TYPE_DUAL_FISHEYE_ORTHOGRAPHIC;
    case LensParam::kGlobe:
      return LUMICE_LENS_TYPE_GLOBE;
  }
  throw std::invalid_argument("unmapped core LensType: " + std::to_string(static_cast<int>(type)));
}

// Map LUMICE_VISIBLE_* to its core enumerator. Throws std::invalid_argument on an unknown value.
inline RenderConfig::VisibleRange MapVisibleFromCApi(int visible) {
  switch (visible) {
    case LUMICE_VISIBLE_UPPER:
      return RenderConfig::kUpper;
    case LUMICE_VISIBLE_LOWER:
      return RenderConfig::kLower;
    case LUMICE_VISIBLE_FULL:
      return RenderConfig::kFull;
    default:
      throw std::invalid_argument("LUMICE_RenderParam.visible is invalid: " + std::to_string(visible));
  }
}

// Inverse of MapVisibleFromCApi. Total over the core enumeration (no default arm) so adding a
// visibility range to core breaks the build here instead of decoding to a wrong C API constant —
// same fail-loud contract as MapLensTypeToCApi.
inline int MapVisibleToCApi(RenderConfig::VisibleRange visible) {
  switch (visible) {
    case RenderConfig::kUpper:
      return LUMICE_VISIBLE_UPPER;
    case RenderConfig::kLower:
      return LUMICE_VISIBLE_LOWER;
    case RenderConfig::kFull:
      return LUMICE_VISIBLE_FULL;
  }
  throw std::invalid_argument("unmapped core VisibleRange: " + std::to_string(static_cast<int>(visible)));
}

}  // namespace lumice::c_api_enum_map

#endif  // LUMICE_SERVER_C_API_ENUM_MAP_HPP
