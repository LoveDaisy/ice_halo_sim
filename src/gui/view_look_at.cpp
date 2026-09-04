#include "gui/view_look_at.hpp"

#include <algorithm>
#include <cmath>

#include "gui/annotation_overlay_cache.hpp"  // GuiSunWorldDir — the GUI's one spelling of the sun
#include "gui/gui_state.hpp"                 // kMarkerDisplayNames — the Overlay list's own labels

namespace lumice::gui {

namespace {

constexpr float kRad2Deg = 180.0f / 3.14159265358979323846f;

}  // namespace

const char* LookAtDisplayName(LookAtId id) {
  if (IsMarkerLookAt(id)) {
    return kMarkerDisplayNames[static_cast<int>(id)];
  }
  if (id == LookAtId::kSunHorizon) {
    // Hyphenated and unabbreviated on purpose: "Sun horizon" reads as a thing, "Sun-side horizon"
    // reads as a place, and the place is what the user asked for ("the horizon on the sun's side").
    return "Sun-side horizon";
  }
  return nullptr;
}

void WorldDirToAzEl(const float dir[3], float* az_deg, float* el_deg) {
  const float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
  // A zero vector has no direction to report. Answering (0, 0) keeps the outputs finite — the whole
  // reason a caller reads them is to write them into a camera angle — and matches what the formula
  // below would give for the +x direction, which is where every other degenerate case in this
  // family lands.
  const float inv = (len > 0.0f) ? (1.0f / len) : 0.0f;
  const float x = dir[0] * inv;
  const float y = dir[1] * inv;
  const float z = dir[2] * inv;

  // asin's domain, defended against the rounding that a normalize can leave behind (|z| can come
  // back at 1.0000001 for a vector that was exactly a pole).
  const float clamped_z = (z < -1.0f) ? -1.0f : ((z > 1.0f) ? 1.0f : z);
  *el_deg = std::asin(-clamped_z) * kRad2Deg;

  // At a pole there is no bearing to report, and atan2 must not be asked for one: it reads the SIGN
  // OF A ZERO, and negating a +0 gives a -0, so atan2(-0, -0) answers -pi rather than the 0 an
  // "undefined, pick anything" reading would expect. Left to that, the zenith and nadir presets
  // would swing the camera a half turn around the vertical on their way to a direction that does
  // not depend on the azimuth at all. Answering 0 explicitly is one of the infinitely many correct
  // answers, and it is the same one the overlay label placer anchors its own pole degeneracy at.
  if (x == 0.0f && y == 0.0f) {
    *az_deg = 0.0f;
    return;
  }
  *az_deg = std::atan2(-y, -x) * kRad2Deg;
}

bool ResolveLookAtAzEl(LookAtId id, float sun_altitude_deg, float* out_az_deg, float* out_el_deg) {
  if (!out_az_deg || !out_el_deg) {
    return false;
  }
  float sun_dir[3] = {};
  GuiSunWorldDir(sun_altitude_deg, sun_dir);

  float target[3] = {};
  LUMICE_ErrorCode rc = LUMICE_ERR_INVALID_VALUE;
  if (IsMarkerLookAt(id)) {
    rc = LUMICE_ResolveAnnotationMarkerDirection(static_cast<int>(id), sun_dir, target);
  } else if (id == LookAtId::kSunHorizon) {
    rc = LUMICE_ResolveSunHorizonDirection(sun_dir, target);
  }
  if (rc != LUMICE_OK) {
    return false;
  }
  WorldDirToAzEl(target, out_az_deg, out_el_deg);
  return true;
}

bool ResolveLookAtPose(LookAtId id, float sun_altitude_deg, const FieldEditorConstraint& el_c,
                       const FieldEditorConstraint& az_c, float* out_az_deg, float* out_el_deg) {
  float az = 0.0f;
  float el = 0.0f;
  if (!ResolveLookAtAzEl(id, sun_altitude_deg, &az, &el)) {
    return false;
  }
  // has_numeric_domain guards a constraint that carries no interval — a combo or a bool row would
  // report [0, 0], and clamping to that would point the camera at the equirect centre for every
  // preset. Neither field this is called with is such a row, which is why the guard passes the
  // value through rather than failing: the interval is missing, not violated.
  *out_el_deg = el_c.has_numeric_domain ?
                    std::clamp(el, static_cast<float>(el_c.min_value), static_cast<float>(el_c.max_value)) :
                    el;
  *out_az_deg = az_c.has_numeric_domain ?
                    std::clamp(az, static_cast<float>(az_c.min_value), static_cast<float>(az_c.max_value)) :
                    az;
  return true;
}

}  // namespace lumice::gui
