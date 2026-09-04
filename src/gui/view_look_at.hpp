#ifndef LUMICE_GUI_VIEW_LOOK_AT_HPP
#define LUMICE_GUI_VIEW_LOOK_AT_HPP

#include "gui/field_editor_registry.hpp"  // FieldEditorConstraint — the sliders' own bounds
#include "include/lumice.h"               // LUMICE_ANNOTATION_MARKER_* — the id space the first six entries ARE

namespace lumice::gui {

// The View group's "Look At" presets: pick a named direction in the sky, get the camera angles
// that point at it.
//
// WHY THIS IS TWO STEPS AND NOT ONE. "Where is the subsun" is a property of the sky, owned once by
// core (annotation::ResolveMarkerDir) and reachable here only through the C API — the GUI may not
// include a core header, and a hand-copied sign rule here would be the second implementation this
// family exists to prevent. "Which camera angles point at a world direction" is a different
// question, one core does not model at all: a camera pose is a GUI/CLI concept. So the sky half is
// forwarded and the pose half is computed here, and neither half duplicates the other side.
//
// The presets write ONLY azimuth and elevation. Roll and fov are the user's framing, not part of
// "look at that"; a preset that reset them would silently undo work every time it was used.

// A direction offered in the Look At menu. The first six ARE the marker id space, value for value
// — the whole point of the feature is that the user reads a name off the Overlay list and turns to
// the same name, so a second numbering would be an invitation to drift.
//
// kSunHorizon is the one entry that is not a marker, and it is deliberately last rather than mixed
// in: it has no landing point on the canvas, which is exactly why core refuses to give it a
// MarkerId (see annotation_overlay.hpp's SunHorizonDir). It reaches the same answer through its own
// C API entry point instead of being smuggled in as a seventh id.
enum class LookAtId : int {
  kZenith = LUMICE_ANNOTATION_MARKER_ZENITH,
  kNadir = LUMICE_ANNOTATION_MARKER_NADIR,
  kSun = LUMICE_ANNOTATION_MARKER_SUN,
  kSubsun = LUMICE_ANNOTATION_MARKER_SUBSUN,
  kAnthelion = LUMICE_ANNOTATION_MARKER_ANTHELION,
  kAntisolar = LUMICE_ANNOTATION_MARKER_ANTISOLAR,
  kSunHorizon = LUMICE_ANNOTATION_MARKER_COUNT,
  kCount = LUMICE_ANNOTATION_MARKER_COUNT + 1,
};

// True for the ids that are markers, i.e. the ones whose direction comes from the marker table
// rather than from the sun-horizon entry point. One predicate so the "< COUNT" boundary is spelled
// once.
inline bool IsMarkerLookAt(LookAtId id) {
  return static_cast<int>(id) >= 0 && static_cast<int>(id) < LUMICE_ANNOTATION_MARKER_COUNT;
}

// The menu label. For the six markers this returns kMarkerDisplayNames[id] verbatim — the SAME
// string the Overlay panel's Reference Points list shows, read from the same table rather than
// re-typed here, because a preset the user cannot match to the dot they can see is worth less than
// no preset at all. kSunHorizon owns the only new literal in this file.
// Returns nullptr for an out-of-range id.
const char* LookAtDisplayName(LookAtId id);

// The camera angles that point at `dir`, inverting the forward vector BuildViewMatrix builds:
//   forward = (-cos(el)cos(az), -cos(el)sin(az), -sin(el))
// so el = asin(-z) and az = atan2(-y, -x), both in degrees. `dir` need not be normalized; only its
// direction is read.
//
// At the poles (x = y = 0) the azimuth is genuinely undefined; this reports 0, one of the infinitely
// many correct answers and the same anchor the overlay label placer picks for the same degeneracy.
// It is stated as its own branch rather than left to atan2, which reads the sign of a zero and
// answers -180 there — see the comment at the branch.
//
// Note that +180 and -180 are the SAME azimuth, both inside the slider's range, and which of the
// two comes back for a direction on the far side is decided by the sign of a zero. No caller can
// tell the resulting cameras apart, so neither value is more correct than the other.
void WorldDirToAzEl(const float dir[3], float* az_deg, float* el_deg);

// The whole preset: id + the sun's altitude (the only sun degree of freedom the GUI exposes) in,
// camera angles out.
//
// Deliberately does NOT clamp: the bounds are a property of the LENS — Globe stops one degree short
// of the pole where its view matrix degenerates — and they live in the field registry that also
// bounds the Az/El sliders, so a copy of them here would be free to drift. This is the raw
// geometric answer; ResolveLookAtPose below is the one a panel calls, and it is where the
// registry's own interval is applied.
//
// Returns false and leaves the outputs untouched if `id` is out of range or the C API rejects the
// query; the caller writes nothing in that case rather than pointing the camera somewhere arbitrary.
bool ResolveLookAtAzEl(LookAtId id, float sun_altitude_deg, float* out_az_deg, float* out_el_deg);

// The same thing, brought inside the bounds the Az/El sliders enforce. The caller passes the
// constraints it already read for those sliders, so the interval is not a second copy of them and
// cannot drift; what this function owns is only the decision that a preset is subject to it.
//
// WHY THIS IS A FUNCTION AND NOT TWO std::clamp CALLS AT THE CALL SITE. The panel redraws the two
// sliders every frame, and SliderWithInput clamps its value unconditionally (panels.cpp), so an
// out-of-range write is repaired one frame later whatever the preset did. That makes the clamp
// invisible to any test that reads the state through the panel: the observed value is the same
// either way. Naming the step gives the proposition somewhere to be judged — and the proposition is
// real, because the frame in which the preset writes is a frame the preview, the overlay cache and
// the export path all read, and under Globe the value they would read is the pole its view matrix
// is documented to degenerate at.
//
// Returns false and writes nothing when ResolveLookAtAzEl does.
bool ResolveLookAtPose(LookAtId id, float sun_altitude_deg, const FieldEditorConstraint& el_c,
                       const FieldEditorConstraint& az_c, float* out_az_deg, float* out_el_deg);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_VIEW_LOOK_AT_HPP
