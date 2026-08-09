#ifndef LUMICE_GUI_SHAPE_SCALAR_DOMAIN_HPP
#define LUMICE_GUI_SHAPE_SCALAR_DOMAIN_HPP

// What each randomizable crystal shape scalar allows: its range, its display format and the
// scale its slider traverses, keyed by LUMICE_SHAPE_SCALAR_*.
//
// This table already existed — as PROSE, in slider_mapping.hpp's header comment ("Call sites in
// edit_modals.cpp MUST match one of these rows"), with the actual numbers hand-written at each of
// the ten RenderShapeDistTableRow call sites. A comment cannot be read by a test and cannot be
// checked against the call sites, so the two were free to drift and the only thing that would
// notice was a human re-reading both. Same numbers, moved to where a test can enumerate them.
//
// The key is the scalar slot, not (slot, crystal type): the slots are already type-specific
// (HEIGHT is prism-only, UPPER_H / PRISM_H / LOWER_H are pyramid-only, FACE_0..5 are shared), so
// the type adds nothing the slot does not already say. The six face slots share one row on
// purpose — that is what the call site does today (one `kFaceSpreadMax` for all six) — but they
// are stored per-slot rather than as a range test, so giving one face its own domain later is a
// data edit, not a control-flow edit.
//
// NOT covered here, stated so the boundary is not read as wider than it is: the two pyramid wedge
// angles (upper_alpha / lower_alpha) are not randomizable and go through RenderWedgeTableRow, which
// carries its own (0.1, 90) band and preset list.

#include <cassert>

#include "gui/panels.hpp"    // SliderScale
#include "include/lumice.h"  // LUMICE_SHAPE_SCALAR_*

namespace lumice::gui {

// One shape scalar's editable domain, in the shape SliderWithInput consumes.
struct ShapeScalarDomain {
  float min_value = 0.0f;
  float max_value = 0.0f;
  const char* fmt = "%.3f";
  SliderScale scale = SliderScale::kLinear;
};

// Indexed by LUMICE_SHAPE_SCALAR_* — the enum's own order, so a new slot added to lumice.h that
// is not given a row here fails the static_assert below rather than silently reading a neighbour.
inline constexpr ShapeScalarDomain kShapeScalarDomains[] = {
  // HEIGHT — prism height spans four orders of magnitude, hence log.
  { 0.01f, 100.0f, "%.2f", SliderScale::kLog },
  // UPPER_H — pyramid wedge fraction; natural linear unit.
  { 0.0f, 1.0f, "%.3f", SliderScale::kLinear },
  // PRISM_H — must allow exactly 0 AND stay finely controllable near 0 over a wide range.
  { 0.0f, 100.0f, "%.4f", SliderScale::kLogLinear },
  // LOWER_H — pyramid wedge fraction; natural linear unit.
  { 0.0f, 1.0f, "%.3f", SliderScale::kLinear },
  // FACE_0..FACE_5 — near-unit distance multipliers, rarely above 2 in practice.
  { 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
};
static_assert(sizeof(kShapeScalarDomains) / sizeof(kShapeScalarDomains[0]) == LUMICE_SHAPE_SCALAR_COUNT,
              "kShapeScalarDomains must carry one row per LUMICE_SHAPE_SCALAR_* slot");

// The domain for `scalar_id`. Total over the enum; out-of-range ids are a programming error the
// caller cannot recover from, so this asserts rather than substituting a domain (a wrong band on
// a geometry parameter is worse than a crash — it silently accepts values the real control
// refuses; see field_editor_registry.hpp's identical stance on unregistered keys).
inline const ShapeScalarDomain& ShapeScalarDomainFor(int scalar_id) {
  assert(scalar_id >= 0 && scalar_id < LUMICE_SHAPE_SCALAR_COUNT);
  return kShapeScalarDomains[scalar_id];
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SHAPE_SCALAR_DOMAIN_HPP
