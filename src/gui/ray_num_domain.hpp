#ifndef LUMICE_GUI_RAY_NUM_DOMAIN_HPP
#define LUMICE_GUI_RAY_NUM_DOMAIN_HPP

// The Rays(M) slider's domain: its range, its display format and the scale its slider traverses.
// One definition for the TWO controls that draw it — the document's sim.ray_num_millions row
// (field_editor_registry.cpp, rendered by panels.cpp) and the raypath-analysis panel's own
// session-tier budget (analysis_panel.cpp). The two rows deliberately read different fields under
// different enable gates and are not rendered by a shared helper; what they must not do is disagree
// on what a ray budget can be, and the way to make that structurally impossible is for neither of
// them to hold a literal.
//
// The whole quadruple is shared, not just the bounds. The format and the scale are a pair: a
// relatively-quantizing law rendered through an absolute format freezes its readout over part of
// the travel (slider_format_rules.hpp states the criterion), so a consumer that copied the bounds
// and kept its own "%.1f" would be a silent precision fork that no compile catches. Sharing all
// four means a format change is one edit that both rows follow.
//
// The pairing gate lives HERE and not in a local block beside either consumer (the form
// sun.diameter takes in field_editor_registry.cpp): that form suits a domain with one consumer,
// where the constants and the row are the same few lines. This domain has two, and the value is
// written once, so the gate guards that one place — whichever translation unit includes this
// header compiles the assertion, and a consumer cannot render a quadruple the gate has not seen.
//
// Why the numbers are what they are:
//   - 0.1 M (1e5 rays) is the smallest finite budget worth a run; unchanged from the linear era.
//   - 100 000 M (1e11 rays) is the owner's "reproducible large budget" ceiling — hours of tracing on
//     a GPU, a working day on the CPU path — the bounded counterpart to Infinite rays, not a
//     replacement for it. LUMICE_RayCount is 64-bit, so the C API and core carry it whole.
//   - kLog, because six decades on one track need a per-pixel step that scales with the value:
//     under kLinear the whole 0.1..100 M range the slider used to span would sit in the first
//     pixel. kLogLinear is not an option here — its law requires min < kLogLinearX0 (0.01), and
//     this floor is above it (shape_scalar_domain.hpp static_asserts that requirement for the rows
//     that do use it).
//   - "%.6g": a relative format for a relative law, wide enough that no decade in [0.1, 1e5]
//     switches to exponent notation (%g does so only once the exponent reaches the precision), so
//     the input box reads "0.1", "5", "12345.6", "100000" — never "1e+05". The pairing gate
//     below is what actually decides the precision; the choice is confirmed by it, not by this
//     comment.

#include "gui/panels.hpp"               // SliderScale
#include "gui/slider_format_rules.hpp"  // FormatIsFineEnough -- the fmt/scale pairing gate

namespace lumice::gui {

inline constexpr float kRayNumMinMillions = 0.1f;
inline constexpr float kRayNumMaxMillions = 100000.0f;
inline constexpr const char* kRayNumSliderFmt = "%.6g";
inline constexpr SliderScale kRayNumSliderScale = SliderScale::kLog;

static_assert(kRayNumMinMillions > 0.0f, "a kLog slider needs a strictly positive floor (panels.cpp routes on it)");
static_assert(slider_format::FormatIsFineEnough(kRayNumSliderFmt, kRayNumSliderScale, kRayNumMinMillions,
                                                kRayNumMaxMillions),
              "the Rays(M) slider's display format is coarser than its mapping resolves: the readout would freeze "
              "over part of the travel (see gui/slider_format_rules.hpp)");

}  // namespace lumice::gui

#endif  // LUMICE_GUI_RAY_NUM_DOMAIN_HPP
