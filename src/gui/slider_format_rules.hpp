#ifndef LUMICE_GUI_SLIDER_FORMAT_RULES_HPP
#define LUMICE_GUI_SLIDER_FORMAT_RULES_HPP

// Whether a slider's display format is fine enough for the law that slider traverses — decided by
// the mapping, at compile time, instead of by whoever typed the format string.
//
// THE DEFECT THIS ADDRESSES. A slider and the number beside it quantize independently. "%.Nf"
// quantizes ABSOLUTELY (it distinguishes values that differ by more than 10^-N); "%.Ng" quantizes
// RELATIVELY (by more than a fixed fraction of the value). The slider quantizes by its own law: a
// kLinear slider moves the value a constant amount per pixel of travel, kLog and kLogLinear's log
// segment move it a constant fraction, kSqrt neither. Pair a format that is too coarse for the law
// with the law, and a stretch of travel renders as a run of identical strings: the handle moves and
// the number does not, which is indistinguishable from a frozen control.
//
// THE CRITERION IS `maxrun <= K`, NOT "no two adjacent pixels agree". maxrun is the longest run of
// consecutive pixels that format to the same string, i.e. the length of the dead travel a user
// actually drags through. The two orderings are not merely different, they are close to reversed:
// measured over this GUI's float sliders at the 179 px width, the sun-diameter row produced 130
// adjacent-pixel collisions but a maxrun of 4, the six `*_alpha` rows produced 79 collisions at a
// maxrun of 2 (one repeated pixel, benign), while the worst row of all — the prism height under its
// old "%.2f" — produced only 57 collisions at a maxrun of 20. Counting collisions ranks the worst
// row as the mildest. K is the tolerance that ordering needs and a collision count cannot express:
// "the number must change within K pixels of drag".
//
// WHAT THIS FILE DOES AND DOES NOT DECIDE. It answers one question — is the format a call site
// DECLARED fine enough — and never picks the format itself. Deriving the format instead would mean
// taking the derived bound as the answer, and the bound is a LOWER bound: taken literally it would
// coarsen rows that are fine today (the six face rows from "%.3f" to "%.2f", the axis mean from
// "%.3f" to "%.0f"). Rows are free to be finer than the bound; only a row coarser than it is
// rejected, so this gate only ever subtracts defects, never rewrites working call sites.
//
// HONEST BOUNDARY: THE BOUND IS SUFFICIENT, NOT NECESSARY. For the constant-step laws (kLinear,
// kLog, kLogLinear) the bound agrees with a brute-force pixel scan exactly. For kSqrt, whose step
// varies along the travel, and for any "%.Ng" whose domain does not start just above a power of ten
// (see WorstRelativeQuantum below), it is conservative by up to one digit — it can reject a format
// that would in fact have measured fine. Every disagreement found in cross-checking went that way;
// none went the other. So the gate does not miss, but it can demand a digit a measurement would
// not. When it does, the fix is to declare the finer format (or to pick the format family that
// matches the law, which is what the domain wanted anyway) — NOT to loosen the gate. The
// cross-check that establishes the direction of the error lives beside the gate's own tests in
// test/unit-correctness/gui/test_gui_widget_rules.cpp.
//
// No <cmath>, and deliberately so: `std::log` is not usable in a constant expression, and
// `__builtin_logf` is usable in one under GCC but not under Clang, so reaching for it would buy a
// compile that succeeds on one of this repo's three platforms and fails on another. Nothing below
// needs a transcendental — the W-th root the log laws would otherwise want is removed by raising
// both sides of the comparison to the W-th power, leaving repeated multiplication.

#include "gui/panels.hpp"          // SliderScale
#include "gui/slider_mapping.hpp"  // kLogLinearX0 / kLogLinearTSwitch -- the hybrid law's shape

namespace lumice::gui::slider_format {

// The widest a Value-column slider gets: measured with the crystal edit modal's own layout, from
// the item rectangle ImGui gave the slider -- 179 px in the horizontal (820 px) modal and 127 px in
// the vertical (420 px) one. The slider does NOT span the modal: it lives in the single stretch
// column of a five-column property table, alongside four fixed-width cells.
inline constexpr int kMeasuredMaxSliderWidthPx = 179;

// The width the gate derives against. Three times the measured one, because collisions appear as a
// slider gets WIDER (more pixels over the same range means a smaller step per pixel) and never
// disappear: clearing the gate at 3x today's width means the width at which the row would first
// break is beyond 3x, which is a margin rather than a coin flip.
//
// The multiple lives HERE, inside the derivation, rather than in the test that used to carry it.
// That relocation is the point: as a test-side constant it was a stand-in for a quantity nobody had
// derived, and it had a known hole -- a "%.4f" row whose first collision was measured at 675 px sat
// inside the 3x margin and the gate said nothing. Folded into a bound on the format's precision,
// the layout margin is a term in the answer instead of a guess bolted onto one.
inline constexpr int kPairingGateWidthPx = 3 * kMeasuredMaxSliderWidthPx;

// K: the dead travel a reader will tolerate, in pixels. 3 means "drag three pixels and the number
// must have changed". K = 1 is the strictest setting and is what "no two adjacent pixels agree"
// amounted to; at K = 1 the six `*_alpha` rows (0..1 paired with "%.2f", measured maxrun 2) would
// be rejected and pushed to "%.3f", i.e. to rendering 0.530 — which is why the strictest setting is
// not the right one. Owner-tunable: change this one number and every covered row is re-derived
// against it at the next compile.
inline constexpr int kPairingGateSlackPx = 3;

// ---- Format-string parsing ----

// A "%.Nf" / "%.Ng" spelling, split into the two things the derivation needs.
struct ParsedFormat {
  bool recognized = false;
  int digits = 0;
  char kind = '\0';  // 'f' or 'g'
};

// Parses exactly "%.Nf" and "%.Ng" with N one or two digits, and NOTHING else.
//
// The narrowness is a feature, not a limitation to be widened on demand: every format string in the
// covered set has this shape, and an unrecognized one leaves `recognized` false, which the gate
// below reports as "not fine enough". So a call site that invents a spelling this parser cannot
// read fails the compile with the same message a genuinely-too-coarse format gets, rather than
// being waved through on a branch nobody wrote a bound for.
constexpr ParsedFormat ParseSliderFormat(const char* fmt) {
  if (fmt == nullptr || fmt[0] != '%' || fmt[1] != '.') {
    return {};
  }
  int i = 2;
  int digits = 0;
  int seen = 0;
  while (fmt[i] >= '0' && fmt[i] <= '9') {
    digits = digits * 10 + (fmt[i] - '0');
    ++i;
    ++seen;
  }
  if (seen == 0 || seen > 2) {
    return {};
  }
  const char kind = fmt[i];
  if ((kind != 'f' && kind != 'g') || fmt[i + 1] != '\0') {
    return {};
  }
  // printf reads a precision of 0 for %g as 1 significant digit; mirror that rather than deriving a
  // bound for a precision the runtime will not use.
  if (kind == 'g' && digits == 0) {
    digits = 1;
  }
  return { true, digits, kind };
}

// ---- Arithmetic helpers (no transcendentals, no overflow) ----

// 10^exp for the small exponents a printf precision produces.
constexpr double Pow10(int exp) {
  double result = 1.0;
  for (int i = 0; i < exp; ++i) {
    result *= 10.0;
  }
  for (int i = 0; i > exp; --i) {
    result /= 10.0;
  }
  return result;
}

// Whether base^n <= limit, for base >= 1 and limit > 0.
//
// Written as repeated multiplication with an early exit rather than as a power, because the honest
// answer for a base that runs away is "no" and the arithmetic that would produce it overflows. A
// floating-point overflow is not a constant expression: forming it would make the enclosing
// static_assert fail to COMPILE with a range diagnostic instead of failing with the gate's own
// message. Stopping the moment the running product passes `limit` keeps every intermediate value
// bounded by limit * base and answers the comparison exactly.
constexpr bool PowerAtMost(double base, int n, double limit) {
  if (base > limit && n >= 1) {
    return false;
  }
  double acc = 1.0;
  for (int i = 0; i < n; ++i) {
    if (acc > limit) {
      return false;
    }
    acc *= base;
  }
  return acc <= limit;
}

constexpr double AbsValue(double v) {
  return v < 0.0 ? -v : v;
}

// ---- The laws' per-pixel steps, as inequalities rather than as values ----

// The mapping a slider ACTUALLY traverses, which is not always the one its call site names:
// RenderNonlinearSlider (panels.cpp) falls through to the plain linear branch when the named law
// cannot be applied to the declared floor -- kLog needs min_val > 0, kSqrt and kLogLinear need
// min_val >= 0. Deriving against the named law where the widget runs the linear one would be
// deriving a bound for a slider nobody renders.
constexpr SliderScale EffectiveScale(SliderScale scale, double min_v) {
  switch (scale) {
    case SliderScale::kSqrt:
      return min_v >= 0.0 ? SliderScale::kSqrt : SliderScale::kLinear;
    case SliderScale::kLog:
      return min_v > 0.0 ? SliderScale::kLog : SliderScale::kLinear;
    case SliderScale::kLogLinear:
      return min_v >= 0.0 ? SliderScale::kLogLinear : SliderScale::kLinear;
    case SliderScale::kLinear:
      break;
  }
  return SliderScale::kLinear;
}

// The log segment of the hybrid law occupies the upper (1 - t_switch) of the travel. Rounding the
// pixel count UP is the safe direction: fewer pixels over the same ratio means a LARGER step per
// pixel, so rounding down would credit the row with resolution it does not have.
constexpr int LogLinearLogSegmentPixels(int width_px) {
  const double exact = static_cast<double>(width_px) * (1.0 - static_cast<double>(slider_mapping::kLogLinearTSwitch));
  int pixels = static_cast<int>(exact);
  if (static_cast<double>(pixels) < exact) {
    ++pixels;
  }
  return pixels < 1 ? 1 : pixels;
}

// Whether `slack * (smallest absolute step per pixel) >= quantum`, i.e. whether an ABSOLUTE-
// quantizing format with that quantum resolves every K-pixel drag. The smallest step per law:
//
//   kLinear     (hi - lo) / W                          constant along the travel
//   kSqrt       hi / W^2                               at the bottom (value = norm^2 * hi)
//   kLog        lo * r,  r = (hi/lo)^(1/W) - 1         at the bottom
//   kLogLinear  min( (x0 - lo) / (W * t_s),  x0 * r_log )   linear segment vs. log segment's foot
//
// The two log forms are not evaluated. `slack * lo * r >= q` rearranges to
// `hi/lo >= (1 + q/(slack*lo))^W`, which has no fractional power left in it, and PowerAtMost
// answers that by multiplying.
constexpr bool AbsoluteStepResolves(SliderScale scale, double min_v, double max_v, int width_px, int slack_px,
                                    double quantum) {
  const double w = static_cast<double>(width_px);
  const double k = static_cast<double>(slack_px);
  const double x0 = static_cast<double>(slider_mapping::kLogLinearX0);
  const double ts = static_cast<double>(slider_mapping::kLogLinearTSwitch);
  switch (EffectiveScale(scale, min_v)) {
    case SliderScale::kSqrt:
      // The sqrt slider ignores its declared floor: it travels 0..sqrt(max) and squares back, so
      // its bottom step is set by max_v alone.
      return k * max_v / (w * w) >= quantum;
    case SliderScale::kLog:
      return PowerAtMost(1.0 + quantum / (k * min_v), width_px, max_v / min_v);
    case SliderScale::kLogLinear:
      return k * (x0 - min_v) / (w * ts) >= quantum &&
             PowerAtMost(1.0 + quantum / (k * x0), LogLinearLogSegmentPixels(width_px), max_v / x0);
    case SliderScale::kLinear:
      break;
  }
  return k * (max_v - min_v) / w >= quantum;
}

// Whether `slack * (smallest RELATIVE step per pixel) >= quantum`, for a relatively-quantizing
// format. Relative step means (step at a pixel) / (value at that pixel); the smallest one per law:
//
//   kLinear     (hi - lo) / (W * max|endpoint|)        at the top, where the constant step is
//                                                      smallest as a fraction
//   kSqrt       (2W - 1) / (W - 1)^2                   at the top; independent of the domain
//   kLog        r                                      constant along the travel
//   kLogLinear  min( (x0 - lo) / (W * t_s * x0),  r_log )
constexpr bool RelativeStepResolves(SliderScale scale, double min_v, double max_v, int width_px, int slack_px,
                                    double quantum) {
  const double w = static_cast<double>(width_px);
  const double k = static_cast<double>(slack_px);
  const double x0 = static_cast<double>(slider_mapping::kLogLinearX0);
  const double ts = static_cast<double>(slider_mapping::kLogLinearTSwitch);
  switch (EffectiveScale(scale, min_v)) {
    case SliderScale::kSqrt:
      return k * (2.0 * w - 1.0) / ((w - 1.0) * (w - 1.0)) >= quantum;
    case SliderScale::kLog:
      return PowerAtMost(1.0 + quantum / k, width_px, max_v / min_v);
    case SliderScale::kLogLinear:
      return k * (x0 - min_v) / (w * ts * x0) >= quantum &&
             PowerAtMost(1.0 + quantum / k, LogLinearLogSegmentPixels(width_px), max_v / x0);
    case SliderScale::kLinear:
      break;
  }
  {
    const double scale_ref = AbsValue(min_v) > AbsValue(max_v) ? AbsValue(min_v) : AbsValue(max_v);
    return scale_ref > 0.0 && k * (max_v - min_v) / (w * scale_ref) >= quantum;
  }
}

// The coarsest relative quantization "%.Ng" can produce anywhere. N significant digits at a value
// in [10^e, 10^(e+1)) quantize by 10^(e-N+1), i.e. by a fraction between 10^-N (just under the next
// decade) and 10^(1-N) (just above 10^e). Taking the larger end is what makes the "%.Ng" arm of the
// gate hold for every domain without the gate having to know which decades a domain spans; it is
// also where its conservatism comes from, since the worst relative step and the worst relative
// quantum need not occur at the same value.
constexpr double WorstRelativeQuantum(int digits) {
  return Pow10(1 - digits);
}

// Whether `fmt` resolves every `slack_px`-pixel drag of a `scale` slider over [min_v, max_v] laid
// out `width_px` wide -- i.e. whether the declared format is at least as fine as the bound the
// mapping implies. False for a format string this file cannot parse (see ParseSliderFormat).
constexpr bool FormatIsFineEnough(const char* fmt, SliderScale scale, float min_v, float max_v, int width_px,
                                  int slack_px) {
  const ParsedFormat parsed = ParseSliderFormat(fmt);
  if (!parsed.recognized || width_px < 2 || slack_px < 1) {
    return false;
  }
  const double lo = static_cast<double>(min_v);
  const double hi = static_cast<double>(max_v);
  if (!(hi > lo)) {
    return false;
  }
  if (parsed.kind == 'f') {
    return AbsoluteStepResolves(scale, lo, hi, width_px, slack_px, Pow10(-parsed.digits));
  }
  return RelativeStepResolves(scale, lo, hi, width_px, slack_px, WorstRelativeQuantum(parsed.digits));
}

// The gate at its configured settings — the spelling every call site should use, so that the width
// and the tolerance are not re-typed per row.
constexpr bool FormatIsFineEnough(const char* fmt, SliderScale scale, float min_v, float max_v) {
  return FormatIsFineEnough(fmt, scale, min_v, max_v, kPairingGateWidthPx, kPairingGateSlackPx);
}

}  // namespace lumice::gui::slider_format

#endif  // LUMICE_GUI_SLIDER_FORMAT_RULES_HPP
