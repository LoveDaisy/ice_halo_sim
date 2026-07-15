#include "server/component_compositor.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "config/color_class_table.hpp"
#include "server/render.hpp"
#include "util/color_space.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

// A color class that participates in this frame's composite, with the live
// per-pixel exposed-Y lane it contributes.
struct ActiveClass {
  size_t idx;
  const float* lane;
};

// Visibility axis (plan §4): solo (when any class is solo'd) restricts to the
// solo set; otherwise the participating set is every visible class. Iteration
// order = ascending z_order_ (task-342.2); ties preserved via stable sort so
// classes with identical z_order_ still walk in vector order. The physical
// index i (== vector position) is retained in ActiveClass::idx so lane access
// through `consumer.GetColorClassLaneY(i)` continues to hit the same permanent
// binding built at RenderConsumer construction — z_order_ ONLY reorders the
// visit sequence, it does NOT reorder the accumulator lanes. A class whose
// lane accessor returns nullptr is defensively skipped (same spirit as the old
// per-bit "visible bit without an allocated lane" guard).
std::vector<ActiveClass> GatherActiveClasses(const RenderConsumer& consumer, const ColorClassTable& class_table) {
  bool any_solo = false;
  for (const auto& cls : class_table.classes_) {
    if (cls.solo_) {
      any_solo = true;
      break;
    }
  }

  std::vector<size_t> order(class_table.classes_.size());
  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  std::stable_sort(order.begin(), order.end(), [&class_table](size_t a, size_t b) {
    return class_table.classes_[a].z_order_ < class_table.classes_[b].z_order_;
  });

  std::vector<ActiveClass> active;
  active.reserve(class_table.classes_.size());
  for (size_t idx : order) {
    const auto& cls = class_table.classes_[idx];
    const bool participates = any_solo ? cls.solo_ : cls.visible_;
    if (!participates) {
      continue;
    }
    const float* lane = consumer.GetColorClassLaneY(idx);
    if (lane == nullptr) {
      continue;
    }
    active.push_back({ idx, lane });
  }
  return active;
}

// Dominant: argmax over participating classes at pixel p, strict > with
// ascending scan → ties go to the class earlier in the list.
void CompositeDominantPixel(const std::vector<ActiveClass>& active, const ColorClassTable& class_table, float s,
                            size_t p, float* out) {
  int best = -1;
  float best_ey = 0.0f;
  for (size_t k = 0; k < active.size(); ++k) {
    const float ey = active[k].lane[p] * s;
    if (ey > best_ey) {
      best_ey = ey;
      best = static_cast<int>(k);
    }
  }
  if (best >= 0) {
    const float* c = class_table.classes_[active[static_cast<size_t>(best)].idx].color_;
    out[0] = c[0] * best_ey;
    out[1] = c[1] * best_ey;
    out[2] = c[2] * best_ey;
  }
}

// Additive: sum every participating class's exposed contribution at pixel p,
// clamped to [0, 1].
void CompositeAdditivePixel(const std::vector<ActiveClass>& active, const ColorClassTable& class_table, float s,
                            size_t p, float* out) {
  float acc[3] = { 0.0f, 0.0f, 0.0f };
  for (size_t k = 0; k < active.size(); ++k) {
    const float ey = active[k].lane[p] * s;
    if (ey <= 0.0f) {
      continue;
    }
    const float* c = class_table.classes_[active[k].idx].color_;
    acc[0] += c[0] * ey;
    acc[1] += c[1] * ey;
    acc[2] += c[2] * ey;
  }
  out[0] = std::clamp(acc[0], 0.0f, 1.0f);
  out[1] = std::clamp(acc[1], 0.0f, 1.0f);
  out[2] = std::clamp(acc[2], 0.0f, 1.0f);
}

// Painter: Porter-Duff "over" front-to-back with `alpha = min(ey, 1)` where
// `ey = lane[p] * participating_exposure_scale` (self-anchor only — NO
// display_exposure_scale here; the caller post-multiplies display EV after
// composite; see doc/gui-custom-spectrum-and-raypath-color.md §4.8). List
// order is ascending z_order (list-first = top layer), so the front-to-back
// recurrence walks the list in-order without reversing:
//   T = 1 (unoccluded transmittance)
//   for k in top..bottom while T > 0:
//     alpha = min(ey_k, 1)
//     out  += T * alpha * color_k   // color slot holds PURE HUE, not color*ey
//     T    *= (1 - alpha)
// out ∈ [0,1]^3 by construction (Σ T_k·alpha_k ≤ 1 - T_final ≤ 1, per-channel
// color ∈ [0,1]). Fixes the old "binary occluder" black-hole where a top-layer
// class with any positive ey (however dim) 100% masked brighter classes below.
void CompositePainterPixel(const std::vector<ActiveClass>& active, const ColorClassTable& class_table,
                           float participating_exposure_scale, size_t p, float* out) {
  float T = 1.0f;
  for (size_t k = 0; k < active.size() && T > 0.0f; ++k) {
    const float alpha = std::min(active[k].lane[p] * participating_exposure_scale, 1.0f);
    if (alpha <= 0.0f) {
      continue;
    }
    const float* c = class_table.classes_[active[k].idx].color_;
    out[0] += T * alpha * c[0];
    out[1] += T * alpha * c[1];
    out[2] += T * alpha * c[2];
    T *= (1.0f - alpha);
  }
}

}  // namespace

CompositeMode ParseCompositeMode(const std::string& mode_str) {
  if (mode_str == "dominant") {
    return CompositeMode::kDominant;
  }
  if (mode_str == "additive") {
    return CompositeMode::kAdditive;
  }
  if (mode_str == "painter") {
    return CompositeMode::kPainter;
  }
  static bool logged_unknown_mode = false;
  if (!logged_unknown_mode) {
    LOG_WARNING("raypath_color: unknown composite mode \"{}\"; falling back to \"painter\"", mode_str);
    logged_unknown_mode = true;
  }
  return CompositeMode::kPainter;
}

namespace {

// P99 over the union of NON-ZERO UNEXPOSED (raw lane) Y values across every
// participating class — the composite-path auto-EV anchor (task-345.3 Step 2).
// Kept as the union-of-lanes rather than a post-composite scan so it stays
// mode-independent (dominant / additive / painter all share the same anchor)
// and does not create a circular dependency between EV-scaled composite and
// the very P99 that drives EV. Non-participating classes (invisible / masked
// by solo) are excluded by construction — GatherActiveClasses has already
// dropped them — which is the whole point of "participating union" vs the
// pre-345.3 mono-xyz-derived P99 that mixed in unrelated pixels.
//
// NOTE (task-345.3 review Minor #1): the (idx * 0.99) partial-sort algorithm
// here is structurally identical to `gui_ev_auto.hpp::ComputeP99Y`'s fine
// path — the two live apart because pulling a shared header down to server/
// or up to gui/ would drag one layer into the other. If you touch one, mirror
// the change here.
float ComputeParticipatingP99Y(const std::vector<ActiveClass>& active, size_t pixel_count) {
  if (active.empty() || pixel_count == 0) {
    return 0.0f;
  }
  std::vector<float> y_vals;
  // Upper bound: every pixel non-zero on every class. Under-allocating is
  // harmless (vector grows); pre-reserving avoids repeated reallocation in
  // the additive-overlap case where multiple classes contribute at the same
  // pixel.
  y_vals.reserve(active.size() * pixel_count);
  for (const auto& ac : active) {
    for (size_t p = 0; p < pixel_count; ++p) {
      const float v = ac.lane[p];
      if (v > 0.0f) {
        y_vals.push_back(v);
      }
    }
  }
  if (y_vals.empty()) {
    return 0.0f;
  }
  auto idx = static_cast<size_t>(static_cast<float>(y_vals.size()) * 0.99f);
  if (idx >= y_vals.size()) {
    idx = y_vals.size() - 1;
  }
  std::nth_element(y_vals.begin(), y_vals.begin() + static_cast<ptrdiff_t>(idx), y_vals.end());
  return y_vals[idx];
}

}  // namespace

bool CompositeColorClassesLinear(const RenderConsumer& consumer, const ColorClassTable& class_table, CompositeMode mode,
                                 float display_exposure_scale, std::vector<float>& out_linear_rgb,
                                 float* out_participating_p99_y) {
  if (class_table.referenced_mask_ == 0) {
    return false;
  }

  const int w = consumer.ImageWidth();
  const int h = consumer.ImageHeight();
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  out_linear_rgb.assign(n * 3, 0.0f);
  if (n == 0) {
    if (out_participating_p99_y != nullptr) {
      *out_participating_p99_y = 0.0f;
    }
    return true;
  }

  // task-347 (Fix B) reorder: gather participants + compute the anchor P99
  // BEFORE the exposure scalar. The old order ran ComputeParticipatingP99Y
  // AFTER the pixel loop (only as an out-parameter report); moving it here
  // lets the P99 feed ParticipatingExposureScale so a visibility change
  // re-anchors the composite in the SAME DoSnapshot call.
  const std::vector<ActiveClass> active = GatherActiveClasses(consumer, class_table);
  if (active.empty()) {
    if (out_participating_p99_y != nullptr) {
      *out_participating_p99_y = 0.0f;
    }
    return true;  // nothing visible → all-black, still a valid composite
  }
  const float participating_p99 = ComputeParticipatingP99Y(active, n);

  // Single-scalar exposure invariant (a03 / doc §4.3): every participating
  // lane sees the SAME anchor; per-lane / per-class renormalization is the
  // scrum-336 spike's false-color bug and is structurally excluded. Fix B:
  // the anchor `A` comes from the participating-P99 self-anchor (not the mono
  // ExposureScale) so hiding a bright class instantly re-anchors the rest.
  //
  // doc §4.8: split A / s.
  //   - dominant / additive: keep the pre-existing single scalar
  //     `s = A * display_exposure_scale` (byte-for-byte behavior preserved).
  //   - painter: alpha driven by `A` alone (self-anchor only) so the occluder
  //     structure is EV-independent; `display_exposure_scale` becomes a pure
  //     post-composite brightness multiplier + clamp, applied per-pixel below.
  const float A = consumer.ParticipatingExposureScale(participating_p99);
  if (A <= 0.0f) {
    // task-347 semantic tightening: `participating_p99` is now known even on
    // this early-return path (it was computed above). Publish it so consumers
    // that read the anchor field regardless of return value see the true
    // current value (all in-tree consumers either skip the false-return
    // renderer entirely or pre-zero the field, so this is a no-op in practice
    // — kept for defense-in-depth against a caller that inspects the field
    // without checking the return).
    if (out_participating_p99_y != nullptr) {
      *out_participating_p99_y = participating_p99;
    }
    return false;
  }
  const float s = A * display_exposure_scale;

  for (size_t p = 0; p < n; ++p) {
    float* out = &out_linear_rgb[p * 3];
    switch (mode) {
      case CompositeMode::kDominant:
        CompositeDominantPixel(active, class_table, s, p, out);
        break;
      case CompositeMode::kAdditive:
        CompositeAdditivePixel(active, class_table, s, p, out);
        break;
      case CompositeMode::kPainter:
        CompositePainterPixel(active, class_table, A, p, out);
        // display EV = pure post-composite brightness (doc §4.8): the painter
        // occluder structure is set by `A` alone; clamp keeps output ∈ [0,1]^3.
        out[0] = std::clamp(out[0] * display_exposure_scale, 0.0f, 1.0f);
        out[1] = std::clamp(out[1] * display_exposure_scale, 0.0f, 1.0f);
        out[2] = std::clamp(out[2] * display_exposure_scale, 0.0f, 1.0f);
        break;
    }
  }

  if (out_participating_p99_y != nullptr) {
    *out_participating_p99_y = participating_p99;
  }

  return true;
}

void LinearRgbToSrgbU8(const std::vector<float>& linear_rgb, std::vector<uint8_t>& out_srgb) {
  const size_t n = linear_rgb.size();
  out_srgb.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const float clamped = std::clamp(linear_rgb[i], 0.0f, 1.0f);
    out_srgb[i] = static_cast<uint8_t>(LinearToSrgb(clamped) * 255.0f);
  }
}

}  // namespace lumice
