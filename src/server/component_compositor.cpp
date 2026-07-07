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

// Painter: list order = z-order; the first (list-first) participating class
// with a positive exposed value at pixel p wins (top layer occludes below).
void CompositePainterPixel(const std::vector<ActiveClass>& active, const ColorClassTable& class_table, float s,
                           size_t p, float* out) {
  for (size_t k = 0; k < active.size(); ++k) {
    const float ey = active[k].lane[p] * s;
    if (ey > 0.0f) {
      const float* c = class_table.classes_[active[k].idx].color_;
      out[0] = c[0] * ey;
      out[1] = c[1] * ey;
      out[2] = c[2] * ey;
      break;
    }
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
    LOG_WARNING("raypath_color: unknown composite mode \"{}\"; falling back to \"dominant\"", mode_str);
    logged_unknown_mode = true;
  }
  return CompositeMode::kDominant;
}

bool CompositeColorClassesLinear(const RenderConsumer& consumer, const ColorClassTable& class_table, CompositeMode mode,
                                 std::vector<float>& out_linear_rgb) {
  if (class_table.referenced_mask_ == 0) {
    return false;
  }
  const float s = consumer.ExposureScale();
  if (s <= 0.0f) {
    return false;
  }

  const int w = consumer.ImageWidth();
  const int h = consumer.ImageHeight();
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  out_linear_rgb.assign(n * 3, 0.0f);
  if (n == 0) {
    return true;
  }

  const std::vector<ActiveClass> active = GatherActiveClasses(consumer, class_table);
  if (active.empty()) {
    return true;  // nothing visible → all-black, still a valid composite
  }

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
        CompositePainterPixel(active, class_table, s, p, out);
        break;
    }
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
