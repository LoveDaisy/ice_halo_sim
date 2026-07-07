#include "server/component_compositor.hpp"

#include <algorithm>
#include <cstddef>

#include "config/component_color_map.hpp"
#include "config/component_table.hpp"
#include "config/raypath_color_config.hpp"
#include "server/render.hpp"
#include "util/color_space.hpp"
#include "util/logger.hpp"

namespace lumice {

CompositeOptions BuildCompositeOptions(const RaypathColorConfig& color_cfg, const ComponentTable& table) {
  CompositeOptions options;

  // Mode string → enum. Unknown strings fall back to dominant with a one-shot
  // warning (a config typo should degrade to the sensible default, not fail).
  if (color_cfg.mode_ == "dominant") {
    options.mode_ = CompositeMode::kDominant;
  } else if (color_cfg.mode_ == "additive") {
    options.mode_ = CompositeMode::kAdditive;
  } else if (color_cfg.mode_ == "painter") {
    options.mode_ = CompositeMode::kPainter;
  } else {
    static bool logged_unknown_mode = false;
    if (!logged_unknown_mode) {
      LOG_WARNING("raypath_color: unknown composite mode \"{}\"; falling back to \"dominant\"", color_cfg.mode_);
      logged_unknown_mode = true;
    }
    options.mode_ = CompositeMode::kDominant;
  }

  // Fold per-entry visibility into hidden_mask_/solo_mask_ via the SAME triple
  // → bit lookup BuildComponentColorMap uses (so the masks agree bit-for-bit
  // with color_map.colored_mask_). Overflowed / absent triples resolve to no
  // usable bit and are skipped — BuildComponentColorMap already validated the
  // triples (throw on absent, warn on overflow) before this runs.
  for (const auto& e : color_cfg.entries_) {
    const auto* entry = FindComponentTableEntry(table, e.layer_, e.crystal_id_, e.summand_idx_);
    if (entry == nullptr || entry->bit_ == ComponentTable::kNoBit) {
      continue;
    }
    const uint64_t bit_mask = static_cast<uint64_t>(1) << entry->bit_;
    if (!e.visible_) {
      options.hidden_mask_ |= bit_mask;
    }
    if (e.solo_) {
      options.solo_mask_ |= bit_mask;
    }
  }

  return options;
}

bool CompositeComponentLinear(const RenderConsumer& consumer, const ComponentColorMap& color_map,
                              const CompositeOptions& options, std::vector<float>& out_linear_rgb) {
  const uint64_t participating = color_map.colored_mask_;
  if (participating == 0) {
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

  // Visibility axis (plan §4): solo (when any bit is solo'd) restricts to the
  // solo set; otherwise the visible set is everything not hidden.
  const uint64_t vis_mask =
      (options.solo_mask_ != 0) ? (participating & options.solo_mask_) : (participating & ~options.hidden_mask_);
  if (vis_mask == 0) {
    return true;  // nothing visible → all-black, still a valid composite
  }

  // Gather the visible participating bits (ascending) with a live lane.
  std::vector<uint8_t> vis_bits;
  const float* lanes[ComponentTable::kMaxBits] = {};
  for (uint8_t bit = 0; bit < ComponentTable::kMaxBits; ++bit) {
    if (((vis_mask >> bit) & 1ULL) == 0) {
      continue;
    }
    const float* lane = consumer.GetComponentLaneY(bit);
    if (lane == nullptr) {
      continue;  // defensive: a visible bit without an allocated lane
    }
    vis_bits.push_back(bit);
    lanes[bit] = lane;
  }
  if (vis_bits.empty()) {
    return true;
  }

  for (size_t p = 0; p < n; ++p) {
    float* out = &out_linear_rgb[p * 3];
    switch (options.mode_) {
      case CompositeMode::kDominant: {
        // argmax over visible bits, strict > with ascending scan → min-bit tie.
        int best_bit = -1;
        float best_ey = 0.0f;
        for (uint8_t bit : vis_bits) {
          const float ey = lanes[bit][p] * s;
          if (ey > best_ey) {
            best_ey = ey;
            best_bit = bit;
          }
        }
        if (best_bit >= 0) {
          const float* c = color_map.colors_[best_bit];
          out[0] = c[0] * best_ey;
          out[1] = c[1] * best_ey;
          out[2] = c[2] * best_ey;
        }
        break;
      }
      case CompositeMode::kAdditive: {
        float acc[3] = { 0.0f, 0.0f, 0.0f };
        for (uint8_t bit : vis_bits) {
          const float ey = lanes[bit][p] * s;
          if (ey <= 0.0f) {
            continue;
          }
          const float* c = color_map.colors_[bit];
          acc[0] += c[0] * ey;
          acc[1] += c[1] * ey;
          acc[2] += c[2] * ey;
        }
        out[0] = std::clamp(acc[0], 0.0f, 1.0f);
        out[1] = std::clamp(acc[1], 0.0f, 1.0f);
        out[2] = std::clamp(acc[2], 0.0f, 1.0f);
        break;
      }
      case CompositeMode::kPainter: {
        // Component order = bit ascending; the first (lowest) bit is the top
        // layer. Take the first visible bit with a positive exposed value.
        for (uint8_t bit : vis_bits) {
          const float ey = lanes[bit][p] * s;
          if (ey > 0.0f) {
            const float* c = color_map.colors_[bit];
            out[0] = c[0] * ey;
            out[1] = c[1] * ey;
            out[2] = c[2] * ey;
            break;
          }
        }
        break;
      }
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
