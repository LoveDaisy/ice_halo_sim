#ifndef CONFIG_COMPONENT_COLOR_MAP_H_
#define CONFIG_COMPONENT_COLOR_MAP_H_

#include <array>
#include <cstdint>

#include "config/component_table.hpp"

namespace lumice {

// Runtime per-bit lookup (interim shape retained across task-339.2 for the
// consumer/compositor migration). Bit i in `colored_mask_` is set iff
// `colors_[i]` carries a user-configured RGB for that component bit. Produced
// by ToLegacyColorMap(ColorClassTable) — the flat entries+FindComponentTableEntry
// builder was retired with the schema switch to color classes; per-class
// consumption ships in 339.3/339.4.
struct ComponentColorMap {
  std::array<float[3], ComponentTable::kMaxBits> colors_{};
  uint64_t colored_mask_ = 0;
};

}  // namespace lumice

#endif  // CONFIG_COMPONENT_COLOR_MAP_H_
