#ifndef SRC_CORE_DEF_H_
#define SRC_CORE_DEF_H_

#include <cstddef>
#include <cstdint>
#include <limits>

namespace lumice {

using ShortIdType = uint16_t;
constexpr ShortIdType kInvalidId = 0xffff;
constexpr size_t kInfSize = std::numeric_limits<size_t>::max();

enum Symmetry : uint8_t {
  kSymmetryNone = 0u,
  kSymmetryPrism = 1u,
  kSymmetryBasal = 2u,
  kSymmetryDirection = 4u,
  kSymmetryRepeatedReflection = 8u,
};


constexpr size_t kMaxMsNum = 4;  // How many multi-satterings at most.
constexpr size_t kMaxHits = 64;  // How many hits in one crystal.
// GPU K-shape pool geometry clock safe upper bound. Numerically equals kMaxHits
// by coincidence but is semantically independent (one caps per-crystal hits, the
// other caps rays-per-sampled-shape on the GPU pool). A future auto-K feature
// adjusting this bound changes only here, never kMaxHits.
constexpr size_t kGeomClockMax = 64;
constexpr size_t kMaxWlNum = 32;       // How many different wavelengths in one configuration.
constexpr size_t kMaxCrystalNum = 16;  // How many crystal types in one configuration.

// task-color-degrade-gui-surfacing: per-committed-config tally of raypath-color
// assignments the GPU backends silently dropped because a device-side capacity
// was exceeded. All three caps are GPU-only (device buffer-layout constants);
// the CPU backend has no equivalent limit and always reports zeros. These are
// config CONSTANTS (fixed for a given committed config), NOT per-batch
// accumulators — the transport path (backend -> simulator -> server -> C API)
// must OVERWRITE, never `+=`. Lives here in def.hpp so both the core backend
// seam (TraceBackend) and the config-layer SimData can carry it without a
// layering violation. Surfaced to the GUI so any color degradation produces a
// modal warning instead of a silent log line.
struct ColorDegradeCounts {
  size_t symmetry_group_overflow = 0;  // kColorMaxGroupsPerSlot (per gate slot)
  size_t or_summand_overflow = 0;      // kDeviceFilterMaxOrClauses (per color group)
  size_t color_class_overflow = 0;     // kMaxColorClassesDevice (per session)

  bool AnyOverflow() const {
    return symmetry_group_overflow != 0 || or_summand_overflow != 0 || color_class_overflow != 0;
  }
};

using IdType = uint16_t;


}  // namespace lumice

#endif  // SRC_CORE_DEF_H_
