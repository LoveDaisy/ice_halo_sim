#ifndef SRC_CORE_DEF_H_
#define SRC_CORE_DEF_H_

#include <cstddef>
#include <limits>

namespace icehalo {

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


constexpr size_t kMaxMsNum = 4;        // How many multi-satterings at most.
constexpr size_t kMaxHits = 8;         // How many hits in one crystal.
constexpr size_t kMaxWlNum = 32;       // How many different wavelengths in one configuration.
constexpr size_t kMaxCrystalNum = 16;  // How many crystal types in one configuration.

using IdType = uint16_t;


}  // namespace icehalo

#endif  // SRC_CORE_DEF_H_
