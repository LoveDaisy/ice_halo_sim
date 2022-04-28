#ifndef SRC_CORE_DEF_H_
#define SRC_CORE_DEF_H_

#include <cstddef>
#include <limits>
#include <unordered_map>

namespace icehalo {

using ShortIdType = uint16_t;
constexpr ShortIdType kInvalidId = 0xffff;
constexpr size_t kInfSize = std::numeric_limits<size_t>::max();

struct RayInfo;
struct RaySegment;

struct RayPath;
using RayPathMap = std::unordered_map<size_t, std::pair<RayPath, size_t>>;
constexpr int kAutoDetectLength = -1;

class RenderContext;
using RenderContextPtrU = std::unique_ptr<RenderContext>;
using RenderContextPtr = std::shared_ptr<RenderContext>;

class CameraContext;
using CameraContextPtrU = std::unique_ptr<CameraContext>;
using CameraContextPtr = std::shared_ptr<CameraContext>;

class CrystalContext;
using CrystalContextPtrU = std::unique_ptr<CrystalContext>;

class MultiScatterContext;
using MultiScatterContextPtrU = std::unique_ptr<MultiScatterContext>;

class ProjectContext;
using ProjectContextPtrU = std::unique_ptr<ProjectContext>;
using ProjectContextPtr = std::shared_ptr<ProjectContext>;

class SunContext;
using SunContextPtrU = std::unique_ptr<SunContext>;
using SunContextPtr = std::shared_ptr<SunContext>;

class Crystal;
using CrystalPtrU = std::unique_ptr<Crystal>;

using CrystalMap = std::unordered_map<ShortIdType, const Crystal*>;

enum Symmetry : uint8_t {
  kSymmetryNone = 0u,
  kSymmetryPrism = 1u,
  kSymmetryBasal = 2u,
  kSymmetryDirection = 4u,
  kSymmetryRepeatedReflection = 8u,
};


namespace v3 {

constexpr size_t kMaxMsNum = 4;        // How many multi-satterings at most.
constexpr size_t kMaxHits = 8;         // How many hits in one crystal.
constexpr size_t kMaxWlNum = 32;       // How many different wavelengths in one configuration.
constexpr size_t kMaxCrystalNum = 16;  // How many crystal types in one configuration.

using IdType = uint16_t;

}  // namespace v3

}  // namespace icehalo

#endif  // SRC_CORE_DEF_H_
