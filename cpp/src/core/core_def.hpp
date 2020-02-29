#ifndef SRC_CORE_DEF_H_
#define SRC_CORE_DEF_H_

#include <unordered_map>

namespace icehalo {

using ShortIdType = uint16_t;
using FaceNumberType = ShortIdType;
constexpr ShortIdType kInvalidId = 0xffff;
constexpr FaceNumberType kInvalidFaceNumber = kInvalidId;

struct RayInfo;
struct RaySegment;

using RayPath = std::vector<FaceNumberType>;
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

}  // namespace icehalo

#endif  // SRC_CORE_DEF_H_
