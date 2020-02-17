#ifndef SRC_CONTEXT_CONTEXT_H_
#define SRC_CONTEXT_CONTEXT_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "context/camera_context.h"
#include "context/crystal_context.h"
#include "context/filter_context.h"
#include "context/multi_scatter_context.h"
#include "context/render_context.h"
#include "context/sun_context.h"
#include "core/crystal.h"
#include "core/optics.h"
#include "io/serialize.h"
#include "rapidjson/document.h"


namespace icehalo {

constexpr int kInvalidId = std::numeric_limits<int>::lowest();

class ProjectContext;
using ProjectContextPtrU = std::unique_ptr<ProjectContext>;
using ProjectContextPtr = std::shared_ptr<ProjectContext>;


struct WavelengthInfo {
  int wavelength;
  float weight;
};


class ProjectContext {
 public:
  size_t GetInitRayNum() const;
  void SetInitRayNum(size_t ray_num);

  int GetRayHitNum() const;
  void SetRayHitNum(int hit_num);

  std::string GetDataDirectory() const;
  std::string GetDefaultImagePath() const;

  const Crystal* GetCrystal(int id) const;
  int32_t GetCrystalId(const Crystal* crystal) const;
  const CrystalContext* GetCrystalContext(int id) const;
  CrystalMap GetCrystalMap() const;

  AbstractRayPathFilter* GetRayPathFilter(int id) const;

  static ProjectContextPtrU CreateFromFile(const char* filename);
  static ProjectContextPtrU CreateDefault();

#ifdef FOR_TEST
  void PrintCrystalInfo() const;
#endif

  static constexpr float kPropMinW = 1e-6;
  static constexpr float kScatMinW = 1e-3;
  static constexpr size_t kMinInitRayNum = 10000;
  static constexpr size_t kDefaultInitRayNum = 500000;
  static constexpr int kMinRayHitNum = 1;
  static constexpr int kMaxRayHitNum = 12;
  static constexpr int kDefaultRayHitNum = 8;

  SunContextPtr sun_ctx_;
  CameraContextPtr cam_ctx_;
  RenderContextPtr render_ctx_;
  RenderContextPtr split_render_ctx_;
  std::vector<WavelengthInfo> wavelengths_;  // (wavelength, weight)
  std::vector<MultiScatterContextPtrU> multi_scatter_info_;

 private:
  ProjectContext();

  void ParseBasicSettings(rapidjson::Document& d);
  void ParseSunSettings(rapidjson::Document& d);
  void ParseRenderSettings(rapidjson::Document& d);
  void ParseCameraSettings(rapidjson::Document& d);
  void ParseCrystalSettings(rapidjson::Document& d);
  void ParseRayPathFilterSettings(rapidjson::Document& d);
  void ParseMultiScatterSettings(rapidjson::Document& d);

  size_t init_ray_num_;
  int ray_hit_num_;

  std::string data_path_;

  std::vector<CrystalContextPtrU> crystal_store_;
  std::vector<RayPathFilterContextPtrU> filter_store_;
};


std::vector<uint16_t> GetReverseRayPath(const Crystal* crystal, const RaySegment* last_ray);

}  // namespace icehalo


#endif  // SRC_CONTEXT_CONTEXT_H_
