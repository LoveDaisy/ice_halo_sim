#ifndef SRC_CONTEXT_CONTEXT_H_
#define SRC_CONTEXT_CONTEXT_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "context/camera_context.hpp"
#include "context/crystal_context.hpp"
#include "context/filter_context.hpp"
#include "context/multi_scatter_context.hpp"
#include "context/render_context.hpp"
#include "context/sun_context.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/optics.hpp"
#include "io/json_util.hpp"
#include "io/serialize.hpp"

namespace icehalo {


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
  std::string GetMainImagePath() const;

  const Crystal* GetCrystal(ShortIdType id) const;
  ShortIdType GetCrystalId(const Crystal* crystal) const;
  const CrystalContext* GetCrystalContext(ShortIdType id) const;
  CrystalMap GetCrystalMap() const;

  AbstractRayPathFilter* GetRayPathFilter(ShortIdType id) const;
  RayPath GetRayPath(const RaySegment* last_ray);

  static ProjectContextPtrU CreateFromFile(const char* filename);
  static ProjectContextPtrU CreateDefault();

  void PrintCrystalInfo() const;

  static constexpr float kPropMinW = 1e-6;
  static constexpr float kScatMinW = 1e-3;
  static constexpr size_t kMinInitRayNum = 1000;
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

  void ParseBasicSettings(const nlohmann::json& obj);
  void ParseSunSettings(const nlohmann::json& obj);
  void ParseRenderSettings(const nlohmann::json& obj);
  void ParseCameraSettings(const nlohmann::json& obj);
  void ParseCrystalSettings(const nlohmann::json& obj);
  void ParseRayPathFilterSettings(const nlohmann::json& obj);
  void ParseMultiScatterSettings(const nlohmann::json& obj);

  size_t init_ray_num_;
  int ray_hit_num_;

  std::string data_path_;
  std::string main_img_filename_;

  std::vector<CrystalContextPtrU> crystal_store_;
  std::vector<RayPathFilterContextPtrU> filter_store_;
};


}  // namespace icehalo


#endif  // SRC_CONTEXT_CONTEXT_H_
