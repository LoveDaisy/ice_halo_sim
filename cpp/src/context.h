#include <utility>

#ifndef SRC_CONTEXT_H_
#define SRC_CONTEXT_H_

#include <atomic>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "crystal.h"
#include "files.h"
#include "mymath.h"
#include "optics.h"
#include "rapidjson/document.h"


namespace IceHalo {

struct RaySegment;
struct CrystalContext;
class ProjectContext;
enum class LensType;
enum class VisibleRange;

using CrystalContextPtr = std::shared_ptr<CrystalContext>;


enum Symmetry : uint8_t {
  kSymmetryNone = 0u,
  kSymmetryPrism = 1u,
  kSymmetryBasal = 2u,
  kSymmetryDirection = 4u,
};


class AbstractRayPathFilter {
 public:
  AbstractRayPathFilter();
  virtual ~AbstractRayPathFilter() = default;

  bool Filter(const CrystalPtr& crystal, RaySegment* last_r) const;

  void SetSymmetryFlag(uint8_t symmetry_flag);
  void AddSymmetry(Symmetry symmetry);
  uint8_t GetSymmetryFlag() const;
  virtual void ApplySymmetry(const CrystalPtr& crystal);

  void EnableComplementary(bool enable);
  bool GetComplementary() const;

  void EnableRemoveHomodromous(bool enable);
  bool GetRemoveHomodromous() const;

 protected:
  size_t RayPathHash(const std::vector<uint16_t>& ray_path, bool reverse = false) const;
  size_t RayPathHash(const CrystalPtr& crystal, const RaySegment* last_ray, int length, bool reverse = false) const;
  virtual bool FilterPath(const CrystalPtr& crystal, RaySegment* last_r) const = 0;

  uint8_t symmetry_flag_;
  bool complementary_;
  bool remove_homodromous_;
};

using RayPathFilterPtr = std::shared_ptr<AbstractRayPathFilter>;


class NoneRayPathFilter : public AbstractRayPathFilter {
 protected:
  bool FilterPath(const CrystalPtr& crystal, RaySegment* last_r) const override;
};


class SpecificRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddPath(const std::vector<uint16_t>& path);
  void ClearPaths();

  void ApplySymmetry(const CrystalPtr& crystal) override;

 protected:
  bool FilterPath(const CrystalPtr& crystal, RaySegment* last_r) const override;

 private:
  std::unordered_set<size_t> ray_path_hashes_;
  std::vector<std::vector<uint16_t>> ray_paths_;
};


class GeneralRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddEntryFace(uint16_t face_number);
  void AddExitFace(uint16_t face_number);
  void AddHitNumber(int hit_num);
  void ClearFaces();
  void ClearHitNumbers();

 protected:
  bool FilterPath(const CrystalPtr& crystal, RaySegment* last_r) const override;

 private:
  std::unordered_set<uint16_t> entry_faces_;
  std::unordered_set<uint16_t> exit_faces_;
  std::unordered_set<int> hit_nums_;
};


class MultiScatterContext {
 public:
  struct CrystalInfo {
    int crystal_id;
    float population;
    int filter_id;

    CrystalInfo(int crystal_id, float pop, int filter_id)
        : crystal_id(crystal_id), population(pop), filter_id(filter_id){};
  };

  explicit MultiScatterContext(float prob = 1.0f);

  float GetProbability() const;
  bool SetProbability(float p);

  const std::vector<CrystalInfo>& GetCrystalInfo() const;
  void ClearCrystalInfo();
  void AddCrystalInfo(int crystal_id, float population, int filter_id);
  void NormalizeCrystalPopulation();

 private:
  std::vector<CrystalInfo> crystal_infos_;  // crystal, population, filter
  float prob_;
};


class SunContext {
 public:
  explicit SunContext(float altitude, float diameter = 0.0f);

  const float* GetSunPosition() const;

  float GetSunAltitude() const;
  bool SetSunAltitude(float altitude);

  float GetSunDiameter() const;
  bool SetSunDiameter(float d);

  static constexpr float kMaxDiameter = 90.0f;
  static constexpr float kDefaultAltitude = 20.0f;

 private:
  float sun_diameter_;     // in degree
  float sun_altitude_;     // in degree
  float sun_position_[3];  // [x, y, z]
};


class CameraContext {
 public:
  CameraContext();

  const float* GetCameraTargetDirection() const;
  void SetCameraTargetDirection(float azimuth, float altitude, float roll);
  void ResetCameraTargetDirection();

  float GetFov() const;
  void SetFov(float fov);

  LensType GetLensType() const;
  void SetLensType(LensType type);

  static constexpr float kMinAngleRound = 0.0f;
  static constexpr float kMaxAngleRound = 360.0f;
  static constexpr float kMinAngleTilt = -90.0f;
  static constexpr float kMaxAngleTilt = 90.0f;
  static constexpr float kMinAngleHeading = -180.0f;
  static constexpr float kMaxAngleHeading = 180.0f;

  static constexpr float kMaxFovLinear = 65.0f;
  static constexpr float kMaxFovFisheye = 120.0f;

  static constexpr float kDefaultCamAzimuth = 90.0f;
  static constexpr float kDefaultCamElevation = 89.9f;
  static constexpr float kDefaultCamRoll = 0.0f;

 private:
  float target_dir_[3];  // azimuth, altitude, roll
  float fov_;
  LensType lens_type_;
};


class RenderContext {
 public:
  RenderContext();

  const float* GetRayColor() const;
  void SetRayColor(float r, float g, float b);
  void ResetRayColor();
  void UseRealRayColor();

  const float* GetBackgroundColor() const;
  void SetBackgroundColor(float r, float g, float b);
  void ResetBackgroundColor();
  void UseSkyBackground();

  float GetIntensity() const;
  void SetIntensity(float intensity);

  int GetImageWidth() const;
  int GetImageHeight() const;
  void SetImageWidth(int w);
  void SetImageHeight(int h);

  int GetImageOffsetX() const;
  int GetImageOffsetY() const;
  void SetImageOffsetX(int offset_x);
  void SetImageOffsetY(int offset_y);

  VisibleRange GetVisibleRange() const;
  void SetVisibleRange(VisibleRange r);

  static constexpr float kMinIntensity = 0.01f;
  static constexpr float kMaxIntensity = 100.0f;

  static constexpr int kMaxImageSize = 4096;

 private:
  float ray_color_[3];
  float background_color_[3];
  float intensity_;
  int image_width_;
  int image_height_;
  int offset_x_;
  int offset_y_;
  VisibleRange visible_range_;
};


class ProjectContext {
 public:
  struct WavelengthInfo {
    int wavelength;
    float weight;
  };

  static std::unique_ptr<ProjectContext> CreateFromFile(const char* filename);
  static std::unique_ptr<ProjectContext> CreateDefault();

  size_t GetInitRayNum() const;
  void SetInitRayNum(size_t ray_num);

  int GetRayHitNum() const;
  void SetRayHitNum(int hit_num);

  std::string GetModelPath() const;
  void SetModelPath(const std::string& path);

  std::string GetDataDirectory() const;
  std::string GetDefaultImagePath() const;

  void ClearCrystals();
  void SetCrystal(int id, const CrystalPtr& crystal, const AxisDistribution& axis);
  const CrystalContextPtr GetCrystalContext(int id) const;
  const CrystalPtr GetCrystal(int id) const;
  void PrintCrystalInfo() const;

  void ClearRayPathFilter();
  void SetRayPathFilter(int id, const RayPathFilterPtr& filter);
  const RayPathFilterPtr GetRayPathFilter(int id) const;

  static constexpr float kPropMinW = 1e-6;
  static constexpr float kScatMinW = 1e-3;
  static constexpr size_t kMinInitRayNum = 10000;
  static constexpr size_t kDefaultInitRayNum = 500000;
  static constexpr int kMinRayHitNum = 1;
  static constexpr int kMaxRayHitNum = 12;
  static constexpr int kDefaultRayHitNum = 8;

  SunContext sun_ctx_;
  CameraContext cam_ctx_;
  RenderContext render_ctx_;
  std::vector<WavelengthInfo> wavelengths_;  // (wavelength, weight)
  std::vector<MultiScatterContext> multi_scatter_info_;

 private:
  ProjectContext();

  void ParseSunSettings(rapidjson::Document& d);
  void ParseRaySettings(rapidjson::Document& d);
  void ParseCameraSettings(rapidjson::Document& d);
  void ParseRenderSettings(rapidjson::Document& d);
  void ParseDataSettings(const char* config_file_path, rapidjson::Document& d);
  void ParseCrystalSettings(rapidjson::Document& d);
  void ParseRayPathFilterSettings(rapidjson::Document& d);
  void ParseMultiScatterSettings(rapidjson::Document& d);

  using CrystalParser = std::function<CrystalPtrU(ProjectContext*, const rapidjson::Value&, int)>;
  static std::unordered_map<std::string, CrystalParser>& GetCrystalParsers();
  void ParseOneCrystal(const rapidjson::Value& c, int ci);
  AxisDistribution ParseCrystalAxis(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalHexPrism(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalHexPyramid(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalHexPyramidStackHalf(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalCubicPyramid(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalIrregularHexPrism(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalIrregularHexPyramid(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalCustom(const rapidjson::Value& c, int ci);

  using FilterParser = std::function<RayPathFilterPtr(ProjectContext*, const rapidjson::Value&, int)>;
  static std::unordered_map<std::string, FilterParser>& GetFilterParsers();
  void ParseOneFilter(const rapidjson::Value& c, int ci);
  void ParseFilterBasic(const rapidjson::Value& c, int ci, RayPathFilterPtr filter);
  RayPathFilterPtr ParseFilterNone(const rapidjson::Value& c, int ci);
  RayPathFilterPtr ParseFilterSpecific(const rapidjson::Value& c, int ci);
  RayPathFilterPtr ParseFilterGeneral(const rapidjson::Value& c, int ci);

  void ParseOneScatter(const rapidjson::Value& c, int ci);

  size_t init_ray_num_;
  int ray_hit_num_;

  std::string model_path_;
  std::string data_path_;

  std::unordered_map<int, CrystalContextPtr> crystal_store_;
  std::unordered_map<int, RayPathFilterPtr> filter_store_;
};


struct CrystalContext {
  CrystalContext(CrystalPtrU&& g, const AxisDistribution& axis);
  CrystalContext(const CrystalContext& other);

  const CrystalPtr crystal;
  const AxisDistribution axis;
};


struct RayInfo {
  RayInfo(RaySegment* seg, CrystalContextPtr crystal_ctx, const float* main_axis_rot);

  RaySegment* first_ray_segment;
  RaySegment* prev_ray_segment;
  CrystalContextPtr crystal_ctx;
  Math::Vec3f main_axis_rot;
};

using RayContextPtr = std::shared_ptr<RayInfo>;
using ProjectContextPtr = std::shared_ptr<ProjectContext>;

}  // namespace IceHalo


#endif  // SRC_CONTEXT_H_
