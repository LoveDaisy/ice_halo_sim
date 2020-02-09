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
#include <utility>
#include <vector>

#include "crystal.h"
#include "file.h"
#include "mymath.h"
#include "optics.h"
#include "rapidjson/document.h"


namespace icehalo {

struct CrystalContext;
class CameraContext;
class ProjectContext;
class RenderContext;
class SunContext;
enum class LensType;
enum class VisibleRange;

using CrystalContextPtrU = std::unique_ptr<CrystalContext>;
using CameraContextPtrU = std::unique_ptr<CameraContext>;
using CameraContextPtr = std::shared_ptr<CameraContext>;
using ProjectContextPtrU = std::unique_ptr<ProjectContext>;
using ProjectContextPtr = std::shared_ptr<ProjectContext>;
using RenderContextPtrU = std::unique_ptr<RenderContext>;
using RenderContextPtr = std::shared_ptr<RenderContext>;
using SunContextPtrU = std::unique_ptr<SunContext>;
using SunContextPtr = std::shared_ptr<SunContext>;


enum Symmetry : uint8_t {
  kSymmetryNone = 0u,
  kSymmetryPrism = 1u,
  kSymmetryBasal = 2u,
  kSymmetryDirection = 4u,
};


size_t RayPathHash(const std::vector<uint16_t>& ray_path, bool reverse = false);
size_t RayPathHash(const Crystal* crystal, const RaySegment* last_ray, int length, bool reverse = false);


class AbstractRayPathFilter {
 public:
  AbstractRayPathFilter();
  virtual ~AbstractRayPathFilter() = default;

  bool Filter(const Crystal* crystal, RaySegment* last_r) const;

  void SetSymmetryFlag(uint8_t symmetry_flag);
  void AddSymmetry(Symmetry symmetry);
  uint8_t GetSymmetryFlag() const;
  virtual void ApplySymmetry(const Crystal* crystal);

  void EnableComplementary(bool enable);
  bool GetComplementary() const;

  void EnableRemoveHomodromous(bool enable);
  bool GetRemoveHomodromous() const;

 protected:
  virtual bool FilterPath(const Crystal* crystal, RaySegment* last_r) const = 0;

  uint8_t symmetry_flag_;
  bool complementary_;
  bool remove_homodromous_;
};

using RayPathFilterPtrU = std::unique_ptr<AbstractRayPathFilter>;


class NoneRayPathFilter : public AbstractRayPathFilter {
 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;
};


class SpecificRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddPath(const std::vector<uint16_t>& path);
  void ClearPaths();

  void ApplySymmetry(const Crystal* crystal) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

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
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

 private:
  std::unordered_set<uint16_t> entry_faces_;
  std::unordered_set<uint16_t> exit_faces_;
  std::unordered_set<int> hit_nums_;
};


class MultiScatterContext {
 public:
  struct CrystalInfo {
    const CrystalContext* crystal_ctx;
    AbstractRayPathFilter* filter;
    float population;

    CrystalInfo(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter, float pop)
        : crystal_ctx(crystal_ctx), filter(filter), population(pop){};
  };

  explicit MultiScatterContext(float prob = 1.0f);

  float GetProbability() const;
  bool SetProbability(float p);

  const std::vector<CrystalInfo>& GetCrystalInfo() const;
  void ClearCrystalInfo();
  void AddCrystalInfo(const CrystalContext* crystal_ctx, AbstractRayPathFilter* filter, float population);
  void NormalizeCrystalPopulation();

 private:
  std::vector<CrystalInfo> crystal_infos_;  // crystal, population, filter
  float prob_;
};


class SunContext {
 public:
  const float* GetSunPosition() const;

  float GetSunAltitude() const;
  bool SetSunAltitude(float altitude);

  float GetSunDiameter() const;
  bool SetSunDiameter(float d);

  static SunContextPtrU CreateFromJson(rapidjson::Document& d);

  static constexpr float kMaxDiameter = 90.0f;
  static constexpr float kDefaultAltitude = 20.0f;

 private:
  SunContext();
  SunContext(float altitude, float diameter);

  float sun_diameter_;     // in degree
  float sun_altitude_;     // in degree
  float sun_position_[3];  // [x, y, z]
};


class CameraContext {
 public:
  const float* GetCameraTargetDirection() const;
  void SetCameraTargetDirection(float azimuth, float altitude, float roll);
  void ResetCameraTargetDirection();

  float GetFov() const;
  void SetFov(float fov);

  LensType GetLensType() const;
  void SetLensType(LensType type);

  static CameraContextPtrU CreateFromJson(rapidjson::Document& d);

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
  CameraContext();

  float target_dir_[3];  // azimuth, altitude, roll
  float fov_;
  LensType lens_type_;
};


class RenderContext {
 public:
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

  static RenderContextPtrU CreateFromJson(rapidjson::Document& d);

  static constexpr float kMinIntensity = 0.01f;
  static constexpr float kMaxIntensity = 100.0f;

  static constexpr int kMaxImageSize = 4096;

 private:
  RenderContext();

  float ray_color_[3];
  float background_color_[3];
  float intensity_;
  int image_width_;
  int image_height_;
  int offset_x_;
  int offset_y_;
  VisibleRange visible_range_;
};


struct WavelengthInfo {
  int wavelength;
  float weight;
};


class ProjectContext {
 public:
  static ProjectContextPtrU CreateFromFile(const char* filename);
  static ProjectContextPtrU CreateDefault();

  size_t GetInitRayNum() const;
  void SetInitRayNum(size_t ray_num);

  int GetRayHitNum() const;
  void SetRayHitNum(int hit_num);

  std::string GetModelPath() const;
  void SetModelPath(const std::string& path);

  std::string GetDataDirectory() const;
  std::string GetDefaultImagePath() const;

  const Crystal* GetCrystal(int id) const;
  int32_t GetCrystalId(const Crystal* crystal) const;

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
  std::vector<WavelengthInfo> wavelengths_;  // (wavelength, weight)
  std::vector<MultiScatterContext> multi_scatter_info_;

 private:
  ProjectContext();

  void ParseRaySettings(rapidjson::Document& d);
  void ParseDataSettings(const char* config_file_path, rapidjson::Document& d);
  void ParseCrystalSettings(rapidjson::Document& d);
  void ParseRayPathFilterSettings(rapidjson::Document& d);
  void ParseMultiScatterSettings(rapidjson::Document& d);

  using CrystalParser = std::function<CrystalPtrU(const rapidjson::Value&, int)>;
  static std::unordered_map<std::string, CrystalParser>& GetCrystalParsers(const std::string& model_path);
  void ParseOneCrystal(const rapidjson::Value& c, int ci);
  static AxisDistribution ParseCrystalAxis(const rapidjson::Value& c, int ci);
  static CrystalPtrU ParseCrystalHexPrism(const rapidjson::Value& c, int ci);
  static CrystalPtrU ParseCrystalHexPyramid(const rapidjson::Value& c, int ci);
  static CrystalPtrU ParseCrystalHexPyramidStackHalf(const rapidjson::Value& c, int ci);
  static CrystalPtrU ParseCrystalCubicPyramid(const rapidjson::Value& c, int ci);
  static CrystalPtrU ParseCrystalIrregularHexPrism(const rapidjson::Value& c, int ci);
  static CrystalPtrU ParseCrystalIrregularHexPyramid(const rapidjson::Value& c, int ci);
  static CrystalPtrU ParseCrystalCustom(const rapidjson::Value& c, int ci, const std::string& model_path);
  const CrystalContext* GetCrystalContext(int id) const;

  using FilterParser = std::function<RayPathFilterPtrU(const rapidjson::Value&, int)>;
  static std::unordered_map<std::string, FilterParser>& GetFilterParsers();
  void ParseOneFilter(const rapidjson::Value& c, int ci);
  static void ParseFilterBasic(const rapidjson::Value& c, int ci, const RayPathFilterPtrU& filter);
  static RayPathFilterPtrU ParseFilterNone(const rapidjson::Value& c, int ci);
  static RayPathFilterPtrU ParseFilterSpecific(const rapidjson::Value& c, int ci);
  static RayPathFilterPtrU ParseFilterGeneral(const rapidjson::Value& c, int ci);
  AbstractRayPathFilter* GetRayPathFilter(int id) const;

  void ParseOneScatter(const rapidjson::Value& c, int ci);

  size_t init_ray_num_;
  int ray_hit_num_;

  std::string model_path_;
  std::string data_path_;

  std::unordered_map<int, CrystalContextPtrU> crystal_store_;
  std::unordered_map<int, RayPathFilterPtrU> filter_store_;
};


struct CrystalContext {
  CrystalContext(CrystalPtrU g, AxisDistribution axis);
  CrystalContext(const CrystalContext& other) = delete;

  int RandomSampleFace(const float* ray_dir) const;

  const CrystalPtrU crystal;
  const AxisDistribution axis;
};


}  // namespace icehalo


#endif  // SRC_CONTEXT_H_
