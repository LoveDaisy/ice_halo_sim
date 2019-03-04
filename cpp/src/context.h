#ifndef SRC_CONTEXT_H_
#define SRC_CONTEXT_H_

#include "mymath.h"
#include "crystal.h"
#include "files.h"

#include "rapidjson/document.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <string>
#include <functional>
#include <memory>
#include <atomic>


namespace IceHalo {

class RaySegment;
struct CrystalContext;
enum class ProjectionType;
enum class VisibleSemiSphere;


struct AxisDistribution {
  AxisDistribution();

  Math::Distribution zenith_dist;
  Math::Distribution azimuth_dist;
  Math::Distribution roll_dist;
  float zenith_mean;
  float azimuth_mean;
  float roll_mean;
  float zenith_std;
  float azimuth_std;
  float roll_std;
};


struct RayPathFilter {
  enum Symmetry : uint8_t {
    kSymmetryNone = 0u,
    kSymmetryPrism = 1u,
    kSymmetryBasal = 2u,
    kSymmetryDirection = 4u,
  };

  enum Type : uint8_t {
    kTypeNone,
    kTypeSpecific,
    kTypeGeneral,
  };

  RayPathFilter();

  bool Filter(RaySegment* r, const CrystalPtr& crystal) const;
  size_t RayPathHash(const std::vector<uint16_t>& ray_path) const;
  void ApplyHash(const CrystalPtr& crystal);

  Type type;
  uint8_t symmetry;
  std::vector<std::vector<uint16_t> > ray_paths;
  std::unordered_set<size_t> ray_path_hashes;
  std::unordered_set<uint16_t> entry_faces;
  std::unordered_set<uint16_t> exit_faces;
  std::unordered_set<int> hit_nums;

private:
  bool FilterRayGeneral(RaySegment* r, const CrystalPtr& crystal) const;
  bool FilterRaySpecific(RaySegment* r, const CrystalPtr& crystal) const;
};


struct MultiScatterContext {
  std::vector<CrystalContext> crystals;
  std::vector<float> populations;
  std::vector<RayPathFilter> ray_path_filters;
  float prob;
};


class SimulationContext {
public:
  uint64_t GetTotalInitRays() const;
  int GetMaxRecursionNum() const;

  const std::vector<MultiScatterContext> GetMultiScatterContext() const ;
  void PrintCrystalInfo();

  void SetCurrentWavelength(float wavelength);
  float GetCurrentWavelength() const;
  std::vector<std::pair<float, float> > GetWavelengths() const;

  const float* GetSunRayDir() const;
  float GetSunDiameter() const;

  std::string GetDataDirectory() const;

  /*! @brief Read a config file and create a SimulationContext
   *
   * @param filename the config file
   * @return a pointer to SimulationContext
   */
  static std::unique_ptr<SimulationContext> CreateFromFile(const char* filename);

  static constexpr float kPropMinW = 1e-6;
  static constexpr float kScatMinW = 1e-3;

private:
  SimulationContext(const char* filename, rapidjson::Document& d);

  void ApplySettings();
  void SetSunRayDirection(float lon, float lat);

  void ParseBasicSettings(rapidjson::Document& d);
  void ParseRaySettings(rapidjson::Document& d);
  void ParseSunSettings(rapidjson::Document& d);
  void ParseMultiScatterSettings(rapidjson::Document& d);
  void ParseCrystalSettings(rapidjson::Document& d);
  void ParseRayPathFilterSettings(rapidjson::Document& d);

  void ParseOneCrystalSetting(const rapidjson::Value& c, int ci);
  AxisDistribution ParseCrystalAxis(const rapidjson::Value& c, int ci);
  // RayPathFilter ParseCrystalRayPathFilter(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalHexPrism(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalHexPyramid(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalHexPyramidStackHalf(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalCubicPyramid(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalIrregularHexPrism(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalIrregularHexPyramid(const rapidjson::Value& c, int ci);
  CrystalPtrU ParseCrystalCustom(const rapidjson::Value& c, int ci);

  void ParseOneScatterSetting(const rapidjson::Value& c, int ci);

  void ParseOneFilterSetting(const rapidjson::Value& c, int ci);
  void ParseFilterSymmetry(const rapidjson::Value& c, int ci, RayPathFilter* filter);
  void ParseFilterType(const rapidjson::Value& c, int ci, RayPathFilter* filter);
  void ParseFilterSpecificSettings(const rapidjson::Value& c, int ci, RayPathFilter* filter);
  void ParseFilterGeneralSettings(const rapidjson::Value& c, int ci, RayPathFilter* filter);

  using CrystalParser = std::function<CrystalPtrU(SimulationContext*, const rapidjson::Value& c, int ci)>;
  static std::unordered_map<std::string, CrystalParser> crystal_parser_;

  float sun_ray_dir_[3];
  float sun_diameter_;

  uint64_t total_ray_num_;
  std::vector<std::pair<float, float> > wavelengths_;
  float current_wavelength_;

  int max_recursion_num_;

  std::vector<MultiScatterContext> multi_scatter_ctx_;
  std::unordered_map<int, CrystalContext> crystal_ctx_;
  std::unordered_map<int, RayPathFilter> ray_path_filters_;

  std::string config_file_name_;
  std::string data_directory_;
};


struct CrystalContext {
  CrystalContext(CrystalPtrU&& g, const AxisDistribution& axis);
  CrystalContext(const CrystalContext& other);

  const CrystalPtr crystal;
  const AxisDistribution axis;
};


class RenderContext {
public:
  ~RenderContext() = default;

  uint32_t GetImageWidth() const;
  uint32_t GetImageHeight() const;
  std::string GetImagePath() const;

  std::string GetDataDirectory() const;

  const float* GetCamRot() const;
  float GetFov() const;
  ProjectionType GetProjectionType() const;
  VisibleSemiSphere GetVisibleSemiSphere() const;

  int GetOffsetX() const;
  int GetOffsetY() const;

  uint32_t GetTotalRayNum() const;

  const float* GetRayColor() const;
  const float* GetBackgroundColor() const;
  double GetIntensityFactor() const;

  static std::unique_ptr<RenderContext> CreateFromFile(const char* filename);

private:
  explicit RenderContext(rapidjson::Document& d);

  /* Parse rendering settings */
  void ParseCameraSettings(rapidjson::Document& d);
  void ParseRenderSettings(rapidjson::Document& d);
  void ParseDataSettings(rapidjson::Document& d);

  float cam_rot_[3];
  float fov_;

  float ray_color_[3];
  float background_color_[3];

  uint32_t img_hei_;
  uint32_t img_wid_;
  int offset_y_;
  int offset_x_;
  VisibleSemiSphere visible_semi_sphere_;
  ProjectionType projection_type_;

  uint32_t total_ray_num_;
  double intensity_factor_;

  bool show_horizontal_;

  std::string data_directory_;
};

using SimulationContextPtr = std::shared_ptr<SimulationContext>;
using RenderContextPtr = std::shared_ptr<RenderContext>;

}  // namespace IceHalo


#endif  // SRC_CONTEXT_H_
