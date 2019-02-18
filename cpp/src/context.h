#ifndef SRC_CONTEXT_H_
#define SRC_CONTEXT_H_

#include "mymath.h"
#include "crystal.h"
#include "render.h"
#include "files.h"

#include "rapidjson/document.h"

#include <vector>
#include <unordered_map>
#include <random>
#include <string>
#include <functional>
#include <memory>
#include <atomic>


namespace IceHalo {

class RaySegment;
class Ray;

class RenderContext;
class RayTracingContext;
class CrystalContext;
class SimulationContext;

using RayTracingContextPtr = std::shared_ptr<RayTracingContext>;
using CrystalContextPtr = std::shared_ptr<CrystalContext>;
using SimulationContextPtr = std::shared_ptr<SimulationContext>;
using RenderContextPtr = std::shared_ptr<RenderContext>;


class SimulationContext {
public:
  uint64_t GetTotalInitRays() const;
  int GetMaxRecursionNum() const;

  int GetMultiScatterTimes() const;
  float GetMultiScatterProb() const;

  void FillActiveCrystal(std::vector<CrystalContextPtr>* crystal_ctxs) const;
  void PrintCrystalInfo();

  void SetCurrentWavelength(float wavelength);
  float GetCurrentWavelength() const;
  std::vector<float> GetWavelengths() const;

  const float* GetSunRayDir() const;
  float GetSunDiameter() const;

  std::string GetDataDirectory() const;

  void ApplySettings();

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

  /* Parse simulation settings */
  void ParseBasicSettings(rapidjson::Document& d);
  void ParseRaySettings(rapidjson::Document& d);
  void ParseSunSettings(rapidjson::Document& d);
  void ParseDataSettings(rapidjson::Document& d);
  void ParseMultiScatterSettings(rapidjson::Document& d);
  void ParseCrystalSettings(const rapidjson::Value& c, int ci);
  void ParseCrystalType(const rapidjson::Value& c, int ci,
                        float population,
                        Math::Distribution axis_dist, float axis_mean, float axis_std,
                        Math::Distribution roll_dist, float roll_mean, float roll_std);
  CrystalPtrU ParseCustomCrystal(std::FILE* file);

  void SetSunPosition(float lon, float lat);

  std::vector<CrystalContextPtr> crystal_ctxs_;

  uint64_t total_ray_num_;
  int max_recursion_num_;

  int multi_scatter_times_;
  float multi_scatter_prob_;

  float current_wavelength_;
  std::vector<float> wavelengths_;

  float sun_ray_dir_[3];
  float sun_diameter_;

  std::string config_file_name_;
  std::string data_directory_;
};


class CrystalContext {
friend class SimulationContext;
public:
  CrystalContext(CrystalPtrU&& g, float population,
                 Math::Distribution axisDist, float axisMean, float axisStd,
                 Math::Distribution rollDist, float rollMean, float rollStd);

  CrystalPtr GetCrystal();
  Math::Distribution GetAxisDist();
  Math::Distribution GetRollDist();
  float GetAxisMean();
  float GetRollMean();
  float GetAxisStd();
  float GetRollStd();
  float GetPopulation();

private:
  CrystalPtr crystal_;
  Math::Distribution axis_dist_;
  Math::Distribution roll_dist_;
  float axis_mean_;
  float roll_mean_;
  float axis_std_;
  float roll_std_;
  float population_;
};


class RenderContext {
public:
  ~RenderContext();

  void LoadData();

  uint32_t GetImageWidth() const;
  uint32_t GetImageHeight() const;
  std::string GetImagePath() const;

  void RenderToRgb(uint8_t* rgbData);

  static std::unique_ptr<RenderContext> CreateFromFile(const char* filename);

private:
  explicit RenderContext(rapidjson::Document& d);

  /* Parse rendering settings */
  void ParseCameraSettings(rapidjson::Document& d);
  void ParseRenderSettings(rapidjson::Document& d);
  void ParseDataSettings(rapidjson::Document& d);

  int LoadDataFromFile(File& file);
  void CopySpectrumData(float* wavelengthData, float* spectrumData) const;

  float cam_rot_[3];
  float fov_;

  float ray_color_[3];
  float background_color_[3];
  SpectrumRenderer render;

  uint32_t img_hei_;
  uint32_t img_wid_;
  int offset_y_;
  int offset_x_;
  VisibleSemiSphere visible_semi_sphere_;

  std::unordered_map<int, float*> spectrum_data_;
  double total_w_;
  double intensity_factor_;

  bool show_horizontal_;

  std::string data_directory_;

  ProjectionType projection_type_;
};

}  // namespace IceHalo


#endif  // SRC_CONTEXT_H_
