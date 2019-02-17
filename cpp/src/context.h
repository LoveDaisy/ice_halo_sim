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
  uint64_t getTotalInitRays() const;
  int getMaxRecursionNum() const;

  int getMultiScatterNum() const;
  float getMultiScatterProb() const;

  void fillActiveCrystal(std::vector<CrystalContextPtr>* crystal_ctxs) const;
  void printCrystalInfo();

  void setCurrentWavelength(float wavelength);

  float getCurrentWavelength() const;
  std::vector<float> getWavelengths() const;

  const float* getSunRayDir() const;
  float getSunDiameter() const;

  std::string getDataDirectory() const;

  void applySettings();

  /*! @brief Read a config file and create a SimulationContext
   *
   * @param filename the config file
   * @return a pointer to SimulationContext
   */
  static std::unique_ptr<SimulationContext> createFromFile(const char* filename);

  static constexpr float kPropMinW = 1e-6;
  static constexpr float kScatMinW = 1e-3;

private:
  SimulationContext(const char* filename, rapidjson::Document& d);

  /* Parse simulation settings */
  void parseBasicSettings(rapidjson::Document& d);
  void parseRaySettings(rapidjson::Document& d);
  void parseSunSettings(rapidjson::Document& d);
  void parseDataSettings(rapidjson::Document& d);
  void parseMultiScatterSettings(rapidjson::Document& d);
  void parseCrystalSettings(const rapidjson::Value& c, int ci);
  void parseCrystalType(const rapidjson::Value& c, int ci,
                        float population,
                        Math::Distribution axisDist, float axisMean, float axisStd,
                        Math::Distribution rollDist, float rollMean, float rollStd);
  CrystalPtrU parseCustomCrystal(std::FILE* file);

  void setSunPosition(float lon, float lat);

  std::vector<CrystalContextPtr> crystalCtxs;

  uint64_t totalRayNum;
  int maxRecursionNum;
  int multiScatterNum;
  float multiScatterProb;
  float currentWavelength;
  std::vector<float> wavelengths;

  float sunDir[3];
  float sunDiameter;

  std::string configFileName;
  std::string dataDirectory;
};


class CrystalContext {
friend class SimulationContext;
public:
  CrystalContext(CrystalPtrU&& g, float population,
                 Math::Distribution axisDist, float axisMean, float axisStd,
                 Math::Distribution rollDist, float rollMean, float rollStd);

  CrystalPtr getCrystal();
  Math::Distribution getAxisDist();
  Math::Distribution getRollDist();
  float getAxisMean();
  float getRollMean();
  float getAxisStd();
  float getRollStd();
  float getPopulation();

private:
  CrystalPtr crystal;
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

  void loadData();

  uint32_t getImageWidth() const;
  uint32_t getImageHeight() const;
  std::string getImagePath() const;

  void renderToRgb(uint8_t* rgbData);

  static std::unique_ptr<RenderContext> createFromFile(const char* filename);

private:
  explicit RenderContext(rapidjson::Document& d);

  /* Parse rendering settings */
  void parseCameraSettings(rapidjson::Document& d);
  void parseRenderSettings(rapidjson::Document& d);
  void parseDataSettings(rapidjson::Document& d);

  int loadDataFromFile(File& file);
  void copySpectrumData(float* wavelengthData, float* spectrumData) const;

  float camRot[3];
  float fov;

  float rayColor[3];
  float backgroundColor[3];
  SpectrumRenderer render;

  uint32_t imgHei;
  uint32_t imgWid;
  int offsetY;
  int offsetX;
  VisibleSemiSphere visibleSemiSphere;

  std::unordered_map<int, float*> spectrumData;
  double totalW;
  double intensityFactor;

  bool showHorizontal;

  std::string dataDirectory;

  ProjectionType projectionType;
};

}  // namespace IceHalo


#endif  // SRC_CONTEXT_H_
