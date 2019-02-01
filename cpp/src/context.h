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
  int getCrystalNum() const;
  void setCurrentWavelength(float wavelength);
  float getCurrentWavelength() const;
  std::vector<float> getWavelengths() const;

  void fillSunDir(float* dir, uint64_t num = 1);
  void setSunPosition(float lon, float lat);
  void applySettings();
  void setCrystalRayNum(int scatterIdx, uint64_t totalRayNum);

  CrystalContextPtr getCrystalContext(int i);
  RayTracingContextPtr getRayTracingContext(int scatterIndx, int crystalIdx);

  /* For output */
  void writeFinalDirections(const char* filename);
  void printCrystalInfo();

  static std::unique_ptr<SimulationContext> createFromFile(const char* filename);

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
  CrystalPtr parseCustomCrystal(std::FILE* file);

  std::vector<CrystalContextPtr> crystalCtxs;
  std::vector<std::vector<RayTracingContextPtr> > rayTracingCtxs;

  uint64_t totalRayNum;
  int maxRecursionNum;
  int multiScatterNum;
  float multiScatterProb;
  float currentWavelength;
  std::vector<float> wavelengths;

  float sunDir[3];
  float sunDiameter;

  std::mt19937 generator;
  std::uniform_real_distribution<float> uniformDistribution;

  std::string configFileName;
  std::string dataDirectory;
};


class CrystalContext {
friend class SimulationContext;
public:
  CrystalContext() = default;

  void setCrystal(const CrystalPtr& g, float populationRatio,
                  Math::Distribution axisDist, float axisMean, float axisStd,
                  Math::Distribution rollDist, float rollMean, float rollStd);
  CrystalPtr getCrystal();

  void fillDir(const float* incDir, float* rayDir, float* mainAxRot, int num = 1);

private:
  float populationRatio;
  CrystalPtr crystal;
  Math::OrientationGenerator oriGen;
};


class RayTracingContext {
friend class SimulationContext;
friend class Optics;
public:
  explicit RayTracingContext(int maxRecursion);
  ~RayTracingContext();

  void setRayNum(int rayNum);

  void initRays(const CrystalContextPtr& ctx, int rayNum, const float* dir, const float* w,
                RaySegment** prevRaySeg = nullptr);
  void commitHitResult();
  void commitPropagateResult(const CrystalContextPtr& ctx);
  bool isFinished();

  size_t copyFinishedRaySegments(RaySegment** segs, float* dir, float prob = 1.0f);

private:
  static constexpr float kPropMinW = 1e-6;
  static constexpr float kScatMinW = 1e-3;

  void deleteArrays();
  int chooseFace(const float* faces, int faceNum, const float* rayDir);

  void fillPts(const float* faces, int idx, float* rayPts);

  void copyFinishedRaySegmentsRange(RaySegment** segs, float* dir, float prob,
                                    std::atomic<std::uint64_t>& k, int startIdx, int endIdx);

  int maxRecursion;
  int initRayNum;
  int currentRayNum;
  int activeRaySegNum;
  Ray** rays;
  RaySegment** activeRaySeg;

  std::default_random_engine gen;
  std::uniform_real_distribution<float> dis;

  float* mainAxRot;

  float* rayDir;
  float* rayPts;
  float* faceNorm;
  int* faceId;

  float* rayDir2;
  float* rayDir3;
  float* rayPts2;
  float* rayW2;
  int* faceId2;
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
