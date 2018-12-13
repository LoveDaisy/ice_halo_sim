#ifndef SRC_CONTEXT_H_
#define SRC_CONTEXT_H_

#include "mymath.h"
#include "optics.h"
#include "crystal.h"
#include "render.h"
#include "files.h"

#include "rapidjson/document.h"

#include <vector>
#include <unordered_map>
#include <random>
#include <string>
#include <functional>


namespace IceHalo {

class RenderContext;

class ContextParser {
public:
  ~ContextParser() = default;

  void parseSimulationSettings(SimulationContext& ctx);
  void parseRenderingSettings(RenderContext& ctx);

  static ContextParser* createFileParser(const char* filename);

private:
  explicit ContextParser(rapidjson::Document& d, const char* filename);

  /* Parse simulation settings */
  void parseBasicSettings(SimulationContext& ctx);
  void parseRaySettings(SimulationContext& ctx);
  void parseSunSettings(SimulationContext& ctx);
  void parseDataSettings(SimulationContext& ctx);
  void parseMultiScatterSettings(SimulationContext& ctx);
  void parseCrystalSettings(SimulationContext& ctx, const rapidjson::Value& c, int ci);
  void parseCrystalType(SimulationContext& ctx, const rapidjson::Value& c, int ci,
                        float population,
                        Math::Distribution axisDist, float axisMean, float axisStd,
                        Math::Distribution rollDist, float rollMean, float rollStd);
  Crystal* parseCustomCrystal(std::FILE* file);

  /* Parse rendering settings */
  void parseCameraSettings(RenderContext& ctx);
  void parseRenderSettings(RenderContext& ctx);
  void parseDataSettings(RenderContext& ctx);

  rapidjson::Document d;
  std::string filename;

  static constexpr int kMsgBufferSize = 512;
};


class SimulationContext {
friend class ContextParser;
public:
  SimulationContext();
  ~SimulationContext();

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

  CrystalContext* getCrystalContext(int i);
  RayTracingContext* getRayTracingContext(int scatterIndx, int crystalIdx);

  /* For output */
  void writeFinalDirections(const char* filename);
  // void writeRayInfo(const char* filename, float lon, float lat, float delta);
  void printCrystalInfo();

private:
  // void writeRayInfo(Files::File& file, Ray* r);     // Helper function

  std::vector<CrystalContext *> crystalCtxs;
  std::vector<std::vector<RayTracingContext *> > rayTracingCtxs;

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

  std::string dataDirectory;
};


class CrystalContext {
friend class SimulationContext;
public:
  CrystalContext() = default;
  ~CrystalContext();

  void setCrystal(Crystal* g, float populationRatio,
                  Math::Distribution axisDist, float axisMean, float axisStd,
                  Math::Distribution rollDist, float rollMean, float rollStd);
  Crystal* getCrystal();

  void fillDir(const float* incDir, float* rayDir, float* mainAxRot, int num = 1);
  // int chooseFace(const float* rayDir);

private:
  float populationRatio;
  Crystal* crystal;
  Math::OrientationGenerator oriGen;
};


class RayTracingContext {
friend class SimulationContext;
friend class Optics;
public:
  explicit RayTracingContext(SimulationContext* ctx);
  ~RayTracingContext();

  void setRayNum(int rayNum);

  void initRays(CrystalContext* ctx, int rayNum, const float* dir, const float* w, RaySegment** prevRaySeg = nullptr);
  // void clearRays();
  void commitHitResult();
  void commitPropagateResult(CrystalContext* ctx);
  bool isFinished();

  size_t copyFinishedRaySegments(RaySegment** segs, float* dir, float prob = 1.0f);

private:
  static constexpr float PROP_MIN_W = 1e-6;
  static constexpr float SCAT_MIN_W = 1e-3;

  void deleteArrays();
  int chooseFace(const float* faces, int faceNum, const float* rayDir);

  void fillPts(const float* faces, int idx, float* rayPts);

  void copyFinishedRaySegmentsRange(RaySegment** segs, float* dir, float prob,
                                    std::atomic<std::uint64_t>& k, int startIdx, int endIdx);

  SimulationContext* simCtx;

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
friend class ContextParser;
public:
  RenderContext();
  ~RenderContext();

  void loadData();

  uint32_t getImageWidth() const;
  uint32_t getImageHeight() const;
  std::string getImagePath() const;

  void renderToRgb(uint8_t* rgbData);

private:
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

}   // namespace IceHalo


#endif  // SRC_CONTEXT_H_
