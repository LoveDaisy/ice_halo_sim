#include "context.h"
#include "optics.h"
#include "threadingpool.h"

#include "rapidjson/pointer.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"

#include <utility>
#include <limits>
#include <algorithm>


namespace IceHalo {

using rapidjson::Pointer;


void CrystalContext::setCrystal(const CrystalPtr& g, float populationWeight,
                                Math::Distribution axisDist, float axisMean, float axisStd,
                                Math::Distribution rollDist, float rollMean, float rollStd) {
  this->crystal = g;
  this->populationRatio = populationWeight;
  this->oriGen = Math::OrientationGenerator(axisDist, axisMean * Math::kPi / 180.0f, axisStd * Math::kPi / 180.0f,
                                            rollDist, rollMean * Math::kPi / 180.0f, rollStd * Math::kPi / 180.0f);
}


CrystalPtr CrystalContext::getCrystal() {
  return this->crystal;
}


void CrystalContext::fillDir(const float* incDir, float* rayDir, float* mainAxRot, int num) {
  oriGen.fillData(incDir, num, rayDir, mainAxRot);
}


RayTracingContext::RayTracingContext(std::shared_ptr<SimulationContext> ctx)
    : simCtx(ctx), initRayNum(0), currentRayNum(0), activeRaySegNum(0),
      rays(nullptr), activeRaySeg(nullptr),
      gen(), dis(0.0f, 1.0f),
      mainAxRot(nullptr),
      rayDir(nullptr), rayPts(nullptr), faceNorm(nullptr), faceId(nullptr),
      rayDir2(nullptr), rayDir3(nullptr), rayPts2(nullptr), rayW2(nullptr), faceId2(nullptr) {}


RayTracingContext::~RayTracingContext() {
  deleteArrays();
}


void RayTracingContext::setRayNum(int num) {
  initRayNum = num;
  currentRayNum = num;
  int maxRayNum = initRayNum * simCtx->getMaxRecursionNum() * 3;

  deleteArrays();

  mainAxRot = new float[initRayNum * 3];
  rayDir = new float[maxRayNum * 3];
  rayPts = new float[maxRayNum * 3];
  faceNorm = new float[maxRayNum * 3];
  faceId = new int[maxRayNum];

  rayDir2 = new float[maxRayNum * 3];
  rayDir3 = new float[maxRayNum * 3];
  rayPts2 = new float[maxRayNum * 3];
  rayW2 = new float[maxRayNum];
  faceId2 = new int[maxRayNum];

  activeRaySeg = new RaySegment*[maxRayNum * 3];
  rays = new Ray*[initRayNum];
}


void RayTracingContext::initRays(CrystalContextPtr ctx,
                                 int rayNum, const float* dir, const float* w, RaySegment** prevRaySeg) {
  setRayNum(rayNum);

  auto crystal = ctx->getCrystal();
  auto* faces = new float[crystal->faceNum() * 9];
  crystal->copyFaceData(faces);

  activeRaySegNum = 0;
  Pool* pool = Pool::getInstance();
  int step = initRayNum / 80;
  for (int startIdx = 0; startIdx < initRayNum; startIdx += step) {
    int endIdx = std::min(startIdx + step, initRayNum);
    pool->addJob([ctx, dir, faces, startIdx, endIdx, this](){
      int faceNum = ctx->getCrystal()->faceNum();
      for (int i = startIdx; i < endIdx; i++) {
        ctx->fillDir(dir+i*3, rayDir+i*3, mainAxRot+i*3);

        int idx = chooseFace(faces, faceNum, rayDir+i*3);
        faceId[i] = idx;

        fillPts(faces, idx, rayPts+i*3);
        ctx->getCrystal()->copyNormalData(idx, faceNorm+i*3);
      }
    });
  }
  pool->waitFinish();

  for (int i = 0; i < initRayNum; i++) {
    auto* r = new Ray(rayPts+i*3, rayDir+i*3, w[i], faceId[i]);
    if (prevRaySeg && prevRaySeg[i]) {
      prevRaySeg[i]->nextRefract = r->firstRaySeg;
    }
    rays[i] = r;
    activeRaySeg[i] = r->firstRaySeg;
  }
  activeRaySegNum = initRayNum;

  delete[] faces;
}


void RayTracingContext::commitHitResult() {
  RaySegmentPool& raySegPool = RaySegmentPool::getInstance();
  for (int i = 0; i < currentRayNum; i++) {
    RaySegment* lastSeg = activeRaySeg[i];
    RaySegment* seg = raySegPool.getRaySegment(rayPts + i*3, rayDir2 + i*3,
                                               lastSeg->w * rayW2[i], faceId[i]);
    lastSeg->nextReflect = seg;
    seg->prev = lastSeg;
    activeRaySeg[i] = seg;

    seg = raySegPool.getRaySegment(rayPts + i*3, rayDir3 + i*3,
                                   lastSeg->w * (1.0f - rayW2[i]), faceId[i]);
    lastSeg->nextRefract = seg;
    seg->prev = lastSeg;
    activeRaySeg[currentRayNum + i] = seg;
  }

  std::memcpy(rayDir, rayDir2, currentRayNum * 3 * sizeof(float));
  std::memcpy(rayDir + currentRayNum*3, rayDir3, currentRayNum * 3 * sizeof(float));
  std::memcpy(rayPts + currentRayNum*3, rayPts, currentRayNum * 3 * sizeof(float));
  std::memcpy(faceId + currentRayNum, faceId, currentRayNum * sizeof(int));

  currentRayNum *= 2;
  activeRaySegNum = currentRayNum;
}


void RayTracingContext::commitPropagateResult(CrystalContextPtr ctx) {
  int k = 0;
  activeRaySegNum = 0;
  for (int i = 0; i < currentRayNum; i++) {
    if (faceId2[i] >= 0 && activeRaySeg[i]->w > PROP_MIN_W) {
      activeRaySeg[activeRaySegNum++] = activeRaySeg[i];
      std::memcpy(rayPts + k*3, rayPts2 + i*3, 3 * sizeof(float));
      std::memcpy(rayDir + k*3, rayDir + i*3, 3 * sizeof(float));
      faceId[k] = faceId2[i];
      ctx->getCrystal()->copyNormalData(faceId2[i], faceNorm + k*3);
      k++;
    } else if (faceId2[i] < 0) {
      activeRaySeg[i]->isFinished = true;
    }
  }
  currentRayNum = k;
}


bool RayTracingContext::isFinished() {
  return activeRaySegNum <= 0;
}


size_t RayTracingContext::copyFinishedRaySegments(RaySegment **segs, float* dir, float prob) {
  std::atomic<uint64_t> k(0);

  Pool* pool = Pool::getInstance();
  int step = initRayNum / 80;
  for (int startIdx = 0; startIdx < initRayNum; startIdx += step) {
    int endIdx = std::min(startIdx + step, initRayNum);
    pool->addJob([segs, dir, prob, &k, startIdx, endIdx, this](){
      copyFinishedRaySegmentsRange(segs, dir, prob, k, startIdx, endIdx);
    });
  }
  pool->waitFinish();

  return k.load();
}


void RayTracingContext::copyFinishedRaySegmentsRange(RaySegment** segs, float* dir, float prob,
                                                     std::atomic<uint64_t>& k, int startIdx, int endIdx) {
  std::default_random_engine randomEngine;
  std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

  std::vector<RaySegment*> v;
  for (int rayIdx = startIdx; rayIdx < endIdx; rayIdx++) {
    v.clear();
    v.push_back(rays[rayIdx]->firstRaySeg);

    while (!v.empty()) {
      RaySegment* p = v.back();
      v.pop_back();
      if (p->nextReflect && !p->isFinished) {
        v.push_back(p->nextReflect);
      }
      if (p->nextRefract && !p->isFinished) {
        v.push_back(p->nextRefract);
      }
      if (p->w > SCAT_MIN_W && p->faceId >= 0 && p->isFinished && uniformDist(randomEngine) < prob) {
        auto tmpk = k++;
        float* finalDir = dir + tmpk * 3;
        std::memcpy(finalDir, p->dir.val(), sizeof(float) * 3);
        Math::rotateZBack(mainAxRot + rayIdx * 3, finalDir);
        segs[tmpk] = p;
      }
    }
  }
}


void RayTracingContext::deleteArrays() {
  delete[] mainAxRot;
  delete[] rayDir;
  delete[] rayPts;
  delete[] faceNorm;
  delete[] faceId;

  delete[] rayDir2;
  delete[] rayDir3;
  delete[] rayPts2;
  delete[] rayW2;
  delete[] faceId2;

  delete[] activeRaySeg;
  delete[] rays;
}


int RayTracingContext::chooseFace(const float* faces, int faceNum, const float* rayDir) {
  auto* frac = new float[faceNum];

  for (int i = 0; i < faceNum; i++) {
    float v1[3], v2[3];
    Math::vec3FromTo(faces + i*9, faces + i*9 + 3, v1);
    Math::vec3FromTo(faces + i*9, faces + i*9 + 6, v2);
    float norm[3];
    Math::cross3(v1, v2, norm);
    float c = Math::dot3(rayDir, norm);
    frac[i] = c < 0 ? (-c) : 0;
  }
  for (int i = 1; i < faceNum; i++) {
    frac[i] += frac[i-1];
  }
  for (int i = 0; i < faceNum; i++) {
    frac[i] /= frac[faceNum-1];
  }
  frac[faceNum-1] = 1.0f;    // Force to 1.0f

  int idx = -1;
  float p = dis(gen);
  for (int j = 0; j < faceNum; j++) {
    if (p <= frac[j]) {
      idx = j;
      break;
    }
  }

  delete[] frac;

  return idx;
}


void RayTracingContext::fillPts(const float* faces, int idx, float* rayPts) {
  float a = dis(gen);
  float b = dis(gen);
  if (a + b > 1.0f) {
    a = 1.0f - a;
    b = 1.0f - b;
  }
  rayPts[0] = faces[idx*9+0] + a * (faces[idx*9+3] - faces[idx*9+0]) +
      b * (faces[idx*9+6] - faces[idx*9+0]);
  rayPts[1] = faces[idx*9+1] + a * (faces[idx*9+4] - faces[idx*9+1]) +
      b * (faces[idx*9+7] - faces[idx*9+1]);
  rayPts[2] = faces[idx*9+2] + a * (faces[idx*9+5] - faces[idx*9+2]) +
      b * (faces[idx*9+8] - faces[idx*9+2]);
}


SimulationContext::SimulationContext()
    : totalRayNum(0), maxRecursionNum(9),
      multiScatterNum(1), multiScatterProb(1.0f),
      currentWavelength(550.0f), sunDiameter(0.5f),
      dataDirectory("./") {}


uint64_t SimulationContext::getTotalInitRays() const {
  return totalRayNum;
}


int SimulationContext::getMaxRecursionNum() const {
  return maxRecursionNum;
}


int SimulationContext::getCrystalNum() const {
  return static_cast<int>(crystalCtxs.size());
}


int SimulationContext::getMultiScatterNum() const {
  return multiScatterNum;
}

float SimulationContext::getMultiScatterProb() const {
  return multiScatterProb;
}


void SimulationContext::setCurrentWavelength(float wavelength) {
  this->currentWavelength = wavelength;
}


float SimulationContext::getCurrentWavelength() const {
  return currentWavelength;
}


std::vector<float> SimulationContext::getWavelengths() const {
  return wavelengths;
}


void SimulationContext::fillSunDir(float* dir, uint64_t num) {
  float sunLon = std::atan2(sunDir[1], sunDir[0]);
  float sunLat = std::asin(sunDir[2] / Math::norm3(sunDir));
  float sunRot[3] = { sunLon, sunLat, 0 };

  float dz = 1.0f - std::cos(sunDiameter / 360 * Math::kPi);
  for (decltype(num) i = 0; i < num; i++) {
    float z = 1.0f - uniformDistribution(generator) * dz;
    float r = std::sqrt(1.0f - z * z);
    float q = uniformDistribution(generator) * 2 * Math::kPi;
    float x = std::cos(q) * r;
    float y = std::sin(q) * r;

    dir[i*3 + 0] = x;
    dir[i*3 + 1] = y;
    dir[i*3 + 2] = z;
  }
  Math::rotateZBack(sunRot, dir, num);
}


void SimulationContext::setSunPosition(float lon, float lat) {
  float x = -std::cos(lat * Math::kPi / 180.0f) * std::cos(lon * Math::kPi / 180.0f);
  float y = -std::cos(lat * Math::kPi / 180.0f) * std::sin(lon * Math::kPi / 180.0f);
  float z = -std::sin(lat * Math::kPi / 180.0f);

  this->sunDir[0] = x;
  this->sunDir[1] = y;
  this->sunDir[2] = z;
}


void SimulationContext::applySettings() {
  if (totalRayNum <= 0) return;

  float popWeightSum = 0.0f;
  for (auto c : crystalCtxs) {
    popWeightSum += c->populationRatio;
  }
  for (auto c : crystalCtxs) {
    c->populationRatio /= popWeightSum;
  }

  setCrystalRayNum(0, totalRayNum);
}


void SimulationContext::setCrystalRayNum(int scatterIdx, uint64_t totalRayNum) {
  if (scatterIdx >= static_cast<int>(rayTracingCtxs.size())) {
    return;
  }

  size_t popSize = crystalCtxs.size();
  size_t currentIdx = 0;
  for (decltype(popSize) i = 0; i < popSize-1; i++) {
    auto tmpPopSize = static_cast<int>(crystalCtxs[i]->populationRatio * totalRayNum);
    rayTracingCtxs[scatterIdx][i]->setRayNum(tmpPopSize);
    currentIdx += tmpPopSize;
  }
  auto tmpPopSize = static_cast<int>(totalRayNum - currentIdx);
  rayTracingCtxs[scatterIdx][popSize-1]->setRayNum(tmpPopSize);
}


CrystalContextPtr SimulationContext::getCrystalContext(int i) {
  return crystalCtxs[i];
}


RayTracingContextPtr SimulationContext::getRayTracingContext(int scatterIdx, int crystalIdx) {
  return rayTracingCtxs[scatterIdx][crystalIdx];
}


void SimulationContext::writeFinalDirections(const char* filename) {
  File file(dataDirectory.c_str(), filename);
  if (!file.open(OpenMode::kWrite | OpenMode::kBinary)) return;

  file.write(currentWavelength);

  std::vector<RaySegment*> v;
  for (const auto& rcs : rayTracingCtxs) {
    for (const auto& rc : rcs) {
      size_t currentIdx = 0;
      for (int i = 0; i < rc->initRayNum; i++) {
        auto r = rc->rays[i];
        v.clear();
        v.push_back(r->firstRaySeg);

        while (!v.empty()) {
          RaySegment* p = v.back();
          v.pop_back();
          if (p->nextReflect && !p->isFinished) {
            v.push_back(p->nextReflect);
          }
          if (p->nextRefract && !p->isFinished) {
            v.push_back(p->nextRefract);
          }
          if (!p->nextReflect && !p->nextRefract &&
                p->isValidEnd() && Math::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0 - 1e-5) {
            float finalDir[3];
            std::memcpy(finalDir, p->dir.val(), sizeof(float) * 3);
            Math::rotateZBack(rc->mainAxRot + currentIdx * 3, finalDir);
            file.write(finalDir, 3);
            file.write(p->w);
          }
        }
        currentIdx++;
      }
    }
  }

  file.close();
}


void SimulationContext::printCrystalInfo() {
  for (const auto& c : crystalCtxs) {
    auto g = c->getCrystal();
    printf("--\n");
    for (auto& v : g->getVertexes()) {
      printf("v %+.4f %+.4f %+.4f\n", v.x(), v.y(), v.z());
    }
    for (const auto& f : g->getFaces()) {
      auto idx = f.idx();
      printf("f %d %d %d\n", idx[0] + 1, idx[1] + 1, idx[2] + 1);
    }
  }
}



RenderContext::RenderContext() :
    imgHei(0), imgWid(0), offsetY(0), offsetX(0),
    visibleSemiSphere(VisibleSemiSphere::UPPER),
    totalW(0), intensityFactor(1.0), showHorizontal(true),
    dataDirectory("./"),
    projectionType(ProjectionType::EQUI_AREA) {}


RenderContext::~RenderContext() {
  for (const auto& kv : spectrumData) {
    delete[] kv.second;
  }
}


uint32_t RenderContext::getImageWidth() const {
  return imgWid;
}


uint32_t RenderContext::getImageHeight() const {
  return imgHei;
}


std::string RenderContext::getImagePath() const {
  return pathJoin(dataDirectory, "img.jpg");
}


void RenderContext::renderToRgb(uint8_t* rgbData) {
  auto wlNum = spectrumData.size();
  auto* wlData = new float[wlNum];
  auto* flatSpecData = new float[wlNum * imgWid * imgHei];

  copySpectrumData(wlData, flatSpecData);
  if (rayColor[0] < 0) {
    render.rgb(static_cast<int>(wlNum), wlData, imgWid * imgHei, flatSpecData, rgbData);
  } else {
    render.gray(static_cast<int>(wlNum), wlData, imgWid * imgHei, flatSpecData, rgbData);
  }
  for (size_t i = 0; i < imgWid * imgHei; i++) {
    for (int c = 0; c <= 2; c++) {
      auto v = static_cast<int>(backgroundColor[c] * 255);
      if (rayColor[0] < 0) {
        v += rgbData[i * 3 + c];
      } else {
        v += static_cast<int>(rgbData[i * 3 + c] * rayColor[c]);
      }
      v = std::max(std::min(v, 255), 0);
      rgbData[i * 3 + c] = static_cast<uint8_t>(v);
    }
  }

  /* Draw horizontal */
  // float imgR = std::min(imgWid / 2, imgHei) / 2.0f;
  // TODO

  delete[] wlData;
  delete[] flatSpecData;
}


void RenderContext::copySpectrumData(float* wavelengthData, float* spectrumData) const {
  int k = 0;
  for (const auto& kv : this->spectrumData) {
    wavelengthData[k] = kv.first;
    std::memcpy(spectrumData + k * imgWid * imgHei, kv.second, imgWid * imgHei * sizeof(float));
    k++;
  }
  for (uint64_t i = 0; i < imgWid * imgHei * this->spectrumData.size(); i++) {
    spectrumData[i] *= 8 * 4e3 / totalW * intensityFactor;
  }
}


void RenderContext::loadData() {
  std::vector<File> files = listDataFiles(dataDirectory.c_str());
  int i = 0;
  for (auto& f : files) {
    auto t0 = std::chrono::system_clock::now();
    auto num = loadDataFromFile(f);
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::ratio<1, 1000> > diff = t1 - t0;
    printf(" Loading data (%d/%lu): %.2fms; total %d pts\n", i + 1, files.size(), diff.count(), num);
    i++;
  }
}


int RenderContext::loadDataFromFile(File& file) {
  auto fileSize = file.getSize();
  auto* readBuffer = new float[fileSize / sizeof(float)];

  file.open(OpenMode::kRead | OpenMode::kBinary);
  auto readCount = file.read(readBuffer, 1);
  if (readCount <= 0) {
    return -1;
  }

  auto wavelength = static_cast<int>(readBuffer[0]);
  if (wavelength < SpectrumRenderer::MIN_WL || wavelength > SpectrumRenderer::MAX_WL) {
    return -1;
  }

  readCount = file.read(readBuffer, fileSize / sizeof(float));
  auto totalCount = readCount / 4;
  file.close();

  if (totalCount == 0) {
    return static_cast<int>(totalCount);
  }

  auto* tmpDir = new float[totalCount * 3];
  auto* tmpW = new float[totalCount];
  for (decltype(readCount) i = 0; i < totalCount; i++) {
    std::memcpy(tmpDir + i * 3, readBuffer + i * 4, 3 * sizeof(float));
    tmpW[i] = readBuffer[i * 4 + 3];
    totalW += tmpW[i];
  }
  delete[] readBuffer;

  auto* tmpXY = new int[totalCount * 2];
  projectionFunctions[projectionType](camRot, fov, totalCount, tmpDir, imgWid, imgHei, tmpXY, visibleSemiSphere);
  delete[] tmpDir;

  float* currentData = nullptr;
  auto it = spectrumData.find(wavelength);
  if (it != spectrumData.end()) {
    currentData = it->second;
  } else {
    currentData = new float[imgHei * imgWid];
    for (decltype(imgHei) i = 0; i < imgHei * imgWid; i++) {
      currentData[i] = 0;
    }
    spectrumData[wavelength] = currentData;
  }

  for (decltype(totalCount) i = 0; i < totalCount; i++) {
    int x = tmpXY[i * 2 + 0];
    int y = tmpXY[i * 2 + 1];
    if (x == std::numeric_limits<int>::min() || y == std::numeric_limits<int>::min()) {
      continue;
    }
    if (projectionType != ProjectionType::DUAL_EQUI_AREA && projectionType != ProjectionType::DUAL_EQUI_DISTANT) {
      x += offsetX;
      y += offsetY;
    }
    if (x < 0 || x >= static_cast<int>(imgWid) || y < 0 || y >= static_cast<int>(imgHei)) {
      continue;
    }
    currentData[y * imgWid + x] += tmpW[i];
  }
  delete[] tmpXY;
  delete[] tmpW;

  return static_cast<int>(totalCount);
}



ContextParser::ContextParser(rapidjson::Document& d, const char* filename) :
    d(std::move(d)), filename(filename) {}


std::shared_ptr<ContextParser> ContextParser::createFileParser(const char* filename) {
  printf("Reading config from: %s\n", filename);

  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    printf("ERROR: file %s cannot be open!\n", filename);
    return nullptr;
  }

  char readBuffer[65536];
  rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  rapidjson::Document d;
  if (d.ParseStream(is).HasParseError()) {
    fprintf(stderr, "\nError(offset %u): %s\n", (unsigned)d.GetErrorOffset(),
      GetParseError_En(d.GetParseError()));
    fclose(fp);
    return nullptr;
  }

  fclose(fp);

  return std::shared_ptr<ContextParser>(new ContextParser(d, filename));
}


void ContextParser::parseRaySettings(std::shared_ptr<SimulationContext> ctx) {
  /* Parsing ray number */
  ctx->totalRayNum = 10000;
  auto* p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <ray.number>, using default 10000!\n");
  } else if (!p->IsUint()) {
    fprintf(stderr, "\nWARNING! Config <ray.number> is not unsigned int, using default 10000!\n");
  } else {
    ctx->totalRayNum = p->GetUint();
  }

  /* Parsing wavelengths */
  ctx->wavelengths.clear();
  ctx->wavelengths.push_back(550);
  p = Pointer("/ray/wavelength").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <ray.wavelength>, using default 550!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <ray.wavelength> is not an array, using default 550!\n");
  } else if (!(*p)[0].IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <ray.wavelength> connot be recgonized, using default 550!\n");
  } else {
    ctx->wavelengths.clear();
    for (const auto& pi : p->GetArray()) {
      ctx->wavelengths.push_back(static_cast<float &&>(pi.GetDouble()));
    }
  }
}


void ContextParser::parseBasicSettings(std::shared_ptr<SimulationContext> ctx) {
  int maxRecursion = 9;
  auto* p = Pointer("/max_recursion").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <max_recursion>, using default 9!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <max_recursion> is not a integer, using default 9!\n");
  } else {
    maxRecursion = std::min(std::max(p->GetInt(), 1), 10);
  }
  ctx->maxRecursionNum = maxRecursion;
}


void ContextParser::parseMultiScatterSettings(std::shared_ptr<SimulationContext> ctx) {
  int multiScattering = 1;
  auto* p = Pointer("/multi_scatter/repeat").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <multi_scatter.repeat>, using default value 1!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! Config <multi_scatter.repeat> is not a integer, using default 1!\n");
  } else {
    multiScattering = std::min(std::max(p->GetInt(), 1), 4);
  }
  ctx->multiScatterNum = multiScattering;

  float prob = 1.0f;
  p = Pointer("/multi_scatter/probability").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <multi_scatter.probability>, using default value 1.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <multi_scatter.probability> is not a number, using default 1.0!\n");
  } else {
    prob = static_cast<float>(p->GetDouble());
    prob = std::max(std::min(prob, 1.0f), 0.0f);
  }
  ctx->multiScatterProb = prob;

  for (int i = 0; i < multiScattering; i++) {
    ctx->rayTracingCtxs.emplace_back(std::vector<RayTracingContextPtr>());
  }
}


void ContextParser::parseSunSettings(std::shared_ptr<SimulationContext> ctx) {
  // Parsing sun altitude
  float sunAltitude = 0.0f;
  auto* p = Pointer("/sun/altitude").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <sun.altitude>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <sun.altitude> is not a number, using default 0.0!\n");
  } else {
    sunAltitude = static_cast<float>(p->GetDouble());
  }
  ctx->setSunPosition(90.0f, sunAltitude);

  // Parsing sun diameter
  float sunDiameter = 0.5f;
  p = Pointer("/sun/diameter").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <sun.diameter>, using default 0.5!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <sun.diameter> is not a number, using default 0.5!\n");
  } else {
    sunDiameter = static_cast<float>(p->GetDouble());
  }
  ctx->sunDiameter = sunDiameter;
}


void ContextParser::parseDataSettings(std::shared_ptr<SimulationContext> ctx) {
  /* Parsing output data directory */
  std::string dir = "./";
  auto* p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default './'!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default './'!\n");
  } else {
    dir = p->GetString();
  }
  ctx->dataDirectory = dir;
}


void ContextParser::parseCrystalSettings(std::shared_ptr<SimulationContext> ctx, const rapidjson::Value& c, int ci) {
  using Math::Distribution;

  char msgBuffer[kMsgBufferSize];

  auto* p = Pointer("/enable").Get(c);
  if (p == nullptr || !p->IsBool()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].enable> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else if (!p->GetBool()) {
    return;
  }

  Distribution axisDist, rollDist;
  float axisMean, rollMean;
  float axisStd, rollStd;

  p = Pointer("/axis/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.type> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else if (*p == "Gauss") {
    axisDist = Distribution::GAUSS;
  } else if (*p == "Uniform") {
    axisDist = Distribution::UNIFORM;
  } else {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.type> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  }

  p = Pointer("/roll/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else if (*p == "Gauss") {
    rollDist = Distribution::GAUSS;
  } else if (*p == "Uniform") {
    rollDist = Distribution::UNIFORM;
  } else {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  }

  p = Pointer("/axis/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.mean> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    axisMean = static_cast<float>(90 - p->GetDouble());
  }

  p = Pointer("/axis/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.std> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    axisStd = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.mean> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    rollMean = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.std> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    rollStd = static_cast<float>(p->GetDouble());
  }

  float population = 1.0;
  p = Pointer("/population").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    fprintf(stderr, "\nWARNING! <crystal[%d].population> cannot recgonize, using default 1.0!\n", ci);
  } else {
    population = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].type> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    parseCrystalType(ctx, c, ci, population, axisDist, axisMean, axisStd, rollDist, rollMean, rollStd);
  }

  for (auto& rtc : ctx->rayTracingCtxs) {
    rtc.push_back(std::make_shared<RayTracingContext>(ctx));
  }
}

void ContextParser::parseCrystalType(std::shared_ptr<SimulationContext> ctx, const rapidjson::Value& c, int ci,
                                     float population,
                                     Math::Distribution axisDist, float axisMean, float axisStd,
                                     Math::Distribution rollDist, float rollMean, float rollStd) {
  char msgBuffer[kMsgBufferSize];

  auto cryCtx = std::make_shared<CrystalContext>();
  const auto* p = Pointer("/parameter").Get(c);
  if (c["type"] == "HexCylinder") {
    if (p == nullptr || !p->IsNumber()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    }
    auto h = static_cast<float>(p->GetDouble());
    cryCtx->setCrystal(Crystal::createHexCylinder(h), population,
      axisDist, axisMean, axisStd,
      rollDist, rollMean, rollStd);
  } else if (c["type"] == "HexPyramid") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 3) {
      auto h1 = static_cast<float>((*p)[0].GetDouble());
      auto h2 = static_cast<float>((*p)[1].GetDouble());
      auto h3 = static_cast<float>((*p)[2].GetDouble());
      cryCtx->setCrystal(Crystal::createHexPyramid(h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    } else if (p->Size() == 5) {
      int i1 = (*p)[0].GetInt();
      int i2 = (*p)[1].GetInt();
      auto h1 = static_cast<float>((*p)[2].GetDouble());
      auto h2 = static_cast<float>((*p)[3].GetDouble());
      auto h3 = static_cast<float>((*p)[4].GetDouble());
      cryCtx->setCrystal(Crystal::createHexPyramid(i1, i2, h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    } else if (p->Size() == 7) {
      int upperIdx1 = (*p)[0].GetInt();
      int upperIdx2 = (*p)[1].GetInt();
      int lowerIdx1 = (*p)[2].GetInt();
      int lowerIdx2 = (*p)[3].GetInt();
      auto h1 = static_cast<float>((*p)[4].GetDouble());
      auto h2 = static_cast<float>((*p)[5].GetDouble());
      auto h3 = static_cast<float>((*p)[6].GetDouble());
      cryCtx->setCrystal(Crystal::createHexPyramid(upperIdx1, upperIdx2, lowerIdx1,
        lowerIdx2, h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "HexPyramidStackHalf") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 7) {
      int upperIdx1 = (*p)[0].GetInt();
      int upperIdx2 = (*p)[1].GetInt();
      int lowerIdx1 = (*p)[2].GetInt();
      int lowerIdx2 = (*p)[3].GetInt();
      auto h1 = static_cast<float>((*p)[4].GetDouble());
      auto h2 = static_cast<float>((*p)[5].GetDouble());
      auto h3 = static_cast<float>((*p)[6].GetDouble());
      cryCtx->setCrystal(Crystal::createHexPyramidStackHalf(upperIdx1, upperIdx2, lowerIdx1,
        lowerIdx2, h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "TriPyramid") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 5) {
      int i1 = (*p)[0].GetInt();
      int i2 = (*p)[1].GetInt();
      auto h1 = static_cast<float>((*p)[2].GetDouble());
      auto h2 = static_cast<float>((*p)[3].GetDouble());
      auto h3 = static_cast<float>((*p)[4].GetDouble());
      cryCtx->setCrystal(Crystal::createTriPyramid(i1, i2, h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "CubicPyramid") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 2) {
      auto h1 = static_cast<float>((*p)[0].GetDouble());
      auto h2 = static_cast<float>((*p)[1].GetDouble());
      cryCtx->setCrystal(Crystal::createCubicPyramid(h1, h2), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "IrregularHexCylinder") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 7) {
      auto d1 = static_cast<float>((*p)[0].GetDouble());
      auto d2 = static_cast<float>((*p)[1].GetDouble());
      auto d3 = static_cast<float>((*p)[2].GetDouble());
      auto d4 = static_cast<float>((*p)[3].GetDouble());
      auto d5 = static_cast<float>((*p)[4].GetDouble());
      auto d6 = static_cast<float>((*p)[5].GetDouble());
      auto h = static_cast<float>((*p)[6].GetDouble());

      float dist[6] = { d1, d2, d3, d4, d5, d6 };
      cryCtx->setCrystal(Crystal::createIrregularHexCylinder(dist, h), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    }
  } else if (c["type"] == "IrregularHexPyramid") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 13) {
      auto d1 = static_cast<float>((*p)[0].GetDouble());
      auto d2 = static_cast<float>((*p)[1].GetDouble());
      auto d3 = static_cast<float>((*p)[2].GetDouble());
      auto d4 = static_cast<float>((*p)[3].GetDouble());
      auto d5 = static_cast<float>((*p)[4].GetDouble());
      auto d6 = static_cast<float>((*p)[5].GetDouble());
      int i1 = (*p)[6].GetInt();
      int i2 = (*p)[7].GetInt();
      int i3 = (*p)[8].GetInt();
      int i4 = (*p)[9].GetInt();
      auto h1 = static_cast<float>((*p)[10].GetDouble());
      auto h2 = static_cast<float>((*p)[11].GetDouble());
      auto h3 = static_cast<float>((*p)[12].GetDouble());

      float dist[6] = { d1, d2, d3, d4, d5, d6 };
      int idx[4] = { i1, i2, i3, i4 };
      float height[3] = { h1, h2, h3 };

      cryCtx->setCrystal(Crystal::createIrregularHexPyramid(dist, idx, height), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "Custom") {
    if (p == nullptr || !p->IsString()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recgonize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else {
      char modelFileNameBuffer[512] = { 0 };
      auto n = filename.rfind('/');
      if (n == std::string::npos) {
        snprintf(modelFileNameBuffer, kMsgBufferSize, "models/%s", p->GetString());
      } else {
        snprintf(modelFileNameBuffer, kMsgBufferSize, "%s/models/%s", filename.substr(0, n).c_str(), p->GetString());
      }
      std::FILE* file = fopen(modelFileNameBuffer, "r");
      if (!file) {
        snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot open model file!", ci);
        throw std::invalid_argument(msgBuffer);
      }
      auto crystal = parseCustomCrystal(file);
      cryCtx->setCrystal(crystal, population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd);
      fclose(file);
    }
  } else {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].type> cannot recgonize!", ci);
    throw std::invalid_argument(msgBuffer);
  }
  ctx->crystalCtxs.push_back(cryCtx);
}


CrystalPtr ContextParser::parseCustomCrystal(std::FILE* file) {
  std::vector<Math::Vec3f> vertexes;
  std::vector<Math::TriangleIdx> faces;
  float vbuf[3];
  int fbuf[3];
  int c;
  while ((c = std::fgetc(file)) != EOF) {
    switch (c) {
      case 'v':
      case 'V':
        std::fscanf(file, "%f %f %f", vbuf+0, vbuf+1, vbuf+2);
        vertexes.emplace_back(vbuf);
        break;
      case 'f':
      case 'F':
        std::fscanf(file, "%d %d %d", fbuf+0, fbuf+1, fbuf+2);
        faces.emplace_back(fbuf[0]-1, fbuf[1]-1, fbuf[2]-1);
        break;
      default:
        break;
    }
  }
  return std::make_shared<Crystal>(vertexes, faces);
}


std::shared_ptr<SimulationContext> ContextParser::parseSimulationSettings() {
  char msgBuffer[kMsgBufferSize];

  auto ctx = std::make_shared<SimulationContext>();
  parseRaySettings(ctx);
  parseBasicSettings(ctx);
  parseSunSettings(ctx);
  parseDataSettings(ctx);
  parseMultiScatterSettings(ctx);

  const auto* p = Pointer("/crystal").Get(d);
  if (p == nullptr || !p->IsArray()) {
    snprintf(msgBuffer, kMsgBufferSize, "Missing <crystal>. Parsing fail!");
    throw std::invalid_argument(msgBuffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    parseCrystalSettings(ctx, c, ci);
    ci++;
  }

  return ctx;
}


RenderContext ContextParser::parseRenderingSettings() {
  RenderContext ctx;
  parseCameraSettings(&ctx);
  parseRenderSettings(&ctx);
  parseDataSettings(&ctx);
  return ctx;
}


void ContextParser::parseCameraSettings(RenderContext* ctx) {
  ctx->camRot[0] = 90.0f;
  ctx->camRot[1] = 89.9f;
  ctx->camRot[2] = 0.0f;
  ctx->fov = 120.0f;
  ctx->imgWid = 800;
  ctx->imgHei = 800;
  ctx->projectionType = ProjectionType::EQUI_AREA;

  auto* p = Pointer("/camera/azimuth").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.azimuth>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.azimuth> is not a number, using default 90.0!\n");
  } else {
    auto az = static_cast<float>(p->GetDouble());
    az = std::max(std::min(az, 360.0f), 0.0f);
    ctx->camRot[0] = 90.0f - az;
  }

  p = Pointer("/camera/elevation").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.elevation>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.elevation> is not a number, using default 90.0!\n");
  } else {
    auto el = static_cast<float>(p->GetDouble());
    el = std::max(std::min(el, 89.999f), -89.999f);
    ctx->camRot[1] = el;
  }

  p = Pointer("/camera/rotation").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.rotation>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.rotation> is not a number, using default 0.0!\n");
  } else {
    auto rot = static_cast<float>(p->GetDouble());
    rot = std::max(std::min(rot, 180.0f), -180.0f);
    ctx->camRot[2] = rot;
  }

  p = Pointer("/camera/fov").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.fov>, using default 120.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.fov> is not a number, using default 120.0!\n");
  } else {
    auto fov = static_cast<float>(p->GetDouble());
    fov = std::max(std::min(fov, 140.0f), 0.0f);
    ctx->fov = fov;
  }

  p = Pointer("/camera/width").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.width>, using default 800!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <camera.width> is not an integer, using default 800!\n");
  } else {
    int width = p->GetInt();
    width = std::max(width, 0);
    ctx->imgWid = static_cast<uint32_t>(width);
  }

  p = Pointer("/camera/height").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.height>, using default 800!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <camera.height> is not an integer, using default 800!\n");
  } else {
    int height = p->GetInt();
    height = std::max(height, 0);
    ctx->imgHei = static_cast<uint32_t>(height);
  }

  p = Pointer("/camera/lens").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.lens>, using default equi-area fisheye!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! config <camera.lens> is not a string, using default equi-area fisheye!\n");
  } else {
    if (*p == "linear") {
      ctx->projectionType = ProjectionType::LINEAR;
    } else if (*p == "fisheye") {
      ctx->projectionType = ProjectionType::EQUI_AREA;
    } else if (*p == "dual_fisheye_equidistant") {
      ctx->projectionType = ProjectionType::DUAL_EQUI_DISTANT;
    } else if (*p == "dual_fisheye_equiarea") {
      ctx->projectionType = ProjectionType::DUAL_EQUI_AREA;
    } else {
      fprintf(stderr, "\nWARNING! config <camera.lens> cannot be recgonized, using default equi-area fisheye!\n");
    }
  }
}


void ContextParser::parseRenderSettings(RenderContext* ctx) {
  ctx->visibleSemiSphere = VisibleSemiSphere::UPPER;
  ctx->intensityFactor = 1.0;
  ctx->offsetY = 0;
  ctx->offsetX = 0;
  ctx->backgroundColor[0] = 0;
  ctx->backgroundColor[1] = 0;
  ctx->backgroundColor[2] = 0;
  ctx->rayColor[0] = -1;
  ctx->rayColor[1] = -1;
  ctx->rayColor[2] = -1;
  ctx->showHorizontal = true;

  auto* p = Pointer("/render/visible_semi_sphere").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.visible_semi_sphere>, using default UPPER!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> is not a string, using default UPPER!\n");
  } else if (*p == "upper") {
    ctx->visibleSemiSphere = VisibleSemiSphere::UPPER;
  } else if (*p == "lower") {
    ctx->visibleSemiSphere = VisibleSemiSphere::LOWER;
  } else if (*p == "camera") {
    ctx->visibleSemiSphere = VisibleSemiSphere::CAMERA;
  } else if (*p == "full") {
    ctx->visibleSemiSphere = VisibleSemiSphere::FULL;
  } else {
    fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> cannot be recgonized, using default UPPER!\n");
  }

  p = Pointer("/render/intensity_factor").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.intensity_factor>, using default 1.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <render.intensity_factor> is not a number, using default 1.0!\n");
  } else {
    double f = p->GetDouble();
    f = std::max(std::min(f, 100.0), 0.01);
    ctx->intensityFactor = f;
  }

  p = Pointer("/render/offset").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.offset>, using default [0, 0]!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <render.offset> is not an array, using default [0, 0]!\n");
  } else if (p->Size() != 2 || !(*p)[0].IsInt() || !(*p)[1].IsInt()) {
    fprintf(stderr, "\nWARNING! Config <render.offset> cannot be recgonized, using default [0, 0]!\n");
  } else {
    int offsetX = (*p)[0].GetInt();
    int offsetY = (*p)[1].GetInt();
    offsetX = std::max(std::min(offsetX, static_cast<int>(ctx->imgWid / 2)),
      -static_cast<int>(ctx->imgWid / 2));
    offsetY = std::max(std::min(offsetY, static_cast<int>(ctx->imgHei / 2)),
      -static_cast<int>(ctx->imgHei / 2));
    ctx->offsetX = offsetX;
    ctx->offsetY = offsetY;
  }

  p = Pointer("/render/background_color").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default [0,0,0]!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array, using default [0,0,0]!\n");
  } else if (p->Size() != 3 || !(*p)[0].IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recgonized, using default [0,0,0]!\n");
  } else {
    auto pa = p->GetArray();
    for (int i = 0; i < 3; i++) {
      ctx->backgroundColor[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
    }
  }

  p = Pointer("/render/ray_color").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default real color!\n");
  } else if (!p->IsArray() && !p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array nor a string, using default real color!\n");
  } else if (p->IsArray() && (p->Size() != 3 || !(*p)[0].IsNumber())) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recgonized, using default real color!\n");
  } else if (p->IsString() && (*p) != "real") {
    fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recgonized, using default real color!\n");
  } else if (p->IsArray()) {
    auto pa = p->GetArray();
    for (int i = 0; i < 3; i++) {
      ctx->rayColor[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
    }
  }

  p = Pointer("/render/show_horizontal").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.show_horizontal>, using default true!\n");
  } else if (!p->IsBool()) {
    fprintf(stderr, "\nWARNING! Config <render.show_horizontal> is not a boolean, using default true!\n");
  } else {
    ctx->showHorizontal = p->GetBool();
  }
}


void ContextParser::parseDataSettings(RenderContext* ctx) {
  ctx->dataDirectory = "./";
  auto* p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default './'!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default './'!\n");
  } else {
    ctx->dataDirectory = p->GetString();
  }
}

}   // namespace IceHalo

