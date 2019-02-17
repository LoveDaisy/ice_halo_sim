#include <utility>

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

CrystalContext::CrystalContext(CrystalPtrU&& g, float population,
                               Math::Distribution axisDist, float axisMean, float axisStd,
                               Math::Distribution rollDist, float rollMean, float rollStd)
    : crystal(std::move(g)),
      axis_dist_(axisDist), roll_dist_(rollDist),
      axis_mean_(axisMean), roll_mean_(rollMean),
      axis_std_(axisStd), roll_std_(rollStd),
      population_(population) {}


CrystalPtr CrystalContext::getCrystal() {
  return this->crystal;
}


Math::Distribution CrystalContext::getAxisDist() {
  return axis_dist_;
}


Math::Distribution CrystalContext::getRollDist() {
  return roll_dist_;
}


float CrystalContext::getAxisMean() {
  return axis_mean_;
}


float CrystalContext::getRollMean() {
  return roll_mean_;
}


float CrystalContext::getAxisStd() {
  return axis_std_;
}


float CrystalContext::getRollStd() {
  return roll_std_;
}


float CrystalContext::getPopulation() {
  return population_;
}


SimulationContext::SimulationContext(const char* filename, rapidjson::Document& d)
    : totalRayNum(0), maxRecursionNum(9),
      multiScatterNum(1), multiScatterProb(1.0f),
      currentWavelength(550.0f), sunDiameter(0.5f),
      configFileName(filename), dataDirectory("./") {
  constexpr size_t kTmpBufferSize = 65536;
  char buffer[kTmpBufferSize];

  parseRaySettings(d);
  parseBasicSettings(d);
  parseSunSettings(d);
  parseDataSettings(d);
  parseMultiScatterSettings(d);

  const auto* p = Pointer("/crystal").Get(d);
  if (p == nullptr || !p->IsArray()) {
    snprintf(buffer, kTmpBufferSize, "Missing <crystal>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    parseCrystalSettings(c, ci);
    ci++;
  }
}


std::unique_ptr<SimulationContext> SimulationContext::createFromFile(const char *filename) {
  printf("Reading config from: %s\n", filename);

  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    printf("ERROR: file %s cannot be open!\n", filename);
    return nullptr;
  }

  constexpr size_t kTmpBufferSize = 65536;
  char buffer[kTmpBufferSize];
  rapidjson::FileReadStream is(fp, buffer, sizeof(buffer));

  rapidjson::Document d;
  if (d.ParseStream(is).HasParseError()) {
    fprintf(stderr, "\nError(offset %u): %s\n", (unsigned)d.GetErrorOffset(),
            GetParseError_En(d.GetParseError()));
    fclose(fp);
    return nullptr;
  }

  fclose(fp);

  return std::unique_ptr<SimulationContext>(new SimulationContext(filename, d));
}


void SimulationContext::parseBasicSettings(rapidjson::Document& d) {
  int maxRecursion = 9;
  auto* p = Pointer("/max_recursion").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <max_recursion>, using default 9!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <max_recursion> is not a integer, using default 9!\n");
  } else {
    maxRecursion = std::min(std::max(p->GetInt(), 1), 10);
  }
  maxRecursionNum = maxRecursion;
}


void SimulationContext::parseRaySettings(rapidjson::Document& d) {
  /* Parsing ray number */
  totalRayNum = 10000;
  auto* p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <ray.number>, using default 10000!\n");
  } else if (!p->IsUint()) {
    fprintf(stderr, "\nWARNING! Config <ray.number> is not unsigned int, using default 10000!\n");
  } else {
    totalRayNum = p->GetUint();
  }

  /* Parsing wavelengths */
  wavelengths.clear();
  wavelengths.push_back(550);
  p = Pointer("/ray/wavelength").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <ray.wavelength>, using default 550!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <ray.wavelength> is not an array, using default 550!\n");
  } else if (!(*p)[0].IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <ray.wavelength> connot be recognized, using default 550!\n");
  } else {
    wavelengths.clear();
    for (const auto& pi : p->GetArray()) {
      wavelengths.push_back(static_cast<float &&>(pi.GetDouble()));
    }
  }
}


void SimulationContext::parseSunSettings(rapidjson::Document& d) {
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
  setSunPosition(90.0f, sunAltitude);

  // Parsing sun diameter
  sunDiameter = 0.5f;
  p = Pointer("/sun/diameter").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <sun.diameter>, using default 0.5!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <sun.diameter> is not a number, using default 0.5!\n");
  } else {
    sunDiameter = static_cast<float>(p->GetDouble());
  }
}


void SimulationContext::parseDataSettings(rapidjson::Document& d) {
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
  dataDirectory = dir;
}


void SimulationContext::parseMultiScatterSettings(rapidjson::Document& d) {
  int multiScattering = 1;
  auto* p = Pointer("/multi_scatter/repeat").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <multi_scatter.repeat>, using default value 1!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! Config <multi_scatter.repeat> is not a integer, using default 1!\n");
  } else {
    multiScattering = std::min(std::max(p->GetInt(), 1), 4);
  }
  multiScatterNum = multiScattering;

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
  multiScatterProb = prob;
}


void SimulationContext::parseCrystalSettings(const rapidjson::Value& c, int ci) {
  using Math::Distribution;

  constexpr size_t kMsgBufferSize = 256;
  char msgBuffer[kMsgBufferSize];

  auto* p = Pointer("/enable").Get(c);
  if (p == nullptr || !p->IsBool()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].enable> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else if (!p->GetBool()) {
    return;
  }

  Distribution axisDist, rollDist;
  float axisMean, rollMean;
  float axisStd, rollStd;

  p = Pointer("/axis/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else if (*p == "Gauss") {
    axisDist = Distribution::GAUSS;
  } else if (*p == "Uniform") {
    axisDist = Distribution::UNIFORM;
  } else {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  }

  p = Pointer("/roll/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else if (*p == "Gauss") {
    rollDist = Distribution::GAUSS;
  } else if (*p == "Uniform") {
    rollDist = Distribution::UNIFORM;
  } else {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  }

  p = Pointer("/axis/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.mean> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    axisMean = static_cast<float>(90 - p->GetDouble());
  }

  p = Pointer("/axis/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].axis.std> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    axisStd = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.mean> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    rollMean = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].roll.std> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    rollStd = static_cast<float>(p->GetDouble());
  }

  float population = 1.0;
  p = Pointer("/population").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    fprintf(stderr, "\nWARNING! <crystal[%d].population> cannot recognize, using default 1.0!\n", ci);
  } else {
    population = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  } else {
    parseCrystalType(c, ci, population, axisDist, axisMean, axisStd, rollDist, rollMean, rollStd);
  }
}


void SimulationContext::parseCrystalType(const rapidjson::Value& c, int ci,
                                         float population,
                                         Math::Distribution axisDist, float axisMean, float axisStd,
                                         Math::Distribution rollDist, float rollMean, float rollStd) {
  constexpr size_t kMsgBufferSize = 256;
  char msgBuffer[kMsgBufferSize];

  const auto* p = Pointer("/parameter").Get(c);
  if (c["type"] == "HexCylinder") {
    if (p == nullptr || !p->IsNumber()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
      throw std::invalid_argument(msgBuffer);
    }
    auto h = static_cast<float>(p->GetDouble());
    crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
      Crystal::createHexPrism(h), population,
      axisDist, axisMean, axisStd,
      rollDist, rollMean, rollStd));
  } else if (c["type"] == "HexPyramid") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 3) {
      auto h1 = static_cast<float>((*p)[0].GetDouble());
      auto h2 = static_cast<float>((*p)[1].GetDouble());
      auto h3 = static_cast<float>((*p)[2].GetDouble());
      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createHexPyramid(h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    } else if (p->Size() == 5) {
      int i1 = (*p)[0].GetInt();
      int i2 = (*p)[1].GetInt();
      auto h1 = static_cast<float>((*p)[2].GetDouble());
      auto h2 = static_cast<float>((*p)[3].GetDouble());
      auto h3 = static_cast<float>((*p)[4].GetDouble());
      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createHexPyramid(i1, i2, h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    } else if (p->Size() == 7) {
      int upperIdx1 = (*p)[0].GetInt();
      int upperIdx2 = (*p)[1].GetInt();
      int lowerIdx1 = (*p)[2].GetInt();
      int lowerIdx2 = (*p)[3].GetInt();
      auto h1 = static_cast<float>((*p)[4].GetDouble());
      auto h2 = static_cast<float>((*p)[5].GetDouble());
      auto h3 = static_cast<float>((*p)[6].GetDouble());
      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createHexPyramid(upperIdx1, upperIdx2, lowerIdx1,
                                  lowerIdx2, h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "HexPyramidStackHalf") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 7) {
      int upperIdx1 = (*p)[0].GetInt();
      int upperIdx2 = (*p)[1].GetInt();
      int lowerIdx1 = (*p)[2].GetInt();
      int lowerIdx2 = (*p)[3].GetInt();
      auto h1 = static_cast<float>((*p)[4].GetDouble());
      auto h2 = static_cast<float>((*p)[5].GetDouble());
      auto h3 = static_cast<float>((*p)[6].GetDouble());
      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createHexPyramidStackHalf(upperIdx1, upperIdx2, lowerIdx1,
                                           lowerIdx2, h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "CubicPyramid") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else if (p->Size() == 2) {
      auto h1 = static_cast<float>((*p)[0].GetDouble());
      auto h2 = static_cast<float>((*p)[1].GetDouble());
      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createCubicPyramid(h1, h2), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "IrregularHexCylinder") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
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
      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createIrregularHexPrism(dist, h), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    }
  } else if (c["type"] == "IrregularHexPyramid") {
    if (p == nullptr || !p->IsArray()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
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

      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createIrregularHexPyramid(dist, idx, height), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    } else {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
      throw std::invalid_argument(msgBuffer);
    }
  } else if (c["type"] == "Custom") {
    if (p == nullptr || !p->IsString()) {
      snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
      throw std::invalid_argument(msgBuffer);
    } else {
      char modelFileNameBuffer[512] = { 0 };
      auto n = configFileName.rfind('/');
      if (n == std::string::npos) {
        snprintf(modelFileNameBuffer, kMsgBufferSize, "models/%s", p->GetString());
      } else {
        snprintf(modelFileNameBuffer, kMsgBufferSize, "%s/models/%s", configFileName.substr(0, n).c_str(), p->GetString());
      }
      std::FILE* file = fopen(modelFileNameBuffer, "r");
      if (!file) {
        snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot open model file!", ci);
        throw std::invalid_argument(msgBuffer);
      }
      crystalCtxs.emplace_back(std::make_shared<CrystalContext>(
        parseCustomCrystal(file), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
      fclose(file);
    }
  } else {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  }
}


CrystalPtrU SimulationContext::parseCustomCrystal(std::FILE* file) {
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
  return Crystal::createCustomCrystal(vertexes, faces);
}


uint64_t SimulationContext::getTotalInitRays() const {
  return totalRayNum;
}


int SimulationContext::getMaxRecursionNum() const {
  return maxRecursionNum;
}


void SimulationContext::fillActiveCrystal(std::vector<CrystalContextPtr>* crystal_ctxs) const {
  crystal_ctxs->clear();
  for (const auto& ctx : crystalCtxs) {
    crystal_ctxs->emplace_back(ctx);
  }
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


const float* SimulationContext::getSunRayDir() const {
  return sunDir;
}


float SimulationContext::getSunDiameter() const {
  return sunDiameter;
}


std::string SimulationContext::getDataDirectory() const {
  return dataDirectory;
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
  for (const auto& c : crystalCtxs) {
    popWeightSum += c->population_;
  }
  for (auto& c : crystalCtxs) {
    c->population_ /= popWeightSum;
  }
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



RenderContext::RenderContext(rapidjson::Document& d) :
    imgHei(0), imgWid(0), offsetY(0), offsetX(0),
    visibleSemiSphere(VisibleSemiSphere::UPPER),
    totalW(0), intensityFactor(1.0), showHorizontal(true),
    dataDirectory("./"),
    projectionType(ProjectionType::EQUI_AREA) {
  parseCameraSettings(d);
  parseRenderSettings(d);
  parseDataSettings(d);
}


RenderContext::~RenderContext() {
  for (const auto& kv : spectrumData) {
    delete[] kv.second;
  }
}


std::unique_ptr<RenderContext> RenderContext::createFromFile(const char *filename) {
  printf("Reading config from: %s\n", filename);

  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    printf("ERROR: file %s cannot be open!\n", filename);
    return nullptr;
  }

  constexpr size_t kTmpBufferSize = 65536;
  char buffer[kTmpBufferSize];
  rapidjson::FileReadStream is(fp, buffer, sizeof(buffer));

  rapidjson::Document d;
  if (d.ParseStream(is).HasParseError()) {
    fprintf(stderr, "\nError(offset %u): %s\n", (unsigned)d.GetErrorOffset(),
            GetParseError_En(d.GetParseError()));
    fclose(fp);
    return nullptr;
  }

  fclose(fp);

  return std::unique_ptr<RenderContext>(new RenderContext(d));
}


void RenderContext::parseCameraSettings(rapidjson::Document& d) {
  camRot[0] = 90.0f;
  camRot[1] = 89.9f;
  camRot[2] = 0.0f;
  fov = 120.0f;
  imgWid = 800;
  imgHei = 800;
  projectionType = ProjectionType::EQUI_AREA;

  auto* p = Pointer("/camera/azimuth").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.azimuth>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.azimuth> is not a number, using default 90.0!\n");
  } else {
    auto az = static_cast<float>(p->GetDouble());
    az = std::max(std::min(az, 360.0f), 0.0f);
    camRot[0] = 90.0f - az;
  }

  p = Pointer("/camera/elevation").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.elevation>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.elevation> is not a number, using default 90.0!\n");
  } else {
    auto el = static_cast<float>(p->GetDouble());
    el = std::max(std::min(el, 89.999f), -89.999f);
    camRot[1] = el;
  }

  p = Pointer("/camera/rotation").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.rotation>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.rotation> is not a number, using default 0.0!\n");
  } else {
    auto rot = static_cast<float>(p->GetDouble());
    rot = std::max(std::min(rot, 180.0f), -180.0f);
    camRot[2] = rot;
  }

  p = Pointer("/camera/fov").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.fov>, using default 120.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.fov> is not a number, using default 120.0!\n");
  } else {
    fov = static_cast<float>(p->GetDouble());
    fov = std::max(std::min(fov, 140.0f), 0.0f);
  }

  p = Pointer("/camera/width").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.width>, using default 800!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <camera.width> is not an integer, using default 800!\n");
  } else {
    int width = p->GetInt();
    width = std::max(width, 0);
    imgWid = static_cast<uint32_t>(width);
  }

  p = Pointer("/camera/height").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.height>, using default 800!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <camera.height> is not an integer, using default 800!\n");
  } else {
    int height = p->GetInt();
    height = std::max(height, 0);
    imgHei = static_cast<uint32_t>(height);
  }

  p = Pointer("/camera/lens").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.lens>, using default equi-area fisheye!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! config <camera.lens> is not a string, using default equi-area fisheye!\n");
  } else {
    if (*p == "linear") {
      projectionType = ProjectionType::LINEAR;
    } else if (*p == "fisheye") {
      projectionType = ProjectionType::EQUI_AREA;
    } else if (*p == "dual_fisheye_equidistant") {
      projectionType = ProjectionType::DUAL_EQUI_DISTANT;
    } else if (*p == "dual_fisheye_equiarea") {
      projectionType = ProjectionType::DUAL_EQUI_AREA;
    } else {
      fprintf(stderr, "\nWARNING! config <camera.lens> cannot be recognized, using default equi-area fisheye!\n");
    }
  }
}


void RenderContext::parseRenderSettings(rapidjson::Document& d) {
  visibleSemiSphere = VisibleSemiSphere::UPPER;
  intensityFactor = 1.0;
  offsetY = 0;
  offsetX = 0;
  backgroundColor[0] = 0;
  backgroundColor[1] = 0;
  backgroundColor[2] = 0;
  rayColor[0] = -1;
  rayColor[1] = -1;
  rayColor[2] = -1;
  showHorizontal = true;

  auto* p = Pointer("/render/visible_semi_sphere").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.visible_semi_sphere>, using default UPPER!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> is not a string, using default UPPER!\n");
  } else if (*p == "upper") {
    visibleSemiSphere = VisibleSemiSphere::UPPER;
  } else if (*p == "lower") {
    visibleSemiSphere = VisibleSemiSphere::LOWER;
  } else if (*p == "camera") {
    visibleSemiSphere = VisibleSemiSphere::CAMERA;
  } else if (*p == "full") {
    visibleSemiSphere = VisibleSemiSphere::FULL;
  } else {
    fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> cannot be recognized, using default UPPER!\n");
  }

  p = Pointer("/render/intensity_factor").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.intensity_factor>, using default 1.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <render.intensity_factor> is not a number, using default 1.0!\n");
  } else {
    double f = p->GetDouble();
    f = std::max(std::min(f, 100.0), 0.01);
    intensityFactor = f;
  }

  p = Pointer("/render/offset").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.offset>, using default [0, 0]!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <render.offset> is not an array, using default [0, 0]!\n");
  } else if (p->Size() != 2 || !(*p)[0].IsInt() || !(*p)[1].IsInt()) {
    fprintf(stderr, "\nWARNING! Config <render.offset> cannot be recognized, using default [0, 0]!\n");
  } else {
    offsetX = (*p)[0].GetInt();
    offsetY = (*p)[1].GetInt();
    offsetX = std::max(std::min(offsetX, static_cast<int>(imgWid / 2)),
                       -static_cast<int>(imgWid / 2));
    offsetY = std::max(std::min(offsetY, static_cast<int>(imgHei / 2)),
                       -static_cast<int>(imgHei / 2));
  }

  p = Pointer("/render/background_color").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default [0,0,0]!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array, using default [0,0,0]!\n");
  } else if (p->Size() != 3 || !(*p)[0].IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default [0,0,0]!\n");
  } else {
    auto pa = p->GetArray();
    for (int i = 0; i < 3; i++) {
      backgroundColor[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
    }
  }

  p = Pointer("/render/ray_color").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default real color!\n");
  } else if (!p->IsArray() && !p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array nor a string, using default real color!\n");
  } else if (p->IsArray() && (p->Size() != 3 || !(*p)[0].IsNumber())) {
    fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default real color!\n");
  } else if (p->IsString() && (*p) != "real") {
    fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default real color!\n");
  } else if (p->IsArray()) {
    auto pa = p->GetArray();
    for (int i = 0; i < 3; i++) {
      rayColor[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
    }
  }

  p = Pointer("/render/show_horizontal").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.show_horizontal>, using default true!\n");
  } else if (!p->IsBool()) {
    fprintf(stderr, "\nWARNING! Config <render.show_horizontal> is not a boolean, using default true!\n");
  } else {
    showHorizontal = p->GetBool();
  }
}


void RenderContext::parseDataSettings(rapidjson::Document& d) {
  dataDirectory = "./";
  auto* p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default './'!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default './'!\n");
  } else {
    dataDirectory = p->GetString();
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

}   // namespace IceHalo

