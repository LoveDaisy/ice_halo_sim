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
    : crystal_(std::move(g)),
      axis_dist_(axisDist), roll_dist_(rollDist),
      axis_mean_(axisMean), roll_mean_(rollMean),
      axis_std_(axisStd), roll_std_(rollStd),
      population_(population) {}


CrystalPtr CrystalContext::GetCrystal() {
  return this->crystal_;
}


Math::Distribution CrystalContext::GetAxisDist() {
  return axis_dist_;
}


Math::Distribution CrystalContext::GetRollDist() {
  return roll_dist_;
}


float CrystalContext::GetAxisMean() {
  return axis_mean_;
}


float CrystalContext::GetRollMean() {
  return roll_mean_;
}


float CrystalContext::GetAxisStd() {
  return axis_std_;
}


float CrystalContext::GetRollStd() {
  return roll_std_;
}


float CrystalContext::GetPopulation() {
  return population_;
}


SimulationContext::SimulationContext(const char* filename, rapidjson::Document& d)
    : total_ray_num_(0), max_recursion_num_(9),
      multi_scatter_times_(1), multi_scatter_prob_(1.0f),
      current_wavelength_(550.0f), sun_diameter_(0.5f),
      config_file_name_(filename), data_directory_("./") {
  constexpr size_t kTmpBufferSize = 65536;
  char buffer[kTmpBufferSize];

  ParseRaySettings(d);
  ParseBasicSettings(d);
  ParseSunSettings(d);
  ParseDataSettings(d);
  ParseMultiScatterSettings(d);

  const auto* p = Pointer("/crystal").Get(d);
  if (p == nullptr || !p->IsArray()) {
    snprintf(buffer, kTmpBufferSize, "Missing <crystal>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    ParseCrystalSettings(c, ci);
    ci++;
  }
}


std::unique_ptr<SimulationContext> SimulationContext::CreateFromFile(const char* filename) {
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


void SimulationContext::ParseBasicSettings(rapidjson::Document& d) {
  int maxRecursion = 9;
  auto* p = Pointer("/max_recursion").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <max_recursion>, using default 9!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <max_recursion> is not a integer, using default 9!\n");
  } else {
    maxRecursion = std::min(std::max(p->GetInt(), 1), 10);
  }
  max_recursion_num_ = maxRecursion;
}


void SimulationContext::ParseRaySettings(rapidjson::Document& d) {
  /* Parsing ray number */
  total_ray_num_ = 10000;
  auto* p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <ray.number>, using default 10000!\n");
  } else if (!p->IsUint()) {
    fprintf(stderr, "\nWARNING! Config <ray.number> is not unsigned int, using default 10000!\n");
  } else {
    total_ray_num_ = p->GetUint();
  }

  /* Parsing wavelengths */
  wavelengths_.clear();
  wavelengths_.push_back(550);
  p = Pointer("/ray/wavelength").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <ray.wavelength>, using default 550!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <ray.wavelength> is not an array, using default 550!\n");
  } else if (!(*p)[0].IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <ray.wavelength> connot be recognized, using default 550!\n");
  } else {
    wavelengths_.clear();
    for (const auto& pi : p->GetArray()) {
      wavelengths_.push_back(static_cast<float &&>(pi.GetDouble()));
    }
  }
}


void SimulationContext::ParseSunSettings(rapidjson::Document& d) {
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
  SetSunPosition(90.0f, sunAltitude);

  // Parsing sun diameter
  sun_diameter_ = 0.5f;
  p = Pointer("/sun/diameter").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <sun.diameter>, using default 0.5!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! Config <sun.diameter> is not a number, using default 0.5!\n");
  } else {
    sun_diameter_ = static_cast<float>(p->GetDouble());
  }
}


void SimulationContext::ParseDataSettings(rapidjson::Document& d) {
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
  data_directory_ = dir;
}


void SimulationContext::ParseMultiScatterSettings(rapidjson::Document& d) {
  int multiScattering = 1;
  auto* p = Pointer("/multi_scatter/repeat").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <multi_scatter.repeat>, using default value 1!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! Config <multi_scatter.repeat> is not a integer, using default 1!\n");
  } else {
    multiScattering = std::min(std::max(p->GetInt(), 1), 4);
  }
  multi_scatter_times_ = multiScattering;

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
  multi_scatter_prob_ = prob;
}


void SimulationContext::ParseCrystalSettings(const rapidjson::Value& c, int ci) {
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
    ParseCrystalType(c, ci, population, axisDist, axisMean, axisStd, rollDist, rollMean, rollStd);
  }
}


void SimulationContext::ParseCrystalType(const rapidjson::Value& c, int ci,
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
    crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
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
      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
        Crystal::createHexPyramid(h1, h2, h3), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
    } else if (p->Size() == 5) {
      int i1 = (*p)[0].GetInt();
      int i2 = (*p)[1].GetInt();
      auto h1 = static_cast<float>((*p)[2].GetDouble());
      auto h2 = static_cast<float>((*p)[3].GetDouble());
      auto h3 = static_cast<float>((*p)[4].GetDouble());
      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
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
      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
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
      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
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
      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
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
      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
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

      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
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
      auto n = config_file_name_.rfind('/');
      if (n == std::string::npos) {
        snprintf(modelFileNameBuffer, kMsgBufferSize, "models/%s", p->GetString());
      } else {
        snprintf(modelFileNameBuffer, kMsgBufferSize, "%s/models/%s", config_file_name_.substr(0, n).c_str(), p->GetString());
      }
      std::FILE* file = fopen(modelFileNameBuffer, "r");
      if (!file) {
        snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].parameter> cannot open model file!", ci);
        throw std::invalid_argument(msgBuffer);
      }
      crystal_ctxs_.emplace_back(std::make_shared<CrystalContext>(
        ParseCustomCrystal(file), population,
        axisDist, axisMean, axisStd,
        rollDist, rollMean, rollStd));
      fclose(file);
    }
  } else {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  }
}


CrystalPtrU SimulationContext::ParseCustomCrystal(std::FILE* file) {
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


uint64_t SimulationContext::GetTotalInitRays() const {
  return total_ray_num_;
}


int SimulationContext::GetMaxRecursionNum() const {
  return max_recursion_num_;
}


void SimulationContext::FillActiveCrystal(std::vector<CrystalContextPtr>* crystal_ctxs) const {
  crystal_ctxs->clear();
  for (const auto& ctx : crystal_ctxs_) {
    crystal_ctxs->emplace_back(ctx);
  }
}


int SimulationContext::GetMultiScatterTimes() const {
  return multi_scatter_times_;
}

float SimulationContext::GetMultiScatterProb() const {
  return multi_scatter_prob_;
}


void SimulationContext::SetCurrentWavelength(float wavelength) {
  this->current_wavelength_ = wavelength;
}


float SimulationContext::GetCurrentWavelength() const {
  return current_wavelength_;
}


std::vector<float> SimulationContext::GetWavelengths() const {
  return wavelengths_;
}


const float* SimulationContext::GetSunRayDir() const {
  return sun_ray_dir_;
}


float SimulationContext::GetSunDiameter() const {
  return sun_diameter_;
}


std::string SimulationContext::GetDataDirectory() const {
  return data_directory_;
}


void SimulationContext::SetSunPosition(float lon, float lat) {
  float x = -std::cos(lat * Math::kPi / 180.0f) * std::cos(lon * Math::kPi / 180.0f);
  float y = -std::cos(lat * Math::kPi / 180.0f) * std::sin(lon * Math::kPi / 180.0f);
  float z = -std::sin(lat * Math::kPi / 180.0f);

  this->sun_ray_dir_[0] = x;
  this->sun_ray_dir_[1] = y;
  this->sun_ray_dir_[2] = z;
}


void SimulationContext::ApplySettings() {
  if (total_ray_num_ <= 0) return;

  float popWeightSum = 0.0f;
  for (const auto& c : crystal_ctxs_) {
    popWeightSum += c->population_;
  }
  for (auto& c : crystal_ctxs_) {
    c->population_ /= popWeightSum;
  }
}


void SimulationContext::PrintCrystalInfo() {
  for (const auto& c : crystal_ctxs_) {
    auto g = c->GetCrystal();
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
    img_hei_(0), img_wid_(0), offset_y_(0), offset_x_(0),
    visible_semi_sphere_(VisibleSemiSphere::UPPER),
    total_w_(0), intensity_factor_(1.0), show_horizontal_(true),
    data_directory_("./"),
    projection_type_(ProjectionType::EQUI_AREA) {
  ParseCameraSettings(d);
  ParseRenderSettings(d);
  ParseDataSettings(d);
}


RenderContext::~RenderContext() {
  for (const auto& kv : spectrum_data_) {
    delete[] kv.second;
  }
}


std::unique_ptr<RenderContext> RenderContext::CreateFromFile(const char* filename) {
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


void RenderContext::ParseCameraSettings(rapidjson::Document& d) {
  cam_rot_[0] = 90.0f;
  cam_rot_[1] = 89.9f;
  cam_rot_[2] = 0.0f;
  fov_ = 120.0f;
  img_wid_ = 800;
  img_hei_ = 800;
  projection_type_ = ProjectionType::EQUI_AREA;

  auto* p = Pointer("/camera/azimuth").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.azimuth>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.azimuth> is not a number, using default 90.0!\n");
  } else {
    auto az = static_cast<float>(p->GetDouble());
    az = std::max(std::min(az, 360.0f), 0.0f);
    cam_rot_[0] = 90.0f - az;
  }

  p = Pointer("/camera/elevation").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.elevation>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.elevation> is not a number, using default 90.0!\n");
  } else {
    auto el = static_cast<float>(p->GetDouble());
    el = std::max(std::min(el, 89.999f), -89.999f);
    cam_rot_[1] = el;
  }

  p = Pointer("/camera/rotation").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.rotation>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.rotation> is not a number, using default 0.0!\n");
  } else {
    auto rot = static_cast<float>(p->GetDouble());
    rot = std::max(std::min(rot, 180.0f), -180.0f);
    cam_rot_[2] = rot;
  }

  p = Pointer("/camera/fov").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.fov>, using default 120.0!\n");
  } else if (!p->IsNumber()) {
    fprintf(stderr, "\nWARNING! config <camera.fov> is not a number, using default 120.0!\n");
  } else {
    fov_ = static_cast<float>(p->GetDouble());
    fov_ = std::max(std::min(fov_, 140.0f), 0.0f);
  }

  p = Pointer("/camera/width").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.width>, using default 800!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <camera.width> is not an integer, using default 800!\n");
  } else {
    int width = p->GetInt();
    width = std::max(width, 0);
    img_wid_ = static_cast<uint32_t>(width);
  }

  p = Pointer("/camera/height").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.height>, using default 800!\n");
  } else if (!p->IsInt()) {
    fprintf(stderr, "\nWARNING! config <camera.height> is not an integer, using default 800!\n");
  } else {
    int height = p->GetInt();
    height = std::max(height, 0);
    img_hei_ = static_cast<uint32_t>(height);
  }

  p = Pointer("/camera/lens").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <camera.lens>, using default equi-area fisheye!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! config <camera.lens> is not a string, using default equi-area fisheye!\n");
  } else {
    if (*p == "linear") {
      projection_type_ = ProjectionType::LINEAR;
    } else if (*p == "fisheye") {
      projection_type_ = ProjectionType::EQUI_AREA;
    } else if (*p == "dual_fisheye_equidistant") {
      projection_type_ = ProjectionType::DUAL_EQUI_DISTANT;
    } else if (*p == "dual_fisheye_equiarea") {
      projection_type_ = ProjectionType::DUAL_EQUI_AREA;
    } else {
      fprintf(stderr, "\nWARNING! config <camera.lens> cannot be recognized, using default equi-area fisheye!\n");
    }
  }
}


void RenderContext::ParseRenderSettings(rapidjson::Document& d) {
  visible_semi_sphere_ = VisibleSemiSphere::UPPER;
  intensity_factor_ = 1.0;
  offset_y_ = 0;
  offset_x_ = 0;
  background_color_[0] = 0;
  background_color_[1] = 0;
  background_color_[2] = 0;
  ray_color_[0] = -1;
  ray_color_[1] = -1;
  ray_color_[2] = -1;
  show_horizontal_ = true;

  auto* p = Pointer("/render/visible_semi_sphere").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.visible_semi_sphere>, using default UPPER!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> is not a string, using default UPPER!\n");
  } else if (*p == "upper") {
    visible_semi_sphere_ = VisibleSemiSphere::UPPER;
  } else if (*p == "lower") {
    visible_semi_sphere_ = VisibleSemiSphere::LOWER;
  } else if (*p == "camera") {
    visible_semi_sphere_ = VisibleSemiSphere::CAMERA;
  } else if (*p == "full") {
    visible_semi_sphere_ = VisibleSemiSphere::FULL;
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
    intensity_factor_ = f;
  }

  p = Pointer("/render/offset").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.offset>, using default [0, 0]!\n");
  } else if (!p->IsArray()) {
    fprintf(stderr, "\nWARNING! Config <render.offset> is not an array, using default [0, 0]!\n");
  } else if (p->Size() != 2 || !(*p)[0].IsInt() || !(*p)[1].IsInt()) {
    fprintf(stderr, "\nWARNING! Config <render.offset> cannot be recognized, using default [0, 0]!\n");
  } else {
    offset_x_ = (*p)[0].GetInt();
    offset_y_ = (*p)[1].GetInt();
    offset_x_ = std::max(std::min(offset_x_, static_cast<int>(img_wid_ / 2)),
                       -static_cast<int>(img_wid_ / 2));
    offset_y_ = std::max(std::min(offset_y_, static_cast<int>(img_hei_ / 2)),
                       -static_cast<int>(img_hei_ / 2));
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
      background_color_[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
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
      ray_color_[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
    }
  }

  p = Pointer("/render/show_horizontal").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <render.show_horizontal>, using default true!\n");
  } else if (!p->IsBool()) {
    fprintf(stderr, "\nWARNING! Config <render.show_horizontal> is not a boolean, using default true!\n");
  } else {
    show_horizontal_ = p->GetBool();
  }
}


void RenderContext::ParseDataSettings(rapidjson::Document& d) {
  data_directory_ = "./";
  auto* p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default './'!\n");
  } else if (!p->IsString()) {
    fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default './'!\n");
  } else {
    data_directory_ = p->GetString();
  }
}


uint32_t RenderContext::GetImageWidth() const {
  return img_wid_;
}


uint32_t RenderContext::GetImageHeight() const {
  return img_hei_;
}


std::string RenderContext::GetImagePath() const {
  return pathJoin(data_directory_, "img.jpg");
}


void RenderContext::RenderToRgb(uint8_t* rgbData) {
  auto wlNum = spectrum_data_.size();
  auto* wlData = new float[wlNum];
  auto* flatSpecData = new float[wlNum * img_wid_ * img_hei_];

  CopySpectrumData(wlData, flatSpecData);
  if (ray_color_[0] < 0) {
    render.rgb(static_cast<int>(wlNum), wlData, img_wid_ * img_hei_, flatSpecData, rgbData);
  } else {
    render.gray(static_cast<int>(wlNum), wlData, img_wid_ * img_hei_, flatSpecData, rgbData);
  }
  for (size_t i = 0; i < img_wid_ * img_hei_; i++) {
    for (int c = 0; c <= 2; c++) {
      auto v = static_cast<int>(background_color_[c] * 255);
      if (ray_color_[0] < 0) {
        v += rgbData[i * 3 + c];
      } else {
        v += static_cast<int>(rgbData[i * 3 + c] * ray_color_[c]);
      }
      v = std::max(std::min(v, 255), 0);
      rgbData[i * 3 + c] = static_cast<uint8_t>(v);
    }
  }

  /* Draw horizontal */
  // float imgR = std::min(img_wid_ / 2, img_hei_) / 2.0f;
  // TODO

  delete[] wlData;
  delete[] flatSpecData;
}


void RenderContext::CopySpectrumData(float* wavelengthData, float* spectrumData) const {
  int k = 0;
  for (const auto& kv : this->spectrum_data_) {
    wavelengthData[k] = kv.first;
    std::memcpy(spectrumData + k * img_wid_ * img_hei_, kv.second, img_wid_ * img_hei_ * sizeof(float));
    k++;
  }
  for (uint64_t i = 0; i < img_wid_ * img_hei_ * this->spectrum_data_.size(); i++) {
    spectrumData[i] *= 8 * 4e3 / total_w_ * intensity_factor_;
  }
}


void RenderContext::LoadData() {
  std::vector<File> files = listDataFiles(data_directory_.c_str());
  int i = 0;
  for (auto& f : files) {
    auto t0 = std::chrono::system_clock::now();
    auto num = LoadDataFromFile(f);
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::ratio<1, 1000> > diff = t1 - t0;
    printf(" Loading data (%d/%lu): %.2fms; total %d pts\n", i + 1, files.size(), diff.count(), num);
    i++;
  }
}


int RenderContext::LoadDataFromFile(File& file) {
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
    total_w_ += tmpW[i];
  }
  delete[] readBuffer;

  auto* tmpXY = new int[totalCount * 2];
  projectionFunctions[projection_type_](cam_rot_, fov_, totalCount, tmpDir, img_wid_, img_hei_, tmpXY, visible_semi_sphere_);
  delete[] tmpDir;

  float* currentData = nullptr;
  auto it = spectrum_data_.find(wavelength);
  if (it != spectrum_data_.end()) {
    currentData = it->second;
  } else {
    currentData = new float[img_hei_ * img_wid_];
    for (decltype(img_hei_) i = 0; i < img_hei_ * img_wid_; i++) {
      currentData[i] = 0;
    }
    spectrum_data_[wavelength] = currentData;
  }

  for (decltype(totalCount) i = 0; i < totalCount; i++) {
    int x = tmpXY[i * 2 + 0];
    int y = tmpXY[i * 2 + 1];
    if (x == std::numeric_limits<int>::min() || y == std::numeric_limits<int>::min()) {
      continue;
    }
    if (projection_type_ != ProjectionType::DUAL_EQUI_AREA && projection_type_ != ProjectionType::DUAL_EQUI_DISTANT) {
      x += offset_x_;
      y += offset_y_;
    }
    if (x < 0 || x >= static_cast<int>(img_wid_) || y < 0 || y >= static_cast<int>(img_hei_)) {
      continue;
    }
    currentData[y * img_wid_ + x] += tmpW[i];
  }
  delete[] tmpXY;
  delete[] tmpW;

  return static_cast<int>(totalCount);
}

}   // namespace IceHalo

