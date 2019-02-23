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

RayPathFilter::RayPathFilter() : symmetry_(RayPathFilter::kSymmetryNone), type_(RayPathFilter::kTypeNone) {}


CrystalContext::CrystalContext(CrystalPtrU&& g, const AxisDistribution& axis,
                               const RayPathFilter& filter, float population)
    : crystal_(std::move(g)), axis_(axis), ray_path_filter_(filter), population_(population) {}


CrystalPtr CrystalContext::GetCrystal() {
  return this->crystal_;
}


Math::Distribution CrystalContext::GetAxisDist() const {
  return axis_.axis_dist;
}


Math::Distribution CrystalContext::GetRollDist() const {
  return axis_.roll_dist;
}


float CrystalContext::GetAxisMean() const {
  return axis_.axis_mean;
}


float CrystalContext::GetRollMean() const {
  return axis_.roll_mean;
}


float CrystalContext::GetAxisStd() const {
  return axis_.axis_std;
}


float CrystalContext::GetRollStd() const {
  return axis_.roll_std;
}


float CrystalContext::GetPopulation() const {
  return population_;
}


void CrystalContext::SetPopulation(float population) {
  population_ = population;
}


bool CrystalContext::FilterRay(RaySegment* last_r) {
  switch (ray_path_filter_.type_) {
    case RayPathFilter::kTypeNone:
      return true;
    case RayPathFilter::kTypeGeneral:
      return FilterRayGeneral(last_r);
    case RayPathFilter::kTypeSpecific:
      return FilterRaySpecific(last_r);
    default:
      return false;
  }
}


bool CrystalContext::FilterRaySpecific(IceHalo::RaySegment* last_r) {
  if (ray_path_filter_.ray_path_.empty()) {
    return true;
  }

  int curr_fn0 = crystal_->FaceNumber(last_r->root_->first_ray_segment_->face_id_);
  if (curr_fn0 < 0) {
    // Do not have a face number mapping.
    return true;
  }

  return FilterRayDirectionalSymm(last_r, true) || FilterRayDirectionalSymm(last_r, false);
}


bool CrystalContext::FilterRayDirectionalSymm(RaySegment* last_r, bool original) {
  int period = crystal_->GetFaceNumberPeriod();
  if (period <= 0) {
    return true;
  }

  int prism_diff = -1;
  int basal_diff = -1;
  auto p = last_r;
  for (auto rit = ray_path_filter_.ray_path_.rbegin();
       rit != ray_path_filter_.ray_path_.rend();
       p = p->prev_, ++rit) {
    if (!p) {
      // Ray path shorter than filter path
      return false;
    }
    int filter_fn = *rit;
    int curr_fn = crystal_->FaceNumber(p->face_id_);
    if (!original && curr_fn != 1 && curr_fn != 2) {
      curr_fn = 5 + period - curr_fn;
    }
    bool filter_basal = filter_fn == 1 || filter_fn == 2;
    bool curr_basal = curr_fn == 1 || curr_fn == 2;
    if (filter_basal != curr_basal) {
      return false;
    }
    if (!filter_basal) {
      prism_diff = (curr_fn - filter_fn + period) % period;
    } else {
      basal_diff = std::abs(curr_fn - filter_fn);
    }
  }
  if (!p || p->prev_) {
    // Ray path shorter or longer than filter path.
    return false;
  }

  p = last_r;
  for (auto rit = ray_path_filter_.ray_path_.rbegin();
       rit != ray_path_filter_.ray_path_.rend();
       p = p->prev_, ++rit) {
    int filter_fn = *rit;
    int curr_fn = crystal_->FaceNumber(p->face_id_);
    if (!original && curr_fn != 1 && curr_fn != 2) {
      curr_fn = 5 + period - curr_fn;
    }

    bool filter_basal = filter_fn == 1 || filter_fn == 2;
    int curr_fn_p = curr_fn % 10;
    int curr_fn_s = curr_fn / 10;
    bool curr_basal = curr_fn == 1 || curr_fn == 2;

    if (prism_diff > 0 && ray_path_filter_.symmetry_ & RayPathFilter::kSymmetryPrism && !curr_basal && !filter_basal) {
      curr_fn_p = (curr_fn_p - prism_diff + period) % period;
      curr_fn = curr_fn_p + curr_fn_s * 10;
    }
    if (curr_fn == filter_fn) {
      // Exactly match. Continue to check next.
      continue;
    }

    if (ray_path_filter_.symmetry_ & RayPathFilter::kSymmetryBasal) {
      // B symmetry is checked;
      if (basal_diff > 0 && filter_basal && curr_basal) {
        continue;
      }
    }
    return false;
  }

  return p && p == p->root_->first_ray_segment_;
}


bool CrystalContext::FilterRayGeneral(RaySegment* last_r) {
  if (ray_path_filter_.entry_.empty() && ray_path_filter_.exit_.empty()) {
    return true;
  }

  int fn0 = crystal_->FaceNumber(last_r->root_->first_ray_segment_->face_id_);
  if (fn0 < 0 || crystal_->GetFaceNumberPeriod() < 0) {
    // Do not have a face number mapping.
    return true;
  }

  bool matched = false;
  for (const auto& fn : ray_path_filter_.entry_) {
    if (fn0 == fn) {
      matched = true;
      break;
    }
  }
  if (!matched) {
    return false;
  }

  fn0 = crystal_->FaceNumber(last_r->face_id_);
  matched = false;
  for (const auto& fn : ray_path_filter_.exit_) {
    if (fn0 == fn) {
      matched = true;
      break;
    }
  }
  return matched;
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

  ApplySettings();
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
  SetSunRayDirection(90.0f, sunAltitude);

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


std::unordered_map<std::string, SimulationContext::CrystalParser>
  SimulationContext::crystal_parser_ = {
  { "HexPrism", &SimulationContext::ParseCrystalHexPrism },
  { "HexPyramid", &SimulationContext::ParseCrystalHexPyramid },
  { "HexPyramidStackHalf", &SimulationContext::ParseCrystalHexPyramidStackHalf },
  { "CubicPyramid", &SimulationContext::ParseCrystalCubicPyramid },
  { "IrregularHexPrism", &SimulationContext::ParseCrystalIrregularHexPrism },
  { "IrregularHexPyramid", &SimulationContext::ParseCrystalIrregularHexPyramid },
  { "Custom", &SimulationContext::ParseCrystalCustom },
};


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

  float population = 1.0;
  p = Pointer("/population").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    fprintf(stderr, "\nWARNING! <crystal[%d].population> cannot recognize, using default 1.0!\n", ci);
  } else {
    population = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/type").Get(c);
  std::string type(c["type"].GetString());
  if (p == nullptr || !p->IsString()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  }
  if (crystal_parser_.find(type) == crystal_parser_.end()) {
    snprintf(msgBuffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msgBuffer);
  }
  crystal_ctx_.emplace_back(std::make_shared<CrystalContext>(
    crystal_parser_[type](this, c, ci), ParseCrystalAxis(c, ci), ParseCrystalRayPathFilter(c, ci), population));
}


AxisDistribution SimulationContext::ParseCrystalAxis(const rapidjson::Value& c, int ci) {
  using Math::Distribution;

  AxisDistribution axis{};
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  const auto* p = Pointer("/axis/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].axis.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (*p == "Gauss") {
    axis.axis_dist = Distribution::GAUSS;
  } else if (*p == "Uniform") {
    axis.axis_dist = Distribution::UNIFORM;
  } else {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].axis.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/roll/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (*p == "Gauss") {
    axis.roll_dist = Distribution::GAUSS;
  } else if (*p == "Uniform") {
    axis.roll_dist = Distribution::UNIFORM;
  } else {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/axis/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].axis.mean> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.axis_mean = static_cast<float>(90 - p->GetDouble());
  }

  p = Pointer("/axis/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].axis.std> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.axis_std = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.mean> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.roll_mean = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.std> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.roll_std = static_cast<float>(p->GetDouble());
  }

  return axis;
}


RayPathFilter SimulationContext::ParseCrystalRayPathFilter(const rapidjson::Value& c, int ci) {
  RayPathFilter filter{};

  const auto* p = Pointer("/ray_path_filter").Get(c);
  if (p == nullptr || !p->IsObject()) {
    fprintf(stderr, "<crystal[%d].ray_path_filter> is not specified. Use default none.\n", ci);
    return filter;
  }

  p = Pointer("/ray_path_filter/symmetry").Get(c);
  if (p == nullptr || !p->IsString()) {
    fprintf(stderr, "<crystal[%d].ray_path_filter.symmetry> cannot recognize! Use default none.\n", ci);
  } else {
    const auto str = p->GetString();
    for (decltype(p->GetStringLength()) i = 0; i < p->GetStringLength(); i++) {
      switch (str[i]) {
        case 'P':
        case 'p':
          filter.symmetry_ |= RayPathFilter::kSymmetryPrism;
          break;
        case 'B':
        case 'b':
          filter.symmetry_ |= RayPathFilter::kSymmetryBasal;
          break;
        case 'D':
        case 'd':
          filter.symmetry_ |= RayPathFilter::kSymmetryDirection;
          break;
        default:
          fprintf(stderr, "<crystal[%d].ray_path_filter.symmetry> some item cannot be recognized! ignored.\n", ci);
          break;
      }
    }
  }

  p = Pointer("/ray_path_filter/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    fprintf(stderr, "<crystal[%d].ray_path_filter.type> cannot recognize! Use default none.\n", ci);
  } else {
    if (*p == "Specific") {
      filter.type_ = RayPathFilter::kTypeSpecific;
    } else if (*p == "General") {
      filter.type_ = RayPathFilter::kTypeGeneral;
    } else if (*p == "None") {
      filter.type_ = RayPathFilter::kTypeNone;
    } else {
      fprintf(stderr, "<crystal[%d].ray_path_filter.type> cannot recognize! Use default none.\n", ci);
    }
  }

  p = Pointer("/ray_path_filter/path").Get(c);
  if (p == nullptr || !p->IsArray() || !((*p)[0].IsInt())) {
    fprintf(stderr, "<crystal[%d].ray_path_filter.path> cannot recognize! Use default [].\n", ci);
  } else {
    for (const auto& fn : p->GetArray()) {
      filter.ray_path_.emplace_back(fn.GetInt());
    }
  }

  p = Pointer("/ray_path_filter/entry").Get(c);
  if (p == nullptr || !p->IsArray() || !((*p)[0].IsInt())) {
    fprintf(stderr, "<crystal[%d].ray_path_filter.entry> cannot recognize! Use default [].\n", ci);
  } else {
    for (const auto& fn : p->GetArray()) {
      filter.entry_.emplace_back(fn.GetInt());
    }
  }

  p = Pointer("/ray_path_filter/exit").Get(c);
  if (p == nullptr || !p->IsArray() || !((*p)[0].IsInt())) {
    fprintf(stderr, "<crystal[%d].ray_path_filter.exit> cannot recognize! Use default [].\n", ci);
  } else {
    for (const auto& fn : p->GetArray()) {
      filter.exit_.emplace_back(fn.GetInt());
    }
  }

  return filter;
}


CrystalPtrU SimulationContext::ParseCrystalHexPrism(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  auto h = static_cast<float>(p->GetDouble());
  return Crystal::CreateHexPrism(h);
}


CrystalPtrU SimulationContext::ParseCrystalHexPyramid(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (p->Size() == 3) {
    auto h1 = static_cast<float>((*p)[0].GetDouble());
    auto h2 = static_cast<float>((*p)[1].GetDouble());
    auto h3 = static_cast<float>((*p)[2].GetDouble());
    return Crystal::CreateHexPyramid(h1, h2, h3);
  } else if (p->Size() == 5) {
    int i1 = (*p)[0].GetInt();
    int i2 = (*p)[1].GetInt();
    auto h1 = static_cast<float>((*p)[2].GetDouble());
    auto h2 = static_cast<float>((*p)[3].GetDouble());
    auto h3 = static_cast<float>((*p)[4].GetDouble());
    return Crystal::CreateHexPyramid(i1, i2, h1, h2, h3);
  } else if (p->Size() == 7) {
    int upper_idx1 = (*p)[0].GetInt();
    int upper_idx2 = (*p)[1].GetInt();
    int lower_idx1 = (*p)[2].GetInt();
    int lower_idx2 = (*p)[3].GetInt();
    auto h1 = static_cast<float>((*p)[4].GetDouble());
    auto h2 = static_cast<float>((*p)[5].GetDouble());
    auto h3 = static_cast<float>((*p)[6].GetDouble());
    return Crystal::CreateHexPyramid(upper_idx1, upper_idx2, lower_idx1, lower_idx2, h1, h2, h3);
  } else {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalHexPyramidStackHalf(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (p->Size() == 7) {
    int upper_idx1 = (*p)[0].GetInt();
    int upper_idx2 = (*p)[1].GetInt();
    int lower_idx1 = (*p)[2].GetInt();
    int lower_idx2 = (*p)[3].GetInt();
    auto h1 = static_cast<float>((*p)[4].GetDouble());
    auto h2 = static_cast<float>((*p)[5].GetDouble());
    auto h3 = static_cast<float>((*p)[6].GetDouble());
    return Crystal::CreateHexPyramidStackHalf(upper_idx1, upper_idx2, lower_idx1, lower_idx2, h1, h2, h3);
  } else {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalCubicPyramid(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (p->Size() == 2) {
    auto h1 = static_cast<float>((*p)[0].GetDouble());
    auto h2 = static_cast<float>((*p)[1].GetDouble());
    return Crystal::CreateCubicPyramid(h1, h2);
  } else {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalIrregularHexPrism(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray() || p->Size() != 7) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    auto d1 = static_cast<float>((*p)[0].GetDouble());
    auto d2 = static_cast<float>((*p)[1].GetDouble());
    auto d3 = static_cast<float>((*p)[2].GetDouble());
    auto d4 = static_cast<float>((*p)[3].GetDouble());
    auto d5 = static_cast<float>((*p)[4].GetDouble());
    auto d6 = static_cast<float>((*p)[5].GetDouble());
    auto h = static_cast<float>((*p)[6].GetDouble());

    float dist[6] = { d1, d2, d3, d4, d5, d6 };
    return Crystal::CreateIrregularHexPrism(dist, h);
  }
}


CrystalPtrU SimulationContext::ParseCrystalIrregularHexPyramid(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
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

    return Crystal::CreateIrregularHexPyramid(dist, idx, height);
  } else {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalCustom(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsString()) {
    snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    auto n = config_file_name_.rfind('/');
    if (n == std::string::npos) {
      snprintf(msg_buffer, kMsgBufferSize, "models/%s", p->GetString());
    } else {
      snprintf(msg_buffer, kMsgBufferSize, "%s/models/%s", config_file_name_.substr(0, n).c_str(), p->GetString());
    }
    std::FILE* file = fopen(msg_buffer, "r");
    if (!file) {
      snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot open model file!", ci);
      throw std::invalid_argument(msg_buffer);
    }

    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    float v_buf[3];
    int f_buf[3];
    int curr_char;
    while ((curr_char = std::fgetc(file)) != EOF) {
      switch (curr_char) {
        case 'v':
        case 'V':
          std::fscanf(file, "%f %f %f", v_buf+0, v_buf+1, v_buf+2);
          vertexes.emplace_back(v_buf);
          break;
        case 'f':
        case 'F':
          std::fscanf(file, "%d %d %d", f_buf+0, f_buf+1, f_buf+2);
          faces.emplace_back(f_buf[0]-1, f_buf[1]-1, f_buf[2]-1);
          break;
        default:
          break;
      }
    }
    fclose(file);

    return Crystal::CreateCustomCrystal(vertexes, faces);
  }
}


uint64_t SimulationContext::GetTotalInitRays() const {
  return total_ray_num_;
}


int SimulationContext::GetMaxRecursionNum() const {
  return max_recursion_num_;
}


void SimulationContext::FillActiveCrystal(std::vector<CrystalContextPtr>* crystal_ctxs) const {
  crystal_ctxs->clear();
  for (const auto& ctx : crystal_ctx_) {
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


void SimulationContext::SetSunRayDirection(float lon, float lat) {
  float x = -std::cos(lat * Math::kPi / 180.0f) * std::cos(lon * Math::kPi / 180.0f);
  float y = -std::cos(lat * Math::kPi / 180.0f) * std::sin(lon * Math::kPi / 180.0f);
  float z = -std::sin(lat * Math::kPi / 180.0f);

  this->sun_ray_dir_[0] = x;
  this->sun_ray_dir_[1] = y;
  this->sun_ray_dir_[2] = z;
}


void SimulationContext::ApplySettings() {
  if (total_ray_num_ <= 0) return;

  float pop_weight_sum = 0.0f;
  for (const auto& c : crystal_ctx_) {
    pop_weight_sum += c->GetPopulation();
  }
  for (auto& c : crystal_ctx_) {
    c->SetPopulation(c->GetPopulation() / pop_weight_sum);
  }
}


void SimulationContext::PrintCrystalInfo() {
  for (const auto& c : crystal_ctx_) {
    auto g = c->GetCrystal();
    printf("--\n");
    for (const auto& v : g->getVertexes()) {
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
    total_ray_num_(0), total_w_(0), intensity_factor_(1.0), show_horizontal_(true),
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

  p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    fprintf(stderr, "\nWARNING! Config missing <ray.number>, using default 10000!\n");
  } else if (!p->IsUint()) {
    fprintf(stderr, "\nWARNING! Config <ray.number> is not unsigned int, using default 10000!\n");
  } else {
    total_ray_num_ = p->GetUint();
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
  return PathJoin(data_directory_, "img.jpg");
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
    spectrumData[i] *= 2e4 / total_w_ * intensity_factor_;
  }
}


void RenderContext::LoadData() {
  std::vector<File> files = ListDataFiles(data_directory_.c_str());
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
  auto fileSize = file.GetSize();
  auto* readBuffer = new float[fileSize / sizeof(float)];

  file.Open(OpenMode::kRead | OpenMode::kBinary);
  auto readCount = file.Read(readBuffer, 1);
  if (readCount <= 0) {
    return -1;
  }

  auto wavelength = static_cast<int>(readBuffer[0]);
  if (wavelength < SpectrumRenderer::MIN_WL || wavelength > SpectrumRenderer::MAX_WL) {
    return -1;
  }

  readCount = file.Read(readBuffer, fileSize / sizeof(float));
  auto totalCount = readCount / 4;
  file.Close();

  if (totalCount == 0) {
    return static_cast<int>(totalCount);
  }

  auto* tmpDir = new float[totalCount * 3];
  auto* tmpW = new float[totalCount];
  for (decltype(readCount) i = 0; i < totalCount; i++) {
    std::memcpy(tmpDir + i * 3, readBuffer + i * 4, 3 * sizeof(float));
    tmpW[i] = readBuffer[i * 4 + 3];
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

  total_w_ += total_ray_num_;
  return static_cast<int>(totalCount);
}

}   // namespace IceHalo

