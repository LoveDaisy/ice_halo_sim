#include <utility>

#include "context.h"
#include "optics.h"
#include "threadingpool.h"
#include "render.h"

#include "rapidjson/pointer.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"

#include <utility>
#include <limits>
#include <algorithm>


namespace IceHalo {

using rapidjson::Pointer;

RayPathFilter::RayPathFilter()
    : type(RayPathFilter::kTypeNone), symmetry(RayPathFilter::kSymmetryNone) {}


bool RayPathFilter::Filter(IceHalo::RaySegment* r, const CrystalPtr& crystal) const {
  switch (type) {
    case RayPathFilter::kTypeNone:
      return true;
    case RayPathFilter::kTypeGeneral:
      return FilterRayGeneral(r, crystal);
    case RayPathFilter::kTypeSpecific:
      return FilterRaySpecific(r, crystal);
    default:
      return false;
  }
}


bool RayPathFilter::FilterRaySpecific(IceHalo::RaySegment* last_r, const CrystalPtr& crystal) const {
  if (ray_path_hashes.empty()) {
    return true;
  }

  int curr_fn0 = crystal->FaceNumber(last_r->root_ctx->first_ray_segment->face_id);
  if (curr_fn0 < 0 || crystal->GetFaceNumberPeriod() < 0) {   // If do not have face number mapping.
    return true;
  }

  // First, check ray path length.
  // And store current ray path for later computing.
  decltype(ray_paths.size()) curr_ray_path_len = 0;
  std::vector<uint16_t> curr_ray_path;
  auto p = last_r;
  while (p->prev) {
    int curr_fn = crystal->FaceNumber(p->face_id);
    if (curr_fn < 0) {
      return false;
    }
    curr_ray_path.emplace_back(static_cast<uint16_t>(curr_fn));
    p = p->prev;
    curr_ray_path_len++;
  }

  bool length_matched = false;
  for (const auto& rp : ray_paths) {
    length_matched = length_matched || (curr_ray_path_len == rp.size());
  }
  if (!length_matched) {
    return false;
  }

  std::reverse(curr_ray_path.begin(), curr_ray_path.end());

  // Second, for each filter path, normalize current ray path, and find it in ray_path_hashes.
  auto current_ray_path_hash = RayPathHash(curr_ray_path);
  return ray_path_hashes.find(current_ray_path_hash) != ray_path_hashes.end();
}


bool RayPathFilter::FilterRayGeneral(RaySegment* last_r, const CrystalPtr& crystal) const {
  if (entry_faces.empty() && exit_faces.empty()) {
    return true;
  }

  if (!hit_nums.empty()) {    // Check hit number.
    auto p = last_r;
    int n = 0;
    while (p) {
      p = p->prev;
      n++;
    }
    if (hit_nums.find(n) == hit_nums.end()) {
      return false;
    }
  }

  int curr_entry_fn = crystal->FaceNumber(last_r->root_ctx->first_ray_segment->face_id);
  int curr_exit_fn = crystal->FaceNumber(last_r->face_id);
  if (curr_entry_fn < 0 || curr_exit_fn < 0 ||
      crystal->GetFaceNumberPeriod() < 0) {    // If do not have a face number mapping
    return true;
  }

  return entry_faces.find(static_cast<uint16_t>(curr_entry_fn)) != entry_faces.end() &&
         exit_faces.find(static_cast<uint16_t>(curr_exit_fn)) != exit_faces.end();
}


size_t RayPathFilter::RayPathHash(const std::vector<uint16_t>& ray_path) const {
  constexpr size_t kStep = 7;
  constexpr size_t kByteBits = 8;
  constexpr size_t kTotalBits = sizeof(size_t) * kByteBits;

  size_t result = 0;
  size_t curr_offset = 0;
  for (auto fn : ray_path) {
    size_t tmp_hash = (fn << curr_offset) | (fn >> (kTotalBits - curr_offset));
    result ^= tmp_hash;
    curr_offset += kStep;
    curr_offset %= kTotalBits;
  }
  return result;
}


void RayPathFilter::ApplyHash(const CrystalPtr& crystal) {
  std::vector<std::vector<uint16_t> > augmented_ray_paths;

  // Add the original path.
  for (const auto& rp : ray_paths) {
    augmented_ray_paths.emplace_back(rp);
  }

  // Add symmetry P.
  auto period = crystal->GetFaceNumberPeriod();
  std::vector<uint16_t> tmp_ray_path;
  if (period > 0 && (symmetry & kSymmetryPrism)) {
    std::vector<std::vector<uint16_t> > ray_paths_copy(augmented_ray_paths);
    for (const auto& rp : ray_paths_copy) {
      for (int i = 0; i < period; i++) {
        tmp_ray_path.clear();
        for (auto fn : rp) {
          if (fn != 1 && fn != 2) {
            fn = static_cast<uint16_t>((fn + period + i - 3) % period + 3);
          }
          tmp_ray_path.emplace_back(fn);
        }
        augmented_ray_paths.emplace_back(tmp_ray_path);
      }
    }
  }

  // Add symmetry B.
  if (symmetry & kSymmetryBasal) {
    std::vector<std::vector<uint16_t> > ray_paths_copy(augmented_ray_paths);
    for (const auto& rp : ray_paths_copy) {
      tmp_ray_path.clear();
      for (auto fn : rp) {
        if (fn == 1 || fn == 2) {
          fn = static_cast<uint16_t>(fn % 2 + 1);
        }
        tmp_ray_path.emplace_back(fn);
      }
      augmented_ray_paths.emplace_back(tmp_ray_path);
    }
  }

  // Add symmetry D.
  if (period > 0 && (symmetry & kSymmetryDirection)) {
    std::vector<std::vector<uint16_t> > ray_paths_copy(augmented_ray_paths);
    for (const auto& rp : ray_paths_copy) {
      tmp_ray_path.clear();
      for (auto fn : rp) {
        if (fn != 1 && fn != 2) {
          fn = static_cast<uint16_t>(5 + period - fn);
        }
        tmp_ray_path.emplace_back(fn);
      }
      augmented_ray_paths.emplace_back(tmp_ray_path);
    }
  }

  // Add them all.
  ray_path_hashes.clear();
  for (const auto& rp : augmented_ray_paths) {
    ray_path_hashes.emplace(RayPathHash(rp));
  }
}


CrystalContext::CrystalContext(CrystalPtrU&& g, const AxisDistribution& axis)
    : crystal(std::move(g)), axis(axis) {}


CrystalContext::CrystalContext(const IceHalo::CrystalContext& other) = default;


SimulationContext::SimulationContext(const char* filename, rapidjson::Document& d)
    : sun_diameter_(0.5f), total_ray_num_(0), current_wavelength_(550.0f), max_recursion_num_(9),
      config_file_name_(filename), data_directory_("./") {
  ParseSunSettings(d);
  ParseRaySettings(d);
  ParseBasicSettings(d);
  ParseCrystalSettings(d);
  ParseRayPathFilterSettings(d);
  ParseMultiScatterSettings(d);
  ApplySettings();
}


std::unique_ptr<SimulationContext> SimulationContext::CreateFromFile(const char* filename) {
  printf("Reading config from: %s\n", filename);

  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    std::fprintf(stderr, "ERROR: file %s cannot be open!\n", filename);
    fclose(fp);
    return nullptr;
  }

  constexpr size_t kTmpBufferSize = 65536;
  char buffer[kTmpBufferSize];
  rapidjson::FileReadStream is(fp, buffer, sizeof(buffer));

  rapidjson::Document d;
  if (d.ParseStream(is).HasParseError()) {
    std::fprintf(stderr, "\nError(offset %zu): %s\n", d.GetErrorOffset(),
                 GetParseError_En(d.GetParseError()));
    fclose(fp);
    return nullptr;
  }

  fclose(fp);
  return std::unique_ptr<SimulationContext>(new SimulationContext(filename, d));
}


void SimulationContext::ParseBasicSettings(rapidjson::Document& d) {
  int maxRecursion = 9;
  auto p = Pointer("/max_recursion").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <max_recursion>, using default 9!\n");
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <max_recursion> is not a integer, using default 9!\n");
  } else {
    maxRecursion = std::min(std::max(p->GetInt(), 1), 10);
  }
  max_recursion_num_ = maxRecursion;

  std::string dir = "./";
  p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default './'!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default './'!\n");
  } else {
    dir = p->GetString();
  }
  data_directory_ = dir;
}


void SimulationContext::ParseRaySettings(rapidjson::Document& d) {
  total_ray_num_ = 10000;
  auto p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <ray.number>, using default 10000!\n");
  } else if (!p->IsUint()) {
    std::fprintf(stderr, "\nWARNING! Config <ray.number> is not unsigned int, using default 10000!\n");
  } else {
    total_ray_num_ = p->GetUint();
  }

  std::vector<float> tmp_wavelengths{ 550.0f };
  auto wl_p = Pointer("/ray/wavelength").Get(d);
  if (wl_p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <ray.wavelength>, using default 550!\n");
  } else if (!wl_p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <ray.wavelength> is not an array, using default 550!\n");
  } else if (!(*wl_p)[0].IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <ray.wavelength> cannot be recognized, using default 550!\n");
  } else {
    tmp_wavelengths.clear();
    for (const auto& pi : wl_p->GetArray()) {
      tmp_wavelengths.push_back(static_cast<float &&>(pi.GetDouble()));
    }
  }

  std::vector<float> tmp_weights{ 1.0f };
  auto wt_p = Pointer("/ray/weight").Get(d);
  if (wt_p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <ray.weight>, using default 1.0!\n");
  } else if (!wt_p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <ray.wavelength> is not an array, using default 1.0!\n");
  } else if (!(*wt_p)[0].IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <ray.wavelength> cannot be recognized, using default 1.0!\n");
  } else {
    tmp_weights.clear();
    for (const auto& pi : wt_p->GetArray()) {
      tmp_weights.push_back(static_cast<float &&>(pi.GetDouble()));
    }
  }

  if (tmp_wavelengths.size() != tmp_weights.size()) {
    throw std::invalid_argument("size of ray.wavelength and ray.weight doesn't match!");
  }

  wavelengths_.clear();
  for (decltype(tmp_wavelengths.size()) i = 0; i < tmp_wavelengths.size(); i++) {
    wavelengths_.emplace_back(tmp_wavelengths[i], tmp_weights[i]);
  }
}


void SimulationContext::ParseSunSettings(rapidjson::Document& d) {
  float sunAltitude = 0.0f;
  auto* p = Pointer("/sun/altitude").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <sun.altitude>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <sun.altitude> is not a number, using default 0.0!\n");
  } else {
    sunAltitude = static_cast<float>(p->GetDouble());
  }
  SetSunRayDirection(90.0f, sunAltitude);

  sun_diameter_ = 0.5f;
  p = Pointer("/sun/diameter").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <sun.diameter>, using default 0.5!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <sun.diameter> is not a number, using default 0.5!\n");
  } else {
    sun_diameter_ = static_cast<float>(p->GetDouble());
  }
}


void SimulationContext::ParseMultiScatterSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  auto p = Pointer("/multi_scatter").Get(d);
  if (p == nullptr || !p->IsArray() || !p->GetArray()[0].IsObject()) {
    std::snprintf(buffer, kTmpBufferSize, "Config <multi_scatter> cannot be recognized!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    ParseOneScatterSetting(c, ci);
    ci++;
  }
}


void SimulationContext::ParseCrystalSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  const auto* p = Pointer("/crystal").Get(d);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(buffer, kTmpBufferSize, "Missing <crystal>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    ParseOneCrystalSetting(c, ci);
    ci++;
  }
}


void SimulationContext::ParseRayPathFilterSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  const auto* p = Pointer("/ray_path_filter").Get(d);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(buffer, kTmpBufferSize, "Missing <ray_path_filter>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    ParseOneFilterSetting(c, ci);
    ci++;
  }
}


std::unordered_map<std::string, SimulationContext::CrystalParser> SimulationContext::crystal_parser_ = {
  { "HexPrism", &SimulationContext::ParseCrystalHexPrism },
  { "HexPyramid", &SimulationContext::ParseCrystalHexPyramid },
  { "HexPyramidStackHalf", &SimulationContext::ParseCrystalHexPyramidStackHalf },
  { "CubicPyramid", &SimulationContext::ParseCrystalCubicPyramid },
  { "IrregularHexPrism", &SimulationContext::ParseCrystalIrregularHexPrism },
  { "IrregularHexPyramid", &SimulationContext::ParseCrystalIrregularHexPyramid },
  { "Custom", &SimulationContext::ParseCrystalCustom },
};


void SimulationContext::ParseOneCrystalSetting(const rapidjson::Value& c, int ci) {
  using Math::Distribution;

  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  std::string type(c["type"].GetString());
  if (crystal_parser_.find(type) == crystal_parser_.end()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/id").Get(c);
  if (p == nullptr || !p->IsInt()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].id> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  crystal_ctx_.emplace(p->GetInt(),
                       std::make_shared<CrystalContext>(crystal_parser_[type](this, c, ci), ParseCrystalAxis(c, ci)));
}


AxisDistribution SimulationContext::ParseCrystalAxis(const rapidjson::Value& c, int ci) {
  using Math::Distribution;

  AxisDistribution axis{};
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  // Start parsing zenith settings.
  const auto* p = Pointer("/zenith/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].zenith.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (*p == "gauss") {
    axis.latitude_dist = Distribution::kGaussian;
  } else if (*p == "uniform") {
    axis.latitude_dist = Distribution::kUniform;
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].zenith.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/zenith/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].zenith.mean> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.latitude_mean = static_cast<float>(90 - p->GetDouble());
  }

  p = Pointer("/zenith/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].zenith.std> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.latitude_std = static_cast<float>(p->GetDouble());
  }

  // Start parsing azimuth settings.
  axis.azimuth_dist = Math::Distribution::kUniform;
  axis.azimuth_mean = 0;
  axis.azimuth_std = 360;
  p = Pointer("/azimuth").Get(c);
  if (p == nullptr || !p->IsObject()) {
    std::fprintf(stderr, "<crystal[%d].azimuth> cannot recognize! Use default.\n", ci);
  } else {
    p = Pointer("/azimuth/type").Get(c);
    if (p == nullptr || !p->IsString()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].azimuth.type> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    } else if (*p == "gauss") {
      axis.azimuth_dist = Distribution::kGaussian;
    } else if (*p == "uniform") {
      axis.azimuth_dist = Distribution::kUniform;
    } else {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].azimuth.type> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    }

    p = Pointer("/azimuth/mean").Get(c);
    if (p == nullptr || !p->IsNumber()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].azimuth.mean> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    } else {
      axis.azimuth_mean = static_cast<float>(p->GetDouble());
    }

    p = Pointer("/azimuth/std").Get(c);
    if (p == nullptr || !p->IsNumber()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].azimuth.std> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    } else {
      axis.azimuth_std = static_cast<float>(p->GetDouble());
    }
  }

  // Start parsing roll settings.
  p = Pointer("/roll/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (*p == "gauss") {
    axis.roll_dist = Distribution::kGaussian;
  } else if (*p == "uniform") {
    axis.roll_dist = Distribution::kUniform;
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/roll/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.mean> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.roll_mean = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].roll.std> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.roll_std = static_cast<float>(p->GetDouble());
  }

  return axis;
}


CrystalPtrU SimulationContext::ParseCrystalHexPrism(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
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
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
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
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalHexPyramidStackHalf(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
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
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalCubicPyramid(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else if (p->Size() == 2) {
    auto h1 = static_cast<float>((*p)[0].GetDouble());
    auto h2 = static_cast<float>((*p)[1].GetDouble());
    return Crystal::CreateCubicPyramid(h1, h2);
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalIrregularHexPrism(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray() || p->Size() != 7) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
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
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
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
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> number doesn't match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU SimulationContext::ParseCrystalCustom(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    auto n = config_file_name_.rfind('/');
    if (n == std::string::npos) {
      std::snprintf(msg_buffer, kMsgBufferSize, "models/%s", p->GetString());
    } else {
      std::snprintf(msg_buffer, kMsgBufferSize, "%s/models/%s", config_file_name_.substr(0, n).c_str(), p->GetString());
    }
    std::FILE* file = fopen(msg_buffer, "r");
    if (!file) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot open model file!", ci);
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


void SimulationContext::ParseOneScatterSetting(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  MultiScatterContext scatter{};
  ParseScatterCrystal(c, ci, &scatter);
  ParseScatterPopulation(c, ci, &scatter);
  ParseScatterProbability(c, ci, &scatter);
  ParseScatterFilter(c, ci, &scatter);

  if (scatter.crystals.size() != scatter.populations.size()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d]> crystal and population size does not match!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  if (scatter.crystals.size() != scatter.ray_path_filters.size()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d]> crystal and ray_path_filters size does not match!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  for (decltype(scatter.crystals.size()) i = 0; i < scatter.crystals.size(); i++) {
    scatter.ray_path_filters[i].ApplyHash(scatter.crystals[i]->crystal);
  }

  multi_scatter_ctx_.emplace_back(scatter);
}


void SimulationContext::ParseScatterCrystal(const rapidjson::Value& c, int ci, MultiScatterContext* scatter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/crystal").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].crystal> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  for (auto& pc : p->GetArray()) {
    if (!pc.IsUint()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].crystal> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    } else {
      int id = pc.GetInt();
      if (crystal_ctx_.find(id) == crystal_ctx_.end()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].crystal> contains invalid ID!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      scatter->crystals.emplace_back(crystal_ctx_.at(id));
    }
  }
}


void SimulationContext::ParseScatterPopulation(const rapidjson::Value& c, int ci, MultiScatterContext* scatter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/population").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].population> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  for (auto& pp : p->GetArray()) {
    if (!pp.IsNumber()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].population> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    } else {
      scatter->populations.emplace_back(static_cast<float>(pp.GetDouble()));
    }
  }
}


void SimulationContext::ParseScatterProbability(const rapidjson::Value& c, int ci, MultiScatterContext* scatter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/probability").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].probability> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  auto prob = static_cast<float>(p->GetDouble());
  if (prob < 0) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].probability> is invalid!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  scatter->prob = prob;
}


void SimulationContext::ParseScatterFilter(const rapidjson::Value& c, int ci, MultiScatterContext* scatter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/ray_path_filter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].ray_path_filter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  for (auto& pf : p->GetArray()) {
    if (!pf.IsUint()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].ray_path_filter> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    } else {
      int id = pf.GetInt();
      if (ray_path_filters_.find(id) == ray_path_filters_.end()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].ray_path_filter> contains invalid ID!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      scatter->ray_path_filters.emplace_back(ray_path_filters_[id]);
    }
  }
}


void SimulationContext::ParseOneFilterSetting(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  RayPathFilter filter{};
  ParseFilterType(c, ci, &filter);
  ParseFilterSymmetry(c, ci, &filter);
  ParseFilterSpecificSettings(c, ci, &filter);
  ParseFilterGeneralSettings(c, ci, &filter);

  auto p = Pointer("/id").Get(c);
  if (p == nullptr || !p->IsInt()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].id> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  int id = p->GetInt();

  ray_path_filters_.emplace(id, filter);
}


void SimulationContext::ParseFilterSymmetry(const rapidjson::Value& c, int ci, RayPathFilter* filter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/symmetry").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].symmetry> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  filter->symmetry = RayPathFilter::kSymmetryNone;
  auto symm = p->GetString();
  for (decltype(p->GetStringLength()) i = 0; i < p->GetStringLength(); i++) {
    switch (symm[i]) {
      case 'P':
        filter->symmetry |= RayPathFilter::kSymmetryPrism;
        break;
      case 'B':
        filter->symmetry |= RayPathFilter::kSymmetryBasal;
        break;
      case 'D':
        filter->symmetry |= RayPathFilter::kSymmetryDirection;
        break;
      default:
        std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].symmetry> cannot recognize!", ci);
        throw std::invalid_argument(msg_buffer);
    }
  }
}


void SimulationContext::ParseFilterType(const rapidjson::Value& c, int ci, RayPathFilter* filter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  if (*p == "none") {
    filter->type = RayPathFilter::kTypeNone;
  } else if (*p == "specific") {
    filter->type = RayPathFilter::kTypeSpecific;
  } else if (*p == "general") {
    filter->type = RayPathFilter::kTypeGeneral;
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
}


void SimulationContext::ParseFilterSpecificSettings(const rapidjson::Value& c, int ci, RayPathFilter* filter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/path").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  if (!p->GetArray().Empty() && !p->GetArray()[0].IsInt() && !p->GetArray()[0].IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  if (p->GetArray().Empty()) {
    std::fprintf(stderr, "<ray_path_filter[%d].path> is empty. Ignore this setting.\n", ci);
  } else if (p->GetArray()[0].IsInt()) {
    filter->ray_paths.emplace_back();
    for (auto const& pi : p->GetArray()) {
      if (!pi.IsInt()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      filter->ray_paths.back().emplace_back(pi.GetInt());
    }
  } else {    // p[0].IsArray()
    for (const auto& pi : p->GetArray()) {
      if (pi.GetArray().Empty()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      filter->ray_paths.emplace_back();
      for (const auto& pii : pi.GetArray()) {
        if (!pii.IsInt()) {
          std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
          throw std::invalid_argument(msg_buffer);
        }
        filter->ray_paths.back().emplace_back(pii.GetInt());
      }
    }
  }
}


void SimulationContext::ParseFilterGeneralSettings(const rapidjson::Value& c, int ci, IceHalo::RayPathFilter* filter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/entry").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].entry> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  for (const auto& pi : p->GetArray()) {
    if (!pi.IsInt()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].entry> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    }
    filter->entry_faces.emplace(pi.GetInt());
  }

  p = Pointer("/exit").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].exit> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  for (const auto& pi : p->GetArray()) {
    if (!pi.IsInt()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].exit> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    }
    filter->exit_faces.emplace(pi.GetInt());
  }

  p = Pointer("/hit").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].hit> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  for (const auto& pi : p->GetArray()) {
    if (!pi.IsInt()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].hit> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    }
    filter->hit_nums.emplace(pi.GetInt());
  }
}


uint64_t SimulationContext::GetTotalInitRays() const {
  return total_ray_num_;
}


int SimulationContext::GetMaxRecursionNum() const {
  return max_recursion_num_;
}


void SimulationContext::SetCurrentWavelength(float wavelength, float weight) {
  this->current_wavelength_ = wavelength;
  this->current_wavelength_weight_ = weight;
}


float SimulationContext::GetCurrentWavelength() const {
  return current_wavelength_;
}


float SimulationContext::GetCurrentWavelengthWeight() const {
  return current_wavelength_weight_;
}


std::vector<std::pair<float, float>> SimulationContext::GetWavelengths() const {
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

  float weight_sum = 0;

  // Normalize wavelength weights.
  for (const auto& wl : wavelengths_) {
    weight_sum += wl.second;
  }
  for (auto& wl : wavelengths_) {
    wl.second /= weight_sum;
  }

  // Normalize crystal weights.
  for (auto& ms : multi_scatter_ctx_) {
    weight_sum = 0;
    for (auto& w : ms.populations) {
      weight_sum += w;
    }
    for (auto& w : ms.populations) {
      w /= weight_sum;
    }
  }
}


const std::vector<MultiScatterContext> SimulationContext::GetMultiScatterContext() const {
  return multi_scatter_ctx_;
}


void SimulationContext::PrintCrystalInfo() {
  for (const auto& c : crystal_ctx_) {
    auto g = c.second->crystal;
    printf("--\n");
    for (const auto& v : g->GetVertexes()) {
      printf("v %+.4f %+.4f %+.4f\n", v.x(), v.y(), v.z());
    }
    for (const auto& f : g->GetFaces()) {
      auto idx = f.idx();
      printf("f %d %d %d\n", idx[0] + 1, idx[1] + 1, idx[2] + 1);
    }
  }
}


RayContext::RayContext(RaySegment* seg, const CrystalContextPtr& crystal_ctx, const float* main_axis_rot)
  : first_ray_segment(seg), prev_ray_segment(nullptr),
    crystal_ctx(crystal_ctx), main_axis_rot(main_axis_rot) {}



RenderContext::RenderContext(rapidjson::Document& d) :
    img_hei_(0), img_wid_(0), offset_y_(0), offset_x_(0),
    visible_semi_sphere_(VisibleSemiSphere::kUpper),
    projection_type_(ProjectionType::kEqualArea),
    total_ray_num_(0), intensity_factor_(1.0), show_horizontal_(true),
    data_directory_("./") {
  ParseCameraSettings(d);
  ParseRenderSettings(d);
  ParseDataSettings(d);
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
    std::fprintf(stderr, "\nError(offset %u): %s\n", (unsigned)d.GetErrorOffset(),
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
  projection_type_ = ProjectionType::kEqualArea;

  auto* p = Pointer("/camera/azimuth").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.azimuth>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.azimuth> is not a number, using default 90.0!\n");
  } else {
    auto az = static_cast<float>(p->GetDouble());
    az = std::max(std::min(az, 360.0f), 0.0f);
    cam_rot_[0] = 90.0f - az;
  }

  p = Pointer("/camera/elevation").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.elevation>, using default 90.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.elevation> is not a number, using default 90.0!\n");
  } else {
    auto el = static_cast<float>(p->GetDouble());
    el = std::max(std::min(el, 89.999f), -89.999f);
    cam_rot_[1] = el;
  }

  p = Pointer("/camera/rotation").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.rotation>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.rotation> is not a number, using default 0.0!\n");
  } else {
    auto rot = static_cast<float>(p->GetDouble());
    rot = std::max(std::min(rot, 180.0f), -180.0f);
    cam_rot_[2] = rot;
  }

  p = Pointer("/camera/fov").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.fov>, using default 120.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.fov> is not a number, using default 120.0!\n");
  } else {
    fov_ = static_cast<float>(p->GetDouble());
    fov_ = std::max(std::min(fov_, 140.0f), 0.0f);
  }

  p = Pointer("/camera/width").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.width>, using default 800!\n");
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <camera.width> is not an integer, using default 800!\n");
  } else {
    int width = p->GetInt();
    width = std::max(width, 0);
    img_wid_ = static_cast<uint32_t>(width);
  }

  p = Pointer("/camera/height").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.height>, using default 800!\n");
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <camera.height> is not an integer, using default 800!\n");
  } else {
    int height = p->GetInt();
    height = std::max(height, 0);
    img_hei_ = static_cast<uint32_t>(height);
  }

  p = Pointer("/camera/lens").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.lens>, using default equi-area fisheye!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! config <camera.lens> is not a string, using default equi-area fisheye!\n");
  } else {
    if (*p == "linear") {
      projection_type_ = ProjectionType::kLinear;
    } else if (*p == "fisheye") {
      projection_type_ = ProjectionType::kEqualArea;
    } else if (*p == "dual_fisheye_equidistant") {
      projection_type_ = ProjectionType::kDualEquidistant;
    } else if (*p == "dual_fisheye_equiarea") {
      projection_type_ = ProjectionType::kDualEqualArea;
    } else {
      std::fprintf(stderr, "\nWARNING! config <camera.lens> cannot be recognized, using default equi-area fisheye!\n");
    }
  }
}


void RenderContext::ParseRenderSettings(rapidjson::Document& d) {
  visible_semi_sphere_ = VisibleSemiSphere::kUpper;
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
    std::fprintf(stderr, "\nWARNING! Config missing <render.visible_semi_sphere>, using default kUpper!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> is not a string, using default kUpper!\n");
  } else if (*p == "upper") {
    visible_semi_sphere_ = VisibleSemiSphere::kUpper;
  } else if (*p == "lower") {
    visible_semi_sphere_ = VisibleSemiSphere::kLower;
  } else if (*p == "camera") {
    visible_semi_sphere_ = VisibleSemiSphere::kCamera;
  } else if (*p == "full") {
    visible_semi_sphere_ = VisibleSemiSphere::kFull;
  } else {
    std::fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> cannot be recognized, using default kUpper!\n");
  }

  p = Pointer("/render/intensity_factor").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.intensity_factor>, using default 1.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <render.intensity_factor> is not a number, using default 1.0!\n");
  } else {
    double f = p->GetDouble();
    f = std::max(std::min(f, 100.0), 0.01);
    intensity_factor_ = f;
  }

  p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <ray.number>, using default 10000!\n");
  } else if (!p->IsUint()) {
    std::fprintf(stderr, "\nWARNING! Config <ray.number> is not unsigned int, using default 10000!\n");
  } else {
    total_ray_num_ = p->GetUint();
  }

  p = Pointer("/render/offset").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.offset>, using default [0, 0]!\n");
  } else if (!p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <render.offset> is not an array, using default [0, 0]!\n");
  } else if (p->Size() != 2 || !(*p)[0].IsInt() || !(*p)[1].IsInt()) {
    std::fprintf(stderr, "\nWARNING! Config <render.offset> cannot be recognized, using default [0, 0]!\n");
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
    std::fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default [0,0,0]!\n");
  } else if (!p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array, using default [0,0,0]!\n");
  } else if (p->Size() != 3 || !(*p)[0].IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default [0,0,0]!\n");
  } else {
    auto pa = p->GetArray();
    for (int i = 0; i < 3; i++) {
      background_color_[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
    }
  }

  p = Pointer("/render/ray_color").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default real color!\n");
  } else if (!p->IsArray() && !p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array nor a string, using default real color!\n");
  } else if (p->IsArray() && (p->Size() != 3 || !(*p)[0].IsNumber())) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default real color!\n");
  } else if (p->IsString() && (*p) != "real") {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default real color!\n");
  } else if (p->IsArray()) {
    auto pa = p->GetArray();
    for (int i = 0; i < 3; i++) {
      ray_color_[i] = static_cast<float>(std::min(std::max(pa[i].GetDouble(), 0.0), 1.0));
    }
  }

  p = Pointer("/render/show_horizontal").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.show_horizontal>, using default true!\n");
  } else if (!p->IsBool()) {
    std::fprintf(stderr, "\nWARNING! Config <render.show_horizontal> is not a boolean, using default true!\n");
  } else {
    show_horizontal_ = p->GetBool();
  }
}


void RenderContext::ParseDataSettings(rapidjson::Document& d) {
  data_directory_ = "./";
  auto* p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default './'!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default './'!\n");
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


std::string RenderContext::GetDataDirectory() const {
  return data_directory_;
}


const float* RenderContext::GetCamRot() const {
  return cam_rot_;
}


float RenderContext::GetFov() const {
  return fov_;
}


ProjectionType RenderContext::GetProjectionType() const {
  return projection_type_;
}


VisibleSemiSphere RenderContext::GetVisibleSemiSphere() const {
  return visible_semi_sphere_;
}


int RenderContext::GetOffsetX() const {
  return offset_x_;
}


int RenderContext::GetOffsetY() const {
  return offset_y_;
}


uint32_t RenderContext::GetTotalRayNum() const {
  return total_ray_num_;
}


const float* RenderContext::GetRayColor() const {
  return ray_color_;
}


const float* RenderContext::GetBackgroundColor() const {
  return background_color_;
}


double RenderContext::GetIntensityFactor() const {
  return intensity_factor_;
}

}   // namespace IceHalo

