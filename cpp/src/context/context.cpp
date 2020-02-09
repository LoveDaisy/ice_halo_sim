#include "context/context.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "core/optics.h"
#include "core/render.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"
#include "util/threadingpool.h"


namespace icehalo {

using rapidjson::Pointer;

AbstractRayPathFilter::AbstractRayPathFilter()
    : symmetry_flag_(kSymmetryNone), complementary_(false), remove_homodromous_(false) {}


bool AbstractRayPathFilter::Filter(const Crystal* crystal, RaySegment* last_r) const {
  if (remove_homodromous_ &&
      math::Dot3(last_r->dir.val(), last_r->root_ctx->first_ray_segment->dir.val()) > 1.0 - 5 * math::kFloatEps) {
    return false;
  }

  bool result = FilterPath(crystal, last_r);
  return result ^ complementary_;
}


void AbstractRayPathFilter::SetSymmetryFlag(uint8_t symmetry_flag) {
  symmetry_flag_ = symmetry_flag;
}


void AbstractRayPathFilter::AddSymmetry(Symmetry symmetry) {
  symmetry_flag_ |= symmetry;
}


uint8_t AbstractRayPathFilter::GetSymmetryFlag() const {
  return symmetry_flag_;
}


void AbstractRayPathFilter::ApplySymmetry(const Crystal* /* crystal */) {}


void AbstractRayPathFilter::EnableComplementary(bool enable) {
  complementary_ = enable;
}


bool AbstractRayPathFilter::GetComplementary() const {
  return complementary_;
}


void AbstractRayPathFilter::EnableRemoveHomodromous(bool enable) {
  remove_homodromous_ = enable;
}


bool AbstractRayPathFilter::GetRemoveHomodromous() const {
  return remove_homodromous_;
}


size_t RayPathHash(const std::vector<uint16_t>& ray_path, bool reverse) {
  constexpr size_t kStep = 7;
  constexpr size_t kTotalBits = sizeof(size_t) * CHAR_BIT;

  size_t result = 0;
  size_t curr_offset = 0;
  if (reverse) {
    for (auto rit = ray_path.rbegin(); rit != ray_path.rend(); ++rit) {
      unsigned int fn = *rit;
      size_t tmp_hash = (fn << curr_offset) | (fn >> (kTotalBits - curr_offset));
      result ^= tmp_hash;
      curr_offset += kStep;
      curr_offset %= kTotalBits;
    }
  } else {
    for (unsigned int fn : ray_path) {
      size_t tmp_hash = (fn << curr_offset) | (fn >> (kTotalBits - curr_offset));
      result ^= tmp_hash;
      curr_offset += kStep;
      curr_offset %= kTotalBits;
    }
  }
  return result;
}


size_t RayPathHash(const Crystal* crystal,                  // used for get face number
                   const RaySegment* last_ray, int length,  // ray path and length
                   bool reverse) {
  constexpr size_t kStep = 7;
  constexpr size_t kTotalBits = sizeof(size_t) * CHAR_BIT;

  size_t result = 0;
  size_t curr_offset = reverse ? kStep * (length - 1) % kTotalBits : 0;
  auto p = last_ray;
  while (p->prev) {
    unsigned int fn = crystal->FaceNumber(p->face_id);
    size_t tmp_hash = (fn << curr_offset) | (fn >> (kTotalBits - curr_offset));
    result ^= tmp_hash;

    if (reverse) {
      curr_offset -= kStep;
    } else {
      curr_offset += kStep;
    }
    curr_offset %= kTotalBits;
    p = p->prev;
  }

  return result;
}


bool NoneRayPathFilter::FilterPath(const Crystal* /* crystal */, RaySegment* /* r */) const {
  return true;
}


void SpecificRayPathFilter::AddPath(const std::vector<uint16_t>& path) {
  ray_paths_.emplace_back(path);
}


void SpecificRayPathFilter::ClearPaths() {
  ray_paths_.clear();
}


void SpecificRayPathFilter::ApplySymmetry(const Crystal* crystal) {
  std::vector<std::vector<uint16_t>> augmented_ray_paths;

  // Add the original path.
  for (const auto& rp : ray_paths_) {
    augmented_ray_paths.emplace_back(rp);
  }

  // Add symmetry P.
  auto period = crystal->GetFaceNumberPeriod();
  std::vector<uint16_t> tmp_ray_path;
  if (period > 0 && (symmetry_flag_ & kSymmetryPrism)) {
    std::vector<std::vector<uint16_t>> ray_paths_copy(augmented_ray_paths);
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
  if (symmetry_flag_ & kSymmetryBasal) {
    std::vector<std::vector<uint16_t>> ray_paths_copy(augmented_ray_paths);
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
  if (period > 0 && (symmetry_flag_ & kSymmetryDirection)) {
    std::vector<std::vector<uint16_t>> ray_paths_copy(augmented_ray_paths);
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
  ray_path_hashes_.clear();
  for (const auto& rp : augmented_ray_paths) {
    ray_path_hashes_.emplace(RayPathHash(rp));
  }
}


bool SpecificRayPathFilter::FilterPath(const Crystal* crystal, RaySegment* last_r) const {
  if (ray_path_hashes_.empty()) {
    return true;
  }

  int curr_fn0 = crystal->FaceNumber(last_r->root_ctx->first_ray_segment->face_id);
  if (curr_fn0 < 0 || crystal->GetFaceNumberPeriod() < 0) {  // If do not have face number mapping.
    return true;
  }

  // First, check ray path length.
  size_t curr_ray_path_len = 0;
  auto p = last_r;
  while (p->prev) {
    int curr_fn = crystal->FaceNumber(p->face_id);
    if (curr_fn < 0) {
      return false;
    }
    p = p->prev;
    curr_ray_path_len++;
  }
  if (curr_ray_path_len == 0) {
    return false;
  }

  bool length_matched = false;
  for (const auto& rp : ray_paths_) {
    length_matched = length_matched || (curr_ray_path_len == rp.size());
  }
  if (!length_matched) {
    return false;
  }

  // Second, for each filter path, normalize current ray path, and find it in ray_path_hashes.
  auto current_ray_path_hash = RayPathHash(crystal, last_r, curr_ray_path_len, true);
  return ray_path_hashes_.count(current_ray_path_hash) != 0;
}


void GeneralRayPathFilter::AddEntryFace(uint16_t face_number) {
  entry_faces_.emplace(face_number);
}


void GeneralRayPathFilter::AddExitFace(uint16_t face_number) {
  exit_faces_.emplace(face_number);
}


void GeneralRayPathFilter::AddHitNumber(int hit_num) {
  hit_nums_.emplace(hit_num);
}


void GeneralRayPathFilter::ClearFaces() {
  entry_faces_.clear();
  exit_faces_.clear();
}


void GeneralRayPathFilter::ClearHitNumbers() {
  hit_nums_.clear();
}


bool GeneralRayPathFilter::FilterPath(const Crystal* crystal, RaySegment* last_r) const {
  if (entry_faces_.empty() && exit_faces_.empty()) {
    return true;
  }

  if (!hit_nums_.empty()) {  // Check hit number.
    auto p = last_r;
    int n = 0;
    while (p) {
      p = p->prev;
      n++;
    }
    if (hit_nums_.count(n) == 0) {
      return false;
    }
  }

  int curr_entry_fn = crystal->FaceNumber(last_r->root_ctx->first_ray_segment->face_id);
  int curr_exit_fn = crystal->FaceNumber(last_r->face_id);
  if (curr_entry_fn < 0 || curr_exit_fn < 0 ||
      crystal->GetFaceNumberPeriod() < 0) {  // If do not have a face number mapping
    return true;
  }

  return entry_faces_.count(static_cast<uint16_t>(curr_entry_fn)) != 0 &&
         exit_faces_.count(static_cast<uint16_t>(curr_exit_fn)) != 0;
}


constexpr float ProjectContext::kPropMinW;
constexpr float ProjectContext::kScatMinW;
constexpr size_t ProjectContext::kMinInitRayNum;
constexpr int ProjectContext::kMinRayHitNum;
constexpr int ProjectContext::kMaxRayHitNum;


std::unique_ptr<ProjectContext> ProjectContext::CreateFromFile(const char* filename) {
  printf("Reading config from: %s\n", filename);

  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    std::fprintf(stderr, "ERROR: file %s cannot be open!\n", filename);
    std::fclose(fp);
    return nullptr;
  }

  constexpr size_t kTmpBufferSize = 65536;
  char buffer[kTmpBufferSize];
  rapidjson::FileReadStream is(fp, buffer, sizeof(buffer));

  rapidjson::Document d;
  if (d.ParseStream(is).HasParseError()) {
    std::fprintf(stderr, "\nError(offset %zu): %s\n", d.GetErrorOffset(), GetParseError_En(d.GetParseError()));
    std::fclose(fp);
    return nullptr;
  }

  fclose(fp);
  std::unique_ptr<ProjectContext> proj = CreateDefault();

  proj->sun_ctx_ = SunContext::CreateFromJson(d);
  proj->cam_ctx_ = CameraContext::CreateFromJson(d);
  proj->render_ctx_ = RenderContext::CreateFromJson(d);
  proj->ParseRaySettings(d);
  proj->ParseDataSettings(filename, d);
  proj->ParseCrystalSettings(d);
  proj->ParseRayPathFilterSettings(d);
  proj->ParseMultiScatterSettings(d);

  return proj;
}


std::unique_ptr<ProjectContext> ProjectContext::CreateDefault() {
  return std::unique_ptr<ProjectContext>(new ProjectContext());
}


size_t ProjectContext::GetInitRayNum() const {
  return init_ray_num_;
}


void ProjectContext::SetInitRayNum(size_t ray_num) {
#ifdef FOR_TEST
  init_ray_num_ = ray_num;
#else
  init_ray_num_ = std::max(ray_num, kMinInitRayNum);
#endif
}


int ProjectContext::GetRayHitNum() const {
  return ray_hit_num_;
}


void ProjectContext::SetRayHitNum(int hit_num) {
  ray_hit_num_ = std::min(std::max(hit_num, kMinRayHitNum), kMaxRayHitNum);
}


std::string ProjectContext::GetModelPath() const {
  return model_path_;
}


void ProjectContext::SetModelPath(const std::string& path) {
  model_path_ = path;
}


std::string ProjectContext::GetDataDirectory() const {
  return data_path_;
}


std::string ProjectContext::GetDefaultImagePath() const {
  return PathJoin(data_path_, "img.jpg");
}


const Crystal* ProjectContext::GetCrystal(int id) const {
  auto crystal_ctx = GetCrystalContext(id);
  if (crystal_ctx) {
    return crystal_ctx->crystal.get();
  } else {
    return nullptr;
  }
}


int32_t ProjectContext::GetCrystalId(const Crystal* crystal) const {
  for (const auto& kv : crystal_store_) {
    if (kv.second->crystal.get() == crystal) {
      return kv.first;
    }
  }
  return std::numeric_limits<int>::lowest();
}


const CrystalContext* ProjectContext::GetCrystalContext(int id) const {
  if (crystal_store_.count(id)) {
    return crystal_store_.at(id).get();
  } else {
    return nullptr;
  }
}


#ifdef FOR_TEST
void ProjectContext::PrintCrystalInfo() const {
  for (const auto& c : crystal_store_) {
    auto g = c.second->crystal.get();
    std::printf("-- ID: %d --\n", c.first);
    for (const auto& v : g->GetVertexes()) {
      std::printf("v %+.4f %+.4f %+.4f\n", v.x(), v.y(), v.z());
    }
    for (const auto& f : g->GetFaces()) {
      auto idx = f.idx();
      std::printf("f %d %d %d\n", idx[0] + 1, idx[1] + 1, idx[2] + 1);
    }
  }
}
#endif


AbstractRayPathFilter* ProjectContext::GetRayPathFilter(int id) const {
  if (filter_store_.count(id)) {
    return filter_store_.at(id).get();
  } else {
    return nullptr;
  }
}


ProjectContext::ProjectContext()
    : sun_ctx_{}, cam_ctx_{}, render_ctx_{}, init_ray_num_(kDefaultInitRayNum), ray_hit_num_(kDefaultRayHitNum),
      model_path_("") {}


void ProjectContext::ParseRaySettings(rapidjson::Document& d) {
  auto p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <ray.number>, using default %d!\n", ProjectContext::kMinRayHitNum);
  } else if (!p->IsUint()) {
    std::fprintf(stderr, "\nWARNING! Config <ray.number> is not unsigned int, using default %d!\n",
                 ProjectContext::kMinRayHitNum);
  } else {
    SetInitRayNum(p->GetUint());
  }

  p = Pointer("/max_recursion").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <max_recursion>, using default %d!\n",
                 ProjectContext::kMinRayHitNum);
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <max_recursion> is not a integer, using default %d!\n",
                 ProjectContext::kMinRayHitNum);
  } else {
    SetRayHitNum(p->GetInt());
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
      tmp_wavelengths.emplace_back(static_cast<float>(pi.GetDouble()));
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
      tmp_weights.emplace_back(static_cast<float>(pi.GetDouble()));
    }
  }

  if (tmp_wavelengths.size() != tmp_weights.size()) {
    throw std::invalid_argument("size of ray.wavelength and ray.weight doesn't match!");
  }

  wavelengths_.clear();
  for (decltype(tmp_wavelengths.size()) i = 0; i < tmp_wavelengths.size(); i++) {
    wavelengths_.emplace_back(WavelengthInfo{ static_cast<int>(tmp_wavelengths[i]), tmp_weights[i] });
  }
}


void ProjectContext::ParseDataSettings(const char* config_file_path, rapidjson::Document& d) {
  std::string dir = boost::filesystem::current_path().string();
  auto p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default current path!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default current path!\n");
  } else {
    dir = p->GetString();
  }
  data_path_ = dir;

  std::string config_file_path_str(config_file_path);
  SetModelPath(PathJoin(config_file_path_str, "models"));
}


void ProjectContext::ParseCrystalSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  const auto* p = Pointer("/crystal").Get(d);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(buffer, kTmpBufferSize, "Missing <crystal>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    ParseOneCrystal(c, ci);
    ci++;
  }
}


void ProjectContext::ParseRayPathFilterSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  const auto* p = Pointer("/ray_path_filter").Get(d);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(buffer, kTmpBufferSize, "Missing <ray_path_filter>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    ParseOneFilter(c, ci);
    ci++;
  }
}


void ProjectContext::ParseMultiScatterSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  auto p = Pointer("/multi_scatter").Get(d);
  if (p == nullptr || !p->IsArray() || !p->GetArray()[0].IsObject()) {
    std::snprintf(buffer, kTmpBufferSize, "Config <multi_scatter> cannot be recognized!");
    throw std::invalid_argument(buffer);
  }

  int ci = 0;
  for (const auto& c : p->GetArray()) {
    ParseOneScatter(c, ci);
    ci++;
  }
}


std::unordered_map<std::string, ProjectContext::CrystalParser>& ProjectContext::GetCrystalParsers(
    const std::string& model_path) {
  static std::unordered_map<std::string, CrystalParser> crystal_parsers = {
    { "HexPrism", &ProjectContext::ParseCrystalHexPrism },
    { "HexPyramid", &ProjectContext::ParseCrystalHexPyramid },
    { "HexPyramidStackHalf", &ProjectContext::ParseCrystalHexPyramidStackHalf },
    { "CubicPyramid", &ProjectContext::ParseCrystalCubicPyramid },
    { "IrregularHexPrism", &ProjectContext::ParseCrystalIrregularHexPrism },
    { "IrregularHexPyramid", &ProjectContext::ParseCrystalIrregularHexPyramid },
    { "Custom", [=, &model_path](const rapidjson::Value& c,
                                 int ci) { return ProjectContext::ParseCrystalCustom(c, ci, model_path); } },
  };
  return crystal_parsers;
}


void ProjectContext::ParseOneCrystal(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  auto& crystal_parsers = GetCrystalParsers(model_path_);
  std::string type(c["type"].GetString());
  if (crystal_parsers.find(type) == crystal_parsers.end()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/id").Get(c);
  if (p == nullptr || !p->IsInt()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].id> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  auto axis = ParseCrystalAxis(c, ci);
  auto id = p->GetInt();
  crystal_store_.emplace(id, new CrystalContext(crystal_parsers[type](c, ci), axis));
}


AxisDistribution ProjectContext::ParseCrystalAxis(const rapidjson::Value& c, int ci) {
  using math::Distribution;

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
  axis.azimuth_dist = math::Distribution::kUniform;
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


CrystalPtrU ProjectContext::ParseCrystalHexPrism(const rapidjson::Value& c, int ci) {
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


CrystalPtrU ProjectContext::ParseCrystalHexPyramid(const rapidjson::Value& c, int ci) {
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


CrystalPtrU ProjectContext::ParseCrystalHexPyramidStackHalf(const rapidjson::Value& c, int ci) {
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


CrystalPtrU ProjectContext::ParseCrystalCubicPyramid(const rapidjson::Value& c, int ci) {
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


CrystalPtrU ProjectContext::ParseCrystalIrregularHexPrism(const rapidjson::Value& c, int ci) {
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


CrystalPtrU ProjectContext::ParseCrystalIrregularHexPyramid(const rapidjson::Value& c, int ci) {
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


CrystalPtrU ProjectContext::ParseCrystalCustom(const rapidjson::Value& c, int ci, const std::string& model_path) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    auto n = model_path.rfind('/');
    if (n == std::string::npos) {
      std::snprintf(msg_buffer, kMsgBufferSize, "models/%s", p->GetString());
    } else {
      std::snprintf(msg_buffer, kMsgBufferSize, "%s/models/%s", model_path.substr(0, n).c_str(), p->GetString());
    }
    std::FILE* file = std::fopen(msg_buffer, "r");
    if (!file) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot open model file!", ci);
      throw std::invalid_argument(msg_buffer);
    }

    std::vector<math::Vec3f> vertexes;
    std::vector<math::TriangleIdx> faces;
    float v_buf[3];
    int f_buf[3];
    int curr_char;
    while ((curr_char = std::fgetc(file)) != EOF) {
      switch (curr_char) {
        case 'v':
        case 'V':
          std::fscanf(file, "%f %f %f", v_buf + 0, v_buf + 1, v_buf + 2);
          vertexes.emplace_back(v_buf);
          break;
        case 'f':
        case 'F':
          std::fscanf(file, "%d %d %d", f_buf + 0, f_buf + 1, f_buf + 2);
          faces.emplace_back(f_buf[0] - 1, f_buf[1] - 1, f_buf[2] - 1);
          break;
        default:
          break;
      }
    }
    std::fclose(file);

    return Crystal::CreateCustomCrystal(vertexes, faces);
  }
}


std::unordered_map<std::string, ProjectContext::FilterParser>& ProjectContext::GetFilterParsers() {
  static std::unordered_map<std::string, FilterParser> filter_parsers = {
    { "none", &ProjectContext::ParseFilterNone },
    { "specific", &ProjectContext::ParseFilterSpecific },
    { "general", &ProjectContext::ParseFilterGeneral },
  };
  return filter_parsers;
}


void ProjectContext::ParseOneFilter(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<filter[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  auto& filter_parsers = GetFilterParsers();
  std::string type(c["type"].GetString());
  if (filter_parsers.count(type) == 0) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<filter[%d].type> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/id").Get(c);
  if (p == nullptr || !p->IsInt()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<filter[%d].id> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  int id = p->GetInt();

  filter_store_.emplace(id, filter_parsers[type](c, ci));
}


void ProjectContext::ParseFilterBasic(const rapidjson::Value& c, int ci, const RayPathFilterPtrU& filter) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/complementary").Get(c);
  if (p == nullptr) {
    filter->EnableComplementary(false);
  } else if (p->IsBool()) {
    filter->EnableComplementary(p->GetBool());
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<filter[%d].complementary> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/remove_homodromous").Get(c);
  if (p == nullptr) {
    filter->EnableRemoveHomodromous(false);
  } else if (p->IsBool()) {
    filter->EnableRemoveHomodromous(p->GetBool());
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<filter[%d].remove_homodromous> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  filter->SetSymmetryFlag(kSymmetryNone);

  p = Pointer("/symmetry").Get(c);
  if (p == nullptr) {
    return;
  } else if (!p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].symmetry> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }

  auto symm = p->GetString();
  for (decltype(p->GetStringLength()) i = 0; i < p->GetStringLength(); i++) {
    switch (symm[i]) {
      case 'P':
      case 'p':
        filter->AddSymmetry(kSymmetryPrism);
        break;
      case 'B':
      case 'b':
        filter->AddSymmetry(kSymmetryBasal);
        break;
      case 'D':
      case 'd':
        filter->AddSymmetry(kSymmetryDirection);
        break;
      default:
        std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].symmetry> cannot recognize!", ci);
        throw std::invalid_argument(msg_buffer);
    }
  }
}


RayPathFilterPtrU ProjectContext::ParseFilterNone(const rapidjson::Value& c, int ci) {
  RayPathFilterPtrU filter = std::unique_ptr<NoneRayPathFilter>(new NoneRayPathFilter());
  ParseFilterBasic(c, ci, filter);
  return filter;
}


RayPathFilterPtrU ProjectContext::ParseFilterSpecific(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto filter_raw_ptr = new SpecificRayPathFilter();
  RayPathFilterPtrU filter{ filter_raw_ptr };
  ParseFilterBasic(c, ci, filter);

  filter_raw_ptr->ClearPaths();
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
    std::vector<uint16_t> tmp_path;
    for (auto const& pi : p->GetArray()) {
      if (!pi.IsInt()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      tmp_path.emplace_back(pi.GetInt());
    }
    filter_raw_ptr->AddPath(tmp_path);
  } else {  // p[0].IsArray()
    for (const auto& pi : p->GetArray()) {
      if (pi.GetArray().Empty()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      std::vector<uint16_t> tmp_path;
      for (const auto& pii : pi.GetArray()) {
        if (!pii.IsInt()) {
          std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].path> cannot recognize!", ci);
          throw std::invalid_argument(msg_buffer);
        }
        tmp_path.emplace_back(pii.GetInt());
        filter_raw_ptr->AddPath(tmp_path);
      }
    }
  }
  return filter;
}


RayPathFilterPtrU ProjectContext::ParseFilterGeneral(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto filter_raw_ptr = new GeneralRayPathFilter();
  RayPathFilterPtrU filter = std::unique_ptr<GeneralRayPathFilter>(filter_raw_ptr);
  ParseFilterBasic(c, ci, filter);
  filter_raw_ptr->ClearHitNumbers();
  filter_raw_ptr->ClearFaces();

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
    filter_raw_ptr->AddEntryFace(pi.GetInt());
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
    filter_raw_ptr->AddExitFace(pi.GetInt());
  }

  p = Pointer("/hit").Get(c);
  if (p == nullptr) {
    std::fprintf(stderr, "<ray_path_filter[%d].hit> is empty. Ignore this setting.\n", ci);
  } else if (!p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].hit> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    for (const auto& pi : p->GetArray()) {
      if (!pi.IsInt()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<ray_path_filter[%d].hit> cannot recognize!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      filter_raw_ptr->AddHitNumber(pi.GetInt());
    }
  }

  return filter;
}


void ProjectContext::ParseOneScatter(const rapidjson::Value& c, int ci) {
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

  MultiScatterContext scatter(prob);
  scatter.ClearCrystalInfo();

  std::vector<int> tmp_crystals;
  p = Pointer("/crystal").Get(c);
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
      if (crystal_store_.find(id) == crystal_store_.end()) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].crystal> contains invalid ID!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      tmp_crystals.emplace_back(id);
    }
  }

  std::vector<float> tmp_population;
  p = Pointer("/population").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].population> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  }
  for (auto& pp : p->GetArray()) {
    if (!pp.IsNumber()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].population> cannot recognize!", ci);
      throw std::invalid_argument(msg_buffer);
    } else {
      tmp_population.emplace_back(static_cast<float>(pp.GetDouble()));
    }
  }

  std::vector<int> tmp_filter;
  p = Pointer("/ray_path_filter").Get(c);
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
      if (filter_store_.count(id) == 0) {
        std::snprintf(msg_buffer, kMsgBufferSize, "<multi_scatter[%d].ray_path_filter> contains invalid ID!", ci);
        throw std::invalid_argument(msg_buffer);
      }
      tmp_filter.emplace_back(id);
    }
  }

  for (size_t i = 0; i < tmp_crystals.size(); i++) {
    auto crystal_ctx = GetCrystalContext(tmp_crystals[i]);
    auto filter = GetRayPathFilter(tmp_filter[i]);
    scatter.AddCrystalInfo(crystal_ctx, filter, tmp_population[i]);
  }

  scatter.NormalizeCrystalPopulation();

  multi_scatter_info_.emplace_back(scatter);
}

}  // namespace icehalo
