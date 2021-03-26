#include "context/context.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "core/optics.hpp"
#include "process/render.hpp"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"
#include "util/log.hpp"


namespace icehalo {

using rapidjson::Pointer;

constexpr float ProjectContext::kPropMinW;
constexpr float ProjectContext::kScatMinW;
constexpr size_t ProjectContext::kMinInitRayNum;
constexpr int ProjectContext::kMinRayHitNum;
constexpr int ProjectContext::kMaxRayHitNum;


ProjectContextPtrU ProjectContext::CreateFromFile(const char* filename) {
  LOG_VERBOSE("Reading config from: %s", filename);

  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    LOG_ERROR("file %s cannot be open!", filename);
    std::fclose(fp);
    return nullptr;
  }

  constexpr size_t kTmpBufferSize = 65536;
  char buffer[kTmpBufferSize];
  rapidjson::FileReadStream is(fp, buffer, sizeof(buffer));

  rapidjson::Document d;
  if (d.ParseStream(is).HasParseError()) {
    LOG_ERROR("Error(offset %zu): %s", d.GetErrorOffset(), GetParseError_En(d.GetParseError()));
    std::fclose(fp);
    return nullptr;
  }

  fclose(fp);
  ProjectContextPtrU proj = CreateDefault();

  proj->ParseBasicSettings(d);
  proj->ParseSunSettings(d);
  proj->ParseCameraSettings(d);
  proj->ParseRenderSettings(d);
  proj->ParseCrystalSettings(d);
  proj->ParseRayPathFilterSettings(d);
  proj->ParseMultiScatterSettings(d);

  return proj;
}


ProjectContextPtrU ProjectContext::CreateDefault() {
  return ProjectContextPtrU(new ProjectContext());
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


std::string ProjectContext::GetDataDirectory() const {
  return data_path_;
}


std::string ProjectContext::GetMainImagePath() const {
  return PathJoin(data_path_, main_img_filename_);
}


const Crystal* ProjectContext::GetCrystal(ShortIdType id) const {
  auto crystal_ctx = GetCrystalContext(id);
  if (crystal_ctx) {
    return crystal_ctx->GetCrystal();
  } else {
    return nullptr;
  }
}


ShortIdType ProjectContext::GetCrystalId(const Crystal* crystal) const {
  for (const auto& ctx : crystal_store_) {
    if (ctx->GetCrystal() == crystal) {
      return ctx->GetId();
    }
  }
  return kInvalidId;
}


const CrystalContext* ProjectContext::GetCrystalContext(ShortIdType id) const {
  for (const auto& ctx : crystal_store_) {
    if (ctx->GetId() == id) {
      return ctx.get();
    }
  }
  return nullptr;
}


CrystalMap ProjectContext::GetCrystalMap() const {
  CrystalMap crystal_map{};
  for (const auto& c : crystal_store_) {
    crystal_map.emplace(c->GetId(), c->GetCrystal());
  }
  return crystal_map;
}


void ProjectContext::PrintCrystalInfo() const {
  for (const auto& ctx : crystal_store_) {
    ctx->PrintCrystal();
  }
}


AbstractRayPathFilter* ProjectContext::GetRayPathFilter(ShortIdType id) const {
  for (const auto& f : filter_store_) {
    if (f->GetId() == id) {
      return f->GetFilter();
    }
  }
  return nullptr;
}


ProjectContext::ProjectContext()
    : sun_ctx_{}, cam_ctx_{}, render_ctx_{}, init_ray_num_(kDefaultInitRayNum), ray_hit_num_(kDefaultRayHitNum) {}


void ProjectContext::ParseBasicSettings(rapidjson::Document& d) {
  auto p = Pointer("/ray/number").Get(d);
  if (p == nullptr) {
    LOG_VERBOSE("Config missing <ray.number>. Use default %d!", ProjectContext::kMinRayHitNum);
  } else if (!p->IsUint()) {
    LOG_VERBOSE("Config <ray.number> is not unsigned int. Use default %d!", ProjectContext::kMinRayHitNum);
  } else {
    SetInitRayNum(p->GetUint());
  }

  p = Pointer("/max_recursion").Get(d);
  if (p == nullptr) {
    LOG_VERBOSE("Config missing <max_recursion>. Use default %d!", ProjectContext::kMinRayHitNum);
  } else if (!p->IsInt()) {
    LOG_VERBOSE("Config <max_recursion> is not an integer. Use default %d!", ProjectContext::kMinRayHitNum);
  } else {
    SetRayHitNum(p->GetInt());
  }

  std::vector<int> tmp_wavelengths{ 550 };
  auto wl_p = Pointer("/ray/wavelength").Get(d);
  if (wl_p == nullptr) {
    LOG_VERBOSE("Config missing <ray.wavelength>. Use default 550!");
  } else if (!wl_p->IsArray()) {
    LOG_VERBOSE("Config <ray.wavelength> is not an array. Use default 550!");
  } else if (!(*wl_p)[0].IsInt()) {
    LOG_VERBOSE("Config <ray.wavelength> cannot be recognized, using default 550!");
  } else {
    tmp_wavelengths.clear();
    for (const auto& pi : wl_p->GetArray()) {
      tmp_wavelengths.emplace_back(static_cast<float>(pi.GetDouble()));
    }
  }

  std::vector<float> tmp_weights{ 1.0f };
  auto wt_p = Pointer("/ray/weight").Get(d);
  if (wt_p == nullptr) {
    LOG_VERBOSE("Config missing <ray.weight>. Use default 1.0!");
  } else if (!wt_p->IsArray()) {
    LOG_VERBOSE("Config <ray.wavelength> is not an array. Use default 1.0!");
  } else if (!(*wt_p)[0].IsNumber()) {
    LOG_VERBOSE("Config <ray.wavelength> cannot be recognized. Use default 1.0!");
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

  std::string dir = boost::filesystem::current_path().string();
  p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    LOG_VERBOSE("Config missing <data_folder>. Use default current path!");
  } else if (!p->IsString()) {
    LOG_VERBOSE("Config <data_folder> is not a string. Use default current path!");
  } else {
    dir = p->GetString();
  }
  data_path_ = dir;

  main_img_filename_ = "img.jpg";
  p = Pointer("/main_image_name").Get(d);
  if (p == nullptr) {
    LOG_VERBOSE("Config missing <main_image_name>. Use default %s", main_img_filename_.c_str());
  } else if (!p->IsString()) {
    LOG_VERBOSE("Config <main_image_name> is not a string. Use default %s", main_img_filename_.c_str());
  } else {
    main_img_filename_ = p->GetString();
  }
}


void ProjectContext::ParseSunSettings(rapidjson::Document& d) {
  sun_ctx_ = SunContext::CreateDefault();
  auto root = Pointer("/sun").Get(d);
  if (!root) {
    LOG_VERBOSE("Config <sun> is missing. Use default!");
  } else {
    sun_ctx_->LoadFromJson(*root);
  }
}


void ProjectContext::ParseCameraSettings(rapidjson::Document& d) {
  cam_ctx_ = CameraContext::CreateDefault();
  auto root = Pointer("/camera").Get(d);
  if (!root) {
    LOG_VERBOSE("Config <camera> is missing. Use default!");
  } else {
    cam_ctx_->LoadFromJson(*root);
  }
}


void ProjectContext::ParseRenderSettings(rapidjson::Document& d) {
  render_ctx_ = RenderContext::CreateDefault();
  auto root = Pointer("/render").Get(d);
  if (!root) {
    LOG_VERBOSE("Config <render> is missing. Use default!");
  } else {
    render_ctx_->LoadFromJson(*root);
  }

  root = Pointer("/split_render").Get(d);
  if (!root) {
    LOG_VERBOSE("Config <split_render> is missing. Leave it empty!");
    split_render_ctx_ = nullptr;
  } else {
    split_render_ctx_ = RenderContext::CreateDefault();
    split_render_ctx_->LoadFromJson(*root);
  }
}


void ProjectContext::ParseCrystalSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  const auto* p = Pointer("/crystal").Get(d);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(buffer, kTmpBufferSize, "Missing <crystal>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  for (const auto& c : p->GetArray()) {
    crystal_store_.emplace_back(CrystalContext::CreateDefault());
    crystal_store_.back()->LoadFromJson(c);
  }

  PrintCrystalInfo();
}


void ProjectContext::ParseRayPathFilterSettings(rapidjson::Document& d) {
  constexpr size_t kTmpBufferSize = 512;
  char buffer[kTmpBufferSize];

  const auto* p = Pointer("/ray_path_filter").Get(d);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(buffer, kTmpBufferSize, "Missing <ray_path_filter>. Parsing fail!");
    throw std::invalid_argument(buffer);
  }

  for (const auto& c : p->GetArray()) {
    filter_store_.emplace_back(RayPathFilterContext::CreateDefault());
    filter_store_.back()->LoadFromJson(c);
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

  for (const auto& c : p->GetArray()) {
    multi_scatter_info_.emplace_back(MultiScatterContext::CreateDefault());
    multi_scatter_info_.back()->LoadFromJson(c);
  }
}


RayPath ProjectContext::GetRayPath(const RaySegment* last_ray) {
  RayPath result(kMaxRayHitNum + 2);
  auto p = last_ray;
  while (p) {
    result << kInvalidId;
    auto crystal = GetCrystal(p->root_ctx->crystal_id);

    while (p->prev) {
      result << crystal->FaceNumber(p->face_id);
      p = p->prev;
    }
    result << p->root_ctx->crystal_id;
    p = p->root_ctx->prev_ray_segment;
  }
  std::reverse(result.begin(), result.end());
  return result;
}


}  // namespace icehalo
