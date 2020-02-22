#include "context/context.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "core/optics.hpp"
#include "core/render.hpp"
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
  LOG_INFO("Reading config from: %s", filename);

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


std::string ProjectContext::GetDefaultImagePath() const {
  return PathJoin(data_path_, "img.jpg");
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


#ifdef FOR_TEST
void ProjectContext::PrintCrystalInfo() const {
  for (const auto& ctx : crystal_store_) {
    auto g = ctx->GetCrystal();
    LOG_DEBUG("-- ID: %d --", ctx->GetId());
    for (const auto& v : g->GetVertexes()) {
      LOG_DEBUG("v %+.4f %+.4f %+.4f", v.x(), v.y(), v.z());
    }
    for (const auto& f : g->GetFaces()) {
      auto idx = f.idx();
      LOG_DEBUG("f %d %d %d", idx[0] + 1, idx[1] + 1, idx[2] + 1);
    }
  }
}
#endif


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
    LOG_INFO("Config missing <ray.number>. Use default %d!", ProjectContext::kMinRayHitNum);
  } else if (!p->IsUint()) {
    LOG_INFO("Config <ray.number> is not unsigned int. Use default %d!", ProjectContext::kMinRayHitNum);
  } else {
    SetInitRayNum(p->GetUint());
  }

  p = Pointer("/max_recursion").Get(d);
  if (p == nullptr) {
    LOG_INFO("Config missing <max_recursion>. Use default %d!", ProjectContext::kMinRayHitNum);
  } else if (!p->IsInt()) {
    LOG_INFO("Config <max_recursion> is not an integer. Use default %d!", ProjectContext::kMinRayHitNum);
  } else {
    SetRayHitNum(p->GetInt());
  }

  std::vector<int> tmp_wavelengths{ 550 };
  auto wl_p = Pointer("/ray/wavelength").Get(d);
  if (wl_p == nullptr) {
    LOG_INFO("Config missing <ray.wavelength>. Use default 550!");
  } else if (!wl_p->IsArray()) {
    LOG_INFO("Config <ray.wavelength> is not an array. Use default 550!");
  } else if (!(*wl_p)[0].IsInt()) {
    LOG_INFO("Config <ray.wavelength> cannot be recognized, using default 550!");
  } else {
    tmp_wavelengths.clear();
    for (const auto& pi : wl_p->GetArray()) {
      tmp_wavelengths.emplace_back(static_cast<float>(pi.GetDouble()));
    }
  }

  std::vector<float> tmp_weights{ 1.0f };
  auto wt_p = Pointer("/ray/weight").Get(d);
  if (wt_p == nullptr) {
    LOG_INFO("Config missing <ray.weight>. Use default 1.0!");
  } else if (!wt_p->IsArray()) {
    LOG_INFO("Config <ray.wavelength> is not an array. Use default 1.0!");
  } else if (!(*wt_p)[0].IsNumber()) {
    LOG_INFO("Config <ray.wavelength> cannot be recognized. Use default 1.0!");
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
    LOG_INFO("Config missing <data_folder>. Use default current path!");
  } else if (!p->IsString()) {
    LOG_INFO("Config <data_folder> is not a string. Use default current path!");
  } else {
    dir = p->GetString();
  }
  data_path_ = dir;
}


void ProjectContext::ParseSunSettings(rapidjson::Document& d) {
  sun_ctx_ = SunContext::CreateDefault();
  auto root = Pointer("/sun").Get(d);
  if (!root) {
    LOG_INFO("Config <sun> is missing. Use default!");
  } else {
    sun_ctx_->LoadFromJson(*root);
  }
}


void ProjectContext::ParseCameraSettings(rapidjson::Document& d) {
  cam_ctx_ = CameraContext::CreateDefault();
  auto root = Pointer("/camera").Get(d);
  if (!root) {
    LOG_INFO("Config <camera> is missing. Use default!");
  } else {
    cam_ctx_->LoadFromJson(*root);
  }
}


void ProjectContext::ParseRenderSettings(rapidjson::Document& d) {
  render_ctx_ = RenderContext::CreateDefault();
  auto root = Pointer("/render").Get(d);
  if (!root) {
    LOG_INFO("Config <render> is missing. Use default!");
  } else {
    render_ctx_->LoadFromJson(*root);
  }

  split_render_ctx_ = RenderContext::CreateDefault();
  root = Pointer("/split_render").Get(d);
  if (!root) {
    LOG_INFO("Config <top_halo_render> is missing. Use default!");
  } else {
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


RayPath GetRayPath(const ProjectContextPtr& ctx, const RaySegment* last_ray) {
  RayPath result{};
  auto p = last_ray;
  while (p) {
    result.emplace_back(kInvalidFaceNumber);
    auto crystal = ctx->GetCrystal(p->root_ctx->crystal_id);

    while (p->prev) {
      result.emplace_back(crystal->FaceNumber(p->face_id));
      p = p->prev;
    }
    result.emplace_back(p->root_ctx->crystal_id);
    p = p->root_ctx->prev_ray_segment;
  }
  std::reverse(result.begin(), result.end());
  return result;
}


}  // namespace icehalo
