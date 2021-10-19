#include "context/context.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "core/optics.hpp"
#include "process/render.hpp"
#include "util/log.hpp"


namespace icehalo {

ProjectContextPtrU ProjectContext::CreateFromFile(const char* filename) {
  LOG_VERBOSE("Reading config from: %s", filename);

  std::ifstream input_stream(filename);
  nlohmann::json json_obj;
  input_stream >> json_obj;

  ProjectContextPtrU proj = CreateDefault();

  proj->ParseBasicSettings(json_obj);
  proj->ParseSunSettings(json_obj);
  proj->ParseCameraSettings(json_obj);
  proj->ParseRenderSettings(json_obj);
  proj->ParseCrystalSettings(json_obj);
  proj->ParseRayPathFilterSettings(json_obj);
  proj->ParseMultiScatterSettings(json_obj);

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
  init_ray_num_ = ray_num;
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


ProjectContext::ProjectContext() : init_ray_num_(kDefaultInitRayNum), ray_hit_num_(kDefaultRayHitNum) {}


void ProjectContext::ParseBasicSettings(const nlohmann::json& obj) {
  SetInitRayNum(obj.at("ray").at("number").get<uint64_t>());
  SetRayHitNum(obj.at("max_recursion").get<int>());

  const auto& ray_obj = obj.at("ray");
  if (!ray_obj.at("wavelength").is_array() || !ray_obj.at("weight").is_array() ||
      ray_obj.at("wavelength").size() != ray_obj.at("weight").size()) {
    throw nlohmann::detail::other_error::create(-1, "wavelength and weight must be arrays with same length!", obj);
  }

  wavelengths_.clear();
  for (size_t i = 0; i < ray_obj.at("wavelength").size(); i++) {
    auto l = ray_obj.at("wavelength")[i].get<int>();
    auto w = ray_obj.at("weight")[i].get<float>();
    wavelengths_.emplace_back(WavelengthInfo{ l, w });
  }

  data_path_ = boost::filesystem::current_path().string();
  try {
    obj.at("data_folder").get_to(data_path_);
  } catch (...) {
    LOG_VERBOSE("cannot parse data_folder. use default: %s", data_path_.c_str());
  }
  main_img_filename_ = "img.jpg";
  try {
    obj.at("main_image_name").get_to(main_img_filename_);
  } catch (...) {
    LOG_VERBOSE("cannot parse main_image_name. use default: %s", main_img_filename_.c_str());
  }
}


void ProjectContext::ParseSunSettings(const nlohmann::json& obj) {
  sun_ctx_ = SunContext::CreateDefault();
  obj.at("sun").get_to(*sun_ctx_);
}


void ProjectContext::ParseCameraSettings(const nlohmann::json& obj) {
  cam_ctx_ = CameraContext::CreateDefault();
  obj.at("camera").get_to(*cam_ctx_);
}


void ProjectContext::ParseRenderSettings(const nlohmann::json& obj) {
  render_ctx_ = RenderContext::CreateDefault();
  obj.at("render").get_to(*render_ctx_);

  if (obj.contains("split_render")) {
    split_render_ctx_ = RenderContext::CreateDefault();
    obj.at("split_render").get_to(*split_render_ctx_);
  } else {
    split_render_ctx_ = nullptr;
  }
}


void ProjectContext::ParseCrystalSettings(const nlohmann::json& obj) {
  if (!obj.at("crystal").is_array()) {
    throw nlohmann::detail::other_error::create(-1, "crystal should be an array!", obj);
  }

  for (const auto& c : obj.at("crystal")) {
    auto crystal = CrystalContext::CreateDefault();
    c.get_to(*crystal);
    crystal_store_.emplace_back(std::move(crystal));
  }

  PrintCrystalInfo();
}


void ProjectContext::ParseRayPathFilterSettings(const nlohmann::json& obj) {
  if (!obj.at("ray_path_filter").is_array()) {
    throw nlohmann::detail::other_error::create(-1, "ray_path_filter should be an array!", obj);
  }

  for (const auto& f : obj.at("ray_path_filter")) {
    auto filter = RayPathFilterContext::CreateDefault();
    f.get_to(*filter);
    filter_store_.emplace_back(std::move(filter));
  }
}


void ProjectContext::ParseMultiScatterSettings(const nlohmann::json& obj) {
  if (!obj.at("multi_scatter").is_array()) {
    throw nlohmann::detail::other_error::create(-1, "multi_scatter should be an array!", obj);
  }

  for (const auto& s : obj.at("multi_scatter")) {
    auto scatter = MultiScatterContext::CreateDefault();
    s.get_to(*scatter);
    multi_scatter_info_.emplace_back(std::move(scatter));
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
