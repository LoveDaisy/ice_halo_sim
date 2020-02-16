#include "context/context.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "core/optics.h"
#include "core/render.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"


namespace icehalo {

using rapidjson::Pointer;

constexpr float ProjectContext::kPropMinW;
constexpr float ProjectContext::kScatMinW;
constexpr size_t ProjectContext::kMinInitRayNum;
constexpr int ProjectContext::kMinRayHitNum;
constexpr int ProjectContext::kMaxRayHitNum;


ProjectContextPtrU ProjectContext::CreateFromFile(const char* filename) {
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


const Crystal* ProjectContext::GetCrystal(int id) const {
  auto crystal_ctx = GetCrystalContext(id);
  if (crystal_ctx) {
    return crystal_ctx->GetCrystal();
  } else {
    return nullptr;
  }
}


int32_t ProjectContext::GetCrystalId(const Crystal* crystal) const {
  for (const auto& ctx : crystal_store_) {
    if (ctx->GetCrystal() == crystal) {
      return ctx->GetId();
    }
  }
  return kInvalidId;
}


const CrystalContext* ProjectContext::GetCrystalContext(int id) const {
  for (const auto& ctx : crystal_store_) {
    if (ctx->GetId() == id) {
      return ctx.get();
    }
  }
  return nullptr;
}


#ifdef FOR_TEST
void ProjectContext::PrintCrystalInfo() const {
  for (const auto& ctx : crystal_store_) {
    auto g = ctx->GetCrystal();
    std::printf("-- ID: %d --\n", ctx->GetId());
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

  std::string dir = boost::filesystem::current_path().string();
  p = Pointer("/data_folder").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <data_folder>, using default current path!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <data_folder> is not a string, using default current path!\n");
  } else {
    dir = p->GetString();
  }
  data_path_ = dir;
}


void ProjectContext::ParseSunSettings(rapidjson::Document& d) {
  sun_ctx_ = SunContext::CreateDefault();
  auto root = Pointer("/sun").Get(d);
  if (!root) {
    std::fprintf(stderr, "\nWARNING! Config <sun> is missing. Use default!\n");
  } else {
    sun_ctx_->LoadFromJson(*root);
  }
}


void ProjectContext::ParseCameraSettings(rapidjson::Document& d) {
  cam_ctx_ = CameraContext::CreateDefault();
  auto root = Pointer("/camera").Get(d);
  if (!root) {
    std::fprintf(stderr, "\nWARNING! Config <camera> is missing. Use default!\n");
  } else {
    cam_ctx_->LoadFromJson(*root);
  }
}


void ProjectContext::ParseRenderSettings(rapidjson::Document& d) {
  render_ctx_ = RenderContext::CreateDefault();
  auto root = Pointer("/render").Get(d);
  if (!root) {
    std::fprintf(stderr, "\nWARNING! Config <render> is missing. Use default!\n");
  } else {
    render_ctx_->LoadFromJson(*root);
  }

  top_halo_render_ctx_ = RenderContext::CreateDefault();
  root = Pointer("/top_halo_render").Get(d);
  if (!root) {
    std::fprintf(stderr, "\nWARNING! Config <top_halo_render> is missing. Use default!\n");
  } else {
    top_halo_render_ctx_->LoadFromJson(*root);
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

}  // namespace icehalo
