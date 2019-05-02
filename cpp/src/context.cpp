#include "context.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "optics.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"
#include "render.h"
#include "threadingpool.h"


namespace IceHalo {

using rapidjson::Pointer;

AbstractRayPathFilter::AbstractRayPathFilter()
    : symmetry_flag_(kSymmetryNone), complementary_(false), remove_homodromous_(false) {}


bool AbstractRayPathFilter::Filter(const CrystalPtr& crystal, RaySegment* last_r) const {
  if (remove_homodromous_ &&
      Math::Dot3(last_r->dir.val(), last_r->root_ctx->first_ray_segment->dir.val()) > 1.0 - 5 * Math::kFloatEps) {
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


void AbstractRayPathFilter::ApplySymmetry(const CrystalPtr& /* crystal */) {}


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


size_t AbstractRayPathFilter::RayPathHash(const std::vector<uint16_t>& ray_path, bool reverse) const {
  constexpr size_t kStep = 7;
  constexpr size_t kTotalBits = sizeof(size_t) * CHAR_BIT;

  size_t result = 0;
  size_t curr_offset = 0;
  if (reverse) {
    for (auto rit = ray_path.rbegin(); rit != ray_path.rend(); ++rit) {
      auto fn = *rit;
      size_t tmp_hash = (fn << curr_offset) | (fn >> (kTotalBits - curr_offset));
      result ^= tmp_hash;
      curr_offset += kStep;
      curr_offset %= kTotalBits;
    }
  } else {
    for (auto fn : ray_path) {
      size_t tmp_hash = (fn << curr_offset) | (fn >> (kTotalBits - curr_offset));
      result ^= tmp_hash;
      curr_offset += kStep;
      curr_offset %= kTotalBits;
    }
  }
  return result;
}


size_t AbstractRayPathFilter::RayPathHash(const CrystalPtr& crystal,               // used for get face number
                                          const RaySegment* last_ray, int length,  // ray path and length
                                          bool reverse) const {
  constexpr size_t kStep = 7;
  constexpr size_t kTotalBits = sizeof(size_t) * CHAR_BIT;

  size_t result = 0;
  size_t curr_offset = reverse ? kStep * (length - 1) % kTotalBits : 0;
  auto p = last_ray;
  while (p->prev) {
    auto fn = static_cast<uint16_t>(crystal->FaceNumber(p->face_id));
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


// void AbstractRayPathFilter::ApplyHash(const CrystalPtr& crystal) {
//   std::vector<std::vector<uint16_t>> augmented_ray_paths;
//
//   // Add the original path.
//   for (const auto& rp : ray_paths) {
//     augmented_ray_paths.emplace_back(rp);
//   }
//
//   // Add symmetry P.
//   auto period = crystal->GetFaceNumberPeriod();
//   std::vector<uint16_t> tmp_ray_path;
//   if (period > 0 && (symmetry_ & kSymmetryPrism)) {
//     std::vector<std::vector<uint16_t>> ray_paths_copy(augmented_ray_paths);
//     for (const auto& rp : ray_paths_copy) {
//       for (int i = 0; i < period; i++) {
//         tmp_ray_path.clear();
//         for (auto fn : rp) {
//           if (fn != 1 && fn != 2) {
//             fn = static_cast<uint16_t>((fn + period + i - 3) % period + 3);
//           }
//           tmp_ray_path.emplace_back(fn);
//         }
//         augmented_ray_paths.emplace_back(tmp_ray_path);
//       }
//     }
//   }
//
//   // Add symmetry B.
//   if (symmetry_ & kSymmetryBasal) {
//     std::vector<std::vector<uint16_t>> ray_paths_copy(augmented_ray_paths);
//     for (const auto& rp : ray_paths_copy) {
//       tmp_ray_path.clear();
//       for (auto fn : rp) {
//         if (fn == 1 || fn == 2) {
//           fn = static_cast<uint16_t>(fn % 2 + 1);
//         }
//         tmp_ray_path.emplace_back(fn);
//       }
//       augmented_ray_paths.emplace_back(tmp_ray_path);
//     }
//   }
//
//   // Add symmetry D.
//   if (period > 0 && (symmetry_ & kSymmetryDirection)) {
//     std::vector<std::vector<uint16_t>> ray_paths_copy(augmented_ray_paths);
//     for (const auto& rp : ray_paths_copy) {
//       tmp_ray_path.clear();
//       for (auto fn : rp) {
//         if (fn != 1 && fn != 2) {
//           fn = static_cast<uint16_t>(5 + period - fn);
//         }
//         tmp_ray_path.emplace_back(fn);
//       }
//       augmented_ray_paths.emplace_back(tmp_ray_path);
//     }
//   }
//
//   // Add them all.
//   ray_path_hashes_.clear();
//   for (const auto& rp : augmented_ray_paths) {
//     ray_path_hashes_.emplace(RayPathHash(rp));
//   }
// }


bool NoneRayPathFilter::FilterPath(const CrystalPtr& /* crystal */, RaySegment* /* r */) const {
  return true;
}


void SpecificRayPathFilter::AddPath(const std::vector<uint16_t>& path) {
  ray_paths_.emplace_back(path);
}


void SpecificRayPathFilter::ClearPaths() {
  ray_paths_.clear();
}


void SpecificRayPathFilter::ApplySymmetry(const IceHalo::CrystalPtr& crystal) {
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


bool SpecificRayPathFilter::FilterPath(const CrystalPtr& crystal, RaySegment* last_r) const {
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


bool GeneralRayPathFilter::FilterPath(const CrystalPtr& crystal, RaySegment* last_r) const {
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


MultiScatterContext::MultiScatterContext(float prob)
    : prob_(std::max(std::min(prob, 1.0f), 0.0f)) {}


float MultiScatterContext::GetProbability() const {
  return prob_;
}


bool MultiScatterContext::SetProbability(float p) {
  if (p < 0 || p > 1) {
    return false;
  } else {
    prob_ = p;
    return true;
  }
}


const std::vector<MultiScatterContext::CrystalInfo>& MultiScatterContext::GetCrystalInfo() const {
  return crystal_infos_;
}


void MultiScatterContext::ClearCrystalInfo() {
  crystal_infos_.clear();
}


void MultiScatterContext::AddCrystalInfo(int crystal_id,    // crystal ptr
                                         float population,  // crystal population, not normalized
                                         int filter_id) {   // ray path filter for this crystal
  crystal_infos_.emplace_back(CrystalInfo{ crystal_id, population, filter_id });
}


void MultiScatterContext::NormalizeCrystalPopulation() {
  float sum = 0;
  for (const auto& c : crystal_infos_) {
    sum += c.population;
  }
  for (auto& c : crystal_infos_) {
    c.population /= sum;
  }
}


SunContext::SunContext(float altitude, float diameter)
    : sun_diameter_(diameter), sun_altitude_(altitude), sun_position_{ 0.0f, -std::cos(altitude * Math::kDegreeToRad),
                                                                       -std::sin(altitude * Math::kDegreeToRad) } {}


const float* SunContext::GetSunPosition() const {
  return sun_position_;
}


float SunContext::GetSunAltitude() const {
  return sun_altitude_;
}


bool SunContext::SetSunAltitude(float altitude) {
  if (altitude < -90 || altitude > 90) {
    return false;
  } else {
    sun_altitude_ = altitude;
    sun_position_[0] = 0.0f;
    sun_position_[1] = -std::cos(altitude * Math::kDegreeToRad);
    sun_position_[2] = -std::sin(altitude * Math::kDegreeToRad);
    return true;
  }
}


float SunContext::GetSunDiameter() const {
  return sun_diameter_;
}


bool SunContext::SetSunDiameter(float d) {
  if (d < 0 || d > kMaxDiameter) {
    return false;
  } else {
    sun_diameter_ = d;
    return true;
  }
}


CameraContext::CameraContext() : target_dir_{ 0, 0, 0 }, fov_(0), lens_type_(LensType::kLinear) {}


constexpr float CameraContext::kMinAngleRound;
constexpr float CameraContext::kMaxAngleRound;
constexpr float CameraContext::kMinAngleTilt;
constexpr float CameraContext::kMaxAngleTilt;
constexpr float CameraContext::kMinAngleHeading;
constexpr float CameraContext::kMaxAngleHeading;

constexpr float CameraContext::kMaxFovLinear;
constexpr float CameraContext::kMaxFovFisheye;

constexpr float CameraContext::kDefaultCamAzimuth;
constexpr float CameraContext::kDefaultCamElevation;
constexpr float CameraContext::kDefaultCamRoll;


const float* CameraContext::GetCameraTargetDirection() const {
  return target_dir_;
}


void CameraContext::SetCameraTargetDirection(float azimuth, float altitude, float roll) {
  azimuth = std::min(std::max(azimuth, kMinAngleRound), kMaxAngleRound);
  altitude = std::min(std::max(altitude, kMinAngleTilt), kMaxAngleTilt);
  roll = std::min(std::max(roll, kMinAngleHeading), kMaxAngleHeading);

  target_dir_[0] = azimuth;
  target_dir_[1] = altitude;
  target_dir_[2] = roll;
}


void CameraContext::ResetCameraTargetDirection() {
  target_dir_[0] = kDefaultCamAzimuth;
  target_dir_[1] = kDefaultCamElevation;
  target_dir_[2] = kDefaultCamRoll;
}


float CameraContext::GetFov() const {
  return fov_;
}


void CameraContext::SetFov(float fov) {
  switch (lens_type_) {
    case LensType::kLinear:
      fov = std::max(std::min(fov, kMaxFovLinear), 0.0f);
      break;
    case LensType::kEqualArea:
      fov = std::max(std::min(fov, kMaxFovFisheye), 0.0f);
      break;
    default:
      fov = 0.0f;
      break;
  }
  fov_ = fov;
}


LensType CameraContext::GetLensType() const {
  return lens_type_;
}


void CameraContext::SetLensType(IceHalo::LensType type) {
  lens_type_ = type;
  switch (lens_type_) {
    case LensType::kEqualArea:
      fov_ = std::min(fov_, kMaxFovFisheye);
      break;
    case LensType::kLinear:
      fov_ = std::min(fov_, kMaxFovLinear);
      break;
    default:
      break;
  }
}


RenderContext::RenderContext()
    : ray_color_{ 1.0f, 1.0f, 1.0f }, background_color_{ 0.0f, 0.0f, 0.0f }, intensity_(1.0f), image_width_(0),
      image_height_(0), offset_x_(0), offset_y_(0), visible_range_(VisibleRange::kUpper) {}


constexpr float RenderContext::kMinIntensity;
constexpr float RenderContext::kMaxIntensity;
constexpr int RenderContext::kMaxImageSize;


const float* RenderContext::GetRayColor() const {
  return ray_color_;
}


void RenderContext::SetRayColor(float r, float g, float b) {
  ray_color_[0] = std::min(std::max(r, 0.0f), 1.0f);
  ray_color_[1] = std::min(std::max(g, 0.0f), 1.0f);
  ray_color_[2] = std::min(std::max(b, 0.0f), 1.0f);
}


void RenderContext::ResetRayColor() {
  UseRealRayColor();
}


void RenderContext::UseRealRayColor() {
  ray_color_[0] = -1.0f;
  ray_color_[1] = -1.0f;
  ray_color_[2] = -1.0f;
}


const float* RenderContext::GetBackgroundColor() const {
  return background_color_;
}


void RenderContext::SetBackgroundColor(float r, float g, float b) {
  background_color_[0] = std::min(std::max(r, 0.0f), 1.0f);
  background_color_[1] = std::min(std::max(g, 0.0f), 1.0f);
  background_color_[2] = std::min(std::max(b, 0.0f), 1.0f);
}


void RenderContext::ResetBackgroundColor() {
  background_color_[0] = 0.0f;
  background_color_[1] = 0.0f;
  background_color_[2] = 0.0f;
}


void RenderContext::UseSkyBackground() {
  background_color_[0] = -1.0f;
  background_color_[1] = -1.0f;
  background_color_[2] = -1.0f;
}


float RenderContext::GetIntensity() const {
  return intensity_;
}


void RenderContext::SetIntensity(float intensity) {
  intensity_ = std::min(std::max(intensity, kMinIntensity), kMaxIntensity);
}


int RenderContext::GetImageWidth() const {
  return image_width_;
}


int RenderContext::GetImageHeight() const {
  return image_height_;
}


void RenderContext::SetImageWidth(int w) {
  image_width_ = std::min(std::max(w, 0), kMaxImageSize);
}


void RenderContext::SetImageHeight(int h) {
  image_height_ = std::min(std::max(h, 0), kMaxImageSize);
}


int RenderContext::GetImageOffsetX() const {
  return offset_x_;
}


int RenderContext::GetImageOffsetY() const {
  return offset_y_;
}


void RenderContext::SetImageOffsetX(int offset_x) {
  offset_x_ = offset_x;
}


void RenderContext::SetImageOffsetY(int offset_y) {
  offset_y_ = offset_y;
}


VisibleRange RenderContext::GetVisibleRange() const {
  return visible_range_;
}


void RenderContext::SetVisibleRange(IceHalo::VisibleRange r) {
  visible_range_ = r;
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

  proj->ParseSunSettings(d);
  proj->ParseRaySettings(d);
  proj->ParseCameraSettings(d);
  proj->ParseRenderSettings(d);
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
  init_ray_num_ = std::max(ray_num, kMinInitRayNum);
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


void ProjectContext::ClearCrystals() {
  crystal_store_.clear();
}


void ProjectContext::SetCrystal(int id, CrystalPtrU&& crystal) {
  SetCrystal(id, std::move(crystal), AxisDistribution{});
}


void ProjectContext::SetCrystal(int id, CrystalPtrU&& crystal, const AxisDistribution& axis) {
  crystal_store_.emplace(id, std::make_shared<CrystalContext>(std::move(crystal), axis));
}


void ProjectContext::RemoveCrystal(int id) {
  crystal_store_.erase(id);
}


const CrystalContextPtr ProjectContext::GetCrystalContext(int id) const {
  if (crystal_store_.count(id)) {
    return crystal_store_.at(id);
  } else {
    return nullptr;
  }
}


const CrystalPtr ProjectContext::GetCrystal(int id) const {
  if (crystal_store_.count(id)) {
    return crystal_store_.at(id)->crystal;
  } else {
    return nullptr;
  }
}


void ProjectContext::PrintCrystalInfo() const {
  for (const auto& c : crystal_store_) {
    auto g = c.second->crystal;
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


void ProjectContext::ClearRayPathFilter() {
  filter_store_.clear();
}


void ProjectContext::SetRayPathFilter(int id, const RayPathFilterPtr& filter) {
  filter_store_.emplace(id, filter);
}


const RayPathFilterPtr ProjectContext::GetRayPathFilter(int id) const {
  if (filter_store_.count(id)) {
    return filter_store_.at(id);
  } else {
    return nullptr;
  }
}


ProjectContext::ProjectContext()
    : sun_ctx_(SunContext::kDefaultAltitude), cam_ctx_{}, render_ctx_{}, init_ray_num_(kDefaultInitRayNum),
      ray_hit_num_(kDefaultRayHitNum), model_path_("") {}


void ProjectContext::ParseSunSettings(rapidjson::Document& d) {
  float sun_altitude = 0.0f;
  auto* p = Pointer("/sun/altitude").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <sun.altitude>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <sun.altitude> is not a number, using default 0.0!\n");
  } else {
    sun_altitude = static_cast<float>(p->GetDouble());
  }
  sun_ctx_.SetSunAltitude(sun_altitude);

  p = Pointer("/sun/diameter").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <sun.diameter>, using default 0.5!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <sun.diameter> is not a number, using default 0.5!\n");
  } else {
    sun_ctx_.SetSunDiameter(static_cast<float>(p->GetDouble()));
  }
}


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
      tmp_wavelengths.push_back(static_cast<float&&>(pi.GetDouble()));
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
      tmp_weights.push_back(static_cast<float&&>(pi.GetDouble()));
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


void ProjectContext::ParseCameraSettings(rapidjson::Document& d) {
  cam_ctx_.ResetCameraTargetDirection();
  cam_ctx_.SetFov(CameraContext::kMaxFovFisheye);
  cam_ctx_.SetLensType(LensType::kEqualArea);

  float cam_az = CameraContext::kDefaultCamAzimuth;
  float cam_el = CameraContext::kDefaultCamElevation;
  float cam_ro = CameraContext::kDefaultCamRoll;

  auto* p = Pointer("/camera/azimuth").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.azimuth>, using default %.1f!\n",
                 CameraContext::kDefaultCamAzimuth);
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.azimuth> is not a number, using default %.1f!\n",
                 CameraContext::kDefaultCamAzimuth);
  } else {
    cam_az = static_cast<float>(p->GetDouble());
    cam_az = std::max(std::min(cam_az, CameraContext::kMaxAngleRound), CameraContext::kMinAngleRound);
    cam_az = 90.0f - cam_az;
  }

  p = Pointer("/camera/elevation").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.elevation>, using default %.1f!\n",
                 CameraContext::kDefaultCamElevation);
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.elevation> is not a number, using default %.1f!\n",
                 CameraContext::kDefaultCamElevation);
  } else {
    cam_el = static_cast<float>(p->GetDouble());
    cam_el = std::max(std::min(cam_el, CameraContext::kMaxAngleTilt), CameraContext::kMinAngleTilt);
  }

  p = Pointer("/camera/rotation").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.rotation>, using default %.1f!\n",
                 CameraContext::kDefaultCamRoll);
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.rotation> is not a number, using default %.1f!\n",
                 CameraContext::kDefaultCamRoll);
  } else {
    cam_ro = static_cast<float>(p->GetDouble());
    cam_ro = std::max(std::min(cam_ro, CameraContext::kMaxAngleHeading), CameraContext::kMinAngleHeading);
  }

  cam_ctx_.SetCameraTargetDirection(cam_az, cam_el, cam_ro);

  p = Pointer("/camera/lens").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.lens>, using default equal-area fisheye!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! config <camera.lens> is not a string, using default equal-area fisheye!\n");
  } else {
    if (*p == "linear") {
      cam_ctx_.SetLensType(LensType::kLinear);
    } else if (*p == "fisheye") {
      cam_ctx_.SetLensType(LensType::kEqualArea);
    } else if (*p == "dual_fisheye_equidistant") {
      cam_ctx_.SetLensType(LensType::kDualEquidistant);
    } else if (*p == "dual_fisheye_equiarea") {
      cam_ctx_.SetLensType(LensType::kDualEqualArea);
    } else {
      std::fprintf(stderr, "\nWARNING! config <camera.lens> cannot be recognized, using default equal-area fisheye!\n");
    }
  }

  p = Pointer("/camera/fov").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.fov>, using default %.1f!\n", cam_ctx_.GetFov());
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.fov> is not a number, using default %.1f!\n", cam_ctx_.GetFov());
  } else {
    cam_ctx_.SetFov(static_cast<float>(p->GetDouble()));
  }
}


void ProjectContext::ParseRenderSettings(rapidjson::Document& d) {
  render_ctx_.SetImageWidth(800);
  auto p = Pointer("/render/width").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.width>, using default 800!\n");
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <camera.width> is not an integer, using default 800!\n");
  } else {
    render_ctx_.SetImageWidth(p->GetInt());
  }

  render_ctx_.SetImageHeight(800);
  p = Pointer("/render/height").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.height>, using default 800!\n");
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <camera.height> is not an integer, using default 800!\n");
  } else {
    render_ctx_.SetImageHeight(p->GetInt());
  }

  render_ctx_.SetVisibleRange(VisibleRange::kUpper);
  p = Pointer("/render/visible_semi_sphere").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.visible_semi_sphere>, using default kUpper!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> is not a string, using default kUpper!\n");
  } else if (*p == "upper") {
    render_ctx_.SetVisibleRange(VisibleRange::kUpper);
  } else if (*p == "lower") {
    render_ctx_.SetVisibleRange(VisibleRange::kLower);
  } else if (*p == "camera") {
    render_ctx_.SetVisibleRange(VisibleRange::kFront);
  } else if (*p == "full") {
    render_ctx_.SetVisibleRange(VisibleRange::kFull);
  } else {
    std::fprintf(stderr,
                 "\nWARNING! Config <render.visible_semi_sphere> cannot be recognized, using default kUpper!\n");
  }

  render_ctx_.SetIntensity(1.0f);
  p = Pointer("/render/intensity_factor").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.intensity_factor>, using default 1.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <render.intensity_factor> is not a number, using default 1.0!\n");
  } else {
    auto f = static_cast<float>(p->GetDouble());
    f = std::max(std::min(f, RenderContext::kMaxIntensity), RenderContext::kMinIntensity);
    render_ctx_.SetIntensity(f);
  }

  render_ctx_.SetImageOffsetX(0);
  render_ctx_.SetImageOffsetY(0);
  p = Pointer("/render/offset").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.offset>, using default [0, 0]!\n");
  } else if (!p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <render.offset> is not an array, using default [0, 0]!\n");
  } else if (p->Size() != 2 || !(*p)[0].IsInt() || !(*p)[1].IsInt()) {
    std::fprintf(stderr, "\nWARNING! Config <render.offset> cannot be recognized, using default [0, 0]!\n");
  } else {
    int offset_x = (*p)[0].GetInt();
    int offset_y = (*p)[1].GetInt();
    offset_x = std::max(std::min(offset_x, static_cast<int>(RenderContext::kMaxImageSize / 2)),
                        -static_cast<int>(RenderContext::kMaxImageSize / 2));
    offset_y = std::max(std::min(offset_y, static_cast<int>(RenderContext::kMaxImageSize / 2)),
                        -static_cast<int>(RenderContext::kMaxImageSize / 2));
    render_ctx_.SetImageOffsetX(offset_x);
    render_ctx_.SetImageOffsetY(offset_y);
  }

  render_ctx_.ResetBackgroundColor();
  p = Pointer("/render/background_color").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default [0,0,0]!\n");
  } else if (!p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array, using default [0,0,0]!\n");
  } else if (p->Size() != 3 || !(*p)[0].IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default [0,0,0]!\n");
  } else {
    auto pa = p->GetArray();
    float r = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float g = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float b = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    render_ctx_.SetBackgroundColor(r, g, b);
  }

  render_ctx_.ResetRayColor();
  p = Pointer("/render/ray_color").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.ray_color>, using default real color!\n");
  } else if (!p->IsArray() && !p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <render.ray_color> is not an array nor a string, ");
    std::fprintf(stderr, "using default real color!\n");
  } else if (p->IsArray() && (p->Size() != 3 || !(*p)[0].IsNumber())) {
    std::fprintf(stderr, "\nWARNING! Config <render.ray_color> cannot be recognized, using default real color!\n");
  } else if (p->IsString() && (*p) != "real") {
    std::fprintf(stderr, "\nWARNING! Config <render.ray_color> cannot be recognized, using default real color!\n");
  } else if (p->IsArray()) {
    auto pa = p->GetArray();
    float r = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float g = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float b = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    render_ctx_.SetRayColor(r, g, b);
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


std::unordered_map<std::string, ProjectContext::CrystalParser>& ProjectContext::GetCrystalParsers() {
  static std::unordered_map<std::string, CrystalParser> crystal_parsers = {
    { "HexPrism", &ProjectContext::ParseCrystalHexPrism },
    { "HexPyramid", &ProjectContext::ParseCrystalHexPyramid },
    { "HexPyramidStackHalf", &ProjectContext::ParseCrystalHexPyramidStackHalf },
    { "CubicPyramid", &ProjectContext::ParseCrystalCubicPyramid },
    { "IrregularHexPrism", &ProjectContext::ParseCrystalIrregularHexPrism },
    { "IrregularHexPyramid", &ProjectContext::ParseCrystalIrregularHexPyramid },
    { "Custom", &ProjectContext::ParseCrystalCustom },
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

  auto& crystal_parsers = GetCrystalParsers();
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
  crystal_store_.emplace(id, std::make_shared<CrystalContext>(crystal_parsers[type](this, c, ci), axis));
}


AxisDistribution ProjectContext::ParseCrystalAxis(const rapidjson::Value& c, int ci) {
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


CrystalPtrU ProjectContext::ParseCrystalCustom(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal[%d].parameter> cannot recognize!", ci);
    throw std::invalid_argument(msg_buffer);
  } else {
    auto n = model_path_.rfind('/');
    if (n == std::string::npos) {
      std::snprintf(msg_buffer, kMsgBufferSize, "models/%s", p->GetString());
    } else {
      std::snprintf(msg_buffer, kMsgBufferSize, "%s/models/%s", model_path_.substr(0, n).c_str(), p->GetString());
    }
    std::FILE* file = std::fopen(msg_buffer, "r");
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

  filter_store_.emplace(id, filter_parsers[type](this, c, ci));
}


void ProjectContext::ParseFilterBasic(const rapidjson::Value& c, int ci, RayPathFilterPtr filter) {
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


RayPathFilterPtr ProjectContext::ParseFilterNone(const rapidjson::Value& c, int ci) {
  auto filter = std::make_shared<NoneRayPathFilter>();
  ParseFilterBasic(c, ci, filter);
  return filter;
}


RayPathFilterPtr ProjectContext::ParseFilterSpecific(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto filter = std::make_shared<SpecificRayPathFilter>();
  ParseFilterBasic(c, ci, filter);

  filter->ClearPaths();
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
    filter->AddPath(tmp_path);
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
        filter->AddPath(tmp_path);
      }
    }
  }
  return filter;
}


RayPathFilterPtr ProjectContext::ParseFilterGeneral(const rapidjson::Value& c, int ci) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto filter = std::make_shared<GeneralRayPathFilter>();
  ParseFilterBasic(c, ci, filter);
  filter->ClearHitNumbers();
  filter->ClearFaces();

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
    filter->AddEntryFace(pi.GetInt());
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
    filter->AddExitFace(pi.GetInt());
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
      filter->AddHitNumber(pi.GetInt());
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

  for (decltype(tmp_crystals.size()) i = 0; i < tmp_crystals.size(); i++) {
    scatter.AddCrystalInfo(tmp_crystals[i], tmp_population[i], tmp_filter[i]);
  }

  scatter.NormalizeCrystalPopulation();

  multi_scatter_info_.emplace_back(scatter);
}


CrystalContext::CrystalContext(CrystalPtrU&& g, const AxisDistribution& axis) : crystal(std::move(g)), axis(axis) {}


CrystalContext::CrystalContext(const CrystalContext& other) = default;


RayInfo::RayInfo(RaySegment* seg, CrystalContextPtr crystal_ctx, const float* main_axis_rot)
    : first_ray_segment(seg), prev_ray_segment(nullptr), crystal_ctx(std::move(crystal_ctx)),
      main_axis_rot(main_axis_rot) {}

}  // namespace IceHalo
