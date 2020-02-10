#include "context/camera_context.h"

#include <algorithm>

#include "core/render.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"


namespace icehalo {

using rapidjson::Pointer;

CameraContext::CameraContext() : target_dir_{ 0, 0, 0 }, fov_(0), lens_type_(LensType::kLinear) {}


CameraContextPtrU CameraContext::CreateFromJson(rapidjson::Document& d) {
  CameraContextPtrU cam_ctx{ new CameraContext };
  rapidjson::Value* cam_dom = Pointer("/camera").Get(d);
  cam_ctx->LoadFromJson(*cam_dom);
  return cam_ctx;
}


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
    case LensType::kEquidistant:
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


void CameraContext::SetLensType(LensType type) {
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


void CameraContext::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  root.Clear();
  Pointer("/azimuth").Set(root, target_dir_[0], allocator);
  Pointer("/elevation").Set(root, target_dir_[1], allocator);
  Pointer("/rotation").Set(root, target_dir_[2], allocator);
  Pointer("/fov").Set(root, fov_, allocator);
  auto p = Pointer("/lens");
  switch (lens_type_) {
    case LensType::kLinear:
      p.Set(root, "linear", allocator);
      break;
    case LensType::kEquidistant:
      p.Set(root, "fisheye_equidistant", allocator);
      break;
    case LensType::kDualEqualArea:
      p.Set(root, "dual_fisheye_equalarea", allocator);
      break;
    case LensType::kDualEquidistant:
      p.Set(root, "dual_fisheye_equidistant", allocator);
      break;
    case LensType::kEqualArea:
    default:
      p.Set(root, "fisheye_equalarea", allocator);
      break;
  }
}


void CameraContext::LoadFromJson(const rapidjson::Value& root) {
  ResetCameraTargetDirection();
  SetFov(CameraContext::kMaxFovFisheye);
  SetLensType(LensType::kEqualArea);

  float cam_az = CameraContext::kDefaultCamAzimuth;
  float cam_el = CameraContext::kDefaultCamElevation;
  float cam_ro = CameraContext::kDefaultCamRoll;

  auto* p = Pointer("/azimuth").Get(root);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Camera config missing <azimuth>, using default %.1f!\n",
                 CameraContext::kDefaultCamAzimuth);
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Camera config <azimuth> is not a number, using default %.1f!\n",
                 CameraContext::kDefaultCamAzimuth);
  } else {
    cam_az = static_cast<float>(p->GetDouble());
    cam_az = std::max(std::min(cam_az, CameraContext::kMaxAngleRound), CameraContext::kMinAngleRound);
    cam_az = 90.0f - cam_az;
  }

  p = Pointer("/elevation").Get(root);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Camera config missing <elevation>, using default %.1f!\n",
                 CameraContext::kDefaultCamElevation);
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Camera config <elevation> is not a number, using default %.1f!\n",
                 CameraContext::kDefaultCamElevation);
  } else {
    cam_el = static_cast<float>(p->GetDouble());
    cam_el = std::max(std::min(cam_el, CameraContext::kMaxAngleTilt), CameraContext::kMinAngleTilt);
  }

  p = Pointer("/rotation").Get(root);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Camera config missing <rotation>, using default %.1f!\n",
                 CameraContext::kDefaultCamRoll);
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Camera config <rotation> is not a number, using default %.1f!\n",
                 CameraContext::kDefaultCamRoll);
  } else {
    cam_ro = static_cast<float>(p->GetDouble());
    cam_ro = std::max(std::min(cam_ro, CameraContext::kMaxAngleHeading), CameraContext::kMinAngleHeading);
  }

  SetCameraTargetDirection(cam_az, cam_el, cam_ro);

  p = Pointer("/lens").Get(root);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Camera config missing <lens>, using default equal-area fisheye!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Camera config <lens> is not a string, using default equal-area fisheye!\n");
  } else {
    if (*p == "linear") {
      SetLensType(LensType::kLinear);
    } else if (*p == "fisheye_equalarea" || *p == "fisheye") {
      SetLensType(LensType::kEqualArea);
    } else if (*p == "fisheye_equidistant") {
      SetLensType(LensType::kEquidistant);
    } else if (*p == "dual_fisheye_equidistant") {
      SetLensType(LensType::kDualEquidistant);
    } else if (*p == "dual_fisheye_equalarea") {
      SetLensType(LensType::kDualEqualArea);
    } else {
      std::fprintf(stderr, "\nWARNING! config <camera.lens> cannot be recognized, using default equal-area fisheye!\n");
    }
  }

  p = Pointer("/fov").Get(root);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.fov>, using default %.1f!\n", GetFov());
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! config <camera.fov> is not a number, using default %.1f!\n", GetFov());
  } else {
    SetFov(static_cast<float>(p->GetDouble()));
  }
}

}  // namespace icehalo
