#include "context/camera_context.hpp"

#include <algorithm>

#include "process/render.hpp"
#include "util/log.hpp"


namespace icehalo {

CameraContext::CameraContext() : target_dir_{ 0, 0, 0 }, fov_(0), lens_type_(LensType::kLinear) {}


CameraContextPtrU CameraContext::CreateDefault() {
  CameraContextPtrU cam_ctx{ new CameraContext };
  return cam_ctx;
}


Pose3f CameraContext::GetCameraTargetDirection() const {
  return Pose3f(target_dir_);
}


void CameraContext::SetCameraTargetDirection(float azimuth, float altitude, float roll) {
  azimuth = std::fmod(azimuth, 360.0f);
  altitude = std::fmod(altitude, 180.0f);
  if (altitude > 90.0f) {
    altitude -= 180.0f;
  }
  roll = std::fmod(roll, 360.0f);

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


void to_json(nlohmann::json& obj, const CameraContext& ctx) {
  auto cam_pos = ctx.GetCameraTargetDirection();
  obj["azimuth"] = 90.0 - cam_pos.lon();
  obj["elevation"] = cam_pos.lat();
  obj["rotation"] = cam_pos.roll();
  obj["fov"] = ctx.GetFov();
  obj["lens"] = ctx.GetLensType();
}


void from_json(const nlohmann::json& obj, CameraContext& ctx) {
  auto cam_az = obj.at("azimuth").get<float>();
  auto cam_el = obj.at("elevation").get<float>();
  float cam_ro = 0.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "roll", cam_ro)
  ctx.SetCameraTargetDirection(90.0 - cam_az, cam_el, cam_ro);

  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "lens", LensType, ctx.SetLensType)
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "fov", float, ctx.SetFov)
}

}  // namespace icehalo
