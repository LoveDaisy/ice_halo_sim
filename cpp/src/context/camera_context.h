#ifndef SRC_CONTEXT_CAMERA_CONTEXT_H_
#define SRC_CONTEXT_CAMERA_CONTEXT_H_

#include <memory>
#include <string>

#include "io/serialize.h"
#include "rapidjson/document.h"


namespace icehalo {

class CameraContext;
enum class LensType;

using CameraContextPtrU = std::unique_ptr<CameraContext>;
using CameraContextPtr = std::shared_ptr<CameraContext>;

class CameraContext : public IJsonizable {
 public:
  const float* GetCameraTargetDirection() const;
  void SetCameraTargetDirection(float azimuth, float altitude, float roll);
  void ResetCameraTargetDirection();

  float GetFov() const;
  void SetFov(float fov);

  LensType GetLensType() const;
  void SetLensType(LensType type);

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

  static CameraContextPtrU CreateFromJson(rapidjson::Document& d);

  static constexpr float kMinAngleRound = 0.0f;
  static constexpr float kMaxAngleRound = 360.0f;
  static constexpr float kMinAngleTilt = -90.0f;
  static constexpr float kMaxAngleTilt = 90.0f;
  static constexpr float kMinAngleHeading = -180.0f;
  static constexpr float kMaxAngleHeading = 180.0f;

  static constexpr float kMaxFovLinear = 65.0f;
  static constexpr float kMaxFovFisheye = 120.0f;

  static constexpr float kDefaultCamAzimuth = 90.0f;
  static constexpr float kDefaultCamElevation = 89.9f;
  static constexpr float kDefaultCamRoll = 0.0f;

 private:
  CameraContext();

  float target_dir_[3];  // azimuth, altitude, roll
  float fov_;
  LensType lens_type_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_CAMERA_CONTEXT_H_
