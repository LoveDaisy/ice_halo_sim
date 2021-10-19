#ifndef SRC_CONTEXT_CAMERA_CONTEXT_H_
#define SRC_CONTEXT_CAMERA_CONTEXT_H_

#include <memory>
#include <string>

#include "core/core_def.hpp"
#include "core/math.hpp"
#include "io/serialize.hpp"
#include "json.hpp"


namespace icehalo {

enum class LensType {
  kLinear,
  kEqualArea,
  kEquidistant,
  kDualEqualArea,
  kDualEquidistant,
  kEquirectangular,
};


NLOHMANN_JSON_SERIALIZE_ENUM(LensType, {
                                           { LensType::kEqualArea, "fisheye_equalarea" },
                                           { LensType::kDualEqualArea, "dual_fisheye_equalarea" },
                                           { LensType::kEquidistant, "fisheye_equidistant" },
                                           { LensType::kDualEquidistant, "dual_fisheye_equidistant" },
                                           { LensType::kLinear, "linear" },
                                           { LensType::kEquirectangular, "equirectangular" },
                                       })


class CameraContext {
 public:
  Pose3f GetCameraTargetDirection() const;
  void SetCameraTargetDirection(float azimuth, float altitude, float roll);
  void ResetCameraTargetDirection();

  float GetFov() const;
  void SetFov(float fov);

  LensType GetLensType() const;
  void SetLensType(LensType type);

  static CameraContextPtrU CreateDefault();

  static constexpr float kMaxFovLinear = 65.0f;
  static constexpr float kMaxFovFisheye = 120.0f;

  static constexpr float kDefaultCamAzimuth = 90.0f;
  static constexpr float kDefaultCamElevation = 90.0f;
  static constexpr float kDefaultCamRoll = 0.0f;

 private:
  CameraContext();

  float target_dir_[3];  // azimuth, altitude, roll
  float fov_;
  LensType lens_type_;
};


void to_json(nlohmann::json& obj, const CameraContext& ctx);

void from_json(const nlohmann::json& obj, CameraContext& ctx);

}  // namespace icehalo


#endif  // SRC_CONTEXT_CAMERA_CONTEXT_H_
