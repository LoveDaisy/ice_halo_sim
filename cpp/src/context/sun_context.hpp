#ifndef SRC_CONTEXT_SUN_CONTEXT_H_
#define SRC_CONTEXT_SUN_CONTEXT_H_

#include <memory>

#include "core/core_def.hpp"
#include "io/json_util.hpp"
#include "io/serialize.hpp"

namespace icehalo {

class SunContext {
 public:
  const float* GetSunPosition() const;

  float GetSunAltitude() const;
  bool SetSunAltitude(float altitude);

  float GetSunDiameter() const;
  bool SetSunDiameter(float d);

  static SunContextPtrU CreateDefault();

  static constexpr float kMaxDiameter = 90.0f;
  static constexpr float kDefaultAltitude = 20.0f;

 private:
  SunContext();

  float diameter_;         // in degree
  float altitude_;         // in degree
  float sun_position_[3];  // [x, y, z]
};

void to_json(nlohmann::json& obj, const SunContext& ctx);
void from_json(const nlohmann::json& obj, SunContext& ctx);

}  // namespace icehalo


#endif  // SRC_CONTEXT_SUN_CONTEXT_H_
