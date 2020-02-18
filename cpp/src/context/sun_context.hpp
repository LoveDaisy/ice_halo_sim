#ifndef SRC_CONTEXT_SUN_CONTEXT_H_
#define SRC_CONTEXT_SUN_CONTEXT_H_

#include <memory>

#include "core/core_def.hpp"
#include "io/serialize.hpp"
#include "rapidjson/document.h"


namespace icehalo {

class SunContext : public IJsonizable {
 public:
  const float* GetSunPosition() const;

  float GetSunAltitude() const;
  bool SetSunAltitude(float altitude);

  float GetSunDiameter() const;
  bool SetSunDiameter(float d);

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

  static SunContextPtrU CreateDefault();

  static constexpr float kMaxDiameter = 90.0f;
  static constexpr float kDefaultAltitude = 20.0f;

 private:
  SunContext();

  float diameter_;         // in degree
  float altitude_;         // in degree
  float sun_position_[3];  // [x, y, z]
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_SUN_CONTEXT_H_
