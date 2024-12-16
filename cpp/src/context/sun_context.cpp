#include "context/sun_context.hpp"

#include <algorithm>

#include "core/optics.hpp"

namespace icehalo {

SunContext::SunContext()
    : diameter_(0.0f), altitude_(kDefaultAltitude),
      sun_position_{ 0.0f, -std::cos(kDefaultAltitude * math::kDegreeToRad),
                     -std::sin(kDefaultAltitude * math::kDegreeToRad) } {}


SunContextPtrU SunContext::CreateDefault() {
  SunContextPtrU sun_ctx{ new SunContext };
  return sun_ctx;
}


const float* SunContext::GetSunPosition() const {
  return sun_position_;
}


float SunContext::GetSunAltitude() const {
  return altitude_;
}


bool SunContext::SetSunAltitude(float altitude) {
  if (altitude < -90 || altitude > 90) {
    return false;
  } else {
    altitude_ = altitude;
    sun_position_[0] = 0.0f;
    sun_position_[1] = -std::cos(altitude * math::kDegreeToRad);
    sun_position_[2] = -std::sin(altitude * math::kDegreeToRad);
    return true;
  }
}


float SunContext::GetSunDiameter() const {
  return diameter_;
}


bool SunContext::SetSunDiameter(float d) {
  if (d < 0 || d > kMaxDiameter) {
    return false;
  } else {
    diameter_ = d;
    return true;
  }
}


void to_json(nlohmann::json& obj, const SunContext& ctx) {
  obj["altitude"] = ctx.GetSunAltitude();
  obj["diameter"] = ctx.GetSunDiameter();
}


void from_json(const nlohmann::json& obj, SunContext& ctx) {
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "altitude", float, ctx.SetSunAltitude)
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "diameter", float, ctx.SetSunDiameter)
}

}  // namespace icehalo
