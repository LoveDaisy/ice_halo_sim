#include "context/sun_context.h"

#include <algorithm>

#include "core/optics.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"


namespace icehalo {

using rapidjson::Pointer;

SunContext::SunContext()
    : diameter_(0.0f),
      altitude_(kDefaultAltitude), sun_position_{ 0.0f, -std::cos(kDefaultAltitude * math::kDegreeToRad),
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


void SunContext::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  root.Clear();
  Pointer("/altitude").Set(root, altitude_, allocator);
  Pointer("/diameter").Set(root, diameter_, allocator);
}


void SunContext::LoadFromJson(const rapidjson::Value& root) {
  float sun_altitude = 0.0f;
  auto* p = Pointer("/altitude").Get(root);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Sun config missing <altitude>, using default 0.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Sun config <altitude> is not a number, using default 0.0!\n");
  } else {
    sun_altitude = static_cast<float>(p->GetDouble());
  }
  SetSunAltitude(sun_altitude);

  p = Pointer("/diameter").Get(root);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Sun config missing <diameter>, using default 0.5!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Sun config <diameter> is not a number, using default 0.5!\n");
  } else {
    SetSunDiameter(static_cast<float>(p->GetDouble()));
  }
}

}  // namespace icehalo
