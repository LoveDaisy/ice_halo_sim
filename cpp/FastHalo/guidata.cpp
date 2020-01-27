#include "guidata.h"

std::vector<WavelengthData>& WavelengthData::getData() {
  static std::vector<WavelengthData> wl_data;
  if (wl_data.empty()) {
    WavelengthData wl_sun(QObject::tr("sun light"));
    wl_sun.icon_ = QIcon(":/icons/icon_wl_sun_dark.png");
    wl_sun.info_ = {
      icehalo::ProjectContext::WavelengthInfo{ 420, 0.9122f },  //
      icehalo::ProjectContext::WavelengthInfo{ 460, 0.9969f },  //
      icehalo::ProjectContext::WavelengthInfo{ 500, 1.0381f },  //
      icehalo::ProjectContext::WavelengthInfo{ 540, 1.0440f },  //
      icehalo::ProjectContext::WavelengthInfo{ 580, 1.0237f },  //
      icehalo::ProjectContext::WavelengthInfo{ 620, 0.9851f },  //
    };
    wl_data.emplace_back(std::move(wl_sun));

    WavelengthData wl_eq(QObject::tr("equal energy"));
    wl_eq.icon_ = QIcon(":/icons/icon_wl_eq_dark.png");
    wl_eq.info_ = {
      icehalo::ProjectContext::WavelengthInfo{ 420, 1.0f },  //
      icehalo::ProjectContext::WavelengthInfo{ 460, 1.0f },  //
      icehalo::ProjectContext::WavelengthInfo{ 500, 1.0f },  //
      icehalo::ProjectContext::WavelengthInfo{ 540, 1.0f },  //
      icehalo::ProjectContext::WavelengthInfo{ 580, 1.0f },  //
      icehalo::ProjectContext::WavelengthInfo{ 620, 1.0f },  //
    };
    wl_data.emplace_back(std::move(wl_eq));
  }

  return wl_data;
}


CrystalData::CrystalData(int id)
    : id_(id), name_(""), type_(icehalo::CrystalType::kPrism), axis_(icehalo::AxisDistribution{}),
      height_{ 0.0f },                                   // prism height, upper height, lower height
      miller_idx_{ 1, 1, 1, 1 },                         // upper1, upper2, lower1, lower2
      prism_dist_{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }  // prism distances
{
  axis_.latitude_mean = 90;
}


MultiScatterData::MultiScatterData(float prob) : prob_(prob) {}


CameraData::CameraData()
    : azimuth_(0), elevation_(0), rotation_(0), fov_(45), lens_type_(icehalo::LensType::kLinear) {}


const std::vector<CameraData::LensTypeData>& CameraData::getLensTypeData() {
  static std::vector<CameraData::LensTypeData> lens_type_data = {
    { icehalo::LensType::kLinear, QObject::tr("Normal lens") },
    { icehalo::LensType::kEqualArea, QObject::tr("Fisheye") },
    { icehalo::LensType::kDualEqualArea, QObject::tr("Dual fisheye") },
  };
  return lens_type_data;
}


RenderData::RenderData()
    : width_(0), height_(0), visible_range_(icehalo::VisibleRange::kUpper),
      ray_color_{ -1, -1, -1 }, background_color_{ -1, -1, -1 }, intensity_(5) {}


GuiData::GuiData()
    : sun_diameter_(0.5f), sun_altitude_(20.0f), max_hits_(kDefaultHitsNum),
      ray_number_(kDefaultRayNum) {
  filter_store_.emplace(0, FilterData::kNone);
}


std::vector<GuiData::SunDiameterData>& GuiData::getSunDiameterData() {
  static std::vector<GuiData::SunDiameterData> data = {
    GuiData::SunDiameterData{ QObject::tr("true"), 0.5f },
    GuiData::SunDiameterData{ QObject::tr("point"), 0.0f },
  };

  return data;
}
