#include "guidata.h"

std::vector<WavelengthData>& WavelengthData::getData() {
  static std::vector<WavelengthData> wl_data;
  if (wl_data.empty()) {
    WavelengthData wl_sun(QObject::tr("sun light"));
    wl_sun.icon_ = QIcon(":/icons/icon_wl_sun_dark.png");
    wl_sun.info_ = {
      IceHalo::ProjectContext::WavelengthInfo{ 420, 0.9122f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 460, 0.9969f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 500, 1.0381f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 540, 1.0440f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 580, 1.0237f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 620, 0.9851f },  //
    };
    wl_data.emplace_back(std::move(wl_sun));

    WavelengthData wl_eq(QObject::tr("equal energy"));
    wl_eq.icon_ = QIcon(":/icons/icon_wl_eq_dark.png");
    wl_eq.info_ = {
      IceHalo::ProjectContext::WavelengthInfo{ 420, 1.0f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 460, 1.0f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 500, 1.0f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 540, 1.0f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 580, 1.0f },  //
      IceHalo::ProjectContext::WavelengthInfo{ 620, 1.0f },  //
    };
    wl_data.emplace_back(std::move(wl_eq));
  }

  return wl_data;
}


CrystalData::CrystalData(int id)
    : id_(id), name_(""), type_(IceHalo::CrystalType::kPrism), axis_(IceHalo::AxisDistribution{}),
      height_{ 0.0f },                                   // prism height, upper height, lower height
      miller_idx_{ 1, 1, 1, 1 },                         // upper1, upper2, lower1, lower2
      prism_dist_{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }  // prism distances
{}


MultiScatterData::MultiScatterData(float prob) : prob_(prob) {}


GuiData::GuiData()
    : sun_diameter_(0.5f), sun_altitude_(20.0f), max_hits_(kDefaultHitsNum),
      ray_number_(kDefaultRayNum) {}


std::vector<GuiData::SunDiameterData>& GuiData::getSunDiameterData() {
  static std::vector<GuiData::SunDiameterData> data = {
    GuiData::SunDiameterData{ QObject::tr("true"), 0.5f },
    GuiData::SunDiameterData{ QObject::tr("point"), 0.0f },
  };

  return data;
}
