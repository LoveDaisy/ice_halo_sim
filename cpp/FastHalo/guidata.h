#ifndef GUIDATA_H
#define GUIDATA_H

#include <QIcon>
#include <QObject>
#include <QString>
#include <QVector>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include "context.h"
#include "crystal.h"
#include "mymath.h"


class WavelengthData {
 public:
  WavelengthData() : name_(""), customized_(false), icon_(QIcon()) {}
  explicit WavelengthData(const QString& name, bool customized = false)
      : name_(name), customized_(customized), icon_(QIcon()) {}

  static std::vector<WavelengthData>& getData();

  const QString name_;
  const bool customized_;
  std::vector<IceHalo::ProjectContext::WavelengthInfo> info_;
  QIcon icon_;
};


class CrystalData {
 public:
  explicit CrystalData(int id);

  static constexpr int kMaxMillerIndex = 10;
  static constexpr int kMinMillerIndex = 1;

  const int id_;
  QString name_;
  IceHalo::CrystalType type_;
  IceHalo::AxisDistribution axis_;
  float height_[3];
  int miller_idx_[4];  // upper1, upper2, lower1, lower2
  float prism_dist_[6];
};


class MultiScatterData {
 public:
  struct CrystalItemData {
    explicit CrystalItemData(int crystal_id)
        : crystal_id(crystal_id), filter_id(0), population(kDefaultPopulation), linked(true),
          enabled(false) {}

    int crystal_id;
    int filter_id;
    int population;
    bool linked;
    bool enabled;
  };

  explicit MultiScatterData(float prob);

  static constexpr int kDefaultPopulation = 10;

  std::vector<CrystalItemData> crystals_;
  float prob_;
};


class GuiData {
 public:
  struct SunDiameterData {
    QString text;
    float value;
  };

  GuiData();

  static std::vector<SunDiameterData>& getSunDiameterData();

  // Basic settings
  float sun_diameter_;
  float sun_altitude_;
  int max_hits_;
  int ray_number_;
  int wavelength_data_idx_;

  // Crystals
  std::unordered_map<int, CrystalData> crystal_store_;

  // Multi scatter data
  std::vector<MultiScatterData> multi_scatter_data_;

  static constexpr int kMinHitsNum = 1;
  static constexpr int kMaxHitsNum = 12;
  static constexpr int kDefaultHitsNum = 8;
  static constexpr int kMinRayNum = 10000;
  static constexpr int kMaxRayNum = 5000000;
  static constexpr int kDefaultRayNum = 500000;
};

#endif  // GUIDATA_H
