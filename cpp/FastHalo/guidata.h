#ifndef GUIDATA_H
#define GUIDATA_H

#include <QIcon>
#include <QObject>
#include <QString>
#include <QVector>
#include <cstddef>
#include <map>
#include <vector>

#include "context.h"
#include "crystal.h"
#include "mymath.h"
#include "render.h"


class WavelengthData {
 public:
  WavelengthData() : name_(""), customized_(false), icon_(QIcon()) {}
  explicit WavelengthData(const QString& name, bool customized = false)
      : name_(name), customized_(customized), icon_(QIcon()) {}

  static std::vector<WavelengthData>& getData();

  const QString name_;
  const bool customized_;
  std::vector<icehalo::ProjectContext::WavelengthInfo> info_;
  QIcon icon_;
};


class CrystalData {
 public:
  explicit CrystalData(int id);

  static constexpr int kMaxMillerIndex = 10;
  static constexpr int kMinMillerIndex = 1;

  const int id_;
  QString name_;
  icehalo::CrystalType type_;
  icehalo::AxisDistribution axis_;
  float height_[3];
  int miller_idx_[4];  // upper1, upper2, lower1, lower2
  float prism_dist_[6];
};


class FilterData {
 public:
  enum Type {
    kNone,
    kSpecific,
    kGeneral,
    kUnkown,
  };

  explicit FilterData(Type type) : type_(type), symmetry_flag_(icehalo::kSymmetryNone), hits_(0) {}

  Type type_;
  uint8_t symmetry_flag_;
  std::vector<std::vector<int>> paths_;
  std::vector<int> enter_faces_;
  std::vector<int> exit_faces_;
  int hits_;
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


class CameraData {
 public:
  class LensTypeData {
   public:
    LensTypeData(icehalo::LensType type, QString txt) : type_(type), text_(std::move(txt)) {}

    icehalo::LensType type_;
    QString text_;
  };

  CameraData();

  static const std::vector<LensTypeData>& getLensTypeData();

  float azimuth_;
  float elevation_;
  float rotation_;
  float fov_;
  icehalo::LensType lens_type_;
};


class RenderData {
 public:
  RenderData();

  static constexpr int kDefaultWidth = 4096;
  static constexpr int kDefaultHeight = 2048;

  int width_;
  int height_;
  icehalo::VisibleRange visible_range_;
  float ray_color_[3];
  float background_color_[3];
  float intensity_;
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
  std::map<int, CrystalData> crystal_store_;

  // Filters
  std::map<int, FilterData> filter_store_;

  // Multi scatter data
  std::vector<MultiScatterData> multi_scatter_data_;

  // Render data

  static constexpr int kMinHitsNum = 1;
  static constexpr int kMaxHitsNum = 12;
  static constexpr int kDefaultHitsNum = 8;
  static constexpr int kMinRayNum = 10000;
  static constexpr int kMaxRayNum = 5000000;
  static constexpr int kDefaultRayNum = 500000;
};

#endif  // GUIDATA_H
