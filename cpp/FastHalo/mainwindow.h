#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QButtonGroup>
#include <QGridLayout>
#include <QMainWindow>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QToolButton>
#include <Qt3DExtras/Qt3DWindow>
#include <QtGui/QScreen>

#include "context.h"
#include "iconbutton.h"

namespace Ui {
class MainWindow;
}


class WavelengthData {
 public:
  WavelengthData() : name_(""), customized_(false), icon_(QIcon()) {}
  explicit WavelengthData(const QString& name, bool customized = false)
      : name_(name), customized_(customized), icon_(QIcon()) {}

  const QString name_;
  const bool customized_;
  QVector<IceHalo::ProjectContext::WavelengthInfo> info_;
  QIcon icon_;
};


class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 private slots:
  // Basic settings group
  void updateRayHitsNum(int n);
  void updateSunAltitude(const QString& altitude_txt);
  void updateSunDiameterType(int index);
  void updateTotalRays(int ray_num);
  void updateWavelength();

  // Scatter
  void insertScatterTab();        // Scatter tab
  void updateScatterTabs();       // Scatter tab
  void updateScatterProb(int v);  // Scatter prob

  // Crystals
  void insertCrystalItem();
  void removeCurrentCrystal();

  // Crystal info
  void updateCurrentCrystalInfo();

  // Filter settings group
  void enableFilterSettings(bool enable);

 private:
  void initUi();
  void initBasicSettings();
  void initScatterTab();
  void initCrystalList();
  void initCrystalInfoPanel();

  void setCrystalPanelEnabled(bool enable);

  IconButton* createScatterTab();
  QToolButton* createScatterAddButton();

  double getScatterProb();

  Ui::MainWindow* ui_;

  QButtonGroup* scatter_tab_group_;
  QToolButton* scatter_tab_add_btn_;

  QGridLayout* crystal_info_layout_;
  Qt3DExtras::Qt3DWindow* view3d_;
  QWidget* crystal_preview_widget_;

  QStandardItemModel* crystal_list_model_;

  IceHalo::ProjectContextPtr project_context_;

  static constexpr int kMaxInitRayNum = 1000000;
  static constexpr int kDefaultPopulation = 100;

  static int current_crystal_id_;
  static QVector<WavelengthData>& getWavelengthData();

  using LensType_t = std::underlying_type<IceHalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<IceHalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
