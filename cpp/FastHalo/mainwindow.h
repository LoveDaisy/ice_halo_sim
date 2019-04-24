#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QButtonGroup>
#include <QMainWindow>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QToolButton>

#include "context.h"
#include "iconbutton.h"

namespace Ui {
class MainWindow;
}


class ColorData {
 public:
  ColorData() : name_(""), color_(QColor(0, 0, 0, 0)), icon_(QIcon()) {}
  ColorData(const QString& name, const QColor& color) : name_(name), color_(color), icon_(QIcon()) {}

  const QString name_;
  QColor color_;
  QIcon icon_;
};


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

  // Scatter tab
  void insertScatterTab();
  void updateScatterTabs();

  // Crystal table
  void insertCrystalItem();

  // Filter settings group
  void enableFilterSettings(bool enable);

  void updateScatterProb(int v);
  void updateSimulationContext();

 private:
  void initUi();
  void initBasicSettings();
  void initScatterTab();
  void initCrystalList();

  IconButton* createScatterTab();
  QToolButton* createScatterAddButton();

  //  QStandardItem* createCrystalItem();

  Ui::MainWindow* ui_;

  QButtonGroup* scatter_tab_group_;
  QToolButton* scatter_tab_add_btn_;

  QStandardItemModel* crystal_list_model_;

  IceHalo::ProjectContextPtr project_context_;

  static constexpr int kMaxInitRayNum = 1000000;
  static QVector<WavelengthData>& getWavelengthData();

  using LensType_t = std::underlying_type<IceHalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<IceHalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
