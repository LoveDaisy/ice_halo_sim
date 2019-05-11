#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QButtonGroup>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QSlider>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QToolButton>
#include <Qt3DExtras/Qt3DWindow>
#include <QtGui/QScreen>

#include "context.h"
#include "cursortable.h"
#include "floatlineedit.h"
#include "guidata.h"
#include "iconbutton.h"

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 private slots:
  // Basic settings group
  void updateRayHitsNum(int n);       // Update model
  void updateSunDiameter(int index);  // Update model
  void updateTotalRays(int ray_num);  // Update model
  void updateWavelength(int index);   // Update model

  // Scatter tabs
  void addScatter();                       // Update model and view
  void removeScatter(IconButton* sender);  // Update model and view

  // Scatter probability
  void updateScatterProb(int v);  // Update model
  void refreshScatterProb();      // Update view

  // Crystal list
  void updateCrystalData(const QModelIndex& index);       // Update model
  void addCrystal();                                      // Update model and view
  void removeCurrentCrystal();                            // Update model and view
  void toggleCrystalLinkState(const QModelIndex& index);  // Update model and view
  void refreshCrystalList();                              // Update view

  // Crystal info
  void updateCrystalType(int combo_idx);          // Update model
  void updatePrismDistance(int idx, int v);       // Update model
  void resetPrismDistance();                      // Update model and view
  void updateUpperHeight(int v);                  // Update model
  void updateLowerHeight(int v);                  // Update model
  void updatePyramidMillerIndex(int idx, int v);  // Update model
  void refreshCrystalInfo();                      // Update view

  // Filter settings group
  void enableFilterSettings(bool enable);

 private:
  void initUi();
  void initBasicSettings();
  void initScatterTab();
  void initCrystalList();
  void initCrystalInfoPanel();

  void setCrystalPanelEnabled(bool enable);

  double getScatterProb();
  double getScatterProb(int v);
  QString getScatterTabText(int idx);
  QString getScatterProbText(double prob);
  IconButton* createScatterTab();
  QToolButton* createScatterAddButton();

  double getPrismDistance(int idx);
  double getPrismDistanceByValue(int value);
  QString getPrismDistanceText(double d);

  double getPyramidHeight(int value);
  QString getPyramidHeightText(double h);

  MultiScatterData* getCurrentScatterData();
  MultiScatterData::CrystalItemData* getCurrentCrystalItemData();
  MultiScatterData::CrystalItemData* getCrystalItemData(const QModelIndex& index);
  CrystalData* getCurrentCrystalData();
  CrystalData* getCrystalData(const QModelIndex& index);

  Ui::MainWindow* ui_;

  // Basic settings
  FloatLineEdit* sun_altitude_edit_;

  // Scatter tabs
  QButtonGroup* scatter_tab_group_;
  QToolButton* scatter_tab_add_btn_;

  // Crystal info panel
  QGridLayout* crystal_info_layout_;
  Qt3DExtras::Qt3DWindow* view3d_;
  QWidget* crystal_preview_widget_;
  FloatLineEdit* crystal_height_edit_;
  FloatLineEdit* axis_zenith_mean_edit_;
  FloatLineEdit* axis_zenith_std_edit_;
  FloatLineEdit* axis_roll_mean_edit_;
  FloatLineEdit* axis_roll_std_edit_;
  FloatLineEdit* axis_azimuth_mean_edit_;
  FloatLineEdit* axis_azimuth_std_edit_;
  QSlider* prism_distance_sliders_[6];
  QLabel* prism_distance_labels_[6];
  QComboBox* miller_index_combos_[4];

  // Crystal list
  QStandardItemModel* crystal_list_model_;
  CursorTable* crystal_table_;

  // Data model
  IceHalo::ProjectContextPtr project_context_;
  GuiData gui_data_;

  static int current_crystal_id_;

  using LensType_t = std::underlying_type<IceHalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<IceHalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
