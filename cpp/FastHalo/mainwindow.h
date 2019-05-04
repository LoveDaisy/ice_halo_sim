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
  void updateRayHitsNum(int n);                         // Update model
  void updateSunAltitude(const QString& altitude_txt);  // Update model
  void updateSunDiameter(int index);                    // Update model
  void updateTotalRays(int ray_num);                    // Update model
  void updateWavelength(int index);                     // Update model

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

  QString getScatterTabText(int idx);
  QString getScatterProbText(double prob);
  IconButton* createScatterTab();
  QToolButton* createScatterAddButton();

  double getScatterProb();
  double getScatterProb(int v);
  MultiScatterData* getCurrentScatterData();
  MultiScatterData::CrystalItemData* getCurrentCrystalItemData();
  MultiScatterData::CrystalItemData* getCrystalItemData(const QModelIndex& index);
  CrystalData* getCurrentCrystalData();
  CrystalData* getCrystalData(const QModelIndex& index);

  Ui::MainWindow* ui_;

  QButtonGroup* scatter_tab_group_;
  QToolButton* scatter_tab_add_btn_;

  QGridLayout* crystal_info_layout_;
  Qt3DExtras::Qt3DWindow* view3d_;
  QWidget* crystal_preview_widget_;

  QStandardItemModel* crystal_list_model_;

  IceHalo::ProjectContextPtr project_context_;
  GuiData gui_data_;

  static int current_crystal_id_;

  using LensType_t = std::underlying_type<IceHalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<IceHalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
