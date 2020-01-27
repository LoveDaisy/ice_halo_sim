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
#include "customwidgets.h"
#include "guidata.h"

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
  FilterData& createNewFilter(
      MultiScatterData::CrystalItemData* crystal_item_data);      // Helper function
  void enableFilterSettings(bool enable);                         // Helper function
  void resetFilterInfo();                                         // Helper function
  void addSpecificFilterRow();                                    // Helper function
  void addSpecificFilterRow(const std::vector<int>& path);        // Helper function
  void updateFilterInfo();                                        // Update model
  void updateFilterSymmetry(FilterData& filter_data);             // Update model
  void updateGeneralFilterInfo(FilterData& filter_data);          // Update model
  void updateSpecificFilterInfo(FilterData& filter_data);         // Update model
  void refreshFilterInfo();                                       // Update view
  void refreshFilterInfo(const FilterData& filter_data);          // Update view
  void refreshFilterSymmetry(const FilterData& filter_data);      // Update view
  void refreshGeneralFilterPage(const FilterData& filter_data);   // Update view
  void refreshSpecificFilterPage(const FilterData& filter_data);  // Update view
  void resetGeneralFilterPage();                                  // Update view
  void resetSpecificFilterPage();                                 // Update view
  void resetFilterSymmetryPanel();                                // Update view

 private:
  void initUi();
  void initBasicSettings();
  void initScatterTab();
  void initCrystalList();
  void initCrystalInfoPanel();
  void initFilter();

  void setCrystalPanelEnabled(bool enable);

  // Scatter probability
  double getScatterProb();
  double getScatterProb(int v);
  QString getScatterTabText(int idx);
  QString getScatterProbText(double prob);

  // Scatter tabs
  IconButton* createScatterTab();
  QToolButton* createScatterAddButton();

  // Prism distances
  double getPrismDistance(int idx);
  double getPrismDistanceByValue(int value);
  QString getPrismDistanceText(double d);

  // Pyramid heights
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

  // Specific filter path list
  QStandardItemModel* specific_path_model_;
  CursorTable* specific_path_table_;

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
  icehalo::ProjectContextPtr project_context_;
  GuiData gui_data_;

  static int current_crystal_id_;
  static int current_filter_id_;

  using LensType_t = std::underlying_type<icehalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<icehalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
