#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QToolButton>
#include <QButtonGroup>

#include "context.h"

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
  void updateRayHitsNum(int n);
  void updateSunAltitude(const QString& altitude_txt);
  void updateSunDiameterType(int index);
  // Ray settings group
  void updateTotalRays(int ray_num);
  void updateRayColor(int index);
  void updateBackgroundColor(int index);
  // Render settings group
  void updateLensType(int index);
  void updateVisibleRange(int index);
  // Scatter tab
  void insertScatterTab();
  // Filter settings group
  void enableFilterSettings(bool enable);

  void updateScatterProb(int v);
  void updateSimulationContext();

 private:
  void initUi();
  void initBasicSettings();
  void initRaySettings();
  void initRenderSettings();
  void initScatterTab();

  QToolButton* createScatterTab(int index);
  QToolButton* createScatterAddButton();

  Ui::MainWindow* ui_;
  QButtonGroup* scatter_tab_group_;
  QToolButton* scatter_tab_add_btn_;

  IceHalo::ProjectContextPtr project_context_;

  static constexpr int kMaxInitRayNumber = 1000000;
  static constexpr int kTabIconSize = 10;

  using LensType_t = std::underlying_type<IceHalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<IceHalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
