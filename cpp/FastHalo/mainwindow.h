#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
  void updateSunPosition(const QString& altitude_txt);
  void updateSunDiameterType(int index);
  // Ray settings group
  void updateTotalRays(int ray_num);
  void updateRayColor(int index);
  void updateBackgroundColor(int index);
  // Render settings group
  void updateLensType(int index);
  void updateVisibleRange(int index);
  // Filter settings group
  void enableFilterSettings(bool enable);

  void updateScatterProb(int v);
  void updateSimulationContext();

 private:
  void initUi();

  Ui::MainWindow* ui;

  IceHalo::ProjectContextPtr project_context;

  using LensType_t = std::underlying_type<IceHalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<IceHalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
