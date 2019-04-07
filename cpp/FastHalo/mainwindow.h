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
  void enableFilterSettings(bool enable);
  void updateTotalRays(int ray_num);
  void updateRayHitsNum(int n);
  void updateSunPosition(const QString& altitude_txt);
  void updateSunDiameterType(int index);

  void updateScatterProb(int v);
  void updateSimulationContext();

 private:
  void initUi();

  Ui::MainWindow* ui;

  IceHalo::SimulationContextPtr simulation_context;
  IceHalo::RenderContextPtr render_context;
};

#endif  // MAINWINDOW_H
