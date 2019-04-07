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
  void updateScatterProb(int v);

private:
  void initUi();

  Ui::MainWindow* ui;

  IceHalo::SimulationContextPtr simulation_context;
};

#endif // MAINWINDOW_H
