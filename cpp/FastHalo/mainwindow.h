#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QButtonGroup>
#include <QMainWindow>
#include <QToolButton>

#include "closabletabwidget.h"
#include "context.h"

namespace Ui {
class MainWindow;
}


class ColorData {
 public:
  ColorData() : name_(""), customized_(false), color_(QColor(0, 0, 0, 0)), icon_(nullptr) {}
  ColorData(const QString& name, const QColor& color, bool customized = false)
      : name_(name), customized_(customized), color_(color), icon_(nullptr) {}

  const QString name_;
  const bool customized_;
  QColor color_;
  QIcon* icon_;
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
  // Ray settings group
  void updateTotalRays(int ray_num);
  void updateRayColor();
  void updateBackgroundColor();
  // Render settings group
  void updateLensType(int index);
  void updateVisibleRange(int index);
  // Scatter tab
  void insertScatterTab();
  void updateScatterTabs();
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

  ClosableTabWidget* createScatterTab();
  QToolButton* createScatterAddButton();

  Ui::MainWindow* ui_;
  QButtonGroup* scatter_tab_group_;
  QToolButton* scatter_tab_add_btn_;

  IceHalo::ProjectContextPtr project_context_;

  static constexpr int kMaxInitRayNumber = 1000000;
  static QVector<ColorData>& getRayColorData();
  static QVector<ColorData>& getBackgroundColorData();
  static QVector<QIcon>& getColorIcons();

  using LensType_t = std::underlying_type<IceHalo::LensType>::type;
  using VisibleRange_t = std::underlying_type<IceHalo::VisibleRange>::type;
};

#endif  // MAINWINDOW_H
