#include "mainwindow.h"

#include <QColorDialog>
#include <QPropertyAnimation>
#include <QtDebug>

#include "closabletabwidget.h"
#include "render.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow), project_context_(IceHalo::ProjectContext::CreateDefault()) {
  ui_->setupUi(this);
  initUi();
}


MainWindow::~MainWindow() {
  delete ui_;
}


void MainWindow::initUi() {
  using namespace IceHalo;

  // Setup some UI properties
  ui_->basicSettingLayout->setAlignment(Qt::AlignTop);
  ui_->crystalAxisSettingLayout->setAlignment(Qt::AlignTop);
  ui_->filterSettingLayout->setAlignment(Qt::AlignTop);

  initBasicSettings();
  initScatterTab();

  // Connect signals and slots
  connect(ui_->filterEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::enableFilterSettings);
  connect(ui_->scatterProbSlider, &QSlider::valueChanged, this, &MainWindow::updateScatterProb);
}


void MainWindow::initBasicSettings() {
  // Sun diameter
  ui_->sunDiameterComboBox->addItem(tr("true diameter"), QVariant(0.5f));
  ui_->sunDiameterComboBox->addItem(tr("point source"), QVariant(0.0f));
  updateSunDiameterType(0);

  // Sun altitude
  QString altitude_txt = QString::number(static_cast<int>(project_context_->sun_ctx_.GetSunAltitude()));
  project_context_->sun_ctx_.SetSunAltitude(altitude_txt.toFloat());
  ui_->sunAltitudeEdit->setValidator(new QIntValidator(-90, 90));
  ui_->sunAltitudeEdit->setText(altitude_txt);

  // Max hits

  // Ray number

  // Wavelength
  foreach (const auto& wl, getWavelengthData()) { ui_->wavelengthComboBox->addItem(wl.name_); }
  updateWavelength();

  connect(ui_->sunDiameterComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateSunDiameterType);
  connect(ui_->sunAltitudeEdit, &QLineEdit::textChanged, this, &MainWindow::updateSunAltitude);
  connect(ui_->maxHitsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateRayHitsNum);
  connect(ui_->rayNumberSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateTotalRays);
  connect(ui_->wavelengthComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateWavelength);
}


void MainWindow::initScatterTab() {
  scatter_tab_add_btn_ = createScatterAddButton();
  scatter_tab_group_ = new QButtonGroup(ui_->scatterTabFrame);
  ui_->scatterTabLayout->addWidget(scatter_tab_add_btn_);

  insertScatterTab();
  auto tab0 = static_cast<ClosableTabWidget*>(ui_->scatterTabLayout->itemAt(0)->widget());
  tab0->enableIcon(false);

  connect(scatter_tab_add_btn_, &QToolButton::clicked, this, &MainWindow::insertScatterTab);
}


void MainWindow::insertScatterTab() {
  int current_item_cnt = ui_->scatterTabLayout->count();
  auto btn = createScatterTab();
  ui_->scatterTabLayout->insertWidget(current_item_cnt - 1, btn);
  scatter_tab_group_->addButton(btn);
  btn->setChecked(true);

  updateScatterTabs();

  connect(btn, &ClosableTabWidget::closeTab, this, [=] {
    auto anim = new QPropertyAnimation(btn, "maximumWidth");
    anim->setStartValue(btn->geometry().width());
    anim->setEndValue(0);
    anim->setDuration(200);
    anim->setTargetObject(btn);
    anim->start(QAbstractAnimation::DeleteWhenStopped);

    connect(anim, &QPropertyAnimation::finished, this, &MainWindow::updateScatterTabs);
  });
}


ClosableTabWidget* MainWindow::createScatterTab() {
  auto btn = new ClosableTabWidget("");
  btn->setChecked(true);
  btn->setVisible(true);
  return btn;
}


QToolButton* MainWindow::createScatterAddButton() {
  QToolButton* btn = new QToolButton();
  btn->setText(tr("+"));
  return btn;
}


void MainWindow::enableFilterSettings(bool enable) {
  ui_->specificRadioButton->setEnabled(enable);
  ui_->generalRadioButton->setEnabled(enable);
  ui_->symBCheckBox->setEnabled(enable);
  ui_->symDCheckBox->setEnabled(enable);
  ui_->symPCheckBox->setEnabled(enable);
}


void MainWindow::updateTotalRays(int ray_num) {
  size_t init_ray_num = static_cast<size_t>(ray_num);
  project_context_->SetInitRayNum(init_ray_num);
  qDebug() << "Updating total ray number: " << ray_num;
}


void MainWindow::updateWavelength() {
  int curr_idx = ui_->wavelengthComboBox->currentIndex();
  auto& wl_data = getWavelengthData()[curr_idx];
  if (wl_data.customized_) {
    // TODO
  }

  project_context_->ClearWavelengthInfo();
  foreach (const auto& wl, wl_data.info_) { project_context_->AddWavelengthInfo(wl.wavelength, wl.weight); }
}


void MainWindow::updateRayHitsNum(int n) {
  project_context_->SetRayHitNum(n);
  qDebug() << "Updating ray hits number: " << n;
}


void MainWindow::updateSunAltitude(const QString& altitude_txt) {
  float altitude = altitude_txt.toFloat();
  project_context_->sun_ctx_.SetSunAltitude(altitude);
  qDebug() << "Updating sun altitude: " << altitude;
}


void MainWindow::updateSunDiameterType(int index) {
  double d = ui_->sunDiameterComboBox->itemData(index).toDouble();
  project_context_->sun_ctx_.SetSunDiameter(static_cast<float>(d));
  QString d_txt = QString::asprintf("Sun diameter: %.1f°", d);
  ui_->sunDiameterLabel->setText(d_txt);
  qDebug() << "Updating sun diameter: " << d_txt;
}


void MainWindow::updateScatterTabs() {
  int tab_cnt = ui_->scatterTabLayout->count() - 1;
  if (tab_cnt <= 0) {
    return;
  }

  // Remove invisible tabs
  bool update_check = false;  // If a checked tab is removed, then the next tab will be checked.
  for (int i = 0; i < tab_cnt; i++) {
    auto tab = static_cast<ClosableTabWidget*>(ui_->scatterTabLayout->itemAt(i)->widget());
    if (tab->geometry().width() > 0) {
      if (update_check) {
        tab->setChecked(true);
        update_check = false;
      }
      continue;
    }

    if (scatter_tab_group_->checkedButton() == tab) {
      update_check = true;
    }
    ui_->scatterTabLayout->removeWidget(tab);
    tab->deleteLater();
    scatter_tab_group_->removeButton(tab);
    i--;  // Ugly... There is no iterator-like thing in QLayout
  }

  // Refresh tab text
  tab_cnt = ui_->scatterTabLayout->count() - 1;
  for (int i = 0; i < tab_cnt; i++) {
    QString tab_txt = tr("  Scatter %1").arg(i + 1);
    auto tab = static_cast<ClosableTabWidget*>(ui_->scatterTabLayout->itemAt(i)->widget());
    tab->setText(tab_txt);
    tab->enableIcon(true);
  }

  // If only one tab, disable close icon
  if (tab_cnt <= 1 && ui_->scatterTabLayout->itemAt(0)) {
    auto tab = static_cast<ClosableTabWidget*>(ui_->scatterTabLayout->itemAt(0)->widget());
    tab->enableIcon(false);
  }

  // If no checked (the right most checked tab is removed), check the first one
  if (!scatter_tab_group_->checkedButton()) {
    auto tab = static_cast<ClosableTabWidget*>(ui_->scatterTabLayout->itemAt(0)->widget());
    tab->setChecked(true);
  }
}


void MainWindow::updateScatterProb(int v) {
  double prob = v / 100.0;
  QString prob_str = tr("Probability: %1").arg(prob, 0, 'f', 2);
  ui_->scatterProbLabel->setText(prob_str);
  qDebug() << "Updating scatter prob: " << v << ", " << prob_str;
}


void MainWindow::updateSimulationContext() {
  updateTotalRays(ui_->rayNumberSpinBox->value());
  updateRayHitsNum(ui_->maxHitsSpinBox->value());

  // Multiscatter settings
  // TODO

  // Wavelength settings
  project_context_->ClearWavelengthInfo();
  // TODO

  updateSunAltitude(ui_->sunAltitudeEdit->text());
  updateSunDiameterType(ui_->sunDiameterComboBox->currentIndex());
}


QVector<ColorData>& MainWindow::getRayColorData() {
  static QVector<ColorData> colors;
  if (colors.empty()) {
    colors << ColorData(tr("real"), QColor(0, 0, 0, 0));
    colors << ColorData(tr("white"), QColor(255, 255, 255, 255));
    colors << ColorData(tr("black"), QColor(0, 0, 0, 255));
    colors << ColorData(tr("customize"), QColor(255, 255, 255, 255));

    colors[0].icon_ = QIcon(":/icons/icon_color_real.png");
  }

  return colors;
}


QVector<ColorData>& MainWindow::getBackgroundColorData() {
  static QVector<ColorData> colors;
  if (colors.empty()) {
    colors << ColorData(tr("black"), QColor(0, 0, 0, 255));
    colors << ColorData(tr("white"), QColor(255, 255, 255, 255));
    //  colors << ColorData(tr("sky"), QColor(0, 0, 0, 0));
    colors << ColorData(tr("customize"), QColor(255, 255, 255, 255));
  }

  return colors;
}


QVector<WavelengthData>& MainWindow::getWavelengthData() {
  static QVector<WavelengthData> wl_data;
  if (wl_data.empty()) {
    WavelengthData wl_sun(tr("sun light"));
    wl_sun.icon_ = QIcon(":/icons/icon_wl_sun_dark.png");
    wl_sun.info_ << IceHalo::ProjectContext::WavelengthInfo{ 420, 0.9122f };
    wl_sun.info_ << IceHalo::ProjectContext::WavelengthInfo{ 460, 0.9969f };
    wl_sun.info_ << IceHalo::ProjectContext::WavelengthInfo{ 500, 1.0381f };
    wl_sun.info_ << IceHalo::ProjectContext::WavelengthInfo{ 540, 1.0440f };
    wl_sun.info_ << IceHalo::ProjectContext::WavelengthInfo{ 580, 1.0237f };
    wl_sun.info_ << IceHalo::ProjectContext::WavelengthInfo{ 620, 0.9851f };
    wl_data.append(std::move(wl_sun));

    WavelengthData wl_eq(tr("equal energy"));
    wl_eq.icon_ = QIcon(":/icons/icon_wl_eq_dark.png");
    wl_eq.info_ << IceHalo::ProjectContext::WavelengthInfo{ 420, 1.0f };
    wl_eq.info_ << IceHalo::ProjectContext::WavelengthInfo{ 460, 1.0f };
    wl_eq.info_ << IceHalo::ProjectContext::WavelengthInfo{ 500, 1.0f };
    wl_eq.info_ << IceHalo::ProjectContext::WavelengthInfo{ 540, 1.0f };
    wl_eq.info_ << IceHalo::ProjectContext::WavelengthInfo{ 580, 1.0f };
    wl_eq.info_ << IceHalo::ProjectContext::WavelengthInfo{ 620, 1.0f };
    wl_data.append(std::move(wl_eq));

    //  WavelengthData wl_customize(tr("customize"), true);
    //  wl_customize.icon_ = QIcon(":/icons/icon_wl_customize_dark.png");
    //  wl_customize.info_ << IceHalo::ProjectContext::WavelengthInfo{ 420, 0.9122f };
    //  wl_customize.info_ << IceHalo::ProjectContext::WavelengthInfo{ 460, 0.9969f };
    //  wl_customize.info_ << IceHalo::ProjectContext::WavelengthInfo{ 500, 1.0381f };
    //  wl_customize.info_ << IceHalo::ProjectContext::WavelengthInfo{ 540, 1.0440f };
    //  wl_customize.info_ << IceHalo::ProjectContext::WavelengthInfo{ 580, 1.0237f };
    //  wl_customize.info_ << IceHalo::ProjectContext::WavelengthInfo{ 620, 0.9851f };
    //  wl_data.append(std::move(wl_customize));
  }

  return wl_data;
}
