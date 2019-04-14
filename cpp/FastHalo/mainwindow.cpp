#include "mainwindow.h"

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
  //  delete ui_;
}


void MainWindow::initUi() {
  using namespace IceHalo;

  // Setup some UI properties
  ui_->renderSettingLayout->setAlignment(Qt::AlignTop);
  ui_->filterSettingLayout->setAlignment(Qt::AlignTop);

  initBasicSettings();
  initRaySettings();
  initRenderSettings();
  initScatterTab();

  // Connect signals and slots
  connect(ui_->filterEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::enableFilterSettings);
  connect(ui_->scatterProbSlider, &QSlider::valueChanged, this, &MainWindow::updateScatterProb);
}


void MainWindow::initBasicSettings() {
  // Sun diameter
  ui_->sunDiameterComboBox->addItem(tr("true diameter (0.5°)"), QVariant(0.5f));
  ui_->sunDiameterComboBox->addItem(tr("point source (0°)"), QVariant(0.0f));
  project_context_->sun_ctx_.SetSunDiameter(0.5f);  // Default value

  // Sun altitude
  QString altitude_txt = QString::number(static_cast<int>(project_context_->sun_ctx_.GetSunAltitude()));
  project_context_->sun_ctx_.SetSunAltitude(altitude_txt.toFloat());
  ui_->sunAltitudeEdit->setValidator(new QIntValidator(-90, 90));
  ui_->sunAltitudeEdit->setText(altitude_txt);

  connect(ui_->sunDiameterComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateSunDiameterType);
  connect(ui_->sunAltitudeEdit, &QLineEdit::textChanged, this, &MainWindow::updateSunAltitude);
  connect(ui_->maxHitsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateRayHitsNum);
}


void MainWindow::initRaySettings() {
  // Ray color
  ui_->rayColorComboBox->addItem(tr("real"), QVariant(QColor(0, 0, 0, 0)));
  ui_->rayColorComboBox->addItem(tr("white"), QVariant(QColor(255, 255, 255, 255)));
  ui_->rayColorComboBox->addItem(tr("black"), QVariant(QColor(0, 0, 0, 255)));
  ui_->rayColorComboBox->addItem(tr("customize"), QVariant(QColor(255, 255, 255, 255)));

  // Background color
  ui_->backgroundColorComboBox->addItem(tr("black"), QVariant(QColor(0, 0, 0, 255)));
  ui_->backgroundColorComboBox->addItem(tr("white"), QVariant(QColor(255, 255, 255, 255)));
  ui_->backgroundColorComboBox->addItem(tr("sky"), QVariant(QColor(0, 0, 0, 0)));
  ui_->backgroundColorComboBox->addItem(tr("customize"), QVariant(QColor(0, 0, 0, 255)));

  connect(ui_->rayColorComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateRayColor);
  connect(ui_->backgroundColorComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateBackgroundColor);
}


void MainWindow::initRenderSettings() {
  using namespace IceHalo;

  // Lens type
  ui_->lensComboBox->addItem(tr("linear (normal lens)"), QVariant(static_cast<LensType_t>(LensType::kLinear)));
  ui_->lensComboBox->addItem(tr("fisheye (equal area)"), QVariant(static_cast<LensType_t>(LensType::kEqualArea)));

  // Visible range
  ui_->visibleRangeComboBox->addItem(tr("upper"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kUpper)));
  ui_->visibleRangeComboBox->addItem(tr("lower"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kLower)));
  ui_->visibleRangeComboBox->addItem(tr("front"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kFront)));
  ui_->visibleRangeComboBox->addItem(tr("full"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kFull)));

  connect(ui_->rayNumberSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateTotalRays);
  connect(ui_->lensComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateLensType);
  connect(ui_->visibleRangeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateVisibleRange);
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


void MainWindow::updateRayColor(int index) {
  QColor c = ui_->rayColorComboBox->itemData(index).value<QColor>();
  if (c.alpha() == 255) {
    project_context_->render_ctx_.SetRayColor(static_cast<float>(c.redF()),    // float r
                                              static_cast<float>(c.greenF()),  // float g
                                              static_cast<float>(c.blueF()));  // float b
  } else {
    project_context_->render_ctx_.UseRealRayColor();
  }
  qDebug() << "Updating ray color: " << c;
}


void MainWindow::updateBackgroundColor(int index) {
  QColor c = ui_->backgroundColorComboBox->itemData(index).value<QColor>();
  if (c.alpha() == 255) {
    project_context_->render_ctx_.SetBackgroundColor(static_cast<float>(c.redF()),    // float r
                                                     static_cast<float>(c.greenF()),  // float g
                                                     static_cast<float>(c.blueF()));  // float b
  } else {
    project_context_->render_ctx_.UseSkyBackground();
  }
  qDebug() << "Updating background color: " << c;
}


void MainWindow::updateLensType(int index) {
  LensType_t t = ui_->lensComboBox->itemData(index).value<LensType_t>();
  project_context_->cam_ctx_.SetLensType(static_cast<IceHalo::LensType>(t));
  qDebug() << "Updating lens type: " << t;
}


void MainWindow::updateVisibleRange(int index) {
  VisibleRange_t t = ui_->visibleRangeComboBox->itemData(index).value<VisibleRange_t>();
  project_context_->render_ctx_.SetVisibleRange(static_cast<IceHalo::VisibleRange>(t));
  qDebug() << "Updating visible range: " << t;
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
  float d = ui_->sunDiameterComboBox->itemData(index).toFloat();
  project_context_->sun_ctx_.SetSunDiameter(d);
  qDebug() << "Updating sun diameter: " << d;
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
