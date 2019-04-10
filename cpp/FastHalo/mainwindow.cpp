#include "mainwindow.h"

#include <QtDebug>

#include "render.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), project_context(IceHalo::ProjectContext::CreateDefault()) {
  ui->setupUi(this);
  initUi();
}


MainWindow::~MainWindow() {
  delete ui;
}


void MainWindow::initUi() {
  using namespace IceHalo;

  // Setup some UI properties
  ui->toolbarLayout->setAlignment(Qt::AlignLeft);
  ui->renderSettingLayout->setAlignment(Qt::AlignTop);
  ui->filterSettingLayout->setAlignment(Qt::AlignTop);

  // Sun diameter
  ui->sunDiameterComboBox->addItem(tr("true diameter (0.5°)"), QVariant(0.5f));
  ui->sunDiameterComboBox->addItem(tr("point source (0°)"), QVariant(0.0f));

  // Sun altitude
  ui->sunAltitudeEdit->setValidator(new QIntValidator(-90, 90));

  // Ray color
  ui->rayColorComboBox->addItem(tr("real"), QVariant(QColor(0, 0, 0, 0)));
  ui->rayColorComboBox->addItem(tr("white"), QVariant(QColor(255, 255, 255, 255)));
  ui->rayColorComboBox->addItem(tr("black"), QVariant(QColor(0, 0, 0, 255)));
  ui->rayColorComboBox->addItem(tr("customize"), QVariant(QColor(255, 255, 255, 255)));

  // Background color
  ui->backgroundColorComboBox->addItem(tr("black"), QVariant(QColor(0, 0, 0, 255)));
  ui->backgroundColorComboBox->addItem(tr("white"), QVariant(QColor(255, 255, 255, 255)));
  ui->backgroundColorComboBox->addItem(tr("sky"), QVariant(QColor(0, 0, 0, 0)));
  ui->backgroundColorComboBox->addItem(tr("customize"), QVariant(QColor(0, 0, 0, 255)));

  // Lens type
  ui->lensComboBox->addItem(tr("linear (normal lens)"), QVariant(static_cast<LensType_t>(LensType::kLinear)));
  ui->lensComboBox->addItem(tr("fisheye (equal area)"), QVariant(static_cast<LensType_t>(LensType::kEqualArea)));

  // Visible range
  ui->visibleRangeComboBox->addItem(tr("upper"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kUpper)));
  ui->visibleRangeComboBox->addItem(tr("lower"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kLower)));
  ui->visibleRangeComboBox->addItem(tr("front"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kFront)));
  ui->visibleRangeComboBox->addItem(tr("full"), QVariant(static_cast<VisibleRange_t>(VisibleRange::kFull)));

  // Connect signals and slots
  connect(ui->filterEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::enableFilterSettings);
  connect(ui->scatterProbSlider, &QSlider::valueChanged, this, &MainWindow::updateScatterProb);
  connect(ui->rayNumberSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateTotalRays);
  connect(ui->maxHitsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateRayHitsNum);
  connect(ui->sunDiameterComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateSunDiameterType);
  connect(ui->sunAltitudeEdit, &QLineEdit::textChanged, this, &MainWindow::updateSunPosition);
  connect(ui->rayColorComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateRayColor);
  connect(ui->backgroundColorComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateBackgroundColor);
  connect(ui->lensComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateLensType);
  connect(ui->visibleRangeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateVisibleRange);
}


void MainWindow::enableFilterSettings(bool enable) {
  ui->specificRadioButton->setEnabled(enable);
  ui->generalRadioButton->setEnabled(enable);
  ui->symBCheckBox->setEnabled(enable);
  ui->symDCheckBox->setEnabled(enable);
  ui->symPCheckBox->setEnabled(enable);
}


void MainWindow::updateTotalRays(int ray_num) {
  size_t init_ray_num = static_cast<size_t>(ray_num);
  project_context->SetInitRayNum(init_ray_num);
  qDebug() << "Updating total ray number: " << ray_num;
}


void MainWindow::updateRayColor(int index) {
  QColor c = ui->rayColorComboBox->itemData(index).value<QColor>();
  if (c.alpha() == 255) {
    project_context->render_ctx_.SetRayColor(static_cast<float>(c.redF()),    // float r
                                             static_cast<float>(c.greenF()),  // float g
                                             static_cast<float>(c.blueF()));  // float b
  } else {
    project_context->render_ctx_.UseRealRayColor();
  }
  qDebug() << "Updating ray color: " << c;
}


void MainWindow::updateBackgroundColor(int index) {
  QColor c = ui->backgroundColorComboBox->itemData(index).value<QColor>();
  if (c.alpha() == 255) {
    project_context->render_ctx_.SetBackgroundColor(static_cast<float>(c.redF()),    // float r
                                                    static_cast<float>(c.greenF()),  // float g
                                                    static_cast<float>(c.blueF()));  // float b
  } else {
    project_context->render_ctx_.UseSkyBackground();
  }
  qDebug() << "Updating background color: " << c;
}


void MainWindow::updateLensType(int index) {
  LensType_t t = ui->lensComboBox->itemData(index).value<LensType_t>();
  project_context->cam_ctx_.SetLensType(static_cast<IceHalo::LensType>(t));
  qDebug() << "Updating lens type: " << t;
}


void MainWindow::updateVisibleRange(int index) {
  VisibleRange_t t = ui->visibleRangeComboBox->itemData(index).value<VisibleRange_t>();
  project_context->render_ctx_.SetVisibleRange(static_cast<IceHalo::VisibleRange>(t));
  qDebug() << "Updating visible range: " << t;
}


void MainWindow::updateRayHitsNum(int n) {
  project_context->SetRayHitNum(n);
  qDebug() << "Updating ray hits number: " << n;
}


void MainWindow::updateSunPosition(const QString& altitude_txt) {
  float altitude = altitude_txt.toFloat();
  project_context->sun_ctx_.SetSunAltitude(altitude);
  qDebug() << "Updating sun altitude: " << altitude;
}


void MainWindow::updateSunDiameterType(int index) {
  float d = ui->sunDiameterComboBox->itemData(index).toFloat();
  project_context->sun_ctx_.SetSunDiameter(d);
  qDebug() << "Updating sun diameter: " << d;
}


void MainWindow::updateScatterProb(int v) {
  double prob = v / 100.0;
  QString prob_str = tr("Probability: %1").arg(prob, 0, 'f', 2);
  ui->scatterProbLabel->setText(prob_str);
  qDebug() << "Updating scatter prob: " << v << ", " << prob_str;
}


void MainWindow::updateSimulationContext() {
  updateTotalRays(ui->rayNumberSpinBox->value());
  updateRayHitsNum(ui->maxHitsSpinBox->value());

  // Multiscatter settings
  // TODO

  // Wavelength settings
  project_context->ClearWavelengthInfo();
  // TODO

  updateSunPosition(ui->sunAltitudeEdit->text());
  updateSunDiameterType(ui->sunDiameterComboBox->currentIndex());
}
