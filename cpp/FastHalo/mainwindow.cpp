#include "mainwindow.h"
#include "datamodel.h"

#include <QtDebug>

#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), simulation_context(IceHalo::SimulationContext::CreateDefault()) {
  ui->setupUi(this);
  initUi();
}


MainWindow::~MainWindow() {
  delete ui;
}


void MainWindow::initUi() {
  // setup some UI properties
  ui->toolbarLayout->setAlignment(Qt::AlignLeft);
  ui->renderSettingLayout->setAlignment(Qt::AlignTop);
  ui->filterSettingLayout->setAlignment(Qt::AlignTop);

  ui->sunDiameterComboBox->addItem(tr("true diameter (0.5°)"), QVariant(SunDiameterType::kReal));
  ui->sunDiameterComboBox->addItem(tr("point source (0°)"), QVariant(SunDiameterType::kPoint));

  ui->sunAltitudeEdit->setValidator(new QIntValidator(-90, 90));

  // connect signals and slots
  connect(ui->filterEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::enableFilterSettings);
  connect(ui->scatterProbSlider, &QSlider::valueChanged, this, &MainWindow::updateScatterProb);
  connect(ui->rayNumberSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateTotalRays);
  connect(ui->maxHitsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::updateRayHitsNum);
  connect(ui->sunDiameterComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateSunDiameterType);
  connect(ui->sunAltitudeEdit, &QLineEdit::textChanged, this, &MainWindow::updateSunPosition);
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
  simulation_context->SetTotalInitRays(init_ray_num);
  qDebug() << "Updating total ray number! Value: " << ray_num;
}


void MainWindow::updateRayHitsNum(int n) {
  simulation_context->SetRayHitsNum(n);
  qDebug() << "Updating ray hits number! Value: " << n;
}


void MainWindow::updateSunPosition(const QString& altitude_txt) {
  float altitude = altitude_txt.toFloat();
  simulation_context->SetSunRayDir(90.0f, altitude);
  qDebug() << "Updating sun altitude! LineEdit text: " << altitude_txt << " value: " << altitude;
}


void MainWindow::updateSunDiameterType(int index) {
  float d = 0.0f;
  switch (ui->sunDiameterComboBox->itemData(index).toInt()) {
    case SunDiameterType::kReal:
      d = 0.5f;
      break;
    case SunDiameterType::kPoint:
      d = 0.0f;
      break;
    case SunDiameterType::kOther:
      // WARNING: should not be here!
      break;
  }
  simulation_context->SetSunDiameter(d);
  qDebug() << "Updating sun diameter! ComboBox index: " << index << ", value: " << d;
}


void MainWindow::updateScatterProb(int v) {
  double prob = v / 100.0;
  QString prob_str = tr("Probability: %1").arg(prob, 0, 'f', 2);
  ui->scatterProbLabel->setText(prob_str);
  qDebug() << "Updating scatter prob! Value: " << v << ", " << prob_str;
}


void MainWindow::updateSimulationContext() {
  updateTotalRays(ui->rayNumberSpinBox->value());
  updateRayHitsNum(ui->maxHitsSpinBox->value());

  // Multiscatter settings
  simulation_context->ClearMultiScatterContext();
  // TODO

  // Wavelength settings
  simulation_context->ClearWavelength();
  // TODO

  updateSunPosition(ui->sunAltitudeEdit->text());
  updateSunDiameterType(ui->sunDiameterComboBox->currentIndex());
}
