#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow) {
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

  // connect signals and slots
  QObject::connect(ui->filterEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::enableFilterSettings);
  QObject::connect(ui->scatterProbSlider, &QSlider::valueChanged, this, &MainWindow::updateScatterProb);
}

void MainWindow::enableFilterSettings(bool enable) {
  ui->specificRadioButton->setEnabled(enable);
  ui->generalRadioButton->setEnabled(enable);
  ui->symBCheckBox->setEnabled(enable);
  ui->symDCheckBox->setEnabled(enable);
  ui->symPCheckBox->setEnabled(enable);
}

void MainWindow::updateScatterProb(int v) {
  double prob = v / 100.0;
  QString prob_str = QString("Probability: %1").arg(prob, 0, 'f', 2);
  ui->scatterProbLabel->setText(prob_str);
}
