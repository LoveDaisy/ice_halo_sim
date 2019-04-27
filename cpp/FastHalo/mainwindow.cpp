#include "mainwindow.h"

#include <QColorDialog>
#include <QPropertyAnimation>
#include <QtDebug>

#include "iconbutton.h"
#include "icons.h"
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
  ui_->filterSettingLayout->setAlignment(Qt::AlignTop);

  initBasicSettings();
  initScatterTab();
  initCrystalList();

  ui_->crystalsTable->setFocus();

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
  ui_->sunAltitudeEdit->setValidator(new QDoubleValidator(-90, 90, 1));
  ui_->sunAltitudeEdit->setText(altitude_txt);
  updateSunAltitude(altitude_txt);

  // Max hits
  auto max_hits_widget = ui_->maxHitsSpinBox;
  max_hits_widget->setMaximum(project_context_->kMaxRayHitNum);
  max_hits_widget->setMinimum(project_context_->kMinRayHitNum);
  max_hits_widget->setValue(project_context_->GetRayHitNum());
  updateRayHitsNum(max_hits_widget->value());

  // Ray number
  auto ray_num_widget = ui_->rayNumberSpinBox;
  ray_num_widget->setMaximum(kMaxInitRayNum);
  ray_num_widget->setMinimum(project_context_->kMinRayHitNum);
  ray_num_widget->setValue(static_cast<int>(project_context_->GetInitRayNum()));
  updateTotalRays(ray_num_widget->value());

  // Wavelength
  for (const auto& wl : getWavelengthData()) {
    ui_->wavelengthComboBox->addItem(wl.name_);
  }
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

  connect(scatter_tab_add_btn_, &QToolButton::clicked, this, &MainWindow::insertScatterTab);
}


void MainWindow::insertScatterTab() {
  int current_item_cnt = ui_->scatterTabLayout->count();
  auto btn = createScatterTab();
  ui_->scatterTabLayout->insertWidget(current_item_cnt - 1, btn);
  scatter_tab_group_->addButton(btn);
  btn->setChecked(true);

  updateScatterTabs();

  connect(btn, &IconButton::closeTab, this, [=] {
    auto anim = new QPropertyAnimation(btn, "maximumWidth");
    anim->setStartValue(btn->geometry().width());
    anim->setEndValue(0);
    anim->setDuration(200);
    anim->setTargetObject(btn);
    anim->start(QAbstractAnimation::DeleteWhenStopped);

    connect(anim, &QPropertyAnimation::finished, this, &MainWindow::updateScatterTabs);
  });
}


void MainWindow::initCrystalList() {
  auto table = ui_->crystalsTable;

  crystal_list_model_ = new QStandardItemModel();
  crystal_list_model_->setHorizontalHeaderLabels(QStringList{ "Enable", "Name", "", "Pop." });

  table->setMouseTracking(true);
  table->setModel(crystal_list_model_);
  table->setColumnWidth(0, 50);
  table->setColumnWidth(2, 30);
  table->setColumnWidth(3, 40);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  table->horizontalHeader()->setMinimumSectionSize(20);
  table->verticalHeader()->hide();
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setShowGrid(false);

  insertCrystalItem();  // TODO

  connect(ui_->crystalsAddButton, &QToolButton::clicked, this, &MainWindow::insertCrystalItem);
  connect(ui_->crystalsRemoveButton, &QToolButton::clicked, this, &MainWindow::removeCurrentCrystal);
  connect(table, &QTableView::clicked, this, [=](const QModelIndex& index) {
    if (index.column() != 2) {
      return;
    }
    auto link = index.data(Qt::UserRole).toBool();
    crystal_list_model_->setData(index, !link, Qt::UserRole);
    if (link) {
      crystal_list_model_->setData(index, Icons::getIcon(Icons::kUnlink), Qt::DecorationRole);
    } else {
      crystal_list_model_->setData(index, Icons::getIcon(Icons::kLink), Qt::DecorationRole);
    }
  });
  connect(table, &QTableView::entered, this, [=](const QModelIndex& index) {
    if (index.column() == 0 || index.column() == 2) {
      ui_->crystalsTable->setCursor(Qt::PointingHandCursor);
    } else {
      ui_->crystalsTable->unsetCursor();
    }
  });
}


void MainWindow::insertCrystalItem() {
  auto item_check = new QStandardItem();
  auto item_name = new QStandardItem("Hex-Prism");
  auto item_link = new QStandardItem();
  auto item_pop = new QStandardItem("100");

  item_check->setCheckable(true);
  item_check->setEditable(false);
  item_check->setData(Qt::Unchecked, Qt::CheckStateRole);
  item_check->setTextAlignment(Qt::AlignCenter);
  item_link->setEditable(false);
  item_link->setData(Icons::getIcon(Icons::kLink), Qt::DecorationRole);
  item_link->setData(true, Qt::UserRole);
  item_name->setEditable(true);
  item_pop->setEditable(true);

  auto row_count = crystal_list_model_->rowCount();
  crystal_list_model_->setItem(row_count, 0, item_check);
  crystal_list_model_->setItem(row_count, 1, item_name);
  crystal_list_model_->setItem(row_count, 2, item_link);
  crystal_list_model_->setItem(row_count, 3, item_pop);

  auto table = ui_->crystalsTable;
  table->selectRow(row_count);
}


void MainWindow::removeCurrentCrystal() {
  auto table = ui_->crystalsTable;
  auto index = table->currentIndex();
  if (!index.isValid()) {
    return;
  }

  auto row_count = crystal_list_model_->rowCount();
  if (row_count <= 1) {
    return;
  }

  auto curr_row = index.row();
  crystal_list_model_->removeRow(curr_row);
  if (curr_row == row_count - 1) {
    table->selectRow(curr_row - 1);
  } else {
    table->selectRow(curr_row);
  }
}


IconButton* MainWindow::createScatterTab() {
  auto btn = new IconButton("");
  btn->setChecked(true);
  btn->setIcons(Icons::getIcon(Icons::kCloseNormal), Icons::getIcon(Icons::kCloseOn));
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
  qDebug() << "Updating total ray number:" << ray_num;
}


void MainWindow::updateWavelength() {
  int curr_idx = ui_->wavelengthComboBox->currentIndex();
  auto& wl_data = getWavelengthData()[curr_idx];
  if (wl_data.customized_) {
    // TODO
  }

  qDebug() << "Updating wavelength:";
  project_context_->ClearWavelengthInfo();
  for (const auto& wl : wl_data.info_) {
    qDebug().nospace() << "  wavelength: " << wl.wavelength << ", weight: " << wl.weight;
    project_context_->AddWavelengthInfo(wl.wavelength, wl.weight);
  }
}


void MainWindow::updateRayHitsNum(int n) {
  project_context_->SetRayHitNum(n);
  qDebug() << "Updating ray hits number:" << n;
}


void MainWindow::updateSunAltitude(const QString& altitude_txt) {
  auto widget = ui_->sunAltitudeEdit;
  auto& ctx = project_context_->sun_ctx_;
  int pos;
  QString txt_cpy = altitude_txt;
  if (widget->validator()->validate(txt_cpy, pos) != QValidator::Acceptable) {
    QString value = QString::asprintf("%.1f", static_cast<double>(ctx.GetSunAltitude()));
    widget->setText(value);
  } else {
    ctx.SetSunAltitude(txt_cpy.toFloat());
  }
  qDebug() << "Updating sun altitude:" << altitude_txt;
}


void MainWindow::updateSunDiameterType(int index) {
  double d = ui_->sunDiameterComboBox->itemData(index).toDouble();
  project_context_->sun_ctx_.SetSunDiameter(static_cast<float>(d));
  QString d_txt = QString::asprintf("Sun diameter: %.1f°", d);
  ui_->sunDiameterLabel->setText(d_txt);
  qDebug() << "Updating sun diameter:" << d_txt;
}


void MainWindow::updateScatterTabs() {
  int tab_cnt = ui_->scatterTabLayout->count() - 1;
  if (tab_cnt <= 0) {
    return;
  }

  // Remove invisible tabs
  bool update_check = false;  // If a checked tab is removed, then the next tab will be checked.
  for (int i = 0; i < tab_cnt; i++) {
    auto tab = static_cast<IconButton*>(ui_->scatterTabLayout->itemAt(i)->widget());
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
    auto tab = static_cast<IconButton*>(ui_->scatterTabLayout->itemAt(i)->widget());
    tab->setText(tab_txt);
    tab->enableIcon(true);
  }

  // If only one tab, disable close icon
  if (tab_cnt <= 1 && ui_->scatterTabLayout->itemAt(0)) {
    auto tab = static_cast<IconButton*>(ui_->scatterTabLayout->itemAt(0)->widget());
    tab->enableIcon(false);
  }

  // If no checked (the right most checked tab is removed), check the first one
  if (!scatter_tab_group_->checkedButton()) {
    auto tab = static_cast<IconButton*>(ui_->scatterTabLayout->itemAt(0)->widget());
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
  }

  return wl_data;
}
