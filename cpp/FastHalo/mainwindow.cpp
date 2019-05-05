#include "mainwindow.h"

#include <QColorDialog>
#include <QPropertyAnimation>
#include <QStackedWidget>
#include <QtDebug>
#include <algorithm>

#include "iconbutton.h"
#include "icons.h"
#include "render.h"
#include "spinboxdelegate.h"
#include "ui_mainwindow.h"


int MainWindow::current_crystal_id_ = 1;


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow),
      project_context_(IceHalo::ProjectContext::CreateDefault()) {
  ui_->setupUi(this);
  initUi();
}


MainWindow::~MainWindow() {
  delete ui_;
}


void MainWindow::updateRayHitsNum(int n) {
  qDebug() << "updateRayHitsNum()";
  gui_data_.max_hits_ = n;
  qDebug() << "Updating ray hits number:" << n;
}


void MainWindow::updateSunDiameter(int index) {
  qDebug() << "updateSunDiameter()";
  double d = ui_->sunDiameterComboBox->itemData(index).toDouble();

  // Update data
  gui_data_.sun_diameter_ = static_cast<float>(d);

  // Update UI
  QString d_txt = QString::asprintf("Sun diameter: %.1f°", d);
  ui_->sunDiameterLabel->setText(d_txt);

  qDebug() << "Updating sun diameter:" << d_txt;
}


void MainWindow::updateTotalRays(int ray_num) {
  qDebug() << "updateTotalRays()";
  gui_data_.ray_number_ = ray_num;
  qDebug() << "Updating total ray number:" << ray_num;
}


void MainWindow::updateWavelength(int index) {
  qDebug() << "updateWavelength()";
  auto& wl_data = WavelengthData::getData()[static_cast<size_t>(index)];
  if (wl_data.customized_) {
    // TODO: open new window
  }

  qDebug() << "Wavelength data:" << index;
  gui_data_.wavelength_data_idx_ = index;
}


void MainWindow::addScatter() {
  qDebug() << "addScatter()";

  // Update model
  gui_data_.multi_scatter_data_.emplace_back(1.0f);
  auto& curr_scatter = gui_data_.multi_scatter_data_.back();
  auto& first_scatter = gui_data_.multi_scatter_data_.front();
  auto& crystals = gui_data_.crystal_store_;
  curr_scatter.crystals_.clear();
  for (const auto& kv : crystals) {
    curr_scatter.crystals_.emplace_back(kv.first);
  }
  std::sort(
      curr_scatter.crystals_.begin(), curr_scatter.crystals_.end(),
      [=](const MultiScatterData::CrystalItemData& a, const MultiScatterData::CrystalItemData& b) {
        return a.crystal_id < b.crystal_id;
      });
  for (size_t i = 0; i < curr_scatter.crystals_.size(); i++) {
    auto& c0 = first_scatter.crystals_.at(i);
    auto& c1 = curr_scatter.crystals_.at(i);
    if (c0.linked) {
      c1.population = c0.population;
    } else {
      c1.linked = false;
    }
  }

  // Update scatter tabs
  auto layout = ui_->scatterTabLayout;
  int curr_item_cnt = layout->count();
  auto btn = createScatterTab();
  btn->setText(getScatterTabText(curr_item_cnt - 1));
  btn->setChecked(true);
  btn->enableIcon(true);
  layout->insertWidget(curr_item_cnt - 1, btn);
  scatter_tab_group_->addButton(btn);

  if (curr_item_cnt == 1) {  // Disable close action when there is only one tab button.
    btn->enableIcon(false);
  } else {  // Enable close action when there are more than one tab buttons.
    for (int i = 0; i < curr_item_cnt; i++) {
      static_cast<IconButton*>(layout->itemAt(i)->widget())->enableIcon(true);
    }
  }

  connect(btn, &IconButton::closeTab, this, &MainWindow::removeScatter);
}


void MainWindow::removeScatter(IconButton* sender) {
  qDebug() << "removeScatter()";

  auto layout = ui_->scatterTabLayout;
  int index = layout->indexOf(sender);
  if (!sender || index < 0 || static_cast<size_t>(index) >= gui_data_.multi_scatter_data_.size()) {
    return;
  }

  // Update model
  auto& data = gui_data_.multi_scatter_data_;
  data.erase(data.begin() + index);

  // Update view
  auto anim = new QPropertyAnimation(sender, "maximumWidth");
  anim->setStartValue(sender->geometry().width());
  anim->setEndValue(0);
  anim->setDuration(200);
  anim->setTargetObject(sender);
  anim->start(QAbstractAnimation::DeleteWhenStopped);

  // Update check state
  if (sender->isChecked()) {
    auto next_idx = index + 1;
    if (next_idx == layout->count() - 1) {  // The last tab
      next_idx = index - 1;
    }
    static_cast<IconButton*>(layout->itemAt(next_idx)->widget())->setChecked(true);
  }

  // After animation finished, delete button and udpate tab text
  connect(anim, &QPropertyAnimation::finished, this, [=] {
    layout->removeWidget(sender);
    scatter_tab_group_->removeButton(sender);
    sender->deleteLater();

    if (layout->count() == 2) {  // Disable close action when there is only one tab button
      static_cast<IconButton*>(layout->itemAt(0)->widget())->enableIcon(false);
    }

    // Update tab text
    auto tab_cnt = layout->count() - 1;
    for (int i = 0; i < tab_cnt; i++) {
      auto tab_txt = getScatterTabText(i);
      auto curr_tab_btn = static_cast<IconButton*>(layout->itemAt(i)->widget());
      curr_tab_btn->setText(tab_txt);
    }
  });
}


void MainWindow::updateScatterProb(int v) {
  qDebug() << "updateScatterProb()";
  double prob = getScatterProb(v);

  // Update UI
  QString prob_str = getScatterProbText(prob);
  ui_->scatterProbLabel->setText(prob_str);

  qDebug() << "Updating scatter prob:" << v << "," << prob_str;

  // Update data
  auto scatter_data = getCurrentScatterData();
  if (scatter_data) {
    scatter_data->prob_ = static_cast<float>(prob);
  }
}


void MainWindow::refreshScatterProb() {
  qDebug() << "refreshScatterProb()";

  double prob = getScatterProb();
  int v = std::min(std::max(static_cast<int>(prob * 100), 0), 100);

  // Update UI
  QString prob_str = getScatterProbText(prob);
  ui_->scatterProbLabel->setText(prob_str);
  ui_->scatterProbSlider->setValue(v);

  qDebug() << "Refreshing scatter prob:" << v << "," << prob_str;
}


void MainWindow::addCrystal() {
  qDebug() << "addCrystal()";

  // Update data
  auto crystal_data = CrystalData(current_crystal_id_);
  crystal_data.name_ = tr("Crystal %1").arg(current_crystal_id_);
  crystal_data.height_[0] = 1.6f;
  gui_data_.crystal_store_.emplace(current_crystal_id_, crystal_data);

  for (auto& m : gui_data_.multi_scatter_data_) {
    m.crystals_.emplace_back(current_crystal_id_);
  }

  // Update crystal list
  auto scatter_data = getCurrentScatterData();
  if (!scatter_data) {
    qWarning() << "Warning! scatter data is null!";
    return;
  }
  auto item_check = new QStandardItem();
  auto item_name = new QStandardItem(crystal_data.name_);
  auto item_link = new QStandardItem();
  auto item_pop = new QStandardItem(QString::number(scatter_data->crystals_.back().population));

  item_check->setCheckable(true);
  item_check->setEditable(false);
  item_check->setData(Qt::Unchecked, Qt::CheckStateRole);
  item_check->setTextAlignment(Qt::AlignCenter);
  item_link->setEditable(false);
  item_link->setData(Icons::getIcon(Icons::kLink), Qt::DecorationRole);
  item_link->setData(true, Qt::UserRole);
  item_name->setEditable(true);
  item_name->setData(current_crystal_id_, Qt::UserRole);
  item_pop->setEditable(true);
  item_pop->setData(scatter_data->crystals_.back().population, Qt::EditRole);

  auto row_count = crystal_list_model_->rowCount();
  crystal_list_model_->setItem(row_count, 0, item_check);
  crystal_list_model_->setItem(row_count, 1, item_name);
  crystal_list_model_->setItem(row_count, 2, item_link);
  crystal_list_model_->setItem(row_count, 3, item_pop);

  auto table = ui_->crystalsTable;
  table->selectRow(row_count);

  current_crystal_id_++;

  refreshCrystalInfo();
}


void MainWindow::removeCurrentCrystal() {
  qDebug() << "removeCurrentCrystal()";

  auto table = ui_->crystalsTable;
  auto index = table->currentIndex();
  if (!index.isValid()) {
    return;
  }

  auto curr_row = index.row();
  auto crystal_id = crystal_list_model_->item(curr_row, 1)->data(Qt::UserRole).toInt();

  // Remove data
  gui_data_.crystal_store_.erase(crystal_id);
  for (auto& m : gui_data_.multi_scatter_data_) {
    decltype(m.crystals_) tmp_list;
    for (auto& c : m.crystals_) {
      if (c.crystal_id != crystal_id) {
        tmp_list.emplace_back(c);
      }
    }
    m.crystals_.swap(tmp_list);
  }

  // Remove UI widget
  auto row_count = crystal_list_model_->rowCount();
  crystal_list_model_->removeRow(curr_row);
  if (curr_row == row_count - 1) {
    table->selectRow(curr_row - 1);
  } else {
    table->selectRow(curr_row);
  }

  refreshCrystalInfo();
}


void MainWindow::refreshCrystalList() {
  qDebug() << "refreshCrystalList()";

  auto scatter = getCurrentScatterData();
  if (!scatter) {
    qWarning() << "Warning! No scatter data!";
    return;
  }

  auto& crystals = scatter->crystals_;
  if (static_cast<int>(crystals.size()) != crystal_list_model_->rowCount()) {
    qWarning() << "Warning! crystal count mismatch!";
    return;
  }

  for (int i = 0; i < crystal_list_model_->rowCount(); i++) {
    auto& crystal_item_data = crystals.at(static_cast<size_t>(i));

    // refresh enable state
    auto item = crystal_list_model_->item(i, 0);
    auto check_state = crystal_item_data.enabled ? Qt::Checked : Qt::Unchecked;
    item->setData(check_state, Qt::CheckStateRole);

    // refresh name
    item = crystal_list_model_->item(i, 1);
    auto old_name = item->data(Qt::DisplayRole);
    auto old_id = item->data(Qt::UserRole);
    auto new_name = gui_data_.crystal_store_.at(crystal_item_data.crystal_id).name_;
    auto new_id = crystal_item_data.crystal_id;
    if (old_id != new_id || old_name != new_name) {
      item->setData(new_name, Qt::DisplayRole);
      item->setData(crystal_item_data.crystal_id, Qt::UserRole);
    }

    // refresh link state
    item = crystal_list_model_->item(i, 2);
    if (item->data(Qt::UserRole) != crystal_item_data.linked) {
      item->setData(crystal_item_data.linked, Qt::UserRole);
      item->setData(
          crystal_item_data.linked ? Icons::getIcon(Icons::kLink) : Icons::getIcon(Icons::kUnlink),
          Qt::DecorationRole);
    }

    // refresh population
    item = crystal_list_model_->item(i, 3);
    auto old_pop = item->data(Qt::EditRole);
    auto new_pop = crystal_item_data.population;
    if (old_pop != new_pop) {
      item->setData(new_pop, Qt::EditRole);
    }
  }
}


void MainWindow::toggleCrystalLinkState(const QModelIndex& index) {
  qDebug() << "toggleCrystalLinkState()";
  auto link = index.data(Qt::UserRole).toBool();

  // Update UI model
  if (link) {
    crystal_list_model_->setData(index, Icons::getIcon(Icons::kUnlink), Qt::DecorationRole);
  } else {
    crystal_list_model_->setData(index, Icons::getIcon(Icons::kLink), Qt::DecorationRole);
  }
  crystal_list_model_->setData(index, !link, Qt::UserRole);

  // Update data
  auto crystal_item_data = getCrystalItemData(index);
  if (!crystal_item_data) {
    return;
  }

  for (auto& m : gui_data_.multi_scatter_data_) {
    for (auto& c : m.crystals_) {
      if (c.crystal_id == crystal_item_data->crystal_id) {
        c.linked = !link;
        if (!link) {
          c.population = crystal_item_data->population;
        }
      }
    }
  }
}


void MainWindow::updateCrystalData(const QModelIndex& index) {
  qDebug() << "updateCrystalData()";

  auto col = index.column();
  auto crystal_item_data = getCrystalItemData(index);
  auto crystal_data = getCrystalData(index);
  if (!crystal_item_data || !crystal_data) {
    return;
  }

  switch (col) {
    case 0:  // Enabled
      crystal_item_data->enabled = index.data(Qt::CheckStateRole) == Qt::Checked;
      break;
    case 1:  // Name
      crystal_data->name_ = index.data(Qt::DisplayRole).toString();
      break;
    case 2:  // Linked
      crystal_item_data->linked = index.data(Qt::UserRole).toBool();
      break;
    case 3: {  // Population
      auto pop = index.data(Qt::EditRole).toInt();
      if (crystal_item_data->linked) {
        for (auto& m : gui_data_.multi_scatter_data_) {
          for (auto& c : m.crystals_) {
            if (c.crystal_id == crystal_item_data->crystal_id) {
              c.population = pop;
            }
          }
        }
      } else {
        crystal_item_data->population = pop;
      }
    } break;
    default:
      break;
  }
}


void MainWindow::updateCrystalType(int combo_idx) {
  qDebug() << "updateCrystalType()";

  // Update model data
  auto type_combo_box = ui_->crystalTypeComboBox;
  auto type =
      static_cast<IceHalo::CrystalType>(type_combo_box->itemData(combo_idx, Qt::UserRole).toInt());
  auto crystal_data = getCurrentCrystalData();
  if (crystal_data) {
    crystal_data->type_ = type;
  }

  // Update UI widget
  if (type == IceHalo::CrystalType::kPyramid) {
    ui_->pyramidParameterPanel->setEnabled(true);
  } else if (type == IceHalo::CrystalType::kPrism) {
    ui_->pyramidParameterPanel->setEnabled(false);
  } else {
    qWarning() << "Invalid crystal type!";
  }
}


void MainWindow::refreshCrystalInfo() {
  qDebug() << "refreshCrystalInfo()";

  auto table = ui_->crystalsTable;
  auto curr_index = table->currentIndex();

  if (!curr_index.isValid()) {
    setCrystalPanelEnabled(false);
    return;
  }

  // Get current data
  auto crystal_data = getCrystalData(curr_index);
  auto crystal_item_data = getCrystalItemData(curr_index);
  if (!crystal_data || !crystal_item_data) {
    return;
  }

  // Update enabled
  if (crystal_item_data->enabled) {
    setCrystalPanelEnabled(true);
  } else {
    setCrystalPanelEnabled(false);
  }

  // Update crystal type
  auto crystal_type = crystal_data->type_;
  auto type_combo_box = ui_->crystalTypeComboBox;
  for (int i = 0; i < type_combo_box->count(); i++) {
    if (type_combo_box->itemData(i, Qt::UserRole) == static_cast<int>(crystal_type)) {
      type_combo_box->setCurrentIndex(i);
      break;
    }
  }

  // Update crystal height
  crystal_height_edit_->setDataSource(crystal_data->height_ + 0);
  auto height = crystal_data->height_[0];
  crystal_height_edit_->setText(crystal_height_edit_->formatValue(height));

  // Update pyramid height panel
  pyramid_upper_height_edit_->setDataSource(crystal_data->height_ + 1);
  pyramid_lower_height_edit_->setDataSource(crystal_data->height_ + 2);
  auto pyramid_height_panel = ui_->pyramidParameterPanel;
  if (crystal_data->type_ == IceHalo::CrystalType::kPrism) {
    pyramid_height_panel->setEnabled(false);
    pyramid_upper_height_edit_->setText("0.0");
    pyramid_lower_height_edit_->setText("0.0");
  } else if (crystal_data->type_ == IceHalo::CrystalType::kPyramid) {
    pyramid_height_panel->setEnabled(crystal_item_data->enabled);
    pyramid_upper_height_edit_->setText(
        pyramid_upper_height_edit_->formatValue(crystal_data->height_[1]));
    pyramid_lower_height_edit_->setText(
        pyramid_lower_height_edit_->formatValue(crystal_data->height_[2]));
  }
  QComboBox* combos[] = {
    ui_->upperMiller1ComboBox,  // upper 1
    ui_->upperMiller2ComboBox,  // upper 2
    ui_->lowerMiller1ComboBox,  // lower 1
    ui_->lowerMiller2ComboBox,  // lower 2
  };
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < combos[i]->count(); j++) {
      if (combos[i]->itemData(j, Qt::UserRole) == crystal_data->miller_idx_[i]) {
        combos[i]->setCurrentIndex(j);
        break;
      }
    }
  }

  // TODO
}


void MainWindow::enableFilterSettings(bool enable) {
  ui_->specificRadioButton->setEnabled(enable);
  ui_->generalRadioButton->setEnabled(enable);
  ui_->symBCheckBox->setEnabled(enable);
  ui_->symDCheckBox->setEnabled(enable);
  ui_->symPCheckBox->setEnabled(enable);
}


void MainWindow::initUi() {
  using namespace IceHalo;

  // Setup some UI properties
  ui_->filterSettingLayout->setAlignment(Qt::AlignTop);

  initBasicSettings();
  initScatterTab();
  initCrystalList();
  initCrystalInfoPanel();

  // Connect signals and slots
  connect(ui_->filterEnableCheckBox, &QCheckBox::clicked, this, &MainWindow::enableFilterSettings);
  connect(ui_->scatterProbSlider, &QSlider::valueChanged, this, &MainWindow::updateScatterProb);
}


void MainWindow::initBasicSettings() {
  // Sun diameter
  for (const auto& s : GuiData::getSunDiameterData()) {
    ui_->sunDiameterComboBox->addItem(s.text, s.value);
  }
  updateSunDiameter(0);

  // Sun altitude
  sun_altitude_edit_ = new FloatLineEdit(-90, 90, 1);
  ui_->basicSettingLayout->addWidget(sun_altitude_edit_, 0, 1);
  sun_altitude_edit_->setText(sun_altitude_edit_->formatValue(gui_data_.sun_altitude_));
  sun_altitude_edit_->setDataSource(&gui_data_.sun_altitude_);

  // Max hits
  auto max_hits_widget = ui_->maxHitsSpinBox;
  max_hits_widget->setMaximum(GuiData::kMaxHitsNum);
  max_hits_widget->setMinimum(GuiData::kMinHitsNum);
  max_hits_widget->setValue(gui_data_.max_hits_);
  updateRayHitsNum(max_hits_widget->value());

  // Ray number
  auto ray_num_widget = ui_->rayNumberSpinBox;
  ray_num_widget->setMaximum(GuiData::kMaxRayNum);
  ray_num_widget->setMinimum(GuiData::kMinRayNum);
  ray_num_widget->setValue(gui_data_.ray_number_);
  updateTotalRays(ray_num_widget->value());

  // Wavelength
  for (const auto& wl : WavelengthData::getData()) {
    ui_->wavelengthComboBox->addItem(wl.name_);
  }
  updateWavelength(0);

  connect(ui_->sunDiameterComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateSunDiameter);
  connect(ui_->maxHitsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &MainWindow::updateRayHitsNum);
  connect(ui_->rayNumberSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &MainWindow::updateTotalRays);
  connect(ui_->wavelengthComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateWavelength);
}


void MainWindow::initScatterTab() {
  scatter_tab_add_btn_ = createScatterAddButton();
  scatter_tab_group_ = new QButtonGroup(ui_->scatterTabFrame);
  ui_->scatterTabLayout->addWidget(scatter_tab_add_btn_);

  addScatter();

  connect(scatter_tab_add_btn_, &QToolButton::clicked, this, &MainWindow::addScatter);
  connect(scatter_tab_group_, QOverload<int, bool>::of(&QButtonGroup::buttonToggled), this, [=] {
    refreshCrystalList();
    refreshCrystalInfo();
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
  table->setItemDelegateForColumn(3, new SpinBoxDelegate());

  // Add crystal
  connect(ui_->crystalsAddButton, &QToolButton::clicked, this, [=] {
    ui_->crystalsTable->setFocus();
    addCrystal();
  });

  // Remove crystal
  connect(ui_->crystalsRemoveButton, &QToolButton::clicked, this, [=] {
    ui_->crystalsTable->setFocus();
    removeCurrentCrystal();
  });

  // Click link/unlink icon
  connect(table, &QTableView::clicked, this, [=](const QModelIndex& index) {
    if (index.column() != 2) {
      return;
    }
    toggleCrystalLinkState(index);
  });

  // Cursor. Use hand cursor for link/unlink icon
  connect(table, &QTableView::entered, this, [=](const QModelIndex& index) {
    if (index.column() == 0 || index.column() == 2) {
      ui_->crystalsTable->setCursor(Qt::PointingHandCursor);
    } else {
      ui_->crystalsTable->unsetCursor();
    }
  });

  // Choose item
  connect(table, &QTableView::clicked, this, &MainWindow::refreshCrystalInfo);

  // Item data changed
  connect(crystal_list_model_, &QStandardItemModel::dataChanged, this,
          &MainWindow::updateCrystalData);
}


void MainWindow::initCrystalInfoPanel() {
  using namespace IceHalo;
  using Dist = Math::Distribution;

  // Height edit
  crystal_height_edit_ = new FloatLineEdit(0, 10, 1);
  ui_->crystalBasicLayout->addWidget(crystal_height_edit_, 1, 1);

  // Pyramid height edits
  pyramid_lower_height_edit_ = new FloatLineEdit(0, 1, 1);
  pyramid_upper_height_edit_ = new FloatLineEdit(0, 1, 1);
  ui_->pyramidHeightLayout->addWidget(pyramid_upper_height_edit_, 0, 1);
  ui_->pyramidHeightLayout->addWidget(pyramid_lower_height_edit_, 0, 4);

  // 3D window and widget
  view3d_ = new Qt3DExtras::Qt3DWindow();
  crystal_preview_widget_ = QWidget::createWindowContainer(view3d_);
  QSize screenSize = view3d_->screen()->size();
  crystal_preview_widget_->setMinimumSize(QSize(200, 100));
  crystal_preview_widget_->setMaximumSize(screenSize);

  // Set layout
  crystal_info_layout_ = new QGridLayout;
  ui_->crystalSettingGroup->setLayout(crystal_info_layout_);

  // Add widgets
  crystal_info_layout_->addWidget(ui_->crystalTypePanel, 0, 0);
  crystal_info_layout_->addWidget(ui_->pyramidParameterPanel, 0, 1);
  crystal_info_layout_->addWidget(ui_->crystalPrismDistancePanel, 1, 1);
  crystal_info_layout_->addWidget(ui_->crystalAxisPanel, 1, 0, 2, 1);
  crystal_info_layout_->addWidget(crystal_preview_widget_, 2, 1);

  crystal_info_layout_->setRowStretch(2, 1);
  crystal_info_layout_->setColumnStretch(1, 1);

  // Widget layout
  ui_->crystalTypePanel->layout()->setAlignment(Qt::AlignTop);
  ui_->pyramidParameterPanel->layout()->setAlignment(Qt::AlignTop);
  ui_->crystalPrismDistancePanel->layout()->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  ui_->crystalAxisPanel->layout()->setAlignment(Qt::AlignTop);

  // Settings for crystalPrismDistancePanel
  auto layout = static_cast<QGridLayout*>(ui_->crystalPrismDistancePanel->layout());
  for (int i = 1; i <= 6; i++) {
    layout->setColumnStretch(i, 1);
  }
  layout->setColumnMinimumWidth(0, 110);

  // Add data to crystal type
  ui_->crystalTypeComboBox->addItem(tr("Prism"), static_cast<int>(CrystalType::kPrism));
  ui_->crystalTypeComboBox->addItem(tr("Pyramid"), static_cast<int>(CrystalType::kPyramid));

  // Add data to axis type
  ui_->axisZenithTypeComboBox->addItem(tr("Uniform"), static_cast<int>(Dist::kUniform));
  ui_->axisZenithTypeComboBox->addItem(tr("Gaussian"), static_cast<int>(Dist::kGaussian));
  ui_->axisRollTypeComboBox->addItem(tr("Uniform"), static_cast<int>(Dist::kUniform));
  ui_->axisRollTypeComboBox->addItem(tr("Gaussian"), static_cast<int>(Dist::kGaussian));
  ui_->axisAzimuthTypeComboBox->addItem(tr("Uniform"), static_cast<int>(Dist::kUniform));
  ui_->axisAzimuthTypeComboBox->addItem(tr("Gaussian"), static_cast<int>(Dist::kGaussian));

  // Settings for Miller index
  for (int i = CrystalData::kMinMillerIndex; i <= CrystalData::kMaxMillerIndex; i++) {
    ui_->upperMiller1ComboBox->addItem(QString::number(i), i);
    ui_->upperMiller2ComboBox->addItem(QString::number(i), i);
    ui_->lowerMiller1ComboBox->addItem(QString::number(i), i);
    ui_->lowerMiller2ComboBox->addItem(QString::number(i), i);
  }

  refreshCrystalInfo();

  connect(ui_->crystalTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateCrystalType);
}


void MainWindow::setCrystalPanelEnabled(bool enable) {
  if (!crystal_info_layout_) {
    return;
  }

  for (int i = 0; i < crystal_info_layout_->count(); i++) {
    auto item = crystal_info_layout_->itemAt(i);
    if (item && item->widget()) {
      item->widget()->setEnabled(enable);
    }
  }
}


QString MainWindow::getScatterTabText(int idx) {
  return tr("  Scatter %1").arg(idx);
}


QString MainWindow::getScatterProbText(double prob) {
  return tr("Probability: %1").arg(prob, 0, 'f', 2);
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


double MainWindow::getScatterProb() {
  return getScatterProb(ui_->scatterProbSlider->value());
}


double MainWindow::getScatterProb(int v) {
  v = std::max(std::min(v, ui_->scatterProbSlider->maximum()), ui_->scatterProbSlider->minimum());
  return v / 100.0;
}


MultiScatterData* MainWindow::getCurrentScatterData() {
  auto checked_btn = scatter_tab_group_->checkedButton();
  auto btn_cnt = ui_->scatterTabLayout->count() - 1;
  for (int i = 0; i < btn_cnt; i++) {
    if (ui_->scatterTabLayout->itemAt(i)->widget() == checked_btn) {
      return &gui_data_.multi_scatter_data_.at(static_cast<size_t>(i));
    }
  }

  return nullptr;
}


MultiScatterData::CrystalItemData* MainWindow::getCurrentCrystalItemData() {
  auto index = ui_->crystalsTable->currentIndex();
  return getCrystalItemData(index);
}


MultiScatterData::CrystalItemData* MainWindow::getCrystalItemData(const QModelIndex& index) {
  if (!index.isValid()) {
    return nullptr;
  }

  auto row = static_cast<size_t>(index.row());
  auto scatter_data = getCurrentScatterData();
  if (!scatter_data || row >= scatter_data->crystals_.size()) {
    return nullptr;
  }
  return &scatter_data->crystals_.at(row);
}


CrystalData* MainWindow::getCurrentCrystalData() {
  auto index = ui_->crystalsTable->currentIndex();
  return getCrystalData(index);
}


CrystalData* MainWindow::getCrystalData(const QModelIndex& index) {
  if (!index.isValid()) {
    return nullptr;
  }

  auto crystal_item_data = getCrystalItemData(index);
  if (!crystal_item_data) {
    return nullptr;
  }
  if (gui_data_.crystal_store_.count(crystal_item_data->crystal_id) == 0) {
    return nullptr;
  }
  return &gui_data_.crystal_store_.at(crystal_item_data->crystal_id);
}
