#include "mainwindow.h"

#include <QColorDialog>
#include <QHeaderView>
#include <QPropertyAnimation>
#include <QStackedWidget>
#include <QtDebug>
#include <algorithm>

#include "icons.h"
#include "render.h"
#include "ui_mainwindow.h"


int MainWindow::current_crystal_id_ = 1;
int MainWindow::current_filter_id_ = 1;


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow),
      project_context_(icehalo::ProjectContext::CreateDefault()) {
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
  btn->setChecked(true);  // This will emit toggle signal, thus refresh relavent panels
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
    auto tab = static_cast<IconButton*>(layout->itemAt(next_idx)->widget());
    tab->setChecked(true);  // And this will emit toggle signal, thus refresh relavent panels
  }

  // After animation finished, delete button and udpate tab text
  connect(anim, &QPropertyAnimation::finished, this, [=] {
    layout->removeWidget(sender);
    scatter_tab_group_->removeButton(sender);
    sender->deleteLater();

    // Update model
    auto& data = gui_data_.multi_scatter_data_;
    data.erase(data.begin() + index);

    if (layout->count() == 2) {  // Disable close action when there is only one tab
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

  auto prob = getCurrentScatterData()->prob_;
  int v = std::min(std::max(static_cast<int>(prob * 100), 0), 100);

  // Update UI
  QString prob_str = getScatterProbText(static_cast<double>(prob));
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
  QList<QStandardItem*> row_items;
  row_items << item_check << item_name << item_link << item_pop;
  crystal_list_model_->appendRow(row_items);
  crystal_table_->selectRow(row_count);

  current_crystal_id_++;

  refreshCrystalInfo();
  refreshFilterInfo();
}


void MainWindow::removeCurrentCrystal() {
  qDebug() << "removeCurrentCrystal()";

  auto index = crystal_table_->currentIndex();
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
    crystal_table_->selectRow(curr_row - 1);
  } else {
    crystal_table_->selectRow(curr_row);
  }

  refreshCrystalInfo();
  refreshFilterInfo();
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
      static_cast<icehalo::CrystalType>(type_combo_box->itemData(combo_idx, Qt::UserRole).toInt());
  auto crystal_data = getCurrentCrystalData();
  if (crystal_data) {
    crystal_data->type_ = type;
  }
  if (type == icehalo::CrystalType::kPrism) {
    crystal_data->height_[1] = 0;
    crystal_data->height_[2] = 0;
  }

  // Update UI widget
  if (type == icehalo::CrystalType::kPyramid) {
    ui_->pyramidParameterPanel->setEnabled(true);
  } else if (type == icehalo::CrystalType::kPrism) {
    ui_->pyramidParameterPanel->setEnabled(false);
  } else {
    qWarning() << "Invalid crystal type!";
  }
}


void MainWindow::updatePrismDistance(int idx, int value) {
  qDebug() << "updatePrismDistance()";

  // Update model data
  auto crystal_data = getCurrentCrystalData();
  auto d = getPrismDistanceByValue(value);
  crystal_data->prism_dist_[idx] = static_cast<float>(d);

  // Update UI widget
  auto txt = getPrismDistanceText(d);
  prism_distance_labels_[idx]->setText(txt);

  qDebug() << "  distance" << idx << "=" << txt;
}


void MainWindow::resetPrismDistance() {
  qDebug() << "resetPrismDistance()";

  auto crystal_data = getCurrentCrystalData();
  for (int idx = 0; idx < 6; idx++) {
    prism_distance_sliders_[idx]->setValue(10);
    prism_distance_labels_[idx]->setText("1.0");
    crystal_data->prism_dist_[idx] = 1.0f;
  }
}


void MainWindow::updateUpperHeight(int v) {
  qDebug() << "updateUpperHeight()";

  auto crystal_data = getCurrentCrystalData();

  // Update model
  auto h = getPyramidHeight(v);
  crystal_data->height_[1] = static_cast<float>(h);

  // Update UI widget
  ui_->upperHeightLabel->setText(tr("Upper h: ") + getPyramidHeightText(h));
}


void MainWindow::updateLowerHeight(int v) {
  qDebug() << "UpdateLowerHeight()";

  auto crystal_data = getCurrentCrystalData();

  // Update model
  auto h = getPyramidHeight(v);
  crystal_data->height_[2] = static_cast<float>(h);

  // Update UI widget
  ui_->lowerHeightLabel->setText(tr("Lower h: ") + getPyramidHeightText(h));
}


void MainWindow::updatePyramidMillerIndex(int idx, int v) {
  qDebug() << "updatePyramidMillerIndex()";

  auto crystal_data = getCurrentCrystalData();

  // Update model
  crystal_data->miller_idx_[idx] = v;
}


void MainWindow::refreshCrystalInfo() {
  qDebug() << "refreshCrystalInfo()";

  auto curr_index = crystal_table_->currentIndex();

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

  // Update pyramid height slider
  if (crystal_data->type_ == icehalo::CrystalType::kPrism) {
    ui_->pyramidParameterPanel->setEnabled(false);
  } else if (crystal_data->type_ == icehalo::CrystalType::kPyramid) {
    ui_->pyramidParameterPanel->setEnabled(crystal_item_data->enabled);
  }
  float upper_h = crystal_data->height_[1];
  float lower_h = crystal_data->height_[2];
  ui_->upperHeightLabel->setText(tr("Upper h: ") +
                                 getPyramidHeightText(static_cast<double>(upper_h)));
  ui_->lowerHeightLabel->setText(tr("Lower h: ") +
                                 getPyramidHeightText(static_cast<double>(lower_h)));
  ui_->upperHeightSlider->setValue(static_cast<int>(upper_h * 100));
  ui_->lowerHeightSlider->setValue(static_cast<int>(lower_h * 100));

  // Update pyramid miller index
  QComboBox* miller_combos[] = {
    ui_->upperMiller1ComboBox,  // upper 1
    ui_->upperMiller2ComboBox,  // upper 2
    ui_->lowerMiller1ComboBox,  // lower 1
    ui_->lowerMiller2ComboBox,  // lower 2
  };
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < miller_combos[i]->count(); j++) {
      if (miller_combos[i]->itemData(j, Qt::UserRole) == crystal_data->miller_idx_[i]) {
        miller_combos[i]->setCurrentIndex(j);
        break;
      }
    }
  }

  // Update axis means and stds
  axis_zenith_mean_edit_->setDataSource(&crystal_data->axis_.latitude_mean);
  axis_zenith_mean_edit_->refreshText();
  axis_zenith_std_edit_->setDataSource(&crystal_data->axis_.latitude_std);
  axis_zenith_std_edit_->refreshText();
  axis_roll_mean_edit_->setDataSource(&crystal_data->axis_.roll_mean);
  axis_roll_mean_edit_->refreshText();
  axis_roll_std_edit_->setDataSource(&crystal_data->axis_.roll_std);
  axis_roll_std_edit_->refreshText();
  axis_azimuth_mean_edit_->setDataSource(&crystal_data->axis_.azimuth_mean);
  axis_azimuth_mean_edit_->refreshText();
  axis_azimuth_std_edit_->setDataSource(&crystal_data->axis_.azimuth_std);
  axis_azimuth_std_edit_->refreshText();

  // Update axis types
  QComboBox* type_combos[] = {
    ui_->axisZenithTypeComboBox,   // zenith type
    ui_->axisRollTypeComboBox,     // roll type
    ui_->axisAzimuthTypeComboBox,  // azimuth type
  };
  int crystal_axis_dists[] = {
    static_cast<int>(crystal_data->axis_.latitude_dist),  // latitude, (= 90 - zenith)
    static_cast<int>(crystal_data->axis_.roll_dist),      // roll
    static_cast<int>(crystal_data->axis_.azimuth_dist),   // azimuth
  };
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < type_combos[i]->count(); j++) {
      if (type_combos[i]->itemData(j, Qt::UserRole) == crystal_axis_dists[i]) {
        type_combos[i]->setCurrentIndex(j);
        break;
      }
    }
  }

  // Update prism distances
  for (int idx = 0; idx < 6; idx++) {
    auto d = crystal_data->prism_dist_[idx];
    prism_distance_sliders_[idx]->setValue(static_cast<int>(d * 10));
    auto txt = getPrismDistanceText(static_cast<double>(d));
    prism_distance_labels_[idx]->setText(txt);
  }

  // TODO
}


FilterData& MainWindow::createNewFilter(MultiScatterData::CrystalItemData* crystal_item_data) {
  qDebug() << "createNewFilter()";

  crystal_item_data->filter_id = current_filter_id_;
  gui_data_.filter_store_.emplace(current_filter_id_, FilterData::kNone);
  current_filter_id_++;
  return gui_data_.filter_store_.at(crystal_item_data->filter_id);
}


void MainWindow::enableFilterSettings(bool enable) {
  qDebug() << "enableFilterSettings():" << enable;

  if (enable) {
    ui_->filterEnableCheckBox->setCheckState(Qt::Checked);
  } else {
    ui_->filterEnableCheckBox->setCheckState(Qt::Unchecked);
  }

  ui_->specificRadioButton->setEnabled(enable);
  ui_->generalRadioButton->setEnabled(enable);
  ui_->symBCheckBox->setEnabled(enable);
  ui_->symDCheckBox->setEnabled(enable);
  ui_->symPCheckBox->setEnabled(enable);
  for (int i = 0; i < ui_->filterPathPages->count(); i++) {
    ui_->filterPathPages->widget(i)->setEnabled(enable);
  }
}


void MainWindow::resetFilterInfo() {
  qDebug() << "resetFilterInfo()";

  ui_->filterEnableCheckBox->setCheckState(Qt::Unchecked);
  ui_->specificRadioButton->setChecked(true);
  ui_->filterPathPages->setCurrentIndex(0);

  resetSpecificFilterPage();
  resetGeneralFilterPage();
  resetFilterSymmetryPanel();
}


void MainWindow::addSpecificFilterRow() {
  std::vector<int> path;
  addSpecificFilterRow(path);
}


void MainWindow::addSpecificFilterRow(const std::vector<int>& path) {
  qDebug() << "addSpecificFilterRow()";

  auto remove_item = new QStandardItem;
  auto path_item = new QStandardItem;

  QStringList tmp_list;
  for (auto f : path) {
    tmp_list.append(QString::number(f));
  }
  path_item->setData(tmp_list.join("-"), Qt::EditRole);

  QList<QStandardItem*> item_list;
  item_list << remove_item << path_item;
  specific_path_model_->appendRow(item_list);
}


void MainWindow::updateFilterInfo() {
  qDebug() << "updateFilterInfo()";

  auto crystal_item_data = getCurrentCrystalItemData();
  if (!crystal_item_data) {
    return;
  }
  if (!ui_->filterEnableCheckBox->isChecked()) {
    return;
  }

  if (crystal_item_data->filter_id == 0) {
    createNewFilter(crystal_item_data);
  }

  FilterData& filter = gui_data_.filter_store_.at(crystal_item_data->filter_id);

  if (ui_->filterEnableCheckBox->checkState() == Qt::Unchecked) {
    filter.type_ = FilterData::kNone;
    return;
  }

  updateFilterSymmetry(filter);

  if (ui_->specificRadioButton->isChecked()) {
    // For specific path filter
    updateSpecificFilterInfo(filter);
  } else if (ui_->generalRadioButton->isChecked()) {
    // For general path filter
    updateGeneralFilterInfo(filter);
  }
}


void MainWindow::updateFilterSymmetry(FilterData& filter) {
  qDebug() << "updateFilterSymmetry()";

  filter.symmetry_flag_ = icehalo::kSymmetryNone;
  if (ui_->symBCheckBox->checkState() == Qt::Checked) {
    filter.symmetry_flag_ |= icehalo::kSymmetryBasal;
  }
  if (ui_->symPCheckBox->checkState() == Qt::Checked) {
    filter.symmetry_flag_ |= icehalo::kSymmetryPrism;
  }
  if (ui_->symDCheckBox->checkState() == Qt::Checked) {
    filter.symmetry_flag_ |= icehalo::kSymmetryDirection;
  }
}


void MainWindow::updateGeneralFilterInfo(FilterData& filter_data) {
  qDebug() << "updateGeneralFilterInfo()";

  if (ui_->filterEnableCheckBox->isChecked()) {
    filter_data.type_ = FilterData::kGeneral;
  }

  auto enter_face_txt = ui_->generalFilterEnterEdit->text();
  auto enter_faces = enter_face_txt.split(",", QString::SkipEmptyParts);
  filter_data.enter_faces_.clear();
  for (const auto& f : enter_faces) {
    filter_data.enter_faces_.emplace_back(f.toInt());
  }

  auto exit_face_txt = ui_->generalFilterExitEdit->text();
  auto exit_faces = exit_face_txt.split(",", QString::SkipEmptyParts);
  filter_data.exit_faces_.clear();
  for (const auto& f : exit_faces) {
    filter_data.exit_faces_.emplace_back(f.toInt());
  }

  filter_data.hits_ = ui_->generalFilterHitsSpinBox->value();
}


void MainWindow::updateSpecificFilterInfo(FilterData& filter_data) {
  qDebug() << "updateSpecificFilterInfo()";

  if (ui_->filterEnableCheckBox->isChecked()) {
    filter_data.type_ = FilterData::kSpecific;
  }

  filter_data.paths_.clear();
  auto count = specific_path_model_->rowCount();
  for (int i = 0; i < count - 1; i++) {
    auto index = specific_path_model_->index(i, 1);

    // Parse path
    // A path is like "3-5-1"
    QString path_txt = specific_path_model_->data(index).toString();
    auto path_list = path_txt.split("-", QString::SkipEmptyParts);

    std::vector<int> tmp_path;
    for (const auto& t : path_list) {
      tmp_path.emplace_back(t.toInt());
    }
    filter_data.paths_.emplace_back(tmp_path);
  }
}


void MainWindow::refreshFilterInfo() {
  auto crystal_item_data = getCurrentCrystalItemData();
  if (!crystal_item_data || !crystal_item_data->enabled) {
    // If no crystal, or the crystl is not enabled, then disable the filter panel
    ui_->filterEnableCheckBox->setEnabled(false);
    resetFilterInfo();
    enableFilterSettings(false);
    return;
  } else {
    ui_->filterEnableCheckBox->setEnabled(true);
  }

  const auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);
  qDebug() << " filter ID:" << crystal_item_data->filter_id;
  refreshFilterInfo(filter_data);
}


void MainWindow::refreshFilterInfo(const FilterData& filter_data) {
  qDebug() << "refreshFilterInfo()";

  qDebug() << "  filter data:";
  qDebug() << "    type:" << filter_data.type_;
  qDebug() << "    symm:" << filter_data.symmetry_flag_;
  qDebug() << "    path:" << filter_data.paths_;
  qDebug() << "    enter:" << filter_data.enter_faces_;
  qDebug() << "    exit:" << filter_data.exit_faces_;
  qDebug() << "    hits:" << filter_data.hits_;

  // Update type (radio button) and path info (stacked widget)
  switch (filter_data.type_) {
    case FilterData::kNone:
      resetFilterInfo();
      enableFilterSettings(false);
      break;
    case FilterData::kSpecific:
      enableFilterSettings(true);
      ui_->specificRadioButton->setChecked(true);
      ui_->filterPathPages->setCurrentIndex(0);
      refreshSpecificFilterPage(filter_data);
      break;
    case FilterData::kGeneral:
      enableFilterSettings(true);
      ui_->generalRadioButton->setChecked(true);
      ui_->filterPathPages->setCurrentIndex(1);
      refreshGeneralFilterPage(filter_data);
      break;
    default:
      qCritical() << "Filter type not recgonized!";
      break;
  }

  // Update symmetry
  refreshFilterSymmetry(filter_data);
}


void MainWindow::refreshFilterSymmetry(const FilterData& filter_data) {
  qDebug() << "refreshFilterSymmetry()";

  if (filter_data.symmetry_flag_ & icehalo::kSymmetryBasal) {
    ui_->symBCheckBox->setCheckState(Qt::Checked);
  } else {
    ui_->symBCheckBox->setCheckState(Qt::Unchecked);
  }

  if (filter_data.symmetry_flag_ & icehalo::kSymmetryPrism) {
    ui_->symPCheckBox->setCheckState(Qt::Checked);
  } else {
    ui_->symPCheckBox->setCheckState(Qt::Unchecked);
  }

  if (filter_data.symmetry_flag_ & icehalo::kSymmetryDirection) {
    ui_->symDCheckBox->setCheckState(Qt::Checked);
  } else {
    ui_->symDCheckBox->setCheckState(Qt::Unchecked);
  }
}


void MainWindow::refreshGeneralFilterPage(const FilterData& filter_data) {
  qDebug() << "refreshGeneralFilterPage()";

  QStringList enter_faces;
  for (auto f : filter_data.enter_faces_) {
    enter_faces << QString::number(f);
  }
  ui_->generalFilterEnterEdit->setText(enter_faces.join(","));

  enter_faces.clear();
  for (auto f : filter_data.exit_faces_) {
    enter_faces << QString::number(f);
  }
  ui_->generalFilterExitEdit->setText(enter_faces.join(","));

  ui_->generalFilterHitsSpinBox->setValue(filter_data.hits_);
}


void MainWindow::refreshSpecificFilterPage(const FilterData& filter_data) {
  qDebug() << "refreshSpecificFilterPage()";

  resetSpecificFilterPage();
  specific_path_model_->removeRow(0);
  for (const auto& p : filter_data.paths_) {
    addSpecificFilterRow(p);
  }
  addSpecificFilterRow();
}


void MainWindow::resetGeneralFilterPage() {
  qDebug() << "resetGeneralFilterPage()";

  ui_->generalFilterEnterEdit->setText("");
  ui_->generalFilterExitEdit->setText("");
  ui_->generalFilterHitsSpinBox->setValue(0);
}


void MainWindow::resetSpecificFilterPage() {
  qDebug() << "resetSpecificFilterPage()";

  specific_path_model_->clear();
  specific_path_model_->setHorizontalHeaderLabels(QStringList{ "", tr("Path") });

  specific_path_table_->setColumnWidth(0, 25);
  specific_path_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  specific_path_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  specific_path_table_->horizontalHeader()->setMinimumSectionSize(20);
  specific_path_table_->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  specific_path_table_->verticalHeader()->hide();
  specific_path_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  specific_path_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  specific_path_table_->setShowGrid(false);

  specific_path_table_->setColumCursor(0, Qt::PointingHandCursor);
  specific_path_table_->setItemDelegateForColumn(1, new RayPathDelegate);

  addSpecificFilterRow();
}


void MainWindow::resetFilterSymmetryPanel() {
  qDebug() << "resetFilterSymmetryPanel()";

  ui_->symBCheckBox->setCheckState(Qt::Unchecked);
  ui_->symPCheckBox->setCheckState(Qt::Unchecked);
  ui_->symDCheckBox->setCheckState(Qt::Unchecked);
}


void MainWindow::initUi() {
  using namespace icehalo;

  // Setup some UI properties
  ui_->filterSettingLayout->setAlignment(Qt::AlignTop);

  initBasicSettings();
  initScatterTab();
  initCrystalList();
  initCrystalInfoPanel();
  initFilter();

  // Connect signals and slots
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
    refreshScatterProb();
    refreshCrystalList();
    refreshCrystalInfo();
    refreshFilterInfo();
  });
}


void MainWindow::initCrystalList() {
  crystal_table_ = new CursorTable;
  ui_->crystalsLayout->insertWidget(0, crystal_table_, 1);

  crystal_list_model_ = new QStandardItemModel();
  crystal_list_model_->setHorizontalHeaderLabels(QStringList{ "", "Name", "", "Pop." });

  crystal_table_->setModel(crystal_list_model_);
  crystal_table_->setColumnWidth(0, 25);
  crystal_table_->setColumnWidth(2, 30);
  crystal_table_->setColumnWidth(3, 40);
  crystal_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  crystal_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  crystal_table_->horizontalHeader()->setMinimumSectionSize(20);
  crystal_table_->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  crystal_table_->verticalHeader()->hide();
  crystal_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  crystal_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  crystal_table_->setShowGrid(false);
  crystal_table_->setItemDelegateForColumn(3, new SpinBoxDelegate);

  crystal_table_->setColumCursor(0, Qt::PointingHandCursor);
  crystal_table_->setColumCursor(2, Qt::PointingHandCursor);

  // Add crystal
  connect(ui_->crystalsAddButton, &QToolButton::clicked, this, [=] {
    crystal_table_->setFocus();
    addCrystal();
  });

  // Remove crystal
  connect(ui_->crystalsRemoveButton, &QToolButton::clicked, this, [=] {
    crystal_table_->setFocus();
    removeCurrentCrystal();
  });

  // Click link/unlink icon
  connect(crystal_table_, &QTableView::clicked, this, [=](const QModelIndex& index) {
    if (index.column() != 2) {
      return;
    }
    toggleCrystalLinkState(index);
  });

  // Choose item
  connect(crystal_table_, &CursorTable::clicked, this, [=] {
    refreshCrystalInfo();
    refreshFilterInfo();
  });

  // Item data changed
  connect(crystal_list_model_, &QStandardItemModel::dataChanged, this,
          &MainWindow::updateCrystalData);
}


void MainWindow::initCrystalInfoPanel() {
  using namespace icehalo;
  using Dist = math::Distribution;

  // Height edit
  crystal_height_edit_ = new FloatLineEdit(0, 10, 1);
  ui_->crystalBasicLayout->addWidget(crystal_height_edit_, 1, 1);

  // Miller index
  miller_index_combos_[0] = ui_->upperMiller1ComboBox;
  miller_index_combos_[1] = ui_->upperMiller2ComboBox;
  miller_index_combos_[2] = ui_->lowerMiller1ComboBox;
  miller_index_combos_[3] = ui_->lowerMiller2ComboBox;

  // Axis edits
  axis_zenith_mean_edit_ = new FloatLineEdit(-90, 90, 1);
  axis_zenith_mean_edit_->setTransform([](float v) { return 90.0f - v; },
                                       [](float v) { return 90.0f - v; });
  axis_zenith_std_edit_ = new FloatLineEdit(0, 180, 1);
  axis_roll_mean_edit_ = new FloatLineEdit(0, 360, 1);
  axis_roll_std_edit_ = new FloatLineEdit(0, 180, 1);
  axis_azimuth_mean_edit_ = new FloatLineEdit(0, 360, 1);
  axis_azimuth_std_edit_ = new FloatLineEdit(0, 180, 1);
  ui_->crystalAxisLayout->addWidget(axis_zenith_mean_edit_, 2, 1);
  ui_->crystalAxisLayout->addWidget(axis_zenith_std_edit_, 3, 1);
  ui_->crystalAxisLayout->addWidget(axis_roll_mean_edit_, 7, 1);
  ui_->crystalAxisLayout->addWidget(axis_roll_std_edit_, 8, 1);
  ui_->crystalAxisLayout->addWidget(axis_azimuth_mean_edit_, 12, 1);
  ui_->crystalAxisLayout->addWidget(axis_azimuth_std_edit_, 13, 1);

  // Prism distances
  prism_distance_sliders_[0] = ui_->crystalPrismDistanceSlider1;
  prism_distance_sliders_[1] = ui_->crystalPrismDistanceSlider2;
  prism_distance_sliders_[2] = ui_->crystalPrismDistanceSlider3;
  prism_distance_sliders_[3] = ui_->crystalPrismDistanceSlider4;
  prism_distance_sliders_[4] = ui_->crystalPrismDistanceSlider5;
  prism_distance_sliders_[5] = ui_->crystalPrismDistanceSlider6;
  prism_distance_labels_[0] = ui_->crystalPrismDistanceLabel1;
  prism_distance_labels_[1] = ui_->crystalPrismDistanceLabel2;
  prism_distance_labels_[2] = ui_->crystalPrismDistanceLabel3;
  prism_distance_labels_[3] = ui_->crystalPrismDistanceLabel4;
  prism_distance_labels_[4] = ui_->crystalPrismDistanceLabel5;
  prism_distance_labels_[5] = ui_->crystalPrismDistanceLabel6;
  for (int idx = 0; idx < 6; idx++) {
    prism_distance_sliders_[idx]->setMinimum(0);
    prism_distance_sliders_[idx]->setMaximum(20);
    prism_distance_sliders_[idx]->setValue(10);
  }

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

  connect(ui_->upperHeightSlider, &QSlider::valueChanged, this, &MainWindow::updateUpperHeight);
  connect(ui_->lowerHeightSlider, &QSlider::valueChanged, this, &MainWindow::updateLowerHeight);

  for (int i = 0; i < 4; i++) {
    connect(miller_index_combos_[i], QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [=](int cmb_idx) {
              int v = miller_index_combos_[i]->itemData(cmb_idx, Qt::UserRole).toInt();
              updatePyramidMillerIndex(i, v);
            });
  }

  connect(ui_->crystalPrismDistanceResetButton, &QPushButton::clicked, this,
          &MainWindow::resetPrismDistance);
  for (int idx = 0; idx < 6; idx++) {
    connect(prism_distance_sliders_[idx], &QSlider::valueChanged, this,
            [=](int v) { updatePrismDistance(idx, v); });
  }
}


void MainWindow::initFilter() {
  specific_path_table_ = new CursorTable;
  ui_->filterPathPages->insertWidget(0, specific_path_table_);
  ui_->filterPathPages->setCurrentIndex(0);

  specific_path_model_ = new QStandardItemModel;
  specific_path_table_->setModel(specific_path_model_);
  resetSpecificFilterPage();

  QRegExp rx("\\d{1,2}(,\\d{1,2})*");
  auto face_edit_validator = new QRegExpValidator(rx);
  ui_->generalFilterEnterEdit->setValidator(face_edit_validator);
  ui_->generalFilterExitEdit->setValidator(face_edit_validator);

  gui_data_.filter_store_.clear();
  gui_data_.filter_store_.emplace(0, FilterData::kNone);

  refreshFilterInfo();

  connect(ui_->filterEnableCheckBox, &QCheckBox::clicked, this, [=](bool checked) {
    qDebug() << "filterEnableCheckBox::clicked:" << checked;

    enableFilterSettings(checked);
    if (!checked) {
      resetGeneralFilterPage();
      resetSpecificFilterPage();
      resetFilterSymmetryPanel();
    }
    updateFilterInfo();
    refreshFilterInfo();
  });

  connect(ui_->specificRadioButton, &QRadioButton::clicked, this, [=](bool checked) {
    qDebug() << "specificRadioButton::toggled:" << checked;

    if (!checked) {
      return;
    }
    ui_->filterPathPages->setCurrentIndex(0);
    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }

    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);
    filter_data.type_ = FilterData::kSpecific;
    refreshFilterInfo(filter_data);
  });

  connect(ui_->generalRadioButton, &QRadioButton::clicked, this, [=](bool checked) {
    qDebug() << "generalRadioButton::toggled:" << checked;

    if (!checked) {
      return;
    }
    ui_->filterPathPages->setCurrentIndex(1);
    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }

    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);
    filter_data.type_ = FilterData::kGeneral;
    refreshFilterInfo(filter_data);
  });

  connect(ui_->symBCheckBox, &QCheckBox::clicked, this, [=](bool checked) {
    qDebug() << "symBCheckBox::clicked:" << checked;

    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }
    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);

    if (!checked) {
      filter_data.symmetry_flag_ &= (~icehalo::kSymmetryBasal);
    } else {
      filter_data.symmetry_flag_ |= icehalo::kSymmetryBasal;
    }
  });

  connect(ui_->symPCheckBox, &QCheckBox::clicked, this, [=](bool checked) {
    qDebug() << "symPCheckBox::clicked:" << checked;

    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }
    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);

    if (!checked) {
      filter_data.symmetry_flag_ &= (~icehalo::kSymmetryPrism);
    } else {
      filter_data.symmetry_flag_ |= icehalo::kSymmetryPrism;
    }
  });

  connect(ui_->symDCheckBox, &QCheckBox::clicked, this, [=](bool checked) {
    qDebug() << "symDCheckBox::clicked:" << checked;

    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }
    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);

    if (!checked) {
      filter_data.symmetry_flag_ &= (~icehalo::kSymmetryDirection);
    } else {
      filter_data.symmetry_flag_ |= icehalo::kSymmetryDirection;
    }
  });

  connect(ui_->generalFilterEnterEdit, &QLineEdit::textEdited, this, [=] {
    qDebug() << "generalFilterEnterEdit::textEdited";

    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }
    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);
    updateGeneralFilterInfo(filter_data);
  });

  connect(ui_->generalFilterExitEdit, &QLineEdit::textEdited, this, [=] {
    qDebug() << "generalFilterExitEdit::textEdited";

    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }
    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);
    updateGeneralFilterInfo(filter_data);
  });

  connect(ui_->generalFilterHitsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=] {
    qDebug() << "generalFilterHitsSpinBox::valueChanged";

    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }
    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);
    updateGeneralFilterInfo(filter_data);
  });

  connect(specific_path_model_, &QStandardItemModel::itemChanged, this, [=](QStandardItem* item) {
    qDebug() << "specific_path_model_::itemChanged:" << item->index();
    if (item->column() != 1) {
      return;
    }

    auto crystal_item_data = getCurrentCrystalItemData();
    if (!crystal_item_data) {
      return;
    }
    if (crystal_item_data->filter_id == 0) {
      createNewFilter(crystal_item_data);
    }
    if (gui_data_.filter_store_.count(crystal_item_data->filter_id) == 0) {
      return;
    }

    // Check whether to add new row
    if (item->row() >= specific_path_model_->rowCount() - 1) {
      addSpecificFilterRow();
    }

    // Update model
    auto& filter_data = gui_data_.filter_store_.at(crystal_item_data->filter_id);
    updateSpecificFilterInfo(filter_data);
  });
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


double MainWindow::getPrismDistance(int idx) {
  int value = prism_distance_sliders_[idx]->value();
  return getPrismDistanceByValue(value);
}


double MainWindow::getPrismDistanceByValue(int value) {
  return value / 10.0;
}


QString MainWindow::getPrismDistanceText(double d) {
  return QString::number(d, 'f', 1);
}


double MainWindow::getPyramidHeight(int value) {
  return value / 100.0;
}


QString MainWindow::getPyramidHeightText(double h) {
  return QString::number(h, 'f', 2);
}


MultiScatterData* MainWindow::getCurrentScatterData() {
  //  qDebug() << "getCurrentScatterData()";

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
  auto index = crystal_table_->currentIndex();
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
  auto index = crystal_table_->currentIndex();
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
