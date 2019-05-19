#include "customwidgets.h"

#include <QDebug>
#include <QDoubleValidator>
#include <QKeyEvent>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPalette>
#include <QSpinBox>
#include <cmath>

#include "icons.h"

/*****************************************************************************/
CursorTable::CursorTable(QWidget* parent) : QTableView(parent) {
  setMouseTracking(true);
}


void CursorTable::setColumCursor(int colum, const QCursor& cursor) {
  cursors_.emplace(colum, cursor);
}


void CursorTable::resetColumCursor() {
  cursors_.clear();
}


void CursorTable::mouseMoveEvent(QMouseEvent* event) {
  auto row = rowAt(event->y());
  if (row < 0) {
    setCursor(Qt::ArrowCursor);
    return;
  }

  auto colum = columnAt(event->x());
  if (cursors_.count(colum)) {
    setCursor(cursors_.at(colum));
  } else {
    setCursor(Qt::ArrowCursor);
  }
}


/*****************************************************************************/
FloatLineEdit::FloatLineEdit(float min_valule, float max_value, int decimals, QWidget* parent)
    : QLineEdit(parent), min_value_(min_valule), max_value_(max_value), decimals_(decimals),
      data_source_(nullptr) {
  auto validator = new QDoubleValidator(static_cast<double>(min_valule),  // min
                                        static_cast<double>(max_value),   // max
                                        decimals);
  setValidator(validator);
  connect(this, &FloatLineEdit::textEdited, this, &FloatLineEdit::updateData);
}


void FloatLineEdit::updateData(const QString& txt) {
  qDebug() << "FloatLineEdit::updateData()";
  if (!data_source_) {
    qDebug() << "No data source! Quit";
    return;
  }

  float value = 0;
  if (model_to_view_ && view_to_model_) {
    value = view_to_model_(value);
  }
  if (txt.isEmpty() || txt == "-") {  // Special cases. Cen be seen as 0
    if (min_value_ <= value && max_value_ >= value) {
      *data_source_ = value;
    } else if (std::abs(min_value_ - value) < std::abs(max_value_ - value)) {
      *data_source_ = min_value_;
    } else {
      *data_source_ = max_value_;
    }
    qDebug() << "New value:" << *data_source_;
    return;
  }

  int pos;
  QString old_txt = txt;
  if (validator()->validate(old_txt, pos) != QValidator::Acceptable) {
    refreshText();
    // TODO: show a tooltip
  } else {
    value = txt.toFloat();
    if (model_to_view_ && view_to_model_) {
      value = view_to_model_(value);
    }
    *data_source_ = value;
  }
  qDebug() << "New value:" << *data_source_;
}


void FloatLineEdit::refreshText() {
  if (!data_source_) {
    qDebug() << "No data source! No refresh!";
    return;
  }

  float value = *data_source_;
  if (model_to_view_ && view_to_model_) {
    value = model_to_view_(value);
  }
  setText(formatValue(value));
}


void FloatLineEdit::setTransform(DataTransform view_to_model, DataTransform model_to_view) {
  view_to_model_ = std::move(view_to_model);
  model_to_view_ = std::move(model_to_view);
}


void FloatLineEdit::focusOutEvent(QFocusEvent* event) {
  if (data_source_) {
    refreshText();
  }
  QLineEdit::focusOutEvent(event);
}


void FloatLineEdit::keyPressEvent(QKeyEvent* event) {
  if (data_source_ && (event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)) {
    refreshText();
    clearFocus();
  }
  QLineEdit::keyPressEvent(event);
}


void FloatLineEdit::setDataSource(float* data_source) {
  data_source_ = data_source;
}


QString FloatLineEdit::formatValue(float value) {
  value = std::max(std::min(value, max_value_), min_value_);
  return QString::number(static_cast<double>(value), 'f', decimals_);
}


/*****************************************************************************/
RegExpLineEdit::RegExpLineEdit(const QRegExp& regex, QWidget* parent) : QLineEdit(parent) {
  auto validator = new QRegExpValidator(regex);
  setValidator(validator);
}


/*****************************************************************************/
IconButton::IconButton(const QString& text, QWidget* parent)
    : QToolButton(parent), icon_enabled_(true) {
  setMouseTracking(true);
  setText(text);
  //  setIcon(Icons::getIcon(Icons::kCloseNormal));
  setCheckable(true);
  setIconSize(kDefaultIconSize);
  setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  setForegroundRole(QPalette::ButtonText);

  connect(this, &IconButton::hoverOnIcon, this, &IconButton::updateIcon);
}


void IconButton::enableIcon(bool enabled) {
  icon_enabled_ = enabled;
}


bool IconButton::iconEnabled() {
  return icon_enabled_;
}


int IconButton::iconSize() const {
  return icon_size_;
}


void IconButton::setIconSize(int size) {
  icon_size_ = size;
  QToolButton::setIconSize(QSize(size, size));
}


void IconButton::setIcons(QIcon icon_normal, QIcon icon_on) {
  icon_normal_ = icon_normal;
  icon_on_ = icon_on;
  updateIcon(false);
}


void IconButton::updateIcon(bool on_icon) {
  if (!icon_enabled_) {
    return;
  }
  if (icon_on_.isNull() && icon_normal_.isNull()) {
    return;
  }

  if (on_icon) {
    if (!icon_on_.isNull()) {
      setIcon(icon_on_);
    } else if (!icon_normal_.isNull()) {
      setIcon(icon_normal_);
    }
  } else {
    if (!icon_normal_.isNull()) {
      setIcon(icon_normal_);
    }
  }
}


void IconButton::mouseMoveEvent(QMouseEvent* e) {
  if (icon_enabled_) {
    emit hoverOnIcon(isOnIcon(e->x()));
  }
  QToolButton::mouseMoveEvent(e);
}


void IconButton::mouseReleaseEvent(QMouseEvent* e) {
  if (icon_enabled_ && isOnIcon(e->x())) {
    emit closeTab(this);
  } else {
    QToolButton::mouseReleaseEvent(e);
  }
}


void IconButton::leaveEvent(QEvent* /* event */) {
  emit hoverOnIcon(false);
}


bool IconButton::isOnIcon(int x) {
  return x < kDefaultIconSize * 2;
}


/*****************************************************************************/
SpinBoxDelegate::SpinBoxDelegate(QObject* parent) : QStyledItemDelegate(parent) {}


QWidget* SpinBoxDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& /* option */,
                                       const QModelIndex& /* index */) const {
  QSpinBox* editor = new QSpinBox(parent);
  editor->setFrame(false);
  editor->setMinimum(0);
  editor->setMaximum(100);
  return editor;
}


void SpinBoxDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const {
  int value = index.data(Qt::EditRole).toInt();

  QSpinBox* spinBox = static_cast<QSpinBox*>(editor);
  spinBox->setValue(value);
}


void SpinBoxDelegate::setModelData(QWidget* editor, QAbstractItemModel* model,
                                   const QModelIndex& index) const {
  QSpinBox* spinBox = static_cast<QSpinBox*>(editor);
  spinBox->interpretText();
  int value = spinBox->value();

  model->setData(index, value, Qt::EditRole);
}


void SpinBoxDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                                           const QModelIndex& /* index */) const {
  editor->setGeometry(option.rect);
}


/*****************************************************************************/
RayPathDelegate::RayPathDelegate(QObject* parent) : QStyledItemDelegate(parent) {}


QWidget* RayPathDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& /* option */,
                                       const QModelIndex& /* index */) const {
  QRegExp regexp("\\d{1,2}(-\\d{1,2})*");
  auto editor = new RegExpLineEdit(regexp, parent);
  return editor;
}


void RayPathDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const {
  RegExpLineEdit* line_editor = static_cast<RegExpLineEdit*>(editor);
  line_editor->setText(index.data(Qt::EditRole).toString());
}


void RayPathDelegate::setModelData(QWidget* editor, QAbstractItemModel* model,
                                   const QModelIndex& index) const {
  RegExpLineEdit* line_editor = static_cast<RegExpLineEdit*>(editor);
  model->setData(index, line_editor->text(), Qt::EditRole);
}


void RayPathDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                                           const QModelIndex& /* index */) const {
  editor->setGeometry(option.rect);
}
