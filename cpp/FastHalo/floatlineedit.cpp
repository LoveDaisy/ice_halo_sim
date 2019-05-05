#include "floatlineedit.h"

#include <QDebug>
#include <QDoubleValidator>
#include <QKeyEvent>
#include <cmath>

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

  if (txt.isEmpty() || txt == "-") {  // Special cases. Cen be seen as 0
    if (min_value_ <= 0 && max_value_ >= 0) {
      *data_source_ = 0;
    } else if (std::abs(min_value_) < std::abs(max_value_)) {
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
    setText(formatValue(*data_source_));
    // TODO: show a tooltip
  } else {
    *data_source_ = txt.toFloat();
  }
  qDebug() << "New value:" << *data_source_;
}


void FloatLineEdit::focusOutEvent(QFocusEvent* event) {
  if (data_source_) {
    setText(formatValue(*data_source_));
  }
  QLineEdit::focusOutEvent(event);
}


void FloatLineEdit::keyPressEvent(QKeyEvent* event) {
  if (data_source_ && (event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)) {
    setText(formatValue(*data_source_));
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
