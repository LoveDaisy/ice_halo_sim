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
