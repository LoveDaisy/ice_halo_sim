#include "floatlineedit.h"

#include <QDoubleValidator>

FloatLineEdit::FloatLineEdit(float min_valule, float max_value, int decimal, QWidget* parent)
    : QLineEdit(parent) {
  auto validator = new QDoubleValidator(static_cast<double>(min_valule),  // min
                                        static_cast<double>(max_value),   // max
                                        decimal);
  setValidator(validator);
}
