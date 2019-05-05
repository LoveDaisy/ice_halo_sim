#ifndef FLOATLINEEDIT_H
#define FLOATLINEEDIT_H

#include <QLineEdit>

class FloatLineEdit : public QLineEdit {
  Q_OBJECT
 public:
  FloatLineEdit(float min_valule, float max_value, int decimal = 1, QWidget* parent = nullptr);

 signals:

 public slots:

 private:
  float* data_source;
};

#endif  // FLOATLINEEDIT_H
