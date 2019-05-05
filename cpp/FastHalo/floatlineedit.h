#ifndef FLOATLINEEDIT_H
#define FLOATLINEEDIT_H

#include <QLineEdit>

class FloatLineEdit : public QLineEdit {
  Q_OBJECT
 public:
  FloatLineEdit(float min_valule, float max_value, int decimals = 1, QWidget* parent = nullptr);

  void setDataSource(float* data_source);
  QString formatValue(float value);

 signals:

 public slots:
  void updateData(const QString& txt);

 protected:
  void focusOutEvent(QFocusEvent* event);
  void keyPressEvent(QKeyEvent* event);

 private:
  float min_value_;
  float max_value_;
  int decimals_;
  float* data_source_;
};

#endif  // FLOATLINEEDIT_H
