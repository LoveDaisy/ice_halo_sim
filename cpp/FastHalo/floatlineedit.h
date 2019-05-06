#ifndef FLOATLINEEDIT_H
#define FLOATLINEEDIT_H

#include <QLineEdit>
#include <functional>

using DataTransform = std::function<float(float)>;

class FloatLineEdit : public QLineEdit {
  Q_OBJECT
 public:
  FloatLineEdit(float min_valule, float max_value, int decimals = 1, QWidget* parent = nullptr);

  void setDataSource(float* data_source);
  void setTransform(DataTransform view_to_model, DataTransform model_to_view);
  QString formatValue(float value);

 signals:

 public slots:
  void updateData(const QString& txt);
  void refreshText();

 protected:
  void focusOutEvent(QFocusEvent* event);
  void keyPressEvent(QKeyEvent* event);

 private:
  float min_value_;
  float max_value_;
  int decimals_;
  float* data_source_;

  DataTransform view_to_model_;
  DataTransform model_to_view_;
};

#endif  // FLOATLINEEDIT_H
