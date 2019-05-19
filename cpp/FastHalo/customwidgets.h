#ifndef CURSORTABLE_H
#define CURSORTABLE_H

#include <QTableView>
#include <QLineEdit>
#include <QMouseEvent>
#include <QToolButton>
#include <QTextStream>
#include <QStyledItemDelegate>

#include <map>
#include <functional>

/*****************************************************************************/
class CursorTable : public QTableView {
  Q_OBJECT
 public:
  explicit CursorTable(QWidget* parent = nullptr);

  void setColumCursor(int colum, const QCursor& cursor);
  void resetColumCursor();

 signals:

 public slots:

 protected:
  void mouseMoveEvent(QMouseEvent* event) override;

 private:
  std::map<int, QCursor> cursors_;
};


/*****************************************************************************/
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


/*****************************************************************************/
class IconButton : public QToolButton {
  Q_OBJECT
 public:
  explicit IconButton(const QString& text, QWidget* parent = nullptr);
  void enableIcon(bool enabled = true);
  bool iconEnabled();

  int iconSize() const;
  void setIconSize(int size);
  void setIcons(QIcon icon_normal, QIcon icon_on = QIcon());

  static constexpr int kDefaultIconSize = 10;

 signals:
  void hoverOnIcon(bool on = false);
  void closeTab(IconButton* sender);

 public slots:
  void updateIcon(bool on_icon);

 protected:
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  void leaveEvent(QEvent* event) override;

  bool icon_enabled_;
  int icon_size_;
  QIcon icon_normal_;
  QIcon icon_on_;

 private:
  bool isOnIcon(int x);
};


/*****************************************************************************/
class SpinBoxDelegate : public QStyledItemDelegate {
  Q_OBJECT
 public:
  explicit SpinBoxDelegate(QObject* parent = nullptr);

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const override;

  void setEditorData(QWidget* editor, const QModelIndex& index) const override;
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const override;

  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index) const override;
};


#endif  // CURSORTABLE_H
