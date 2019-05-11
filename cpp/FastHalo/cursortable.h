#ifndef CURSORTABLE_H
#define CURSORTABLE_H

#include <QTableView>
#include <map>

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

#endif  // CURSORTABLE_H
