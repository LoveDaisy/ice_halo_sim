#include "cursortable.h"

#include <QMouseEvent>

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
