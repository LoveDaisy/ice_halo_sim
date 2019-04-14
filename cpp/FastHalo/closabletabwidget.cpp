#include "closabletabwidget.h"

#include <QDebug>
#include <QPalette>

QIcon& ClosableTabWidget::getIconNormal() {
  static QIcon icon(":/icons/icon_close_checked_normal_32.png");
  return icon;
}


QIcon& ClosableTabWidget::getIconOn() {
  static QIcon icon(":/icons/icon_close_on_32.png");
  return icon;
}


ClosableTabWidget::ClosableTabWidget(const QString& text, QWidget* parent)
    : QToolButton(parent), icon_enabled_(true) {
  setMouseTracking(true);
  setText(text);
  setIcon(getIconNormal());
  setCheckable(true);
  setIconSize(QSize(kIconSize, kIconSize));
  setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  setForegroundRole(QPalette::ButtonText);

  connect(this, &ClosableTabWidget::hoverOnIcon, this, &ClosableTabWidget::updateIcon);
}


void ClosableTabWidget::enableIcon(bool enabled) {
  icon_enabled_ = enabled;
}


bool ClosableTabWidget::iconEnabled() {
  return icon_enabled_;
}


void ClosableTabWidget::updateIcon(bool on_icon) {
  if (!icon_enabled_) {
    return;
  }
  if (on_icon) {
    setIcon(getIconOn());
  } else {
    setIcon(getIconNormal());
  }
}


void ClosableTabWidget::mouseMoveEvent(QMouseEvent* e) {
  if (icon_enabled_) {
    emit hoverOnIcon(isOnIcon(e->x()));
  }
  QToolButton::mouseMoveEvent(e);
}


void ClosableTabWidget::mouseReleaseEvent(QMouseEvent* e) {
  if (icon_enabled_ && isOnIcon(e->x())) {
    emit closeTab();
  } else {
    QToolButton::mouseReleaseEvent(e);
  }
}


void ClosableTabWidget::leaveEvent(QEvent* /* event */) {
  emit hoverOnIcon(false);
}


bool ClosableTabWidget::isOnIcon(int x) {
  return x < kIconSize * 2;
}
