#include "iconbutton.h"

#include <QDebug>
#include <QPalette>

#include "icons.h"


IconButton::IconButton(const QString& text, QWidget* parent) : QToolButton(parent), icon_enabled_(true) {
  setMouseTracking(true);
  setText(text);
  //  setIcon(Icons::getIcon(Icons::kCloseNormal));
  setCheckable(true);
  setIconSize(kDefaultIconSize);
  setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  setForegroundRole(QPalette::ButtonText);

  connect(this, &IconButton::hoverOnIcon, this, &IconButton::updateIcon);
}


void IconButton::enableIcon(bool enabled) {
  icon_enabled_ = enabled;
}


bool IconButton::iconEnabled() {
  return icon_enabled_;
}


int IconButton::iconSize() const {
  return icon_size_;
}


void IconButton::setIconSize(int size) {
  icon_size_ = size;
  QToolButton::setIconSize(QSize(size, size));
}


void IconButton::setIcons(QIcon icon_normal, QIcon icon_on) {
  icon_normal_ = icon_normal;
  icon_on_ = icon_on;
  updateIcon(false);
}


void IconButton::updateIcon(bool on_icon) {
  if (!icon_enabled_) {
    return;
  }
  if (icon_on_.isNull() && icon_normal_.isNull()) {
    return;
  }

  if (on_icon) {
    if (!icon_on_.isNull()) {
      setIcon(icon_on_);
    } else if (!icon_normal_.isNull()) {
      setIcon(icon_normal_);
    }
  } else {
    if (!icon_normal_.isNull()) {
      setIcon(icon_normal_);
    }
  }
}


void IconButton::mouseMoveEvent(QMouseEvent* e) {
  if (icon_enabled_) {
    emit hoverOnIcon(isOnIcon(e->x()));
  }
  QToolButton::mouseMoveEvent(e);
}


void IconButton::mouseReleaseEvent(QMouseEvent* e) {
  if (icon_enabled_ && isOnIcon(e->x())) {
    emit closeTab();
  } else {
    QToolButton::mouseReleaseEvent(e);
  }
}


void IconButton::leaveEvent(QEvent* /* event */) {
  emit hoverOnIcon(false);
}


bool IconButton::isOnIcon(int x) {
  return x < kDefaultIconSize * 2;
}
