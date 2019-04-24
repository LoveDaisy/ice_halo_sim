#ifndef ICONBUTTON_H
#define ICONBUTTON_H

#include <QDebug>
#include <QMouseEvent>
#include <QToolButton>

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
  void closeTab();

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

#endif  // ICONBUTTON_H
