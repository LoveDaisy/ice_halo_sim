#ifndef CLOSABLETABWIDGET_H
#define CLOSABLETABWIDGET_H

#include <QToolButton>
#include <QMouseEvent>
#include <QDebug>

class ClosableTabWidget : public QToolButton {
  Q_OBJECT
 public:
  explicit ClosableTabWidget(const QString& text, QWidget* parent = nullptr);
  void enableIcon(bool enabled = true);
  bool iconEnabled();

  static constexpr int kIconSize = 10;

 signals:
  void hoverOnIcon(bool on = false);
  void closeTab();

 public slots:
  void updateIcon(bool on_icon);

 protected:
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  void leaveEvent(QEvent *event) override;

  bool icon_enabled_;

  static QIcon& getIconNormal();
  static QIcon& getIconOn();

private:
  bool isOnIcon(int x);
};

#endif  // CLOSABLETABWIDGET_H
