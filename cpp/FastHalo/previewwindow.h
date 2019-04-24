#ifndef PREVIEWWINDOW_H
#define PREVIEWWINDOW_H

#include <QMainWindow>

namespace Ui {
class PreviewWindow;
}

class PreviewWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit PreviewWindow(QWidget* parent = nullptr);
  ~PreviewWindow();

 private:
  Ui::PreviewWindow* ui;
};

#endif  // PREVIEWWINDOW_H
