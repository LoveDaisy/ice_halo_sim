#include <QApplication>

#include "mainwindow.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  MainWindow w;

  QFile file(":/styles/my_style.qss");
  file.open(QFile::ReadOnly | QFile::Text);
  QTextStream stream(&file);
  a.setStyleSheet(stream.readAll());

  w.show();

  return a.exec();
}
