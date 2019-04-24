#include <QApplication>

#include "mainwindow.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  QFile file(":/styles/my_style.qss");
  file.open(QFile::ReadOnly | QFile::Text);
  QTextStream stream(&file);
  a.setStyleSheet(stream.readAll());

  return a.exec();
}
