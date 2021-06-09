#include "WbQjsFile.hpp"

#include <QtCore/QFile>

WbQjsFile::WbQjsFile() {
}

WbQjsFile::~WbQjsFile() {
}

int WbQjsFile::callFunc(int number1, int number2) {
  // qDebug() << __FUNCTION__;
  return number1 + number2;
}

QString WbQjsFile::readfile(QString filename) {
  if (filename == "a")
    return "ASdsasad \n asdasdsa \n  sdasdsa";
  else
    return "bbbbb \n bbbbbbbbbbbb \n bbbbbbb";
}

QString WbQjsFile::readTextFile(const QString &filePath) {
  QFile file(filePath);

  if (!file.open(QIODevice::ReadOnly)) {
    // jsEngine->throwError(file.errorString());
    return "not found";
  }

  return file.readAll();
}

QString WbQjsFile::writeTextFile(const QString &filePath) {
}
