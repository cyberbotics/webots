#include "WbQjsFile.hpp"

#include <QtCore/QFile>

WbQjsFile::WbQjsFile() {
}

WbQjsFile::~WbQjsFile() {
}

QString WbQjsFile::readTextFile(const QString &filePath) {
  QFile file(filePath);

  if (!file.open(QIODevice::ReadOnly)) {
    // jsEngine->throwError(file.errorString());
    return "nada";
  }

  return file.readAll();
}

void WbQjsFile::writeTextFile(const QString &filePath) {
}
