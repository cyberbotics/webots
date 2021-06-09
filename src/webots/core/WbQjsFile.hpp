#ifndef WB_QJS_FILE_HPP
#define WB_QJS_FILE_HPP

#include <QtCore/QObject>

class WbQjsFile : public QObject {
  Q_OBJECT

public:
  Q_INVOKABLE WbQjsFile();
  ~WbQjsFile();

  Q_INVOKABLE int callFunc(int number1, int number2);
  Q_INVOKABLE QString readfile(QString filename);
  Q_INVOKABLE QString readTextFile(const QString &filePath);
  Q_INVOKABLE QString writeTextFile(const QString &filePath);
};

#endif
