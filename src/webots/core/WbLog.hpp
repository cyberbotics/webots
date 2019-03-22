// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_LOG_HPP
#define WB_LOG_HPP

//
// Description: message/error reporting system
//   All messages should be reported here, this class is responsible to forward the messages to
//   the correct targe, e.g. Webots console, terminal, error dialog
//

#include <QtCore/QObject>
#include <QtCore/QVector>

// ERROR is apparently already defined under Windows, which causes trouble with the enums below
#ifdef ERROR
#undef ERROR
#endif

class WbLog : public QObject {
  Q_OBJECT

public:
  enum Level { DEBUG, INFO, WARNING, ERROR, FATAL, STATUS, STDOUT, STDERR };

  static WbLog *instance();

  // Webots messages, e.g. parse error
  // the 'message' argument should not be '\n' terminated
  static void debug(const QString &message, bool popup = false);
  static void info(const QString &message, bool popup = false);
  static void warning(const QString &message, bool popup = false);
  static void error(const QString &message, bool popup = false);

  // display a message in main window's status bar
  static void status(const QString &message);

  // display message and terminate Webots
  static void fatal(const QString &message);

  // controller or compilation output
  // the 'message' argument can contain newlines (multi-line output)
  static void appendStdout(const QString &message, const QString &prefix = "");
  static void appendStderr(const QString &message, const QString &prefix = "");

  static void javascriptLogToConsole(const QString &message, int lineNumber, const QString &sourceUrl);
  // clear output
  static void clear();

  // queue pop up messages to be shown later
  static void setPopUpPostponed(bool postponed) { instance()->mPopUpMessagesPostponed = postponed; }
  static void showPostponedPopUpMessages();

signals:
  // the above function emit this signal that can be connected to a message sink (console)
  void logEmitted(WbLog::Level level, const QString &message, bool popup);
  void controllerLogEmitted(WbLog::Level level, const QString &message, const QString &prefix, bool popup);
  void cleared();
  void popupOpen();
  void popupClosed();

private:
  WbLog() : mPopUpMessagesPostponed(false) {}
  virtual ~WbLog() {}
  void emitLog(Level level, const QString &message, bool popup);
  static void cleanup();

  struct PostponedPopUpMessage {
    QString text;
    Level level;
  };

  bool mPopUpMessagesPostponed;
  QVector<PostponedPopUpMessage> mPostponedPopUpMessageQueue;
  void enqueuePopUpMessage(const QString &message, Level level);
};

#endif
