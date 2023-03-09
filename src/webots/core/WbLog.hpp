// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
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

#include <stdio.h>

#include <QtCore/QObject>
#include <QtCore/QVector>

// ERROR is apparently already defined under Windows, which causes trouble with the enums below
#ifdef ERROR
#undef ERROR
#endif

class WbLog : public QObject {
  Q_OBJECT

public:
  enum Level { DEBUG, INFO, WARNING, ERROR, FATAL, STATUS, STDOUT, STDERR, LEVEL_SIZE };
  enum Filter {
    ALL,
    WEBOTS,
    ALL_WEBOTS,
    CONTROLLERS,
    ALL_CONTROLLERS,
    WEBOTS_OTHERS,
    ODE,
    PHYSICS_PLUGINS,
    JAVASCRIPT,
    PARSING,
    COMPILATION,
    FILTER_SIZE
  };

  static WbLog *instance();

  // Webots messages, e.g. parse error
  // the 'message' argument should not be '\n' terminated
  static void debug(const QString &message, bool popup = false, Filter filter = WEBOTS_OTHERS);
  static void info(const QString &message, bool popup = false, Filter filter = WEBOTS_OTHERS);
  static void warning(const QString &message, bool popup = false, Filter filter = WEBOTS_OTHERS);
  static void error(const QString &message, bool popup = false, Filter filter = WEBOTS_OTHERS);
  static void debug(const QString &message, const QString &name, bool popup = false);
  static void info(const QString &message, const QString &name, bool popup = false);
  static void warning(const QString &message, const QString &name, bool popup = false);
  static void error(const QString &message, const QString &name, bool popup = false);

  // log type filter names
  static const QString &filterName(Filter filter);
  static const QStringList &webotsFilterNames();
  // level type names
  static const QString &levelName(Level level);

  // display a message in main window's status bar
  static void status(const QString &message);

  // display message and terminate Webots
  static void fatal(const QString &message);

  // enable redirecting messages to the terminal
  static void enableStdOutRedirectToTerminal();
  static void enableStdErrRedirectToTerminal();

  // controller or compilation output
  // the 'message' argument can contain newlines (multi-line output)
  static void appendStdout(const QString &message, Filter filter = WEBOTS_OTHERS);
  static void appendStderr(const QString &message, Filter filter = WEBOTS_OTHERS);
  static void appendStdout(const QString &message, const QString &name);
  static void appendStderr(const QString &message, const QString &name);

  static void javascriptLogToConsole(const QString &message, int lineNumber, const QString &sourceUrl);

  // queue pop up messages to be shown later
  static void setPopUpPostponed(bool postponed) { instance()->mPopUpMessagesPostponed = postponed; }
  static void showPostponedPopUpMessages();
  static void setConsoleLogsPostponed(bool postponed) { instance()->mConsoleLogsPostponed = postponed; }
  // show messages in console emitted before WbConsole was listening
  static void showPendingConsoleMessages();
  static void toggle(FILE *std_stream);  // disable or enable stderr or stdout
signals:
  // the above function emit this signal that can be connected to a message sink (console)
  void logEmitted(WbLog::Level level, const QString &message, bool popup, const QString &name);
  void popupOpen();
  void popupClosed();

private:
  WbLog() : mPopUpMessagesPostponed(false), mConsoleLogsPostponed(false) {}
  virtual ~WbLog() {}
  void emitLog(Level level, const QString &message, bool popup, const QString &name);
  static void cleanup();

  struct PostponedMessage {
    QString text;
    QString name;
    Level level;
  };
  bool mPopUpMessagesPostponed;
  bool mConsoleLogsPostponed;
  QList<PostponedMessage> mPostponedPopUpMessageQueue;
  QList<PostponedMessage> mPendingConsoleMessages;
  void enqueueMessage(QList<PostponedMessage> &list, const QString &message, const QString &name, Level level);
};

#endif
