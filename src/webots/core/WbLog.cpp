// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "WbLog.hpp"

#include <stdio.h>
#include <QtCore/QCoreApplication>
#include <QtCore/QMetaType>
#include <QtCore/QUrl>
#include <cstdlib>

static WbLog *gInstance = NULL;

void WbLog::cleanup() {
  gInstance->mPostponedPopUpMessageQueue.clear();
  gInstance->mPendingConsoleMessages.clear();
  delete gInstance;
  gInstance = NULL;
}

WbLog *WbLog::instance() {
  if (!gInstance) {
    gInstance = new WbLog();
    gInstance->mPopUpMessagesPostponed = false;
    qRegisterMetaType<WbLog::Level>("WbLog::Level");
    qAddPostRoutine(WbLog::cleanup);
  }

  return gInstance;
}

void WbLog::debug(const QString &message, bool popup, const QString &name) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, DEBUG);
    return;
  }

  fprintf(stderr, "DEBUG: %s\n", qPrintable(message));
  fflush(stderr);
  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))) > 1)
    instance()->emitLog(DEBUG, "DEBUG: " + message, popup, name);
  else
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "DEBUG: " + message, name, DEBUG);
}

void WbLog::info(const QString &message, bool popup, const QString &name) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, INFO);
    return;
  }

  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))) > 1)
    instance()->emitLog(INFO, "INFO: " + message, popup, name);
  else {
    printf("INFO: %s\n", qPrintable(message));
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "INFO: " + message, name, INFO);
  }
}

void WbLog::warning(const QString &message, bool popup, const QString &name) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, WARNING);
    return;
  }

  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))) > 1)
    instance()->emitLog(WARNING, "WARNING: " + message, popup, name);
  else {
    fprintf(stderr, "WARNING: %s\n", qPrintable(message));
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "WARNING: " + message, name, WARNING);
  }
}

void WbLog::error(const QString &message, bool popup, const QString &name) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, name, ERROR);
    return;
  }
  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))) > 1)
    instance()->emitLog(ERROR, "ERROR: " + message, popup, name);
  else {
    fprintf(stderr, "ERROR: %s\n", qPrintable(message));
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "ERROR: " + message, name, ERROR);
  }
}

void WbLog::fatal(const QString &message) {
  fprintf(stderr, "FATAL: %s\n", qPrintable(message));
  fflush(stderr);
  instance()->emitLog(FATAL, "FATAL: " + message, true, QString());
  ::exit(EXIT_FAILURE);
}

void WbLog::status(const QString &message) {
  instance()->emitLog(STATUS, message, false, QString());
}

const QString &WbLog::filterName(Filter filter) {
  static const QStringList names = QStringList() << "All"
                                                 << "Webots"
                                                 << "All Webots"
                                                 << "Controller(s)"
                                                 << "All Controller(s)"
                                                 << "Others"
                                                 << "Physics"
                                                 << "Javascript"
                                                 << "Parsing";
  assert(names.size() == FILTER_SIZE);
  assert(filter < FILTER_SIZE);
  return names.at(filter);
};

const QString &WbLog::levelName(Level level) {
  static const QStringList names = QStringList() << "All"
                                                 << "Debug"
                                                 << "Info"
                                                 << "Warning"
                                                 << "Error"
                                                 << "Fatal"
                                                 << "Status"
                                                 << "Standard"
                                                 << "Error";
  assert(names.size() == LEVEL_SIZE);
  assert(level < LEVEL_SIZE);
  return names.at(level);
};

void WbLog::appendStdout(const QString &message, const QString &name) {
  emit instance()->logEmitted(STDOUT, message, false, name);
}

void WbLog::appendStderr(const QString &message, const QString &name) {
  emit instance()->logEmitted(STDERR, message, false, name);
}

void WbLog::javascriptLogToConsole(const QString &message, int lineNumber, const QString &sourceUrl) {
  QString sourceFile = QUrl::fromLocalFile(sourceUrl).fileName();
  QString log = "[javascript] " + message + " (" + sourceFile + ":" + QString::number(lineNumber) + ")";
  WbLog::appendStdout(log, filterName(WbLog::JAVASCRIPT));
}

void WbLog::emitLog(Level level, const QString &message, bool popup, const QString &name) {
  if (popup)
    emit popupOpen();
  emit logEmitted(level, message, popup, name);
  if (popup)
    emit popupClosed();
}

void WbLog::clear() {
  emit instance()->cleared();
}

void WbLog::enqueueMessage(QList<PostponedMessage> &list, const QString &message, const QString &name, Level level) {
  PostponedMessage msg;
  msg.text = message;
  msg.name = name;
  msg.level = level;
  list.append(msg);
}

void WbLog::showPostponedPopUpMessages() {
  bool tmp = instance()->mPopUpMessagesPostponed;
  setPopUpPostponed(false);
  foreach (PostponedMessage msg, instance()->mPostponedPopUpMessageQueue) {
    switch (msg.level) {
      case DEBUG:
        debug(msg.text, true, msg.name);
        break;
      case INFO:
        info(msg.text, true, msg.name);
        break;
      case WARNING:
        warning(msg.text, true, msg.name);
        break;
      case ERROR:
        error(msg.text, true, msg.name);
        break;
      default:
        break;
    }
  }
  setPopUpPostponed(tmp);

  instance()->mPostponedPopUpMessageQueue.clear();
}

void WbLog::showPendingConsoleMessages() {
  foreach (PostponedMessage msg, instance()->mPendingConsoleMessages)
    emit instance()->logEmitted(msg.level, msg.text, false, msg.name);
  instance()->mPendingConsoleMessages.clear();
}
