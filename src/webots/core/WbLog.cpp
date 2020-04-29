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

void WbLog::debug(const QString &message, bool popup, const QString &logName) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, DEBUG);
    return;
  }

  fprintf(stderr, "DEBUG: %s\n", qPrintable(message));
  fflush(stderr);
  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))))
    instance()->emitLog(DEBUG, "DEBUG: " + message, popup, logName);
  else
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "DEBUG: " + message, DEBUG);
}

void WbLog::info(const QString &message, bool popup, const QString &logName) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, INFO);
    return;
  }

  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))))
    instance()->emitLog(INFO, "INFO: " + message, popup, logName);
  else {
    printf("INFO: %s\n", qPrintable(message));
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "INFO: " + message, INFO);
  }
}

void WbLog::warning(const QString &message, bool popup, const QString &logName) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, WARNING);
    return;
  }

  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))))
    instance()->emitLog(WARNING, "WARNING: " + message, popup, logName);
  else {
    fprintf(stderr, "WARNING: %s\n", qPrintable(message));
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "WARNING: " + message, WARNING);
  }
}

void WbLog::error(const QString &message, bool popup, const QString &logName) {
  if (popup && instance()->mPopUpMessagesPostponed) {
    instance()->enqueueMessage(instance()->mPostponedPopUpMessageQueue, message, ERROR);
    return;
  }

  if (instance()->receivers(SIGNAL(logEmitted(WbLog::Level, const QString &, bool, const QString &))))
    instance()->emitLog(ERROR, "ERROR: " + message, popup, logName);
  else {
    fprintf(stderr, "ERROR: %s\n", qPrintable(message));
    instance()->enqueueMessage(instance()->mPendingConsoleMessages, "ERROR: " + message, ERROR);
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

void WbLog::appendStdout(const QString &message, const QString &logName) {
  emit instance()->logEmitted(STDOUT, message, false, logName);
}

void WbLog::appendStderr(const QString &message, const QString &logName) {
  emit instance()->logEmitted(STDERR, message, false, logName);
}

void WbLog::javascriptLogToConsole(const QString &message, int lineNumber, const QString &sourceUrl) {
  QString sourceFile = QUrl::fromLocalFile(sourceUrl).fileName();
  QString log = "[javascript] " + message + " (" + sourceFile + ":" + QString::number(lineNumber) + ")";
  WbLog::appendStdout(log, QString());  // TODO: we might move the javascipt console to a dedicated tab
}

void WbLog::emitLog(Level level, const QString &message, bool popup, const QString &logName) {
  if (popup)
    emit popupOpen();
  emit logEmitted(level, message, popup, logName);
  if (popup)
    emit popupClosed();
}

void WbLog::clear() {
  emit instance()->cleared();
}

void WbLog::enqueueMessage(QList<PostponedMessage> &list, const QString &message, Level level) {
  PostponedMessage msg;
  msg.text = message;
  msg.level = level;
  list.append(msg);
}

void WbLog::showPostponedPopUpMessages() {
  bool tmp = instance()->mPopUpMessagesPostponed;
  setPopUpPostponed(false);
  foreach (PostponedMessage msg, instance()->mPostponedPopUpMessageQueue) {
    switch (msg.level) {
      case DEBUG:
        debug(msg.text, true);
        break;
      case INFO:
        info(msg.text, true);
        break;
      case WARNING:
        warning(msg.text, true);
        break;
      case ERROR:
        error(msg.text, true);
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
    emit instance()->logEmitted(msg.level, msg.text, false, QString());  // TODO
  instance()->mPendingConsoleMessages.clear();
}
