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

#include "WbMessageBox.hpp"

#include "WbLog.hpp"

static bool gEnabled = true;

static QString removeHtmlTags(QString string) {
  string.replace("<br/>", "\n");
  string.replace("<strong>", "");
  string.replace("</strong>", "");
  return string;
}

void WbMessageBox::disable() {
  gEnabled = false;
}

bool WbMessageBox::enabled() {
  return gEnabled;
}

QMessageBox::StandardButton WbMessageBox::warning(const QString &warning, QWidget *parent, const QString &title,
                                                  QMessageBox::StandardButton defaultButton,
                                                  QMessageBox::StandardButtons buttons) {
  if (gEnabled)
    return QMessageBox::warning(parent, title, warning, buttons, defaultButton);
  else {
    WbLog::warning(removeHtmlTags(warning));
    return defaultButton;
  }
}

void WbMessageBox::info(const QString &message, QWidget *parent, const QString &title) {
  if (gEnabled)
    QMessageBox::information(parent, title, message);
  else
    WbLog::info(removeHtmlTags(message));
}

QMessageBox::StandardButton WbMessageBox::question(const QString &question, QWidget *parent, const QString &title,
                                                   QMessageBox::StandardButton defaultButton,
                                                   QMessageBox::StandardButtons buttons) {
  if (gEnabled)
    return QMessageBox::question(parent, title, question, buttons, defaultButton);
  else {
    WbLog::info(removeHtmlTags(question));
    return defaultButton;
  }
}

void WbMessageBox::critical(const QString &message, QWidget *parent, const QString &title) {
  if (gEnabled)
    QMessageBox::critical(parent, title, message);
  else
    WbLog::error(message);
}

void WbMessageBox::about(const QString &message, QWidget *parent, const QString &title) {
  if (gEnabled)
    QMessageBox::about(parent, title, message);
  else
    WbLog::info(removeHtmlTags(message));
}
