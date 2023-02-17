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

#ifndef WB_MESSAGE_BOX_HPP
#define WB_MESSAGE_BOX_HPP

#include <QtWidgets/QMessageBox>
#include <QtWidgets/QWidget>

namespace WbMessageBox {
  QMessageBox::StandardButton warning(const QString &warning, QWidget *parent = NULL,
                                      const QString &title = QObject::tr("Warning"),
                                      QMessageBox::StandardButton defaultButton = QMessageBox::NoButton,
                                      QMessageBox::StandardButtons buttons = QMessageBox::Ok);
  void info(const QString &message, QWidget *parent = NULL, const QString &title = QObject::tr("Information"));
  QMessageBox::StandardButton question(const QString &question, QWidget *parent = NULL,
                                       const QString &title = QObject::tr("Question"),
                                       QMessageBox::StandardButton defaultButton = QMessageBox::Cancel,
                                       QMessageBox::StandardButtons buttons = QMessageBox::Ok | QMessageBox::Cancel);
  void critical(const QString &message, QWidget *parent = NULL, const QString &title = QObject::tr("Critical"));
  void about(const QString &message, QWidget *parent = NULL, const QString &title = QObject::tr("About"));
  void disable();
  bool enabled();
};  // namespace WbMessageBox

#endif
