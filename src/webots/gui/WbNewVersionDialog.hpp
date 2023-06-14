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

#ifndef WB_NEW_VERSION_DIALOG_HPP
#define WB_NEW_VERSION_DIALOG_HPP

//
// Description: This window is opened the first time this minor version of Webots is started
//

#include <QtWidgets/QDialog>

#define NUMBER_OF_THEMES 3

class QCheckBox;
class QLabel;
class QPushButton;
class QRadioButton;

class WbNewVersionDialog : public QDialog {
  Q_OBJECT
  Q_PROPERTY(QColor backgroundColor MEMBER mBackgroundColor READ backgroundColor WRITE setBackgroundColor)
  Q_PROPERTY(QString newVersionIconPath MEMBER mNewVersionIconPath READ newVersionIconPath WRITE setNewVersionIconPath)

public:
  static bool run();

  // qproperty methods
  const QColor &backgroundColor() const { return mBackgroundColor; }
  const QString &newVersionIconPath() const { return mNewVersionIconPath; }

  void setBackgroundColor(const QColor &color) { mBackgroundColor = color; }
  void setNewVersionIconPath(const QString &path) { mNewVersionIconPath = path; }

private slots:
  void startButtonPressed();
  void updatePreview();

private:
  WbNewVersionDialog();
  virtual ~WbNewVersionDialog() {}

  QColor mBackgroundColor;
  QString mNewVersionIconPath;
  QRadioButton *mRadioButtons[NUMBER_OF_THEMES];
  QLabel *mPreviewLabel;
  QCheckBox *mTelemetryCheckBox;
};

#endif
