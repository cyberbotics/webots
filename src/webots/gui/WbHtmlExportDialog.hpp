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

#ifndef WB_HTML_EXPORT_DIALOG_HPP
#define WB_HTML_EXPORT_DIALOG_HPP

//
// Description: dialog to set HTML5/X3D export parameters.
//              The chosen X3D parameters are stored in WBPROJ file.
//

#include <QtWidgets/QDialog>

class QCheckBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QSlider;
class QSpinBox;

class WbHtmlExportDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbHtmlExportDialog(const QString &title, const QString &worldFilePath, QWidget *parent = NULL);
  virtual ~WbHtmlExportDialog();

  QString fileName();

signals:
  void changedByUser();

private:
  QDialogButtonBox *mButtonBox;
  QDoubleSpinBox *mShadowIntensityEdit;
  QSlider *mShadowIntensitySlider;
  QLabel *mShadowMapLabel;
  QSlider *mShadowMapSlider;
  QSpinBox *mShadowFilterEdit;
  QSlider *mShadowFilterSlider;
  QSpinBox *mShadowCascadesEdit;
  QSlider *mShadowCascadesSlider;
  QCheckBox *mFrustumCullingCheckBox;
  QLineEdit *mFileLineEdit;
  QString mTitle;
  QString mWorldFilePath;

  int computeShadowMapSliderIndex(int value);
  QSlider *createSlider(const QString &parameterName);
  QSpinBox *createSpinBox(const QString &parameterName);
  static QString convertBoolToString(bool value) { return value ? "true" : "false"; }

private slots:
  void accept() override;
  void updateShadowEditValue(int value);
  void updateShadowSliderValue(int value);
  void updateShadowIntensitySliderValue(double value);
  void browse();
};

#endif
