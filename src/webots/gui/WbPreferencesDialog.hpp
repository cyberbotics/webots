// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_PREFERENCES_DIALOG_HPP
#define WB_PREFERENCES_DIALOG_HPP

//
// Description: dialog allows the user to modify Webots preferences
//

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>

class WbLineEdit;

class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QTabWidget;

class WbPreferencesDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbPreferencesDialog(QWidget *parent = NULL, const QString &defaultTab = "");
  virtual ~WbPreferencesDialog();

signals:
  void changedByUser();
  void restartRequested();

private slots:
  void accept() override;
  void openFontDialog();
  void clearCache();

private:
  QString retrieveThemeName(const QString &filename) const;

  QTabWidget *mTabWidget;

  int mNumberOfThreads;

  // General
  QComboBox *mNumberOfThreadsCombo;

  QDialogButtonBox *mButtonBox;
  QComboBox *mLanguageCombo, *mThemeCombo, *mStartupModeCombo, *mAmbientOcclusionCombo, *mTextureQualityCombo,
    *mTextureFilteringCombo;
  WbLineEdit *mEditorFontEdit, *mPythonCommand, *mMatlabCommand, *mExtraProjectsPath, *mHttpProxyHostName, *mHttpProxyPort,
    *mHttpProxyUsername, *mHttpProxyPassword, *mCacheSize;
  QCheckBox *mDisableSaveWarningCheckBox, *mCheckWebotsUpdateCheckBox, *mTelemetryCheckBox, *mDisableShadowsCheckBox,
    *mDisableAntiAliasingCheckBox, *mHttpProxySocks5CheckBox, *mRenderingCheckBox;

  QStringList mValidThemeFilenames;

  QWidget *createGeneralTab();
  QWidget *createOpenGLTab();
  QWidget *createNetworkTab();
};

#endif
