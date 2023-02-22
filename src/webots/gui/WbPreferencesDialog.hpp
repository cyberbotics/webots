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

#ifndef WB_PREFERENCES_DIALOG_HPP
#define WB_PREFERENCES_DIALOG_HPP

//
// Description: dialog allows the user to modify Webots preferences
//

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QListWidget>

class WbLineEdit;

class QCheckBox;
class QComboBox;
class QListWidget;
class QSpinBox;
class QDialogButtonBox;
class QTabWidget;
class QKeyEvent;
class QLabel;

class WbPreferencesDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbPreferencesDialog(QWidget *parent = NULL, const QString &defaultTab = "");
  virtual ~WbPreferencesDialog();

protected:
  void keyPressEvent(QKeyEvent *event) override;

signals:
  void changedByUser();
  void restartRequested();

private slots:
  void accept() override;
  void openFontDialog();
  void clearCache();
  void addNewIp();
  void removeSelectedIp();

private:
  QString retrieveThemeName(const QString &filename) const;

  QTabWidget *mTabWidget;

  int mNumberOfThreads;

  // General
  QComboBox *mNumberOfThreadsCombo;

  QDialogButtonBox *mButtonBox;
  QComboBox *mLanguageCombo, *mThemeCombo, *mStartupModeCombo, *mAmbientOcclusionCombo, *mTextureQualityCombo,
    *mTextureFilteringCombo;
  WbLineEdit *mEditorFontEdit, *mPythonCommand, *mMatlabCommand, *mExtraProjectPath, *mHttpProxyHostName, *mHttpProxyPort,
    *mHttpProxyUsername, *mHttpProxyPassword, *mUploadUrl, *mBrowserProgram;
  QCheckBox *mDisableSaveWarningCheckBox, *mThumnailCheckBox, *mCheckWebotsUpdateCheckBox, *mTelemetryCheckBox,
    *mDisableShadowsCheckBox, *mDisableAntiAliasingCheckBox, *mHttpProxySocks5CheckBox, *mRenderingCheckBox, *mNewBrowserWindow;
  QSpinBox *mCacheSize;
  QListWidget *mAllowedIps;
  QLabel *mCacheSizeLabel;

  QStringList mValidThemeFilenames;

  QWidget *createGeneralTab();
  QWidget *createOpenGLTab();
  QWidget *createNetworkTab();
};

#endif
