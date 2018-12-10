// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbPreferencesDialog.hpp"

#include "WbApplication.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbNetwork.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"
#include "WbSysInfo.hpp"
#include "WbTranslator.hpp"

#include <QtCore/QDir>
#include <QtCore/QStringList>
#include <QtCore/QTextStream>
#include <QtCore/QThread>
#include <QtNetwork/QNetworkProxy>

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFontDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>

static QStringList gStartupModes;

WbPreferencesDialog::WbPreferencesDialog(QWidget *parent, const QString &defaultTab) : QDialog(parent) {
  setWindowTitle(tr("Preferences"));

  gStartupModes.clear();
  gStartupModes << "Pause"
                << "Real-time"
                << "Run"
                << "Fast";

  mTabWidget = new QTabWidget(this);
  mTabWidget->addTab(createGeneralTab(), tr("General"));
  mTabWidget->addTab(createOpenGLTab(), tr("OpenGL"));
  mTabWidget->addTab(createNetworkTab(), tr("Network Proxy"));

  for (int i = 0; i < mTabWidget->count(); ++i) {
    if (mTabWidget->tabText(i) == defaultTab) {
      mTabWidget->setCurrentIndex(i);
      break;
    }
  }

  mButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
  connect(mButtonBox, &QDialogButtonBox::accepted, this, &WbPreferencesDialog::accept);
  connect(mButtonBox, &QDialogButtonBox::rejected, this, &WbPreferencesDialog::reject);

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(mTabWidget);
  mainLayout->addWidget(mButtonBox);

  WbPreferences *prefs = WbPreferences::instance();
  connect(this, &WbPreferencesDialog::changedByUser, prefs, &WbPreferences::changedByUser);

  // general tab
  int index = gStartupModes.indexOf(prefs->value("General/startupMode").toString());
  mStartupModeCombo->setCurrentIndex(index != -1 ? index : 0);
  mEditorFontEdit->setText(prefs->value("Editor/font").toString());
  mNumberOfThreads = prefs->value("General/numberOfThreads", 1).toInt();
  mNumberOfThreadsCombo->setCurrentIndex(mNumberOfThreads - 1);
  mPythonCommand->setText(prefs->value("General/pythonCommand").toString());
  mCheckWebotsUpdateCheckBox->setChecked(prefs->value("General/checkWebotsUpdateOnStartup").toBool());
  mDisableSaveWarningCheckBox->setChecked(prefs->value("General/disableSaveWarning").toBool());

  // openGL tab
  index = prefs->value("OpenGL/SMAA", true).toBool();
  // compatibility with old preferences
  if (index > 0)
    mAntiAliasingCombo->setCurrentIndex(1);
  else
    mAntiAliasingCombo->setCurrentIndex(0);

  mAmbientOcclusionCombo->setCurrentIndex(prefs->value("OpenGL/GTAO", 2).toInt());
  mTextureQualityCombo->setCurrentIndex(prefs->value("OpenGL/TextureQuality", 2).toInt());

  mDisableShadowsCheckBox->setChecked(prefs->value("OpenGL/disableShadows").toBool());
  mDisableCameraAntiAliasingCheckBox->setChecked(prefs->value("OpenGL/disableCameraAntiAliasing").toBool());

  // network tab
  mHttpProxySocks5CheckBox->setChecked(prefs->value("Network/httpProxyType").toInt() == QNetworkProxy::Socks5Proxy);
  mHttpProxyHostName->setText(prefs->value("Network/httpProxyHostName").toString());
  const int port = prefs->value("Network/httpProxyPort", 0).toInt();
  if (port > 0)
    mHttpProxyPort->setText(QString::number(port));
  mHttpProxyUsername->setText(prefs->value("Network/httpProxyUsername").toString());
  mHttpProxyPassword->setText(prefs->value("Network/httpProxyPassword").toString());
  mLanguageCombo->setFocus();
}

WbPreferencesDialog::~WbPreferencesDialog() {
}

void WbPreferencesDialog::accept() {
  WbPreferences *prefs = WbPreferences::instance();

  // Inform when restarting Webots is required
  bool willRestart = false;
  const QString &languageKey = WbTranslator::instance()->findKey(mLanguageCombo->currentText());
  if (languageKey != prefs->value("General/language") ||
      prefs->value("General/theme").toString() != mValidThemeFilenames.at(mThemeCombo->currentIndex()) ||
      prefs->value("OpenGL/disableCameraAntiAliasing").toBool() != mDisableCameraAntiAliasingCheckBox->isChecked()) {
    willRestart = WbMessageBox::question(
                    tr("You have changed some settings which require Webots to be restarted. Restart Webots Now?"), this,
                    tr("Restart Now?"), QMessageBox::Yes, QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
  }
  if (!willRestart && prefs->value("OpenGL/TextureQuality", 2).toInt() != mTextureQualityCombo->currentIndex())
    WbMessageBox::info(tr("The new texture quality will be applied next time the world is loaded."), this);
  // Inform the user about possible issues with multi-threading
  if (mNumberOfThreadsCombo->currentIndex() + 1 != mNumberOfThreads && mNumberOfThreadsCombo->currentIndex() != 0)
    WbMessageBox::warning(
      tr("Physics multi-threading is enabled. "
         "This can have a noticeable impact on the simulation speed (negative or positive depending on the simulated world). "
         "In case of multi-threading, simulation replicability is not guaranteed. "),
      this);

  // general tab
  prefs->setValue("General/startupMode", gStartupModes.value(mStartupModeCombo->currentIndex()));
  prefs->setValue("Editor/font", mEditorFontEdit->text());
  prefs->setValue("General/language", languageKey);
  prefs->setValue("General/theme", mValidThemeFilenames.at(mThemeCombo->currentIndex()));
  prefs->setValue("General/numberOfThreads", mNumberOfThreadsCombo->currentIndex() + 1);
  prefs->setValue("General/pythonCommand", mPythonCommand->text());
  prefs->setValue("General/checkWebotsUpdateOnStartup", mCheckWebotsUpdateCheckBox->isChecked());
  prefs->setValue("General/disableSaveWarning", mDisableSaveWarningCheckBox->isChecked());

  // openGL
  prefs->setValue("OpenGL/SMAA", static_cast<bool>(mAntiAliasingCombo->currentIndex()));
  prefs->setValue("OpenGL/GTAO", mAmbientOcclusionCombo->currentIndex());
  prefs->setValue("OpenGL/TextureQuality", mTextureQualityCombo->currentIndex());
  prefs->setValue("OpenGL/disableShadows", mDisableShadowsCheckBox->isChecked());
  prefs->setValue("OpenGL/disableCameraAntiAliasing", mDisableCameraAntiAliasingCheckBox->isChecked());

  // network
  enum QNetworkProxy::ProxyType type;
  const bool changed =
    (prefs->value("Network/httpProxyHostName").toString() != mHttpProxyHostName->text()) ||
    (prefs->value("Network/httpProxyPort").toString() != mHttpProxyPort->text()) ||
    (prefs->value("Network/httpProxyUsername").toString() != mHttpProxyUsername->text()) ||
    (prefs->value("Network/httpProxyPassword").toString() != mHttpProxyPassword->text()) ||
    ((prefs->value("Network/httpProxyType").toInt() == QNetworkProxy::Socks5Proxy) != mHttpProxySocks5CheckBox->isChecked());
  if (mHttpProxyHostName->text().isEmpty())  // if any setting was changed and hostname is empty, init from default system proxy
    type = changed ? QNetworkProxy::DefaultProxy : QNetworkProxy::NoProxy;  // otherwise, don't use any proxy
  else if (mHttpProxySocks5CheckBox->isChecked())
    type = QNetworkProxy::Socks5Proxy;
  else
    type = QNetworkProxy::HttpProxy;
  prefs->setValue("Network/httpProxyType", type);
  prefs->setValue("Network/httpProxyHostName", mHttpProxyHostName->text());
  bool ok;
  int port = mHttpProxyPort->text().toInt(&ok);
  if (!ok)
    port = 0;
  prefs->setValue("Network/httpProxyPort", port);
  prefs->setValue("Network/httpProxyUsername", mHttpProxyUsername->text());
  prefs->setValue("Network/httpProxyPassword", mHttpProxyPassword->text());
  prefs->sync();
  if (changed)
    WbNetwork::instance()->setProxy();
  emit changedByUser();
  QDialog::accept();
  if (willRestart)
    emit restartRequested();
}

void WbPreferencesDialog::openFontDialog() {
  QFont initial;
  initial.fromString(mEditorFontEdit->text());

  bool ok;
  QFont font = QFontDialog::getFont(&ok, initial, this);
  if (ok)
    mEditorFontEdit->setText(font.toString());
}

QWidget *WbPreferencesDialog::createGeneralTab() {
  QWidget *widget = new QWidget(this);
  QGridLayout *layout = new QGridLayout(widget);

  mLanguageCombo = new QComboBox(this);
  WbTranslator *t = WbTranslator::instance();
  const QString &languageKey = WbPreferences::instance()->value("General/language").toString();
  const QStringList &languages = t->computeUserReadableLanguages();
  foreach (const QString &language, languages)
    mLanguageCombo->addItem(language);
  mLanguageCombo->setCurrentIndex(t->findIndex(languageKey));

  mThemeCombo = new QComboBox(this);
  QDir dir(WbStandardPaths::resourcesPath());
  QStringList filters;
  filters << "*.qss";
  QStringList stylesheets = dir.entryList(filters);
  foreach (const QString &stylesheet, stylesheets) {
    if (!stylesheet.contains(".macOS", Qt::CaseInsensitive) && !stylesheet.contains(".linux", Qt::CaseInsensitive) &&
        !stylesheet.contains(".windows", Qt::CaseInsensitive)) {
      QString themeName = retrieveThemeName(stylesheet);
      if (!themeName.isEmpty()) {
        mValidThemeFilenames << stylesheet;
        mThemeCombo->addItem(themeName);
      }
    }
  }
  // set correct index of theme combo box
  for (int i = 0; i < mValidThemeFilenames.size(); ++i) {
    if (WbPreferences::instance()->value("General/theme").toString() == mValidThemeFilenames.at(i)) {
      mThemeCombo->setCurrentIndex(i);
      break;
    }
  }

  mStartupModeCombo = new QComboBox(this);
  foreach (const QString &mode, gStartupModes)
    mStartupModeCombo->addItem(mode);

  mNumberOfThreadsCombo = new QComboBox(this);
  int max = qMax(1, QThread::idealThreadCount());
  int def = WbSysInfo::coreCount();
  for (int i = 1; i <= max; i++) {
    QString item = QString::number(i);
    if (i == def)
      item += " " + tr("(recommended)");
    mNumberOfThreadsCombo->addItem(item);
  }

  mEditorFontEdit = new WbLineEdit(this);
  mPythonCommand = new WbLineEdit(this);

  // row 0
  layout->addWidget(new QLabel(tr("Language:"), this), 0, 0);
  layout->addWidget(mLanguageCombo, 0, 1);
  layout->addWidget(new QLabel(tr("(restart needed)"), this), 0, 2);

  // row 1
  layout->addWidget(new QLabel(tr("Theme:"), this), 1, 0);
  layout->addWidget(mThemeCombo, 1, 1);
  layout->addWidget(new QLabel(tr("(restart needed)"), this), 1, 2);

  // row 2
  layout->addWidget(new QLabel(tr("Startup mode:"), this), 2, 0);
  layout->addWidget(mStartupModeCombo, 2, 1);

  QPushButton *chooseFontButton = new QPushButton("...", this);
  connect(chooseFontButton, &QPushButton::pressed, this, &WbPreferencesDialog::openFontDialog);

  // row 3
  layout->addWidget(new QLabel(tr("Editor font:"), this), 3, 0);
  layout->addWidget(mEditorFontEdit, 3, 1);
  layout->addWidget(chooseFontButton, 3, 2);

  // row 4
  layout->addWidget(new QLabel(tr("Number of threads:"), this), 4, 0);
  layout->addWidget(mNumberOfThreadsCombo, 4, 1);

  // row 5
  layout->addWidget(new QLabel(tr("Python command:"), this), 5, 0);
  layout->addWidget(mPythonCommand, 5, 1);

  // row 6
  mDisableSaveWarningCheckBox = new QCheckBox(tr("Display save warning only for scene tree edit"), this);
  mDisableSaveWarningCheckBox->setToolTip(
    tr("If this option is enabled, Webots will not display any warning when you quit, reload\nor load a new world after the "
       "current world was modified by either changing the viewpoint,\ndragging, rotating, applying a force or applying a "
       "torque to an object. It will however\nstill display a warning if the world was modified from the scene tree."));
  layout->addWidget(new QLabel(tr("Warnings:"), this), 6, 0);
  layout->addWidget(mDisableSaveWarningCheckBox, 6, 1);

  // row 7
  mCheckWebotsUpdateCheckBox = new QCheckBox(tr("Check for Webots updates on startup"), this);
  mCheckWebotsUpdateCheckBox->setToolTip(tr("If this option is enabled, Webots will check if a new version is available for "
                                            "download\nat every startup. If available, it will inform you about it."));
  layout->addWidget(new QLabel(tr("Update policy:"), this), 7, 0);
  layout->addWidget(mCheckWebotsUpdateCheckBox, 7, 1);

  setTabOrder(mStartupModeCombo, mEditorFontEdit);
  setTabOrder(mEditorFontEdit, chooseFontButton);

  return widget;
}

QWidget *WbPreferencesDialog::createOpenGLTab() {
  QWidget *widget = new QWidget(this);
  QGridLayout *layout = new QGridLayout(widget);

  // row 0
  mAntiAliasingCombo = new QComboBox(this);
  mAntiAliasingCombo->addItem(tr("Disabled"));
  mAntiAliasingCombo->addItem(tr("Enabled"));
  layout->addWidget(new QLabel(tr("Main 3D view anti-aliasing:"), this), 0, 0);
  layout->addWidget(mAntiAliasingCombo, 0, 1, Qt::AlignLeft);

  // row 1
  mAmbientOcclusionCombo = new QComboBox(this);
  mAmbientOcclusionCombo->addItem(tr("Disabled"));
  mAmbientOcclusionCombo->addItem(tr("Low"));
  mAmbientOcclusionCombo->addItem(tr("Medium"));
  mAmbientOcclusionCombo->addItem(tr("High"));
  mAmbientOcclusionCombo->addItem(tr("Ultra"));
  layout->addWidget(new QLabel(tr("Ambient Occlusion:"), this), 1, 0);
  layout->addWidget(mAmbientOcclusionCombo, 1, 1, Qt::AlignLeft);

  mTextureQualityCombo = new QComboBox(this);
  mTextureQualityCombo->addItem(tr("Low"));
  mTextureQualityCombo->addItem(tr("Medium"));
  mTextureQualityCombo->addItem(tr("High"));
  layout->addWidget(new QLabel(tr("Texture Quality:"), this), 2, 0);
  layout->addWidget(mTextureQualityCombo, 2, 1, Qt::AlignLeft);

  // row 3
  layout->addWidget(new QLabel(tr("Options:"), this), 3, 0);
  mDisableShadowsCheckBox = new QCheckBox(tr("Disable shadows"), this);
  layout->addWidget(mDisableShadowsCheckBox, 3, 1, Qt::AlignLeft);

  // row 4
  mDisableCameraAntiAliasingCheckBox = new QCheckBox(tr("Disable camera anti-aliasing"), this);
  layout->addWidget(mDisableCameraAntiAliasingCheckBox, 4, 1, Qt::AlignLeft);

  return widget;
}

QWidget *WbPreferencesDialog::createNetworkTab() {
  QWidget *widget = new QWidget(this);
  QGridLayout *layout = new QGridLayout(widget);
  // row 0
  mHttpProxySocks5CheckBox = new QCheckBox(tr("SOCKS v5"), this);
  mHttpProxySocks5CheckBox->setToolTip(tr("Activate SOCK5 proxying."));
  layout->addWidget(new QLabel(tr("Proxy type:"), this), 0, 0);
  layout->addWidget(mHttpProxySocks5CheckBox, 0, 1);

  // row 1
  mHttpProxyHostName = new WbLineEdit(this);
  layout->addWidget(new QLabel(tr("Proxy hostname:"), this), 1, 0);
  layout->addWidget(mHttpProxyHostName, 1, 1);

  // row 2
  mHttpProxyPort = new WbLineEdit(this);
  mHttpProxyPort->setValidator(new QIntValidator(0, 65535));
  layout->addWidget(new QLabel(tr("Proxy port:"), this), 2, 0);
  layout->addWidget(mHttpProxyPort, 2, 1);

  // row 3
  mHttpProxyUsername = new WbLineEdit(this);
  layout->addWidget(new QLabel(tr("Proxy username:"), this), 3, 0);
  layout->addWidget(mHttpProxyUsername, 3, 1);

  // row 4
  mHttpProxyPassword = new WbLineEdit(this);
  mHttpProxyPassword->setEchoMode(QLineEdit::PasswordEchoOnEdit);
  layout->addWidget(new QLabel(tr("Proxy password:"), this), 4, 0);
  layout->addWidget(mHttpProxyPassword, 4, 1);

  return widget;
}

QString WbPreferencesDialog::retrieveThemeName(const QString &filename) const {
  QFile inputFile(WbStandardPaths::resourcesPath() + filename);
  QString themeNameLine;
  if (inputFile.open(QIODevice::ReadOnly)) {
    QTextStream in(&inputFile);
    themeNameLine = in.readLine();
  }

  QRegularExpression re("/\\*\\s*theme-name\\s*:\\s*([^\\*]*)\\*/");
  QRegularExpressionMatch match = re.match(themeNameLine);
  if (match.hasMatch())
    return match.captured(1);
  else
    return "";
}
