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

#include "WbPreferencesDialog.hpp"

#include "WbApplication.hpp"
#include "WbDesktopServices.hpp"
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
#include <QtGui/QKeyEvent>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkDiskCache>
#include <QtNetwork/QNetworkProxy>

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFontDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

static QStringList gStartupModes;

WbPreferencesDialog::WbPreferencesDialog(QWidget *parent, const QString &defaultTab) : QDialog(parent) {
  setWindowTitle(tr("Preferences"));

  gStartupModes.clear();
  gStartupModes << "Pause"
                << "Real-time"
                << "Fast";

  mTabWidget = new QTabWidget(this);
  mTabWidget->setObjectName("preferenceTabWidget");
  mTabWidget->addTab(createGeneralTab(), tr("General"));
  mTabWidget->addTab(createOpenGLTab(), tr("OpenGL"));
  mTabWidget->addTab(createNetworkTab(), tr("Network"));

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
  if (mPythonCommand)
    mPythonCommand->setText(prefs->value("General/pythonCommand").toString());
  if (mMatlabCommand)
    mMatlabCommand->setText(prefs->value("General/matlabCommand").toString());
  mExtraProjectPath->setText(prefs->value("General/extraProjectPath").toString());
  mTelemetryCheckBox->setChecked(prefs->value("General/telemetry").toBool());
  mCheckWebotsUpdateCheckBox->setChecked(prefs->value("General/checkWebotsUpdateOnStartup").toBool());
  mRenderingCheckBox->setChecked(prefs->value("General/rendering").toBool());
  mDisableSaveWarningCheckBox->setChecked(prefs->value("General/disableSaveWarning").toBool());
  mThumnailCheckBox->setChecked(prefs->value("General/thumbnail").toBool());

  // openGL tab
  mAmbientOcclusionCombo->setCurrentIndex(prefs->value("OpenGL/GTAO", 2).toInt());
  mTextureQualityCombo->setCurrentIndex(prefs->value("OpenGL/textureQuality", 4).toInt());
  mTextureFilteringCombo->setCurrentIndex(prefs->value("OpenGL/textureFiltering", 4).toInt());

  mDisableShadowsCheckBox->setChecked(prefs->value("OpenGL/disableShadows").toBool());
  mDisableAntiAliasingCheckBox->setChecked(prefs->value("OpenGL/disableAntiAliasing").toBool());

  // network tab
  mHttpProxySocks5CheckBox->setChecked(prefs->value("Network/httpProxyType").toInt() == QNetworkProxy::Socks5Proxy);
  mHttpProxyHostName->setText(prefs->value("Network/httpProxyHostName").toString());
  const int port = prefs->value("Network/httpProxyPort", 0).toInt();
  if (port > 0)
    mHttpProxyPort->setText(QString::number(port));
  mHttpProxyUsername->setText(prefs->value("Network/httpProxyUsername").toString());
  mHttpProxyPassword->setText(prefs->value("Network/httpProxyPassword").toString());
  mLanguageCombo->setFocus();

  // robot window
  mNewBrowserWindow->setChecked(prefs->value("RobotWindow/newBrowserWindow").toBool());
  mBrowserProgram->setText(prefs->value("RobotWindow/browser").toString());

  for (int i = 0; i < prefs->value("Network/nAllowedIPs").toInt(); i++) {
    const QString IpKey = "Network/allowedIP" + QString::number(i);
    mAllowedIps->insertItem(i, prefs->value(IpKey).toString());
  }
}

WbPreferencesDialog::~WbPreferencesDialog() {
}

void WbPreferencesDialog::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)
    return;
  QDialog::keyPressEvent(event);
}

void WbPreferencesDialog::accept() {
  WbPreferences *prefs = WbPreferences::instance();

  // Inform when restarting Webots is required
  bool willRestart = false;
  const QString &languageKey = WbTranslator::instance()->findKey(mLanguageCombo->currentText());
  if (languageKey != prefs->value("General/language") ||
      prefs->value("General/theme").toString() != mValidThemeFilenames.at(mThemeCombo->currentIndex()) ||
      prefs->value("General/extraProjectPath").toString() != mExtraProjectPath->text() ||
      prefs->value("OpenGL/disableAntiAliasing").toBool() != mDisableAntiAliasingCheckBox->isChecked()) {
    willRestart = WbMessageBox::question(
                    tr("You have changed some settings which require Webots to be restarted. Restart Webots Now?"), this,
                    tr("Restart Now?"), QMessageBox::Yes, QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
  }
  if (!willRestart && prefs->value("OpenGL/textureQuality", 4).toInt() != mTextureQualityCombo->currentIndex())
    WbMessageBox::info(tr("The new texture quality will be applied next time the world is loaded."), this);
  // Inform the user about possible issues with multi-threading
  if (mNumberOfThreadsCombo->currentIndex() + 1 != mNumberOfThreads && mNumberOfThreadsCombo->currentIndex() != 0)
    WbMessageBox::warning(
      tr("Physics multi-threading is enabled. "
         "This can have a noticeable impact on the simulation speed (negative or positive depending on the simulated world). "
         "In case of multi-threading, simulation replicability is not guaranteed. "),
      this);
  const bool propagate =
    !willRestart && (prefs->value("Editor/font").toString() != mEditorFontEdit->text() ||
                     mNumberOfThreadsCombo->currentIndex() + 1 != mNumberOfThreads ||
                     prefs->value("OpenGL/disableShadows").toBool() != mDisableShadowsCheckBox->isChecked() ||
                     prefs->value("OpenGL/textureFiltering").toInt() != mTextureFilteringCombo->currentIndex() ||
                     prefs->value("OpenGL/GTAO").toInt() != mAmbientOcclusionCombo->currentIndex());
  // general tab
  prefs->setValue("General/startupMode", gStartupModes.value(mStartupModeCombo->currentIndex()));
  prefs->setValue("Editor/font", mEditorFontEdit->text());
  prefs->setValue("General/language", languageKey);
  prefs->setValue("General/theme", mValidThemeFilenames.at(mThemeCombo->currentIndex()));
  prefs->setValue("General/numberOfThreads", mNumberOfThreadsCombo->currentIndex() + 1);
  if (mPythonCommand)
    prefs->setValue("General/pythonCommand", mPythonCommand->text());
  if (mMatlabCommand)
    prefs->setValue("General/matlabCommand", mMatlabCommand->text());
  prefs->setValue("General/extraProjectPath", mExtraProjectPath->text());
  prefs->setValue("General/telemetry", mTelemetryCheckBox->isChecked());
  prefs->setValue("General/checkWebotsUpdateOnStartup", mCheckWebotsUpdateCheckBox->isChecked());
  prefs->setValue("General/rendering", mRenderingCheckBox->isChecked());
  prefs->setValue("General/disableSaveWarning", mDisableSaveWarningCheckBox->isChecked());
  prefs->setValue("General/thumbnail", mThumnailCheckBox->isChecked());

  // openGL
  prefs->setValue("OpenGL/GTAO", mAmbientOcclusionCombo->currentIndex());
  prefs->setValue("OpenGL/textureQuality", mTextureQualityCombo->currentIndex());
  prefs->setValue("OpenGL/textureFiltering", mTextureFilteringCombo->currentIndex());
  prefs->setValue("OpenGL/disableShadows", mDisableShadowsCheckBox->isChecked());
  prefs->setValue("OpenGL/disableAntiAliasing", mDisableAntiAliasingCheckBox->isChecked());

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

  prefs->setValue("Network/cacheSize", mCacheSize->value());

  if (!mUploadUrl->text().isEmpty())
    prefs->setValue("Network/uploadUrl", mUploadUrl->text());
  prefs->setValue("RobotWindow/newBrowserWindow", mNewBrowserWindow->isChecked());
  prefs->setValue("RobotWindow/browser", mBrowserProgram->text());

  prefs->setValue("Network/nAllowedIPs", mAllowedIps->count());
  for (int i = 0; i < mAllowedIps->count(); i++) {
    const QString IpKey = "Network/allowedIP" + QString::number(i);
    prefs->setValue(IpKey, mAllowedIps->item(i)->data(Qt::DisplayRole));
  }

  if (propagate)
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

void WbPreferencesDialog::clearCache() {
  WbNetwork::instance()->clearCache();
  WbMessageBox::info(tr("The cache has been cleared."), this);
  mCacheSizeLabel->setText(tr("Amount of cache used: %1 MB.").arg(WbNetwork::instance()->cacheSize() / (1024 * 1024)));
}

void WbPreferencesDialog::addNewIp() {
  bool ok;
  const QString text =
    QInputDialog::getText(this, tr("Add IP address(es)"), tr("New allowed IP address(es) (<X.X.X.X> or <X.X.X.X>/<netmask>):"),
                          QLineEdit::Normal, tr(""), &ok);
  if (ok && !text.isEmpty())
    mAllowedIps->insertItem(0, text);
}

void WbPreferencesDialog::removeSelectedIp() {
  foreach (const QListWidgetItem *item, mAllowedIps->selectedItems())
    mAllowedIps->takeItem(mAllowedIps->row(item));
}

QWidget *WbPreferencesDialog::createGeneralTab() {
  QWidget *widget = new QWidget(this);
  widget->setObjectName("preferenceWidget");
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
  mExtraProjectPath = new WbLineEdit(this);
  mExtraProjectPath->setToolTip(
    tr("Extra projects may include PROTOs, controllers, plugins, etc. that you can use in your current project."));

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

  mRenderingCheckBox = new QCheckBox(tr("Rendering"), this);
  layout->addWidget(mRenderingCheckBox, 2, 2);

  // row 3
  layout->addWidget(new QLabel(tr("Editor font:"), this), 3, 0);
  layout->addWidget(mEditorFontEdit, 3, 1);
  layout->addWidget(chooseFontButton, 3, 2);

  // row 4
  layout->addWidget(new QLabel(tr("Number of threads:"), this), 4, 0);
  layout->addWidget(mNumberOfThreadsCombo, 4, 1);

  // row 5
  mBrowserProgram = new WbLineEdit(this);
  mBrowserProgram->setObjectName("defaultBrowser");
  mBrowserProgram->setText(WbPreferences::instance()->value("RobotWindow/browser").toString());
  if (WbPreferences::instance()->value("General/theme").toString() != "webots_classic.qss") {
    if (mBrowserProgram->text().isEmpty())
      mBrowserProgram->setStyleSheet("#defaultBrowser { color: gray;}");
    connect(mBrowserProgram, &QLineEdit::textChanged, [=] {
      if (mBrowserProgram->text().isEmpty())
        mBrowserProgram->setStyleSheet("#defaultBrowser { color: gray;}");
      else
        mBrowserProgram->setStyleSheet("#defaultBrowser { color: white;}");
    });
  }
  mBrowserProgram->setMinimumWidth(270);
  layout->addWidget(new QLabel(tr("Default robot window web browser:"), this), 5, 0);
  layout->addWidget(mBrowserProgram, 5, 1);
#ifdef __linux__
  mBrowserProgram->setPlaceholderText(tr("\"firefox\", \"google-chrome\" (default if empty)"));
#elif defined(_WIN32)
  mBrowserProgram->setPlaceholderText(tr("firefox, chrome, or msedge (default if empty)"));
#else  // macOS
  mBrowserProgram->setPlaceholderText(tr("firefox, chrome, or safari (default if empty)"));
#endif

  // row 6
  mNewBrowserWindow = new QCheckBox(tr("Always open in a new window"), this);
  mNewBrowserWindow->setDisabled(mBrowserProgram->text().isEmpty());
  connect(mBrowserProgram, &QLineEdit::textChanged, mNewBrowserWindow, [=]() {
    mNewBrowserWindow->setDisabled(mBrowserProgram->text().isEmpty());
    if (mBrowserProgram->text().isEmpty())
      mNewBrowserWindow->setChecked(false);
  });
  layout->addWidget(mNewBrowserWindow, 6, 1);

  // row 7
  layout->addWidget(new QLabel(tr("Python command:"), this), 7, 0);
  if (WbSysInfo::isSnap()) {
    QLabel *label = new QLabel(
      tr("built-in python (snap), see <a href=\"https://cyberbotics.com/doc/guide/running-extern-robot-controllers\">extern "
         "controllers</a> for alternatives."),
      this);
    layout->addWidget(label, 7, 1);
    connect(label, &QLabel::linkActivated, &WbDesktopServices::openUrl);
    mPythonCommand = NULL;
  } else
    layout->addWidget(mPythonCommand = new WbLineEdit(this), 7, 1);

  // row 8
  layout->addWidget(new QLabel(tr("MATLAB command:"), this), 8, 0);
  layout->addWidget(mMatlabCommand = new WbLineEdit(this), 8, 1);

  // row 9
  layout->addWidget(new QLabel(tr("Extra project path:"), this), 9, 0);
  layout->addWidget(mExtraProjectPath, 9, 1);

  // row 10
  mDisableSaveWarningCheckBox = new QCheckBox(tr("Display save warning only for scene tree edit"), this);
  mDisableSaveWarningCheckBox->setToolTip(
    tr("If this option is enabled, Webots will not display any warning when you quit, reload\nor load a new world after the "
       "current world was modified by either changing the viewpoint,\ndragging, rotating, applying a force or applying a "
       "torque to an object. It will however\nstill display a warning if the world was modified from the scene tree."));
  layout->addWidget(new QLabel(tr("Warnings:"), this), 10, 0);
  layout->addWidget(mDisableSaveWarningCheckBox, 10, 1);

  // row 11
  mThumnailCheckBox = new QCheckBox(tr("Capture thumbnail on world save or share"), this);
  mThumnailCheckBox->setToolTip(tr("If this option is enabled, Webots will take and store a 768px by 432px (16:9)\nscreenshot "
                                   "of the world in .jpg format when the it is saved, shared or exported."));
  layout->addWidget(new QLabel(tr("Thumbnail:"), this), 11, 0);
  layout->addWidget(mThumnailCheckBox, 11, 1);

  // row 12
  mTelemetryCheckBox = new QCheckBox(tr("Send technical data to Webots developers"), this);
  mTelemetryCheckBox->setToolTip(tr("We need your help to continue to improve Webots: more information at:\n"
                                    "https://cyberbotics.com/doc/guide/telemetry"));
  QLabel *label =
    new QLabel(tr("Telemetry (<a style='color: #5DADE2;' href='https://cyberbotics.com/doc/guide/telemetry'>info</a>):"), this);
  connect(label, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  layout->addWidget(label, 12, 0);
  layout->addWidget(mTelemetryCheckBox, 12, 1);

  // row 13
  mCheckWebotsUpdateCheckBox = new QCheckBox(tr("Check for Webots updates on startup"), this);
  mCheckWebotsUpdateCheckBox->setToolTip(tr("If this option is enabled, Webots will check if a new version is available for "
                                            "download\nat every startup. If available, it will inform you about it."));
  layout->addWidget(new QLabel(tr("Update policy:"), this), 13, 0);
  layout->addWidget(mCheckWebotsUpdateCheckBox, 13, 1);

  setTabOrder(mStartupModeCombo, mEditorFontEdit);
  setTabOrder(mEditorFontEdit, chooseFontButton);

  return widget;
}

QWidget *WbPreferencesDialog::createOpenGLTab() {
  QWidget *widget = new QWidget(this);
  widget->setObjectName("preferenceWidget");
  QGridLayout *layout = new QGridLayout(widget);

  // row 0
  mAmbientOcclusionCombo = new QComboBox(this);
  mAmbientOcclusionCombo->addItem(tr("Disabled"));
  mAmbientOcclusionCombo->addItem(tr("Low"));
  mAmbientOcclusionCombo->addItem(tr("Medium"));
  mAmbientOcclusionCombo->addItem(tr("High"));
  mAmbientOcclusionCombo->addItem(tr("Ultra"));
  layout->addWidget(new QLabel(tr("Ambient Occlusion:"), this), 0, 0);
  layout->addWidget(mAmbientOcclusionCombo, 0, 1, 1, -1);

  // row 1
  mTextureQualityCombo = new QComboBox(this);
  mTextureQualityCombo->addItem(tr("Low"));
  mTextureQualityCombo->addItem(tr("Low (anti-aliasing)"));
  mTextureQualityCombo->addItem(tr("Medium"));
  mTextureQualityCombo->addItem(tr("Medium (anti-aliasing)"));
  mTextureQualityCombo->addItem(tr("High"));
  layout->addWidget(new QLabel(tr("Texture Quality:"), this), 1, 0);
  layout->addWidget(mTextureQualityCombo, 1, 1, 1, -1);

  // row 2
  mTextureFilteringCombo = new QComboBox(this);
  for (int i = 0; i < 6; ++i)
    mTextureFilteringCombo->addItem(QString::number(i));
  layout->addWidget(new QLabel(tr("Max Texture Filtering:"), this), 2, 0);
  layout->addWidget(mTextureFilteringCombo, 2, 1, 1, -1);

  // row 3
  layout->addWidget(new QLabel(tr("Options:"), this), 3, 0);
  mDisableShadowsCheckBox = new QCheckBox(tr("Disable shadows"), this);
  layout->addWidget(mDisableShadowsCheckBox, 3, 1);
  mDisableAntiAliasingCheckBox = new QCheckBox(tr("Disable anti-aliasing"), this);
  layout->addWidget(mDisableAntiAliasingCheckBox, 3, 2);

  return widget;
}

QWidget *WbPreferencesDialog::createNetworkTab() {
  QWidget *widget = new QWidget(this);
  widget->setObjectName("preferenceWidget");
  QGridLayout *network = new QGridLayout(widget);
  QGroupBox *proxy = new QGroupBox(tr("Proxy"), this);
  proxy->setObjectName("networkGroupBox");
  proxy->setMaximumHeight(160);
  QGroupBox *upload = new QGroupBox(tr("Web Services"), this);
  upload->setObjectName("networkGroupBox");
  upload->setMaximumHeight(160);
  QGroupBox *cache = new QGroupBox(tr("Disk Cache"), this);
  cache->setObjectName("networkGroupBox");
  cache->setMaximumHeight(160);
  QGroupBox *remoteControllers = new QGroupBox(tr("Remote Extern Controllers"), this);
  remoteControllers->setObjectName("networkGroupBox");
  remoteControllers->setMaximumHeight(160);

  network->addWidget(proxy, 0, 1);
  network->addWidget(upload, 1, 1);
  network->addWidget(cache, 2, 1);
  network->addWidget(remoteControllers, 3, 1);

  // Proxy
  QGridLayout *layout = new QGridLayout(proxy);

  // row 0
  mHttpProxySocks5CheckBox = new QCheckBox(tr("SOCKS v5"), this);
  mHttpProxySocks5CheckBox->setToolTip(tr("Activate SOCK5 proxying."));
  layout->addWidget(new QLabel(tr("Type:"), this), 0, 0);
  layout->addWidget(mHttpProxySocks5CheckBox, 0, 1);

  // row 1
  mHttpProxyHostName = new WbLineEdit(this);
  layout->addWidget(new QLabel(tr("Hostname:"), this), 1, 0);
  layout->addWidget(mHttpProxyHostName, 1, 1);

  // row 2
  mHttpProxyPort = new WbLineEdit(this);
  mHttpProxyPort->setValidator(new QIntValidator(0, 65535));
  layout->addWidget(new QLabel(tr("Port:"), this), 2, 0);
  layout->addWidget(mHttpProxyPort, 2, 1);

  // row 3
  mHttpProxyUsername = new WbLineEdit(this);
  layout->addWidget(new QLabel(tr("Username:"), this), 3, 0);
  layout->addWidget(mHttpProxyUsername, 3, 1);

  // row 4
  mHttpProxyPassword = new WbLineEdit(this);
  mHttpProxyPassword->setEchoMode(QLineEdit::PasswordEchoOnEdit);
  layout->addWidget(new QLabel(tr("Password:"), this), 4, 0);
  layout->addWidget(mHttpProxyPassword, 4, 1);

  // Upload
  layout = new QGridLayout(upload);

  // row 0
  mUploadUrl = new WbLineEdit(this);
  mUploadUrl->setText(WbPreferences::instance()->value("Network/uploadUrl").toString());
  layout->addWidget(new QLabel(tr("Simulation upload service:"), this), 1, 0);
  layout->addWidget(mUploadUrl, 1, 1);

  // Cache
  layout = new QGridLayout(cache);

  // row 0
  mCacheSize = new QSpinBox(this);
  mCacheSize->setRange(0, 65535);
  mCacheSize->setValue(WbPreferences::instance()->value("Network/cacheSize", 1024).toInt());
  connect(mCacheSize, &QSpinBox::editingFinished, [=]() {
    if (mCacheSize->value() < 512) {
      mCacheSize->setValue(512);
      WbMessageBox::info(tr("At least 512 MB of cache are necessary."), this);
    }
  });
  layout->addWidget(new QLabel(tr("Set the size of the cache (in MB):"), this), 0, 0);
  layout->addWidget(mCacheSize, 0, 1);

  // row 1
  QPushButton *clearCacheButton = new QPushButton(QString("Clear the cache"), this);
  connect(clearCacheButton, &QPushButton::pressed, this, &WbPreferencesDialog::clearCache);
  mCacheSizeLabel =
    new QLabel(tr("Amount of cache used: %1 MB.").arg(WbNetwork::instance()->cacheSize() / (1024 * 1024)), this);
  layout->addWidget(mCacheSizeLabel, 1, 0);
  layout->addWidget(clearCacheButton, 1, 1);

  // Remote extern controllers
  layout = new QGridLayout(remoteControllers);

  // row 0
  mAllowedIps = new QListWidget(this);
  mAllowedIps->setMaximumSize(200, 80);
  mAllowedIps->setSelectionMode(QAbstractItemView::ExtendedSelection);

  QPushButton *addIpButton = new QPushButton(QString("+"), this);
  connect(addIpButton, &QPushButton::pressed, this, &WbPreferencesDialog::addNewIp);
  QPushButton *removeIpButton = new QPushButton(QString("-"), this);
  connect(removeIpButton, &QPushButton::pressed, this, &WbPreferencesDialog::removeSelectedIp);

  QGridLayout *buttonsLayout = new QGridLayout();
  buttonsLayout->setSpacing(5);
  buttonsLayout->addWidget(addIpButton, 0, 0);
  buttonsLayout->addWidget(removeIpButton, 1, 0);

  QLabel *allowedIpsLabel = new QLabel(tr("Allowed IPv4 addresses:\n(Leave empty for all hosts)"), this);
  layout->setSpacing(30);
  layout->addWidget(allowedIpsLabel, 0, 0);
  layout->addWidget(mAllowedIps, 0, 1);
  layout->addLayout(buttonsLayout, 0, 2);

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
