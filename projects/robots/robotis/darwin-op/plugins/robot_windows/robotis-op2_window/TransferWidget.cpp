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

#include "TransferWidget.hpp"

#include "SSH.hpp"
#include "ZIP.hpp"

#include <webots/camera.h>
#include <webots/robot.h>
#include <core/StandardPaths.hpp>

#include <QtConcurrent/QtConcurrent>
#include <QtCore/QSettings>
#include <QtGui/QIcon>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QProgressDialog>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <cassert>
#include <iostream>

#ifdef _WIN32
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#else
// dummy function used to know the full path
// of the current library
extern "C" {
static void foo() {
}
}
#endif

using namespace webotsQtUtils;
using namespace std;

static RobotisModel findActualRobotisModel(RobotisModel defaultValue) {
  const char *modelStr = wb_robot_get_model();
  if (strcmp("DARwIn-OP", modelStr) == 0)
    return DARWIN_OP;
  if (strcmp("ROBOTIS OP2", modelStr) == 0)
    return ROBOTIS_OP2;
  return defaultValue;
}

TransferWidget::TransferWidget(QWidget *parent) : QWidget(parent), mActualModel(findActualRobotisModel(ROBOTIS_OP2)) {
  mContainerGridLayout = new QGridLayout(this);

  //***  SETTINGS  ***//
  mSettingsGridLayout = new QGridLayout();
  mSettingsGroupBox = new QGroupBox(tr("Connection settings"));

  // IP
  mIPAddressLineEdit = new QLineEdit(this);
  mIPAddressLineEdit->setToolTip(
    tr("IP address or name of the robot, the robot must be on the same network that the computer.\nIf the robot is connected "
       "with an Ethernet cable, the default address is 192.168.123.1\nIf the robot is connected by Wifi, run (on the robot) "
       "the 'ifconfig' command to display the IP address"));
  mIPLabel = new QLabel(tr("IP address:"));
  mSettingsGridLayout->addWidget(mIPLabel, 0, 0, 1, 1);
  mSettingsGridLayout->addWidget(mIPAddressLineEdit, 0, 1, 1, 1);

  // Username
  mUsernameLineEdit = new QLineEdit(this);
  mUsernameLineEdit->setToolTip(tr(QString("The default username is '%1'").arg(defaultUsername()).toLatin1().data()));
  mUsernameLabel = new QLabel(tr("Username:"));
  mSettingsGridLayout->addWidget(mUsernameLabel, 1, 0, 1, 1);
  mSettingsGridLayout->addWidget(mUsernameLineEdit, 1, 1, 1, 1);

  // Password
  mPasswordLineEdit = new QLineEdit(this);
  mPasswordLineEdit->setToolTip(tr("The default password is '111111'"));
  mPasswordLabel = new QLabel(tr("Password:"));
  mSettingsGridLayout->addWidget(mPasswordLabel, 2, 0, 1, 1);
  mSettingsGridLayout->addWidget(mPasswordLineEdit, 2, 1, 1, 1);

  // Restore settings button
  mDefaultSettingsButton = new QPushButton(tr("&Restore defaults settings"), this);
  mDefaultSettingsButton->setToolTip(tr("Restore defaults settings for the connection with the real robot"));
  mSettingsGridLayout->addWidget(mDefaultSettingsButton, 3, 0, 1, 2);

  // Load settings
  mSettings = new QSettings("Cyberbotics", "Webots", this);
  loadSettings();

  mSettingsGroupBox->setLayout(mSettingsGridLayout);
  mSettingsGroupBox->setToolTip(tr("Specify the parameters of the connection with the robot"));
  mContainerGridLayout->addWidget(mSettingsGroupBox, 0, 0, 1, 1);

  //***  Upload controller  ***//
  mActionGridLayout = new QGridLayout();
  mActionGroupBox = new QGroupBox(tr("Upload controller"), this);

  // Send Controller
  QString iconsPath = getCurrentLibraryPath() + "/images/";
  mSendControllerIcon = new QIcon(QPixmap(iconsPath + "send" + resourceNameSuffix() + ".png"));
  mStopControllerIcon = new QIcon(QPixmap(iconsPath + "stop" + resourceNameSuffix() + ".png"));
  mSendControllerButton = new QPushButton(this);
  mSendControllerButton->setIconSize(QSize(64, 64));
  mActionGridLayout->addWidget(mSendControllerButton, 0, 0, 1, 1);

  // Make default controller
  mMakeDefaultControllerCheckBox = new QCheckBox(tr("Make default controller"), this);
  mMakeDefaultControllerCheckBox->setToolTip(tr("Set this controller to be the default controller of the robot"));
  mActionGridLayout->addWidget(mMakeDefaultControllerCheckBox, 1, 0, 1, 2);

  // remote control
  mSSH = NULL;
  mRemoteEnable = false;
  mRemoteControlIcon = new QIcon(QPixmap(iconsPath + "remote" + resourceNameSuffix() + ".png"));
  mRemoteControlButton = new QPushButton(this);
  mRemoteControlButton->setIconSize(QSize(64, 64));
  mActionGridLayout->addWidget(mRemoteControlButton, 0, 1, 1, 1);

  // Wrapper
  mUninstallButton = new QPushButton(this);
  mUninstallButton->setIcon(QIcon(QPixmap(iconsPath + "uninstall" + resourceNameSuffix() + ".png")));
  mUninstallButton->setIconSize(QSize(64, 64));
  mUninstallButton->setToolTip(tr("If you don't need it any more, you can uninstall Webots API from the real robot"));
  mActionGridLayout->addWidget(mUninstallButton, 0, 2, 1, 1);

  mActionGroupBox->setLayout(mActionGridLayout);
  mContainerGridLayout->addWidget(mActionGroupBox, 0, 1, 1, 1);

  //***  OUTPUT  ***//

  mOutputGridLayout = new QGridLayout();
  mOutputGroupBox = new QGroupBox(tr("Robot console"), this);

  // Status label and progress bar
  mStatusLabel = new QLabel(this);
  mOutputGridLayout->addWidget(mStatusLabel, 1, 0, 1, 1);
  mProgressBar = new QProgressBar(this);
  mProgressBar->setMinimum(0);  // min = max = 0 => busy indicator
  mProgressBar->setMaximum(0);
  mProgressBar->hide();
  mOutputGridLayout->addWidget(mProgressBar, 3, 0, 1, 2);

  // Console Show
  mConsoleShowTextEdit = new QTextEdit(this);
  mConsoleShowTextEdit->setReadOnly(true);
  mConsoleShowTextEdit->setOverwriteMode(false);
  mOutputGridLayout->addWidget(mConsoleShowTextEdit, 0, 0, 1, 2);

  mOutputGroupBox->setLayout(mOutputGridLayout);
  mContainerGridLayout->addWidget(mOutputGroupBox, 1, 0, 1, 2);

  mConnectionState = false;

  // Signals TransferWidget->TransferWidget
  QObject::connect(mSendControllerButton, SIGNAL(clicked()), this, SLOT(sendController()));
  QObject::connect(mRemoteControlButton, SIGNAL(clicked()), this, SLOT(remoteControl()));
  QObject::connect(mUninstallButton, SIGNAL(clicked()), this, SLOT(uninstall()));
  QObject::connect(mDefaultSettingsButton, SIGNAL(clicked()), this, SLOT(restoreSettings()));
  QObject::connect(mMakeDefaultControllerCheckBox, SIGNAL(stateChanged(int)), this, SLOT(makeDefaultControllerWarning(int)));

  mSSH = new SSH(this, mActualModel);
  mSSH->connect(mSSH, SIGNAL(print(const QString &, bool)), this, SLOT(print(const QString &, bool)), Qt::QueuedConnection);
  mSSH->connect(mSSH, SIGNAL(status(const QString &)), this, SLOT(status(const QString &)), Qt::QueuedConnection);
  mSSH->connect(mSSH, SIGNAL(done()), this, SLOT(SSHSessionDone()));
  connect(&mFutureWatcher, SIGNAL(finished()), this, SLOT(SSHSessionComplete()));

  enableButtons();
}

TransferWidget::~TransferWidget() {
  delete mSSH;
}

void TransferWidget::showProgressBox(const QString &title, const QString &message) {
  disableButtons();
  mRemoteProgressDialog =
    new QProgressDialog(title, QString(), 0, 100, this, Qt::Dialog | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
  mRemoteProgressDialog->setWindowTitle(title);
  mRemoteProgressDialog->setLabelText(tr("%1...").arg(message) + "\n\n" + tr("Please wait, it can take a few seconds."));
  mRemoteProgressBar = new QProgressBar(mRemoteProgressDialog);
  mRemoteProgressBar->setTextVisible(false);
  mRemoteProgressBar->setMinimum(0);  // min = max = 0 => busy indicator
  mRemoteProgressBar->setMaximum(0);
  mRemoteProgressDialog->setBar(mRemoteProgressBar);
  mRemoteProgressDialog->show();
}

void TransferWidget::print(const QString &message, bool error) {
  static QString str_out;
  static QString str_err;

  if (error)
    str_err += message;
  else
    str_out += message;

  do {
    int n = str_out.indexOf("\n");
    if (n == -1)
      break;
    QString pr = str_out.left(n);
    mConsoleShowTextEdit->setTextColor(QColor(0, 0, 0));
    mConsoleShowTextEdit->append(pr);
    mConsoleShowTextEdit->moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);
    str_out = str_out.mid(n + 1);
  } while (!str_out.isEmpty());

  do {
    int n = str_err.indexOf("\n");
    if (n == -1)
      break;
    QString pr = str_err.left(n);
    mConsoleShowTextEdit->setTextColor(QColor(255, 0, 0));
    mConsoleShowTextEdit->append(pr);
    mConsoleShowTextEdit->moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);
    str_err = str_err.mid(n + 1);
  } while (!str_err.isEmpty());
}

void TransferWidget::status(const QString &message) {
  mStatusLabel->setText(tr("Status: %1...").arg(message));
  if (mRemoteProgressDialog)
    mRemoteProgressDialog->setLabelText(tr("%1...").arg(message) + "\n\n" + tr("Please wait, it can take a few seconds."));
}

void TransferWidget::sendController() {
  if (mStatus == DISCONNECTED) {  // send controller
    mRemoteControlButton->setEnabled(false);
    mUninstallButton->setEnabled(false);
    mSendControllerButton->setEnabled(true);
    mMakeDefaultControllerCheckBox->setEnabled(false);
    showProgressBox(tr("Starting remote compilation..."), tr("Initializing"));
    mFuture = QtConcurrent::run(mSSH, &SSH::startRemoteCompilation, mIPAddressLineEdit->text(), mUsernameLineEdit->text(),
                                mPasswordLineEdit->text(), mMakeDefaultControllerCheckBox->isChecked());
    mFutureWatcher.setFuture(mFuture);
    mStatus = START_REMOTE_COMPILATION;
    mStatusLabel->setText(tr("Status: Starting remote compilation"));
  } else {  // stop controller
    assert(mStatus == RUN_REMOTE_COMPILATION);
    showProgressBox(tr("Stopping remote controller..."), tr("Initializing"));
    mSSH->terminate();
    mStatus = STOP_REMOTE_COMPILATION;
    mStatusLabel->setText(tr("Status: Stopping remote controller"));
  }
}

void TransferWidget::remoteControl() {
  if (mStatus == DISCONNECTED) {  // start the remote control
    showProgressBox(tr("Starting remote control..."), tr("Initializing"));
    static QString ip = mIPAddressLineEdit->text();
    mFuture = QtConcurrent::run(mSSH, &SSH::startRemoteControl, ip, mUsernameLineEdit->text(), mPasswordLineEdit->text());
    mFutureWatcher.setFuture(mFuture);
    mStatus = START_REMOTE_CONTROL;
  } else if (mStatus == RUN_REMOTE_CONTROL) {  // stop the remote control
    showProgressBox(tr("Stopping remote control..."), tr("Initializing"));
    wb_robot_set_mode(WB_MODE_SIMULATION, NULL);
    mSSH->terminate();
    mStatus = STOP_REMOTE_CONTROL;
    mStatusLabel->setText(tr("Status: Stopping remote control"));
  }
}

void TransferWidget::uninstall() {
  QMessageBox msgBox;
  msgBox.setWindowTitle(tr("Webots API uninstallation"));
  msgBox.setText(tr("You are going to completely uninstall Webots API from the robot"));
  msgBox.setInformativeText(tr("Are you sure this is what do you want to do?"));
  msgBox.setStandardButtons(QMessageBox ::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::No);
  msgBox.setIcon(QMessageBox::Warning);
  msgBox.setWindowFlags(Qt::WindowStaysOnTopHint);
  msgBox.setDetailedText(
    tr("All the Webots API and controllers will be uninstalled, the original demo program will be restored"));
  int warning = msgBox.exec();
  if (warning == QMessageBox::Yes) {
    showProgressBox(tr("Uninstall"), tr("Uninstalling the Webots API"));
    mFuture = QtConcurrent::run(mSSH, &SSH::uninstall, mIPAddressLineEdit->text(), mUsernameLineEdit->text(),
                                mPasswordLineEdit->text());
    mFutureWatcher.setFuture(mFuture);
    mStatus = UNINSTALL;
  }
}

void TransferWidget::finishStartRemoteCompilation() {
  delete mRemoteProgressDialog;
  mRemoteProgressDialog = NULL;
  mSendControllerButton->setIcon(*mStopControllerIcon);
  mSendControllerButton->setToolTip(tr("Stop the controller on the real robot"));
  mSendControllerButton->setEnabled(true);
  mRemoteControlButton->setEnabled(false);
  mUninstallButton->setEnabled(false);
  mMakeDefaultControllerCheckBox->setEnabled(false);
  wb_robot_set_mode(WB_MODE_SIMULATION, mIPAddressLineEdit->text().toStdString().c_str());
  mStatus = RUN_REMOTE_COMPILATION;
  mStatusLabel->setText(tr("Status: Running remote controller"));
}

void TransferWidget::finishStartRemoteControl() {
  delete mRemoteProgressDialog;
  mRemoteProgressDialog = NULL;
  mRemoteControlButton->setIcon(*mStopControllerIcon);
  mRemoteControlButton->setToolTip(tr("Stop remote control"));
  mRemoteControlButton->setEnabled(true);
  mUninstallButton->setEnabled(false);
  mSendControllerButton->setEnabled(false);
  mMakeDefaultControllerCheckBox->setEnabled(false);
  wb_robot_set_mode(WB_MODE_REMOTE_CONTROL, mIPAddressLineEdit->text().toStdString().c_str());
  mStatus = RUN_REMOTE_CONTROL;
  mStatusLabel->setText(tr("Status: Running remote control"));
}

void TransferWidget::finish() {
  delete mRemoteProgressDialog;
  mRemoteProgressDialog = NULL;
  enableButtons();
}

void TransferWidget::SSHSessionDone() {
  saveSettings();  // connection was successful, so we want to save the network settings
  switch (mStatus) {
    case START_REMOTE_COMPILATION:
      finishStartRemoteCompilation();
      break;
    case START_REMOTE_CONTROL:
      finishStartRemoteControl();
      break;
    default:
      assert(false);
      break;
  }
}

void TransferWidget::SSHSessionComplete() {
  if (mFutureWatcher.result() < 0) {
    delete mRemoteProgressDialog;
    mRemoteProgressDialog = NULL;
    mStatusLabel->setText(mSSH->error());
    QMessageBox msgBox;
    msgBox.setWindowTitle(tr("Connection failed"));
    msgBox.setText(tr("Unable to establish connection with the robot."));
    msgBox.setInformativeText(mSSH->error());
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.setWindowFlags(Qt::WindowStaysOnTopHint);
    msgBox.exec();
    enableButtons();
  } else {
    saveSettings();  // connection was successful, so we want to save the network settings
    switch (mStatus) {
      case STOP_REMOTE_CONTROL:
      case STOP_REMOTE_COMPILATION:
      case UNINSTALL:
      case RUN_REMOTE_COMPILATION:  // the remote controller crashed or could not start
      case RUN_REMOTE_CONTROL:      // this one should not happen
        finish();
        break;
      case START_REMOTE_COMPILATION:  // the robot was not in a stable position
      {
        finish();
        if (!mSSH->error().isEmpty())
          // we don't want to display the following message box in case of a compilation error
          break;
        QMessageBox msgBox;
        msgBox.setWindowTitle(tr("Unstable position"));
        msgBox.setText(tr("The robot is not in ready position."));
        msgBox.setInformativeText(tr("Please set the robot in ready position (refer to robot documentation) and retry."));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setWindowFlags(Qt::WindowStaysOnTopHint);
        msgBox.exec();
      } break;
      default:
        cerr << "Unknown status: " << mStatus << endl;
        break;
    }
  }
}

// ---------------------------------------------------------------------------- //
// ***                        Auxiliary functions                           *** //
// ---------------------------------------------------------------------------- //

void TransferWidget::restoreSettings() {
  mIPAddressLineEdit->setText("192.168.123.1");
  mUsernameLineEdit->setText(defaultUsername());
  mPasswordLineEdit->setText("111111");
}

void TransferWidget::saveSettings() {
  mSettings->setValue("robotis-op2_window/IP", mIPAddressLineEdit->text());
  mSettings->setValue("robotis-op2_window/username", mUsernameLineEdit->text());
  mSettings->setValue("robotis-op2_window/password", mPasswordLineEdit->text());
}

void TransferWidget::loadSettings() {
  mIPAddressLineEdit->setText(mSettings->value("robotis-op2_window/IP", "192.168.123.1").toString());
  mUsernameLineEdit->setText(mSettings->value("robotis-op2_window/username", defaultUsername()).toString());
  mPasswordLineEdit->setText(mSettings->value("robotis-op2_window/password", "111111").toString());
}

const char *TransferWidget::defaultUsername() const {
  if (mActualModel == DARWIN_OP)
    return "darwin";
  if (mActualModel == ROBOTIS_OP2)
    return "robotis";
  assert(false);  // should not happen
  return NULL;
}

const char *TransferWidget::resourceNameSuffix() const {
  if (mActualModel == DARWIN_OP)
    return "";
  if (mActualModel == ROBOTIS_OP2)
    return "_op2";
  assert(false);  // should not happen
  return NULL;
}

void TransferWidget::enableButtons() {
  mSendControllerButton->setIcon(*mSendControllerIcon);
  mSendControllerButton->setToolTip(tr("Send the controller curently used in simulation on the real robot and play it"));
  mRemoteControlButton->setIcon(*mRemoteControlIcon);
  mRemoteControlButton->setToolTip(tr("Start remote control"));
  mUninstallButton->setEnabled(true);
  mSendControllerButton->setEnabled(true);
  mMakeDefaultControllerCheckBox->setEnabled(true);
  mRemoteControlButton->setEnabled(true);
  mStatusLabel->setText(tr("Status: Disconnected"));
  mStatus = DISCONNECTED;
}

void TransferWidget::disableButtons() {
  mUninstallButton->setEnabled(false);
  mSendControllerButton->setEnabled(false);
  mMakeDefaultControllerCheckBox->setEnabled(false);
  mRemoteControlButton->setEnabled(false);
}

void TransferWidget::makeDefaultControllerWarning(int s) {
  if (s) {
    QMessageBox msgBox;
    msgBox.setWindowTitle("Default controller installation");
    msgBox.setText("Checking this box will install the current controller as a default controller on the robot.");
    msgBox.setInformativeText("After upload, the controller will run automatically when the robot starts.\n\n"
                              "Warning: the initial position of the robot will not be checked, so make sure that the robot is "
                              "at the correct initial position before starting it.");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setWindowFlags(Qt::WindowStaysOnTopHint);
    msgBox.exec();
  }
}

QString TransferWidget::getCurrentLibraryPath() {
  static bool defined = false;
  static QString path;

  if (!defined) {
#ifdef _WIN32
    WCHAR buffer[MAX_PATH] = {0};
    GetModuleFileNameW((HINSTANCE)&__ImageBase, buffer, sizeof(buffer));
    path = QString::fromWCharArray(buffer);
    path = path.replace('\\', '/');
#else
    Dl_info dl_info;
    dladdr((void *)foo, &dl_info);
    path = dl_info.dli_fname;
#endif
    path = path.mid(0, path.lastIndexOf('/') + 1);
    assert(!path.isEmpty());
    defined = true;
  }

  return path;
}
