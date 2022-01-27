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

/*
 * Description:  Widget for communicating with DARwIn-OP and
 ROBOTIS OP2 robots.
 */

#ifndef TRANSFER_WIDGET_HPP
#define TRANSFER_WIDGET_HPP

#include <QtCore/QFuture>
#include <QtCore/QFutureWatcher>
#include <QtWidgets/QWidget>

class QSettings;
class QIcon;
class QCheckBox;
class QGridLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QProgressDialog;
class QPushButton;
class QTextEdit;

#include "common.hpp"

class SSH;

class TransferWidget : public QWidget {
  Q_OBJECT

public:
  enum Status {
    DISCONNECTED,
    START_REMOTE_CONTROL,
    RUN_REMOTE_CONTROL,
    STOP_REMOTE_CONTROL,
    UNINSTALL,
    START_REMOTE_COMPILATION,
    RUN_REMOTE_COMPILATION,
    STOP_REMOTE_COMPILATION
  };

  explicit TransferWidget(QWidget *parent = 0);
  virtual ~TransferWidget();

  // void robotInstableSlot();  // TODO: this function is never call: when has it been unused?

public slots:
  void makeDefaultControllerWarning(int s);
  void restoreSettings();
  void uninstall();
  void sendController();
  void remoteControl();
  void SSHSessionComplete();
  void SSHSessionDone();
  void print(const QString &message, bool error);
  void status(const QString &message);

  /*
  signals:
    void setStabilityResponseSignal(int stability);
    void isRobotStableSignal();
  */

private:
  void saveSettings();
  void enableButtons();
  void disableButtons();

  void showProgressBox(const QString &title, const QString &message);
  void finishStartRemoteCompilation();
  void finishStartRemoteControl();
  void finish();
  void loadSettings();

  const char *defaultUsername() const;
  const char *resourceNameSuffix() const;

  // reimplement StandardPaths::getCurrentLibraryPath to retrieve path of this robot window library
  QString getCurrentLibraryPath();

  QFutureWatcher<int> mFutureWatcher;
  QFuture<int> mFuture;

  Status mStatus;

  //***  SSH  ***//
  SSH *mSSH;
  bool mConnectionState;

  QGridLayout *mContainerGridLayout;

  //***  SETTINGS  ***//
  QGridLayout *mSettingsGridLayout;
  QGroupBox *mSettingsGroupBox;

  // IP adresse
  QLineEdit *mIPAddressLineEdit;
  QLabel *mIPLabel;

  // Username
  QLineEdit *mUsernameLineEdit;
  QLabel *mUsernameLabel;

  // Password
  QLineEdit *mPasswordLineEdit;
  QLabel *mPasswordLabel;

  // Restore
  QPushButton *mDefaultSettingsButton;
  QSettings *mSettings;

  //***  Upload controller  ***//
  QGridLayout *mActionGridLayout;
  QGroupBox *mActionGroupBox;

  // Controller
  QIcon *mSendControllerIcon;
  QIcon *mStopControllerIcon;
  QPushButton *mSendControllerButton;
  QCheckBox *mMakeDefaultControllerCheckBox;

  // remote control
  QIcon *mRemoteControlIcon;
  QPushButton *mRemoteControlButton;
  bool mRemoteEnable;

  // Remote control wait during starting
  QProgressDialog *mRemoteProgressDialog;
  QProgressBar *mRemoteProgressBar;

  // Wrapper
  QPushButton *mUninstallButton;

  //***  OUTPUT  ***//
  QGridLayout *mOutputGridLayout;
  QGroupBox *mOutputGroupBox;

  // Status label and progress bar
  QLabel *mStatusLabel;
  QProgressBar *mProgressBar;

  // Console Show
  QTextEdit *mConsoleShowTextEdit;

  /** Actual Robot node Darwin model */
  RobotisModel mActualModel;
};

#endif
