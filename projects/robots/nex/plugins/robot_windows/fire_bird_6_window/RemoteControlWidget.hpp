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
 * Description:   Class defining the remote control interface group box
 */

#ifndef REMOTE_CONTROL_GROUP_BOX_HPP
#define REMOTE_CONTROL_GROUP_BOX_HPP

#include <QtCore/QFutureWatcher>
#include <QtWidgets/QWidget>

class QComboBox;
class QPushButton;
class QGridLayout;
class QProgressDialog;
class UploaderData;

class RemoteControlWidget : public QWidget {
  Q_OBJECT

public:
  static RemoteControlWidget *instance();
  static void clear();
  static void setParentInstance(QWidget *parent) { cParent = parent; }
  void updateValues();

public slots:
  void portsUpdated(int index);
  void populatePorts();
  void enableAllSensors();

signals:
  void progressChanged(int type, int newValue);

private:
  static RemoteControlWidget *cInstance;
  static QWidget *cParent;

  explicit RemoteControlWidget(QWidget *parent = NULL);
  ~RemoteControlWidget();

  QGridLayout *mLayout;
  QComboBox *mPortsComboBox;
  QPushButton *mRefreshPortsButton;
  QPushButton *mEnableButton;
  QStringList mPorts;
  QString mPortName;
  QString mHexFileName;
  QProgressDialog *mProgressDialog;
  QProgressDialog *mPressButtonDialog;
  QProgressDialog *mConnectProgressDialog;
  QFutureWatcher<int> *mConnectFutureWatcher;
  QFutureWatcher<int> *mUploadFutureWatcher;
};

#endif
