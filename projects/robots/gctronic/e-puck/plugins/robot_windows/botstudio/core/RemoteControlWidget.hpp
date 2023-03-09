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

/*
 * Description:  Class defining the remote control interface group box
 */

#ifndef REMOTE_CONTROL_GROUP_BOX_HPP
#define REMOTE_CONTROL_GROUP_BOX_HPP

#include <QtWidgets/QWidget>

class QComboBox;
class QPushButton;
class QGridLayout;

class RemoteControlWidget : public QWidget {
  Q_OBJECT

public:
  explicit RemoteControlWidget(QWidget *parent = NULL);
  virtual ~RemoteControlWidget();
  void updateValues();

public slots:
  void portsUpdated(int index);
  void populatePorts();

private:
  QGridLayout *mLayout;
  QComboBox *mPortsComboBox;
  QPushButton *mRefreshPortsButton;
  QStringList mPorts;
};

#endif
