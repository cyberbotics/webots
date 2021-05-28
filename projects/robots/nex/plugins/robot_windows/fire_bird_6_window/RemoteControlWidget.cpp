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

#include "RemoteControlWidget.hpp"
#include "FireBird6Representation.hpp"

#include <webots/remote_control.h>
#include <webots/robot.h>

#include <QtConcurrent/QtConcurrent>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QProgressDialog>
#include <QtWidgets/QPushButton>

#include <vector>

using namespace std;

RemoteControlWidget *RemoteControlWidget::cInstance = NULL;
QWidget *RemoteControlWidget::cParent = NULL;

RemoteControlWidget *RemoteControlWidget::instance() {
  if (!cInstance)
    cInstance = new RemoteControlWidget(cParent);

  return cInstance;
}

void RemoteControlWidget::clear() {
  delete cInstance;
  cInstance = NULL;
}

// constructor
RemoteControlWidget::RemoteControlWidget(QWidget *parent) :
  QWidget(parent),
  mHexFileName(""),
  mProgressDialog(NULL),
  mPressButtonDialog(NULL),
  mConnectProgressDialog(NULL) {
  mConnectFutureWatcher = new QFutureWatcher<int>(this);
  mUploadFutureWatcher = new QFutureWatcher<int>(this);

  // create and set the combo box into this widget
  mLayout = new QGridLayout();
  mPortsComboBox = new QComboBox();
  mRefreshPortsButton = new QPushButton("Refresh ports");
  populatePorts();
  mEnableButton = new QPushButton("Enable all");
  mEnableButton->setToolTip("Enable all the sensors");

  mLayout->addWidget(mPortsComboBox, 0, 0);
  mLayout->addWidget(mRefreshPortsButton, 0, 1);
  mLayout->addWidget(mEnableButton, 1, 1);
  setLayout(mLayout);

  // connect the signals with their respective slots
  connect(mPortsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(portsUpdated(int)));
  connect(mRefreshPortsButton, SIGNAL(clicked()), this, SLOT(populatePorts()));
  connect(mEnableButton, SIGNAL(clicked()), this, SLOT(enableAllSensors()));
}

// destructor
RemoteControlWidget::~RemoteControlWidget() {
  delete mPortsComboBox;
  delete mRefreshPortsButton;
  delete mLayout;
  delete mConnectFutureWatcher;
  delete mUploadFutureWatcher;
  delete mConnectProgressDialog;
  delete mProgressDialog;
  delete mPressButtonDialog;
}

// populate the ports combo box
void RemoteControlWidget::populatePorts() {
  mPortsComboBox->blockSignals(true);
  mRefreshPortsButton->blockSignals(true);

  mPorts.clear();
  mPortsComboBox->clear();

  mPortsComboBox->addItem(tr("Simulation"));
  mPortsComboBox->setCurrentIndex(0);

  char *p = (char *)wb_remote_control_custom_function(NULL);
  QStringList ports = QString::fromUtf8(p).split('\n');
  free(p);
  for (int i = 0; i < ports.size(); ++i) {
    mPorts << ports[i];
    mPortsComboBox->addItem(ports[i]);
  }

  mPortsComboBox->blockSignals(false);
  mRefreshPortsButton->blockSignals(false);
}

// update
void RemoteControlWidget::updateValues() {
  // check that the remote control is still active
  if (wb_robot_get_mode() == 0 && mPortsComboBox->currentIndex() != 0)
    mPortsComboBox->setCurrentIndex(0);
}

// handle the combo box selection
void RemoteControlWidget::portsUpdated(int index) {
  if (index == 0)
    wb_robot_set_mode(WB_MODE_SIMULATION, NULL);
  else
    wb_robot_set_mode(WB_MODE_REMOTE_CONTROL, mPortsComboBox->currentText().toStdString().c_str());
}

void RemoteControlWidget::enableAllSensors() {
  FireBird6Representation::instance()->enableAllSensors();
  mEnableButton->setEnabled(false);
}
