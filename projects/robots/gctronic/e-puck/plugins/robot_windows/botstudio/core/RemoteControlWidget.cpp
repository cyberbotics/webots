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
 * Description:  Implementation of the RemoteControlWidget.hpp functions
 */

#include "RemoteControlWidget.hpp"

#include <webots/remote_control.h>
#include <webots/robot.h>

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QPushButton>

#include <vector>

using namespace std;

// constructor
RemoteControlWidget::RemoteControlWidget(QWidget *parent) : QWidget(parent) {
  // create and set the combo box into this widget
  mLayout = new QGridLayout();
  mPortsComboBox = new QComboBox();
  mRefreshPortsButton = new QPushButton(tr("Refresh ports"));
  populatePorts();
  mLayout->addWidget(mPortsComboBox, 0, 0);
  mLayout->addWidget(mRefreshPortsButton, 0, 1);
  setLayout(mLayout);

  // connect the signals with their respective slots
  connect(mPortsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(portsUpdated(int)));
  connect(mRefreshPortsButton, SIGNAL(clicked()), this, SLOT(populatePorts()));
}

// destructor
RemoteControlWidget::~RemoteControlWidget() {
  delete mPortsComboBox;
  delete mRefreshPortsButton;
  delete mLayout;
}

// populate the ports combo box
void RemoteControlWidget::populatePorts() {
  mPortsComboBox->blockSignals(true);
  mRefreshPortsButton->blockSignals(true);

  mPorts.clear();
  mPortsComboBox->clear();

  mPortsComboBox->addItem(tr("Simulation"));
  mPortsComboBox->setCurrentIndex(0);

  char *p = static_cast<char *>(wb_remote_control_custom_function(NULL));
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
