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

#include "MagnetometerGroupBox.hpp"
#include "FireBird6Representation.hpp"

#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

// components
enum { X, Y, Z };

// defining the prefixes and the colors of the tree mLabels
static const QString prefixes[3] = {QString("x = "), QString("y = "), QString("z = ")};
static const QString colorLabels[4] = {QString("redLabel"), QString("greenLabel"), QString("blueLabel"), QString("blackLabel")};

// constructor
MagnetometerGroupBox::MagnetometerGroupBox(QWidget *parent) : QGroupBox(parent) {
  // create and set the mLabels
  mVBox = new QVBoxLayout;
  for (int i = 0; i < 3; i++) {
    mLabels[i] = new QLabel;
    mLabels[i]->setObjectName(colorLabels[i]);
    mVBox->addWidget(mLabels[i]);
  }
  setLayout(mVBox);

  // set the title
  setTitle("Magnetometer");
}

// destructor
MagnetometerGroupBox::~MagnetometerGroupBox() {
  delete mVBox;
  for (int i = 0; i < 3; i++)
    delete mLabels[i];
}

// update
void MagnetometerGroupBox::updateValues() {
  FireBird6Representation *firebird6 = FireBird6Representation::instance();

  bool enable = firebird6->isAccelerometerEnabled();
  setEnabled(enable);
  if (!enable)
    return;

  const double *magValuesXY = firebird6->magnetometerXYValues();
  const double *magValuesZ = firebird6->magnetometerZValues();
  if (magValuesXY && magValuesZ) {
    for (int i = 0; i < 2; i++)
      mLabels[i]->setText(prefixes[i] + QString::number(magValuesXY[i], 'f', 4));

    mLabels[2]->setText(prefixes[2] + QString::number(magValuesZ[2], 'f', 4));

  } else {
    mLabels[X]->setText("Disabled");
    mLabels[Y]->setText("");
    mLabels[Z]->setText("");
  }
}
