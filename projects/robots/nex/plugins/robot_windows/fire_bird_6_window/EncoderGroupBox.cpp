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

#include "EncoderGroupBox.hpp"
#include "FireBird6Representation.hpp"

#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

// prefixes of the mLabels
static const QString prefixes[2] = {QString("right = "), QString("left = ")};

// constructor
EncoderGroupBox::EncoderGroupBox(QWidget *parent) : QGroupBox(parent) {
  // set the mLabels into this widget
  mVBox = new QVBoxLayout;
  for (int i = 0; i < 2; i++) {
    mLabels[i] = new QLabel;
    mLabels[i]->setObjectName("blueLabel");
    mVBox->addWidget(mLabels[i]);
  }
  setLayout(mVBox);

  // set the title
  setTitle("Encoders");
}

// destructor
EncoderGroupBox::~EncoderGroupBox() {
  delete mVBox;
  for (int i = 0; i < 2; i++)
    delete mLabels[i];
}

// update the mLabels
void EncoderGroupBox::updateValues() {
  FireBird6Representation *fireBird6 = FireBird6Representation::instance();

  bool enable = fireBird6->areEncodersEnabled();
  setEnabled(enable);
  if (!enable)
    return;

  double encoders[] = {fireBird6->leftEncoderValue(), fireBird6->rightEncoderValue()};
  for (int i = 0; i < 2; i++)
    mLabels[i]->setText(prefixes[i] + QString::number((int)encoders[i]));
}
