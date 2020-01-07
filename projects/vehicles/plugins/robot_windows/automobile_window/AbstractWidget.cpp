// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "AbstractWidget.hpp"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

AbstractWidget::AbstractWidget(QWidget *parent) : QWidget(parent) {
  mLayout = new QGridLayout(this);

  mEnableCheckBox = new QCheckBox("Disabled", this);
  connect(mEnableCheckBox, &QCheckBox::stateChanged, this, &AbstractWidget::updateEnableCheckBoxText);
  mLayout->addWidget(mEnableCheckBox, 0, 0);

  mValueLabel = new QLabel("", this);
  mLayout->addWidget(mValueLabel, 0, 1, Qt::AlignRight);

  mGraph = new Graph2D(this);
  mGraph->setYRange(0, 0);
  mLayout->addWidget(mGraph, 1, 0, 1, 2);
}

AbstractWidget::~AbstractWidget() {
}

void AbstractWidget::updateEnableCheckBoxText() {
  if (mEnableCheckBox->isChecked())
    mEnableCheckBox->setText("Enabled");
  else {
    mEnableCheckBox->setText("Disabled");
    mValueLabel->setText("");
  }
}
