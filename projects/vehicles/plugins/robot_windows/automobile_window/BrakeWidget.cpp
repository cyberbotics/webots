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

#include "BrakeWidget.hpp"

#include <graph2d/Point2D.hpp>

#include <webots/robot.h>
#include <webots/vehicle/driver.h>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>

using namespace std;

BrakeWidget::BrakeWidget(QWidget *parent) : AbstractWidget(parent) {
  mGraph->setYLabel("Brake [%]");
}

BrakeWidget::~BrakeWidget() {
}

void BrakeWidget::update() {
  if (!mEnableCheckBox->isChecked())
    return;

  double brake = wbu_driver_get_brake_intensity();
  mValueLabel->setText(QString::number(brake, 'f', 3));
  mGraph->addPoint2D(Point2D(wb_robot_get_time(), brake));
  mGraph->updateXRange();
  mGraph->extendYRange();
  mGraph->keepNPoints(pointsKeptNumber());
}
