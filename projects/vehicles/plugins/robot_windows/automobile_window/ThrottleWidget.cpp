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

#include "ThrottleWidget.hpp"

#include <graph2d/Point2D.hpp>

#include <webots/robot.h>
#include <webots/vehicle/driver.h>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>

using namespace std;

ThrottleWidget::ThrottleWidget(QWidget *parent) : AbstractWidget(parent) {
  mGraph->setYLabel("Throttle [%]");
}

ThrottleWidget::~ThrottleWidget() {
}

void ThrottleWidget::update() {
  if (!mEnableCheckBox->isChecked())
    return;

  if (wbu_driver_get_control_mode() == TORQUE) {
    double throttle = wbu_driver_get_throttle();
    mValueLabel->setText(QString::number(throttle, 'f', 3));
    mGraph->addPoint2D(Point2D(wb_robot_get_time(), throttle));
    mGraph->updateXRange();
    mGraph->extendYRange();
    mGraph->keepNPoints(pointsKeptNumber());
  } else
    mValueLabel->setText("No engine model in speed control");
}
