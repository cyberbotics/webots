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

#include "SteeringWidget.hpp"

#include <graph2d/Point2D.hpp>

#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>

using namespace std;

SteeringWidget::SteeringWidget(QWidget *parent) : AbstractWidget(parent) {
  mGraph->setYLabel("Steering [rad]");
  mValueLabel->setText("\n\n\n");
}

SteeringWidget::~SteeringWidget() {
}

void SteeringWidget::update() {
  if (!mEnableCheckBox->isChecked())
    return;

  double steering = wbu_driver_get_steering_angle();
  double steeringRight = wbu_car_get_right_steering_angle();
  double steeringLeft = wbu_car_get_left_steering_angle();

  mValueLabel->setText(
    QString("<font color='red'>Steering angle:") + QString::number(steering, 'f', 4) + QString(" rad</font><br>") +
    QString("<font color='blue'>Right steering angle:") + QString::number(steeringRight, 'f', 4) + QString(" rad</font><br>") +
    QString("<font color='green'>Left steering angle:") + QString::number(steeringLeft, 'f', 4) + QString(" rad</font>"));

  mGraph->addPoint2D(Point2D(wb_robot_get_time(), steering, red()));
  mGraph->addPoint2D(Point2D(wb_robot_get_time(), steeringRight, blue()));
  mGraph->addPoint2D(Point2D(wb_robot_get_time(), steeringLeft, green()));
  mGraph->updateXRange();
  mGraph->extendYRange();
  mGraph->keepNPoints(3 * pointsKeptNumber());
}
