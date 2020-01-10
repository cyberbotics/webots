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

#include "SpeedWidget.hpp"

#include <graph2d/Point2D.hpp>

#include <webots/robot.h>
#include <webots/vehicle/driver.h>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>

using namespace std;

SpeedWidget::SpeedWidget(QWidget *parent) : AbstractWidget(parent) {
  mGraph->setYLabel("Speed [km/h]");
  mValueLabel->setText("\n\n");
}

SpeedWidget::~SpeedWidget() {
}

void SpeedWidget::update() {
  if (!mEnableCheckBox->isChecked())
    return;

  mValueLabel->setText("");

  if (wbu_driver_get_control_mode() == SPEED) {
    double targetSpeed = wbu_driver_get_target_cruising_speed();
    mGraph->addPoint2D(Point2D(wb_robot_get_time(), targetSpeed, red()));
    mValueLabel->setText("<font color='red'>Target speed: " + QString::number(targetSpeed, 'f', 2) +
                         QString(" km/h</font><br>"));
  }

  double speed = wbu_driver_get_current_speed();
  mValueLabel->setText(mValueLabel->text() + QString("Current speed: ") + QString::number(speed, 'f', 2) + QString(" km/h"));
  mGraph->addPoint2D(Point2D(wb_robot_get_time(), speed));
  mGraph->updateXRange();
  mGraph->extendYRange();
  mGraph->keepNPoints(2 * pointsKeptNumber());
}
