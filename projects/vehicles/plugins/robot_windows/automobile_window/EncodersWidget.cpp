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

#include "EncodersWidget.hpp"

#include <graph2d/Point2D.hpp>

#include <webots/robot.h>
#include <webots/vehicle/car.h>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>

using namespace std;

EncodersWidget::EncodersWidget(QWidget *parent) : AbstractWidget(parent) {
  mGraph->setYLabel("Encoders [rad]");
  mValueLabel->setText("\n\n\n\n");
}

EncodersWidget::~EncodersWidget() {
}

void EncodersWidget::update() {
  if (!mEnableCheckBox->isChecked())
    return;

  double encoder[WBU_CAR_WHEEL_NB];
  for (int i = WBU_CAR_WHEEL_FRONT_RIGHT; i < WBU_CAR_WHEEL_NB; ++i)
    encoder[i] = wbu_car_get_wheel_encoder(WbuCarWheelIndex(i));

  mValueLabel->setText(QString("<font color='black'>Front right: ") +
                       QString::number(encoder[WBU_CAR_WHEEL_FRONT_RIGHT], 'f', 2) + QString(" rad</font><br>") +
                       QString("<font color='red'>Front left: ") + QString::number(encoder[WBU_CAR_WHEEL_FRONT_LEFT], 'f', 2) +
                       QString(" rad</font><br>") + QString("<font color='green'>Rear right: ") +
                       QString::number(encoder[WBU_CAR_WHEEL_REAR_RIGHT], 'f', 2) + QString(" rad</font><br>") +
                       QString("<font color='blue'>Rear left: ") + QString::number(encoder[WBU_CAR_WHEEL_REAR_LEFT], 'f', 2) +
                       QString(" rad</font><br>"));

  mGraph->addPoint2D(Point2D(wb_robot_get_time(), encoder[WBU_CAR_WHEEL_FRONT_RIGHT], black()));
  mGraph->addPoint2D(Point2D(wb_robot_get_time(), encoder[WBU_CAR_WHEEL_FRONT_LEFT], red()));
  mGraph->addPoint2D(Point2D(wb_robot_get_time(), encoder[WBU_CAR_WHEEL_REAR_RIGHT], blue()));
  mGraph->addPoint2D(Point2D(wb_robot_get_time(), encoder[WBU_CAR_WHEEL_REAR_LEFT], green()));

  mGraph->updateXRange();
  mGraph->extendYRange();
  mGraph->keepNPoints(4 * pointsKeptNumber());
}
