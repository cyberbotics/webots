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
 * Description:  Implementation of the EPuckViewWidget.hpp functions
 */

#include "EPuckViewWidget.hpp"
#include "EPuckDrawingHelper.hpp"
#include "EPuckFacade.hpp"
#include "EPuckSlider.hpp"

#include <core/CommonProperties.hpp>
#include <core/Model.hpp>

#include <QtWidgets/QStyle>

#include <cmath>

EPuckViewWidget::EPuckViewWidget(QWidget *parent) : RobotViewWidget(parent) {
  mModel = Model::instance();

  EPuckDrawingHelper::initQGraphicsView(this);
  QGraphicsScene *scene = new QGraphicsScene;
  EPuckDrawingHelper::initQGraphicsScene(scene, style()->standardPalette().color(QPalette::Window));

  int radius = EPuckDrawingHelper::distanceSensorRadius();
  QSize size = EPuckDrawingHelper::distanceSensorSliderSize();

  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++) {
    double angleRad = EPuckDrawingHelper::distanceSensorAngle(i);
    double angleDeg = 180.0 * angleRad / M_PI;
    mDistanceSensorSliders[i] =
      new EPuckSlider(scene, QPoint(radius * cos(angleRad), -radius * sin(angleRad)), -angleDeg, size);
    mDistanceSensorSliders[i]->setType(EPuckSlider::ProgressBar);
    mDistanceSensorSliders[i]->setTextLocation(EPuckSlider::Right);
    mDistanceSensorSliders[i]->setInvertedAppearance(true);
    mDistanceSensorSliders[i]->setEnabled(false);
  }

  size = EPuckDrawingHelper::groundSensorSliderSize();
  int distance = EPuckDrawingHelper::groundSensorSliderDistance();
  radius = EPuckDrawingHelper::groundSensorSliderRadius();

  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++) {
    if (EPuckFacade::groundSensorsExist()) {
      mGroundSensorSliders[i] = new EPuckSlider(scene, QPoint((i - 1) * distance, -radius), -90, size);
      mGroundSensorSliders[i]->setType(EPuckSlider::ProgressBar);
      mGroundSensorSliders[i]->setInvertedAppearance(true);
      mGroundSensorSliders[i]->setEnabled(false);
    } else {
      mGroundSensorSliders[i] = NULL;
    }
  }

  radius = EPuckDrawingHelper::internalSliderRadius();
  size = EPuckDrawingHelper::internalSliderSize();

  mSpeedSliders[EPuckFacade::LEFT] = new EPuckSlider(scene, QPoint(-radius, 0), -90.0, size);
  mSpeedSliders[EPuckFacade::LEFT]->setTextLocation(EPuckSlider::Down);
  mSpeedSliders[EPuckFacade::LEFT]->setRange(-100, 100);
  mSpeedSliders[EPuckFacade::LEFT]->setEnabled(false);
  mSpeedSliders[EPuckFacade::RIGHT] = new EPuckSlider(scene, QPoint(radius, 0), -90.0, size);
  mSpeedSliders[EPuckFacade::RIGHT]->setTextLocation(EPuckSlider::Up);
  mSpeedSliders[EPuckFacade::RIGHT]->setRange(-100, 100);
  mSpeedSliders[EPuckFacade::RIGHT]->setEnabled(false);

  mCameraSliders = new EPuckSlider(scene, QPoint(0, radius), 0.0, size);
  mCameraSliders->setTextLocation(EPuckSlider::Up);
  mCameraSliders->setRange(-100, 100);
  mCameraSliders->setEnabled(false);
  mCameraSliders->setIcon("e-puck/camera.png");

  setScene(scene);
}

EPuckViewWidget::~EPuckViewWidget() {
  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++)
    delete mDistanceSensorSliders[i];
  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++)
    delete mGroundSensorSliders[i];
  for (int i = 0; i < 2; i++)
    delete mSpeedSliders[i];
  delete mCameraSliders;
}

void EPuckViewWidget::updateValues() {
  setUpdatesEnabled(false);
  EPuckFacade *epuck = static_cast<EPuckFacade *>(mModel->robotFacade());

  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++)
    mDistanceSensorSliders[i]->setValue(epuck->distanceSensorValue(i));

  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++)
    if (mGroundSensorSliders[i])
      mGroundSensorSliders[i]->setValue(epuck->groundSensorValue(i));

  mSpeedSliders[EPuckFacade::LEFT]->setValue(epuck->leftSpeed());
  mSpeedSliders[EPuckFacade::RIGHT]->setValue(epuck->rightSpeed());

  mCameraSliders->setValue(epuck->cameraValue());
  setUpdatesEnabled(true);
}
