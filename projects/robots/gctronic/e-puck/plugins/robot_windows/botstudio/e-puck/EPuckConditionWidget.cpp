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
 * Description:  Implementation of the EPuckConditionWidget.hpp functions
 */

#include "EPuckConditionWidget.hpp"
#include "EPuckDrawingHelper.hpp"
#include "EPuckSensorCondition.hpp"
#include "EPuckSlider.hpp"

#include <core/CommonProperties.hpp>
#include <core/Model.hpp>

#include <cmath>

EPuckConditionWidget::EPuckConditionWidget(QWidget *parent) : RobotConditionWidget(parent) {
  EPuckDrawingHelper::initQGraphicsView(this);
  QGraphicsScene *scene = new QGraphicsScene;
  EPuckDrawingHelper::initQGraphicsScene(scene, CommonProperties::transitionColor());

  int radius = EPuckDrawingHelper::distanceSensorRadius();
  QSize size = EPuckDrawingHelper::distanceSensorSliderSize();

  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++) {
    double angleRad = EPuckDrawingHelper::distanceSensorAngle(i);
    double angleDeg = 180.0 * angleRad / M_PI;
    mDistanceSensorSliders[i] =
      new EPuckSlider(scene, QPoint(radius * cos(angleRad), -radius * sin(angleRad)), -angleDeg, size);
    mDistanceSensorSliders[i]->setType(EPuckSlider::RevertibleSlider);
    mDistanceSensorSliders[i]->setTextLocation(EPuckSlider::Right);
    mDistanceSensorSliders[i]->setInvertedAppearance(true);
    mDistanceSensorSliders[i]->setDisplayPrefix(true);
    mDistanceSensorSliders[i]->setIndex(i);
    mDistanceSensorSliders[i]->setNeutralValue(0);
    connect(mDistanceSensorSliders[i], SIGNAL(valueChanged()), this, SLOT(updateSensorCondition()));
  }

  size = EPuckDrawingHelper::groundSensorSliderSize();
  int distance = EPuckDrawingHelper::groundSensorSliderDistance();
  radius = EPuckDrawingHelper::groundSensorSliderRadius();

  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++) {
    if (EPuckFacade::groundSensorsExist()) {
      mGroundSensorSliders[i] = new EPuckSlider(scene, QPoint((i - 1) * distance, -radius), -90, size);
      mGroundSensorSliders[i]->setType(EPuckSlider::RevertibleSlider);
      mGroundSensorSliders[i]->setIndex(8 + i);
      connect(mGroundSensorSliders[i], SIGNAL(valueChanged()), this, SLOT(updateSensorCondition()));
    } else {
      mGroundSensorSliders[i] = NULL;
    }
  }

  radius = EPuckDrawingHelper::internalSliderRadius();
  size = EPuckDrawingHelper::internalSliderSize();

  mCameraSliders = new EPuckSlider(scene, QPoint(0, radius), 0.0, size);
  mCameraSliders->setTextLocation(EPuckSlider::Up);
  mCameraSliders->setType(EPuckSlider::RevertibleSlider);
  mCameraSliders->setDisplayPrefix(true);
  mCameraSliders->setRange(-100, 100);
  mCameraSliders->setIndex(11);
  mCameraSliders->setNeutralValue(0);
  mCameraSliders->setIcon("e-puck/camera.png");
  connect(mCameraSliders, SIGNAL(valueChanged()), this, SLOT(updateSensorCondition()));

  mTimerSlider = new EPuckSlider(scene, QPoint(0, -radius), 0.0, size);
  mTimerSlider->setTextLocation(EPuckSlider::Down);
  mTimerSlider->setTextSuffix("s");
  mTimerSlider->setTextRatio(0.1);
  mTimerSlider->setRange(0, 40);
  mTimerSlider->setIndex(12);
  mTimerSlider->setNeutralValue(0);
  mTimerSlider->setIcon("e-puck/timer.png");
  connect(mTimerSlider, SIGNAL(valueChanged()), this, SLOT(updateSensorCondition()));

  setScene(scene);
}

EPuckConditionWidget::~EPuckConditionWidget() {
  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++)
    delete mDistanceSensorSliders[i];
  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++)
    delete mGroundSensorSliders[i];
  delete mCameraSliders;
  delete mTimerSlider;
}

void EPuckConditionWidget::blockSlidersSignals(bool block) {
  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++)
    mDistanceSensorSliders[i]->blockSignals(block);
  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++) {
    if (mGroundSensorSliders[i])
      mGroundSensorSliders[i]->blockSignals(block);
  }
  mCameraSliders->blockSignals(block);
  mTimerSlider->blockSignals(block);
}

void EPuckConditionWidget::setSensorCondition(RobotSensorCondition *condition) {
  blockSlidersSignals(true);
  RobotConditionWidget::setSensorCondition(condition);
  EPuckSensorCondition *esc = static_cast<EPuckSensorCondition *>(sensorCondition());

  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++) {
    mDistanceSensorSliders[i]->setInverted(esc->distanceSensorInverted(i));
    mDistanceSensorSliders[i]->setValue(esc->distanceSensorValue(i));
  }
  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++) {
    if (mGroundSensorSliders[i]) {
      mGroundSensorSliders[i]->setInverted(esc->groundSensorInverted(i));
      mGroundSensorSliders[i]->setValue(esc->groundSensorValue(i));
    }
  }
  mCameraSliders->setInverted(esc->cameraInverted());
  mCameraSliders->setValue(esc->cameraValue());
  mTimerSlider->setValue(esc->timerValue());
  blockSlidersSignals(false);
}

void EPuckConditionWidget::updateSensorCondition() {
  EPuckSlider *slider = dynamic_cast<EPuckSlider *>(sender());
  if (slider) {
    blockSlidersSignals(true);

    EPuckSensorCondition *esc = static_cast<EPuckSensorCondition *>(sensorCondition());
    int i = slider->index();
    int value = slider->value();
    bool inverted = slider->isInverted();

    if (i >= 0 && i < 8) {
      esc->setDistanceSensorInverted(i, inverted);
      esc->setDistanceSensorValue(i, value);
    } else if (i >= 8 && i < 11) {
      esc->setGroundSensorInverted(i - 8, inverted);
      esc->setGroundSensorValue(i - 8, value);
    } else if (i == 11) {
      esc->setCameraInverted(inverted);
      esc->setCameraValue(value);
    } else if (i == 12)
      esc->setTimerValue(value);
    blockSlidersSignals(false);
  }
}
