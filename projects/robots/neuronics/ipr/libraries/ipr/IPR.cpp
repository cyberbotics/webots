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

#include "IPR.hpp"

#include <cassert>
#include <cmath>
#include <sstream>

static const double UPPER_ARM_MOTOR_TRANSITION_POSITION = -0.726919;
static const double POSITION_TOLERANCE = 0.0002;

string IPR::motorName(int motorIndex) {
  switch (motorIndex) {
    case BASE_MOTOR:
      return "base";
    case UPPER_ARM_MOTOR:
      return "upperarm";
    case FOREARM_MOTOR:
      return "forearm";
    case WRIST_MOTOR:
      return "wrist";
    case ROTATIONAL_WRIST_MOTOR:
      return "rotational_wrist";
    case RIGHT_GRIPPER_MOTOR:
      return "right_gripper";
    case LEFT_GRIPPER_MOTOR:
      return "left_gripper";
    default:
      assert(false);
      return "";
  }
}

IPR::IPR() {
  mTimeStep = getBasicTimeStep();

  // initialize distance sensors
  mDistanceSensors = new DistanceSensor *[DISTANCE_SENSOR_NUMBER];
  for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i) {
    stringstream sensorName;
    sensorName << "ds" << i;
    mDistanceSensors[i] = getDistanceSensor(sensorName.str());
    if (mDistanceSensors[i])
      mDistanceSensors[i]->enable(mTimeStep);
  }

  // initialize touch sensors
  mTouchSensors = new TouchSensor *[TOUCH_SENSOR_NUMBER];
  for (int i = 0; i < TOUCH_SENSOR_NUMBER; ++i) {
    stringstream sensorName;
    sensorName << "ts" << i;
    mTouchSensors[i] = getTouchSensor(sensorName.str());
    if (mTouchSensors[i])
      mTouchSensors[i]->enable(mTimeStep);
  }

  // initialize motors
  mMotors = new Motor *[MOTOR_NUMBER];
  for (int i = 0; i < MOTOR_NUMBER; ++i) {
    mMotors[i] = getMotor(motorName(i));
  }

  // initialize position sensors
  mPositionSensors = new PositionSensor *[MOTOR_NUMBER];
  for (int i = 0; i < MOTOR_NUMBER; ++i) {
    stringstream sensorName;
    sensorName << motorName(i) << "_sensor";
    mPositionSensors[i] = getPositionSensor(sensorName.str());
    mPositionSensors[i]->enable(mTimeStep);
  }
}

IPR::~IPR() {
  delete mMotors;
  delete mPositionSensors;
  delete mDistanceSensors;
  delete mTouchSensors;
}

string IPR::name() const {
  return getName();
}

void IPR::simulationStep(int stepsCount) {
  while (stepsCount > 0) {
    Robot::step(mTimeStep);
    --stepsCount;
  }
}

double IPR::motorPosition(int motorIndex) const {
  if (motorIndex < 0 || motorIndex >= MOTOR_NUMBER)
    return INFINITY;

  PositionSensor *sensors = mPositionSensors[motorIndex];
  if (sensors)
    return sensors->getValue();

  return INFINITY;
}

double IPR::distanceSensorValue(int sensorIndex) const {
  if (sensorIndex < 0 || sensorIndex >= DISTANCE_SENSOR_NUMBER)
    return INFINITY;

  DistanceSensor *ds = mDistanceSensors[sensorIndex];
  if (ds)
    return ds->getValue();

  return INFINITY;
}

double IPR::touchSensorValue(int sensorIndex) const {
  if (sensorIndex < 0 || sensorIndex >= TOUCH_SENSOR_NUMBER)
    return INFINITY;

  TouchSensor *ts = mTouchSensors[sensorIndex];
  if (ts)
    return ts->getValue();

  return INFINITY;
}

bool IPR::objectDetectedInGripper() const {
  double valueCenter = distanceSensorValue(4);
  double valueRight1 = distanceSensorValue(5);
  double valueRight2 = distanceSensorValue(6);
  return (valueCenter + valueRight1 + valueRight2) > 80;
}

bool IPR::positionReached(int motorIndex, double targetPosition) const {
  if (motorIndex < 0 || motorIndex >= MOTOR_NUMBER)
    return false;

  PositionSensor *sensor = mPositionSensors[motorIndex];
  if (sensor)
    return (fabs(sensor->getValue() - targetPosition) <= POSITION_TOLERANCE);  // tolerance

  return false;
}

void IPR::setMotorPosition(int motorIndex, double position) {
  if (motorIndex < 0 || motorIndex >= MOTOR_NUMBER)
    return;

  Motor *motor = mMotors[motorIndex];
  if (motor)
    motor->setPosition(position);
}

void IPR::moveToInitPosition() {
  for (int i = 0; i < MOTOR_NUMBER; ++i)
    setMotorPosition(i, 0.0);

  // check if position reached
  for (int i = 0; i < MOTOR_NUMBER; ++i) {
    while (!positionReached(i, 0.0))
      step(mTimeStep);
  }
}

void IPR::moveToPosition(const double *motorPositions, bool moveGripper) {
  int motorCount = moveGripper ? MOTOR_NUMBER : RIGHT_GRIPPER_MOTOR;
  for (int i = 0; i < motorCount; ++i)
    setMotorPosition(i, motorPositions[i]);

  // check if position reached
  for (int i = 0; i < motorCount; ++i) {
    while (!positionReached(i, motorPositions[i]))
      step(mTimeStep);
  }
}

void IPR::openGripper(double position) {
  setMotorPosition(RIGHT_GRIPPER_MOTOR, position);
  setMotorPosition(LEFT_GRIPPER_MOTOR, 0 - position);

  while (!positionReached(RIGHT_GRIPPER_MOTOR, position))
    step(mTimeStep);

  while (!positionReached(LEFT_GRIPPER_MOTOR, 0 - position))
    step(mTimeStep);
}

void IPR::closeGripper() {
  setMotorPosition(RIGHT_GRIPPER_MOTOR, 0.0);
  setMotorPosition(LEFT_GRIPPER_MOTOR, 0.0);

  // wait until it is closed as much as possible
  double previousRightGripperPosition = INFINITY;
  double previousLeftGripperPosition = INFINITY;
  while (true) {
    const double currentRightGripperPosition = motorPosition(RIGHT_GRIPPER_MOTOR);
    const double currentLeftGripperPosition = motorPosition(LEFT_GRIPPER_MOTOR);

    if ((fabs(currentRightGripperPosition - previousRightGripperPosition) <= POSITION_TOLERANCE) &&
        (fabs(currentLeftGripperPosition - previousLeftGripperPosition) <= POSITION_TOLERANCE))
      break;

    previousRightGripperPosition = currentRightGripperPosition;
    previousLeftGripperPosition = currentLeftGripperPosition;

    step(mTimeStep);
  }
}

void IPR::grabCube(const double *grabPosition) {
  // set motors position objectives
  for (int i = 0; i < MOTOR_NUMBER; ++i) {
    if (i == UPPER_ARM_MOTOR)
      setMotorPosition(UPPER_ARM_MOTOR, UPPER_ARM_MOTOR_TRANSITION_POSITION);
    else
      setMotorPosition(i, grabPosition[i]);
  }

  // check if position reached
  double position;
  for (int i = 0; i < MOTOR_NUMBER; ++i) {
    if (i == UPPER_ARM_MOTOR)
      position = UPPER_ARM_MOTOR_TRANSITION_POSITION;
    else
      position = grabPosition[i];

    while (!positionReached(i, position))
      step(mTimeStep);
  }

  // lower arm
  setMotorPosition(UPPER_ARM_MOTOR, grabPosition[UPPER_ARM_MOTOR]);
  while (!positionReached(UPPER_ARM_MOTOR, grabPosition[UPPER_ARM_MOTOR]))
    step(mTimeStep);

  // wait until sensor detects an objects
  while (!objectDetectedInGripper())
    step(mTimeStep);

  closeGripper();
}

void IPR::dropCube(const double *dropPosition) {
  // raise arm
  setMotorPosition(UPPER_ARM_MOTOR, UPPER_ARM_MOTOR_TRANSITION_POSITION);
  while (!positionReached(UPPER_ARM_MOTOR, UPPER_ARM_MOTOR_TRANSITION_POSITION))
    step(mTimeStep);

  // rotate
  setMotorPosition(BASE_MOTOR, dropPosition[BASE_MOTOR]);

  // set motors position objectives
  for (int i = FOREARM_MOTOR; i < RIGHT_GRIPPER_MOTOR; ++i) {
    if (i != UPPER_ARM_MOTOR)
      setMotorPosition(i, dropPosition[i]);
  }

  while (!positionReached(BASE_MOTOR, dropPosition[BASE_MOTOR]))
    step(mTimeStep);
  // check if position reached
  for (int i = FOREARM_MOTOR; i < RIGHT_GRIPPER_MOTOR; ++i) {
    while (!positionReached(i, dropPosition[i]))
      step(mTimeStep);
  }

  // lower arm
  setMotorPosition(UPPER_ARM_MOTOR, dropPosition[UPPER_ARM_MOTOR]);
  while (!positionReached(UPPER_ARM_MOTOR, dropPosition[UPPER_ARM_MOTOR]))
    step(mTimeStep);

  openGripper();

  // raise arm
  setMotorPosition(UPPER_ARM_MOTOR, 0.0);
  while (!positionReached(UPPER_ARM_MOTOR, 0.0))
    step(mTimeStep);
}
