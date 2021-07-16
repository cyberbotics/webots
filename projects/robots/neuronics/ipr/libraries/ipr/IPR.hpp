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

#ifndef IPR_HPP
#define IPR_HPP

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

using namespace webots;
using namespace std;

class IPR : public Robot {
public:
  enum Motors {
    BASE_MOTOR = 0,
    UPPER_ARM_MOTOR = 1,
    FOREARM_MOTOR = 2,
    WRIST_MOTOR = 3,
    ROTATIONAL_WRIST_MOTOR = 4,
    RIGHT_GRIPPER_MOTOR = 5,
    LEFT_GRIPPER_MOTOR = 6,
    MOTOR_NUMBER = 7
  };

  static string motorName(int motorIndex);
  static const int DISTANCE_SENSOR_NUMBER = 9;
  static const int TOUCH_SENSOR_NUMBER = 4;

  IPR();
  virtual ~IPR();

  string name() const;
  int basicTimeStep() const { return mTimeStep; }
  void simulationStep(int stepsCount = 1);

  double motorPosition(int motorIndex) const;
  double distanceSensorValue(int sensorIndex) const;
  double touchSensorValue(int sensorIndex) const;

  virtual bool objectDetectedInGripper() const;
  bool positionReached(int motorIndex, double targetPosition) const;

  // set motor position without waiting until the position is reached
  void setMotorPosition(int motorIndex, double position);

  // move functions that wait until the position is reached
  void moveToInitPosition();
  void moveToPosition(const double *motorPositions, bool moveGripper = false);
  void openGripper(double position = 0.662);
  void closeGripper();

  // complex movement functions
  void grabCube(const double *grabPosition);
  void dropCube(const double *dropPosition);

private:
  int mTimeStep;
  DistanceSensor **mDistanceSensors;
  Motor **mMotors;
  PositionSensor **mPositionSensors;
  TouchSensor **mTouchSensors;
};

#endif
