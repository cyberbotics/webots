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
 * Description:  Class defining the way to get the e-puck information
 */

#ifndef EPUCK_FACADE_HPP
#define EPUCK_FACADE_HPP

#include <core/RobotFacade.hpp>

#include <webots/types.h>

#include <QtCore/QString>

class EPuckFacade : public RobotFacade {
public:
  EPuckFacade();
  virtual ~EPuckFacade();

  enum { WIDTH, HEIGHT };
  enum { LEFT, RIGHT };
  enum { OFF, ON, NONE };
  enum {
    NUMBER_OF_LEDS = 10,
    NUMBER_OF_DISTANCE_SENSORS = 8,
    NUMBER_OF_LIGHT_SENSORS = 8,
    NUMBER_OF_GROUND_SENSORS = 3,
    NUMBER_OF_MOTORS = 2
  };

  virtual void update();
  virtual void stop();
  virtual void sendActuatorCommands();

  int rightSpeed() const;
  int leftSpeed() const;
  bool ledValue(int id) const;
  int distanceSensorValue(int id) const;
  int groundSensorValue(int id) const;
  int cameraValue() const;
  double timer() const;

  void setLeftSpeed(int speed);
  void setRightSpeed(int speed);
  void setSpeeds(int leftSpeed, int rightSpeed);
  void setLedValue(int id, bool value);

  // notify the use of a ground sensor and display warning if the sensor is not included in the robot
  // returns TRUE if the warning message is displayed
  bool notifyUseOfGroundSensor(int id);
  static bool groundSensorsExist();

private:
  void printDeviceError(const QString &deviceName);

  void initDevices();
  void initializeRobot();
  void resetActuators();

  double mSpeeds[2];
  int mLedValues[NUMBER_OF_LEDS];
  double mDistanceSensorValues[NUMBER_OF_DISTANCE_SENSORS];
  double mGroundSensorValues[NUMBER_OF_GROUND_SENSORS];
  const unsigned char *mCameraValues;
  int mCameraSize[2];

  WbDeviceTag mLeds[NUMBER_OF_LEDS];
  WbDeviceTag mDistanceSensors[NUMBER_OF_DISTANCE_SENSORS];
  WbDeviceTag mGroundSensors[NUMBER_OF_GROUND_SENSORS];
  WbDeviceTag mCamera;
  WbDeviceTag mMotors[NUMBER_OF_MOTORS];

  double mTimer;
};

#endif
