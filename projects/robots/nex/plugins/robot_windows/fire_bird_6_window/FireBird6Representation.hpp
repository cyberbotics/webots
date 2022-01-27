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

/*
 * Description:  Facade of an Fire Bird 6. A call of the update function updates the representation using the Webots API
 */

#ifndef FIREBIRD6_REPRESENTATION_HPP
#define FIREBIRD6_REPRESENTATION_HPP

#include <webots/types.h>

//#define NUMBER_OF_LEDS 10
#define NUMBER_OF_DISTANCE_SENSORS 8
#define NUMBER_OF_DISTANCE_SENSORS_SHARP 8
#define NUMBER_OF_GROUND_SENSORS 8
#define NUMBER_OF_MOTORS 2

enum { LEFT, RIGHT };

typedef struct _Device {
  WbDeviceTag tag;
  int samplingPeriod;
  union {
    int iValue;
    double dValue;
    const double *dValues;
    const unsigned char *image;
  };
} Device;

class FireBird6Representation {
public:
  static FireBird6Representation *instance();
  static void killInstance();
  void update();

  double distanceSensorValue(int id) const;
  double distanceSensorSharpValue(int id) const;
  double lightSensorValue(int id) const;
  double groundSensorValue(int id) const;
  const double *accelerometerValues() const;
  const double *gyroValues() const;
  const double *magnetometerXYValues() const;
  const double *magnetometerZValues() const;
  double rightSpeed() const;
  double leftSpeed() const;
  double rightEncoderValue() const;
  double leftEncoderValue() const;

  bool isDistanceSensorEnabled(int id) const;
  bool isDistanceSensorSharpEnabled(int id) const;
  bool isGroundSensorEnabled(int id) const;
  bool isAccelerometerEnabled() const;
  bool isGyroEnabled() const;
  bool isMagnetometerXYEnabled() const;
  bool isMagnetometerZEnabled() const;
  bool areEncodersEnabled() const;

  bool groundSensorsExist() const { return mGroundSensorsExist; }

  void enableAllSensors();

protected:
  static FireBird6Representation *cInstance;

  explicit FireBird6Representation(int);
  virtual ~FireBird6Representation();

  Device mDistanceSensors[NUMBER_OF_DISTANCE_SENSORS];
  Device mDistanceSensors_Sharp[NUMBER_OF_DISTANCE_SENSORS_SHARP];
  Device mAccelerometer;
  Device mGyro;
  Device mMagnetometerXY;
  Device mMagnetometerZ;
  Device mMotors[NUMBER_OF_MOTORS];
  Device mPositionSensors[NUMBER_OF_MOTORS];

  bool mGroundSensorsExist;
};

#endif
