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
 * Description:  Abstract class defining a condition over the sensors
 */

#ifndef EPUCK_SENSOR_CONDITION_HPP
#define EPUCK_SENSOR_CONDITION_HPP

#include <core/RobotSensorCondition.hpp>

class EPuckFacade;
class EPuckCondition;

class EPuckSensorCondition : public RobotSensorCondition {
public:
  EPuckSensorCondition();
  virtual ~EPuckSensorCondition();

  virtual bool isFired() const;
  virtual void reset();
  virtual void resetSensorsUseNotification();

  void setDistanceSensorValue(int i, int v);
  void setDistanceSensorInverted(int i, bool v);
  int distanceSensorValue(int i) const;
  bool distanceSensorInverted(int i) const;

  void setGroundSensorValue(int i, int v);
  void setGroundSensorInverted(int i, bool v);
  int groundSensorValue(int i) const;
  bool groundSensorInverted(int i) const;

  void setTimerValue(int v);
  int timerValue() const;

  void setCameraValue(int v);
  void setCameraInverted(bool v);
  int cameraValue() const;
  bool cameraInverted() const;

  virtual void fromString(const QString &string);
  virtual void fromStringVersion3(const QString &string);  // backward compatibility code
  virtual QString toString() const;

protected:
  EPuckCondition *mDistanceSensorConditions[8];
  EPuckCondition *mGroundSensorConditions[3];
  EPuckCondition *mTimerCondition;
  EPuckCondition *mCameraCondition;

  EPuckFacade *mEPuckFacade;
  double mStartTimer;
};

#endif
