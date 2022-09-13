// Copyright 1996-2022 Cyberbotics Ltd.
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

#ifndef THYMIO_2_MODEL_HPP
#define THYMIO_2_MODEL_HPP

#include <webots/Robot.hpp>

namespace webots {
  class Accelerometer;
  class DistanceSensor;
  class Motor;
  class PositionSensor;
  class TouchSensor;
}  // namespace webots

class Thymio2AsebaHub;

class Thymio2Model : public webots::Robot {
public:
  Thymio2Model();
  virtual ~Thymio2Model();

  bool safeStep();

  void sensorToHub(Thymio2AsebaHub *hub);
  void hubToActuators(const Thymio2AsebaHub *hub);
  void updateEvents(const Thymio2AsebaHub *hub, bool events[]);

  void setAutomaticBehaviorAccLeds(bool enable) { mAutomaticBehaviorAccLeds = enable; }
  void setAutomaticBehaviorButtonLeds(bool enable) { mAutomaticBehaviorButtonLeds = enable; }
  void setAutomaticBehaviorProxLeds(bool enable) { mAutomaticBehaviorProxLeds = enable; }

private:
  enum Button { B_BACKWARD, B_LEFT, B_CENTER, B_FORWARD, B_RIGHT };
  enum MotorSide { M_LEFT, M_RIGHT };

  void initDevices();
  void reset();

  void behaviorAccLeds();
  void behaviorButtonLeds();
  void behaviorProxLeds();

  int hertzToBestMsPeriod(int hertz) const;
  bool isPeriodicEventFired(int period, int startCounter = 0) const;

  int mTimeStep;
  int mStepCounter;

  int mAccelerometerPeriod;
  int mProxsPeriod;
  int mButtonsPeriod;
  int mMotorsPeriod;

  int mLastTimerPeriod[2];
  int mTimerStepStart[2];

  bool mAutomaticBehaviorAccLeds;
  bool mAutomaticBehaviorButtonLeds;
  bool mAutomaticBehaviorProxLeds;

  double mLastMotorPosition[2];
  bool mLastButtonsState[5];

  void updateWindowData();
  int mWindowData[7];  // should match with the robot window implementation

  webots::Accelerometer *mAccelerometer;
  webots::TouchSensor *mButtons[5];
  webots::Motor *mMotors[2];
  webots::PositionSensor *mPositionSensors[2];
  webots::DistanceSensor *mHorizontalProxs[7];
  webots::DistanceSensor *mVerticalProxs[2];
};

#endif
