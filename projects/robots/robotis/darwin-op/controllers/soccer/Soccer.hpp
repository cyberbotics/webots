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

// Description:   Simple soccer player showing how to use the middleware between webots and
//                the robotis-op2 framework

#ifndef SOCCER_HPP
#define SOCCER_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class RobotisOp2MotionManager;
  class RobotisOp2GaitManager;
  class RobotisOp2VisionManager;
}  // namespace managers

namespace webots {
  class Motor;
  class LED;
  class Camera;
  class Accelerometer;
  class PositionSensor;
  class Gyro;
  class Speaker;
};  // namespace webots

class Soccer : public webots::Robot {
public:
  Soccer();
  virtual ~Soccer();
  void run();

private:
  int mTimeStep;

  void myStep();
  void wait(int ms);
  bool getBallCenter(double &x, double &y);

  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::LED *mEyeLED;
  webots::LED *mHeadLED;
  webots::LED *mBackLedRed;
  webots::LED *mBackLedGreen;
  webots::LED *mBackLedBlue;
  webots::Camera *mCamera;
  webots::Accelerometer *mAccelerometer;
  webots::Gyro *mGyro;
  webots::Speaker *mSpeaker;

  managers::RobotisOp2MotionManager *mMotionManager;
  managers::RobotisOp2GaitManager *mGaitManager;
  managers::RobotisOp2VisionManager *mVisionManager;
};

#endif
