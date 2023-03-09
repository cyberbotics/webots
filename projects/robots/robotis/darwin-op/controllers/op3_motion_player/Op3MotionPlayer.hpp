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

#ifndef OP3_MOTION_PLAYER_HPP
#define OP3_MOTION_PLAYER_HPP

#include <webots/Robot.hpp>

#define NMOTORS 20

namespace managers {
  class RobotisOp2MotionManager;
}  // namespace managers

namespace webots {
  class LED;
  class Camera;
  class Speaker;
};  // namespace webots

class Op3MotionPlayer : public webots::Robot {
public:
  Op3MotionPlayer();
  virtual ~Op3MotionPlayer();
  void run();

private:
  int mTimeStep;

  void myStep();
  void wait(int ms);

  webots::LED *mHeadLED;
  webots::LED *mBodyLED;
  webots::Camera *mCamera;
  webots::Speaker *mSpeaker;

  managers::RobotisOp2MotionManager *mMotionManager;
};

#endif
