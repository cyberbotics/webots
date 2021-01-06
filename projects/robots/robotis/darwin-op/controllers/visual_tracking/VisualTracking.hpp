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

// Description:   Simple controller showing how to use vision manager

#ifndef VISUALTRACKING_HPP
#define VISUALTRACKING_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class RobotisOp2VisionManager;
}

namespace webots {
  class Motor;
  class LED;
  class Camera;
};  // namespace webots

class VisualTracking : public webots::Robot {
public:
  VisualTracking();
  virtual ~VisualTracking();
  void run();

private:
  int mTimeStep;

  void myStep();

  webots::Motor *mMotors[NMOTORS];
  webots::LED *mEyeLED;
  webots::LED *mHeadLED;
  webots::Camera *mCamera;

  managers::RobotisOp2VisionManager *mVisionManager;
};

#endif
