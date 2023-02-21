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

// Description:   Facade between webots and the robotis-op2 framework
//                allowing to to start the LinuxMotionTimer in order
//                to play the Robotis motion files and the walking algorithm.

#ifndef ROBOTISOP2_MOTION_TIMER_MANAGER_HPP
#define ROBOTISOP2_MOTION_TIMER_MANAGER_HPP

namespace Robot {
  class LinuxMotionTimer;
}

namespace managers {
  using namespace Robot;
  class RobotisOp2MotionTimerManager {
  public:
    RobotisOp2MotionTimerManager();
    virtual ~RobotisOp2MotionTimerManager();
    static void MotionTimerInit();

  private:
    static bool mStarted;
  };
}  // namespace managers

#endif
