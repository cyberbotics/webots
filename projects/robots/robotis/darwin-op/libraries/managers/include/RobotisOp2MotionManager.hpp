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
//                allowing to play the Robotis motion files

#ifndef ROBOTISOP2_MOTION_MANAGER_HPP
#define ROBOTISOP2_MOTION_MANAGER_HPP

#include <string>

#define DMM_NMOTORS 20

namespace webots {
  class Robot;
  class Motor;
  class PositionSensor;
}  // namespace webots

namespace Robot {
  class Action;
}

namespace managers {
  using namespace Robot;
  class RobotisOp2MotionManager {
  public:
    RobotisOp2MotionManager(webots::Robot *robot, const std::string &customMotionFile = "");
    virtual ~RobotisOp2MotionManager();
    bool isCorrectlyInitialized() { return mCorrectlyInitialized; }
    void playPage(int id, bool sync = true);
    void step(int duration);
    bool isMotionPlaying() { return mMotionPlaying; }

  private:
    webots::Robot *mRobot;
    bool mCorrectlyInitialized;
    Action *mAction;
    int mBasicTimeStep;
    bool mMotionPlaying;

#ifndef CROSSCOMPILATION
    void myStep();
    void wait(int duration);
    void achieveTarget(int timeToAchieveTarget);
    double valueToPosition(unsigned short value);
    void InitMotionAsync();

    webots::Motor *mMotors[DMM_NMOTORS];
    webots::PositionSensor *mPositionSensors[DMM_NMOTORS];
    double mTargetPositions[DMM_NMOTORS];
    double mCurrentPositions[DMM_NMOTORS];
    int mRepeat;
    int mStepnum;
    int mWait;
    int mStepNumberToAchieveTarget;
    void *mPage;
#else
    static void *MotionThread(void *param);  // thread function

    pthread_t mMotionThread;  // thread structure
#endif
  };
}  // namespace managers

#endif
