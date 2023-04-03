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
//                allowing to handle the gait generator

#ifndef ROBOTISOP2_GAIT_MANAGER_HPP
#define ROBOTISOP2_GAIT_MANAGER_HPP

#include <string>

#define DGM_NMOTORS 20
#define DGM_BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

namespace webots {
  class Robot;
  class Motor;
}  // namespace webots

namespace Robot {
  class Walking;
}

namespace managers {
  using namespace Robot;
  class RobotisOp2GaitManager {
  public:
    RobotisOp2GaitManager(webots::Robot *robot, const std::string &iniFilename);
    virtual ~RobotisOp2GaitManager();
    bool isCorrectlyInitialized() { return mCorrectlyInitialized; }

    void setXAmplitude(double x) { mXAmplitude = DGM_BOUND(x, -1.0, 1.0) * 20.0; }
    void setYAmplitude(double y) { mYAmplitude = DGM_BOUND(y, -1.0, 1.0) * 40.0; }
    void setAAmplitude(double a) { mAAmplitude = DGM_BOUND(a, -1.0, 1.0) * 50.0; }
    void setMoveAimOn(bool q) { mMoveAimOn = q; }
    void setBalanceEnable(bool q) { mBalanceEnable = q; }

    void start();
    void step(int duration);
    void stop();

  private:
    webots::Robot *mRobot;
    bool mCorrectlyInitialized;
    Walking *mWalking;
    int mBasicTimeStep;
    double mXAmplitude;
    double mAAmplitude;
    double mYAmplitude;
    bool mMoveAimOn;
    bool mBalanceEnable;
    bool mIsWalking;

#ifndef CROSSCOMPILATION
    void myStep();
    double valueToPosition(unsigned short value);
    webots::Motor *mMotors[DGM_NMOTORS];
#endif
  };
}  // namespace managers

#endif
