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

#include "RobotisOp2GaitManager.hpp"

#include <MX28.h>
#include <Walking.h>
#include <minIni.h>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#ifdef CROSSCOMPILATION
#include <MotionManager.h>
#include <RobotisOp2MotionTimerManager.hpp>
#else
#include <MotionStatus.h>
#endif

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace Robot;
using namespace managers;
using namespace webots;
using namespace std;

static const string sotorNames[DGM_NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

RobotisOp2GaitManager::RobotisOp2GaitManager(webots::Robot *robot, const std::string &iniFilename) :
  mRobot(robot),
  mCorrectlyInitialized(true),
  mXAmplitude(0.0),
  mAAmplitude(0.0),
  mYAmplitude(0.0),
  mMoveAimOn(false),
  mBalanceEnable(true),
  mIsWalking(false) {
  if (!mRobot) {
    cerr << "RobotisOp2GaitManager: The robot instance is required" << endl;
    mCorrectlyInitialized = false;
    return;
  }
  mBasicTimeStep = mRobot->getBasicTimeStep();

#ifndef CROSSCOMPILATION
  for (int i = 0; i < DGM_NMOTORS; i++)
    mMotors[i] = mRobot->getMotor(sotorNames[i]);
#endif

  minIni ini(iniFilename.c_str());
  mWalking = Walking::GetInstance();
  mWalking->Initialize();
  mWalking->LoadINISettings(&ini);

#ifdef CROSSCOMPILATION
  RobotisOp2MotionTimerManager::MotionTimerInit();
  MotionManager::GetInstance()->AddModule(dynamic_cast<MotionModule *>(mWalking));
#endif
}

RobotisOp2GaitManager::~RobotisOp2GaitManager() {
}

void RobotisOp2GaitManager::step(int step) {
  if (step < 8) {
    cerr << "RobotisOp2GaitManager: steps of less than 8ms are not supported" << endl;
    return;
  }
#ifdef CROSSCOMPILATION
  mWalking->m_Joint.SetEnableBodyWithoutHead(true, true);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(true);
  MotionManager::GetInstance()->SetEnable(true);
#endif

  if (mIsWalking) {
    mWalking->X_MOVE_AMPLITUDE = mXAmplitude;
    mWalking->A_MOVE_AMPLITUDE = mAAmplitude;
    mWalking->Y_MOVE_AMPLITUDE = mYAmplitude;
    mWalking->A_MOVE_AIM_ON = mMoveAimOn;
    mWalking->BALANCE_ENABLE = mBalanceEnable;
  }

#ifndef CROSSCOMPILATION
  int numberOfStepToProcess = step / 8;

  if (mBalanceEnable && (mRobot->getGyro("Gyro")->getSamplingPeriod() <= 0)) {
    cerr << "The Gyro is not enabled. RobotisOp2GaitManager need the Gyro to run the balance algorithm. The Gyro will be "
            "automatically enabled."
         << endl;
    mRobot->getGyro("Gyro")->enable(mBasicTimeStep);
    myStep();
  }

  for (int i = 0; i < numberOfStepToProcess; i++) {
    if (mBalanceEnable) {
      const double *gyro = mRobot->getGyro("Gyro")->getValues();
      MotionStatus::RL_GYRO = gyro[0] - 512;  // 512 = central value, skip calibration step of the MotionManager,
      MotionStatus::FB_GYRO = gyro[1] - 512;  // because the influence of the calibration is imperceptible.
    }
    mWalking->Process();
  }

  for (int i = 0; i < (DGM_NMOTORS - 2); i++)
    mMotors[i]->setPosition(valueToPosition(mWalking->m_Joint.GetValue(i + 1)));
#endif
}

void RobotisOp2GaitManager::stop() {
  mIsWalking = false;
  mWalking->Stop();
  while (mWalking->IsRunning())
    this->step(8);
#ifdef CROSSCOMPILATION
  // Reset Goal Position of all motors (except Head) after walking //
  for (int i = 0; i < (DGM_NMOTORS - 2); i++)
    mRobot->getMotor(sotorNames[i])->setPosition(MX28::Value2Angle(mWalking->m_Joint.GetValue(i + 1)) * (M_PI / 180));

  // Disable the Joints in the Gait Manager, this allow to control them again 'manualy' //
  mWalking->m_Joint.SetEnableBodyWithoutHead(false, true);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  MotionManager::GetInstance()->SetEnable(false);
#endif
}

void RobotisOp2GaitManager::start() {
  mIsWalking = true;
  mWalking->Start();
}

#ifndef CROSSCOMPILATION
double RobotisOp2GaitManager::valueToPosition(unsigned short value) {
  double degree = MX28::Value2Angle(value);
  double position = degree / 180.0 * M_PI;
  return position;
}

void RobotisOp2GaitManager::myStep() {
  int ret = mRobot->step(mBasicTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}
#endif
