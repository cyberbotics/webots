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

#include "RobotisOp2MotionManager.hpp"

#include "RobotisOp2DirectoryManager.hpp"

#include <Action.h>
#include <MX28.h>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#ifdef CROSSCOMPILATION
#include <MotionManager.h>
#include <unistd.h>
#include <RobotisOp2MotionTimerManager.hpp>
#endif

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

using namespace Robot;
using namespace managers;
using namespace webots;
using namespace std;

extern "C" {
#ifdef CROSSCOMPILATION
const char *wbu_system_short_path(const char *path) {
  return path;
}
#else
const char *wbu_system_short_path(const char *);
#endif
}

static const string motorNames[DMM_NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

#ifndef CROSSCOMPILATION
static double minMotorPositions[DMM_NMOTORS];
static double maxMotorPositions[DMM_NMOTORS];

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}
#endif

RobotisOp2MotionManager::RobotisOp2MotionManager(webots::Robot *robot, const std::string &customMotionFile) :
  mRobot(robot),
  mCorrectlyInitialized(true) {
  if (!mRobot) {
    cerr << "RobotisOp2MotionManager: The robot instance is required" << endl;
    mCorrectlyInitialized = false;
    return;
  }
  mBasicTimeStep = mRobot->getBasicTimeStep();
  string filename;
  mMotionPlaying = false;

#ifdef CROSSCOMPILATION
  RobotisOp2MotionTimerManager::MotionTimerInit();

  int firm_ver = 0;
  CM730 *cm730 = mRobot->getCM730();
  if (cm730->ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0) != CM730::SUCCESS) {
    cerr << "Can't read firmware version from Dynamixel ID " << JointData::ID_HEAD_PAN << endl;
    mCorrectlyInitialized = false;
    mAction = NULL;
    return;
  }

  if (0 < firm_ver && firm_ver < 27)
    filename = RobotisOp2DirectoryManager::getDataDirectory() + "motion_1024.bin";
  else if (27 <= firm_ver)
    filename = RobotisOp2DirectoryManager::getDataDirectory() + "motion_4096.bin";
  else {
    cerr << "The firmware version of Dynamixel ID " << JointData::ID_HEAD_PAN << " is corrupted." << endl;
    mCorrectlyInitialized = false;
    mAction = NULL;
    return;
  }
#else
  for (int i = 0; i < DMM_NMOTORS; i++) {
    mCurrentPositions[i] = 0.0;
    mMotors[i] = mRobot->getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = mRobot->getPositionSensor(sensorName);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }
  if (customMotionFile == "")
    filename = RobotisOp2DirectoryManager::getDataDirectory() + "motion_4096.bin";
  else
    filename = customMotionFile;
#endif

  mAction = Action::GetInstance();

  const char *path = wbu_system_short_path(filename.c_str());
  if (mAction->LoadFile(const_cast<char *>(path)) == false) {
    cerr << "RobotisOp2MotionManager: Cannot load the motion from " << filename << endl;
    mCorrectlyInitialized = false;
    mAction = NULL;
    return;
  }

#ifdef CROSSCOMPILATION
  MotionManager::GetInstance()->AddModule(dynamic_cast<MotionModule *>(mAction));
#endif
}

RobotisOp2MotionManager::~RobotisOp2MotionManager() {
  if (mAction && mAction->IsRunning())
    mAction->Stop();
}

void RobotisOp2MotionManager::playPage(int id, bool sync) {
  if (!mCorrectlyInitialized)
    return;

#ifdef CROSSCOMPILATION
  mAction->m_Joint.SetEnableBody(true, true);
  MotionStatus::m_CurrentJoints.SetEnableBody(true);
  MotionManager::GetInstance()->SetEnable(true);
  usleep(8000);
  Action::GetInstance()->Start(id);
  if (sync) {
    while (Action::GetInstance()->IsRunning())
      usleep(mBasicTimeStep * 1000);

    // Reset Goal Position of all motors after a motion //
    int i;
    for (i = 0; i < DMM_NMOTORS; i++)
      mRobot->getMotor(motorNames[i])->setPosition(MX28::Value2Angle(mAction->m_Joint.GetValue(i + 1)) * (M_PI / 180));

    // Disable the Joints in the Gait Manager, this allow to control them again 'manualy' //
    mAction->m_Joint.SetEnableBody(false, true);
    MotionStatus::m_CurrentJoints.SetEnableBody(false);
    MotionManager::GetInstance()->SetEnable(false);
  } else {
    int error = pthread_create(&this->mMotionThread, NULL, this->MotionThread, this);
    if (error != 0)
      cerr << "Motion thread error = " << error << endl;
  }
#else
  if (sync) {
    Action::PAGE page;
    if (mAction->LoadPage(id, &page)) {
      // cout << "Play motion " << setw(2) << id << ": " << page.header.name << endl;
      for (int i = 0; i < page.header.repeat; i++) {
        for (int j = 0; j < page.header.stepnum; j++) {
          for (int k = 0; k < DMM_NMOTORS; k++)
            mTargetPositions[k] = valueToPosition(page.step[j].position[k + 1]);
          achieveTarget(8 * page.step[j].time);
          wait(8 * page.step[j].pause);
        }
      }
      if (page.header.next != 0)
        playPage(page.header.next);
    } else
      cerr << "Cannot load the page" << endl;
  } else {
    InitMotionAsync();
    mPage = new Action::PAGE;
    if (!(mAction->LoadPage(id, (Action::PAGE *)mPage)))
      cerr << "Cannot load the page" << endl;
  }
#endif
}

#ifndef CROSSCOMPILATION
void RobotisOp2MotionManager::myStep() {
  int ret = mRobot->step(mBasicTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void RobotisOp2MotionManager::wait(int duration) {
  double startTime = mRobot->getTime();
  double s = (double)duration / 1000.0;
  while (s + startTime >= mRobot->getTime())
    myStep();
}

void RobotisOp2MotionManager::achieveTarget(int timeToAchieveTarget) {
  int stepNumberToAchieveTarget = timeToAchieveTarget / mBasicTimeStep;
  bool stepNeeded = false;

  for (int i = 0; i < DMM_NMOTORS; i++) {
    if (mPositionSensors[i]->getSamplingPeriod() <= 0) {
      cerr << "The position feedback of the position sensor " << motorNames[i]
           << "S is not enabled. RobotisOp2MotionManager need to read the position of all position sensors. The position will "
              "be automatically enable."
           << endl;
      mPositionSensors[i]->enable(mBasicTimeStep);
      stepNeeded = true;
    }
    if (stepNeeded)
      myStep();
    mCurrentPositions[i] = mPositionSensors[i]->getValue();
  }

  while (stepNumberToAchieveTarget > 0) {
    for (int i = 0; i < DMM_NMOTORS; i++) {
      double dX = mTargetPositions[i] - mCurrentPositions[i];
      double newPosition = mCurrentPositions[i] + dX / stepNumberToAchieveTarget;
      newPosition = clamp(newPosition, minMotorPositions[i], maxMotorPositions[i]);
      mCurrentPositions[i] = newPosition;
      mMotors[i]->setPosition(newPosition);
    }
    myStep();
    stepNumberToAchieveTarget--;
  }
}

double RobotisOp2MotionManager::valueToPosition(unsigned short value) {
  double degree = MX28::Value2Angle(value);
  double position = degree / 180.0 * M_PI;
  return position;
}

void RobotisOp2MotionManager::InitMotionAsync() {
  bool stepNeeded = false;
  for (int i = 0; i < DMM_NMOTORS; i++) {
    if (mPositionSensors[i]->getSamplingPeriod() <= 0) {
      cerr << "The position feedback of the position sensor " << motorNames[i]
           << "S is not enabled. RobotisOp2MotionManager need to read the position of all position sensors. The position will "
              "be automatically enable."
           << endl;
      mPositionSensors[i]->enable(mBasicTimeStep);
      stepNeeded = true;
    }
    if (stepNeeded)
      myStep();
    mCurrentPositions[i] = mPositionSensors[i]->getValue();
  }
  mStepnum = 0;
  mRepeat = 1;
  mStepNumberToAchieveTarget = 0;
  mWait = 0;
  mMotionPlaying = true;
}

void RobotisOp2MotionManager::step(int duration) {
  if (mStepNumberToAchieveTarget > 0) {
    for (int i = 0; i < DMM_NMOTORS; i++) {
      double dX = mTargetPositions[i] - mCurrentPositions[i];
      double newPosition = mCurrentPositions[i] + dX / mStepNumberToAchieveTarget;
      mCurrentPositions[i] = newPosition;
      mMotors[i]->setPosition(newPosition);
    }
    mStepNumberToAchieveTarget--;
  } else if (mWait > 0)
    mWait--;
  else {
    if (mStepnum < ((Action::PAGE *)mPage)->header.stepnum) {
      for (int k = 0; k < DMM_NMOTORS; k++)
        mTargetPositions[k] = valueToPosition(((Action::PAGE *)mPage)->step[mStepnum].position[k + 1]);
      mStepNumberToAchieveTarget = (8 * ((Action::PAGE *)mPage)->step[mStepnum].time) / mBasicTimeStep;
      if (mStepNumberToAchieveTarget == 0)
        mStepNumberToAchieveTarget = 1;
      mWait = (8 * ((Action::PAGE *)mPage)->step[mStepnum].pause) / mBasicTimeStep + 0.5;
      mStepnum++;
      step(duration);
    } else if (mRepeat < (((Action::PAGE *)mPage)->header.repeat)) {
      mRepeat++;
      mStepnum = 0;
      step(duration);
    } else if (((Action::PAGE *)mPage)->header.next != 0)
      playPage(((Action::PAGE *)mPage)->header.next, true);
    else
      mMotionPlaying = false;
  }
}
#else

void *RobotisOp2MotionManager::MotionThread(void *param) {
  RobotisOp2MotionManager *instance = (dynamic_cast<RobotisOp2MotionManager *>(param));
  instance->mMotionPlaying = true;
  while (Action::GetInstance()->IsRunning())
    usleep(instance->mBasicTimeStep * 1000);

  // Reset Goal Position of all motors after a motion //
  for (int i = 0; i < DMM_NMOTORS; i++)
    instance->mRobot->getMotor(motorNames[i])
      ->setPosition(MX28::Value2Angle(instance->mAction->m_Joint.GetValue(i + 1)) * (M_PI / 180));

  // Disable the Joints in the Gait Manager, this allow to control them again 'manualy' //
  instance->mAction->m_Joint.SetEnableBody(false, true);
  MotionStatus::m_CurrentJoints.SetEnableBody(false);
  MotionManager::GetInstance()->SetEnable(false);
  // cppcheck-suppress redundantAssignment
  instance->mMotionPlaying = false;
  return NULL;
}

#endif
