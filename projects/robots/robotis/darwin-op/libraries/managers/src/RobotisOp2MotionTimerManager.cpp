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

#include <RobotisOp2MotionTimerManager.hpp>

#include <webots/Robot.hpp>

#include <LinuxMotionTimer.h>
#include <MotionManager.h>

using namespace Robot;
using namespace managers;
using namespace webots;
using namespace std;

RobotisOp2MotionTimerManager::RobotisOp2MotionTimerManager() {
}

void RobotisOp2MotionTimerManager::MotionTimerInit() {
  if (!mStarted) {
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    mStarted = true;
  }
}

RobotisOp2MotionTimerManager::~RobotisOp2MotionTimerManager() {
}

bool RobotisOp2MotionTimerManager::mStarted = false;
