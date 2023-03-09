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

#include "VisualTracking.hpp"
#include <RobotisOp2VisionManager.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

VisualTracking::VisualTracking() : Robot() {
  mTimeStep = getBasicTimeStep();

  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mCamera = getCamera("Camera");
  mCamera->enable(2 * mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  mVisionManager = new RobotisOp2VisionManager(mCamera->getWidth(), mCamera->getHeight(), 355, 15, 60, 15, 0, 30);
}

VisualTracking::~VisualTracking() {
}

void VisualTracking::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

// function containing the main feedback loop
void VisualTracking::run() {
  double horizontal = 0.0;
  double vertical = 0.0;
  int width = mCamera->getWidth();
  int height = mCamera->getHeight();

  cout << "---------------Visual Tracking---------------" << endl;
  cout << "This example illustrates the possibilities of Vision Manager." << endl;
  cout << "Move the red ball by dragging the mouse while keeping both shift key and mouse button pressed." << endl;

  // First step to update sensors values
  myStep();

  while (true) {
    double x, y;
    bool ballInFieldOfView = mVisionManager->getBallCenter(x, y, mCamera->getImage());
    // Eye led indicate if ball has been found
    if (ballInFieldOfView)
      mEyeLED->set(0x00FF00);
    else
      mEyeLED->set(0xFF0000);
    // Move the head in direction of the ball if found
    if (ballInFieldOfView) {
      double dh = 0.1 * ((x / width) - 0.5);
      horizontal -= dh;
      double dv = 0.1 * ((y / height) - 0.5);
      vertical -= dv;
      horizontal = clamp(horizontal, minMotorPositions[18], maxMotorPositions[18]);
      horizontal = clamp(horizontal, minMotorPositions[19], maxMotorPositions[19]);
      mMotors[18]->setPosition(horizontal);
      mMotors[19]->setPosition(vertical);
    }

    // step
    myStep();
  }
}
