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

#include "Op3MotionPlayer.hpp"
#include <RobotisOp2MotionManager.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Speaker.hpp>

#include <stdlib.h>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

Op3MotionPlayer::Op3MotionPlayer() : Robot() {
  cout << "--- Demo of ROBOTIS OP3 ---" << endl;
  cout << "This demo shows how to play the RobotisOp3 motions." << endl;

  mTimeStep = getBasicTimeStep();

  mHeadLED = getLED("HeadLed");
  mBodyLED = getLED("BodyLed");
  mCamera = getCamera("Camera");
  mSpeaker = getSpeaker("Speaker");

  mCamera->enable(mTimeStep);
  for (int i = 0; i < NMOTORS; i++) {
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    getPositionSensor(sensorName)->enable(mTimeStep);
  }

  mMotionManager = new RobotisOp2MotionManager(this, "motion_4095.bin");
}

Op3MotionPlayer::~Op3MotionPlayer() {
}

void Op3MotionPlayer::myStep() {
  if (step(mTimeStep) == -1)
    exit(EXIT_SUCCESS);
}

void Op3MotionPlayer::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void Op3MotionPlayer::run() {
  mHeadLED->set(true);
  mBodyLED->set(0xFF00FF);
  myStep();

  mMotionManager->playPage(1);  // Standing position.

  while (true) {
    mSpeaker->speak("Hi, my name is ROBOTIS OP3.", 1.0);
    mMotionManager->playPage(38);  // Hi.
    wait(1000);

    mSpeaker->speak("I have 20 degrees of freedom, and a lot of sensors.", 1.0);
    mMotionManager->playPage(46);  // Talk.
    wait(1000);

    mSpeaker->speak("I love to play football.", 1.0);
    mMotionManager->playPage(83);  // kick right.
    mMotionManager->playPage(84);  // kick left.
    wait(1000);

    mSpeaker->speak("I'm at your disposal.", 1.0);
    mMotionManager->playPage(55);  // Small bow.

    wait(3000);
  }
}
