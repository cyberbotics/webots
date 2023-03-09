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

#include "Symmetry.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <webots/Speaker.hpp>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
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

// Constructor
Symmetry::Symmetry() : Robot() {
  // Get time step
  mTimeStep = getBasicTimeStep();

  // Get the two RGB_LEDs
  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");

  // Get all the 20 Motors and PositionSensors enable them
  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }
}

// Destructor
Symmetry::~Symmetry() {
}

// Step function
void Symmetry::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

// Wait function
void Symmetry::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// function containing the main feedback loop
void Symmetry::run() {
  getSpeaker("Speaker")->speak(
    "Hello, my name is ROBOTIS OP2. Please move my right arm, and I will move the left one symmetrically.", 1.0);

  cout << "-------Symmetry example of ROBOTIS OP2-------" << endl;
  cout << "The right arm is free while the left one mimics it." << endl;
  cout << "In order to move the left arm, add a force to the right arm:" << endl;
  cout << "keep alt pressed and select the right arm." << endl;
  cout << "Now you just have to move the mouse without releasing it." << endl;
  cout << "This example illustrate also the selfCollision which is activated by default" << endl;

  double position[3] = {0, 0, 0};

  mMotors[0]->setAvailableTorque(0.0);
  mMotors[2]->setAvailableTorque(0.0);
  mMotors[4]->setAvailableTorque(0.0);

  // step
  myStep();

  while (true) {
    // Get position of right arm of the robot
    // invert (symmetry) and bound the positions
    position[0] = clamp(-mPositionSensors[0]->getValue(), minMotorPositions[1], maxMotorPositions[1]);
    position[1] = clamp(-mPositionSensors[2]->getValue(), minMotorPositions[3], maxMotorPositions[3]);
    position[2] = clamp(-mPositionSensors[4]->getValue(), minMotorPositions[5], maxMotorPositions[5]);

    // Set position of left arm of the robot
    mMotors[1]->setPosition(position[0]);
    mMotors[3]->setPosition(position[1]);
    mMotors[5]->setPosition(position[2]);

    // step
    myStep();
  }
}
