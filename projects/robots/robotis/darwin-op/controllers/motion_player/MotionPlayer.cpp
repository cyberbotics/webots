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

#include "MotionPlayer.hpp"

#include <webots/utils/Motion.hpp>

#include <cstdlib>
#include <iostream>

using namespace std;
using namespace webots;

// Constructor
MotionPlayer::MotionPlayer() : Robot() {
  // Get time step
  mTimeStep = getBasicTimeStep();
}

// Destructor
MotionPlayer::~MotionPlayer() {
}

// Step function
void MotionPlayer::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

// Wait function
void MotionPlayer::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// function containing the main feedback loop
void MotionPlayer::run() {
  cout << "-------MotionPlayer example of ROBOTIS OP2-------" << endl;
  cout << "This example plays a Webots motion file" << endl;

  // step
  myStep();

  Motion motion("hand_high.motion");
  motion.setLoop(true);
  motion.play();
  while (true)
    myStep();
}
