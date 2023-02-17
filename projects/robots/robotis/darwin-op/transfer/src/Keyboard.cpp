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

#include <webots/Keyboard.hpp>
#include <webots/Robot.hpp>

#include "keyboardInterface.hpp"

#include <stdlib.h>
#include <unistd.h>
#include <iostream>

using namespace webots;
using namespace std;

void *Keyboard::KeyboardTimerProc(void *param) {
  (static_cast<Keyboard *>(param))->mKeyboardInterface->startListenKeyboard();
  return NULL;
}

Keyboard::Keyboard() {
  mKeyboardRate = 0;
  mKeyboardInterface = new KeyboardInterface();
}

Keyboard::~Keyboard() {
}

void Keyboard::enable(int samplingPeriod) {
  if (mKeyboardRate <= 0) {
    // Starting keyboard listenning in a thread
    int error = 0;

    mKeyboardInterface->createWindow();

    // create and start the thread
    if ((error = pthread_create(&this->mKeyboardThread, NULL, this->KeyboardTimerProc, this)) != 0) {
      cerr << "Keyboard thread error = " << error << endl;
      exit(-1);
    }
  }

  mKeyboardRate = samplingPeriod;
}

void Keyboard::disable() {
  if (mKeyboardRate > 0) {
    int error = 0;
    // End the thread
    if ((error = pthread_cancel(this->mKeyboardThread)) != 0) {
      cerr << "Keyboard thread error = " << error << endl;
      exit(-1);
    }
    mKeyboardInterface->closeWindow();
    mKeyboardInterface->resetKeyPressed();
  }

  mKeyboardRate = 0;
}

int Keyboard::getKey() const {
  if (mKeyboardRate >= 0)
    return mKeyboardInterface->getKeyPressed();
  else
    return -1;
}

int Keyboard::getSamplingPeriod() const {
  return mKeyboardRate;
}

void Keyboard::resetKeyboard() {
  if (mKeyboardRate >= 0)
    mKeyboardInterface->resetKeyPressed();
}
