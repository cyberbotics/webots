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

#include "WbJoystickListener.hpp"

#include <cassert>

WbJoystickListener::WbJoystickListener(int numberOfAxes, int numberOfButtons, int numberOfPovs) {
  mAxesNumber = numberOfAxes;
  mButtonsNumber = numberOfButtons;
  mPovsNumber = numberOfPovs;
  mAxesValue = new int[mAxesNumber];
  mButtonsIsPressed = new bool[mButtonsNumber];
  mPovsValue = new int[mPovsNumber];
  mButtonsIsJustReleased = new bool[mButtonsNumber];
  for (int i = 0; i < mAxesNumber; ++i)
    mAxesValue[i] = 0;
  for (int i = 0; i < mPovsNumber; ++i)
    mPovsValue[i] = 0;
  for (int i = 0; i < mButtonsNumber; ++i) {
    mButtonsIsPressed[i] = false;
    mButtonsIsJustReleased[i] = false;
  }
  mButtonHasChanged = false;
  mPovHasChanged = false;
  mAxisHasChanged = false;
}

WbJoystickListener::~WbJoystickListener() {
  delete[] mAxesValue;
  delete[] mButtonsIsPressed;
  delete[] mButtonsIsJustReleased;
  delete[] mPovsValue;
}

bool WbJoystickListener::isButtonPressed(int button) const {
  assert(button < mButtonsNumber);
  return mButtonsIsPressed[button];
}

int WbJoystickListener::axisValue(int axis) const {
  assert(axis < mAxesNumber);
  return mAxesValue[axis];
}

int WbJoystickListener::povValue(int pov) const {
  assert(pov < mPovsNumber);
  return mPovsValue[pov];
}

int WbJoystickListener::numberOfPressedButtons() const {
  int number = 0;
  for (int i = 0; i < mButtonsNumber; ++i) {
    if (mButtonsIsPressed[i])
      ++number;
  }
  return number;
}

void WbJoystickListener::releaseButtons() {
  for (int i = 0; i < mButtonsNumber; ++i) {
    if (mButtonsIsJustReleased[i]) {
      mButtonsIsJustReleased[i] = false;
      mButtonsIsPressed[i] = false;
    }
  }
}

bool WbJoystickListener::buttonPressed(const OIS::JoyStickEvent &arg, int button) {
  if (button < mButtonsNumber) {
    mButtonHasChanged = true;
    mButtonsIsPressed[button] = true;
    emit changed();
  }
  return true;
}

bool WbJoystickListener::buttonReleased(const OIS::JoyStickEvent &arg, int button) {
  if (button < mButtonsNumber) {
    mButtonHasChanged = true;
    mButtonsIsJustReleased[button] = true;
    emit changed();
  }
  return true;
}

bool WbJoystickListener::axisMoved(const OIS::JoyStickEvent &arg, int axis) {
  if (axis < mAxesNumber) {
    mAxisHasChanged = true;
    mAxesValue[axis] = arg.state.mAxes[axis].abs;
    emit changed();
  }
  return true;
}

bool WbJoystickListener::povMoved(const OIS::JoyStickEvent &arg, int index) {
  if (index < mPovsNumber) {
    mPovHasChanged = true;
    mPovsValue[index] = arg.state.mPOV[index].direction;
    emit changed();
  }
  return true;
}
