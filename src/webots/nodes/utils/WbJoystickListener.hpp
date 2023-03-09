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

#ifndef WB_JOYSTICK_LISTENER_HPP
#define WB_JOYSTICK_LISTENER_HPP

#include <OIS/OISJoyStick.h>
#include <QtCore/QObject>

class WbJoystickListener : public QObject, public OIS::JoyStickListener {
  Q_OBJECT

public:
  WbJoystickListener(int numberOfAxes, int numberOfButtons, int numberOfPovs);
  virtual ~WbJoystickListener();

  bool isButtonPressed(int button) const;
  int axisValue(int axis) const;
  int povValue(int pov) const;

  int numberOfAxes() const { return mAxesNumber; }
  int numberOfButtons() const { return mButtonsNumber; }
  int numberOfPovs() const { return mPovsNumber; }
  int numberOfPressedButtons() const;

  bool buttonHasChanged() const { return mButtonHasChanged; }
  bool povHasChanged() const { return mPovHasChanged; }
  bool axisHasChanged() const { return mAxisHasChanged; }
  void resetButtonHasChanged() { mButtonHasChanged = false; }
  void resetPovHasChanged() { mPovHasChanged = false; }
  void resetAxisHasChanged() { mAxisHasChanged = false; }

  void releaseButtons();

  // reimplemeted functions from OIS::JoyStickListener
  bool buttonPressed(const OIS::JoyStickEvent &arg, int button) override;
  bool buttonReleased(const OIS::JoyStickEvent &arg, int button) override;
  bool axisMoved(const OIS::JoyStickEvent &arg, int axis) override;
  bool povMoved(const OIS::JoyStickEvent &arg, int index) override;

private:
  int *mAxesValue;
  int *mPovsValue;
  bool *mButtonsIsPressed;
  bool *mButtonsIsJustReleased;
  int mAxesNumber;
  int mPovsNumber;
  int mButtonsNumber;

  bool mButtonHasChanged;
  bool mPovHasChanged;
  bool mAxisHasChanged;

signals:
  void changed();
};

#endif  // WB_JOYSTICK_LISTENER_HPP
