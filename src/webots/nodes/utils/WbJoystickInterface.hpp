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

#ifndef WB_JOYSTICK_INTERFACE__HPP
#define WB_JOYSTICK_INTERFACE__HPP

#include <QtGui/qwindowdefs.h>  // for WId
#include <QtCore/QObject>
#include <QtCore/QString>

namespace OIS {
  class JoyStick;
  class ForceFeedback;
  class Effect;
}  // namespace OIS

class WbJoystickListener;

class WbJoystickInterface : public QObject {
  Q_OBJECT

public:
  WbJoystickInterface();
  virtual ~WbJoystickInterface();

  void capture();
  bool isButtonPressed(int button) const;
  int axisValue(int axis) const;
  int povValue(int pov) const;
  int numberOfAxes() const;
  int numberOfPovs() const;
  int numberOfButtons() const;
  int numberOfPressedButtons() const;
  void releaseButtons();
  const QString model() const;

  // return at wich rate the step function should be called (in ms)
  int updateRate() const { return 50; }

  void addForce(int level);
  void setAutoCenteringGain(double gain) { mAutoCenteringGain = gain; }
  void setResistanceGain(double gain) { mResistanceGain = gain; }
  void setConstantForceDuration(double duration) { mConstantForceDuration = duration; }
  void setForceAxis(int axis);

  QString initializationError() const { return mError; }
  bool isCorrectlyInitialized() const { return mCorrectlyInitialized; }
  bool hasForceFeedback() const { return mHasForceFeedback; }
  bool supportConstantForceFeedbackEffect() const { return mSupportConstantForceFeedbackEffect; }

  bool buttonHasChanged();
  bool povHasChanged();
  bool axisHasChanged();
  void resetButtonHasChanged();
  void resetPovHasChanged();
  void resetAxisHasChanged();

  static void setWindowHandle(WId windowHandle);

public slots:
  void step();

signals:
  void changed();

private:
  void computeAutoCentering();
  void computeLowSpeedResistance();
  void setForceFeedback();
  void init();

  OIS::JoyStick *mJoystick;
  OIS::ForceFeedback *mForceFeedback;
  OIS::Effect *mEffect;

  WbJoystickListener *mListener;

  QString mError;

  double mAutoCenteringGain;
  double mResistanceGain;
  int mPreviousResistanceAngle;
  bool mPreviousResistanceAngleInitialized;

  double mAddedForce;
  double mAutoCenteringForce;
  double mResistantForce;
  double mConstantForceDuration;
  double mConstantForceTimer;
  int mForceAxis;

  bool mCorrectlyInitialized;
  bool mHasForceFeedback;
  bool mForceFeedbackUsed;
  bool mSupportConstantForceFeedbackEffect;
};

#endif  // WB_JOYSTICK_INTERFACE__HPP
