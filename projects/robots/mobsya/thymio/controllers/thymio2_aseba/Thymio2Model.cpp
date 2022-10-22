// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Thymio2Model.hpp"

#include "Thymio2AsebaHub.hpp"

#include <webots/Accelerometer.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>

#include <cstdio>  // sscanf
#include <iostream>
#include <limits>
#include <sstream>

#define VELOCITY_RATIO (9.53 / 500.0)

using namespace webots;
using namespace std;

static Thymio2Model *thymio2 = NULL;

Thymio2Model::Thymio2Model() : Robot() {
  thymio2 = this;

  mStepCounter = -1;  // the first step will be step 0, the modulo tests on the period is facilitated this way
  mTimeStep = (int)getBasicTimeStep();

  if (mTimeStep != 10)
    cout << "In order to have an accurate simulation, it is recommended to set WorldInfo::basicTimeStep to 10." << endl;

  // Hertz value from https://aseba.wdfiles.com/local--files/en:thymioprogram/ThymioCheatSheet.pdf
  mAccelerometerPeriod = hertzToBestMsPeriod(16);  // only sensor not tolerand to the recommended 10ms basic time step
  mProxsPeriod = hertzToBestMsPeriod(10);
  mButtonsPeriod = hertzToBestMsPeriod(20);
  mMotorsPeriod = hertzToBestMsPeriod(100);

  initDevices();
  reset();
  safeStep();
}

Thymio2Model::~Thymio2Model() {
}

void Thymio2Model::reset() {
  mLastMotorPosition[M_LEFT] = 0.0;
  mLastMotorPosition[M_RIGHT] = 0.0;

  for (int i = 0; i < 5; ++i)
    mLastButtonsState[i] = false;

  for (int i = 0; i < 2; ++i) {
    mLastTimerPeriod[i] = 0;
    mTimerStepStart[i] = -1;
  }

  mAutomaticBehaviorAccLeds = true;
  mAutomaticBehaviorButtonLeds = true;
  mAutomaticBehaviorProxLeds = true;
}

bool Thymio2Model::safeStep() {
  if (mAutomaticBehaviorAccLeds)
    behaviorAccLeds();
  if (mAutomaticBehaviorButtonLeds)
    behaviorButtonLeds();
  if (mAutomaticBehaviorProxLeds)
    behaviorProxLeds();

  ++mStepCounter;
  return step(mTimeStep) != -1;
}

void Thymio2Model::initDevices() {
  mAccelerometer = getAccelerometer("acc");
  mAccelerometer->enable(mAccelerometerPeriod);

  for (int i = 0; i < 7; ++i) {
    ostringstream s;
    s << "prox.horizontal." << i;
    mHorizontalProxs[i] = getDistanceSensor(s.str());
    mHorizontalProxs[i]->enable(mProxsPeriod);
  }

  mVerticalProxs[M_LEFT] = getDistanceSensor("prox.ground.0");
  mVerticalProxs[M_RIGHT] = getDistanceSensor("prox.ground.1");
  for (int i = 0; i < 2; ++i)
    mVerticalProxs[i]->enable(mProxsPeriod);

  mMotors[M_LEFT] = getMotor("motor.left");
  mMotors[M_RIGHT] = getMotor("motor.right");
  for (int i = 0; i < 2; ++i)
    mMotors[i]->setPosition(numeric_limits<double>::infinity());

  mPositionSensors[M_LEFT] = getPositionSensor("motor.left.position");
  mPositionSensors[M_RIGHT] = getPositionSensor("motor.right.position");
  for (int i = 0; i < 2; ++i)
    mPositionSensors[i]->enable(mMotorsPeriod);

  mButtons[B_FORWARD] = getTouchSensor("button.forward");
  mButtons[B_RIGHT] = getTouchSensor("button.right");
  mButtons[B_BACKWARD] = getTouchSensor("button.backward");
  mButtons[B_LEFT] = getTouchSensor("button.left");
  mButtons[B_CENTER] = getTouchSensor("button.center");
  for (int i = 0; i < 5; ++i)
    mButtons[i]->enable(mButtonsPeriod);
}

void Thymio2Model::sensorToHub(Thymio2AsebaHub *hub) {
  updateWindowData();

  if (isPeriodicEventFired(mAccelerometerPeriod)) {
    const double *accValues = mAccelerometer->getValues();
    for (int i = 0; i < 3; ++i)
      hub->variables.acc[i] = accValues[i];
  }

  if (isPeriodicEventFired(mProxsPeriod)) {
    std::stringstream wwiMessage;
    for (int i = 0; i < 7; ++i) {
      const double value = mHorizontalProxs[i]->getValue();
      hub->variables.prox[i] = value;
      wwiMessage << int(value) << " ";
    }
    for (int i = 0; i < 2; ++i) {
      // TODO:
      // the following is a already good approximation
      // a more precise solution could be achieved by adding calibrated LightSensors
      // and to set ground_ambiant, and to compute delta as
      // ground_reflected - ground_ambiant
      const double value = mVerticalProxs[i]->getValue();
      hub->variables.ground_reflected[i] = value;
      hub->variables.ground_delta[i] = value;
      wwiMessage << int(value) << " ";
    }

    wwiSendText(wwiMessage.str());
  }

  if (isPeriodicEventFired(mButtonsPeriod))
    for (int i = 0; i < 5; ++i)
      hub->variables.buttons_state[i] = mButtons[i]->getValue() || mWindowData[i];

  if (isPeriodicEventFired(mMotorsPeriod))
    for (int i = 0; i < 2; ++i) {
      double currentPosition = mPositionSensors[i]->getValue();
      double velocity = (currentPosition - mLastMotorPosition[i]) / (0.001 * mTimeStep);
      hub->variables.uind[i] = velocity / VELOCITY_RATIO;
      hub->variables.pwm[i] = -1.6 * hub->variables.uind[i];  // poor approximation
      mLastMotorPosition[i] = currentPosition;
    }
}

static double clamp(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

void Thymio2Model::hubToActuators(const Thymio2AsebaHub *hub) {
  double leftVelocity = VELOCITY_RATIO * clamp(hub->variables.target[0], -500, 500);
  double rightVelocity = VELOCITY_RATIO * clamp(hub->variables.target[1], -500, 500);
  mMotors[M_LEFT]->setVelocity(leftVelocity);
  mMotors[M_RIGHT]->setVelocity(rightVelocity);
}

void Thymio2Model::updateEvents(const Thymio2AsebaHub *hub, bool events[]) {
  // clear all events
  for (int i = 0; i < EVENT_COUNT; ++i)
    events[i] = false;

  // set the appropriate events
  events[EVENT_ACC] = isPeriodicEventFired(mButtonsPeriod);
  events[EVENT_BUTTONS] = isPeriodicEventFired(mButtonsPeriod);
  events[EVENT_MOTOR] = isPeriodicEventFired(mMotorsPeriod);
  events[EVENT_PROX] = isPeriodicEventFired(mProxsPeriod);

  if (events[EVENT_BUTTONS]) {
    for (int i = 0; i < 5; ++i) {
      bool state = mButtons[i]->getValue() || mWindowData[i];
      if (mLastButtonsState[i] != state)
        events[EVENT_B_BACKWARD + i] = true;
      mLastButtonsState[i] = state;
    }
  }

  // timer management
  for (int i = 0; i < 2; i++) {
    if (hub->variables.timers[i] != mLastTimerPeriod[i]) {
      if (hub->variables.timers[i] > 0) {
        mTimerStepStart[i] = mStepCounter;

        // check that the timer period is a multiple of WorldInfo::timeStep
        // but only for small periods (arbitrarily: < 2*timeStep) because after the generated error is less important
        if (hub->variables.timers[i] < 2 * mTimeStep && hub->variables.timers[i] != mTimeStep)
          cout << "In simulation, timer " << i << " period should be above or equal to WorldInfo::timeStep" << endl;
      } else
        mTimerStepStart[i] = -1;

      mLastTimerPeriod[i] = hub->variables.timers[i];
    }

    events[EVENT_TIMER0 + i] = isPeriodicEventFired(hub->variables.timers[i], mTimerStepStart[i]);
  }

  // read the robot window events if any
  events[EVENT_MIC] = mWindowData[5];
  events[EVENT_TAP] = mWindowData[6];
}

void Thymio2Model::updateWindowData() {
  std::string text = wwiReceiveText();
  if (text.compare(0, 10, "mousedown ") == 0) {
    std::string buttonName = text.substr(10);
    if (buttonName.compare("backward") == 0)
      mWindowData[B_BACKWARD] = true;
    else if (buttonName.compare("forward") == 0)
      mWindowData[B_FORWARD] = true;
    else if (buttonName.compare("center") == 0)
      mWindowData[B_CENTER] = true;
    else if (buttonName.compare("right") == 0)
      mWindowData[B_RIGHT] = true;
    else if (buttonName.compare("left") == 0)
      mWindowData[B_LEFT] = true;
    else if (buttonName.compare("clap") == 0)
      mWindowData[5] = true;
    else if (buttonName.compare("tap") == 0)
      mWindowData[6] = true;
  } else if (text.compare("mouseup") == 0) {
    for (int i = 0; i < 7; ++i)
      mWindowData[i] = false;
  }
}

void Thymio2Model::behaviorButtonLeds() {
  // cf. https://github.com/aseba-community/aseba-target-thymio2/blob/master/behavior.c

  static unsigned int counter[5];
  int i;
  for (i = 0; i < 5; ++i) {
    if (mButtons[i]->getValue()) {
      counter[i] += 3;
      if (counter[i] > 32)
        counter[i] = 32;
    } else
      counter[i] = 0;
  }

  if (counter[2]) {
    thymio2->getLED("leds.buttons.led0")->set(counter[2]);
    thymio2->getLED("leds.buttons.led1")->set(counter[2]);
    thymio2->getLED("leds.buttons.led2")->set(counter[2]);
    thymio2->getLED("leds.buttons.led3")->set(counter[2]);
  } else {
    thymio2->getLED("leds.buttons.led0")->set(counter[3]);
    thymio2->getLED("leds.buttons.led1")->set(counter[4]);
    thymio2->getLED("leds.buttons.led2")->set(counter[0]);
    thymio2->getLED("leds.buttons.led3")->set(counter[1]);
  }

  // Note: removed the multi touch behavior: not possible in Webots
}

void Thymio2Model::behaviorProxLeds() {
  // cf. https://github.com/aseba-community/aseba-target-thymio2/blob/master/behavior.c

  static int max[9] = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 900, 900};
  static int min[9] = {1200, 1200, 1200, 1200, 1200, 1200, 1200, 0, 0};
  static string led[10] = {"leds.prox.h.led0", "leds.prox.h.led1", "leds.prox.h.led2", "leds.prox.h.led3", "leds.prox.h.led4",
                           "leds.prox.h.led5", "leds.prox.h.led6", "leds.prox.h.led7", "leds.prox.v.led0", "leds.prox.v.led1"};
  int i;
  for (i = 0; i < 7; ++i) {
    if (max[i] < mHorizontalProxs[i]->getValue())
      max[i] = mHorizontalProxs[i]->getValue();
    if (mHorizontalProxs[i]->getValue() != 0 && min[i] > mHorizontalProxs[i]->getValue())
      min[i] = mHorizontalProxs[i]->getValue();
  }
  for (i = 0; i < 2; ++i) {
    if (max[i + 7] < mVerticalProxs[i]->getValue())
      max[i + 7] = mVerticalProxs[i]->getValue();
  }

  for (i = 0; i < 7; ++i) {
    int s = mHorizontalProxs[i]->getValue() - min[i];
    int d = max[i] - min[i];

    if (s < 0)
      s = 0;

    int b = d ? (s * 32 / d) : 0;

    if (i == 2) {
      getLED(led[i])->set(b);
      getLED(led[i + 1])->set(b);
    } else if (i > 2)
      getLED(led[i + 1])->set(b);
    else
      getLED(led[i])->set(b);
  }

  for (i = 0; i < 2; ++i) {
    int s = mVerticalProxs[i]->getValue() > 0 ? mVerticalProxs[i]->getValue() : 0;
    int b = s * 32 / max[i + 7];
    getLED(led[i + 8])->set(b);
  }
}

void Thymio2Model::behaviorAccLeds() {
  // cf. https://github.com/aseba-community/aseba-target-thymio2/blob/master/behavior.c

  static string previous_led = "";

  string led = "";

  if (mAccelerometer->getValues()[2] < 21) {
    static double M_PI_8 = M_PI / 8.0;
    double ha = atan2(mAccelerometer->getValues()[0], mAccelerometer->getValues()[1]);
    if (ha >= -M_PI_8 && ha < M_PI_8)
      led = "leds.circle.led4";
    else if (ha < -M_PI_8 && ha >= -3.0 * M_PI_8)
      led = "leds.circle.led3";
    else if (ha < -3.0 * M_PI_8 && ha >= -5.0 * M_PI_8)
      led = "leds.circle.led2";
    else if (ha < -5.0 * M_PI_8 && ha >= -7.0 * M_PI_8)
      led = "leds.circle.led1";
    else if (ha < -7.0 * M_PI_8 || ha >= 7.0 * M_PI_8)
      led = "leds.circle.led0";
    else if (ha < 3.0 * M_PI_8 && ha >= M_PI_8)
      led = "leds.circle.led5";
    else if (ha < 5.0 * M_PI_8 && ha >= 3.0 * M_PI_8)
      led = "leds.circle.led6";
    else if (ha < 7.0 * M_PI_8 && ha >= 5.0 * M_PI_8)
      led = "leds.circle.led7";

    int intensity = 40 - abs(mAccelerometer->getValues()[2]) * 2;
    if (intensity < 0)
      intensity = 0;
    else if (intensity > 32)
      intensity = 32;

    if (!led.empty()) {
      // cppcheck-suppress knownConditionTrueFalse
      if (!previous_led.empty())
        getLED(previous_led)->set(0);
      getLED(led)->set(intensity);
    }
    previous_led = led;
  } else {
    // cppcheck-suppress knownConditionTrueFalse
    if (!previous_led.empty())
      getLED(previous_led)->set(0);

    previous_led = "";
  }
}

int Thymio2Model::hertzToBestMsPeriod(int hertz) const {
  return mTimeStep * (1000 / hertz / mTimeStep);
}

bool Thymio2Model::isPeriodicEventFired(int period, int startCounter) const {
  if (period > 0) {
    if (period < mTimeStep)
      // Webots cannot go quicker. time constraints are not fullfilled.
      return true;
    else {
      // Hypothesis: Webots is modifying the sensors according to the following rule:
      bool fireRequired((mStepCounter - startCounter) % (period / mTimeStep) == 0);

      // for the timer case, don't fire the event directly, but wait at least on the next step
      if (startCounter > 0 && mStepCounter == startCounter)
        return false;

      return fireRequired;
    }
  }
  return false;
}

static int convertRGBColorToWebotsColor(int r, int g, int b, int scale) {
  return ((0xFF & (r * 0xFF / scale)) << 16) + ((0xFF & (g * 0xFF / scale)) << 8) + (0xFF & (b * 0xFF / scale));
}

extern "C" {
#include <vm/natives.h>

// cf. https://github.com/aseba-community/aseba-target-thymio2/blob/master/thymio_natives.c

void set_rgb_top(AsebaVMState *vm) {
  int r = vm->variables[AsebaNativePopArg(vm)];
  int g = vm->variables[AsebaNativePopArg(vm)];
  int b = vm->variables[AsebaNativePopArg(vm)];

  thymio2->getLED("leds.top")->set(convertRGBColorToWebotsColor(r, g, b, 32));
}

void set_led(AsebaVMState *vm) {
  /*int led = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
  /*int b = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;

  // This function is not accessible from the user
}

void set_led_circle(AsebaVMState *vm) {
  int l0 = vm->variables[AsebaNativePopArg(vm)];
  int l1 = vm->variables[AsebaNativePopArg(vm)];
  int l2 = vm->variables[AsebaNativePopArg(vm)];
  int l3 = vm->variables[AsebaNativePopArg(vm)];
  int l4 = vm->variables[AsebaNativePopArg(vm)];
  int l5 = vm->variables[AsebaNativePopArg(vm)];
  int l6 = vm->variables[AsebaNativePopArg(vm)];
  int l7 = vm->variables[AsebaNativePopArg(vm)];

  thymio2->setAutomaticBehaviorAccLeds(false);

  thymio2->getLED("leds.circle.led0")->set(l0);
  thymio2->getLED("leds.circle.led1")->set(l1);
  thymio2->getLED("leds.circle.led2")->set(l2);
  thymio2->getLED("leds.circle.led3")->set(l3);
  thymio2->getLED("leds.circle.led4")->set(l4);
  thymio2->getLED("leds.circle.led5")->set(l5);
  thymio2->getLED("leds.circle.led6")->set(l6);
  thymio2->getLED("leds.circle.led7")->set(l7);
}

void set_rgb_br(AsebaVMState *vm) {
  int r = vm->variables[AsebaNativePopArg(vm)];
  int g = vm->variables[AsebaNativePopArg(vm)];
  int b = vm->variables[AsebaNativePopArg(vm)];

  thymio2->getLED("leds.bottom.right")->set(convertRGBColorToWebotsColor(r, g, b, 32));
}

void set_rgb_bl(AsebaVMState *vm) {
  int r = vm->variables[AsebaNativePopArg(vm)];
  int g = vm->variables[AsebaNativePopArg(vm)];
  int b = vm->variables[AsebaNativePopArg(vm)];

  thymio2->getLED("leds.bottom.left")->set(convertRGBColorToWebotsColor(r, g, b, 32));
}

void set_buttons_leds(AsebaVMState *vm) {
  int l0 = vm->variables[AsebaNativePopArg(vm)];
  int l1 = vm->variables[AsebaNativePopArg(vm)];
  int l2 = vm->variables[AsebaNativePopArg(vm)];
  int l3 = vm->variables[AsebaNativePopArg(vm)];

  thymio2->setAutomaticBehaviorButtonLeds(false);

  thymio2->getLED("leds.buttons.led0")->set(l0);
  thymio2->getLED("leds.buttons.led1")->set(l1);
  thymio2->getLED("leds.buttons.led2")->set(l2);
  thymio2->getLED("leds.buttons.led3")->set(l3);
}

void set_hprox_leds(AsebaVMState *vm) {
  int l0 = vm->variables[AsebaNativePopArg(vm)];
  int l1 = vm->variables[AsebaNativePopArg(vm)];
  int l2 = vm->variables[AsebaNativePopArg(vm)];
  int l3 = vm->variables[AsebaNativePopArg(vm)];
  int l4 = vm->variables[AsebaNativePopArg(vm)];
  int l5 = vm->variables[AsebaNativePopArg(vm)];
  int l6 = vm->variables[AsebaNativePopArg(vm)];
  int l7 = vm->variables[AsebaNativePopArg(vm)];

  thymio2->setAutomaticBehaviorProxLeds(false);

  thymio2->getLED("leds.prox.h.led0")->set(l0);
  thymio2->getLED("leds.prox.h.led1")->set(l1);
  thymio2->getLED("leds.prox.h.led2")->set(l2);
  thymio2->getLED("leds.prox.h.led3")->set(l3);
  thymio2->getLED("leds.prox.h.led4")->set(l4);
  thymio2->getLED("leds.prox.h.led5")->set(l5);
  thymio2->getLED("leds.prox.h.led6")->set(l6);
  thymio2->getLED("leds.prox.h.led7")->set(l7);
}

void set_vprox_leds(AsebaVMState *vm) {
  int l0 = vm->variables[AsebaNativePopArg(vm)];
  int l1 = vm->variables[AsebaNativePopArg(vm)];

  thymio2->setAutomaticBehaviorProxLeds(false);

  thymio2->getLED("leds.prox.v.led0")->set(l0);
  thymio2->getLED("leds.prox.v.led1")->set(l1);
}

void set_rc_leds(AsebaVMState *vm) {
  int l = vm->variables[AsebaNativePopArg(vm)];
  thymio2->getLED("leds.rc")->set(l);
}

void set_sound_leds(AsebaVMState *vm) {
  int l = vm->variables[AsebaNativePopArg(vm)];
  thymio2->getLED("leds.sound")->set(l);
}

void set_ntc_leds(AsebaVMState *vm) {
  int l0 = vm->variables[AsebaNativePopArg(vm)];
  int l1 = vm->variables[AsebaNativePopArg(vm)];
  thymio2->getLED("leds.temperature.red")->set(l0);
  thymio2->getLED("leds.temperature.blue")->set(l1);
}
}
