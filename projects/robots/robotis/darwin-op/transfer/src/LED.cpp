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

#include <webots/LED.hpp>
#include <webots/Robot.hpp>

#include <CM730.h>

using namespace webots;
using namespace Robot;

std::map<const std::string, int> LED::mNamesToIDs;
int LED::mBackPanel = 0;

#define LED_BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

LED::LED(const std::string &name) : Device(name) {
  initStaticMap();
}

LED::~LED() {
}

void LED::set(int value) {
  CM730 *cm730 = Robot::getInstance()->getCM730();

  // RGB Led
  if (getName() == "EyeLed" || getName() == "HeadLed") {
    // convert RGB-24bits to RGB-15bits
    int r5 = ((value >> 16) & 0xFF) >> 3;
    int g5 = ((value >> 8) & 0xFF) >> 3;
    int b5 = (value & 0xFF) >> 3;
    int rgb15 = (b5 << 10) | (g5 << 5) | r5;

    // set the led value (only if the color is different from actual color)
    if (mColor != rgb15)
      cm730->WriteWord(mNamesToIDs[getName()], rgb15, 0);
  }

  // BackPannel Led
  else if (getName() == "BackLedRed") {
    if (value == 1 && (0x01 | mBackPanel) != mBackPanel)  // Switch selected led on without changing the two others
      cm730->WriteByte(CM730::P_LED_PANNEL, 0x01 | mBackPanel, NULL);
    else if ((0x06 & mBackPanel) != mBackPanel)  // Switch selected led off without changing the two others
      cm730->WriteByte(CM730::P_LED_PANNEL, 0x06 & mBackPanel, NULL);
  } else if (getName() == "BackLedBlue") {
    if (value == 1 && (0x02 | mBackPanel) != mBackPanel)  // Switch selected led on without changing the two others
      cm730->WriteByte(CM730::P_LED_PANNEL, 0x02 | mBackPanel, NULL);
    else if ((0x05 & mBackPanel) != mBackPanel)  // Switch selected led off without changing the two others
      cm730->WriteByte(CM730::P_LED_PANNEL, 0x05 & mBackPanel, NULL);
  } else if (getName() == "BackLedGreen") {
    if (value == 1 && (0x04 | mBackPanel) != mBackPanel)  // Switch selected led on without changing the two others
      cm730->WriteByte(CM730::P_LED_PANNEL, 0x04 | mBackPanel, NULL);
    else if ((0x03 & mBackPanel) != mBackPanel)  // Switch selected led off without changing the two others
      cm730->WriteByte(CM730::P_LED_PANNEL, 0x03 & mBackPanel, NULL);
  }
}

void LED::initStaticMap() {
  static bool firstCall = true;
  if (firstCall) {
    firstCall = false;

    mNamesToIDs["EyeLed"] = CM730::P_LED_EYE_L;
    mNamesToIDs["HeadLed"] = CM730::P_LED_HEAD_L;
  }
}

void LED::setColor(int color) {
  mColor = color;
}

void LED::setBackPanel(int state) {
  mBackPanel = state;
}

int LED::get() const {
  if (getName() == "EyeLed" || getName() == "HeadLed")
    return mColor;
  else if (getName() == "BackLedRed")
    return (0x01 & mBackPanel);
  else if (getName() == "BackLedBlue")
    return (0x02 & mBackPanel);
  else if (getName() == "BackLedGreen")
    return (0x04 & mBackPanel);
  return 0;
}
