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

#include "EPuckCommandPacket.hpp"

#include "Camera.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

EPuckCommandPacket::EPuckCommandPacket() :
  mDistanceSensorRequested(false),
  mGroundSensorRequested(false),
  mLightSensorRequested(false),
  mAccelerometerRequested(false),
  mEncoderRequested(false) {
  for (size_t i = 0; i < sizeof(mData); i++)
    mData[i] = 0;
}

EPuckCommandPacket::~EPuckCommandPacket() {
}

void EPuckCommandPacket::clear() {
  for (size_t i = 0; i < sizeof(mData); i++)
    mData[i] = 0;
  mDistanceSensorRequested = false;
  mGroundSensorRequested = false;
  mLightSensorRequested = false;
  mAccelerometerRequested = false;
  mEncoderRequested = false;
}

int EPuckCommandPacket::apply(int simulationTime) {
  mData[0] = 0x80;  // commands packet
  mData[1] = DeviceManager::instance()->camera()->isEnabled() ? 1 : 0;
  mData[2] = 0;                                                         // send the motor commands (velocity)
  short l = DeviceManager::instance()->motor(0)->velocity() / 0.00628;  // 0.00628 = ( 2 * pi) / encoder_resolution
  short r = DeviceManager::instance()->motor(1)->velocity() / 0.00628;
  if (l > 1000)
    l = 1000;
  else if (l < -1000)
    l = -1000;
  if (r > 1000)
    r = 1000;
  else if (r < -1000)
    r = -1000;
  mData[3] = l & 0xff;
  mData[4] = ((unsigned short)l >> 8) & 0xff;
  mData[5] = r & 0xff;
  mData[6] = ((unsigned short)r >> 8) & 0xff;
  // send the led commands
  mData[7] = 0;
  for (int i = 0; i < 10; i++) {
    Led *led = DeviceManager::instance()->led(i);
    const char d = mData[7];
    const int state = led->state();
    switch (led->index()) {
      case 0:  // LED1
        mData[7] = state ? d | 0b00000001 : d & 0b11111110;
        break;
      case 1:  // LED2
        mData[8] = state & 0xff;
        mData[9] = ((unsigned int)state >> 8) & 0xff;
        mData[10] = ((unsigned int)state >> 16) & 0xff;
        break;
      case 2:  // LED3
        mData[7] = state ? d | 0b00000010 : d & 0b11111101;
        break;
      case 3:  // LED4
        mData[11] = state & 0xff;
        mData[12] = ((unsigned int)state >> 8) & 0xff;
        mData[13] = ((unsigned int)state >> 16) & 0xff;
        break;
      case 4:  // LED5
        mData[7] = state ? d | 0b00000100 : d & 0b11111011;
        break;
      case 5:  // LED6
        mData[14] = state & 0xff;
        mData[15] = ((unsigned int)state >> 8) & 0xff;
        mData[16] = ((unsigned int)state >> 16) & 0xff;
        break;
      case 6:  // LED7
        mData[7] = state ? d | 0b00001000 : d & 0b11110111;
        break;
      case 7:  // LED8
        mData[16] = state & 0xff;
        mData[18] = ((unsigned int)state >> 8) & 0xff;
        mData[19] = ((unsigned int)state >> 16) & 0xff;
        break;
      case 8:  // body LED
        mData[7] = state ? d | 0b00010000 : d & 0b11101111;
        break;
      case 9:  // front LED
        mData[7] = state ? d | 0x00100000 : d & 0b11011111;
        break;
    }
  }
  mData[20] = 0x00;  // don't play any sound
  // Sensors
  if (DeviceManager::instance()->accelerometer()->isEnabled()) {
    mAccelerometerRequested = true;
    mData[1] |= 2;  // enable sensor packet
  }
  for (int i = 0; i < 3; i++) {  // optional ground sensors
    const SingleValueSensor *gs = DeviceManager::instance()->groundSensor(i);
    if (gs && gs->isEnabled()) {
      mGroundSensorRequested = true;  // if at least one sensor is required
      mData[1] |= 2;                  // enable sensor packet
      break;
    }
  }
  for (int i = 0; i < 8; i++) {
    if (DeviceManager::instance()->distanceSensor(i)->isEnabled()) {
      mDistanceSensorRequested = true;
      mData[1] |= 2;  // enable sensor packet
      break;
    }
  }
  for (int i = 0; i < 8; i++) {
    if (DeviceManager::instance()->lightSensor(i)->isEnabled()) {
      mLightSensorRequested = true;
      mData[1] |= 2;  // enable packet sensor
      break;
    }
  }
  if (DeviceManager::instance()->positionSensor(0)->isEnabled() || DeviceManager::instance()->positionSensor(1)->isEnabled()) {
    mEncoderRequested = true;
    // FIXME uncomment when next assignment is fixed
    // mData[1] |= 2;  // enable packet sensor
  }
  mData[1] = 3;  // FIXME: remove this
  return mData[1];
}
