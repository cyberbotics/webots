// Copyright 1996-2021 Cyberbotics Ltd.
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

/*
 * Description:  Defines an interface wrapping the libController with this library for the ROBOTIS OP2
 */

#ifndef WRAPPER_HPP
#define WRAPPER_HPP

#include <webots/types.h>

class Communication;
class Time;

class Wrapper {
public:
  // init
  static void init();
  static void cleanup();

  // mandatory functions
  static bool start(const char *);
  static void stop();
  static bool hasFailed() { return !cSuccess; }
  static int robotStep(int);
  static void stopActuators();

  // redefined functions
  static void setSamplingPeriod(WbDeviceTag tag, int samplingPeriod);
  static void ledSet(WbDeviceTag tag, int state);
  static void motorSetPosition(WbDeviceTag tag, double position);
  static void motorSetVelocity(WbDeviceTag tag, double velocity);
  static void motorSetAcceleration(WbDeviceTag tag, double acceleration);
  static void motorSetAvailableTorque(WbDeviceTag tag, double torque);
  static void motorSetTorque(WbDeviceTag tag, double torque);
  static void motorSetControlPID(WbDeviceTag tag, double p, double i, double d);

  // unimplemented required functions
  static void cameraSetFOV(WbDeviceTag tag, double fov) {}

private:
  Wrapper() {}
  ~Wrapper() {}

  static Communication *cCommunication;
  static Time *cTime;
  static bool cSuccess;
};

#endif
