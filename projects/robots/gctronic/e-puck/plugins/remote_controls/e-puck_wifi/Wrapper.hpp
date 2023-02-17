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

/*
 * Description:  Defines an interface wrapping the libController with this library
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
  static void motorSetVelocity(WbDeviceTag tag, double velocity);
  static void ledSet(WbDeviceTag tag, int state);
  static void motorSetTorqueSamplingPeriod(WbDeviceTag tag, int samplingPeriod) {}

  static void *callCustomFunction(void *args);

private:
  Wrapper() {}
  ~Wrapper() {}
  static Communication *cCommunication;
  static Time *cTime;
  static bool cSuccess;
  static bool cCameraInitialized;
  static int *cUploadReturnValue;
};

#endif
