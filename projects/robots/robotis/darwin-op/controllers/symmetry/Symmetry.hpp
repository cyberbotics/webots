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

// Description:   Sample showing how to use the motors and to cross-compile
//                for the robotis-op2

#ifndef SYMMETRY_HPP
#define SYMMETRY_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace webots {
  class Motor;
  class LED;
  class PositionSensor;
};  // namespace webots

class Symmetry : public webots::Robot {
public:
  Symmetry();
  virtual ~Symmetry();
  void run();

private:
  int mTimeStep;

  void myStep();
  void wait(int ms);

  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::LED *mEyeLED;
  webots::LED *mHeadLED;
};

#endif
