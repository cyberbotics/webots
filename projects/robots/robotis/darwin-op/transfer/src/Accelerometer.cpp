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

#include <webots/Accelerometer.hpp>
#include <webots/Robot.hpp>

using namespace webots;

Accelerometer::Accelerometer(const std::string &name) : Device(name) {
  for (int i = 0; i < 3; i++)
    mValues[i] = 512;  // 512 = central value -> no acceleration
}

Accelerometer::~Accelerometer() {
}

void Accelerometer::enable(int samplingPeriod) {
}

void Accelerometer::disable() {
}

const double *Accelerometer::getValues() const {
  return mValues;
}

void Accelerometer::setValues(const int *integerValues) {
  for (int i = 0; i < 3; i++)
    mValues[i] = integerValues[i];
}

int Accelerometer::getSamplingPeriod() const {
  return Robot::getInstance()->getBasicTimeStep();
}
