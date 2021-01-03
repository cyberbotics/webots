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

#include <webots/Gyro.hpp>
#include <webots/Robot.hpp>

using namespace webots;

Gyro::Gyro(const std::string &name) : Device(name) {
  for (int i = 0; i < 3; i++)
    mValues[i] = 512;  // 512 = central value -> no rotation
}

Gyro::~Gyro() {
}

void Gyro::enable(int samplingPeriod) {
}

void Gyro::disable() {
}

const double *Gyro::getValues() const {
  return mValues;
}

void Gyro::setValues(const int *integerValues) {
  for (int i = 0; i < 3; i++)
    mValues[i] = integerValues[i];
}

int Gyro::getSamplingPeriod() const {
  return Robot::getInstance()->getBasicTimeStep();
}
