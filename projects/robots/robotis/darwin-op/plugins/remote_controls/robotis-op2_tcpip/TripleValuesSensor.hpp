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
 * Description:  Abstraction of a sensor returning three floating point value
 */

#ifndef TRIPLE_VALUES_SENSOR_HPP
#define TRIPLE_VALUES_SENSOR_HPP

#include "Sensor.hpp"

class TripleValuesSensor : public Sensor {
public:
  // Device Manager is responsible to create/destroy devices
  TripleValuesSensor(WbDeviceTag tag, int index) : Sensor(tag, index) {
    mValues[0] = 0.0;
    mValues[1] = 0.0;
    mValues[2] = 0.0;
  }
  virtual ~TripleValuesSensor() {}

  const double *values() const { return mValues; }
  void setValues(double v0, double v1, double v2) {
    mValues[0] = v0;
    mValues[1] = v1;
    mValues[2] = v2;
  }

private:
  double mValues[3];
};

#endif
