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

#ifndef DISTANCESENSOR_HPP
#define DISTANCESENSOR_HPP

#include <webots/Device.hpp>

namespace webots {
  class DistanceSensor : public Device {
  public:
    typedef enum { GENERIC = 0, INFRA_RED, SONAR, LASER } Type;

    explicit DistanceSensor(const std::string &name) : Device(name) {}  // Use Robot::getDistanceSensor() instead
    virtual ~DistanceSensor() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    double getValue() const;
    double getMaxValue() const;
    double getMinValue() const;
    double getAperture() const;
    int getLookupTableSize() const;
    const double *getLookupTable() const;
    Type getType() const;
  };
}  // namespace webots

#endif  // DISTANCESENSOR_HPP
