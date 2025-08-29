// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef LIGHTSENSOR_HPP
#define LIGHTSENSOR_HPP

#include <webots/Device.hpp>

namespace webots {
  class LightSensor : public Device {
  public:
    explicit LightSensor(const std::string &name) : Device(name) {}  // Use Robot::getLightSensor() instead
    explicit LightSensor(WbDeviceTag tag) : Device(tag) {}
    virtual ~LightSensor() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    double getValue() const;
    int getLookupTableSize() const;
    const double *getLookupTable() const;
  };
}  // namespace webots

#endif  // LIGHTSENSOR_HPP
