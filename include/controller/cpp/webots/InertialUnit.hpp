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

#ifndef INERTIAL_UNIT_HPP
#define INERTIAL_UNIT_HPP

#include <webots/Device.hpp>

namespace webots {
  class InertialUnit : public Device {
  public:
    explicit InertialUnit(const std::string &name) : Device(name) {}  // Use Robot::getInertialUnit() instead
    virtual ~InertialUnit() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    const double *getRollPitchYaw() const;
    const double *getQuaternion() const;
    double getNoise() const;
  };
}  // namespace webots

#endif  // INERTIAL_UNIT_HPP
