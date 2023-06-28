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

#ifndef VACUUM_GRIPPER_HPP
#define VACUUM_GRIPPER_HPP

#include <webots/Device.hpp>

namespace webots {
  class VacuumGripper : public Device {
  public:
    explicit VacuumGripper(const std::string &name) : Device(name) {}  // Use Robot::getVacuumGripper() instead
    virtual ~VacuumGripper() {}
    virtual void enablePresence(int samplingPeriod);
    virtual void disablePresence();
    int getPresenceSamplingPeriod() const;
    bool getPresence() const;
    bool isOn() const;
    virtual void turnOn();
    virtual void turnOff();
  };
}  // namespace webots

#endif  // VACUUM_GRIPPER_HPP
