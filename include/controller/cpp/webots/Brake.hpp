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

#ifndef BRAKE_HPP
#define BRAKE_HPP

#include <stdio.h>
#include <webots/Device.hpp>

namespace webots {
  class Motor;
  class PositionSensor;

  class Brake : public Device {
  public:
    typedef enum { ROTATIONAL = 0, LINEAR } Type;

    explicit Brake(const std::string &name) :
      Device(name),
      motor(NULL),
      positionSensor(NULL) {}  // Use Robot::getBrake() instead
    explicit Brake(WbDeviceTag tag) : Device(tag), motor(NULL), positionSensor(NULL) {}
    virtual ~Brake() {}
    Type getType() const;
    void setDampingConstant(double dampingConstant) const;

    Motor *getMotor();
    PositionSensor *getPositionSensor();

    // internal functions
    int getMotorTag() const;
    int getPositionSensorTag() const;

    enum {  // kept for backward compatibility R2018b
      ANGULAR = 0
    };

  private:
    Motor *motor;
    PositionSensor *positionSensor;
  };
}  // namespace webots

#endif  // BRAKE_HPP
