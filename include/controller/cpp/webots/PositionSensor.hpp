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

#ifndef POSITION_SENSOR_HPP
#define POSITION_SENSOR_HPP

#include <webots/Device.hpp>

namespace webots {
  class Brake;
  class Motor;

  class PositionSensor : public Device {
  public:
    typedef enum { ROTATIONAL = 0, LINEAR } Type;

    explicit PositionSensor(const std::string &name) :
      Device(name),
      brake(NULL),
      motor(NULL) {}  // Use Robot::getPositionSensor() instead
    explicit PositionSensor(WbDeviceTag tag) : Device(tag), brake(NULL), motor(NULL) {}
    virtual ~PositionSensor() {}
    virtual void enable(int samplingPeriod);  // milliseconds
    virtual void disable();
    int getSamplingPeriod() const;
    double getValue() const;  // rad or meters
    Type getType() const;

    Brake *getBrake();
    Motor *getMotor();

    // internal functions
    int getBrakeTag() const;
    int getMotorTag() const;

    enum {  // kept for backward compatibility R2018b
      ANGULAR = 0
    };

  private:
    Brake *brake;
    Motor *motor;
  };
}  // namespace webots

#endif  // POSITION_SENSOR_HPP
