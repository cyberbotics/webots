// Copyright 1996-2018 Cyberbotics Ltd.
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

#ifndef DIFFERENTIAL_WHEELS_HPP
#define DIFFERENTIAL_WHEELS_HPP

#include <webots/Robot.hpp>

namespace webots {
  class DifferentialWheels : public Robot {
  public:
    explicit DifferentialWheels() : Robot() {}
    virtual ~DifferentialWheels() {}

    virtual void setSpeed(double left, double right);
    double getLeftSpeed() const;
    double getRightSpeed() const;
    double getMaxSpeed() const;
    double getSpeedUnit() const;

    virtual void enableEncoders(int samplingPeriod);
    virtual void disableEncoders();
    int getEncodersSamplingPeriod() const;
    double getLeftEncoder() const;
    double getRightEncoder() const;
    virtual void setEncoders(double left, double right);
  };
}  // namespace webots

#endif  // DIFFERENTIAL_WHEELS_HPP
