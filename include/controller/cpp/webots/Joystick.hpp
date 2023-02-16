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

#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <string>

namespace webots {
  class Joystick {
  public:
    Joystick() {}  // Use Robot::getJoystick() instead
    virtual ~Joystick() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    bool isConnected() const;
    std::string getModel() const;
    int getNumberOfAxes() const;
    int getAxisValue(int axis) const;
    int getNumberOfPovs() const;
    int getPovValue(int pov) const;
    int getPressedButton() const;
    void setConstantForce(int level);
    void setConstantForceDuration(double duration);
    void setAutoCenteringGain(double gain);
    void setResistanceGain(double gain);
    void setForceAxis(int axis);
  };
}  // namespace webots

#endif  // JOYSTICK_HPP
