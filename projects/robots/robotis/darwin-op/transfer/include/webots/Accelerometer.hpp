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

/*******************************************************************************************************/
/* Description:  Wrapper of the Accelerometer Webots API for the ROBOTIS OP2 real robot                  */
/*******************************************************************************************************/

#ifndef ACCELEROMETER_HPP
#define ACCELEROMETER_HPP

#include <webots/Device.hpp>
#include <webots/Robot.hpp>

namespace webots {
  class Accelerometer : public Device {
  public:
    Accelerometer(const std::string &name);  // Use Robot::getAccelerometer() instead
    virtual ~Accelerometer();

    virtual void enable(int samplingPeriod);
    virtual void disable();

    const double *getValues() const;
    int getSamplingPeriod() const;

  private:
    double mValues[3];
    void setValues(const int *integerValues);

    friend int Robot::step(int duration);
  };
}  // namespace webots

#endif  // ACCELEROMETER_HPP
