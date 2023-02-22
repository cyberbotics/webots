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
/* Description:  Wrapper of the PositionSensor Webots API for the ROBOTIS OP2 real robot                 */
/*******************************************************************************************************/

#ifndef POSITION_SENSOR_HPP
#define POSITION_SENSOR_HPP

#include <map>
#include <webots/Device.hpp>
#include <webots/Robot.hpp>

namespace webots {
  class PositionSensor : public Device {
  public:
    enum { ROTATIONAL = 0 };

    PositionSensor(const std::string &name);  // Use Robot::getMotor() instead
    virtual ~PositionSensor();

    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    double getValue() const;

    int getType() const;

  private:
    static void initStaticMap();
    static std::map<const std::string, int> mNamesToIDs;
    static std::map<const std::string, int> mNamesToInitPos;

    void setPresentPosition(int position);

    // For Bulk Read //
    int mPresentPosition;

    int mFeedback;

    friend int Robot::step(int duration);
    friend Robot::Robot();
  };
}  // namespace webots

#endif  // POSITION_SENSOR_HPP
