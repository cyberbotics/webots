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
/* Description:  Wrapper of the LED Webots API for the ROBOTIS OP2 real robot                            */
/*******************************************************************************************************/

#ifndef LED_HPP
#define LED_HPP

#include <webots/Device.hpp>
#include <webots/Robot.hpp>

#include <map>

namespace webots {
  class LED : public Device {
  public:
    LED(const std::string &name);  // Use Robot::getLED() instead
    virtual ~LED();
    virtual void set(int value);
    int get() const;

  private:
    static void initStaticMap();
    static void setBackPanel(int state);
    static int mBackPanel;

    static std::map<const std::string, int> mNamesToIDs;

    void setColor(int color);

    int mColor;

    friend int Robot::step(int samplingPeriod);
  };
}  // namespace webots

#endif  // LED_HPP
