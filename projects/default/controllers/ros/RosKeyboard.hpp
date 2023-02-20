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

#ifndef ROS_KEYBOARD_HPP
#define ROS_KEYBOARD_HPP

#include <webots/Keyboard.hpp>
#include "RosSensor.hpp"

using namespace webots;

class RosKeyboard : public RosSensor {
public:
  RosKeyboard(Keyboard *keyboard, Ros *ros);
  virtual ~RosKeyboard() { cleanup(); }

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mKeyboard->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mKeyboard->getSamplingPeriod(); }

private:
  void cleanup() { mKeyboard->disable(); }

  Keyboard *mKeyboard;
};

#endif  // ROS_KEYBOARD_HPP
