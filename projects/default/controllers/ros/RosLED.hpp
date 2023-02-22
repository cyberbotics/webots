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

#ifndef ROS_LED_HPP
#define ROS_LED_HPP

#include <webots/LED.hpp>
#include "RosDevice.hpp"

#include <webots_ros/get_int.h>
#include <webots_ros/set_int.h>

using namespace webots;

class RosLED : public RosDevice {
public:
  RosLED(LED *led, Ros *ros);
  virtual ~RosLED();

  bool setCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool getCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);

private:
  LED *mLED;
  ros::ServiceServer mSetServer;
  ros::ServiceServer mGetServer;
};

#endif  // ROS_LED_HPP
