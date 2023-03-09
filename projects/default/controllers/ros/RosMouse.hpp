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

#ifndef ROS_MOUSE_HPP
#define ROS_MOUSE_HPP

#include <webots/Mouse.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_bool.h>
#include <webots_ros/mouse_get_state.h>
#include <webots_ros/set_bool.h>

using namespace webots;

class RosMouse : public RosSensor {
public:
  RosMouse(Mouse *mouse, Ros *ros);
  virtual ~RosMouse();

  void rosEnable(int samplingPeriod) override { mMouse->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mMouse->getSamplingPeriod(); }

  bool getStateCallback(webots_ros::mouse_get_state::Request &req, webots_ros::mouse_get_state::Response &res);
  bool enable3dPositionCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool disable3dPositionCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool is3dPositionEnabledCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);

private:
  void cleanup() { mMouse->disable(); }

  Mouse *mMouse;

  ros::ServiceServer mGetStateServer;
  ros::ServiceServer mEnable3dPositionServer;
  ros::ServiceServer mDisable3dPositionServer;
  ros::ServiceServer mIs3dPositionEnabledServer;
};

#endif  // ROS_MOUSE_HPP
