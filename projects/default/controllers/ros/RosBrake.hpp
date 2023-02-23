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

#ifndef ROS_BRAKE_HPP
#define ROS_BRAKE_HPP

#include <webots/Brake.hpp>
#include "RosDevice.hpp"

#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>
#include <webots_ros/set_float.h>

using namespace webots;

class RosBrake : public RosDevice {
public:
  RosBrake(Brake *brake, Ros *ros);
  virtual ~RosBrake();

  bool setDampingConstantCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool getTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getMotorNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getPositionSensorNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);

private:
  Brake *mBrake;
  ros::ServiceServer mSetDampingConstantServer;
  ros::ServiceServer mGetTypeServer;
  ros::ServiceServer mGetMotorNameServer;
  ros::ServiceServer mGetPositionSensorNameServer;
  ros::Publisher mPublisher;
};

#endif  // ROS_BRAKE_HPP
