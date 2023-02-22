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

#ifndef ROS_JOYSTICK_HPP
#define ROS_JOYSTICK_HPP

#include <webots/Joystick.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_bool.h>
#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

using namespace webots;

class RosJoystick : public RosSensor {
public:
  RosJoystick(Joystick *joystick, Ros *ros);
  virtual ~RosJoystick();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void publishAuxiliaryValue() override;
  bool getModelCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getNumberOfAxesCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getNumberOfPovsCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool isConnectedCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool setConstantForceCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool setConstantForceDurationCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setAutoCenteringCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setResistanceGainCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setForceAxisCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);

  void rosEnable(int samplingPeriod) override { mJoystick->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mJoystick->getSamplingPeriod(); }

private:
  void publishAxesValue();
  void cleanup() { mJoystick->disable(); }

  Joystick *mJoystick;
  ros::Publisher *mAxesValuePublisher;
  ros::Publisher *mPovsValuePublisher;

  ros::ServiceServer mGetModelServer;
  ros::ServiceServer mGetNumberOfAxesServer;
  ros::ServiceServer mGetNumberOfPovsServer;
  ros::ServiceServer mIsConnectedServer;
  ros::ServiceServer mSetConstantForceServer;
  ros::ServiceServer mSetConstantForceDurationServer;
  ros::ServiceServer mSetAutoCenteringGainServer;
  ros::ServiceServer mSetResistanceGainServer;
  ros::ServiceServer mSetForceAxisServer;
};

#endif  // ROS_JOYSTICK_HPP
