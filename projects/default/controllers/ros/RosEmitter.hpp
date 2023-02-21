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

#ifndef ROS_EMITTER_HPP
#define ROS_EMITTER_HPP

#include <webots/Emitter.hpp>
#include "RosDevice.hpp"

#include <webots_ros/get_float.h>
#include <webots_ros/get_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>

using namespace webots;

class RosEmitter : public RosDevice {
public:
  RosEmitter(Emitter *emitter, Ros *ros);
  virtual ~RosEmitter();

  bool sendCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);
  bool getBufferSizeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getChannelCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getRangeCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool setChannelCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool setRangeCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);

private:
  Emitter *mEmitter;
  ros::ServiceServer mSendServer;
  ros::ServiceServer mGetBufferSizeServer;
  ros::ServiceServer mGetChannelServer;
  ros::ServiceServer mGetRangeServer;
  ros::ServiceServer mSetChannelServer;
  ros::ServiceServer mSetRangeServer;
  ros::Publisher mPublisher;
};

#endif  // ROS_EMITTER_HPP
