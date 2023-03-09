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

#ifndef ROS_SKIN_HPP
#define ROS_SKIN_HPP

#include <webots/Skin.hpp>
#include "RosDevice.hpp"

#include "webots_ros/get_int.h"

#include "webots_ros/skin_get_bone_name.h"
#include "webots_ros/skin_get_bone_orientation.h"
#include "webots_ros/skin_get_bone_position.h"
#include "webots_ros/skin_set_bone_orientation.h"
#include "webots_ros/skin_set_bone_position.h"

using namespace webots;

class RosSkin : public RosDevice {
public:
  RosSkin(Skin *skin, Ros *ros);
  virtual ~RosSkin(){};

  bool setBoneOrientationCallback(webots_ros::skin_set_bone_orientation::Request &req,
                                  webots_ros::skin_set_bone_orientation::Response &res);
  bool setBonePositionCallback(webots_ros::skin_set_bone_position::Request &req,
                               webots_ros::skin_set_bone_position::Response &res);
  bool getBoneNameCallback(webots_ros::skin_get_bone_name::Request &req, webots_ros::skin_get_bone_name::Response &res);
  bool getBoneCountCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getBoneOrientationCallback(webots_ros::skin_get_bone_orientation::Request &req,
                                  webots_ros::skin_get_bone_orientation::Response &res);
  bool getBonePositionCallback(webots_ros::skin_get_bone_position::Request &req,
                               webots_ros::skin_get_bone_position::Response &res);

private:
  Skin *mSkin;
  ros::ServiceServer mSetBoneOrientationServer;
  ros::ServiceServer mSetBonePositionServer;
  ros::ServiceServer mGetBoneNameServer;
  ros::ServiceServer mGetBoneCountServer;
  ros::ServiceServer mGetBoneOrientationServer;
  ros::ServiceServer mGetBonePositionServer;
  ros::Publisher mPublisher;
};

#endif  // ROS_SKIN_HPP
