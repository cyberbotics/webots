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

#ifndef ROS_PEN_HPP
#define ROS_PEN_HPP

#include <webots/Pen.hpp>
#include "RosDevice.hpp"

#include <webots_ros/set_bool.h>

#include <webots_ros/pen_set_ink_color.h>

using namespace webots;

class RosPen : public RosDevice {
public:
  RosPen(Pen *pen, Ros *ros);
  virtual ~RosPen();

  bool writeCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool setInkColorCallback(webots_ros::pen_set_ink_color::Request &req, webots_ros::pen_set_ink_color::Response &res);

private:
  Pen *mPen;
  ros::ServiceServer mWriteServer;
  ros::ServiceServer mSetInkColorServer;
};

#endif  // ROS_PEN_HPP
