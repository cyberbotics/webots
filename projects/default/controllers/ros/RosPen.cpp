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

#include "RosPen.hpp"

RosPen::RosPen(Pen *pen, Ros *ros) : RosDevice(pen, ros) {
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mWriteServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/write", &RosPen::writeCallback);
  mSetInkColorServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_ink_color", &RosPen::setInkColorCallback);
  mPen = pen;
}

RosPen::~RosPen() {
  mWriteServer.shutdown();
  mSetInkColorServer.shutdown();
}

bool RosPen::writeCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  mPen->write(req.value);
  res.success = true;
  return true;
}

bool RosPen::setInkColorCallback(webots_ros::pen_set_ink_color::Request &req, webots_ros::pen_set_ink_color::Response &res) {
  mPen->setInkColor(req.color, req.density);
  res.success = 1;
  return true;
}
