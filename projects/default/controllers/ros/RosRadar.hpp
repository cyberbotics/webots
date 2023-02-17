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

#ifndef ROS_RADAR_HPP
#define ROS_RADAR_HPP

#include <webots/Radar.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float.h>

using namespace webots;

class RosRadar : public RosSensor {
public:
  RosRadar(Radar *radar, Ros *ros);
  virtual ~RosRadar();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  bool getMaxRangeCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getMinRangeCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getVerticalFovCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getHorizontalFovCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);

  void rosEnable(int samplingPeriod) override { mRadar->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mRadar->getSamplingPeriod(); }

private:
  void cleanup() { mRadar->disable(); }

  Radar *mRadar;
  ros::Publisher mTargetsNumberPublisher;
  ros::ServiceServer mGetMaxRangeServer;
  ros::ServiceServer mGetMinRangeServer;
  ros::ServiceServer mGetVerticalFovServer;
  ros::ServiceServer mGetHorizontalFovServer;
};

#endif  // ROS_RADAR_HPP
