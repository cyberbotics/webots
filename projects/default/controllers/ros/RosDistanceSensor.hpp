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

#ifndef ROS_DISTANCE_SENSOR_HPP
#define ROS_DISTANCE_SENSOR_HPP

#include <webots/DistanceSensor.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float.h>
#include <webots_ros/get_float_array.h>
#include <webots_ros/get_int.h>

using namespace webots;

class RosDistanceSensor : public RosSensor {
public:
  RosDistanceSensor(DistanceSensor *distanceSensor, Ros *ros);
  virtual ~RosDistanceSensor();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mDistanceSensor->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mDistanceSensor->getSamplingPeriod(); }

  bool getMinValueCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getMaxValueCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getApertureCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res);
  bool getTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);

private:
  void cleanup() { mDistanceSensor->disable(); }

  DistanceSensor *mDistanceSensor;
  ros::ServiceServer mMinValueServer;
  ros::ServiceServer mMaxValueServer;
  ros::ServiceServer mApertureServer;
  ros::ServiceServer mLookupTableServer;
  ros::ServiceServer mTypeServer;
};

#endif  // ROS_DISTANCE_SENSOR_HPP
