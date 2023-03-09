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

#ifndef ROS_TOUCH_SENSOR_HPP
#define ROS_TOUCH_SENSOR_HPP

#include <webots/TouchSensor.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float_array.h>
#include <webots_ros/get_int.h>

using namespace webots;

class RosTouchSensor : public RosSensor {
public:
  RosTouchSensor(TouchSensor *touchSensor, Ros *ros);
  virtual ~RosTouchSensor();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mTouchSensor->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mTouchSensor->getSamplingPeriod(); }
  bool getTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res);

private:
  void cleanup() { mTouchSensor->disable(); }

  TouchSensor *mTouchSensor;
  ros::ServiceServer mTypeServer;

  ros::ServiceServer mLookupTableServer;
};

#endif  // ROS_TOUCH_SENSOR_HPP
