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

#ifndef ROS_LIGHT_SENSOR_HPP
#define ROS_LIGHT_SENSOR_HPP

#include <webots/LightSensor.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float_array.h>

using namespace webots;

class RosLightSensor : public RosSensor {
public:
  RosLightSensor(LightSensor *lightSensor, Ros *ros);
  virtual ~RosLightSensor();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mLightSensor->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mLightSensor->getSamplingPeriod(); }
  bool getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res);

private:
  void cleanup() { mLightSensor->disable(); }

  LightSensor *mLightSensor;

  ros::ServiceServer mLookupTableServer;
};

#endif  // ROS_LIGHT_SENSOR_HPP
