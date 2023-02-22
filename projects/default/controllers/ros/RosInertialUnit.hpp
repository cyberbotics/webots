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

#ifndef ROS_INERTIAL_UNIT_HPP
#define ROS_INERTIAL_UNIT_HPP

#include <webots/InertialUnit.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float_array.h>

using namespace webots;

class RosInertialUnit : public RosSensor {
public:
  RosInertialUnit(InertialUnit *inertialUnit, Ros *ros);
  virtual ~RosInertialUnit();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mInertialUnit->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mInertialUnit->getSamplingPeriod(); }
  bool getNoise(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);

private:
  void cleanup() { mInertialUnit->disable(); }

  InertialUnit *mInertialUnit;
  ros::ServiceServer mNoiseServer;
};

#endif  // ROS_INERTIAL_UNIT_HPP
