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

#ifndef ROS_COMPASS_HPP
#define ROS_COMPASS_HPP

#include <webots/Compass.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float_array.h>

using namespace webots;

class RosCompass : public RosSensor {
public:
  RosCompass(Compass *compass, Ros *ros);
  virtual ~RosCompass();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mCompass->enable(samplingPeriod); }
  void rosDisable() override { mCompass->disable(); }
  int rosSamplingPeriod() override { return mCompass->getSamplingPeriod(); }
  bool getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res);

private:
  void cleanup() { mCompass->disable(); }

  Compass *mCompass;

  ros::ServiceServer mLookupTableServer;
};

#endif  // ROS_COMPASS_HPP
