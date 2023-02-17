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

#ifndef ROS_BATTERY_SENSOR_HPP
#define ROS_BATTERY_SENSOR_HPP

#include <webots/Robot.hpp>
#include "RosSensor.hpp"

using namespace webots;

class RosBatterySensor : public RosSensor {
public:
  RosBatterySensor(Robot *robot, Ros *ros);
  virtual ~RosBatterySensor() { cleanup(); }

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mRobot->batterySensorEnable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  std::string deviceName() override { return "battery_sensor"; }
  int rosSamplingPeriod() override { return mRobot->batterySensorGetSamplingPeriod(); }

private:
  void cleanup() { mRobot->batterySensorDisable(); }

  Robot *mRobot;
};

#endif  // ROS_BATTERY_HPP
