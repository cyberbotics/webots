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

#ifndef ROS_VACUUM_GRIPPER_HPP
#define ROS_VACUUM_GRIPPER_HPP

#include <webots/VacuumGripper.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_bool.h>
#include <webots_ros/set_bool.h>

using namespace webots;

class RosVacuumGripper : public RosSensor {
public:
  RosVacuumGripper(VacuumGripper *vacuumGripper, Ros *ros);
  virtual ~RosVacuumGripper();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mVacuumGripper->enablePresence(samplingPeriod); }
  void rosDisable() override { mVacuumGripper->disablePresence(); }
  int rosSamplingPeriod() override { return mVacuumGripper->getPresenceSamplingPeriod(); }

  bool turnOnCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool isOnCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);

private:
  void cleanup() { mVacuumGripper->disablePresence(); }

  VacuumGripper *mVacuumGripper;
  ros::ServiceServer mTurnOnServer;
  ros::ServiceServer mIsOnServer;
};

#endif  // ROS_VACUUM_GRIPPER_HPP
