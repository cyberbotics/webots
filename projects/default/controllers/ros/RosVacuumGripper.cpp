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

#include "RosVacuumGripper.hpp"
#include "webots_ros/BoolStamped.h"

RosVacuumGripper::RosVacuumGripper(VacuumGripper *vacuumGripper, Ros *ros) :
  RosSensor(vacuumGripper->getName() + "/presence_sensor", vacuumGripper, ros) {
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mTurnOnServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/turn_on", &RosVacuumGripper::turnOnCallback);
  mIsOnServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/is_on", &RosVacuumGripper::isOnCallback);
  mVacuumGripper = vacuumGripper;
}

RosVacuumGripper::~RosVacuumGripper() {
  mTurnOnServer.shutdown();
  mIsOnServer.shutdown();
  cleanup();
}

ros::Publisher RosVacuumGripper::createPublisher() {
  webots_ros::BoolStamped type;
  std::string topicName = RosDevice::fixedDeviceName() + "/presence";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the vacuum gripper and publish it
void RosVacuumGripper::publishValue(ros::Publisher publisher) {
  webots_ros::BoolStamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  value.data = mVacuumGripper->getPresence();
  publisher.publish(value);
}

// cppcheck-suppress constParameter
// cppcheck-suppress constParameterCallback
bool RosVacuumGripper::turnOnCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  if (req.value) {
    mVacuumGripper->turnOn();
    res.success = true;
  } else {
    mVacuumGripper->turnOff();
    res.success = true;
  }
  return true;
}

bool RosVacuumGripper::isOnCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = mVacuumGripper->isOn();
  return true;
}
