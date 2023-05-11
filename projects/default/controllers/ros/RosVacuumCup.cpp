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

#include "RosVacuumCup.hpp"
#include "webots_ros/BoolStamped.h"

RosVacuumCup::RosVacuumCup(VacuumCup *vacuumCup, Ros *ros) :
  RosSensor(vacuumCup->getName() + "/presence_sensor", vacuumCup, ros) {
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mTurnOnServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/turn_on", &RosVacuumCup::turnOnCallback);
  mIsOnServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/is_on", &RosVacuumCup::isOnCallback);
  mVacuumCup = vacuumCup;
}

RosVacuumCup::~RosVacuumCup() {
  mTurnOnServer.shutdown();
  mIsOnServer.shutdown();
  cleanup();
}

ros::Publisher RosVacuumCup::createPublisher() {
  webots_ros::BoolStamped type;
  std::string topicName = RosDevice::fixedDeviceName() + "/presence";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the vacuum cup and publish it
void RosVacuumCup::publishValue(ros::Publisher publisher) {
  webots_ros::BoolStamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  value.data = mVacuumCup->getPresence();
  publisher.publish(value);
}

// cppcheck-suppress constParameter
// cppcheck-suppress constParameterCallback
bool RosVacuumCup::turnOnCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  if (req.value) {
    mVacuumCup->turnOn();
    res.success = true;
  } else {
    mVacuumCup->turnOff();
    res.success = true;
  }
  return true;
}

bool RosVacuumCup::isOnCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = mVacuumCup->isOn();
  return true;
}
