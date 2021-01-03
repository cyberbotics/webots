// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "RosRadar.hpp"
#include "webots_ros/Int8Stamped.h"
#include "webots_ros/RadarTarget.h"

RosRadar::RosRadar(Radar *radar, Ros *ros) : RosSensor(radar->getName(), radar, ros) {
  mRadar = radar;
  std::string deviceNameFixed = RosDevice::fixedDeviceName();
  mGetMaxRangeServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + "/get_max_range", &RosRadar::getMaxRangeCallback);
  mGetMinRangeServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + "/get_min_range", &RosRadar::getMinRangeCallback);
  mGetVerticalFovServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + "/get_vertical_fov",
                                                         &RosRadar::getVerticalFovCallback);
  mGetHorizontalFovServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + "/get_horizontal_fov",
                                                           &RosRadar::getHorizontalFovCallback);
}

RosRadar::~RosRadar() {
  mTargetsNumberPublisher.shutdown();
  cleanup();
}

// creates a publisher for radar targets with a {RadarTarget} as message type
ros::Publisher RosRadar::createPublisher() {
  std::string deviceNameFixed = RosDevice::fixedDeviceName();
  webots_ros::Int8Stamped targetsNumberType;
  mTargetsNumberPublisher =
    RosDevice::rosAdvertiseTopic(mRos->name() + '/' + deviceNameFixed + "/number_of_targets", targetsNumberType);

  webots_ros::RadarTarget type;
  std::string topicName = mRos->name() + '/' + deviceNameFixed + "/targets";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get the target from the radar and publish them
void RosRadar::publishValue(ros::Publisher publisher) {
  int target_number = mRadar->getNumberOfTargets();
  webots_ros::Int8Stamped targetsNumber;
  targetsNumber.header.stamp = ros::Time::now();
  targetsNumber.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  targetsNumber.data = target_number;
  mTargetsNumberPublisher.publish(targetsNumber);

  webots_ros::RadarTarget target;
  target.header.stamp = ros::Time::now();
  target.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  const WbRadarTarget *targets = mRadar->getTargets();
  for (int i = 0; i < target_number; ++i) {
    target.distance = targets[i].distance;
    target.receivedPower = targets[i].received_power;
    target.speed = targets[i].speed;
    target.azimuth = targets[i].azimuth;
    publisher.publish(target);
  }
}

bool RosRadar::getMaxRangeCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mRadar);
  res.value = mRadar->getMaxRange();
  return true;
}

bool RosRadar::getMinRangeCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mRadar);
  res.value = mRadar->getMinRange();
  return true;
}

bool RosRadar::getVerticalFovCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mRadar);
  res.value = mRadar->getVerticalFov();
  return true;
}

bool RosRadar::getHorizontalFovCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mRadar);
  res.value = mRadar->getHorizontalFov();
  return true;
}
