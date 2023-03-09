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

#include "RosDistanceSensor.hpp"
#include "sensor_msgs/Range.h"

RosDistanceSensor::RosDistanceSensor(DistanceSensor *distanceSensor, Ros *ros) :
  RosSensor(distanceSensor->getName(), distanceSensor, ros) {
  mDistanceSensor = distanceSensor;
  mMinValueServer = RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_min_value",
                                                   &RosDistanceSensor::getMinValueCallback);
  mMaxValueServer = RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_max_value",
                                                   &RosDistanceSensor::getMaxValueCallback);
  mApertureServer = RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_aperture",
                                                   &RosDistanceSensor::getApertureCallback);
  mLookupTableServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_lookup_table", &RosDistanceSensor::getLookupTable);
  mTypeServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_type", &RosDistanceSensor::getTypeCallback);
}

RosDistanceSensor::~RosDistanceSensor() {
  mMinValueServer.shutdown();
  mMaxValueServer.shutdown();
  mApertureServer.shutdown();
  mLookupTableServer.shutdown();
  mTypeServer.shutdown();
  cleanup();
}

// creates a publisher for distance sensor value with a {double} as message type
ros::Publisher RosDistanceSensor::createPublisher() {
  sensor_msgs::Range type;
  std::string topicName = RosDevice::fixedDeviceName() + "/value";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the distance sensor and publish it
void RosDistanceSensor::publishValue(ros::Publisher publisher) {
  sensor_msgs::Range value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  if (mDistanceSensor->getType() == DistanceSensor::SONAR)
    value.radiation_type = 0;
  else if (mDistanceSensor->getType() == DistanceSensor::INFRA_RED)
    value.radiation_type = 1;
  value.field_of_view = mDistanceSensor->getAperture();
  value.min_range = mDistanceSensor->getMinValue();
  value.max_range = mDistanceSensor->getMaxValue();
  value.range = mDistanceSensor->getValue();
  publisher.publish(value);
}

bool RosDistanceSensor::getMinValueCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mDistanceSensor);
  res.value = mDistanceSensor->getMinValue();
  return true;
}

bool RosDistanceSensor::getMaxValueCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mDistanceSensor);
  res.value = mDistanceSensor->getMaxValue();
  return true;
}

bool RosDistanceSensor::getApertureCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mDistanceSensor);
  res.value = mDistanceSensor->getAperture();
  return true;
}

bool RosDistanceSensor::getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res) {
  assert(mDistanceSensor);
  const double *values = mDistanceSensor->getLookupTable();
  res.value.assign(values, values + mDistanceSensor->getLookupTableSize() * 3);
  return true;
}

bool RosDistanceSensor::getTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mDistanceSensor);
  res.value = mDistanceSensor->getType();
  return true;
}
