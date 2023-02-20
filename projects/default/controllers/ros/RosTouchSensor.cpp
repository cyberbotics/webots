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

#include "RosTouchSensor.hpp"
#include "geometry_msgs/WrenchStamped.h"
#include "webots_ros/BoolStamped.h"
#include "webots_ros/Float64Stamped.h"

RosTouchSensor::RosTouchSensor(TouchSensor *touchSensor, Ros *ros) : RosSensor(touchSensor->getName(), touchSensor, ros) {
  mTouchSensor = touchSensor;

  mTypeServer = RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + "/get_type", &RosTouchSensor::getTypeCallback);
  mLookupTableServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_lookup_table", &RosTouchSensor::getLookupTable);
}

RosTouchSensor::~RosTouchSensor() {
  mTypeServer.shutdown();
  mLookupTableServer.shutdown();
  cleanup();
}

// creates a publisher for touch sensor value with either a {Bool}, a {Float64}
// or a {WrenchStamped} as message type depending on TouchSensorType
ros::Publisher RosTouchSensor::createPublisher() {
  if (mTouchSensor->getType() == TouchSensor::FORCE) {
    webots_ros::Float64Stamped type;
    std::string topicName = RosDevice::fixedDeviceName() + "/value";
    return RosDevice::rosAdvertiseTopic(topicName, type);
  } else if (mTouchSensor->getType() == TouchSensor::FORCE3D) {
    geometry_msgs::WrenchStamped type;
    std::string topicName = RosDevice::fixedDeviceName() + "/values";
    return RosDevice::rosAdvertiseTopic(topicName, type);
  } else {  // TouchSensor::BUMPER
    webots_ros::BoolStamped type;
    std::string topicName = RosDevice::fixedDeviceName() + "/value";
    return RosDevice::rosAdvertiseTopic(topicName, type);
  }
}

// get value(s) from the touch sensor and publish it
void RosTouchSensor::publishValue(ros::Publisher publisher) {
  if (mTouchSensor->getType() == TouchSensor::BUMPER) {
    webots_ros::BoolStamped value;
    value.header.stamp = ros::Time::now();
    value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
    value.data = mTouchSensor->getValue();
    publisher.publish(value);
  } else if (mTouchSensor->getType() == TouchSensor::FORCE) {
    webots_ros::Float64Stamped value;
    value.header.stamp = ros::Time::now();
    value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
    value.data = mTouchSensor->getValue();
    publisher.publish(value);
  } else if (mTouchSensor->getType() == TouchSensor::FORCE3D) {
    geometry_msgs::WrenchStamped value;
    value.header.stamp = ros::Time::now();
    value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
    value.wrench.force.x = mTouchSensor->getValues()[0];
    value.wrench.force.y = mTouchSensor->getValues()[1];
    value.wrench.force.z = mTouchSensor->getValues()[2];
    value.wrench.torque.x = 0.0;
    value.wrench.torque.y = 0.0;
    value.wrench.torque.z = 0.0;
    publisher.publish(value);
  }
}

bool RosTouchSensor::getTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mTouchSensor);
  res.value = mTouchSensor->getType();
  return true;
}

bool RosTouchSensor::getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res) {
  assert(mTouchSensor);
  const double *values = mTouchSensor->getLookupTable();
  res.value.assign(values, values + mTouchSensor->getLookupTableSize() * 3);
  return true;
}
