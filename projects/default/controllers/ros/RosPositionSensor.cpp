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

#include "RosPositionSensor.hpp"

#include <webots/Brake.hpp>
#include <webots/Motor.hpp>
#include "webots_ros/Float64Stamped.h"

RosPositionSensor::RosPositionSensor(PositionSensor *positionSensor, Ros *ros) :
  RosSensor(positionSensor->getName(), positionSensor, ros) {
  mPositionSensor = positionSensor;
  mTypeServer = RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + "/get_type", &RosPositionSensor::getTypeCallback);
  mGetBrakeNameServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + "/get_brake_name", &RosPositionSensor::getBrakeNameCallback);
  mGetMotorNameServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + "/get_motor_name", &RosPositionSensor::getMotorNameCallback);
}

RosPositionSensor::~RosPositionSensor() {
  mTypeServer.shutdown();
  cleanup();
}

// creates a publisher for position sensor value with a {double} as message type
ros::Publisher RosPositionSensor::createPublisher() {
  webots_ros::Float64Stamped type;
  std::string topicName = RosDevice::fixedDeviceName() + "/value";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the position sensor and publish it
void RosPositionSensor::publishValue(ros::Publisher publisher) {
  webots_ros::Float64Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  value.data = mPositionSensor->getValue();
  publisher.publish(value);
}

bool RosPositionSensor::getTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mPositionSensor);
  res.value = mPositionSensor->getType();
  return true;
}

bool RosPositionSensor::getBrakeNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  Brake *brake = mPositionSensor->getBrake();
  if (brake)
    res.value = brake->getName();
  else
    res.value = "";
  return true;
}

bool RosPositionSensor::getMotorNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  Motor *motor = mPositionSensor->getMotor();
  if (motor)
    res.value = motor->getName();
  else
    res.value = "";
  return true;
}
