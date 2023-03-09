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

#include "RosMotorSensor.hpp"
#include "webots_ros/Float64Stamped.h"

RosMotorSensor::RosMotorSensor(Motor *motor, const std::string &sensorName, Ros *ros) :
  RosSensor(sensorName, motor, ros, false) {
  mMotor = motor;
}

// creates a publisher for motor sensor values with a {double} as message
ros::Publisher RosMotorSensor::createPublisher() {
  webots_ros::Float64Stamped type;
  std::string topicName;
  if (mMotor->getType() == Motor::LINEAR)
    topicName = RosDevice::fixedDeviceName() + "/force_feedback";
  else
    topicName = RosDevice::fixedDeviceName() + "/torque_feedback";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the motor sensor and publish it into a {double} message
void RosMotorSensor::publishValue(ros::Publisher publisher) {
  webots_ros::Float64Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  if (mMotor->getType() == Motor::LINEAR)
    value.data = mMotor->getForceFeedback();
  else
    value.data = mMotor->getTorqueFeedback();
  publisher.publish(value);
}

void RosMotorSensor::rosEnable(int samplingPeriod) {
  if (mMotor->getType() == Motor::LINEAR)
    mMotor->enableForceFeedback(samplingPeriod);
  else
    mMotor->enableTorqueFeedback(samplingPeriod);
}
void RosMotorSensor::cleanup() {
  if (mMotor->getType() == Motor::LINEAR)
    mMotor->disableForceFeedback();
  else
    mMotor->disableTorqueFeedback();
}

int RosMotorSensor::rosSamplingPeriod() {
  if (mMotor->getType() == Motor::LINEAR)
    return mMotor->getForceFeedbackSamplingPeriod();
  else
    return mMotor->getTorqueFeedbackSamplingPeriod();
}
