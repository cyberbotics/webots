// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "RosMotorSensor.hpp"
#include "webots_ros/Float64Stamped.h"

RosMotorSensor::RosMotorSensor(Motor *motor, const std::string &sensorName, Ros *ros) :
  RosSensor(sensorName, motor, ros, false) {
  mMotor = motor;
  if (sensorName == (RosDevice::fixedDeviceName() + "/force_feedback_sensor"))
    mType = 1;
  else if (sensorName == (RosDevice::fixedDeviceName() + "/torque_feedback_sensor"))
    mType = 2;
}

// creates a publisher for motor sensor values with a {double} as message mType
ros::Publisher RosMotorSensor::createPublisher() {
  webots_ros::Float64Stamped type;
  std::string topicName;
  if (mType == 1)
    topicName = mRos->name() + '/' + RosDevice::fixedDeviceName() + "/force_feedback";
  else if (mType == 2)
    topicName = mRos->name() + '/' + RosDevice::fixedDeviceName() + "/torque_feedback";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the motor sensor and publish it into a {double} message
void RosMotorSensor::publishValue(ros::Publisher publisher) {
  webots_ros::Float64Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  if (mType == 1)
    value.data = mMotor->getForceFeedback();
  else if (mType == 2)
    value.data = mMotor->getTorqueFeedback();
  publisher.publish(value);
}

void RosMotorSensor::rosEnable(int samplingPeriod) {
  if (mType == 1)
    mMotor->enableForceFeedback(samplingPeriod);
  else if (mType == 2)
    mMotor->enableTorqueFeedback(samplingPeriod);
}
void RosMotorSensor::cleanup() {
  if (mType == 1)
    mMotor->disableForceFeedback();
  else if (mType == 2)
    mMotor->disableTorqueFeedback();
}

std::string RosMotorSensor::deviceName() {
  if (mType == 1)
    return RosDevice::fixedDeviceName() + "/force_feedback_sensor";
  else if (mType == 2)
    return RosDevice::fixedDeviceName() + "/torque_feedback_sensor";
  else
    return "wrong_mType_motor_sensor";
}

int RosMotorSensor::rosSamplingPeriod() {
  if (mType == 1)
    return mMotor->getForceFeedbackSamplingPeriod();
  else if (mType == 2)
    return mMotor->getTorqueFeedbackSamplingPeriod();
  else
    return 0;
}
