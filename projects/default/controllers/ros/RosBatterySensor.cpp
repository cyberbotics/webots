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

#include "RosBatterySensor.hpp"
#include "webots_ros/Float64Stamped.h"

RosBatterySensor::RosBatterySensor(Robot *robot, Ros *ros) : RosSensor("battery_sensor", NULL, ros) {
  mRobot = robot;
}

// creates a publisher for battery sensor value with a {double} as message type
ros::Publisher RosBatterySensor::createPublisher() {
  webots_ros::Float64Stamped type;
  std::string topicName = "battery_sensor/value";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the battery sensor and publish it
void RosBatterySensor::publishValue(ros::Publisher publisher) {
  // creates a {double} message to put battery sensor data inside
  webots_ros::Float64Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + "battery_sensor";
  value.data = mRobot->batterySensorGetValue();
  publisher.publish(value);
}
