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

#include "RosAltimeter.hpp"
#include "webots_ros/Float64Stamped.h"

RosAltimeter::RosAltimeter(Altimeter *altimeter, Ros *ros) : RosSensor(altimeter->getName(), altimeter, ros) {
  mAltimeter = altimeter;
}

RosAltimeter::~RosAltimeter() {
  cleanup();
}

// creates a publisher for altimeter value with a {Float64}
ros::Publisher RosAltimeter::createPublisher() {
  webots_ros::Float64Stamped type;
  std::string topicName = RosDevice::fixedDeviceName() + "/value";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from altimeter and publish it
void RosAltimeter::publishValue(ros::Publisher publisher) {
  webots_ros::Float64Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  value.data = mAltimeter->getValue();
  publisher.publish(value);
}
