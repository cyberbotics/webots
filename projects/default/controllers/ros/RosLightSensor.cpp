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

#include "RosLightSensor.hpp"
#include "sensor_msgs/Illuminance.h"

RosLightSensor::RosLightSensor(LightSensor *lightSensor, Ros *ros) : RosSensor(lightSensor->getName(), lightSensor, ros) {
  mLightSensor = lightSensor;

  mLookupTableServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_lookup_table", &RosLightSensor::getLookupTable);
}

RosLightSensor::~RosLightSensor() {
  mLookupTableServer.shutdown();
  cleanup();
}

// creates a publisher for light sensor value with a {double} as message type
ros::Publisher RosLightSensor::createPublisher() {
  sensor_msgs::Illuminance type;
  std::string topicName = RosDevice::fixedDeviceName() + "/value";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the light sensor and publish it
void RosLightSensor::publishValue(ros::Publisher publisher) {
  sensor_msgs::Illuminance value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  value.illuminance = mLightSensor->getValue();
  value.variance = 0.0;
  publisher.publish(value);
}

bool RosLightSensor::getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res) {
  assert(mLightSensor);
  const double *values = mLightSensor->getLookupTable();
  res.value.assign(values, values + mLightSensor->getLookupTableSize() * 3);
  return true;
}
