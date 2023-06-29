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

#include "RosCompass.hpp"
#include "sensor_msgs/MagneticField.h"

RosCompass::RosCompass(Compass *compass, Ros *ros) : RosSensor(compass->getName(), compass, ros) {
  mCompass = compass;

  mLookupTableServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_lookup_table", &RosCompass::getLookupTable);
}

RosCompass::~RosCompass() {
  mLookupTableServer.shutdown();
  cleanup();
}

// creates a publisher for compass values with a [3x1] {double} array
// for x,y and z north's coordinates as message type
ros::Publisher RosCompass::createPublisher() {
  sensor_msgs::MagneticField type;
  std::string topicName = RosDevice::fixedDeviceName() + "/values";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the compass and publish it into a [3x1] {double} array
void RosCompass::publishValue(ros::Publisher publisher) {
  sensor_msgs::MagneticField value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  value.magnetic_field.x = mCompass->getValues()[0];
  value.magnetic_field.y = mCompass->getValues()[1];
  value.magnetic_field.z = mCompass->getValues()[2];
  for (int i = 0; i < 9; ++i)  // means "covariance unknown"
    value.magnetic_field_covariance[i] = 0;
  publisher.publish(value);
}

bool RosCompass::getLookupTable(webots_ros::get_float_array::Request &req, webots_ros::get_float_array::Response &res) {
  assert(mCompass);
  const double *values = mCompass->getLookupTable();
  res.value.assign(values, values + mCompass->getLookupTableSize() * 3);
  return true;
}
