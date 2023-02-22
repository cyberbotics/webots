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

#include "RosInertialUnit.hpp"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"

RosInertialUnit::RosInertialUnit(InertialUnit *inertialUnit, Ros *ros) : RosSensor(inertialUnit->getName(), inertialUnit, ros) {
  mInertialUnit = inertialUnit;

  mNoiseServer = RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + '/' + "get_noise", &RosInertialUnit::getNoise);
}

RosInertialUnit::~RosInertialUnit() {
  cleanup();
}

// creates a publisher for InertialUnit values with a sensor_msgs/Imu as message type
ros::Publisher RosInertialUnit::createPublisher() {
  sensor_msgs::Imu type;
  std::string topicName = RosDevice::fixedDeviceName() + "/quaternion";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the InertialUnit and publish it into a sensor_msgs/Imu
void RosInertialUnit::publishValue(ros::Publisher publisher) {
  sensor_msgs::Imu value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();

  // switch roll and pitch axes because the Webots and ROS coordinate systems are not equivalent
  // https://stackoverflow.com/questions/56074321/quaternion-calculation-in-rosinertialunit-cpp-of-webots-ros-default-controller?answertab=oldest#tab-top
  tf2::Quaternion orientation(mInertialUnit->getQuaternion()[0], mInertialUnit->getQuaternion()[1],
                              mInertialUnit->getQuaternion()[2], mInertialUnit->getQuaternion()[3]);
  value.orientation.x = orientation.getX();
  value.orientation.y = orientation.getY();
  value.orientation.z = orientation.getZ();
  value.orientation.w = orientation.getW();
  for (int i = 0; i < 9; ++i)  // means "covariance unknown"
    value.orientation_covariance[i] = 0;
  value.angular_velocity.x = 0.0;
  value.angular_velocity.y = 0.0;
  value.angular_velocity.z = 0.0;
  value.angular_velocity_covariance[0] = -1;  // means no angular_velocity information
  value.linear_acceleration.x = 0.0;
  value.linear_acceleration.y = 0.0;
  value.linear_acceleration.z = 0.0;
  value.linear_acceleration_covariance[0] = -1.0;  // means no linear_acceleration information
  publisher.publish(value);
}

bool RosInertialUnit::getNoise(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mInertialUnit);
  res.value = mInertialUnit->getNoise();
  return true;
}
