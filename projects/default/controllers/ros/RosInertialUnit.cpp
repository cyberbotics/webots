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

#include "RosInertialUnit.hpp"
#include "sensor_msgs/Imu.h"

RosInertialUnit::RosInertialUnit(InertialUnit *inertialUnit, Ros *ros) : RosSensor(inertialUnit->getName(), inertialUnit, ros) {
  mInertialUnit = inertialUnit;
}

// creates a publisher for InertialUnit values with a sensor_msgs/Imu as message type
ros::Publisher RosInertialUnit::createPublisher() {
  sensor_msgs::Imu type;
  std::string topicName = mRos->name() + '/' + RosDevice::fixedDeviceName() + "/roll_pitch_yaw";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the InertialUnit and publish it into a sensor_msgs/Imu
void RosInertialUnit::publishValue(ros::Publisher publisher) {
  sensor_msgs::Imu value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();

  // switch roll and pitch axes because the Webots and ROS coordinate systems are not equivalent
  // https://stackoverflow.com/questions/56074321/quaternion-calculation-in-rosinertialunit-cpp-of-webots-ros-default-controller?answertab=oldest#tab-top
  double halfYaw = mInertialUnit->getRollPitchYaw()[1] * 0.5;
  double halfPitch = mInertialUnit->getRollPitchYaw()[0] * 0.5;
  double halfRoll = mInertialUnit->getRollPitchYaw()[2] * 0.5;
  double cosYaw = cos(halfYaw);
  double sinYaw = sin(halfYaw);
  double cosPitch = cos(halfPitch);
  double sinPitch = sin(halfPitch);
  double cosRoll = cos(halfRoll);
  double sinRoll = sin(halfRoll);
  value.orientation.x = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  value.orientation.y = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  value.orientation.z = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  value.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
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
