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

#include "RosGPS.hpp"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "webots_ros/Float64Stamped.h"

RosGPS::RosGPS(GPS *gps, Ros *ros) : RosSensor(gps->getName(), gps, ros) {
  mGPS = gps;
  mCoordinateTypeServer =
    RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + "/get_coordinate_system", &RosGPS::getCoordinateTypeCallback);
  mConvertServer = RosDevice::rosAdvertiseService(RosDevice::fixedDeviceName() + "/decimal_degrees_to_degrees_minutes_seconds",
                                                  &RosGPS::convertToDegreesMinutesSecondsCallback);
}

RosGPS::~RosGPS() {
  mCoordinateTypeServer.shutdown();
  mConvertServer.shutdown();
  mSpeedPublisher.shutdown();
  mSpeedVectorPublisher.shutdown();
  cleanup();
}

// creates a publisher for GPS values with a [3x1] {double} array
// for x,y and z absolute coordinates as message type
ros::Publisher RosGPS::createPublisher() {
  webots_ros::Float64Stamped speedType;
  mSpeedPublisher = RosDevice::rosAdvertiseTopic(RosDevice::fixedDeviceName() + "/speed", speedType);

  std::string speedVectorTopicName = RosDevice::fixedDeviceName() + "/speed_vector";
  mSpeedVectorPublisher = RosDevice::rosAdvertiseTopic(speedVectorTopicName, geometry_msgs::PointStamped());

  std::string topicName = RosDevice::fixedDeviceName() + "/values";
  if (mGPS->getCoordinateSystem() == GPS::WGS84)
    return RosDevice::rosAdvertiseTopic(topicName, sensor_msgs::NavSatFix());
  return RosDevice::rosAdvertiseTopic(topicName, geometry_msgs::PointStamped());
}

// get value from the GPS and publish it into a [3x1] {double} array
void RosGPS::publishValue(ros::Publisher publisher) {
  if (mGPS->getCoordinateSystem() == GPS::WGS84) {
    sensor_msgs::NavSatFix value;
    value.header.stamp = ros::Time::now();
    value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
    value.latitude = mGPS->getValues()[0];
    value.longitude = mGPS->getValues()[1];
    value.altitude = mGPS->getValues()[2];
    value.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    value.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    publisher.publish(value);
  } else {
    geometry_msgs::PointStamped value;
    value.header.stamp = ros::Time::now();
    value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
    value.point.x = mGPS->getValues()[0];
    value.point.y = mGPS->getValues()[1];
    value.point.z = mGPS->getValues()[2];
    publisher.publish(value);
  }
}

void RosGPS::publishAuxiliaryValue() {
  if (mGPS->getSamplingPeriod() > 0) {
    if (mSpeedVectorPublisher.getNumSubscribers() >= 1) {
      geometry_msgs::PointStamped value;
      value.header.stamp = ros::Time::now();
      value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
      const double *speed_vector = mGPS->getSpeedVector();
      value.point.x = speed_vector[0];
      value.point.y = speed_vector[1];
      value.point.z = speed_vector[2];
      mSpeedVectorPublisher.publish(value);
    }
    if (mSpeedPublisher.getNumSubscribers() >= 1) {
      webots_ros::Float64Stamped speedValue;
      speedValue.header.stamp = ros::Time::now();
      speedValue.data = mGPS->getSpeed();
      mSpeedPublisher.publish(speedValue);
    }
  }
}

bool RosGPS::getCoordinateTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mGPS);
  res.value = mGPS->getCoordinateSystem();
  return true;
}

bool RosGPS::convertToDegreesMinutesSecondsCallback(webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds::Request &req,
                                                    webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds::Response &res) {
  assert(this);
  res.degreesMinutesSeconds = GPS::convertToDegreesMinutesSeconds(req.decimalDegrees);
  return true;
}
