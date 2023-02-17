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

#ifndef ROS_GPS_HPP
#define ROS_GPS_HPP

#include <webots/GPS.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_int.h>

#include <webots_ros/gps_decimal_degrees_to_degrees_minutes_seconds.h>

using namespace webots;

class RosGPS : public RosSensor {
public:
  RosGPS(GPS *gps, Ros *ros);
  virtual ~RosGPS();

  ros::Publisher createPublisher() override;
  void publishAuxiliaryValue() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mGPS->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mGPS->getSamplingPeriod(); }

  bool getCoordinateTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool convertToDegreesMinutesSecondsCallback(webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds::Request &req,
                                              webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds::Response &res);

private:
  void cleanup() { mGPS->disable(); }

  GPS *mGPS;
  ros::Publisher mSpeedPublisher;
  ros::Publisher mSpeedVectorPublisher;
  ros::ServiceServer mCoordinateTypeServer;
  ros::ServiceServer mConvertServer;
};

#endif  // ROS_GPS_HPP
