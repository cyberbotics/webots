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

#ifndef ROS_SENSOR_HPP
#define ROS_SENSOR_HPP

#include "RosDevice.hpp"

#include <webots_ros/get_int.h>
#include <webots_ros/set_int.h>

#include <webots/Device.hpp>

using namespace webots;

class RosSensor : public RosDevice {
public:
  virtual ~RosSensor();

  bool sensorEnableCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  virtual bool samplingPeriodCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  void publishValues(int step);
  bool enableSensor(int timestep);

protected:
  RosSensor(std::string deviceName, Device *device, Ros *ros, bool enableDefaultServices = true);

  virtual ros::Publisher createPublisher() = 0;
  virtual void publishValue(ros::Publisher publisher) = 0;
  virtual void publishAuxiliaryValue(){};
  virtual void rosEnable(int samplingPeriod) = 0;
  virtual void rosDisable() = 0;
  virtual int rosSamplingPeriod() = 0;

  struct publisherDetails {
    ros::Publisher mPublisher;
    int mSamplingPeriod;
    bool mNewPublisher;
  };

  std::vector<publisherDetails> mPublishList;
  std::string mFrameIdPrefix;

private:
  ros::ServiceServer mSensorEnableServer;
  ros::ServiceServer mSamplingPeriodServer;
};

#endif  // ROS_SENSOR_HPP
