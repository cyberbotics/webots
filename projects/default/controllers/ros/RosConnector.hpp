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

#ifndef ROS_CONNECTOR_HPP
#define ROS_CONNECTOR_HPP

#include <webots/Connector.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_bool.h>
#include <webots_ros/set_bool.h>

using namespace webots;

class RosConnector : public RosSensor {
public:
  RosConnector(Connector *connector, Ros *ros);
  virtual ~RosConnector();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mConnector->enablePresence(samplingPeriod); }
  void rosDisable() override { mConnector->disablePresence(); }
  int rosSamplingPeriod() override { return mConnector->getPresenceSamplingPeriod(); }

  bool lockCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool isLockedCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);

private:
  void cleanup() { mConnector->disablePresence(); }

  Connector *mConnector;
  ros::ServiceServer mLockServer;
  ros::ServiceServer mIsLockedServer;
};

#endif  // ROS_CONNECTOR_HPP
