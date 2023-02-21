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

#ifndef ROS_RECEIVER_HPP
#define ROS_RECEIVER_HPP

#include <webots/Receiver.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float.h>
#include <webots_ros/get_int.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_int.h>

#include <webots_ros/receiver_get_emitter_direction.h>

using namespace webots;

class RosReceiver : public RosSensor {
public:
  RosReceiver(Receiver *receiver, Ros *ros);
  virtual ~RosReceiver();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  bool setChannelCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool getChannelCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getQueueLengthCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getDataSizeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getSignalStrengthCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getEmitterDirectionCallback(webots_ros::receiver_get_emitter_direction::Request &req,
                                   webots_ros::receiver_get_emitter_direction::Response &res);
  bool nextPacketCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);

  void rosEnable(int samplingPeriod) override { mReceiver->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mReceiver->getSamplingPeriod(); }

private:
  void cleanup() { mReceiver->disable(); }

  Receiver *mReceiver;
  ros::ServiceServer mSetServer;
  ros::ServiceServer mSamplingPeriodServer;
  ros::ServiceServer mSetChannelServer;
  ros::ServiceServer mGetChannelServer;
  ros::ServiceServer mGetQueueLengthServer;
  ros::ServiceServer mGetDataSizeServer;
  ros::ServiceServer mGetSignalStrengthServer;
  ros::ServiceServer mGetEmitterDirectionServer;
  ros::ServiceServer mNextPacketServer;
};

#endif  // ROS_RECEIVER_HPP
