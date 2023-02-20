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

#include "RosReceiver.hpp"
#include "webots_ros/StringStamped.h"

RosReceiver::RosReceiver(Receiver *receiver, Ros *ros) : RosSensor(receiver->getName(), receiver, ros) {
  mReceiver = receiver;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mSetChannelServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_channel", &RosReceiver::setChannelCallback);
  mGetChannelServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_channel", &RosReceiver::getChannelCallback);
  mGetQueueLengthServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/get_queue_length", &RosReceiver::getQueueLengthCallback);
  mNextPacketServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/next_packet", &RosReceiver::nextPacketCallback);
  mGetDataSizeServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_data_size", &RosReceiver::getDataSizeCallback);
  mGetSignalStrengthServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/get_signal_strength", &RosReceiver::getSignalStrengthCallback);
  mGetEmitterDirectionServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/get_emitter_direction", &RosReceiver::getEmitterDirectionCallback);
}

RosReceiver::~RosReceiver() {
  mSetChannelServer.shutdown();
  mGetChannelServer.shutdown();
  mGetQueueLengthServer.shutdown();
  mNextPacketServer.shutdown();
  mGetDataSizeServer.shutdown();
  mGetSignalStrengthServer.shutdown();
  mGetEmitterDirectionServer.shutdown();
  cleanup();
}

// creates a publisher for receiver datas with a string as message type
ros::Publisher RosReceiver::createPublisher() {
  webots_ros::StringStamped type;
  std::string topicName = RosDevice::fixedDeviceName() + "/data";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the receiver and publish it
void RosReceiver::publishValue(ros::Publisher publisher) {
  // creates a string message to put receiver datas inside
  webots_ros::StringStamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  if (mReceiver->getQueueLength() > 0)
    value.data = reinterpret_cast<const char *>(mReceiver->getData());
  publisher.publish(value);
}

bool RosReceiver::setChannelCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  mReceiver->setChannel(req.value);
  res.success = true;
  return true;
}

bool RosReceiver::getChannelCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mReceiver);
  res.value = mReceiver->getChannel();
  return true;
}

bool RosReceiver::getQueueLengthCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mReceiver);
  res.value = mReceiver->getQueueLength();
  return true;
}

bool RosReceiver::getDataSizeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mReceiver);
  if (mReceiver->getQueueLength() > 0)
    res.value = mReceiver->getDataSize();
  else {
    ROS_WARN("Illegal call to Receiver::getDataSize().");
    res.value = -1;
  }
  return true;
}

bool RosReceiver::getSignalStrengthCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mReceiver);
  if (mReceiver->getQueueLength() > 0)
    res.value = mReceiver->getSignalStrength();
  else {
    ROS_WARN("Illegal call to Receiver::getSignalStrength().");
    res.value = -1;
  }
  return true;
}

bool RosReceiver::getEmitterDirectionCallback(webots_ros::receiver_get_emitter_direction::Request &req,
                                              webots_ros::receiver_get_emitter_direction::Response &res) {
  assert(mReceiver);
  if (mReceiver->getQueueLength() > 0) {
    const double *direction;
    direction = mReceiver->getEmitterDirection();
    res.direction.push_back(direction[0]);
    res.direction.push_back(direction[1]);
    res.direction.push_back(direction[2]);
  } else {
    ROS_WARN("Illegal call to Receiver::getEmitterDirection().");
    res.direction.push_back(0.0);
    res.direction.push_back(0.0);
    res.direction.push_back(0.0);
  }
  return true;
}

bool RosReceiver::nextPacketCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  if (mReceiver->getQueueLength() > 0) {
    mReceiver->nextPacket();
    res.value = true;
  } else {
    ROS_WARN("Illegal call to Receiver::nextPacket().");
    res.value = false;
  }
  return true;
}
