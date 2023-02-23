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

#include "RosEmitter.hpp"

RosEmitter::RosEmitter(Emitter *emitter, Ros *ros) : RosDevice(emitter, ros) {
  mEmitter = emitter;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mSendServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/send", &RosEmitter::sendCallback);
  mGetBufferSizeServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/get_buffer_size", &RosEmitter::getBufferSizeCallback);
  mGetChannelServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_channel", &RosEmitter::getChannelCallback);
  mGetRangeServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_range", &RosEmitter::getRangeCallback);
  mSetChannelServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_channel", &RosEmitter::setChannelCallback);
  mSetRangeServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_range", &RosEmitter::setRangeCallback);
}

RosEmitter::~RosEmitter() {
  mSendServer.shutdown();
  mGetBufferSizeServer.shutdown();
  mGetChannelServer.shutdown();
  mGetRangeServer.shutdown();
  mSetChannelServer.shutdown();
  mSetRangeServer.shutdown();
}

bool RosEmitter::sendCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res) {
  mEmitter->send(static_cast<void *>(const_cast<char *>(req.value.c_str())), req.value.size());
  res.success = true;
  return true;
}

bool RosEmitter::getBufferSizeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mEmitter);
  res.value = mEmitter->getBufferSize();
  return true;
}

bool RosEmitter::getChannelCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mEmitter);
  res.value = mEmitter->getChannel();
  return true;
}

bool RosEmitter::getRangeCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mEmitter);
  res.value = mEmitter->getRange();
  return true;
}

bool RosEmitter::setChannelCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  mEmitter->setChannel(req.value);
  res.success = true;
  return true;
}

bool RosEmitter::setRangeCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mEmitter->setRange(req.value);
  res.success = true;
  return true;
}
