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

#include "RosDevice.hpp"

RosDevice::RosDevice(Device *device, Ros *ros, bool enableDefaultServices) :
  mRos(ros),
  mDevice(device),
  mEnableDefaultServices(enableDefaultServices) {
}

RosDevice::~RosDevice() {
  mGetModel.shutdown();
  mGetName.shutdown();
  mGetNodeType.shutdown();
}

void RosDevice::init() {
  if (mDevice && mEnableDefaultServices) {
    mGetModel = mRos->nodeHandle()->advertiseService(fixedDeviceName() + "/get_model", &RosDevice::getModelCallback, this);
    mGetName = mRos->nodeHandle()->advertiseService(fixedDeviceName() + "/get_name", &RosDevice::getNameCallback, this);
    mGetNodeType =
      mRos->nodeHandle()->advertiseService(fixedDeviceName() + "/get_node_type", &RosDevice::getNodeTypeCallback, this);
  }
}

bool RosDevice::getModelCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  assert(this);
  if (mDevice)
    res.value = mDevice->getModel();
  else
    res.value = "";
  return true;
}

bool RosDevice::getNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  assert(this);
  if (mDevice)
    res.value = mDevice->getName();
  else
    res.value = "";
  return true;
}

bool RosDevice::getNodeTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(this);
  if (mDevice)
    res.value = mDevice->getNodeType();
  else
    res.value = 0;
  return true;
}

std::string RosDevice::deviceName() {
  assert(this);
  if (mDevice)
    return mDevice->getName();
  else
    return "";
}

std::string RosDevice::fixedDeviceName() {
  return Ros::fixedNameString(deviceName());
}
