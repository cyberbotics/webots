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

#include "RosLED.hpp"

RosLED::RosLED(LED *led, Ros *ros) : RosDevice(led, ros) {
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mSetServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_led", &RosLED::setCallback);
  mGetServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_led", &RosLED::getCallback);
  mLED = led;
}

RosLED::~RosLED() {
  mSetServer.shutdown();
  mGetServer.shutdown();
}

// set LED device on/off depending on req value
bool RosLED::setCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  mLED->set(req.value);
  res.success = true;
  return true;
}

// send back LED value
bool RosLED::getCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mLED);
  res.value = mLED->get();
  return true;
}
