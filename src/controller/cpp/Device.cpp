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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/device.h>
#include <webots/robot.h>
#include <webots/Device.hpp>

#include <cstdlib>

using namespace std;
using namespace webots;

Device::Device(const string &deviceName) : name(deviceName) {
  tag = wb_robot_get_device(name.c_str());
}

int Device::getNodeType() const {
  return wb_device_get_node_type(tag);
}

string Device::getModel() const {
  return string(wb_device_get_model(tag));
}

bool Device::hasType(int tag, int type) {
  if (tag == 0)
    return false;
  return wb_device_get_node_type(tag) == type;
}
