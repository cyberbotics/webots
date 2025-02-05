// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef LED_HPP
#define LED_HPP

#include <webots/Device.hpp>

namespace webots {
  class LED : public Device {
  public:
    explicit LED(const std::string &name) : Device(name) {}  // Use Robot::getLED() instead
    explicit LED(WbDeviceTag tag) : Device(tag) {}
    virtual ~LED() {}
    virtual void set(int value);
    int get() const;
  };
}  // namespace webots

#endif  // LED_HPP
