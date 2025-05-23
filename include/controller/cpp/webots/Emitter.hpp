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

#ifndef EMITTER_HPP
#define EMITTER_HPP

#include <webots/Device.hpp>

namespace webots {
  class Emitter : public Device {
  public:
    explicit Emitter(const std::string &name) : Device(name) {}  // Use Robot::getEmitter() instead
    explicit Emitter(WbDeviceTag tag) : Device(tag) {}
    virtual ~Emitter() {}
    enum { CHANNEL_BROADCAST = -1 };
    virtual int send(const void *data, int size);
    int getBufferSize() const;
    virtual void setChannel(int channel);
    int getChannel() const;
    double getRange() const;
    virtual void setRange(double range);
  };
}  // namespace webots

#endif  // EMITTER_HPP
