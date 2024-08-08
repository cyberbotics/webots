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

#ifndef SKIN_HPP
#define SKIN_HPP

#include <webots/Device.hpp>

namespace webots {
  class Skin : public Device {
  public:
    explicit Skin(const std::string &name) : Device(name) {}
    explicit Skin(WbDeviceTag tag) : Device(tag) {}
    virtual ~Skin() {}
    void setBoneOrientation(int index, const double orientation[4], bool absolute);
    void setBonePosition(int index, const double position[3], bool absolute);
    int getBoneCount() const;
    const std::string getBoneName(int index) const;
    const double *getBoneOrientation(int index, bool absolute) const;
    const double *getBonePosition(int index, bool absolute) const;
  };
}  // namespace webots

#endif  // SKIN_HPP
