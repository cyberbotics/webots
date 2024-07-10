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

#ifndef DEVICE_HPP
#define DEVICE_HPP

#define WB_USING_CPP_API
#include <string>
#include "../../c/webots/types.h"

namespace webots {
  class Device {
  public:
    virtual ~Device() {}
    const std::string &getName() const { return name; }
    std::string getModel() const;
    int getNodeType() const;
    int getTag() const { return tag; }

    static bool hasType(int tag, int type);

  protected:
    explicit Device(const std::string &name);
    explicit Device(WbDeviceTag tag);

  private:
    WbDeviceTag tag;
    std::string name;
  };
}  // namespace webots

#endif  // DEVICE_HPP
