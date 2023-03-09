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

/*******************************************************************************************************/
/* Description:  Wrapper of the Device Webots API for the ROBOTIS OP2 real robot                         */
/*******************************************************************************************************/

#ifndef DEVICE_HPP
#define DEVICE_HPP

#define WB_USING_CPP_API
#include <string>

namespace webots {
  class Device {
  public:
    virtual ~Device() {}
    const std::string &getName() const { return mName; }

  protected:
    Device(const std::string &n) : mName(n) {}

  private:
    std::string mName;
  };
}  // namespace webots

#endif  // DEVICE_HPP
