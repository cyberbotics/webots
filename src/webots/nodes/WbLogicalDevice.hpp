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

// Abstract node class representing a logical device, i.e., a device without physics properties

#ifndef WB_LOGICAL_DEVICE_HPP
#define WB_LOGICAL_DEVICE_HPP

#include "WbBaseNode.hpp"
#include "WbDevice.hpp"
#include "WbSFString.hpp"

class WbLogicalDevice : public WbBaseNode, public WbDevice {
public:
  virtual ~WbLogicalDevice();
  const QString &deviceName() const override { return mDeviceName->value(); }
  int deviceNodeType() const override { return nodeType(); }

protected:
  explicit WbLogicalDevice(const QString &modelName, WbTokenizer *tokenizer = NULL);
  WbLogicalDevice(const WbLogicalDevice &other);
  explicit WbLogicalDevice(const WbNode &other);
  bool exportNodeHeader(WbWriter &writer) const override;

protected:
  WbSFString *mDeviceName;

private:
  WbLogicalDevice &operator=(const WbLogicalDevice &);  // non copyable
  void init();
};

#endif
