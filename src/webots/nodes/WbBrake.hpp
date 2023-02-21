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

// Concrete class representing a brake placed in mechanical joints

#ifndef WB_BRAKE_HPP
#define WB_BRAKE_HPP

#include "WbJointDevice.hpp"

class WbBrake : public WbJointDevice {
  Q_OBJECT

public:
  virtual ~WbBrake() {}
  explicit WbBrake(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbBrake(WbTokenizer *tokenizer = NULL);
  WbBrake(const WbBrake &other);
  explicit WbBrake(const WbNode &other);
  int nodeType() const override { return WB_NODE_BRAKE; }

  double getBrakingDampingConstant() const { return mBrakingDampingConstant; }

  // inherited from WbBaseNode
  void reset(const QString &id) override;

  // inherited from WbDevice
  void writeConfigure(WbDataStream &stream) override;
  void writeAnswer(WbDataStream &stream) override;
  void handleMessage(QDataStream &stream) override;

signals:
  // emitted when received command from controller
  void brakingChanged();

private:
  WbBrake &operator=(const WbBrake &);  // non copyable
  WbNode *clone() const override { return new WbBrake(*this); }
  void init();
  double mBrakingDampingConstant;
  WbDeviceTag *mRequestedDeviceTag;
};

#endif
