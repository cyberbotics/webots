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

//
//  WbPositionSensor.hpp
//

// Concrete class representing position sensors placed in mechanical joints

#ifndef WB_POSITION_SENSOR_HPP
#define WB_POSITION_SENSOR_HPP

#include "WbJointDevice.hpp"
#include "WbSensor.hpp"

class WbPositionSensor : public WbJointDevice {
  Q_OBJECT

public:
  virtual ~WbPositionSensor() { delete mSensor; }
  explicit WbPositionSensor(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbPositionSensor(WbTokenizer *tokenizer = NULL);
  WbPositionSensor(const WbPositionSensor &other);
  explicit WbPositionSensor(const WbNode &other);
  int nodeType() const override { return WB_NODE_POSITION_SENSOR; }

  // inherited from WbBaseNode
  void postFinalize() override;

  // inherited from WbDevice
  void writeConfigure(WbDataStream &stream) override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(WbDataStream &stream) override;
  bool refreshSensorIfNeeded() override;

  // inherited from WbJointDevice
  double position() const;

private:
  // user accessible field
  WbSFDouble *mResolution;
  WbSFDouble *mNoise;

  WbSensor *mSensor;
  double mValue;
  WbPositionSensor &operator=(const WbPositionSensor &);  // non copyable
  WbNode *clone() const override { return new WbPositionSensor(*this); }
  void init();

  WbDeviceTag *mRequestedDeviceTag;

private slots:
  void updateResolution();
  void updateNoise();
};

#endif
