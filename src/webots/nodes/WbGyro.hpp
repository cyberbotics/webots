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

#ifndef WB_GYRO_HPP
#define WB_GYRO_HPP

#include "WbSolidDevice.hpp"

class WbSensor;
class WbMFVector3;
class WbSFBool;
class WbLookupTable;

class WbGyro : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbGyro(WbTokenizer *tokenizer = NULL);
  WbGyro(const WbGyro &other);
  explicit WbGyro(const WbNode &other);
  virtual ~WbGyro();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_GYRO; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(WbDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  bool refreshSensorIfNeeded() override;

private:
  // user accessible fields
  WbMFVector3 *mLookupTable;
  WbSFBool *mXAxis, *mYAxis, *mZAxis;
  WbSFDouble *mResolution;

  // other stuff
  WbSensor *mSensor;
  WbLookupTable *mLut;
  double mValues[3];  // current sensor value according to lookup table
  bool mNeedToReconfigure;

  // private functions
  WbGyro &operator=(const WbGyro &);  // non copyable
  WbNode *clone() const override { return new WbGyro(*this); }
  void init();
  void computeValue();
  void addConfigure(WbDataStream &);

private slots:
  void updateLookupTable();
  void updateResolution();
};

#endif
