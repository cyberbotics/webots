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

#ifndef WB_ACCELEROMETER_HPP
#define WB_ACCELEROMETER_HPP

#include "WbSolidDevice.hpp"

class WbSensor;
class WbLookupTable;

class WbAccelerometer : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbAccelerometer(WbTokenizer *tokenizer = NULL);
  WbAccelerometer(const WbAccelerometer &other);
  explicit WbAccelerometer(const WbNode &other);
  virtual ~WbAccelerometer();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ACCELEROMETER; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(WbDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  bool refreshSensorIfNeeded() override;

  // field accessors
  void setAcceleration(double x, double y, double z) {
    mValues[0] = x;
    mValues[1] = y;
    mValues[2] = z;
  }
  const double *getAcceleration() const { return mValues; }

private:
  // user accessible fields
  WbMFVector3 *mLookupTable;
  WbSFBool *mXAxis, *mYAxis, *mZAxis;
  WbSFDouble *mResolution;

  // other stuff
  WbSensor *mSensor;
  WbLookupTable *mLut;
  double mValues[3];  // current sensor value according to lookup table
  double mVelocity[3];
  bool mNeedToReconfigure;
  bool mWarningWasPrinted;

  // private functions
  WbAccelerometer &operator=(const WbAccelerometer &);  // non copyable
  WbNode *clone() const override { return new WbAccelerometer(*this); }
  void addConfigure(WbDataStream &);
  void init();
  void computeValue();

private slots:
  void updateLookupTable();
  void updateResolution();
};

#endif
