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

#ifndef WB_GPS_HPP
#define WB_GPS_HPP

#include "WbSolidDevice.hpp"

class WbSFString;
class WbSFDouble;
class WbSensor;
class WbUTMConverter;

class WbGps : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbGps(WbTokenizer *tokenizer = NULL);
  WbGps(const WbGps &other);
  explicit WbGps(const WbNode &other);
  virtual ~WbGps();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_GPS; }
  enum CoordinateSystem { LOCAL = 0, WGS84 };
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(WbDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;

private:
  // user accessible fields
  WbSFString *mType;
  WbSFDouble *mAccuracy;
  WbSFDouble *mNoiseCorrelation;
  WbSFDouble *mResolution;
  WbSFDouble *mSpeedNoise;
  WbSFDouble *mSpeedResolution;

  // other fields
  WbSensor *mSensor;
  WbVector3 mMeasuredPosition;
  WbVector3 mPreviousPosition;
  WbVector3 mSpeedVector;
  double mMeasuredSpeed;
  WbUTMConverter *mUTMConverter;
  bool mNeedToUpdateCoordinateSystem;

  WbGps &operator=(const WbGps &);  // non copyable
  WbNode *clone() const override { return new WbGps(*this); }
  void init();

  void addConfigureToStream(WbDataStream &stream);

private slots:
  void updateResolution();
  void updateSpeedNoise();
  void updateSpeedResolution();
  void updateCorrelation();
  void updateCoordinateSystem();
  void updateReferences();
};

#endif
