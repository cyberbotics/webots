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

#include "WbAltimeter.hpp"

#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbMathsUtilities.hpp"
#include "WbRandom.hpp"
#include "WbSFDouble.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>

#include <cassert>

void WbAltimeter::init() {
  mType = findSFString("type");
  mAccuracy = findSFDouble("accuracy");
  mResolution = findSFDouble("resolution");
  mSensor = NULL;
}

WbAltimeter::WbAltimeter(WbTokenizer *tokenizer) : WbSolidDevice("Altimeter", tokenizer) {
  init();
}

WbAltimeter::WbAltimeter(const WbAltimeter &other) : WbSolidDevice(other) {
  init();
}

WbAltimeter::WbAltimeter(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbAltimeter::~WbAltimeter() {
  delete mSensor;
}

void WbAltimeter::preFinalize() {
  WbSolidDevice::preFinalize();
  mSensor = new WbSensor();
}

void WbAltimeter::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mResolution, &WbSFDouble::changed, this, &WbAltimeter::updateResolution);
}

void WbAltimeter::updateResolution() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

bool WbAltimeter::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  const WbVector3 &t = matrix().translation();

  // compute current altitude
  double accuracy = mAccuracy->value();

  const WbVector3 reference = WbWorld::instance()->worldInfo()->gpsReference();
  const QString &coordinateSystem = WbWorld::instance()->worldInfo()->coordinateSystem();
  const int upIndex = coordinateSystem.indexOf('U');
  if (WbWorld::instance()->worldInfo()->gpsCoordinateSystem() == "WGS84")
    mMeasuredAltitude = reference[2];
  else
    mMeasuredAltitude = reference[upIndex];

  mMeasuredAltitude += t[upIndex];  // get exact altitude
  // add noise if necessary
  if (accuracy != 0.0)
    mMeasuredAltitude += accuracy * WbRandom::nextGaussian();
  // apply resolution if necessary
  if (mResolution->value() != -1.0)
    mMeasuredAltitude = WbMathsUtilities::discretize(mMeasuredAltitude, mResolution->value());
  mSensor->updateTimer();
  return true;
}

void WbAltimeter::reset(const QString &id) {
  WbSolidDevice::reset(id);
}

void WbAltimeter::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      break;
    default:
      assert(0);
  }
}

void WbAltimeter::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_ALTIMETER_DATA;
    stream << (double)mMeasuredAltitude;

    mSensor->resetPendingValue();
  }
}

void WbAltimeter::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
}
