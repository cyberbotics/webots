// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbCompass.hpp"
#include "WbFieldChecker.hpp"
#include "WbLookupTable.hpp"
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"

#include "../../lib/Controller/api/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

void WbCompass::init() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;
  mLut = NULL;
  mSensor = NULL;

  mLookupTable = findMFVector3("lookupTable");
  mXAxis = findSFBool("xAxis");
  mYAxis = findSFBool("yAxis");
  mZAxis = findSFBool("zAxis");
  mResolution = findSFDouble("resolution");
}

WbCompass::WbCompass(WbTokenizer *tokenizer) : WbSolidDevice("Compass", tokenizer) {
  init();
}

WbCompass::WbCompass(const WbCompass &other) : WbSolidDevice(other) {
  init();
}

WbCompass::WbCompass(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbCompass::~WbCompass() {
  delete mLut;
  delete mSensor;
}

void WbCompass::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();
  updateLookupTable();
}

void WbCompass::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mLookupTable, &WbMFVector3::changed, this, &WbCompass::updateLookupTable);
  connect(mResolution, &WbSFDouble::changed, this, &WbCompass::updateResolution);
}

void WbCompass::updateLookupTable() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;

  // create the lookup table
  delete mLut;
  mLut = new WbLookupTable(*mLookupTable);
}

void WbCompass::updateResolution() {
  WbFieldChecker::checkDoubleIsPositiveOrDisabled(this, mResolution, -1.0, -1.0);
}

void WbCompass::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> (unsigned char &)command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> (short &)refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    default:
      assert(0);
  }
}

void WbCompass::writeAnswer(QDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << (short unsigned int)tag();
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];
    mSensor->resetPendingValue();
  }
}

void WbCompass::writeConfigure(QDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
}

bool WbCompass::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbCompass::computeValue() {
  // get global north
  WbWorldInfo *info = WbWorld::instance()->worldInfo();
  assert(info);
  const WbVector3 &globalNorth = info->northDirection();

  // convert from global to Compass local coordinate system
  WbVector3 localNorth = globalNorth * matrix();

  // normalize
  localNorth.normalize();

  // lookup
  mValues[0] = mXAxis->isTrue() ? mLut->lookup(localNorth.x()) : NAN;
  mValues[1] = mYAxis->isTrue() ? mLut->lookup(localNorth.y()) : NAN;
  mValues[2] = mZAxis->isTrue() ? mLut->lookup(localNorth.z()) : NAN;

  // apply resolution if needed
  if (mResolution->value() != -1.0) {
    if (mXAxis->isTrue())
      mValues[0] = WbMathsUtilities::discretize(mValues[0], mResolution->value());
    if (mYAxis->isTrue())
      mValues[1] = WbMathsUtilities::discretize(mValues[1], mResolution->value());
    if (mZAxis->isTrue())
      mValues[2] = WbMathsUtilities::discretize(mValues[2], mResolution->value());
  }
}
