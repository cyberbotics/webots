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

#include "WbGyro.hpp"

#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbLookupTable.hpp"
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbSFDouble.hpp"
#include "WbSensor.hpp"

#include "../../controller/c/messages.h"

#include <ode/ode.h>
#include <QtCore/QDataStream>
#include <cassert>

void WbGyro::init() {
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

  mNeedToReconfigure = false;
}

WbGyro::WbGyro(WbTokenizer *tokenizer) : WbSolidDevice("Gyro", tokenizer) {
  init();
}

WbGyro::WbGyro(const WbGyro &other) : WbSolidDevice(other) {
  init();
}

WbGyro::WbGyro(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbGyro::~WbGyro() {
  delete mLut;
  delete mSensor;
}

void WbGyro::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();
  updateLookupTable();
}

void WbGyro::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mLookupTable, &WbMFVector3::changed, this, &WbGyro::updateLookupTable);
  connect(mResolution, &WbSFDouble::changed, this, &WbGyro::updateResolution);
}

void WbGyro::updateLookupTable() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;

  // create the lookup table
  delete mLut;
  mLut = new WbLookupTable(*mLookupTable);

  mNeedToReconfigure = true;
}

void WbGyro::updateResolution() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void WbGyro::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    default:
      assert(0);
  }
}

void WbGyro::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_GYRO_DATA;
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void WbGyro::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

void WbGyro::addConfigure(WbDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mLookupTable->size();
  for (int i = 0; i < mLookupTable->size(); i++) {
    stream << (double)mLookupTable->item(i).x();
    stream << (double)mLookupTable->item(i).y();
    stream << (double)mLookupTable->item(i).z();
  }
  mNeedToReconfigure = false;
}

bool WbGyro::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbGyro::computeValue() {
  dBodyID bodyID = body();
  if (!bodyID)
    bodyID = upperSolidBody();

  if (bodyID) {
    // get angular velocity in global coordinate system
    const dReal *v = dBodyGetAngularVel(bodyID);
    WbVector3 globalVelocity(v);

    // from global to Gyro's local coordinate system
    WbVector3 localVelocity = globalVelocity * matrix();

    // lookup
    mValues[0] = mXAxis->isTrue() ? mLut->lookup(localVelocity.x()) : NAN;
    mValues[1] = mYAxis->isTrue() ? mLut->lookup(localVelocity.y()) : NAN;
    mValues[2] = mZAxis->isTrue() ? mLut->lookup(localVelocity.z()) : NAN;

    // apply resolution if needed
    if (mResolution->value() != -1.0) {
      if (mXAxis->isTrue())
        mValues[0] = WbMathsUtilities::discretize(mValues[0], mResolution->value());
      if (mYAxis->isTrue())
        mValues[1] = WbMathsUtilities::discretize(mValues[1], mResolution->value());
      if (mZAxis->isTrue())
        mValues[2] = WbMathsUtilities::discretize(mValues[2], mResolution->value());
    }
  } else
    parsingWarn(tr("this node or its parents requires a 'physics' field to be functional."));
}
