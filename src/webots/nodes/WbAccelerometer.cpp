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

#include "WbAccelerometer.hpp"

#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbLookupTable.hpp"
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"

#include <ode/ode.h>
#include <QtCore/QDataStream>
#include <cassert>
#include "../../controller/c/messages.h"

void WbAccelerometer::init() {
  for (int i = 0; i < 3; i++) {
    mVelocity[i] = 0.0;
    mValues[i] = 0.0;
  }

  mLut = NULL;
  mSensor = NULL;

  mLookupTable = findMFVector3("lookupTable");
  mXAxis = findSFBool("xAxis");
  mYAxis = findSFBool("yAxis");
  mZAxis = findSFBool("zAxis");
  mResolution = findSFDouble("resolution");

  mNeedToReconfigure = false;
  mWarningWasPrinted = false;
}

WbAccelerometer::WbAccelerometer(WbTokenizer *tokenizer) : WbSolidDevice("Accelerometer", tokenizer) {
  init();
}

WbAccelerometer::WbAccelerometer(const WbAccelerometer &other) : WbSolidDevice(other) {
  init();
}

WbAccelerometer::WbAccelerometer(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbAccelerometer::~WbAccelerometer() {
  delete mLut;
  delete mSensor;
}

void WbAccelerometer::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();
  updateLookupTable();
}

void WbAccelerometer::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mLookupTable, &WbMFVector3::changed, this, &WbAccelerometer::updateLookupTable);
  connect(mResolution, &WbSFDouble::changed, this, &WbAccelerometer::updateResolution);
}

void WbAccelerometer::updateLookupTable() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;

  // create the lookup table
  delete mLut;
  mLut = new WbLookupTable(*mLookupTable);

  mNeedToReconfigure = true;
}

void WbAccelerometer::updateResolution() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void WbAccelerometer::handleMessage(QDataStream &stream) {
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

void WbAccelerometer::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

void WbAccelerometer::addConfigure(WbDataStream &stream) {
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

void WbAccelerometer::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_ACCELEROMETER_DATA;
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

bool WbAccelerometer::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbAccelerometer::computeValue() {
  // set acceleration due to gravity
  const WbVector3 &gravity = WbWorld::instance()->worldInfo()->gravityVector();
  WbVector3 acceleration(-gravity);

  // add other acceleration (computed from changes in velocity)
  dBodyID upperSolidBodyId = upperSolid()->bodyMerger();
  if (upperSolidBodyId) {
    dVector3 newVelocity;
    const WbVector3 &t = position();
    dBodyGetPointVel(upperSolidBodyId, t.x(), t.y(), t.z(), newVelocity);
    const double inverseDt = 1000.0 / mSensor->elapsedTime();

    for (int i = 0; i < 3; ++i) {
      acceleration[i] += (newVelocity[i] - mVelocity[i]) * inverseDt;
      mVelocity[i] = newVelocity[i];
    }
  } else {
    if (!mWarningWasPrinted) {
      warn(tr("Parent of Accelerometer node has no physics: measurements may be wrong."));
      mWarningWasPrinted = true;
    }
    mValues[0] = mValues[1] = mValues[2] = NAN;
    return;
  }

  const WbVector3 &result = acceleration * matrix();

  // apply lookup table
  mValues[0] = mXAxis->isTrue() ? mLut->lookup(result.x()) : NAN;
  mValues[1] = mYAxis->isTrue() ? mLut->lookup(result.y()) : NAN;
  mValues[2] = mZAxis->isTrue() ? mLut->lookup(result.z()) : NAN;

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
