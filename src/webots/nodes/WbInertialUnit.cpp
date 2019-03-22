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

#include "WbInertialUnit.hpp"
#include "WbFieldChecker.hpp"
#include "WbLookupTable.hpp"
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"

#include "../../lib/Controller/api/messages.h"

#include <ode/ode.h>
#include <QtCore/QDataStream>
#include <cassert>

void WbInertialUnit::init() {
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

WbInertialUnit::WbInertialUnit(WbTokenizer *tokenizer) : WbSolidDevice("InertialUnit", tokenizer) {
  init();
}

WbInertialUnit::WbInertialUnit(const WbInertialUnit &other) : WbSolidDevice(other) {
  init();
}

WbInertialUnit::WbInertialUnit(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbInertialUnit::~WbInertialUnit() {
  delete mLut;
  delete mSensor;
}

void WbInertialUnit::preFinalize() {
  WbSolidDevice::preFinalize();
  mSensor = new WbSensor();
  updateLookupTable();
}

void WbInertialUnit::postFinalize() {
  WbSolidDevice::postFinalize();
  connect(mLookupTable, &WbMFVector3::changed, this, &WbInertialUnit::updateLookupTable);
  connect(mResolution, &WbSFDouble::changed, this, &WbInertialUnit::updateResolution);
}

void WbInertialUnit::updateLookupTable() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;

  // create the lookup table
  delete mLut;
  mLut = new WbLookupTable(*mLookupTable);
}

void WbInertialUnit::updateResolution() {
  WbFieldChecker::checkDoubleIsPositiveOrDisabled(this, mResolution, -1.0, -1.0);
}

void WbInertialUnit::handleMessage(QDataStream &stream) {
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

void WbInertialUnit::writeAnswer(QDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << (short unsigned int)tag();
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];

    mSensor->resetPendingValue();
  }
}

void WbInertialUnit::writeConfigure(QDataStream &) {
  mSensor->connectToRobotSignal(robot());
}

bool WbInertialUnit::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbInertialUnit::computeValue() {
  // get north and -gravity in global coordinate systems
  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  const WbVector3 &north = wi->northDirection().normalized();
  const WbVector3 &minusGravity = -wi->gravity().normalized();

  WbMatrix3 rm(north, minusGravity, north.cross(minusGravity));  // reference frame
  rm.transpose();
  const WbMatrix3 &e = rotationMatrix() * rm;  // extrensic rotation matrix e = Y(yaw) Z(pitch) X(roll) w.r.t reference frame
  const double roll = atan2(-e(1, 2), e(1, 1));
  const double pitch = asin(e(1, 0));
  assert(!std::isnan(pitch));
  const double yaw = -atan2(e(2, 0), e(0, 0));

  mValues[0] = mXAxis->isTrue() ? mLut->lookup(roll) : NAN;
  mValues[1] = mZAxis->isTrue() ? mLut->lookup(pitch) : NAN;
  mValues[2] = mYAxis->isTrue() ? mLut->lookup(yaw) : NAN;

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
