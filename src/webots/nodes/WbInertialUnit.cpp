// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "../../Controller/api/messages.h"

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

  mNeedToReconfigure = false;
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

  mNeedToReconfigure = true;
}

void WbInertialUnit::updateResolution() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void WbInertialUnit::handleMessage(QDataStream &stream) {
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

void WbInertialUnit::writeAnswer(QDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_INERTIAL_UNIT_DATA;
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];

    stream << (double)mQuaternion.x() << (double)mQuaternion.y() << (double)mQuaternion.z() << (double)mQuaternion.w();

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void WbInertialUnit::addConfigure(QDataStream &stream) {
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
  const WbVector3 &north = wi->northVector();
  WbVector3 minusGravity = -wi->gravityUnitVector();
  WbMatrix3 rm(north, minusGravity, north.cross(minusGravity));  // reference frame
  rm.transpose();
  WbMatrix3 e = rotationMatrix() * rm;  // extrensic rotation matrix e = Y(yaw) Z(pitch) X(roll) w.r.t reference frame

  if (!mXAxis->isTrue()) {
    const double roll = atan2(-e(1, 2), e(1, 1));
    e *= WbMatrix3(1, 0, 0, roll).transposed();
  }

  mQuaternion = e.toQuaternion();

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
