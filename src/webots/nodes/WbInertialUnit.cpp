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
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbRandom.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"

#include "../../Controller/api/messages.h"

#include <ode/ode.h>
#include <QtCore/QDataStream>
#include <cassert>

void WbInertialUnit::init() {
  mSensor = NULL;

  mNoise = findSFDouble("noise");
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
  delete mSensor;
}

void WbInertialUnit::preFinalize() {
  WbSolidDevice::preFinalize();
  mSensor = new WbSensor();
}

void WbInertialUnit::postFinalize() {
  WbSolidDevice::postFinalize();
  connect(mResolution, &WbSFDouble::changed, this, &WbInertialUnit::updateResolution);
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
    stream << (double)mQuaternion.x() << (double)mQuaternion.y() << (double)mQuaternion.z() << (double)mQuaternion.w();

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void WbInertialUnit::addConfigure(QDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (double)mNoise->value();
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

  if (mNoise->value() != 0.0) {
    const double noise = mNoise->value() * M_PI;
    e *= WbMatrix3(noise * WbRandom::nextGaussian(), noise * WbRandom::nextGaussian(), noise * WbRandom::nextGaussian());
  }

  if (!mXAxis->isTrue()) {
    const double roll = atan2(-e(1, 2), e(1, 1));
    e *= WbMatrix3(1, 0, 0, roll).transposed();
  }

  mQuaternion = e.toQuaternion();

  // apply resolution if needed
  if (mResolution->value() != -1.0) {
    mQuaternion.setX(WbMathsUtilities::discretize(mQuaternion.x(), mResolution->value()));
    mQuaternion.setY(WbMathsUtilities::discretize(mQuaternion.y(), mResolution->value()));
    mQuaternion.setZ(WbMathsUtilities::discretize(mQuaternion.z(), mResolution->value()));
    mQuaternion.setW(WbMathsUtilities::discretize(mQuaternion.w(), mResolution->value()));
  }
}
