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

#include "WbSFRotation.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

#include <cassert>

void WbSFRotation::readSFRotation(WbTokenizer *tokenizer, const QString &worldPath) {
  try {
    double xCoordinate = tokenizer->nextToken()->toDouble();
    double yCoordinate = tokenizer->nextToken()->toDouble();
    double zCoordinate = tokenizer->nextToken()->toDouble();
    double angleValue = tokenizer->nextToken()->toDouble();
    mValue.setAxisAngle(xCoordinate, yCoordinate, zCoordinate, angleValue);
  } catch (...) {
    tokenizer->reportError(tr("Expected floating point value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void WbSFRotation::setValue(const WbRotation &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changed();
}

void WbSFRotation::setValue(double x, double y, double z, double angle) {
  if (mValue == WbRotation(x, y, z, angle))
    return;

  mValue.setAxisAngle(x, y, z, angle);
  emit changed();
}

void WbSFRotation::setValueByUser(const WbRotation &v, bool changedFromSupervisor) {
  if (mValue == v)
    return;

  mValue = v;
  // emit before changed() to be sure this instance is not deleted during PROTO template regeneration
  emit changedByUser(changedFromSupervisor);
  emit changed();
}

double WbSFRotation::component(int index) const {
  switch (index) {
    case 0:
      return mValue.x();
    case 1:
      return mValue.y();
    case 2:
      return mValue.z();
    case 3:
      return mValue.angle();
    default:
      assert(0);
      return 0.0;
  }
}

void WbSFRotation::setComponent(int index, double d) {
  if (component(index) == d)
    return;

  switch (index) {
    case 0:
      mValue.setX(d);
      break;
    case 1:
      mValue.setY(d);
      break;
    case 2:
      mValue.setZ(d);
      break;
    case 3:
      mValue.setAngle(d);
      break;
    default:
      assert(0);
      break;
  }
  emit changed();
}

WbSFRotation &WbSFRotation::operator=(const WbSFRotation &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool WbSFRotation::equals(const WbValue *other) const {
  const WbSFRotation *that = dynamic_cast<const WbSFRotation *>(other);
  return that && *this == *that;
}

void WbSFRotation::copyFrom(const WbValue *other) {
  const WbSFRotation *that = dynamic_cast<const WbSFRotation *>(other);
  *this = *that;
}
