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

#include "WbMFVector3.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFVector3::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  double x = tokenizer->nextToken()->toDouble();
  double y = tokenizer->nextToken()->toDouble();
  double z = tokenizer->nextToken()->toDouble();
  mVector.append(WbVector3(x, y, z));
}

void WbMFVector3::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void WbMFVector3::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toVector3());
  emit itemInserted(index);
  emit changed();
}

void WbMFVector3::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFVector3::setItem(int index, const WbVector3 &vec) {
  assert(index >= 0 && index < size());
  if (mVector[index] != vec) {
    mVector[index] = vec;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFVector3::rescale(const WbVector3 &scale) {
  double sx = scale.x();
  double sy = scale.y();
  double sz = scale.z();
  if (sx == 1.0 && sy == 1.0 && sz == 1.0)
    return;

  const int vectorSize = mVector.size();

  for (int index = 0; index < vectorSize; index++) {
    const WbVector3 &previousValue = mVector[index];
    mVector[index].setXyz(sx * previousValue.x(), sy * previousValue.y(), sz * previousValue.z());
    emit itemChanged(index);
  }

  emit changed();
}

void WbMFVector3::rescaleAndTranslate(int coordinate, double scale, double translation) {
  if (scale == 1.0) {
    translate(coordinate, translation);
    return;
  }

  const int vectorSize = mVector.size();
  for (int index = 0; index < vectorSize; index++) {
    mVector[index][coordinate] = scale * mVector[index][coordinate] + translation;
    emit itemChanged(index);
  }

  emit changed();
}

void WbMFVector3::rescaleAndTranslate(const WbVector3 &scale, const WbVector3 &translation) {
  double sx = scale.x();
  double sy = scale.y();
  double sz = scale.z();

  if (sx == 1.0 && sy == 1.0 && sz == 1.0) {
    translate(translation);
    return;
  }

  double tx = translation.x();
  double ty = translation.y();
  double tz = translation.z();
  const int vectorSize = mVector.size();

  for (int index = 0; index < vectorSize; index++) {
    const WbVector3 &previousValue = mVector[index];
    mVector[index].setXyz(sx * previousValue.x() + tx, sy * previousValue.y() + ty, sz * previousValue.z() + tz);
    emit itemChanged(index);
  }

  emit changed();
}

void WbMFVector3::translate(int coordinate, double translation) {
  if (translation == 0.0)
    return;

  const int vectorSize = mVector.size();
  for (int index = 0; index < vectorSize; index++) {
    mVector[index][coordinate] = mVector[index][coordinate] + translation;
    emit itemChanged(index);
  }

  emit changed();
}

void WbMFVector3::translate(const WbVector3 &translation) {
  double x = translation.x();
  double y = translation.y();
  double z = translation.z();

  if (x == 0.0 && y == 0.0 && z == 0.0)
    return;

  const int vectorSize = mVector.size();

  for (int index = 0; index < vectorSize; index++) {
    const WbVector3 &previousValue = mVector[index];
    mVector[index].setXyz(previousValue.x() + x, previousValue.y() + y, previousValue.z() + z);
    emit itemChanged(index);
  }

  emit changed();
}

void WbMFVector3::addItem(const WbVector3 &vec) {
  mVector.append(vec);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFVector3::insertItem(int index, const WbVector3 &vec) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, vec);
  emit itemInserted(index);
  emit changed();
}

void WbMFVector3::mult(double factor) {
  for (int i = 0, size = mVector.size(); i < size; ++i)
    mVector[i] *= factor;
}

WbMFVector3 &WbMFVector3::operator=(const WbMFVector3 &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFVector3::equals(const WbValue *other) const {
  const WbMFVector3 *that = dynamic_cast<const WbMFVector3 *>(other);
  return that && *this == *that;
}

void WbMFVector3::copyFrom(const WbValue *other) {
  const WbMFVector3 *that = dynamic_cast<const WbMFVector3 *>(other);
  *this = *that;
}
