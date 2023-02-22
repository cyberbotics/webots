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

/*
 * Description:  Implementation of the AutomatonObject.hpp functions
 */

#include "AutomatonObject.hpp"

int AutomatonObject::cCurrentGlobalId = 0;

AutomatonObject::AutomatonObject(const QPointF &position) : QObject(), mPosition(position), mIsSelected(false) {
  mUniqueId = cCurrentGlobalId++;
  mName = QString::number(mUniqueId);
  setObjectName(mName);
}

void AutomatonObject::setName(const QString &name) {
  mName = name;
  emit nameUpdated();
}

void AutomatonObject::setSelected(bool isSelected) {
  mIsSelected = isSelected;
  emit selectionChanged();
}

void AutomatonObject::setPosition(const QPointF &position) {
  mPosition = position;
  emit positionUpdated();
}

void AutomatonObject::computeCurrentGlobalId(int id) {
  if (cCurrentGlobalId <= id)
    cCurrentGlobalId = id + 1;
}
