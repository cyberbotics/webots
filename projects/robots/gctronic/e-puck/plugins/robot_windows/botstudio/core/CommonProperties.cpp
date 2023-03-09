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
 * Description:  Implementation of the CommonProperties.hpp functions
 */

#include "CommonProperties.hpp"

CommonProperties::CommonProperties() {
}

CommonProperties::~CommonProperties() {
}

const QColor &CommonProperties::stateColor() {
  static const QColor &stateColor = QColor(142, 191, 232);
  return stateColor;
}

const QColor &CommonProperties::transitionColor() {
  static const QColor &transitionColor = QColor(135, 230, 133);
  return transitionColor;
}

const QColor &CommonProperties::selectionColor() {
  static const QColor &selectionColor = QColor(255, 255, 102);
  return selectionColor;
}

const QColor &CommonProperties::currentStateColor() {
  static const QColor &currentStateColor = QColor(238, 33, 84);
  return currentStateColor;
}
