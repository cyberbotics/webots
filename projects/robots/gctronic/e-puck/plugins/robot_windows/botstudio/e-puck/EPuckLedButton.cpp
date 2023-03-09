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
 * Description:  Implementation of the EPuckLedButton.hpp functions
 */

#include "EPuckLedButton.hpp"
#include "EPuckDrawingHelper.hpp"
#include "EPuckFacade.hpp"

#include <QtGui/QBrush>

EPuckLedButton::EPuckLedButton() :
  QObject(),
  QGraphicsEllipseItem(EPuckDrawingHelper::ledRect()),
  mValue(EPuckFacade::NONE),
  mIndex(-1) {
  updateColor();
}

void EPuckLedButton::setValue(int value) {
  mValue = value;
  updateColor();
  emit stateChanged();
}

void EPuckLedButton::updateColor() {
  switch (mValue) {
    case EPuckFacade::OFF:
      setBrush(QBrush(Qt::black));
      break;
    case EPuckFacade::ON:
      setBrush(QBrush(Qt::red));
      break;
    default:
      setBrush(QBrush(Qt::gray));
  }
}

void EPuckLedButton::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  setValue((mValue + 1) % 3);
  QGraphicsEllipseItem::mousePressEvent(event);
}
