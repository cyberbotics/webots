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
 * Description:  Class defining a led button
 */

#ifndef EPUCKmLed_BUTTON_HPP
#define EPUCKmLed_BUTTON_HPP

#include <QtCore/QObject>
#include <QtWidgets/QGraphicsEllipseItem>

class EPuckLedButton : public QObject, public QGraphicsEllipseItem {
  Q_OBJECT

public:
  EPuckLedButton();
  virtual ~EPuckLedButton() {}

  void setIndex(int index) { mIndex = index; }
  void setValue(int value);

  int index() const { return mIndex; }
  int value() const { return mValue; }

  virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);

signals:
  void stateChanged();

protected:
  void updateColor();

  int mValue;
  int mIndex;
};

#endif
