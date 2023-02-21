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
 * Description:  Class defining some helper functions to draw the e-puck
 */

#ifndef EPUCK_DRAWING_HELPER_HPP
#define EPUCK_DRAWING_HELPER_HPP

#include <QtCore/QRectF>
#include <QtCore/QSize>

QT_BEGIN_NAMESPACE
class QGraphicsView;
class QGraphicsScene;
class QColor;
QT_END_NAMESPACE

class EPuckDrawingHelper {
public:
  static void initQGraphicsView(QGraphicsView *view);
  static void initQGraphicsScene(QGraphicsScene *scene, const QColor &color);

  static int widgetWidth();
  static int widgetHeight();

  static int ePuckRadius();

  static double distanceSensorAngle(int i);
  static int distanceSensorRadius();
  static QSize distanceSensorSliderSize();

  static double ledAngle(int i);
  static int ledRadius();
  static QRectF ledRect();

  static int internalSliderRadius();
  static QSize internalSliderSize();

  static QSize groundSensorSliderSize();
  static int groundSensorSliderDistance();
  static int groundSensorSliderRadius();

private:
  static void drawEPuck(QGraphicsScene *scene);

  EPuckDrawingHelper();
  ~EPuckDrawingHelper();
};

#endif
