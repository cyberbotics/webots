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
 * Description:  Implementation of the EPuckDrawingHelper.hpp functions
 */

#include "EPuckDrawingHelper.hpp"
#include "EPuckFacade.hpp"

#include <QtCore/qmath.h>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>

static double gAngleDistanceSensor[EPuckFacade::NUMBER_OF_DISTANCE_SENSORS] = {
  M_PI_2 - 20.0 * M_PI / 180.0,       M_PI_2 - 55.0 * M_PI / 180.0,       0.0,
  3.0 * M_PI_2 + 25.0 * M_PI / 180.0, 3.0 * M_PI_2 - 25.0 * M_PI / 180.0, M_PI,
  M_PI_2 + 55.0 * M_PI / 180.0,       M_PI_2 + 20.0 * M_PI / 180.0};
static double gAngleLed[EPuckFacade::NUMBER_OF_LEDS] = {
  M_PI_2,
  M_PI_2 - 60.0 * M_PI / 180.0,
  0.0,
  3.0 * M_PI_2 + 30.0 * M_PI / 180.0,
  3.0 * M_PI_2,
  3.0 * M_PI_2 - 30.0 * M_PI / 180.0,
  M_PI,
  M_PI_2 + 60.0 * M_PI / 180.0,
  0.0,                          // body led - not used
  M_PI_2 - 15.0 * M_PI / 180.0  // front led
};

EPuckDrawingHelper::EPuckDrawingHelper() {
}

EPuckDrawingHelper::~EPuckDrawingHelper() {
}

double EPuckDrawingHelper::distanceSensorAngle(int i) {
  if (i >= 0 && i < 8)
    return gAngleDistanceSensor[i];
  return 0.0;
}

double EPuckDrawingHelper::ledAngle(int i) {
  if (i >= 0 && i < 10)
    return gAngleLed[i];
  return 0.0;
}

QRectF EPuckDrawingHelper::ledRect() {
  return QRectF(-5, -5, 10, 10);
}

void EPuckDrawingHelper::initQGraphicsView(QGraphicsView *view) {
  int w = widgetWidth();
  int h = widgetHeight();

  view->setRenderHint(QPainter::Antialiasing);
  view->setSceneRect(-w / 2, -h / 2, w, h);
  view->setMinimumSize(w, h);
  view->centerOn(0, 0);
  view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

void EPuckDrawingHelper::initQGraphicsScene(QGraphicsScene *scene, const QColor &color) {
  scene->setBackgroundBrush(color);
  drawEPuck(scene);
}

int EPuckDrawingHelper::widgetWidth() {
  return 420;
}

int EPuckDrawingHelper::widgetHeight() {
  return 380;
}

int EPuckDrawingHelper::ePuckRadius() {
  return 80;
}

int EPuckDrawingHelper::distanceSensorRadius() {
  return 120;
}

int EPuckDrawingHelper::ledRadius() {
  return 70;
}

int EPuckDrawingHelper::internalSliderRadius() {
  return 50;
}

QSize EPuckDrawingHelper::distanceSensorSliderSize() {
  return QSize(100, 12);
}

QSize EPuckDrawingHelper::internalSliderSize() {
  return QSize(60, 12);
}

void EPuckDrawingHelper::drawEPuck(QGraphicsScene *scene) {
  int r = ePuckRadius();
  scene->addEllipse(-r, -r, 2 * r, 2 * r, QPen(), QBrush(QColor(Qt::gray)));
}

QSize EPuckDrawingHelper::groundSensorSliderSize() {
  return QSize(50, 12);
}

int EPuckDrawingHelper::groundSensorSliderDistance() {
  return 15;
}

int EPuckDrawingHelper::groundSensorSliderRadius() {
  return 105;
}
