// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "MainWidget.hpp"
#include "FireBird6Representation.hpp"

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/robot.h>

#include <cmath>

// direction of the IR Devices and of the Leds from the center along the xz plane
// these angles are given in radians and starting from the right of the robot
static double angleIRDevices[NUMBER_OF_DISTANCE_SENSORS] = {
  // M_PI_2,
  // M_PI_2 - 135.0*M_PI/180.0,
  // 0.0 + 25.0*M_PI/180.0,
  // 3*M_PI_2 + 25.0*M_PI/180.0,
  // 3*M_PI_2 - 25.0*M_PI/180.0,
  M_PI,          M_PI - 45.0 * M_PI / 180.0, M_PI_2, 45.0 * M_PI / 180.0, 0.0, M_PI + 135.0 * M_PI / 180.0,
  M_PI + M_PI_2, M_PI + 45.0 * M_PI / 180.0

};
/*static double angleLed[NUMBER_OF_LEDS] = {
  M_PI_2,
  M_PI_2 - 60.0*M_PI/180.0,
  0.0,
  3*M_PI_2 + 30.0*M_PI/180.0,
  3*M_PI_2,
  3*M_PI_2 - 30.0*M_PI/180.0,
  M_PI,
  M_PI_2 + 60.0*M_PI/180.0,
  0.0,                        // body led - not used
  M_PI_2 - 15.0*M_PI/180.0    // front led
};*/

// predetermined colors (extending the colors of the Qt namespace)
static QColor red(200, 0, 0);
static QColor green(0, 210, 0);
static QColor blue(0, 0, 200);
static QColor yellow(200, 200, 0);

// constructor
MainWidget::MainWidget(QWidget *parent) : QWidget(parent) {
  // take as surface as possible
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

// destructor
MainWidget::~MainWidget() {
  // nothing to destroy
}

// Set the minimum size of this widget
QSize MainWidget::minimumSizeHint() const {
  static QSize s(180, 180);
  return s;
}

// draw the widget
void MainWidget::paintEvent(QPaintEvent *event) {
  // compute stuff
  int w = width();
  int h = height();
  int ref = (w > h) ? h : w;                 // ref = minimum between w and h
  int irDeviceRadius = ref * 60 / 100 / 2;   // 60% of the half of the ref
  int firebird6Radius = ref * 65 / 100 / 2;  // 65% of the half of the ref
  int labelRadius = ref * 87 / 100 / 2;      // 87% of the half of the ref

  // create the painter
  QPainter painter(this);

  // draw the Firebird6
  drawFirebird6(painter, firebird6Radius);

  // draw the squares representing the ir devices, draw the distance sensor values,
  // and draw the light sensor values
  drawIRSensors(painter, irDeviceRadius, labelRadius);

  // draw the speed bars
  drawSpeeds(painter, firebird6Radius);
}

// draw the squares representing the ir devices, draw the distance sensor values,
// and draw the light sensor values
void MainWidget::drawIRSensors(QPainter &p, int innerRadius, int outerRadius) {
  int w = width();
  int h = height();
  int squareSize = 5;
  QFontMetrics metrics(font());
  int fontHeight_2 = metrics.height() / 2;
  FireBird6Representation *fb6 = FireBird6Representation::instance();

  p.setBrush(QBrush(Qt::gray));
  for (int i = 0; i < 8; i++) {
    // compute the coordinates
    double uI = w / 2 + innerRadius * cos(angleIRDevices[i]);
    double vI = h / 2 - innerRadius * sin(angleIRDevices[i]);
    double uO = w / 2 + outerRadius * cos(angleIRDevices[i]);
    double vO = h / 2 - outerRadius * sin(angleIRDevices[i]);
    int offset_ds = fontHeight_2;
    int offset_ls = -fontHeight_2;
    bool dsEnabled = fb6->isDistanceSensorEnabled(i);
    bool dsSharpEnabled = fb6->isDistanceSensorSharpEnabled(i);

    if ((dsEnabled && !dsSharpEnabled) || (!dsEnabled && dsSharpEnabled)) {
      offset_ds = 0;
      offset_ls = 0;
    }

    // draw the ir representation
    p.setPen(QPen(Qt::black));
    p.save();
    p.translate(uI, vI);
    p.rotate(-angleIRDevices[i] * 180.0 / M_PI);
    p.drawRect(-squareSize / 2, -squareSize, squareSize, 2 * squareSize);
    p.restore();

    // draw the values of the distance sensors if needed
    if (dsEnabled) {
      p.setPen(QPen(red));
      float ds = fb6->distanceSensorValue(i);
      drawCenteredText(p, uO, vO + offset_ds, QString::number(ds, 'f', 2));
    }

    // draw the values of the sharp sensors if needed
    if (dsSharpEnabled) {
      p.setPen(QPen(yellow));
      float ls = fb6->distanceSensorSharpValue(i);
      drawCenteredText(p, uO, vO + offset_ls, QString::number(ls, 'f', 2));
    }
  }
}

// draw the speed sliders
void MainWidget::drawSpeeds(QPainter &p, int radius) {
  int w = width();
  int h = height();
  QFontMetrics metrics(font());
  FireBird6Representation *fb6 = FireBird6Representation::instance();
  int speeds[2] = {static_cast<int>(fb6->leftSpeed()), static_cast<int>(fb6->rightSpeed())};
  int barWidth = 10;
  int barHeight = 0.75 * (2.0 * 0.7071 * radius);
  int sliderWidth = 14;
  int sliderHeight = 4;
  int worstTextWidth_2 = metrics.tightBoundingRect("-1000").width() / 2;

  // draw the two sliders
  for (int i = 0; i < 2; i++) {
    int sign = (i == 0) ? -1 : 1;
    int centerX = w / 2 + sign * 0.7071 * radius;
    int centerY = h / 2;

    // draw the bar
    p.setPen(QPen(Qt::black));
    p.setBrush(QBrush(Qt::lightGray));
    p.drawRect(centerX - barWidth / 2, centerY - barHeight / 2, barWidth, barHeight);

    // draw the slider
    centerY -= speeds[i] * barHeight / 1000 / 2;
    p.setBrush(QBrush(blue));
    p.drawRect(centerX - sliderWidth / 2, centerY - sliderHeight / 2, sliderWidth, sliderHeight);

    // draw the text
    p.setPen(QPen(blue));
    centerX = w / 2 + sign * 0.7071 * radius - sign * (worstTextWidth_2 + sliderWidth / 2 + 1);
    drawCenteredText(p, centerX, centerY, QString::number(speeds[i]));
  }
}

// draw an Firebird6 at the center of the widget
// if the body led is enabled, draw the circle in green
void MainWidget::drawFirebird6(QPainter &p, int radius) {
  int w = width();
  int h = height();

  p.setPen(QPen(Qt::black));
  p.setBrush(QBrush(Qt::lightGray));
  p.drawEllipse(w / 2 - radius, h / 2 - radius, 2 * radius, 2 * radius);
}

// draw a text centered around the x,y point (instead of bottom-left)
void MainWidget::drawCenteredText(QPainter &p, int x, int y, const QString &text) {
  QFontMetrics metrics(font());
  QRect textBounds = metrics.tightBoundingRect(text);
  double u = -0.5 * textBounds.width();
  double v = 0.5 * textBounds.height();
  p.drawText(x + u, y + v, text);
}

void MainWidget::updateValues() {
  update();
}
