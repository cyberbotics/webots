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

/*
 * Description:  Class defining the widget containing the firebird6 representation
 */

#ifndef MAIN_WIDGET_HPP
#define MAIN_WIDGET_HPP

#include <QtGui/QPainter>
#include <QtWidgets/QWidget>

class MainWidget : public QWidget {
public:
  explicit MainWidget(QWidget *parent = 0);
  virtual ~MainWidget();
  virtual QSize minimumSizeHint() const;
  void updateValues();

protected:
  virtual void paintEvent(QPaintEvent *event);
  void drawCenteredText(QPainter &p, int x, int y, const QString &text);
  void drawFirebird6(QPainter &p, int radius);
  void drawSpeeds(QPainter &p, int radius);
  void drawIRSensors(QPainter &p, int innerRadius, int outerRadius);
  void drawLeds(QPainter &p, int radius);
};

#endif  // MAIN_WIDGET_HPP
