// Copyright 1996-2020 Cyberbotics Ltd.
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
 * Description:  Abstract tab used to build specifi tabs
 */

#ifndef ABSTRACT_WIDGET_HPP
#define ABSTRACT_WIDGET_HPP

#include <QtWidgets/QWidget>

#include <graph2d/Graph2D.hpp>

class QGridLayout;
class QCheckBox;
class QLabel;

using namespace webotsQtUtils;

class AbstractWidget : public QWidget {
  Q_OBJECT

public:
  AbstractWidget(QWidget *parent = 0);
  virtual ~AbstractWidget();
  virtual void update() = 0;

public slots:
  void updateEnableCheckBoxText();

protected:
  webotsQtUtils::Graph2D *mGraph;

  // Disable cppcheck errors reproducible only on Windows on David's computer
  // cppcheck-suppress unsafeClassCanLeak
  QGridLayout *mLayout;
  // cppcheck-suppress unsafeClassCanLeak
  QCheckBox *mEnableCheckBox;
  // cppcheck-suppress unsafeClassCanLeak
  QLabel *mValueLabel;

  // colors
  static QColor black() { return QColor(0, 0, 0); }
  static QColor red() { return QColor(200, 50, 50); }
  static QColor green() { return QColor(50, 200, 50); }
  static QColor blue() { return QColor(50, 50, 200); }

  static int pointsKeptNumber() { return 200; }
};

#endif  // ABSTRACT_WIDGET_HPP
