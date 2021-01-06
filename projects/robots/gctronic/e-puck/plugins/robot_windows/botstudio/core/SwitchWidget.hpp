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
 * Description:  Class defining a widget allowing to switch between three viewers
 */

#ifndef SWITCH_WIDGET_HPP
#define SWITCH_WIDGET_HPP

#include <QtWidgets/QWidget>

class Model;
class RobotConditionWidget;
class RobotStateWidget;
class RobotViewWidget;

QT_BEGIN_NAMESPACE
class QStackedLayout;
QT_END_NAMESPACE

class SwitchWidget : public QWidget {
  Q_OBJECT

public:
  explicit SwitchWidget(QWidget *parent = NULL);
  virtual ~SwitchWidget();
  void updateValues();

protected slots:
  void selectWidget();

protected:
  enum { StateWidget, TransitionWidget, RobotWidget };

  RobotStateWidget *mStateWidget;
  RobotConditionWidget *mConditionWidget;
  RobotViewWidget *mViewWidget;

  Model *mModel;
  QStackedLayout *mStackedLayout;
};

#endif
