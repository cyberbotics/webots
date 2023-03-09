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
 * Description:  Class defining a widget allowing to view the selected state
 */

#ifndef EPUCK_STATE_WIDGET_HPP
#define EPUCK_STATE_WIDGET_HPP

#include <core/RobotStateWidget.hpp>
#include "EPuckFacade.hpp"

class EPuckSlider;
class EPuckLedButton;

class EPuckStateWidget : public RobotStateWidget {
  Q_OBJECT

public:
  explicit EPuckStateWidget(QWidget *parent = NULL);
  virtual ~EPuckStateWidget();
  virtual void setActuatorCommand(RobotActuatorCommand *command);

protected slots:
  void updateSpeedCommand();
  void updateLedCommand();

protected:
  EPuckSlider *mSpeedSliders[2];
  EPuckLedButton *mLedButtons[EPuckFacade::NUMBER_OF_LEDS];
};

#endif
