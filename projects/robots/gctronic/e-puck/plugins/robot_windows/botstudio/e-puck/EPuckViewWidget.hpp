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
 * Description:  Class defining a widget allowing to view the current values of the epuck
 */

#ifndef EPUCK_VIEW_WIDGET_HPP
#define EPUCK_VIEW_WIDGET_HPP

#include "EPuckFacade.hpp"

#include <core/RobotViewWidget.hpp>

class EPuckSlider;
class Model;

class EPuckViewWidget : public RobotViewWidget {
public:
  explicit EPuckViewWidget(QWidget *parent = NULL);
  virtual ~EPuckViewWidget();

  virtual void updateValues();

protected:
  EPuckSlider *mDistanceSensorSliders[EPuckFacade::NUMBER_OF_DISTANCE_SENSORS];
  EPuckSlider *mGroundSensorSliders[EPuckFacade::NUMBER_OF_GROUND_SENSORS];
  EPuckSlider *mSpeedSliders[2];
  EPuckSlider *mCameraSliders;

  Model *mModel;
};

#endif
