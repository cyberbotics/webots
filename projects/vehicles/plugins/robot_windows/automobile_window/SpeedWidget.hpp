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
 * Description:  Tab showing the current speed of the car (and the target speed if controlled in speed)
 */

#ifndef SPEED_WIDGET_HPP
#define SPEED_WIDGET_HPP

#include "AbstractWidget.hpp"

using namespace webotsQtUtils;

class SpeedWidget : public AbstractWidget {
public:
  explicit SpeedWidget(QWidget *parent = 0);
  virtual ~SpeedWidget();
  void update();
};

#endif  // SPEED_WIDGET_HPP
