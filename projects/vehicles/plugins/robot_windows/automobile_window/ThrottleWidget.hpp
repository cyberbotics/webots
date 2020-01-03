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
 * Description:  Tab showing (if controlled in torque) the throttle command
 */

#ifndef THROTTLE_WIDGET_HPP
#define THROTTLE_WIDGET_HPP

#include "AbstractWidget.hpp"

using namespace webotsQtUtils;

class ThrottleWidget : public AbstractWidget {
public:
  explicit ThrottleWidget(QWidget *parent = 0);
  virtual ~ThrottleWidget();
  void update();
};

#endif  // THROTTLE_WIDGET_HPP
