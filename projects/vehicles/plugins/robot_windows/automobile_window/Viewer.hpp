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
 * Description:  Add tabs specific to the Car PROTO (and libraries)
 */

#ifndef VIEWER_HPP
#define VIEWER_HPP

class GeneralInformationWidget;
class SpeedWidget;
class SteeringWidget;
class EncodersWidget;
class BrakeWidget;
class ThrottleWidget;
class RPMWidget;

#include <gui/GenericWindow.hpp>

class Viewer : public webotsQtUtils::GenericWindow {
public:
  Viewer();
  virtual ~Viewer();

  void readSensors();

private:
  GeneralInformationWidget *mGeneralInformationWidget;
  SpeedWidget *mSpeedWidget;
  SteeringWidget *mSteeringWidget;
  EncodersWidget *mEncodersWidget;
  BrakeWidget *mBrakeWidget;
  ThrottleWidget *mThrottleWidget;
  RPMWidget *mRPMWidget;
};

#endif
