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

#include "Viewer.hpp"

#include "BrakeWidget.hpp"
#include "EncodersWidget.hpp"
#include "GeneralInformationWidget.hpp"
#include "RPMWidget.hpp"
#include "SpeedWidget.hpp"
#include "SteeringWidget.hpp"
#include "ThrottleWidget.hpp"

#include <QtCore/QDir>

using namespace webotsQtUtils;

QStringList hiddenDevices = (QStringList() << "left_steer"
                                           << "right_steer"
                                           << "right_steer_sensor"
                                           << "left_steer_sensor"
                                           << "left_front_wheel"
                                           << "right_front_wheel"
                                           << "left_rear_wheel"
                                           << "right_rear_wheel"
                                           << "left_front_sensor"
                                           << "right_front_sensor"
                                           << "left_rear_sensor"
                                           << "right_rear_sensor"
                                           << "left_wiper_motor"
                                           << "right_wiper_motor"
                                           << "wiper_sensor"
                                           << "left_wing_yaw_mirror_motor"
                                           << "left_wing_pitch_mirror_motor"
                                           << "right_wing_yaw_mirror_motor"
                                           << "right_wing_pitch_mirror_motor"
                                           << "rear_yaw_mirror_motor"
                                           << "rear_pitch_mirror_motor"
                                           << "rear_yaw_mirror_frame_motor"
                                           << "rear_pitch_mirror_frame_motor"
                                           << "steering_wheel_motor"
                                           << "speed_needle_motor"
                                           << "rpm_needle_motor");

Viewer::Viewer() : GenericWindow(hiddenDevices) {
  mGeneralInformationWidget = new GeneralInformationWidget(this);
  mSpeedWidget = new SpeedWidget(this);
  mSteeringWidget = new SteeringWidget(this);
  mEncodersWidget = new EncodersWidget(this);
  mBrakeWidget = new BrakeWidget(this);
  mThrottleWidget = new ThrottleWidget(this);
  mRPMWidget = new RPMWidget(this);

  mTabWidget->insertTab(0, mGeneralInformationWidget, "Overview");
  mTabWidget->insertTab(1, mSpeedWidget, "Speed");
  mTabWidget->insertTab(2, mSteeringWidget, "Steering");
  mTabWidget->insertTab(3, mEncodersWidget, "Encoders");
  mTabWidget->insertTab(4, mBrakeWidget, "Brake");
  mTabWidget->insertTab(5, mThrottleWidget, "Throttle");
  mTabWidget->insertTab(6, mRPMWidget, "RPM");

  mTabWidget->setCurrentIndex(0);
  setMinimumHeight(550);
}

Viewer::~Viewer() {
}

void Viewer::readSensors() {
  mGeneralInformationWidget->updateInformation();
  mSpeedWidget->update();
  mSteeringWidget->update();
  mEncodersWidget->update();
  mBrakeWidget->update();
  mThrottleWidget->update();
  mRPMWidget->update();
  webotsQtUtils::GenericWindow::readSensors();
}
