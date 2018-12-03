// Copyright 1996-2018 Cyberbotics Ltd.
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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/differential_wheels.h>
#include <webots/DifferentialWheels.hpp>

using namespace webots;

void DifferentialWheels::setSpeed(double left, double right) {
  wb_differential_wheels_set_speed(left, right);
}

double DifferentialWheels::getLeftSpeed() const {
  return wb_differential_wheels_get_left_speed();
}

double DifferentialWheels::getRightSpeed() const {
  return wb_differential_wheels_get_right_speed();
}

double DifferentialWheels::getMaxSpeed() const {
  return wb_differential_wheels_get_max_speed();
}

double DifferentialWheels::getSpeedUnit() const {
  return wb_differential_wheels_get_speed_unit();
}

void DifferentialWheels::enableEncoders(int sampling_period) {
  wb_differential_wheels_enable_encoders(sampling_period);
}

void DifferentialWheels::disableEncoders() {
  wb_differential_wheels_disable_encoders();
}

int DifferentialWheels::getEncodersSamplingPeriod() const {
  return wb_differential_wheels_get_encoders_sampling_period();
}

double DifferentialWheels::getLeftEncoder() const {
  return wb_differential_wheels_get_left_encoder();
}

double DifferentialWheels::getRightEncoder() const {
  return wb_differential_wheels_get_right_encoder();
}

void DifferentialWheels::setEncoders(double left, double right) {
  wb_differential_wheels_set_encoders(left, right);
}
