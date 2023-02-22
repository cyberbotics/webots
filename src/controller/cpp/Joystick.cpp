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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/joystick.h>
#include <webots/Joystick.hpp>

using namespace webots;

void Joystick::enable(int sampling_period) {
  wb_joystick_enable(sampling_period);
}

void Joystick::disable() {
  wb_joystick_disable();
}

int Joystick::getSamplingPeriod() const {
  return wb_joystick_get_sampling_period();
}

bool Joystick::isConnected() const {
  return wb_joystick_is_connected();
}

std::string Joystick::getModel() const {
  const char *model = wb_joystick_get_model();
  return std::string(model ? model : "");
}

int Joystick::getNumberOfAxes() const {
  return wb_joystick_get_number_of_axes();
}

int Joystick::getAxisValue(int axis) const {
  return wb_joystick_get_axis_value(axis);
}

int Joystick::getNumberOfPovs() const {
  return wb_joystick_get_number_of_povs();
}

int Joystick::getPovValue(int pov) const {
  return wb_joystick_get_pov_value(pov);
}

int Joystick::getPressedButton() const {
  return wb_joystick_get_pressed_button();
}

void Joystick::setConstantForce(int level) {
  wb_joystick_set_constant_force(level);
}

void Joystick::setConstantForceDuration(double duration) {
  wb_joystick_set_constant_force_duration(duration);
}

void Joystick::setAutoCenteringGain(double gain) {
  wb_joystick_set_auto_centering_gain(gain);
}

void Joystick::setResistanceGain(double gain) {
  wb_joystick_set_resistance_gain(gain);
}

void Joystick::setForceAxis(int axis) {
  wb_joystick_set_force_axis(axis);
}
