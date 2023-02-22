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
#include <webots/inertial_unit.h>
#include <webots/InertialUnit.hpp>

using namespace webots;

void InertialUnit::enable(int sampling_period) {
  wb_inertial_unit_enable(getTag(), sampling_period);
}

void InertialUnit::disable() {
  wb_inertial_unit_disable(getTag());
}

int InertialUnit::getSamplingPeriod() const {
  return wb_inertial_unit_get_sampling_period(getTag());
}

const double *InertialUnit::getRollPitchYaw() const {
  return wb_inertial_unit_get_roll_pitch_yaw(getTag());
}

const double *InertialUnit::getQuaternion() const {
  return wb_inertial_unit_get_quaternion(getTag());
}

double InertialUnit::getNoise() const {
  return wb_inertial_unit_get_noise(getTag());
}
