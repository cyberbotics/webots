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
#include <webots/vacuum_cup.h>
#include <webots/VacuumCup.hpp>

using namespace webots;

void VacuumCup::enablePresence(int sampling_period) {
  wb_vacuum_cup_enable_presence(getTag(), sampling_period);
}

void VacuumCup::disablePresence() {
  wb_vacuum_cup_disable_presence(getTag());
}

int VacuumCup::getPresenceSamplingPeriod() const {
  return wb_vacuum_cup_get_presence_sampling_period(getTag());
}

int VacuumCup::getPresence() const {
  return wb_vacuum_cup_get_presence(getTag());
}

bool VacuumCup::isOn() const {
  return wb_vacuum_cup_is_on(getTag());
}

void VacuumCup::turnOn() {
  wb_vacuum_cup_turn_on(getTag());
}

void VacuumCup::turnOff() {
  wb_vacuum_cup_turn_off(getTag());
}
