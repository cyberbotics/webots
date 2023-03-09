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
#include <webots/radar.h>
#include <webots/Radar.hpp>

using namespace webots;

void Radar::enable(int sampling_period) {
  wb_radar_enable(getTag(), sampling_period);
}

void Radar::disable() {
  wb_radar_disable(getTag());
}

int Radar::getSamplingPeriod() const {
  return wb_radar_get_sampling_period(getTag());
}

int Radar::getNumberOfTargets() const {
  return wb_radar_get_number_of_targets(getTag());
}

const RadarTarget *Radar::getTargets() const {
  return wb_radar_get_targets(getTag());
}

double Radar::getMinRange() const {
  return wb_radar_get_min_range(getTag());
}

double Radar::getMaxRange() const {
  return wb_radar_get_max_range(getTag());
}

double Radar::getHorizontalFov() const {
  return wb_radar_get_horizontal_fov(getTag());
}

double Radar::getVerticalFov() const {
  return wb_radar_get_vertical_fov(getTag());
}
