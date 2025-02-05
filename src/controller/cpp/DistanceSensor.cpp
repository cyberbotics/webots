// Copyright 1996-2024 Cyberbotics Ltd.
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
#include <webots/distance_sensor.h>
#include <webots/DistanceSensor.hpp>

using namespace webots;

void DistanceSensor::enable(int sampling_period) {
  wb_distance_sensor_enable(getTag(), sampling_period);
}

void DistanceSensor::disable() {
  wb_distance_sensor_disable(getTag());
}

int DistanceSensor::getSamplingPeriod() const {
  return wb_distance_sensor_get_sampling_period(getTag());
}

double DistanceSensor::getValue() const {
  return wb_distance_sensor_get_value(getTag());
}

double DistanceSensor::getMaxValue() const {
  return wb_distance_sensor_get_max_value(getTag());
}

double DistanceSensor::getMinValue() const {
  return wb_distance_sensor_get_min_value(getTag());
}

double DistanceSensor::getAperture() const {
  return wb_distance_sensor_get_aperture(getTag());
}

int DistanceSensor::getLookupTableSize() const {
  return wb_distance_sensor_get_lookup_table_size(getTag());
}

const double *DistanceSensor::getLookupTable() const {
  return wb_distance_sensor_get_lookup_table(getTag());
}

DistanceSensor::Type DistanceSensor::getType() const {
  return Type(wb_distance_sensor_get_type(getTag()));
}
