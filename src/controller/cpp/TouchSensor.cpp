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
#include <webots/touch_sensor.h>
#include <webots/TouchSensor.hpp>

using namespace webots;

void TouchSensor::enable(int sampling_period) {
  wb_touch_sensor_enable(getTag(), sampling_period);
}

void TouchSensor::disable() {
  wb_touch_sensor_disable(getTag());
}

int TouchSensor::getSamplingPeriod() const {
  return wb_touch_sensor_get_sampling_period(getTag());
}

double TouchSensor::getValue() const {
  return wb_touch_sensor_get_value(getTag());
}

const double *TouchSensor::getValues() const {
  return wb_touch_sensor_get_values(getTag());
}

TouchSensor::Type TouchSensor::getType() const {
  return Type(wb_touch_sensor_get_type(getTag()));
}

int TouchSensor::getLookupTableSize() const {
  return wb_touch_sensor_get_lookup_table_size(getTag());
}

const double *TouchSensor::getLookupTable() const {
  return wb_touch_sensor_get_lookup_table(getTag());
}
