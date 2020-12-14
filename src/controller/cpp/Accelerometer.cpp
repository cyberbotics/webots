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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/accelerometer.h>
#include <webots/Accelerometer.hpp>

using namespace webots;

void Accelerometer::enable(int sampling_period) {
  wb_accelerometer_enable(getTag(), sampling_period);
}

void Accelerometer::disable() {
  wb_accelerometer_disable(getTag());
}

int Accelerometer::getSamplingPeriod() const {
  return wb_accelerometer_get_sampling_period(getTag());
}

const double *Accelerometer::getValues() const {
  return wb_accelerometer_get_values(getTag());
}

int Accelerometer::getLookupTableSize() const {
  return wb_accelerometer_get_lookup_table_size(getTag());
}

const double *Accelerometer::getLookupTable() const {
  return wb_accelerometer_get_lookup_table(getTag());
}