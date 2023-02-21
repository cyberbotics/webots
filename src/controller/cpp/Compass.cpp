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
#include <webots/compass.h>
#include <webots/Compass.hpp>

using namespace webots;

void Compass::enable(int sampling_period) {
  wb_compass_enable(getTag(), sampling_period);
}

void Compass::disable() {
  wb_compass_disable(getTag());
}

int Compass::getSamplingPeriod() const {
  return wb_compass_get_sampling_period(getTag());
}

const double *Compass::getValues() const {
  return wb_compass_get_values(getTag());
}

int Compass::getLookupTableSize() const {
  return wb_compass_get_lookup_table_size(getTag());
}

const double *Compass::getLookupTable() const {
  return wb_compass_get_lookup_table(getTag());
}
