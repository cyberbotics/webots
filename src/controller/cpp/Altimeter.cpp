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
#include <webots/altimeter.h>
#include <webots/Altimeter.hpp>

#include <cstdlib>

using namespace std;
using namespace webots;

void Altimeter::enable(int sampling_period) {
  wb_altimeter_enable(getTag(), sampling_period);
}

void Altimeter::disable() {
  wb_altimeter_disable(getTag());
}

int Altimeter::getSamplingPeriod() const {
  return wb_altimeter_get_sampling_period(getTag());
}

double Altimeter::getValue() const {
  return wb_altimeter_get_value(getTag());
}
