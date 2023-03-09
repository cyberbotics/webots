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
#include <webots/gps.h>
#include <webots/GPS.hpp>

#include <cstdlib>

using namespace std;
using namespace webots;

void GPS::enable(int sampling_period) {
  wb_gps_enable(getTag(), sampling_period);
}

void GPS::disable() {
  wb_gps_disable(getTag());
}

int GPS::getSamplingPeriod() const {
  return wb_gps_get_sampling_period(getTag());
}

const double *GPS::getValues() const {
  return wb_gps_get_values(getTag());
}

double GPS::getSpeed() const {
  return wb_gps_get_speed(getTag());
}

const double *GPS::getSpeedVector() const {
  return wb_gps_get_speed_vector(getTag());
}

const GPS::CoordinateSystem GPS::getCoordinateSystem() const {
  return CoordinateSystem(wb_gps_get_coordinate_system(getTag()));
}

string GPS::convertToDegreesMinutesSeconds(double decimalDegree) {
  const char *coordinatesString = wb_gps_convert_to_degrees_minutes_seconds(decimalDegree);
  std::string coordinates = string(coordinatesString);
  free(static_cast<void *>(const_cast<char *>(coordinatesString)));
  return coordinates;
}
