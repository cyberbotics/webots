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
#include <webots/range_finder.h>
#include <webots/RangeFinder.hpp>

using namespace std;
using namespace webots;

void RangeFinder::enable(int sampling_period) {
  wb_range_finder_enable(getTag(), sampling_period);
}

void RangeFinder::disable() {
  wb_range_finder_disable(getTag());
}

int RangeFinder::getSamplingPeriod() const {
  return wb_range_finder_get_sampling_period(getTag());
}

const float *RangeFinder::getRangeImage() const {
  return wb_range_finder_get_range_image(getTag());
}

int RangeFinder::getWidth() const {
  return wb_range_finder_get_width(getTag());
}

int RangeFinder::getHeight() const {
  return wb_range_finder_get_height(getTag());
}

double RangeFinder::getFov() const {
  return wb_range_finder_get_fov(getTag());
}

double RangeFinder::getMinRange() const {
  return wb_range_finder_get_min_range(getTag());
}

double RangeFinder::getMaxRange() const {
  return wb_range_finder_get_max_range(getTag());
}

int RangeFinder::saveImage(const string &filename, int quality) const {
  return wb_range_finder_save_image(getTag(), filename.c_str(), quality);
}

float RangeFinder::rangeImageGetDepth(const float *image, int width, int x, int y) {
  return wb_range_finder_image_get_depth(image, width, x, y);
}
