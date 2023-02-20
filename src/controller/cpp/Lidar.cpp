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
#include <webots/lidar.h>
#include <webots/Lidar.hpp>

using namespace std;
using namespace webots;

void Lidar::enable(int sampling_period) {
  wb_lidar_enable(getTag(), sampling_period);
}

void Lidar::enablePointCloud() {
  wb_lidar_enable_point_cloud(getTag());
}

void Lidar::disable() {
  wb_lidar_disable(getTag());
}

void Lidar::disablePointCloud() {
  wb_lidar_disable_point_cloud(getTag());
}

int Lidar::getSamplingPeriod() const {
  return wb_lidar_get_sampling_period(getTag());
}

bool Lidar::isPointCloudEnabled() const {
  return wb_lidar_is_point_cloud_enabled(getTag());
}

const float *Lidar::getRangeImage() const {
  return wb_lidar_get_range_image(getTag());
}

const float *Lidar::getLayerRangeImage(int layer) const {
  return wb_lidar_get_layer_range_image(getTag(), layer);
}

const LidarPoint *Lidar::getPointCloud() const {
  return wb_lidar_get_point_cloud(getTag());
}

const LidarPoint *Lidar::getLayerPointCloud(int layer) const {
  return wb_lidar_get_layer_point_cloud(getTag(), layer);
}

int Lidar::getNumberOfPoints() const {
  return wb_lidar_get_number_of_points(getTag());
}

int Lidar::getHorizontalResolution() const {
  return wb_lidar_get_horizontal_resolution(getTag());
}

int Lidar::getNumberOfLayers() const {
  return wb_lidar_get_number_of_layers(getTag());
}

double Lidar::getMinFrequency() const {
  return wb_lidar_get_min_frequency(getTag());
}

double Lidar::getMaxFrequency() const {
  return wb_lidar_get_max_frequency(getTag());
}

double Lidar::getFrequency() const {
  return wb_lidar_get_frequency(getTag());
}

void Lidar::setFrequency(double frequency) {
  wb_lidar_set_frequency(getTag(), frequency);
}

double Lidar::getFov() const {
  return wb_lidar_get_fov(getTag());
}

double Lidar::getVerticalFov() const {
  return wb_lidar_get_vertical_fov(getTag());
}

double Lidar::getMinRange() const {
  return wb_lidar_get_min_range(getTag());
}

double Lidar::getMaxRange() const {
  return wb_lidar_get_max_range(getTag());
}
