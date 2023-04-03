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

#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <webots/Device.hpp>
#include "../../c/webots/lidar_point.h"

namespace webots {
  typedef WbLidarPoint LidarPoint;

  class Lidar : public Device {
  public:
    explicit Lidar(const std::string &name) : Device(name) {}  // Use Robot::getLidar() instead
    virtual ~Lidar() {}
    virtual void enable(int samplingPeriod);
    void enablePointCloud();
    virtual void disable();
    void disablePointCloud();
    int getSamplingPeriod() const;
    bool isPointCloudEnabled() const;
    const float *getRangeImage() const;
    const float *getLayerRangeImage(int layer) const;
    const LidarPoint *getPointCloud() const;
    const LidarPoint *getLayerPointCloud(int layer) const;
    int getNumberOfPoints() const;
    int getHorizontalResolution() const;
    int getNumberOfLayers() const;
    double getMinFrequency() const;
    double getMaxFrequency() const;
    double getFrequency() const;
    void setFrequency(double frequency);
    double getFov() const;
    double getVerticalFov() const;
    double getMinRange() const;
    double getMaxRange() const;
  };
}  // namespace webots
#endif  // LIDAR_HPP
