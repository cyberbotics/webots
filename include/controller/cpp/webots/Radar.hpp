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

#ifndef RADAR_HPP
#define RADAR_HPP

#include <webots/Device.hpp>
#include "../../c/webots/radar_target.h"

namespace webots {
  typedef WbRadarTarget RadarTarget;

  class Radar : public Device {
  public:
    explicit Radar(const std::string &name) : Device(name) {}  // Use Robot::getRadar() instead
    virtual ~Radar() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    int getNumberOfTargets() const;
    const RadarTarget *getTargets() const;
    double getMinRange() const;
    double getMaxRange() const;
    double getHorizontalFov() const;
    double getVerticalFov() const;
  };
}  // namespace webots

#endif  // RADAR_HPP
