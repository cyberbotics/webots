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

#ifndef ROS_ALTIMETER_HPP
#define ROS_ALTIMETER_HPP

#include <webots/Altimeter.hpp>
#include "RosSensor.hpp"

using namespace webots;

class RosAltimeter : public RosSensor {
public:
  RosAltimeter(Altimeter *altimeter, Ros *ros);
  virtual ~RosAltimeter();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override { mAltimeter->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mAltimeter->getSamplingPeriod(); }

private:
  void cleanup() { mAltimeter->disable(); }

  Altimeter *mAltimeter;
};

#endif  // ROS_ALTIMETER_HPP
